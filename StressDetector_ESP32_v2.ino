// ============================================================
//  Sistem Deteksi Stres Psikofisiologis Multimodal
//  Berbasis ESP32 + Logika Fuzzy (eFLL / Mamdani)
//  PRD v2.0
//
//  Library yang dibutuhkan (install via Arduino Library Manager):
//    - MAX30105 by SparkFun Electronics  (untuk sensor MAX30102)
//    - LiquidCrystal_I2C by Frank de Brabander
//    - eFLL (Embedded Fuzzy Logic Library) by ItamarRocha
//
//  Wiring Ringkas:
//    MAX30102  -> SDA=GPIO21, SCL=GPIO22 (I2C default ESP32)
//    LCD 16x2  -> SDA=GPIO21, SCL=GPIO22 (berbagi I2C, addr 0x27)
//    Grove GSR -> OUT=GPIO34 (ADC1_CH6, input-only, 12-bit)
//    LED Indikator (opsional) -> GPIO2 (LED onboard)
// ============================================================

// ── Library ─────────────────────────────────────────────────
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"
#include "heartRate.h"        // bagian dari SparkFun MAX30105
#include <eFLL.h>             // Embedded Fuzzy Logic Library

// ── Pin & Konstanta ──────────────────────────────────────────
#define GSR_PIN          34   // ADC1_CH6 – aman untuk analogRead()
#define LED_PIN           2   // LED onboard ESP32

// Durasi kalibrasi baseline (ms)
#define BASELINE_DURATION_MS  30000UL

// Interval sampling utama (ms) — 50 ms = 20 Hz
#define SAMPLE_INTERVAL_MS     50

// Low-Pass Filter: koefisien IIR sederhana (alpha ∈ 0–1)
// Semakin kecil alpha → filter lebih halus, respons lebih lambat
#define LPF_ALPHA_BPM    0.15f
#define LPF_ALPHA_GSR    0.10f

// ── Objek Hardware ───────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);   // ubah 0x27 -> 0x3F jika perlu
MAX30105 particleSensor;

// ── Objek Fuzzy (eFLL) ───────────────────────────────────────
Fuzzy          *fuzzy        = new Fuzzy();

// Input 1: Perubahan BPM (delta persen terhadap baseline)
FuzzyInput     *fiBPM        = new FuzzyInput(1);
FuzzySet       *bpmSangatLambat = new FuzzySet(-20, -20, -5, 0);
FuzzySet       *bpmLambat       = new FuzzySet(-5,    0, 10, 20);
FuzzySet       *bpmCepat        = new FuzzySet(10,   20, 30, 40);
FuzzySet       *bpmSangatCepat  = new FuzzySet(30,   40, 100, 100);

// Input 2: Perubahan GSR (delta persen terhadap baseline)
FuzzyInput     *fiGSR        = new FuzzyInput(2);
FuzzySet       *gsrSangatKering = new FuzzySet(-20, -20, -5, 0);
FuzzySet       *gsrKering       = new FuzzySet(-5,    0, 10, 20);
FuzzySet       *gsrBasah        = new FuzzySet(10,   20, 30, 40);
FuzzySet       *gsrSangatBasah  = new FuzzySet(30,   40, 100, 100);

// Output: Tingkat Stres (skor 0–100)
FuzzyOutput    *foStress     = new FuzzyOutput(1);
FuzzySet       *outRileks    = new FuzzySet(0,  0,  20, 30);
FuzzySet       *outTenang    = new FuzzySet(20, 30, 40, 50);
FuzzySet       *outCemas     = new FuzzySet(40, 50, 65, 75);
FuzzySet       *outStresBerat = new FuzzySet(65, 75, 100, 100);

// ── Variabel State ───────────────────────────────────────────
float    baselineBPM   = 0.0f;
float    baselineGSR   = 0.0f;
bool     baselineDone  = false;

float    filteredBPM   = 0.0f;
float    filteredGSR   = 0.0f;

// Akumulasi baseline
float    sumBPM        = 0.0f;
float    sumGSR        = 0.0f;
uint32_t sampleCount   = 0;

// Penghitung waktu
uint32_t lastSampleTime   = 0;
uint32_t baselineStartTime = 0;

// Beat detection (SparkFun heartRate.h)
const   byte  RATE_SIZE = 8;
byte    rates[RATE_SIZE];
byte    rateSpot = 0;
long    lastBeat = 0;
float   beatsPerMinute = 0;
int     beatAvg = 0;

// Label hasil deteksi
const char* stressLabel = "---";

// ── Forward Declaration ──────────────────────────────────────
void     setupFuzzy();
void     doBaselineSampling();
void     doRealTimeSampling();
float    readBPM();
float    readGSR();
float    lpfUpdate(float prev, float raw, float alpha);
String   evaluateStress(float deltaBPM, float deltaGSR);
void     displayLCD(String line1, String line2);
void     blinkLED(uint8_t times, uint16_t delayMs);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n[BOOT] Sistem Deteksi Stres v2.0"));

  // ── GPIO
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ── I2C & LCD
  Wire.begin(21, 22);           // SDA, SCL
  lcd.init();
  lcd.backlight();
  displayLCD("Inisialisasi...", "");
  delay(1000);

  // ── MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("[ERROR] MAX30102 tidak ditemukan!"));
    displayLCD("ERROR: MAX30102", "Cek kabel I2C");
    while (true) { blinkLED(3, 200); delay(1000); }
  }

  // Konfigurasi sensor MAX30102
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);   // mati IR merah (hemat)
  particleSensor.setPulseAmplitudeGreen(0);
  particleSensor.enableDIETEMPRDY();

  // ── Fuzzy
  setupFuzzy();

  // ── Instruksi ke pengguna
  displayLCD("Letakkan Jari", "& Rileks...");
  delay(3000);

  // ── Mulai fase kalibrasi
  displayLCD("Kalibrasi 30dtk", "Tetap Diam...");
  baselineStartTime = millis();
  lastSampleTime    = millis();

  Serial.println(F("[INFO] Fase Kalibrasi Baseline dimulai..."));
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  uint32_t now = millis();

  // Throttle: hanya proses setiap SAMPLE_INTERVAL_MS
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = now;

  if (!baselineDone) {
    // ── FASE A: Kalibrasi Baseline ──────────────────────────
    doBaselineSampling();

    uint32_t elapsed = now - baselineStartTime;
    uint8_t  secsLeft = (uint8_t)((BASELINE_DURATION_MS - elapsed) / 1000);

    // Update countdown di LCD setiap detik
    static uint8_t lastCountdown = 99;
    if (secsLeft != lastCountdown) {
      lastCountdown = secsLeft;
      char buf[17];
      snprintf(buf, sizeof(buf), "Sisa: %2d detik  ", secsLeft);
      displayLCD("Kalibrasi...", buf);
    }

    // Selesai kalibrasi
    if (elapsed >= BASELINE_DURATION_MS) {
      if (sampleCount > 0) {
        baselineBPM = sumBPM / sampleCount;
        baselineGSR = sumGSR / sampleCount;
      } else {
        // Fallback jika tidak ada sampel valid
        baselineBPM = 75.0f;
        baselineGSR = 512.0f;
      }

      filteredBPM = baselineBPM;
      filteredGSR = baselineGSR;

      Serial.printf("[BASELINE] BPM=%.1f  GSR=%.1f  (dari %d sampel)\n",
                    baselineBPM, baselineGSR, sampleCount);

      displayLCD("Baseline OK!", "Memulai monitor");
      blinkLED(2, 150);
      delay(1500);
      baselineDone = true;
    }

  } else {
    // ── FASE B+C: Real-Time Monitoring ─────────────────────
    doRealTimeSampling();
  }
}

// ============================================================
//  FASE A — Sampling Baseline
// ============================================================
void doBaselineSampling() {
  float rawBPM = readBPM();
  float rawGSR = readGSR();

  // Hanya akumulasi jika BPM valid (> 30 BPM = jari terdeteksi)
  if (rawBPM > 30.0f) {
    sumBPM += rawBPM;
    sumGSR += rawGSR;
    sampleCount++;
  }

  Serial.printf("[BASELINE] raw BPM=%.1f  raw GSR=%.1f\n", rawBPM, rawGSR);
}

// ============================================================
//  FASE B+C — Real-Time Sampling + Fuzzy
// ============================================================
void doRealTimeSampling() {
  //float rawBPM = readBPM();
  float rawGSR = readGSR();
  

  // IIR Low-Pass Filter
  filteredBPM = lpfUpdate(filteredBPM, rawBPM, LPF_ALPHA_BPM);
  filteredGSR = lpfUpdate(filteredGSR, rawGSR, LPF_ALPHA_GSR);

  // Hitung delta persen terhadap baseline
  float gsrPercent = (filteredGSR / 4095.0f) * 100.0f;
  float deltaBPM = 0.0f;
  float deltaGSR = 0.0f;

  if (baselineBPM > 0.0f)
    deltaBPM = ((filteredBPM - baselineBPM) / baselineBPM) * 100.0f;
  if (baselineGSR > 0.0f)
    deltaGSR = ((filteredGSR - baselineGSR) / baselineGSR) * 100.0f;

  // Evaluasi fuzzy
  String label = evaluateStress(deltaBPM, deltaGSR);
  stressLabel  = label.c_str();

  // --- PEMBATAS WAKTU UNTUK LCD ---
  static uint32_t lastLcdUpdate = 0;
  if (millis() - lastLcdUpdate >= 1000) {   
    lastLcdUpdate = millis();

  // Tampilkan di LCD
  char bpmBuf[17];
  snprintf(bpmBuf, sizeof(bpmBuf), "BPM:%.0f GSR:%.0f",
           filteredBPM, filteredGSR);
  displayLCD(bpmBuf, label);

  // Serial Monitor (untuk debugging / logging)
  Serial.printf("[DATA] BPM=%.1f ΔBPM=%.1f%%  GSR=%.1f ΔGSR=%.1f%%  -> %s\n",
                filteredBPM, deltaBPM, filteredGSR, deltaGSR, label.c_str());
}

// ============================================================
//  BACA BPM (SparkFun heartRate.h — algoritma deteksi beat)
// ============================================================
float readBPM() {
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat   = millis();

    beatsPerMinute = 60.0f / (delta / 1000.0f);

    // Batasi rentang fisiologis wajar
    if (beatsPerMinute >= 30 && beatsPerMinute <= 220) {
      rates[rateSpot++ % RATE_SIZE] = (byte)beatsPerMinute;
    }
  }

  // Rolling average dari 8 sampel terakhir
  int sum = 0;
  for (byte i = 0; i < RATE_SIZE; i++) sum += rates[i];
  beatAvg = sum / RATE_SIZE;

  // Jika IR terlalu rendah = jari belum diletakkan
  if (irValue < 50000) return 0.0f;

  return (beatAvg > 0) ? (float)beatAvg : beatsPerMinute;
}

// ============================================================
//  BACA GSR (Analog, Grove GSR V1.2)
//  Output mentah ADC (0–4095, 12-bit ESP32)
//  Konduktivitas lebih tinggi = nilai ADC lebih tinggi = basah
// ============================================================
float readGSR() {
  // Rata-rata 8 pembacaan ADC untuk mengurangi noise
  uint32_t acc = 0;
  for (uint8_t i = 0; i < 8; i++) {
    acc += analogRead(GSR_PIN);
    delayMicroseconds(100);
  }
  return (float)(acc / 8);
}

// ============================================================
//  IIR LOW-PASS FILTER
//  out = alpha * raw + (1 - alpha) * prev
// ============================================================
float lpfUpdate(float prev, float raw, float alpha) {
  if (raw < 1.0f) return prev;   // abaikan pembacaan tidak valid
  return alpha * raw + (1.0f - alpha) * prev;
}

// ============================================================
//  SETUP FUZZY (eFLL — Mamdani)
// ============================================================
void setupFuzzy() {
  // ── Input 1: Delta BPM (%)
  fiBPM->addFuzzySet(bpmSangatLambat);
  fiBPM->addFuzzySet(bpmLambat);
  fiBPM->addFuzzySet(bpmCepat);
  fiBPM->addFuzzySet(bpmSangatCepat);
  fuzzy->addFuzzyInput(fiBPM);

  // ── Input 2: Delta GSR (%)
  fiGSR->addFuzzySet(gsrSangatKering);
  fiGSR->addFuzzySet(gsrKering);
  fiGSR->addFuzzySet(gsrBasah);
  fiGSR->addFuzzySet(gsrSangatBasah);
  fuzzy->addFuzzyInput(fiGSR);

  // ── Output: Skor Stres
  foStress->addFuzzySet(outRileks);
  foStress->addFuzzySet(outTenang);
  foStress->addFuzzySet(outCemas);
  foStress->addFuzzySet(outStresBerat);
  fuzzy->addFuzzyOutput(foStress);

  // ── 16 Aturan Fuzzy (IF-THEN Mamdani) ──────────────────────
  // Tabel: BPM (SL/L/C/SC) x GSR (SK/K/B/SB) = 4x4 = 16 aturan

  // --- BPM Sangat Lambat ---
  { // R1: BPM-SL + GSR-SK = Rileks
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmSangatLambat, gsrSangatKering);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outRileks);
    fuzzy->addFuzzyRule(new FuzzyRule(1, ant, con));
  }
  { // R2: BPM-SL + GSR-K = Rileks
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmSangatLambat, gsrKering);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outRileks);
    fuzzy->addFuzzyRule(new FuzzyRule(2, ant, con));
  }
  { // R3: BPM-SL + GSR-B = Tenang
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmSangatLambat, gsrBasah);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outTenang);
    fuzzy->addFuzzyRule(new FuzzyRule(3, ant, con));
  }
  { // R4: BPM-SL + GSR-SB = Cemas
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmSangatLambat, gsrSangatBasah);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outCemas);
    fuzzy->addFuzzyRule(new FuzzyRule(4, ant, con));
  }

  // --- BPM Lambat ---
  { // R5: BPM-L + GSR-SK = Rileks
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmLambat, gsrSangatKering);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outRileks);
    fuzzy->addFuzzyRule(new FuzzyRule(5, ant, con));
  }
  { // R6: BPM-L + GSR-K = Tenang
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmLambat, gsrKering);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outTenang);
    fuzzy->addFuzzyRule(new FuzzyRule(6, ant, con));
  }
  { // R7: BPM-L + GSR-B = Tenang
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmLambat, gsrBasah);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outTenang);
    fuzzy->addFuzzyRule(new FuzzyRule(7, ant, con));
  }
  { // R8: BPM-L + GSR-SB = Cemas
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmLambat, gsrSangatBasah);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outCemas);
    fuzzy->addFuzzyRule(new FuzzyRule(8, ant, con));
  }

  // --- BPM Cepat ---
  { // R9: BPM-C + GSR-SK = Tenang
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmCepat, gsrSangatKering);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outTenang);
    fuzzy->addFuzzyRule(new FuzzyRule(9, ant, con));
  }
  { // R10: BPM-C + GSR-K = Cemas
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmCepat, gsrKering);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outCemas);
    fuzzy->addFuzzyRule(new FuzzyRule(10, ant, con));
  }
  { // R11: BPM-C + GSR-B = Cemas
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmCepat, gsrBasah);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outCemas);
    fuzzy->addFuzzyRule(new FuzzyRule(11, ant, con));
  }
  { // R12: BPM-C + GSR-SB = Stres Berat
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmCepat, gsrSangatBasah);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outStresBerat);
    fuzzy->addFuzzyRule(new FuzzyRule(12, ant, con));
  }

  // --- BPM Sangat Cepat ---
  { // R13: BPM-SC + GSR-SK = Cemas
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmSangatCepat, gsrSangatKering);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outCemas);
    fuzzy->addFuzzyRule(new FuzzyRule(13, ant, con));
  }
  { // R14: BPM-SC + GSR-K = Cemas
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmSangatCepat, gsrKering);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outCemas);
    fuzzy->addFuzzyRule(new FuzzyRule(14, ant, con));
  }
  { // R15: BPM-SC + GSR-B = Stres Berat
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmSangatCepat, gsrBasah);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outStresBerat);
    fuzzy->addFuzzyRule(new FuzzyRule(15, ant, con));
  }
  { // R16: BPM-SC + GSR-SB = Stres Berat (kondisi paling kritis)
    FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
    ant->joinWithAND(bpmSangatCepat, gsrSangatBasah);
    FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
    con->addOutput(outStresBerat);
    fuzzy->addFuzzyRule(new FuzzyRule(16, ant, con));
  }

  Serial.println(F("[FUZZY] 16 aturan berhasil dimuat."));
}

// ============================================================
//  EVALUASI FUZZY → Label Teks
// ============================================================
String evaluateStress(float deltaBPM, float deltaGSR) {
  // Batasi input agar tidak melebihi domain fungsi keanggotaan
  deltaBPM = constrain(deltaBPM, -20.0f, 100.0f);
  deltaGSR = constrain(deltaGSR, -20.0f, 100.0f);

  fuzzy->setInput(1, deltaBPM);
  fuzzy->setInput(2, deltaGSR);
  fuzzy->fuzzify();

  float score = fuzzy->defuzzify(1);   // CoG via eFLL

  Serial.printf("[FUZZY] deltaBPM=%.1f  deltaGSR=%.1f  score=%.1f\n",
                deltaBPM, deltaGSR, score);

  // Mapping skor ke label
  if      (score < 25.0f) return "Rileks";
  else if (score < 50.0f) return "Tenang";
  else if (score < 70.0f) return "Cemas";
  else                    return "Stres Berat!";
}

// ============================================================
//  TAMPILAN LCD
// ============================================================
void displayLCD(String line1, String line2) {
  lcd.setCursor(0, 0);
  lcd.print(line1 + "                ");   // padding kiri agar bersih
  lcd.setCursor(0, 1);
  lcd.print(line2 + "                ");
}

// ============================================================
//  BLINK LED (onboard / indikator)
// ============================================================
void blinkLED(uint8_t times, uint16_t delayMs) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}
