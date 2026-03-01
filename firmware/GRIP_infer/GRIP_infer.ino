/*
 * GRIP_infer.ino — Real-Time Terrain Display Mode
 * Ground Recognition Intelligence Platform
 *
 * Streams fused IMU + audio sensor data to the Linux MPU at 33 Hz.
 * The Linux MPU runs the Edge Impulse model and calls set_terrain() via
 * arduino-router RPC every 500 ms. This sketch displays the result on the LCD.
 *
 * Hardware:
 *   - LSM303DLHC accel (0x19) + mag (0x1E) via I2C
 *   - MAX4466 analog mic on A2
 *   - 16x2 LCD with PCF8574T backpack (0x27) via I2C
 *
 * Board rules:
 *   - Use Monitor.print() instead of Serial.print()
 *   - Call Wire.begin() before any I2C device init
 *   - No while(!Serial) — use delay(2000)
 *   - Build full lines in _lineBuf before Monitor.println()
 *
 * LCD layout:
 *   Row 0: "~grass    87.3%"  ← inference result (updated via RPC every 500ms)
 *   Row 1: "N=00012  33Hz  "  ← sample count + rate
 *
 * Feature vector (v2, 12 axes, matches EI training data):
 *   AX, AY, AZ, MX, MY, MZ, AX_HP, AY_HP, AZ_HP, VMAG, MIC_LOW, MIC_HIGH
 */

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// ---------------------------------------------------------------------------
// Sampling — 33 Hz to match training data effective rate
// ---------------------------------------------------------------------------
#define SAMPLE_INTERVAL_MS  30    // ~33 Hz
#define HP_ALPHA  0.90f           // High-pass accel IIR (matches GRIP_collect)
#define LP_ALPHA  0.85f           // Low-pass mic IIR   (matches GRIP_collect)

// ---------------------------------------------------------------------------
// Peripheral objects
// ---------------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(12345);

// ---------------------------------------------------------------------------
// IIR filter state
// ---------------------------------------------------------------------------
float hp_prev_raw[3]  = {0, 0, 0};
float hp_prev_filt[3] = {0, 0, 0};
float mic_lp          = 0.0f;

// ---------------------------------------------------------------------------
// Inference result — updated by set_terrain_rpc(), applied in loop()
// ---------------------------------------------------------------------------
char          _infer_label[17] = "";   // empty = no result yet
float         _infer_conf      = 0.0f;
volatile bool _infer_pending   = false;

// Timing & counters
unsigned long lastSampleTime = 0;
unsigned long sampleCount    = 0;
char _lineBuf[220];

// ---------------------------------------------------------------------------
// set_terrain_rpc() — called by arduino-router when Linux sends inference result
// ---------------------------------------------------------------------------
void set_terrain_rpc(String label, float conf) {
  strncpy(_infer_label, label.c_str(), 16);
  _infer_label[16] = '\0';
  _infer_conf    = conf;
  _infer_pending = true;
}

// ---------------------------------------------------------------------------
// lcdShowInference() — row 0 only
// ---------------------------------------------------------------------------
void lcdShowInference() {
  char row0[17];
  if (_infer_label[0] == '\0') {
    strncpy(row0, "< waiting...   >", 16);
    row0[16] = '\0';
  } else {
    // "~flat_surf 87.3%"  (1 + 9 + 5 + 1 = 16 chars)
    snprintf(row0, sizeof(row0), "~%-9.9s%5.1f%%", _infer_label, _infer_conf);
  }
  lcd.setCursor(0, 0);
  lcd.print(row0);
}

// ---------------------------------------------------------------------------
// readMicBands() — two-band RMS, must match GRIP_collect.ino
// ---------------------------------------------------------------------------
void readMicBands(float *micLow, float *micHigh) {
  const int n = 16;
  float sum = 0;
  for (int i = 0; i < n; i++) {
    sum += analogRead(A2);
    delayMicroseconds(125);
  }
  float mic_raw = sum / n;
  mic_lp = LP_ALPHA * mic_lp + (1.0f - LP_ALPHA) * mic_raw;
  *micLow  = mic_lp;
  *micHigh = mic_raw - mic_lp;
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Monitor.begin();
  Bridge.provide("set_terrain", set_terrain_rpc);
  delay(2000);

  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("< waiting...   >");
  lcd.setCursor(0, 1);
  lcd.print("GRIP Infer Init ");

  if (!accel.begin()) {
    Monitor.println("ERROR: LSM303 accel not found");
    lcd.setCursor(0, 1); lcd.print("ACCEL FAIL      ");
    while (1) { delay(100); }
  }
  if (!mag.begin()) {
    Monitor.println("ERROR: LSM303 mag not found");
    lcd.setCursor(0, 1); lcd.print("MAG FAIL        ");
    while (1) { delay(100); }
  }

  Monitor.println("GRIP_infer ready — streaming sensor data to Linux MPU");
  lcd.setCursor(0, 1);
  lcd.print("Sensors OK      ");
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("N=00000  33Hz   ");

  lastSampleTime = millis();
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  // Apply any new inference result from the Linux MPU to LCD row 0
  if (_infer_pending) {
    _infer_pending = false;
    lcdShowInference();
  }

  // Sample at 33 Hz and stream 14-field v2 CSV to Linux MPU
  if (millis() - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime += SAMPLE_INTERVAL_MS;

    sensors_event_t accelEvent, magEvent;
    accel.getEvent(&accelEvent);
    mag.getEvent(&magEvent);

    float ax = accelEvent.acceleration.x;
    float ay = accelEvent.acceleration.y;
    float az = accelEvent.acceleration.z;

    // IIR high-pass — strip gravity, isolate vibration
    float raw[3] = {ax, ay, az};
    float hp[3];
    for (int i = 0; i < 3; i++) {
      hp[i] = HP_ALPHA * (hp_prev_filt[i] + raw[i] - hp_prev_raw[i]);
      hp_prev_raw[i]  = raw[i];
      hp_prev_filt[i] = hp[i];
    }
    float vmag = sqrt(hp[0]*hp[0] + hp[1]*hp[1] + hp[2]*hp[2]);

    float micLow, micHigh;
    readMicBands(&micLow, &micHigh);

    sampleCount++;

    snprintf(_lineBuf, sizeof(_lineBuf),
      "%lu,idle,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
      millis(),
      (double)ax,    (double)ay,    (double)az,
      (double)magEvent.magnetic.x,
      (double)magEvent.magnetic.y,
      (double)magEvent.magnetic.z,
      (double)hp[0], (double)hp[1], (double)hp[2],
      (double)vmag,
      (double)micLow, (double)micHigh);
    Monitor.println(_lineBuf);

    // Update row 1 with sample count every 33 samples (~1 s)
    if (sampleCount % 33 == 0) {
      lcd.setCursor(0, 1);
      char row1[17];
      snprintf(row1, sizeof(row1), "N=%-5lu  33Hz   ", sampleCount);
      lcd.print(row1);
    }
  }
}
