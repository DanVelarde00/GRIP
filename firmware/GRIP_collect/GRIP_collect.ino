/*
 * GRIP_collect.ino — Data Collection Mode
 * Ground Recognition Intelligence Platform
 *
 * Collects labeled terrain sensor data at 100Hz and streams CSV over
 * RouterBridge (WiFi) to a laptop logger. Runs on Arduino Uno Q (STM32U585).
 *
 * Hardware:
 *   - LSM303DLHC accel (0x19) + mag (0x1E) via I2C
 *   - MAX4466 analog mic on A2
 *   - 16x2 LCD with PCF8574T backpack (0x27) via I2C
 *   - Tactile button on D2 (INPUT_PULLUP, LOW = pressed)
 *
 * Board rules:
 *   - Use Monitor.print() instead of Serial.print()
 *   - Call Wire.begin() before any I2C device init
 *   - No SD card — data streams over WiFi via RouterBridge
 *   - No while(!Serial) — use delay(2000)
 *
 * CSV format (14 fields):
 *   timestamp_ms, label,
 *   AX, AY, AZ,          raw accelerometer (m/s²) — includes gravity
 *   MX, MY, MZ,          magnetometer (µT)
 *   AX_HP, AY_HP, AZ_HP, high-pass filtered accel — vibration only
 *   VMAG,                vibration magnitude sqrt(AX_HP²+AY_HP²+AZ_HP²)
 *   MIC_LOW, MIC_HIGH    mic energy in low (<750Hz) and high (>750Hz) bands
 *
 * LCD states:
 *   Init:       "GRIP Collect  " / "Init...       "
 *   Mic fail:   "MIC FAIL A2=NN" / "Press btn: ok "
 *   Sensor fail:"ACCEL/MAG FAIL" / "              "  (halts)
 *   Idle:       "Select terrain" / "> label  (N/4)"
 *   Warmup:     "Warming up... " / "label   in Xs "
 *   Recording:  "REC: label    " / "N=NNNN  [stop]"
 *   Good stop:  "Sent: NNNN    " / "Data saving.. "  (2s)
 *   Short stop: "TOO SHORT:NNNN" / "need ~700 rows"  (3s)
 */

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// ---------------------------------------------------------------------------
// Hardware pins & addresses
// ---------------------------------------------------------------------------
#define BUTTON_PIN  2
#define MIC_PIN     A2    // MAX4466 analog mic

// ---------------------------------------------------------------------------
// Sampling parameters
// ---------------------------------------------------------------------------
#define SAMPLE_RATE_HZ    100          // 100 Hz → 10 ms per sample
#define SAMPLE_INTERVAL   (1000 / SAMPLE_RATE_HZ)  // 10 ms
#define LONG_PRESS_MS     1000         // Button hold threshold for confirm

// Data trimming — first 3 seconds skipped (startup noise).
// Last 5 seconds trimmed by Python side (GRIP_wifi_stream.py).
#define WARMUP_SAMPLES    (3 * SAMPLE_RATE_HZ)  // 300 samples = 3 sec

// Mic health check: MAX4466 at rest should read 400–624 (centre ~512 ± 20%)
#define MIC_REST_MIN  400
#define MIC_REST_MAX  624

// ---------------------------------------------------------------------------
// Terrain labels
// ---------------------------------------------------------------------------
const char* TERRAIN_LABELS[] = {
  "snow", "flat_surfaces", "gravel", "grass"
};
#define NUM_TERRAINS 4

// ---------------------------------------------------------------------------
// Peripheral objects
// ---------------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(12345);

// ---------------------------------------------------------------------------
// High-pass filter state — isolates vibration from gravity
//
// Simple first-order IIR high-pass:  y[n] = α * (y[n-1] + x[n] - x[n-1])
// α = 0.90 → cutoff ≈ 1.6 Hz at 100 Hz sample rate
// Strips the ~9.8 m/s² gravity DC component, leaving vibration signal.
// Filter settles in ~200 ms (well within the 3-second warmup window).
// ---------------------------------------------------------------------------
#define HP_ALPHA 0.90f
float ax_prev = 0.0f, ay_prev = 0.0f, az_prev = 0.0f;
float ax_hp   = 0.0f, ay_hp   = 0.0f, az_hp   = 0.0f;

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum State {
  STATE_SELECT_LABEL,
  STATE_RECORDING
};

State currentState      = STATE_SELECT_LABEL;
int   selectedTerrain   = 0;
unsigned long sampleCount  = 0;   // samples actually sent
unsigned long rawCount     = 0;   // total samples taken (including warmup)

// Button debounce / press tracking
bool     lastButtonState  = HIGH;
bool     buttonState      = HIGH;
unsigned long btnDownTime  = 0;
unsigned long lastDebounce = 0;
const unsigned long DEBOUNCE_MS = 50;

// Sampling timing
unsigned long lastSampleTime = 0;

// Line buffer — build entire output lines before sending to avoid
// RouterBridge fragmentation (multiple print() calls get interleaved).
// 220 chars covers the 14-field CSV row worst case (~120 chars) with margin.
char _lineBuf[220];

// Inference result from Linux MPU — written by set_terrain_rpc(), read by loop().
// Volatile flag ensures the main loop sees the write from the RPC thread.
char           _infer_label[17]  = "";    // empty = no result yet
float          _infer_conf       = 0.0f;
volatile bool  _infer_pending    = false;

// ---------------------------------------------------------------------------
// set_terrain_rpc() — RPC handler registered with Bridge.provide().
// Called by arduino-router when the Linux inference thread sends a result.
// Stores label + confidence; loop() picks them up and updates the LCD.
// ---------------------------------------------------------------------------
void set_terrain_rpc(String label, float conf) {
  strncpy(_infer_label, label.c_str(), 16);
  _infer_label[16] = '\0';
  _infer_conf   = conf;
  _infer_pending = true;
}

// ---------------------------------------------------------------------------
// lcdShowInference() — writes row 0 with the current inference result.
// "< waiting...   >" until the first result arrives, then "~terrain  91.2%"
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
// readMicBands() — two-band RMS from MAX4466 analog mic on A2
//
// Samples 64 readings at ~8kHz. Splits into two frequency bands using a
// first-order IIR low-pass filter (α=0.85 → cutoff ~750 Hz):
//
//   MIC_LOW  = RMS of the smoothed (low-pass) signal  → tyre hum, rumble
//   MIC_HIGH = RMS of the residual (high-pass) signal → gravel impacts,
//              sharp transients, acoustic texture
//
// Key terrain signatures:
//   gravel       → high MIC_HIGH (sharp impulsive spikes)
//   grass        → moderate MIC_LOW (soft rustling)
//   snow         → both near zero (acoustic dampening)
//   flat_surfaces  → low MIC_HIGH, variable MIC_LOW (hard, smooth surface)
// ---------------------------------------------------------------------------
void readMicBands(float *micLow, float *micHigh) {
  const int samples = 64;
  float smooth = 0.0f;
  float sumLow = 0.0f, sumHigh = 0.0f;
  for (int i = 0; i < samples; i++) {
    float v = (float)analogRead(MIC_PIN) - 512.0f;
    smooth    = 0.85f * smooth + 0.15f * v;   // IIR LPF ~750 Hz
    float hi  = v - smooth;                    // residual = high freq
    sumLow  += smooth * smooth;
    sumHigh += hi * hi;
    delayMicroseconds(125);  // ~8 kHz effective sample rate
  }
  *micLow  = sqrt(sumLow  / samples);
  *micHigh = sqrt(sumHigh / samples);
}

// ---------------------------------------------------------------------------
// LCD helpers
// ---------------------------------------------------------------------------
void lcdClearLine(int row) {
  lcd.setCursor(0, row);
  lcd.print("                ");  // 16 spaces
}

// Show label selection on row 1 only.
// Row 0 is owned by lcdShowInference() — do not touch it here.
// Row 1: "> label   (N/4)"
void lcdShowLabel(int idx) {
  lcdClearLine(1);
  lcd.setCursor(0, 1);
  lcd.print("> ");
  // Label truncated to 8 chars to leave room for " (N/4)"
  char buf[9];
  strncpy(buf, TERRAIN_LABELS[idx], 8);
  buf[8] = '\0';
  lcd.print(buf);
  // Position indicator e.g. "(2/4)"
  lcd.setCursor(11, 1);
  lcd.print("(");
  lcd.print(idx + 1);
  lcd.print("/");
  lcd.print(NUM_TERRAINS);
  lcd.print(")");
}

// Show recording-in-progress on row 1 only.
// Row 0 is owned by lcdShowInference() — do not touch it here.
// Row 1: "REC:grav N=1234 "  (first 4 chars of label + sample count)
void lcdShowRecording(const char* label, unsigned long count) {
  lcd.setCursor(0, 1);
  char buf[17];
  snprintf(buf, sizeof(buf), "REC:%-4.4s N=%-5lu", label, count);
  lcd.print(buf);
}

// ---------------------------------------------------------------------------
// Button reading with debounce
// ---------------------------------------------------------------------------
// Returns: 0 = no event, 1 = short press released, 2 = long press detected
int readButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounce = millis();
  }
  lastButtonState = reading;

  if ((millis() - lastDebounce) < DEBOUNCE_MS) {
    return 0;
  }

  bool prevState = buttonState;
  buttonState = reading;

  // Detect press-down edge
  if (prevState == HIGH && buttonState == LOW) {
    btnDownTime = millis();
    return 0;
  }

  // Detect release edge
  if (prevState == LOW && buttonState == HIGH) {
    unsigned long held = millis() - btnDownTime;
    if (held >= LONG_PRESS_MS) {
      return 2;  // long press
    }
    return 1;    // short press
  }

  // While held, check if we've crossed long-press threshold (live detect)
  if (buttonState == LOW && (millis() - btnDownTime) >= LONG_PRESS_MS) {
    // Report long press once; prevent re-trigger until release
    btnDownTime = millis() + 100000UL;
    return 2;
  }

  return 0;
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Monitor.begin();
  // Register RPC handler so Linux inference thread can update the LCD.
  // Bridge is initialised by Monitor.begin(); provide() must come after it.
  Bridge.provide("set_terrain", set_terrain_rpc);
  delay(2000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin();  // MUST be before lcd.init() and sensor.begin()

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("< waiting...   >");  // row 0 = inference placeholder
  lcd.setCursor(0, 1);
  lcd.print("Init...         ");

  if (!accel.begin()) {
    Monitor.println("ERROR: LSM303 accel not found at 0x19");
    lcd.setCursor(0, 0);
    lcd.print("ACCEL FAIL      ");
    lcd.setCursor(0, 1);
    lcd.print("Check wiring    ");
    while (1) { delay(100); }
  }
  if (!mag.begin()) {
    Monitor.println("ERROR: LSM303 mag not found at 0x1E");
    lcd.setCursor(0, 0);
    lcd.print("MAG FAIL        ");
    lcd.setCursor(0, 1);
    lcd.print("Check wiring    ");
    while (1) { delay(100); }
  }

  // Mic health check — MAX4466 at rest should read near ADC midpoint (512).
  // Values outside 400–624 indicate disconnected mic or wrong supply voltage.
  // Block here until the user acknowledges — mic data will be garbage otherwise.
  int micVal = analogRead(MIC_PIN);
  if (micVal < MIC_REST_MIN || micVal > MIC_REST_MAX) {
    snprintf(_lineBuf, sizeof(_lineBuf),
      "WARN: Mic A2=%d (expect %d-%d) — check MAX4466",
      micVal, MIC_REST_MIN, MIC_REST_MAX);
    Monitor.println(_lineBuf);
    lcd.setCursor(0, 0);
    lcd.print("MIC FAIL        ");
    lcd.setCursor(0, 1);
    lcd.print("A2=");
    lcd.print(micVal);
    lcd.print(" btn:ok ");
    // Wait for button press to acknowledge — don't record with bad mic
    while (digitalRead(BUTTON_PIN) == HIGH) { delay(50); }
    while (digitalRead(BUTTON_PIN) == LOW)  { delay(50); }
  }

  Monitor.println("GRIP Data Collector ready.");
  Monitor.println("CSV: timestamp_ms,label,AX,AY,AZ,MX,MY,MZ,AX_HP,AY_HP,AZ_HP,VMAG,MIC_LOW,MIC_HIGH");

  lcd.setCursor(0, 1);
  lcd.print("Long press: rec ");
  delay(1500);

  // Enter label selection: row 0 = inference placeholder, row 1 = label
  currentState = STATE_SELECT_LABEL;
  lcdShowInference();
  lcdShowLabel(selectedTerrain);
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  // Apply inference result to row 0 whenever the Linux MPU sends a new one.
  // Safe to call from any state — row 0 is dedicated to inference display.
  if (_infer_pending) {
    _infer_pending = false;
    lcdShowInference();
  }

  int btnEvent = readButton();

  switch (currentState) {

    // -----------------------------------------------------------------------
    // LABEL SELECTION: short press cycles labels, long press confirms
    // -----------------------------------------------------------------------
    case STATE_SELECT_LABEL:
      if (btnEvent == 1) {
        // Short press — cycle to next label
        selectedTerrain = (selectedTerrain + 1) % NUM_TERRAINS;
        lcdShowLabel(selectedTerrain);
      }
      else if (btnEvent == 2) {
        // Long press — confirm and start recording
        sampleCount = 0;
        rawCount = 0;
        currentState = STATE_RECORDING;
        lastSampleTime = millis();

        // Reset HP filter so it re-settles cleanly during the warmup window
        ax_hp = 0.0f; ay_hp = 0.0f; az_hp = 0.0f;
        ax_prev = 0.0f; ay_prev = 0.0f; az_prev = 0.0f;

        // Row 0 keeps the inference display — only update row 1
        lcd.setCursor(0, 1);
        lcd.print("Warming up...   ");

        snprintf(_lineBuf, sizeof(_lineBuf),
          "START_RECORDING,label=%s", TERRAIN_LABELS[selectedTerrain]);
        Monitor.println(_lineBuf);
      }
      break;

    // -----------------------------------------------------------------------
    // RECORDING: sample at 100 Hz, stream CSV, short press stops
    // -----------------------------------------------------------------------
    case STATE_RECORDING:
      // Short press stops recording
      if (btnEvent == 1) {
        snprintf(_lineBuf, sizeof(_lineBuf),
          "STOP_RECORDING,sent=%lu,raw=%lu,trimmed_head=%d",
          sampleCount, rawCount, WARMUP_SAMPLES);
        Monitor.println(_lineBuf);

        // Show result on LCD for 2-3 seconds so user can read it.
        // Minimum needed: MIN_ROWS(200) + TAIL_TRIM(500) = 700 samples
        const unsigned long MIN_SAMPLES_NEEDED = 700;
        if (sampleCount < MIN_SAMPLES_NEEDED) {
          // Too short — session will be discarded by streamer (row 1 only)
          char _tb[17];
          snprintf(_tb, sizeof(_tb), "SHORT:%-10lu", sampleCount);
          lcd.setCursor(0, 1);
          lcd.print(_tb);
          delay(3000);
        } else {
          // Enough data — streamer will save it (row 1 only)
          char _sb[17];
          snprintf(_sb, sizeof(_sb), "Sent:%-11lu", sampleCount);
          lcd.setCursor(0, 1);
          lcd.print(_sb);
          delay(2000);
        }

        // Wait for button release so it doesn't immediately re-trigger
        while (digitalRead(BUTTON_PIN) == LOW) { delay(10); }

        currentState = STATE_SELECT_LABEL;
        lcdShowLabel(selectedTerrain);
        break;
      }

      // Sample at 100 Hz
      if (millis() - lastSampleTime >= SAMPLE_INTERVAL) {
        lastSampleTime += SAMPLE_INTERVAL;

        // Read sensors every tick (keeps timing consistent during warmup)
        sensors_event_t accelEvent;
        accel.getEvent(&accelEvent);

        sensors_event_t magEvent;
        mag.getEvent(&magEvent);

        // Two-band microphone
        float micLow, micHigh;
        readMicBands(&micLow, &micHigh);

        // High-pass filter — strip gravity, expose terrain vibration
        float ax = accelEvent.acceleration.x;
        float ay = accelEvent.acceleration.y;
        float az = accelEvent.acceleration.z;
        ax_hp = HP_ALPHA * (ax_hp + ax - ax_prev);
        ay_hp = HP_ALPHA * (ay_hp + ay - ay_prev);
        az_hp = HP_ALPHA * (az_hp + az - az_prev);
        ax_prev = ax;  ay_prev = ay;  az_prev = az;

        // Vibration magnitude — rotation-invariant total vibration energy
        float vmag = sqrt(ax_hp*ax_hp + ay_hp*ay_hp + az_hp*az_hp);

        rawCount++;

        // Skip first 3 seconds — startup noise from placing car / accelerating
        // HP filter also fully settles during this window (time const ~200ms)
        if (rawCount <= WARMUP_SAMPLES) {
          // Update LCD countdown once per second
          if (rawCount % SAMPLE_RATE_HZ == 0) {
            int secsLeft = (WARMUP_SAMPLES - rawCount) / SAMPLE_RATE_HZ;
            lcd.setCursor(0, 1);
            if (secsLeft > 0) {
              lcd.print("Starting in ");
              lcd.print(secsLeft);
              lcd.print("s ");
            }
          }
          // Transition to recording display when warmup ends
          if (rawCount == WARMUP_SAMPLES) {
            Monitor.println("WARMUP_DONE");
            lcdShowRecording(TERRAIN_LABELS[selectedTerrain], 0);
          }
          break;
        }

        // Stream 14-field CSV row.
        // Single println() to avoid RouterBridge fragmentation.
        snprintf(_lineBuf, sizeof(_lineBuf),
          "%lu,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
          millis(), TERRAIN_LABELS[selectedTerrain],
          (double)ax,    (double)ay,    (double)az,
          (double)magEvent.magnetic.x,
          (double)magEvent.magnetic.y,
          (double)magEvent.magnetic.z,
          (double)ax_hp, (double)ay_hp, (double)az_hp,
          (double)vmag,
          (double)micLow, (double)micHigh);
        Monitor.println(_lineBuf);

        sampleCount++;

        // Update LCD every 50 samples to reduce I2C overhead
        if (sampleCount % 50 == 0) {
          lcdShowRecording(TERRAIN_LABELS[selectedTerrain], sampleCount);
        }
      }
      break;

    default:
      currentState = STATE_SELECT_LABEL;
      break;
  }
}
