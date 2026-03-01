/*
 * GRIP_comms_test.ino — Data Communication Test
 * Ground Recognition Intelligence Platform
 *
 * Tests the full data pipeline that GRIP_collect uses: MCU reads sensors,
 * formats CSV, and streams it over RouterBridge to the Linux MPU side.
 * This lets you verify that data actually reaches the other side before
 * you start collecting real training data.
 *
 * What it does:
 *   Phase 1 — ECHO TEST (first 30 seconds)
 *     Sends numbered heartbeat lines every second so you can verify the
 *     Monitor/RouterBridge link is working.  On the Linux MPU side, run
 *     GRIP_wifi_stream.py (or nc 127.0.0.1 7500) and confirm the lines
 *     appear.
 *
 *   Phase 2 — CSV BURST (next 10 seconds)
 *     Sends 100 Hz sensor CSV identical to GRIP_collect format for 10 s
 *     (1000 rows).  You can pipe this into GRIP_wifi_stream.py and then
 *     download it with GRIP_laptop_logger.py to test the full pipeline.
 *
 *   Phase 3 — INTERACTIVE (runs forever)
 *     Button controls:
 *       Short press  → send a single CSV row on demand
 *       Long press   → start/stop continuous 100 Hz streaming
 *     LCD shows row count and streaming state.
 *
 * This sketch is safe to run repeatedly — it doesn't record to any file;
 * all data goes to Monitor only.
 *
 * Hardware (same wiring as GRIP_collect):
 *   - LSM303DLHC accel (0x19) + mag (0x1E) via I2C
 *   - MAX4466 analog mic on A2
 *   - 16x2 LCD with PCF8574T backpack (0x27) via I2C
 *   - Tactile button on D2 (INPUT_PULLUP, LOW = pressed)
 *
 * Board rules (Arduino Uno Q / STM32U585):
 *   - Use Monitor.print() instead of Serial.print()
 *   - Call Wire.begin() before any I2C device init
 *   - No while(!Serial) — use delay(2000)
 *   - 3.3V logic only
 */

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// ---------------------------------------------------------------------------
// Hardware pins
// ---------------------------------------------------------------------------
#define BUTTON_PIN  2
#define MIC_PIN     A2

// ---------------------------------------------------------------------------
// Peripheral objects
// ---------------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(12345);

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
enum Phase {
  PHASE_ECHO,         // heartbeat echo test
  PHASE_CSV_BURST,    // 10 s of 100 Hz CSV
  PHASE_INTERACTIVE   // button-controlled
};

Phase currentPhase = PHASE_ECHO;

// Echo phase
unsigned long echoStart    = 0;
int           echoCount    = 0;
unsigned long lastEchoTime = 0;
#define ECHO_DURATION_MS  30000  // 30 seconds
#define ECHO_INTERVAL_MS  1000   // 1 per second

// CSV burst phase
unsigned long burstStart     = 0;
unsigned long burstRowCount  = 0;
unsigned long lastBurstTime  = 0;
#define BURST_DURATION_MS 10000  // 10 seconds
#define SAMPLE_INTERVAL   10     // 100 Hz

// Interactive phase
bool          streaming      = false;
unsigned long interRowCount  = 0;
unsigned long lastStreamTime = 0;

// Button
#define LONG_PRESS_MS  1000
#define DEBOUNCE_MS    50
bool     lastButtonState  = HIGH;
bool     buttonState      = HIGH;
unsigned long btnDownTime  = 0;
unsigned long lastDebounce = 0;

// ---------------------------------------------------------------------------
// Sensor helpers
// ---------------------------------------------------------------------------
float readMicRMS() {
  const int samples = 64;
  float sum = 0.0;
  for (int i = 0; i < samples; i++) {
    float v = analogRead(MIC_PIN) - 512.0;
    sum += v * v;
    delayMicroseconds(125);
  }
  return sqrt(sum / samples);
}

// Send one CSV row matching GRIP_collect format:
// timestamp_ms,label,AX,AY,AZ,MX,MY,MZ,MIC_RMS
//
// IMPORTANT: Build the entire line in a buffer and send it as a single
// Monitor.println() call.  The RouterBridge multiplexes the UART, so
// individual print() calls can be fragmented / interleaved, producing
// garbled output on the Linux side.  One atomic println() avoids this.
char _csvBuf[160];  // shared buffer (only used from loop, no concurrency)

void sendCSVRow(const char* label) {
  sensors_event_t accelEvent, magEvent;
  accel.getEvent(&accelEvent);
  mag.getEvent(&magEvent);
  float micRMS = readMicRMS();

  snprintf(_csvBuf, sizeof(_csvBuf),
    "%lu,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
    millis(), label,
    (double)accelEvent.acceleration.x,
    (double)accelEvent.acceleration.y,
    (double)accelEvent.acceleration.z,
    (double)magEvent.magnetic.x,
    (double)magEvent.magnetic.y,
    (double)magEvent.magnetic.z,
    (double)micRMS);
  Monitor.println(_csvBuf);
}

// ---------------------------------------------------------------------------
// Button reader (returns 0=none, 1=short, 2=long)
// ---------------------------------------------------------------------------
int readButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounce = millis();
  }
  lastButtonState = reading;

  if ((millis() - lastDebounce) < DEBOUNCE_MS) return 0;

  bool prev = buttonState;
  buttonState = reading;

  if (prev == HIGH && buttonState == LOW) {
    btnDownTime = millis();
    return 0;
  }

  if (prev == LOW && buttonState == HIGH) {
    return (millis() - btnDownTime >= LONG_PRESS_MS) ? 2 : 1;
  }

  if (buttonState == LOW && (millis() - btnDownTime) >= LONG_PRESS_MS) {
    btnDownTime = millis() + 100000;
    return 2;
  }

  return 0;
}

// ---------------------------------------------------------------------------
// LCD helpers
// ---------------------------------------------------------------------------
void lcdClear() {
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Monitor.begin();
  delay(2000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin();

  lcd.init();
  lcd.backlight();

  // Init sensors (non-fatal here — echo test still works)
  bool accelOK = accel.begin();
  bool magOK   = mag.begin();

  Monitor.println("╔══════════════════════════════════════════════════╗");
  Monitor.println("║     GRIP Communications Test                    ║");
  Monitor.println("║     Arduino Uno Q  (STM32U585)                  ║");
  Monitor.println("╚══════════════════════════════════════════════════╝");
  Monitor.println();

  Monitor.print("  Accel: ");
  Monitor.println(accelOK ? "OK" : "FAIL (CSV burst will have zeros)");
  Monitor.print("  Mag:   ");
  Monitor.println(magOK   ? "OK" : "FAIL (CSV burst will have zeros)");
  Monitor.println();

  // --- Start Phase 1: Echo ---
  Monitor.println("━━━ PHASE 1: Echo Test (30s) ━━━━━━━━━━━━━━━━━━━━");
  Monitor.println("Open the Linux MPU bridge to see these lines:");
  Monitor.println("  nc 127.0.0.1 7500");
  Monitor.println("  — or —");
  Monitor.println("  python3 GRIP_wifi_stream.py");
  Monitor.println();

  lcdClear();
  lcd.setCursor(0, 0);
  lcd.print("COMMS TEST");
  lcd.setCursor(0, 1);
  lcd.print("Echo phase...");

  echoStart = millis();
  lastEchoTime = millis();
  currentPhase = PHASE_ECHO;
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  int btn = readButton();

  switch (currentPhase) {

    // =====================================================================
    // PHASE 1: ECHO — numbered heartbeat lines once per second for 30s
    // =====================================================================
    case PHASE_ECHO: {
      if (millis() - lastEchoTime >= ECHO_INTERVAL_MS) {
        lastEchoTime += ECHO_INTERVAL_MS;
        echoCount++;

        int secsLeft = (ECHO_DURATION_MS - (millis() - echoStart)) / 1000;

        snprintf(_csvBuf, sizeof(_csvBuf),
          "ECHO #%d | uptime_ms=%lu | remaining=%ds",
          echoCount, millis(), secsLeft);
        Monitor.println(_csvBuf);

        // Update LCD countdown
        lcd.setCursor(0, 1);
        lcd.print("Echo #");
        lcd.print(echoCount);
        lcd.print(" ");
        lcd.print(secsLeft);
        lcd.print("s   ");
      }

      // Transition to Phase 2 after 30s
      if (millis() - echoStart >= ECHO_DURATION_MS) {
        snprintf(_csvBuf, sizeof(_csvBuf),
          "\nEcho test done — sent %d heartbeats.", echoCount);
        Monitor.println(_csvBuf);
        Monitor.println("If you saw them on the Linux side, the bridge works!\n");

        // Start Phase 2
        Monitor.println("━━━ PHASE 2: CSV Burst (10s @ 100Hz) ━━━━━━━━━━━━");
        Monitor.println("This sends 1000 rows of real sensor CSV.");
        Monitor.println("CSV: timestamp_ms,label,AX,AY,AZ,MX,MY,MZ,MIC_RMS");
        Monitor.println();
        Monitor.println("START_RECORDING,label=test_burst");

        lcdClear();
        lcd.setCursor(0, 0);
        lcd.print("CSV Burst 100Hz");

        burstStart = millis();
        lastBurstTime = millis();
        burstRowCount = 0;
        currentPhase = PHASE_CSV_BURST;
      }
      break;
    }

    // =====================================================================
    // PHASE 2: CSV BURST — 10 seconds of 100 Hz sensor data
    // =====================================================================
    case PHASE_CSV_BURST: {
      if (millis() - lastBurstTime >= SAMPLE_INTERVAL) {
        lastBurstTime += SAMPLE_INTERVAL;
        sendCSVRow("test_burst");
        burstRowCount++;

        // Update LCD every 100 rows
        if (burstRowCount % 100 == 0) {
          lcd.setCursor(0, 1);
          lcd.print("Rows: ");
          lcd.print(burstRowCount);
          lcd.print("      ");
        }
      }

      // Transition to Phase 3 after 10s
      if (millis() - burstStart >= BURST_DURATION_MS) {
        snprintf(_csvBuf, sizeof(_csvBuf),
          "STOP_RECORDING,sent=%lu,raw=%lu,trimmed_head=0",
          burstRowCount, burstRowCount);
        Monitor.println(_csvBuf);
        snprintf(_csvBuf, sizeof(_csvBuf),
          "\nCSV burst done — sent %lu rows.\n", burstRowCount);
        Monitor.println(_csvBuf);

        Monitor.println("━━━ PHASE 3: Interactive Mode ━━━━━━━━━━━━━━━━━━━");
        Monitor.println("Controls:");
        Monitor.println("  Short press → send one CSV row");
        Monitor.println("  Long press  → start/stop continuous 100Hz stream");
        Monitor.println();

        lcdClear();
        lcd.setCursor(0, 0);
        lcd.print("Interactive");
        lcd.setCursor(0, 1);
        lcd.print("Btn=send LP=strm");

        interRowCount = 0;
        streaming = false;
        currentPhase = PHASE_INTERACTIVE;
      }
      break;
    }

    // =====================================================================
    // PHASE 3: INTERACTIVE — button-controlled sending
    // =====================================================================
    case PHASE_INTERACTIVE: {
      if (btn == 1) {
        // Short press — send one row
        if (!streaming) {
          sendCSVRow("manual_shot");
          interRowCount++;
          snprintf(_csvBuf, sizeof(_csvBuf),
            "  ^ single row #%lu", interRowCount);
          Monitor.println(_csvBuf);

          lcd.setCursor(0, 0);
          lcd.print("Sent row #");
          lcd.print(interRowCount);
          lcd.print("     ");
          lcd.setCursor(0, 1);
          lcd.print("ShortP=1 LP=strm");
        } else {
          // Short press while streaming → stop
          streaming = false;
          snprintf(_csvBuf, sizeof(_csvBuf),
            "STOP_RECORDING,sent=%lu", interRowCount);
          Monitor.println(_csvBuf);
          Monitor.println("  Stream stopped.");

          lcd.setCursor(0, 0);
          lcd.print("Stopped         ");
          lcd.setCursor(0, 1);
          lcd.print("Total: ");
          lcd.print(interRowCount);
          lcd.print("      ");
        }
      }

      if (btn == 2) {
        // Long press — toggle streaming
        if (!streaming) {
          streaming = true;
          lastStreamTime = millis();
          Monitor.println("START_RECORDING,label=interactive_stream");
          Monitor.println("  Streaming at 100Hz... short press to stop.");

          lcd.setCursor(0, 0);
          lcd.print("STREAMING 100Hz ");
        } else {
          streaming = false;
          snprintf(_csvBuf, sizeof(_csvBuf),
            "STOP_RECORDING,sent=%lu", interRowCount);
          Monitor.println(_csvBuf);
          Monitor.println("  Stream stopped.");

          lcd.setCursor(0, 0);
          lcd.print("Stopped         ");
          lcd.setCursor(0, 1);
          lcd.print("Total: ");
          lcd.print(interRowCount);
          lcd.print("      ");
        }
      }

      // Continuous streaming at 100 Hz
      if (streaming && (millis() - lastStreamTime >= SAMPLE_INTERVAL)) {
        lastStreamTime += SAMPLE_INTERVAL;
        sendCSVRow("stream");
        interRowCount++;

        if (interRowCount % 50 == 0) {
          lcd.setCursor(0, 1);
          lcd.print("N=");
          lcd.print(interRowCount);
          lcd.print("          ");
        }
      }
      break;
    }
  }
}
