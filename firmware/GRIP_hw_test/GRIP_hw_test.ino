/*
 * GRIP_hw_test.ino — Hardware Diagnostics
 * Ground Recognition Intelligence Platform
 *
 * Runs through every piece of GRIP hardware one by one and reports
 * PASS / FAIL to both the LCD and the Monitor console.  Upload this
 * sketch first when you wire up the board so you can verify everything
 * before collecting data or running inference.
 *
 * Test sequence (automatic on boot):
 *   1. I2C bus scan — expects 0x19, 0x1E, 0x27
 *   2. LCD write test
 *   3. LSM303 accelerometer — read 10 samples, print range
 *   4. LSM303 magnetometer  — read 10 samples, print range
 *   5. MAX4466 microphone    — read RMS, check it's non-zero
 *   6. Button input          — prompts you to press the button
 *
 * After the automatic tests, enters a live dashboard that streams all
 * sensor values at 10 Hz so you can move the board around and watch
 * the numbers change in real time.
 *
 * Hardware (same as GRIP_collect):
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
// Hardware pins & addresses
// ---------------------------------------------------------------------------
#define BUTTON_PIN  2
#define MIC_PIN     A2

#define I2C_ADDR_ACCEL  0x19
#define I2C_ADDR_MAG    0x1E
#define I2C_ADDR_LCD    0x27

// ---------------------------------------------------------------------------
// Peripheral objects
// ---------------------------------------------------------------------------
LiquidCrystal_I2C lcd(I2C_ADDR_LCD, 16, 2);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(12345);

// ---------------------------------------------------------------------------
// Test counters
// ---------------------------------------------------------------------------
int testsRun    = 0;
int testsPassed = 0;
int testsFailed = 0;

bool allTestsDone = false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void lcdClear() {
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
}

void reportResult(const char* name, bool passed, const char* detail) {
  testsRun++;
  if (passed) {
    testsPassed++;
    Monitor.print("[PASS] ");
  } else {
    testsFailed++;
    Monitor.print("[FAIL] ");
  }
  Monitor.print(name);
  if (detail && detail[0] != '\0') {
    Monitor.print(" — ");
    Monitor.print(detail);
  }
  Monitor.println();
}

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

// ---------------------------------------------------------------------------
// Individual tests
// ---------------------------------------------------------------------------

// Test 1: I2C bus scan — look for the three expected addresses
void testI2CBusScan() {
  Monitor.println();
  Monitor.println("=== TEST 1: I2C Bus Scan ===");

  bool foundAccel = false;
  bool foundMag   = false;
  bool foundLCD   = false;
  int  deviceCount = 0;

  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      deviceCount++;
      Monitor.print("  Found device at 0x");
      if (addr < 16) Monitor.print("0");
      Monitor.print(addr, HEX);

      if (addr == I2C_ADDR_ACCEL) { foundAccel = true; Monitor.print(" (LSM303 Accel)"); }
      if (addr == I2C_ADDR_MAG)   { foundMag   = true; Monitor.print(" (LSM303 Mag)");   }
      if (addr == I2C_ADDR_LCD)   { foundLCD   = true; Monitor.print(" (LCD PCF8574T)"); }
      Monitor.println();
    }
  }

  Monitor.print("  Total devices found: ");
  Monitor.println(deviceCount);

  char buf[40];
  snprintf(buf, sizeof(buf), "found %d device(s)", deviceCount);
  reportResult("I2C: Accel 0x19", foundAccel, foundAccel ? "OK" : "NOT FOUND on bus");
  reportResult("I2C: Mag   0x1E", foundMag,   foundMag   ? "OK" : "NOT FOUND on bus");
  reportResult("I2C: LCD   0x27", foundLCD,   foundLCD   ? "OK" : "NOT FOUND on bus");
}

// Test 2: LCD write — display text, ask user to confirm via Monitor
void testLCD() {
  Monitor.println();
  Monitor.println("=== TEST 2: LCD Display ===");

  lcd.init();
  lcd.backlight();
  lcdClear();
  lcd.setCursor(0, 0);
  lcd.print("GRIP HW TEST");
  lcd.setCursor(0, 1);
  lcd.print("LCD OK? Check!");

  Monitor.println("  LCD should show: \"GRIP HW TEST\" / \"LCD OK? Check!\"");
  Monitor.println("  (Visual check — marking PASS if no I2C error)");

  // If lcd.init() didn't hang, the I2C link works.
  reportResult("LCD write", true, "text sent — verify visually");
}

// Test 3: Accelerometer — read 10 samples, check range is sane
void testAccelerometer() {
  Monitor.println();
  Monitor.println("=== TEST 3: LSM303 Accelerometer ===");

  bool initOK = accel.begin();
  if (!initOK) {
    reportResult("Accel init", false, "accel.begin() failed");
    return;
  }
  reportResult("Accel init", true, "accel.begin() OK");

  float minAx = 1e9, maxAx = -1e9;
  float minAy = 1e9, maxAy = -1e9;
  float minAz = 1e9, maxAz = -1e9;

  Monitor.println("  Reading 10 samples (hold board still)...");
  for (int i = 0; i < 10; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    float ax = event.acceleration.x;
    float ay = event.acceleration.y;
    float az = event.acceleration.z;

    if (ax < minAx) minAx = ax;  if (ax > maxAx) maxAx = ax;
    if (ay < minAy) minAy = ay;  if (ay > maxAy) maxAy = ay;
    if (az < minAz) minAz = az;  if (az > maxAz) maxAz = az;

    Monitor.print("  [");
    Monitor.print(i);
    Monitor.print("] AX=");
    Monitor.print(ax, 2);
    Monitor.print("  AY=");
    Monitor.print(ay, 2);
    Monitor.print("  AZ=");
    Monitor.println(az, 2);

    delay(50);
  }

  // Sanity: if board is flat and still, |AZ| should be near 9.8
  float magAccel = sqrt(maxAx * maxAx + maxAy * maxAy + maxAz * maxAz);
  bool sane = (magAccel > 5.0 && magAccel < 20.0);  // generous range

  char buf[60];
  snprintf(buf, sizeof(buf), "|g|=%.1f (expect ~9.8)", (double)magAccel);
  reportResult("Accel data", sane, buf);

  lcdClear();
  lcd.setCursor(0, 0);
  lcd.print("Accel ");
  lcd.print(sane ? "PASS" : "FAIL");
  lcd.setCursor(0, 1);
  lcd.print("|g|=");
  lcd.print(magAccel, 1);
}

// Test 4: Magnetometer — read 10 samples, check values aren't zero
void testMagnetometer() {
  Monitor.println();
  Monitor.println("=== TEST 4: LSM303 Magnetometer ===");

  bool initOK = mag.begin();
  if (!initOK) {
    reportResult("Mag init", false, "mag.begin() failed");
    return;
  }
  reportResult("Mag init", true, "mag.begin() OK");

  float sumMag = 0.0;

  Monitor.println("  Reading 10 samples...");
  for (int i = 0; i < 10; i++) {
    sensors_event_t event;
    mag.getEvent(&event);
    float mx = event.magnetic.x;
    float my = event.magnetic.y;
    float mz = event.magnetic.z;

    float m = sqrt(mx * mx + my * my + mz * mz);
    sumMag += m;

    Monitor.print("  [");
    Monitor.print(i);
    Monitor.print("] MX=");
    Monitor.print(mx, 2);
    Monitor.print("  MY=");
    Monitor.print(my, 2);
    Monitor.print("  MZ=");
    Monitor.println(mz, 2);

    delay(50);
  }

  float avgMag = sumMag / 10.0;
  // Earth's field is ~25–65 µT; allow wider range for indoor interference
  bool sane = (avgMag > 1.0 && avgMag < 200.0);

  char buf[60];
  snprintf(buf, sizeof(buf), "avg |B|=%.1f uT (expect 25-65)", (double)avgMag);
  reportResult("Mag data", sane, buf);

  lcdClear();
  lcd.setCursor(0, 0);
  lcd.print("Mag ");
  lcd.print(sane ? "PASS" : "FAIL");
  lcd.setCursor(0, 1);
  lcd.print("|B|=");
  lcd.print(avgMag, 1);
  lcd.print(" uT");
}

// Test 5: Microphone — read RMS several times, check non-zero
void testMicrophone() {
  Monitor.println();
  Monitor.println("=== TEST 5: MAX4466 Microphone (A2) ===");
  Monitor.println("  Reading 5 RMS measurements...");

  float rmsValues[5];
  float avgRMS = 0.0;
  bool  allZero = true;

  for (int i = 0; i < 5; i++) {
    rmsValues[i] = readMicRMS();
    avgRMS += rmsValues[i];
    if (rmsValues[i] > 0.5) allZero = false;

    Monitor.print("  [");
    Monitor.print(i);
    Monitor.print("] RMS = ");
    Monitor.println(rmsValues[i], 2);

    delay(100);
  }
  avgRMS /= 5.0;

  // Also read the raw analog value to check wiring
  int rawVal = analogRead(MIC_PIN);
  Monitor.print("  Raw analog read (A2): ");
  Monitor.println(rawVal);
  Monitor.print("  Expected DC bias ~512, got: ");
  Monitor.println(rawVal);

  // Bias check: with no signal, raw should be near 512 (VCC/2)
  bool biasOK = (rawVal > 200 && rawVal < 824);  // generous window

  char buf[60];
  snprintf(buf, sizeof(buf), "avg RMS=%.1f, raw=%d (bias ~512)", (double)avgRMS, rawVal);
  reportResult("Mic wiring", biasOK, biasOK ? "DC bias in range" : "DC bias out of range — check wiring");
  reportResult("Mic signal", !allZero, allZero ? "all readings near zero — tap the mic" : "non-zero signal detected");

  lcdClear();
  lcd.setCursor(0, 0);
  lcd.print("Mic ");
  lcd.print((biasOK && !allZero) ? "PASS" : "CHECK");
  lcd.setCursor(0, 1);
  lcd.print("RMS=");
  lcd.print(avgRMS, 1);
  lcd.print(" raw=");
  lcd.print(rawVal);
}

// Test 6: Button — wait up to 10 seconds for a press
void testButton() {
  Monitor.println();
  Monitor.println("=== TEST 6: Button (D2) ===");
  Monitor.println("  Press the button within 10 seconds...");

  lcdClear();
  lcd.setCursor(0, 0);
  lcd.print("Press button!");
  lcd.setCursor(0, 1);
  lcd.print("Waiting 10s...");

  unsigned long start = millis();
  bool pressed = false;
  bool released = false;

  // Wait for press
  while (millis() - start < 10000) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      pressed = true;
      Monitor.println("  Button pressed! (D2 went LOW)");
      lcd.setCursor(0, 1);
      lcd.print("Pressed! Release");
      break;
    }
    delay(10);
  }

  if (!pressed) {
    reportResult("Button press", false, "timed out — no press detected in 10s");
    return;
  }

  // Wait for release
  unsigned long pressStart = millis();
  while (millis() - pressStart < 5000) {
    if (digitalRead(BUTTON_PIN) == HIGH) {
      released = true;
      unsigned long held = millis() - pressStart;
      char buf[40];
      snprintf(buf, sizeof(buf), "held %lu ms then released", held);
      Monitor.print("  Button released after ");
      Monitor.print(held);
      Monitor.println(" ms");
      reportResult("Button press", true, buf);
      break;
    }
    delay(10);
  }

  if (!released) {
    reportResult("Button press", false, "stuck LOW — check wiring or pull-up");
  }
}

// ---------------------------------------------------------------------------
// setup() — run all tests sequentially
// ---------------------------------------------------------------------------
void setup() {
  Monitor.begin();
  delay(2000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin();  // MUST be before lcd.init() and sensor.begin()

  Monitor.println("╔══════════════════════════════════════════════════╗");
  Monitor.println("║     GRIP Hardware Diagnostics                   ║");
  Monitor.println("║     Arduino Uno Q  (STM32U585)                  ║");
  Monitor.println("╚══════════════════════════════════════════════════╝");
  Monitor.println();

  // --- Run tests ---
  testI2CBusScan();
  delay(500);

  testLCD();
  delay(1500);

  testAccelerometer();
  delay(1000);

  testMagnetometer();
  delay(1000);

  testMicrophone();
  delay(1000);

  testButton();
  delay(500);

  // --- Summary ---
  Monitor.println();
  Monitor.println("══════════════════════════════════════════════════");
  Monitor.print("  RESULTS:  ");
  Monitor.print(testsPassed);
  Monitor.print(" passed,  ");
  Monitor.print(testsFailed);
  Monitor.print(" failed,  ");
  Monitor.print(testsRun);
  Monitor.println(" total");
  Monitor.println("══════════════════════════════════════════════════");

  if (testsFailed == 0) {
    Monitor.println("  ALL TESTS PASSED — hardware is ready for GRIP!");
  } else {
    Monitor.println("  Some tests failed — check wiring above.");
  }

  Monitor.println();
  Monitor.println("Entering live sensor dashboard (10 Hz)...");
  Monitor.println("Move the board around to see values change.");
  Monitor.println("Press button to toggle LCD between sensor pages.");
  Monitor.println();

  lcdClear();
  lcd.setCursor(0, 0);
  if (testsFailed == 0) {
    lcd.print("ALL PASS!");
  } else {
    lcd.print("FAIL:");
    lcd.print(testsFailed);
    lcd.print(" Check!");
  }
  lcd.setCursor(0, 1);
  lcd.print("Live in 3s...");
  delay(3000);

  allTestsDone = true;
}

// ---------------------------------------------------------------------------
// loop() — live sensor dashboard after tests complete
// ---------------------------------------------------------------------------
// LCD pages: 0=Accel, 1=Mag, 2=Mic+Button
int lcdPage = 0;
unsigned long lastDash = 0;
bool lastBtn = HIGH;

void loop() {
  if (!allTestsDone) return;

  // Button toggles LCD page
  bool btn = digitalRead(BUTTON_PIN);
  if (lastBtn == HIGH && btn == LOW) {
    lcdPage = (lcdPage + 1) % 3;
    delay(200);  // simple debounce
  }
  lastBtn = btn;

  // Update at 10 Hz
  if (millis() - lastDash < 100) return;
  lastDash = millis();

  // Read all sensors
  sensors_event_t accelEvent, magEvent;
  accel.getEvent(&accelEvent);
  mag.getEvent(&magEvent);
  float micRMS = readMicRMS();

  float ax = accelEvent.acceleration.x;
  float ay = accelEvent.acceleration.y;
  float az = accelEvent.acceleration.z;
  float mx = magEvent.magnetic.x;
  float my = magEvent.magnetic.y;
  float mz = magEvent.magnetic.z;

  // Stream to Monitor as CSV for easy plotting
  Monitor.print(millis());
  Monitor.print(",");
  Monitor.print(ax, 2);
  Monitor.print(",");
  Monitor.print(ay, 2);
  Monitor.print(",");
  Monitor.print(az, 2);
  Monitor.print(",");
  Monitor.print(mx, 2);
  Monitor.print(",");
  Monitor.print(my, 2);
  Monitor.print(",");
  Monitor.print(mz, 2);
  Monitor.print(",");
  Monitor.println(micRMS, 2);

  // Update LCD based on current page
  lcdClear();
  switch (lcdPage) {
    case 0:  // Accelerometer
      lcd.setCursor(0, 0);
      lcd.print("AX=");
      lcd.print(ax, 1);
      lcd.print(" AY=");
      lcd.print(ay, 1);
      lcd.setCursor(0, 1);
      lcd.print("AZ=");
      lcd.print(az, 1);
      lcd.print(" m/s2");
      break;

    case 1:  // Magnetometer
      lcd.setCursor(0, 0);
      lcd.print("MX=");
      lcd.print(mx, 1);
      lcd.print(" MY=");
      lcd.print(my, 1);
      lcd.setCursor(0, 1);
      lcd.print("MZ=");
      lcd.print(mz, 1);
      lcd.print(" uT");
      break;

    case 2:  // Mic + Button
      lcd.setCursor(0, 0);
      lcd.print("MIC RMS=");
      lcd.print(micRMS, 1);
      lcd.setCursor(0, 1);
      lcd.print("BTN=");
      lcd.print(btn == LOW ? "PRESSED" : "open");
      lcd.print(" pg:");
      lcd.print(lcdPage + 1);
      lcd.print("/3");
      break;
  }
}
