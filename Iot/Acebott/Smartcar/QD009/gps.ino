#include <Arduino.h>
#include <TinyGPS++.h>
#include <math.h>
/**
 * ACEBOTT ESP32-Max-V1.0 - Normal (Differential / Skid-Steer) Wheels
 * Improved GPS-only "drive North" test
 *
 * Hardware Pin Connections:
 *  - PWM1_PIN (19) : Motor Speed Control for Right Motors
 *  - PWM2_PIN (23) : Motor Speed Control for Left Motors
 *  - SHCP_PIN (18) : 74HC595 Shift Register Clock
 *  - STCP_PIN (17) : 74HC595 Shift Register Latch
 *  - DATA_PIN (5)  : 74HC595 Shift Register Data
 *  - EN_PIN   (16) : 74HC595 Output Enable (Active LOW)
 *  - LED_PIN  (2)  : Onboard LED
 *  - LEFT_LED_PIN  (4)  : External left LED
 *  - RIGHT_LED_PIN (33) : External right LED
 *  - GPS_RX_PIN (27) : QD009 GPS TX -> ESP32 RX
 *
 * Shift Register Bit Mapping:
 *  - M1 (Left Front)  : Forward = 128 (0x80), Backward = 64 (0x40)
 *  - M2 (Left Rear)   : Forward = 32  (0x20), Backward = 16 (0x10)
 *  - M3 (Right Front) : Forward = 2   (0x02), Backward = 4  (0x04)
 *  - M4 (Right Rear)  : Forward = 1   (0x01), Backward = 8  (0x08)
 *
 * Standard Direction Byte Values:
 *  - STOP       : 0   (0x00)
 *  - FORWARD    : 163 (0xA3)
 *  - BACKWARD   : 92  (0x5C)
 *  - SPIN_LEFT  : 83  (0x53)   [Left Backward, Right Forward]
 *  - SPIN_RIGHT : 172 (0xAC)  [Left Forward, Right Backward]
 */
// ===================== PIN DEFINITIONS =====================
#define PWM1_PIN  19  // Right motors PWM
#define PWM2_PIN  23  // Left motors PWM
#define SHCP_PIN  18
#define STCP_PIN  17
#define DATA_PIN  5
#define EN_PIN    16
#define LED_PIN   2
#define LEFT_LED_PIN   4
#define RIGHT_LED_PIN  33
#define GPS_RX_PIN     27
#define GPS_TX_PIN     -1
#define GPS_BAUD       9600
// ===================== GPS NORTH TEST TUNING =====================
#define GPS_NORTH_DRIVE_MS        5500    // drive longer for clearer lat change
#define GPS_NORTH_TURN_MS         700     // in-place spin needs time once wheels actually roll
#define GPS_NORTH_SETTLE_MS       2800    // give GPS time to update after stop
#define GPS_NORTH_FIX_HOLD_MS     2500    // require a continuous fix before the first move
#define GPS_NORTH_MIN_LAT_GAIN    0.000007
#define GPS_NORTH_SPEED           155
#define GPS_NORTH_TURN_SPEED      255     // 4WD skid-steer spins stall well below full PWM
#define MOTOR_PWM_FREQ_HZ         500     // Acebott vehicle library default
#define TURN_MIN_SPEED            240     // in-place yaw needs much more torque than forward
#define AUTO_GPS_NORTH_ON_BOOT    true
#define GPS_COURSE_TOLERANCE_DEG  28.0    // how close to 0°/360° is "north"
#define GPS_MAX_CONSECUTIVE_TURNS 8       // safety limit
#define GPS_FIX_MAX_AGE_MS        5000    // 1 Hz modules plus a missed sentence used to trip 2500 ms
#define MOTOR_TRIM_DEFAULT        85      // calibrated: 4 s @ 200, straight (was 363 mm left at 0)
#define MOTOR_TRIM_MIN            -120
#define MOTOR_TRIM_MAX            120
#define STRAIGHT_TEST_MS          4000    // timed straight run for measuring pull
// ===================== MOTOR DIRECTION BYTES =====================
// If your robot spins the wrong way, swap SPIN_LEFT and SPIN_RIGHT values
const uint8_t DIR_STOP       = 0;
const uint8_t DIR_FORWARD    = 163;
const uint8_t DIR_BACKWARD   = 92;
const uint8_t DIR_SPIN_LEFT  = 83;    // Left reverse + Right forward
const uint8_t DIR_SPIN_RIGHT = 172;   // Left forward + Right reverse
const uint8_t DIR_LEFT_ONLY  = 160;
const uint8_t DIR_RIGHT_ONLY = 3;
// ===================== GLOBAL STATE =====================
uint8_t currentSpeed = 255;
bool continuousTestMode = false;
unsigned long continuousTestStepMs = 0;
uint8_t continuousStep = 0;
bool leftExternalLedOn = true;
unsigned long lastExternalLedFlashMs = 0;
HardwareSerial gpsSerial(1);
bool gpsRawMonitorEnabled = false;
bool gpsDataSeen = false;
unsigned long gpsCharacterCount = 0;
unsigned long gpsSentenceCount = 0;
unsigned long lastGpsStatusMs = 0;
TinyGPSPlus gps;
bool gpsNorthTestMode = false;
uint8_t gpsNorthState = 0;
double gpsNorthStartLat = 0.0;
unsigned long gpsNorthStepMs = 0;
unsigned long gpsNorthFixHeldMs = 0;
unsigned long lastNorthWaitLogMs = 0;
uint8_t consecutiveTurns = 0;
int motorTrim = MOTOR_TRIM_DEFAULT;  // +boosts left / -boosts right
uint8_t lastMotorDir = 0;
uint8_t lastMotorLeft = 0;
uint8_t lastMotorRight = 0;
enum GpsNorthState : uint8_t {
  NORTH_WAIT_FOR_FIX,
  NORTH_DRIVE_FORWARD,
  NORTH_COMPARE,
  NORTH_TURN
};
// ===================== FUNCTION PROTOTYPES =====================
void handleGpsTest();
void handleGpsNorthTest();
void setMotors(uint8_t directionByte, uint8_t leftSpeed, uint8_t rightSpeed);
void moveStop();
void moveForward(uint8_t speed = currentSpeed);
void moveBackward(uint8_t speed = currentSpeed);
void turnLeft(uint8_t speed = currentSpeed);
void turnRight(uint8_t speed = currentSpeed);
uint8_t turnPwm(uint8_t speed);
uint8_t clampPwm(int value);
void printMotorTrim();
void setMotorTrim(int value);
void runStraightBiasTest();
// ===================== GPS HELPERS =====================
bool hasFreshGpsFix() {
  return gps.location.isValid() && gps.location.age() < GPS_FIX_MAX_AGE_MS;
}

uint32_t gpsFixAgeMs() {
  return gps.location.isValid() ? gps.location.age() : 99999;
}

uint32_t gpsSatCount() {
  return gps.satellites.isValid() ? gps.satellites.value() : 0;
}
void gpsInit() {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  lastGpsStatusMs = millis();
}
void handleGpsTest() {
  while (gpsSerial.available() > 0) {
    char gpsChar = (char)gpsSerial.read();
    gps.encode(gpsChar);
    gpsDataSeen = true;
    gpsCharacterCount++;
    if (gpsChar == '\n') {
      gpsSentenceCount++;
    }
    if (gpsRawMonitorEnabled) {
      Serial.write(gpsChar);
    }
  }
  unsigned long now = millis();
  if (now - lastGpsStatusMs >= 5000) {
    lastGpsStatusMs = now;
    if (hasFreshGpsFix()) {
      Serial.printf("[GPS] fix | lat: %.6f | lng: %.6f | sats: %lu | course: %.1f | speed: %.1f km/h | chars: %lu\n",
                    gps.location.lat(),
                    gps.location.lng(),
                    gps.satellites.isValid() ? gps.satellites.value() : 0,
                    gps.course.isValid() ? gps.course.deg() : -1.0,
                    gps.speed.isValid() ? gps.speed.kmph() : 0.0,
                    gpsCharacterCount);
    } else {
      Serial.printf("[GPS] %s, no fresh fix | chars: %lu | NMEA lines: %lu\n",
                    gpsDataSeen ? "data detected" : "waiting for data",
                    gpsCharacterCount,
                    gpsSentenceCount);
    }
  // Both solid = motors commanded on. Fast alternate = GPS bytes seen.
  // Slow alternate = no GPS data yet.
  if (gpsNorthTestMode &&
      (gpsNorthState == NORTH_DRIVE_FORWARD || gpsNorthState == NORTH_TURN) &&
      (lastMotorLeft > 0 || lastMotorRight > 0)) {
    digitalWrite(LEFT_LED_PIN, HIGH);
    digitalWrite(RIGHT_LED_PIN, HIGH);
    return;
  }
  }
}
// ===================== LED HELPERS =====================
void setExternalLeds(bool leftOn) {
  digitalWrite(LEFT_LED_PIN, leftOn ? HIGH : LOW);
  digitalWrite(RIGHT_LED_PIN, leftOn ? LOW : HIGH);
}
void handleExternalLedFlash() {
  unsigned long now = millis();
  unsigned long flashIntervalMs = gpsDataSeen ? 150 : 500;
  if (now - lastExternalLedFlashMs >= flashIntervalMs) {
    lastExternalLedFlashMs = now;
    leftExternalLedOn = !leftExternalLedOn;
    setExternalLeds(leftExternalLedOn);
  }
}
void delayWithExternalLedFlash(unsigned long durationMs) {
  unsigned long startMs = millis();
  while (millis() - startMs < durationMs) {
    handleExternalLedFlash();
    handleGpsTest();
    delay(10);
  }
}
// ===================== MOTOR CONTROL =====================
void writePwmDuty(uint8_t pin, uint8_t val) {
  analogWrite(pin, val);
}
uint8_t turnPwm(uint8_t speed) {
  return (speed < TURN_MIN_SPEED) ? TURN_MIN_SPEED : speed;
}
uint8_t clampPwm(int value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return (uint8_t)value;
}
void printMotorTrim() {
  Serial.printf("Motor trim %d  (+ more LEFT PWM, - more RIGHT PWM)\n", motorTrim);
  Serial.printf("  Example @ speed %d → left %d, right %d\n",
                currentSpeed,
                clampPwm((int)currentSpeed + motorTrim),
                clampPwm((int)currentSpeed - motorTrim));
}
void setMotorTrim(int value) {
  if (value < MOTOR_TRIM_MIN) value = MOTOR_TRIM_MIN;
  if (value > MOTOR_TRIM_MAX) value = MOTOR_TRIM_MAX;
  motorTrim = value;
  printMotorTrim();
  if (lastMotorLeft != 0 || lastMotorRight != 0) {
    setMotors(lastMotorDir, lastMotorLeft, lastMotorRight);
  }
}
void motorInit() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
  setExternalLeds(leftExternalLedOn);
  lastExternalLedFlashMs = millis();
  pinMode(SHCP_PIN, OUTPUT);
  pinMode(STCP_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  analogWriteResolution(8);
  analogWriteFrequency(MOTOR_PWM_FREQ_HZ);
  writePwmDuty(PWM1_PIN, 0);
  writePwmDuty(PWM2_PIN, 0);
  digitalWrite(STCP_PIN, LOW);
  shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, DIR_STOP);
  digitalWrite(STCP_PIN, HIGH);
  digitalWrite(EN_PIN, LOW);   // OE active LOW
}
void setMotors(uint8_t directionByte, uint8_t leftSpeed, uint8_t rightSpeed) {
  lastMotorDir = directionByte;
  lastMotorLeft = leftSpeed;
  lastMotorRight = rightSpeed;
  uint8_t leftOut = leftSpeed;
  uint8_t rightOut = rightSpeed;
  if (leftSpeed > 0) {
    leftOut = clampPwm((int)leftSpeed + motorTrim);
  }
  if (rightSpeed > 0) {
    rightOut = clampPwm((int)rightSpeed - motorTrim);
  }
  digitalWrite(EN_PIN, LOW);
  writePwmDuty(PWM1_PIN, rightOut);
  writePwmDuty(PWM2_PIN, leftOut);
  digitalWrite(STCP_PIN, LOW);
  shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, directionByte);
  digitalWrite(STCP_PIN, HIGH);
}
void moveStop() {
  setMotors(DIR_STOP, 0, 0);
}
void moveForward(uint8_t speed) {
  setMotors(DIR_FORWARD, speed, speed);
}
void moveBackward(uint8_t speed) {
  setMotors(DIR_BACKWARD, speed, speed);
}
void turnLeft(uint8_t speed) {
  uint8_t pwm = turnPwm(speed);
  setMotors(DIR_SPIN_LEFT, pwm, pwm);
}
void turnRight(uint8_t speed) {
  uint8_t pwm = turnPwm(speed);
  setMotors(DIR_SPIN_RIGHT, pwm, pwm);
}
// ===================== GPS NORTH TEST =====================
void stopGpsNorthTest(const char *message) {
  if (gpsNorthTestMode && message != nullptr) {
    Serial.println(message);
  }
  gpsNorthTestMode = false;
  gpsNorthState = NORTH_WAIT_FOR_FIX;
  consecutiveTurns = 0;
  gpsNorthFixHeldMs = 0;
  moveStop();
}
void handleGpsNorthTest() {
  if (!gpsNorthTestMode) return;
  unsigned long now = millis();

  switch (gpsNorthState) {
    case NORTH_WAIT_FOR_FIX:
      moveStop();
      if (!hasFreshGpsFix()) {
        gpsNorthFixHeldMs = 0;
        if (now - lastNorthWaitLogMs >= 2000) {
          lastNorthWaitLogMs = now;
          Serial.printf("[North] Waiting for GPS lock | chars %lu | lines %lu | age %lu ms | sats %lu\n",
                        gpsCharacterCount,
                        gpsSentenceCount,
                        gpsFixAgeMs(),
                        gpsSatCount());
        }
        return;
      }
      if (gpsNorthFixHeldMs == 0) {
        gpsNorthFixHeldMs = now;
        Serial.printf("[North] Fix seen (lat %.6f, sats %lu). Holding %.1f s...\n",
                      gps.location.lat(), gpsSatCount(), GPS_NORTH_FIX_HOLD_MS / 1000.0);
        return;
      }
      if (now - gpsNorthFixHeldMs < GPS_NORTH_FIX_HOLD_MS) return;
      gpsNorthStartLat = gps.location.lat();
      gpsNorthStepMs = now;
      consecutiveTurns = 0;
      gpsNorthFixHeldMs = 0;
      gpsNorthState = NORTH_DRIVE_FORWARD;
      Serial.printf("[North] Baseline lat %.6f. Driving forward for %.1f s\n",
                    gpsNorthStartLat, GPS_NORTH_DRIVE_MS / 1000.0);
      moveForward(GPS_NORTH_SPEED);
      break;

    case NORTH_DRIVE_FORWARD:
      // Keep driving even if GPS age blips; cheap 1 Hz modules often exceed 2.5 s.
      if (now - gpsNorthStepMs >= GPS_NORTH_DRIVE_MS) {
        moveStop();
        gpsNorthStepMs = now;
        gpsNorthState = NORTH_COMPARE;
        Serial.println("[North] Stopped. Settling for GPS update...");
      }
      break;

    case NORTH_COMPARE: {
      if (now - gpsNorthStepMs < GPS_NORTH_SETTLE_MS) return;
      if (!hasFreshGpsFix()) {
        if (now - lastNorthWaitLogMs >= 2000) {
          lastNorthWaitLogMs = now;
          Serial.printf("[North] Settled, still no fresh location (age %lu ms). Waiting...\n",
                        gpsFixAgeMs());
        }
        return;
      }
      double currentLat = gps.location.lat();
      double latGain = currentLat - gpsNorthStartLat;
      bool hasCourse = gps.course.isValid() && gps.speed.kmph() > 0.4;
      double course = hasCourse ? gps.course.deg() : -1.0;
      bool goingNorthByLat = (latGain >= GPS_NORTH_MIN_LAT_GAIN);
      bool goingNorthByCourse = hasCourse &&
        (course < GPS_COURSE_TOLERANCE_DEG || course > (360.0 - GPS_COURSE_TOLERANCE_DEG));
      Serial.printf("[North] lat %.6f → %.6f (gain %.6f) | course %.1f° | sats %lu\n",
                    gpsNorthStartLat, currentLat, latGain, course, gpsSatCount());
      if (goingNorthByLat || goingNorthByCourse) {
        gpsNorthStartLat = currentLat;
        gpsNorthStepMs = now;
        consecutiveTurns = 0;
        gpsNorthState = NORTH_DRIVE_FORWARD;
        Serial.println("[North] Heading looks good → continue forward");
        moveForward(GPS_NORTH_SPEED);
      } else {
        if (consecutiveTurns >= GPS_MAX_CONSECUTIVE_TURNS) {
          stopGpsNorthTest("[North] Too many consecutive turns – stopping for safety.");
          return;
        }
        consecutiveTurns++;
        gpsNorthStepMs = now;
        gpsNorthState = NORTH_TURN;
        Serial.printf("[North] Not north enough (turn %d/%d). Spinning right @ %d...\n",
                      consecutiveTurns, GPS_MAX_CONSECUTIVE_TURNS, GPS_NORTH_TURN_SPEED);
        turnRight(GPS_NORTH_TURN_SPEED);   // change to turnLeft() if your mapping is inverted
      }
      break;
    }

    case NORTH_TURN:
      if (now - gpsNorthStepMs >= GPS_NORTH_TURN_MS) {
        moveStop();
        gpsNorthFixHeldMs = 0;
        gpsNorthState = NORTH_WAIT_FOR_FIX;   // force new baseline after turn
        Serial.println("[North] Turn finished. Taking new baseline.");
      }
      break;
  }
}
// ===================== DIAGNOSTIC =====================
void runWheelDiagnostic() {
  Serial.printf("\n--- Full-Power Wheel Diagnostic (Speed: %d/255) ---\n", currentSpeed);
  Serial.println("TIP: Ensure battery switch is ON (USB alone cannot power motors under load)");
  Serial.println("\n[1/6] FORWARD (3.0s)...");
  moveForward(currentSpeed);
  delayWithExternalLedFlash(3000);
  moveStop();
  delayWithExternalLedFlash(800);
  Serial.println("[2/6] BACKWARD (3.0s)...");
  moveBackward(currentSpeed);
  delayWithExternalLedFlash(3000);
  moveStop();
  delayWithExternalLedFlash(800);
  Serial.println("[3/6] SPIN LEFT (2.5s)...");
  turnLeft(currentSpeed);
  delayWithExternalLedFlash(2500);
  moveStop();
  delayWithExternalLedFlash(800);
  Serial.println("[4/6] SPIN RIGHT (2.5s)...");
  turnRight(currentSpeed);
  delayWithExternalLedFlash(2500);
  moveStop();
  delayWithExternalLedFlash(800);
  Serial.println("[5/6] LEFT WHEELS ONLY (2.0s)...");
  setMotors(DIR_LEFT_ONLY, currentSpeed, 0);
  delayWithExternalLedFlash(2000);
  moveStop();
  delayWithExternalLedFlash(800);
  Serial.println("[6/6] RIGHT WHEELS ONLY (2.0s)...");
  setMotors(DIR_RIGHT_ONLY, 0, currentSpeed);
  delayWithExternalLedFlash(2000);
  moveStop();
  delayWithExternalLedFlash(800);
  Serial.println("\n--- Wheel Diagnostic Complete ---\n");
}
void runStraightBiasTest() {
  Serial.println("\n--- Straight bias test ---");
  printMotorTrim();
  Serial.printf("Driving FORWARD %d ms at speed %d. Do not steer.\n", STRAIGHT_TEST_MS, currentSpeed);
  Serial.println("After it stops, measure:");
  Serial.println("  1) distance traveled (m or ft)");
  Serial.println("  2) sideways miss at the end (cm or in), LEFT or RIGHT of the start line");
  Serial.println("Example: 3.0 m forward, 40 cm left. Then use ] to add left PWM if it pulled left.");
  moveForward(currentSpeed);
  delayWithExternalLedFlash(STRAIGHT_TEST_MS);
  moveStop();
  Serial.println("STOPPED. Report distance + left/right offset (or press ] / [ and run b again).");
  printMotorTrim();
}
// ===================== SERIAL COMMANDS =====================
void printHelpMenu() {
  Serial.println("\n=========================================");
  Serial.println("  ACEBOTT ESP32 Normal Wheels + GPS North");
  Serial.printf("  Current Speed: %d / 255 | Trim: %d\n", currentSpeed, motorTrim);
  Serial.println("=========================================");
  Serial.println("External LEDs: slow = no GPS data, fast = GPS bytes seen, both solid = driving");
  Serial.println("Auto GPS-North test starts on boot (outdoors recommended)");
  Serial.println("Commands:");
  Serial.println("  w / s / a / d   = Forward / Backward / Spin Left / Spin Right");
  Serial.println("  space or x      = STOP");
  Serial.println("  t               = One-shot wheel diagnostic");
  Serial.println("  b               = Straight bias test (timed forward run)");
  Serial.println("  [ / ]           = Trim -1 / +1  (+ more left PWM if it pulls left)");
  Serial.println("  p               = Print motor trim");
  Serial.println("  c               = Toggle continuous loop test");
  Serial.println("  g               = Toggle raw NMEA monitor");
  Serial.println("  n               = Toggle GPS North test");
  Serial.println("  + / -           = Speed up / down");
  Serial.println("  1..5            = Speed presets (160 → 255)");
  Serial.println("  h or ?          = This help");
  Serial.println("=========================================\n");
}
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    if (cmd == '\r' || cmd == '\n') continue;
    switch (cmd) {
      case 'w': case 'W':
        stopGpsNorthTest(nullptr);
        continuousTestMode = false;
        Serial.printf("FORWARD @ %d\n", currentSpeed);
        moveForward(currentSpeed);
        break;
      case 's': case 'S':
        stopGpsNorthTest(nullptr);
        continuousTestMode = false;
        Serial.printf("BACKWARD @ %d\n", currentSpeed);
        moveBackward(currentSpeed);
        break;
      case 'a': case 'A':
        stopGpsNorthTest(nullptr);
        continuousTestMode = false;
        Serial.printf("SPIN LEFT @ %d\n", currentSpeed);
        turnLeft(currentSpeed);
        break;
      case 'd': case 'D':
        stopGpsNorthTest(nullptr);
        continuousTestMode = false;
        Serial.printf("SPIN RIGHT @ %d\n", currentSpeed);
        turnRight(currentSpeed);
        break;
      case ' ': case 'x': case 'X':
        stopGpsNorthTest("[North] stopped by user");
        continuousTestMode = false;
        Serial.println("STOPPED");
        moveStop();
        break;
      case 't': case 'T':
        stopGpsNorthTest(nullptr);
        continuousTestMode = false;
        runWheelDiagnostic();
        break;
      case 'b': case 'B':
        stopGpsNorthTest(nullptr);
        continuousTestMode = false;
        runStraightBiasTest();
        break;
      case ']':
        setMotorTrim(motorTrim + 1);
        break;
      case '[':
        setMotorTrim(motorTrim - 1);
        break;
      case 'p': case 'P':
        printMotorTrim();
        break;
      case 'c': case 'C':
        stopGpsNorthTest(nullptr);
        continuousTestMode = !continuousTestMode;
        if (continuousTestMode) {
          continuousStep = 0;
          continuousTestStepMs = millis();
          Serial.println("CONTINUOUS test ENABLED (x or c to stop)");
        } else {
          moveStop();
          Serial.println("CONTINUOUS test DISABLED");
        }
        break;
      case 'g': case 'G':
        gpsRawMonitorEnabled = !gpsRawMonitorEnabled;
        Serial.printf("Raw NMEA monitor: %s\n", gpsRawMonitorEnabled ? "ON" : "OFF");
        break;
      case 'n': case 'N':
        continuousTestMode = false;
        if (gpsNorthTestMode) {
          stopGpsNorthTest("[North] GPS North test stopped by user");
        } else {
          gpsNorthTestMode = true;
          gpsNorthState = NORTH_WAIT_FOR_FIX;
          consecutiveTurns = 0;
          Serial.println("[North] GPS North test STARTED. Needs open sky. Press n or x to stop.");
        }
        break;
      case '+': case '=':
        if (currentSpeed <= 235) currentSpeed += 20;
        else currentSpeed = 255;
        Serial.printf("Speed → %d\n", currentSpeed);
        break;
      case '-': case '_':
        if (currentSpeed >= 120) currentSpeed -= 20;
        else currentSpeed = 100;
        Serial.printf("Speed → %d\n", currentSpeed);
        break;
      case '1': currentSpeed = 160; Serial.println("Speed Level 1 (160)"); break;
      case '2': currentSpeed = 185; Serial.println("Speed Level 2 (185)"); break;
      case '3': currentSpeed = 210; Serial.println("Speed Level 3 (210)"); break;
      case '4': currentSpeed = 235; Serial.println("Speed Level 4 (235)"); break;
      case '5': currentSpeed = 255; Serial.println("Speed Level 5 (255 MAX)"); break;
      case 'h': case 'H': case '?':
        printHelpMenu();
        break;
      default:
        break;
    }
  }
}
// ===================== CONTINUOUS LOOP TEST =====================
void handleContinuousTest() {
  if (!continuousTestMode) return;
  unsigned long now = millis();
  if (now - continuousTestStepMs >= 3000) {
    continuousTestStepMs = now;
    continuousStep = (continuousStep + 1) % 8;
    switch (continuousStep) {
      case 0: Serial.println("[Loop] FORWARD");  moveForward(currentSpeed); break;
      case 1: Serial.println("[Loop] Pause");    moveStop(); continuousTestStepMs += 2000; break;
      case 2: Serial.println("[Loop] BACKWARD"); moveBackward(currentSpeed); break;
      case 3: Serial.println("[Loop] Pause");    moveStop(); continuousTestStepMs += 2000; break;
      case 4: Serial.println("[Loop] SPIN LEFT"); turnLeft(currentSpeed); break;
      case 5: Serial.println("[Loop] Pause");    moveStop(); continuousTestStepMs += 2000; break;
      case 6: Serial.println("[Loop] SPIN RIGHT"); turnRight(currentSpeed); break;
      case 7: Serial.println("[Loop] Pause");    moveStop(); continuousTestStepMs += 2000; break;
    }
  }
}
// ===================== SETUP & LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(400);
  motorInit();
  gpsInit();
  printHelpMenu();
  Serial.println("Place robot outdoors with clear sky view.");
  Serial.println("Auto GPS-North test will arm in 3 seconds (waits for fix before moving).\n");
  for (int i = 3; i > 0; i--) {
    Serial.printf("Starting in %d...\n", i);
    digitalWrite(LED_PIN, HIGH);
    delayWithExternalLedFlash(450);
    digitalWrite(LED_PIN, LOW);
    delayWithExternalLedFlash(450);
  }
  if (AUTO_GPS_NORTH_ON_BOOT) {
    gpsNorthTestMode = true;
    gpsNorthState = NORTH_WAIT_FOR_FIX;
    consecutiveTurns = 0;
    moveStop();
    Serial.println("[North] Auto GPS-North test armed. Waiting for fresh fix...");
  }
  printHelpMenu();
}
void loop() {
  handleSerialCommands();
  handleContinuousTest();
  handleExternalLedFlash();
  handleGpsTest();
  handleGpsNorthTest();
  // Heartbeat
  static unsigned long lastBlinkMs = 0;
  if (millis() - lastBlinkMs >= 1000) {
    lastBlinkMs = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  delay(8);
}
