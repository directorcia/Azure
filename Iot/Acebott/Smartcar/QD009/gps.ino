#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <esp_system.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include "telemetry_secrets.h"  // WIFI_SSID/PASSWORD + AIO_* - gitignored, not committed
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
 *  - SSD1306 OLED I2C (SDA=21, SCL=22). VCC = 3.3V only, not 5V.
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
#define OLED_SDA_PIN   21   // Acebott car-shield I2C header SDA (ESP32 GPIO 21)
#define OLED_SCL_PIN   22   // Acebott car-shield I2C header SCL (ESP32 GPIO 22)
#define OLED_ADDR      0x3C // Core Electronics white SSD1306 default
#define OLED_WIDTH     128
#define OLED_HEIGHT    64
#define OLED_RESET     -1
#define LCD_REFRESH_MS 400
// ===================== GPS NORTH TEST TUNING =====================
#define GPS_NORTH_DRIVE_MS        5500    // drive longer for clearer lat change
#define GPS_NORTH_TURN_DEG_PER_SEC 90.0   // measured in-place yaw rate at GPS_NORTH_TURN_SPEED
#define GPS_NORTH_TURN_MIN_MS     180     // below this the wheels barely break static friction
#define GPS_NORTH_TURN_MAX_MS     1400    // cap so one bad heading can't spin us right around
#define GPS_NORTH_SETTLE_MS       2800    // give GPS time to update after stop
#define GPS_NORTH_FIX_HOLD_MS     2500    // require a continuous fix before the first move
#define GPS_NORTH_MIN_RELIABLE_MOVE_M 3.0  // ignore displacement course below GPS noise floor (~2-3 m)
#define GPS_NORTH_SPEED           155
#define GPS_NORTH_TURN_SPEED      255     // 4WD skid-steer spins stall well below full PWM
#define MOTOR_PWM_FREQ_HZ         500     // Acebott vehicle library default
#define TURN_MIN_SPEED            240     // in-place yaw needs much more torque than forward
#define AUTO_GPS_NORTH_ON_BOOT    true
#define GPS_COURSE_TOLERANCE_DEG  28.0    // how close to 0°/360° is "north"
#define GPS_MAX_CONSECUTIVE_TURNS 8       // safety limit
#define GPS_FIX_MAX_AGE_MS        5000    // 1 Hz modules plus a missed sentence used to trip 2500 ms
#define GPS_COURSE_MIN_KMPH       1.5     // below this a receiver's course output is noise
#define MOTOR_TRIM_DEFAULT        85      // calibrated: 4 s @ 200, straight (was 363 mm left at 0)
#define MOTOR_TRIM_REFERENCE_SPEED 200    // speed used when MOTOR_TRIM_DEFAULT was calibrated
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
double gpsNorthStartLng = 0.0;
double lastTravelHeading = -1.0;
double drivingCourseDeg = -1.0;      // course sampled while actually rolling
unsigned long drivingCourseMs = 0;
int lastNorthTurnDir = 0;  // -1 left, +1 right, 0 none
unsigned long gpsNorthStepMs = 0;
unsigned long gpsNorthTurnMs = 0;    // proportional duration of the current turn
unsigned long gpsNorthFixHeldMs = 0;
unsigned long lastNorthWaitLogMs = 0;
uint8_t consecutiveTurns = 0;
int motorTrim = MOTOR_TRIM_DEFAULT;  // +boosts left / -boosts right
uint8_t lastMotorDir = 0;
uint8_t lastMotorLeft = 0;
uint8_t lastMotorRight = 0;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
bool lcdReady = false;
bool lcdHoldBoot = true;  // keep splash until setup() finishes
uint8_t lcdAddr = OLED_ADDR;
unsigned long lastLcdMs = 0;
unsigned long lastLcdRetryMs = 0;
unsigned long lastLcdPingMs = 0;
uint32_t lcdInitAttempts = 0;
uint32_t lcdRecoverCount = 0;
// ===================== TELEMETRY (Adafruit IO) =====================
#define TELEMETRY_INTERVAL_MS   10000  // stay under Adafruit IO's free-tier rate limit
#define WIFI_RETRY_INTERVAL_MS  5000
WiFiClient telemetryWifiClient;
Adafruit_MQTT_Client mqtt(&telemetryWifiClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
// Single feed with one log-line string per publish - easier to scroll back
// through as a chronological debug log than correlating several feeds/graphs.
Adafruit_MQTT_Publish feedLog = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/car-log");
unsigned long lastWifiAttemptMs = 0;
unsigned long lastTelemetryMs = 0;
bool wifiWasConnected = false;
// Survives software/brownout resets (RTC domain) but clears on true battery
// disconnect - a rising count with no physical power cycle means reset-looping.
RTC_DATA_ATTR uint32_t bootCount = 0;
const char *resetReasonStr = "?";
const char *describeResetReason(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "OTHER";
  }
}
// 1=POWERON 2=EXT 3=SW 4=PANIC 5=INT_WDT 6=TASK_WDT 7=WDT 8=BROWNOUT 9=other
int resetReasonCode(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return 1;
    case ESP_RST_EXT:       return 2;
    case ESP_RST_SW:        return 3;
    case ESP_RST_PANIC:     return 4;
    case ESP_RST_INT_WDT:   return 5;
    case ESP_RST_TASK_WDT:  return 6;
    case ESP_RST_WDT:       return 7;
    case ESP_RST_BROWNOUT:  return 8;
    default:                return 9;
  }
}
void blinkCount(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}
// LED-only readout so the reset cause is visible even if the OLED never
// lights up: blink reset-reason code, pause, blink boot count, pause.
void blinkDiagnostic(int reasonCode, uint32_t boots) {
  delay(600);
  blinkCount(reasonCode);
  delay(1000);
  blinkCount(boots > 15 ? 15 : (int)boots);
  delay(1500);
}
// If the OLED's power rail rises slower than the ESP32's, the display can be
// mid-reset while the I2C bus is toggled and leave SDA held low. Bit-bang a
// clock recovery (up to 9 clocks) so the slave releases SDA before Wire.begin.
void recoverI2CBus() {
  pinMode(OLED_SDA_PIN, INPUT_PULLUP);
  pinMode(OLED_SCL_PIN, OUTPUT);
  digitalWrite(OLED_SCL_PIN, HIGH);
  delayMicroseconds(5);
  if (digitalRead(OLED_SDA_PIN) == HIGH) {
    return; // bus already idle
  }
  lcdRecoverCount++;
  Serial.println("[OLED] SDA stuck low, clocking bus free");
  for (int i = 0; i < 9 && digitalRead(OLED_SDA_PIN) == LOW; i++) {
    digitalWrite(OLED_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(OLED_SCL_PIN, HIGH);
    delayMicroseconds(5);
  }
  pinMode(OLED_SDA_PIN, OUTPUT);
  digitalWrite(OLED_SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(OLED_SDA_PIN, HIGH);
  delayMicroseconds(5);
}
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
void lcdInit();
void handleLcd();
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
double wrap180(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}
double headingErrorToNorth(double headingDeg) {
  return wrap180(headingDeg);
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
      Serial.printf("[GPS] %s, no fresh fix | chars: %lu | NMEA lines: %lu | ok %lu | bad %lu\n",
                    gpsDataSeen ? "data detected" : "waiting for data",
                    gpsCharacterCount,
                    gpsSentenceCount,
                    (unsigned long)gps.passedChecksum(),
                    (unsigned long)gps.failedChecksum());
    }
  }
}
// ===================== TELEMETRY (Adafruit IO) =====================
void wifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttemptMs = millis();
  Serial.printf("[WiFi] connecting to %s...\n", WIFI_SSID);
}
// Non-blocking: never delay()s the main loop, so a missing hotspot can't
// stall driving/GPS/OLED. Just keeps retrying in the background.
void handleWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.printf("[WiFi] CONNECTED | IP: %s | RSSI: %d dBm\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
    }
    return;
  }
  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("[WiFi] connection lost");
  }
  unsigned long now = millis();
  if (now - lastWifiAttemptMs < WIFI_RETRY_INTERVAL_MS) return;
  lastWifiAttemptMs = now;
  Serial.println("[WiFi] retrying connection...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}
bool mqttConnect() {
  if (mqtt.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  int8_t ret = mqtt.connect();
  if (ret != 0) {
    Serial.printf("[MQTT] connect failed (%d)\n", ret);
    mqtt.disconnect();
    return false;
  }
  Serial.println("[MQTT] Adafruit IO connected");
  return true;
}
const char *northStateStr() {
  switch (gpsNorthState) {
    case NORTH_WAIT_FOR_FIX:  return "WAIT";
    case NORTH_DRIVE_FORWARD: return "DRIVE";
    case NORTH_COMPARE:       return "CHECK";
    case NORTH_TURN:          return "TURN";
    default:                  return "?";
  }
}
void handleTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetryMs < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryMs = now;
  if (WiFi.status() != WL_CONNECTED || !mqttConnect()) return;
  const char *mode = gpsNorthTestMode ? "NORTH" : (continuousTestMode ? "LOOP" : "IDLE");
  char turnCh = (lastNorthTurnDir < 0) ? 'L' : ((lastNorthTurnDir > 0) ? 'R' : '-');
  char logLine[160];
  if (gps.location.isValid()) {
    snprintf(logLine, sizeof(logLine),
             "t=%lu mode=%s st=%s lat=%.6f lng=%.6f hd=%.1f sats=%lu nmea=%lu ok=%lu bad=%lu turns=%d dir=%c",
             millis() / 1000, mode, northStateStr(), gps.location.lat(), gps.location.lng(),
             lastTravelHeading, (unsigned long)gpsSatCount(), gpsSentenceCount,
             (unsigned long)gps.passedChecksum(), (unsigned long)gps.failedChecksum(),
             consecutiveTurns, turnCh);
  } else {
    snprintf(logLine, sizeof(logLine),
             "t=%lu mode=%s st=%s lat=- lng=- hd=%.1f sats=%lu nmea=%lu ok=%lu bad=%lu turns=%d dir=%c",
             millis() / 1000, mode, northStateStr(), lastTravelHeading, (unsigned long)gpsSatCount(),
             gpsSentenceCount, (unsigned long)gps.passedChecksum(), (unsigned long)gps.failedChecksum(),
             consecutiveTurns, turnCh);
  }
  feedLog.publish(logLine);
  Serial.printf("[Telemetry] %s\n", logLine);
}
// ===================== LED HELPERS =====================
void setExternalLeds(bool leftOn) {
  digitalWrite(LEFT_LED_PIN, leftOn ? HIGH : LOW);
  digitalWrite(RIGHT_LED_PIN, leftOn ? LOW : HIGH);
}
void handleExternalLedFlash() {
  // Both solid = motors commanded on. Fast alternate = GPS bytes seen.
  // Slow alternate = no GPS data yet.
  if (gpsNorthTestMode &&
      (gpsNorthState == NORTH_DRIVE_FORWARD || gpsNorthState == NORTH_TURN) &&
      (lastMotorLeft > 0 || lastMotorRight > 0)) {
    digitalWrite(LEFT_LED_PIN, HIGH);
    digitalWrite(RIGHT_LED_PIN, HIGH);
    return;
  }
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
    handleLcd();
    delay(10);
  }
}
// ===================== OLED =====================
void lcdPrintLine(const char *text) {
  display.println(text != nullptr ? text : "");
}
void lcdShowBoot(const char *line2 = nullptr, const char *line3 = nullptr, const char *line4 = nullptr) {
  if (!lcdReady) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  lcdPrintLine("ACEBOTT BOOT");
  lcdPrintLine(line2 != nullptr ? line2 : "SSD1306 128x64");
  lcdPrintLine(line3 != nullptr ? line3 : "GPS North test");
  lcdPrintLine(line4 != nullptr ? line4 : "starting...");
  display.display();
}
void lcdInit() {
  // Same sequence that survived battery OFF→ON: Serial delay + motors first,
  // then Wire + scan so SSD1306 POR finishes before display.begin().
  lcdInitAttempts++;
  recoverI2CBus();
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN, 100000);
  delay(80);
  Serial.print("[OLED] I2C scan:");
  uint8_t foundAddr = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      if (addr == 0x3C || addr == 0x3D) foundAddr = addr;
    }
  }
  Serial.println();
  lcdAddr = foundAddr ? foundAddr : OLED_ADDR;
  lcdReady = display.begin(SSD1306_SWITCHCAPVCC, lcdAddr);
  if (!lcdReady) {
    Serial.println("[OLED] SSD1306 not found. Check 3.3V/GND/SDA21/SCL22.");
    return;
  }
  display.invertDisplay(false);
  display.dim(false);
  lcdHoldBoot = true;
  lcdShowBoot();
  Serial.printf("[OLED] ready at 0x%02X. GPS lock not required for text.\n", lcdAddr);
}
void handleLcd() {
  unsigned long now = millis();
  if (!lcdReady) {
    // Keep trying forever - a power-on glitch shouldn't require a re-flash.
    if (now - lastLcdRetryMs >= 1000) {
      lastLcdRetryMs = now;
      lcdInit();
    }
    return;
  }
  // Confirm the display still acks; if it silently dropped off the bus,
  // clear lcdReady so the retry above re-inits it without a power cycle.
  if (now - lastLcdPingMs >= 2000) {
    lastLcdPingMs = now;
    Wire.beginTransmission(lcdAddr);
    if (Wire.endTransmission() != 0) {
      Serial.println("[OLED] lost ack, re-initializing");
      lcdReady = false;
      return;
    }
  }
  if (lcdHoldBoot) return;
  if (now - lastLcdMs < LCD_REFRESH_MS) return;
  lastLcdMs = now;
  const char *mode = "IDLE";
  if (gpsNorthTestMode) {
    switch (gpsNorthState) {
      case NORTH_WAIT_FOR_FIX:  mode = "HOLD";  break;
      case NORTH_DRIVE_FORWARD: mode = "DRIVE"; break;
      case NORTH_COMPARE:       mode = "CHECK"; break;
      case NORTH_TURN:          mode = "TURN";  break;
      default:                  mode = "NORTH"; break;
    }
  }
  const char *lock = hasFreshGpsFix() ? "FIX" : (gpsDataSeen ? "NMEA" : "NO");
  unsigned long ageSec = gpsFixAgeMs() / 1000;
  if (ageSec > 99) ageSec = 99;
  double course = gps.course.isValid() ? gps.course.deg() : -1.0;
  double spd = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  double hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9;
  char turnCh = (lastNorthTurnDir < 0) ? 'L' : ((lastNorthTurnDir > 0) ? 'R' : '-');
  char line[22];
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  lcdPrintLine(WiFi.status() == WL_CONNECTED ? "WiFi CONNECTED" : "WiFi connecting");
  snprintf(line, sizeof(line), "%s s%02lu a%02lus %s",
           lock, (unsigned long)gpsSatCount(), ageSec, mode);
  lcdPrintLine(line);
  snprintf(line, sizeof(line), "ok%lu bad%lu",
           (unsigned long)gps.passedChecksum(),
           (unsigned long)gps.failedChecksum());
  lcdPrintLine(line);
  if (gps.location.isValid()) {
    snprintf(line, sizeof(line), "%.5f", gps.location.lat());
    lcdPrintLine(line);
    snprintf(line, sizeof(line), "%.5f", gps.location.lng());
    lcdPrintLine(line);
  } else {
    lcdPrintLine("no lat yet");
    lcdPrintLine("no lng yet");
  }
  snprintf(line, sizeof(line), "c%03.0f %4.1fk %c", course, spd, turnCh);
  lcdPrintLine(line);
  snprintf(line, sizeof(line), "hd%03.0f hdop%3.1f", lastTravelHeading, hdop);
  lcdPrintLine(line);
  display.display();
}
// ===================== MOTOR CONTROL =====================
int motorTrimAtSpeed(uint8_t speed) {
  return motorTrim * (int)speed / MOTOR_TRIM_REFERENCE_SPEED;
}
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
                clampPwm((int)currentSpeed + motorTrimAtSpeed(currentSpeed)),
                clampPwm((int)currentSpeed - motorTrimAtSpeed(currentSpeed)));
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
  // Trim is a straight-line calibration. Applying it to an in-place spin makes
  // the two sides unequal, so the car arcs instead of yawing and left/right
  // turns sweep different angles for the same duration.
  bool isSpin = (directionByte == DIR_SPIN_LEFT || directionByte == DIR_SPIN_RIGHT);
  if (!isSpin) {
    if (leftSpeed > 0) {
      leftOut = clampPwm((int)leftSpeed + motorTrimAtSpeed(leftSpeed));
    }
    if (rightSpeed > 0) {
      rightOut = clampPwm((int)rightSpeed - motorTrimAtSpeed(rightSpeed));
    }
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
      gpsNorthStartLng = gps.location.lng();
      gpsNorthStepMs = now;
      gpsNorthFixHeldMs = 0;
      drivingCourseDeg = -1.0;
      gpsNorthState = NORTH_DRIVE_FORWARD;
      Serial.printf("[North] Baseline lat %.6f lng %.6f. Driving forward for %.1f s\n",
                    gpsNorthStartLat, gpsNorthStartLng, GPS_NORTH_DRIVE_MS / 1000.0);
      moveForward(GPS_NORTH_SPEED);
      break;
    case NORTH_DRIVE_FORWARD:
      // A course fix is only meaningful while the wheels are actually turning;
      // after the stop+settle the receiver reports leftover noise.
      if (gps.course.isValid() && gps.course.age() < GPS_FIX_MAX_AGE_MS &&
          gps.speed.isValid() && gps.speed.kmph() >= GPS_COURSE_MIN_KMPH) {
        drivingCourseDeg = gps.course.deg();
        drivingCourseMs = now;
      }
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
      double currentLng = gps.location.lng();
      double movedM = TinyGPSPlus::distanceBetween(
          gpsNorthStartLat, gpsNorthStartLng, currentLat, currentLng);
      double displacementCourse = TinyGPSPlus::courseTo(
          gpsNorthStartLat, gpsNorthStartLng, currentLat, currentLng);
      bool hasCourse = (drivingCourseDeg >= 0.0) &&
                       (now - drivingCourseMs) < (GPS_NORTH_SETTLE_MS + GPS_NORTH_DRIVE_MS);
      double instantCourse = hasCourse ? drivingCourseDeg : -1.0;
      // GPS position noise is commonly 2-3 m; a displacement course computed
      // from a shorter move is essentially random. Prefer it once we've moved
      // far enough, otherwise fall back to the receiver's own Doppler-derived
      // course (independent of position noise) if it has one.
      bool haveReliableHeading = false;
      double heading = 0.0;
      if (movedM >= GPS_NORTH_MIN_RELIABLE_MOVE_M) {
        heading = displacementCourse;
        haveReliableHeading = true;
      } else if (hasCourse) {
        heading = instantCourse;
        haveReliableHeading = true;
      }
      if (haveReliableHeading) {
        lastTravelHeading = heading;
      }
      if (!haveReliableHeading) {
        // Don't guess a turn from noise - keep driving so the next check has
        // a longer, more trustworthy displacement to measure against.
        Serial.printf("[North] move %.1fm too small for a reliable heading, driving on\n", movedM);
        gpsNorthStepMs = now;
        gpsNorthState = NORTH_DRIVE_FORWARD;
        moveForward(GPS_NORTH_SPEED);
        break;
      }
      double err = headingErrorToNorth(heading);
      bool goingNorth = fabs(err) <= GPS_COURSE_TOLERANCE_DEG;
      Serial.printf("[North] move %.1fm hd %.0f inst %.0f err %.0f sats %lu\n",
                    movedM, heading, instantCourse, err, gpsSatCount());
      if (goingNorth) {
        gpsNorthStartLat = currentLat;
        gpsNorthStartLng = currentLng;
        gpsNorthStepMs = now;
        consecutiveTurns = 0;
        lastNorthTurnDir = 0;
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
        // Open-loop turns must be scaled to the error. A fixed duration
        // overshoots small corrections and undershoots large ones, which is
        // what made the heading oscillate around north instead of settling.
        double turnMs = (fabs(err) / GPS_NORTH_TURN_DEG_PER_SEC) * 1000.0;
        if (turnMs < GPS_NORTH_TURN_MIN_MS) turnMs = GPS_NORTH_TURN_MIN_MS;
        if (turnMs > GPS_NORTH_TURN_MAX_MS) turnMs = GPS_NORTH_TURN_MAX_MS;
        gpsNorthTurnMs = (unsigned long)turnMs;
        // err > 0 means course is east of north (clockwise) - turn LEFT
        // (counter-clockwise) to reduce it back to 0; err < 0 (west of
        // north) needs a RIGHT turn. This was previously inverted, which
        // made every "correction" push heading further off, spiraling.
        if (err > 0.0) {
          lastNorthTurnDir = -1;
          Serial.printf("[North] Heading %.0f, need left %.0f deg for %lu ms (turn %d/%d)\n",
                        heading, err, gpsNorthTurnMs, consecutiveTurns, GPS_MAX_CONSECUTIVE_TURNS);
          turnLeft(GPS_NORTH_TURN_SPEED);
        } else {
          lastNorthTurnDir = 1;
          Serial.printf("[North] Heading %.0f, need right %.0f deg for %lu ms (turn %d/%d)\n",
                        heading, -err, gpsNorthTurnMs, consecutiveTurns, GPS_MAX_CONSECUTIVE_TURNS);
          turnRight(GPS_NORTH_TURN_SPEED);
        }
      }
      break;
    }
    case NORTH_TURN:
      if (now - gpsNorthStepMs >= gpsNorthTurnMs) {
        moveStop();
        gpsNorthFixHeldMs = 0;
        drivingCourseDeg = -1.0;              // stale: it predates the turn
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
  Serial.println("OLED: SSD1306 128x64 I2C 0x3C on SDA=21 SCL=22. Power 3.3V only.");
  Serial.println("OLED: FIX/NMEA, sats, age, mode, lat/lng, course, L/R turn, travel heading");
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
  // A marginal battery/regulator can sag enough to trip the ESP32's brownout
  // detector during boot, causing a silent reset loop before setup() ever
  // finishes. Disable it here so we at least get a chance to run and report.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  bootCount++;
  resetReasonStr = describeResetReason(esp_reset_reason());
  Serial.begin(115200);
  delay(400);
  Serial.printf("[OLED] boot #%lu reason=%s\n", (unsigned long)bootCount, resetReasonStr);
  pinMode(LED_PIN, OUTPUT);
  // Report reason-code blinks then boot-count blinks so the cause is
  // readable purely from the LED if the screen stays blank.
  blinkDiagnostic(resetReasonCode(esp_reset_reason()), bootCount);
  motorInit();
  lcdInit();
  gpsInit();
  wifiInit();
  printHelpMenu();
  Serial.println("Place robot outdoors with clear sky view.");
  Serial.println("Auto GPS-North test will arm in 3 seconds (waits for fix before moving).\n");
  for (int i = 3; i > 0; i--) {
    char countLine[22];
    snprintf(countLine, sizeof(countLine), "starting in %d...", i);
    lcdShowBoot("SSD1306 128x64", "GPS North test", countLine);
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
    lcdShowBoot("SSD1306 128x64", "North test armed", "waiting for GPS");
    Serial.println("[North] Auto GPS-North test armed. Waiting for fresh fix...");
  }
  lcdHoldBoot = false;
  lastLcdMs = 0;
  printHelpMenu();
}
void loop() {
  handleSerialCommands();
  handleContinuousTest();
  handleExternalLedFlash();
  handleGpsTest();
  handleLcd();
  handleGpsNorthTest();
  handleWifi();
  handleTelemetry();
  static unsigned long lastBlinkMs = 0;
  if (millis() - lastBlinkMs >= 1000) {
    lastBlinkMs = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  delay(8);
}