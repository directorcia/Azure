#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ultrasonic.h>
#include <vehicle.h>
#include "gemini_config.h"
#include "wifi_config.h"

vehicle myCar;
ultrasonic sensor;
Servo panServo;

enum Action {
  ACTION_STOP = 0,
  ACTION_LEFT,
  ACTION_RIGHT,
  ACTION_BACKWARD,
};

enum ManeuverType {
  MANEUVER_STOP = 0,
  MANEUVER_STRAFE_LEFT,
  MANEUVER_STRAFE_RIGHT,
  MANEUVER_BACKWARD,
  MANEUVER_TURN_LEFT_90,
  MANEUVER_TURN_RIGHT_90,
  MANEUVER_RESCAN,
};

struct HazardSnapshot {
  float leftCm;
  float frontCm;
  float rightCm;
  unsigned long atMs;
};

struct ManeuverPlan {
  ManeuverType primary;
  uint16_t primaryDurationMs;
  ManeuverType secondary;
  uint16_t secondaryDurationMs;
  float confidence;
  bool repeatedTrap;
};

const int FORWARD_SPEED = 255;
const int TURN_SPEED = 240;
const unsigned long TURN_90_DURATION_MS = 520;
const int TRIG_PIN = 13;
const int ECHO_PIN = 14;
const int ULTRASONIC_PAN_PIN = 27;
const int LEFT_LED_PIN = 2;
const int RIGHT_LED_PIN = 12;
const bool LED_ACTIVE_HIGH = true;

const int PAN_CENTER_TRIM_DEG = 0;
const int PAN_CENTER_DEG = 90 + PAN_CENTER_TRIM_DEG;
const int PAN_HALF_ARC_DEG = 85;
const int PAN_LEFT_DEG = ((PAN_CENTER_DEG + PAN_HALF_ARC_DEG) > 180) ? 180 : (PAN_CENTER_DEG + PAN_HALF_ARC_DEG);
const int PAN_RIGHT_DEG = ((PAN_CENTER_DEG - PAN_HALF_ARC_DEG) < 0) ? 0 : (PAN_CENTER_DEG - PAN_HALF_ARC_DEG);
const int PAN_STEP_DEG = 5;
const unsigned long PAN_STEP_INTERVAL_MS = 22;
const int PAN_CENTER_BUCKET_DEG = 18;

const float ULTRASONIC_ALERT_CM = 30.0f;
const float ULTRASONIC_MIN_VALID_CM = 4.0f;
const float EMERGENCY_REVERSE_CM = 18.0f;
const float WALL_HEADON_FRONT_CM = 28.0f;
const float WALL_HEADON_SIDE_CM = 42.0f;
const float WALL_HEADON_SIDE_BALANCE_CM = 12.0f;
const float WIDE_OPEN_DIFF_CM = 18.0f;
const float REPEAT_PATTERN_TOLERANCE_CM = 9.0f;

const unsigned long SENSOR_INTERVAL_MS = 70;
const unsigned long STRAFE_DURATION_MS = 450;
const unsigned long BACKUP_DURATION_MS = 380;
const unsigned long GEMINI_HTTP_TIMEOUT_MS = 12000;
const unsigned long HAZARD_DECISION_COOLDOWN_MS = 800;
const unsigned long FRONT_STALE_MS = 450;
const unsigned long HAZARD_BURST_WINDOW_MS = 3000;
const uint8_t HAZARD_BURST_THRESHOLD = 2;
const unsigned long HAZARD_CLEAR_RESET_MS = 1200;
const unsigned long PAN_SETTLE_MS = 120;
const unsigned long RESCAN_PAUSE_MS = 140;
const unsigned long THINK_LED_ON_MS = 140;
const unsigned long THINK_LED_OFF_MS = 80;
const unsigned long DECISION_LED_MS = 220;

const int SAFE_UNKNOWN_DISTANCE_CM = 120;
const uint8_t NAV_HISTORY_SIZE = 8;
const uint8_t TRAP_REPEAT_THRESHOLD = 3;
const float MIN_LLM_CONFIDENCE = 0.45f;
const uint16_t MIN_MANEUVER_DURATION_MS = 180;
const uint16_t MAX_MANEUVER_DURATION_MS = 700;

bool obstacleNearby = false;
bool previousObstacleNearby = false;
unsigned long lastSensorMs = 0;
unsigned long lastPanStepMs = 0;
unsigned long lastHazardDecisionMs = 0;
unsigned long hazardBurstWindowStartMs = 0;
uint8_t hazardBurstCount = 0;
unsigned long hazardClearSinceMs = 0;
Action lastNonStopDecision = ACTION_LEFT;
bool panServoReady = false;
int panCurrentDeg = PAN_CENTER_DEG;
bool panSweepTowardLeft = true;
float leftDistanceCm = -1.0f;
float frontDistanceCm = -1.0f;
float rightDistanceCm = -1.0f;
unsigned long frontUpdatedMs = 0;
HazardSnapshot navHistory[NAV_HISTORY_SIZE];
uint8_t navHistoryCount = 0;
uint8_t navHistoryWriteIndex = 0;
ManeuverType recentPlans[4] = {MANEUVER_STOP, MANEUVER_STOP, MANEUVER_STOP, MANEUVER_STOP};
uint8_t recentPlanWriteIndex = 0;

void setBothLeds(bool on) {
  uint8_t level = on ? (LED_ACTIVE_HIGH ? HIGH : LOW) : (LED_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(LEFT_LED_PIN, level);
  digitalWrite(RIGHT_LED_PIN, level);
}

void flashGeminiThinkingLeds() {
  setBothLeds(true);
  delay(THINK_LED_ON_MS);
  setBothLeds(false);
  delay(THINK_LED_OFF_MS);
  setBothLeds(true);
  delay(THINK_LED_ON_MS);
  setBothLeds(false);
}

void flashDecisionDirectionLed(ManeuverType maneuver) {
  int pin = -1;
  if (maneuver == MANEUVER_STRAFE_LEFT || maneuver == MANEUVER_TURN_LEFT_90) {
    pin = LEFT_LED_PIN;
  } else if (maneuver == MANEUVER_STRAFE_RIGHT || maneuver == MANEUVER_TURN_RIGHT_90) {
    pin = RIGHT_LED_PIN;
  }
  if (pin < 0) {
    return;
  }
  uint8_t onLevel = LED_ACTIVE_HIGH ? HIGH : LOW;
  uint8_t offLevel = LED_ACTIVE_HIGH ? LOW : HIGH;
  digitalWrite(pin, onLevel);
  delay(DECISION_LED_MS);
  digitalWrite(pin, offLevel);
}

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000UL) {
    Serial.print('.');
    delay(300);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi not connected. Navigation falls back to local planner");
  }
}

bool isValidDistance(float distanceCm) {
  return distanceCm >= ULTRASONIC_MIN_VALID_CM;
}

int effectiveDistance(float distanceCm) {
  if (isValidDistance(distanceCm)) {
    return (int)distanceCm;
  }
  return SAFE_UNKNOWN_DISTANCE_CM;
}

Action chooseFallbackTurn() {
  int leftEff = effectiveDistance(leftDistanceCm);
  int rightEff = effectiveDistance(rightDistanceCm);
  if (leftEff == rightEff) {
    return lastNonStopDecision == ACTION_LEFT ? ACTION_RIGHT : ACTION_LEFT;
  }
  return (leftEff > rightEff) ? ACTION_LEFT : ACTION_RIGHT;
}

bool isHeadOnWall(float frontCm, float leftCm, float rightCm) {
  if (!isValidDistance(frontCm) || !isValidDistance(leftCm) || !isValidDistance(rightCm)) {
    return false;
  }
  if (frontCm > WALL_HEADON_FRONT_CM) {
    return false;
  }
  bool bothSidesClose = (leftCm <= WALL_HEADON_SIDE_CM && rightCm <= WALL_HEADON_SIDE_CM);
  bool sideBalanced = (fabs(leftCm - rightCm) <= WALL_HEADON_SIDE_BALANCE_CM);
  return bothSidesClose && sideBalanced;
}

const char *maneuverTypeToString(ManeuverType maneuver) {
  switch (maneuver) {
    case MANEUVER_STRAFE_LEFT:
      return "STRAFE_LEFT";
    case MANEUVER_STRAFE_RIGHT:
      return "STRAFE_RIGHT";
    case MANEUVER_BACKWARD:
      return "BACKWARD";
    case MANEUVER_TURN_LEFT_90:
      return "TURN_LEFT_90";
    case MANEUVER_TURN_RIGHT_90:
      return "TURN_RIGHT_90";
    case MANEUVER_RESCAN:
      return "RESCAN";
    default:
      return "STOP";
  }
}

ManeuverType parseManeuverType(const String &text) {
  String normalized = text;
  normalized.toUpperCase();
  normalized.trim();
  if (normalized.indexOf("STRAFE_LEFT") >= 0 || normalized.indexOf("LEFT") >= 0) {
    return MANEUVER_STRAFE_LEFT;
  }
  if (normalized.indexOf("STRAFE_RIGHT") >= 0 || normalized.indexOf("RIGHT") >= 0) {
    return MANEUVER_STRAFE_RIGHT;
  }
  if (normalized.indexOf("TURN_LEFT_90") >= 0) {
    return MANEUVER_TURN_LEFT_90;
  }
  if (normalized.indexOf("TURN_RIGHT_90") >= 0) {
    return MANEUVER_TURN_RIGHT_90;
  }
  if (normalized.indexOf("BACKWARD") >= 0 || normalized.indexOf("REVERSE") >= 0) {
    return MANEUVER_BACKWARD;
  }
  if (normalized.indexOf("RESCAN") >= 0) {
    return MANEUVER_RESCAN;
  }
  return MANEUVER_STOP;
}

uint16_t clampDuration(uint16_t value, uint16_t fallbackMs) {
  if (value < MIN_MANEUVER_DURATION_MS || value > MAX_MANEUVER_DURATION_MS) {
    return fallbackMs;
  }
  return value;
}

void rememberPlan(ManeuverType maneuver) {
  recentPlans[recentPlanWriteIndex] = maneuver;
  recentPlanWriteIndex = (recentPlanWriteIndex + 1) % 4;
}

void recordHazardSnapshot(float leftCm, float frontCm, float rightCm, unsigned long nowMs) {
  navHistory[navHistoryWriteIndex].leftCm = leftCm;
  navHistory[navHistoryWriteIndex].frontCm = frontCm;
  navHistory[navHistoryWriteIndex].rightCm = rightCm;
  navHistory[navHistoryWriteIndex].atMs = nowMs;
  navHistoryWriteIndex = (navHistoryWriteIndex + 1) % NAV_HISTORY_SIZE;
  if (navHistoryCount < NAV_HISTORY_SIZE) {
    navHistoryCount++;
  }
}

bool snapshotSimilar(const HazardSnapshot &a, float leftCm, float frontCm, float rightCm) {
  return fabs(a.leftCm - leftCm) <= REPEAT_PATTERN_TOLERANCE_CM &&
         fabs(a.frontCm - frontCm) <= REPEAT_PATTERN_TOLERANCE_CM &&
         fabs(a.rightCm - rightCm) <= REPEAT_PATTERN_TOLERANCE_CM;
}

bool detectRepeatedTrap(float leftCm, float frontCm, float rightCm) {
  uint8_t similarCount = 0;
  for (uint8_t i = 0; i < navHistoryCount; ++i) {
    if (snapshotSimilar(navHistory[i], leftCm, frontCm, rightCm)) {
      similarCount++;
    }
  }
  return similarCount >= TRAP_REPEAT_THRESHOLD;
}

String buildRecentPlanSummary() {
  String summary;
  for (uint8_t i = 0; i < 4; ++i) {
    if (i > 0) {
      summary += ",";
    }
    uint8_t idx = (recentPlanWriteIndex + i) % 4;
    summary += maneuverTypeToString(recentPlans[idx]);
  }
  return summary;
}

String buildHistorySummary() {
  String summary;
  uint8_t start = (navHistoryCount == NAV_HISTORY_SIZE) ? navHistoryWriteIndex : 0;
  for (uint8_t i = 0; i < navHistoryCount; ++i) {
    uint8_t idx = (start + i) % NAV_HISTORY_SIZE;
    if (summary.length() > 0) {
      summary += " | ";
    }
    summary += "L=" + String(navHistory[idx].leftCm, 1);
    summary += ",F=" + String(navHistory[idx].frontCm, 1);
    summary += ",R=" + String(navHistory[idx].rightCm, 1);
  }
  if (summary.length() == 0) {
    summary = "none";
  }
  return summary;
}

ManeuverPlan defaultPlan() {
  ManeuverPlan plan;
  plan.primary = MANEUVER_STOP;
  plan.primaryDurationMs = 220;
  plan.secondary = MANEUVER_STOP;
  plan.secondaryDurationMs = 0;
  plan.confidence = 0.0f;
  plan.repeatedTrap = false;
  return plan;
}

ManeuverPlan chooseLocalPlan(bool repeatedTrap) {
  ManeuverPlan plan = defaultPlan();
  plan.repeatedTrap = repeatedTrap;

  int leftEff = effectiveDistance(leftDistanceCm);
  int rightEff = effectiveDistance(rightDistanceCm);
  int frontEff = effectiveDistance(frontDistanceCm);
  Action preferredTurn = chooseFallbackTurn();

  if (frontEff <= EMERGENCY_REVERSE_CM || repeatedTrap) {
    plan.primary = MANEUVER_BACKWARD;
    plan.primaryDurationMs = BACKUP_DURATION_MS;
    plan.secondary = (preferredTurn == ACTION_LEFT) ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
    plan.secondaryDurationMs = TURN_90_DURATION_MS;
    plan.confidence = repeatedTrap ? 0.95f : 0.85f;
    return plan;
  }

  if (abs(leftEff - rightEff) >= WIDE_OPEN_DIFF_CM) {
    plan.primary = (leftEff > rightEff) ? MANEUVER_STRAFE_LEFT : MANEUVER_STRAFE_RIGHT;
    plan.primaryDurationMs = STRAFE_DURATION_MS;
    plan.secondary = MANEUVER_RESCAN;
    plan.secondaryDurationMs = RESCAN_PAUSE_MS;
    plan.confidence = 0.78f;
    return plan;
  }

  plan.primary = (preferredTurn == ACTION_LEFT) ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
  plan.primaryDurationMs = TURN_90_DURATION_MS;
  plan.secondary = MANEUVER_RESCAN;
  plan.secondaryDurationMs = RESCAN_PAUSE_MS;
  plan.confidence = 0.68f;
  return plan;
}

bool extractFirstJsonObject(const String &input, String &jsonText) {
  int start = input.indexOf('{');
  if (start < 0) {
    return false;
  }
  int depth = 0;
  for (int i = start; i < input.length(); ++i) {
    char ch = input.charAt(i);
    if (ch == '{') {
      depth++;
    } else if (ch == '}') {
      depth--;
      if (depth == 0) {
        jsonText = input.substring(start, i + 1);
        return true;
      }
    }
  }
  return false;
}

bool parsePlanFromJsonText(const String &modelText, ManeuverPlan &plan) {
  String jsonText;
  if (!extractFirstJsonObject(modelText, jsonText)) {
    return false;
  }

  JsonDocument planDoc;
  DeserializationError err = deserializeJson(planDoc, jsonText);
  if (err) {
    Serial.print("Plan JSON parse error: ");
    Serial.println(err.c_str());
    return false;
  }

  String primary = planDoc["primary"] | "STOP";
  String secondary = planDoc["secondary"] | "STOP";
  uint16_t primaryDuration = planDoc["primary_duration_ms"] | (uint16_t)STRAFE_DURATION_MS;
  uint16_t secondaryDuration = planDoc["secondary_duration_ms"] | (uint16_t)0;
  float confidence = planDoc["confidence"] | 0.0f;

  plan.primary = parseManeuverType(primary);
  plan.primaryDurationMs = clampDuration(primaryDuration, STRAFE_DURATION_MS);
  plan.secondary = parseManeuverType(secondary);
  plan.secondaryDurationMs = secondaryDuration == 0 ? 0 : clampDuration(secondaryDuration, RESCAN_PAUSE_MS);
  plan.confidence = confidence;
  return true;
}

void sanitizePlan(ManeuverPlan &plan, const ManeuverPlan &fallbackPlan) {
  if (plan.primary == MANEUVER_STOP && fallbackPlan.primary != MANEUVER_STOP) {
    plan = fallbackPlan;
    return;
  }

  if (plan.primary == MANEUVER_RESCAN) {
    plan.primary = fallbackPlan.primary;
    plan.primaryDurationMs = fallbackPlan.primaryDurationMs;
  }

  if (plan.primary == MANEUVER_TURN_LEFT_90 || plan.primary == MANEUVER_TURN_RIGHT_90) {
    plan.primaryDurationMs = TURN_90_DURATION_MS;
  }

  if (plan.primary == MANEUVER_BACKWARD && isValidDistance(frontDistanceCm) && frontDistanceCm > ULTRASONIC_ALERT_CM) {
    plan = fallbackPlan;
  }

  if (plan.secondary == MANEUVER_TURN_LEFT_90 || plan.secondary == MANEUVER_TURN_RIGHT_90) {
    plan.secondaryDurationMs = TURN_90_DURATION_MS;
  }

  if (plan.confidence < MIN_LLM_CONFIDENCE) {
    plan = fallbackPlan;
  }
}

ManeuverPlan queryGeminiForNavigationPlan(const ManeuverPlan &fallbackPlan, bool repeatedTrap) {
  flashGeminiThinkingLeds();
  if (WiFi.status() != WL_CONNECTED) {
    return fallbackPlan;
  }

  WiFiClientSecure secureClient;
  secureClient.setInsecure();
  HTTPClient http;
  String url = String(GEMINI_URL) + "?key=" + String(GEMINI_API_KEY);
  if (!http.begin(secureClient, url)) {
    Serial.println("Gemini: HTTP begin failed");
    return fallbackPlan;
  }

  String prompt =
      "You are planning a safe indoor robot avoidance maneuver. "
      "Use only these maneuvers: STRAFE_LEFT, STRAFE_RIGHT, BACKWARD, TURN_LEFT_90, TURN_RIGHT_90, STOP, RESCAN. "
      "Safety rules: do not pick STOP unless fully blocked, use BACKWARD when trapped, and prefer the more open side. "
      "Return JSON only with keys primary, primary_duration_ms, secondary, secondary_duration_ms, confidence, reason. "
      "Current distances cm: front=" + String(frontDistanceCm, 1) +
      ", left=" + String(leftDistanceCm, 1) +
      ", right=" + String(rightDistanceCm, 1) +
      ". Repeated trap=" + String(repeatedTrap ? "true" : "false") +
      ". Baseline primary=" + String(maneuverTypeToString(fallbackPlan.primary)) +
      ", baseline secondary=" + String(maneuverTypeToString(fallbackPlan.secondary)) +
      ". Recent hazard history=" + buildHistorySummary() +
      ". Recent plans=" + buildRecentPlanSummary() +
      ". Example JSON: {\"primary\":\"BACKWARD\",\"primary_duration_ms\":350,\"secondary\":\"TURN_LEFT_90\",\"secondary_duration_ms\":520,\"confidence\":0.82,\"reason\":\"repeated_trap\"}.";

  String body = String("{\"contents\":[{\"parts\":[{\"text\":\"") + prompt +
                "\"}]}],\"generationConfig\":{\"temperature\":0.15,\"maxOutputTokens\":120}}";

  http.addHeader("Content-Type", "application/json");
  http.setTimeout(GEMINI_HTTP_TIMEOUT_MS);
  int statusCode = http.POST(body);
  String responseBody = http.getString();
  http.end();

  if (statusCode < 200 || statusCode >= 300) {
    Serial.print("Gemini HTTP error: ");
    Serial.println(statusCode);
    return fallbackPlan;
  }

  JsonDocument responseDoc;
  DeserializationError err = deserializeJson(responseDoc, responseBody);
  if (err) {
    Serial.print("Gemini response parse error: ");
    Serial.println(err.c_str());
    return fallbackPlan;
  }

  String modelText = responseDoc["candidates"][0]["content"]["parts"][0]["text"] | "";
  Serial.print("Gemini raw text: ");
  Serial.println(modelText);

  ManeuverPlan plan = fallbackPlan;
  if (!parsePlanFromJsonText(modelText, plan)) {
    return fallbackPlan;
  }

  plan.repeatedTrap = repeatedTrap;
  sanitizePlan(plan, fallbackPlan);
  flashDecisionDirectionLed(plan.primary);
  Serial.print("Gemini plan: ");
  Serial.print(maneuverTypeToString(plan.primary));
  Serial.print(" then ");
  Serial.print(maneuverTypeToString(plan.secondary));
  Serial.print(" conf=");
  Serial.println(plan.confidence, 2);
  return plan;
}

void movePanToCenter() {
  if (!panServoReady) {
    return;
  }
  panCurrentDeg = PAN_CENTER_DEG;
  panServo.write(panCurrentDeg);
}

void refreshHazardScanSnapshot() {
  if (!panServoReady) {
    float front = sensor.Ranging();
    if (isValidDistance(front)) {
      frontDistanceCm = front;
      frontUpdatedMs = millis();
    }
    return;
  }

  panServo.write(PAN_LEFT_DEG);
  panCurrentDeg = PAN_LEFT_DEG;
  delay(PAN_SETTLE_MS);
  float left = sensor.Ranging();
  if (isValidDistance(left)) {
    leftDistanceCm = left;
  }

  panServo.write(PAN_RIGHT_DEG);
  panCurrentDeg = PAN_RIGHT_DEG;
  delay(PAN_SETTLE_MS);
  float right = sensor.Ranging();
  if (isValidDistance(right)) {
    rightDistanceCm = right;
  }

  movePanToCenter();
  delay(PAN_SETTLE_MS);
  float front = sensor.Ranging();
  if (isValidDistance(front)) {
    frontDistanceCm = front;
    frontUpdatedMs = millis();
  } else {
    frontDistanceCm = -1.0f;
  }

  panSweepTowardLeft = true;
  lastPanStepMs = millis();

  Serial.print("Decision scan L/F/R: ");
  Serial.print(leftDistanceCm, 1);
  Serial.print("/");
  Serial.print(frontDistanceCm, 1);
  Serial.print("/");
  Serial.println(rightDistanceCm, 1);
}

void updatePanSweep(unsigned long nowMs) {
  if (!panServoReady) {
    return;
  }
  if (nowMs - lastPanStepMs < PAN_STEP_INTERVAL_MS) {
    return;
  }

  lastPanStepMs = nowMs;
  if (panSweepTowardLeft) {
    panCurrentDeg += PAN_STEP_DEG;
    if (panCurrentDeg >= PAN_LEFT_DEG) {
      panCurrentDeg = PAN_LEFT_DEG;
      panSweepTowardLeft = false;
    }
  } else {
    panCurrentDeg -= PAN_STEP_DEG;
    if (panCurrentDeg <= PAN_RIGHT_DEG) {
      panCurrentDeg = PAN_RIGHT_DEG;
      panSweepTowardLeft = true;
    }
  }
  panServo.write(panCurrentDeg);
}

void updateScanAndHazard() {
  unsigned long nowMs = millis();
  updatePanSweep(nowMs);
  if (nowMs - lastSensorMs < SENSOR_INTERVAL_MS) {
    return;
  }

  lastSensorMs = nowMs;
  float distanceCm = sensor.Ranging();
  if (isValidDistance(distanceCm)) {
    if (panCurrentDeg >= (PAN_CENTER_DEG + PAN_CENTER_BUCKET_DEG)) {
      leftDistanceCm = distanceCm;
    } else if (panCurrentDeg <= (PAN_CENTER_DEG - PAN_CENTER_BUCKET_DEG)) {
      rightDistanceCm = distanceCm;
    } else {
      frontDistanceCm = distanceCm;
      frontUpdatedMs = nowMs;
    }
  } else if (panCurrentDeg > (PAN_CENTER_DEG - PAN_CENTER_BUCKET_DEG) &&
             panCurrentDeg < (PAN_CENTER_DEG + PAN_CENTER_BUCKET_DEG)) {
    frontDistanceCm = -1.0f;
  }

  bool frontFresh = (frontUpdatedMs != 0 && (nowMs - frontUpdatedMs) <= FRONT_STALE_MS);
  if (frontFresh && isValidDistance(frontDistanceCm)) {
    obstacleNearby = (frontDistanceCm <= ULTRASONIC_ALERT_CM);
  } else {
    obstacleNearby = false;
  }

  Serial.print("Scan L/F/R: ");
  Serial.print(leftDistanceCm, 1);
  Serial.print("/");
  Serial.print(frontDistanceCm, 1);
  Serial.print("/");
  Serial.print(rightDistanceCm, 1);
  Serial.print(" cm | hazard: ");
  Serial.println(obstacleNearby ? "YES" : "NO");
}

bool shouldForceBackward(unsigned long nowMs) {
  if (hazardBurstWindowStartMs == 0 || (nowMs - hazardBurstWindowStartMs) > HAZARD_BURST_WINDOW_MS) {
    hazardBurstWindowStartMs = nowMs;
    hazardBurstCount = 1;
    return false;
  }
  if (hazardBurstCount < 255) {
    hazardBurstCount++;
  }
  return hazardBurstCount >= HAZARD_BURST_THRESHOLD;
}

void executeManeuver(ManeuverType maneuver, uint16_t durationMs) {
  switch (maneuver) {
    case MANEUVER_STRAFE_LEFT:
      myCar.Move(Move_Left, TURN_SPEED);
      delay(durationMs);
      break;
    case MANEUVER_STRAFE_RIGHT:
      myCar.Move(Move_Right, TURN_SPEED);
      delay(durationMs);
      break;
    case MANEUVER_BACKWARD:
      myCar.Move(Backward, TURN_SPEED);
      delay(durationMs);
      break;
    case MANEUVER_TURN_LEFT_90:
      myCar.Move(Contrarotate, TURN_SPEED);
      delay(TURN_90_DURATION_MS);
      break;
    case MANEUVER_TURN_RIGHT_90:
      myCar.Move(Clockwise, TURN_SPEED);
      delay(TURN_90_DURATION_MS);
      break;
    case MANEUVER_RESCAN:
      myCar.Move(Stop, 0);
      delay(durationMs == 0 ? RESCAN_PAUSE_MS : durationMs);
      refreshHazardScanSnapshot();
      break;
    default:
      myCar.Move(Stop, 0);
      delay(durationMs == 0 ? 180 : durationMs);
      break;
  }
  myCar.Move(Stop, 0);
}

void executePlan(const ManeuverPlan &plan) {
  executeManeuver(plan.primary, plan.primaryDurationMs);
  rememberPlan(plan.primary);

  if (plan.secondary != MANEUVER_STOP) {
    executeManeuver(plan.secondary, plan.secondaryDurationMs);
    rememberPlan(plan.secondary);
  }

  if (plan.primary == MANEUVER_STRAFE_LEFT || plan.primary == MANEUVER_TURN_LEFT_90) {
    lastNonStopDecision = ACTION_LEFT;
  } else if (plan.primary == MANEUVER_STRAFE_RIGHT || plan.primary == MANEUVER_TURN_RIGHT_90) {
    lastNonStopDecision = ACTION_RIGHT;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
  setBothLeds(false);

  myCar.Init();
  sensor.Init(TRIG_PIN, ECHO_PIN);
  panServo.setPeriodHertz(50);
  panServo.attach(ULTRASONIC_PAN_PIN, 500, 2500);
  panServoReady = panServo.attached();
  panCurrentDeg = PAN_CENTER_DEG;
  panSweepTowardLeft = true;
  lastPanStepMs = millis();
  movePanToCenter();

  Serial.print("Pan calibration L/C/R: ");
  Serial.print(PAN_LEFT_DEG);
  Serial.print("/");
  Serial.print(PAN_CENTER_DEG);
  Serial.print("/");
  Serial.println(PAN_RIGHT_DEG);

  delay(250);
  connectWiFi();
  Serial.println("LLM-assisted navigation planner enabled");
}

void loop() {
  updateScanAndHazard();
  unsigned long nowMs = millis();

  if (obstacleNearby && (!previousObstacleNearby || (nowMs - lastHazardDecisionMs) >= HAZARD_DECISION_COOLDOWN_MS)) {
    myCar.Move(Stop, 0);
    refreshHazardScanSnapshot();
    recordHazardSnapshot(leftDistanceCm, frontDistanceCm, rightDistanceCm, nowMs);

    ManeuverPlan plan;
    bool repeatedTrap = detectRepeatedTrap(leftDistanceCm, frontDistanceCm, rightDistanceCm);
    if (isHeadOnWall(frontDistanceCm, leftDistanceCm, rightDistanceCm)) {
      plan = defaultPlan();
      plan.primary = MANEUVER_BACKWARD;
      plan.primaryDurationMs = BACKUP_DURATION_MS;
      plan.secondary = (chooseFallbackTurn() == ACTION_LEFT) ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
      plan.secondaryDurationMs = TURN_90_DURATION_MS;
      plan.confidence = 1.0f;
      Serial.println("Head-on wall escape: local forced plan");
    } else if (isValidDistance(frontDistanceCm) && frontDistanceCm <= EMERGENCY_REVERSE_CM) {
      plan = defaultPlan();
      plan.primary = MANEUVER_BACKWARD;
      plan.primaryDurationMs = BACKUP_DURATION_MS;
      plan.secondary = (chooseFallbackTurn() == ACTION_LEFT) ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
      plan.secondaryDurationMs = TURN_90_DURATION_MS;
      plan.confidence = 1.0f;
      Serial.println("Emergency close obstacle: local forced plan");
    } else if (shouldForceBackward(nowMs)) {
      plan = defaultPlan();
      plan.primary = MANEUVER_BACKWARD;
      plan.primaryDurationMs = BACKUP_DURATION_MS;
      plan.secondary = (chooseFallbackTurn() == ACTION_LEFT) ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
      plan.secondaryDurationMs = TURN_90_DURATION_MS;
      plan.confidence = 1.0f;
      Serial.println("Burst hazard escalation: local forced plan");
    } else {
      ManeuverPlan localPlan = chooseLocalPlan(repeatedTrap);
      plan = queryGeminiForNavigationPlan(localPlan, repeatedTrap);
    }

    executePlan(plan);
    lastHazardDecisionMs = nowMs;
  }

  if (!obstacleNearby) {
    if (hazardClearSinceMs == 0) {
      hazardClearSinceMs = nowMs;
    }
    if ((nowMs - hazardClearSinceMs) >= HAZARD_CLEAR_RESET_MS) {
      hazardBurstCount = 0;
      hazardBurstWindowStartMs = 0;
    }
  } else {
    hazardClearSinceMs = 0;
  }

  previousObstacleNearby = obstacleNearby;
  if (obstacleNearby) {
    myCar.Move(Stop, 0);
  } else {
    myCar.Move(Forward, FORWARD_SPEED);
  }

  delay(20);
}