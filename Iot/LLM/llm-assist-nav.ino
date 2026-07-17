
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

/*
  LLM-Assisted Indoor Navigation (ESP32 + Ultrasonic + Servo + Robot Chassis)

  What this sketch does:
  1) Continuously drives the robot forward while scanning left/front/right distance with
    an ultrasonic sensor mounted on a pan servo.
  2) Detects hazards based on front clearance and freshness of sensor data.
  3) Builds a local, safety-first maneuver plan (strafe, turn, reverse, rescan).
  4) Optionally asks Gemini for a refined plan when Wi-Fi is available, but keeps strict
    local safety guardrails and falls back to local logic whenever confidence is low or
    responses are invalid.
  5) Executes one or two-step maneuvers, then rescans and records outcome quality so
    future decisions can avoid oscillation/trap patterns.

  High-level design goals:
  - Prefer collision avoidance and recovery over aggressive movement.
  - Handle repeated trap patterns (e.g., left-right ping-pong near obstacles).
  - Keep behavior deterministic even when cloud/LLM is unavailable.
*/

vehicle myCar;
ultrasonic sensor;
Servo panServo;

// Action is a simple left/right/stop/backward direction abstraction used by fallback logic.
enum Action {
  ACTION_STOP = 0,
  ACTION_LEFT,
  ACTION_RIGHT,
  ACTION_BACKWARD,
};

// ManeuverType is the concrete motion primitive vocabulary used in executable plans.
enum ManeuverType {
  MANEUVER_STOP = 0,
  MANEUVER_STRAFE_LEFT,
  MANEUVER_STRAFE_RIGHT,
  MANEUVER_BACKWARD,
  MANEUVER_TURN_LEFT_90,
  MANEUVER_TURN_RIGHT_90,
  MANEUVER_RESCAN,
};

// Captures one L/F/R measurement set used for trap/repetition detection.
struct HazardSnapshot {
  float leftCm;
  float frontCm;
  float rightCm;
  unsigned long atMs;
};

// Stores a two-step maneuver decision with confidence/risk metadata.
struct ManeuverPlan {
  ManeuverType primary;
  uint16_t primaryDurationMs;
  ManeuverType secondary;
  uint16_t secondaryDurationMs;
  float confidence;
  float riskScore;
  bool repeatedTrap;
};

// Speed tuning for nominal movement and turning behavior.
const int FORWARD_SPEED = 255;
const int TURN_SPEED = 240;
const unsigned long TURN_90_DURATION_MS = 520;

// Hardware pin mapping.
const int TRIG_PIN = 13;
const int ECHO_PIN = 14;
const int ULTRASONIC_PAN_PIN = 27;
const int LEFT_LED_PIN = 2;
const int RIGHT_LED_PIN = 12;
const bool LED_ACTIVE_HIGH = true;

// Servo scan geometry and stepping behavior.
const int PAN_CENTER_TRIM_DEG = 0;
const int PAN_CENTER_DEG = 90 + PAN_CENTER_TRIM_DEG;
const int PAN_HALF_ARC_DEG = 85;
const int PAN_LEFT_DEG = ((PAN_CENTER_DEG + PAN_HALF_ARC_DEG) > 180) ? 180 : (PAN_CENTER_DEG + PAN_HALF_ARC_DEG);
const int PAN_RIGHT_DEG = ((PAN_CENTER_DEG - PAN_HALF_ARC_DEG) < 0) ? 0 : (PAN_CENTER_DEG - PAN_HALF_ARC_DEG);
const int PAN_STEP_DEG = 5;
const unsigned long PAN_STEP_INTERVAL_MS = 22;
const int PAN_CENTER_BUCKET_DEG = 18;

// Distance thresholds and validation rules (cm).
const float ULTRASONIC_ALERT_CM = 30.0f;
const float ULTRASONIC_MIN_VALID_CM = 4.0f;
const float EMERGENCY_REVERSE_CM = 18.0f;
const float WALL_HEADON_FRONT_CM = 28.0f;
const float WALL_HEADON_SIDE_CM = 42.0f;
const float WALL_HEADON_SIDE_BALANCE_CM = 12.0f;
const float WIDE_OPEN_DIFF_CM = 18.0f;
const float REPEAT_PATTERN_TOLERANCE_CM = 9.0f;

// Timing for sampling, maneuvers, cloud I/O, and hazard debouncing.
const unsigned long SENSOR_INTERVAL_MS = 70;
const unsigned long STRAFE_DURATION_MS = 450;
const unsigned long BACKUP_DURATION_MS = 380;
const unsigned long GEMINI_HTTP_TIMEOUT_MS = 12000;
const unsigned long GEMINI_HTTP_TIMEOUT_TIGHT_MS = 3800;
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

// Conservative defaults and planning memory sizes.
const int SAFE_UNKNOWN_DISTANCE_CM = 120;
const uint8_t NAV_HISTORY_SIZE = 8;
const uint8_t TRAP_REPEAT_THRESHOLD = 3;
const float MIN_LLM_CONFIDENCE = 0.45f;
const float MIN_LLM_CONFIDENCE_FOR_DISAGREEMENT = 0.72f;
const float PLAN_IMPROVEMENT_MARGIN_CM = 5.0f;
const uint16_t MIN_MANEUVER_DURATION_MS = 180;
const uint16_t MAX_MANEUVER_DURATION_MS = 700;

// Runtime state for hazard sensing/decision cadence.
bool obstacleNearby = false;
bool previousObstacleNearby = false;
unsigned long lastSensorMs = 0;
unsigned long lastPanStepMs = 0;
unsigned long lastHazardDecisionMs = 0;
unsigned long hazardBurstWindowStartMs = 0;
uint8_t hazardBurstCount = 0;
unsigned long hazardClearSinceMs = 0;

// Runtime state for servo sweep and latest sampled distances.
Action lastNonStopDecision = ACTION_LEFT;
bool panServoReady = false;
int panCurrentDeg = PAN_CENTER_DEG;
bool panSweepTowardLeft = true;
float leftDistanceCm = -1.0f;
float frontDistanceCm = -1.0f;
float rightDistanceCm = -1.0f;
unsigned long frontUpdatedMs = 0;

// Short-term memory of environment and recent decisions for anti-oscillation logic.
HazardSnapshot navHistory[NAV_HISTORY_SIZE];
uint8_t navHistoryCount = 0;
uint8_t navHistoryWriteIndex = 0;
ManeuverType recentPlans[4] = {MANEUVER_STOP, MANEUVER_STOP, MANEUVER_STOP, MANEUVER_STOP};
uint8_t recentPlanWriteIndex = 0;
int8_t lastPlanOutcome = 0;
float lastPlanFrontBeforeCm = -1.0f;
float lastPlanFrontAfterCm = -1.0f;

// Writes both direction LEDs in one call, honoring active-high/low wiring.
void setBothLeds(bool on) {
  uint8_t level = on ? (LED_ACTIVE_HIGH ? HIGH : LOW) : (LED_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(LEFT_LED_PIN, level);
  digitalWrite(RIGHT_LED_PIN, level);
}

// Short blink pattern used while waiting on Gemini planning response.
void flashGeminiThinkingLeds() {
  setBothLeds(true);
  delay(THINK_LED_ON_MS);
  setBothLeds(false);
  delay(THINK_LED_OFF_MS);
  setBothLeds(true);
  delay(THINK_LED_ON_MS);
  setBothLeds(false);
}

// Briefly flashes the LED that matches chosen turn/strafe direction.
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

// Connects to Wi-Fi once at startup; local planner remains functional if this fails.
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

// Filters out impossible/too-close sensor values that should not drive decisions.
bool isValidDistance(float distanceCm) {
  return distanceCm >= ULTRASONIC_MIN_VALID_CM;
}

// Converts invalid readings into a safe "assume open" value for scoring math.
int effectiveDistance(float distanceCm) {
  if (isValidDistance(distanceCm)) {
    return (int)distanceCm;
  }
  return SAFE_UNKNOWN_DISTANCE_CM;
}

// Picks the better side to turn based on effective clearance; alternates on ties.
Action chooseFallbackTurn() {
  int leftEff = effectiveDistance(leftDistanceCm);
  int rightEff = effectiveDistance(rightDistanceCm);
  if (leftEff == rightEff) {
    return lastNonStopDecision == ACTION_LEFT ? ACTION_RIGHT : ACTION_LEFT;
  }
  return (leftEff > rightEff) ? ACTION_LEFT : ACTION_RIGHT;
}
const char *maneuverTypeToString(ManeuverType maneuver);
uint16_t clampDuration(uint16_t value, uint16_t fallbackMs);
ManeuverPlan defaultPlan();
ManeuverType actionToStrafeManeuver(Action action) {
  return action == ACTION_LEFT ? MANEUVER_STRAFE_LEFT : MANEUVER_STRAFE_RIGHT;
}
ManeuverType actionToTurnManeuver(Action action) {
  return action == ACTION_LEFT ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
}

// Creates a one-step lateral move followed by a pause/rescan.
ManeuverPlan buildStrafePlan(Action action, uint16_t durationMs) {
  ManeuverPlan plan = defaultPlan();
  plan.primary = actionToStrafeManeuver(action);
  plan.primaryDurationMs = clampDuration(durationMs, STRAFE_DURATION_MS);
  plan.secondary = MANEUVER_RESCAN;
  plan.secondaryDurationMs = RESCAN_PAUSE_MS;
  plan.confidence = 0.0f;
  return plan;
}

// Creates a fixed-duration 90-degree turn followed by a pause/rescan.
ManeuverPlan buildTurnPlan(Action action) {
  ManeuverPlan plan = defaultPlan();
  plan.primary = actionToTurnManeuver(action);
  plan.primaryDurationMs = TURN_90_DURATION_MS;
  plan.secondary = MANEUVER_RESCAN;
  plan.secondaryDurationMs = RESCAN_PAUSE_MS;
  plan.confidence = 0.0f;
  return plan;
}

// Creates a recovery sequence: reverse, then rotate to escape local minima/traps.
ManeuverPlan buildRecoveryPlan(Action action) {
  ManeuverPlan plan = defaultPlan();
  plan.primary = MANEUVER_BACKWARD;
  plan.primaryDurationMs = BACKUP_DURATION_MS;
  plan.secondary = actionToTurnManeuver(action);
  plan.secondaryDurationMs = TURN_90_DURATION_MS;
  plan.confidence = 0.0f;
  return plan;
}

// Serializes plan fields into compact text for the Gemini prompt context.
String planToPromptText(const char *label, const ManeuverPlan &plan) {
  return String(label) + "={primary:" + maneuverTypeToString(plan.primary) +
         ",primary_ms:" + String(plan.primaryDurationMs) +
         ",secondary:" + maneuverTypeToString(plan.secondary) +
         ",secondary_ms:" + String(plan.secondaryDurationMs) +
         ",confidence:" + String(plan.confidence, 2) +
         ",risk:" + String(plan.riskScore, 2) + "}";
}

// Heuristic local scorer for candidate maneuvers based on clearance, trend, and oscillation.
float scoreLocalPlanCandidate(const ManeuverPlan &candidate, int leftEff, int frontEff, int rightEff, bool repeatedTrap, uint8_t oscillationCount, float frontTrendCmValue) {
  float score = 0.0f;
  int preferredSideEff = 0;
  if (candidate.primary == MANEUVER_STRAFE_LEFT || candidate.primary == MANEUVER_TURN_LEFT_90) {
    preferredSideEff = leftEff;
  } else if (candidate.primary == MANEUVER_STRAFE_RIGHT || candidate.primary == MANEUVER_TURN_RIGHT_90) {
    preferredSideEff = rightEff;
  }
  switch (candidate.primary) {
    case MANEUVER_BACKWARD:
      score = 0.42f;
      if (frontEff <= EMERGENCY_REVERSE_CM) {
        score += 0.35f;
      } else if (frontEff <= ULTRASONIC_ALERT_CM) {
        score += 0.18f;
      } else {
        score -= 0.22f;
      }
      if (repeatedTrap) {
        score += 0.18f;
      }
      if (frontTrendCmValue < -4.0f) {
        score += 0.08f;
      }
      break;
    case MANEUVER_STRAFE_LEFT:
    case MANEUVER_STRAFE_RIGHT:
      score = 0.48f + ((preferredSideEff - frontEff) / 180.0f);
      score += (preferredSideEff - ((candidate.primary == MANEUVER_STRAFE_LEFT) ? rightEff : leftEff)) / 220.0f;
      if (frontEff <= ULTRASONIC_ALERT_CM) {
        score += 0.08f;
      } else {
        score -= 0.04f;
      }
      if (repeatedTrap) {
        score -= 0.05f;
      }
      break;
    case MANEUVER_TURN_LEFT_90:
    case MANEUVER_TURN_RIGHT_90:
      score = 0.44f + (preferredSideEff / 240.0f);
      if (frontEff <= ULTRASONIC_ALERT_CM) {
        score += 0.12f;
      }
      if (repeatedTrap || oscillationCount >= 2) {
        score += 0.10f;
      }
      if (frontTrendCmValue < -4.0f) {
        score += 0.05f;
      }
      break;
    case MANEUVER_RESCAN:
      score = 0.16f;
      if (frontTrendCmValue < 0.0f || oscillationCount >= 2) {
        score += 0.10f;
      }
      break;
    default:
      score = 0.0f;
      break;
  }
  if (candidate.secondary == MANEUVER_RESCAN) {
    score += 0.05f;
  }
  if (candidate.primary == actionToStrafeManeuver(lastNonStopDecision) && oscillationCount >= 2 && !repeatedTrap) {
    score -= 0.08f;
  }
  if ((candidate.primary == MANEUVER_TURN_LEFT_90 || candidate.primary == MANEUVER_STRAFE_LEFT) && leftEff < rightEff) {
    score -= 0.02f;
  }
  if ((candidate.primary == MANEUVER_TURN_RIGHT_90 || candidate.primary == MANEUVER_STRAFE_RIGHT) && rightEff < leftEff) {
    score -= 0.02f;
  }
  if (score < 0.0f) {
    score = 0.0f;
  }
  if (score > 1.0f) {
    score = 1.0f;
  }
  return score;
}

// Detects a symmetric near-wall condition where reverse+turn is usually safest.
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

// String helpers for logs/prompting.
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

// Robust parser that accepts canonical tokens and common directional synonyms.
ManeuverType parseManeuverType(const String &text) {
  String normalized = text;
  normalized.toUpperCase();
  normalized.trim();
  if (normalized.indexOf("TURN_LEFT_90") >= 0) {
    return MANEUVER_TURN_LEFT_90;
  }
  if (normalized.indexOf("TURN_RIGHT_90") >= 0) {
    return MANEUVER_TURN_RIGHT_90;
  }
  if (normalized.indexOf("STRAFE_LEFT") >= 0 || normalized.indexOf("LEFT") >= 0) {
    return MANEUVER_STRAFE_LEFT;
  }
  if (normalized.indexOf("STRAFE_RIGHT") >= 0 || normalized.indexOf("RIGHT") >= 0) {
    return MANEUVER_STRAFE_RIGHT;
  }
  if (normalized.indexOf("BACKWARD") >= 0 || normalized.indexOf("REVERSE") >= 0) {
    return MANEUVER_BACKWARD;
  }
  if (normalized.indexOf("RESCAN") >= 0) {
    return MANEUVER_RESCAN;
  }
  return MANEUVER_STOP;
}

// Keeps plan durations within practical and safe motion bounds.
uint16_t clampDuration(uint16_t value, uint16_t fallbackMs) {
  if (value < MIN_MANEUVER_DURATION_MS || value > MAX_MANEUVER_DURATION_MS) {
    return fallbackMs;
  }
  return value;
}

// Stores the last few executed maneuvers to detect left-right oscillation patterns.
void rememberPlan(ManeuverType maneuver) {
  recentPlans[recentPlanWriteIndex] = maneuver;
  recentPlanWriteIndex = (recentPlanWriteIndex + 1) % 4;
}

// Adds one hazard snapshot to a ring buffer for repeat-pattern analysis.
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

// Compares new snapshot to previous snapshots with tolerance for noisy sensors.
bool snapshotSimilar(const HazardSnapshot &a, float leftCm, float frontCm, float rightCm) {
  return fabs(a.leftCm - leftCm) <= REPEAT_PATTERN_TOLERANCE_CM &&
         fabs(a.frontCm - frontCm) <= REPEAT_PATTERN_TOLERANCE_CM &&
         fabs(a.rightCm - rightCm) <= REPEAT_PATTERN_TOLERANCE_CM;
}

// Flags likely trap states when several recent snapshots look essentially the same.
bool detectRepeatedTrap(float leftCm, float frontCm, float rightCm) {
  uint8_t similarCount = 0;
  for (uint8_t i = 0; i < navHistoryCount; ++i) {
    if (snapshotSimilar(navHistory[i], leftCm, frontCm, rightCm)) {
      similarCount++;
    }
  }
  return similarCount >= TRAP_REPEAT_THRESHOLD;
}

// Produces CSV-like summary of recent plans for prompt/debug visibility.
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

// Produces compact rolling L/F/R history string for logs and LLM grounding.
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

// Positive means front clearance improved over history window; negative means worsening.
float frontTrendCm() {
  if (navHistoryCount < 2) {
    return 0.0f;
  }
  uint8_t newest = (navHistoryWriteIndex + NAV_HISTORY_SIZE - 1) % NAV_HISTORY_SIZE;
  uint8_t oldest = (navHistoryCount == NAV_HISTORY_SIZE) ? navHistoryWriteIndex : 0;
  return navHistory[newest].frontCm - navHistory[oldest].frontCm;
}

// Counts directional flips (left<->right) in the recent plan buffer.
uint8_t detectPlanOscillationCount() {
  uint8_t swaps = 0;
  for (uint8_t i = 1; i < 4; ++i) {
    uint8_t prevIdx = (recentPlanWriteIndex + i - 1) % 4;
    uint8_t currIdx = (recentPlanWriteIndex + i) % 4;
    ManeuverType prev = recentPlans[prevIdx];
    ManeuverType curr = recentPlans[currIdx];
    bool leftRightSwap = (prev == MANEUVER_STRAFE_LEFT && curr == MANEUVER_STRAFE_RIGHT) ||
                         (prev == MANEUVER_STRAFE_RIGHT && curr == MANEUVER_STRAFE_LEFT) ||
                         (prev == MANEUVER_TURN_LEFT_90 && curr == MANEUVER_TURN_RIGHT_90) ||
                         (prev == MANEUVER_TURN_RIGHT_90 && curr == MANEUVER_TURN_LEFT_90);
    if (leftRightSwap) {
      swaps++;
    }
  }
  return swaps;
}

// Human-readable label of previous maneuver result quality.
const char *lastPlanOutcomeString() {
  if (lastPlanOutcome > 0) {
    return "improved_clearance";
  }
  if (lastPlanOutcome < 0) {
    return "worsened_or_no_gain";
  }
  return "unknown";
}

// Estimates local collision risk in [0..1] with heavier front weighting.
float estimateLocalRiskScore() {
  float frontEff = (float)effectiveDistance(frontDistanceCm);
  float leftEff = (float)effectiveDistance(leftDistanceCm);
  float rightEff = (float)effectiveDistance(rightDistanceCm);
  float minSide = (leftEff < rightEff) ? leftEff : rightEff;
  float frontRisk = 1.0f - (frontEff / 120.0f);
  float sideRisk = 1.0f - (minSide / 120.0f);
  float weighted = (frontRisk * 0.7f) + (sideRisk * 0.3f);
  if (weighted < 0.0f) {
    weighted = 0.0f;
  }
  if (weighted > 1.0f) {
    weighted = 1.0f;
  }
  return weighted;
}

// Returns a conservative default (mostly stop) plan scaffold with current risk estimate.
ManeuverPlan defaultPlan() {
  ManeuverPlan plan;
  plan.primary = MANEUVER_STOP;
  plan.primaryDurationMs = 220;
  plan.secondary = MANEUVER_STOP;
  plan.secondaryDurationMs = 0;
  plan.confidence = 0.0f;
  plan.riskScore = estimateLocalRiskScore();
  plan.repeatedTrap = false;
  return plan;
}

// Pure local planner used both as baseline and as fallback when cloud planning is unavailable.
ManeuverPlan chooseLocalPlan(bool repeatedTrap) {
  ManeuverPlan plan = defaultPlan();
  plan.repeatedTrap = repeatedTrap;
  int leftEff = effectiveDistance(leftDistanceCm);
  int rightEff = effectiveDistance(rightDistanceCm);
  int frontEff = effectiveDistance(frontDistanceCm);
  Action preferredTurn = chooseFallbackTurn();
  uint8_t oscillationCount = detectPlanOscillationCount();
  float frontTrend = frontTrendCm();
  if (frontEff <= EMERGENCY_REVERSE_CM || repeatedTrap) {
    plan = buildRecoveryPlan(preferredTurn);
    plan.confidence = repeatedTrap ? 0.95f : 0.85f;
    return plan;
  }
  Action openSideAction = chooseFallbackTurn();
  int sideGap = abs(leftEff - rightEff);
  uint16_t strafeDuration = STRAFE_DURATION_MS;
  if (sideGap >= WIDE_OPEN_DIFF_CM) {
    strafeDuration -= 60;
  } else if (sideGap >= 10) {
    strafeDuration -= 20;
  } else {
    strafeDuration += 30;
  }
  if (frontEff <= ULTRASONIC_ALERT_CM) {
    strafeDuration -= 20;
  } else if (frontEff >= 45) {
    strafeDuration += 20;
  }
  if (strafeDuration < MIN_MANEUVER_DURATION_MS) {
    strafeDuration = MIN_MANEUVER_DURATION_MS;
  }
  if (strafeDuration > MAX_MANEUVER_DURATION_MS) {
    strafeDuration = MAX_MANEUVER_DURATION_MS;
  }
  ManeuverPlan strafePlan = buildStrafePlan(openSideAction, strafeDuration);
  ManeuverPlan turnPlan = buildTurnPlan(openSideAction);
  ManeuverPlan recoveryPlan = buildRecoveryPlan(openSideAction);
  float strafeScore = scoreLocalPlanCandidate(strafePlan, leftEff, frontEff, rightEff, repeatedTrap, oscillationCount, frontTrend);
  float turnScore = scoreLocalPlanCandidate(turnPlan, leftEff, frontEff, rightEff, repeatedTrap, oscillationCount, frontTrend);
  float recoveryScore = scoreLocalPlanCandidate(recoveryPlan, leftEff, frontEff, rightEff, repeatedTrap, oscillationCount, frontTrend);
  ManeuverPlan bestPlan = strafePlan;
  float bestScore = strafeScore;
  if (turnScore > bestScore) {
    bestPlan = turnPlan;
    bestScore = turnScore;
  }
  if (recoveryScore > bestScore) {
    bestPlan = recoveryPlan;
    bestScore = recoveryScore;
  }
  bestPlan.riskScore = estimateLocalRiskScore();
  bestPlan.repeatedTrap = repeatedTrap;
  bestPlan.confidence = bestScore;
  return bestPlan;
}

// Extracts the first complete JSON object from model text that may include extra wrapping.
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

// Parses model JSON into ManeuverPlan and normalizes numeric bounds.
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
  float risk = planDoc["risk"] | estimateLocalRiskScore();
  plan.primary = parseManeuverType(primary);
  plan.primaryDurationMs = clampDuration(primaryDuration, STRAFE_DURATION_MS);
  plan.secondary = parseManeuverType(secondary);
  plan.secondaryDurationMs = secondaryDuration == 0 ? 0 : clampDuration(secondaryDuration, RESCAN_PAUSE_MS);
  plan.confidence = confidence;
  if (risk < 0.0f) {
    risk = 0.0f;
  }
  if (risk > 1.0f) {
    risk = 1.0f;
  }
  plan.riskScore = risk;
  return true;
}
bool isPlanDirectionLeft(ManeuverType maneuver) {
  return maneuver == MANEUVER_STRAFE_LEFT || maneuver == MANEUVER_TURN_LEFT_90;
}
bool isPlanDirectionRight(ManeuverType maneuver) {
  return maneuver == MANEUVER_STRAFE_RIGHT || maneuver == MANEUVER_TURN_RIGHT_90;
}

// Returns true when two plans choose opposite lateral directions.
bool plansDisagreeDirection(const ManeuverPlan &a, const ManeuverPlan &b) {
  bool aLeft = isPlanDirectionLeft(a.primary);
  bool aRight = isPlanDirectionRight(a.primary);
  bool bLeft = isPlanDirectionLeft(b.primary);
  bool bRight = isPlanDirectionRight(b.primary);
  return (aLeft && bRight) || (aRight && bLeft);
}

// Applies safety guardrails to LLM output and falls back when output is weak/risky.
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
    return;
  }
  if (plansDisagreeDirection(plan, fallbackPlan) && plan.confidence < MIN_LLM_CONFIDENCE_FOR_DISAGREEMENT) {
    plan = fallbackPlan;
    return;
  }
}

// Calls Gemini for plan refinement; never bypasses local fallbacks/safety sanitization.
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
  Action openSideAction = chooseFallbackTurn();
  ManeuverPlan localStrafePlan = buildStrafePlan(openSideAction, fallbackPlan.primaryDurationMs);
  ManeuverPlan localTurnPlan = buildTurnPlan(openSideAction);
  ManeuverPlan localRecoveryPlan = buildRecoveryPlan(openSideAction);
  localStrafePlan.riskScore = estimateLocalRiskScore();
  localTurnPlan.riskScore = estimateLocalRiskScore();
  localRecoveryPlan.riskScore = estimateLocalRiskScore();
  String candidateSummary = planToPromptText("baseline", fallbackPlan) + "; " +
                            planToPromptText("strafe", localStrafePlan) + "; " +
                            planToPromptText("turn", localTurnPlan) + "; " +
                            planToPromptText("recovery", localRecoveryPlan);
  String prompt =
      "You are a safety-first indoor navigation planner for a robot. "
      "Choose the safest maneuver that is most likely to increase front clearance and break oscillation. "
      "You may only use these maneuvers: STRAFE_LEFT, STRAFE_RIGHT, BACKWARD, TURN_LEFT_90, TURN_RIGHT_90, RESCAN, STOP. "
      "Never invent a new action. Do not use STOP unless all motion options are blocked or unsafe. "
      "If repeatedTrap is true or front trend is negative, prefer a recovery maneuver that backs up before turning. "
      "Prefer the more open side when choosing between left and right. "
      "If the baseline candidate is already safe, keep it unless another candidate clearly improves clearance. "
      "Return strict JSON only, no markdown, with keys primary, primary_duration_ms, secondary, secondary_duration_ms, confidence, risk, reason. "
      "Current distances cm: front=" + String(frontDistanceCm, 1) +
      ", left=" + String(leftDistanceCm, 1) +
      ", right=" + String(rightDistanceCm, 1) +
      ". Repeated trap=" + String(repeatedTrap ? "true" : "false") +
      ". Front trend cm over recent history=" + String(frontTrendCm(), 1) +
      ". Left-right oscillation count=" + String(detectPlanOscillationCount()) +
      ". Last plan outcome=" + String(lastPlanOutcomeString()) +
      ". Last plan front before=" + String(lastPlanFrontBeforeCm, 1) +
      ", after=" + String(lastPlanFrontAfterCm, 1) +
      ". Baseline primary=" + String(maneuverTypeToString(fallbackPlan.primary)) +
      ", baseline secondary=" + String(maneuverTypeToString(fallbackPlan.secondary)) +
      ". Candidate plans: " + candidateSummary +
      ". Recent hazard history=" + buildHistorySummary() +
      ". Recent plans=" + buildRecentPlanSummary() +
      ". Example JSON: {\"primary\":\"BACKWARD\",\"primary_duration_ms\":350,\"secondary\":\"TURN_LEFT_90\",\"secondary_duration_ms\":520,\"confidence\":0.82,\"risk\":0.74,\"reason\":\"recovery_from_trap\"}.";
  // responseMimeType requests JSON, but code still validates/guards against malformed output.
  String body = String("{\"contents\":[{\"parts\":[{\"text\":\"") + prompt +
          "\"}]}],\"generationConfig\":{\"temperature\":0.15,\"maxOutputTokens\":160,\"responseMimeType\":\"application/json\"}}";
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(GEMINI_HTTP_TIMEOUT_TIGHT_MS);
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

// Returns pan servo to center for forward-facing sensing and cleaner drive state.
void movePanToCenter() {
  if (!panServoReady) {
    return;
  }
  panCurrentDeg = PAN_CENTER_DEG;
  panServo.write(panCurrentDeg);
}

// Takes an explicit left-right-front snapshot used right before/after maneuvers.
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

// Runs continuous servo sweep so periodic samples can approximate L/F/R zones.
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

// Updates rolling scan values and derives current hazard state from fresh front data.
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

// Escalates to forced reverse when hazards repeat in a short burst window.
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

// Converts confidence/risk into safer motor speed for the selected maneuver.
int computeManeuverSpeed(const ManeuverPlan &plan) {
  int speed = TURN_SPEED;
  // Lower speed when risk is high or confidence is modest to reduce collision risk.
  if (plan.riskScore >= 0.75f || plan.confidence < 0.65f) {
    speed -= 40;
  } else if (plan.riskScore >= 0.55f) {
    speed -= 20;
  }
  if (plan.primary == MANEUVER_BACKWARD) {
    speed -= 15;
  }
  if (speed < 165) {
    speed = 165;
  }
  if (speed > TURN_SPEED) {
    speed = TURN_SPEED;
  }
  return speed;
}

// Executes a single motion primitive for a bounded duration, then hard-stops.
void executeManeuver(ManeuverType maneuver, uint16_t durationMs, int speed) {
  switch (maneuver) {
    case MANEUVER_STRAFE_LEFT:
      myCar.Move(Move_Left, speed);
      delay(durationMs);
      break;
    case MANEUVER_STRAFE_RIGHT:
      myCar.Move(Move_Right, speed);
      delay(durationMs);
      break;
    case MANEUVER_BACKWARD:
      myCar.Move(Backward, speed);
      delay(durationMs);
      break;
    case MANEUVER_TURN_LEFT_90:
      myCar.Move(Contrarotate, speed);
      delay(TURN_90_DURATION_MS);
      break;
    case MANEUVER_TURN_RIGHT_90:
      myCar.Move(Clockwise, speed);
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

// Executes primary and optional secondary maneuver; updates directional memory.
void executePlan(const ManeuverPlan &plan) {
  int maneuverSpeed = computeManeuverSpeed(plan);
  executeManeuver(plan.primary, plan.primaryDurationMs, maneuverSpeed);
  rememberPlan(plan.primary);
  if (plan.secondary != MANEUVER_STOP) {
    executeManeuver(plan.secondary, plan.secondaryDurationMs, maneuverSpeed);
    rememberPlan(plan.secondary);
  }
  if (plan.primary == MANEUVER_STRAFE_LEFT || plan.primary == MANEUVER_TURN_LEFT_90) {
    lastNonStopDecision = ACTION_LEFT;
  } else if (plan.primary == MANEUVER_STRAFE_RIGHT || plan.primary == MANEUVER_TURN_RIGHT_90) {
    lastNonStopDecision = ACTION_RIGHT;
  }
}

// Hardware/network initialization and initial calibration logging.
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

// Main control loop: scan -> detect hazard -> decide plan -> execute -> learn outcome.
void loop() {
  updateScanAndHazard();
  unsigned long nowMs = millis();
  if (obstacleNearby && (!previousObstacleNearby || (nowMs - lastHazardDecisionMs) >= HAZARD_DECISION_COOLDOWN_MS)) {
    myCar.Move(Stop, 0);
    refreshHazardScanSnapshot();
    recordHazardSnapshot(leftDistanceCm, frontDistanceCm, rightDistanceCm, nowMs);
    ManeuverPlan plan;
    bool repeatedTrap = detectRepeatedTrap(leftDistanceCm, frontDistanceCm, rightDistanceCm);
    // These branches are hard safety overrides that bypass LLM planning entirely.
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
      // Normal path: create local baseline plan, then allow Gemini to refine within guardrails.
      ManeuverPlan localPlan = chooseLocalPlan(repeatedTrap);
      plan = queryGeminiForNavigationPlan(localPlan, repeatedTrap);
    }
    lastPlanFrontBeforeCm = frontDistanceCm;
    executePlan(plan);
    refreshHazardScanSnapshot();
    lastPlanFrontAfterCm = frontDistanceCm;
    if (isValidDistance(lastPlanFrontBeforeCm) && isValidDistance(lastPlanFrontAfterCm)) {
      // Post-action scoring tracks whether front clearance got better/worse for future decisions.
      float gain = lastPlanFrontAfterCm - lastPlanFrontBeforeCm;
      if (gain >= PLAN_IMPROVEMENT_MARGIN_CM) {
        lastPlanOutcome = 1;
      } else if (gain <= -PLAN_IMPROVEMENT_MARGIN_CM) {
        lastPlanOutcome = -1;
      } else {
        lastPlanOutcome = 0;
      }
    } else {
      lastPlanOutcome = 0;
    }
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
  // Cruise forward only when no front hazard is active.
  if (obstacleNearby) {
    myCar.Move(Stop, 0);
  } else {
    myCar.Move(Forward, FORWARD_SPEED);
  }
  delay(20);
}
