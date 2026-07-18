
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ultrasonic.h>
#include <vehicle.h>
#include "foundry_config.h"
#include "wifi_config.h"

/*
 * What this sketch does
 * ---------------------
 * This program controls a small car that normally drives forward, then pauses and
 * plans a maneuver whenever an obstacle/hazard is detected by ultrasonic sensing.
 *
 * Planning pipeline:
 * 1) Collect distance data (continuous front updates + focused left/front/right scan).
 * 2) Build a local fallback maneuver plan from deterministic safety heuristics.
 * 3) Optionally ask Azure AI Foundry to arbitrate among maneuver candidates.
 * 4) Validate/sanitize model output and execute the selected primary/secondary plan.
 * 5) Track outcomes to reduce oscillation and repeated trap patterns over time.
 */

// Hardware control objects.
vehicle myCar;
ultrasonic sensor;
Servo panServo;

// Coarse directional decision memory for local fallback behavior.
enum Action {
  ACTION_STOP = 0,
  ACTION_LEFT,
  ACTION_RIGHT,
  ACTION_BACKWARD,
};

// Primitive motion commands that can be executed by the car.
enum ManeuverType {
  MANEUVER_STOP = 0,
  MANEUVER_STRAFE_LEFT,
  MANEUVER_STRAFE_RIGHT,
  MANEUVER_BACKWARD,
  MANEUVER_TURN_LEFT_90,
  MANEUVER_TURN_RIGHT_90,
  MANEUVER_RESCAN,
};

// One hazard observation frame used for pattern history/trap detection.
struct HazardSnapshot {
  float leftCm;
  float frontCm;
  float rightCm;
  unsigned long atMs;
};

// A 2-step maneuver plan plus confidence/risk metadata.
struct ManeuverPlan {
  ManeuverType primary;
  uint16_t primaryDurationMs;
  ManeuverType secondary;
  uint16_t secondaryDurationMs;
  float confidence;
  float riskScore;
  bool repeatedTrap;
};

// Motion tuning constants.
const int FORWARD_SPEED = 190;
const int TURN_SPEED = 240;
const unsigned long TURN_90_DURATION_MS = 520;

// Pin assignments.
const int TRIG_PIN = 13;
const int ECHO_PIN = 14;
const int ULTRASONIC_PAN_PIN = 27;
const int LEFT_LED_PIN = 2;
const int RIGHT_LED_PIN = 12;
const bool LED_ACTIVE_HIGH = true;

// Servo scan geometry and movement cadence.
const int PAN_CENTER_TRIM_DEG = 0;
const int PAN_CENTER_DEG = 90 + PAN_CENTER_TRIM_DEG;
const int PAN_HALF_ARC_DEG = 85;
const int PAN_LEFT_DEG = ((PAN_CENTER_DEG + PAN_HALF_ARC_DEG) > 180) ? 180 : (PAN_CENTER_DEG + PAN_HALF_ARC_DEG);
const int PAN_RIGHT_DEG = ((PAN_CENTER_DEG - PAN_HALF_ARC_DEG) < 0) ? 0 : (PAN_CENTER_DEG - PAN_HALF_ARC_DEG);
const int PAN_STEP_DEG = 5;
const unsigned long PAN_STEP_INTERVAL_MS = 22;
const int PAN_CENTER_BUCKET_DEG = 18;

// Distance thresholds (cm) for hazard and escape logic.
const float ULTRASONIC_ALERT_CM = 45.0f;
const float ULTRASONIC_CLEAR_CM = 50.0f;
const float ULTRASONIC_MIN_VALID_CM = 4.0f;
const float EMERGENCY_REVERSE_CM = 18.0f;
const float WALL_HEADON_FRONT_CM = 28.0f;
const float WALL_HEADON_SIDE_CM = 42.0f;
const float WALL_HEADON_SIDE_BALANCE_CM = 12.0f;
const float WIDE_OPEN_DIFF_CM = 18.0f;
const float REPEAT_PATTERN_TOLERANCE_CM = 9.0f;

// Sensor and maneuver timing.
const unsigned long SENSOR_INTERVAL_MS = 70;
const unsigned long STRAFE_DURATION_MS = 450;
const unsigned long BACKUP_DURATION_MS = 380;
const uint8_t ALL_UNKNOWN_STREAK_THRESHOLD = 8;

// Foundry request limits/timeouts.
const uint16_t FOUNDRY_MAX_OUTPUT_TOKENS = 160;
const uint16_t FOUNDRY_RETRY_OUTPUT_TOKENS = 120;
const unsigned long FOUNDRY_HTTP_TIMEOUT_TIGHT_MS = 3800;
const unsigned long FOUNDRY_HTTP_TIMEOUT_RETRY_MS = 9000;

// Decision pacing and stale-data handling.
const unsigned long HAZARD_DECISION_COOLDOWN_MS = 800;
const unsigned long FRONT_STALE_MS = 450;
const uint8_t FRONT_BLIND_STREAK_THRESHOLD = 4;
const unsigned long HAZARD_BURST_WINDOW_MS = 3000;
const uint8_t HAZARD_BURST_THRESHOLD = 2;
const unsigned long HAZARD_CLEAR_RESET_MS = 1200;

// Scan settle and visual telemetry timing.
const unsigned long PAN_SETTLE_MS = 120;
const unsigned long RESCAN_PAUSE_MS = 140;
const unsigned long DECISION_STOP_PAUSE_MS = 250;
const unsigned long THINK_LED_ON_MS = 140;
const unsigned long THINK_LED_OFF_MS = 80;
const unsigned long DECISION_LED_MS = 220;

// Filtering/history/planner guardrail settings.
const float FRONT_EMA_ALPHA = 0.35f;
const int SAFE_UNKNOWN_DISTANCE_CM = 120;
const uint8_t NAV_HISTORY_SIZE = 8;
const uint8_t TRAP_REPEAT_THRESHOLD = 3;
const float MIN_LLM_CONFIDENCE = 0.45f;
const float MIN_LLM_CONFIDENCE_FOR_DISAGREEMENT = 0.72f;
const float PLAN_IMPROVEMENT_MARGIN_CM = 5.0f;
const uint16_t MIN_MANEUVER_DURATION_MS = 180;
const uint16_t MAX_MANEUVER_DURATION_MS = 700;

// Runtime state and rolling telemetry/history.
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
float frontFilteredCm = -1.0f;
bool frontFilterReady = false;
unsigned long frontUpdatedMs = 0;
HazardSnapshot navHistory[NAV_HISTORY_SIZE];
uint8_t navHistoryCount = 0;
uint8_t navHistoryWriteIndex = 0;
ManeuverType recentPlans[4] = {MANEUVER_STOP, MANEUVER_STOP, MANEUVER_STOP, MANEUVER_STOP};
uint8_t recentPlanWriteIndex = 0;
int8_t lastPlanOutcome = 0;
float lastPlanFrontBeforeCm = -1.0f;
float lastPlanFrontAfterCm = -1.0f;
uint8_t allUnknownStreak = 0;
uint8_t frontBlindStreak = 0;
bool foundryRequestSent = false;
bool foundryResponseOk = false;
bool foundryPlanParsed = false;
String foundryDecisionStatus = "idle";

// Turn both status LEDs on/off, honoring active-high vs active-low wiring.
void setBothLeds(bool on) {
  uint8_t level = on ? (LED_ACTIVE_HIGH ? HIGH : LOW) : (LED_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(LEFT_LED_PIN, level);
  digitalWrite(RIGHT_LED_PIN, level);
}

// Visual "thinking" pattern while waiting for cloud decisioning.
void flashFoundryThinkingLeds() {
  setBothLeds(true);
  delay(THINK_LED_ON_MS);
  setBothLeds(false);
  delay(THINK_LED_OFF_MS);
  setBothLeds(true);
  delay(THINK_LED_ON_MS);
  setBothLeds(false);
}

// Blink directional LED to show left/right bias of selected maneuver.
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

// Connect once at startup; if disconnected later, planner falls back locally.
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

// Reject invalid/too-close readings that are likely ultrasonic noise.
bool isValidDistance(float distanceCm) {
  return distanceCm >= ULTRASONIC_MIN_VALID_CM;
}

// True when left/front/right readings are all currently unavailable.
bool allDistancesUnknown() {
  return !isValidDistance(leftDistanceCm) && !isValidDistance(frontDistanceCm) && !isValidDistance(rightDistanceCm);
}

// Convert unknown readings to a conservative safe-default value for scoring.
int effectiveDistance(float distanceCm) {
  if (isValidDistance(distanceCm)) {
    return (int)distanceCm;
  }
  return SAFE_UNKNOWN_DISTANCE_CM;
}

// Choose left/right fallback direction using larger open space and tie-flip memory.
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
bool extractFoundryModelText(const JsonDocument &responseDoc, String &modelText);
bool extractFoundryModelTextFromRawResponse(const String &responseBody, String &modelText);
void updateFrontDistanceEstimate(float measuredCm, unsigned long nowMs, bool resetFilter);

// Maintain a smoothed front distance using an exponential moving average.
void updateFrontDistanceEstimate(float measuredCm, unsigned long nowMs, bool resetFilter) {
  if (!isValidDistance(measuredCm)) {
    return;
  }
  if (resetFilter || !frontFilterReady || !isValidDistance(frontFilteredCm)) {
    frontFilteredCm = measuredCm;
    frontFilterReady = true;
  } else {
    frontFilteredCm = (FRONT_EMA_ALPHA * measuredCm) + ((1.0f - FRONT_EMA_ALPHA) * frontFilteredCm);
  }
  frontDistanceCm = frontFilteredCm;
  frontUpdatedMs = nowMs;
}

// Map high-level side choice to strafe primitive.
ManeuverType actionToStrafeManeuver(Action action) {
  return action == ACTION_LEFT ? MANEUVER_STRAFE_LEFT : MANEUVER_STRAFE_RIGHT;
}

// Map high-level side choice to 90-degree turn primitive.
ManeuverType actionToTurnManeuver(Action action) {
  return action == ACTION_LEFT ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
}

// Build a lateral move and add a short rescan as a secondary action.
ManeuverPlan buildStrafePlan(Action action, uint16_t durationMs) {
  ManeuverPlan plan = defaultPlan();
  plan.primary = actionToStrafeManeuver(action);
  plan.primaryDurationMs = clampDuration(durationMs, STRAFE_DURATION_MS);
  plan.secondary = MANEUVER_RESCAN;
  plan.secondaryDurationMs = RESCAN_PAUSE_MS;
  plan.confidence = 0.0f;
  return plan;
}

// Build a pure turn plan followed by a rescan.
ManeuverPlan buildTurnPlan(Action action) {
  ManeuverPlan plan = defaultPlan();
  plan.primary = actionToTurnManeuver(action);
  plan.primaryDurationMs = TURN_90_DURATION_MS;
  plan.secondary = MANEUVER_RESCAN;
  plan.secondaryDurationMs = RESCAN_PAUSE_MS;
  plan.confidence = 0.0f;
  return plan;
}

// Build a recovery plan that backs up first, then turns.
ManeuverPlan buildRecoveryPlan(Action action) {
  ManeuverPlan plan = defaultPlan();
  plan.primary = MANEUVER_BACKWARD;
  plan.primaryDurationMs = BACKUP_DURATION_MS;
  plan.secondary = actionToTurnManeuver(action);
  plan.secondaryDurationMs = TURN_90_DURATION_MS;
  plan.confidence = 0.0f;
  return plan;
}

// Compact candidate-plan serialization used in the LLM prompt.
String planToPromptText(const char *label, const ManeuverPlan &plan) {
  return String(label) + "={primary:" + maneuverTypeToString(plan.primary) +
         ",primary_ms:" + String(plan.primaryDurationMs) +
         ",secondary:" + maneuverTypeToString(plan.secondary) +
         ",secondary_ms:" + String(plan.secondaryDurationMs) +
         ",confidence:" + String(plan.confidence, 2) +
         ",risk:" + String(plan.riskScore, 2) + "}";
}

// Heuristic candidate scoring for local planning when cloud guidance is absent/untrusted.
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

// Detect a likely head-on wall condition where reversing is safer than sliding.
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

// Convert maneuver enum values to stable logging/prompt strings.
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

// Parse maneuver names from model text with tolerant matching.
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

// Keep maneuver durations within configured safe bounds.
uint16_t clampDuration(uint16_t value, uint16_t fallbackMs) {
  if (value < MIN_MANEUVER_DURATION_MS || value > MAX_MANEUVER_DURATION_MS) {
    return fallbackMs;
  }
  return value;
}

// Track recently executed maneuvers in a ring buffer for oscillation analysis.
void rememberPlan(ManeuverType maneuver) {
  recentPlans[recentPlanWriteIndex] = maneuver;
  recentPlanWriteIndex = (recentPlanWriteIndex + 1) % 4;
}

// Save one hazard scan frame to rolling navigation history.
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

// Compare two scan frames with tolerance to detect repeated geometry.
bool snapshotSimilar(const HazardSnapshot &a, float leftCm, float frontCm, float rightCm) {
  return fabs(a.leftCm - leftCm) <= REPEAT_PATTERN_TOLERANCE_CM &&
         fabs(a.frontCm - frontCm) <= REPEAT_PATTERN_TOLERANCE_CM &&
         fabs(a.rightCm - rightCm) <= REPEAT_PATTERN_TOLERANCE_CM;
}

// Repeated trap is true when similar layouts recur often in recent history.
bool detectRepeatedTrap(float leftCm, float frontCm, float rightCm) {
  uint8_t similarCount = 0;
  for (uint8_t i = 0; i < navHistoryCount; ++i) {
    if (snapshotSimilar(navHistory[i], leftCm, frontCm, rightCm)) {
      similarCount++;
    }
  }
  return similarCount >= TRAP_REPEAT_THRESHOLD;
}

// Build compact CSV-style summary of recent plans for telemetry/prompt context.
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

// Build compact textual summary of hazard snapshots for telemetry/prompt context.
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

// Measure front clearance trend across history window (positive is improving).
float frontTrendCm() {
  if (navHistoryCount < 2) {
    return 0.0f;
  }
  uint8_t newest = (navHistoryWriteIndex + NAV_HISTORY_SIZE - 1) % NAV_HISTORY_SIZE;
  uint8_t oldest = (navHistoryCount == NAV_HISTORY_SIZE) ? navHistoryWriteIndex : 0;
  return navHistory[newest].frontCm - navHistory[oldest].frontCm;
}

// Count directional flip-flops that suggest left/right oscillation.
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

// Convert numeric plan outcome into a diagnostic string.
const char *lastPlanOutcomeString() {
  if (lastPlanOutcome > 0) {
    return "improved_clearance";
  }
  if (lastPlanOutcome < 0) {
    return "worsened_or_no_gain";
  }
  return "unknown";
}

// Compute coarse risk score from front and side effective clearances.
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

// Baseline plan object used as a safe initialization for specialized builders.
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

// Build and score local candidate plans; pick the highest-scoring option.
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

// Extract the first balanced JSON object from model free-form text.
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

// Recovery parser for truncated JSON by cutting at the reason field boundary.
bool extractTruncatedPlanPrefix(const String &input, String &jsonText) {
  int start = input.indexOf('{');
  if (start < 0) {
    return false;
  }
  int reasonIdx = input.indexOf(",\"reason\"", start);
  if (reasonIdx > start) {
    jsonText = input.substring(start, reasonIdx) + "}";
    return true;
  }
  return false;
}

// Parse model JSON into ManeuverPlan with bounds checking and fallback defaults.
bool parsePlanFromJsonText(const String &modelText, ManeuverPlan &plan) {
  String jsonText;
  if (!extractFirstJsonObject(modelText, jsonText)) {
    if (!extractTruncatedPlanPrefix(modelText, jsonText)) {
      return false;
    }
    Serial.println("Plan JSON truncated; using prefix without reason field");
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

// Direction helpers used by disagreement guardrails.
bool isPlanDirectionLeft(ManeuverType maneuver) {
  return maneuver == MANEUVER_STRAFE_LEFT || maneuver == MANEUVER_TURN_LEFT_90;
}
bool isPlanDirectionRight(ManeuverType maneuver) {
  return maneuver == MANEUVER_STRAFE_RIGHT || maneuver == MANEUVER_TURN_RIGHT_90;
}
bool plansDisagreeDirection(const ManeuverPlan &a, const ManeuverPlan &b) {
  bool aLeft = isPlanDirectionLeft(a.primary);
  bool aRight = isPlanDirectionRight(a.primary);
  bool bLeft = isPlanDirectionLeft(b.primary);
  bool bRight = isPlanDirectionRight(b.primary);
  return (aLeft && bRight) || (aRight && bLeft);
}

// Extract model text from various Foundry response envelope shapes.
bool extractFoundryModelText(const JsonDocument &responseDoc, String &modelText) {
  modelText = "";
  JsonVariantConst outputTextVar = responseDoc["output_text"];
  if (outputTextVar.is<const char *>()) {
    const char *text = outputTextVar.as<const char *>();
    if (text != nullptr && strlen(text) > 0) {
      modelText = text;
      return true;
    }
  }
  JsonArrayConst outputArray = responseDoc["output"].as<JsonArrayConst>();
  for (JsonVariantConst outputItem : outputArray) {
    const char *itemType = outputItem["type"] | "";
    bool isMessage = strcmp(itemType, "message") == 0;
    JsonArrayConst contentArray = outputItem["content"].as<JsonArrayConst>();
    for (JsonVariantConst contentItem : contentArray) {
      const char *contentType = contentItem["type"] | "";
      const char *text = contentItem["text"] | nullptr;
      if (text != nullptr && strlen(text) > 0 && (isMessage || strcmp(contentType, "output_text") == 0)) {
        modelText += text;
      }
      const char *textValue = contentItem["text"]["value"] | nullptr;
      if (textValue != nullptr && strlen(textValue) > 0) {
        if (modelText.length() > 0) {
          modelText += "\n";
        }
        modelText += textValue;
      }
      const char *outputText = contentItem["output_text"] | nullptr;
      if (outputText != nullptr && strlen(outputText) > 0) {
        if (modelText.length() > 0) {
          modelText += "\n";
        }
        modelText += outputText;
      }
    }
  }
  if (modelText.length() > 0) {
    return true;
  }
  const char *choicesText = responseDoc["choices"][0]["message"]["content"] | nullptr;
  if (choicesText != nullptr && strlen(choicesText) > 0) {
    modelText = choicesText;
    return true;
  }
  return false;
}

// Lightweight raw-string fallback extraction when JSON shape is unexpected.
bool extractFoundryModelTextFromRawResponse(const String &responseBody, String &modelText) {
  modelText = "";
  int typeCursor = responseBody.indexOf("\"type\"");
  while (typeCursor >= 0) {
    int typeValue = responseBody.indexOf("output_text", typeCursor);
    if (typeValue < 0 || (typeValue - typeCursor) > 120) {
      typeCursor = responseBody.indexOf("\"type\"", typeCursor + 1);
      continue;
    }
    int textKey = responseBody.indexOf("\"text\"", typeValue);
    if (textKey < 0) {
      return false;
    }
    int colon = responseBody.indexOf(':', textKey);
    int quoteStart = responseBody.indexOf('"', colon + 1);
    if (colon < 0 || quoteStart < 0) {
      return false;
    }
    bool escape = false;
    for (int i = quoteStart + 1; i < responseBody.length(); ++i) {
      char ch = responseBody.charAt(i);
      if (escape) {
        switch (ch) {
          case '"':
            modelText += '"';
            break;
          case '\\':
            modelText += '\\';
            break;
          case 'n':
            modelText += '\n';
            break;
          case 'r':
            modelText += '\r';
            break;
          case 't':
            modelText += '\t';
            break;
          default:
            modelText += ch;
            break;
        }
        escape = false;
        continue;
      }
      if (ch == '\\') {
        escape = true;
        continue;
      }
      if (ch == '"') {
        return modelText.length() > 0;
      }
      modelText += ch;
    }
    return modelText.length() > 0;
  }
  return false;
}

// Enforce safety/confidence constraints before accepting model-selected plan.
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

// Ask Foundry for maneuver arbitration; always degrade safely to local fallback plan.
ManeuverPlan queryFoundryForNavigationPlan(const ManeuverPlan &fallbackPlan, bool repeatedTrap) {
  foundryRequestSent = false;
  foundryResponseOk = false;
  foundryPlanParsed = false;
  foundryDecisionStatus = "init";
  flashFoundryThinkingLeds();
  if (WiFi.status() != WL_CONNECTED) {
    foundryDecisionStatus = "wifi_disconnected_local_fallback";
    Serial.println("Foundry skipped: WiFi disconnected, using local fallback plan");
    return fallbackPlan;
  }
  HTTPClient http;
  if (!http.begin(FOUNDRY_RESPONSES_URL)) {
    foundryDecisionStatus = "http_begin_failed_local_fallback";
    Serial.println("Foundry: HTTP begin failed");
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
      "Return strict JSON only, no markdown, with keys primary, primary_duration_ms, secondary, secondary_duration_ms, confidence, risk. "
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
      ". Example JSON: {\"primary\":\"BACKWARD\",\"primary_duration_ms\":350,\"secondary\":\"TURN_LEFT_90\",\"secondary_duration_ms\":520,\"confidence\":0.82,\"risk\":0.74}.";
  JsonDocument requestDoc;
  requestDoc["model"] = FOUNDRY_MODEL;
  requestDoc["input"] = prompt;
  requestDoc["max_output_tokens"] = FOUNDRY_MAX_OUTPUT_TOKENS;
  requestDoc["reasoning"]["effort"] = "minimal";
  requestDoc["text"]["format"]["type"] = "json_object";
  requestDoc["text"]["verbosity"] = "low";
  String body;
  serializeJson(requestDoc, body);
  requestDoc["max_output_tokens"] = FOUNDRY_RETRY_OUTPUT_TOKENS;
  String retryBody;
  serializeJson(requestDoc, retryBody);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("api-key", FOUNDRY_API_KEY);
  http.setTimeout(FOUNDRY_HTTP_TIMEOUT_TIGHT_MS);
  foundryRequestSent = true;
  foundryDecisionStatus = "request_sent";
  Serial.println("Foundry request sent");
  unsigned long requestStartedMs = millis();
  int statusCode = http.POST(body);
  String responseBody = http.getString();
  if (statusCode == HTTPC_ERROR_READ_TIMEOUT) {
    Serial.println("Foundry read timeout, retrying once with longer timeout and smaller output budget");
    http.setTimeout(FOUNDRY_HTTP_TIMEOUT_RETRY_MS);
    foundryDecisionStatus = "request_timeout_retry";
    statusCode = http.POST(retryBody);
    responseBody = http.getString();
  }
  Serial.print("Foundry round-trip ms: ");
  Serial.println(millis() - requestStartedMs);
  http.end();
  if (statusCode < 200 || statusCode >= 300) {
    foundryDecisionStatus = String("http_error_") + String(statusCode) + "_local_fallback";
    Serial.print("Foundry HTTP error: ");
    Serial.println(statusCode);
    Serial.print("Foundry error body: ");
    Serial.println(responseBody);
    return fallbackPlan;
  }
  foundryResponseOk = true;
  JsonDocument responseDoc;
  DeserializationError err = deserializeJson(responseDoc, responseBody);
  if (err) {
    foundryDecisionStatus = "response_json_parse_error_local_fallback";
    Serial.print("Foundry response parse error: ");
    Serial.println(err.c_str());
    return fallbackPlan;
  }
  String modelText;
  if (!extractFoundryModelText(responseDoc, modelText)) {
    if (!extractFoundryModelTextFromRawResponse(responseBody, modelText)) {
      foundryDecisionStatus = "no_model_text_local_fallback";
      Serial.println("Foundry response had no extractable model text, using local fallback plan");
      Serial.print("Foundry raw response body: ");
      Serial.println(responseBody);
      return fallbackPlan;
    }
    Serial.println("Recovered model text from raw response fallback");
  }
  Serial.print("Foundry raw text: ");
  Serial.println(modelText);
  ManeuverPlan plan = fallbackPlan;
  if (!parsePlanFromJsonText(modelText, plan)) {
    foundryDecisionStatus = "plan_json_parse_error_local_fallback";
    Serial.println("Foundry plan parse failed, using local fallback plan");
    return fallbackPlan;
  }
  foundryPlanParsed = true;
  plan.repeatedTrap = repeatedTrap;
  sanitizePlan(plan, fallbackPlan);
  foundryDecisionStatus = "plan_parsed_and_applied";
  flashDecisionDirectionLed(plan.primary);
  Serial.print("Foundry plan: ");
  Serial.print(maneuverTypeToString(plan.primary));
  Serial.print(" then ");
  Serial.print(maneuverTypeToString(plan.secondary));
  Serial.print(" conf=");
  Serial.println(plan.confidence, 2);
  return plan;
}

// Center ultrasonic pan servo so forward readings align with heading.
void movePanToCenter() {
  if (!panServoReady) {
    return;
  }
  panCurrentDeg = PAN_CENTER_DEG;
  panServo.write(panCurrentDeg);
}

// Perform a deliberate left-right-front snapshot before making a hazard decision.
void refreshHazardScanSnapshot() {
  if (!panServoReady) {
    float front = sensor.Ranging();
    if (isValidDistance(front)) {
      updateFrontDistanceEstimate(front, millis(), true);
    } else {
      frontDistanceCm = -1.0f;
      frontFilteredCm = -1.0f;
      frontFilterReady = false;
    }
    return;
  }
  panServo.write(PAN_LEFT_DEG);
  panCurrentDeg = PAN_LEFT_DEG;
  delay(PAN_SETTLE_MS);
  float left = sensor.Ranging();
  if (isValidDistance(left)) {
    leftDistanceCm = left;
  } else {
    leftDistanceCm = -1.0f;
  }
  panServo.write(PAN_RIGHT_DEG);
  panCurrentDeg = PAN_RIGHT_DEG;
  delay(PAN_SETTLE_MS);
  float right = sensor.Ranging();
  if (isValidDistance(right)) {
    rightDistanceCm = right;
  } else {
    rightDistanceCm = -1.0f;
  }
  movePanToCenter();
  delay(PAN_SETTLE_MS);
  float front = sensor.Ranging();
  if (isValidDistance(front)) {
    updateFrontDistanceEstimate(front, millis(), true);
  } else {
    frontDistanceCm = -1.0f;
    frontFilteredCm = -1.0f;
    frontFilterReady = false;
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

// Continuous sweep movement for the pan servo between configured left/right limits.
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

// Periodic sensor update with stale/front-blind handling and hazard hysteresis.
void updateScanAndHazard() {
  unsigned long nowMs = millis();
  if (nowMs - lastSensorMs < SENSOR_INTERVAL_MS) {
    return;
  }
  lastSensorMs = nowMs;
  float distanceCm = sensor.Ranging();
  if (isValidDistance(distanceCm)) {
    updateFrontDistanceEstimate(distanceCm, nowMs, false);
  }
  bool frontFresh = (frontUpdatedMs != 0 && (nowMs - frontUpdatedMs) <= FRONT_STALE_MS);
  if (!frontFresh) {
    // One immediate retry reduces false "hazard" triggers from occasional echo dropouts.
    float retryFrontCm = sensor.Ranging();
    if (isValidDistance(retryFrontCm)) {
      updateFrontDistanceEstimate(retryFrontCm, nowMs, false);
      frontFresh = true;
    }
  }
  if (frontFresh && isValidDistance(frontDistanceCm)) {
    frontBlindStreak = 0;
    if (obstacleNearby) {
      obstacleNearby = (frontDistanceCm <= ULTRASONIC_CLEAR_CM);
    } else {
      obstacleNearby = (frontDistanceCm <= ULTRASONIC_ALERT_CM);
    }
  } else {
    if (frontBlindStreak < 255) {
      frontBlindStreak++;
    }
    obstacleNearby = (frontBlindStreak >= FRONT_BLIND_STREAK_THRESHOLD);
  }
  if (allDistancesUnknown()) {
    if (allUnknownStreak < 255) {
      allUnknownStreak++;
    }
  } else {
    allUnknownStreak = 0;
  }
  if (allUnknownStreak >= ALL_UNKNOWN_STREAK_THRESHOLD) {
    obstacleNearby = true;
  }
  Serial.print("Scan L/F/R: ");
  Serial.print(leftDistanceCm, 1);
  Serial.print("/");
  Serial.print(frontDistanceCm, 1);
  Serial.print("/");
  Serial.print(rightDistanceCm, 1);
  Serial.print(" cm | hazard: ");
  Serial.println(obstacleNearby ? "YES" : "NO");
  if (allUnknownStreak >= ALL_UNKNOWN_STREAK_THRESHOLD) {
    Serial.println("Sensor blind state detected: forcing decision cycle");
  }
}

// Escalation logic: multiple hazard events in a short burst force recovery behavior.
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

// Lower maneuver speed under high-risk or low-confidence conditions.
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

// Execute a single maneuver primitive, then ensure vehicle stops at segment end.
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

// Execute primary and optional secondary maneuvers; update directional memory.
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

// Initialize serial, GPIO, drive train, ultrasonic sensor, servo, and Wi-Fi.
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
  Serial.println("Azure AI Foundry-assisted navigation planner enabled");
}

// Main runtime loop: sense -> detect hazard -> plan -> execute -> learn outcome.
void loop() {
  updateScanAndHazard();
  unsigned long nowMs = millis();
  if (obstacleNearby && (!previousObstacleNearby || (nowMs - lastHazardDecisionMs) >= HAZARD_DECISION_COOLDOWN_MS)) {
    myCar.Move(Stop, 0);
    Serial.println("Hazard detected: STOP -> SCAN -> DECIDE");
    delay(DECISION_STOP_PAUSE_MS);

    // Capture a focused scan and add it to rolling trap/history telemetry.
    refreshHazardScanSnapshot();
    recordHazardSnapshot(leftDistanceCm, frontDistanceCm, rightDistanceCm, nowMs);
    ManeuverPlan plan;
    String decisionSource;
    bool repeatedTrap = detectRepeatedTrap(leftDistanceCm, frontDistanceCm, rightDistanceCm);
    if (allDistancesUnknown()) {
      // Sensor-blind mode: avoid committed movement, request rescan first.
      plan = defaultPlan();
      plan.primary = MANEUVER_RESCAN;
      plan.primaryDurationMs = RESCAN_PAUSE_MS + 80;
      plan.secondary = MANEUVER_STOP;
      plan.secondaryDurationMs = 0;
      plan.confidence = 1.0f;
      decisionSource = "local_sensor_blind";
      Serial.println("Sensor blind state: local forced rescan plan");
    } else if (isHeadOnWall(frontDistanceCm, leftDistanceCm, rightDistanceCm)) {
      // Symmetric close side walls + close front implies head-on wall escape.
      plan = defaultPlan();
      plan.primary = MANEUVER_BACKWARD;
      plan.primaryDurationMs = BACKUP_DURATION_MS;
      plan.secondary = (chooseFallbackTurn() == ACTION_LEFT) ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
      plan.secondaryDurationMs = TURN_90_DURATION_MS;
      plan.confidence = 1.0f;
      decisionSource = "local_head_on_wall";
      Serial.println("Head-on wall escape: local forced plan");
    } else if (isValidDistance(frontDistanceCm) && frontDistanceCm <= EMERGENCY_REVERSE_CM) {
      // Immediate hard safety override when obstacle is critically close.
      plan = defaultPlan();
      plan.primary = MANEUVER_BACKWARD;
      plan.primaryDurationMs = BACKUP_DURATION_MS;
      plan.secondary = (chooseFallbackTurn() == ACTION_LEFT) ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
      plan.secondaryDurationMs = TURN_90_DURATION_MS;
      plan.confidence = 1.0f;
      decisionSource = "local_emergency_close";
      Serial.println("Emergency close obstacle: local forced plan");
    } else if (shouldForceBackward(nowMs)) {
      // Repeated hazards in burst window trigger deterministic recovery.
      plan = defaultPlan();
      plan.primary = MANEUVER_BACKWARD;
      plan.primaryDurationMs = BACKUP_DURATION_MS;
      plan.secondary = (chooseFallbackTurn() == ACTION_LEFT) ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
      plan.secondaryDurationMs = TURN_90_DURATION_MS;
      plan.confidence = 1.0f;
      decisionSource = "local_burst_escalation";
      Serial.println("Burst hazard escalation: local forced plan");
    } else {
      // Normal decision path: local candidate + cloud arbitration.
      ManeuverPlan localPlan = chooseLocalPlan(repeatedTrap);
      Serial.println("Decision path: local candidate + Foundry arbitration");
      plan = queryFoundryForNavigationPlan(localPlan, repeatedTrap);
      decisionSource = foundryDecisionStatus;
      Serial.print("Decision telemetry: status=");
      Serial.print(foundryDecisionStatus);
      Serial.print(", requestSent=");
      Serial.print(foundryRequestSent ? "true" : "false");
      Serial.print(", responseOk=");
      Serial.print(foundryResponseOk ? "true" : "false");
      Serial.print(", planParsed=");
      Serial.println(foundryPlanParsed ? "true" : "false");
    }
    Serial.print("Executing plan source=");
    Serial.print(decisionSource);
    Serial.print(" primary=");
    Serial.print(maneuverTypeToString(plan.primary));
    Serial.print(" secondary=");
    Serial.println(maneuverTypeToString(plan.secondary));
    lastPlanFrontBeforeCm = frontDistanceCm;
    executePlan(plan);

    // Re-scan and score whether this plan materially improved front clearance.
    refreshHazardScanSnapshot();
    lastPlanFrontAfterCm = frontDistanceCm;
    if (isValidDistance(lastPlanFrontBeforeCm) && isValidDistance(lastPlanFrontAfterCm)) {
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

  // Reset burst-escalation window after sustained clear path.
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

  // Cruise behavior outside hazard mode.
  if (obstacleNearby) {
    myCar.Move(Stop, 0);
  } else {
    myCar.Move(Forward, FORWARD_SPEED);
  }
  delay(20);
}
