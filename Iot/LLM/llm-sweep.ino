// ESP32 rover navigation sketch.
//
// High-level behavior:
// - Continuously sweep a forward-facing ultrasonic sensor across left/front/right.
// - Maintain short-lived "radar" buckets for those directions.
// - Drive forward when the path is clear.
// - Stop immediately when hazards are detected and choose a bounded escape maneuver.
// - Optionally ask Gemini for a turn decision only when the local data is fresh but ambiguous.
// - Fall back to fully local safety logic whenever Wi-Fi, sensor quality, or response timing is not good enough.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
#include <esp_heap_caps.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ultrasonic.h>
#include <vehicle.h>
#include "gemini_config.h"
#include "wifi_config.h"

// Hardware abstractions for the ultrasonic sensor, motor chassis, and panning servo.
ultrasonic sensor;
vehicle robot;
Servo ultrasonicPanServo;

// Actions are the user-intent layer: what the navigation logic wants the rover to do next.
enum Action {
  ACTION_STOP = 0,
  ACTION_FORWARD,
  ACTION_BACKWARD,
  ACTION_LEFT,
  ACTION_RIGHT,
};

// Drive modes are the motor-control layer: how the chassis is currently being driven.
// These are more specific than Action because a turn can be a pivot or a steering arc.
enum DriveMode {
  DRIVE_STOPPED = 0,
  DRIVE_FORWARD,
  DRIVE_REVERSE,
  DRIVE_ARC_LEFT,
  DRIVE_ARC_RIGHT,
  DRIVE_PIVOT_LEFT,
  DRIVE_PIVOT_RIGHT,
};

// Snapshot of the latest left/right scan so escape logic can compare which side is more open.
struct SideScanResult {
  float leftDistanceCm;
  float rightDistanceCm;
  Action bestAction;
};

// Aggregated sensor state used by both local recovery logic and Gemini prompts.
// The "age" fields let the code reason about stale samples instead of trusting old data.
struct NavigationSnapshot {
  float frontCm;
  float leftCm;
  float rightCm;
  unsigned long frontAgeMs;
  unsigned long leftAgeMs;
  unsigned long rightAgeMs;
  bool frontValid;
  bool leftValid;
  bool rightValid;
  bool frontBlocked;
  int confidence;
};

// Small whitelist of actions that are currently considered safe enough to execute.
struct SafeActionSet {
  bool forward;
  bool backward;
  bool left;
  bool right;
  bool stop;
};

// Prompt variants let Gemini know whether this is a first obstacle, a failed maneuver,
// or a near-trapped recovery case.
enum GeminiPromptTemplate {
  GEMINI_PROMPT_STANDARD_OBSTACLE = 0,
  GEMINI_PROMPT_FAILED_MANEUVER,
  GEMINI_PROMPT_TRAPPED_RECOVERY,
};

// One directional ultrasonic sample, annotated with servo angle, capture time, and filtering confidence.
struct DirectionalReading {
  float distanceCm;
  uint8_t angleDeg;
  unsigned long capturedMs;
  uint8_t sampleConfidence;
  bool valid;
};

// Historical record of a maneuver so the rover can penalize patterns that did not improve clearance.
struct ManeuverOutcome {
  Action action;
  float frontBeforeCm;
  float frontAfterCm;
  float improvementCm;
  float turnSequenceStartCm;
  uint8_t pulsesUsed;
  bool improved;
  unsigned long completedMs;
};

// Parsed Gemini response. The rover accepts only a strict one-field JSON object.
struct ParsedAction {
  bool valid;
  Action action;
};

// Main rover state machine.
// - ROAMING: normal forward travel.
// - HAZARD: obstacle is near; hold position until a decision can be made.
// - DECIDING: choosing the next escape action locally or via Gemini.
// - MANEUVERING: executing a bounded turn/reverse pulse.
// - REVERSE_RESCAN: reverse finished; do a fresh scan before resuming.
enum RoverState {
  STATE_ROAMING = 0,
  STATE_HAZARD,
  STATE_DECIDING,
  STATE_MANEUVERING,
  STATE_REVERSE_RESCAN,
};

// Async Gemini request lifecycle. The main loop never blocks on a network request.
enum GeminiDecisionRequestState {
  GEMINI_REQUEST_IDLE = 0,
  GEMINI_REQUEST_PENDING,
  GEMINI_REQUEST_TIMED_OUT,
  GEMINI_REQUEST_READY,
  GEMINI_REQUEST_FAILED,
};

// Parameters copied into the background Gemini worker task.
struct GeminiDecisionRequestArgs {
  NavigationSnapshot snapshot;
  Action previousAction;
  SideScanResult scan;
  GeminiPromptTemplate templateType;
  uint32_t generation;
};

// Result pushed back from the worker to the main loop through a single-slot queue.
struct GeminiDecisionResultMessage {
  uint32_t generation;
  Action decision;
};
const uint8_t TRIG_PIN = 13;
const uint8_t ECHO_PIN = 14;
const uint8_t BUZZER_PIN = 33;
const bool BUZZER_ACTIVE_HIGH = true;
const uint8_t ULTRASONIC_PAN_PIN = 27;
const uint8_t PAN_FORWARD_DEG = 90;
const uint8_t PAN_LEFT_DEG = 180;
const uint8_t PAN_RIGHT_DEG = 0;
const uint8_t PAN_FRONT_WINDOW_DEG = 24;
const unsigned long PAN_FRONT_SETTLE_MS = 90;
const uint8_t ULTRASONIC_NO_ECHO_RETRIES = 2;
const unsigned long ULTRASONIC_NO_ECHO_RETRY_DELAY_MS = 40;
const unsigned long ULTRASONIC_SAMPLE_GAP_MS = 45;
const uint16_t ULTRASONIC_NO_ECHO_RECOVERY_STREAK = 6;
const unsigned long ULTRASONIC_NO_ECHO_RECOVERY_HOLD_MS = 1200;
const float ULTRASONIC_ALERT_CM = 30.0f;
const float ULTRASONIC_MIN_VALID_CM = 4.0f;
const float ULTRASONIC_EMERGENCY_CLOSE_CM = 12.0f;
const uint8_t HAZARD_ENTER_SAMPLES = 2;
const uint8_t HAZARD_CLEAR_SAMPLES = 3;
const float HAZARD_ENTER_CM = 30.0f;
const float HAZARD_CLEAR_CM = 36.0f;
const int BASE_SPEED = 128;
const float LEFT_TRIM = 0.66f;
const float RIGHT_TRIM = 1.10f;
const unsigned long SENSOR_INTERVAL_MS = 70;
const unsigned long OBSTACLE_DECISION_COOLDOWN_MS = 700;
const uint8_t ACTION_HISTORY_SIZE = 8;
const float STEER_OUTER_SCALE = 1.00f;
const float STEER_INNER_SCALE = 0.18f;
const int MIN_DRIVE_RIGHT_PWM = 155;
const int MIN_DRIVE_LEFT_PWM = 118;
const int MIN_REVERSE_RIGHT_PWM = 175;
const int MIN_REVERSE_LEFT_PWM = 170;
const int MIN_STEER_OUTER_PWM = 112;
const int MIN_STEER_INNER_PWM = 52;
const uint8_t HAZARD_CONFIRM_SAMPLES = HAZARD_ENTER_SAMPLES;
const unsigned long FORWARD_RESTART_BOOST_MS = 220;
const int FORWARD_BOOST_RIGHT_PWM = 176;
const int FORWARD_BOOST_LEFT_PWM = 156;
const unsigned long STARTUP_MOTOR_STABILIZE_MS = 250;
const unsigned long EMERGENCY_CLOSE_REVERSE_DURATION_MS = 1650;
const unsigned long EMERGENCY_CLOSE_REVERSE_COOLDOWN_MS = 1400;
const unsigned long EMERGENCY_REVERSE_DURATION_MS = 980;
const float TTC_BRAKE_MIN_CLOSING_CMPS = 18.0f;
const float TTC_BRAKE_MAX_FRONT_CM = 45.0f;
const unsigned long TTC_BRAKE_COOLDOWN_MS = 900;
const unsigned long TTC_MAX_SAMPLE_GAP_MS = 350;
const uint8_t TTC_CLOSE_CONFIRM_SAMPLES = 2;
const float TTC_CLOSE_TOLERANCE_CMPS = 7.0f;
const unsigned long STOPPING_MOTOR_DELAY_MS = 180;
const float STOPPING_MODEL_DECEL_CMPS2 = 180.0f;
const float STOPPING_MODEL_MARGIN_CM = 12.0f;
const int ADAPTIVE_FORWARD_MIN_BASE_SPEED = 98;
const uint16_t GEMINI_HTTP_TIMEOUT_MS = 2500;
const uint16_t GEMINI_MAX_OUTPUT_TOKENS = 12;
const unsigned long GEMINI_DECISION_REQUEST_TIMEOUT_MS = 4000;
const unsigned long MOTOR_CONTROL_HEARTBEAT_TIMEOUT_MS = 1000;
const unsigned long RADAR_SIDE_STALE_MS = 1200;
const unsigned long TURN_PULSE_MS = 180;
const uint8_t TURN_MAX_PULSES = 4;
const unsigned long MAX_ESCAPE_TURN_TOTAL_MS = 1600;
const float TURN_CLEARANCE_IMPROVEMENT_CM = 4.0f;
const uint8_t ULTRASONIC_MEDIAN_SAMPLE_COUNT = 3;
const uint8_t ULTRASONIC_MEDIAN_MAX_ATTEMPTS = 5;
const int ACTION_SCORE_BLOCKED_PENALTY = 10000;
const int ACTION_SCORE_OSCILLATION_PENALTY = 30;
const int ACTION_SCORE_RECENT_REPEAT_PENALTY = 12;
const int ACTION_SCORE_IMMEDIATE_REPEAT_PENALTY = 24;
const int ACTION_SCORE_DOUBLE_REPEAT_PENALTY = 60;
const int ACTION_SCORE_ABAB_PENALTY = 55;
const int ACTION_SCORE_FAILED_OUTCOME_PENALTY = 45;
const bool SERVO_SWEEP_DEBUG_ONLY = false;

// Servo scan state machine for collecting left/front/right samples without blocking the main loop.
enum ScanState {
  SCAN_IDLE,
  SCAN_MOVE_LEFT,
  SCAN_SETTLE_LEFT,
  SCAN_SAMPLE_LEFT,
  SCAN_MOVE_FRONT,
  SCAN_SETTLE_FRONT,
  SCAN_SAMPLE_FRONT,
  SCAN_MOVE_RIGHT,
  SCAN_SETTLE_RIGHT,
  SCAN_SAMPLE_RIGHT,
  SCAN_TURN_MOVE_FRONT,
  SCAN_TURN_SETTLE_FRONT,
  SCAN_TURN_SAMPLE_FRONT,
  SCAN_COMPLETE,
};

// Ordered escape strategy. Each failure advances to a more conservative recovery stage.
enum EscapeAttemptStage {
  ESCAPE_ATTEMPT_CLEARER_SIDE = 1,
  ESCAPE_ATTEMPT_CONTINUE_FARTHER,
  ESCAPE_ATTEMPT_OPPOSITE_SIDE,
  ESCAPE_ATTEMPT_SHORT_REVERSE,
  ESCAPE_ATTEMPT_TRAPPED,
};

// Runtime state: timing, hazard tracking, maneuver state, and rolling histories.
unsigned long lastSensorMs = 0;
unsigned long lastObstacleDecisionMs = 0;
unsigned long lastValidMotorCommandMs = 0;
unsigned long lastControlHeartbeatMs = 0;
float lastDistanceCm = -1.0f;
float currentFrontDistanceCm = -1.0f;
float previousFrontDistanceCm = -1.0f;
bool obstacleNearby = false;
uint8_t hazardSampleCount = 0;
unsigned long lastEmergencyCloseReverseMs = 0;
bool hazardClearArmed = true;
Action lastAction = ACTION_STOP;
Action requestedAction = ACTION_STOP;
Action currentMotorAction = ACTION_STOP;
DriveMode currentDriveMode = DRIVE_STOPPED;
Action actionHistory[ACTION_HISTORY_SIZE];
uint8_t actionHistoryCount = 0;
uint8_t actionHistoryHead = 0;
RoverState roverState = STATE_ROAMING;
Action maneuverResumeAction = ACTION_FORWARD;
unsigned long maneuverUntilMs = 0;
bool maneuverUsesSteering = false;
Action activeManeuverAction = ACTION_STOP;
bool buzzerActive = false;
unsigned long buzzerOffMs = 0;
bool forwardRestartBoostActive = false;
unsigned long forwardRestartBoostUntilMs = 0;
bool panServoReady = false;
uint8_t panCurrentDeg = PAN_FORWARD_DEG;
bool panSweepTowardLeft = true;
unsigned long lastPanStepMs = 0;
volatile bool panSweepEnabled = true;
unsigned long panSweepRecoveryUntilMs = 0;
ScanState ultrasonicScanState = SCAN_IDLE;
unsigned long ultrasonicScanStateMs = 0;
DirectionalReading ultrasonicScanSamples[ULTRASONIC_MEDIAN_SAMPLE_COUNT];
uint8_t ultrasonicScanValidCount = 0;
uint8_t ultrasonicScanAttemptCount = 0;
bool turnCheckPending = false;
bool turnFrontReadingReady = false;
float turnSequenceStartCm = -1.0f;
unsigned long turnSequenceStartedMs = 0;
float turnLatestFrontCm = -1.0f;
uint8_t turnPulsesExecuted = 0;
bool turnReverseDrive = false;
EscapeAttemptStage escapeAttemptStage = ESCAPE_ATTEMPT_CLEARER_SIDE;
Action escapePrimaryAction = ACTION_STOP;
bool escapeProgressResetPending = false;
float escapeProgressReferenceFrontCm = -1.0f;
bool escapeTrapSignaled = false;
uint16_t ultrasonicNoEchoStreak = 0;
int adaptiveForwardBaseSpeed = BASE_SPEED;
int lastAppliedForwardBaseSpeed = -1;
bool ttcSampleValid = false;
float lastTtcFrontCm = -1.0f;
unsigned long lastTtcMs = 0;
unsigned long lastTtcBrakeMs = 0;
bool ttcClosingCorroborated = false;
float lastTtcClosingCmPerSec = -1.0f;
float corroboratedTtcClosingCmPerSec = -1.0f;
uint8_t ttcClosingConfirmCount = 0;
bool motionCalibrationActive = false;
unsigned long motionCalibrationStopCommandMs = 0;
float motionCalibrationStopDistanceCm = -1.0f;
float motionCalibrationMinDistanceAfterStopCm = -1.0f;
float motionCalibrationClosingSpeedCmPerSec = -1.0f;
int lastForwardCommandBaseSpeed = 0;
int lastForwardCommandRightPwm = 0;
int lastForwardCommandLeftPwm = 0;
bool sensorRetryActive = false;
bool sensorRetryPreviousObstacle = false;
uint8_t sensorRetryAttempt = 0;
unsigned long sensorRetryDueMs = 0;
unsigned long lastUltrasonicSampleMs = 0;
bool startupFrontScanPending = false;
GeminiDecisionRequestState geminiDecisionRequestState = GEMINI_REQUEST_IDLE;
TaskHandle_t geminiDecisionTaskHandle = nullptr;
uint32_t geminiRequestGeneration = 0;
Action geminiDecisionFallbackAction = ACTION_STOP;
unsigned long geminiDecisionRequestStartedMs = 0;
QueueHandle_t geminiDecisionResultQueue = nullptr;
float radarFrontCm = -1.0f;
float radarLeftCm = -1.0f;
float radarRightCm = -1.0f;
unsigned long radarFrontUpdatedMs = 0;
unsigned long radarLeftUpdatedMs = 0;
unsigned long radarRightUpdatedMs = 0;
uint8_t radarFrontSampleConfidence = 0;
uint8_t radarLeftSampleConfidence = 0;
uint8_t radarRightSampleConfidence = 0;
const unsigned long RADAR_FRONT_STALE_MS = 600;
const uint8_t MANEUVER_OUTCOME_HISTORY_SIZE = 8;
ManeuverOutcome maneuverOutcomeHistory[MANEUVER_OUTCOME_HISTORY_SIZE];
uint8_t maneuverOutcomeCount = 0;
uint8_t maneuverOutcomeHead = 0;

// Forward declarations keep Arduino happy while letting related logic stay grouped below.
float readUltrasonicCm();
DirectionalReading readDirectionalUltrasonic(uint8_t measurementAngle);
DirectionalReading medianOfThreeDirectionalReadings(
  const DirectionalReading &a,
  const DirectionalReading &b,
  const DirectionalReading &c);
bool updateHazardFromDistance(float distanceCm);
NavigationSnapshot buildNavigationSnapshot(unsigned long nowMs);
void updateForwardRestartBoost(unsigned long nowMs);
void setPanAngle(uint8_t angleDeg);
void runPanServoSelfTest();
void updatePanSweepDebug(unsigned long nowMs);
bool updateStationaryUltrasonicSampling(unsigned long nowMs, bool previousObstacle);
void updateRadarBuckets(const DirectionalReading &reading);
SideScanResult getRadarScanSnapshot();
const char *scanBestActionString(const SideScanResult &scan);
const char *hazardClassString(float distanceCm);
const char *environmentHintString(float frontCm, const SideScanResult &scan);
const char *driveModeString(DriveMode mode);
bool driveModeHasForwardMotion(DriveMode mode);
const char *geminiPromptTemplateString(GeminiPromptTemplate templateType);
const char *escapeAttemptStageString(EscapeAttemptStage stage);
int sensorConfidenceScore(unsigned long frontAgeMs, unsigned long leftAgeMs, unsigned long rightAgeMs, uint16_t noEchoStreak);
int computeAdaptiveForwardBaseSpeed(float frontDistanceCm, bool closingRateCorroborated, float corroboratedClosingRateCmPerSec);
void processDirectionalScanReading(const DirectionalReading &reading);
bool processFrontSafetyReading(unsigned long now, bool previousObstacle, const DirectionalReading &reading);
bool handleSensorRetryState(unsigned long now);
void setMotorAction(Action action);
void startManeuver(Action maneuverAction, unsigned long durationMs, Action resumeAction);
float estimateFrontClearanceCm();
void recordManeuverOutcome(Action action, bool improved, float frontBeforeCm, float frontAfterCm,
                           float turnSequenceStartCm, uint8_t pulsesUsed, unsigned long completedMs);
String recentManeuverOutcomesString(uint8_t limit);
Action chooseEscapeActionForStage(const NavigationSnapshot &snapshot, const SideScanResult &scan, Action previousAction);
Action chooseLocalEscapeActionForStage(const NavigationSnapshot &snapshot, const SideScanResult &scan,
                                       Action previousAction);
void noteEscapeOutcome(Action action, bool improved, float frontAfterCm, unsigned long completedMs);
void maybeResetEscapeAttemptStages(unsigned long nowMs);
void signalTrappedState(unsigned long nowMs);
GeminiPromptTemplate geminiPromptTemplateForStage(EscapeAttemptStage stage);
EscapeAttemptStage nextEscapeStage(EscapeAttemptStage stage);
void refreshControlHeartbeat(unsigned long nowMs);
void emergencyMotorStop();
Action queryGeminiForObstacleTurnBlocking(const NavigationSnapshot &snapshot, Action previousAction,
                                          const SideScanResult &scan, GeminiPromptTemplate templateType);
bool startGeminiDecisionRequest(const NavigationSnapshot &snapshot, Action previousAction,
                                const SideScanResult &scan, GeminiPromptTemplate templateType);
bool updateGeminiDecisionRequest(unsigned long nowMs, Action &decisionOut);
void geminiDecisionWorkerTask(void *parameter);

// Clamp every PWM write into the valid 8-bit range expected by the motor driver.
int clampPwm(int pwm) {
  if (pwm < 0) {
    return 0;
  }
  if (pwm > 255) {
    return 255;
  }
  return pwm;
}

// Start a short non-blocking buzzer pulse; loop() turns it off when the deadline passes.
void startBuzzer(uint16_t durationMs) {
  if (durationMs == 0) {
    return;
  }
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_HIGH ? HIGH : LOW);
  buzzerActive = true;
  buzzerOffMs = millis() + durationMs;
}

// Service the buzzer timer so alerts do not stall the navigation loop.
void updateBuzzer(unsigned long nowMs) {
  if (buzzerActive && (long)(nowMs - buzzerOffMs) >= 0) {
    digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_HIGH ? LOW : HIGH);
    buzzerActive = false;
  }
}

// Reset the time-to-collision sequence whenever motion mode changes or sensor continuity is broken.
void resetTtcSequence() {
  ttcSampleValid = false;
  lastTtcFrontCm = -1.0f;
  lastTtcMs = 0;
  ttcClosingCorroborated = false;
  lastTtcClosingCmPerSec = -1.0f;
  corroboratedTtcClosingCmPerSec = -1.0f;
  ttcClosingConfirmCount = 0;
}
void refreshControlHeartbeat(unsigned long nowMs) {
  lastControlHeartbeatMs = nowMs;
}
void emergencyStop(const char *reason) {
  if ((activeManeuverAction == ACTION_LEFT || activeManeuverAction == ACTION_RIGHT) &&
      turnSequenceStartCm >= ULTRASONIC_MIN_VALID_CM) {
    float abortedFrontCm = estimateFrontClearanceCm();
    finalizeTurnSequenceOutcome(false, abortedFrontCm, millis());
  }
  robot.Move(Stop, 0);
  requestedAction = ACTION_STOP;
  currentMotorAction = ACTION_STOP;
  currentDriveMode = DRIVE_STOPPED;
  forwardRestartBoostActive = false;
  maneuverUsesSteering = false;
  maneuverResumeAction = ACTION_STOP;
  maneuverUntilMs = 0;
  activeManeuverAction = ACTION_STOP;
  Serial.print("EMERGENCY STOP: ");
  Serial.println(reason);
}
void printHeapDiagnostics(const char *label) {
  Serial.print("Heap diagnostics (");
  Serial.print(label);
  Serial.println(")");
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("Minimum free heap: ");
  Serial.println(ESP.getMinFreeHeap());
  Serial.print("Largest free block: ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}
void emergencyMotorStop() {
  Serial.print("Motor watchdog: stopping | requested=");
  Serial.print(actionToString(requestedAction));
  Serial.print(" | drive_mode=");
  Serial.println(driveModeString(currentDriveMode));
  emergencyStop("watchdog");
  roverState = STATE_HAZARD;
}
const char *actionToString(Action action) {
  switch (action) {
    case ACTION_FORWARD:
      return "FORWARD";
    case ACTION_BACKWARD:
      return "BACKWARD";
    case ACTION_LEFT:
      return "LEFT";
    case ACTION_RIGHT:
      return "RIGHT";
    default:
      return "STOP";
  }
}
Action oppositeAction(Action action) {
  switch (action) {
    case ACTION_FORWARD:
      return ACTION_BACKWARD;
    case ACTION_BACKWARD:
      return ACTION_FORWARD;
    case ACTION_LEFT:
      return ACTION_RIGHT;
    case ACTION_RIGHT:
      return ACTION_LEFT;
    default:
      return ACTION_STOP;
  }
}
ParsedAction parseActionFromText(const String &text) {
  ParsedAction result = {false, ACTION_STOP};
  String trimmed = text;
  trimmed.trim();
  if (!trimmed.startsWith("{") || !trimmed.endsWith("}")) {
    return result;
  }
  StaticJsonDocument<128> responseDoc;
  DeserializationError err = deserializeJson(responseDoc, trimmed);
  if (err) {
    return result;
  }
  if (responseDoc.size() != 1 || !responseDoc.containsKey("action")) {
    return result;
  }
  if (!responseDoc["action"].is<const char *>()) {
    return result;
  }
  String actionText = responseDoc["action"].as<const char *>();
  actionText.trim();
  if (actionText == "FORWARD") {
    result.valid = true;
    result.action = ACTION_FORWARD;
    return result;
  }
  if (actionText == "BACKWARD") {
    result.valid = true;
    result.action = ACTION_BACKWARD;
    return result;
  }
  if (actionText == "LEFT") {
    result.valid = true;
    result.action = ACTION_LEFT;
    return result;
  }
  if (actionText == "RIGHT") {
    result.valid = true;
    result.action = ACTION_RIGHT;
    return result;
  }
  if (actionText == "STOP") {
    result.valid = true;
    result.action = ACTION_STOP;
    return result;
  }
  return result;
}
Action getRecentAction(uint8_t offsetFromLatest) {
  if (offsetFromLatest >= actionHistoryCount) {
    return ACTION_STOP;
  }
  int index = (int)actionHistoryHead - 1 - (int)offsetFromLatest;
  while (index < 0) {
    index += ACTION_HISTORY_SIZE;
  }
  return actionHistory[index % ACTION_HISTORY_SIZE];
}
void recordAction(Action action) {
  if (action == ACTION_STOP) {
    return;
  }
  actionHistory[actionHistoryHead] = action;
  actionHistoryHead = (actionHistoryHead + 1) % ACTION_HISTORY_SIZE;
  if (actionHistoryCount < ACTION_HISTORY_SIZE) {
    actionHistoryCount++;
  }
}
int countRecent(Action candidate) {
  int count = 0;
  for (uint8_t i = 0; i < actionHistoryCount; i++) {
    if (getRecentAction(i) == candidate) {
      count++;
    }
  }
  return count;
}
bool createsAbabPattern(Action candidate) {
  if (actionHistoryCount < 3) {
    return false;
  }
  Action last0 = getRecentAction(0);
  Action last1 = getRecentAction(1);
  Action last2 = getRecentAction(2);
  return (last2 == last0 && candidate == last1 && candidate != last0);
}
int scoreCandidateAction(Action candidate, Action physicallyBlockedAction, Action discouragedOscillationAction) {
  if (candidate == ACTION_STOP) {
    return 100000;
  }
  int score = 0;
  if (candidate == physicallyBlockedAction) {
    score += ACTION_SCORE_BLOCKED_PENALTY;
  }
  if (candidate == discouragedOscillationAction) {
    score += ACTION_SCORE_OSCILLATION_PENALTY;
  }
  score += countRecent(candidate) * ACTION_SCORE_RECENT_REPEAT_PENALTY;
  if (actionHistoryCount >= 1 && getRecentAction(0) == candidate) {
    score += ACTION_SCORE_IMMEDIATE_REPEAT_PENALTY;
  }
  if (actionHistoryCount >= 2 && getRecentAction(0) == candidate && getRecentAction(1) == candidate) {
    score += ACTION_SCORE_DOUBLE_REPEAT_PENALTY;
  }
  if (createsAbabPattern(candidate)) {
    score += ACTION_SCORE_ABAB_PENALTY;
  }
  for (uint8_t i = 0; i < maneuverOutcomeCount; i++) {
    int index = (int)maneuverOutcomeHead - 1 - (int)i;
    while (index < 0) {
      index += MANEUVER_OUTCOME_HISTORY_SIZE;
    }
    ManeuverOutcome outcome = maneuverOutcomeHistory[index % MANEUVER_OUTCOME_HISTORY_SIZE];
    if (outcome.action == candidate && !outcome.improved) {
      score += ACTION_SCORE_FAILED_OUTCOME_PENALTY;
      break;
    }
  }
  return score;
}
Action chooseBestAction(Action physicallyBlockedAction, Action discouragedOscillationAction) {
  Action options[4] = {ACTION_FORWARD, ACTION_LEFT, ACTION_RIGHT, ACTION_BACKWARD};
  Action best = ACTION_LEFT;
  int bestScore = 100000;
  for (int i = 0; i < 4; i++) {
    Action candidate = options[i];
    int score = scoreCandidateAction(candidate, physicallyBlockedAction, discouragedOscillationAction);
    if (score < bestScore) {
      bestScore = score;
      best = candidate;
    } else if (score == bestScore && random(0, 2) == 0) {
      best = candidate;
    }
  }
  return best;
}
bool reverseRecoveryPermitted(EscapeAttemptStage stage) {
  return stage == ESCAPE_ATTEMPT_SHORT_REVERSE;
}
bool actionIsInSafeSet(Action action, const SafeActionSet &safeActions) {
  switch (action) {
    case ACTION_FORWARD:
      return safeActions.forward;
    case ACTION_BACKWARD:
      return safeActions.backward;
    case ACTION_LEFT:
      return safeActions.left;
    case ACTION_RIGHT:
      return safeActions.right;
    case ACTION_STOP:
      return safeActions.stop;
    default:
      return false;
  }
}
Action chooseBestActionFromSafeSet(const SafeActionSet &safeActions, Action physicallyBlockedAction,
                                   Action discouragedOscillationAction) {
  Action options[4];
  uint8_t optionCount = 0;
  if (safeActions.forward) {
    options[optionCount++] = ACTION_FORWARD;
  }
  if (safeActions.left) {
    options[optionCount++] = ACTION_LEFT;
  }
  if (safeActions.right) {
    options[optionCount++] = ACTION_RIGHT;
  }
  if (safeActions.stop) {
    options[optionCount++] = ACTION_STOP;
  }
  if (optionCount == 0) {
    return ACTION_STOP;
  }
  Action best = options[0];
  int bestScore = 100000;
  for (uint8_t i = 0; i < optionCount; i++) {
    Action candidate = options[i];
    int score = scoreCandidateAction(candidate, physicallyBlockedAction, discouragedOscillationAction);
    if (score < bestScore) {
      bestScore = score;
      best = candidate;
    } else if (score == bestScore && random(0, 2) == 0) {
      best = candidate;
    }
  }
  return best;
}
uint8_t countSafeActions(const SafeActionSet &safeActions) {
  uint8_t count = 0;
  if (safeActions.forward) {
    count++;
  }
  if (safeActions.left) {
    count++;
  }
  if (safeActions.right) {
    count++;
  }
  if (safeActions.stop) {
    count++;
  }
  return count;
}
bool hasClearLocalWinner(const SideScanResult &scan) {
  if (scan.leftDistanceCm < ULTRASONIC_MIN_VALID_CM || scan.rightDistanceCm < ULTRASONIC_MIN_VALID_CM) {
    return false;
  }
  float delta = scan.leftDistanceCm - scan.rightDistanceCm;
  if (delta < 0.0f) {
    delta = -delta;
  }
  return delta >= 10.0f;
}
struct GeminiGateResult {
  bool shouldCall;
  Action localRecommendation;
  const char *reason;
};

// Decide whether the network call is worth making.
// Gemini is only consulted when local sensing is fresh, the rover is safely stopped near a hazard,
// and the scene is ambiguous enough that a left/right choice is not obvious locally.
GeminiGateResult evaluateGeminiGate(const NavigationSnapshot &snapshot, const SideScanResult &scan, Action previousAction,
                  GeminiPromptTemplate templateType) {
  GeminiGateResult gate = {};
  Action physicallyBlockedAction = ACTION_FORWARD;
  Action discouragedOscillationAction = oppositeAction(previousAction);
  SafeActionSet safeActions = buildSafeActionSet(snapshot, scan);
  gate.localRecommendation = chooseBestActionFromSafeSet(safeActions, physicallyBlockedAction, discouragedOscillationAction);
  bool frontValid = snapshot.frontValid;
  bool criticalHazard = frontValid && snapshot.frontCm <= ULTRASONIC_EMERGENCY_CLOSE_CM;
  bool stoppedNearHazard =
      currentMotorAction == ACTION_STOP &&
      frontValid &&
    snapshot.frontCm > ULTRASONIC_EMERGENCY_CLOSE_CM &&
    snapshot.frontCm <= ULTRASONIC_ALERT_CM;
  bool wifiUnavailable = (WiFi.status() != WL_CONNECTED);
  bool lowConfidence = snapshot.confidence < 60;
  bool freshFullScan = snapshot.frontValid && snapshot.leftValid && snapshot.rightValid;
  bool invalidScan = !snapshot.frontValid || !snapshot.leftValid || !snapshot.rightValid;
  bool staleMeasurement = !freshFullScan;
  bool onlyOneSafeAction = (countSafeActions(safeActions) <= 1);
  bool clearLocalWinner = hasClearLocalWinner(scan);
  bool repeatedRecoveryFailure =
      (templateType != GEMINI_PROMPT_STANDARD_OBSTACLE) &&
      (escapeAttemptStage >= ESCAPE_ATTEMPT_CONTINUE_FARTHER);
  bool ambiguousDecision =
      !criticalHazard &&
      (!stoppedNearHazard || freshFullScan) &&
      !wifiUnavailable &&
      !lowConfidence &&
      !staleMeasurement &&
      !onlyOneSafeAction &&
      !clearLocalWinner;
  gate.shouldCall = stoppedNearHazard && freshFullScan &&
                    ambiguousDecision &&
                    (templateType == GEMINI_PROMPT_STANDARD_OBSTACLE || repeatedRecoveryFailure);
  if (!gate.shouldCall) {
    if (wifiUnavailable) {
      gate.reason = "Gemini gate: Wi-Fi unavailable, using local decision";
    } else if (criticalHazard) {
      gate.reason = "Gemini gate: critical hazard, using local recovery only";
    } else if (invalidScan || staleMeasurement) {
      gate.reason = "Gemini gate: stale or invalid scan, using local stop and rescan";
      gate.localRecommendation = ACTION_STOP;
    } else if (lowConfidence) {
      gate.reason = "Gemini gate: low sensor confidence, using local decision";
    } else if (!stoppedNearHazard) {
      gate.reason = "Gemini gate: rover not stopped near hazard, using local recovery only";
    } else if (onlyOneSafeAction) {
      gate.reason = "Gemini gate: only one safe action, using local decision";
    } else if (clearLocalWinner) {
      gate.reason = "Gemini gate: clear local winner, using local decision";
    } else {
      gate.reason = "Gemini gate: recovery not ambiguous enough, using local decision";
    }
  }
  gate.localRecommendation = validateSafeAction(gate.localRecommendation, snapshot, scan);
  return gate;
}
Action chooseSmartAction(Action preferred, Action physicallyBlockedAction, Action discouragedOscillationAction) {
  Action best = chooseBestAction(physicallyBlockedAction, discouragedOscillationAction);
  if (preferred == ACTION_STOP || preferred == physicallyBlockedAction) {
    return best;
  }
  int preferredScore = scoreCandidateAction(preferred, physicallyBlockedAction, discouragedOscillationAction);
  int bestScore = scoreCandidateAction(best, physicallyBlockedAction, discouragedOscillationAction);
  if (preferredScore <= bestScore + 10) {
    return preferred;
  }
  Serial.print("Anti-loop override: ");
  Serial.print(actionToString(preferred));
  Serial.print(" -> ");
  Serial.println(actionToString(best));
  return best;
}
SafeActionSet buildSafeActionSet(const NavigationSnapshot &snapshot, const SideScanResult &scan) {
  SafeActionSet safeActions = {};
  safeActions.forward = snapshot.frontValid && snapshot.frontCm >= HAZARD_CLEAR_CM;
  safeActions.left = snapshot.leftValid && snapshot.leftCm >= HAZARD_CLEAR_CM;
  safeActions.right = snapshot.rightValid && snapshot.rightCm >= HAZARD_CLEAR_CM;
  safeActions.backward = false;
  safeActions.stop = true;
  return safeActions;
}
SafeActionSet buildSafeActionSet(float frontCm, const SideScanResult &scan) {
  NavigationSnapshot snapshot = buildNavigationSnapshot(millis());
  snapshot.frontCm = frontCm;
  snapshot.frontValid = frontCm >= ULTRASONIC_MIN_VALID_CM;
  snapshot.frontBlocked = snapshot.frontValid && frontCm <= ULTRASONIC_ALERT_CM;
  return buildSafeActionSet(snapshot, scan);
}
String safeActionSetString(const SafeActionSet &safeActions) {
  String out = "";
  if (safeActions.forward) {
    out += "FORWARD";
  }
  if (safeActions.left) {
    if (out.length() > 0) {
      out += ",";
    }
    out += "LEFT";
  }
  if (safeActions.right) {
    if (out.length() > 0) {
      out += ",";
    }
    out += "RIGHT";
  }
  if (safeActions.stop) {
    if (out.length() > 0) {
      out += ",";
    }
    out += "STOP";
  }
  return out;
}
Action validateSafeAction(Action proposed, const NavigationSnapshot &snapshot, const SideScanResult &scan) {
  bool frontBlocked = snapshot.frontBlocked;
  if (frontBlocked && proposed == ACTION_FORWARD) {
    Serial.println("Safety override: rejecting FORWARD while front blocked");
    if (scan.bestAction == ACTION_LEFT ||
        scan.bestAction == ACTION_RIGHT) {
      return scan.bestAction;
    }
    return ACTION_BACKWARD;
  }
  return proposed;
}
Action validateSafeAction(Action proposed, float frontCm, const SideScanResult &scan) {
  NavigationSnapshot snapshot = buildNavigationSnapshot(millis());
  snapshot.frontCm = frontCm;
  snapshot.frontValid = frontCm >= ULTRASONIC_MIN_VALID_CM;
  snapshot.frontBlocked = snapshot.frontValid && frontCm <= ULTRASONIC_ALERT_CM;
  return validateSafeAction(proposed, snapshot, scan);
}
String recentActionsString(uint8_t limit) {
  if (actionHistoryCount == 0) {
    return "NONE";
  }
  String out = "";
  uint8_t n = actionHistoryCount < limit ? actionHistoryCount : limit;
  for (uint8_t i = 0; i < n; i++) {
    if (i > 0) {
      out += ",";
    }
    out += actionToString(getRecentAction(i));
  }
  return out;
}
void recordManeuverOutcome(Action action, bool improved, float frontBeforeCm, float frontAfterCm,
                           float turnSequenceStartCm, uint8_t pulsesUsed, unsigned long completedMs) {
  ManeuverOutcome outcome;
  outcome.action = action;
  outcome.frontBeforeCm = frontBeforeCm;
  outcome.frontAfterCm = frontAfterCm;
  outcome.improvementCm = outcome.frontAfterCm - outcome.frontBeforeCm;
  outcome.turnSequenceStartCm = turnSequenceStartCm;
  outcome.pulsesUsed = pulsesUsed;
  outcome.improved = improved;
  outcome.completedMs = completedMs;
  maneuverOutcomeHistory[maneuverOutcomeHead] = outcome;
  maneuverOutcomeHead = (maneuverOutcomeHead + 1) % MANEUVER_OUTCOME_HISTORY_SIZE;
  if (maneuverOutcomeCount < MANEUVER_OUTCOME_HISTORY_SIZE) {
    maneuverOutcomeCount++;
  }
}
String recentManeuverOutcomesString(uint8_t limit) {
  if (maneuverOutcomeCount == 0) {
    return "NONE";
  }
  String out = "";
  uint8_t n = maneuverOutcomeCount < limit ? maneuverOutcomeCount : limit;
  for (uint8_t i = 0; i < n; i++) {
    if (i > 0) {
      out += ";";
    }
    int index = (int)maneuverOutcomeHead - 1 - (int)i;
    while (index < 0) {
      index += MANEUVER_OUTCOME_HISTORY_SIZE;
    }
    ManeuverOutcome outcome = maneuverOutcomeHistory[index % MANEUVER_OUTCOME_HISTORY_SIZE];
    out += actionToString(outcome.action);
    out += "[";
    out += String(outcome.frontBeforeCm, 1);
    out += "->";
    out += String(outcome.frontAfterCm, 1);
    out += ",d=";
    out += String(outcome.improvementCm, 1);
    out += ",start=";
    out += String(outcome.turnSequenceStartCm, 1);
    out += ",pulses=";
    out += String(outcome.pulsesUsed);
    out += ",";
    out += outcome.improved ? "OK" : "FAIL";
    out += "]";
  }
  return out;
}
const char *escapeAttemptStageString(EscapeAttemptStage stage) {
  switch (stage) {
    case ESCAPE_ATTEMPT_CLEARER_SIDE:
      return "CLEARER_SIDE";
    case ESCAPE_ATTEMPT_CONTINUE_FARTHER:
      return "CONTINUE_FARTHER";
    case ESCAPE_ATTEMPT_OPPOSITE_SIDE:
      return "OPPOSITE_SIDE";
    case ESCAPE_ATTEMPT_SHORT_REVERSE:
      return "SHORT_REVERSE";
    case ESCAPE_ATTEMPT_TRAPPED:
      return "TRAPPED";
    default:
      return "CLEARER_SIDE";
  }
}
void noteEscapeOutcome(Action action, bool improved, float frontAfterCm, unsigned long completedMs) {
  recordManeuverOutcome(action, improved, turnSequenceStartCm, frontAfterCm, -1.0f, 0, completedMs);
  if (action == ACTION_LEFT || action == ACTION_RIGHT) {
    if (escapePrimaryAction == ACTION_STOP) {
      escapePrimaryAction = action;
    }
    if (improved) {
      escapeProgressResetPending = true;
      escapeProgressReferenceFrontCm = frontAfterCm;
    }
  }
  if (action == ACTION_BACKWARD) {
    if (improved) {
      escapeProgressResetPending = true;
      escapeProgressReferenceFrontCm = frontAfterCm;
    }
  }
}
void recordTurnSequenceOutcome(Action action, bool improved, float turnSequenceStartCmValue, float frontAfterCm,
                               uint8_t pulsesUsed, unsigned long completedMs) {
  recordManeuverOutcome(action, improved, turnSequenceStartCmValue, frontAfterCm, turnSequenceStartCmValue,
                        pulsesUsed, completedMs);
}
void resetTurnSequenceState() {
  turnSequenceStartCm = -1.0f;
  turnSequenceStartedMs = 0;
  turnPulsesExecuted = 0;
  turnCheckPending = false;
  turnFrontReadingReady = false;
}
void finalizeTurnSequenceOutcome(bool improved, float frontAfterCm, unsigned long completedMs) {
  recordTurnSequenceOutcome(activeManeuverAction, improved, turnSequenceStartCm, frontAfterCm,
                            turnPulsesExecuted, completedMs);
  invalidateSideScan();
  resetTurnSequenceState();
}
void maybeResetEscapeAttemptStages(unsigned long nowMs) {
  if (!escapeProgressResetPending) {
    return;
  }
  if (currentMotorAction != ACTION_FORWARD) {
    return;
  }
  float frontCm = estimateFrontClearanceCm();
  if (frontCm >= ULTRASONIC_MIN_VALID_CM &&
      frontCm >= escapeProgressReferenceFrontCm + TURN_CLEARANCE_IMPROVEMENT_CM) {
    Serial.print("Meaningful forward progress at ");
    Serial.print(frontCm);
    Serial.print("cm after outcome at ");
    Serial.print(escapeProgressReferenceFrontCm);
    Serial.println("cm; resetting escape stage");
    escapeAttemptStage = ESCAPE_ATTEMPT_CLEARER_SIDE;
    escapePrimaryAction = ACTION_STOP;
    escapeProgressResetPending = false;
    escapeProgressReferenceFrontCm = -1.0f;
    escapeTrapSignaled = false;
    lastObstacleDecisionMs = nowMs;
  }
}
void signalTrappedState(unsigned long nowMs) {
  if (escapeTrapSignaled) {
    return;
  }
  escapeTrapSignaled = true;
  Serial.println("Escape stage 5: trapped, stopping and signalling");
  startBuzzer(250);
  emergencyStop("trapped state");
  roverState = STATE_HAZARD;
  lastObstacleDecisionMs = nowMs;
}
EscapeAttemptStage nextEscapeStage(EscapeAttemptStage stage) {
  switch (stage) {
    case ESCAPE_ATTEMPT_CLEARER_SIDE:
      return ESCAPE_ATTEMPT_CONTINUE_FARTHER;
    case ESCAPE_ATTEMPT_CONTINUE_FARTHER:
      return ESCAPE_ATTEMPT_OPPOSITE_SIDE;
    case ESCAPE_ATTEMPT_OPPOSITE_SIDE:
      return ESCAPE_ATTEMPT_SHORT_REVERSE;
    case ESCAPE_ATTEMPT_SHORT_REVERSE:
      return ESCAPE_ATTEMPT_TRAPPED;
    case ESCAPE_ATTEMPT_TRAPPED:
    default:
      return ESCAPE_ATTEMPT_TRAPPED;
  }
}
bool applyImmediatePostManeuverHazardCheck(const DirectionalReading &reading) {
  bool immediatePostManeuverHazard =
      (reading.valid && reading.distanceCm <= ULTRASONIC_ALERT_CM);
  if (immediatePostManeuverHazard) {
    hazardSampleCount = HAZARD_CONFIRM_SAMPLES;
    obstacleNearby = true;
  } else {
    obstacleNearby = updateHazardFromDistance(reading.distanceCm);
  }
  lastDistanceCm = reading.distanceCm;
  lastSensorMs = reading.capturedMs;
  if (reading.valid) {
    updateRadarBuckets(reading);
  }
  return immediatePostManeuverHazard;
}
Action chooseLocalEscapeActionForStage(const NavigationSnapshot &snapshot, const SideScanResult &scan,
                                       Action previousAction) {
  Action physicallyBlockedAction = ACTION_FORWARD;
  Action discouragedOscillationAction = oppositeAction(previousAction);
  switch (escapeAttemptStage) {
    case ESCAPE_ATTEMPT_CLEARER_SIDE:
      if (scan.bestAction == ACTION_LEFT || scan.bestAction == ACTION_RIGHT) {
        escapePrimaryAction = scan.bestAction;
        return scan.bestAction;
      }
      return validateSafeAction(
          chooseBestActionFromSafeSet(buildSafeActionSet(snapshot, scan),
                                      physicallyBlockedAction, discouragedOscillationAction),
          snapshot, scan);
    case ESCAPE_ATTEMPT_CONTINUE_FARTHER:
      if (escapePrimaryAction == ACTION_LEFT || escapePrimaryAction == ACTION_RIGHT) {
        return escapePrimaryAction;
      }
      return validateSafeAction(
          chooseBestActionFromSafeSet(buildSafeActionSet(snapshot, scan),
                                      physicallyBlockedAction, discouragedOscillationAction),
          snapshot, scan);
    case ESCAPE_ATTEMPT_OPPOSITE_SIDE:
      if (escapePrimaryAction == ACTION_LEFT || escapePrimaryAction == ACTION_RIGHT) {
        return oppositeAction(escapePrimaryAction);
      }
      return oppositeAction(previousAction);
    case ESCAPE_ATTEMPT_SHORT_REVERSE:
      return reverseRecoveryPermitted(escapeAttemptStage) ? ACTION_BACKWARD : ACTION_STOP;
    case ESCAPE_ATTEMPT_TRAPPED:
      return ACTION_STOP;
    default:
      return validateSafeAction(
          chooseBestActionFromSafeSet(buildSafeActionSet(snapshot, scan),
                                      physicallyBlockedAction, discouragedOscillationAction),
          snapshot, scan);
  }
}
Action chooseEscapeActionForStage(const NavigationSnapshot &snapshot, const SideScanResult &scan,
                                  Action previousAction) {
  return chooseLocalEscapeActionForStage(snapshot, scan, previousAction);
}
Action chooseLocalEscapeActionForStage(const SideScanResult &scan, Action previousAction) {
  NavigationSnapshot snapshot = buildNavigationSnapshot(millis());
  return chooseLocalEscapeActionForStage(snapshot, scan, previousAction);
}
Action chooseEscapeActionForStage(const SideScanResult &scan, Action previousAction) {
  NavigationSnapshot snapshot = buildNavigationSnapshot(millis());
  return chooseEscapeActionForStage(snapshot, scan, previousAction);
}
const char *currentTurnDirectionString(Action previousAction) {
  if (previousAction == ACTION_LEFT) {
    return "LEFT";
  }
  if (previousAction == ACTION_RIGHT) {
    return "RIGHT";
  }
  return "NONE";
}
const char *hazardClassString(float distanceCm) {
  if (distanceCm < ULTRASONIC_MIN_VALID_CM) {
    return "UNKNOWN";
  }
  if (distanceCm <= ULTRASONIC_EMERGENCY_CLOSE_CM) {
    return "CRITICAL";
  }
  if (distanceCm <= ULTRASONIC_ALERT_CM) {
    return "NEAR";
  }
  return "CLEAR";
}
const char *environmentHintString(float frontCm, const SideScanResult &scan) {
  bool frontValid = (frontCm >= ULTRASONIC_MIN_VALID_CM);
  bool leftValid = (scan.leftDistanceCm >= ULTRASONIC_MIN_VALID_CM);
  bool rightValid = (scan.rightDistanceCm >= ULTRASONIC_MIN_VALID_CM);
  if (frontValid && frontCm <= ULTRASONIC_EMERGENCY_CLOSE_CM) {
    if (leftValid && rightValid && scan.leftDistanceCm <= ULTRASONIC_ALERT_CM &&
        scan.rightDistanceCm <= ULTRASONIC_ALERT_CM) {
      return "DEAD_END";
    }
    if (leftValid && (!rightValid || scan.leftDistanceCm > scan.rightDistanceCm + 8.0f)) {
      return "OPEN_LEFT";
    }
    if (rightValid && (!leftValid || scan.rightDistanceCm > scan.leftDistanceCm + 8.0f)) {
      return "OPEN_RIGHT";
    }
    return "FRONT_BLOCKED";
  }
  if (frontValid && frontCm <= ULTRASONIC_ALERT_CM) {
    return "FRONT_WALL";
  }
  if (leftValid && rightValid) {
    float delta = scan.leftDistanceCm - scan.rightDistanceCm;
    if (delta < 0.0f) {
      delta = -delta;
    }
    if (delta <= 8.0f) {
      return "CORRIDOR";
    }
    return (scan.leftDistanceCm > scan.rightDistanceCm) ? "OPEN_LEFT" : "OPEN_RIGHT";
  }
  if (leftValid) {
    return "OPEN_LEFT";
  }
  if (rightValid) {
    return "OPEN_RIGHT";
  }
  return "UNCERTAIN";
}
const char *driveModeString(DriveMode mode) {
  switch (mode) {
    case DRIVE_FORWARD:
      return "FORWARD";
    case DRIVE_REVERSE:
      return "REVERSE";
    case DRIVE_ARC_LEFT:
      return "ARC_LEFT";
    case DRIVE_ARC_RIGHT:
      return "ARC_RIGHT";
    case DRIVE_PIVOT_LEFT:
      return "PIVOT_LEFT";
    case DRIVE_PIVOT_RIGHT:
      return "PIVOT_RIGHT";
    default:
      return "STOPPED";
  }
}
bool driveModeHasForwardMotion(DriveMode mode) {
  return mode == DRIVE_FORWARD || mode == DRIVE_ARC_LEFT || mode == DRIVE_ARC_RIGHT;
}
const char *geminiPromptTemplateString(GeminiPromptTemplate templateType) {
  switch (templateType) {
    case GEMINI_PROMPT_FAILED_MANEUVER:
      return "FAILED_MANEUVER";
    case GEMINI_PROMPT_TRAPPED_RECOVERY:
      return "TRAPPED_RECOVERY";
    default:
      return "STANDARD_OBSTACLE";
  }
}
GeminiPromptTemplate geminiPromptTemplateForStage(EscapeAttemptStage stage) {
  switch (stage) {
    case ESCAPE_ATTEMPT_CLEARER_SIDE:
      return GEMINI_PROMPT_STANDARD_OBSTACLE;
    case ESCAPE_ATTEMPT_CONTINUE_FARTHER:
    case ESCAPE_ATTEMPT_OPPOSITE_SIDE:
    case ESCAPE_ATTEMPT_SHORT_REVERSE:
      return GEMINI_PROMPT_FAILED_MANEUVER;
    case ESCAPE_ATTEMPT_TRAPPED:
      return GEMINI_PROMPT_TRAPPED_RECOVERY;
    default:
      return GEMINI_PROMPT_STANDARD_OBSTACLE;
  }
}
int sensorConfidenceScore(unsigned long frontAgeMs, unsigned long leftAgeMs, unsigned long rightAgeMs,
                         uint16_t noEchoStreak) {
  int score = 100;
  if (frontAgeMs > RADAR_FRONT_STALE_MS) {
    score -= 20;
  }
  if (leftAgeMs > 1200UL) {
    score -= 8;
  }
  if (rightAgeMs > 1200UL) {
    score -= 8;
  }
  if (radarFrontSampleConfidence < 3) {
    score -= 12;
  }
  if (radarLeftSampleConfidence < 3) {
    score -= 4;
  }
  if (radarRightSampleConfidence < 3) {
    score -= 4;
  }
  int noEchoPenalty = (int)noEchoStreak * 9;
  if (noEchoPenalty > 55) {
    noEchoPenalty = 55;
  }
  score -= noEchoPenalty;
  if (score < 5) {
    score = 5;
  }
  if (score > 100) {
    score = 100;
  }
  return score;
}
NavigationSnapshot buildNavigationSnapshot(unsigned long nowMs) {
  // Build a single coherent view over the latest front and side measurements.
  // The snapshot normalizes raw buckets into "valid", "blocked", and confidence decisions
  // so the rest of the rover logic can avoid duplicating stale-data checks everywhere.
  NavigationSnapshot snapshot = {};
  snapshot.frontCm = currentFrontDistanceCm;
  snapshot.leftCm = radarLeftCm;
  snapshot.rightCm = radarRightCm;
  snapshot.frontAgeMs = (radarFrontUpdatedMs == 0) ? 9999UL : (nowMs - radarFrontUpdatedMs);
  snapshot.leftAgeMs = (radarLeftUpdatedMs == 0) ? 9999UL : (nowMs - radarLeftUpdatedMs);
  snapshot.rightAgeMs = (radarRightUpdatedMs == 0) ? 9999UL : (nowMs - radarRightUpdatedMs);
  if (radarFrontUpdatedMs != 0 && snapshot.frontAgeMs <= RADAR_FRONT_STALE_MS && radarFrontCm >= ULTRASONIC_MIN_VALID_CM) {
    snapshot.frontCm = radarFrontCm;
  }
  if (radarLeftUpdatedMs != 0 && snapshot.leftAgeMs <= RADAR_SIDE_STALE_MS && radarLeftCm >= ULTRASONIC_MIN_VALID_CM) {
    snapshot.leftCm = radarLeftCm;
  }
  if (radarRightUpdatedMs != 0 && snapshot.rightAgeMs <= RADAR_SIDE_STALE_MS && radarRightCm >= ULTRASONIC_MIN_VALID_CM) {
    snapshot.rightCm = radarRightCm;
  }
  snapshot.frontValid = snapshot.frontAgeMs <= RADAR_FRONT_STALE_MS && snapshot.frontCm >= ULTRASONIC_MIN_VALID_CM;
  snapshot.leftValid = snapshot.leftAgeMs <= RADAR_SIDE_STALE_MS && snapshot.leftCm >= ULTRASONIC_MIN_VALID_CM;
  snapshot.rightValid = snapshot.rightAgeMs <= RADAR_SIDE_STALE_MS && snapshot.rightCm >= ULTRASONIC_MIN_VALID_CM;
  snapshot.frontBlocked = snapshot.frontValid && snapshot.frontCm <= ULTRASONIC_ALERT_CM;
  snapshot.confidence = sensorConfidenceScore(snapshot.frontAgeMs, snapshot.leftAgeMs, snapshot.rightAgeMs,
                                              ultrasonicNoEchoStreak);
  return snapshot;
}
float estimateForwardSpeedCmPerSecFromBaseSpeed(int speedBase) {
  if (speedBase < ADAPTIVE_FORWARD_MIN_BASE_SPEED) {
    speedBase = ADAPTIVE_FORWARD_MIN_BASE_SPEED;
  }
  if (speedBase > BASE_SPEED) {
    speedBase = BASE_SPEED;
  }
  float normalized = (float)(speedBase - ADAPTIVE_FORWARD_MIN_BASE_SPEED);
  float normalizedRange = (float)(BASE_SPEED - ADAPTIVE_FORWARD_MIN_BASE_SPEED);
  if (normalizedRange <= 0.0f) {
    return 0.0f;
  }
  normalized /= normalizedRange;
  float minimumCmPerSec = 11.0f;
  float maximumCmPerSec = 24.0f;
  return minimumCmPerSec + (normalized * (maximumCmPerSec - minimumCmPerSec));
}
void startMotionCalibrationRecord(unsigned long stopCommandMs, float stopDistanceCm, int commandedBaseSpeed,
                                  int commandedRightPwm, int commandedLeftPwm, float closingSpeedCmPerSec) {
  // Capture the exact conditions of a stop event so serial logs can be used to tune
  // the stopping-distance model later without changing runtime behavior now.
  motionCalibrationActive = true;
  motionCalibrationStopCommandMs = stopCommandMs;
  motionCalibrationStopDistanceCm = stopDistanceCm;
  motionCalibrationMinDistanceAfterStopCm = stopDistanceCm;
  motionCalibrationClosingSpeedCmPerSec = closingSpeedCmPerSec;
  lastForwardCommandBaseSpeed = commandedBaseSpeed;
  lastForwardCommandRightPwm = commandedRightPwm;
  lastForwardCommandLeftPwm = commandedLeftPwm;
}
void updateMotionCalibrationMinDistance(const DirectionalReading &reading) {
  if (!motionCalibrationActive || !reading.valid || reading.angleDeg != PAN_FORWARD_DEG) {
    return;
  }
  if (motionCalibrationMinDistanceAfterStopCm < 0.0f || reading.distanceCm < motionCalibrationMinDistanceAfterStopCm) {
    motionCalibrationMinDistanceAfterStopCm = reading.distanceCm;
  }
}
void flushMotionCalibrationRecord(const char *reason) {
  if (!motionCalibrationActive) {
    return;
  }
  Serial.print("STOP_CALIBRATION|");
  Serial.print("reason=");
  Serial.print(reason);
  Serial.print("|base_pwm=");
  Serial.print(lastForwardCommandBaseSpeed);
  Serial.print("|right_pwm=");
  Serial.print(lastForwardCommandRightPwm);
  Serial.print("|left_pwm=");
  Serial.print(lastForwardCommandLeftPwm);
  Serial.print("|closing_cmps=");
  Serial.print(motionCalibrationClosingSpeedCmPerSec);
  Serial.print("|stop_ms=");
  Serial.print(motionCalibrationStopCommandMs);
  Serial.print("|stop_dist_cm=");
  Serial.print(motionCalibrationStopDistanceCm);
  Serial.print("|min_after_stop_cm=");
  Serial.println(motionCalibrationMinDistanceAfterStopCm);
  motionCalibrationActive = false;
}
float estimateStoppingDistanceCm(int speedBase, bool closingRateCorroborated, float corroboratedClosingRateCmPerSec) {
  // Simple stopping model:
  // commanded travel during control/sensor delay + braking distance + fixed safety margin.
  float commandedSpeedCmPerSec = estimateForwardSpeedCmPerSecFromBaseSpeed(speedBase);
  float effectiveSpeedCmPerSec = commandedSpeedCmPerSec;
  if (closingRateCorroborated && corroboratedClosingRateCmPerSec >= TTC_BRAKE_MIN_CLOSING_CMPS) {
    effectiveSpeedCmPerSec = (commandedSpeedCmPerSec + corroboratedClosingRateCmPerSec) * 0.5f;
  }
  float stopDelaySec = ((float)STOPPING_MOTOR_DELAY_MS + (float)SENSOR_INTERVAL_MS) / 1000.0f;
  float stoppingDistanceCm = effectiveSpeedCmPerSec * stopDelaySec;
  stoppingDistanceCm += (effectiveSpeedCmPerSec * effectiveSpeedCmPerSec) / (2.0f * STOPPING_MODEL_DECEL_CMPS2);
  stoppingDistanceCm += STOPPING_MODEL_MARGIN_CM;
  return stoppingDistanceCm;
}
int computeAdaptiveForwardBaseSpeed(float frontDistanceCm, bool closingRateCorroborated, float corroboratedClosingRateCmPerSec) {
  // Reduce cruise speed as clearance shrinks, but never pick a speed whose modeled stopping
  // distance would exceed the currently available space in front of the rover.
  int target = BASE_SPEED;
  if (frontDistanceCm >= ULTRASONIC_MIN_VALID_CM) {
    if (frontDistanceCm <= 18.0f) {
      target = BASE_SPEED - 34;
    } else if (frontDistanceCm <= 28.0f) {
      target = BASE_SPEED - 26;
    } else if (frontDistanceCm <= 42.0f) {
      target = BASE_SPEED - 16;
    } else if (frontDistanceCm <= 70.0f) {
      target = BASE_SPEED - 8;
    }
    int candidateSpeeds[] = {
        BASE_SPEED,
        BASE_SPEED - 8,
        BASE_SPEED - 16,
        BASE_SPEED - 26,
        BASE_SPEED - 34,
        ADAPTIVE_FORWARD_MIN_BASE_SPEED,
    };
    float availableClearanceCm = frontDistanceCm - ULTRASONIC_EMERGENCY_CLOSE_CM;
    if (availableClearanceCm < 0.0f) {
      availableClearanceCm = 0.0f;
    }
    int safeCandidate = 0;
    for (uint8_t i = 0; i < sizeof(candidateSpeeds) / sizeof(candidateSpeeds[0]); i++) {
      int candidate = candidateSpeeds[i];
      if (candidate > target) {
        continue;
      }
      float stoppingDistanceCm = estimateStoppingDistanceCm(candidate, closingRateCorroborated, corroboratedClosingRateCmPerSec);
      if (stoppingDistanceCm <= availableClearanceCm) {
        safeCandidate = candidate;
        break;
      }
    }
    target = safeCandidate;
  }
  if (target > BASE_SPEED) {
    target = BASE_SPEED;
  }
  return target;
}
DirectionalReading readDirectionalUltrasonic(uint8_t measurementAngle) {
  DirectionalReading reading;
  reading.angleDeg = measurementAngle;
  reading.distanceCm = readUltrasonicCm();
  reading.capturedMs = millis();
  reading.sampleConfidence = 1;
  reading.valid = reading.distanceCm >= ULTRASONIC_MIN_VALID_CM;
  lastUltrasonicSampleMs = reading.capturedMs;
  return reading;
}
DirectionalReading medianOfThreeDirectionalReadings(
    const DirectionalReading &a,
    const DirectionalReading &b,
    const DirectionalReading &c) {
  DirectionalReading first = a;
  DirectionalReading second = b;
  DirectionalReading third = c;
  if (first.distanceCm > second.distanceCm) {
    DirectionalReading temp = first;
    first = second;
    second = temp;
  }
  if (second.distanceCm > third.distanceCm) {
    DirectionalReading temp = second;
    second = third;
    third = temp;
  }
  if (first.distanceCm > second.distanceCm) {
    DirectionalReading temp = first;
    first = second;
    second = temp;
  }
  second.sampleConfidence = 3;
  return second;
}
DirectionalReading filterDirectionalReadings(const DirectionalReading *samples, uint8_t validCount, uint8_t angleDeg,
                                            unsigned long capturedMs) {
  DirectionalReading filteredReading;
  filteredReading.angleDeg = angleDeg;
  filteredReading.capturedMs = capturedMs;
  filteredReading.distanceCm = -1.0f;
  filteredReading.sampleConfidence = 0;
  filteredReading.valid = false;
  if (validCount == 0) {
    return filteredReading;
  }
  if (validCount == 1) {
    filteredReading = samples[0];
    filteredReading.sampleConfidence = 1;
    return filteredReading;
  }
  if (validCount == 2) {
    filteredReading = samples[0];
    if (samples[1].distanceCm < filteredReading.distanceCm) {
      filteredReading = samples[1];
    }
    filteredReading.angleDeg = angleDeg;
    filteredReading.capturedMs = capturedMs;
    filteredReading.sampleConfidence = 2;
    filteredReading.valid = true;
    return filteredReading;
  }
  filteredReading = medianOfThreeDirectionalReadings(samples[0], samples[1], samples[2]);
  filteredReading.angleDeg = angleDeg;
  filteredReading.capturedMs = capturedMs;
  filteredReading.sampleConfidence = 3;
  filteredReading.valid = true;
  return filteredReading;
}
float estimateFrontClearanceCm() {
  unsigned long nowMs = millis();
  if (radarFrontUpdatedMs != 0 && (nowMs - radarFrontUpdatedMs) <= RADAR_FRONT_STALE_MS &&
      radarFrontCm >= ULTRASONIC_MIN_VALID_CM) {
    return radarFrontCm;
  }
  return currentFrontDistanceCm;
}
void startNoEchoRecovery(unsigned long nowMs) {
  // A long no-echo streak is treated as a sensor-health problem, not as "clear path".
  // The rover stops, holds position, and forces a rescan after a cooldown.
  panSweepRecoveryUntilMs = nowMs + ULTRASONIC_NO_ECHO_RECOVERY_HOLD_MS;
  sensorRetryActive = false;
  sensorRetryAttempt = 0;
  sensorRetryDueMs = 0;
  ultrasonicScanState = SCAN_IDLE;
  resetDirectionalSampleCollector();
  adaptiveForwardBaseSpeed = ADAPTIVE_FORWARD_MIN_BASE_SPEED;
  resetTtcSequence();
  obstacleNearby = true;
  roverState = STATE_HAZARD;
  emergencyStop("invalid sensor state");
  hazardClearArmed = false;
  Serial.println("No echo recovery: stopping and scheduling a rescan hold");
}
void processDirectionalScanReading(const DirectionalReading &reading) {
  // Every valid reading updates the short-lived directional radar cache.
  // Only forward-facing reads participate in time-to-collision logic.
  updateRadarBuckets(reading);
  lastSensorMs = reading.capturedMs;
  if (reading.angleDeg != PAN_FORWARD_DEG) {
bool applyMotorAction(Action action, const NavigationSnapshot &snapshot);
    resetTtcSequence();
    updateMotionCalibrationMinDistance(reading);
  }
}
bool processFrontSafetyReading(unsigned long now, bool previousObstacle, const DirectionalReading &reading) {
  // Front readings are the safety-critical path.
  // This function merges raw distance, retry behavior, time-to-collision braking,
  // hazard hysteresis, and immediate-stop rules into one decision point.
  processDirectionalScanReading(reading);
  if (reading.angleDeg != PAN_FORWARD_DEG) {
    return false;
  }
  updateMotionCalibrationMinDistance(reading);
  lastDistanceCm = reading.distanceCm;
  if (reading.valid) {
    previousFrontDistanceCm = currentFrontDistanceCm;
    currentFrontDistanceCm = reading.distanceCm;
  }
  bool ttcSequenceValid =
      currentDriveMode == DRIVE_FORWARD &&
      reading.angleDeg == PAN_FORWARD_DEG &&
      reading.valid &&
      lastTtcMs != 0 &&
      (reading.capturedMs - lastTtcMs) <= TTC_MAX_SAMPLE_GAP_MS;
  if (lastDistanceCm < 0.0f) {
    if (ultrasonicNoEchoStreak < 65535) {
      ultrasonicNoEchoStreak++;
    }
    if (!sensorRetryActive && sensorRetryAttempt == 0) {
      sensorRetryPreviousObstacle = previousObstacle;
      sensorRetryActive = true;
      sensorRetryAttempt = 0;
      sensorRetryDueMs = now + ULTRASONIC_NO_ECHO_RETRY_DELAY_MS;
      return true;
    }
    if (ultrasonicNoEchoStreak >= ULTRASONIC_NO_ECHO_RECOVERY_STREAK) {
      startNoEchoRecovery(now);
      return true;
    }
  } else {
    ultrasonicNoEchoStreak = 0;
    sensorRetryActive = false;
    sensorRetryAttempt = 0;
    sensorRetryDueMs = 0;
  }
  float hazardDistance = lastDistanceCm;
  if (radarFrontUpdatedMs != 0 && (now - radarFrontUpdatedMs) <= RADAR_FRONT_STALE_MS) {
    hazardDistance = radarFrontCm;
    Serial.print("Using front-radar hazard distance: ");
    Serial.println(hazardDistance);
  }
  if (hazardDistance >= ULTRASONIC_MIN_VALID_CM) {
    adaptiveForwardBaseSpeed = computeAdaptiveForwardBaseSpeed(
        hazardDistance, ttcClosingCorroborated, corroboratedTtcClosingCmPerSec);
    if (adaptiveForwardBaseSpeed == 0) {
      Serial.println("Adaptive forward speed unsafe; stopping instead");
      resetTtcSequence();
      emergencyStop("critical hazard");
      roverState = STATE_HAZARD;
      obstacleNearby = true;
      return true;
    }
    if (ttcSampleValid && !ttcSequenceValid) {
      resetTtcSequence();
      ttcSequenceValid = false;
    }
    if (ttcSequenceValid) {
      unsigned long dtMs = reading.capturedMs - lastTtcMs;
      if (dtMs > 0) {
        float closingCmPerSec = ((lastTtcFrontCm - hazardDistance) * 1000.0f) / (float)dtMs;
        bool closingSampleValid =
            hazardDistance <= TTC_BRAKE_MAX_FRONT_CM &&
            closingCmPerSec >= TTC_BRAKE_MIN_CLOSING_CMPS;
        if (closingSampleValid) {
          if (lastTtcClosingCmPerSec >= 0.0f) {
            float closingDelta = closingCmPerSec - lastTtcClosingCmPerSec;
            if (closingDelta < 0.0f) {
              closingDelta = -closingDelta;
            }
            if (closingDelta <= TTC_CLOSE_TOLERANCE_CMPS) {
              if (ttcClosingConfirmCount < 255) {
                ttcClosingConfirmCount++;
              }
            } else {
              ttcClosingConfirmCount = 1;
            }
          } else {
            ttcClosingConfirmCount = 1;
          }
          lastTtcClosingCmPerSec = closingCmPerSec;
          ttcClosingCorroborated = ttcClosingConfirmCount >= TTC_CLOSE_CONFIRM_SAMPLES;
          if (ttcClosingCorroborated) {
            if (corroboratedTtcClosingCmPerSec < 0.0f) {
              corroboratedTtcClosingCmPerSec = closingCmPerSec;
            } else {
              corroboratedTtcClosingCmPerSec =
                  (corroboratedTtcClosingCmPerSec + closingCmPerSec) * 0.5f;
            }
          }
          if (ttcClosingCorroborated) {
            float availableClearanceCm = hazardDistance - ULTRASONIC_EMERGENCY_CLOSE_CM;
            if (availableClearanceCm < 0.0f) {
              availableClearanceCm = 0.0f;
            }
            float stoppingDistanceCm = estimateStoppingDistanceCm(
                adaptiveForwardBaseSpeed, true, corroboratedTtcClosingCmPerSec);
            bool ttcBrakeCooldownElapsed =
                (lastTtcBrakeMs == 0) || ((now - lastTtcBrakeMs) >= TTC_BRAKE_COOLDOWN_MS);
            if (availableClearanceCm <= stoppingDistanceCm &&
                ttcBrakeCooldownElapsed && roverState != STATE_MANEUVERING) {
              float ttcMs = (availableClearanceCm / corroboratedTtcClosingCmPerSec) * 1000.0f;
              Serial.print("Stopping-distance brake: front=");
              Serial.print(hazardDistance);
              Serial.print("cm closing=");
              Serial.print(corroboratedTtcClosingCmPerSec);
              Serial.print("cm/s stopDist=");
              Serial.print(stoppingDistanceCm);
              Serial.print("cm available=");
              Serial.print(availableClearanceCm);
              Serial.print("cm ttc=");
              Serial.print(ttcMs);
              Serial.println("ms -> emergency reverse");
              startBuzzer(140);
              emergencyStop("critical hazard");
              startMotionCalibrationRecord(now, hazardDistance, lastForwardCommandBaseSpeed,
                                          lastForwardCommandRightPwm, lastForwardCommandLeftPwm,
                                          corroboratedTtcClosingCmPerSec);
              startManeuver(ACTION_BACKWARD, EMERGENCY_REVERSE_DURATION_MS, ACTION_FORWARD);
              lastTtcBrakeMs = now;
              hazardSampleCount = HAZARD_CONFIRM_SAMPLES;
              obstacleNearby = true;
              lastTtcFrontCm = hazardDistance;
              lastTtcMs = reading.capturedMs;
              ttcSampleValid = true;
              return true;
            }
          }
        } else {
          ttcClosingConfirmCount = 0;
          ttcClosingCorroborated = false;
          corroboratedTtcClosingCmPerSec = -1.0f;
        }
      }
    }
    lastTtcFrontCm = hazardDistance;
    lastTtcMs = reading.capturedMs;
    ttcSampleValid = true;
  } else {
    adaptiveForwardBaseSpeed = BASE_SPEED;
    resetTtcSequence();
  }
  if (startupFrontScanPending && reading.valid && !obstacleNearby) {
    startupFrontScanPending = false;
  }
  bool veryCloseHazard =
      (hazardDistance >= ULTRASONIC_MIN_VALID_CM && hazardDistance <= ULTRASONIC_EMERGENCY_CLOSE_CM);
  bool closeReverseCooldownElapsed =
      (lastEmergencyCloseReverseMs == 0) ||
      ((now - lastEmergencyCloseReverseMs) >= EMERGENCY_CLOSE_REVERSE_COOLDOWN_MS);
  if (veryCloseHazard && roverState != STATE_MANEUVERING && closeReverseCooldownElapsed) {
    Serial.print("Emergency close hazard: ");
    Serial.print(hazardDistance);
    Serial.println(" cm, executing extended reverse");
    startBuzzer(150);
    emergencyStop("critical hazard");
    startManeuver(ACTION_BACKWARD, EMERGENCY_CLOSE_REVERSE_DURATION_MS, ACTION_FORWARD);
    lastEmergencyCloseReverseMs = now;
    hazardSampleCount = HAZARD_CONFIRM_SAMPLES;
    obstacleNearby = true;
    return true;
  }
  obstacleNearby = updateHazardFromDistance(hazardDistance);
  if (obstacleNearby) {
    if (!previousObstacle && hazardClearArmed) {
      hazardClearArmed = false;
    }
  }
  if (obstacleNearby && !previousObstacle) {
    Serial.println("Hazard threshold crossed: immediate stop");
    startBuzzer(100);
    emergencyStop("critical hazard");
    roverState = STATE_HAZARD;
  }
  if (!obstacleNearby && previousObstacle) {
    hazardClearArmed = true;
  }
  return false;
}
bool handleSensorRetryState(unsigned long now) {
  // Retry a small number of no-echo reads before escalating to full sensor recovery.
  if (!sensorRetryActive) {
    return false;
  }
  if ((long)(now - sensorRetryDueMs) < 0) {
    return true;
  }
  if (!ultrasonicSampleGapElapsed(now)) {
    return true;
  }
  DirectionalReading retryReading = readDirectionalUltrasonic(panCurrentDeg);
  if (!retryReading.valid && sensorRetryAttempt < ULTRASONIC_NO_ECHO_RETRIES) {
    sensorRetryAttempt++;
    sensorRetryDueMs = now + ULTRASONIC_NO_ECHO_RETRY_DELAY_MS;
    return true;
  }
  sensorRetryActive = false;
  return processFrontSafetyReading(now, sensorRetryPreviousObstacle, retryReading);
}
void setPanAngle(uint8_t angleDeg) {
  if (!panServoReady) {
    return;
  }
  uint8_t clamped = angleDeg;
  if (clamped > 180) {
    clamped = 180;
  }
  panCurrentDeg = clamped;
  ultrasonicPanServo.write((int)clamped);
}
void runPanServoSelfTest() {
  if (!panServoReady) {
    return;
  }
  Serial.println("Pan self-test: CENTER");
  setPanAngle(PAN_FORWARD_DEG);
  delay(400);
  Serial.println("Pan self-test: RIGHT");
  setPanAngle(PAN_RIGHT_DEG);
  delay(500);
  Serial.println("Pan self-test: LEFT");
  setPanAngle(PAN_LEFT_DEG);
  delay(500);
  Serial.println("Pan self-test: CENTER");
  setPanAngle(PAN_FORWARD_DEG);
  delay(400);
}
void updatePanSweepDebug(unsigned long nowMs) {
  if (!panServoReady) {
    return;
  }
  if (!panSweepEnabled) {
    return;
  }
  const unsigned long debugStepMs = 120;
  const uint8_t debugStepDeg = 8;
  if ((nowMs - lastPanStepMs) < debugStepMs) {
    return;
  }
  lastPanStepMs = nowMs;
  if (panSweepTowardLeft) {
    int next = (int)panCurrentDeg + (int)debugStepDeg;
    if (next >= PAN_LEFT_DEG) {
      next = PAN_LEFT_DEG;
      panSweepTowardLeft = false;
    }
    setPanAngle((uint8_t)next);
  } else {
    int next = (int)panCurrentDeg - (int)debugStepDeg;
    if (next <= PAN_RIGHT_DEG) {
      next = PAN_RIGHT_DEG;
      panSweepTowardLeft = true;
    }
    setPanAngle((uint8_t)next);
  }
}
void resetDirectionalSampleCollector() {
  ultrasonicScanValidCount = 0;
  ultrasonicScanAttemptCount = 0;
}
bool updateStationaryUltrasonicSampling(unsigned long nowMs, bool previousObstacle) {
  // Non-blocking scan controller.
  // The servo visits left, front, and right; each direction is allowed multiple attempts,
  // then collapsed into a filtered reading before the next scan state begins.
  if (!panServoReady) {
    return false;
  }
  if (panSweepRecoveryUntilMs != 0 && (long)(nowMs - panSweepRecoveryUntilMs) < 0) {
    return true;
  }
  if (ultrasonicScanState == SCAN_IDLE) {
    if ((nowMs - lastSensorMs) < SENSOR_INTERVAL_MS) {
      return false;
    }
    resetDirectionalSampleCollector();
    ultrasonicScanState = SCAN_MOVE_LEFT;
    ultrasonicScanStateMs = nowMs;
  }
  switch (ultrasonicScanState) {
    case SCAN_MOVE_LEFT:
      setPanAngle(PAN_LEFT_DEG);
      resetDirectionalSampleCollector();
      ultrasonicScanState = SCAN_SETTLE_LEFT;
      ultrasonicScanStateMs = nowMs;
      return true;
    case SCAN_SETTLE_LEFT:
      if ((nowMs - ultrasonicScanStateMs) < PAN_FRONT_SETTLE_MS) {
        return true;
      }
      ultrasonicScanState = SCAN_SAMPLE_LEFT;
      return true;
    case SCAN_SAMPLE_LEFT:
      {
        if (!ultrasonicSampleGapElapsed(nowMs)) {
          return true;
        }
        DirectionalReading reading = readDirectionalUltrasonic(PAN_LEFT_DEG);
        if (reading.valid && ultrasonicScanValidCount < ULTRASONIC_MEDIAN_SAMPLE_COUNT) {
          ultrasonicScanSamples[ultrasonicScanValidCount++] = reading;
        }
        ultrasonicScanAttemptCount++;
        if (ultrasonicScanValidCount < ULTRASONIC_MEDIAN_SAMPLE_COUNT &&
            ultrasonicScanAttemptCount < ULTRASONIC_MEDIAN_MAX_ATTEMPTS) {
          return true;
        }
        DirectionalReading filteredReading = filterDirectionalReadings(
            ultrasonicScanSamples, ultrasonicScanValidCount, PAN_LEFT_DEG, reading.capturedMs);
        processDirectionalScanReading(filteredReading);
        resetDirectionalSampleCollector();
      }
      ultrasonicScanState = SCAN_MOVE_FRONT;
      ultrasonicScanStateMs = nowMs;
      return true;
    case SCAN_MOVE_FRONT:
      setPanAngle(PAN_FORWARD_DEG);
      resetDirectionalSampleCollector();
      ultrasonicScanState = SCAN_SETTLE_FRONT;
      ultrasonicScanStateMs = nowMs;
      return true;
    case SCAN_SETTLE_FRONT:
      if ((nowMs - ultrasonicScanStateMs) < PAN_FRONT_SETTLE_MS) {
        return true;
      }
      ultrasonicScanState = SCAN_SAMPLE_FRONT;
      return true;
    case SCAN_SAMPLE_FRONT:
      {
        if (!ultrasonicSampleGapElapsed(nowMs)) {
          return true;
        }
        DirectionalReading reading = readDirectionalUltrasonic(PAN_FORWARD_DEG);
        if (reading.valid && ultrasonicScanValidCount < ULTRASONIC_MEDIAN_SAMPLE_COUNT) {
          ultrasonicScanSamples[ultrasonicScanValidCount++] = reading;
        }
        ultrasonicScanAttemptCount++;
        if (ultrasonicScanValidCount < ULTRASONIC_MEDIAN_SAMPLE_COUNT &&
            ultrasonicScanAttemptCount < ULTRASONIC_MEDIAN_MAX_ATTEMPTS) {
          return true;
        }
        DirectionalReading filteredReading = filterDirectionalReadings(
            ultrasonicScanSamples, ultrasonicScanValidCount, PAN_FORWARD_DEG, reading.capturedMs);
        processFrontSafetyReading(nowMs, previousObstacle, filteredReading);
        resetDirectionalSampleCollector();
      }
      ultrasonicScanState = SCAN_MOVE_RIGHT;
      ultrasonicScanStateMs = nowMs;
      return true;
    case SCAN_MOVE_RIGHT:
      setPanAngle(PAN_RIGHT_DEG);
      resetDirectionalSampleCollector();
      ultrasonicScanState = SCAN_SETTLE_RIGHT;
      ultrasonicScanStateMs = nowMs;
      return true;
    case SCAN_SETTLE_RIGHT:
      if ((nowMs - ultrasonicScanStateMs) < PAN_FRONT_SETTLE_MS) {
        return true;
      }
      ultrasonicScanState = SCAN_SAMPLE_RIGHT;
      return true;
    case SCAN_SAMPLE_RIGHT:
      {
        if (!ultrasonicSampleGapElapsed(nowMs)) {
          return true;
        }
        DirectionalReading reading = readDirectionalUltrasonic(PAN_RIGHT_DEG);
        if (reading.valid && ultrasonicScanValidCount < ULTRASONIC_MEDIAN_SAMPLE_COUNT) {
          ultrasonicScanSamples[ultrasonicScanValidCount++] = reading;
        }
        ultrasonicScanAttemptCount++;
        if (ultrasonicScanValidCount < ULTRASONIC_MEDIAN_SAMPLE_COUNT &&
            ultrasonicScanAttemptCount < ULTRASONIC_MEDIAN_MAX_ATTEMPTS) {
          return true;
        }
        DirectionalReading filteredReading = filterDirectionalReadings(
            ultrasonicScanSamples, ultrasonicScanValidCount, PAN_RIGHT_DEG, reading.capturedMs);
        processDirectionalScanReading(filteredReading);
        resetDirectionalSampleCollector();
      }
      ultrasonicScanState = SCAN_COMPLETE;
      ultrasonicScanStateMs = nowMs;
      return true;
    case SCAN_TURN_MOVE_FRONT:
      setPanAngle(PAN_FORWARD_DEG);
      ultrasonicScanState = SCAN_TURN_SETTLE_FRONT;
      ultrasonicScanStateMs = nowMs;
      return true;
    case SCAN_TURN_SETTLE_FRONT:
      if ((nowMs - ultrasonicScanStateMs) < PAN_FRONT_SETTLE_MS) {
        return true;
      }
      ultrasonicScanState = SCAN_TURN_SAMPLE_FRONT;
      return true;
    case SCAN_TURN_SAMPLE_FRONT:
      {
        if (!ultrasonicSampleGapElapsed(nowMs)) {
          return true;
        }
        DirectionalReading reading = readDirectionalUltrasonic(PAN_FORWARD_DEG);
        processFrontSafetyReading(nowMs, obstacleNearby, reading);
        turnLatestFrontCm = reading.distanceCm;
        turnFrontReadingReady = true;
      }
      ultrasonicScanState = SCAN_COMPLETE;
      ultrasonicScanStateMs = nowMs;
      return true;
    case SCAN_COMPLETE:
      lastSensorMs = lastUltrasonicSampleMs;
      ultrasonicScanState = SCAN_IDLE;
      ultrasonicScanStateMs = nowMs;
      if (roverState == STATE_REVERSE_RESCAN) {
        roverState = obstacleNearby ? STATE_HAZARD : STATE_ROAMING;
        if (!obstacleNearby) {
          hazardClearArmed = true;
        }
      }
      if (startupFrontScanPending && !obstacleNearby) {
        startupFrontScanPending = false;
      }
      return true;
    case SCAN_IDLE:
    default:
      return false;
  }
  return true;
}
void updateRadarBuckets(const DirectionalReading &reading) {
  // Store the latest valid reading into front/left/right buckets based on servo angle.
  // These buckets expire quickly, which prevents old side scans from driving new maneuvers.
  if (!reading.valid) {
    return;
  }
  int deltaFromFront = (int)reading.angleDeg - (int)PAN_FORWARD_DEG;
  if (deltaFromFront < 0) {
    deltaFromFront = -deltaFromFront;
  }
  if (deltaFromFront <= PAN_FRONT_WINDOW_DEG) {
    radarFrontCm = reading.distanceCm;
    radarFrontUpdatedMs = reading.capturedMs;
    radarFrontSampleConfidence = reading.sampleConfidence;
    return;
  }
  if (reading.angleDeg > PAN_FORWARD_DEG) {
    radarLeftCm = reading.distanceCm;
    radarLeftUpdatedMs = reading.capturedMs;
    radarLeftSampleConfidence = reading.sampleConfidence;
  } else if (reading.angleDeg < PAN_FORWARD_DEG) {
    radarRightCm = reading.distanceCm;
    radarRightUpdatedMs = reading.capturedMs;
    radarRightSampleConfidence = reading.sampleConfidence;
  }
}
void invalidateSideScan() {
  radarLeftCm = -1.0f;
  radarRightCm = -1.0f;
  radarLeftUpdatedMs = 0;
  radarRightUpdatedMs = 0;
  radarLeftSampleConfidence = 0;
  radarRightSampleConfidence = 0;
}
bool ultrasonicSampleGapElapsed(unsigned long nowMs) {
  return lastUltrasonicSampleMs == 0 || (nowMs - lastUltrasonicSampleMs) >= ULTRASONIC_SAMPLE_GAP_MS;
}
const char *scanBestActionString(const SideScanResult &scan) {
  if (scan.bestAction == ACTION_LEFT) {
    return "LEFT";
  }
  if (scan.bestAction == ACTION_RIGHT) {
    return "RIGHT";
  }
  if (scan.bestAction == ACTION_BACKWARD) {
    return "BACKWARD";
  }
  return "NONE";
}
SideScanResult getRadarScanSnapshot() {
  // Convert the live radar buckets into a decision-friendly left/right comparison.
  // If neither side is trustworthy, the scan reports BACKWARD as the conservative escape bias.
  SideScanResult result;
  unsigned long nowMs = millis();
  bool leftFresh =
      radarLeftUpdatedMs != 0 &&
      (nowMs - radarLeftUpdatedMs) <= RADAR_SIDE_STALE_MS;
  bool rightFresh =
      radarRightUpdatedMs != 0 &&
      (nowMs - radarRightUpdatedMs) <= RADAR_SIDE_STALE_MS;
  result.leftDistanceCm = leftFresh ? radarLeftCm : -1.0f;
  result.rightDistanceCm = rightFresh ? radarRightCm : -1.0f;
  result.bestAction = ACTION_STOP;
  bool leftValid = (result.leftDistanceCm >= ULTRASONIC_MIN_VALID_CM);
  bool rightValid = (result.rightDistanceCm >= ULTRASONIC_MIN_VALID_CM);
  if (leftValid && rightValid) {
    result.bestAction = (result.leftDistanceCm >= result.rightDistanceCm) ? ACTION_LEFT : ACTION_RIGHT;
  } else if (leftValid) {
    result.bestAction = ACTION_LEFT;
  } else if (rightValid) {
    result.bestAction = ACTION_RIGHT;
  } else {
    result.bestAction = ACTION_BACKWARD;
  }
  Serial.print("Radar snapshot: left=");
  Serial.print(result.leftDistanceCm);
  Serial.print("cm right=");
  Serial.print(result.rightDistanceCm);
  Serial.print("cm best=");
  Serial.println(scanBestActionString(result));
  return result;
}
void setMotorAction(Action action) {
  // Low-level motor application path.
  // This is where trims, minimum useful PWM, restart boost, and state bookkeeping are applied.
  requestedAction = action;
  if (action == currentMotorAction && action != ACTION_STOP) {
    if (action != ACTION_FORWARD || forwardRestartBoostActive || adaptiveForwardBaseSpeed == lastAppliedForwardBaseSpeed) {
      return;
    }
  }
  Action previousAction = currentMotorAction;
  if (action != ACTION_FORWARD) {
    forwardRestartBoostActive = false;
  }
  int speedBase = (action == ACTION_FORWARD) ? adaptiveForwardBaseSpeed : BASE_SPEED;
  int rightSpeed = (int)(speedBase * RIGHT_TRIM);
  int leftSpeed = (int)(speedBase * LEFT_TRIM);
  if (action == ACTION_FORWARD) {
    if (motionCalibrationActive) {
      flushMotionCalibrationRecord("forward_resume");
    }
    lastForwardCommandBaseSpeed = speedBase;
    lastForwardCommandRightPwm = rightSpeed;
    lastForwardCommandLeftPwm = leftSpeed;
    if (rightSpeed < MIN_DRIVE_RIGHT_PWM) {
      rightSpeed = MIN_DRIVE_RIGHT_PWM;
    }
    if (leftSpeed < MIN_DRIVE_LEFT_PWM) {
      leftSpeed = MIN_DRIVE_LEFT_PWM;
    }
  } else if (action == ACTION_BACKWARD) {
    if (rightSpeed < MIN_REVERSE_RIGHT_PWM) {
      rightSpeed = MIN_REVERSE_RIGHT_PWM;
    }
    if (leftSpeed < MIN_REVERSE_LEFT_PWM) {
      leftSpeed = MIN_REVERSE_LEFT_PWM;
    }
  }
  if (action == ACTION_STOP) {
    resetTtcSequence();
    currentDriveMode = DRIVE_STOPPED;
  } else if (action == ACTION_FORWARD) {
    currentDriveMode = DRIVE_FORWARD;
  } else if (action == ACTION_BACKWARD) {
    resetTtcSequence();
    currentDriveMode = DRIVE_REVERSE;
  } else if (action == ACTION_LEFT) {
    resetTtcSequence();
    currentDriveMode = DRIVE_PIVOT_LEFT;
  } else {
    resetTtcSequence();
    currentDriveMode = DRIVE_PIVOT_RIGHT;
  }
  Serial.print("Applying request: ");
  Serial.print(actionToString(requestedAction));
  Serial.print(" | drive mode: ");
  Serial.println(driveModeString(currentDriveMode));
  switch (action) {
    case ACTION_FORWARD:
      if (previousAction == ACTION_LEFT || previousAction == ACTION_RIGHT || previousAction == ACTION_BACKWARD) {
        int boostRight = clampPwm(FORWARD_BOOST_RIGHT_PWM);
        int boostLeft = clampPwm(FORWARD_BOOST_LEFT_PWM);
        Serial.print("Applying forward restart boost: right=");
        Serial.print(boostRight);
        Serial.print(" left=");
        Serial.println(boostLeft);
        robot.MoveBalanced(Forward, boostRight, boostLeft);
        forwardRestartBoostActive = true;
        forwardRestartBoostUntilMs = millis() + FORWARD_RESTART_BOOST_MS;
        lastAppliedForwardBaseSpeed = adaptiveForwardBaseSpeed;
      } else {
        robot.MoveBalanced(Forward, rightSpeed, leftSpeed);
        lastAppliedForwardBaseSpeed = adaptiveForwardBaseSpeed;
      }
      break;
    case ACTION_BACKWARD:
      robot.MoveBalanced(Backward, rightSpeed, leftSpeed);
      break;
    case ACTION_LEFT:
      robot.MoveBalanced(Move_Left, rightSpeed, leftSpeed);
      break;
    case ACTION_RIGHT:
      robot.MoveBalanced(Move_Right, rightSpeed, leftSpeed);
      break;
    default:
      robot.Move(Stop, 0);
      currentMotorAction = ACTION_STOP;
        currentDriveMode = DRIVE_STOPPED;
      return;
  }
  currentMotorAction = action;
  if (action != ACTION_STOP) {
    lastValidMotorCommandMs = millis();
  }
  lastAction = action;
  recordAction(action);
}
bool applyMotorAction(Action action, const NavigationSnapshot &snapshot) {
  // Final safety wrapper before touching the motors.
  // Even a chosen maneuver is revalidated against the freshest local scan.
  SideScanResult safetyScan = getRadarScanSnapshot();
  Action approvedAction = validateSafeAction(action, snapshot, safetyScan);
  if (approvedAction != action) {
    Serial.print("Motor safety wrapper adjusted ");
    Serial.print(actionToString(action));
    Serial.print(" -> ");
    Serial.println(actionToString(approvedAction));
  }
  setMotorAction(approvedAction);
  return approvedAction == action;
}
void updateForwardRestartBoost(unsigned long nowMs) {
  // After a turn or reverse, briefly overdrive forward PWM to overcome drivetrain stiction,
  // then fall back to the adaptive cruise PWM once movement is re-established.
  if (!forwardRestartBoostActive) {
    return;
  }
  if ((long)(nowMs - forwardRestartBoostUntilMs) < 0) {
    return;
  }
  if (currentMotorAction != ACTION_FORWARD) {
    forwardRestartBoostActive = false;
    return;
  }
  int rightSpeed = (int)(BASE_SPEED * RIGHT_TRIM);
  int leftSpeed = (int)(BASE_SPEED * LEFT_TRIM);
  rightSpeed = (int)(adaptiveForwardBaseSpeed * RIGHT_TRIM);
  leftSpeed = (int)(adaptiveForwardBaseSpeed * LEFT_TRIM);
  NavigationSnapshot snapshot = buildNavigationSnapshot(nowMs);
  SideScanResult safetyScan = getRadarScanSnapshot();
  Action safeAction = validateSafeAction(ACTION_FORWARD, snapshot, safetyScan);
  if (safeAction != ACTION_FORWARD) {
    forwardRestartBoostActive = false;
    applyMotorAction(safeAction, snapshot);
    return;
  }
  if (rightSpeed < MIN_DRIVE_RIGHT_PWM) {
    rightSpeed = MIN_DRIVE_RIGHT_PWM;
  }
  if (leftSpeed < MIN_DRIVE_LEFT_PWM) {
    leftSpeed = MIN_DRIVE_LEFT_PWM;
  }
  Serial.println("Forward restart boost complete; applying cruise forward PWM");
  robot.MoveBalanced(Forward, rightSpeed, leftSpeed);
  forwardRestartBoostActive = false;
  lastAppliedForwardBaseSpeed = adaptiveForwardBaseSpeed;
}
void applySteeringDrive(int direction, float rightScale, float leftScale) {
  int rightSpeed = (int)(BASE_SPEED * RIGHT_TRIM * rightScale);
  int leftSpeed = (int)(BASE_SPEED * LEFT_TRIM * leftScale);
  if (rightSpeed < 0) {
    rightSpeed = 0;
  }
  if (leftSpeed < 0) {
    leftSpeed = 0;
  }
  float maxScale = rightScale > leftScale ? rightScale : leftScale;
  bool rightIsOuter = (rightScale == maxScale);
  if (rightIsOuter) {
    if (rightSpeed < MIN_STEER_OUTER_PWM) {
      rightSpeed = MIN_STEER_OUTER_PWM;
    }
    if (leftSpeed < MIN_STEER_INNER_PWM) {
      leftSpeed = MIN_STEER_INNER_PWM;
    }
  } else {
    if (leftSpeed < MIN_STEER_OUTER_PWM) {
      leftSpeed = MIN_STEER_OUTER_PWM;
    }
    if (rightSpeed < MIN_STEER_INNER_PWM) {
      rightSpeed = MIN_STEER_INNER_PWM;
    }
  }
  robot.MoveBalanced(direction, rightSpeed, leftSpeed);
}
void setSteeringManeuver(Action action, bool reverseDrive) {
  // Steering maneuvers are short pulses, not open-ended turns.
  // They let the rover re-check forward clearance between pulses and stop early if needed.
  requestedAction = action;
  DriveMode proposedDriveMode = reverseDrive
      ? DRIVE_REVERSE
      : (action == ACTION_LEFT ? DRIVE_PIVOT_LEFT : DRIVE_PIVOT_RIGHT);
  resetTtcSequence();
  currentDriveMode = proposedDriveMode;
  Serial.print("Applying steering request: ");
  Serial.print(actionToString(requestedAction));
  Serial.print(" | drive mode: ");
  Serial.println(driveModeString(currentDriveMode));
  int driveDirection = reverseDrive ? Backward : Forward;
  unsigned long nowMs = millis();
  NavigationSnapshot snapshot = buildNavigationSnapshot(nowMs);
  SideScanResult safetyScan = getRadarScanSnapshot();
  bool frontBlocked = snapshot.frontBlocked;
  if (frontBlocked && driveModeHasForwardMotion(proposedDriveMode)) {
    Serial.println("Safety override: rejecting steering mode with forward motion while front blocked");
    if (reverseRecoveryPermitted(escapeAttemptStage)) {
      applyMotorAction(ACTION_BACKWARD, snapshot);
    } else {
      setMotorAction(ACTION_STOP);
    }
    return;
  }
  if (action == ACTION_LEFT) {
    applySteeringDrive(driveDirection, STEER_OUTER_SCALE, STEER_INNER_SCALE);
  } else {
    applySteeringDrive(driveDirection, STEER_INNER_SCALE, STEER_OUTER_SCALE);
  }
  currentMotorAction = action;
  requestedAction = action;
  lastValidMotorCommandMs = millis();
  lastAction = action;
  recordAction(action);
}
void startManeuver(Action maneuverAction, unsigned long durationMs, Action resumeAction) {
  maneuverResumeAction = resumeAction;
  maneuverUntilMs = millis() + durationMs;
  roverState = STATE_MANEUVERING;
  maneuverUsesSteering = false;
  activeManeuverAction = maneuverAction;
  NavigationSnapshot snapshot = buildNavigationSnapshot(millis());
  applyMotorAction(maneuverAction, snapshot);
}
void startSteeringManeuver(Action maneuverAction, unsigned long durationMs, Action resumeAction, bool reverseDrive) {
  maneuverResumeAction = resumeAction;
  maneuverUntilMs = millis() + durationMs;
  roverState = STATE_MANEUVERING;
  maneuverUsesSteering = true;
  activeManeuverAction = maneuverAction;
  if (turnPulsesExecuted == 1) {
    turnSequenceStartCm = estimateFrontClearanceCm();
    turnSequenceStartedMs = millis();
  }
  turnReverseDrive = reverseDrive;
  setSteeringManeuver(maneuverAction, reverseDrive);
}
void beginDecisionManeuver(Action decision) {
  // Translate a high-level decision into a bounded maneuver sequence.
  // Left/right become pulsed steering turns; backward becomes a timed reverse; forward resumes roaming.
  NavigationSnapshot snapshot = buildNavigationSnapshot(millis());
  switch (decision) {
    case ACTION_LEFT:
      Serial.println("Executing pulsed left steering turn until clearance improves");
      turnPulsesExecuted = 1;
      turnCheckPending = false;
      turnFrontReadingReady = false;
      startSteeringManeuver(ACTION_LEFT, TURN_PULSE_MS, ACTION_FORWARD, false);
      break;
    case ACTION_RIGHT:
      Serial.println("Executing pulsed right steering turn until clearance improves");
      turnPulsesExecuted = 1;
      turnCheckPending = false;
      turnFrontReadingReady = false;
      startSteeringManeuver(ACTION_RIGHT, TURN_PULSE_MS, ACTION_FORWARD, false);
      break;
    case ACTION_BACKWARD:
      Serial.println("Executing bounded reverse escape");
      applyMotorAction(ACTION_BACKWARD, snapshot);
      startManeuver(ACTION_BACKWARD, EMERGENCY_REVERSE_DURATION_MS, ACTION_STOP);
      break;
    case ACTION_FORWARD:
    default:
      roverState = STATE_ROAMING;
      applyMotorAction(ACTION_FORWARD, snapshot);
      break;
  }
}
void updateManeuverState(unsigned long now) {
  // Advance or finish the active timed maneuver.
  // Turn pulses are always followed by a forward rescan before the next pulse or a resume decision.
  if (roverState != STATE_MANEUVERING) {
    return;
  }
  if (currentMotorAction != ACTION_STOP) {
    lastValidMotorCommandMs = now;
  }
  if ((long)(now - maneuverUntilMs) < 0) {
    return;
  }
  if (activeManeuverAction == ACTION_LEFT || activeManeuverAction == ACTION_RIGHT) {
    if (turnSequenceStartedMs != 0 && (now - turnSequenceStartedMs) >= MAX_ESCAPE_TURN_TOTAL_MS) {
      Serial.println("Turn sequence timed out; advancing escape stage to limit repeat pulses");
      float abortedFrontCm = estimateFrontClearanceCm();
      if (escapeAttemptStage < ESCAPE_ATTEMPT_TRAPPED) {
        escapeAttemptStage = nextEscapeStage(escapeAttemptStage);
      }
      finalizeTurnSequenceOutcome(false, abortedFrontCm, now);
      maneuverUsesSteering = false;
      activeManeuverAction = ACTION_STOP;
      roverState = STATE_HAZARD;
      emergencyStop("invalid manoeuvre state");
      lastObstacleDecisionMs = 0;
      return;
    }
    if (!turnCheckPending) {
      Serial.println("Turn pulse complete, stopping and rescanning front");
      emergencyStop("invalid manoeuvre state");
      turnCheckPending = true;
      turnFrontReadingReady = false;
      ultrasonicScanState = SCAN_TURN_MOVE_FRONT;
      ultrasonicScanStateMs = now;
      return;
    }
    if (!turnFrontReadingReady) {
      return;
    }
    float improvedFrontCm = turnLatestFrontCm;
    turnFrontReadingReady = false;
    turnCheckPending = false;
    bool improved =
        (improvedFrontCm >= ULTRASONIC_MIN_VALID_CM && turnSequenceStartCm >= ULTRASONIC_MIN_VALID_CM) &&
        (improvedFrontCm >= turnSequenceStartCm + TURN_CLEARANCE_IMPROVEMENT_CM ||
         improvedFrontCm >= HAZARD_CLEAR_CM);
    if (improved) {
      Serial.print("Turn clearance improved: baseline=");
      Serial.print(turnSequenceStartCm);
      Serial.print("cm current=");
      Serial.print(improvedFrontCm);
      Serial.println("cm -> moving forward");
      finalizeTurnSequenceOutcome(true, improvedFrontCm, now);
      maneuverUsesSteering = false;
      activeManeuverAction = ACTION_STOP;
      roverState = STATE_ROAMING;
      applyMotorAction(ACTION_FORWARD, buildNavigationSnapshot(now));
      hazardClearArmed = true;
      return;
    }
    if (turnPulsesExecuted < TURN_MAX_PULSES) {
      turnPulsesExecuted++;
      Serial.print("Turn clearance unchanged; continuing pulse ");
      Serial.println(turnPulsesExecuted);
      maneuverUntilMs = now + TURN_PULSE_MS;
      maneuverUsesSteering = true;
      setSteeringManeuver(activeManeuverAction, turnReverseDrive);
      return;
    }
    if (escapeAttemptStage < ESCAPE_ATTEMPT_TRAPPED) {
      escapeAttemptStage = nextEscapeStage(escapeAttemptStage);
    }
    Serial.print("Turn clearance did not improve; advancing escape stage to ");
    Serial.println(escapeAttemptStageString(escapeAttemptStage));
    finalizeTurnSequenceOutcome(false, improvedFrontCm, now);
    maneuverUsesSteering = false;
    activeManeuverAction = ACTION_STOP;
    roverState = STATE_HAZARD;
    emergencyStop("invalid manoeuvre state");
    lastObstacleDecisionMs = 0;
    return;
  }
  if (activeManeuverAction == ACTION_BACKWARD) {
    Serial.println("Reverse escape complete, stopping for fresh scan");
    emergencyStop("invalid manoeuvre state");
    maneuverUsesSteering = false;
    activeManeuverAction = ACTION_STOP;
    roverState = STATE_REVERSE_RESCAN;
    resetTurnSequenceState();
    ultrasonicScanState = SCAN_MOVE_FRONT;
    ultrasonicScanStateMs = now;
    lastObstacleDecisionMs = 0;
    hazardClearArmed = !obstacleNearby;
    return;
  }
  Serial.println("Maneuver complete, checking distance before resuming forward");
  DirectionalReading postTurnReading = readDirectionalUltrasonic(panCurrentDeg);
  bool immediatePostTurnHazard = applyImmediatePostManeuverHazardCheck(postTurnReading);
  bool validPostTurnReading = postTurnReading.valid;
  maneuverUsesSteering = false;
  activeManeuverAction = ACTION_STOP;
  if (!validPostTurnReading) {
    Serial.println("Post-turn safety check: invalid/no echo reading, holding position");
    emergencyStop("invalid sensor state");
    roverState = STATE_HAZARD;
    lastObstacleDecisionMs = 0;
    hazardClearArmed = false;
    return;
  }
  if (immediatePostTurnHazard || obstacleNearby) {
    Serial.println("Post-turn safety check: obstacle still nearby, holding position");
    emergencyStop("critical hazard");
    roverState = STATE_HAZARD;
    lastObstacleDecisionMs = 0;
    hazardClearArmed = false;
    return;
  }
  Serial.println("Post-turn safety check: clear, resuming forward");
  roverState = STATE_ROAMING;
  setMotorAction(maneuverResumeAction);
  hazardClearArmed = true;
}
float readUltrasonicCm() {
  // Driver wrapper around the sensor library with serial diagnostics for both successful reads and no-echo cases.
  float distance = sensor.Ranging();
  unsigned long pulseWidthUs = sensor.LastPulseWidthUs();
  if (distance < 0.0f) {
    Serial.print("Ultrasonic: no echo | pulseWidthUs=");
    Serial.println(pulseWidthUs);
    return -1.0f;
  }
  Serial.print("Ultrasonic: ");
  Serial.print(distance);
  Serial.print(" cm | pulseWidthUs=");
  Serial.println(pulseWidthUs);
  return distance;
}
bool updateHazardFromDistance(float distanceCm) {
  // Hazard hysteresis keeps the rover from chattering between clear/blocked on noisy edge readings.
  // Entering a hazard needs repeated close samples; clearing it needs repeated safe samples.
  bool validDistance = (distanceCm >= ULTRASONIC_MIN_VALID_CM);
  if (!validDistance) {
    if (!obstacleNearby) {
      hazardSampleCount = 0;
    }
    return obstacleNearby;
  }
  if (obstacleNearby) {
    if (distanceCm >= HAZARD_CLEAR_CM) {
      if (hazardSampleCount < 255) {
        hazardSampleCount++;
      }
      if (hazardSampleCount >= HAZARD_CLEAR_SAMPLES) {
        hazardSampleCount = 0;
        return false;
      }
    } else {
      hazardSampleCount = 0;
    }
    return true;
  }
  if (distanceCm <= HAZARD_ENTER_CM) {
    if (hazardSampleCount < 255) {
      hazardSampleCount++;
    }
    if (hazardSampleCount >= HAZARD_ENTER_SAMPLES) {
      hazardSampleCount = 0;
      return true;
    }
  } else {
    hazardSampleCount = 0;
  }
  return false;
}
void connectWiFi() {
  // Wi-Fi is optional for core driving; lack of connectivity only disables Gemini-assisted decisions.
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
    printHeapDiagnostics("after Wi-Fi connection");
  } else {
    Serial.println("WiFi not connected. Gemini decisions will fallback locally.");
  }
}
Action chooseLocalFallback(float frontCm, const SideScanResult &scan, Action physicallyBlockedAction,
                           Action discouragedOscillationAction) {
  SafeActionSet safeActions = buildSafeActionSet(frontCm, scan);
  return chooseBestActionFromSafeSet(safeActions, physicallyBlockedAction, discouragedOscillationAction);
}
Action queryGeminiForObstacleTurnBlocking(const NavigationSnapshot &snapshot, Action previousAction,
                                          const SideScanResult &scan, GeminiPromptTemplate templateType) {
  // Build a tightly-scoped prompt from the freshest navigation snapshot.
  // The model is treated as a constrained adviser: it must pick exactly one action from the local safe set.
  Action physicallyBlockedAction = ACTION_FORWARD;
  Action discouragedOscillationAction = oppositeAction(previousAction);
  if (WiFi.status() != WL_CONNECTED) {
    return validateSafeAction(chooseLocalFallback(snapshot.frontCm, scan, physicallyBlockedAction,
                                                   discouragedOscillationAction),
                              snapshot, scan);
  }
  WiFiClientSecure secureClient;
  // TODO: prototype-only transport; replace with certificate validation before production use.
  secureClient.setInsecure();
  HTTPClient http;
  // TODO: prototype-only key handling; move the API key out of the query string before production use.
  String url = String(GEMINI_URL) + "?key=" + String(GEMINI_API_KEY);
  if (!http.begin(secureClient, url)) {
    Serial.println("Gemini: HTTP begin failed");
    return validateSafeAction(chooseLocalFallback(snapshot.frontCm, scan, physicallyBlockedAction,
                                                   discouragedOscillationAction),
                              snapshot, scan);
  }
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(GEMINI_HTTP_TIMEOUT_MS);
  String recentActions = recentActionsString(6);
  String recentOutcomes = recentManeuverOutcomesString(4);
  const char *currentTurnDirection = currentTurnDirectionString(previousAction);
  const char *bestFromScan = scanBestActionString(scan);
  const char *hazardClass = hazardClassString(snapshot.frontCm);
  const char *envHint = environmentHintString(snapshot.frontCm, scan);
  const char *requestedMotion = actionToString(requestedAction);
  const char *currentDriveModeText = driveModeString(currentDriveMode);
  const char *escapeStage = escapeAttemptStageString(escapeAttemptStage);
  const char *templateName = geminiPromptTemplateString(templateType);
  const char *blockedAction = actionToString(physicallyBlockedAction);
  const char *discouragedAction = actionToString(discouragedOscillationAction);
  SafeActionSet safeActions = buildSafeActionSet(snapshot, scan);
  Action localRecommendation = chooseBestActionFromSafeSet(safeActions, physicallyBlockedAction, discouragedOscillationAction);
  String safeActionList = safeActionSetString(safeActions);
  const char *localRecommendationText = actionToString(localRecommendation);
  DynamicJsonDocument requestDoc(4096);
  char prompt[2400];
  size_t promptLen = 0;
  auto appendPrompt = [&](const char *format, ...) {
    if (promptLen >= sizeof(prompt)) {
      return;
    }
    va_list args;
    va_start(args, format);
    int written = vsnprintf(prompt + promptLen, sizeof(prompt) - promptLen, format, args);
    va_end(args);
    if (written < 0) {
      promptLen = sizeof(prompt) - 1;
      prompt[promptLen] = '\0';
      return;
    }
    if ((size_t)written >= sizeof(prompt) - promptLen) {
      promptLen = sizeof(prompt) - 1;
    } else {
      promptLen += (size_t)written;
    }
  };

  GeminiGateResult gate = evaluateGeminiGate(snapshot, scan, previousAction, templateType);
  if (!gate.shouldCall) {
    if (gate.reason != nullptr) {
      Serial.println(gate.reason);
    }
    return gate.localRecommendation;
  }

  appendPrompt("You are the navigation controller for a small autonomous robot car.\n");
  appendPrompt("Robot context:\n");
  appendPrompt("- Floor-level robot, common obstacles: walls, boxes, table legs\n");
  appendPrompt("- No map, no camera, no GPS\n");
  appendPrompt("- Use freshest distances and avoid oscillation loops\n");
  appendPrompt("- navigation_state=%s\n\n", templateName);
  if (templateType == GEMINI_PROMPT_STANDARD_OBSTACLE) {
    appendPrompt("Prompt type: standard obstacle\n");
    appendPrompt("Use this when the rover is avoiding an obstacle but has not yet failed a maneuver.\n\n");
    appendPrompt("Decision policy:\n");
    appendPrompt("1) Safety first: CRITICAL/NEAR hazard must avoid forward.\n");
    appendPrompt("2) Prefer the more open side from scans and environment_hint.\n");
    appendPrompt("3) Avoid immediate retraction; do not choose blocked_action.\n");
    appendPrompt("4) If confidence is low, choose conservative escape (LEFT/RIGHT/BACKWARD).\n");
  } else if (templateType == GEMINI_PROMPT_FAILED_MANEUVER) {
    appendPrompt("Prompt type: failed manoeuvre\n");
    appendPrompt("Use this when a prior turn or reverse did not improve clearance enough.\n\n");
    appendPrompt("Decision policy:\n");
    appendPrompt("1) Safety first: CRITICAL/NEAR hazard must avoid forward.\n");
    appendPrompt("2) Prefer the action that changes clearance rather than repeating the failed one.\n");
    appendPrompt("3) Use recent maneuver outcomes to avoid repeating a failure pattern.\n");
    appendPrompt("4) If confidence is low, choose the safest escape alternative from safe_actions.\n");
  } else {
    appendPrompt("Prompt type: trapped recovery\n");
    appendPrompt("Use this when the rover has exhausted normal escape stages and is likely trapped.\n\n");
    appendPrompt("Decision policy:\n");
    appendPrompt("1) Safety first: never choose forward if the front is blocked.\n");
    appendPrompt("2) Prefer the safest recovery action that is still inside safe_actions.\n");
    appendPrompt("3) If the scene is still ambiguous, choose the most conservative escape alternative.\n");
    appendPrompt("4) Do not invent a new maneuver outside safe_actions.\n");
  }
  appendPrompt("\nRespond with exactly one JSON object only.\n");
  appendPrompt("Format: {\"action\":\"LEFT\"}\n");
  appendPrompt("Choose exactly ONE action from safe_actions and use it as the value.\n");
  appendPrompt("Do not add explanation, markdown, code fences, or extra keys.\n");
  appendPrompt("Output only the JSON object and nothing else.\n\n");
  appendPrompt("State (compact):\n");
  appendPrompt("front_cm=%.1f\n", snapshot.frontCm);
  appendPrompt("left_cm=%.1f\n", snapshot.leftCm);
  appendPrompt("right_cm=%.1f\n", snapshot.rightCm);
  appendPrompt("hazard_class=%s\n", hazardClass);
  appendPrompt("environment_hint=%s\n", envHint);
  appendPrompt("sensor_confidence=%d\n", snapshot.confidence);
  appendPrompt("front_age_ms=%lu\n", snapshot.frontAgeMs);
  appendPrompt("left_age_ms=%lu\n", snapshot.leftAgeMs);
  appendPrompt("right_age_ms=%lu\n", snapshot.rightAgeMs);
  appendPrompt("front_valid=%d\n", snapshot.frontValid ? 1 : 0);
  appendPrompt("left_valid=%d\n", snapshot.leftValid ? 1 : 0);
  appendPrompt("right_valid=%d\n", snapshot.rightValid ? 1 : 0);
  appendPrompt("front_blocked=%d\n", snapshot.frontBlocked ? 1 : 0);
  appendPrompt("no_echo_streak=%u\n", ultrasonicNoEchoStreak);
  appendPrompt("recent_actions=%s\n", recentActions.c_str());
  appendPrompt("recent_maneuver_outcomes=%s\n", recentOutcomes.c_str());
  appendPrompt("escape_attempt_stage=%s\n", escapeStage);
  appendPrompt("requested_action=%s\n", requestedMotion);
  appendPrompt("current_drive_mode=%s\n", currentDriveModeText);
  appendPrompt("current_turn_direction=%s\n", currentTurnDirection);
  appendPrompt("physically_blocked_action=%s\n", blockedAction);
  appendPrompt("discouraged_oscillation_action=%s\n", discouragedAction);
  appendPrompt("safe_actions=%s\n", safeActionList.c_str());
  appendPrompt("local_recommendation=%s\n", localRecommendationText);
  appendPrompt("left_scan_cm=%.1f\n", scan.leftDistanceCm);
  appendPrompt("right_scan_cm=%.1f\n", scan.rightDistanceCm);
  appendPrompt("best_scan_action=%s\n", bestFromScan);
  JsonObject content = requestDoc["contents"].createNestedObject();
  JsonArray parts = content["parts"].to<JsonArray>();
  JsonObject part = parts.createNestedObject();
  part["text"] = prompt;
  JsonObject generationConfig = requestDoc["generationConfig"].to<JsonObject>();
  generationConfig["temperature"] = 0.2;
  generationConfig["maxOutputTokens"] = GEMINI_MAX_OUTPUT_TOKENS;
  String requestBody;
  serializeJson(requestDoc, requestBody);
  int statusCode = http.POST(requestBody);
  printHeapDiagnostics("after first TLS request");
  String responseBody = http.getString();
  http.end();
  if (statusCode < 200 || statusCode >= 300) {
    Serial.print("Gemini HTTP error: ");
    Serial.println(statusCode);
    return validateSafeAction(chooseLocalFallback(snapshot.frontCm, scan, physicallyBlockedAction,
                                                   discouragedOscillationAction),
                              snapshot, scan);
  }
  StaticJsonDocument<2048> responseDoc;
  DeserializationError err = deserializeJson(responseDoc, responseBody);
  if (err) {
    Serial.print("Gemini JSON parse error: ");
    Serial.println(err.c_str());
    return validateSafeAction(chooseLocalFallback(snapshot.frontCm, scan, physicallyBlockedAction,
                                                   discouragedOscillationAction),
                              snapshot, scan);
  }
  printHeapDiagnostics("after Gemini response parsing");
  JsonArray candidates = responseDoc["candidates"].as<JsonArray>();
  if (candidates.isNull() || candidates.size() == 0) {
    Serial.println("Gemini response rejected: missing candidates");
    return localRecommendation;
  }
  JsonArray responseParts = candidates[0]["content"]["parts"].as<JsonArray>();
  if (responseParts.isNull() || responseParts.size() == 0) {
    Serial.println("Gemini response rejected: missing content parts");
    return localRecommendation;
  }
  const char *modelText = responseParts[0]["text"];
  if (modelText == nullptr || modelText[0] == '\0') {
    Serial.println("Gemini response rejected: missing text content");
    return localRecommendation;
  }
  ParsedAction parsedDecision = parseActionFromText(modelText);
  if (!parsedDecision.valid) {
    Serial.println("Gemini response rejected: malformed or unrecognized JSON action");
    return validateSafeAction(localRecommendation, snapshot, scan);
  }
  Action decision = parsedDecision.action;
  if (!actionIsInSafeSet(decision, safeActions)) {
    Serial.println("Gemini response rejected: action not in safe set");
    return localRecommendation;
  }
  if (decision == physicallyBlockedAction) {
    Serial.println("Gemini proposed retraction; selecting fallback instead");
    return validateSafeAction(localRecommendation, snapshot, scan);
  }
  decision = chooseSmartAction(decision, physicallyBlockedAction, discouragedOscillationAction);
  decision = validateSafeAction(decision, snapshot, scan);
  Serial.print("Decision source: Gemini | decision=");
  Serial.println(actionToString(decision));
  return decision;
}
void geminiDecisionWorkerTask(void *parameter) {
  // Background task wrapper so network latency never blocks loop().
  GeminiDecisionRequestArgs *request = static_cast<GeminiDecisionRequestArgs *>(parameter);
  Action decision = queryGeminiForObstacleTurnBlocking(request->snapshot, request->previousAction, request->scan,
                                                       request->templateType);
  if (geminiDecisionResultQueue != nullptr) {
    GeminiDecisionResultMessage result;
    result.generation = request->generation;
    result.decision = decision;
    xQueueOverwrite(geminiDecisionResultQueue, &result);
  }
  geminiDecisionTaskHandle = nullptr;
  delete request;
  vTaskDelete(nullptr);
}
bool startGeminiDecisionRequest(const NavigationSnapshot &snapshot, Action previousAction, const SideScanResult &scan,
                                GeminiPromptTemplate templateType) {
  // Fire exactly one asynchronous Gemini request at a time and remember the local fallback
  // that should be used if the request later fails or times out.
  if (geminiDecisionResultQueue == nullptr) {
    return false;
  }
  if (geminiDecisionTaskHandle != nullptr) {
    return false;
  }
  if (geminiDecisionRequestState == GEMINI_REQUEST_PENDING) {
    return false;
  }
  if (geminiDecisionRequestState == GEMINI_REQUEST_TIMED_OUT) {
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
  GeminiDecisionRequestArgs *request = new GeminiDecisionRequestArgs;
  if (request == nullptr) {
    return false;
  }
  request->snapshot = snapshot;
  request->previousAction = previousAction;
  request->scan = scan;
  request->templateType = templateType;
  request->generation = ++geminiRequestGeneration;
  geminiDecisionFallbackAction = chooseLocalEscapeActionForStage(snapshot, scan, previousAction);
  geminiDecisionRequestStartedMs = millis();
  geminiDecisionRequestState = GEMINI_REQUEST_PENDING;
  GeminiDecisionResultMessage staleResult;
  while (xQueueReceive(geminiDecisionResultQueue, &staleResult, 0) == pdPASS) {
  }
  BaseType_t taskCreated = xTaskCreatePinnedToCore(geminiDecisionWorkerTask, "GeminiDecision", 8192, request, 1,
                                                   const_cast<TaskHandle_t *>(&geminiDecisionTaskHandle), 1);
  if (taskCreated != pdPASS) {
    geminiDecisionRequestState = GEMINI_REQUEST_FAILED;
    geminiDecisionTaskHandle = nullptr;
    geminiRequestGeneration = request->generation;
    delete request;
    return false;
  }
  printHeapDiagnostics("after Gemini task creation");
  return true;
}
bool updateGeminiDecisionRequest(unsigned long nowMs, Action &decisionOut) {
  // Poll the async request state without blocking.
  // If the reply is late, the main loop marks the request timed out and stays on local logic.
  GeminiDecisionResultMessage result;
  while (geminiDecisionRequestState == GEMINI_REQUEST_PENDING && geminiDecisionResultQueue != nullptr &&
         xQueueReceive(geminiDecisionResultQueue, &result, 0) == pdPASS) {
    if (result.generation != geminiRequestGeneration) {
      continue;
    }
    decisionOut = result.decision;
    geminiDecisionRequestState = GEMINI_REQUEST_READY;
    break;
  }
  if (geminiDecisionRequestState == GEMINI_REQUEST_READY) {
    geminiDecisionRequestState = GEMINI_REQUEST_IDLE;
    return true;
  }
  if (geminiDecisionRequestState == GEMINI_REQUEST_TIMED_OUT) {
    if (geminiDecisionTaskHandle == nullptr) {
      if (geminiDecisionResultQueue != nullptr) {
        while (xQueueReceive(geminiDecisionResultQueue, &result, 0) == pdPASS) {
        }
      }
      geminiDecisionRequestState = GEMINI_REQUEST_IDLE;
    }
    return false;
  }
  if (geminiDecisionRequestState != GEMINI_REQUEST_PENDING) {
    return false;
  }
  if ((nowMs - geminiDecisionRequestStartedMs) >= GEMINI_DECISION_REQUEST_TIMEOUT_MS) {
    Serial.println("Gemini async timeout: using local decision");
    decisionOut = geminiDecisionFallbackAction;
    geminiDecisionRequestState = GEMINI_REQUEST_TIMED_OUT;
    return false;
  }
  return false;
}
void setup() {
  // One-time initialization sequence:
  // serial logging, Gemini result queue, buzzer, motors, ultrasonic sensor, pan servo,
  // Wi-Fi, and an initial front reading that seeds the first hazard-aware scan cycle.
  Serial.begin(115200);
  randomSeed((unsigned long)esp_random());
  printHeapDiagnostics("startup");
  geminiDecisionResultQueue = xQueueCreate(1, sizeof(GeminiDecisionResultMessage));
  if (geminiDecisionResultQueue == nullptr) {
    Serial.println("Gemini result queue alloc failed");
  }
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_HIGH ? LOW : HIGH);
  robot.Init();
  currentMotorAction = ACTION_FORWARD;
  setMotorAction(ACTION_STOP);
  delay(STARTUP_MOTOR_STABILIZE_MS);
  setMotorAction(ACTION_STOP);
  sensor.Init(TRIG_PIN, ECHO_PIN);
  ultrasonicPanServo.setPeriodHertz(50);
  int panAttachChannel = ultrasonicPanServo.attach(ULTRASONIC_PAN_PIN, 500, 2400);
  panServoReady = ultrasonicPanServo.attached();
  Serial.print("Pan attach result: channel=");
  Serial.print(panAttachChannel);
  Serial.print(" attached=");
  Serial.println(panServoReady ? "1" : "0");
  if (!panServoReady) {
    Serial.println("Pan attach retry with default pulse range");
    panAttachChannel = ultrasonicPanServo.attach(ULTRASONIC_PAN_PIN);
    panServoReady = ultrasonicPanServo.attached();
    Serial.print("Pan attach retry result: channel=");
    Serial.print(panAttachChannel);
    Serial.print(" attached=");
    Serial.println(panServoReady ? "1" : "0");
  }
  if (panServoReady) {
    runPanServoSelfTest();
    setPanAngle(PAN_FORWARD_DEG);
    panSweepEnabled = false;
    ultrasonicScanState = SCAN_IDLE;
    ultrasonicScanStateMs = millis();
    lastPanStepMs = millis();
    if (SERVO_SWEEP_DEBUG_ONLY) {
      Serial.println("SERVO SWEEP DEBUG MODE: main rover logic paused");
      Serial.println("Controlled stationary sampling enabled for validation");
    } else {
      Serial.println("Ultrasonic pan servo initialized; controlled stationary sampling enabled");
    }
  } else {
    Serial.println("Safety fault: pan servo unavailable");
    Serial.println("No-servo safe mode: staying stopped");
    panSweepEnabled = false;
    ultrasonicScanState = SCAN_IDLE;
    ultrasonicScanStateMs = millis();
    obstacleNearby = true;
    roverState = STATE_HAZARD;
    startupFrontScanPending = false;
    emergencyStop("invalid sensor state");
  }
  if (SERVO_SWEEP_DEBUG_ONLY) {
    return;
  }
  Serial.println("ESP32 Rover: random roam + Gemini obstacle decisions");
  startBuzzer(150);
  connectWiFi();
  DirectionalReading initialReading = readDirectionalUltrasonic(panCurrentDeg);
  lastDistanceCm = initialReading.distanceCm;
  currentFrontDistanceCm = initialReading.distanceCm;
  previousFrontDistanceCm = initialReading.distanceCm;
  radarFrontCm = initialReading.distanceCm;
  radarFrontUpdatedMs = initialReading.capturedMs;
  radarFrontSampleConfidence = initialReading.valid ? 1 : 0;
  obstacleNearby = true;
  roverState = STATE_HAZARD;
  startupFrontScanPending = true;
  ultrasonicScanState = SCAN_MOVE_FRONT;
  ultrasonicScanStateMs = millis();
  lastSensorMs = initialReading.capturedMs;
  lastObstacleDecisionMs = 0;
  lastValidMotorCommandMs = millis();
  lastControlHeartbeatMs = millis();
}
void loop() {
  // Main cooperative control loop.
  // Priority order is deliberate:
  // 1) watchdog safety
  // 2) servo/sensor recovery
  // 3) active maneuver timing
  // 4) fresh sensor scanning
  // 5) obstacle decision making
  // 6) normal forward roaming
  unsigned long now = millis();
  if (currentMotorAction != ACTION_STOP &&
      (now - lastControlHeartbeatMs) > MOTOR_CONTROL_HEARTBEAT_TIMEOUT_MS) {
    emergencyMotorStop();
  }
  if (!panServoReady) {
    emergencyStop("invalid sensor state");
    roverState = STATE_HAZARD;
    obstacleNearby = true;
    refreshControlHeartbeat(now);
    return;
  }
  if (SERVO_SWEEP_DEBUG_ONLY) {
    updatePanSweepDebug(now);
    static unsigned long lastSweepLogMs = 0;
    if ((now - lastSweepLogMs) >= 1000) {
      lastSweepLogMs = now;
      Serial.print("Sweep debug: panReady=");
      Serial.print(panServoReady ? "1" : "0");
      Serial.print(" dir=");
      Serial.print(panSweepTowardLeft ? "L" : "R");
      Serial.print(" angle=");
      Serial.println(panCurrentDeg);
    }
    refreshControlHeartbeat(now);
    return;
  }
  updateBuzzer(now);
  updateForwardRestartBoost(now);
  if (panSweepRecoveryUntilMs != 0 && (long)(now - panSweepRecoveryUntilMs) >= 0) {
    panSweepRecoveryUntilMs = 0;
    Serial.println("No echo recovery complete: resuming scan");
  } else if (panSweepRecoveryUntilMs != 0) {
    refreshControlHeartbeat(now);
    return;
  }
  updateManeuverState(now);
  if (handleSensorRetryState(now)) {
    refreshControlHeartbeat(now);
    return;
  }
  if (!sensorRetryActive) {
    bool previousObstacle = obstacleNearby;
    if (updateStationaryUltrasonicSampling(now, previousObstacle)) {
      refreshControlHeartbeat(now);
      return;
    }
  }
  if (obstacleNearby) {
    if (roverState == STATE_MANEUVERING) {
      refreshControlHeartbeat(now);
      return;
    }
    if (escapeAttemptStage == ESCAPE_ATTEMPT_TRAPPED) {
      signalTrappedState(now);
      refreshControlHeartbeat(now);
      return;
    }
    if (now - lastObstacleDecisionMs >= OBSTACLE_DECISION_COOLDOWN_MS && roverState != STATE_DECIDING) {
      roverState = STATE_DECIDING;
    }
    if (roverState == STATE_DECIDING) {
      Serial.print("Obstacle nearby. Escape stage=");
      Serial.println(escapeAttemptStageString(escapeAttemptStage));
      setMotorAction(ACTION_STOP);
      NavigationSnapshot snapshot = buildNavigationSnapshot(now);
      SideScanResult scan = getRadarScanSnapshot();
      Action decision = ACTION_STOP;
      if (updateGeminiDecisionRequest(now, decision)) {
        beginDecisionManeuver(decision);
        lastObstacleDecisionMs = millis();
        refreshControlHeartbeat(now);
        return;
      }
      if (geminiDecisionRequestState == GEMINI_REQUEST_TIMED_OUT) {
        refreshControlHeartbeat(now);
        return;
      }
      if (geminiDecisionRequestState == GEMINI_REQUEST_PENDING) {
        refreshControlHeartbeat(now);
        return;
      }
      if (geminiDecisionRequestState == GEMINI_REQUEST_FAILED) {
        geminiDecisionRequestState = GEMINI_REQUEST_IDLE;
      }
      GeminiGateResult gate = evaluateGeminiGate(snapshot, scan, lastAction,
                                                 geminiPromptTemplateForStage(escapeAttemptStage));
      if (!gate.shouldCall) {
        if (gate.reason != nullptr) {
          Serial.println(gate.reason);
        }
        beginDecisionManeuver(gate.localRecommendation);
        lastObstacleDecisionMs = millis();
        refreshControlHeartbeat(now);
        return;
      }
      if (startGeminiDecisionRequest(snapshot, lastAction, scan,
                                     geminiPromptTemplateForStage(escapeAttemptStage))) {
        refreshControlHeartbeat(now);
        return;
      }
      decision = gate.localRecommendation;
      beginDecisionManeuver(decision);
      lastObstacleDecisionMs = millis();
    }
    refreshControlHeartbeat(now);
    return;
  }
  if (roverState == STATE_MANEUVERING) {
    refreshControlHeartbeat(now);
    return;
  }
  maybeResetEscapeAttemptStages(now);
  if (!startupFrontScanPending && roverState != STATE_ROAMING) {
    roverState = STATE_ROAMING;
  }
  if (!startupFrontScanPending && currentMotorAction == ACTION_STOP) {
    applyMotorAction(ACTION_FORWARD, buildNavigationSnapshot(now));
  }
  refreshControlHeartbeat(now);
}

