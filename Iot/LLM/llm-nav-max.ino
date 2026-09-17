/**
 * llm-nav-max.ino — Azure AI Foundry-assisted autonomous obstacle-avoidance
 *
 * An ESP32 robot car drives forward until its ultrasonic sensor (mounted on a pan
 * servo) detects an obstacle.  On each hazard event a two-tier decision pipeline
 * fires:
 *
 *   1. LOCAL PLANNER  – scores three candidate maneuvers (strafe, 90° turn, and
 *      backup+turn) using live L/F/R distance readings, front-clearance trend, and
 *      oscillation history.  If the result is "obvious" (high confidence, low risk,
 *      no repeated-trap condition) it is executed immediately without any network
 *      round-trip.
 *
 *   2. FOUNDRY ARBITER – when the local answer is ambiguous, or a repeated-trap is
 *      detected, a structured prompt is posted to an Azure AI Foundry Responses API
 *      endpoint.  The LLM names one candidate plan and returns a confidence score.
 *      The response is validated and sanitized; any failure falls back to the local
 *      plan.
 *
 * After every maneuver the robot re-scans, records the front-clearance delta, and
 * feeds plan quality (improved / worsened) back into the next decision cycle.
 *
 * Required libraries : ArduinoJson, ESP32Servo, HTTPClient, WiFi, WiFiClientSecure,
 *                      ultrasonic (custom), vehicle (custom)
 * Config headers     : foundry_config.h  – FOUNDRY_RESPONSES_URL, FOUNDRY_MODEL,
 *                                          FOUNDRY_API_KEY
 *                      wifi_config.h     – WIFI_SSID, WIFI_PASSWORD
 */
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
// ─── Hardware instances ──────────────────────────────────────────────────────
vehicle myCar;     // 4-wheel-drive chassis abstraction (forward, backward, strafe, turn)
ultrasonic sensor; // HC-SR04 ultrasonic distance sensor
Servo panServo;    // Servo motor that rotates the sensor for left / front / right scans

// ─── Coarse directional intent ───────────────────────────────────────────────
// A simplified direction value used by tie-breaking helpers that only need to
// know which side to favour, not the full maneuver detail of ManeuverType.
enum Action {
  ACTION_STOP = 0,
  ACTION_LEFT,
  ACTION_RIGHT,
  ACTION_BACKWARD,
};

// ─── Full maneuver type ──────────────────────────────────────────────────────
// All atomic robot motions.  A ManeuverPlan chains a primary and optional
// secondary maneuver, both drawn from this enum.
enum ManeuverType {
  MANEUVER_STOP = 0,       // Halt all motors
  MANEUVER_STRAFE_LEFT,    // Slide left (mecanum / holonomic lateral move)
  MANEUVER_STRAFE_RIGHT,   // Slide right
  MANEUVER_BACKWARD,       // Reverse
  MANEUVER_TURN_LEFT_90,   // Spin counter-clockwise ~90 degrees in place
  MANEUVER_TURN_RIGHT_90,  // Spin clockwise ~90 degrees in place
  MANEUVER_RESCAN,         // Stop and retake L/F/R distance readings
};

// ─── Hazard snapshot (saved at each obstacle event) ─────────────────────────
// A ring buffer of these records is used to detect when the robot keeps
// returning to the same stuck position — a "repeated trap".
struct HazardSnapshot {
  float leftCm;        // Left-side clearance (cm) at the time of the snapshot
  float frontCm;       // Front clearance (cm)
  float rightCm;       // Right-side clearance (cm)
  unsigned long atMs;  // Timestamp (millis()) when the snapshot was taken
};

// ─── Complete maneuver plan ──────────────────────────────────────────────────
// Produced by the local planner or the Azure AI Foundry arbiter and consumed
// by executePlan().  Carries both the motion instructions and the metadata
// (confidence, risk, trap flag) used by the decision logic.
struct ManeuverPlan {
  ManeuverType primary;           // First motion to execute
  uint16_t primaryDurationMs;     // Duration of the primary motion (ms)
  ManeuverType secondary;         // Follow-up motion (often RESCAN or STOP)
  uint16_t secondaryDurationMs;   // Duration of the secondary motion (ms)
  float confidence;               // Planner confidence in this plan (0–1)
  float riskScore;                // Estimated environmental risk (0 = safe, 1 = critical)
  bool repeatedTrap;              // True when a repeated-trap pattern was detected
};
// ─── Motor speeds (PWM range 0–255) ───────────────────────────────────────────────
const int FORWARD_SPEED = 190;              // Speed when driving straight ahead
const int TURN_SPEED = 240;                 // Speed during turns and strafes
const unsigned long TURN_90_DURATION_MS = 520; // Milliseconds required for a ~90° in-place spin

// ─── Hardware pin assignments ─────────────────────────────────────────────────
const int TRIG_PIN = 13;            // HC-SR04 trigger pin
const int ECHO_PIN = 14;            // HC-SR04 echo pin
const int ULTRASONIC_PAN_PIN = 27;  // PWM signal pin for the pan servo
const int LEFT_LED_PIN = 2;         // Left status LED
const int RIGHT_LED_PIN = 12;       // Right status LED
const bool LED_ACTIVE_HIGH = true;  // true = driving pin HIGH turns the LED on

// ─── Pan servo geometry ────────────────────────────────────────────────────────
// The pan servo sweeps the ultrasonic sensor between PAN_LEFT_DEG and
// PAN_RIGHT_DEG.  PAN_CENTER_TRIM_DEG compensates for servo mounting offset.
const int PAN_CENTER_TRIM_DEG = 0;  // Fine-tune servo centre for physical alignment
const int PAN_CENTER_DEG = 90 + PAN_CENTER_TRIM_DEG;  // Servo angle pointing straight ahead
const int PAN_HALF_ARC_DEG = 85;   // Half the total sweep arc (centre ± this value)
// Computed endpoint angles clamped to the valid servo range [0, 180]
const int PAN_LEFT_DEG = ((PAN_CENTER_DEG + PAN_HALF_ARC_DEG) > 180) ? 180 : (PAN_CENTER_DEG + PAN_HALF_ARC_DEG);
const int PAN_RIGHT_DEG = ((PAN_CENTER_DEG - PAN_HALF_ARC_DEG) < 0) ? 0 : (PAN_CENTER_DEG - PAN_HALF_ARC_DEG);
const int PAN_STEP_DEG = 5;                     // Degrees moved per step during continuous background sweep
const unsigned long PAN_STEP_INTERVAL_MS = 22;  // Time between sweep steps (ms)
const int PAN_CENTER_BUCKET_DEG = 18;           // Half-width of the "front" zone (± degrees from centre)

// ─── Distance thresholds (cm) ───────────────────────────────────────────────────
const float ULTRASONIC_ALERT_CM = 45.0f;         // Distance that sets obstacleNearby = true
const float ULTRASONIC_CLEAR_CM = 50.0f;         // Distance required to clear obstacleNearby (hysteresis)
const float ULTRASONIC_MIN_VALID_CM = 4.0f;      // Readings below this are treated as noise / no-echo
const float EMERGENCY_REVERSE_CM = 18.0f;        // Critically close; triggers immediate backup regardless of other logic
const float WALL_HEADON_FRONT_CM = 28.0f;        // Front threshold for classifying a head-on wall approach
const float WALL_HEADON_SIDE_CM = 42.0f;         // Both sides must be closer than this for a head-on diagnosis
const float WALL_HEADON_SIDE_BALANCE_CM = 12.0f; // Max L/R difference still considered symmetric (head-on)
const float WIDE_OPEN_DIFF_CM = 18.0f;           // L/R gap difference qualifying one side as "clearly more open"
const float REPEAT_PATTERN_TOLERANCE_CM = 9.0f;  // Max distance difference for two snapshots to count as the same trap

// ─── Timing constants (ms) ──────────────────────────────────────────────────────
const unsigned long SENSOR_INTERVAL_MS = 70;   // Minimum interval between sensor reads in the main loop
const unsigned long STRAFE_DURATION_MS = 450;  // Default duration of a strafe (lateral slide) maneuver
const unsigned long BACKUP_DURATION_MS = 380;  // Default duration of a backward maneuver

// ─── Hazard detection streak thresholds ─────────────────────────────────────────
const uint8_t ALL_UNKNOWN_STREAK_THRESHOLD = 8;   // Consecutive all-invalid readings before forcing a decision
const uint8_t NEAR_OBSTACLE_CONFIRM_STREAK = 2;   // Consecutive close readings needed to latch obstacleNearby

// ─── Open-space detection ────────────────────────────────────────────────────────
// The robot is in "open space" when all three directions are large and L/R are
// roughly balanced.  The planner avoids unnecessary turns in open space.
const float OPEN_SPACE_DISTANCE_CM = 110.0f;   // All-directions clear threshold
const float OPEN_SPACE_FRONT_BIAS_CM = 95.0f;  // Front must exceed this for open-space classification
const float OPEN_SPACE_SIDE_BIAS_CM = 70.0f;   // Both sides must also exceed this
const float OPEN_SPACE_BALANCE_CM = 22.0f;      // Max L/R imbalance allowed in open space
const unsigned long OPEN_SPACE_HOLD_MS = 1500; // How long to trust an open-space reading after it goes stale

// ─── Azure AI Foundry LLM request settings ───────────────────────────────────────
const uint16_t FOUNDRY_MAX_OUTPUT_TOKENS = 160;           // Token budget for the first request attempt
const uint16_t FOUNDRY_RETRY_OUTPUT_TOKENS = 120;         // Reduced budget for the single retry attempt
const unsigned long FOUNDRY_HTTP_TIMEOUT_TIGHT_MS = 3800; // Tight HTTP timeout aiming for fast turnaround
const unsigned long FOUNDRY_HTTP_TIMEOUT_RETRY_MS = 9000; // Longer timeout used on the retry after a timeout

// ─── Decision-loop timing ───────────────────────────────────────────────────────
const unsigned long HAZARD_DECISION_COOLDOWN_MS = 800;  // Minimum gap between consecutive hazard decisions
const unsigned long FRONT_STALE_MS = 450;               // Age after which a front reading is considered stale
const uint8_t FRONT_BLIND_STREAK_THRESHOLD = 4;         // Stale-front reads before triggering blind-state hazard
const unsigned long HAZARD_BURST_WINDOW_MS = 3000;      // Window for counting rapid hazard events
const uint8_t HAZARD_BURST_THRESHOLD = 2;               // Events within the window that escalate to forced backup
const unsigned long HAZARD_CLEAR_RESET_MS = 1200;       // Time with no obstacle before resetting the burst counter
const unsigned long PAN_SETTLE_MS = 120;                // Wait after moving the pan servo before taking a reading
const unsigned long RESCAN_PAUSE_MS = 140;              // Pause duration for a RESCAN maneuver
const unsigned long DECISION_STOP_PAUSE_MS = 250;       // Brief stop inserted before hazard scan-and-decide

// ─── LED feedback timings ────────────────────────────────────────────────────────
const unsigned long THINK_LED_ON_MS = 140;   // LED on-time per flash in the Foundry "thinking" animation
const unsigned long THINK_LED_OFF_MS = 80;   // LED off-time per flash in the "thinking" animation
const unsigned long DECISION_LED_MS = 220;   // Duration of the single direction flash after a decision

// ─── Front-distance EMA filter ─────────────────────────────────────────────────────
// An exponential moving average smooths noisy front-sensor readings.
// Higher alpha = more responsive to change; lower alpha = smoother output.
const float FRONT_EMA_ALPHA = 0.35f;      // EMA smoothing factor for the front distance
const int SAFE_UNKNOWN_DISTANCE_CM = 120; // Substitute value when a distance reading is invalid

// ─── Navigation history and trap detection ───────────────────────────────────────
const uint8_t NAV_HISTORY_SIZE = 8;      // Depth of the circular hazard-snapshot ring buffer
const uint8_t TRAP_REPEAT_THRESHOLD = 3; // Matching snapshots needed to declare a repeated trap

// ─── Foundry bypass ("obvious local answer") gate ────────────────────────────────
// The Foundry API is bypassed when the local plan is clearly correct, saving
// the ~4 s network round-trip.  All four conditions below must be satisfied.
const uint8_t OBVIOUS_OSCILLATION_MAX = 0;        // Skip bypass if any L/R oscillation is detected
const float OBVIOUS_LOCAL_CONFIDENCE_MIN = 0.74f; // Local plan must reach at least this confidence
const float OBVIOUS_LOCAL_RISK_MAX = 0.58f;       // Environmental risk must be below this value

// ─── LLM confidence gates ─────────────────────────────────────────────────────────
const float MIN_LLM_CONFIDENCE = 0.45f;                  // Minimum to accept any LLM plan
const float MIN_LLM_CONFIDENCE_FOR_DISAGREEMENT = 0.72f; // Higher bar when LLM contradicts the local planner

// ─── Plan quality tracking and maneuver duration limits ───────────────────────────
const float PLAN_IMPROVEMENT_MARGIN_CM = 5.0f;  // Front clearance gain (cm) counted as "improved"
const uint16_t MIN_MANEUVER_DURATION_MS = 180;  // Durations shorter than this fall back to the default
const uint16_t MAX_MANEUVER_DURATION_MS = 700;  // Durations longer than this fall back to the default
const uint8_t MAX_SAME_TURN_STREAK = 2;         // Anti-spin: max consecutive same-direction turns before switching
// ─── Runtime state ─────────────────────────────────────────────────────────────────

// --- Obstacle / hazard flags ---
bool obstacleNearby = false;         // True when a close obstacle has been confirmed
bool previousObstacleNearby = false; // Value from the previous loop iteration (detects transitions)
// --- Timing bookmarks ---
unsigned long lastSensorMs = 0;            // When the sensor was last polled
unsigned long lastPanStepMs = 0;           // When the pan servo last moved one step
unsigned long lastHazardDecisionMs = 0;    // When the last hazard decision was made

// --- Hazard-burst (rapid-repeat) tracking ---
unsigned long hazardBurstWindowStartMs = 0; // Start time of the current burst-counting window
uint8_t hazardBurstCount = 0;               // Number of hazard events in the current window
unsigned long hazardClearSinceMs = 0;       // When the path last became clear (for burst reset timer)

// --- Turn direction memory ---
Action lastNonStopDecision = ACTION_LEFT;   // Last lateral direction chosen; used for tie-breaking

// --- Pan servo state ---
bool panServoReady = false;           // True if the servo successfully attached at startup
int panCurrentDeg = PAN_CENTER_DEG;   // Current servo angle (degrees)
bool panSweepTowardLeft = true;       // Sweep direction: true = moving toward left endpoint

// --- Distance readings (cm; -1 = invalid / no echo) ---
float leftDistanceCm = -1.0f;    // Last valid left-side reading
float frontDistanceCm = -1.0f;   // EMA-filtered front reading (see updateFrontDistanceEstimate)
float rightDistanceCm = -1.0f;   // Last valid right-side reading
float frontFilteredCm = -1.0f;   // Internal EMA accumulator for the front sensor
bool frontFilterReady = false;   // True once the EMA has been seeded with at least one reading
unsigned long frontUpdatedMs = 0; // Timestamp of the last front-distance update

// --- Navigation history ring buffer ---
HazardSnapshot navHistory[NAV_HISTORY_SIZE]; // Circular buffer of recent hazard snapshots
uint8_t navHistoryCount = 0;                 // How many entries are currently stored (up to NAV_HISTORY_SIZE)
uint8_t navHistoryWriteIndex = 0;            // Next write position in the ring buffer

// --- Recent plan ring buffer (last 4 maneuver types) ---
ManeuverType recentPlans[4] = {MANEUVER_STOP, MANEUVER_STOP, MANEUVER_STOP, MANEUVER_STOP};
uint8_t recentPlanWriteIndex = 0; // Next write position

// --- Plan outcome feedback ---
int8_t lastPlanOutcome = 0;            // +1 = improved clearance, -1 = worsened, 0 = unknown
float lastPlanFrontBeforeCm = -1.0f;   // Front clearance immediately before the last maneuver
float lastPlanFrontAfterCm = -1.0f;    // Front clearance immediately after the last maneuver

// --- Sensor streak counters ---
uint8_t allUnknownStreak = 0;     // Consecutive cycles where all three distances were invalid
uint8_t frontBlindStreak = 0;     // Consecutive cycles with a stale/invalid front reading
uint8_t nearObstacleStreak = 0;   // Consecutive cycles with front <= ULTRASONIC_ALERT_CM
unsigned long lastOpenSpaceSeenMs = 0; // Timestamp of the last open-space detection

// --- Foundry (LLM) request telemetry ---
bool foundryRequestSent = false;      // True if an HTTP request was sent this cycle
bool foundryResponseOk = false;       // True if a 2xx HTTP response was received
bool foundryPlanParsed = false;       // True if the model text was successfully parsed
String foundryDecisionStatus = "idle"; // Human-readable status for Serial telemetry
// ─── LED helpers ────────────────────────────────────────────────────────────────────

// Sets both left and right status LEDs to the same on/off state,
// respecting the LED_ACTIVE_HIGH polarity setting.
void setBothLeds(bool on) {
  uint8_t level = on ? (LED_ACTIVE_HIGH ? HIGH : LOW) : (LED_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(LEFT_LED_PIN, level);
  digitalWrite(RIGHT_LED_PIN, level);
}
// Animates both LEDs in a double-flash pattern to indicate the robot is
// waiting for a Foundry (LLM) response.  Provides visible feedback during
// the network round-trip.
void flashFoundryThinkingLeds() {
  setBothLeds(true);
  delay(THINK_LED_ON_MS);
  setBothLeds(false);
  delay(THINK_LED_OFF_MS);
  setBothLeds(true);
  delay(THINK_LED_ON_MS);
  setBothLeds(false);
}
// Flashes the left LED for left maneuvers and the right LED for right maneuvers
// to provide visible direction feedback after a navigation decision.
// No-ops for non-directional maneuvers (backward, stop, rescan).
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
// ─── WiFi ───────────────────────────────────────────────────────────────────────────

// Attempts to connect to the configured WiFi network, retrying for up to 20 s.
// Navigation continues with local-only planning if WiFi is unavailable.
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
// ─── Distance utility helpers ───────────────────────────────────────────────────────

// Returns true if distanceCm represents a physically plausible sensor reading
// (above ULTRASONIC_MIN_VALID_CM, ruling out no-echo / too-close returns).
bool isValidDistance(float distanceCm) {
  return distanceCm >= ULTRASONIC_MIN_VALID_CM;
}
// Returns true when all three directional distances are invalid (sensor blind
// state).  Used to trigger a forced rescan rather than a directional maneuver.
bool allDistancesUnknown() {
  return !isValidDistance(leftDistanceCm) && !isValidDistance(frontDistanceCm) && !isValidDistance(rightDistanceCm);
}
// Returns distanceCm as an integer if the reading is valid, or
// SAFE_UNKNOWN_DISTANCE_CM as a conservative stand-in so arithmetic
// comparisons in the planner remain meaningful.
int effectiveDistance(float distanceCm) {
  if (isValidDistance(distanceCm)) {
    return (int)distanceCm;
  }
  return SAFE_UNKNOWN_DISTANCE_CM;
}
// Picks a fallback turn direction based on which side has more clearance.
// Breaks ties by alternating from the last non-stop decision to avoid spin.
Action chooseFallbackTurn() {
  int leftEff = effectiveDistance(leftDistanceCm);
  int rightEff = effectiveDistance(rightDistanceCm);
  if (leftEff == rightEff) {
    return lastNonStopDecision == ACTION_LEFT ? ACTION_RIGHT : ACTION_LEFT;
  }
  return (leftEff > rightEff) ? ACTION_LEFT : ACTION_RIGHT;
}
// ─── Forward declarations (defined later in this file) ───────────────────────────
const char *maneuverTypeToString(ManeuverType maneuver);
uint16_t clampDuration(uint16_t value, uint16_t fallbackMs);
ManeuverPlan defaultPlan();
bool extractFoundryModelText(const JsonDocument &responseDoc, String &modelText);
bool extractFoundryModelTextFromRawResponse(const String &responseBody, String &modelText);
void updateFrontDistanceEstimate(float measuredCm, unsigned long nowMs, bool resetFilter);
bool isOpenSpaceSnapshot(int leftEff, int frontEff, int rightEff);
uint8_t countRecentSameTurnStreak(ManeuverType turnManeuver);
// ─── Front-distance EMA filter ─────────────────────────────────────────────────────

// Applies an exponential moving average to a raw front-sensor reading to
// reduce noise while still tracking genuine distance changes.  When resetFilter
// is true (e.g., right after a maneuver) the accumulator is reinitialised from
// the new reading so stale history does not pollute the fresh estimate.
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
// ─── Plan builder helpers ───────────────────────────────────────────────────────────

// Converts a coarse Action direction to the corresponding strafe ManeuverType.
ManeuverType actionToStrafeManeuver(Action action) {
  return action == ACTION_LEFT ? MANEUVER_STRAFE_LEFT : MANEUVER_STRAFE_RIGHT;
}
// Converts a coarse Action direction to the corresponding 90-degree turn ManeuverType.
ManeuverType actionToTurnManeuver(Action action) {
  return action == ACTION_LEFT ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
}
// Builds a plan that slides the robot laterally toward the more-open side.
// durationMs is clamped by clampDuration; the secondary step rescans.
ManeuverPlan buildStrafePlan(Action action, uint16_t durationMs) {
  ManeuverPlan plan = defaultPlan();
  plan.primary = actionToStrafeManeuver(action);
  plan.primaryDurationMs = clampDuration(durationMs, STRAFE_DURATION_MS);
  plan.secondary = MANEUVER_RESCAN;
  plan.secondaryDurationMs = RESCAN_PAUSE_MS;
  plan.confidence = 0.0f;
  return plan;
}
// Builds a plan that rotates the robot 90 degrees in place toward the open side.
// The secondary step rescans after the turn completes.
ManeuverPlan buildTurnPlan(Action action) {
  ManeuverPlan plan = defaultPlan();
  plan.primary = actionToTurnManeuver(action);
  plan.primaryDurationMs = TURN_90_DURATION_MS;
  plan.secondary = MANEUVER_RESCAN;
  plan.secondaryDurationMs = RESCAN_PAUSE_MS;
  plan.confidence = 0.0f;
  return plan;
}
// Builds a recovery plan: reverse first, then turn 90 degrees.
// Used to escape tight corners and break out of repeated-trap positions.
ManeuverPlan buildRecoveryPlan(Action action) {
  ManeuverPlan plan = defaultPlan();
  plan.primary = MANEUVER_BACKWARD;
  plan.primaryDurationMs = BACKUP_DURATION_MS;
  plan.secondary = actionToTurnManeuver(action);
  plan.secondaryDurationMs = TURN_90_DURATION_MS;
  plan.confidence = 0.0f;
  return plan;
}
// Formats a ManeuverPlan into a compact key=value string for inclusion in the
// Foundry prompt so the LLM can compare the candidate options by name.
String planToPromptText(const char *label, const ManeuverPlan &plan) {
  return String(label) + "={primary:" + maneuverTypeToString(plan.primary) +
         ",primary_ms:" + String(plan.primaryDurationMs) +
         ",secondary:" + maneuverTypeToString(plan.secondary) +
         ",secondary_ms:" + String(plan.secondaryDurationMs) +
         ",confidence:" + String(plan.confidence, 2) +
         ",risk:" + String(plan.riskScore, 2) + "}";
}
// ─── Local plan scoring ─────────────────────────────────────────────────────────────

// Returns a score in [0, 1] for a candidate ManeuverPlan against the current
// sensor snapshot.  Higher is better.  Scoring factors:
//   - Base score per maneuver type (backward=0.42, strafe=0.48, turn=0.38, ...)
//   - Available clearance on the chosen side vs. the front
//   - Whether the front distance is within the alert range
//   - Front-distance trend (negative trend rewards more aggressive escapes)
//   - Repeated-trap and oscillation penalties / bonuses
//   - Open-space penalty for turns (prefer strafe in open corridors)
float scoreLocalPlanCandidate(const ManeuverPlan &candidate, int leftEff, int frontEff, int rightEff, bool repeatedTrap, uint8_t oscillationCount, float frontTrendCmValue) {
  float score = 0.0f;
  int preferredSideEff = 0;
  bool openSpace = isOpenSpaceSnapshot(leftEff, frontEff, rightEff);
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
      score = 0.38f + ((preferredSideEff - frontEff) / 220.0f);
      if (frontEff <= ULTRASONIC_ALERT_CM) {
        score += 0.20f;
      } else if (frontEff >= OPEN_SPACE_DISTANCE_CM) {
        score -= 0.24f;
      }
      if (repeatedTrap || oscillationCount >= 2) {
        score += 0.12f;
      }
      if (frontTrendCmValue < -4.0f) {
        score += 0.06f;
      }
      if (openSpace) {
        score -= 0.08f;
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
// ─── Environment geometry helpers ──────────────────────────────────────────────────

// Returns true when the robot is facing a head-on wall: front is very close
// AND both sides are similarly close and balanced (symmetric near-wall geometry).
// In this case the only safe exit is to reverse before turning.
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
// Returns true when all three directions show ample clear space and left/right
// are approximately balanced — the "open corridor" case.  Used to suppress
// unnecessary turns that would waste time in unobstructed areas.
bool isOpenSpaceSnapshot(int leftEff, int frontEff, int rightEff) {
  bool frontOpen = frontEff >= OPEN_SPACE_FRONT_BIAS_CM;
  bool sideOpen = (leftEff >= OPEN_SPACE_SIDE_BIAS_CM) && (rightEff >= OPEN_SPACE_SIDE_BIAS_CM);
  bool sideBalanced = abs(leftEff - rightEff) <= OPEN_SPACE_BALANCE_CM;
  return frontOpen && sideOpen && sideBalanced;
}
// ─── Maneuver history helpers ──────────────────────────────────────────────────────

// Counts how many of the last four plan entries are the same turn direction,
// stopping when an opposite or different maneuver is encountered.  Used by the
// anti-spin guard to detect turn fixation (robot spinning in one direction).
uint8_t countRecentSameTurnStreak(ManeuverType turnManeuver) {
  if (turnManeuver != MANEUVER_TURN_LEFT_90 && turnManeuver != MANEUVER_TURN_RIGHT_90) {
    return 0;
  }
  uint8_t streak = 0;
  ManeuverType oppositeTurn = (turnManeuver == MANEUVER_TURN_LEFT_90) ? MANEUVER_TURN_RIGHT_90 : MANEUVER_TURN_LEFT_90;
  for (uint8_t i = 0; i < 4; ++i) {
    uint8_t idx = (recentPlanWriteIndex + 4 - 1 - i) % 4;
    ManeuverType recent = recentPlans[idx];
    if (recent == MANEUVER_RESCAN || recent == MANEUVER_STOP) {
      continue;
    }
    if (recent == turnManeuver) {
      streak++;
      continue;
    }
    if (recent == oppositeTurn || recent == MANEUVER_STRAFE_LEFT || recent == MANEUVER_STRAFE_RIGHT || recent == MANEUVER_BACKWARD) {
      break;
    }
  }
  return streak;
}
// Returns the printable string name for a ManeuverType (used in Serial logs
// and in the Foundry prompt so the LLM sees human-readable plan names).
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
// Parses a ManeuverType from a text string (case-insensitive).  Used to
// interpret keyword tokens returned by the LLM or read from test inputs.
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
// Clamps value to [MIN_MANEUVER_DURATION_MS, MAX_MANEUVER_DURATION_MS].
// Returns fallbackMs when value is out of that range.
uint16_t clampDuration(uint16_t value, uint16_t fallbackMs) {
  if (value < MIN_MANEUVER_DURATION_MS || value > MAX_MANEUVER_DURATION_MS) {
    return fallbackMs;
  }
  return value;
}
// Pushes a ManeuverType into the 4-entry circular recent-plans ring buffer.
void rememberPlan(ManeuverType maneuver) {
  recentPlans[recentPlanWriteIndex] = maneuver;
  recentPlanWriteIndex = (recentPlanWriteIndex + 1) % 4;
}
// Saves a L/F/R distance snapshot to the hazard history ring buffer.
// Older entries are silently overwritten once the buffer is full.
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
// Returns true if snapshot a is within REPEAT_PATTERN_TOLERANCE_CM of the
// given distances in all three directions (i.e., same spatial position).
bool snapshotSimilar(const HazardSnapshot &a, float leftCm, float frontCm, float rightCm) {
  return fabs(a.leftCm - leftCm) <= REPEAT_PATTERN_TOLERANCE_CM &&
         fabs(a.frontCm - frontCm) <= REPEAT_PATTERN_TOLERANCE_CM &&
         fabs(a.rightCm - rightCm) <= REPEAT_PATTERN_TOLERANCE_CM;
}
// Returns true when the current L/F/R distances match at least
// TRAP_REPEAT_THRESHOLD stored snapshots, indicating the robot is stuck in a
// repeated-obstacle loop and needs a more aggressive escape strategy.
bool detectRepeatedTrap(float leftCm, float frontCm, float rightCm) {
  uint8_t similarCount = 0;
  for (uint8_t i = 0; i < navHistoryCount; ++i) {
    if (snapshotSimilar(navHistory[i], leftCm, frontCm, rightCm)) {
      similarCount++;
    }
  }
  return similarCount >= TRAP_REPEAT_THRESHOLD;
}
// Builds a comma-separated string of the four most recent plan names
// (oldest first) for inclusion in the Foundry prompt.
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
// Builds a pipe-separated string of all stored L/F/R history snapshots
// (oldest first) for inclusion in the Foundry prompt as context.
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
// Returns the change in front clearance from the oldest to the newest history
// snapshot (positive = path opening up, negative = closing in / getting worse).
float frontTrendCm() {
  if (navHistoryCount < 2) {
    return 0.0f;
  }
  uint8_t newest = (navHistoryWriteIndex + NAV_HISTORY_SIZE - 1) % NAV_HISTORY_SIZE;
  uint8_t oldest = (navHistoryCount == NAV_HISTORY_SIZE) ? navHistoryWriteIndex : 0;
  return navHistory[newest].frontCm - navHistory[oldest].frontCm;
}
// Counts left↔right direction swaps in the last four plan entries.
// A high count indicates oscillatory behaviour (robot bouncing between two walls).
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
// Returns a human-readable label for the outcome of the last executed plan;
// fed back to the Foundry prompt so the LLM can learn from recent history.
const char *lastPlanOutcomeString() {
  if (lastPlanOutcome > 0) {
    return "improved_clearance";
  }
  if (lastPlanOutcome < 0) {
    return "worsened_or_no_gain";
  }
  return "unknown";
}
// Computes an environmental risk score in [0, 1] from current L/F/R distances.
// Front clearance is weighted 70%, the nearer side 30%.  Higher = more danger.
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
// Creates a ManeuverPlan initialised to STOP with zero confidence and the
// current risk estimate.  Used as a safe base before fields are overwritten.
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
// ─── Local (offline) planner ─────────────────────────────────────────────────────────

// Selects the best local maneuver plan without any network call by scoring
// strafe, turn, and recovery candidates against the current sensor snapshot.
// Applies anti-spin (turn-streak) and open-space override rules before returning.
// If the robot is in an emergency or repeated-trap state, a recovery plan is
// returned immediately with high confidence.
ManeuverPlan chooseLocalPlan(bool repeatedTrap) {
  ManeuverPlan plan = defaultPlan();
  plan.repeatedTrap = repeatedTrap;
  int leftEff = effectiveDistance(leftDistanceCm);
  int rightEff = effectiveDistance(rightDistanceCm);
  int frontEff = effectiveDistance(frontDistanceCm);
  Action preferredTurn = chooseFallbackTurn();
  uint8_t oscillationCount = detectPlanOscillationCount();
  float frontTrend = frontTrendCm();
  bool openSpace = isOpenSpaceSnapshot(leftEff, frontEff, rightEff);
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
  if (openSpace && !repeatedTrap && (bestPlan.primary == MANEUVER_TURN_LEFT_90 || bestPlan.primary == MANEUVER_TURN_RIGHT_90)) {
    bestPlan = strafePlan;
    bestScore = strafeScore;
  }
  if (!repeatedTrap && (bestPlan.primary == MANEUVER_TURN_LEFT_90 || bestPlan.primary == MANEUVER_TURN_RIGHT_90)) {
    uint8_t turnStreak = countRecentSameTurnStreak(bestPlan.primary);
    if (turnStreak >= MAX_SAME_TURN_STREAK) {
      Action oppositeAction = (bestPlan.primary == MANEUVER_TURN_LEFT_90) ? ACTION_RIGHT : ACTION_LEFT;
      bestPlan = buildStrafePlan(oppositeAction, STRAFE_DURATION_MS);
      bestScore = strafeScore + 0.06f;
      if (bestScore > 1.0f) {
        bestScore = 1.0f;
      }
      Serial.println("Anti-spin guard: replacing repeated turn streak with opposite strafe");
    }
  }
  bestPlan.riskScore = estimateLocalRiskScore();
  bestPlan.repeatedTrap = repeatedTrap;
  bestPlan.confidence = bestScore;
  return bestPlan;
}
// ─── JSON / LLM response parsing helpers ───────────────────────────────────────────

// Scans input for the first complete, brace-balanced JSON object and copies it
// into jsonText.  Returns false if no complete object is found.
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
// Fallback JSON extractor for truncated LLM output: strips everything from the
// "reason" field onward and closes the brace, recovering the fields the planner
// actually needs (choice, confidence) when the token budget was exhausted.
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
// Parses the LLM's JSON choice response, extracting:
//   "choice"     – one of: baseline, strafe, turn, recovery
//   "confidence" – float in [0, 1]
// Tries extractFirstJsonObject first; falls back to extractTruncatedPlanPrefix
// if the JSON appears to be truncated.  Returns false on any parse failure.
bool parseChoiceFromJsonText(const String &modelText, String &choice, float &confidence) {
  String jsonText;
  if (!extractFirstJsonObject(modelText, jsonText)) {
    if (!extractTruncatedPlanPrefix(modelText, jsonText)) {
      return false;
    }
    Serial.println("Choice JSON truncated; using prefix without reason field");
  }
  JsonDocument choiceDoc;
  DeserializationError err = deserializeJson(choiceDoc, jsonText);
  if (err) {
    Serial.print("Choice JSON parse error: ");
    Serial.println(err.c_str());
    return false;
  }
  choice = choiceDoc["choice"] | "baseline";
  choice.trim();
  choice.toLowerCase();
  confidence = choiceDoc["confidence"] | 0.0f;
  if (confidence < 0.0f) {
    confidence = 0.0f;
  }
  if (confidence > 1.0f) {
    confidence = 1.0f;
  }
  if (choice != "baseline" && choice != "strafe" && choice != "turn" && choice != "recovery") {
    return false;
  }
  return true;
}
// Returns true if the plan's primary direction is left (strafe or turn).
bool isPlanDirectionLeft(ManeuverType maneuver) {
  return maneuver == MANEUVER_STRAFE_LEFT || maneuver == MANEUVER_TURN_LEFT_90;
}
// Returns true if the plan's primary direction is right (strafe or turn).
bool isPlanDirectionRight(ManeuverType maneuver) {
  return maneuver == MANEUVER_STRAFE_RIGHT || maneuver == MANEUVER_TURN_RIGHT_90;
}
// Returns true when plan a and plan b point in opposite lateral directions
// (one turns/strafes left while the other turns/strafes right).
bool plansDisagreeDirection(const ManeuverPlan &a, const ManeuverPlan &b) {
  bool aLeft = isPlanDirectionLeft(a.primary);
  bool aRight = isPlanDirectionRight(a.primary);
  bool bLeft = isPlanDirectionLeft(b.primary);
  bool bRight = isPlanDirectionRight(b.primary);
  return (aLeft && bRight) || (aRight && bLeft);
}
// Returns true when the local plan is confident enough that the ~4 s Foundry
// round-trip can be skipped.  All of the following must hold:
//   - Not a repeated-trap situation
//   - Last plan did not worsen clearance
//   - No detected L/R oscillation
//   - Local confidence >= OBVIOUS_LOCAL_CONFIDENCE_MIN
//   - Risk score <= OBVIOUS_LOCAL_RISK_MAX
//   - If the plan is a turn, the current snapshot is not open space
bool shouldBypassFoundry(const ManeuverPlan &localPlan, bool repeatedTrap) {
  if (repeatedTrap) {
    return false;
  }
  if (lastPlanOutcome < 0) {
    return false;
  }
  uint8_t oscillationCount = detectPlanOscillationCount();
  if (oscillationCount > OBVIOUS_OSCILLATION_MAX) {
    return false;
  }
  if (localPlan.confidence < OBVIOUS_LOCAL_CONFIDENCE_MIN) {
    return false;
  }
  if (localPlan.riskScore > OBVIOUS_LOCAL_RISK_MAX) {
    return false;
  }
  if (localPlan.primary == MANEUVER_TURN_LEFT_90 || localPlan.primary == MANEUVER_TURN_RIGHT_90) {
    int leftEff = effectiveDistance(leftDistanceCm);
    int frontEff = effectiveDistance(frontDistanceCm);
    int rightEff = effectiveDistance(rightDistanceCm);
    if (isOpenSpaceSnapshot(leftEff, frontEff, rightEff)) {
      return false;
    }
  }
  return true;
}
// Walks the structured JSON response document to extract the model's text output.
// Tries multiple known Azure AI Foundry response shapes in order:
//   1. Top-level "output_text" field (simplest format)
//   2. "output[].content[].text" (standard Responses API message format)
//   3. "choices[0].message.content" (Chat Completions API compatibility)
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
// Raw-string fallback for extracting model text when full JSON parsing fails or
// produces an unexpected shape.  Scans the response body for an "output_text"
// type entry and manually decodes the escaped string value character by character.
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
// Validates and repairs an LLM-returned ManeuverPlan before it is executed:
//   - Replaces a STOP primary with the fallback plan
//   - Replaces RESCAN primary with the fallback's primary
//   - Fixes turn durations to the calibrated TURN_90_DURATION_MS constant
//   - Rejects backward plans when front is already clear
//   - Falls back when LLM confidence is below MIN_LLM_CONFIDENCE
//   - Falls back when the LLM disagrees with the local planner on direction
//     and confidence is below MIN_LLM_CONFIDENCE_FOR_DISAGREEMENT
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
// ─── Azure AI Foundry arbiter ──────────────────────────────────────────────────────────

// Posts the current sensor state, history, and all four candidate plans to the
// Azure AI Foundry Responses API.  The LLM picks one candidate by name and
// returns a confidence score.  The chosen plan is sanitized before being
// returned.  Falls back to fallbackPlan on any of:
//   - WiFi not connected
//   - HTTP begin / send failure
//   - Non-2xx status code (one timeout retry is attempted)
//   - JSON parse or model-text extraction failure
//   - Choice parse failure
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
      "Do not invent maneuvers or durations. "
      "Choose only one candidate from: baseline, strafe, turn, recovery. "
      "If repeatedTrap is true or front trend is negative, prefer a recovery maneuver that backs up before turning. "
      "If the baseline candidate is already safe, keep it unless another candidate clearly improves clearance. "
      "Return strict JSON only, no markdown, with keys choice and confidence. "
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
      ". Example JSON: {\"choice\":\"recovery\",\"confidence\":0.82}.";
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
  String choice;
  float confidence = 0.0f;
  if (!parseChoiceFromJsonText(modelText, choice, confidence)) {
    foundryDecisionStatus = "choice_json_parse_error_local_fallback";
    Serial.println("Foundry choice parse failed, using local fallback plan");
    return fallbackPlan;
  }
  foundryPlanParsed = true;
  ManeuverPlan plan = fallbackPlan;
  if (choice == "strafe") {
    plan = localStrafePlan;
  } else if (choice == "turn") {
    plan = localTurnPlan;
  } else if (choice == "recovery") {
    plan = localRecoveryPlan;
  }
  plan.confidence = confidence;
  plan.repeatedTrap = repeatedTrap;
  sanitizePlan(plan, fallbackPlan);
  foundryDecisionStatus = "choice_parsed_and_applied";
  flashDecisionDirectionLed(plan.primary);
  Serial.print("Foundry choice: ");
  Serial.print(choice);
  Serial.print(" -> ");
  Serial.print(maneuverTypeToString(plan.primary));
  Serial.print(" then ");
  Serial.print(maneuverTypeToString(plan.secondary));
  Serial.print(" conf=");
  Serial.println(plan.confidence, 2);
  return plan;
}
// ─── Pan servo and sensor scanning ──────────────────────────────────────────────────

// Moves the pan servo to the calibrated centre position and updates the
// tracking variable.  No-ops if the servo did not attach at startup.
void movePanToCenter() {
  if (!panServoReady) {
    return;
  }
  panCurrentDeg = PAN_CENTER_DEG;
  panServo.write(panCurrentDeg);
}
// Sweeps the pan servo to left, right, and centre positions in sequence,
// waits PAN_SETTLE_MS after each move, then takes a distance reading.
// Updates leftDistanceCm, frontDistanceCm (via EMA), and rightDistanceCm.
// Called before every hazard decision to get a fresh L/F/R snapshot.
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
// Advances the pan servo one PAN_STEP_DEG in the current sweep direction
// during normal forward driving (background scanning between hazard events).
// Reverses direction at the PAN_LEFT_DEG / PAN_RIGHT_DEG endpoints.
// No-ops if the servo is not attached or if called too soon after the last step.
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
// ─── Main hazard detection loop (called every iteration from loop()) ────────────

// Polls the ultrasonic sensor, applies the EMA filter to the front reading, and
// updates the obstacleNearby flag using hysteresis and streak counters:
//   - nearObstacleStreak: latches obstacleNearby after NEAR_OBSTACLE_CONFIRM_STREAK
//     consecutive close readings; clears it when front exceeds ULTRASONIC_CLEAR_CM.
//   - frontBlindStreak: escalates to hazard after FRONT_BLIND_STREAK_THRESHOLD
//     consecutive stale/invalid front readings.
//   - allUnknownStreak: forces obstacleNearby when all three distances are
//     persistently invalid (complete sensor blind state).
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
    if (frontDistanceCm >= OPEN_SPACE_DISTANCE_CM) {
      lastOpenSpaceSeenMs = nowMs;
    }
    frontBlindStreak = 0;
    if (frontDistanceCm <= ULTRASONIC_ALERT_CM) {
      if (nearObstacleStreak < 255) {
        nearObstacleStreak++;
      }
    } else {
      nearObstacleStreak = 0;
    }
    if (obstacleNearby) {
      obstacleNearby = (frontDistanceCm <= ULTRASONIC_CLEAR_CM) || (nearObstacleStreak > 0);
    } else {
      obstacleNearby = (nearObstacleStreak >= NEAR_OBSTACLE_CONFIRM_STREAK);
    }
  } else {
    nearObstacleStreak = 0;
    if (lastOpenSpaceSeenMs != 0 && (nowMs - lastOpenSpaceSeenMs) <= OPEN_SPACE_HOLD_MS) {
      frontBlindStreak = 0;
      obstacleNearby = false;
    } else {
      if (frontBlindStreak < 255) {
        frontBlindStreak++;
      }
      obstacleNearby = (frontBlindStreak >= FRONT_BLIND_STREAK_THRESHOLD);
    }
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
// ─── Hazard burst escalation ───────────────────────────────────────────────────────────

// Returns true when hazard events are occurring too frequently: if
// HAZARD_BURST_THRESHOLD events happen within HAZARD_BURST_WINDOW_MS, the
// robot is stuck and should force a backup regardless of sensor geometry.
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
// ─── Maneuver execution ──────────────────────────────────────────────────────────────

// Returns the motor speed to use for a maneuver.  Speed is reduced when risk
// is high or confidence is modest to give the robot more reaction time, and
// reduced slightly further for backward moves to avoid wall collisions.
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
// Executes a single atomic maneuver: drives the robot for durationMs at speed,
// then stops the motors.  RESCAN maneuvers pause the motors and call
// refreshHazardScanSnapshot() to update L/F/R readings mid-plan.
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
// Executes both the primary and secondary maneuvers in the plan at the
// computed speed, records each to the recent-plans ring buffer, and updates
// lastNonStopDecision for future tie-breaking.
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
// ─── Arduino entry points ────────────────────────────────────────────────────────────────

// Initialises hardware peripherals (Serial, GPIO, chassis, sensor, pan servo),
// connects to WiFi, and prints a pan-servo calibration report to Serial.
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
// Main control loop executed repeatedly by the Arduino runtime.
//
// Normal operation (no obstacle):
//   - Calls updateScanAndHazard() each iteration to poll the sensor.
//   - Drives the car forward at FORWARD_SPEED.
//
// Hazard response (obstacle detected OR cooldown expired):
//   1. Stops the car.
//   2. Takes a full L/F/R scan snapshot.
//   3. Records the snapshot to navigation history.
//   4. Runs the two-tier decision pipeline:
//        a. Immediate forced plans for edge cases (blind sensor, head-on wall,
//           emergency close obstacle, hazard burst).
//        b. Local planner scores candidates; if "obvious", skips Foundry.
//        c. Otherwise posts to Azure AI Foundry and uses the LLM's choice.
//   5. Executes the chosen plan.
//   6. Re-scans and records the front-clearance delta for outcome feedback.
void loop() {
  updateScanAndHazard();
  unsigned long nowMs = millis();
  if (obstacleNearby && (!previousObstacleNearby || (nowMs - lastHazardDecisionMs) >= HAZARD_DECISION_COOLDOWN_MS)) {
    myCar.Move(Stop, 0);
    Serial.println("Hazard detected: STOP -> SCAN -> DECIDE");
    delay(DECISION_STOP_PAUSE_MS);
    refreshHazardScanSnapshot();
    recordHazardSnapshot(leftDistanceCm, frontDistanceCm, rightDistanceCm, nowMs);
    ManeuverPlan plan;
    String decisionSource;
    bool repeatedTrap = detectRepeatedTrap(leftDistanceCm, frontDistanceCm, rightDistanceCm);
    if (allDistancesUnknown()) {
      plan = defaultPlan();
      plan.primary = MANEUVER_RESCAN;
      plan.primaryDurationMs = RESCAN_PAUSE_MS + 80;
      plan.secondary = MANEUVER_STOP;
      plan.secondaryDurationMs = 0;
      plan.confidence = 1.0f;
      decisionSource = "local_sensor_blind";
      Serial.println("Sensor blind state: local forced rescan plan");
    } else if (isHeadOnWall(frontDistanceCm, leftDistanceCm, rightDistanceCm)) {
      plan = defaultPlan();
      plan.primary = MANEUVER_BACKWARD;
      plan.primaryDurationMs = BACKUP_DURATION_MS;
      plan.secondary = (chooseFallbackTurn() == ACTION_LEFT) ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
      plan.secondaryDurationMs = TURN_90_DURATION_MS;
      plan.confidence = 1.0f;
      decisionSource = "local_head_on_wall";
      Serial.println("Head-on wall escape: local forced plan");
    } else if (isValidDistance(frontDistanceCm) && frontDistanceCm <= EMERGENCY_REVERSE_CM) {
      plan = defaultPlan();
      plan.primary = MANEUVER_BACKWARD;
      plan.primaryDurationMs = BACKUP_DURATION_MS;
      plan.secondary = (chooseFallbackTurn() == ACTION_LEFT) ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
      plan.secondaryDurationMs = TURN_90_DURATION_MS;
      plan.confidence = 1.0f;
      decisionSource = "local_emergency_close";
      Serial.println("Emergency close obstacle: local forced plan");
    } else if (shouldForceBackward(nowMs)) {
      plan = defaultPlan();
      plan.primary = MANEUVER_BACKWARD;
      plan.primaryDurationMs = BACKUP_DURATION_MS;
      plan.secondary = (chooseFallbackTurn() == ACTION_LEFT) ? MANEUVER_TURN_LEFT_90 : MANEUVER_TURN_RIGHT_90;
      plan.secondaryDurationMs = TURN_90_DURATION_MS;
      plan.confidence = 1.0f;
      decisionSource = "local_burst_escalation";
      Serial.println("Burst hazard escalation: local forced plan");
    } else {
      ManeuverPlan localPlan = chooseLocalPlan(repeatedTrap);
      if (shouldBypassFoundry(localPlan, repeatedTrap)) {
        plan = localPlan;
        decisionSource = "local_obvious_bypass";
        foundryRequestSent = false;
        foundryResponseOk = false;
        foundryPlanParsed = false;
        foundryDecisionStatus = decisionSource;
        Serial.println("Decision path: local obvious answer, Foundry bypassed");
      } else {
        Serial.println("Decision path: ambiguous/trap state, Foundry arbitration");
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
    }
    Serial.print("Executing plan source=");
    Serial.print(decisionSource);
    Serial.print(" primary=");
    Serial.print(maneuverTypeToString(plan.primary));
    Serial.print(" secondary=");
    Serial.println(maneuverTypeToString(plan.secondary));
    lastPlanFrontBeforeCm = frontDistanceCm;
    executePlan(plan);
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
