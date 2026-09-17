
/*
  llm-look-move.ino

  What this sketch does:
  - Drives a small robot car forward while continuously scanning distance with an
    ultrasonic sensor mounted on a pan servo.
  - Builds a live left/front/right distance picture from the sweep and detects
    near obstacles in front.
  - When a hazard is detected, it stops and decides how to avoid it using:
    1) hard safety rules (head-on wall, emergency close obstacle, hazard burst),
    2) Gemini API guidance (LEFT/RIGHT/BACKWARD/STOP),
    3) deterministic local fallback if Wi-Fi/API/JSON parsing fails.
  - Executes the selected maneuver, and if it had to reverse, performs a
    follow-up ~90-degree turn to establish a safer new heading.
  - Uses onboard LEDs to indicate "thinking" and left/right decision direction.

  Control strategy summary:
  - Normal state: keep moving forward.
  - Hazard state: stop, refresh a fresh L/F/R snapshot, choose action, maneuver.
  - Recovery state: clear hazard counters only after sustained clear distance,
    preventing rapid oscillation between forward and avoidance commands.

  Notes:
  - Sensor and motion behavior are tuned via constants below (thresholds, timing,
    servo arc, maneuver durations).
  - Unknown/invalid distance reads are handled defensively so autonomy continues
    even with intermittent sensor noise.
*/

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

// Robot hardware abstractions.
// `vehicle` controls drivetrain motions, `ultrasonic` reads obstacle distance,
// and `Servo` pans the ultrasonic sensor left/right for environmental scanning.
vehicle myCar;
ultrasonic sensor;
Servo panServo;

// High-level maneuver outcomes used by both local logic and Gemini output parsing.
enum Action {
  ACTION_STOP = 0,
  ACTION_LEFT,
  ACTION_RIGHT,
  ACTION_BACKWARD,
};

// Movement tuning constants.
// Speeds and timing determine how aggressively the platform avoids obstacles.
const int FORWARD_SPEED = 255;
const int TURN_SPEED = 240;
const unsigned long TURN_90_DURATION_MS = 520;

// Pin map for sensor, pan servo, and status LEDs.
const int TRIG_PIN = 13;
const int ECHO_PIN = 14;
const int ULTRASONIC_PAN_PIN = 27;
const int LEFT_LED_PIN = 2;
const int RIGHT_LED_PIN = 12;
const bool LED_ACTIVE_HIGH = true;

// Servo scan calibration:
// Increase/decrease PAN_CENTER_TRIM_DEG to align true front.
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
const unsigned long SENSOR_INTERVAL_MS = 70;
const unsigned long MANEUVER_DURATION_MS = 450;
const unsigned long GEMINI_HTTP_TIMEOUT_MS = 12000;
const unsigned long HAZARD_DECISION_COOLDOWN_MS = 800;
const unsigned long FRONT_STALE_MS = 450;
const unsigned long HAZARD_BURST_WINDOW_MS = 3000;
const uint8_t HAZARD_BURST_THRESHOLD = 2;
const unsigned long HAZARD_CLEAR_RESET_MS = 1200;
const int SAFE_UNKNOWN_DISTANCE_CM = 120;
const unsigned long PAN_SETTLE_MS = 120;
const unsigned long THINK_LED_ON_MS = 140;
const unsigned long THINK_LED_OFF_MS = 80;
const unsigned long DECISION_LED_MS = 220;

// Runtime state tracked across loop iterations.
// - obstacle flags drive stop/forward behavior
// - timestamps enforce non-blocking timing windows
// - burst counters escalate repeated hazard events
bool obstacleNearby = false;
bool previousObstacleNearby = false;
unsigned long lastSensorMs = 0;
unsigned long lastPanStepMs = 0;
unsigned long lastHazardDecisionMs = 0;
unsigned long hazardBurstWindowStartMs = 0;
uint8_t hazardBurstCount = 0;
unsigned long hazardClearSinceMs = 0;
Action lastNonStopDecision = ACTION_LEFT;

// Logical scan poses used for point measurements (distinct from smooth sweep motion).
enum ScanPose {
  SCAN_LEFT = 0,
  SCAN_CENTER,
  SCAN_RIGHT,
};
ScanPose currentScanPose = SCAN_CENTER;
bool panServoReady = false;
int panCurrentDeg = PAN_CENTER_DEG;
bool panSweepTowardLeft = true;
float leftDistanceCm = -1.0f;
float frontDistanceCm = -1.0f;
float rightDistanceCm = -1.0f;
unsigned long frontUpdatedMs = 0;

// Helper to switch both LEDs together (used for status signaling patterns).
void setBothLeds(bool on) {
  uint8_t level = on ? (LED_ACTIVE_HIGH ? HIGH : LOW) : (LED_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(LEFT_LED_PIN, level);
  digitalWrite(RIGHT_LED_PIN, level);
}

// Blink both LEDs in a short "thinking" pattern while waiting for Gemini decisions.
void flashGeminiThinkingLeds() {
  setBothLeds(true);
  delay(THINK_LED_ON_MS);
  setBothLeds(false);
  delay(THINK_LED_OFF_MS);
  setBothLeds(true);
  delay(THINK_LED_ON_MS);
  setBothLeds(false);
}

// Blink the LED that corresponds to the chosen turn direction.
// No blink for STOP/BACKWARD to keep directional meaning unambiguous.
void flashDecisionDirectionLed(Action decision) {
  int pin = -1;
  if (decision == ACTION_LEFT) {
    pin = LEFT_LED_PIN;
  } else if (decision == ACTION_RIGHT) {
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

// Robustly parse free-text model output into one of the supported actions.
// The parser is intentionally permissive (substring checks after uppercase transform)
// because LLM responses can include extra punctuation or explanation text.
Action parseActionFromText(const String &text) {
  String normalized = text;
  normalized.toUpperCase();
  if (normalized.indexOf("LEFT") >= 0) {
    return ACTION_LEFT;
  }
  if (normalized.indexOf("RIGHT") >= 0) {
    return ACTION_RIGHT;
  }
  if (normalized.indexOf("BACKWARD") >= 0 || normalized.indexOf("REVERSE") >= 0) {
    return ACTION_BACKWARD;
  }
  return ACTION_STOP;
}

// Connect to Wi-Fi with a bounded retry window so boot cannot hang forever.
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
    Serial.println("WiFi not connected. Hazard fallback=alternating turn");
  }
}

// Treat very small/invalid ultrasonic values as unreliable.
bool isValidDistance(float distanceCm) {
  return distanceCm >= ULTRASONIC_MIN_VALID_CM;
}

// Convert unknown readings into a conservative-but-drivable synthetic distance,
// allowing decision logic to continue when one side lacks a valid sample.
int effectiveDistance(float distanceCm) {
  if (isValidDistance(distanceCm)) {
    return (int)distanceCm;
  }
  return SAFE_UNKNOWN_DISTANCE_CM;
}

// Local deterministic fallback when Gemini is unavailable or uncertain.
// Prefer the side with more free space; alternate if tied to avoid oscillation.
Action chooseFallbackFromScan() {
  int leftEff = effectiveDistance(leftDistanceCm);
  int rightEff = effectiveDistance(rightDistanceCm);
  if (leftEff == rightEff) {
    return lastNonStopDecision == ACTION_LEFT ? ACTION_RIGHT : ACTION_LEFT;
  }
  return (leftEff > rightEff) ? ACTION_LEFT : ACTION_RIGHT;
}

// Detect a "wall directly ahead" pattern:
// front close + both sides also close + roughly balanced side distances.
// This helps force a reverse escape instead of choosing a weak lateral move.
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

// Ask Gemini for a maneuver recommendation using the latest distance snapshot.
// On any networking/API/parse issue, fallback logic preserves autonomous behavior.
Action queryGeminiForHazardDecision(float frontCm, float leftCm, float rightCm) {
  flashGeminiThinkingLeds();
  if (WiFi.status() != WL_CONNECTED) {
    return chooseFallbackFromScan();
  }
  WiFiClientSecure secureClient;
  secureClient.setInsecure();
  HTTPClient http;
  String url = String(GEMINI_URL) + "?key=" + String(GEMINI_API_KEY);
  if (!http.begin(secureClient, url)) {
    Serial.println("Gemini: HTTP begin failed");
    return chooseFallbackFromScan();
  }
  String prompt =
      "Obstacle detected. Front=" + String(frontCm, 1) + "cm, Left=" + String(leftCm, 1) +
      "cm, Right=" + String(rightCm, 1) +
      "cm. Reply with exactly one word: LEFT, RIGHT, BACKWARD, or STOP. "
      "Prefer turning toward the larger side distance. Avoid STOP unless totally blocked.";
  String body = String("{\"contents\":[{\"parts\":[{\"text\":\"") + prompt +
                "\"}]}],\"generationConfig\":{\"temperature\":0.2,\"maxOutputTokens\":8}}";
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(GEMINI_HTTP_TIMEOUT_MS);
  int statusCode = http.POST(body);
  String responseBody = http.getString();
  http.end();
  if (statusCode < 200 || statusCode >= 300) {
    Serial.print("Gemini HTTP error: ");
    Serial.println(statusCode);
    return chooseFallbackFromScan();
  }
  JsonDocument responseDoc;
  DeserializationError err = deserializeJson(responseDoc, responseBody);
  if (err) {
    Serial.print("Gemini JSON parse error: ");
    Serial.println(err.c_str());
    return chooseFallbackFromScan();
  }
  String modelText = responseDoc["candidates"][0]["content"]["parts"][0]["text"] | "";
  Action action = parseActionFromText(modelText);
  Serial.print("Gemini raw text: ");
  Serial.println(modelText);
  if (action == ACTION_STOP) {
    action = chooseFallbackFromScan();
  }
  if (action != ACTION_STOP) {
    lastNonStopDecision = action;
  }
  flashDecisionDirectionLed(action);
  Serial.print("Gemini decision: ");
  if (action == ACTION_LEFT) {
    Serial.println("LEFT");
  } else if (action == ACTION_RIGHT) {
    Serial.println("RIGHT");
  } else if (action == ACTION_BACKWARD) {
    Serial.println("BACKWARD");
  } else {
    Serial.println("STOP");
  }
  return action;
}

// Move pan servo to one of the canonical scan poses (left/center/right).
void movePanToPose(ScanPose pose) {
  if (!panServoReady) {
    return;
  }
  if (pose == SCAN_LEFT) {
    panCurrentDeg = PAN_LEFT_DEG;
  } else if (pose == SCAN_CENTER) {
    panCurrentDeg = PAN_CENTER_DEG;
  } else {
    panCurrentDeg = PAN_RIGHT_DEG;
  }
  panServo.write(panCurrentDeg);
}

// Cycles discrete scan poses if needed by external logic.
ScanPose nextPose(ScanPose pose) {
  if (pose == SCAN_LEFT) {
    return SCAN_CENTER;
  }
  if (pose == SCAN_CENTER) {
    return SCAN_RIGHT;
  }
  return SCAN_LEFT;
}

// Execute a short avoidance maneuver, then always stop at the end.
// Stops are explicit to prevent drift between control decisions.
void executeManeuver(Action action) {
  switch (action) {
    case ACTION_LEFT:
      // Normal left avoidance is a short lateral move.
      myCar.Move(Move_Left, TURN_SPEED);
      delay(MANEUVER_DURATION_MS);
      break;
    case ACTION_RIGHT:
      // Normal right avoidance is a short lateral move.
      myCar.Move(Move_Right, TURN_SPEED);
      delay(MANEUVER_DURATION_MS);
      break;
    case ACTION_BACKWARD:
      myCar.Move(Backward, TURN_SPEED);
      delay(MANEUVER_DURATION_MS);
      break;
    default:
      myCar.Move(Stop, 0);
      delay(200);
      break;
  }
  myCar.Move(Stop, 0);
}

// Rotate in place approximately 90 degrees to establish a new heading.
// Used after backing away from a blocked front path.
void executeTurn90(Action turnDirection) {
  if (turnDirection == ACTION_LEFT) {
    myCar.Move(Contrarotate, TURN_SPEED);
  } else {
    myCar.Move(Clockwise, TURN_SPEED);
  }
  delay(TURN_90_DURATION_MS);
  myCar.Move(Stop, 0);
}

// Escalation strategy for repeated hazards:
// multiple obstacle events within a short window trigger forced BACKWARD.
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

// Take an immediate point-in-time left/right/front snapshot before making a decision.
// This reduces stale-data decisions that can happen during continuous sweeps.
void refreshHazardScanSnapshot() {
  if (!panServoReady) {
    // Fallback if servo is unavailable: at least refresh front.
    float front = sensor.Ranging();
    if (isValidDistance(front)) {
      frontDistanceCm = front;
      frontUpdatedMs = millis();
    }
    return;
  }
  movePanToPose(SCAN_LEFT);
  delay(PAN_SETTLE_MS);
  float left = sensor.Ranging();
  if (isValidDistance(left)) {
    leftDistanceCm = left;
  }
  movePanToPose(SCAN_RIGHT);
  delay(PAN_SETTLE_MS);
  float right = sensor.Ranging();
  if (isValidDistance(right)) {
    rightDistanceCm = right;
  }
  movePanToPose(SCAN_CENTER);
  delay(PAN_SETTLE_MS);
  float front = sensor.Ranging();
  if (isValidDistance(front)) {
    frontDistanceCm = front;
    frontUpdatedMs = millis();
  } else {
    frontDistanceCm = -1.0f;
  }
  // Resume regular sweep from left so both sides continue getting refreshed.
  currentScanPose = SCAN_LEFT;
  panSweepTowardLeft = true;
  lastPanStepMs = millis();
  Serial.print("Decision scan L/F/R: ");
  Serial.print(leftDistanceCm, 1);
  Serial.print("/");
  Serial.print(frontDistanceCm, 1);
  Serial.print("/");
  Serial.println(rightDistanceCm, 1);
}
// Continuously sweep pan servo between right and left extremes.
// This keeps spatial context updated without blocking the main loop.
void updatePanSweep(unsigned long now) {
  if (!panServoReady) {
    return;
  }
  if (now - lastPanStepMs < PAN_STEP_INTERVAL_MS) {
    return;
  }
  lastPanStepMs = now;
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

// Main perception update:
// 1) sweep pan servo
// 2) sample ultrasonic
// 3) bucket sample into left/front/right by current servo angle
// 4) determine whether a fresh front hazard is present
void updateScanAndHazard() {
  unsigned long now = millis();
  updatePanSweep(now);
  if (now - lastSensorMs < SENSOR_INTERVAL_MS) {
    return;
  }
  lastSensorMs = now;
  float distanceCm = sensor.Ranging();
  if (isValidDistance(distanceCm)) {
    if (panCurrentDeg >= (PAN_CENTER_DEG + PAN_CENTER_BUCKET_DEG)) {
      leftDistanceCm = distanceCm;
    } else if (panCurrentDeg <= (PAN_CENTER_DEG - PAN_CENTER_BUCKET_DEG)) {
      rightDistanceCm = distanceCm;
    } else {
      frontDistanceCm = distanceCm;
      frontUpdatedMs = now;
    }
  } else if (panCurrentDeg > (PAN_CENTER_DEG - PAN_CENTER_BUCKET_DEG) &&
             panCurrentDeg < (PAN_CENTER_DEG + PAN_CENTER_BUCKET_DEG)) {
    // Front read failed at center pose; mark unknown so hazard does not latch forever.
    frontDistanceCm = -1.0f;
  }
  bool frontFresh = (frontUpdatedMs != 0 && (now - frontUpdatedMs) <= FRONT_STALE_MS);
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

// One-time initialization of serial logging, pins, car driver, ultrasonic sensor,
// pan servo calibration state, and Wi-Fi connection.
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
  movePanToPose(currentScanPose);
  Serial.print("Pan calibration L/C/R: ");
  Serial.print(PAN_LEFT_DEG);
  Serial.print("/");
  Serial.print(PAN_CENTER_DEG);
  Serial.print("/");
  Serial.println(PAN_RIGHT_DEG);
  delay(250);
  connectWiFi();
  Serial.println("Robot servo scan + Gemini hazard decisions enabled");
}

// Main control loop:
// - Continuously refresh scan/hazard state
// - When hazard triggers, stop and choose maneuver (rule-based or Gemini)
// - Apply burst/reset logic to prevent repeated near-collisions
// - Drive forward only while hazard-free
void loop() {
  updateScanAndHazard();
  unsigned long now = millis();
  if (obstacleNearby && (!previousObstacleNearby || (now - lastHazardDecisionMs) >= HAZARD_DECISION_COOLDOWN_MS)) {
    myCar.Move(Stop, 0);
    // Before deciding, force a fresh left-right-front snapshot.
    refreshHazardScanSnapshot();
    Action decision;
    if (isHeadOnWall(frontDistanceCm, leftDistanceCm, rightDistanceCm)) {
      decision = ACTION_BACKWARD;
      Serial.print("Head-on wall escape: forcing BACKWARD (L/F/R=");
      Serial.print(leftDistanceCm, 1);
      Serial.print("/");
      Serial.print(frontDistanceCm, 1);
      Serial.print("/");
      Serial.print(rightDistanceCm, 1);
      Serial.println(")");
    } else if (isValidDistance(frontDistanceCm) && frontDistanceCm <= EMERGENCY_REVERSE_CM) {
      decision = ACTION_BACKWARD;
      Serial.print("Emergency close obstacle: forcing BACKWARD (frontCm=");
      Serial.print(frontDistanceCm, 1);
      Serial.println(")");
    } else if (shouldForceBackward(now)) {
      decision = ACTION_BACKWARD;
      Serial.print("Hazard burst escalation: forcing BACKWARD (count=");
      Serial.print(hazardBurstCount);
      Serial.print(" windowMs=");
      Serial.print(HAZARD_BURST_WINDOW_MS);
      Serial.println(")");
    } else {
      decision = queryGeminiForHazardDecision(frontDistanceCm, leftDistanceCm, rightDistanceCm);
    }
    executeManeuver(decision);
    if (decision == ACTION_BACKWARD) {
      // Any backward move means front path is not viable; pick a new heading with a 90-degree turn.
      Action turnDirection = chooseFallbackFromScan();
      Serial.print("Post-backward 90 turn: ");
      Serial.println(turnDirection == ACTION_LEFT ? "LEFT" : "RIGHT");
      executeTurn90(turnDirection);
    }
    lastHazardDecisionMs = now;
    // Return to scan loop; forward resumes when hazard clears.
  }
  if (!obstacleNearby) {
    if (hazardClearSinceMs == 0) {
      hazardClearSinceMs = now;
    }
    // Clear burst state only after a sustained clear period.
    if ((now - hazardClearSinceMs) >= HAZARD_CLEAR_RESET_MS) {
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
