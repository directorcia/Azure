// =================================================
// llm-cellmove.ino
// ESP32 Grid Rover — LLM-guided navigation
//
// Overview:
//   This sketch drives a 4-wheeled robot car across a virtual 5×5 grid.
//   At each step the robot asks a locally-hosted Ollama LLM (e.g. llama3.1:8b)
//   which direction to move in order to reach its current destination.
//   The robot shuttles back and forth between two fixed waypoints:
//     • Origin      (0, 0)  – home position
//     • Patrol point (4, 3) – far waypoint
//   If the LLM is unreachable or returns an invalid move, a deterministic
//   Manhattan-distance fallback ensures the robot always makes progress.
//
// Dependencies (install via Arduino Library Manager or PlatformIO):
//   • Arduino.h      – core Arduino API
//   • WiFi.h         – ESP32 built-in Wi-Fi driver
//   • HTTPClient.h   – ESP32 built-in HTTP client
//   • ArduinoJson    – JSON serialisation / deserialisation (v7+)
//   • vehicle.h      – custom motor-shield wrapper for this robot chassis
// =================================================
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <vehicle.h>
// =================================================
// WIFI SETTINGS
// Replace these placeholders with your actual network credentials before flashing.
// WIFI_SSID     – the name (SSID) of your 2.4 GHz Wi-Fi network.
// WIFI_PASSWORD – the WPA/WPA2 passphrase for that network.
// =================================================
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
// =================================================
// OLLAMA SETTINGS
// Ollama is an open-source local LLM server (https://ollama.com).
// OLLAMA_URL   – full URL to the /api/generate endpoint on the machine
//                running Ollama.  Replace YOUR_OLLAMA_HOST with that
//                machine's LAN IP address (e.g. 192.168.1.100).
//                Port 11434 is Ollama's default HTTP port.
// OLLAMA_MODEL – the model tag to run.  llama3.1:8b works well and fits
//                in ~5 GB RAM.  Swap for any other pulled model.
// =================================================
const char *OLLAMA_URL = "http://YOUR_OLLAMA_HOST:11434/api/generate";
const char *OLLAMA_MODEL = "llama3.1:8b";
// =================================================
// HARDWARE
// myCar – global instance of the vehicle motor-shield library.
//         Provides high-level methods: Init(), Move(), MoveBalanced().
//         The underlying motor-shield maps direction constants (Forward,
//         Backward, Move_Left, Move_Right, Stop) to H-bridge PWM signals.
// =================================================
vehicle myCar;
// =================================================
// GRID STATE
// The robot navigates a logical 5×5 grid (X: 0–4, Y: 0–4).
//   • X increases going EAST; Y increases going NORTH.
//   • The robot tracks its position and destination as integer cell coords.
//   • Physical movement for one cell = STEP_MOVE_MS milliseconds of motor run.
// =================================================

// Current robot position on the grid (updated after each successful move).
int robotX = 0;
int robotY = 0;

// Fixed waypoints the robot patrols between.
const int originX = 0;   // Home position – bottom-left corner of the grid.
const int originY = 0;
const int patrolX = 4;   // Far patrol point.
const int patrolY = 3;

// Active destination – starts at the patrol point on boot.
int targetX = patrolX;
int targetY = patrolY;

// Tracks which leg of the patrol the robot is on.
// true  = heading to patrol point; false = heading back to origin.
bool headingToPatrol = true;

// Timestamp (millis) of the last LLM request; used to throttle requests.
unsigned long lastRequest = 0;

// How long to wait between movement steps (milliseconds).
// Increase this if the robot needs more time for the LLM to respond.
const unsigned long REQUEST_INTERVAL = 5000;

// =================================================
// HTTP / OLLAMA TUNING
// =================================================

// Per-request HTTP timeout (ms). 60 s is generous for a warm LLM.
const uint16_t HTTP_TIMEOUT_MS = 60000;

// Total time budget (ms) allowed for the very first request while Ollama
// loads the model into VRAM ("cold start").  180 s covers most hardware.
// The code splits this into multiple shorter attempts because
// HTTPClient::setTimeout() is a uint16_t and cannot exceed ~65 s.
const uint32_t HTTP_COLD_START_BUDGET_MS = 180000;

// Maximum number of HTTP retry attempts on a warm connection.
const int MAX_HTTP_ATTEMPTS = 2;

// Flag set to true after the first successful Ollama response so that
// subsequent requests use the shorter HTTP_TIMEOUT_MS instead of cold-start.
bool ollamaWarmed = false;

// =================================================
// MOTOR TUNING
// Physical motors are never perfectly matched, so trim factors compensate
// for any left/right speed imbalance that causes the robot to veer.
// Measure straight-line travel and adjust these until the robot tracks true.
// =================================================

// Base PWM duty cycle (0–255) applied to both motors before trim.
const uint8_t BASE_MOTOR_SPEED = 150;

// Forward trim multipliers applied to left/right motor speeds.
// Values < 1.0 slow that side down; 1.0 means full base speed.
const float LEFT_MOTOR_TRIM = 0.66f;   // Left motor runs slower forward.
const float RIGHT_MOTOR_TRIM = 1.0f;   // Right motor at full base speed.

// Separate trim values for reverse because motor back-EMF characteristics
// often differ from forward drive.
const float LEFT_MOTOR_TRIM_REVERSE = 0.82f;
const float RIGHT_MOTOR_TRIM_REVERSE = 1.0f;

// Duration of motor run for a single grid-cell step (milliseconds).
// Tune this so one STEP_MOVE_MS burst moves the robot exactly one cell.
const uint16_t STEP_MOVE_MS = 2000;

// =================================================
// MOTOR START-BOOST
// Motors need a brief over-voltage pulse to overcome static friction before
// settling to the cruise PWM.  Two sets: one for forward, one for reverse.
// =================================================

// Forward boost: extra PWM added on top of the cruise speed at move start.
const uint8_t MOTOR_START_BOOST_PWM = 25;
// How long the forward boost pulse lasts (ms).
const uint16_t MOTOR_START_BOOST_MS = 140;

// Reverse minimum PWM – prevents stalling if trim would drop speed too low.
const uint8_t REVERSE_MIN_PWM = 125;
// Extra PWM added at the start of a reverse move.
const uint8_t REVERSE_BOOST_PWM = 40;
// Duration of the reverse boost pulse (ms).
const uint16_t REVERSE_BOOST_MS = 220;

// =================================================
// BUZZER
// A passive (or active) buzzer on BUZZER_PIN provides audio feedback:
//   • 1 beep  = startup
//   • 2 beeps = waypoint reached, switching destination
// =================================================

// GPIO pin number the buzzer is wired to.
const uint8_t BUZZER_PIN = 33;
// Tone frequency in Hz (2000 Hz is a clear mid-range beep).
const int BUZZER_FREQ_HZ = 2000;

// =================================================
// MOTOR SELF-TEST
// When RUN_MOTOR_SELF_TEST_ON_BOOT is true, the robot briefly drives each
// motor/direction in sequence on startup so you can verify wiring.
// Set to false for normal operation.
// =================================================
const bool RUN_MOTOR_SELF_TEST_ON_BOOT = false;
// Duration of each direction burst during the self-test (ms).
const uint16_t SELF_TEST_STEP_MS = 450;

// =================================================
// MOTOR DIRECTION BIT-FLAGS
// The vehicle library uses bitmask constants to select which motors to drive
// and in which direction.  Each motor has a forward and backward bit.
//   Motor 1 (M1): bits 128 (fwd) and 64  (rev)
//   Motor 2 (M2): bits 32  (fwd) and 16  (rev)
//   Motor 3 (M3): bits 2   (fwd) and 4   (rev)
//   Motor 4 (M4): bits 1   (fwd) and 8   (rev)
// These are used only during the self-test to drive individual motors.
// =================================================
const int M1_Forward = 128;
const int M1_Backward = 64;
const int M2_Forward = 32;
const int M2_Backward = 16;
const int M3_Forward = 2;
const int M3_Backward = 4;
const int M4_Forward = 1;
const int M4_Backward = 8;
// -------------------------------------------------
// playToneHz(pin, freq, durationMs)
// Generates a software square wave on the given GPIO pin to drive a passive
// buzzer (or any active buzzer that accepts a PWM tone signal).
//
// How it works:
//   A square wave at frequency F Hz has a period of (1,000,000 / F) µs.
//   We toggle the pin HIGH then LOW for half that period each cycle,
//   repeating for as many full cycles as fit inside durationMs.
//
// Parameters:
//   pin        – GPIO pin number connected to the buzzer.
//   freq       – Desired tone frequency in Hz (e.g. 2000 = 2 kHz beep).
//   durationMs – How long to play the tone in milliseconds.
// -------------------------------------------------
void playToneHz(int pin, int freq, int durationMs) {
  // Guard: skip if frequency or duration are nonsensical.
  if (freq <= 0 || durationMs <= 0) return;
  pinMode(pin, OUTPUT);
  // Period of one complete cycle in microseconds.
  unsigned long periodUs = 1000000UL / (unsigned long)freq;
  // Total number of complete cycles that fit in the requested duration.
  unsigned long cycles = ((unsigned long)durationMs * 1000UL) / periodUs;
  for (unsigned long i = 0; i < cycles; i++) {
    digitalWrite(pin, HIGH);             // High half of the cycle.
    delayMicroseconds(periodUs / 2);
    digitalWrite(pin, LOW);              // Low half of the cycle.
    delayMicroseconds(periodUs / 2);
  }
}
// -------------------------------------------------
// playBuzzerPattern(count, onMs, offMs)
// Plays a repeated beep pattern – useful for audible status feedback.
//
// Parameters:
//   count – Number of beeps to play.
//   onMs  – Duration of each beep (milliseconds).
//   offMs – Silence between beeps (milliseconds); ignored after the last beep.
//
// Usage examples:
//   playBuzzerPattern(1, 300, 0);    // One 300 ms startup beep.
//   playBuzzerPattern(2, 180, 120);  // Two short beeps when a waypoint is reached.
// -------------------------------------------------
void playBuzzerPattern(uint8_t count, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < count; i++) {
    playToneHz(BUZZER_PIN, BUZZER_FREQ_HZ, onMs);  // Sound the tone.
    digitalWrite(BUZZER_PIN, LOW);                  // Ensure pin is low after tone.
    if (i < count - 1) {
      delay(offMs);   // Pause between beeps (not after the final one).
    }
  }
}
// -------------------------------------------------
// moveForMs(label, dir, runMs)  [self-test helper]
// Drives the robot in a single direction for a fixed duration, then stops.
// Used exclusively by runMotorSelfTest() to verify each motor direction.
//
// Parameters:
//   label  – Human-readable direction name printed to Serial for diagnostics.
//   dir    – Direction constant (Forward, Backward, Move_Left, Move_Right, or
//            an individual motor bitmask) passed to the vehicle library.
//   runMs  – How long to run the motors (milliseconds).
// -------------------------------------------------
void moveForMs(const char *label, int dir, uint16_t runMs) {
  // Apply forward trim factors to balance left and right motor speeds.
  int leftSpeed  = constrain((int)(BASE_MOTOR_SPEED * LEFT_MOTOR_TRIM),  0, 255);
  int rightSpeed = constrain((int)(BASE_MOTOR_SPEED * RIGHT_MOTOR_TRIM), 0, 255);
  Serial.printf("Motor test: %s | Dir=%d | R=%d L=%d | %u ms\n", label, dir, rightSpeed,
                leftSpeed, runMs);
  myCar.MoveBalanced(dir, rightSpeed, leftSpeed);  // Start moving.
  delay(runMs);                                    // Run for the specified duration.
  myCar.Move(Stop, 0);                             // Stop all motors.
  delay(200);                                      // Brief pause before the next test step.
}
// -------------------------------------------------
// runMotorSelfTest()
// Sequentially drives every motor and direction for SELF_TEST_STEP_MS ms.
// Lets you verify that all four motors are wired correctly and spinning in
// the expected direction before running the full LLM navigation loop.
//
// Tip: Lift the robot off the ground before enabling this test to avoid
// unintended movement across your work surface.
// Enabled by setting RUN_MOTOR_SELF_TEST_ON_BOOT = true.
// -------------------------------------------------
void runMotorSelfTest() {
  Serial.println("Starting motor self-test (lift wheels off ground if possible)...");
  // Test compound directions (all four wheels acting together).
  moveForMs("FORWARD",    Forward,    SELF_TEST_STEP_MS);
  moveForMs("BACKWARD",   Backward,   SELF_TEST_STEP_MS);
  moveForMs("MOVE_RIGHT", Move_Right, SELF_TEST_STEP_MS);
  moveForMs("MOVE_LEFT",  Move_Left,  SELF_TEST_STEP_MS);
  // Test each individual motor in both directions.
  moveForMs("M1 forward",  M1_Forward,  SELF_TEST_STEP_MS);
  moveForMs("M1 backward", M1_Backward, SELF_TEST_STEP_MS);
  moveForMs("M2 forward", M2_Forward, SELF_TEST_STEP_MS);
  moveForMs("M2 backward", M2_Backward, SELF_TEST_STEP_MS);
  moveForMs("M3 forward", M3_Forward, SELF_TEST_STEP_MS);
  moveForMs("M3 backward", M3_Backward, SELF_TEST_STEP_MS);
  moveForMs("M4 forward", M4_Forward, SELF_TEST_STEP_MS);
  moveForMs("M4 backward", M4_Backward, SELF_TEST_STEP_MS);
  myCar.Move(Stop, 0);
  Serial.println("Motor self-test complete.");
}
// =================================================
// -------------------------------------------------
// getValidMovesForCurrentPosition()
// Returns a space-separated string listing only the moves that keep the
// robot inside the 5×5 grid from its current position.
//
// This string is injected into the LLM prompt so the model never suggests
// a move that would walk off the edge of the grid.
//
// Example: if the robot is at (0,2) the result is "MOVE_NORTH MOVE_SOUTH MOVE_EAST"
// because MOVE_WEST would require X=-1 which is out of bounds.
// -------------------------------------------------
String getValidMovesForCurrentPosition() {
  String moves = "";
  if (robotY < 4) moves += "MOVE_NORTH ";  // Can move north if not at top row.
  if (robotY > 0) moves += "MOVE_SOUTH ";  // Can move south if not at bottom row.
  if (robotX < 4) moves += "MOVE_EAST ";   // Can move east if not at right column.
  if (robotX > 0) moves += "MOVE_WEST ";   // Can move west if not at left column.
  moves.trim();  // Remove the trailing space after the last entry.
  return moves;
}
// -------------------------------------------------
// getFallbackAction()
// Deterministic fallback navigator used when the LLM is unavailable,
// returns an invalid move, or times out.
//
// Strategy: greedy Manhattan-distance reduction.
//   1. Close the X gap first (move east or west).
//   2. Then close the Y gap (move north or south).
//   3. Return STOP only if already at the target.
//
// This guarantees the robot always makes progress even without the LLM.
// -------------------------------------------------
String getFallbackAction() {
  if (robotX < targetX) return "MOVE_EAST";   // Target is to the right.
  if (robotX > targetX) return "MOVE_WEST";   // Target is to the left.
  if (robotY < targetY) return "MOVE_NORTH";  // Target is above.
  if (robotY > targetY) return "MOVE_SOUTH";  // Target is below.
  return "STOP";                               // Already at destination.
}
// -------------------------------------------------
// manhattanDistance(x1, y1, x2, y2)
// Returns the Manhattan ("taxicab") distance between two grid cells.
// On a grid where you can only move in 4 cardinal directions, this is the
// minimum number of steps required to travel from (x1,y1) to (x2,y2).
// Used by isActionValidAndImproving() to verify that the LLM's chosen move
// actually reduces the distance to the target rather than wasting a step.
// -------------------------------------------------
int manhattanDistance(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}
// -------------------------------------------------
// isActionValidAndImproving(action)
// Safety gate that validates the action chosen by the LLM before the robot
// commits to executing it.  Returns true only if the action:
//   1. Is a recognised command string.
//   2. Keeps the resulting position within the 5×5 grid bounds.
//   3. Strictly reduces the Manhattan distance to the current target
//      (i.e. the robot gets closer — sideways or backward moves are rejected).
//
// STOP is accepted only when the robot is already at the target, which is
// the only situation where stopping is the correct action.
//
// Any action that fails these checks is replaced by the deterministic
// fallback from getFallbackAction().
// -------------------------------------------------
bool isActionValidAndImproving(const String &action) {
  // STOP is valid only if the robot has already reached its destination.
  if (action == "STOP") {
    return robotX == targetX && robotY == targetY;
  }
  // Simulate where this action would place the robot.
  int nextX = robotX;
  int nextY = robotY;
  if      (action == "MOVE_EAST")  { nextX++; }
  else if (action == "MOVE_WEST")  { nextX--; }
  else if (action == "MOVE_NORTH") { nextY++; }
  else if (action == "MOVE_SOUTH") { nextY--; }
  else {
    return false;  // Unknown action string — reject immediately.
  }
  // Reject moves that leave the grid boundaries.
  if (nextX < 0 || nextX > 4 || nextY < 0 || nextY > 4) {
    return false;
  }
  // Accept the action only if it strictly closes the gap to the target.
  int currentDist = manhattanDistance(robotX, robotY, targetX, targetY);
  int nextDist    = manhattanDistance(nextX,   nextY,  targetX, targetY);
  return nextDist < currentDist;
}
// -------------------------------------------------
// showGridState(action)
// Prints a compact status line to the Serial monitor after each move.
// Useful for tracking progress when watching the robot over a USB connection.
// Output format:
//   Current location: X,Y | Target: TX,TY
//   Next choice: ACTION
// -------------------------------------------------
void showGridState(const String &action) {
  Serial.printf("Current location: %d,%d | Target: %d,%d\nNext choice: %s\n", robotX, robotY,
                targetX, targetY, action.c_str());
}
// -------------------------------------------------
// applyAction(action)
// Translates a logical grid action ("MOVE_NORTH" etc.) into physical motor
// commands, executes the move, then updates the robot's tracked position.
//
// Motor drive profile for each step:
//   Phase 1 – Boost:  run at (cruise + boost) PWM for startBoostMs to overcome
//                     static friction and get the wheels rolling quickly.
//   Phase 2 – Cruise: drop to the trimmed cruise PWM for the remaining time
//                     until STEP_MOVE_MS has elapsed.
//   Phase 3 – Stop:   cut all motors to brake the robot at the cell boundary.
//
// Separate trim and boost values are used for reverse (Backward) because
// most DC motors behave asymmetrically under back-EMF.
//
// If the action is blocked (e.g. already at a grid edge) the robot stops and
// the position is left unchanged — the fallback logic will retry next cycle.
// -------------------------------------------------
void applyAction(const String &action) {
  // Record position before the move so we can report whether it changed.
  int beforeX = robotX;
  int beforeY = robotY;

  int moveCommand = Stop;   // Vehicle library direction constant.
  bool shouldMove = false;  // Set true only if the move is grid-legal.

  // Map the action string to a vehicle direction and update the logical position.
  // Each branch also checks the boundary to prevent moving off the grid.
  if (action == "MOVE_EAST" && robotX < 4) {
    moveCommand = Move_Right;
    shouldMove = true;
    robotX++;  // Advance one cell east.
  } else if (action == "MOVE_WEST" && robotX > 0) {
    moveCommand = Move_Left;
    shouldMove = true;
    robotX--;  // Retreat one cell west.
  } else if (action == "MOVE_NORTH" && robotY < 4) {
    moveCommand = Forward;
    shouldMove = true;
    robotY++;  // Advance one cell north.
  } else if (action == "MOVE_SOUTH" && robotY > 0) {
    moveCommand = Backward;
    shouldMove = true;
    robotY--;  // Retreat one cell south.
  }

  if (shouldMove) {
    // Select the correct trim and boost parameters based on direction.
    bool isReverse = (moveCommand == Backward);
    float leftTrim  = isReverse ? LEFT_MOTOR_TRIM_REVERSE  : LEFT_MOTOR_TRIM;
    float rightTrim = isReverse ? RIGHT_MOTOR_TRIM_REVERSE : RIGHT_MOTOR_TRIM;

    // Calculate trimmed cruise speeds and clamp to valid PWM range.
    int leftSpeed  = constrain((int)(BASE_MOTOR_SPEED * leftTrim),  0, 255);
    int rightSpeed = constrain((int)(BASE_MOTOR_SPEED * rightTrim), 0, 255);

    // Pick boost parameters for this direction.
    uint8_t  startBoostPwm = isReverse ? REVERSE_BOOST_PWM      : MOTOR_START_BOOST_PWM;
    uint16_t startBoostMs  = isReverse ? REVERSE_BOOST_MS       : MOTOR_START_BOOST_MS;

    // For reverse, ensure cruise speed never drops below the minimum
    // needed to overcome motor stall torque.
    if (isReverse) {
      leftSpeed  = max(leftSpeed,  (int)REVERSE_MIN_PWM);
      rightSpeed = max(rightSpeed, (int)REVERSE_MIN_PWM);
    }

    // Add the boost pulse on top of the cruise speed (clamped to 0–255).
    int leftBoostSpeed  = constrain(leftSpeed  + startBoostPwm, 0, 255);
    int rightBoostSpeed = constrain(rightSpeed + startBoostPwm, 0, 255);

    // Split the total step time into boost phase and cruise phase.
    uint16_t boostMs  = min(STEP_MOVE_MS, startBoostMs);
    uint16_t cruiseMs = STEP_MOVE_MS - boostMs;

    Serial.printf("Applying action: %s | Dir=%d | R=%d L=%d | %u ms%s\n", action.c_str(),
                  moveCommand, rightSpeed, leftSpeed, STEP_MOVE_MS,
                  isReverse ? " | reverse torque assist" : "");

    // Phase 1: boost – higher PWM to overcome static friction.
    myCar.MoveBalanced(moveCommand, rightBoostSpeed, leftBoostSpeed);
    delay(boostMs);

    // Phase 2: cruise – settle to the trimmed steady-state speed.
    if (cruiseMs > 0) {
      myCar.MoveBalanced(moveCommand, rightSpeed, leftSpeed);
      delay(cruiseMs);
    }

    // Phase 3: stop – cut motors at the end of the cell step.
    myCar.Move(Stop, 0);

  } else {
    // Action was blocked by a boundary check – log and stay put.
    Serial.printf("Action blocked: %s at position %d,%d (target %d,%d)\n", action.c_str(),
                  robotX, robotY, targetX, targetY);
    myCar.Move(Stop, 0);
  }

  // Report whether the robot's logical position actually changed.
  if (beforeX == robotX && beforeY == robotY) {
    Serial.println("Move result: no position change");
  } else {
    Serial.printf("Move result: %d,%d -> %d,%d\n", beforeX, beforeY, robotX, robotY);
  }
}
// -------------------------------------------------
// updateDestinationIfReached()
// Checks whether the robot has arrived at its current target.
// If so, it plays a two-beep confirmation tone and flips the destination
// to the opposite waypoint, creating an endless back-and-forth patrol:
//
//   Origin (0,0)  <-->  Patrol point (4,3)
//
// This function is called after every move so the robot immediately
// starts navigating toward the new target on the very next loop cycle.
// -------------------------------------------------
void updateDestinationIfReached() {
  // Nothing to do if the robot hasn't arrived yet.
  if (robotX != targetX || robotY != targetY) {
    return;
  }

  // Arrival confirmation: two short beeps.
  playBuzzerPattern(2, 180, 120);

  // Toggle the target to the other waypoint.
  if (headingToPatrol) {
    Serial.println("Reached patrol point, returning to origin.");
    targetX = originX;
    targetY = originY;
    headingToPatrol = false;
  } else {
    Serial.println("Reached origin, returning to patrol point.");
    targetX = patrolX;
    targetY = patrolY;
    headingToPatrol = true;
  }
}
// -------------------------------------------------
// connectWiFi()
// Connects the ESP32 to the Wi-Fi network defined by WIFI_SSID/WIFI_PASSWORD.
// Blocks until the connection is established, printing a dot every 500 ms
// so you can monitor progress on the Serial monitor.
// After connecting it prints the assigned IP address (useful for verifying
// the device is on the correct subnet to reach the Ollama server).
// This function is also called from getNextAction() to reconnect if the
// Wi-Fi link drops mid-session.
// -------------------------------------------------
void connectWiFi() {
  WiFi.mode(WIFI_STA);                      // Station mode (client, not access point).
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);     // Start the connection attempt.
  Serial.print("Connecting");
  // Poll until the ESP32 reports a successful association and DHCP lease.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');  // Progress indicator.
  }
  Serial.println();
  Serial.println("Connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP().toString());  // Print the DHCP-assigned IP.
  delay(2000);  // Brief pause to let the stack stabilise before HTTP use.
}
// -------------------------------------------------
// getNextAction()
// Queries the Ollama LLM server for the next movement action and returns
// a validated action string ("MOVE_NORTH", "MOVE_SOUTH", etc.).
//
// Flow:
//   1. Ensure Wi-Fi is connected; reconnect if the link dropped.
//   2. Build a plain-English prompt that tells the LLM:
//        • The robot's current grid position.
//        • The destination cell.
//        • The coordinate axis conventions.
//        • Which moves are currently in-bounds.
//        • That it must respond with a single JSON object {"action":"..."}.
//   3. POST the request to the Ollama /api/generate endpoint.
//   4. On the first call (cold start), the model must load into memory which
//      can take up to HTTP_COLD_START_BUDGET_MS ms. This is handled by
//      splitting the budget into multiple shorter attempts (each ≤ 65 s)
//      because HTTPClient::setTimeout() takes a uint16_t.
//   5. Parse the response:
//        a. Try strict JSON parsing: extract res["response"] then parse the
//           inner JSON for the "action" key.
//        b. If strict parse fails, fall back to substring search for any
//           recognised action keyword in the raw response text.
//   6. Validate the parsed action with isActionValidAndImproving().
//      If invalid, or if any HTTP/JSON error occurs, fall back to the
//      deterministic getFallbackAction().
//
// LLM parameters used:
//   temperature = 0.2  – low temperature for more deterministic output.
//   num_predict = 16   – tiny token limit; we only need a short JSON object.
//   keep_alive  = 30m  – keep the model loaded in VRAM between requests.
// -------------------------------------------------
String getNextAction() {
  // Ensure the network is up before attempting any HTTP calls.
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi dropped, reconnecting...");
    connectWiFi();
  }

  Serial.println("Querying Ollama for next move...");

  // On the first call the model may not yet be loaded into VRAM (cold start).
  // We use a longer per-attempt timeout and allow more retries to cover the
  // full cold-start budget without exceeding the uint16_t limit of setTimeout.
  uint16_t requestTimeoutMs = HTTP_TIMEOUT_MS;
  int maxAttempts = MAX_HTTP_ATTEMPTS;
  if (!ollamaWarmed) {
    // Cap each attempt at 65 s (safe uint16_t ceiling) and calculate how many
    // attempts are needed to cover the full cold-start budget.
    requestTimeoutMs = 65000;
    maxAttempts = (HTTP_COLD_START_BUDGET_MS + requestTimeoutMs - 1) / requestTimeoutMs;
    if (maxAttempts < 1) { maxAttempts = 1; }
    Serial.printf("Cold start budget: %lu ms (%d attempts x %u ms)\n",
                  HTTP_COLD_START_BUDGET_MS, maxAttempts, requestTimeoutMs);
  }

  // ---- Build the JSON request body ----
  JsonDocument req;
  req["model"] = OLLAMA_MODEL;

  // Construct the natural-language prompt.
  // Keeping the prompt concise reduces token usage and response latency.
  String prompt = "You are controlling a robot on a 5 by 5 grid.\n\n";
  prompt += "Current robot position:\n";
  prompt += "X=" + String(robotX) + "\n";
  prompt += "Y=" + String(robotY) + "\n\n";
  prompt += "Destination:\n";
  prompt += "X=" + String(targetX) + "\n";
  prompt += "Y=" + String(targetY) + "\n\n";
  prompt += "Rules:\n";
  prompt += "X increases to the EAST.\n";
  prompt += "Y increases to the NORTH.\n";
  prompt += "Grid limits are 0<=X<=4 and 0<=Y<=4.\n";
  prompt += "Never choose a move that would place X or Y outside those limits.\n";
  prompt += "Only choose valid in-bounds moves.\n\n";
  // Provide only the moves that are actually legal from the current cell.
  prompt += "Valid moves from current position:\n";
  prompt += getValidMovesForCurrentPosition() + "\n\n";
  prompt += "Allowed actions:\n";
  prompt += "MOVE_NORTH\n";
  prompt += "MOVE_SOUTH\n";
  prompt += "MOVE_EAST\n";
  prompt += "MOVE_WEST\n";
  prompt += "STOP\n\n";
  prompt += "Choose exactly one action that gets closer to the destination.\n";
  prompt += "Never choose a move that increases or keeps the same Manhattan distance.\n";
  // Ask for JSON output so parsing is straightforward.  Provide an example
  // to anchor the model's output format.
  prompt += "Return JSON only:\n";
  prompt += "{\"action\":\"MOVE_NORTH\"}";

  req["prompt"]     = prompt;
  req["stream"]     = false;   // Collect the full response before returning.
  req["keep_alive"] = "30m";   // Keep model in VRAM for 30 minutes.

  // Low temperature → more deterministic token selection (less creative).
  JsonObject options = req["options"].to<JsonObject>();
  options["temperature"] = 0.2;
  options["num_predict"]  = 16;  // We need at most ~10 tokens: {"action":"MOVE_NORTH"}

  // Serialise the request object to a JSON string for the HTTP POST body.
  String body;
  serializeJson(req, body);

  // ---- HTTP request loop (with retries) ----
  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    HTTPClient http;

    // Begin the connection to the Ollama endpoint.
    if (!http.begin(OLLAMA_URL)) {
      Serial.println("HTTP begin failed");
      String fallback = getFallbackAction();
      Serial.print("Fallback action: "); Serial.println(fallback);
      return fallback;
    }

    http.addHeader("Content-Type", "application/json");  // Required by Ollama API.
    http.setTimeout(requestTimeoutMs);

    Serial.printf("Sending Ollama request (attempt %d/%d)...\n", attempt, maxAttempts);
    unsigned long t0 = millis();
    int httpCode = http.POST(body);  // Send the prompt and wait for the response.
    Serial.printf("POST returned in %lu ms with code %d\n", millis() - t0, httpCode);

    // Handle HTTP errors (network failure, server error, timeout).
    if (httpCode != HTTP_CODE_OK) {
      Serial.printf("HTTP Error %d (attempt %d/%d)\n", httpCode, attempt, maxAttempts);
      Serial.println(http.errorToString(httpCode));
      http.end();
      if (attempt < maxAttempts) { delay(500); continue; }  // Retry.
      String fallback = getFallbackAction();
      Serial.print("Fallback action: "); Serial.println(fallback);
      return fallback;
    }

    // Successful HTTP response – read the body.
    String response = http.getString();
    http.end();
    Serial.println("Ollama response received.");
    ollamaWarmed = true;  // Mark model as loaded so future calls use shorter timeout.

    // ---- Parse the Ollama response envelope ----
    // Ollama wraps the model output in:  { "response": "<model text>", ... }
    JsonDocument res;
    DeserializationError err = deserializeJson(res, response);
    if (err) {
      Serial.printf("JSON parse error (attempt %d/%d): %s\n", attempt, maxAttempts, err.c_str());
      if (attempt < maxAttempts) { delay(300); continue; }  // Retry.
      String fallback = getFallbackAction();
      Serial.print("Fallback action: "); Serial.println(fallback);
      return fallback;
    }

    // Extract the model's text output from the "response" field.
    String actionText = res["response"] | "";
    actionText.trim();

    // ---- Attempt 1: strict JSON parse of the model output ----
    // Expected format: {"action":"MOVE_NORTH"}
    JsonDocument actionDoc;
    if (deserializeJson(actionDoc, actionText) == DeserializationError::Ok) {
      String action = actionDoc["action"] | "STOP";
      action.trim();
      if (isActionValidAndImproving(action)) {
        return action;  // Valid JSON action accepted.
      }
      // The JSON parsed but the action is invalid or non-improving.
      Serial.print("Rejected LLM action: "); Serial.println(action);
      String fallback = getFallbackAction();
      Serial.print("Fallback action: "); Serial.println(fallback);
      return fallback;
    }

    // ---- Attempt 2: keyword search in raw response text ----
    // The model sometimes wraps its answer in prose (e.g. "I recommend MOVE_NORTH").
    // Search for any recognised keyword and extract it.
    String parsedAction = "STOP";  // Default if no keyword is found.
    if      (actionText.indexOf("MOVE_NORTH") >= 0) parsedAction = "MOVE_NORTH";
    else if (actionText.indexOf("MOVE_SOUTH") >= 0) parsedAction = "MOVE_SOUTH";
    else if (actionText.indexOf("MOVE_EAST")  >= 0) parsedAction = "MOVE_EAST";
    else if (actionText.indexOf("MOVE_WEST")  >= 0) parsedAction = "MOVE_WEST";

    if (isActionValidAndImproving(parsedAction)) {
      return parsedAction;  // Keyword-extracted action accepted.
    }

    // Keyword action is also invalid — fall back to deterministic logic.
    Serial.print("Rejected LLM action: "); Serial.println(parsedAction);
    String fallback = getFallbackAction();
    Serial.print("Fallback action: "); Serial.println(fallback);
    return fallback;
  }

  // All retry attempts exhausted – use the deterministic fallback.
  String fallback = getFallbackAction();
  Serial.print("Fallback action: "); Serial.println(fallback);
  return fallback;
}
// =================================================
// setup()
// Arduino entry point – runs once after power-on or reset.
// Initialisation sequence:
//   1. Open Serial at 115200 baud for debug output.
//   2. Initialise the motor controller and ensure motors are stopped.
//   3. Configure the buzzer pin and play a startup beep.
//   4. Optionally run the motor self-test (see RUN_MOTOR_SELF_TEST_ON_BOOT).
//   5. Connect to Wi-Fi.
//   6. Perform the very first LLM query and execute the resulting move.
//      Running one move in setup() means the robot begins navigating
//      immediately rather than waiting for the first loop() interval.
// =================================================
void setup() {
  Serial.begin(115200);   // Open USB serial at 115200 baud.
  delay(1000);            // Allow time for the serial monitor to connect.

  myCar.Init();           // Initialise the motor shield hardware.
  myCar.Move(Stop, 0);    // Ensure all motors are stopped at startup.

  // Configure the buzzer pin and make sure it starts silent.
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("ESP32 Grid Rover");
  playBuzzerPattern(1, 300, 0);  // Single startup beep.

  // Optional motor self-test (disabled by default).
  if (RUN_MOTOR_SELF_TEST_ON_BOOT) {
    runMotorSelfTest();
  }

  connectWiFi();  // Block until Wi-Fi is connected.

  // Execute the first navigation step immediately so the robot begins moving
  // without waiting for the first REQUEST_INTERVAL in loop().
  Serial.println("Consulting the AI oracle...");
  String action = getNextAction();
  applyAction(action);
  updateDestinationIfReached();
  showGridState(action);
  lastRequest = millis();  // Seed the timer so loop() waits a full interval.
}
// =================================================
// loop()
// Arduino main loop – called repeatedly after setup() completes.
// Only runs the navigation pipeline when REQUEST_INTERVAL ms have elapsed
// since the last step, effectively rate-limiting how fast the robot moves.
//
// Each iteration of the navigation pipeline:
//   1. Query the LLM (or fallback) for the next action.
//   2. Execute the physical motor move via applyAction().
//   3. Check whether a waypoint was reached and flip the target if so.
//   4. Print the current grid state to Serial.
//   5. Reset the timer for the next interval.
// =================================================
void loop() {
  // Rate-limit: only act when enough time has passed since the last move.
  if (millis() - lastRequest > REQUEST_INTERVAL) {
    Serial.println("Consulting the AI oracle...");
    String action = getNextAction();    // Ask LLM (or fallback) what to do.
    applyAction(action);                // Drive the motors for one grid step.
    updateDestinationIfReached();       // Flip target if waypoint was reached.
    showGridState(action);              // Print position summary to Serial.
    lastRequest = millis();             // Reset the interval timer.
  }
  // The robot is idle between steps; no other background work is needed.
}
