// Tic-tac-toe robot arm with an on-device web site.
//   http://<device-ip>/           game page (play, live output, controls)
//   http://<device-ip>/calibrate  calibration page (jog, test, map and store poses; stand-alone D11 test servo)
// The Serial Monitor commands from tic_tac_toe.cpp still work (send ? for help).
#include <Arduino.h>
#include <Servo.h>
#include <WiFiS3.h>
#include <EEPROM.h>
#include <FspTimer.h>
#include <stdlib.h>
#include <string.h>
#include "board_camera.h"
#include "camera_board_diff.h"
#include "gemini_connection.h"
#include "wifi_connection.h"
#include "tic_tac_toe_rules.h"

// ---------------------------------------------------------------- arm setup
constexpr uint8_t motorCount = 6;
const uint8_t motorPins[motorCount] = {0, 1, 2, 3, 4, 5};
// Stand-alone test servo, independent of the arm and its poses. Web API motor index 6.
constexpr uint8_t auxMotor = motorCount;
constexpr uint8_t auxPin = 11;
// Average travel time per degree. Moves are eased, so they peak at 1.5x this average speed.
constexpr unsigned long stepMs = 40;
constexpr unsigned long jogStepMs = 20;
constexpr unsigned long testStepMs = 80;
constexpr unsigned long gripStepMs = 30;
constexpr unsigned long minMoveMs = 200;
// Servo positions are recomputed in a 200 Hz timer interrupt, independent of Wi-Fi delays.
constexpr float motionTickHz = 200.0f;
constexpr unsigned long testHeartbeatMs = 4000;
constexpr unsigned long dropPauseMs = 1000;
constexpr unsigned long grabPauseMs = 500;
constexpr unsigned long arrivePauseMs = 500;
constexpr unsigned long commandIdleMs = 500;
constexpr uint8_t liftMotor = 3;
constexpr int clearanceAngle = 90;
constexpr uint8_t wristMotor = 1;
// Transfer phases overlap: travel starts when lift is this far through, lowering when travel is.
constexpr float liftOverlap = 0.6f;
constexpr float lowerOverlap = 0.7f;

// Motor 0 is the gripper. Home pose M0 = open angle; closed = open -/+ gripAmount.
constexpr uint8_t gripMotor = 0;
int gripAmount = 15;
int gripOpenSign = 1;  // +1: increasing M0 opens the gripper; -1: decreasing opens.
bool gripClosed = false;

constexpr uint8_t cellCount = 9;
constexpr uint8_t pickupTarget = 9;
constexpr uint8_t homeTarget = 10;
constexpr uint8_t targetCount = 11;
// Rows 0-8: cells 1-9. Row 9: outside-board pickup point. Row 10: home/rest pose.
const int defaultPoses[targetCount][motorCount] = {
  {66, 73, 13, 170, 77, 65},
  {66, 73, 13, 170, 77, 82},
  {66, 73, 13, 170, 77, 97},
  {66, 73, 22, 163, 69, 70},
  {66, 73, 22, 163, 69, 84},
  {66, 73, 22, 163, 69, 98},
  {66, 73, 22, 143, 52, 75},
  {66, 73, 22, 143, 52, 86},
  {66, 73, 22, 143, 52, 98},
  {66, 73, 13, 170, 77, 128},
  {105, 70, 81, 128, 88, 80}
};
int poses[targetCount][motorCount];

// Wrist compensation angles applied at clearance height to keep pieces straight during placement.
// Each cell (0-8) requires a different wrist angle adjustment to compensate for kinematic drift.
// These values offset the base wrist angle while the arm is high above the board.
// Calibrated estimate based on pickup at 150mm right of center, top row.
// Pieces drift by 5-15° depending on destination; compensation needs to match this magnitude.
// Square 1-3 (top row): pickup reference area, minimal to moderate drift
// Square 4-6 (middle row): increased distance from pickup, stronger drift
// Square 7-9 (bottom row): maximum distance from pickup, strongest drift (±20-30°)
const int wristCompensation[cellCount] = {
  15,   // Cell 0 (top-left): square 1 - clockwise correction (arm reaches left)
  8,    // Cell 1 (top-center): square 2 - slight clockwise correction
  -5,   // Cell 2 (top-right): square 3 - slight counterclockwise correction (right side, near pickup)
  22,   // Cell 3 (middle-left): square 4 - strong clockwise correction with distance
  0,    // Cell 4 (center): square 5 - balanced position
  -12,  // Cell 5 (middle-right): square 6 - counterclockwise correction
  30,   // Cell 6 (bottom-left): square 7 - maximum clockwise correction (long left + distance)
  12,   // Cell 7 (bottom-center): square 8 - clockwise correction (farther left than position 2)
  -25   // Cell 8 (bottom-right): square 9 - maximum counterclockwise correction (long right + distance)
};
bool posesFromFlash = false;
bool posesUnstored = false;

int minAngle(uint8_t motor) { return motor == 0 ? 10 : 0; }
int maxAngle(uint8_t motor) { return motor == 0 ? 175 : 180; }

// ---------------------------------------------------------------- state
enum class Turn { ChooseOrder, Human, Robot, Finished, Stopped };
enum class Motion { Idle, PrepHome, OpenGrip, ToPickup, Grab, GrabSettle, ToCell, Arrive, Release, Drop, ToHome, Jig, Preview, GripTest, Park };

// The web page, serial help, and Gemini prompt all use these symbols.
// The camera reports the same letters, plus '.' for an empty cell.
constexpr char humanPiece = 'O';
constexpr char robotPiece = 'X';

// Camera capture is driven from loop(), never from the servo timer.
// WaitMotion: travelling to the home/park pose. Settle: ~500 ms after motion
// finishes. Read: HTTP GET /board (the arm is already still).
enum class CameraJob { Idle, WaitMotion, Settle, Read };
CameraJob cameraJob = CameraJob::Idle;
unsigned long cameraSettleAt = 0;
String cameraNotice;
constexpr unsigned long cameraSettleMs = 500;
constexpr unsigned long cameraAttemptMs = 3000;
constexpr uint8_t cameraAttempts = 3;

Servo motors[motorCount];
tic_tac_toe::Board board;
Turn turn = Turn::ChooseOrder;
Motion motion = Motion::Idle;
bool armed = false;
bool rulesPassed = false;
bool robotRequested = false;
int angles[motorCount];
int targetSquare = 0;
uint8_t jigStage = 0;
unsigned long pauseAt = 0;

Servo auxServo;
bool auxAttached = false;
int auxAngle = 90;

constexpr uint8_t jointCount = motorCount + 1;  // index auxMotor = D11 servo
Servo &jointServo(int joint) { return joint == auxMotor ? auxServo : motors[joint]; }

// ---------------------------------------------------------------- motion plan
// A plan gives each joint up to 3 eased segments on a shared clock. The timer interrupt
// evaluates it and writes microsecond pulses; loop() only builds plans and reacts when done.
struct Segment {
  uint32_t t0, t1;
  float from, to;
};
constexpr uint8_t maxSegments = 3;
struct Plan {
  Segment seg[jointCount][maxSegments];
  uint8_t count[jointCount];
  uint32_t length;
};
Plan plan;
volatile uint32_t planStart = 0;
volatile bool planActive = false;
volatile float positions[jointCount];
FspTimer motionTimer;
bool motionTimerRunning = false;

float smoothstep(float t) { return t * t * (3.0f - 2.0f * t); }

void writeJoint(uint8_t joint, float degrees) {
  degrees = constrain(degrees, 0.0f, 180.0f);
  positions[joint] = degrees;
  jointServo(joint).writeMicroseconds(
      MIN_PULSE_WIDTH + int((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * degrees / 180.0f + 0.5f));
}

float planValue(uint8_t joint, uint32_t t) {
  const Segment *s = plan.seg[joint];
  float value = s[0].from;
  for (uint8_t i = 0; i < plan.count[joint]; i++) {
    if (t >= s[i].t1) { value = s[i].to; continue; }
    if (t > s[i].t0) value = s[i].from + (s[i].to - s[i].from) * smoothstep(float(t - s[i].t0) / float(s[i].t1 - s[i].t0));
    break;
  }
  return value;
}

void applyPlan(uint32_t t) {
  for (uint8_t joint = 0; joint < jointCount; joint++) {
    if (plan.count[joint] > 0) writeJoint(joint, planValue(joint, t));
  }
}

void motionTick(timer_callback_args_t *) {
  if (planActive) applyPlan(millis() - planStart);
}

void say(const String &line);

void startMotionTimer() {
  uint8_t type = 0;
  int8_t channel = FspTimer::get_available_timer(type);
  motionTimerRunning = channel >= 0 &&
      motionTimer.begin(TIMER_MODE_PERIODIC, type, channel, motionTickHz, 0.0f, motionTick, nullptr) &&
      motionTimer.setup_overflow_irq(12) && motionTimer.open() && motionTimer.start();
  say(motionTimerRunning ? "[Motion] 200 Hz motion timer running." : "[Motion] Motion timer unavailable; using loop updates.");
}

// Freezes every joint at its current commanded position.
void haltPlan() {
  noInterrupts();
  planActive = false;
  interrupts();
}

void clearPlan(Plan &next) {
  memset(&next, 0, sizeof(next));
}

uint32_t moveDuration(float distance, unsigned long msPerDegree) {
  return distance < 0.25f ? 0 : max(minMoveMs, (unsigned long)(distance * msPerDegree));
}

void addSegment(Plan &next, uint8_t joint, uint32_t t0, uint32_t t1, float from, float to) {
  if (next.count[joint] >= maxSegments || t1 <= t0) return;
  next.seg[joint][next.count[joint]++] = {t0, t1, from, to};
  next.length = max(next.length, t1);
}

void commitPlan(const Plan &next) {
  noInterrupts();
  plan = next;
  planStart = millis();
  planActive = true;
  interrupts();
}

// True once the plan has run its full length; snaps every joint to its exact end value.
bool planFinished() {
  if (!planActive) return true;
  if (millis() - planStart < plan.length) {
    if (!motionTimerRunning) applyPlan(millis() - planStart);
    return false;
  }
  noInterrupts();
  applyPlan(plan.length);
  planActive = false;
  interrupts();
  return true;
}

void syncAngles() {
  for (uint8_t motor = 0; motor < motorCount; motor++) angles[motor] = int(positions[motor] + 0.5f);
  auxAngle = int(positions[auxMotor] + 0.5f);
}

int gripOpenAngle();
int gripClosedAngle();

// Calibration single-joint jog/test.
int calMotor = -1;
int calTarget = 90;
bool calTesting = false;
unsigned long calHeartbeatAt = 0;
unsigned long calProgressAt = 0;
const char *calStatus = "Idle";
uint8_t selectedTarget = 0;

// Serial input.
String command;
bool commandOverflow = false;
unsigned long lastCommandByteAt = 0;
unsigned long lastStatusAt = 0;

// Network.
WiFiServer server(80);
bool networkConnected = false;
unsigned long lastNetworkAttempt = 0;
unsigned long lastNetworkCheckAt = 0;

bool busy() {
  return motion != Motion::Idle || calMotor >= 0 || robotRequested || cameraJob != CameraJob::Idle;
}
bool networkRestartRequested = false;

// ---------------------------------------------------------------- output log
// Every message goes to Serial and to a ring buffer that the web pages poll.
constexpr uint8_t logCapacity = 24;
String logLines[logCapacity];
uint32_t logCount = 0;

void say(const String &line) {
  Serial.println(line);
  logLines[logCount % logCapacity] = line.length() > 200 ? line.substring(0, 200) : line;
  logCount++;
}

// ---------------------------------------------------------------- pose storage
struct StoredPoses {
  uint32_t magic;
  int16_t angles[targetCount][motorCount];
  uint16_t checksum;
};
constexpr uint32_t poseMagic = 0x54545431;

uint16_t poseChecksum(const StoredPoses &stored) {
  uint16_t sum = 0x5A5A;
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(stored.angles);
  for (size_t i = 0; i < sizeof(stored.angles); i++) sum = uint16_t((sum << 1) | (sum >> 15)) ^ bytes[i];
  return sum;
}

void loadDefaultPoses() {
  memcpy(poses, defaultPoses, sizeof(poses));
}

bool loadStoredPoses() {
  StoredPoses stored;
  EEPROM.get(0, stored);
  if (stored.magic != poseMagic || stored.checksum != poseChecksum(stored)) return false;
  for (uint8_t t = 0; t < targetCount; t++)
    for (uint8_t m = 0; m < motorCount; m++)
      if (stored.angles[t][m] < 0 || stored.angles[t][m] > 180) return false;
  for (uint8_t t = 0; t < targetCount; t++)
    for (uint8_t m = 0; m < motorCount; m++) poses[t][m] = stored.angles[t][m];
  return true;
}

void storePoses() {
  StoredPoses stored;
  stored.magic = poseMagic;
  for (uint8_t t = 0; t < targetCount; t++)
    for (uint8_t m = 0; m < motorCount; m++) stored.angles[t][m] = int16_t(poses[t][m]);
  stored.checksum = poseChecksum(stored);
  EEPROM.put(0, stored);
  posesFromFlash = true;
  posesUnstored = false;
}

// Gripper settings live after the pose block so stored poses stay valid.
struct StoredGrip {
  uint32_t magic;
  int16_t amount;
  int16_t openSign;
};
constexpr uint32_t gripMagic = 0x47524950;
constexpr int gripAddress = 256;

void loadGripSettings() {
  StoredGrip stored;
  EEPROM.get(gripAddress, stored);
  if (stored.magic != gripMagic || stored.amount < 0 || stored.amount > 90 || (stored.openSign != 1 && stored.openSign != -1)) return;
  gripAmount = stored.amount;
  gripOpenSign = stored.openSign;
}

void storeGripSettings() {
  StoredGrip stored = {gripMagic, int16_t(gripAmount), int16_t(gripOpenSign)};
  EEPROM.put(gripAddress, stored);
}

int gripOpenAngle() { return constrain(poses[homeTarget][gripMotor], minAngle(gripMotor), maxAngle(gripMotor)); }
int gripClosedAngle() { return constrain(gripOpenAngle() - gripOpenSign * gripAmount, minAngle(gripMotor), maxAngle(gripMotor)); }

String targetName(uint8_t target) {
  if (target == pickupTarget) return "pickup";
  if (target == homeTarget) return "home";
  return "cell " + String(target + 1);
}

String poseText(const int pose[motorCount]) {
  String text;
  for (uint8_t m = 0; m < motorCount; m++) {
    if (m > 0) text += ", ";
    text += String(pose[m]);
  }
  return text;
}

// ---------------------------------------------------------------- game text
const char *turnName() {
  switch (turn) {
    case Turn::ChooseOrder: return "waiting for FIRST/SECOND";
    case Turn::Human: return "human O: I've moved, or square 1-9";
    case Turn::Robot: return "robot X turn (RETRY if last request failed)";
    case Turn::Finished: return "game finished; NEW to restart";
    case Turn::Stopped: return "stopped; NEW to restart";
  }
  return "unknown";
}

const char *turnCode() {
  switch (turn) {
    case Turn::ChooseOrder: return "choose";
    case Turn::Human: return "human";
    case Turn::Robot: return "robot";
    case Turn::Finished: return "finished";
    case Turn::Stopped: return "stopped";
  }
  return "unknown";
}

const char *motionName(Motion phase) {
  switch (phase) {
    case Motion::Idle: return "idle";
    case Motion::PrepHome: return "returning home before the move";
    case Motion::OpenGrip: return "opening gripper at home";
    case Motion::ToPickup: return "moving to pickup with gripper open";
    case Motion::Grab: return "closing gripper on the piece at pickup";
    case Motion::GrabSettle: return "settling grip";
    case Motion::ToCell: return "carrying piece to X square";
    case Motion::Arrive: return "pausing over X square";
    case Motion::Release: return "opening gripper to drop piece";
    case Motion::Drop: return "pausing after drop";
    case Motion::ToHome: return "lifting clear of the piece and returning home";
    case Motion::Jig: return "victory jig";
    case Motion::Preview: return "moving to calibration target";
    case Motion::GripTest: return "moving gripper";
    case Motion::Park: return "moving to park/home so the camera can see the board";
  }
  return "unknown";
}

void printBoard() {
  for (int row = 0; row < 3; row++) {
    String line = "  ";
    for (int column = 0; column < 3; column++) {
      int square = row * 3 + column;
      line += board.cells[square] ? board.cells[square] : char('1' + square);
      if (column < 2) line += " | ";
    }
    say(line);
  }
}

void printStatus(bool toLog) {
  String line = "[Status t=" + String(millis() / 1000) + "s] servos=" + (armed ? "ARMED" : "DISARMED") +
      "; turn=" + turnName() + "; motion=" + motionName(motion) + "; Wi-Fi status=" + String(WiFi.status());
  if (networkConnected) line += "; web http://" + WiFi.localIP().toString() + "/";
  if (toLog) say(line); else Serial.println(line);
  if (armed) {
    line = "[Angles M0-M5] " + poseText(angles) + " (commanded; no position feedback)";
    if (toLog) say(line); else Serial.println(line);
  }
}

void printHelp() {
  say("You are O. Gemini and the arm are X. Squares: 1 2 3 / 4 5 6 / 7 8 9.");
  say("ARM: enable servos at home. Initial alignment can be sudden; clear and support the arm.");
  say("FIRST: arm starts. SECOND: human starts. Enter one square 1-9 for each human O.");
  say("MOVED: on your turn, park the arm and read your O from the board camera (web: I've moved).");
  say("RETRY: retry Gemini. NEW: clear board. BOARD: show board. TEST: check rules. STATUS: show activity.");
  say("WEB: reset the Wi-Fi module and restart the web server if the pages stop responding.");
  say("STOP or !: cancel motion/game, retain holding torque. RELEASE: remove pulses; support arm first.");
  say("Motor 0 is the gripper. Each X: open at home, go to pickup, close on the piece, lift clear,");
  say("carry it directly above the X square, orient M1 while raised, lower, drop, and return home open.");
  say("Keep an X piece at the pickup point before each robot move (home M0 = gripper open angle).");
  say("Web site: / for the game, /calibrate to map and store arm poses and test the stand-alone D11 servo.");
}

// ---------------------------------------------------------------- motion
int pendingSquare = 0;

void beginMotion(const Plan &next, Motion phase) {
  motion = phase;
  commitPlan(next);
  say(String("[Motion] ") + motionName(phase) + "; ms=" + String(next.length));
}

// All motors move together and arrive at the same time.
void startPose(const int pose[motorCount], Motion phase, unsigned long msPerDegree) {
  Plan next;
  clearPlan(next);
  float distance = 0;
  for (uint8_t m = 0; m < motorCount; m++) distance = max(distance, fabsf(pose[m] - positions[m]));
  uint32_t duration = moveDuration(distance, msPerDegree);
  for (uint8_t m = 0; m < motorCount; m++) {
    int target = constrain(pose[m], minAngle(m), maxAngle(m));
    if (fabsf(target - positions[m]) >= 0.25f) addSegment(next, m, 0, duration, positions[m], target);
  }
  beginMotion(next, phase);
}

// One continuous plan: M3 moves toward clearance (clamped to its start/target range), the other joints start travelling before the
// lift ends, and M3 lowers into the pose before travel ends. holdGrip keeps M0 where it is.
// liftFirst finishes the M3 lift before anything else moves, so the gripper clears a placed piece.
// ToCell lifts to clearance, travels above the square, applies wrist compensation at clearance, resets M1 to square orientation,
// then lowers. Wrist compensation at clearance height corrects for kinematic drift to keep pieces straight.
void startTransfer(const int pose[motorCount], Motion phase, bool holdGrip, bool liftFirst = false, int destSquare = 0) {
  bool orientAboveBoard = phase == Motion::ToCell;
  int target[motorCount];
  for (uint8_t m = 0; m < motorCount; m++) target[m] = constrain(pose[m], minAngle(m), maxAngle(m));
  if (holdGrip) target[gripMotor] = int(positions[gripMotor] + 0.5f);
  
  // For ToCell, apply wrist compensation at clearance height based on the destination square.
  int wristAtClearance = constrain(poses[homeTarget][wristMotor], minAngle(wristMotor), maxAngle(wristMotor));
  if (orientAboveBoard && destSquare > 0 && destSquare <= cellCount) {
    int cellIndex = destSquare - 1;
    wristAtClearance = constrain(wristAtClearance + wristCompensation[cellIndex], minAngle(wristMotor), maxAngle(wristMotor));
    say(String("[Wrist] ToCell square ") + String(destSquare) + " compensation=" + String(wristCompensation[cellIndex]) + 
        " current=" + String(int(positions[wristMotor] + 0.5f)) + " clearance=" + String(wristAtClearance));
    // For ToCell with compensation: set target wrist to the compensated angle, not the original board angle.
    // This gives one smooth movement to the compensated position, maintaining it while lowering.
    target[wristMotor] = wristAtClearance;
  }

  float travelDistance = 0;
  for (uint8_t m = 0; m < motorCount; m++) {
    if (m != liftMotor && !(orientAboveBoard && m == wristMotor)) travelDistance = max(travelDistance, fabsf(target[m] - positions[m]));
  }
  if (!orientAboveBoard && travelDistance < 0.25f) {
    startPose(target, phase, stepMs);
    return;
  }

  Plan next;
  clearPlan(next);
  float liftFrom = positions[liftMotor];
  // Other transfers keep M3 within the start/target range; ToCell needs clearance for wrist rotation.
  float lowEnd = min(liftFrom, float(target[liftMotor]));
  float highEnd = max(liftFrom, float(target[liftMotor]));
  float clearance = orientAboveBoard ? min(float(clearanceAngle), lowEnd) : constrain(float(clearanceAngle), lowEnd, highEnd);
  uint32_t liftMs = moveDuration(fabsf(clearance - liftFrom), stepMs);
  
  // For ToCell: first apply compensation wrist angle at clearance, then move to final wrist angle during descent.
  uint32_t compensationMs = orientAboveBoard ? moveDuration(fabsf(wristAtClearance - positions[wristMotor]), stepMs) : 0;
  uint32_t wristMs = 0;  // No second movement - we stay at the compensated angle
  
  uint32_t travelStart = (orientAboveBoard || liftFirst) ? liftMs : uint32_t(liftMs * liftOverlap);
  uint32_t travelMs = moveDuration(travelDistance, stepMs);
  uint32_t compensationStart = travelStart + travelMs;
  uint32_t wristStart = compensationStart + compensationMs;
  // For ToCell: lower ONLY after all XY positioning and wrist adjustment are complete
  // This ensures straight vertical descent from directly above the square
  uint32_t lowerStart = orientAboveBoard ? (wristStart + wristMs) : max(liftMs, travelStart + uint32_t(travelMs * lowerOverlap));
  uint32_t lowerMs = moveDuration(fabsf(target[liftMotor] - clearance), stepMs);

  // For ToCell: ensure M3 (lift) stays at clearance during ALL wrist adjustments to avoid collision
  // M3 will only start lowering AFTER wrist compensation and ALL XY motors complete
  addSegment(next, liftMotor, 0, liftMs, liftFrom, clearance);
  // Add an extra hold segment to keep M3 at clearance while XY and wrist adjustments happen
  if (orientAboveBoard) {
    addSegment(next, liftMotor, liftMs, lowerStart, clearance, clearance);
  }
  addSegment(next, liftMotor, lowerStart, lowerStart + lowerMs, liftMs ? clearance : liftFrom, target[liftMotor]);
  if (orientAboveBoard) {
    // Single smooth wrist movement to compensation angle at clearance, then lower while maintaining that angle.
    say(String("[Wrist] compMs=") + String(compensationMs) + " (single movement to compensated angle)");
    addSegment(next, wristMotor, compensationStart, compensationStart + compensationMs, positions[wristMotor], target[wristMotor]);
  }
  for (uint8_t m = 0; m < motorCount; m++) {
    if (m == liftMotor || (orientAboveBoard && m == wristMotor) || fabsf(target[m] - positions[m]) < 0.25f) continue;
    if (orientAboveBoard) {
      // For ToCell: complete all XY positioning (all non-lift motors) BEFORE lowering
      // This ensures the piece descends straight down from directly above the square, not at an angle
      addSegment(next, m, travelStart, travelStart + travelMs, positions[m], target[m]);
    } else {
      addSegment(next, m, travelStart, travelStart + travelMs, positions[m], target[m]);
    }
  }
  if (orientAboveBoard) {
    say(String("[Motion] ToCell travel ms=") + String(travelMs) + " wrist at clearance until ms=" + String(lowerStart) + 
        " then lower ms=" + String(lowerMs));
  }
  beginMotion(next, phase);
  say("[Target M0-M5] " + poseText(target));
}

void startGrip(bool close, Motion phase) {
  int target = close ? gripClosedAngle() : gripOpenAngle();
  if (!close) gripClosed = false;
  Plan next;
  clearPlan(next);
  addSegment(next, gripMotor, 0, moveDuration(fabsf(target - positions[gripMotor]), gripStepMs), positions[gripMotor], target);
  beginMotion(next, phase);
}

bool atHome() {
  for (uint8_t m = 0; m < motorCount; m++) {
    if (m != gripMotor && fabsf(positions[m] - poses[homeTarget][m]) > 1.0f) return false;
  }
  return true;
}

// Robot move: open at home -> pickup -> close -> lift -> travel above square -> orient -> lower -> open -> home.
void startRobotTransfer(int square) {
  pendingSquare = square;
  if (!atHome()) {
    startTransfer(poses[homeTarget], Motion::PrepHome, true, true);
  } else if (fabsf(positions[gripMotor] - gripOpenAngle()) >= 1.0f) {
    startGrip(false, Motion::OpenGrip);
  } else {
    startTransfer(poses[pickupTarget], Motion::ToPickup, true);
  }
}

void stopCalMotion(const char *reason) {
  bool wasMoving = calMotor >= 0;
  if (wasMoving) {
    haltPlan();
    syncAngles();
  }
  calMotor = -1;
  calTesting = false;
  calStatus = reason;
  if (wasMoving) say(String("[Calibrate] ") + reason);
}

void stopGame(bool release) {
  stopCalMotion("Stopped. Servos retain holding torque.");
  haltPlan();
  syncAngles();
  motion = Motion::Idle;
  pendingSquare = 0;
  gripClosed = fabsf(positions[gripMotor] - gripClosedAngle()) < 1.0f;
  robotRequested = false;
  cameraJob = CameraJob::Idle;
  cameraNotice = "";
  turn = Turn::Stopped;
  if (release) {
    for (auto &motor : motors) motor.detach();
    armed = false;
    gripClosed = false;
    auxServo.detach();
    auxAttached = false;
  }
  say(release ? "Servo pulses released. Support the arm." : "Stopped. Servos still hold position. NEW starts a new game.");
}

const char *armServos() {
  if (armed) return nullptr;
  say("[Arm] Enabling all servos at the home pose (gripper open). Initial alignment is not speed-controlled.");
  haltPlan();
  for (uint8_t motor = 0; motor < motorCount; motor++) {
    int angle = constrain(poses[homeTarget][motor], minAngle(motor), maxAngle(motor));
    if (!motors[motor].attach(motorPins[motor])) {
      say("[Arm] Failed to attach motor " + String(motor));
      for (auto &servo : motors) servo.detach();
      return "Servo attachment failed; support the arm and check output";
    }
    writeJoint(motor, angle);
    say("[Arm] Motor " + String(motor) + " -> " + String(angle));
    delay(150);
  }
  syncAngles();
  gripClosed = false;
  armed = true;
  return nullptr;
}

void jigPose(int offset) {
  int pose[motorCount];
  for (uint8_t m = 0; m < motorCount; m++) pose[m] = poses[homeTarget][m];
  pose[gripMotor] = int(positions[gripMotor] + 0.5f);
  pose[5] += offset;
  startPose(pose, Motion::Jig, stepMs);
}

void finishRobotMove() {
  char winner = board.winner();
  if (winner == 'X') {
    say("X wins! Small victory jig at home.");
    turn = Turn::Finished;
    jigStage = 0;
    jigPose(-5);
  } else if (board.full()) {
    say("Draw. Send NEW to play again.");
    turn = Turn::Finished;
  } else {
    turn = Turn::Human;
    say("Your turn: place O, then press I've moved or enter its square 1-9.");
  }
}

// Park has finished (or was already at home). The 500 ms settle is timed in loop().
void beginCameraSettle() {
  if (cameraJob != CameraJob::WaitMotion) return;
  cameraJob = CameraJob::Settle;
  cameraSettleAt = millis();
  cameraNotice = "Arm is home. Settling before the camera reads the board.";
  say("[Camera] " + cameraNotice);
}

void updateGameMotion() {
  if (motion == Motion::Idle) return;
  if (motion == Motion::GrabSettle) {
    if (millis() - pauseAt >= grabPauseMs) {
      int square = pendingSquare;
      pendingSquare = 0;
      say("[Motion] Carrying piece directly to square " + String(square) + "; orienting M1 at clearance before lowering.");
      startTransfer(poses[square - 1], Motion::ToCell, true, true, square);
    }
    return;
  }
  if (motion == Motion::Arrive) {
    if (millis() - pauseAt >= arrivePauseMs) {
      say("[Motion] Opening gripper to drop the piece.");
      startGrip(false, Motion::Release);
    }
    return;
  }
  if (motion == Motion::Drop) {
    if (millis() - pauseAt >= dropPauseMs) startTransfer(poses[homeTarget], Motion::ToHome, true, true);
    return;
  }
  if (!planFinished()) return;
  syncAngles();
  switch (motion) {
    case Motion::PrepHome:
    case Motion::OpenGrip:
      startRobotTransfer(pendingSquare);
      break;
    case Motion::ToPickup:
      say("[Motion] At pickup. Closing gripper on the piece.");
      startGrip(true, Motion::Grab);
      break;
    case Motion::Grab:
      gripClosed = true;
      motion = Motion::GrabSettle;
      pauseAt = millis();
      break;
    case Motion::ToCell:
      say("[Motion] Over the X square. Pausing before release.");
      motion = Motion::Arrive;
      pauseAt = millis();
      break;
    case Motion::Release:
      motion = Motion::Drop;
      pauseAt = millis();
      break;
    case Motion::ToHome:
      motion = Motion::Idle;
      say("[Motion] Home with gripper open. Checking win/draw...");
      finishRobotMove();
      break;
    case Motion::Preview:
      motion = Motion::Idle;
      say("[Calibrate] Arrived at " + targetName(selectedTarget) + ".");
      break;
    case Motion::GripTest:
      motion = Motion::Idle;
      gripClosed = fabsf(positions[gripMotor] - gripClosedAngle()) < 1.0f;
      say(String("[Calibrate] Gripper ") + (gripClosed ? "closed" : "moved") + " at " + String(angles[gripMotor]) + " degrees.");
      break;
    case Motion::Jig:
      jigStage++;
      if (jigStage >= 6) {
        motion = Motion::Idle;
        say("Jig complete. Send NEW to play again.");
      } else {
        jigPose(jigStage == 5 ? 0 : (jigStage % 2 ? 5 : -5));
      }
      break;
    case Motion::Park:
      motion = Motion::Idle;
      beginCameraSettle();
      break;
    default:
      motion = Motion::Idle;
      break;
  }
}

void updateCalMotion() {
  if (calMotor < 0) return;
  if (calTesting && (!networkConnected || millis() - calHeartbeatAt >= testHeartbeatMs)) {
    stopCalMotion("Test stopped: browser heartbeat or Wi-Fi lost. Holding position.");
    return;
  }
  if (!planFinished()) {
    if (millis() - calProgressAt >= 1000) {
      calProgressAt = millis();
      Serial.println("[Calibrate] Motor " + String(calMotor) + ": commanded " + String(int(positions[calMotor] + 0.5f)) + "; target " + String(calTarget));
    }
    return;
  }
  syncAngles();
  calMotor = -1;
  calTesting = false;
  calStatus = "Command complete (commanded position; no position feedback).";
  say(String("[Calibrate] ") + calStatus);
}

void serviceMotion() {
  updateGameMotion();
  updateCalMotion();
  if (planActive) syncAngles();
}

// ---------------------------------------------------------------- Gemini move
void robotMove() {
  say("[Network] Checking Wi-Fi before Gemini request...");
  if (WiFi.status() != WL_CONNECTED && !wifi_connection::connect()) {
    say("Wi-Fi unavailable. Send RETRY; no move made.");
    return;
  }
  say("[Minimax] Evaluating every legal X move...");
  unsigned long searchStartedAt = millis();
  tic_tac_toe::BestMoves best = tic_tac_toe::bestMovesForX(board);
  say("[Minimax] Score for X: " + String(best.score) + "; search ms: " + String(millis() - searchStartedAt));
  if (best.squares == 0) {
    say("[Minimax] No available moves. No motion.");
    return;
  }
  String optimalSquares;
  for (int square = 1; square <= 9; square++) {
    if (!best.contains(square)) continue;
    if (optimalSquares.length() > 0) optimalSquares += ", ";
    optimalSquares += String(square);
  }
  say("[Minimax] Equally optimal squares: " + optimalSquares);
  String prompt = "Play tic tac toe as X against human O. Either player may start. ";
  prompt += "A complete minimax search has identified these equally best moves: [" + optimalSquares + "]. ";
  prompt += "Choose ONLY from this set. For an empty board prefer a corner from the set, to create fork opportunities. ";
  prompt += "Return only JSON {\"square\":N} with one integer from the optimal set. Board: ";
  for (int square = 0; square < 9; square++) {
    prompt += String(square + 1) + "=";
    prompt += board.cells[square] ? String(board.cells[square]) : String("empty");
    prompt += "; ";
  }
  say("Gemini is choosing X's move...");
  String reply;
  String error;
  unsigned long requestStartedAt = millis();
  if (!gemini_connection::generate(prompt.c_str(), reply, error, true, true)) {
    say("[Gemini] Failed after ms: " + String(millis() - requestStartedAt));
    say(error);
    say("Send RETRY. The board is unchanged.");
    return;
  }
  say("[Gemini] Completed in ms: " + String(millis() - requestStartedAt));
  say("[Gemini] Model reply: " + reply.substring(0, 160));
  JsonDocument choice;
  DeserializationError parsed = deserializeJson(choice, reply);
  if (parsed || !choice["square"].is<int>()) {
    say("Gemini returned an invalid move format. Send RETRY.");
    return;
  }
  int square = choice["square"].as<int>();
  if (!best.contains(square)) {
    say("[Minimax] Rejected Gemini move: not in the optimal set. No motion. Send RETRY.");
    return;
  }
  if (!board.place(square, 'X')) {
    say("Gemini chose an occupied or invalid square. No motion. Send RETRY.");
    return;
  }
  say("X chooses square " + String(square));
  printBoard();
  startRobotTransfer(square);
}

// Deferred so web requests get an immediate reply; loop() runs the blocking Gemini call.
void requestRobotMove() {
  robotRequested = true;
  say("[Game] Robot X to move; Gemini request queued.");
}

// Shared by a tapped square and a camera reading. Caller has already checked
// that it is the human's turn and the servos are idle.
void acceptHumanSquare(int square) {
  cameraNotice = "";
  if (!board.place(square, humanPiece)) {
    say("That square is occupied. Enter an empty square.");
    return;
  }
  say(String("[Game] Human ") + humanPiece + " accepted in square " + String(square));
  printBoard();
  if (board.winner() == humanPiece) {
    turn = Turn::Finished;
    say(String(humanPiece) + " wins! Send NEW to play again.");
  } else if (board.full()) {
    turn = Turn::Finished;
    say("Draw. Send NEW to play again.");
  } else {
    turn = Turn::Robot;
    requestRobotMove();
  }
}

// Queue the park pose and return. loop() waits out the move and the settle,
// then talks to the camera. The HTTP handler must not block on that read:
// the page is still polling, and the servo timer has to keep running.
void beginCameraScan() {
  cameraNotice = "Parking the arm at home so the camera can see the board.";
  say("[Camera] " + cameraNotice);
  cameraJob = CameraJob::WaitMotion;
  // Home pose includes the open gripper. Lift clear of the board before travelling.
  // If the arm is already home this plan has zero length and the settle still runs.
  startTransfer(poses[homeTarget], Motion::Park, false, true);
}

// ---------------------------------------------------------------- commands
void processCommand(String text) {
  text.trim();
  text.toUpperCase();
  if (text.length() == 0) return;
  say("[Command] " + text);
  if (text == "STOP" || text == "!") { stopGame(false); return; }
  if (text == "RELEASE") { stopGame(true); return; }
  if (text == "BOARD") { printBoard(); return; }
  if (text == "STATUS") { printStatus(true); return; }
  if (text == "TEST") {
    say(tic_tac_toe::selfTest() ? "Rules/minimax PASS: wins, draw, invalid moves, openings, forced win/block, fork defense." : "Rules/minimax tests FAILED.");
    return;
  }
  if (text == "?" || text == "HELP") { printHelp(); return; }
  if (text == "WEB") {
    networkRestartRequested = true;
    say("[Network] Web server restart queued (runs when the arm is idle).");
    return;
  }
  if (!rulesPassed) { say("Rules self-test failed. Motion disabled."); return; }
  if (busy()) { say("Arm is busy; only STOP, RELEASE, BOARD or STATUS accepted."); return; }
  if (text == "NEW") {
    board = tic_tac_toe::Board{};
    turn = Turn::ChooseOrder;
    cameraNotice = "";
    printBoard();
    say("Send FIRST for arm first, or SECOND for arm second.");
  } else if (text == "ARM") {
    const char *error = armServos();
    say(error ? String(error) : String("Armed. FIRST = arm starts, SECOND = human starts."));
  } else if (text == "FIRST" || text == "SECOND") {
    if (!armed) { say("Send ARM first, with the mechanism clear."); return; }
    if (turn != Turn::ChooseOrder) { say("Game already started. Send NEW to reset."); return; }
    turn = text == "FIRST" ? Turn::Robot : Turn::Human;
    if (turn == Turn::Robot) requestRobotMove();
    else say("[Game] O starts. Place O, then press I've moved or enter its square 1-9.");
  } else if (text == "RETRY") {
    if (armed && turn == Turn::Robot) requestRobotMove();
    else say("No Gemini move is pending.");
  } else if (text == "MOVED") {
    if (!armed || turn != Turn::Human) { say("Not the human's turn."); return; }
    beginCameraScan();
  } else if (text.length() == 1 && text[0] >= '1' && text[0] <= '9') {
    if (!armed || turn != Turn::Human) { say("Not the human's turn."); return; }
    acceptHumanSquare(text[0] - '0');
  } else say("Unknown command. Send ? for help.");
}

void submitCommand() {
  String submitted = command;
  bool overflow = commandOverflow;
  command = "";
  commandOverflow = false;
  if (overflow) say("Command too long; discarded.");
  else processCommand(submitted);
}

void readSerial() {
  while (Serial.available()) {
    char character = Serial.read();
    lastCommandByteAt = millis();
    if (character == '!') {
      stopGame(false);
      command = "";
      commandOverflow = false;
    } else if (character == '\r' || character == '\n') {
      submitCommand();
    } else if (command.length() < 32 && !commandOverflow) command += character;
    else commandOverflow = true;
  }
  if ((command.length() > 0 || commandOverflow) && millis() - lastCommandByteAt >= commandIdleMs) {
    Serial.println("[Serial] No line ending received; submitting buffered command.");
    submitCommand();
  }
}

// ---------------------------------------------------------------- web pages
const char gamePage[] = R"GAMEUI(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Tic-Tac-Toe Arm</title>
<style>
:root{--ink:#183832;--muted:#63736d;--paper:#f3f5ef;--panel:#fff;--line:#d6dfd8;--green:#18765b;--lime:#d4e85c;--red:#a52c2c}
*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);font:16px/1.45 "Trebuchet MS",Arial,sans-serif}
header{background:var(--ink);color:#fff;padding:16px max(20px,calc((100vw - 1040px)/2));display:flex;align-items:center;justify-content:space-between;gap:16px;flex-wrap:wrap}
h1{font-size:22px;margin:0}nav a{color:#fff;margin-left:16px;text-decoration:none;border-bottom:2px solid transparent}nav a.on{border-color:var(--lime)}
.network{font-size:13px;color:#d8e8d9}main{max-width:1040px;margin:0 auto;padding:22px 20px 48px}
.layout{display:grid;grid-template-columns:minmax(260px,360px) 1fr;gap:28px}h2{font-size:17px;margin:0 0 12px}
.board{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.cell{aspect-ratio:1;border:1px solid var(--line);background:var(--panel);color:var(--ink);font-size:44px;font-weight:700;border-radius:6px;cursor:pointer}
.cell .n{font-size:14px;color:#9aa6a0;font-weight:400}.cell.X{color:var(--green)}.cell.O{color:#b5651d}.cell:disabled{cursor:default}.cell.play:hover{background:#e9f3df}
.status{margin:14px 0;padding:10px 14px;background:#e9eee4;border-left:4px solid var(--lime);font-size:15px}
.buttons{display:flex;flex-wrap:wrap;gap:10px;margin-bottom:14px}button.action{border:0;border-radius:4px;background:var(--green);color:#fff;padding:10px 14px;cursor:pointer;font:inherit;font-weight:700}
#moved{display:block;width:100%;margin:0 0 12px}
button.action:disabled{opacity:.4;cursor:not-allowed}button.stop{background:var(--red)}button.light{background:#fff;color:var(--ink);border:1px solid var(--line)}
.meta{color:var(--muted);font-size:14px}pre{background:#10231f;color:#d8f0dc;padding:12px;border-radius:6px;height:420px;overflow:auto;font:12px/1.45 Consolas,monospace;white-space:pre-wrap;margin:0}
@media(max-width:760px){.layout{grid-template-columns:1fr}pre{height:300px}}
</style>
</head>
<body>
<header><h1>Tic-Tac-Toe Arm</h1><nav><a class="on" href="/">Game</a><a href="/calibrate">Calibrate</a></nav><div class="network" id="network">Connecting…</div></header>
<main><div class="layout">
<section>
<h2>Board <span class="meta">(you are O, the arm is X)</span></h2>
<div class="board" id="board"></div>
<div class="status" id="status">Loading…</div>
<button class="action" id="moved" disabled>I've moved</button>
<div class="buttons">
<button class="action" id="arm">Arm servos</button>
<button class="action" data-cmd="FIRST">Arm plays first</button>
<button class="action" data-cmd="SECOND">I play first</button>
<button class="action light" data-cmd="RETRY">Retry Gemini</button>
<button class="action light" data-cmd="NEW">New game</button>
</div>
<div class="buttons">
<button class="action stop" id="stop">Stop</button>
<button class="action light" id="release">Release servos</button>
<button class="action light" data-cmd="TEST">Rules self-test</button>
<button class="action light" data-cmd="STATUS">Status</button>
</div>
<p class="meta">Place your O on the physical board, then press <b>I've moved</b>. The arm parks at home, waits half a second, and reads the camera. You can still tap the square yourself if the camera is unsure. Keep an X piece at the pickup point: for each X the arm opens its gripper at home, picks the piece up, lifts clear, carries it directly above its square, orients the wrist while raised, lowers, drops it, lifts clear and returns home. Keep clear of the arm. Use <a href="/calibrate">Calibrate</a> if the arm misses a square.</p>
<p class="meta" id="angles"></p>
</section>
<section><h2>Output</h2><pre id="log"></pre><p class="meta"><button class="action light" id="clear">Clear output</button></p></section>
</div></main>
<script>
const $=id=>document.getElementById(id),logEl=$('log'),boardEl=$('board');
let since=0,s=null,timer=null,sending=false;
for(let i=1;i<=9;i++){const b=document.createElement('button');b.className='cell';b.dataset.sq=i;b.onclick=()=>send(String(i));boardEl.appendChild(b)}
function addLog(lines){if(!lines.length)return;const atEnd=logEl.scrollTop+logEl.clientHeight>=logEl.scrollHeight-20;logEl.textContent+=lines.join('\n')+'\n';if(logEl.textContent.length>40000)logEl.textContent=logEl.textContent.slice(-30000);if(atEnd)logEl.scrollTop=logEl.scrollHeight}
async function get(url){const a=new AbortController(),t=setTimeout(()=>a.abort(),8000);try{const r=await fetch(url,{cache:'no-store',signal:a.signal});const d=await r.json();if(!r.ok)throw new Error(d.error||'Request failed');return d}finally{clearTimeout(t)}}
function apply(d){if(d.logNext<since)logEl.textContent+='--- controller restarted ---\n';addLog(d.log);since=d.logNext;s=d;render()}
function render(){if(!s)return;const live=!sending,idle=live&&!s.busy;
$('network').textContent='Device online'+(s.ip?' · '+s.ip:'');
[...boardEl.children].forEach((b,i)=>{const c=s.board[i];b.className='cell'+(c!=='-'?' '+c:'');b.innerHTML=c!=='-'?c:'<span class="n">'+(i+1)+'</span>';const ok=idle&&s.armed&&s.turnCode==='human'&&c==='-';b.disabled=!ok;if(ok)b.classList.add('play')});
let t=s.notice?s.notice:s.thinking?'Gemini is choosing X\u2019s move…':s.motionCode!=='idle'?'Arm: '+s.motion:s.turn;
if(s.winner==='X')t='X (arm) wins! '+t;else if(s.winner==='O')t='O (you) win! '+t;else if(s.winner==='D')t='Draw. '+t;
if(!s.armed)t='Servos disarmed. Clear the arm, then press Arm servos. ('+s.turn+')';if(!s.rules)t='Rules self-test FAILED – motion disabled.';$('status').textContent=t;
$('moved').disabled=!(idle&&s.armed&&s.rules&&s.turnCode==='human');
$('arm').disabled=!live||s.armed||s.busy;$('arm').textContent=s.armed?'Servos armed':'Arm servos';$('release').disabled=!live||!s.armed;
document.querySelectorAll('[data-cmd]').forEach(b=>{const c=b.dataset.cmd;let ok=live;if(c==='FIRST'||c==='SECOND')ok=idle&&s.armed&&s.turnCode==='choose';else if(c==='RETRY')ok=idle&&s.armed&&s.turnCode==='robot';else if(c==='NEW')ok=idle;b.disabled=!ok});
$('angles').textContent=s.armed?'Commanded angles M0–M5: '+s.angles.join(', '):''}
async function poll(){clearTimeout(timer);try{apply(await get('/api/state?since='+since))}catch(e){$('network').textContent='Controller busy or offline – retrying…'}timer=setTimeout(poll,s&&s.busy?500:1000)}
async function send(c){sending=true;render();try{apply(await get('/api/cmd?c='+encodeURIComponent(c)+'&since='+since))}catch(e){addLog(['[Web] '+c+' failed: '+(e.name==='AbortError'?'timed out':e.message)])}sending=false;render();poll()}
document.querySelectorAll('[data-cmd]').forEach(b=>b.onclick=()=>send(b.dataset.cmd));
$('moved').onclick=()=>send('MOVED');
$('arm').onclick=()=>{if(confirm('Arm all six servos at the home pose? Initial movement can be sudden. Support the arm and clear its travel.'))send('ARM')};
$('release').onclick=()=>{if(confirm('Release all servo pulses? The arm may drop. Support it first.'))send('RELEASE')};
$('stop').onclick=()=>send('STOP');$('clear').onclick=()=>{logEl.textContent=''};
poll();
</script>
</body>
</html>)GAMEUI";

const char calibrationPage[] = R"CALUI(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Arm Calibration</title>
<style>
:root{color-scheme:light;--ink:#183832;--muted:#63736d;--paper:#f3f5ef;--panel:#fff;--line:#d6dfd8;--green:#18765b;--lime:#d4e85c}
*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);font:16px/1.45 "Trebuchet MS",Arial,sans-serif}
header{background:var(--ink);color:#fff;padding:16px max(20px,calc((100vw - 1040px)/2));display:flex;align-items:center;justify-content:space-between;gap:16px;flex-wrap:wrap}
h1{font-size:22px;margin:0;font-weight:700}nav a{color:#fff;margin-left:16px;text-decoration:none;border-bottom:2px solid transparent}nav a.on{border-color:var(--lime)}
.network{font-size:13px;color:#d8e8d9}main{max-width:1040px;margin:0 auto;padding:24px 20px 48px}
.notice{border-left:4px solid var(--lime);padding:10px 14px;margin-bottom:22px;background:#e9eee4;font-size:14px}
h2{font-size:17px;margin:0 0 14px}.layout{display:grid;grid-template-columns:minmax(240px,320px) 1fr;gap:28px}
.board{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.cell{aspect-ratio:1;border:1px solid var(--line);background:var(--panel);color:var(--ink);font-size:22px;font-weight:700;cursor:pointer;border-radius:4px}
.cell[aria-pressed="true"]{background:var(--green);color:#fff;border-color:var(--green)}
.wide{width:100%;aspect-ratio:auto;font-size:16px;padding:12px;margin-top:10px}
.section{border-bottom:1px solid var(--line);padding:0 0 22px;margin-bottom:22px}.section:last-child{border-bottom:0}
button{font:inherit}.action{border:0;border-radius:4px;background:var(--green);color:#fff;padding:11px 16px;cursor:pointer;font-weight:700}.action:disabled{opacity:.45;cursor:not-allowed}.secondary{border:1px solid var(--line);border-radius:4px;background:#fff;color:var(--ink);padding:7px 11px;cursor:pointer}
.arm-row{display:flex;align-items:center;gap:14px;flex-wrap:wrap;margin-bottom:10px}.state{color:var(--muted);font-size:14px}.motor{display:grid;grid-template-columns:72px 1fr 42px 34px 34px;align-items:center;gap:10px;padding:10px 0;border-bottom:1px solid var(--line)}
.motor:last-child{border-bottom:0}.motor-name{font-weight:700}.motor input{width:100%;accent-color:var(--green)}.angle{text-align:right;font-variant-numeric:tabular-nums}.nudge{width:34px;height:34px;padding:0;font-size:20px}
.test,.test-aux{grid-column:2/-1;justify-self:start}.aux{margin-top:26px;padding-top:20px;border-top:1px solid var(--line)}.aux .nudge,#auxMinus,#auxPlus{width:34px;height:34px;padding:0;font-size:20px}.stop{background:#a52c2c}.secondary:disabled,.cell:disabled{opacity:.45;cursor:not-allowed}
.table-wrap{overflow:auto}table{border-collapse:collapse;width:100%;min-width:580px;font-size:13px}th,td{border-bottom:1px solid var(--line);padding:8px;text-align:center}th:first-child,td:first-child{text-align:left}tr.sel{background:#eef5e2}
textarea{width:100%;min-height:170px;margin-top:12px;padding:12px;border:1px solid var(--line);border-radius:4px;background:#fbfcf9;color:var(--ink);font:12px/1.4 Consolas,monospace}
@media(max-width:720px){main{padding:18px 14px 32px}.layout{grid-template-columns:1fr;gap:22px}.board{max-width:340px}.motor{grid-template-columns:62px 1fr 38px 32px 32px;gap:7px}.notice{font-size:13px}}
</style>
</head>
<body>
<header><h1>Arm Calibration</h1><nav><a href="/">Game</a><a class="on" href="/calibrate">Calibrate</a></nav><div class="network" id="network">Connecting…</div></header>
<main>
<div class="notice">Keep the work area clear. Stop if a joint strains. Saving a pose updates the game immediately; press <b>Store to flash</b> to keep the poses after a reset.</div>
<div class="layout">
<section>
<div class="section"><h2>Target</h2><div class="board" id="board"></div><button class="cell wide" id="t10" aria-pressed="false">Pickup (outside board)</button><button class="cell wide" id="t11" aria-pressed="false">Home / rest</button><p class="state" id="selected">Selected cell 1</p><p class="state">Select a target (does not move the arm). <b>Move to target</b> moves M3 toward 90° (never past its start or target angle), travels, then lowers into the saved pose so you can check and fine-tune it.</p></div>
<div class="section"><h2>Servo control</h2><div class="arm-row"><button class="action" id="arm">Arm servos</button><button class="action stop" id="stop">Stop motion</button><span class="state" id="armState">Disarmed</span></div><p class="state">Arming moves every motor to the home pose. Initial alignment can be sudden. Support the arm and clear its entire travel first.</p></div>
<div><h2>Poses</h2><p class="state" id="storeState"></p><div class="arm-row"><button class="action" id="store">Store to flash</button><button class="secondary" id="defaults">Restore compiled defaults</button></div><div class="table-wrap"><table id="poseTable"></table></div><button class="secondary" id="copy">Copy C++ table</button><textarea id="export" readonly aria-label="Exported angle table"></textarea></div>
</section>
<section><div class="section"><h2>Motor angles <span class="state">(0 = D0 … 5 = D5)</span></h2><p class="state">Release a slider to send its angle. Test sweeps one motor slowly to 0°, then 180°, then back (about 29 s from 90°). Other motors hold. Keep this page open during a test. Stop retains holding torque and depends on Wi-Fi; disconnect servo power if a stop cannot reach the controller. Angles are commands, not measured feedback.</p><p class="state" id="motionState">Idle</p><div id="motors"></div></div>
<div class="arm-row"><button class="action" id="goto" disabled>Move to target</button><button class="action" id="save" disabled>Save current angles to target</button></div><p class="state" id="message">Select a target, then arm the servos.</p>
<div class="section aux"><h2>Gripper <span class="state">(motor 0)</span></h2><p class="state" id="gripState">The home pose M0 is the open angle. The game closes the gripper by this amount at the pickup point and reopens it over the X square.</p><div class="arm-row"><label>Close amount <input id="gripAmount" type="number" min="0" max="90" value="15" style="width:70px;padding:8px">°</label><label>Opens when M0 <select id="gripDir" style="padding:8px"><option value="1">increases</option><option value="-1">decreases</option></select></label><button class="action" id="gripSave">Store gripper settings</button></div><div class="arm-row"><button class="secondary" id="gripOpen">Open gripper</button><button class="secondary" id="gripClose">Close gripper</button></div></div>
<div class="section aux"><h2>Stand-alone servo <span class="state">(D11 – not part of the arm)</span></h2><p class="state">For testing a separate servo's angles. It works without arming the arm and is not stored in the poses. Attaching commands it to its last angle (90° at power-up); the first move can be sudden.</p><div class="arm-row"><button class="action" id="auxAttach">Attach D11 servo</button><button class="secondary" id="auxRelease">Release D11</button><span class="state" id="auxState">Detached</span></div><div class="motor"><span class="motor-name">D11</span><input id="auxSlider" type="range" min="0" max="180" value="90" disabled><span class="angle" id="auxAngle">90°</span><button class="secondary" id="auxMinus" disabled aria-label="Decrease D11 servo">−</button><button class="secondary" id="auxPlus" disabled aria-label="Increase D11 servo">+</button><button class="secondary test-aux" id="auxTest" disabled>Test D11: 0–180°</button></div><div class="arm-row"><input id="auxInput" type="number" min="0" max="180" value="90" style="width:80px;padding:8px" disabled aria-label="D11 angle"><button class="secondary" id="auxGo" disabled>Go to angle</button></div></div></section>
</div>
</main>
<script>
const $=id=>document.getElementById(id),board=$('board'),motorList=$('motors');
const targetName=b=>b===10?'Pickup (outside board)':b===11?'Home / rest':'cell '+b;
for(let i=1;i<=9;i++){const b=document.createElement('button');b.className='cell';b.id='t'+i;b.textContent=i;b.setAttribute('aria-pressed',i===1);board.appendChild(b)}
for(let i=1;i<=11;i++)$('t'+i).onclick=()=>call('/api/cal/select?box='+i);
for(let i=0;i<6;i++){motorList.insertAdjacentHTML('beforeend',`<div class="motor"><span class="motor-name">Motor ${i}</span><input id="slider${i}" type="range" min="0" max="180" value="90" disabled><span class="angle" id="angle${i}">90°</span><button class="secondary nudge" data-delta="-1" data-motor="${i}" disabled aria-label="Decrease motor ${i}">−</button><button class="secondary nudge" data-delta="1" data-motor="${i}" disabled aria-label="Increase motor ${i}">+</button><button class="secondary test" data-test="${i}" disabled>Test motor ${i}: 0–180°</button></div>`)}
function status(text){$('message').textContent=text}
let requestQueue=Promise.resolve(),pendingRequests=0,s=null,connected=false,pollTimer=null,requestVersion=0,renderedVersion=0;
function updateControls(){const busy=pendingRequests>0,moving=!!s?.busy,armed=connected&&s?.armed;
$('arm').disabled=busy||!connected||armed||moving;$('save').disabled=$('goto').disabled=busy||moving||!armed;$('store').disabled=$('defaults').disabled=busy||moving||!connected;
document.querySelectorAll('.cell').forEach(b=>b.disabled=busy||moving||!connected);document.querySelectorAll('#motors input,.nudge,.test').forEach(b=>b.disabled=busy||moving||!armed);
if(armed&&!busy&&!moving)document.querySelectorAll('.nudge').forEach(b=>{const a=s.angles[Number(b.dataset.motor)];b.disabled=Number(b.dataset.delta)>0?a>=180:a<=0});
const aux=connected&&!!s?.auxAttached,auxOk=aux&&!busy&&!moving;$('auxAttach').disabled=busy||moving||!connected||aux;$('auxRelease').disabled=busy||!aux;
['auxSlider','auxTest','auxInput','auxGo'].forEach(id=>$(id).disabled=!auxOk);$('auxMinus').disabled=!auxOk||s.auxAngle<=0;$('auxPlus').disabled=!auxOk||s.auxAngle>=180;
$('gripSave').disabled=busy||moving||!connected;$('gripOpen').disabled=$('gripClose').disabled=busy||moving||!armed}
function call(path,priority=false,background=false){
  clearTimeout(pollTimer);if(!background)pendingRequests++;updateControls();
  const execute=async()=>{
    const version=++requestVersion,abort=new AbortController(),timeout=setTimeout(()=>abort.abort(),10000);
    try{
      const url=typeof path==='function'?path():path,passive=url==='/api/state'||url==='/api/stop'||url==='/api/cal/aux/release';
      if(!connected&&!passive&&s)throw new Error('Reload to reconnect before sending more commands');
      if(s?.busy&&!passive)throw new Error('Wait for motion to finish or stop it first');
      if(url!=='/api/state')status('Sending command; waiting for controller...');
      const r=await fetch(url,{cache:'no-store',signal:abort.signal}),data=await r.json();
      if(!r.ok)throw new Error(data.error||'Request failed');
      const jog=url.startsWith('/api/cal/jog?')?new URLSearchParams(url.split('?')[1]):null;
      if(jog&&(!(jog.get('motor')==='6'?data.auxAttached:data.armed)||data.movingMotor!==Number(jog.get('motor'))||data.targetAngle!==Number(jog.get('angle'))))throw new Error('Controller did not acknowledge the requested motor angle');
      if(version>=renderedVersion){renderedVersion=version;connected=true;render(data);if(jog)status((jog.get('motor')==='6'?'D11 servo':'Motor '+jog.get('motor'))+': controller accepted target '+jog.get('angle')+'°.')}
      return data;
    }catch(e){
      if(version>=renderedVersion){renderedVersion=version;if(e.name==='AbortError'||e instanceof TypeError){connected=false;clearTimeout(pollTimer);$('network').textContent='Connection lost; reload to reconnect';status('Controller timed out (it may be busy talking to Gemini). Motion may have occurred; use Stop motion and check the arm, then reload.')}else{status(e.message);schedule()}}
      throw e;
    }finally{clearTimeout(timeout);if(!background)pendingRequests--;updateControls()}
  };
  const request=priority?execute():requestQueue.then(execute);if(!priority)requestQueue=request.catch(()=>{});return request;
}
function schedule(){clearTimeout(pollTimer);pollTimer=setTimeout(()=>{call('/api/state',false,true).catch(()=>{})},s&&s.busy?500:1500)}
function render(d){s=d;schedule();
const motorLabel=m=>m===6?'D11 servo':'motor '+m,motorAngle=m=>m===6?s.auxAngle:s.angles[m];
$('motionState').textContent=s.movingMotor>=0?(s.testing?'Testing ':'Moving ')+motorLabel(s.movingMotor)+': '+motorAngle(s.movingMotor)+'° → '+s.targetAngle+'°':s.motionCode!=='idle'?'Arm: '+s.motion:s.thinking?'Game: waiting for Gemini':s.motionStatus;
$('auxState').textContent=s.auxAttached?'Attached at '+s.auxAngle+'° (commanded)':'Detached';if(document.activeElement!==$('auxSlider')||s.movingMotor===6){$('auxSlider').value=s.auxAngle;$('auxAngle').textContent=s.auxAngle+'°'}if(document.activeElement!==$('auxInput'))$('auxInput').value=s.auxAngle;
$('network').textContent='Device online';$('gripState').textContent='Open = home M0 = '+s.gripOpen+'°, closed = '+s.gripClosedAngle+'°. Gripper is '+(s.gripClosed?'closed':'open')+' (M0 now '+s.angles[0]+'°).';if(![$('gripAmount'),$('gripDir')].includes(document.activeElement)&&!gripDirty){$('gripAmount').value=s.gripAmount;$('gripDir').value=String(s.gripDir)}$('selected').textContent='Selected '+targetName(s.box);$('armState').textContent=s.armed?'Armed':'Disarmed';$('arm').textContent=s.armed?'Servos armed':'Arm servos';
for(let i=1;i<=11;i++)$('t'+i).setAttribute('aria-pressed',i===s.box);
for(let i=0;i<6;i++){const slider=$('slider'+i);if(document.activeElement!==slider||s.movingMotor>=0||!s.armed){slider.value=s.angles[i];$('angle'+i).textContent=s.angles[i]+'°'}}
let html='<thead><tr><th>Target</th>';for(let m=0;m<6;m++)html+='<th>M'+m+'</th>';html+='</tr></thead><tbody>';
for(let b=0;b<11;b++){html+='<tr'+(b+1===s.box?' class="sel"':'')+'><th>'+targetName(b+1)+'</th>';for(let m=0;m<6;m++)html+='<td>'+s.poses[b][m]+'</td>';html+='</tr>'}$('poseTable').innerHTML=html+'</tbody>';
$('storeState').textContent='Poses loaded from '+(s.poseSource==='flash'?'flash':'compiled defaults')+'. '+(s.unstored?'Unstored changes – press Store to flash to keep them after reset.':'No unstored changes.');
const row=i=>s.poses[i].join(', ');
$('export').value='const int boxAngles[9][6] = {\n'+s.poses.slice(0,9).map((r,i)=>'  {'+row(i)+'}'+(i<8?',':'')).join('\n')+'\n};\n\n// Outside-board pickup point; keep the resting/home pose separate.\nconst int pickupAngles[6] = {'+row(9)+'};\nconst int homeAngles[6] = {'+row(10)+'};';
updateControls()}
$('arm').onclick=async()=>{if(!confirm('Arm all six servos at the home pose? Initial movement can be sudden. Support the arm and clear its entire travel first.'))return;try{await call('/api/cal/arm');status('Armed at home. Adjust one motor at a time.')}catch(e){}};
$('stop').onclick=async()=>{try{await call('/api/stop',true);status('Stopped. Servos retain holding torque.')}catch(e){}};
$('goto').onclick=async()=>{if(!confirm('Move the arm to the saved pose for '+targetName(s.box)+'? M3 moves toward 90° (never past its start or target angle), then the arm travels and lowers.'))return;try{await call('/api/cal/goto');status('Moving to '+targetName(s.box)+'…')}catch(e){}};
document.querySelectorAll('.test').forEach(b=>b.onclick=async()=>{const m=Number(b.dataset.test);if(!confirm('Test motor '+m+' through its FULL 0-180 degree range and return to its starting angle? Clear the whole travel and support the arm. Stop immediately if a joint strains.'))return;try{await call('/api/cal/test?motor='+m)}catch(e){}});
$('save').onclick=async()=>{try{const d=await call('/api/cal/save');status('Pose saved to '+targetName(d.box)+' (game uses it now).')}catch(e){}};
$('store').onclick=async()=>{if(!confirm('Write all 11 poses to flash so they survive a reset?'))return;try{await call('/api/cal/store');status('Poses stored to flash.')}catch(e){}};
$('defaults').onclick=async()=>{if(!confirm('Replace all poses in RAM with the compiled defaults? (Flash is unchanged until you press Store to flash.)'))return;try{await call('/api/cal/defaults');status('Compiled defaults restored in RAM.')}catch(e){}};
for(let i=0;i<6;i++){const slider=$('slider'+i);slider.oninput=e=>{$('angle'+i).textContent=e.target.value+'° (pending)'};slider.onchange=async e=>{try{await call('/api/cal/jog?motor='+i+'&angle='+Number(e.target.value))}catch(err){}}}
document.querySelectorAll('.nudge').forEach(b=>b.onclick=async()=>{const m=Number(b.dataset.motor),d=Number(b.dataset.delta);try{await call(()=>'/api/cal/jog?motor='+m+'&angle='+Math.max(0,Math.min(180,s.angles[m]+d)))}catch(e){}});
const auxJog=a=>call(()=>'/api/cal/jog?motor=6&angle='+Math.max(0,Math.min(180,typeof a==='function'?a():a))).catch(()=>{});
$('auxAttach').onclick=async()=>{if(!confirm('Attach the stand-alone servo on D11? It will move to '+s.auxAngle+'° immediately. Keep it clear.'))return;try{await call('/api/cal/aux/attach');status('D11 servo attached.')}catch(e){}};
$('auxRelease').onclick=async()=>{try{await call('/api/cal/aux/release',true);status('D11 servo released.')}catch(e){}};
let gripDirty=false;$('gripAmount').oninput=$('gripDir').onchange=()=>{gripDirty=true};
$('gripSave').onclick=async()=>{const a=Number($('gripAmount').value);if(!Number.isInteger(a)||a<0||a>90){status('Close amount must be 0-90.');return}try{await call('/api/cal/grip?amount='+a+'&dir='+$('gripDir').value);gripDirty=false;status('Gripper settings stored to flash.')}catch(e){}};
$('gripOpen').onclick=async()=>{try{await call('/api/cal/grip/open');status('Opening gripper…')}catch(e){}};
$('gripClose').onclick=async()=>{try{await call('/api/cal/grip/close');status('Closing gripper…')}catch(e){}};
$('auxSlider').oninput=e=>{$('auxAngle').textContent=e.target.value+'° (pending)'};$('auxSlider').onchange=e=>auxJog(Number(e.target.value));
$('auxMinus').onclick=()=>auxJog(()=>s.auxAngle-1);$('auxPlus').onclick=()=>auxJog(()=>s.auxAngle+1);
$('auxGo').onclick=()=>{const a=Number($('auxInput').value);if(!Number.isInteger(a)||a<0||a>180){status('Enter a whole angle from 0 to 180.');return}auxJog(a)};
$('auxInput').onkeydown=e=>{if(e.key==='Enter')$('auxGo').click()};
$('auxTest').onclick=async()=>{if(!confirm('Sweep the D11 servo through 0-180 degrees and back to its current angle? Keep this page open.'))return;try{await call('/api/cal/test?motor=6')}catch(e){}};
$('copy').onclick=async()=>{try{await navigator.clipboard.writeText($('export').value);status('C++ angle table copied.')}catch(e){$('export').select();status('Select and copy the angle table.')}};
call('/api/state').catch(()=>{$('network').textContent='Controller not responding';status('Reload this page to reconnect.')});
</script>
</body>
</html>)CALUI";

// ---------------------------------------------------------------- HTTP
String jsonEscape(const String &text) {
  String out;
  out.reserve(text.length() + 8);
  for (size_t i = 0; i < text.length(); i++) {
    char c = text[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c < 0x20) out += ' ';
    else out += c;
  }
  return out;
}

String stateJson(uint32_t since) {
  String body;
  body.reserve(3200);
  body += "{\"armed\":";
  body += armed ? "true" : "false";
  body += ",\"rules\":";
  body += rulesPassed ? "true" : "false";
  body += ",\"busy\":";
  body += busy() ? "true" : "false";
  body += ",\"thinking\":";
  body += robotRequested ? "true" : "false";
  body += ",\"turnCode\":\"";
  body += turnCode();
  body += "\",\"turn\":\"";
  body += turnName();
  body += "\",\"motionCode\":\"";
  body += motion == Motion::Idle ? "idle" : "moving";
  body += "\",\"motion\":\"";
  body += motionName(motion);
  body += "\",\"notice\":\"";
  body += jsonEscape(cameraNotice);
  body += "\",\"board\":\"";
  for (char cell : board.cells) body += cell ? cell : '-';
  body += "\",\"winner\":\"";
  char winner = board.winner();
  if (winner) body += winner;
  else if (board.full()) body += 'D';
  body += "\",\"ip\":\"";
  if (networkConnected) body += WiFi.localIP().toString();
  body += "\",\"movingMotor\":";
  body += String(calMotor);
  body += ",\"testing\":";
  body += calTesting ? "true" : "false";
  body += ",\"targetAngle\":";
  body += String(calTarget);
  body += ",\"motionStatus\":\"";
  body += jsonEscape(calStatus);
  body += "\",\"box\":";
  body += String(selectedTarget + 1);
  body += ",\"poseSource\":\"";
  body += posesFromFlash ? "flash" : "defaults";
  body += "\",\"unstored\":";
  body += posesUnstored ? "true" : "false";
  body += ",\"auxPin\":";
  body += String(auxPin);
  body += ",\"auxAttached\":";
  body += auxAttached ? "true" : "false";
  body += ",\"auxAngle\":";
  body += String(auxAngle);
  body += ",\"gripAmount\":" + String(gripAmount);
  body += ",\"gripDir\":" + String(gripOpenSign);
  body += ",\"gripOpen\":" + String(gripOpenAngle());
  body += ",\"gripClosedAngle\":" + String(gripClosedAngle());
  body += ",\"gripClosed\":";
  body += gripClosed ? "true" : "false";
  body += ",\"angles\":[";
  for (uint8_t motor = 0; motor < motorCount; motor++) {
    if (motor > 0) body += ',';
    body += String(angles[motor]);
  }
  body += "],\"poses\":[";
  for (uint8_t target = 0; target < targetCount; target++) {
    if (target > 0) body += ',';
    body += '[';
    for (uint8_t motor = 0; motor < motorCount; motor++) {
      if (motor > 0) body += ',';
      body += String(poses[target][motor]);
    }
    body += ']';
  }
  uint32_t oldest = logCount > logCapacity ? logCount - logCapacity : 0;
  uint32_t first = (since > logCount || since < oldest) ? oldest : since;
  body += "],\"logNext\":";
  body += String(logCount);
  body += ",\"log\":[";
  for (uint32_t i = first; i < logCount; i++) {
    if (i > first) body += ',';
    body += '"';
    body += jsonEscape(logLines[i % logCapacity]);
    body += '"';
  }
  body += "]}";
  return body;
}

bool queryNumber(const char *query, const char *key, long &value) {
  if (query == nullptr) return false;
  size_t keyLength = strlen(key);
  while (*query != '\0') {
    if (strncmp(query, key, keyLength) == 0 && query[keyLength] == '=') {
      const char *start = query + keyLength + 1;
      char *end = nullptr;
      value = strtol(start, &end, 10);
      return end != start && (*end == '\0' || *end == '&');
    }
    query = strchr(query, '&');
    if (query == nullptr) break;
    query++;
  }
  return false;
}

int queryAngle(const char *query, const char *key) {
  long value;
  if (!queryNumber(query, key, value) || value < 0 || value > 180) return -1;
  return int(value);
}

// Only letters, digits and '!' are accepted; anything else yields an empty command.
String queryCommand(const char *query) {
  if (query == nullptr) return "";
  const char *found = query;
  while (found != nullptr && strncmp(found, "c=", 2) != 0) {
    found = strchr(found, '&');
    if (found) found++;
  }
  if (found == nullptr) return "";
  String text;
  for (const char *p = found + 2; *p && *p != '&'; p++) {
    if (!isalnum(*p) && *p != '!') return "";
    if (text.length() >= 12) return "";
    text += *p;
  }
  return text;
}

void respond(WiFiClient &client, const char *status, const char *type, const char *body) {
  size_t length = strlen(body);
  String header = String("HTTP/1.1 ") + status + "\r\nContent-Type: " + type +
      "\r\nContent-Length: " + String(length) +
      "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
  client.print(header);
  serviceMotion();
  size_t sent = 0;
  while (sent < length) {
    size_t written = client.write(reinterpret_cast<const uint8_t *>(body + sent), min(size_t(512), length - sent));
    serviceMotion();
    if (written == 0) {
      Serial.println("[HTTP] Response write failed.");
      break;
    }
    sent += written;
  }
}

bool readRequestLine(WiFiClient &client, String &line, unsigned long deadlineStart) {
  line = "";
  while (millis() - deadlineStart < 3000) {
    serviceMotion();
    if (!client.available()) {
      delay(1);
      continue;
    }
    int byte = client.read();
    if (byte < 0) continue;
    if (byte == '\n') return true;
    if (line.length() >= 512) return false;
    line += char(byte);
  }
  return false;
}

const char *startCalMotion(int motor, bool test, int angle) {
  if (!test && angle < 0) return "Invalid angle";
  if (!jointServo(motor).attached()) return "Servo is not attached";
  if (!test) angle = constrain(angle, 0, 180);
  calMotor = motor;
  calTesting = test;
  calHeartbeatAt = calProgressAt = millis();
  String name = motor == auxMotor ? "D" + String(auxPin) + " servo" : "motor " + String(motor);
  float from = positions[motor];
  Plan next;
  clearPlan(next);
  if (test) {
    calStatus = "Full-range test in progress";
    int low = motor == auxMotor ? 0 : minAngle(motor);
    int high = motor == auxMotor ? 180 : maxAngle(motor);
    calTarget = int(from + 0.5f);
    uint32_t t1 = moveDuration(fabsf(from - low), testStepMs);
    uint32_t t2 = t1 + moveDuration(high - low, testStepMs);
    uint32_t t3 = t2 + moveDuration(fabsf(high - from), testStepMs);
    addSegment(next, motor, 0, t1, from, low);
    addSegment(next, motor, t1, t2, low, high);
    addSegment(next, motor, t2, t3, high, from);
    say("[Calibrate] Testing " + name + ": " + String(low) + " -> " + String(high) + " -> " + String(calTarget));
  } else {
    calStatus = "Jog in progress";
    calTarget = angle;
    Serial.println("[Jog] " + name + ": " + String(int(from + 0.5f)) + " -> " + String(angle));
    addSegment(next, motor, 0, moveDuration(fabsf(angle - from), jogStepMs), from, angle);
  }
  commitPlan(next);
  return nullptr;
}

const char *handleAuxServo(const String &route, const char *query) {
  if (route == "/api/cal/aux/attach") {
    if (auxAttached) return nullptr;
    if (busy()) return "Arm is busy; wait or stop first";
    if (!auxServo.attach(auxPin)) return "D11 servo attachment failed";
    auxAttached = true;
    writeJoint(auxMotor, auxAngle);
    say("[Calibrate] D" + String(auxPin) + " servo attached at " + String(auxAngle) + " degrees.");
    return nullptr;
  }
  if (route == "/api/cal/aux/release") {
    if (calMotor == auxMotor) stopCalMotion("D11 servo released.");
    auxServo.detach();
    auxAttached = false;
    say("[Calibrate] D" + String(auxPin) + " servo pulses released.");
    return nullptr;
  }
  if (!auxAttached) return "Attach the D11 servo first";
  if (busy()) return "Arm is busy; wait or stop first";
  return startCalMotion(auxMotor, route == "/api/cal/test", queryAngle(query, "angle"));
}

const char *handleCalibration(const String &route, const char *query) {
  if (route == "/api/cal/arm") return armServos();
  if (route.startsWith("/api/cal/aux/") ||
      ((route == "/api/cal/jog" || route == "/api/cal/test") && queryAngle(query, "motor") == auxMotor)) {
    return handleAuxServo(route, query);
  }
  if (route == "/api/cal/select") {
    long box;
    if (!queryNumber(query, "box", box) || box < 1 || box > targetCount) return "Invalid target";
    if (busy()) return "Arm is busy; wait or stop first";
    selectedTarget = uint8_t(box - 1);
    return nullptr;
  }
  if (route == "/api/cal/grip") {
    long amount, dir;
    if (!queryNumber(query, "amount", amount) || amount < 0 || amount > 90) return "Grip amount must be 0-90";
    if (!queryNumber(query, "dir", dir) || (dir != 1 && dir != -1)) return "Invalid open direction";
    if (busy()) return "Arm is busy; wait or stop first";
    gripAmount = int(amount);
    gripOpenSign = int(dir);
    storeGripSettings();
    say("[Calibrate] Gripper settings stored: open " + String(gripOpenAngle()) + ", closed " + String(gripClosedAngle()) + " degrees.");
    return nullptr;
  }
  if (!rulesPassed) return "Rules self-test failed; motion disabled";
  if (route == "/api/cal/store") {
    if (busy()) return "Wait for motion to finish before storing";
    storePoses();
    say("[Calibrate] All poses stored to flash.");
    return nullptr;
  }
  if (route == "/api/cal/defaults") {
    if (busy()) return "Wait for motion to finish first";
    loadDefaultPoses();
    posesUnstored = true;
    say("[Calibrate] Compiled default poses restored in RAM (not yet stored).");
    return nullptr;
  }
  if (!armed) return "Arm servos first";
  if (busy()) return "Arm is busy; wait or stop first";
  if (route == "/api/cal/jog") {
    int motor = queryAngle(query, "motor");
    int angle = queryAngle(query, "angle");
    if (motor < 0 || motor >= motorCount || angle < 0) return "Invalid motor or angle";
    return startCalMotion(motor, false, angle);
  }
  if (route == "/api/cal/test") {
    int motor = queryAngle(query, "motor");
    if (motor < 0 || motor >= motorCount) return "Invalid motor";
    return startCalMotion(motor, true, 0);
  }
  if (route == "/api/cal/save") {
    for (uint8_t motor = 0; motor < motorCount; motor++) poses[selectedTarget][motor] = angles[motor];
    posesUnstored = true;
    say("[Calibrate] Saved " + targetName(selectedTarget) + " = {" + poseText(angles) + "}");
    return nullptr;
  }
  if (route == "/api/cal/goto") {
    say("[Calibrate] Moving to " + targetName(selectedTarget) + ".");
    startTransfer(poses[selectedTarget], Motion::Preview, false);
    return nullptr;
  }
  if (route == "/api/cal/grip/open" || route == "/api/cal/grip/close") {
    bool close = route.endsWith("close");
    say(String("[Calibrate] ") + (close ? "Closing" : "Opening") + " gripper (M0).");
    startGrip(close, Motion::GripTest);
    return nullptr;
  }
  return "Not found";
}

void handleRequest(WiFiClient &client) {
  unsigned long headerStart = millis();
  String request;
  if (!readRequestLine(client, request, headerStart)) {
    respond(client, "400 Bad Request", "text/plain", "Invalid request");
    return;
  }
  bool headersComplete = false;
  while (millis() - headerStart < 3000) {
    String header;
    if (!readRequestLine(client, header, headerStart)) break;
    if (header == "\r" || header.length() == 0) {
      headersComplete = true;
      break;
    }
  }
  int pathEnd = request.indexOf(' ', 4);
  if (!headersComplete || !request.startsWith("GET ") || pathEnd < 0) {
    respond(client, "400 Bad Request", "text/plain", "Invalid request");
    return;
  }
  String path = request.substring(4, pathEnd);
  int queryStart = path.indexOf('?');
  String route = queryStart < 0 ? path : path.substring(0, queryStart);
  const char *query = queryStart < 0 ? nullptr : path.c_str() + queryStart + 1;
  if (route != "/api/state") Serial.println("[HTTP] " + path);

  if (route == "/") {
    respond(client, "200 OK", "text/html; charset=utf-8", gamePage);
    return;
  }
  if (route == "/calibrate") {
    respond(client, "200 OK", "text/html; charset=utf-8", calibrationPage);
    return;
  }
  if (route == "/favicon.ico") {
    respond(client, "204 No Content", "text/plain", "");
    return;
  }

  const char *error = nullptr;
  if (route == "/api/state") {
    if (calTesting) calHeartbeatAt = millis();
  } else if (route == "/api/stop") {
    if (robotRequested || (motion != Motion::Idle && motion != Motion::Preview && motion != Motion::GripTest)) {
      stopGame(false);
    } else {
      if (motion == Motion::Preview || motion == Motion::GripTest) {
        haltPlan();
        syncAngles();
        motion = Motion::Idle;
        say("[Calibrate] Move stopped. Servos retain holding torque.");
      }
      stopCalMotion("Stopped by user. Servos retain holding torque.");
    }
  } else if (route == "/api/cmd") {
    String text = queryCommand(query);
    if (text.length() == 0) error = "Invalid command";
    else processCommand(text);
  } else if (route.startsWith("/api/cal/")) {
    error = handleCalibration(route, query);
    if (error != nullptr && strcmp(error, "Not found") == 0) {
      respond(client, "404 Not Found", "application/json", "{\"error\":\"Not found\"}");
      return;
    }
  } else {
    respond(client, "404 Not Found", "application/json", "{\"error\":\"Not found\"}");
    return;
  }

  if (error != nullptr) {
    String body = String("{\"error\":\"") + jsonEscape(error) + "\"}";
    respond(client, "400 Bad Request", "application/json", body.c_str());
  } else {
    long since = 0;
    if (!queryNumber(query, "since", since) || since < 0) since = 0;
    String body = stateJson(uint32_t(since));
    respond(client, "200 OK", "application/json", body.c_str());
  }
}

IPAddress serverAddress;

// fresh=true soft-resets the Wi-Fi module first. The module is not reset with the sketch,
// so stale sockets from a previous run can leave port 80 dead while Wi-Fi still looks connected.
void startNetwork(bool fresh = false) {
  server.end();
  if (fresh) {
    say("[Network] Resetting Wi-Fi module (firmware " + String(WiFi.firmwareVersion()) + ")...");
    WiFi.end();
    delay(500);
  }
  networkConnected = !fresh && WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0);
  if (!networkConnected) {
    say("[Network] Connecting to Wi-Fi...");
    networkConnected = wifi_connection::connect() && WiFi.localIP() != IPAddress(0, 0, 0, 0);
  }
  if (networkConnected) {
    server.begin();
    serverAddress = WiFi.localIP();
    say("[Network] Game page:        http://" + serverAddress.toString() + "/");
    say("[Network] Calibration page: http://" + serverAddress.toString() + "/calibrate");
    if (!server) say("[Network] HTTP server failed to open port 80; will retry.");
  } else {
    say("[Network] Wi-Fi unavailable; retrying every 15s. Serial commands still work.");
  }
  lastNetworkAttempt = millis();
  lastNetworkCheckAt = millis();
}

void serviceNetwork() {
  if (millis() - lastNetworkCheckAt >= 1000) {
    lastNetworkCheckAt = millis();
    networkConnected = WiFi.status() == WL_CONNECTED;
    // The module can reconnect on its own with a new address; reopen the listener if so.
    if (networkConnected && server && WiFi.localIP() != serverAddress) {
      say("[Network] IP address changed; restarting web server.");
      startNetwork();
    }
  }
  // Reconnecting blocks for up to 30s, so only try while the arm is still.
  if ((!networkConnected || !server) && !busy() && millis() - lastNetworkAttempt >= 15000) startNetwork();
  if (!networkConnected || !server) return;
  WiFiClient client = server.available();
  if (client) {
    handleRequest(client);
    client.stop();
  }
}

// ---------------------------------------------------------------- main
void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);
  loadDefaultPoses();
  posesFromFlash = loadStoredPoses();
  say(posesFromFlash ? "[Calibrate] Loaded calibrated poses from flash." : "[Calibrate] Using compiled default poses.");
  for (uint8_t motor = 0; motor < motorCount; motor++) {
    angles[motor] = poses[homeTarget][motor];
    positions[motor] = angles[motor];
  }
  positions[auxMotor] = auxAngle;
  loadGripSettings();
  say("[Gripper] M0 open " + String(gripOpenAngle()) + ", closed " + String(gripClosedAngle()) + " degrees.");
  startMotionTimer();
  rulesPassed = tic_tac_toe::selfTest();
  say(rulesPassed ? "Tic tac toe rules self-test PASS." : "Rules self-test FAILED.");
  printHelp();
  printBoard();
  say("Choose arm FIRST or SECOND after ARM. No automatic startup movement.");
  startNetwork(true);
  printStatus(false);
}

// Between WiFi modem round-trips while GET /board is in progress.
// This keeps the no-timer motion fallback moving. '!' cancels the capture
// before a piece is committed. Other serial and HTTP commands wait until the
// attempt returns: parsing them here would run while the socket is still open,
// and STOP clears "busy".
void pumpDuringCameraRead() {
  serviceMotion();
  if (Serial.available() && Serial.peek() == '!') {
    Serial.read();
    stopGame(false);
  }
}

String knownBoardText() {
  String text;
  text.reserve(9);
  for (char cell : board.cells) text += cell ? cell : '.';
  return text;
}

// Short list of cells the camera disagrees with, for the status line.
String cameraChanges(const char seen[9]) {
  String text;
  int shown = 0;
  for (int i = 0; i < 9; i++) {
    char known = board.cells[i];
    char expected = known == 0 ? '.' : known;
    if (seen[i] == expected) continue;
    if (shown == 3) {
      text += "; ...";
      break;
    }
    if (shown > 0) text += "; ";
    text += "square " + String(i + 1) + " ";
    text += known ? String(known) : String("empty");
    text += "->";
    text += seen[i];
    shown++;
  }
  return text;
}

bool cameraStillWanted() {
  return cameraJob == CameraJob::Read && armed && turn == Turn::Human;
}

void rejectCamera(const String &message) {
  cameraJob = CameraJob::Idle;
  cameraNotice = message;
  say("[Camera] " + message);
}

// Up to 3 GET /board attempts, each about 3 s. Two identical reads in a row
// are the debounced picture. A failed attempt breaks the pair. The FspTimer
// is not stopped; pumpDuringCameraRead only fills the gaps between modem calls.
void readCameraAndApply() {
  char seen[9];
  char previous[9];
  bool havePrevious = false;
  int badBodies = 0;
  int goodReads = 0;
  bool gotPair = false;

  for (uint8_t attempt = 1; attempt <= cameraAttempts; attempt++) {
    // The camera socket is closed here, so a waiting poll can show status
    // and Stop can cancel the capture before the next GET.
    serviceNetwork();
    if (!cameraStillWanted()) return;
    char got[9];
    String oneError;
    bool ok = board_camera::fetchBoard(got, oneError, cameraAttemptMs, pumpDuringCameraRead);
    if (!cameraStillWanted()) return;
    if (!ok) {
      // A 200 with a bad body is not the same failure as a camera that never answers.
      bool badBody = oneError.indexOf("body") >= 0 || oneError.indexOf("HTTP") >= 0 || oneError.indexOf("unreadable") >= 0;
      if (badBody) badBodies++;
      havePrevious = false;
      say("[Camera] Attempt " + String(attempt) + " of " + String(cameraAttempts) + " failed: " + oneError);
      continue;
    }
    goodReads++;
    char shown[10];
    memcpy(shown, got, 9);
    shown[9] = '\0';
    say(String("[Camera] Attempt ") + String(attempt) + " of " + String(cameraAttempts) + " read " + shown);
    if (havePrevious && memcmp(previous, got, 9) == 0) {
      memcpy(seen, got, 9);
      gotPair = true;
      break;
    }
    memcpy(previous, got, 9);
    havePrevious = true;
  }

  if (!cameraStillWanted()) return;
  if (!gotPair) {
    String message;
    if (goodReads == 0 && badBodies == 0) {
      message = String("Camera unreachable at ") + CAMERA_HOST;
      if (WiFi.status() != WL_CONNECTED) message += " (Wi-Fi is not connected)";
      else message += String(" (GET /board failed after ") + String(cameraAttempts) + " tries)";
      message += ".";
    } else if (goodReads == 0) {
      message = "Camera answered, but the body was not 9 cells of X, O, and dot.";
    } else {
      message = "Camera readings did not match (need two identical looks in a row).";
    }
    message += " Board left unchanged. Press I've moved again, or tap the square.";
    rejectCamera(message);
    return;
  }

  char shown[10];
  memcpy(shown, seen, 9);
  shown[9] = '\0';
  say(String("[Camera] Debounced board ") + shown + " (known " + knownBoardText() + ").");

  char known[9];
  for (int i = 0; i < 9; i++) known[i] = board.cells[i];
  CameraDiff diff = diffCameraBoard(known, seen, humanPiece, robotPiece);
  if (diff.kind == CameraDiffKind::OneHumanMove) {
    say(String("[Camera] Confirmed ") + humanPiece + " in square " + String(diff.square) + ".");
    cameraJob = CameraJob::Idle;
    acceptHumanSquare(diff.square);
    return;
  }

  String detail = cameraChanges(seen);
  String message;
  if (diff.kind == CameraDiffKind::NoChange) {
    message = String("Camera saw no new ") + humanPiece + ".";
  } else if (diff.kind == CameraDiffKind::RobotMissing) {
    message = String("Camera is missing a robot ") + robotPiece;
    if (detail.length() > 0) message += " (" + detail + ")";
    message += ".";
  } else {
    message = String("Camera saw ") + String(diff.changes) + (diff.changes == 1 ? " change" : " changes");
    if (detail.length() > 0) message += " (" + detail + ")";
    message += String(", not one new ") + humanPiece + " in an empty square.";
  }
  message += " Board left unchanged. Press I've moved again, or tap the square.";
  rejectCamera(message);
}

void serviceCamera() {
  // If the park pose ended without the Park handler, still honour the settle.
  if (cameraJob == CameraJob::WaitMotion && motion == Motion::Idle) {
    if (!armed || turn != Turn::Human) {
      cameraJob = CameraJob::Idle;
      return;
    }
    beginCameraSettle();
  }
  if (cameraJob != CameraJob::Settle) return;
  if (millis() - cameraSettleAt < cameraSettleMs) return;
  cameraJob = CameraJob::Read;
  cameraNotice = "Reading the board camera.";
  say("[Camera] " + cameraNotice);
  readCameraAndApply();
}

void loop() {
  readSerial();
  serviceMotion();
  // Park, settle, then the camera read. A web STOP that arrived during the
  // read is handled by serviceNetwork below, before Gemini is asked to move.
  serviceCamera();
  // A STOP that arrived during the camera read lands here, before Gemini runs.
  readSerial();
  serviceNetwork();
  serviceMotion();
  if (robotRequested && motion == Motion::Idle && calMotor < 0 && cameraJob == CameraJob::Idle) {
    robotRequested = false;
    if (armed && turn == Turn::Robot) robotMove();
  }
  if (networkRestartRequested && !busy()) {
    networkRestartRequested = false;
    startNetwork(true);
  }
  if (millis() - lastStatusAt >= 10000) {
    printStatus(false);
    lastStatusAt = millis();
  }
}
