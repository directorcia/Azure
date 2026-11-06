#include <Arduino.h>
#include <Servo.h>
#include "gemini_connection.h"
#include "wifi_connection.h"
#include "tic_tac_toe_rules.h"

constexpr uint8_t motorCount = 6;
constexpr unsigned long stepMs = 40;
constexpr unsigned long commandIdleMs = 500;
constexpr uint8_t liftMotor = 3;
constexpr int clearanceAngle = 90;
const int homeAngles[motorCount] = {170, 90, 90, 135, 180, 90};
const int boxAngles[9][6] = {
  {175, 60, 3, 157, 179, 65},
  {175, 80, 3, 157, 179, 80},
  {175, 100, 3, 157, 179, 95},
  {175, 100, 13, 152, 174, 65},
  {175, 100, 13, 152, 174, 80},
  {175, 100, 18, 152, 174, 95},
  {175, 100, 33, 157, 179, 75},
  {175, 100, 33, 157, 179, 85},
  {175, 100, 38, 157, 179, 95}
};
// Outside-board pickup point; keep the resting/home pose separate.
const int pickupAngles[6] = {175, 100, 38, 172, 180, 125};
enum class Turn { ChooseOrder, Human, Robot, Finished, Stopped };
enum class Motion { Idle, Lift, Travel, Point, Pause, Return, Jig };
Servo motors[motorCount];
tic_tac_toe::Board board;
Turn turn = Turn::ChooseOrder;
Motion motion = Motion::Idle;
bool armed = false;
bool rulesPassed = false;
int angles[motorCount];
int origins[motorCount];
int targets[motorCount];
int destinationAngles[motorCount];
Motion arrivalMotion = Motion::Idle;
int totalSteps = 0;
int currentStep = 0;
uint8_t jigStage = 0;
unsigned long lastStepAt = 0;
unsigned long pauseAt = 0;
String command;
bool commandOverflow = false;
unsigned long lastStatusAt = 0;
unsigned long lastCommandByteAt = 0;

const char *turnName() {
  switch (turn) {
    case Turn::ChooseOrder: return "waiting for FIRST/SECOND";
    case Turn::Human: return "human O: waiting for square 1-9";
    case Turn::Robot: return "robot X turn (RETRY if last request failed)";
    case Turn::Finished: return "game finished; NEW to restart";
    case Turn::Stopped: return "stopped";
  }
  return "unknown";
}

const char *motionName(Motion phase) {
  switch (phase) {
    case Motion::Idle: return "idle";
    case Motion::Lift: return "lifting M3 to 90 degrees";
    case Motion::Travel: return "travelling above board with M3 at 90 degrees";
    case Motion::Point: return "pointing to X square";
    case Motion::Pause: return "holding at square for 5s";
    case Motion::Return: return "returning to pickup";
    case Motion::Jig: return "victory jig";
  }
  return "unknown";
}

void printStatus() {
  Serial.print("[Status t=");
  Serial.print(millis() / 1000);
  Serial.print("s] servos=");
  Serial.print(armed ? "ARMED" : "DISARMED");
  Serial.print("; turn=");
  Serial.print(turnName());
  Serial.print("; motion=");
  Serial.print(motionName(motion));
  Serial.print("; Wi-Fi status=");
  Serial.println(WiFi.status());
  if (armed) {
    Serial.print("[Angles M0-M5] ");
    for (uint8_t motor = 0; motor < motorCount; motor++) {
      if (motor > 0) Serial.print(", ");
      Serial.print(angles[motor]);
    }
    Serial.println(" (commanded; no position feedback)");
  }
}

void printBoard() {
  for (int row = 0; row < 3; row++) {
    for (int column = 0; column < 3; column++) {
      int square = row * 3 + column;
      Serial.print(board.cells[square] ? board.cells[square] : char('1' + square));
      if (column < 2) Serial.print(" | ");
    }
    Serial.println();
  }
}

void printHelp() {
  Serial.println("You are O. Gemini and the arm are X. Squares: 1 2 3 / 4 5 6 / 7 8 9.");
  Serial.println("ARM: enable servos at home. Initial alignment can be sudden; clear and support the arm.");
  Serial.println("FIRST: arm starts. SECOND: human starts. Enter one square 1-9 for each human O.");
  Serial.println("RETRY: retry Gemini. NEW: clear board. BOARD: show board. TEST: check rules. STATUS: show activity.");
  Serial.println("Commands ignore case. Press Send; commands also work without a line ending after 500ms.");
  Serial.println("STOP or !: cancel motion/game, retain holding torque. RELEASE: remove pulses; support arm first.");
  Serial.println("The arm points only. Place an X marker at its indicated square yourself.");
  Serial.println("The arm holds at the selected square for 5s, then returns to the pickup point.");
  Serial.println("Transfers lift M3 to 90 degrees before travel, then lower at the destination.");
}

void startMotion(const int pose[motorCount], Motion next) {
  totalSteps = 0;
  currentStep = 0;
  for (uint8_t motor = 0; motor < motorCount; motor++) {
    origins[motor] = angles[motor];
    targets[motor] = constrain(pose[motor], motor == 0 ? 10 : 0, motor == 0 ? 175 : 180);
    totalSteps = max(totalSteps, abs(targets[motor] - origins[motor]));
  }
  motion = next;
  lastStepAt = millis();
  Serial.print("[Motion] ");
  Serial.print(motionName(next));
  Serial.print("; estimated travel ms=");
  Serial.println(totalSteps * stepMs);
  Serial.print("[Target M0-M5] ");
  for (uint8_t motor = 0; motor < motorCount; motor++) {
    if (motor > 0) Serial.print(", ");
    Serial.print(targets[motor]);
  }
  Serial.println();
}

void startTransfer(const int pose[motorCount], Motion arrival) {
  int liftPose[motorCount];
  for (uint8_t motor = 0; motor < motorCount; motor++) {
    destinationAngles[motor] = pose[motor];
    liftPose[motor] = angles[motor];
  }
  arrivalMotion = arrival;
  liftPose[liftMotor] = clearanceAngle;
  startMotion(liftPose, Motion::Lift);
}

void stopGame(bool release) {
  motion = Motion::Idle;
  turn = Turn::Stopped;
  if (release) {
    for (auto &motor : motors) motor.detach();
    armed = false;
  }
  Serial.println(release ? "Servo pulses released. Support the arm." : "Stopped. Servos still hold position. NEW starts a new game.");
}

void finishRobotMove() {
  char winner = board.winner();
  if (winner == 'X') {
    Serial.println("X wins! Small victory jig at home.");
    turn = Turn::Finished;
    jigStage = 0;
    int pose[motorCount];
    for (uint8_t motor = 0; motor < motorCount; motor++) pose[motor] = homeAngles[motor];
    pose[5] = 85;
    startTransfer(pose, Motion::Jig);
  } else if (board.full()) {
    Serial.println("Draw. Send NEW to play again.");
    turn = Turn::Finished;
  } else {
    turn = Turn::Human;
    Serial.println("Your turn: place O, then enter its square 1-9.");
  }
}

void updateMotion() {
  if (motion == Motion::Idle) return;
  if (motion == Motion::Pause) {
    if (millis() - pauseAt >= 5000) startTransfer(pickupAngles, Motion::Return);
    return;
  }
  if (millis() - lastStepAt < stepMs) return;
  lastStepAt = millis();
  if (currentStep < totalSteps) {
    currentStep++;
    for (uint8_t motor = 0; motor < motorCount; motor++) {
      angles[motor] = origins[motor] + (targets[motor] - origins[motor]) * currentStep / totalSteps;
      motors[motor].write(angles[motor]);
    }
    return;
  }
  if (motion == Motion::Lift) {
    int travelPose[motorCount];
    for (uint8_t motor = 0; motor < motorCount; motor++) {
      travelPose[motor] = destinationAngles[motor];
    }
    travelPose[liftMotor] = clearanceAngle;
    startMotion(travelPose, Motion::Travel);
  } else if (motion == Motion::Travel) {
    startMotion(destinationAngles, arrivalMotion);
  } else if (motion == Motion::Point) {
    Serial.println("[Motion] Cell reached. Holding for 5s; place X at the announced square.");
    motion = Motion::Pause;
    pauseAt = millis();
  } else if (motion == Motion::Return) {
    Serial.println("[Motion] Pickup point reached. Checking win/draw...");
    motion = Motion::Idle;
    finishRobotMove();
  } else if (motion == Motion::Jig) {
    jigStage++;
    if (jigStage >= 6) {
      motion = Motion::Idle;
      Serial.println("Jig complete. Send NEW to play again.");
    } else {
      int pose[motorCount];
      for (uint8_t motor = 0; motor < motorCount; motor++) pose[motor] = homeAngles[motor];
      pose[5] = jigStage == 5 ? 90 : (jigStage % 2 ? 95 : 85);
      startMotion(pose, Motion::Jig);
    }
  }
}

void robotMove() {
  Serial.println("[Network] Checking Wi-Fi before Gemini request...");
  if (WiFi.status() != WL_CONNECTED && !wifi_connection::connect()) {
    Serial.println("Wi-Fi unavailable. Send RETRY; no move made.");
    return;
  }
  Serial.print("[Network] Connected. IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("[Minimax] Evaluating every legal X move...");
  unsigned long searchStartedAt = millis();
  tic_tac_toe::BestMoves best = tic_tac_toe::bestMovesForX(board);
  Serial.print("[Minimax] Score for X: ");
  Serial.print(best.score);
  Serial.print("; search ms: ");
  Serial.println(millis() - searchStartedAt);
  if (best.squares == 0) {
    Serial.println("[Minimax] No available moves. No motion.");
    return;
  }
  String optimalSquares;
  for (int square = 1; square <= 9; square++) {
    if (!best.contains(square)) continue;
    if (optimalSquares.length() > 0) optimalSquares += ", ";
    optimalSquares += String(square);
  }
  Serial.print("[Minimax] Equally optimal squares: ");
  Serial.println(optimalSquares);
  String prompt = "Play tic tac toe as X against human O. Either player may start. ";
  prompt += "A complete minimax search has identified these equally best moves: [" + optimalSquares + "]. ";
  prompt += "Choose ONLY from this set. For an empty board prefer a corner from the set, to create fork opportunities. ";
  prompt += "Return only JSON {\"square\":N} with one integer from the optimal set. Board: ";
  for (int square = 0; square < 9; square++) {
    prompt += String(square + 1) + "=";
    prompt += board.cells[square] ? String(board.cells[square]) : String("empty");
    prompt += "; ";
  }
  Serial.println("Gemini is choosing X's move...");
  Serial.print("[Gemini] Legal squares: ");
  for (int square = 0; square < 9; square++) {
    if (board.cells[square] == 0) {
      Serial.print(square + 1);
      Serial.print(' ');
    }
  }
  Serial.println();
  String reply;
  String error;
  unsigned long requestStartedAt = millis();
  if (!gemini_connection::generate(prompt.c_str(), reply, error, true, true)) {
    Serial.print("[Gemini] Failed after ms: ");
    Serial.println(millis() - requestStartedAt);
    Serial.println(error);
    Serial.println("Send RETRY. The board is unchanged.");
    return;
  }
  Serial.print("[Gemini] Completed in ms: ");
  Serial.println(millis() - requestStartedAt);
  Serial.print("[Gemini] Model reply: ");
  Serial.println(reply.substring(0, 256));
  JsonDocument choice;
  DeserializationError parsed = deserializeJson(choice, reply);
  if (parsed || !choice["square"].is<int>()) {
    Serial.println("Gemini returned an invalid move format. Send RETRY.");
    return;
  }
  int square = choice["square"].as<int>();
  if (!best.contains(square)) {
    Serial.println("[Minimax] Rejected Gemini move: not in the optimal set. No motion. Send RETRY.");
    return;
  }
  if (!board.place(square, 'X')) {
    Serial.println("Gemini chose an occupied or invalid square. No motion. Send RETRY.");
    return;
  }
  Serial.print("X chooses square ");
  Serial.println(square);
  printBoard();
  startTransfer(boxAngles[square - 1], Motion::Point);
}

void processCommand(String text) {
  text.trim();
  text.toUpperCase();
  if (text.length() == 0) return;
  Serial.print("[Command] ");
  Serial.println(text);
  if (text == "STOP" || text == "!") { stopGame(false); return; }
  if (text == "RELEASE") { stopGame(true); return; }
  if (text == "BOARD") { printBoard(); return; }
  if (text == "STATUS") { printStatus(); return; }
  if (text == "TEST") {
    Serial.println(tic_tac_toe::selfTest() ? "Rules/minimax PASS: wins, draw, invalid moves, openings, forced win/block, fork defense." : "Rules/minimax tests FAILED.");
    return;
  }
  if (text == "?") { printHelp(); return; }
  if (!rulesPassed) { Serial.println("Rules self-test failed. Motion disabled."); return; }
  if (motion != Motion::Idle) { Serial.println("Arm is moving; only STOP, RELEASE or BOARD accepted."); return; }
  if (text == "NEW") {
    board = tic_tac_toe::Board{};
    turn = Turn::ChooseOrder;
    printBoard();
    Serial.println("Send FIRST for arm first, or SECOND for arm second.");
  } else if (text == "ARM") {
    if (!armed) {
      Serial.println("[Arm] Enabling all servos at home. Initial alignment is not speed-controlled.");
      for (uint8_t motor = 0; motor < motorCount; motor++) {
        Serial.print("[Arm] Motor ");
        Serial.print(motor);
        Serial.print(" -> ");
        Serial.println(homeAngles[motor]);
        angles[motor] = homeAngles[motor];
        motors[motor].attach(motor);
        motors[motor].write(angles[motor]);
      }
      armed = true;
    }
    Serial.println("Armed. FIRST = arm starts, SECOND = human starts.");
  } else if (text == "FIRST" || text == "SECOND") {
    if (!armed) { Serial.println("Send ARM first, with the mechanism clear."); return; }
    if (turn != Turn::ChooseOrder) { Serial.println("Game already started. Send NEW to reset."); return; }
    turn = text == "FIRST" ? Turn::Robot : Turn::Human;
    Serial.println(text == "FIRST" ? "[Game] X starts; requesting Gemini move." : "[Game] O starts; waiting for human square.");
    if (turn == Turn::Robot) robotMove();
    else Serial.println("Place O and enter its square 1-9.");
  } else if (text == "RETRY") {
    if (armed && turn == Turn::Robot) robotMove();
    else Serial.println("No Gemini move is pending.");
  } else if (text.length() == 1 && text[0] >= '1' && text[0] <= '9') {
    if (!armed || turn != Turn::Human) { Serial.println("Not the human's turn."); return; }
    if (!board.place(text[0] - '0', 'O')) { Serial.println("That square is occupied. Enter an empty square."); return; }
    Serial.print("[Game] Human O accepted in square ");
    Serial.println(text);
    printBoard();
    if (board.winner() == 'O') {
      turn = Turn::Finished;
      Serial.println("O wins! Send NEW to play again.");
    } else if (board.full()) {
      turn = Turn::Finished;
      Serial.println("Draw. Send NEW to play again.");
    } else {
      turn = Turn::Robot;
      robotMove();
    }
  } else Serial.println("Unknown command. Send ? for help.");
}

void submitCommand() {
  String submitted = command;
  bool overflow = commandOverflow;
  command = "";
  commandOverflow = false;
  if (overflow) Serial.println("Command too long; discarded.");
  else processCommand(submitted);
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);
  rulesPassed = tic_tac_toe::selfTest();
  Serial.println(rulesPassed ? "Tic tac toe rules self-test PASS." : "Rules self-test FAILED.");
  printHelp();
  printBoard();
  Serial.println("Choose arm FIRST or SECOND after ARM. No automatic startup movement.");
  printStatus();
}

void loop() {
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
  updateMotion();
  if (millis() - lastStatusAt >= 10000) {
    printStatus();
    lastStatusAt = millis();
  }
}