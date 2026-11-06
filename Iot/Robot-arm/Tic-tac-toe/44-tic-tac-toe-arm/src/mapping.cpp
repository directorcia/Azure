#include <Arduino.h>
#include <Servo.h>
#include <WiFiS3.h>
#include <stdlib.h>
#include <string.h>
#include "arm_page.h"
#include "wifi_connection.h"

constexpr uint8_t motorCount = 6;
constexpr uint8_t boxCount = 9;
constexpr uint8_t targetCount = boxCount + 1;
const uint8_t motorPins[motorCount] = {0, 1, 2, 3, 4, 5};
int currentAngles[motorCount] = {90, 90, 90, 90, 90, 90};
int targetAngles[targetCount][motorCount] = {};
bool targetSaved[targetCount] = {};
uint8_t selectedTarget = 0;
bool motorsArmed = false;
WiFiServer server(80);
Servo motors[motorCount];
unsigned long lastNetworkAttempt = 0;
unsigned long lastNetworkCheckAt = 0;
bool networkConnected = false;
constexpr unsigned long jogStepMs = 20;
constexpr unsigned long testStepMs = 80;
constexpr unsigned long testHeartbeatMs = 4000;
int movingMotor = -1;
int targetAngle = 90;
bool testingMotor = false;
uint8_t testStage = 0;
int testReturnAngle = 90;
unsigned long lastMotionStepAt = 0;
unsigned long lastTestHeartbeatAt = 0;
unsigned long lastProgressAt = 0;
const char *motionStatus = "Idle";

void stopMotion(const char *reason) {
  movingMotor = -1;
  testingMotor = false;
  motionStatus = reason;
  Serial.print("[Motion] ");
  Serial.println(reason);
}

void updateMotion() {
  if (movingMotor < 0) return;
  if (testingMotor && (!networkConnected ||
      millis() - lastTestHeartbeatAt >= testHeartbeatMs)) {
    stopMotion("Test stopped: browser heartbeat or Wi-Fi lost. Holding position.");
    return;
  }
  if (millis() - lastMotionStepAt < (testingMotor ? testStepMs : jogStepMs)) return;
  lastMotionStepAt = millis();
  int &angle = currentAngles[movingMotor];
  if (angle != targetAngle) {
    angle += angle < targetAngle ? 1 : -1;
    motors[movingMotor].write(angle);
  }
  if (millis() - lastProgressAt >= 1000) {
    lastProgressAt = millis();
    Serial.print("[Motion] Motor ");
    Serial.print(movingMotor);
    Serial.print(": commanded ");
    Serial.print(angle);
    Serial.print(" degrees; target ");
    Serial.println(targetAngle);
  }
  if (angle != targetAngle) return;
  if (testingMotor && testStage < 2) {
    testStage++;
    targetAngle = testStage == 1 ? 180 : testReturnAngle;
  } else {
    stopMotion("Command complete (commanded position; no position feedback).");
  }
}

String stateJson() {
  String body;
  body.reserve(900);
  body += "{\"armed\":";
  body += motorsArmed ? "true" : "false";
  body += ",\"movingMotor\":";
  body += String(movingMotor);
  body += ",\"testing\":";
  body += testingMotor ? "true" : "false";
  body += ",\"targetAngle\":";
  body += String(targetAngle);
  body += ",\"motionStatus\":\"";
  body += motionStatus;
  body += '"';
  body += ",\"box\":";
  body += String(selectedTarget + 1);
  body += ",\"angles\":[";
  for (uint8_t motor = 0; motor < motorCount; motor++) {
    if (motor > 0) body += ',';
    body += String(currentAngles[motor]);
  }
  body += "],\"saved\":[";
  for (uint8_t target = 0; target < targetCount; target++) {
    if (target > 0) body += ',';
    body += targetSaved[target] ? "true" : "false";
  }
  body += "],\"poses\":[";
  for (uint8_t target = 0; target < targetCount; target++) {
    if (target > 0) body += ',';
    body += '[';
    for (uint8_t motor = 0; motor < motorCount; motor++) {
      if (motor > 0) body += ',';
      body += String(targetSaved[target] ? targetAngles[target][motor] : -1);
    }
    body += ']';
  }
  body += "]}";
  return body;
}

int queryValue(const char *query, const char *key) {
  if (query == nullptr) return -1;
  size_t keyLength = strlen(key);
  while (*query != '\0') {
    if (strncmp(query, key, keyLength) == 0 && query[keyLength] == '=') {
      const char *start = query + keyLength + 1;
      char *end = nullptr;
      long value = strtol(start, &end, 10);
      if (end == start || (*end != '\0' && *end != '&') || value < 0 || value > 180) return -1;
      return static_cast<int>(value);
    }
    query = strchr(query, '&');
    if (query == nullptr) break;
    query++;
  }
  return -1;
}

void respond(WiFiClient &client, const char *status, const char *type, const char *body) {
  size_t length = strlen(body);
  String header = String("HTTP/1.1 ") + status + "\r\nContent-Type: " + type +
      "\r\nContent-Length: " + String(length) +
      "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
  client.print(header);
  updateMotion();
  size_t sent = 0;
  while (sent < length) {
    size_t written = client.write(reinterpret_cast<const uint8_t *>(body + sent), min(size_t(512), length - sent));
    updateMotion();
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
    updateMotion();
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

void handleRequest(WiFiClient &client) {
  unsigned long headerStart = millis();
  String request;
  if (!readRequestLine(client, request, headerStart)) {
    Serial.println("[HTTP] Request line incomplete or too long.");
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
  if (!headersComplete || !request.startsWith("GET ")) {
    respond(client, "400 Bad Request", "text/plain", "Invalid request");
    return;
  }
  int pathEnd = request.indexOf(' ', 4);
  if (pathEnd < 0) {
    respond(client, "400 Bad Request", "text/plain", "Invalid request");
    return;
  }
  String path = request.substring(4, pathEnd);
  int queryStart = path.indexOf('?');
  String route = queryStart < 0 ? path : path.substring(0, queryStart);
  const char *query = queryStart < 0 ? nullptr : path.c_str() + queryStart + 1;
  Serial.println(path);

  if (route == "/") {
    respond(client, "200 OK", "text/html; charset=utf-8", armPage);
    return;
  }
  const char *error = nullptr;
  if (route == "/api/arm") {
    if (!motorsArmed) {
      for (uint8_t motor = 0; motor < motorCount; motor++) {
        currentAngles[motor] = 90;
        if (!motors[motor].attach(motorPins[motor])) {
          Serial.print("[Arm] Failed to attach motor ");
          Serial.println(motor);
          for (auto &servo : motors) servo.detach();
          error = "Servo attachment failed; support the arm and check Serial";
          break;
        }
        motors[motor].write(currentAngles[motor]);
        delay(150);
      }
      motorsArmed = error == nullptr;
    }
  } else if (route == "/api/stop") {
    stopMotion("Stopped by user. Servos retain holding torque.");
  } else if (route == "/api/select") {
    int box = queryValue(query, "box");
    if (box < 1 || box > targetCount) error = "Invalid target";
    else selectedTarget = box - 1;
  } else if (route == "/api/jog") {
    int motor = queryValue(query, "motor");
    int angle = queryValue(query, "angle");
    if (!motorsArmed) error = "Arm servos first";
    else if (movingMotor >= 0) error = "Motion in progress; stop or wait before another command";
    else if (motor < 0 || motor >= motorCount || angle < 0 || angle > 180) error = "Invalid motor or angle";
    else if (!motors[motor].attached()) error = "Servo is not attached; reset and arm again";
    else {
      Serial.print("[Jog] Motor ");
      Serial.print(motor);
      Serial.print(": ");
      Serial.print(currentAngles[motor]);
      Serial.print(" -> ");
      Serial.println(angle);
      movingMotor = motor;
      targetAngle = angle;
      testingMotor = false;
      motionStatus = "Jog in progress";
      lastMotionStepAt = millis();
      lastProgressAt = millis();
      motors[motor].write(currentAngles[motor]);
    }
  } else if (route == "/api/test") {
    int motor = queryValue(query, "motor");
    if (!motorsArmed) error = "Arm servos first";
    else if (movingMotor >= 0) error = "Motion in progress; stop or wait before testing";
    else if (motor < 0 || motor >= motorCount) error = "Invalid motor";
    else if (!motors[motor].attached()) error = "Servo is not attached; reset and arm again";
    else {
      movingMotor = motor;
      testingMotor = true;
      motionStatus = "Full-range test in progress";
      testStage = 0;
      testReturnAngle = currentAngles[motor];
      targetAngle = 0;
      lastMotionStepAt = millis();
      lastTestHeartbeatAt = millis();
      lastProgressAt = millis();
      Serial.print("[Test] Starting 0 -> 180 -> starting angle for motor ");
      Serial.println(motor);
    }
  } else if (route == "/api/save") {
    if (!motorsArmed) error = "Arm servos first";
    else if (movingMotor >= 0) error = "Wait for motion to finish before saving";
    else {
      for (uint8_t motor = 0; motor < motorCount; motor++) targetAngles[selectedTarget][motor] = currentAngles[motor];
      targetSaved[selectedTarget] = true;
    }
  } else if (route != "/api/state") {
    respond(client, "404 Not Found", "application/json", "{\"error\":\"Not found\"}");
    return;
  } else if (testingMotor) {
    lastTestHeartbeatAt = millis();
  }
  if (error != nullptr) {
    String body = String("{\"error\":\"") + error + "\"}";
    respond(client, "400 Bad Request", "application/json", body.c_str());
  } else {
    String body = stateJson();
    respond(client, "200 OK", "application/json", body.c_str());
  }
}

void startNetwork() {
  networkConnected = WiFi.status() == WL_CONNECTED;
  if (!networkConnected) {
    server.end();
    Serial.println("Connecting to Wi-Fi...");
    networkConnected = wifi_connection::connect();
  }
  if (networkConnected) {
    server.begin();
    Serial.print("Mapping page: http://");
    Serial.print(WiFi.localIP());
    Serial.println('/');
    Serial.println(server ? "HTTP server started." : "HTTP server failed; will retry.");
  }
  lastNetworkAttempt = millis();
  lastNetworkCheckAt = millis();
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);
  Serial.println("Motor mapping program. Servos idle until armed in the browser.");
  Serial.println("Arming commands all six motors to 90 degrees. Tests: 1 degree per 80ms; STOP retains torque.");
  Serial.println("Map cells 1-9 and the separate outside-board pickup point. Export before resetting.");
  startNetwork();
}

void loop() {
  updateMotion();
  if (millis() - lastNetworkCheckAt >= 1000) {
    lastNetworkCheckAt = millis();
    networkConnected = WiFi.status() == WL_CONNECTED;
    updateMotion();
  }
  if ((!networkConnected || !server) && millis() - lastNetworkAttempt >= 15000) startNetwork();
  if (!networkConnected || !server) return;
  WiFiClient client = server.available();
  updateMotion();
  if (client) {
    handleRequest(client);
    client.stop();
    updateMotion();
  }
}