
#include <Arduino.h>
#include <Servo.h>
#include <WiFiS3.h>
#include <limits.h>
#include <string.h>
#include "arm_page.h"
#include "wifi_connection.h"

// The controller exposes six servo joints and stores up to nine poses (one for
// each cell in a 3x3 layout). The included headers provide the web page and the
// project-specific Wi-Fi connection helper.
constexpr uint8_t motorCount = 6;
constexpr uint8_t boxCount = 9;

// These arrays use the same motor order throughout the program. Keep the pin,
// angle-limit, home-pose, and current-angle entries aligned by motor index.
const uint8_t motorPins[motorCount] = {0, 1, 2, 3, 4, 5};
const int minimumAngles[motorCount] = {0, 0, 0, 0, 0, 0};
const int maximumAngles[motorCount] = {180, 180, 180, 180, 180, 180};
const int homeAngles[motorCount] = {180, 90, 90, 135, 180, 90};

// Network and motion state. Motors are not attached until the user calls the
// arm endpoint, so starting the controller does not immediately drive servos.
WiFiServer server(80);
Servo motors[motorCount];
int currentAngles[motorCount] = {180, 90, 90, 135, 180, 90};
int boxAngles[boxCount][motorCount] = {};
bool boxSaved[boxCount] = {};
uint8_t selectedBox = 0;
bool motorsArmed = false;
bool serverStarted = false;
unsigned long lastReconnectAttempt = 0;
unsigned long lastServerStartAttempt = 0;
unsigned long lastStatusPrint = 0;

// Move one joint gradually rather than jumping directly to the target. This
// keeps each commanded change to one degree, with a short settling interval.
void moveMotorTo(uint8_t motorIndex, int targetAngle) {
  targetAngle = constrain(targetAngle, minimumAngles[motorIndex], maximumAngles[motorIndex]);
  while (currentAngles[motorIndex] != targetAngle) {
    currentAngles[motorIndex] += currentAngles[motorIndex] < targetAngle ? 1 : -1;
    motors[motorIndex].write(currentAngles[motorIndex]);
    delay(15);
  }
}

// Attach every servo only on the first explicit arm request. Write the known
// starting angles immediately so the controller's state matches the commands.
void armMotors() {
  if (motorsArmed) return;
  for (uint8_t motor = 0; motor < motorCount; motor++) {
    motors[motor].attach(motorPins[motor]);
    motors[motor].write(currentAngles[motor]);
    delay(150);
  }
  motorsArmed = true;
}

// Match a complete endpoint path, optionally followed by a query string.
bool pathMatches(const char *path, const char *endpoint) {
  size_t endpointLength = strlen(endpoint);
  return strncmp(path, endpoint, endpointLength) == 0 &&
         (path[endpointLength] == '\0' || path[endpointLength] == '?');
}

// Parse a required, non-negative decimal query parameter without accepting
// trailing characters or overflowing the result.
bool queryValue(const char *path, const char *key, int &result) {
  const char *query = strchr(path, '?');
  if (query == nullptr) return false;
  size_t keyLength = strlen(key);
  const char *parameter = query + 1;
  while (*parameter != '\0') {
    const char *parameterEnd = strchr(parameter, '&');
    if (parameterEnd == nullptr) parameterEnd = parameter + strlen(parameter);
    const char *equals = strchr(parameter, '=');
    if (equals != nullptr && equals < parameterEnd &&
        static_cast<size_t>(equals - parameter) == keyLength &&
        strncmp(parameter, key, keyLength) == 0) {
      if (equals + 1 == parameterEnd) return false;
      int value = 0;
      for (const char *digit = equals + 1; digit < parameterEnd; digit++) {
        if (*digit < '0' || *digit > '9') return false;
        int nextDigit = *digit - '0';
        if (value > (INT_MAX - nextDigit) / 10) return false;
        value = value * 10 + nextDigit;
      }
      result = value;
      return true;
    }
    if (*parameterEnd == '\0') break;
    parameter = parameterEnd + 1;
  }
  return false;
}

// Send a complete HTTP response with explicit content length and close the
// connection after this request. The caller supplies the status, MIME type,
// body pointer, and body size.
void sendResponse(WiFiClient &client, const char *status, const char *contentType,
                  const char *body, size_t bodyLength) {
  client.print("HTTP/1.1 ");
  client.print(status);
  client.print("\r\nContent-Type: ");
  client.print(contentType);
  client.print("\r\nContent-Length: ");
  client.print(bodyLength);
  client.print("\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n");
  client.write(reinterpret_cast<const uint8_t *>(body), bodyLength);
}

// Convenience wrapper for JSON responses built as Arduino String values.
void sendJson(WiFiClient &client, const String &body) {
  sendResponse(client, "200 OK", "application/json", body.c_str(), body.length());
}

// Start listening only when connected to Wi-Fi. The timestamp throttles retry
// attempts in loop(), and the flag prevents repeated server.begin() calls.
void startHttpServer() {
  if (serverStarted || WiFi.status() != WL_CONNECTED) {
    return;
  }
  server.begin();
  serverStarted = static_cast<bool>(server);
  lastServerStartAttempt = millis();
  Serial.println(serverStarted ? "HTTP server listening on port 80." : "HTTP server failed to start; retrying.");
}

// Serialize the controller's current state for the browser UI. Unsaved poses
// are represented by -1 in the poses array; saved[] indicates which entries
// contain valid angles. The UI uses one-based box numbering.
String makeStateJson() {
  String body;
  body.reserve(900);
  body += "{\"armed\":";
  body += motorsArmed ? "true" : "false";
  body += ",\"box\":";
  body += String(selectedBox + 1);
  body += ",\"angles\":[";
  for (uint8_t motor = 0; motor < motorCount; motor++) {
    if (motor > 0) body += ',';
    body += String(currentAngles[motor]);
  }
  body += "],\"saved\":[";
  for (uint8_t box = 0; box < boxCount; box++) {
    if (box > 0) body += ',';
    body += boxSaved[box] ? "true" : "false";
  }
  body += "],\"poses\":[";
  for (uint8_t box = 0; box < boxCount; box++) {
    if (box > 0) body += ',';
    body += '[';
    for (uint8_t motor = 0; motor < motorCount; motor++) {
      if (motor > 0) body += ',';
      body += String(boxSaved[box] ? boxAngles[box][motor] : -1);
    }
    body += ']';
  }
  body += "]}";
  return body;
}

// Read one HTTP request line, extract its path, then dispatch to the matching
// page or API endpoint. Every request is handled synchronously and the caller
// closes its client after this function returns.
void handleRequest(WiFiClient &client) {
  client.setTimeout(1500);
  char requestLine[128];
  size_t length = client.readBytesUntil('\n', requestLine, sizeof(requestLine) - 1);
  requestLine[length] = '\0';
  if (length == 0) return;
  if (length == sizeof(requestLine) - 1) {
    const char body[] = "Request line too long";
    sendResponse(client, "414 URI Too Long", "text/plain", body, sizeof(body) - 1);
    return;
  }
  if (length > 0 && requestLine[length - 1] == '\r') {
    requestLine[--length] = '\0';
  }
  char *methodEnd = strchr(requestLine, ' ');
  if (methodEnd == nullptr) {
    const char body[] = "Bad request";
    sendResponse(client, "400 Bad Request", "text/plain", body, sizeof(body) - 1);
    return;
  }
  *methodEnd = '\0';
  if (strcmp(requestLine, "GET") != 0) {
    const char body[] = "Method not allowed";
    sendResponse(client, "405 Method Not Allowed", "text/plain", body, sizeof(body) - 1);
    return;
  }
  char *path = methodEnd + 1;
  char *pathEnd = strchr(path, ' ');
  if (pathEnd == nullptr) {
    const char body[] = "Bad request";
    sendResponse(client, "400 Bad Request", "text/plain", body, sizeof(body) - 1);
    return;
  }
  *pathEnd = '\0';
  if (strcmp(pathEnd + 1, "HTTP/1.1") != 0 && strcmp(pathEnd + 1, "HTTP/1.0") != 0) {
    const char body[] = "Bad request";
    sendResponse(client, "400 Bad Request", "text/plain", body, sizeof(body) - 1);
    return;
  }
  Serial.print("HTTP request: ");
  Serial.println(path);

  // Serve the embedded control page and a snapshot of controller state.
  if (strcmp(path, "/") == 0) {
    sendResponse(client, "200 OK", "text/html; charset=utf-8", armPage, sizeof(armPage) - 1);
  } else if (pathMatches(path, "/api/state")) {
    sendJson(client, makeStateJson());

  // Attach servos on demand, then return the updated state.
  } else if (pathMatches(path, "/api/arm")) {
    armMotors();
    sendJson(client, makeStateJson());

  // Select a pose slot. The URL uses one-based box numbers, while the arrays
  // use zero-based indexes internally.
  } else if (pathMatches(path, "/api/select")) {
    int boxNumber;
    if (!queryValue(path, "box", boxNumber)) {
      const char body[] = "{\"error\":\"box must be an integer from 1 to 9\"}";
      sendResponse(client, "400 Bad Request", "application/json", body, sizeof(body) - 1);
      return;
    }
    if (boxNumber < 1 || boxNumber > boxCount) {
      const char body[] = "{\"error\":\"box must be 1-9\"}";
      sendResponse(client, "400 Bad Request", "application/json", body, sizeof(body) - 1);
      return;
    }
    selectedBox = boxNumber - 1;
    sendJson(client, makeStateJson());

  // Jog one joint to a requested angle. Reject motion until armed, and validate
  // both the motor index and angle before using them to index the arrays.
  } else if (pathMatches(path, "/api/jog")) {
    int motor;
    int angle;
    if (!queryValue(path, "motor", motor) || !queryValue(path, "angle", angle)) {
      const char body[] = "{\"error\":\"motor and angle must be integers\"}";
      sendResponse(client, "400 Bad Request", "application/json", body, sizeof(body) - 1);
      return;
    }
    if (!motorsArmed) {
      const char body[] = "{\"error\":\"arm first\"}";
      sendResponse(client, "403 Forbidden", "application/json", body, sizeof(body) - 1);
      return;
    }
    if (motor < 0 || motor >= motorCount || angle < minimumAngles[motor] || angle > maximumAngles[motor]) {
      const char body[] = "{\"error\":\"invalid motor or angle\"}";
      sendResponse(client, "400 Bad Request", "application/json", body, sizeof(body) - 1);
      return;
    }
    moveMotorTo(motor, angle);
    sendJson(client, makeStateJson());

  // Save the current six joint angles into the selected pose slot.
  } else if (pathMatches(path, "/api/save")) {
    if (!motorsArmed) {
      const char body[] = "{\"error\":\"arm first\"}";
      sendResponse(client, "403 Forbidden", "application/json", body, sizeof(body) - 1);
      return;
    }
    for (uint8_t motor = 0; motor < motorCount; motor++) {
      boxAngles[selectedBox][motor] = currentAngles[motor];
    }
    boxSaved[selectedBox] = true;
    sendJson(client, makeStateJson());
  } else {
    const char body[] = "Not found";
    sendResponse(client, "404 Not Found", "text/plain", body, sizeof(body) - 1);
  }
}

// Ask the project Wi-Fi helper to connect. On success, report the device's
// local address and attempt to start the HTTP server; in either case, record
// the attempt time so loop() can apply the reconnect interval.
void connectToNetwork() {
  Serial.println("Connecting to Wi-Fi...");
  if (wifi_connection::connect()) {
    Serial.print("Wi-Fi connected. Open http://");
    Serial.print(WiFi.localIP());
    Serial.println("/ on a device on the same network.");
    startHttpServer();
  } else {
    Serial.println("Wi-Fi connection failed. Check the 2.4 GHz credentials.");
  }
  lastReconnectAttempt = millis();
}

// Initialize serial diagnostics and make an initial network connection. The
// servo outputs remain detached until the user explicitly arms the motors.
void setup() {
  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000) delay(10);
  Serial.println("3x3 arm pose controller starting. Motors stay idle until Arm is pressed.");
  connectToNetwork();
}

// Maintain Wi-Fi/server availability without blocking on repeated connection
// attempts. Periodically print a small status line and service one HTTP client
// whenever the server has a pending connection.
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    serverStarted = false;
    if (millis() - lastReconnectAttempt >= 15000) {
      connectToNetwork();
    }
  }
  if (WiFi.status() == WL_CONNECTED && !serverStarted && millis() - lastServerStartAttempt >= 5000) {
    startHttpServer();
  }
  if (millis() - lastStatusPrint >= 10000) {
    Serial.print("Wi-Fi status: ");
    Serial.print(WiFi.status());
    Serial.print("; HTTP server: ");
    Serial.println(serverStarted ? "running" : "not running");
    lastStatusPrint = millis();
  }
  WiFiClient client = server.available();
  if (client) {
    handleRequest(client);
    client.stop();
  }
}


