#include <Arduino.h>
#include <IRremote.h>
#include <vehicle.h>
#include <ultrasonic.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include "io_config.h"
#include <WebServer.h>

// IR receiver pin
#define IR_RECEIVE_PIN 4

// Hardware pin definitions
#define TRIG_PIN 13
#define ECHO_PIN 14
#define SERVO_PIN 25
#define LED_PIN 2
#define BUZZER_PIN 33

// Global objects
vehicle myCar;
ultrasonic sensor;
Servo scanServo;

// System state variables
bool ledState = false;
bool buzzerState = false;
int servoPosition = 90;
bool wifiConnectedOnce = false; // track if we've already celebrated connection

// WiFi retry control
unsigned long lastWifiAttemptMs = 0;
const unsigned long wifiRetryIntervalMs = 10000; // 10s between reconnect attempts

// Simple tone helper using blocking pin toggling (works on ESP32 without tone())
void playToneHz(int pin, int freq, int durationMs) {
  if (freq <= 0 || durationMs <= 0) return;
  pinMode(pin, OUTPUT);
  unsigned long periodUs = 1000000UL / (unsigned long)freq; // full wave period
  unsigned long cycles = ((unsigned long)durationMs * 1000UL) / periodUs;
  for (unsigned long i = 0; i < cycles; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(periodUs / 2);
    digitalWrite(pin, LOW);
    delayMicroseconds(periodUs / 2);
  }
}

void playConnectedTune() {
  // Simple ascending chirp indicating success
  playToneHz(BUZZER_PIN, 880, 130);   // A5
  delay(50);
  playToneHz(BUZZER_PIN, 1245, 130);  // D#6
  delay(50);
  playToneHz(BUZZER_PIN, 1760, 200);  // A6
  // ensure buzzer low
  digitalWrite(BUZZER_PIN, LOW);
}

// Forward declarations for robot functions
void moveLeft();
void moveRight(); 
void moveForward();
void moveBackward();
void stopRobot();
void rotateLeft();
void rotateRight();
void servoLeft();
void servoRight();
void servoCenter();
void toggleLED();
void toggleBuzzer();
void readUltrasonic();

// Forward declaration for web handler
void handleCommand();

// ------------ Web Server -------------
WebServer server(WEB_SERVER_PORT);

String formatUptime() {
  unsigned long ms = millis();
  unsigned long sec = ms / 1000UL;
  unsigned long days = sec / 86400UL; sec %= 86400UL;
  unsigned long hrs = sec / 3600UL;  sec %= 3600UL;
  unsigned long mins = sec / 60UL;   sec %= 60UL;
  char buf[64];
  snprintf(buf, sizeof(buf), "%lud %02lu:%02lu:%02lu", days, hrs, mins, sec);
  return String(buf);
}

void handleRoot() {
  String html;
  html.reserve(2048);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>");
  html += F("<title>Acebott Robot Control</title>");
  html += F("<style>");
  html += F("body{font-family:system-ui,Segoe UI,Roboto,Arial;margin:20px;background:#f5f5f5}");
  html += F("code{background:#e0e0e0;padding:2px 4px;border-radius:4px}");
  html += F(".container{max-width:600px;margin:0 auto;background:white;padding:20px;border-radius:8px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}");
  html += F(".remote{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;max-width:300px;margin:20px auto}");
  html += F(".btn{padding:15px;font-size:18px;border:none;border-radius:8px;cursor:pointer;background:#007acc;color:white;transition:background 0.2s}");
  html += F(".btn:hover{background:#005a9e}.btn:active{background:#004080}");
  html += F(".btn-danger{background:#dc3545}.btn-danger:hover{background:#c82333}");
  html += F(".btn-success{background:#28a745}.btn-success:hover{background:#218838}");
  html += F(".info{margin-bottom:20px}ul{list-style:none;padding:0}li{padding:5px 0;border-bottom:1px solid #eee}");
  html += F("</style></head><body>");
  html += F("<div class='container'>");
  html += F("<h2>🤖 Acebott Robot Control</h2>");
  
  // Device info section
  html += F("<div class='info'><h3>Device Status</h3><ul>");
  html += F("<li>Status: <b>"); html += (WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"); html += F("</b></li>");
  html += F("<li>SSID: <code>"); html += (WiFi.status()==WL_CONNECTED ? WiFi.SSID() : String("-")); html += F("</code></li>");
  html += F("<li>IP: <code>"); html += WiFi.localIP().toString(); html += F("</code></li>");
  html += F("<li>MAC: <code>"); html += WiFi.macAddress(); html += F("</code></li>");
  html += F("<li>RSSI: <code>"); html += String(WiFi.status()==WL_CONNECTED ? WiFi.RSSI() : 0); html += F(" dBm</code></li>");
#ifdef HOSTNAME
  html += F("<li>Hostname: <code>"); html += HOSTNAME; html += F("</code></li>");
#endif
  html += F("<li>Uptime: <code>"); html += formatUptime(); html += F("</code></li>");
  html += F("</ul></div>");
  
  // Remote control section
  html += F("<h3>Remote Control</h3>");
  html += F("<div class='remote'>");
  html += F("<div></div><button class='btn' onclick=\"cmd('forward')\">⬆️ Forward</button><div></div>");
  html += F("<button class='btn' onclick=\"cmd('left')\">⬅️ Left</button>");
  html += F("<button class='btn btn-danger' onclick=\"cmd('stop')\">⏹️ Stop</button>");
  html += F("<button class='btn' onclick=\"cmd('right')\">➡️ Right</button>");
  html += F("<div></div><button class='btn' onclick=\"cmd('backward')\">⬇️ Back</button><div></div>");
  html += F("</div>");
  
  // Additional controls
  html += F("<div style='margin:20px 0;text-align:center'>");
  html += F("<button class='btn' onclick=\"cmd('rotleft')\">↶ Rotate L</button> ");
  html += F("<button class='btn' onclick=\"cmd('rotright')\">↷ Rotate R</button>");
  html += F("</div>");
  
  html += F("<div style='margin:20px 0;text-align:center'>");
  html += F("<button class='btn' onclick=\"cmd('servoleft')\">📷⬅️ Servo L</button> ");
  html += F("<button class='btn btn-success' onclick=\"cmd('servocenter')\">📷⚫ Center</button> ");
  html += F("<button class='btn' onclick=\"cmd('servoright')\">📷➡️ Servo R</button>");
  html += F("</div>");
  
  html += F("<div style='margin:20px 0;text-align:center'>");
  html += F("<button class='btn' onclick=\"cmd('led')\">💡 LED</button> ");
  html += F("<button class='btn' onclick=\"cmd('buzzer')\">🔊 Buzzer</button> ");
  html += F("<button class='btn' onclick=\"cmd('distance')\">📏 Distance</button>");
  html += F("</div>");
  
  html += F("<p><a href='/status'>JSON Status</a></p>");
  
  html += F("<script>");
  html += F("function cmd(action){");
  html += F("fetch('/cmd?action='+action).then(r=>r.text()).then(d=>{");
  html += F("if(d.trim())alert('Response: '+d);");
  html += F("}).catch(e=>alert('Error: '+e));}");
  html += F("</script>");
  
  html += F("</div></body></html>");
  server.send(200, "text/html", html);
}

void handleStatusJson() {
  // Return JSON without blocking; use lightweight manual composition to avoid heap spikes
  String json;
  json.reserve(512);
  json += F("{");
  json += F("\"connected\":"); json += (WiFi.status()==WL_CONNECTED ? F("true") : F("false")); json += F(",");
  json += F("\"ssid\":\""); json += (WiFi.status()==WL_CONNECTED ? WiFi.SSID() : String("")); json += F("\",");
  json += F("\"ip\":\""); json += WiFi.localIP().toString(); json += F("\",");
  json += F("\"mac\":\""); json += WiFi.macAddress(); json += F("\",");
  json += F("\"rssi\":"); json += String(WiFi.status()==WL_CONNECTED ? WiFi.RSSI() : 0); json += F(",");
#ifdef HOSTNAME
  json += F("\"hostname\":\""); json += HOSTNAME; json += F("\",");
#endif
  json += F("\"uptime\":\""); json += formatUptime(); json += F("\",");
  json += F("\"freeHeap\":"); json += String(ESP.getFreeHeap()); json += F(",");
  json += F("\"chipModel\":\""); json += String(ESP.getChipModel()); json += F("\",");
  json += F("\"chipRevision\":"); json += String(ESP.getChipRevision());
  json += F("}");
  server.send(200, "application/json", json);
}

void handleNotFound() {
  String msg = F("Not found: ");
  msg += server.uri();
  server.send(404, "text/plain", msg);
}

void startWebServer() {
  server.on("/", handleRoot);
  server.on("/status", handleStatusJson);
  server.on("/cmd", handleCommand);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.print("[Web] Server started on port ");
  Serial.println(WEB_SERVER_PORT);
}


void tryConnectWiFi(unsigned long timeoutMs = 15000) {
  Serial.println("\n[WiFi] Connecting...");
  // Set hostname if provided
#ifdef HOSTNAME
  WiFi.setHostname(HOSTNAME);
#endif

#ifdef STATIC_IP
  IPAddress ip; IPAddress gateway; IPAddress subnet; IPAddress dns;
  if (ip.fromString(STATIC_IP)) {
#ifdef GATEWAY_IP
    gateway.fromString(GATEWAY_IP);
#endif
#ifdef SUBNET_MASK
    subnet.fromString(SUBNET_MASK);
#endif
#ifdef DNS_SERVER
    dns.fromString(DNS_SERVER);
#endif
    if (!WiFi.config(ip, gateway, subnet, dns)) {
      Serial.println("[WiFi] Static IP config failed, falling back to DHCP");
    }
  }
#endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Connected!");
    Serial.print("[WiFi] IP: "); Serial.println(WiFi.localIP());
    Serial.print("[WiFi] RSSI: "); Serial.println(WiFi.RSSI());
    if (!wifiConnectedOnce) {
      playConnectedTune();
      wifiConnectedOnce = true;
    }
  } else {
    Serial.println("[WiFi] Connection timeout");
  }
}

// Function to identify which key was pressed
String getKeyName(uint32_t command) {
  switch (command) {
    case 0x44: return "LEFT ARROW";
    case 0x46: return "UP ARROW";
    case 0x15: return "DOWN ARROW";
    case 0x43: return "RIGHT ARROW";
    case 0x40: return "OK BUTTON";
    case 0x16: return "1";
    case 0x19: return "2";
    case 0xD:  return "3";
    case 0xC:  return "4";
    case 0x18: return "5";
    case 0x5E: return "6";
    case 0x8:  return "7";
    case 0x1C: return "8";
    case 0x5A: return "9";
    case 0x52: return "0";
    case 0x42: return "* (STAR)";
    case 0x4A: return "# (HASH)";
    default:   return "UNKNOWN KEY";
  }
}

// Robot action functions
void moveLeft() { 
  Serial.println("Robot: Move Left");
  myCar.Move(Move_Left, 150);
}

void moveRight() { 
  Serial.println("Robot: Move Right");
  myCar.Move(Move_Right, 150);
}

void moveForward() { 
  Serial.println("Robot: Move Forward");
  myCar.Move(Forward, 150);
}

void moveBackward() { 
  Serial.println("Robot: Move Backward");
  myCar.Move(Backward, 150);
}

void stopRobot() { 
  Serial.println("Robot: Stop");
  myCar.Move(Stop, 0);
}

void rotateLeft() { 
  Serial.println("Robot: Rotate Left");
  myCar.Move(Contrarotate, 120);
}

void rotateRight() { 
  Serial.println("Robot: Rotate Right");
  myCar.Move(Clockwise, 120);
}

void servoLeft() { 
  Serial.println("Robot: Servo Left");
  servoPosition = min(180, servoPosition + 30);
  scanServo.write(servoPosition);
  Serial.print("Servo position: ");
  Serial.println(servoPosition);
  delay(500); // Give servo time to move
}

void servoRight() { 
  Serial.println("Robot: Servo Right");
  servoPosition = max(0, servoPosition - 30);
  scanServo.write(servoPosition);
  Serial.print("Servo position: ");
  Serial.println(servoPosition);
  delay(500); // Give servo time to move
}

void servoCenter() { 
  Serial.println("Robot: Servo Center");
  servoPosition = 90;
  scanServo.write(servoPosition);
  Serial.println("Servo centered at 90 degrees");
  delay(500); // Give servo time to move
}

void toggleLED() { 
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);
  Serial.print("LED: ");
  Serial.println(ledState ? "ON" : "OFF");
}

void toggleBuzzer() { 
  buzzerState = !buzzerState;
  if (buzzerState) {
    // Save current servo position
    int savedPosition = servoPosition;
    
    // Simple high-frequency digital toggle for louder sound - no PWM used
    for (int beep = 0; beep < 2; beep++) {
      // Create 1000Hz tone by toggling pin rapidly
      for (int i = 0; i < 500; i++) { // 500 cycles = ~500ms at 1000Hz
        digitalWrite(BUZZER_PIN, HIGH);
        delayMicroseconds(500); // 500us high
        digitalWrite(BUZZER_PIN, LOW);
        delayMicroseconds(500); // 500us low = 1000Hz
      }
      delay(100); // Pause between beeps
    }
    
    // Ensure buzzer is off
    digitalWrite(BUZZER_PIN, LOW);
    
    // Restore servo to exact position without re-attaching
    scanServo.write(savedPosition);
    servoPosition = savedPosition;
    
    Serial.println("Buzzer: ON (high-freq digital toggle)");
    buzzerState = false; // Reset state after beeping
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Buzzer: OFF");
  }
  
  Serial.print("Servo should remain at position: ");
  Serial.println(servoPosition);
}

void readUltrasonic() { 
  float distance = sensor.Ranging();
  Serial.print("Ultrasonic distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void handleCommand() {
  String action = server.arg("action");
  String response = "";
  
  if (action == "forward") {
    moveForward();
    response = "Moving forward";
  } else if (action == "backward") {
    moveBackward();
    response = "Moving backward";
  } else if (action == "left") {
    moveLeft();
    response = "Moving left";
  } else if (action == "right") {
    moveRight();
    response = "Moving right";
  } else if (action == "stop") {
    stopRobot();
    response = "Stopped";
  } else if (action == "rotleft") {
    rotateLeft();
    response = "Rotating left";
  } else if (action == "rotright") {
    rotateRight();
    response = "Rotating right";
  } else if (action == "servoleft") {
    servoLeft();
    response = "Servo left";
  } else if (action == "servoright") {
    servoRight();
    response = "Servo right";
  } else if (action == "servocenter") {
    servoCenter();
    response = "Servo centered";
  } else if (action == "led") {
    toggleLED();
    response = String("LED ") + (ledState ? "ON" : "OFF");
  } else if (action == "buzzer") {
    toggleBuzzer();
    response = "Buzzer activated";
  } else if (action == "distance") {
    float distance = sensor.Ranging();
    response = String("Distance: ") + String(distance) + " cm";
  } else {
    response = "Unknown command: " + action;
  }
  
  server.send(200, "text/plain", response);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Acebott Robot IR Controller Starting...");
  
  // Initialize vehicle
  myCar.Init();
  Serial.println("Vehicle initialized");
  
  // Initialize ultrasonic sensor
  sensor.Init(TRIG_PIN, ECHO_PIN);
  Serial.println("Ultrasonic sensor initialized");
  
  // Initialize servo
  scanServo.attach(SERVO_PIN);
  delay(100); // Give servo time to attach
  scanServo.write(90); // Center position
  servoPosition = 90; // Update position variable
  delay(1000); // Give servo time to move to center
  Serial.println("Servo initialized at center position (90 degrees)");
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("LED and buzzer pins initialized");
  
  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.println("IR receiver initialized");
  
  Serial.println("=== ROBOT READY FOR IR COMMANDS ===");
  Serial.println("Use your remote to control the robot:");
  Serial.println("Arrows: Move, OK: Stop, 1/2: Rotate, 3/4/5: Servo, 6: LED, 7: Buzzer, 8: Distance");
  Serial.println("=====================================");

  // Connect to WiFi using credentials from io_config.h
  tryConnectWiFi();
  lastWifiAttemptMs = millis();

  // Start web server
  startWebServer();
}

void loop() {
  // Maintain WiFi connection without blocking core robot features
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - lastWifiAttemptMs >= wifiRetryIntervalMs) {
      lastWifiAttemptMs = now;
      tryConnectWiFi(7000); // brief retry
    }
  }

  // Service HTTP requests
  server.handleClient();

  // Check for IR commands
  if (IrReceiver.decode()) {
    uint32_t command = IrReceiver.decodedIRData.command;
    
    // Map IR code to robot actions
    switch (command) {
      case 0x46: moveForward(); break;      // UP ARROW
      case 0x15: moveBackward(); break;     // DOWN ARROW
      case 0x44: moveLeft(); break;         // LEFT ARROW
      case 0x43: moveRight(); break;        // RIGHT ARROW
      case 0x40: stopRobot(); break;        // OK BUTTON
      case 0x16: rotateLeft(); break;       // 1
      case 0x19: rotateRight(); break;      // 2
      case 0xD:  
        Serial.println("Button 3 pressed - Servo Left");
        servoLeft(); 
        break;        // 3
      case 0xC:  
        Serial.println("Button 4 pressed - Servo Right");
        servoRight(); 
        break;       // 4
      case 0x18: 
        Serial.println("Button 5 pressed - Servo Center");
        servoCenter(); 
        break;      // 5
      case 0x5E: toggleLED(); break;        // 6
      case 0x8:  toggleBuzzer(); break;     // 7
      case 0x1C: readUltrasonic(); break;   // 8
      // 9, 0, *, # reserved for future use
      default:
        Serial.print("KEY: ");
        Serial.print(getKeyName(command));
        Serial.print(" | Code: 0x");
        Serial.print(command, HEX);
        Serial.print(" | Raw: 0x");
        Serial.print(IrReceiver.decodedIRData.decodedRawData, HEX);
        Serial.print(" | Protocol: ");
        Serial.println(getProtocolString(IrReceiver.decodedIRData.protocol));
        break;
    }
    IrReceiver.resume(); // Ready for next signal
  }
  
  delay(100); // Small delay for stability
}
