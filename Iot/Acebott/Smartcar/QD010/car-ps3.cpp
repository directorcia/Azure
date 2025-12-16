#include <Arduino.h>       
#include <IRremote.h>      
#include <vehicle.h>       
#include <ultrasonic.h>    
#include <ESP32Servo.h>    
#include <WiFi.h>
#include "io_config.h"     
#include <WebServer.h>
#include <Ps3Controller.h>     
// IR receiver pin
#define IR_RECEIVE_PIN 4   
// Hardware pin definitions
#define TRIG_PIN 13        
#define ECHO_PIN 14        
#define SERVO_PIN 25       
#define LED_PIN 2
#define LED2_PIN 12
#define BUZZER_PIN 33      
// Global objects
vehicle myCar;
ultrasonic sensor;
Servo scanServo;
// System state variables  
bool ledState = false;     
bool led2State = false;
bool buzzerState = false;
int servoPosition = 90;
bool wifiConnectedOnce = false;
int motorSpeed = 255;  // Default full speed
int currentMotorSpeed = 0;  // Track current speed for smooth acceleration
unsigned long lastAccelTime = 0;  // Time tracking for acceleration
// Motor speed compensation (adjust these to make car go straight)
float leftMotorTrim = 0.66;   // Adjusted from 0.77 for 10cm remaining deviation
float rightMotorTrim = 1.0;  // Keep right at full speed
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
void moveWithTrim(int direction, int baseSpeed);
void servoLeft();
void servoRight();
void servoCenter();
void toggleLED();
void toggleLED2();
void toggleBuzzer();
void readUltrasonic();
// Forward declaration for web handler
void handleCommand();
// PS3 Controller callback
void onPs3Notify() {
  // Read left joystick for direction control
  // PS3 joystick returns SIGNED values: -128 to 127, with 0 as center
  int joyX = Ps3.data.analog.stick.lx; // Left stick X: -128 (left) to 127 (right)
  int joyY = Ps3.data.analog.stick.ly; // Left stick Y: -128 (up) to 127 (down)
  
  // Debug joystick values
  Serial.print("JoyX: ");
  Serial.print(joyX);
  Serial.print(" JoyY: ");
  Serial.println(joyY);
  
  // Right joystick Y for speed control - but default to good speed when centered
  int speedStick = Ps3.data.analog.stick.ry; // -128 (up) to 127 (down)
  
  // If right stick is near center (within deadzone), use default speed of 200
  // Otherwise map stick position to speed range
  if(abs(speedStick) < 30) {
    motorSpeed = 200;  // Default good speed when stick is centered
  } else {
    motorSpeed = map(-speedStick, -128, 127, 150, 255);
    motorSpeed = constrain(motorSpeed, 150, 255);
  }
  
  Serial.print("SpeedStick: ");
  Serial.print(speedStick);
  Serial.print(" MotorSpeed: ");
  Serial.println(motorSpeed);
  
  // Joystick deadzone
  int deadzone = 20;
  
  // Check if L1 or R1 is held for turning
  bool turningLeft = Ps3.data.button.l1;
  bool turningRight = Ps3.data.button.r1;
  
  // Check joystick input
  bool movingForward = (joyY < -deadzone);
  bool movingBackward = (joyY > deadzone);
  bool movingLeft = (joyX < -deadzone);
  bool movingRight = (joyX > deadzone);
  
  // Process movement with turning
  if(abs(joyY) > deadzone && (turningLeft || turningRight)) {
    // Moving forward/backward while turning - all wheels move in joystick direction with differential speed (arcing turn)
    if(movingForward) {
      if(turningLeft) {
        // Turn left while moving forward: ALL wheels move FORWARD, left slower
        int leftSpeed = (int)(motorSpeed * 0.7 * leftMotorTrim);   // Left at 70% forward
        int rightSpeed = (int)(motorSpeed * rightMotorTrim);       // Right at 100% forward
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);
        myCar.MoveWithTrim(Forward, rightSpeed, leftSpeed);
      } else {
        // Turn right while moving forward: ALL wheels move FORWARD, right slower
        int leftSpeed = (int)(motorSpeed * leftMotorTrim);         // Left at 100% forward
        int rightSpeed = (int)(motorSpeed * 0.7 * rightMotorTrim); // Right at 70% forward
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);
        myCar.MoveWithTrim(Forward, rightSpeed, leftSpeed);
      }
    } else if(movingBackward) {
      if(turningLeft) {
        // Turn left while reversing: ALL wheels move BACKWARD, left slower
        int leftSpeed = (int)(motorSpeed * 0.7 * leftMotorTrim);
        int rightSpeed = (int)(motorSpeed * rightMotorTrim);
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);
        myCar.MoveWithTrim(Backward, rightSpeed, leftSpeed);
      } else {
        // Turn right while reversing: ALL wheels move BACKWARD, right slower
        int leftSpeed = (int)(motorSpeed * leftMotorTrim);
        int rightSpeed = (int)(motorSpeed * 0.7 * rightMotorTrim);
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);
        myCar.MoveWithTrim(Backward, rightSpeed, leftSpeed);
      }
    }
  }
  // L1/R1 pressed alone (no joystick forward/back) - rotate in place
  else if(turningLeft) {
    rotateLeft();  // Spin in place
  } else if(turningRight) {
    rotateRight();  // Spin in place
  }
  // Joystick without L1/R1 - normal movement
  else if(abs(joyY) > deadzone || abs(joyX) > deadzone) {
    if(abs(joyY) > abs(joyX)) {
      if(movingForward) {
        moveForward();
      } else if(movingBackward) {
        moveBackward();
      }
    } else {
      if(movingLeft) {
        moveLeft();
      } else if(movingRight) {
        moveRight();
      }
    }
  } else {
    // Nothing pressed - stop
    stopRobot();
  }
  
  if(Ps3.event.button_down.triangle) servoLeft();
  if(Ps3.event.button_down.circle) servoCenter();
  if(Ps3.event.button_down.cross) servoRight();
  if(Ps3.event.button_down.square) toggleBuzzer();
  
  // L2 trigger - toggle LED1 (pin 2)
  if(Ps3.event.button_down.l2) toggleLED();
  
  // Select button - toggle LED2 (pin 12)
  if(Ps3.event.button_down.select) toggleLED2();
}
void onPs3Connect() {
  Serial.println("PS3 Controller Connected!");
  Ps3.setPlayer(1);  // Set player LED to stop flashing
  playConnectedTune();
}
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
// Custom movement with motor speed compensation
void moveWithTrim(int direction, int baseSpeed) {
  // Gradual acceleration: ramp from current to target speed over time
  int targetSpeed = baseSpeed;
  unsigned long currentTime = millis();
  
  // If starting from stop, jump to a minimum working speed immediately
  if(currentMotorSpeed < 100) {
    currentMotorSpeed = 180;  // Start high enough that even with trim (0.77) wheels turn
  }
  
  // Only update speed every 30ms for smooth acceleration
  if(currentTime - lastAccelTime >= 30) {
    lastAccelTime = currentTime;
    int speedStep = 15;  // How much to increase per 30ms
    
    if(currentMotorSpeed < targetSpeed) {
      currentMotorSpeed += speedStep;
      if(currentMotorSpeed > targetSpeed) currentMotorSpeed = targetSpeed;
    } else if(currentMotorSpeed > targetSpeed) {
      currentMotorSpeed -= speedStep;
      if(currentMotorSpeed < targetSpeed) currentMotorSpeed = targetSpeed;
    }
  }
  
  // Apply trim to current speed
  int leftSpeed = (int)(currentMotorSpeed * leftMotorTrim);
  int rightSpeed = (int)(currentMotorSpeed * rightMotorTrim);
  
  // CRITICAL: Constrain to 0-255 range for PWM
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Debug output
  Serial.print("moveWithTrim: dir=");
  Serial.print(direction);
  Serial.print(" target=");
  Serial.print(targetSpeed);
  Serial.print(" current=");
  Serial.print(currentMotorSpeed);
  Serial.print(" L=");
  Serial.print(leftSpeed);
  Serial.print(" R=");
  Serial.println(rightSpeed);
  
  // PWM1 is RIGHT side, PWM2 is LEFT side
  myCar.MoveWithTrim(direction, rightSpeed, leftSpeed);
}
void moveLeft() {
  Serial.print("Robot: Move Left (Speed: ");
  Serial.print(motorSpeed);
  Serial.println(")");
  moveWithTrim(Move_Left, motorSpeed);
}
void moveRight() {
  Serial.print("Robot: Move Right (Speed: ");
  Serial.print(motorSpeed);
  Serial.println(")");
  moveWithTrim(Move_Right, motorSpeed);
}
void moveForward() {
  Serial.print("Robot: Move Forward (Speed: ");
  Serial.print(motorSpeed);
  Serial.println(")");
  moveWithTrim(Forward, motorSpeed);
}
void moveBackward() {
  Serial.print("Robot: Move Backward (Speed: ");
  Serial.print(motorSpeed);
  Serial.println(")");
  moveWithTrim(Backward, motorSpeed);
}
void stopRobot() {
  Serial.println("Robot: Stop");
  currentMotorSpeed = 0;  // Reset for next acceleration
  myCar.Move(Stop, 0);
}
void rotateLeft() {
  Serial.print("Robot: Rotate Left (Speed: ");
  Serial.print(motorSpeed);
  Serial.println(")");
  myCar.Move(Contrarotate, motorSpeed);
}
void rotateRight() {
  Serial.print("Robot: Rotate Right (Speed: ");
  Serial.print(motorSpeed);
  Serial.println(")");
  myCar.Move(Clockwise, motorSpeed);
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
  Serial.print("LED1: ");
  Serial.println(ledState ? "ON" : "OFF");
}
void toggleLED2() {
  led2State = !led2State;
  digitalWrite(LED2_PIN, led2State);
  Serial.print("LED2: ");
  Serial.println(led2State ? "ON" : "OFF");
}
void toggleBuzzer() {
  buzzerState = !buzzerState;
  if (buzzerState) {
    // Play sharp beep tones using the existing playToneHz function
    playToneHz(BUZZER_PIN, 2000, 100);  // 2000Hz beep for 100ms
    delay(50);
    playToneHz(BUZZER_PIN, 2000, 100);  // Second beep
    
    // Ensure buzzer is off
    digitalWrite(BUZZER_PIN, LOW);
    
    Serial.println("Buzzer: BEEP");
    buzzerState = false; // Reset state after beeping
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Buzzer: OFF");
  }
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
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("LED and buzzer pins initialized");
  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.println("IR receiver initialized");
  // Initialize PS3 Controller - connects directly to ESP32
  Ps3.attach(onPs3Notify);
  Ps3.attachOnConnect(onPs3Connect);
  Ps3.begin("20:00:00:00:82:62");
  Serial.println("PS3 Controller ready - Press PS button to connect");
  Serial.println("=== ROBOT READY ===");
  Serial.println("IR Remote & PS3 Controller supported");
  Serial.println("====================");
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
  // PS3 handled by callback - no code needed here
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