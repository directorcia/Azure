// ========================================================================
// ACEBOTT QD010 Smart Car - PS3 Controller with Camera Integration
// ========================================================================
// This code controls a 4-wheel robot car using:
// - PS3 Controller for wireless control (primary interface)
// - ESP32-CAM module via UART for live video streaming
// - Ultrasonic sensor for distance measurement
// - Two servo motors (ultrasonic scanner + camera tilt)
// - WiFi connectivity for camera access

#include <Arduino.h>        // Arduino core functions
#include <vehicle.h>        // Custom vehicle motor control library
#include <ultrasonic.h>     // Ultrasonic distance sensor library
#include <ESP32Servo.h>     // Servo motor control for ESP32
#include <WiFi.h>           // WiFi connectivity
#include "io_config.h"      // WiFi credentials (WIFI_SSID, WIFI_PASSWORD, etc.)
#include <Ps3Controller.h>  // PS3 controller Bluetooth library

// ========================================================================
// Hardware Pin Definitions
// ========================================================================
// Ultrasonic sensor pins (HC-SR04)
#define TRIG_PIN 13        // Ultrasonic trigger pin
#define ECHO_PIN 14        // Ultrasonic echo pin

// Servo motor pins
#define SERVO_PIN 25       // Ultrasonic scanner servo (head movement)
#define CAMERA_PAN_TILT_PIN 27  // Camera pan/tilt servo

// LED and buzzer pins
#define LED_PIN 2          // Built-in LED indicator
#define LED2_PIN 12        // Secondary LED
#define BUZZER_PIN 33      // Buzzer for audio feedback

// ========================================================================
// Camera UART Communication Configuration
// ========================================================================
// ESP32 communicates with ESP32-CAM module via UART (Serial1)
// This allows sending AT commands to control camera settings
#define CAMERA_UART_PORT UART_NUM_1  // Use UART1 (Serial1) for camera
#define CAMERA_UART_BAUD 115200      // Baud rate for camera communication
#define CAMERA_UART_RX 9   // GPIO9 - Receive data from camera
#define CAMERA_UART_TX 10  // GPIO10 - Transmit commands to camera

// ========================================================================
// Global Hardware Objects
// ========================================================================
vehicle myCar;            // Vehicle motor controller (manages 4 DC motors)
ultrasonic sensor;        // Ultrasonic distance sensor (HC-SR04)
Servo scanServo;          // Servo for ultrasonic sensor scanning
Servo cameraPanTilt;      // Servo for camera tilt control (pin 27)

// ========================================================================
// System State Variables
// ========================================================================
// LED and buzzer states
bool ledState = false;     // Current state of LED1 (pin 2)
bool led2State = false;    // Current state of LED2 (pin 12)
bool buzzerState = false;  // Current state of buzzer

// Servo position tracking
int servoPosition = 90;           // Current position of ultrasonic scanner servo (0-180)
int cameraPanTiltPosition = 90;   // Current position of camera tilt servo (0-180)

// WiFi connection tracking
bool wifiConnectedOnce = false;   // Flag to play connection tune only once

// Motor speed control
int motorSpeed = 255;             // Target motor speed (150-255 range)
int currentMotorSpeed = 0;        // Current speed for smooth acceleration
unsigned long lastAccelTime = 0;  // Timestamp for acceleration ramping

// ========================================================================
// Motor Trim Calibration
// ========================================================================
// These values compensate for motor and wheel differences to make car go straight
// Adjust if robot veers left or right during forward movement
float leftMotorTrim = 0.66;   // Left motor power multiplier (66% of base speed)
float rightMotorTrim = 1.0;   // Right motor power multiplier (100% of base speed)

// ========================================================================
// WiFi Reconnection Control
// ========================================================================
unsigned long lastWifiAttemptMs = 0;           // Last WiFi connection attempt time
const unsigned long wifiRetryIntervalMs = 10000; // Wait 10 seconds between retry attempts

// ========================================================================
// Audio Feedback Functions
// ========================================================================

/**
 * Generates a tone by toggling a pin at specified frequency
 * Note: ESP32 doesn't have Arduino's tone() function, so we use direct pin toggling
 * This is a blocking function - code waits until tone completes
 * 
 * @param pin - GPIO pin connected to buzzer
 * @param freq - Frequency in Hertz (e.g., 440 = A4 note)
 * @param durationMs - How long to play tone in milliseconds
 */
void playToneHz(int pin, int freq, int durationMs) {
  // Validate inputs
  if (freq <= 0 || durationMs <= 0) return;
  
  pinMode(pin, OUTPUT);
  
  // Calculate period: time for one complete wave cycle
  unsigned long periodUs = 1000000UL / (unsigned long)freq; // Period in microseconds
  
  // Calculate how many cycles needed for desired duration
  unsigned long cycles = ((unsigned long)durationMs * 1000UL) / periodUs;
  
  // Generate square wave by toggling pin
  for (unsigned long i = 0; i < cycles; i++) {
    digitalWrite(pin, HIGH);              // Half cycle high
    delayMicroseconds(periodUs / 2);
    digitalWrite(pin, LOW);               // Half cycle low
    delayMicroseconds(periodUs / 2);
  }
}

/**
 * Plays a pleasant ascending tune to indicate successful connection
 * Used when PS3 controller or WiFi connects
 */
void playConnectedTune() {
  // Three-note ascending chirp indicating success
  playToneHz(BUZZER_PIN, 880, 130);   // A5 note (880 Hz)
  delay(50);                           // Short pause between notes
  playToneHz(BUZZER_PIN, 1245, 130);  // D#6 note (1245 Hz)
  delay(50);
  playToneHz(BUZZER_PIN, 1760, 200);  // A6 note (1760 Hz) - held longer
  
  // Ensure buzzer pin is low (silent) after tune
  digitalWrite(BUZZER_PIN, LOW);
}

// ========================================================================
// Forward Declarations
// ========================================================================
// Movement functions
void moveLeft();
void moveRight();
void moveForward();
void moveBackward();
void stopRobot();
void rotateLeft();
void rotateRight();
void moveWithTrim(int direction, int baseSpeed);

// Servo control functions
void servoLeft();
void servoRight();
void servoCenter();

// Accessory functions
void toggleLED();
void toggleLED2();
void toggleBuzzer();
void readUltrasonic();

// Camera control functions
void flipCameraImage();
void resetCameraImage();

// Web handler (unused - kept for future)
void handleCommand();

// ========================================================================
// PS3 Controller Event Handler
// ========================================================================

/**
 * Called whenever PS3 controller state changes (buttons, joysticks)
 * This is the main control interface for the robot
 * 
 * Controller mapping:
 * - Left joystick: Direction control (X = left/right, Y = forward/back)
 * - Right joystick Y: Speed control (up = faster, center = default 200)
 * - L1/R1: Turn left/right while moving (arc turn) or rotate in place
 * - D-pad Left/Right: Move ultrasonic sensor servo
 * - D-pad Up/Down: Tilt camera up/down
 * - Triangle: Flip camera image 180 degrees
 * - X (Cross): Reset camera to normal orientation
 * - Circle: Center ultrasonic servo
 * - Square: Sound buzzer
 * - L2: Toggle LED
 * - Select: Center ultrasonic servo
 */
void onPs3Notify() {
  // ========================================================================
  // Read Joystick Values
  // ========================================================================
  // PS3 joystick values are SIGNED: -128 to +127, with 0 as center/neutral
  int joyX = Ps3.data.analog.stick.lx; // Left stick horizontal: -128 (left) to 127 (right)
  int joyY = Ps3.data.analog.stick.ly; // Left stick vertical: -128 (up/forward) to 127 (down/back)
  
  // Debug output to monitor joystick values
  Serial.print("JoyX: ");
  Serial.print(joyX);
  Serial.print(" JoyY: ");
  Serial.println(joyY);
  
  // ========================================================================
  // Speed Control via Right Joystick
  // ========================================================================
  int speedStick = Ps3.data.analog.stick.ry; // Right stick vertical: -128 (up) to 127 (down)
  
  // Use default speed (200) when stick is centered for consistent performance
  // Only adjust speed when user actively moves right stick
  if(abs(speedStick) < 30) {
    motorSpeed = 200;  // Default optimal speed when stick is near center
  } else {
    // Map stick position to speed range: up = faster, down = slower
    // Negative speedStick (up) maps to higher speed values
    motorSpeed = map(-speedStick, -128, 127, 150, 255);
    motorSpeed = constrain(motorSpeed, 150, 255);  // Ensure valid PWM range
  }
  
  // Debug output for speed control
  Serial.print("SpeedStick: ");
  Serial.print(speedStick);
  Serial.print(" MotorSpeed: ");
  Serial.println(motorSpeed);
  
  // ========================================================================
  // Movement Control Logic
  // ========================================================================
  // Deadzone: ignore small joystick movements to prevent drift
  int deadzone = 20;
  
  // Check shoulder buttons for turning control
  bool turningLeft = Ps3.data.button.l1;   // L1 button for left turn
  bool turningRight = Ps3.data.button.r1;  // R1 button for right turn
  
  // Determine joystick direction (beyond deadzone threshold)
  bool movingForward = (joyY < -deadzone);   // Joystick pushed up
  bool movingBackward = (joyY > deadzone);   // Joystick pushed down
  bool movingLeft = (joyX < -deadzone);      // Joystick pushed left
  bool movingRight = (joyX > deadzone);      // Joystick pushed right
  
  // ========================================================================
  // Arc Turning: Joystick + L1/R1 for smooth curved turns
  // ========================================================================
  // When moving forward/back AND L1/R1 pressed: differential speed creates arc
  if(abs(joyY) > deadzone && (turningLeft || turningRight)) {
    // Arc turning: all wheels move in same direction but at different speeds
    // This creates smooth curved paths instead of sharp point turns
    
    if(movingForward) {
      // === FORWARD ARC TURNS ===
      if(turningLeft) {
        // Arc left: slow down left wheels, right wheels full speed
        // This makes the car curve to the left while moving forward
        int leftSpeed = (int)(motorSpeed * 0.7 * leftMotorTrim);   // Left at 70% speed
        int rightSpeed = (int)(motorSpeed * rightMotorTrim);       // Right at 100% speed
        leftSpeed = constrain(leftSpeed, 0, 255);    // Ensure valid PWM
        rightSpeed = constrain(rightSpeed, 0, 255);
        myCar.MoveWithTrim(Forward, rightSpeed, leftSpeed);
      } else {
        // Arc right: slow down right wheels, left wheels full speed
        // This makes the car curve to the right while moving forward
        int leftSpeed = (int)(motorSpeed * leftMotorTrim);         // Left at 100% speed
        int rightSpeed = (int)(motorSpeed * 0.7 * rightMotorTrim); // Right at 70% speed
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);
        myCar.MoveWithTrim(Forward, rightSpeed, leftSpeed);
      }
    } else if(movingBackward) {
      // === BACKWARD ARC TURNS ===
      if(turningLeft) {
        // Arc left while reversing: left wheels slower than right
        int leftSpeed = (int)(motorSpeed * 0.7 * leftMotorTrim);
        int rightSpeed = (int)(motorSpeed * rightMotorTrim);
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);
        myCar.MoveWithTrim(Backward, rightSpeed, leftSpeed);
      } else {
        // Arc right while reversing: right wheels slower than left
        int leftSpeed = (int)(motorSpeed * leftMotorTrim);
        int rightSpeed = (int)(motorSpeed * 0.7 * rightMotorTrim);
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);
        myCar.MoveWithTrim(Backward, rightSpeed, leftSpeed);
      }
    }
  }
  // ========================================================================
  // Point Rotation: L1/R1 alone (no joystick) - spin in place
  // ========================================================================
  else if(turningLeft) {
    rotateLeft();  // Left wheels backward, right wheels forward (counter-clockwise)
  } else if(turningRight) {
    rotateRight();  // Left wheels forward, right wheels backward (clockwise)
  }
  // ========================================================================
  // Normal Movement: Joystick only (no L1/R1)
  // ========================================================================
  else if(abs(joyY) > deadzone || abs(joyX) > deadzone) {
    // Prioritize the axis with larger deflection
    if(abs(joyY) > abs(joyX)) {
      // Vertical movement takes priority
      if(movingForward) {
        moveForward();   // All wheels forward at same speed
      } else if(movingBackward) {
        moveBackward();  // All wheels backward at same speed
      }
    } else {
      // Horizontal movement takes priority
      if(movingLeft) {
        moveLeft();      // Left wheels back, right wheels forward (crab left)
      } else if(movingRight) {
        moveRight();     // Left wheels forward, right wheels back (crab right)
      }
    }
  } else {
    // ========================================================================
    // No Input: Stop robot
    // ========================================================================
    stopRobot();  // All motors off
  }
  
  // ========================================================================
  // Button Controls
  // ========================================================================
  // D-pad Left/Right: Control ultrasonic sensor servo (head scanning)
  if(Ps3.event.button_down.left) servoLeft();    // Turn sensor left
  if(Ps3.event.button_down.right) servoRight();  // Turn sensor right
  if(Ps3.event.button_down.circle) servoCenter(); // Center sensor
  
  // Square: Sound buzzer
  if(Ps3.event.button_down.square) toggleBuzzer();
  
  // Triangle: Flip camera image 180 degrees (for inverted mounting)
  if(Ps3.event.button_down.triangle) flipCameraImage();
  
  // X (Cross): Reset camera to normal orientation
  if(Ps3.event.button_down.cross) resetCameraImage();
  
  // D-pad Up/Down: Control camera tilt servo (pin 27)
  if(Ps3.event.button_down.up) {
    // Tilt camera up (decrease angle) in 10-degree increments
    cameraPanTiltPosition = constrain(cameraPanTiltPosition - 10, 0, 180);
    cameraPanTilt.write(cameraPanTiltPosition);
    Serial.print("Camera tilt UP: ");
    Serial.println(cameraPanTiltPosition);
  }
  if(Ps3.event.button_down.down) {
    // Tilt camera down (increase angle) in 10-degree increments
    cameraPanTiltPosition = constrain(cameraPanTiltPosition + 10, 0, 180);
    cameraPanTilt.write(cameraPanTiltPosition);
    Serial.print("Camera tilt DOWN: ");
    Serial.println(cameraPanTiltPosition);
  }
  
  // L2 Trigger: Toggle LED1 on/off
  if(Ps3.event.button_down.l2) toggleLED();
  
  // Select Button: Return ultrasonic servo to center position
  if(Ps3.event.button_down.select) servoCenter();
}

/**
 * Called when PS3 controller successfully connects via Bluetooth
 * Provides audio and visual feedback to confirm connection
 */
void onPs3Connect() {
  Serial.println("PS3 Controller Connected!");
  Ps3.setPlayer(1);  // Set player number (stops controller LEDs from flashing)
  playConnectedTune();  // Play success melody
}

// ========================================================================
// Utility Functions
// ========================================================================

/**
 * Formats system uptime into human-readable string
 * @return Uptime as "days hours:minutes:seconds"
 */
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

/**
 * Attempts to connect to WiFi network using credentials from io_config.h
 * Supports both DHCP and static IP configuration
 * WiFi is used for accessing the ESP32-CAM video stream
 * 
 * @param timeoutMs - Maximum time to wait for connection (default 15 seconds)
 */
void tryConnectWiFi(unsigned long timeoutMs = 15000) {
  Serial.println("\n[WiFi] Connecting...");
  
  // Set custom hostname if defined in io_config.h
#ifdef HOSTNAME
  WiFi.setHostname(HOSTNAME);
#endif

  // Configure static IP if defined, otherwise use DHCP
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
    // Attempt static IP configuration
    if (!WiFi.config(ip, gateway, subnet, dns)) {
      Serial.println("[WiFi] Static IP config failed, falling back to DHCP");
    }
  }
#endif

  // Set to Station mode (connect to existing WiFi network)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Wait for connection with timeout
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(250);
    Serial.print('.');  // Progress indicator
  }
  Serial.println();
  
  // Check connection result
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Connected!");
    Serial.print("[WiFi] IP: "); Serial.println(WiFi.localIP());
    Serial.print("[WiFi] RSSI: "); Serial.println(WiFi.RSSI());  // Signal strength
    
    // Play tune only on first successful connection
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

// ========================================================================
// Robot Movement Functions
// ========================================================================

/**
 * Core movement function with motor trim compensation and smooth acceleration
 * 
 * Motor trim compensates for manufacturing differences between motors to
 * make the car drive straight. Without trim, most robots veer to one side.
 * 
 * Smooth acceleration prevents wheel slipping and jerky motion by gradually
 * ramping speed from current to target over time.
 * 
 * @param direction - Movement direction (Forward, Backward, Move_Left, etc.)
 * @param baseSpeed - Target motor speed (0-255 PWM value)
 */
void moveWithTrim(int direction, int baseSpeed) {
  int targetSpeed = baseSpeed;
  unsigned long currentTime = millis();
  
  // ========================================================================
  // Smooth Start: Jump to minimum working speed
  // ========================================================================
  // Motors need minimum speed to overcome static friction
  // With trim of 0.66, we need at least 180 to get ~120 PWM after trim
  if(currentMotorSpeed < 100) {
    currentMotorSpeed = 180;  // Start high enough that trimmed wheels still turn
  }
  
  // ========================================================================
  // Gradual Acceleration/Deceleration
  // ========================================================================
  // Update speed in small increments every 30ms for smooth transitions
  // This prevents wheel slip and provides better control
  if(currentTime - lastAccelTime >= 30) {
    lastAccelTime = currentTime;
    int speedStep = 15;  // Change speed by 15 PWM units per step
    
    // Ramp up to target speed
    if(currentMotorSpeed < targetSpeed) {
      currentMotorSpeed += speedStep;
      if(currentMotorSpeed > targetSpeed) currentMotorSpeed = targetSpeed;  // Don't overshoot
    }
    // Ramp down to target speed
    else if(currentMotorSpeed > targetSpeed) {
      currentMotorSpeed -= speedStep;
      if(currentMotorSpeed < targetSpeed) currentMotorSpeed = targetSpeed;  // Don't undershoot
    }
  }
  
  // ========================================================================
  // Apply Motor Trim Calibration
  // ========================================================================
  // Multiply current speed by trim factors to compensate for motor differences
  int leftSpeed = (int)(currentMotorSpeed * leftMotorTrim);   // 0.66x = left motors run at 66%
  int rightSpeed = (int)(currentMotorSpeed * rightMotorTrim); // 1.0x = right motors run at 100%
  
  // Ensure values are valid PWM range (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Debug output showing speed calculations
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

// Simple movement wrappers that call moveWithTrim with specific direction
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
  currentMotorSpeed = 0;  // Reset acceleration for next movement
  myCar.Move(Stop, 0);
}

void rotateLeft() {
  Serial.print("Robot: Rotate Left (Speed: ");
  Serial.print(motorSpeed);
  Serial.println(")");
  myCar.Move(Contrarotate, motorSpeed);  // Counter-clockwise rotation
}

void rotateRight() {
  Serial.print("Robot: Rotate Right (Speed: ");
  Serial.print(motorSpeed);
  Serial.println(")");
  myCar.Move(Clockwise, motorSpeed);  // Clockwise rotation
}

// ========================================================================
// Servo Control Functions
// ========================================================================

void servoLeft() {
  Serial.println("Robot: Servo Left");
  servoPosition = min(180, servoPosition + 30);  // Move 30 degrees left, max 180
  scanServo.write(servoPosition);
  Serial.print("Servo position: ");
  Serial.println(servoPosition);
  delay(500); // Give servo time to move
}

void servoRight() {
  Serial.println("Robot: Servo Right");
  servoPosition = max(0, servoPosition - 30);  // Move 30 degrees right, min 0
  scanServo.write(servoPosition);
  Serial.print("Servo position: ");
  Serial.println(servoPosition);
  delay(500); // Give servo time to move
}

void servoCenter() {
  Serial.println("Robot: Servo Center");
  servoPosition = 90;  // Center position
  scanServo.write(servoPosition);
  Serial.println("Servo centered at 90 degrees");
  delay(500); // Give servo time to move
}

// ========================================================================
// Accessory Control Functions
// ========================================================================

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
    // Play two sharp beeps using playToneHz function
    playToneHz(BUZZER_PIN, 2000, 100);  // 2000Hz beep for 100ms
    delay(50);
    playToneHz(BUZZER_PIN, 2000, 100);  // Second beep
    
    // Ensure buzzer is off after beeping
    digitalWrite(BUZZER_PIN, LOW);
    
    Serial.println("Buzzer: BEEP");
    buzzerState = false; // Reset state after beeping
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Buzzer: OFF");
  }
}

void readUltrasonic() {
  float distance = sensor.Ranging();  // Get distance in cm
  Serial.print("Ultrasonic distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

// ========================================================================
// Camera Control Functions (via UART to ESP32-CAM)
// ========================================================================

/**
 * Flip camera image 180 degrees (vertical + horizontal flip)
 * Useful when camera is mounted upside-down on the robot
 * Sends AT commands to ESP32-CAM module via Serial1
 */
void flipCameraImage() {
  // Send AT commands to ESP32-CAM
  Serial1.println("AT+VFLIP=1");  // Enable vertical flip (upside-down)
  delay(50);                      // Wait for command to process
  Serial1.println("AT+HFLIP=1");  // Enable horizontal flip (mirror)
  delay(100);
  Serial.println("[Camera] Image flipped 180 degrees");
}

/**
 * Reset camera to normal orientation (no flipping)
 * Use this when camera is mounted right-side up
 */
void resetCameraImage() {
  // Send AT commands to ESP32-CAM
  Serial1.println("AT+VFLIP=0");  // Disable vertical flip
  delay(50);
  Serial1.println("AT+HFLIP=0");  // Disable horizontal flip
  delay(100);
  Serial.println("[Camera] Image reset to normal orientation");
}

// ========================================================================
// Arduino Setup Function - Runs once at power-on
// ========================================================================
void setup() {
  // ========================================================================
  // Initialize Serial Communication
  // ========================================================================
  Serial.begin(115200);  // USB serial for debugging
  Serial.println("Acebott Robot Starting (PS3 Controller Primary)...");
  
  // ========================================================================
  // Initialize Motor Controller
  // ========================================================================
  myCar.Init();  // Configure motor driver pins and PWM channels
  Serial.println("Vehicle initialized");
  
  // ========================================================================
  // Initialize Ultrasonic Distance Sensor
  // ========================================================================
  sensor.Init(TRIG_PIN, ECHO_PIN);  // Setup HC-SR04 sensor
  Serial.println("Ultrasonic sensor initialized");
  
  // ========================================================================
  // Initialize Ultrasonic Scanner Servo
  // ========================================================================
  scanServo.attach(SERVO_PIN);  // Attach servo to pin 25
  delay(100);                   // Allow servo driver to initialize
  scanServo.write(90);          // Move to center position
  servoPosition = 90;           // Update tracking variable
  delay(1000);                  // Wait for servo to reach position
  Serial.println("Servo initialized at center position (90 degrees)");
  
  // ========================================================================
  // Initialize Camera Tilt Servo
  // ========================================================================
  cameraPanTilt.attach(CAMERA_PAN_TILT_PIN);  // Attach to pin 27
  delay(100);
  cameraPanTilt.write(90);      // Center position for level view
  cameraPanTiltPosition = 90;
  delay(500);
  Serial.println("Camera pan/tilt servo initialized on pin 27");
  
  // ========================================================================
  // Initialize UART Communication with ESP32-CAM
  // ========================================================================
  // Serial1 is used to send AT commands to camera module
  Serial1.begin(CAMERA_UART_BAUD, SERIAL_8N1, CAMERA_UART_RX, CAMERA_UART_TX);
  Serial.println("Camera UART initialized (Serial1 at 115200 baud)");
  
  // ========================================================================
  // Initialize Output Pins
  // ========================================================================
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Set all outputs to OFF state initially
  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("LED and buzzer pins initialized");
  
  // ========================================================================
  // Initialize PS3 Controller (Bluetooth)
  // ========================================================================
  // PS3 controller connects directly to ESP32 via Bluetooth
  // MAC address "20:00:00:00:82:62" must match controller pairing
  Ps3.attach(onPs3Notify);          // Register callback for controller events
  Ps3.attachOnConnect(onPs3Connect); // Register callback for connection event
  Ps3.begin("20:00:00:00:82:62");   // Start with ESP32's Bluetooth MAC address
  Serial.println("PS3 Controller ready - Press PS button to connect");
  
  Serial.println("=== ROBOT READY ===");
  Serial.println("IR Remote & PS3 Controller supported");
  Serial.println("====================");
  
  // ========================================================================
  // Connect to WiFi Network
  // ========================================================================
  // WiFi is needed to access ESP32-CAM video stream from browser/app
  tryConnectWiFi();              // Attempt initial connection
  lastWifiAttemptMs = millis();  // Record attempt time for retry logic
  
  // Note: Web server removed - camera is accessed directly at camera's IP
  // Robot control is via PS3 controller only
}

// ========================================================================
// Arduino Main Loop - Runs continuously
// ========================================================================
void loop() {
  // ========================================================================
  // WiFi Connection Maintenance
  // ========================================================================
  // Periodically check and restore WiFi connection if lost
  // This is non-blocking - robot continues to function during WiFi issues
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    
    // Only retry connection every 10 seconds to avoid constant attempts
    if (now - lastWifiAttemptMs >= wifiRetryIntervalMs) {
      lastWifiAttemptMs = now;
      tryConnectWiFi(7000); // Quick 7-second retry attempt
    }
  }
  
  // ========================================================================
  // PS3 Controller Handling
  // ========================================================================
  // All PS3 controller logic is handled by the onPs3Notify() callback
  // The callback is triggered automatically when controller state changes
  // No explicit code needed here
  
  // Small delay for system stability and to prevent watchdog timer reset
  delay(100);
}


