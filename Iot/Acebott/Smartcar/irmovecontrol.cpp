#include <Arduino.h>
#include <IRremote.h>
#include <vehicle.h>
#include <ultrasonic.h>
#include <ESP32Servo.h>

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
  servoPosition = max(0, servoPosition - 30);
  scanServo.write(servoPosition);
  Serial.print("Servo position: ");
  Serial.println(servoPosition);
  delay(500); // Give servo time to move
}

void servoRight() { 
  Serial.println("Robot: Servo Right");
  servoPosition = min(180, servoPosition + 30);
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
}

void loop() {
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

