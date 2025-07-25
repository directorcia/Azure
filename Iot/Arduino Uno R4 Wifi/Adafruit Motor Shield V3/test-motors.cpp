#include <Arduino.h>
#include <Adafruit_MotorShield.h>

// Function declarations
void testMotor(Adafruit_DCMotor *motor, String motorName);
void testAllMotors();

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create motor objects for all 4 motors
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit Motor Shield v3 - 4 DC Motor Test");
  
  // Initialize the motor shield
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  
  // Set initial speed for all motors (0-255)
  motor1->setSpeed(150);
  motor2->setSpeed(150);
  motor3->setSpeed(150);
  motor4->setSpeed(150);
}

void loop() {
  Serial.println("\n=== Starting Motor Test Sequence ===");
  
  // Test each motor individually
  testMotor(motor1, "Motor 1");
  delay(1000);
  
  testMotor(motor2, "Motor 2");
  delay(1000);
  
  testMotor(motor3, "Motor 3");
  delay(1000);
  
  testMotor(motor4, "Motor 4");
  delay(1000);
  
  // Test all motors together
  testAllMotors();
  
  Serial.println("\n=== Test sequence complete. Waiting 5 seconds... ===");
  delay(5000);
}

void testMotor(Adafruit_DCMotor *motor, String motorName) {
  Serial.println("=== Testing " + motorName + " ===");
  
  // Test forward direction
  Serial.println("  → " + motorName + " running FORWARD at speed 150");
  motor->run(FORWARD);
  delay(2000);
  
  // Test backward direction
  Serial.println("  ← " + motorName + " running BACKWARD at speed 150");
  motor->run(BACKWARD);
  delay(2000);
  
  // Stop the motor
  Serial.println("  ■ " + motorName + " STOPPED (RELEASED)");
  motor->run(RELEASE);
  delay(500);
  Serial.println("  " + motorName + " test complete.\n");
}

void testAllMotors() {
  Serial.println("\n=== Testing all motors together ===");
  
  // All motors forward
  Serial.println("  → ALL MOTORS running FORWARD at speed 150");
  Serial.println("    Motor 1: FORWARD | Motor 2: FORWARD | Motor 3: FORWARD | Motor 4: FORWARD");
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor3->run(FORWARD);
  motor4->run(FORWARD);
  delay(3000);
  
  // All motors backward
  Serial.println("  ← ALL MOTORS running BACKWARD at speed 150");
  Serial.println("    Motor 1: BACKWARD | Motor 2: BACKWARD | Motor 3: BACKWARD | Motor 4: BACKWARD");
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
  motor3->run(BACKWARD);
  motor4->run(BACKWARD);
  delay(3000);
  
  // Stop all motors
  Serial.println("  ■ ALL MOTORS STOPPED (RELEASED)");
  Serial.println("    Motor 1: STOPPED | Motor 2: STOPPED | Motor 3: STOPPED | Motor 4: STOPPED");
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);
  delay(1000);
  
  // Speed ramping test
  Serial.println("\n  === Speed Ramping Test (All Motors Forward) ===");
  for (int speed = 0; speed <= 255; speed += 25) {
    Serial.println("    → Speed: " + String(speed) + "/255 - All motors FORWARD");
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    motor3->setSpeed(speed);
    motor4->setSpeed(speed);
    
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor3->run(FORWARD);
    motor4->run(FORWARD);
    delay(1000);
  }
  
  // Stop all motors
  Serial.println("  ■ Speed test complete - ALL MOTORS STOPPED");
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);
  
  // Reset speed to default
  motor1->setSpeed(150);
  motor2->setSpeed(150);
  motor3->setSpeed(150);
  motor4->setSpeed(150);
  Serial.println("  Speed reset to default (150) for all motors\n");
}