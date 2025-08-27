#include <Arduino.h>
#include <vehicle.h>
#include <ultrasonic.h>
#include <ESP32Servo.h>
#include <IRremote.h>

// LED pin definitions
#define LED1_PIN 2    // Built-in LED on most ESP32 boards
#define LED2_PIN 2    // Additional LED pin
#define LED3_PIN 12   // Additional LED pin

// Ultrasonic sensor pins
#define TRIG_PIN 13
#define ECHO_PIN 14

// Servo pin
#define SERVO_PIN 25

// Buzzer pin
#define BUZZER_PIN 33  // Common buzzer pin on ESP32 boards

// IR receiver pin
#define IR_RECEIVE_PIN 4  // IR receiver pin (avoiding conflict with LED pins)

vehicle myCar;
ultrasonic sensor;
Servo scanServo;
void setup() {
  Serial.begin(115200);
  myCar.Init();//Initialize all motors
  
  // Initialize LED pins
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  
  // Turn off all LEDs initially
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  
  // === LED TEST PATTERN ===
  Serial.println("=== LED TEST PATTERN ===");
  
  // Flash all LEDs together 3 times
  Serial.println("Testing all LEDs together...");
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH);
    delay(300);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
    delay(300);
  }
  
  // Flash LEDs individually
  Serial.println("Testing LEDs individually...");
  for(int i = 0; i < 2; i++) {
    Serial.println("LED1...");
    digitalWrite(LED1_PIN, HIGH);
    delay(200);
    digitalWrite(LED1_PIN, LOW);
    delay(100);
    
    Serial.println("LED2...");
    digitalWrite(LED2_PIN, HIGH);
    delay(200);
    digitalWrite(LED2_PIN, LOW);
    delay(100);
    
    Serial.println("LED3...");
    digitalWrite(LED3_PIN, HIGH);
    delay(200);
    digitalWrite(LED3_PIN, LOW);
    delay(100);
  }
  
  Serial.println("LED test complete!");
  delay(500);
  
  // Initialize buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  
  // === BUZZER TEST PATTERN ===
  Serial.println("=== BUZZER TEST PATTERN ===");
  
  // Test basic tones
  Serial.println("Testing buzzer frequencies...");
  int frequencies[] = {500, 800, 1000, 1500, 2000, 2500};
  for(int i = 0; i < 6; i++) {
    Serial.print("Playing ");
    Serial.print(frequencies[i]);
    Serial.println("Hz");
    tone(BUZZER_PIN, frequencies[i], 300);
    digitalWrite(LED1_PIN, HIGH);
    delay(400);
    digitalWrite(LED1_PIN, LOW);
    noTone(BUZZER_PIN);
    delay(200);
  }
  
  // Musical scale test
  Serial.println("Playing musical scale...");
  int notes[] = {523, 587, 659, 698, 784, 880, 988, 1047}; // C major scale
  for(int i = 0; i < 8; i++) {
    tone(BUZZER_PIN, notes[i], 250);
    digitalWrite(LED2_PIN, HIGH);
    delay(300);
    digitalWrite(LED2_PIN, LOW);
    noTone(BUZZER_PIN);
    delay(100);
  }
  
  // Startup melody
  Serial.println("Playing startup melody...");
  int startupMelody[] = {659, 659, 698, 784, 784, 698, 659};
  int startupDurations[] = {200, 200, 200, 400, 200, 200, 400};
  for(int i = 0; i < 7; i++) {
    tone(BUZZER_PIN, startupMelody[i], startupDurations[i]);
    digitalWrite(LED3_PIN, HIGH);
    delay(startupDurations[i] + 50);
    digitalWrite(LED3_PIN, LOW);
    noTone(BUZZER_PIN);
    delay(50);
  }
  
  Serial.println("Buzzer test complete!");
  delay(500);
  
  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  
  // === IR CONTROLLER TEST ===
  Serial.println("=== IR CONTROLLER TEST ===");
  Serial.println("Point your IR remote at the receiver and press any button");
  Serial.println("The buzzer will sound when IR signals are received");
  Serial.println("Test will run for 10 seconds...");
  
  unsigned long irTestStart = millis();
  int irSignalsReceived = 0;
  
  while(millis() - irTestStart < 10000) { // 10 second test
    if (IrReceiver.decode()) {
      irSignalsReceived++;
      
      // Sound buzzer when IR signal received
      tone(BUZZER_PIN, 1500, 200);
      
      // Flash all LEDs
      digitalWrite(LED1_PIN, HIGH);
      digitalWrite(LED2_PIN, HIGH);
      digitalWrite(LED3_PIN, HIGH);
      delay(200);
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED2_PIN, LOW);
      digitalWrite(LED3_PIN, LOW);
      
      // Print IR signal info
      Serial.print("IR Signal #");
      Serial.print(irSignalsReceived);
      Serial.print(" received! Protocol: ");
      Serial.print(getProtocolString(IrReceiver.decodedIRData.protocol));
      Serial.print(", Command: 0x");
      Serial.println(IrReceiver.decodedIRData.command, HEX);
      
      noTone(BUZZER_PIN);
      IrReceiver.resume(); // Enable receiving of the next value
      delay(100);
    }
    delay(50); // Small delay to prevent overwhelming the processor
  }
  
  Serial.print("IR test complete! Received ");
  Serial.print(irSignalsReceived);
  Serial.println(" signals.");
  
  if(irSignalsReceived == 0) {
    Serial.println("No IR signals detected - check IR remote and receiver connection");
    // Play error tone
    tone(BUZZER_PIN, 300, 500);
    delay(600);
    noTone(BUZZER_PIN);
  } else {
    Serial.println("IR controller working properly!");
    // Play success melody
    int successTones[] = {523, 659, 784};
    for(int i = 0; i < 3; i++) {
      tone(BUZZER_PIN, successTones[i], 200);
      delay(250);
    }
    noTone(BUZZER_PIN);
  }
  
  delay(500);
  
  // Initialize ultrasonic sensor
  sensor.Init(TRIG_PIN, ECHO_PIN);
  
  // Initialize servo
  Serial.println("Initializing servo...");
  
  // Use attach with explicit pulse width range for better control
  scanServo.attach(SERVO_PIN, 500, 2500); // Min 500µs, Max 2500µs for full range
  
  // === SERVO CALIBRATION TEST ===
  Serial.println("=== SERVO CALIBRATION TEST ===");
  Serial.println("Testing servo positions: 0° → 90° → 180°");
  
  Serial.println("Position 90°:");
  scanServo.write(90);
  
  Serial.println("Servo calibration test complete!");
  
  // === ULTRASONIC + SERVO TEST ===
  Serial.println("=== ULTRASONIC + SERVO TEST ===");
  Serial.println("Testing ultrasonic sensor at different servo positions");
  
  int testPositions[] = {0, 45, 90, 135, 180}; // Test 5 positions
  for(int i = 0; i < 5; i++) {
    Serial.print("Servo position ");
    Serial.print(testPositions[i]);
    Serial.print("°: ");
    
    scanServo.write(testPositions[i]);
    delay(1000); // Wait for servo to reach position
    
    // Take 3 distance readings and average them
    float totalDistance = 0;
    int validReadings = 0;
    
    for(int j = 0; j < 3; j++) {
      float distance = sensor.Ranging();
      if(distance > 0 && distance < 400) { // Valid range for most ultrasonic sensors
        totalDistance += distance;
        validReadings++;
      }
      delay(100);
    }
    
    if(validReadings > 0) {
      float avgDistance = totalDistance / validReadings;
      Serial.print("Distance = ");
      Serial.print(avgDistance, 1);
      Serial.println(" cm");
      
      // Flash LEDs based on distance
      if(avgDistance < 20) {
        // Close object - flash red (LED1)
        for(int k = 0; k < 3; k++) {
          digitalWrite(LED1_PIN, HIGH);
          delay(100);
          digitalWrite(LED1_PIN, LOW);
          delay(100);
        }
      } else if(avgDistance < 50) {
        // Medium distance - flash yellow (LED2)
        for(int k = 0; k < 3; k++) {
          digitalWrite(LED2_PIN, HIGH);
          delay(100);
          digitalWrite(LED2_PIN, LOW);
          delay(100);
        }
      } else {
        // Far object - flash green (LED3)
        for(int k = 0; k < 3; k++) {
          digitalWrite(LED3_PIN, HIGH);
          delay(100);
          digitalWrite(LED3_PIN, LOW);
          delay(100);
        }
      }
    } else {
      Serial.println("No valid readings");
      // Flash all LEDs for error
      digitalWrite(LED1_PIN, HIGH);
      digitalWrite(LED2_PIN, HIGH);
      digitalWrite(LED3_PIN, HIGH);
      delay(200);
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED2_PIN, LOW);
      digitalWrite(LED3_PIN, LOW);
    }
    
    delay(500);
  }
  
  Serial.println("Ultrasonic + Servo test complete!");
  
  
}
void loop() {
  // === SERVO + LED TEST LOOP ===
  Serial.println("Position 0°:");
  scanServo.write(0);
  // Flash LED1 for left position
  for(int i = 0; i < 5; i++) {
    digitalWrite(LED1_PIN, HIGH);
    delay(100);
    digitalWrite(LED1_PIN, LOW);
    delay(100);
  }
  
  Serial.println("Position 180°:");
  scanServo.write(180);
  // Flash LED3 for right position  
  for(int i = 0; i < 5; i++) {
    digitalWrite(LED3_PIN, HIGH);
    delay(100);
    digitalWrite(LED3_PIN, LOW);
    delay(100);
  }
  
  Serial.println("Position 135°:");
  scanServo.write(135);
  // Flash LED2 for middle-right position
  for(int i = 0; i < 5; i++) {
    digitalWrite(LED2_PIN, HIGH);
    delay(100);
    digitalWrite(LED2_PIN, LOW);
    delay(100);
  }
  
  Serial.println("Position 90°:");
  scanServo.write(90);
  // Flash all LEDs for center position
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH);
    delay(150);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
    delay(150);
  }
  
  // === MOTOR TEST: ALL 4 WHEELS ===
  Serial.println("\n=== MOTOR TEST ===");
  
  // All wheels FORWARD
  Serial.println("All 4 wheels FORWARD:");
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);
  myCar.Move(Forward, 200);
  delay(2000); // Run for 2 seconds
  myCar.Move(Stop, 0);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  delay(500);
  
  // All wheels BACKWARD
  Serial.println("All 4 wheels BACKWARD:");
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED3_PIN, HIGH);
  myCar.Move(Backward, 200);
  delay(2000); // Run for 2 seconds
  myCar.Move(Stop, 0);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  delay(500);
  
  // === INDIVIDUAL MOTOR SPEED RANGE TEST ===
  Serial.println("\n=== INDIVIDUAL MOTOR SPEED RANGE TEST ===");
  Serial.println("Testing all motors through full speed range");
  Serial.println("Motor Layout: M1=Front Left, M2=Rear Left, M3=Front Right, M4=Rear Right");
  Serial.println("");
  
  // Test each motor individually through speed range
  String motorNames[] = {"M1 (Front Left)", "M2 (Rear Left)", "M3 (Front Right)", "M4 (Rear Right)"};
  int motorForward[] = {M1_Forward, M2_Forward, M3_Forward, M4_Forward};
  int motorBackward[] = {M1_Backward, M2_Backward, M3_Backward, M4_Backward};
  int speedRange[] = {100, 150, 200, 255}; // Test 4 different speeds
  
  for(int motor = 0; motor < 4; motor++) {
    Serial.print("\n=== Testing ");
    Serial.print(motorNames[motor]);
    Serial.println(" ===");
    Serial.print("Forward constant: ");
    Serial.print(motorForward[motor]);
    Serial.print(", Backward constant: ");
    Serial.println(motorBackward[motor]);
    
    // Test forward direction at all speeds
    Serial.println("FORWARD direction tests:");
    for(int speed = 0; speed < 4; speed++) {
      Serial.print("  Speed ");
      Serial.print(speedRange[speed]);
      Serial.print(": ");
      
      // LED indicator: All LEDs for current test
      digitalWrite(LED1_PIN, HIGH);
      digitalWrite(LED2_PIN, HIGH);
      digitalWrite(LED3_PIN, HIGH);
      
      myCar.Move(motorForward[motor], speedRange[speed]);
      
      // Check for IR signals during motor test
      unsigned long motorTestStart = millis();
      while(millis() - motorTestStart < 1500) { // 1.5 seconds per speed test
        if (IrReceiver.decode()) {
          // Stop motor for Yankee Doodle
          myCar.Move(Stop, 0);
          
          Serial.println("\nIR Signal detected during motor test!");
          Serial.print("Command: 0x");
          Serial.print(IrReceiver.decodedIRData.command, HEX);
          Serial.println(" - Playing Yankee Doodle...");
          
          // Play Yankee Doodle (shortened version for motor tests)
          int yankeeNotes[] = {262, 262, 294, 330, 262, 330, 294, 262, 262, 294, 330, 262};
          int yankeeDurations[] = {200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 400};
          
          for(int note = 0; note < 12; note++) {
            tone(BUZZER_PIN, yankeeNotes[note], yankeeDurations[note]);
            digitalWrite(LED1_PIN + (note % 3), HIGH);
            delay(yankeeDurations[note]);
            digitalWrite(LED1_PIN, LOW);
            digitalWrite(LED2_PIN, LOW);
            digitalWrite(LED3_PIN, LOW);
            noTone(BUZZER_PIN);
            delay(25);
          }
          
          Serial.println("Yankee Doodle complete! Resuming motor test...");
          IrReceiver.resume();
          break; // Exit the motor test timing loop
        }
        delay(50);
      }
      
      myCar.Move(Stop, 0);
      
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED2_PIN, LOW);
      digitalWrite(LED3_PIN, LOW);
      
      Serial.println("Did wheel move?");
      delay(500);
    }
    
    // Test backward direction at all speeds
    Serial.println("BACKWARD direction tests:");
    for(int speed = 0; speed < 4; speed++) {
      Serial.print("  Speed ");
      Serial.print(speedRange[speed]);
      Serial.print(": ");
      
      // LED indicator: Single LED for backward
      digitalWrite(LED1_PIN + (motor % 3), HIGH);
      
      myCar.Move(motorBackward[motor], speedRange[speed]);
      
      // Check for IR signals during backward motor test
      unsigned long motorTestStart = millis();
      while(millis() - motorTestStart < 1500) { // 1.5 seconds per speed test
        if (IrReceiver.decode()) {
          // Stop motor for Yankee Doodle
          myCar.Move(Stop, 0);
          
          Serial.println("\nIR Signal detected during backward motor test!");
          Serial.print("Command: 0x");
          Serial.print(IrReceiver.decodedIRData.command, HEX);
          Serial.println(" - Playing Yankee Doodle...");
          
          // Play Yankee Doodle (shortened version)
          int yankeeNotes[] = {262, 262, 294, 330, 262, 330, 294, 262, 262, 294, 330, 262};
          int yankeeDurations[] = {200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 400};
          
          for(int note = 0; note < 12; note++) {
            tone(BUZZER_PIN, yankeeNotes[note], yankeeDurations[note]);
            digitalWrite(LED1_PIN + (note % 3), HIGH);
            delay(yankeeDurations[note]);
            digitalWrite(LED1_PIN, LOW);
            digitalWrite(LED2_PIN, LOW);
            digitalWrite(LED3_PIN, LOW);
            noTone(BUZZER_PIN);
            delay(25);
          }
          
          Serial.println("Yankee Doodle complete! Resuming motor test...");
          IrReceiver.resume();
          break; // Exit the motor test timing loop
        }
        delay(50);
      }
      
      myCar.Move(Stop, 0);
      
      digitalWrite(LED1_PIN + (motor % 3), LOW);
      
      Serial.println("Did wheel move?");
      delay(500);
    }
    
    Serial.print("=== ");
    Serial.print(motorNames[motor]);
    Serial.println(" testing complete ===");
    delay(1000);
  }
  
  Serial.println("\n=== ALL MOTOR SPEED RANGE TESTING COMPLETE ===");
  Serial.println("Check which motors moved at which speeds!");
  delay(2000);
  
  // === IR CONTROLLER CONTINUOUS TEST ===
  Serial.println("\n=== IR CONTROLLER CONTINUOUS TEST ===");
  Serial.println("Press any IR remote button to play Yankee Doodle!");
  Serial.println("This test runs continuously during the loop...");
  
  // Check for IR signals during loop
  if (IrReceiver.decode()) {
    // Print IR signal info
    Serial.print("IR Command received: 0x");
    Serial.print(IrReceiver.decodedIRData.command, HEX);
    Serial.print(" (Protocol: ");
    Serial.print(getProtocolString(IrReceiver.decodedIRData.protocol));
    Serial.println(")");
    Serial.println("Playing Yankee Doodle...");
    
    // Flash all LEDs quickly to indicate IR received
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH);
    delay(100);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
    
    // Yankee Doodle melody - notes and durations
    int yankeeNotes[] = {
      262, 262, 294, 330, 262, 330, 294,  // "Yankee Doodle went to town"
      262, 262, 294, 330, 262, 0,         // "A-riding on a pony"
      262, 262, 294, 330, 262, 330, 294,  // "Stuck a feather in his hat"
      220, 247, 262, 0                    // "And called it macaroni"
    };
    
    int yankeeDurations[] = {
      250, 250, 250, 250, 250, 250, 500,  // "Yankee Doodle went to town"
      250, 250, 250, 250, 500, 250,       // "A-riding on a pony" 
      250, 250, 250, 250, 250, 250, 500,  // "Stuck a feather in his hat"
      250, 250, 500, 250                  // "And called it macaroni"
    };
    
    // Play the melody with LED patterns
    for(int i = 0; i < 24; i++) {
      if(yankeeNotes[i] > 0) {
        tone(BUZZER_PIN, yankeeNotes[i], yankeeDurations[i]);
        
        // Different LED pattern for each note
        switch(i % 3) {
          case 0:
            digitalWrite(LED1_PIN, HIGH);
            break;
          case 1:
            digitalWrite(LED2_PIN, HIGH);
            break;
          case 2:
            digitalWrite(LED3_PIN, HIGH);
            break;
        }
      }
      
      delay(yankeeDurations[i]);
      
      // Turn off LEDs and buzzer
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED2_PIN, LOW);
      digitalWrite(LED3_PIN, LOW);
      noTone(BUZZER_PIN);
      
      delay(50); // Small pause between notes
    }
    
    Serial.println("Yankee Doodle complete!");
    IrReceiver.resume(); // Enable receiving of the next value
  }
  
}