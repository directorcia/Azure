#include <Arduino.h>
#include <Servo.h>  //include the library code:
Servo myservo0;
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo servos[] = {myservo0, myservo1, myservo2, myservo3, myservo4, myservo5};
int16_t m0_open = 0; // Servo 0 open
int16_t m0_close = 180; // Servo 0 close
int16_t m1_up = 0; // Servo 1 up
int16_t m1_mid = 90; // Servo 1 mid
int16_t m1_down = 180; // Servo 1 down
int16_t m2_up = 180; // Servo 2 up
int16_t m2_mid = 90; // Servo 2 mid
int16_t m2_down = 0; // Servo 2 down
int16_t m3_up = 0; // Servo 3 up
int16_t m3_mid = 90; // Servo 3 mid 
int16_t m3_down = 180; // Servo 3 down
int16_t m4_down = 0; // Servo 4 down
int16_t m4_mid = 90; // Servo 4 mid 
int16_t m4_up = 180; // Servo 4 up
int16_t m5_right = 0; // Servo 5 down
int16_t m5_mid = 90; // Servo 5 mid
int16_t m5_left = 180; // Servo 5 up
// Servo positions
int currentPosition0 = m0_close; // Initial position of servo0
int currentPosition1 = m1_mid;  // Initial position of servo1
int currentPosition2 = m2_mid;  // Initial position of servo2
int currentPosition3 = 135; // Initial position of servo3
int currentPosition4 = m4_up; // Initial position of servo4
int currentPosition5 = m5_mid;  // Initial position of servo5
int homePosition[] = {m0_close, m1_mid, m2_mid, 135, m4_up, m5_mid};
int firstPosition[] = {m0_open, m1_mid, m2_down+22, 155, m4_up, m5_left};
int secondPosition[] = {m0_open, m1_mid, m2_down+22, 172, m4_up, m5_left};
int thirdPosition[] = {145, m1_mid, m2_down+22, 172, m4_up, m5_left};
int fourthTargetPosition[] = {145, m1_mid, m2_up, 155, m4_up, m5_right};
int fifthTargetPosition[] = {145, m1_mid, m2_down+22, 155, m4_up, m5_right};
int sixthTargetPosition[] = {145, m1_mid, m2_down+22, 168, m4_up, m5_right};
int seventhTargetPosition[] = {m0_open, m1_mid, m2_down+22, 168, m4_up, m5_right};
int eighthTargetPosition[] = {m0_open, m1_mid, m2_down+22, 155, m4_up, m5_right};

void smoothMove(Servo servos[], int currentPositions[], int targetPositions[], int numServos, int stepDelay) {
  bool movementComplete = false;
  while (!movementComplete) {
    movementComplete = true;
    for (int i = 0; i < numServos; i++) {
      if (currentPositions[i] < targetPositions[i]) {
        currentPositions[i]++;
        servos[i].write(currentPositions[i]);
        movementComplete = false;
      } else if (currentPositions[i] > targetPositions[i]) {
        currentPositions[i]--;
        servos[i].write(currentPositions[i]);
        movementComplete = false;
      }
    }
    delay(stepDelay);
  }
}

void updateCurrentPositions(int currentPositions[], int numServos) {
  currentPosition0 = currentPositions[0];
  currentPosition1 = currentPositions[1];
  currentPosition2 = currentPositions[2];
  currentPosition3 = currentPositions[3];
  currentPosition4 = currentPositions[4];
  currentPosition5 = currentPositions[5];
}

void setup() {
  
  myservo0.attach(0);  //link the servo to digital port 0
  myservo1.attach(1);  //link the servo to digital port 1
  myservo2.attach(2);  //link the servo to digital port 2
  myservo3.attach(3);  //link the servo to digital port 3
  myservo4.attach(4);  //link the servo to digital port 4
  myservo5.attach(5);  //link the servo to digital port 5
  myservo0.write(currentPosition0);
  myservo1.write(currentPosition1);
  myservo2.write(currentPosition2);
  myservo3.write(currentPosition3);
  myservo4.write(currentPosition4);
  myservo5.write(currentPosition5);
  delay(5000); //wait for 2 seconds
}
void loop() {
  int currentPositions[] = {currentPosition0, currentPosition1, currentPosition2, currentPosition3, currentPosition4, currentPosition5};
 
  smoothMove(servos, currentPositions, firstPosition, 6, 20);
  // Update global positions after movement
  updateCurrentPositions(currentPositions, 6);
  delay(2000); // Wait for 2 seconds before the next movement
  smoothMove(servos, currentPositions, secondPosition, 6, 20);
  // Update global positions after movement
  updateCurrentPositions(currentPositions, 6);
  delay(2000); // Wait for 2 seconds before the next movement
  smoothMove(servos, currentPositions, thirdPosition, 6, 20);
  // Update global positions after movement
  updateCurrentPositions(currentPositions, 6);
  delay(2000); // Wait for 2 seconds before the next movement
  smoothMove(servos, currentPositions, fourthTargetPosition, 6, 20);
  // Update global positions after second movement
  updateCurrentPositions(currentPositions, 6);
  delay(2000); // Wait for 2 seconds before the next movement
  smoothMove(servos, currentPositions, fifthTargetPosition, 6, 20);
  // Update global positions after second movement
  updateCurrentPositions(currentPositions, 6);
  delay(2000); // Wait for 2 seconds before the next movement
  smoothMove(servos, currentPositions, sixthTargetPosition, 6, 20);
  // Update global positions after second movement
  updateCurrentPositions(currentPositions, 6);
  delay(2000); // Wait for 2 seconds before the next movement
  smoothMove(servos, currentPositions, seventhTargetPosition, 6, 20);
  // Update global positions after second movement
  updateCurrentPositions(currentPositions, 6);
  delay(2000); // Wait for 2 seconds before the next movement
  smoothMove(servos, currentPositions, eighthTargetPosition, 6, 20);
  // Update global positions after second movement
  updateCurrentPositions(currentPositions, 6);
  delay(2000); // Wait for 2 seconds before the next movement
  smoothMove(servos, currentPositions, homePosition, 6, 20);
  updateCurrentPositions(currentPositions, 6);
  delay(2000); // Wait for 2 seconds before the next movement
  
}