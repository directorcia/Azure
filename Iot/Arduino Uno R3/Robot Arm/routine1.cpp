#include <Arduino.h>
#include <Servo.h>  //include the library code:
Servo myservo0;
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
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

void servomove(Servo &servo, int &currentPosition, int targetPosition) {
  if (currentPosition < targetPosition) {
    for (int i = currentPosition; i < targetPosition; i++) {
      servo.write(i);
      delay(20);
    }
  } else if (currentPosition > targetPosition) {
    for (int i = currentPosition; i > targetPosition; i--) {
      servo.write(i);
      delay(20);
    }
  }
  currentPosition = targetPosition; // Update the current position
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
void loop () {
  const int pick_up_3 = 158; //pick up the object
  const int pick_up_0 = 145; //pick up the object
  servomove(myservo0, currentPosition0, m0_open); //open the gripper
  servomove(myservo3, currentPosition3, pick_up_3); //move the arm down
  delay(2000); //wait for 1 second
 
  servomove(myservo2, currentPosition2, m2_down); //move the arm down
  servomove(myservo0, currentPosition0, pick_up_0); //close the gripper
  delay(2000); //wait for 2 second
  servomove(myservo3, currentPosition3, 145); //move arm up
  servomove(myservo2, currentPosition2, m2_mid); //move the arm up
  delay(2000); //wait for 2 second
  servomove(myservo1, currentPosition1, m1_down); //twist gripper
  servomove(myservo1, currentPosition1, m1_mid); //twist the arm down
  
  servomove(myservo5, currentPosition5, m5_left); //move the arm down
  //servomove(myservo3, currentPosition3, 145); //move the arm down
  servomove(myservo2, currentPosition2, m2_down+22); 
  servomove(myservo3, currentPosition3, 172); //move the arm down
  delay(2000); //wait for 1 second
  servomove(myservo0, currentPosition0, m0_open); //open the gripper
  delay(2000); //wait for 1 second
  servomove(myservo3, currentPosition3, 135); //move the arm up
  servomove(myservo2, currentPosition2, m2_mid); //move the arm up
  servomove(myservo5, currentPosition5, m5_mid); //move the arm down
  servomove(myservo0, currentPosition0, m0_close); //twist the gripper
}

