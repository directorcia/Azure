#include <ESP32Servo.h>
Servo myServo; // create servo object to control a servo

int myServo_PIN = 25; //Declare the pin of the motor
void setup() {
  myServo.attach(myServo_PIN);//initialize servo motor
  //Set the servo motor to move to the initial position
  myServo.write(0); 
}
void loop() {
  for(int angle = 0;angle <= 180;angle++){
    //Servo motor from 0 degrees to 180 degrees
    myServo.write(angle);
    delay(10); 
  }
  for(int angle = 180;angle >= 0;angle--){
    //Servo motor from 180 degrees to 0 degrees
    myServo.write(angle);
    delay(10); 
  }
}
