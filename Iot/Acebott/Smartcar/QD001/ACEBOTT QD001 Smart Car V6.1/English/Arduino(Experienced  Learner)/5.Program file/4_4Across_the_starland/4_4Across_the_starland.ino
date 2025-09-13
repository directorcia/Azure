#include <vehicle.h>
#include <ultrasonic.h>
#include <ESP32Servo.h>

vehicle myCar;
ultrasonic myUltrasonic;
Servo myServo;

int myServo_PIN = 25; //Declare the pin of the motor
int leftDistance = 0;
int middleDistance = 0;
int rightDistance = 0;


void setup() {
  myCar.Init();
  myUltrasonic.Init(13,14);
  myServo.attach(myServo_PIN);//initialize servo motor
}

void loop() {
  middleDistance = myUltrasonic.Ranging();
  myServo.write(90);
  if(middleDistance <= 25){
    myCar.Move(Stop,0);
    myServo.write(0);
    for(int angle = 90;angle >= 0;angle--){
      //Servo motor from 90 degrees to 0 degrees
      myServo.write(angle);
      delay(10); 
    }
    delay(500);
    rightDistance = myUltrasonic.Ranging();
    for(int angle = 0;angle <= 180;angle++){
      //Servo motor from 0 degrees to 180 degrees
      myServo.write(angle);
      delay(10); 
    }
    delay(500);
    leftDistance = myUltrasonic.Ranging();
    if(rightDistance<20 && leftDistance<20){
      myCar.Move(Backward, 180);
      delay(500);
      myCar.Move(Contrarotate, 180);
      delay(1000);
    }
    else if(rightDistance > leftDistance){
      myCar.Move(Backward, 180);
      delay(500);
      myCar.Move(Clockwise, 180);
      delay(1000);
    }
    else if(rightDistance < leftDistance){
      myCar.Move(Backward, 180);
      delay(500);
      myCar.Move(Contrarotate, 180);
      delay(1000);
    }
    else{
      myCar.Move(Backward, 180);
      delay(500);
      myCar.Move(Clockwise, 180);
      delay(1000);      
    }
  }
  else{
    myCar.Move(Forward,150);
  }
}
