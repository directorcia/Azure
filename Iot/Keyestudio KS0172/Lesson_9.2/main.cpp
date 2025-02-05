#include <Arduino.h>
/*
  keyestudio sun_follower
  lesson 9.2
  servo
  http://www.keyestudio.com
*/
#include <Servo.h>  //include the library code:
Servo myservo;
void setup() {
  myservo.attach(9);  //link the servo to digital port 9
}
void loop () {
  //rotate from 0 degree to 180 degrees
  for (int i = 0; i < 180; i++) {
    myservo.write(i);
    delay(20);
  }
  delay(1000);  //wait for 1s
  //rotate from 180 degree to 0 degree
  for (int i = 180; i > 0; i--) {
    myservo.write(i);
    delay(20);
  }
  delay(1000);  //wait for 1s
}
