#include <Arduino.h>
/*
  keyestudio sun_follower
  lesson 9.1
  servo
  http://www.keyestudio.com
*/
int servoPin = 9; //set the pin of the servo
void servopulse(int pin, int myangle) { //the function of pluse
  int pulsewidth = map(myangle, 0, 180, 500, 2500); //Map angle to pulse width
  for (int i = 0; i < 10; i++) { //output pulse
    digitalWrite(pin, HIGH);//set the servo interface level to high
    delayMicroseconds(pulsewidth);//the delay time of pulse width
    digitalWrite(pin, LOW);//turn the servo interface level to low
    delay(20 - pulsewidth / 1000);
  }
}
void setup() {
  pinMode(servoPin, OUTPUT);//set the pin of the servo
}
void loop() {
  servopulse(servoPin, 0);//rotate to 0 degree
  delay(1000);//delay in 1s
  servopulse(servoPin, 90);//rotate to 90 degrees
  delay(1000);
  servopulse(servoPin, 180);//rotate to 180 degrees
  delay(1000);
}
