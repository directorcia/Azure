#include <ESP32Servo.h> 

Servo myservo;

int servoPin = 25;
void setup() {
  // put your setup code here, to run once:
myservo.attach(servoPin);
}

void loop() {
  // put your main code here, to run repeatedly:
myservo.write(90);
}
