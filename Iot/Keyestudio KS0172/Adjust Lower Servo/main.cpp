#include <Arduino.h>
#include <wire.h>
#include <Servo.h>

Servo lr_servo;//define the name of the servo rotating right and left
int lr_angle = 90;//set the initial angle to 90 degreeset the initial angle to 90 degree
const byte lr_servopin = 9;//define the name of the servo rotating upwards and downwards and its control pin

void setup() {
    lr_servo.attach(lr_servopin);  // set the control pin of the servo
    lr_servo.write(lr_angle);//return to initial angle
    delay(1000);
}

void loop() {}