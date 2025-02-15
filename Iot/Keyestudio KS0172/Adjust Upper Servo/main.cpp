#include <Arduino.h>
#include <wire.h>
#include <Servo.h>

Servo ud_servo;//define the name of the servo rotating right and left
int ud_angle = 10;//set the initial angle to 10 degree;keep the solar panels upright to detect the strongest light
const byte ud_servopin = 10;//define the servo rotating upwards and downwards and its control pin

void setup() {
    ud_servo.attach(ud_servopin);  // set the control pin of the servo
    ud_servo.write(ud_angle);
    delay(1000);
}
void loop() {}