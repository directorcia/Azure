#include <ultrasonic.h>
ultrasonic myUltrasonic;
int UT_distance = 0;
void setup() {
  Serial.begin(9600);//The serial port monitor is initialized with baud rate of 9600
  myUltrasonic.Init(13,14);
}
void loop() {
  UT_distance = myUltrasonic.Ranging();
  //the distance of the ultrasonic detection
  Serial.print(UT_distance);
  // The serial port shows the distance of ultrasonic detection
  Serial.println("cm");
  delay(1000);
}