#include <Arduino.h>
/*
  keyestudio sun_follower
  lesson 6.1
  photovaristor
  http://www.keyestudio.com
*/
#define photos  A0   //photoresistance pin to A0
#define LED 3   //define the LED pin as D3
#define buzzer 6 //buzzer pin to D6
volatile int value = 0;
void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);// initialize digital pin LED as an output.
}
void loop () {
  value = analogRead(photos);  //read the value detected by the sensor
  Serial.println(value);
  if (value < 500) {  //when the analog value is less than 300
    digitalWrite(LED, HIGH); //the LED lights up
    tone(buzzer, 300); //buzzer sounds
  }
  else {  //when the analog value is bigger than 300
    digitalWrite(LED, LOW); //the LED is off
    noTone(buzzer); //buzzer is off
  }
  delay(100);               //delay in 100ms
}
