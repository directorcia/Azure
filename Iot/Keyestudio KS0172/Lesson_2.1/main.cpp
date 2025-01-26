
#include <Arduino.h>
/*
  keyestudio sun_follower
  lesson 1.1
  Blink
  http://www.keyestudio.com
*/
#define LED  3 //define the pin of LED as D3
int value;
void setup()
{
  pinMode(LED, OUTPUT);// initialize digital pin LED as an output.
}
void loop () {
  for (value = 0; value < 255; value = value + 1) {
    analogWrite (LED, value); // LED lights gradually light up
    delay (5); // delay 5MS
  }
  for (value = 255; value > 0; value = value - 1) {
    analogWrite (LED, value); // LED gradually goes out
    delay (5); // delay 5MS
  }
}