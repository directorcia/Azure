
#include <Arduino.h>
/*
  keyestudio sun_follower
  lesson 3.1
  button
  http://www.keyestudio.com
*/
#define button 2 //define the pin of the push button module as D2
volatile int buttonState;  //the state of the level output by the push button module
void setup()
{
  Serial.begin(115200);//set baud rate to 115200
  pinMode(button, INPUT);// initialize digital pin button as an input.
}
void loop () {
  buttonState = digitalRead(button);
  Serial.println(buttonState);  //Automatically wrap and output the digital signal read from digital port 2
  delay(100);//delay in 100ms
}


