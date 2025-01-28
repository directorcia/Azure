#include <Arduino.h>
/*
  keyestudio sun_follower
  lesson 3.2
  button
  http://www.keyestudio.com
*/
#define LED 3 //define the LED pin as D3
#define button 2 //define the pin of the push button module as D2
volatile int buttonState;  //the state of the level output by the push
void setup()
{
  Serial.begin(115200); //set baud rate to 115200
  pinMode(button, INPUT); // initialize digital pin button as an input.
  pinMode(LED, OUTPUT); // initialize digital pin LED as an output.
}
void loop ()
{
  buttonState = digitalRead(button);    //read the state of the push button module
  if (buttonState == 0) //if the button is pressed
  {
    digitalWrite(LED, HIGH);  //the LED lights up
  }
  else
  {
    digitalWrite(LED, LOW);   //the LED is off
  }
  delay(100); //delay in 100ms
}