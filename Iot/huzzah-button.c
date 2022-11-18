#include <Arduino.h>

int ledPin1 = 5; // Green
int ledPin2 = 4; // Red
int buttonpin = 2;
int buttonstate = 0;
int toggle = 0;
byte leds = 0;

void setup() 
{
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(buttonpin, INPUT_PULLUP);
  Serial.begin(115200);
  delay(100);    
}
void loop() 
{
    buttonstate = (digitalRead(buttonpin));
    // Serial.println(buttonstate);
    if (buttonstate == LOW) {
      if (toggle) {
        digitalWrite(ledPin1, LOW);  // Green = Off
        digitalWrite(ledPin2, HIGH); // Red = On
        toggle = 0;
      }
      else {
        digitalWrite(ledPin1, HIGH);  // Green = On
        digitalWrite(ledPin2, LOW); // Red = Off
        toggle = 1;
      }
    }
    delay(100);
 }