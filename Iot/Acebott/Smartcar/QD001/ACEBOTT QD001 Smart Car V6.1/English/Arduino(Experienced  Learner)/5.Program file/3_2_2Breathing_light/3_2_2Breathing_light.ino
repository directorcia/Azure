#define leftLed 12
#define rightLed 2
const int fadeDelay = 10; // Latency per brightness change (ms)
void setup()
{
  pinMode(leftLed, OUTPUT);
  pinMode(rightLed, OUTPUT);
}
void loop()
{
  for (int brightness = 0; brightness <= 255; brightness++)
  {
    analogWrite(leftLed,brightness);
    analogWrite(rightLed,brightness);
    delay(fadeDelay);
  }//gradually Turn on the headlights 
  for (int brightness = 255; brightness >= 0; brightness--)
  {
    analogWrite(leftLed,brightness);
    analogWrite(rightLed,brightness);
    delay(fadeDelay);
  }//Gradually extinguish the headlights

}