#define leftLed 12
#define rightLed 2
void setup()
{  
  pinMode(leftLed, OUTPUT);
  pinMode(rightLed, OUTPUT);
  //Change the red part of the parameter, the parameter range 0-255, so that the lights display different brightness
  analogWrite(leftLed,50);
  analogWrite(rightLed,50);
}
void loop()
{

}
