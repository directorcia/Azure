#define leftLed 12
#define rightLed 2
void setup(){
  pinMode(leftLed, OUTPUT);
  pinMode(rightLed, OUTPUT);
}
void loop(){
  digitalWrite(leftLed,HIGH);//Turn on the left LED
  digitalWrite(rightLed,HIGH);//Turn on the right LED
}