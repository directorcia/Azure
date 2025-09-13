#include <vehicle.h>

#define Left_sensor 35 //declare the pin of left tracking sensor
#define Middle_sensor 36 //declare the pin of middle tracking sensor
#define Right_sensor 39 //declare the pin of right tracking sensor
int Left_Tra_Value;
int Middle_Tra_Value;
int Right_Tra_Value;
int Black_Line = 2000;
int Speed = 250;
int RotateSpeed = 150;

vehicle myCar;
void setup()
{
  pinMode(Left_sensor, INPUT);
  pinMode(Middle_sensor, INPUT);
  pinMode(Right_sensor, INPUT);
  myCar.Init();
}
void loop()
{
  Left_Tra_Value = analogRead(Left_sensor);
  Middle_Tra_Value = analogRead(Middle_sensor);
  Right_Tra_Value = analogRead(Right_sensor);
  delay(5);
  if (Left_Tra_Value < Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
  {
    myCar.Move(Forward, Speed);//Smart car forward
  }
  if (Left_Tra_Value < Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
  {
    myCar.Move(Forward, 180);
  }
  if (Left_Tra_Value >= Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
  {
    myCar.Move(Forward, 180);
  }
  else if (Left_Tra_Value >= Black_Line && Middle_Tra_Value < Black_Line && Right_Tra_Value < Black_Line)
  {
    myCar.Move(Contrarotate, 220);
  }
  else if (Left_Tra_Value < Black_Line && Middle_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line)
  {
    myCar.Move(Clockwise, 220);
  }
  else if (Left_Tra_Value >= Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
  {
    myCar.Move(Forward, 180);
  }
}