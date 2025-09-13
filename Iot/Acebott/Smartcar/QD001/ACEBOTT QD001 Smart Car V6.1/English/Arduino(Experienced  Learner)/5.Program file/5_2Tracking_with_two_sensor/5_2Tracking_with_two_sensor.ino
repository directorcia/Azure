#include <vehicle.h>

#define Left_sensor 35 //declare the pin of left tracking sensor
#define Right_sensor 39 //declare the pin of right tracking sensor

int Left_Tra_Value;
int Right_Tra_Value;
int Black_Line = 2000;
int Speed = 150;
int RotateSpeed = 150;
vehicle myCar;

void setup()
{
  pinMode(Left_sensor, INPUT);
  pinMode(Right_sensor, INPUT);
  myCar.Init();
}
void loop()
{
  Left_Tra_Value = analogRead(Left_sensor);
  Right_Tra_Value = analogRead(Right_sensor);
  delay(10);
  if (Left_Tra_Value < Black_Line && Right_Tra_Value < Black_Line)
  {//Both sensors are on the black line
    myCar.Move(Forward, Speed);//Smart car forward
  }
  else if (Left_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
  {//left sensor is on black line,right sensor is on white background
    myCar.Move(Contrarotate, RotateSpeed);//Smart car turn left
  }
  else if (Left_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line)
  {//right sensor is on black line,left sensor is on the white background
    myCar.Move(Clockwise, RotateSpeed);//Smart car turn right
  }
  else if (Left_Tra_Value >= Black_Line&& Right_Tra_Value >= Black_Line )
  {//Both sensors are on the white background
    myCar.Move(Stop, 0);//Smart car stop
  }
}