#include <vehicle.h>
#include <ultrasonic.h>

vehicle myCar;
ultrasonic myUltrasonic;
int UT_distance = 0;

void setup(){ 
  myCar.Init();
  myUltrasonic.Init(13,14);
}

void loop()
{
  UT_distance = myUltrasonic.Ranging();
  if (UT_distance <= 25){
  //The distance is less than 25cm to achieve the effect of turning
    myCar.Move(Contrarotate, 180);
    delay(1500); //The turning time is modified to realize the rotation of different angles
    myCar.Move(Stop, 0);
  } 
  else {
  //If the distance is greater than 25, move forward
  myCar.Move(Forward, 150);
  }
}