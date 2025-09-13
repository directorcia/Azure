#include <vehicle.h>
vehicle myCar;
void setup() {
  myCar.Init();//Initialize all motors
  myCar.Move(Clockwise , 255);//Control car counterclockwise rotate
  delay(750);//Modify the parameters in red
  myCar.Move(Stop,0);//Control car stop.
}
void loop() {
}