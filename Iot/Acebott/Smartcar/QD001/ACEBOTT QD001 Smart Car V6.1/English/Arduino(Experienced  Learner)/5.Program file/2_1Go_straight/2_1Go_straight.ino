#include <vehicle.h>
vehicle myCar;
void setup() {
  myCar.Init();//Initialize all motors
  myCar.Move(Forward, 255);//Control car forward moving
  delay(3000);
  myCar.Move(Stop,0);//Control car stop.
}
void loop() {
}