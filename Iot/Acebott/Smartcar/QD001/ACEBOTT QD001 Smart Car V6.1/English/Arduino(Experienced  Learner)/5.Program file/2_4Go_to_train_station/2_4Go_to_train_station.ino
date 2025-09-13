#include <vehicle.h>

vehicle myCar;

void setup() {
  myCar.Init();//Initialize all motors
  myCar.Move(Forward, 255);//Control car forward moving
  delay(3000);
  myCar.Move(Clockwise , 255);//Control car counterclockwise rotate
  delay(750);//Modify the parameters in red
  myCar.Move(Forward, 255);//Control car forward moving
  delay(1500);
  myCar.Move(Clockwise , 255);//Control car counterclockwise rotate
  delay(750);//Modify the parameters in red
  myCar.Move(Forward, 255);//Control car forward moving
  delay(3000);
  myCar.Move(Contrarotate , 255);//Control car contrarotate
  delay(750);//Modify the parameters in red
  myCar.Move(Forward, 255);//Control car forward moving
  delay(1500);
  myCar.Move(Contrarotate , 255);//Control car contrarotate
  delay(750);//Modify the parameters in red
  myCar.Move(Forward, 255);//Control car forward moving
  delay(3000);
  myCar.Move(Move_Right , 255);//Control car left moving
  delay(750);//Modify the parameters in red
  myCar.Move(Stop,0);//Control car sto
}

void loop() {
  // put your main code here, to run repeatedly:

}
