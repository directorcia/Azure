#include <vehicle.h>
#include <Ps3Controller.h>//Import Bluetooth controller's library file

vehicle myCar;
int Speed = 255;//car speed

void setup() {
  myCar.Init();
  Ps3.begin("20:00:00:00:38:40");//Modified according to the pairing code on the back of your Bluetooth controller
}

void loop() {
   if (Ps3.data.analog.stick.lx > 110 && Ps3.data.analog.stick.ly == 0) {
      myCar.Move(Move_Right, Speed);  // Turn right
    }
  else if (Ps3.data.analog.stick.lx < -110 && Ps3.data.analog.stick.ly == 0) {
      myCar.Move(Move_Left, Speed);  // Turn left
    }
   else if (Ps3.data.analog.stick.lx == 0 && Ps3.data.analog.stick.ly >= 110) {
      myCar.Move(Backward, Speed);  // Move backward
    }
   else if (Ps3.data.analog.stick.lx == 0 && Ps3.data.analog.stick.ly <= -110) {
      myCar.Move(Forward, Speed);  // Move forward
    }

   else if ((Ps3.data.analog.stick.ly <= 40 && Ps3.data.analog.stick.ly >= -40) && (Ps3.data.analog.stick.lx <= 40 && Ps3.data.analog.stick.lx >= -40)) 
   {
      myCar.Move(Stop, 0);  // Move Stop
    }
  delay(100);
}