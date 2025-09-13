#include <vehicle.h>
#include <IRremote.h>

int IRpin = 4;
unsigned long lastCommandTime; // Record the time of the last received command
const unsigned long commandTimeout = 100; // Set the timeout period (milliseconds)
uint32_t last_decode = 0; // Variable to store the previously decoded raw data
uint32_t current_decode = 0;// Variable to store the currently decoded raw data

IRrecv myIRrecv(IRpin); // Create IRrecv object for receiving IR signals
vehicle myCar;
int Speed = 255;
void setup() {
  Serial.begin(9600); // Set the serial baud rate to 9600
  myIRrecv.enableIRIn(); // Start IR decoding
  myCar.Init();
}

void loop() {
  if (myIRrecv.decode()) { // Check if an IR signal is received
    lastCommandTime = millis(); // Update the last command time
    current_decode = myIRrecv.decodedIRData.decodedRawData;
    if (myIRrecv.decodedIRData.flags) { // Check if it's a repeated IR code
      current_decode = last_decode; 
      // Set current decodedRawData as the previous one
      }
    Serial.print(current_decode, HEX);
    Serial.println("");
    switch (current_decode) {
      case 0xB946FF00: myCar.Move(Forward, Speed); break; 
      // Press "up" button to move forward
      case 0xEA15FF00: myCar.Move(Backward, Speed); break; 
      // Press "down" button to move backward
      case 0xBB44FF00: myCar.Move(Contrarotate, Speed); break; 
      // Press "left" button to turn left
      case 0xBC43FF00: myCar.Move(Clockwise, Speed); break; 
      // Press "right" button to turn right
      case 0xE916FF00: myCar.Move(Move_Left, Speed); break; 
      // Press button "1" to move left
      case 0xF20DFF00: myCar.Move(Move_Right, Speed); break; 
      // Press button "3" to move right
    }
    last_decode = current_decode; 
    // Update the stored previous decodedRawData
    myIRrecv.resume(); // Wait for the next IR signal
    }
  if (millis() - lastCommandTime > commandTimeout) {
    myCar.Move(Stop, 0); 
    // If no new IR signal within 100 milliseconds, stop the robot
  }
}