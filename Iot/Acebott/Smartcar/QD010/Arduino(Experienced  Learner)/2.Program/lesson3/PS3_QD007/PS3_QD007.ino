#include <WiFi.h>
#include "esp_camera.h"
#include <vehicle.h>
#include <ultrasonic.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <Ps3Controller.h>
#include <ACB_CAR_ARM.h>

ACB_CAR_ARM CAR_ARM;

int Chassis_input, Shoulder_input, Elbow_input, Wrist_input, Claws_input;
int Chassis_silde, Shoulder_silde, Elbow_silde, Wrist_silde, Claws_silde;
int PTPX, PTPY, PTPZ;  // Coordinate X Y Z
bool record = false;   // record mode
bool st = false;       // Record whether the APP is disconnected

// Servo angles for different parts of the system (angles of joints or actuators)
int angle1 = 90;  // Initial angle for the first joint (e.g., shoulder, chassis, etc.)
int angle2 = 40;  // Initial angle for the second joint (e.g., elbow, wrist, etc.)
int angle3 = 50;  // Initial angle for the third joint
int angle4 = 90;  // Initial angle for the fourth joint
int angle5 = 90;  // Initial angle for the fifth joint

#define Left_sensor 35   //Left   Line patrol Pin
#define Middle_sensor 36 //center Line patrol Pin
#define Right_sensor 39  //Right  Line patrol Pin

#define CMD_RUN 1      //Motion marker bit
#define CMD_STANDBY 3  //Task flag bit
#define CMD_TRACK_1 4  //Patrol duty 1 mode
#define CMD_TRACK_2 5  //Patrol duty 2 mode
#define CMD_AVOID 6    //Obstacle avoidance mode
#define CMD_FOLLOW 7   //Following mode

const char *ssid = "ESP32-CAR";  //Set WIFI name
const char *password = "12345678";   //Set WIFI password
String macAddress = "20:00:00:00:38:40";  // PS3 Bluetooth controller MAC address
WiFiServer server(100);              //Set server port
WiFiClient client;                   //client

vehicle Acebott;              // Car control object
ultrasonic Ultrasonic;        //Ultrasonic object
 
int Left_Tra_Value;           // Value from the left track sensor
int Middle_Tra_Value;         // Value from the middle track sensor
int Right_Tra_Value;          // Value from the right track sensor
int Black_Line = 2000;        // Threshold for detecting black line
int Off_Road = 4000;          // Threshold for detecting off-road (outside of
int speeds = 250;             // Default speed of the car
int leftDistance = 0;         // Left ultrasonic distance
int middleDistance = 0;       // Middle ultrasonic distance
int rightDistance = 0;        // Right ultrasonic distance

String sendBuff;  // Buffer for sending data
String Version = "Firmware Version is 2.0.0";
byte dataLen, index_a = 0;    // Data length and index
char buffer[52];              // Buffer for data storage
unsigned char prevc = 0;      // Previous character (used for UART communication)
bool isStart = false;         // Flag to indicate if the robot has started
bool ED_client = true;        // Flag indicating if the client is active
bool WA_en = false;           // Flag for enabling WA (could be related to
byte RX_package[17] = { 0 };  // Array for receiving data (17 bytes buffer)
byte action = Stop, device;   // Action state (e.g., Stop) and device identifier
byte val = 0;                 // A general-purpose variable, potentially for control or settings
char model_var = 0;           // Model version or specific mode variable
int UT_distance = 0;          // Ultrasonic sensor distance value
char claw ;

char Robot_arm_mode = 1 ;

// Function to read a byte from the buffer at a specified index
// Parameters:
//   index_r: The index in the buffer from which the byte will be read
// Returns:
//   The byte value at the specified index
unsigned char readBuffer(int index_r) {
  return buffer[index_r];
}

// Function to write a byte to the buffer at a specified index
// Parameters:
//   index_w: The index in the buffer where the byte will be written
//   c: The byte value to write to the buffer
void writeBuffer(int index_w, unsigned char c) {
  buffer[index_w] = c;
}
enum FUNCTION_MODE {
  STANDBY,
  FOLLOW,
  TRACK_1,
  TRACK_2,
  AVOID,
} function_mode;
char Ps3_Key_Steta[20];  // Array to store the state of the PS3 keys
void Ps3_Control()       // PS3 Bluetooth controller input and state reading
{
  // Assign a value to Ps3_Key_Steta
  // Check if a button is pressed and store its state
  if (Ps3.event.button_down.cross) strcpy(Ps3_Key_Steta, "cross");
  if (Ps3.event.button_down.square) strcpy(Ps3_Key_Steta, "square");
  if (Ps3.event.button_down.triangle) strcpy(Ps3_Key_Steta, "triangle");
  if (Ps3.event.button_down.circle) strcpy(Ps3_Key_Steta, "circle");

  if (Ps3.event.button_down.r1){ Robot_arm_mode = 0;runModuleBLU(11);} // stop 
  if (Ps3.event.button_down.l1){ Robot_arm_mode = 1;runModuleBLU(11);}// stop 

  if (Ps3.event.button_down.ps) strcpy(Ps3_Key_Steta, "Playstation");

  // Check if the L1 button is pressed, then set the key state to "left shoulder"
  if (Ps3.event.button_down.l1) strcpy(Ps3_Key_Steta, "left shoulder");
  // Check if the R1 button is pressed, then set the key state to "right shoulder"
  if (Ps3.event.button_down.r1) strcpy(Ps3_Key_Steta, "right shoulder");
  // Check if the select button is pressed, then set the key state to "select"
  if (Ps3.event.button_down.select) strcpy(Ps3_Key_Steta, "select");
  // Check if the start button is pressed, then set the key state to "start"
  if (Ps3.event.button_down.start) strcpy(Ps3_Key_Steta, "start");

if(Robot_arm_mode == 1){

    // Check if the sum of left stick movements exceeds a threshold
    if (abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 1) {

        memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));

        // Check if the left stick is in a specific position for turning left
        if ((Ps3.data.analog.stick.ly < 90 && Ps3.data.analog.stick.ly > -90) && Ps3.data.analog.stick.lx <= -90) {
            runModuleBLU(10);  // Turn left
        }

        // Check if the left stick is in a specific position for turning right
        if ((Ps3.data.analog.stick.ly < 90 && Ps3.data.analog.stick.ly > -90) && Ps3.data.analog.stick.lx >= 90) {
            runModuleBLU(9);  // Turn right
        }

        // Check if the left stick is in a specific position to move backward
        if ((Ps3.data.analog.stick.lx <= 90 && Ps3.data.analog.stick.lx >= -90) && Ps3.data.analog.stick.ly >= 90) {
            runModuleBLU(2);  // Move backward
        }

        // Check if the left stick is in a specific position to move forward
        if ((Ps3.data.analog.stick.lx <= 90 && Ps3.data.analog.stick.lx >= -90) && Ps3.data.analog.stick.ly <= -90) {
            runModuleBLU(1);  // Move forward
        }

        // Check if both sticks are centered and stop movement
        if ((Ps3.data.analog.stick.ly <= 20 && Ps3.data.analog.stick.ly >= -20) && (Ps3.data.analog.stick.lx <= 20 && Ps3.data.analog.stick.lx >= -20)) {
            runModuleBLU(11); // Stop
        }
    }

    // Check if the sum of right stick movements exceeds a threshold
    if (abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 1) {
        memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));
        delay(10);

        // Perform right spin if the right stick is moved to the right
        if (Ps3.data.analog.stick.rx > 110 && Ps3.data.analog.stick.ry == 0)
            runModuleBLU(4);  // Perform right spin

        // Perform left spin if the right stick is moved to the left
        if (Ps3.data.analog.stick.rx < -110 && Ps3.data.analog.stick.ry == 0)
            runModuleBLU(3);  // Perform left spin

        // Stop movement if both sticks are centered
        if ((Ps3.data.analog.stick.rx <= 40 && Ps3.data.analog.stick.rx >= -40) && (Ps3.data.analog.stick.ry <= 40 && Ps3.data.analog.stick.ry >= -40)) {
            runModuleBLU(11); // Stop
        }

        // Handle diagonal movements for the right stick
        if (Ps3.data.analog.stick.rx > 20 && Ps3.data.analog.stick.ry < -20) {
            runModuleBLU(7);  // Move right forward
        }

        if (Ps3.data.analog.stick.rx > 20 && Ps3.data.analog.stick.ry > 20) {
            runModuleBLU(8);  // Move right backward
        }

        if (Ps3.data.analog.stick.rx < -20 && Ps3.data.analog.stick.ry < -20) {
            runModuleBLU(5);  // Move left forward
        }

        if (Ps3.data.analog.stick.rx < -20 && Ps3.data.analog.stick.ry > 20) {
            runModuleBLU(6);  // Move left backward
        }
    }
}

else if(Robot_arm_mode == 0 ){

    // Check if the "right" button is pressed, set 'claw' to 1 if pressed, else set to 0
    if (Ps3.event.button_down.right) claw = 1;
    if (Ps3.event.button_up.right) claw = 0;

    // Check if the "left" button is pressed, set 'elbow' to 2 if pressed, else set to 0
    if (Ps3.event.button_down.left) claw = 2;
    if (Ps3.event.button_up.left) claw = 0;

    if ( (Ps3.data.analog.stick.ly < 90 && Ps3.data.analog.stick.ly > -90) && Ps3.data.analog.stick.lx <= -90  ) {
        Servocontrol(1);  // The arm chassis turns left 
    }

    if ( (Ps3.data.analog.stick.ly < 90 && Ps3.data.analog.stick.ly > -90) && Ps3.data.analog.stick.lx >= 90  ) {
        Servocontrol(6);  // The arm chassis turns right
    }

    if ( (Ps3.data.analog.stick.lx <= 90 && Ps3.data.analog.stick.lx >= -90) && Ps3.data.analog.stick.ly >= 90  ) {
        Servocontrol(7);  // The robotic arm should be lowered
    }

    if ( (Ps3.data.analog.stick.lx <= 90 && Ps3.data.analog.stick.lx >= -90) && Ps3.data.analog.stick.ly <= -90  ) {
        Servocontrol(2);  // The robotic arm should be raised
    }

    if ( (Ps3.data.analog.stick.ry < 90 && Ps3.data.analog.stick.ry > -90) && Ps3.data.analog.stick.rx <= -90  ) {
        Servocontrol(9);  // The robotic wrist is closed 
    }

    if ( (Ps3.data.analog.stick.ry < 90 && Ps3.data.analog.stick.ry > -90) && Ps3.data.analog.stick.rx >= 90  ) {
        Servocontrol(4); // The robotic wrist opens
    }

    if ( (Ps3.data.analog.stick.rx <= 90 && Ps3.data.analog.stick.rx >= -90) && Ps3.data.analog.stick.ry >= 90  ) {
        Servocontrol(3);  // The elbow of the manipulator decreases
    }

    if ( (Ps3.data.analog.stick.rx <= 90 && Ps3.data.analog.stick.rx >= -90) && Ps3.data.analog.stick.ry <= -90  ) {
        Servocontrol(8);  // The elbow of the robotic arm rises 
    }


}
  switch(claw){

    case 1 : Servocontrol(10); 
             if (angle5<=95)break;            // Robotic claw closed
      break;

    case 2 : Servocontrol(5); // Mechanical arm claw open
             if (angle5>=180)break;
      break;
  }

  if (strcmp(Ps3_Key_Steta, "Playstation") == 0) {  // Playstation button pressed

    Robot_arm_init();
    
    Ps3.end(1);
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));
  }

if (strcmp(Ps3_Key_Steta, "start") == 0) {  // Check if the Playstation "start" button is pressed

    Robot_arm_init();

    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));
}

  if (strcmp(Ps3_Key_Steta, "cross") == 0) {  // X button
    model3_func();                            // Following mode
  }
  if (strcmp(Ps3_Key_Steta, "square") == 0) {  // Square button
    model1_func();                             // Line tracking mode 1
  }
  if (strcmp(Ps3_Key_Steta, "triangle") == 0) {  // Triangle button
    model2_func();                               // Obstacle avoidance mode
  }
  if (strcmp(Ps3_Key_Steta, "circle") == 0) {  // Circle button
    model4_func();                             // Line tracking mode 2
  }
}

// Function to control the servo motors based on the speed parameter
void Servocontrol(int speed) {
  // Switch case based on the input 'speed' value
  switch (speed) {
    case 1:
      angle1 = angle1 + 1;
      if (angle1 >= 190) {
        angle1 = 190;
      }
      CAR_ARM.Silde_ChassisCmd(angle1);
      break;
    case 2:
      angle2 = angle2 + 1;
      if (angle2 >= 180) {
        angle2 = 180;
      }
      CAR_ARM.Silde_ShoulderCmd(angle2);

      break;
    case 3:
      angle3 = angle3 + 1;
      if (angle3 >= 180) {
        angle3 = 180;
      }
      CAR_ARM.Silde_ElbowCmd(angle3);

      break;
    case 4:
      angle4 = angle4 + 1;
      if (angle4 >= 180) {
        angle4 = 180;
      }
      CAR_ARM.Silde_WristCmd(angle4);

      break;
    case 5:
      angle5 = angle5 + 1;
      if (angle5 >= 180) {
        angle5 = 180;
      }
      CAR_ARM.Silde_ClawsCmd(angle5);

      break;
    case 6:
      angle1 = angle1 - 1;
      if (angle1 < 0) {
        angle1 = 0; 

      }
      CAR_ARM.Silde_ChassisCmd(angle1);

      break;
    case 7:
      angle2 = angle2 - 1;
      if (angle2 < 0) {
        angle2 = 0;
      }
      CAR_ARM.Silde_ShoulderCmd(angle2);

      break;
    case 8:
      angle3 = angle3 - 1;
      if (angle3 < 0) {
        angle3 = 0;
      }
      CAR_ARM.Silde_ElbowCmd(angle3);

      break;
    case 9:
      angle4 = angle4 - 1;
      if (angle4 < 0) {
        angle4 =0;
      }
      CAR_ARM.Silde_WristCmd(angle4);
      break;
    case 10:
      angle5 = angle5 - 1;
      if (angle5 <= 95) {
        angle5 = 95;
      }
      CAR_ARM.Silde_ClawsCmd(angle5);
      break;
  }
  delay(10);
}


void Robot_arm_init(){

    for(char i=0; i<120; i++){

    if(angle1>100){ angle1-=2;  CAR_ARM.Silde_ChassisCmd(angle1); }
    else if(angle1<100){ angle1+=2;  CAR_ARM.Silde_ChassisCmd(angle1); }

    if(angle2>40){ angle2-=1;  CAR_ARM.Silde_ShoulderCmd(angle2); }
    else if(angle2<40){ angle2+=1;  CAR_ARM.Silde_ShoulderCmd(angle2); }

    if(angle3>50){ angle3-=1;   CAR_ARM.Silde_ElbowCmd(angle3);  }
    else if(angle3<50){ angle3+=1;   CAR_ARM.Silde_ElbowCmd(angle3); }

    if(angle4>=90){ angle4-=2;   CAR_ARM.Silde_WristCmd(angle4);  }
    else if(angle4<=90){ angle4+=2;   CAR_ARM.Silde_WristCmd(angle4); }

    if(angle5>90){ angle5-=2;   CAR_ARM.Silde_ClawsCmd(angle5);   }
    else if(angle5<90){ angle5+=2;  CAR_ARM.Silde_ClawsCmd(angle5);  }
    }

}

void setup() {
  Serial.setTimeout(10);                       // Set the serial port timeout to 10 milliseconds
  Serial.begin(115200);                        // Initialize serial communication, baud rate is 115200

  Acebott.Init();                              // Initialize Acebott
  Ultrasonic.Init(13,14);                      // Initialize the ultrasound module
  CAR_ARM.ARM_init(25, 26, 27, 33, 4);         // servo initialize
       
  pinMode(Left_sensor, INPUT);                 // Set the infrared left line pin as input
  pinMode(Middle_sensor, INPUT);               // Set the infrared middle line pin as input
  pinMode(Right_sensor, INPUT);                // Set the infrared right line pin as input
             
       
  Acebott.Move(Stop, 0);                       // Stop Acebott Movement
  delay(3000);                                 // Delay 3 seconds
       
  WiFi.setTxPower(WIFI_POWER_19_5dBm);         // Set Wi-Fi transmit power to 19.5dBm
  WiFi.mode(WIFI_AP);                          // Set Wi-Fi working mode to access point mode
  WiFi.softAP(ssid, password, 5);              // Create a Wi-Fi access point, the SSID is ssid, the password is password, and the maximum number of connections is 5
  Serial.print("\r\n");
  Serial.print("Camera Ready! Use 'http://");  // Print prompt information
  Serial.print(WiFi.softAPIP());               // Print access point IP address
  Serial.println("' to connect");              // Print prompt information

  delay(100);
  server.begin();  // Start the server
  delay(1000);

  CAR_ARM.startAppServer();  // start app server
  Ps3.begin(macAddress.c_str());  // Initialize PS3 controller with MAC address
}

void loop() {
  RXpack_func();
  //model4_func();
}

void functionMode() {
  switch (function_mode) {
    case FOLLOW:
      {
        model3_func();  // Enter the follow mode and call the model3_func() function
      }
      break;
    case TRACK_1:
      {
        model1_func();  // Enter tracking mode 1 and call the model1_func() function
      }
      break;
    case TRACK_2:
      {
        model4_func();  // Enter tracking mode 2 and call the model4_func() function
      }
      break;
    case AVOID:
      {
        model2_func();  // Enter obstacle avoidance mode and call the model2_func() function
      }
      break;
    default:
      break;
  }
}

void Receive_data()  // Receive data
{
  if (client.available())  // If data is available
  {
    unsigned char c = client.read() & 0xff;  // Read one byte of data
    Serial.write(c);                         // Send received data on serial port
    if (c == 0x55 && isStart == false)       // If start flag 0x55 is received and data reception has not started yet
    {
      if (prevc == 0xff)  // If the previous byte is also the start flag 0xff
      {
        index_a = 1;     // Data index is set to 1
        isStart = true;  // Start receiving data
      }
    } else {
      prevc = c;    // Update the value of the previous byte
      if (isStart)  // If you have started receiving data
      {
        if (index_a == 2)  // If it is the second byte, it indicates the data length
        {
          dataLen = c;           // Update data length
        } else if (index_a > 2)  // If it is the subsequent byte
        {
          dataLen--;  // Data length minus one
        }
        writeBuffer(index_a, c);  // Write data to buffer
      }
    }
    index_a++;          // Index increase
    if (index_a > 120)  // If the index exceeds the upper limit
    {
      index_a = 0;      // reset index to 0
      isStart = false;  // End data reception
    }
    if (isStart && dataLen == 0 && index_a > 3)  // If data reception is completed
    {
      isStart = false;  // End data reception
      parseData();      // Analytical data
      index_a = 0;      // reset index to 0
    }
  }
}

void model1_func() {
  // Read the analog value from the left tracking sensor
  Left_Tra_Value = analogRead(Left_sensor);
  // Read the analog value from the right tracking sensor
  Right_Tra_Value = analogRead(Right_sensor);
  // Small delay to allow sensor readings to stabilize
  delay(5);

  // If both left and right sensors detect the black line
  if (Left_Tra_Value < Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Forward, 130);  // Move forward at speed 130
  }
  // If the left sensor detects the black line and the right sensor does not
  else if (Left_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Contrarotate, 150);  // Contrarotate at speed 150
  }
  // If the left sensor does not detect the black line and the right sensor does
  else if (Left_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line) {
    Acebott.Move(Clockwise, 150);  // Rotate clockwise at speed 150
  }
  // If both sensors detect they are on the track but not off-road
  else if (Left_Tra_Value >= Black_Line && Left_Tra_Value < Off_Road && Right_Tra_Value >= Black_Line && Right_Tra_Value < Off_Road) {
    Acebott.Move(Stop, 0);  // Stop the movement
  }
  // If both sensors detect they are off-road
  else if (Left_Tra_Value >= Off_Road && Right_Tra_Value >= Off_Road) {
    Acebott.Move(Stop, 0);  // Stop the movement
  }
}

void model2_func()  // OA
{
  // Get the distance measurement from the ultrasonic sensor
  UT_distance = Ultrasonic.Ranging();
  
  // Store the distance value in middleDistance
  middleDistance = UT_distance;

  // If the distance is less than or equal to 25 (close obstacle detected)
  if (middleDistance <= 25) {
    // Stop the robot's movement
    Acebott.Move(Stop, 0);
    // Wait for 100 milliseconds
    delay(100);
    
    // Generate a random number between 1 and 4
    int randNumber = random(1, 4);
    
    // Perform different actions based on the random number
    switch (randNumber) {
      case 1:
        // Move backward for 180 units and wait for 500ms
        Acebott.Move(Backward, 180);
        delay(500);
        // Move left for 180 units and wait for 500ms
        Acebott.Move(Move_Left, 180);
        delay(500);
        break;
      case 2:
        // Rotate clockwise for 180 degrees and wait for 1000ms
        Acebott.Move(Clockwise, 180);
        delay(1000);
        break;
      case 3:
        // Rotate counterclockwise for 180 degrees and wait for 1000ms
        Acebott.Move(Contrarotate, 180);
        delay(1000);
        break;
      case 4:
        // Move backward for 180 units and wait for 500ms
        Acebott.Move(Backward, 180);
        delay(500);
        // Move right for 180 units and wait for 500ms
        Acebott.Move(Move_Right, 180);
        delay(500);
        break;
    }
  } else {
    // If no obstacle is close, move forward for 180 units
    Acebott.Move(Forward, 180);
  }
}

void model3_func()  // follow model
{
  UT_distance = Ultrasonic.Ranging();
  //Serial.println(UT_distance);
  if (UT_distance < 15) {
    Acebott.Move(Backward, 200);
  } else if (15 <= UT_distance && UT_distance <= 20) {
    Acebott.Move(Stop, 0);
  } else if (20 <= UT_distance && UT_distance <= 25) {
    Acebott.Move(Forward, speeds - 70);
  } else if (25 <= UT_distance && UT_distance <= 50) {
    Acebott.Move(Forward, 220);
  } else {
    Acebott.Move(Stop, 0);
  }
}

void model4_func()  // Tracking model to follow the line
{
  // Read values from the left, middle, and right sensors
  Left_Tra_Value = analogRead(Left_sensor);
  Middle_Tra_Value = analogRead(Middle_sensor);
  Right_Tra_Value = analogRead(Right_sensor);
  
  // Small delay to avoid overloading the processor
  delay(5);

  // Check for different conditions based on the sensor readings

  // If the middle sensor detects the black line and both the left and right sensors detect the light color
  if (Left_Tra_Value < Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    // Move the robot forward
    Acebott.Move(Forward, 180);
  }

  // If the middle sensor detects the black line and the left sensor detects the light color
  if (Left_Tra_Value < Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line) {
    // Move the robot forward
    Acebott.Move(Forward, 180);
  }

  // If the middle sensor detects the black line and the right sensor detects the light color
  if (Left_Tra_Value >= Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    // Move the robot forward
    Acebott.Move(Forward, 180);
  }

  // If the left sensor detects the black line and the middle and right sensors detect the light color
  else if (Left_Tra_Value >= Black_Line && Middle_Tra_Value < Black_Line && Right_Tra_Value < Black_Line) {
    // Rotate the robot counterclockwise to adjust its path
    Acebott.Move(Contrarotate, 220);
  }

  // If the right sensor detects the black line and the left and middle sensors detect the light color
  else if (Left_Tra_Value < Black_Line && Middle_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line) {
    // Rotate the robot clockwise to adjust its path
    Acebott.Move(Clockwise, 220);
  }

  // If all sensors detect the light color (off the line completely)
  else if (Left_Tra_Value >= Off_Road && Middle_Tra_Value >= Off_Road && Right_Tra_Value >= Off_Road) {
    // Stop the robot as it is off the track
    Acebott.Move(Stop, 0);
  }
}

void runModuleBLU(int device)  // Car movement control function
{
  // This function is responsible for controlling the movement of the car
  // based on the device input (which corresponds to different movement actions).

  switch (device) {
    case 1:
      // Move forward with the selected speed
      Acebott.Move(Forward, speeds);
      delay(50);  // Delay for smooth movement
      break;
    case 2:
      // Move backward with the selected speed
      Acebott.Move(Backward, speeds);
      delay(50);
      break;
    case 3:
      // Turn left with the selected speed
      Acebott.Move(Move_Left, speeds);
      delay(50);
      break;
    case 4:
      // Turn right with the selected speed
      Acebott.Move(Move_Right, speeds);
      delay(50);
      break;
    case 5:
      // Move diagonally top-left with the selected speed
      Acebott.Move(Top_Left, speeds);
      delay(50);
      break;
    case 6:
      // Move diagonally bottom-left with the selected speed
      Acebott.Move(Bottom_Left, speeds);
      delay(50);
      break;
    case 7:
      // Move diagonally top-right with the selected speed
      Acebott.Move(Top_Right, speeds);
      delay(50);
      break;
    case 8:
      // Move diagonally bottom-right with the selected speed
      Acebott.Move(Bottom_Right, speeds);
      delay(50);
      break;
    case 9:
      // Perform a clockwise rotation with the selected speed
      Acebott.Move(Clockwise, speeds);
      delay(50);
      break;
    case 10:
      // Perform a counter-clockwise rotation with the selected speed
      Acebott.Move(Contrarotate, speeds);
      delay(50);
      break;
    case 11:
      // Stop the car
      Acebott.Move(Stop, 0);  // Stop the car with speed 0
      delay(50);
      break;
  }
}

void runModule(int device) {
  val = readBuffer(12);
  switch (device) {
    case 0x0C:
      {
        switch (val) {
          case 0x01:
            Acebott.Move(Forward, speeds);
            break;
          case 0x02:
            Acebott.Move(Backward, speeds);
            break;
          case 0x03:
            Acebott.Move(Move_Left, speeds);
            break;
          case 0x04:
            Acebott.Move(Move_Right, speeds);
            break;
          case 0x05:
            Acebott.Move(Top_Left, speeds);
            break;
          case 0x06:
            Acebott.Move(Bottom_Left, speeds);
            break;
          case 0x07:
            Acebott.Move(Top_Right, speeds);
            break;
          case 0x08:
            Acebott.Move(Bottom_Right, speeds);
            break;
          case 0x0A:
            Acebott.Move(Clockwise, speeds);
            break;
          case 0x09:
            Acebott.Move(Contrarotate, speeds);
            break;
          case 0x00:
            Acebott.Move(Stop, 0);
            break;
          default:
            break;
        }
      }
      break;
    case 0x0D:
      {
        speeds = val;
      }
      break;
  }
}
void parseData() {
  isStart = false;
  int action = readBuffer(9);
  int device = readBuffer(10);
  switch (action) {
    case CMD_RUN:
      //callOK_Len01();
      function_mode = STANDBY;
      runModule(device);
      break;
    case CMD_STANDBY:
      //callOK_Len01();
      function_mode = STANDBY;
      Acebott.Move(Stop, 0);
      break;
    case CMD_TRACK_1:
      //callOK_Len01();
      function_mode = TRACK_1;
      //Serial.write(0x01);
      break;
    case CMD_TRACK_2:
      //callOK_Len01();
      function_mode = TRACK_2;
      break;
    case CMD_AVOID:
      //callOK_Len01();
      function_mode = AVOID;
      break;
    case CMD_FOLLOW:
      //callOK_Len01();
      function_mode = FOLLOW;
      break;
    default: break;
  }
}

void RXpack_func()  //Receive data
{
  client = server.available();  // Waiting for client to connect
  if (client)                   // If there is a client connection
  {
    WA_en = true;                                // enable write enable
    ED_client = true;                            // Client connection flag set to true
    Serial.println("[Client connected]");        // Print client connection information
    unsigned long previousMillis = millis();     // Last check time
    const unsigned long timeoutDuration = 2000;  // time out 2 second
    while (client.connected())                   // While the client is still connected
    {
      // Check if the current value of CAR_ARM is 3 (no action taken)
      if (CAR_ARM.val == 3) {

      }

      // Check if CAR_ARM.val is 21, indicating the chassis angle should be adjusted
      else if (CAR_ARM.val == 21) {
          // Get the chassis slide angle input from the CAR_ARM object
          Chassis_input = CAR_ARM.Chassis_Silde_Angle;
          // Command the chassis to adjust to the new angle
          CAR_ARM.ChassisCmd(Chassis_input);
          // Update the chassis angle state
          CAR_ARM.chassis_angle = Chassis_input;
          // Set CAR_ARM.val back to 3 to indicate the action is complete
          CAR_ARM.val = 3;
      }
      // Check if CAR_ARM.val is 22, indicating the shoulder angle should be adjusted
      else if (CAR_ARM.val == 22) {
          // Get the shoulder slide angle input from the CAR_ARM object
          Shoulder_input = CAR_ARM.Shoulder_Silde_Angle;
          // Command the shoulder to adjust to the new angle
          CAR_ARM.ShoulderCmd(Shoulder_input);
          // Update the shoulder angle state
          CAR_ARM.shoulder_angle = Shoulder_input;
          // Set CAR_ARM.val back to 3 to indicate the action is complete
          CAR_ARM.val = 3;
      }
      // Check if CAR_ARM.val is 23, indicating the elbow angle should be adjusted
      else if (CAR_ARM.val == 23) {
          // Get the elbow slide angle input from the CAR_ARM object
          Elbow_input = CAR_ARM.Elbow_Silde_Angle;
          // Command the elbow to adjust to the new angle
          CAR_ARM.ElbowCmd(Elbow_input);
          // Update the elbow angle state
          CAR_ARM.elbow_angle = Elbow_input;
          // Set CAR_ARM.val back to 3 to indicate the action is complete
          CAR_ARM.val = 3;
      }
      // Check if CAR_ARM.val is 24, indicating the claws angle should be adjusted
      else if (CAR_ARM.val == 24) {
          // Get the claws slide angle input from the CAR_ARM object
          Claws_input = CAR_ARM.Claws_Silde_Angle;
          // Command the claws to adjust to the new angle
          CAR_ARM.ClawsCmd(Claws_input);
          // Update the claws angle state
          CAR_ARM.claws_angle = Claws_input;
          // Set CAR_ARM.val back to 3 to indicate the action is complete
          CAR_ARM.val = 3;
      }
      // Check if CAR_ARM.val is 29, indicating the wrist angle should be adjusted
      else if (CAR_ARM.val == 29) {
          // Get the wrist slide angle input from the CAR_ARM object
          Wrist_input = CAR_ARM.Wrist_Silde_Angle;
          // Command the wrist to adjust to the new angle
          CAR_ARM.WristCmd(Wrist_input);
          // Update the wrist angle state
          CAR_ARM.wrist_angle = Wrist_input;
          // Set CAR_ARM.val back to 3 to indicate the action is complete
          CAR_ARM.val = 3;
      }

      // Check for sliding adjustments
      // If CAR_ARM.val is 25, adjust the chassis slide angle
      else if (CAR_ARM.val == 25) {
          // Get the chassis slide angle input from the CAR_ARM object
          Chassis_silde = CAR_ARM.Chassis_Silde_Angle;
          // Command the chassis to adjust to the new slide angle
          CAR_ARM.Silde_ChassisCmd(Chassis_silde);
          // Update the chassis angle state
          CAR_ARM.chassis_angle = Chassis_silde;
          // Set CAR_ARM.val back to 3 to indicate the action is complete
          CAR_ARM.val = 3;
      }
      // If CAR_ARM.val is 26, adjust the shoulder slide angle
      else if (CAR_ARM.val == 26) {
          // Get the shoulder slide angle input from the CAR_ARM object
          Shoulder_silde = CAR_ARM.Shoulder_Silde_Angle;
          // Command the shoulder to adjust to the new slide angle
          CAR_ARM.Silde_ShoulderCmd(Shoulder_silde);
          // Update the shoulder angle state
          CAR_ARM.shoulder_angle = Shoulder_silde;
          // Set CAR_ARM.val back to 3 to indicate the action is complete
          CAR_ARM.val = 3;
      }
      // If CAR_ARM.val is 27, adjust the elbow slide angle
      else if (CAR_ARM.val == 27) {
          // Get the elbow slide angle input from the CAR_ARM object
          Elbow_silde = CAR_ARM.Elbow_Silde_Angle;
          // Command the elbow to adjust to the new slide angle
          CAR_ARM.Silde_ElbowCmd(Elbow_silde);
          // Update the elbow angle state
          CAR_ARM.elbow_angle = Elbow_silde;
          // Set CAR_ARM.val back to 3 to indicate the action is complete
          CAR_ARM.val = 3;
      }
      // If CAR_ARM.val is 28, adjust the claws slide angle
      else if (CAR_ARM.val == 28) {
          // Get the claws slide angle input from the CAR_ARM object
          Claws_silde = CAR_ARM.Claws_Silde_Angle;
          // Command the claws to adjust to the new slide angle
          CAR_ARM.Silde_ClawsCmd(Claws_silde);
          // Update the claws angle state
          CAR_ARM.claws_angle = Claws_silde;
          // Set CAR_ARM.val back to 3 to indicate the action is complete
          CAR_ARM.val = 3;
      }
      // If CAR_ARM.val is 30, adjust the wrist slide angle
      else if (CAR_ARM.val == 30) {
          // Get the wrist slide angle input from the CAR_ARM object
          Wrist_silde = CAR_ARM.Wrist_Silde_Angle;
          // Command the wrist to adjust to the new slide angle
          CAR_ARM.Silde_WristCmd(Wrist_silde);
          // Update the wrist angle state
          CAR_ARM.wrist_angle = Wrist_silde;
          // Set CAR_ARM.val back to 3 to indicate the action is complete
          CAR_ARM.val = 3;
      }
      // Mode control
      else if (CAR_ARM.val == 31 && record) {  // save
        CAR_ARM.saveState();
        delay(200);
        CAR_ARM.val = 3;
      } else if (CAR_ARM.val == 32) {  // end Record
        record = true;
      } else if (CAR_ARM.val == 33) {  // start Record
        record = false;
      } else if (CAR_ARM.val == 34 && !record) {  // Run
        CAR_ARM.executeStates();
        delay(200);
        CAR_ARM.val = 3;
      } else if (CAR_ARM.val == 35 && !record) {  // Reset
        CAR_ARM.clearSavedStates();
        delay(200);
        CAR_ARM.val = 3;
      }

      // Mode select 1-6
      else if (CAR_ARM.val == 40) {    // 0
        CAR_ARM.mode = 0;
      } else if (CAR_ARM.val == 41) {  // 1
        CAR_ARM.mode = 1;
      } else if (CAR_ARM.val == 42) {  // 2
        CAR_ARM.mode = 2;
      } else if (CAR_ARM.val == 43) {  // 3
        CAR_ARM.mode = 3;
      } else if (CAR_ARM.val == 44) {  // 4
        CAR_ARM.mode = 4;
      } else if (CAR_ARM.val == 45) {  // 5
        CAR_ARM.mode = 5;
      } else if (CAR_ARM.val == 46) {  // 6
        CAR_ARM.mode = 6;
      }

      else if (CAR_ARM.val == 54) {  // ptp
        CAR_ARM.PtpCmd(CAR_ARM.PTP_X, CAR_ARM.PTP_Y, CAR_ARM.PTP_Z);
        delay(100);
        CAR_ARM.val = 3;
      }

      if ((millis() - previousMillis) > timeoutDuration && client.available() == 0 && st == true) {
        break;
      }

      if (client.available())  // If there is data to read
      {
        previousMillis = millis();
        unsigned char c = client.read() & 0xff;  // Read data
        Serial.write(c);                         // Print received data
        Serial.println(c);
        st = false;
        if (c == 200) {
          st = true;
        }

        if (c == 0x55 && isStart == false)  // If the received data is 0x55 and isStart is false
        {
          if (prevc == 0xff)  // If the previous byte is 0xff
          {
            index_a = 1;     // Index is set to 1
            isStart = true;  // Data start flag is set to true
          }
        } else {
          prevc = c;    // Update the value of the previous byte
          if (isStart)  // If data start flag is true
          {
            if (index_a == 2)  // if index is 2
            {
              dataLen = c;           // The data length is set to c
            } else if (index_a > 2)  // if index is greater than 2
            {
              dataLen--;  // Data length minus 1
            }
            writeBuffer(index_a, c);  // Write data to buffer
          }
        }
        index_a++;          // Index increases by 1
        if (index_a > 120)  // If the index is greater than 120
        {
          index_a = 0;      // Index reset to 0
          isStart = false;  // Data start flag is set to false
        }
        if (isStart && dataLen == 0 && index_a > 3)  // If the data start flag is true and the data length is 0 and the index is greater than 3
        {
          isStart = false;  // Data start flag is set to false
          parseData();      // Analytical data
          index_a = 0;      // Index is set to 0
        }
      }
      functionMode();          // Function pattern processing
      if (Serial.available())  // If there is data in the serial port, it can be read
      {
        char c = Serial.read();  // Read data
        sendBuff += c;           // Add data to send buffer
        client.print(sendBuff);  // Send data to client
        Serial.print(sendBuff);  // Print sent data
        sendBuff = "";           // Clear send buffer
      }
    }
    client.stop();  // Disconnect client
    Acebott.Move(Stop, 0);
    Serial.println("[Client disconnected]");  // Print client disconnect information
  } else  // If no client is connected
  {
    if (ED_client == true)  // If there was a client connection before
    {
      ED_client = false;  // Client connection flag set to false
    }
  }
  if (!Ps3.isConnected()) {      // if ps3 no connected, try to receconnect the car
    Ps3.begin(macAddress.c_str());
    delay(1000);
  } else {
    Ps3_Control();   // ps3 control function
  }
}
