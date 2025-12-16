#include <WiFi.h>
#include "esp_camera.h"
#include <vehicle.h>
#include <ultrasonic.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <Ps3Controller.h>
#include <time.h>

#define FIXED_SERVO_PIN 25  //ltrasonic chassis
#define TURN_SERVO_PIN 26   //solar panel

#define LED_Module1 2       //Left LED Pin
#define LED_Module2 12      //Right LED Pin
#define Left_sensor 35      //Left   Line patrol Pin
#define Middle_sensor 36    //Middle Line patrol Pin
#define Right_sensor 39     //Right  Line patrol Pin
#define Buzzer 33           //Buzzer Pin

#define CMD_RUN 1           //Motion marker bit
#define CMD_STANDBY 3       //Task flag bit
#define CMD_TRACK_1 4       //Patrol duty 1 mode
#define CMD_TRACK_2 5       //Patrol duty 2 mode
#define CMD_AVOID 6         //Obstacle avoidance mode
#define CMD_FOLLOW 7        //Following mode

//app music
#define C3 131
#define D3 147
#define E3 165
#define F3 175
#define G3 196
#define A3 221
#define B3 248

#define C4 262
#define D4 294
#define E4 330
#define F4 350
#define G4 393
#define A4 441
#define B4 495

#define C5 525
#define D5 589
#define E5 661
#define F5 700
#define G5 786
#define A5 882
#define B5 990
#define N 0

const char *ssid = "ESP32-CAR";     //Set WIFI name
const char *password = "12345678";  //Set WIFI password
String macAddress = "20:00:00:00:38:40";  // PS3 Bluetooth controller MAC address
WiFiServer server(100);             //Set server port
WiFiClient client;                  //client

vehicle Acebott;                    //car
ultrasonic Ultrasonic;              //ultrasonic
Servo fixedServo;                   //Servo
Servo turnServo;                    //shooting servo


// Left trajectory value
int Left_Tra_Value;
// Middle trajectory value
int Middle_Tra_Value;
// Right trajectory value
int Right_Tra_Value;
// Threshold value for black line detection (indicates where the black line is located)
int Black_Line = 2000;
// Threshold value for off-road detection (indicates when the sensor detects off-road area)
int Off_Road = 4000;
// Speed value for the motor or robot movement
int speeds = 250;
// Distance traveled by the left wheel or sensor
int leftDistance = 0;
// Distance traveled by the middle wheel or sensor
int middleDistance = 0;
// Distance traveled by the right wheel or sensor
int rightDistance = 0;

String sendBuff;              // Buffer for sending data
String Version = "Firmware Version is 0.12.21";
byte dataLen, index_a = 0;    // Data length and index
char buffer[52];              // Buffer for data storage
unsigned char prevc = 0;      // Previous character (used for UART communication)
bool isStart = false;         // Flag to indicate if the robot has started
bool ED_client = true;        // Flag indicating if the client is active
bool WA_en = false;           // Flag for enabling WA (could be related to
byte RX_package[17] = { 0 };  // Array for receiving data (17 bytes buffer)
uint16_t angle = 90;          // Robot's initial angle (set to 90 degrees)
byte action = Stop, device;   // Action state (e.g., Stop) and device identifier
byte val = 0;                 // A general-purpose variable, potentially for control or settings
char model_var = 0;           // Model version or specific mode variable
int UT_distance = 0;          // Ultrasonic sensor distance value

char Speed_Set_State = 2;     // PS3 Bluetooth controller speed state
char angles = 90;             // Initial setting of gun Angle
char current_angles = 90;     // Current gun Angle
bool rstate = false;          // servo move state
bool lstate = false;          // servo move state

int length0;
int length1;
int length2;
int length3;
/*****app music*****/
// littel star
int tune0[] = { C4, N, C4, G4, N, G4, A4, N, A4, G4, N, F4, N, F4, E4, N, E4, D4, N, D4, C4 };
float durt0[] = { 0.99, 0.01, 1, 0.99, 0.01, 1, 0.99, 0.01, 1, 1.95, 0.05, 0.99, 0.01, 1, 0.99, 0.01, 1, 0.99, 0.01, 1, 2 };
// jingle bell
int tune1[] = { E4, N, E4, N, E4, N, E4, N, E4, N, E4, N, E4, G4, C4, D4, E4 };
float durt1[] = { 0.49, 0.01, 0.49, 0.01, 0.99, 0.01, 0.49, 0.01, 0.49, 0.01, 0.99, 0.01, 0.5, 0.5, 0.75, 0.25, 1, 2 };
// happy new year
int tune2[] = { C5, N, C5, N, C5, G4, E5, N, E5, N, E5, C5, N, C5, E5, G5, N, G5, F5, E5, D5, N };
float durt2[] = { 0.49, 0.01, 0.49, 0.01, 1, 1, 0.49, 0.01, 0.49, 0.01, 1, 0.99, 0.01, 0.5, 0.5, 0.99, 0.01, 1, 0.5, 0.5, 1, 1 };
// have a farm
int tune3[] = { C4, N, C4, N, C4, G3, A3, N, A3, G3, E4, N, E4, D4, N, D4, C4 };
float durt3[] = { 0.99, 0.01, 0.99, 0.01, 1, 1, 0.99, 0.01, 1, 2, 0.99, 0.01, 1, 0.99, 0.01, 1, 1 };
/*****app music*****/

char Ps3_Key_Steta[20];  // Array to store the state of the PS3 keys

unsigned long lastDataTimes = 0;
bool st = false;

enum FUNCTION_MODE {
  STANDBY,
  FOLLOW,
  TRACK_1,
  TRACK_2,
  AVOID,
} function_mode;

unsigned char readBuffer(int index_r) {
  return buffer[index_r];
}
void writeBuffer(int index_w, unsigned char c) {
  buffer[index_w] = c;
}

void Ps3_Control()       // PS3 Bluetooth controller and read
{
  // Assign a value to Ps3_Key_Steta
  // Check if a button is pressed and store its state
  if (Ps3.event.button_down.cross) strcpy(Ps3_Key_Steta, "cross");      // Cross button pressed
  if (Ps3.event.button_down.square) strcpy(Ps3_Key_Steta, "square");    // Square button pressed
  if (Ps3.event.button_down.triangle) strcpy(Ps3_Key_Steta, "triangle");// Triangle button pressed
  if (Ps3.event.button_down.circle) strcpy(Ps3_Key_Steta, "circle");    // Circle button pressed
  if (Ps3.event.button_down.up) strcpy(Ps3_Key_Steta, "up");            // Up button pressed
  if (Ps3.event.button_down.right) strcpy(Ps3_Key_Steta, "right");      // Right button pressed
  if (Ps3.event.button_down.down) strcpy(Ps3_Key_Steta, "down");        // Down button pressed
  if (Ps3.event.button_down.left) strcpy(Ps3_Key_Steta, "left");        // Left button pressed

  // Check if L1 button is press
  if (Ps3.event.button_down.l1) {
    lstate = true;
  }
  // Check if L1 button is released
  if (Ps3.event.button_up.l1) {
    lstate = false;
    Servo_MoveBLU(angles);  // servo move
  }
  // Check if R1 button is pressed
  if (Ps3.event.button_down.r1) {
    rstate = true;
  }
  // Check if R1 button is released
  if (Ps3.event.button_up.r1) {
    rstate = false;
    Servo_MoveBLU(angles);  // servo move
  }
// If right state (rstate) is true
if (rstate) {
  // If the angle is greater than 10, decrease it gradually
  if (angles > 10) {
    angles -= 1;  // Decrease the angle by 1 unit
    // Once the angle is reduced to 10 or below, set the angle to 10 and turn off the right state (rstate)
    if (angles <= 10) {
      angles = 10;  // Ensure the angle doesn't go below 10
      rstate = false;  // Set rstate to false to stop further reduction
    }
  }
  delay(20);  // Wait for 20 milliseconds before checking again
}

// If left state (lstate) is true
if (lstate) {
  // If the angle is less than 170, increase it gradually
  if (angles < 170) {
    angles += 1;  // Increase the angle by 1 unit
    // Once the angle reaches 170 or exceeds it, set the angle to 170 and turn off the left state (lstate)
    if (angles >= 170) {
      angles = 170;  // Ensure the angle doesn't exceed 170
      lstate = false;  // Set lstate to false to stop further increase
    }
  }
  delay(20);  // Wait for 20 milliseconds before checking again
}
  if (Ps3.event.button_down.l2) strcpy(Ps3_Key_Steta, "left trigger");
  if (Ps3.event.button_down.r2) strcpy(Ps3_Key_Steta, "right trigger");
  if (Ps3.event.button_down.select) strcpy(Ps3_Key_Steta, "select");
  if (Ps3.event.button_down.ps) strcpy(Ps3_Key_Steta, "Playstation");

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


/*************************************************  The tank track version needs to block the subprogram *************************************************************/
/*************************************************  Because it doesn't require a right stick to operate  *************************************************************/
  // Check if the right stick is moved
    if (abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 1) {
        memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));

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


/*******************************************************************     END      ***********************************************************************************/

 
  if (strcmp(Ps3_Key_Steta, "Playstation") == 0) {  // Playstation button pressed
    Ps3.end(1);
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));
  }
  if (strcmp(Ps3_Key_Steta, "cross") == 0) {  // X button
    model3_func();                            // Following mode
  }
  if (strcmp(Ps3_Key_Steta, "square") == 0) {  // Square button
    model1_func();                             // Line tracking mode 1
  }
  if (strcmp(Ps3_Key_Steta, "triangle") == 0) {  // Triangle button
    function_mode = AVOID;
    model2_func();                               // Obstacle avoidance mode
  }

  if (strcmp(Ps3_Key_Steta, "circle") == 0) {  // Circle button
    model4_func();                             // Line tracking 2
  }
  if (strcmp(Ps3_Key_Steta, "down") == 0) {       // Left down
    int currentState = digitalRead(LED_Module2);  // Read current LED state
    digitalWrite(LED_Module2, !currentState);     // Toggle LED state
    digitalWrite(LED_Module1, !currentState);
    delay(250);
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));
  }
  if (strcmp(Ps3_Key_Steta, "up") == 0) {  // Left up
    Buzzer_runBLU();
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));
  }
  if (strcmp(Ps3_Key_Steta, "select") == 0) {         // Select button
    Speed_Set_State++;                                // Increment the speed setting state
    Speed_Set_State %= 5;                             // Cycle through speed levels (0-4)
    Speed_State(Speed_Set_State);                     // Apply the new speed setting
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));  // Clear button state
  }
}

void setup() {
  Serial.setTimeout(10);  // Set the serial port timeout to 10 milliseconds
  Serial.begin(115200);   // Initialize serial communication, baud rate is 115200

  Acebott.Init();     // Initialize Acebott
  Ultrasonic.Init(13,14);  // Initialize the Ultrasonic Module

  pinMode(LED_Module1, OUTPUT);  // Set  LED module1 as output
  pinMode(LED_Module2, OUTPUT);  // Set  LED module2 as output
  pinMode(Left_sensor, INPUT);     // Set the infrared left line pin as input
  pinMode(Middle_sensor, INPUT);   // Set the infrared middle line pin as input
  pinMode(Right_sensor, INPUT);    // Set the infrared right line pin as input

  ESP32PWM::allocateTimer(1);  // Assign timer 1 to ESP32PWM library
  fixedServo.attach(FIXED_SERVO_PIN);   // Connect the servo to the FIXED_SERVO_PIN pin
  fixedServo.write(angle);         // Set the servo angle to angle
  turnServo.attach(TURN_SERVO_PIN);   // Connect the servo to the TURN_SERVO_PIN pin
  turnServo.write(angle);         // Set the servo angle to angle
  Acebott.Move(Stop, 0);       // Stop Acebott Movement
  delay(3000);                 // Delay 3 seconds

  length0 = sizeof(tune0) / sizeof(tune0[0]);  // Calculate the length of tune0 array
  length1 = sizeof(tune1) / sizeof(tune1[0]);  // Calculate the length of tune1 array
  length2 = sizeof(tune2) / sizeof(tune2[0]);  // Calculate the length of tune2 array
  length3 = sizeof(tune3) / sizeof(tune3[0]);  // Calculate the length of tune3 array

  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Set Wi-Fi transmit power to 19.5dBm
  WiFi.mode(WIFI_AP);                   // Set Wi-Fi working mode to access point mode
  WiFi.softAP(ssid, password, 5);       // Create a Wi-Fi access point, the SSID is ssid, the password is password, and the maximum number of connections is 5
  Serial.print("\r\n");
  Serial.print("Camera Ready! Use 'http://");  // Print prompt message
  Serial.print(WiFi.softAPIP());               // Print access point IP address
  Serial.println("' to connect");              // Print prompt message

  delay(100);
  server.begin();  // Starting the server
  delay(1000);

  Ps3.begin(macAddress.c_str());
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
    lastDataTimes = millis(); // 
    if (c == 200)
    {
      st = false;
    }
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
    index_a++;                    // Index increase
    if (index_a > 120)            // If the index exceeds the upper limit
    {          
      index_a = 0;                // reset index to 0
      isStart = false;            // End data reception
    }
    if (isStart && dataLen == 0 && index_a > 3)  // If data reception is completed
    {
      isStart = false;            // End data reception
      parseData();                // Analytical data
      index_a = 0;                // reset index to 0
    }          
  }
  if (client.available() == 0 && (millis() - lastDataTimes)>3000)
    {
      st = true;
    }
}

void model2_func()  // OA (Obstacle Avoidance)
{
  // Set the servo to 90 degrees
  fixedServo.write(90);

  // Measure the distance using the ultrasonic sensor
  UT_distance = Ultrasonic.Ranging();
  middleDistance = UT_distance;

  // If the distance is less than or equal to 25 cm, begin obstacle avoidance
  if (middleDistance <= 25) {
    // Stop the robot
    Acebott.Move(Stop, 0);

    // Wait for 500 milliseconds while receiving data
    for (int i = 0; i < 500; i++) {
      delay(1);
      Receive_data();  // Check for received data

      // If the function mode is not AVOID  flag is set, return
      if (function_mode != AVOID )
        return;
    }

    // Move the servo to 45 degrees
    fixedServo.write(45);

    // Wait for 300 milliseconds while receiving data
    for (int i = 0; i < 300; i++) {
      delay(1);
      Receive_data();
      if (function_mode != AVOID )
        return;
    }

    // Measure right distance using ultrasonic sensor
    rightDistance = Ultrasonic.Ranging();

    // Move the servo to 135 degrees
    fixedServo.write(135);

    // Wait for 300 milliseconds while receiving data
    for (int i = 0; i < 300; i++) {
      delay(1);
      Receive_data();
      if (function_mode != AVOID )
        return;
    }

    // Measure left distance using ultrasonic sensor
    leftDistance = Ultrasonic.Ranging();

    // Reset servo position to 90 degrees
    fixedServo.write(90);

    // If both right and left distances are less than 10 cm, reverse and rotate
    if ((rightDistance < 10) && (leftDistance < 10)) {
      Acebott.Move(Backward, 180);  // Move backward for 180 degrees
      for (int i = 0; i < 1000; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID )
          return;
      }
      Acebott.Move(Contrarotate, 180);  // Rotate in reverse for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID )
          return;
      }
    }
    // If the right distance is less than the left distance, move backward and rotate
    else if (rightDistance < leftDistance) {
      Acebott.Move(Backward, 180);  // Move backward for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID )
          return;
      }
      Acebott.Move(Contrarotate, 180);  // Rotate in reverse for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID )
          return;
      }
    }  
    // If the right distance is greater than the left distance, move backward and rotate clockwise
    else if (rightDistance > leftDistance) {
      Acebott.Move(Backward, 180);  // Move backward for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID )
          return;
      }
      Acebott.Move(Clockwise, 180);  // Rotate clockwise for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID )
          return;
      }
    } else {
      // If distances are equal, move backward and rotate clockwise
      Acebott.Move(Backward, 180);  // Move backward for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID )
          return;
      }
      Acebott.Move(Clockwise, 180);  // Rotate clockwise for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID ) 
          return;
      }
    }
  } else {
    // If there is no obstacle, move forward
    Acebott.Move(Forward, 150);
  }
}

void model3_func()  // Follow model
{
  fixedServo.write(90);  // Set servo to 90 degrees

  UT_distance = Ultrasonic.Ranging();  // Measure the distance using the ultrasonic sensor

  // If the detected distance is less than 15 cm, move backward
  if (UT_distance < 15) {
    Acebott.Move(Backward, 200);
  } 
  // If the distance is between 15 cm and 20 cm, stop the robot
  else if (15 <= UT_distance && UT_distance <= 20) {
    Acebott.Move(Stop, 0);
  } 
  // If the distance is between 20 cm and 25 cm, move forward with reduced speed
  else if (20 <= UT_distance && UT_distance <= 25) {
    Acebott.Move(Forward, speeds - 70);  // Adjust the speed by subtracting 70
  } 
  // If the distance is between 25 cm and 50 cm, move forward with full speed (220)
  else if (25 <= UT_distance && UT_distance <= 50) {
    Acebott.Move(Forward, 220);
  } 
  // If the distance is greater than 50 cm, stop the robot
  else {
    Acebott.Move(Stop, 0);
  }
}

void model4_func()  // Tracking model based on sensor values
{
  fixedServo.write(90);  // Set servo to 90 degrees to position the sensor

  // Read values from the left, middle, and right sensors
  Left_Tra_Value = analogRead(Left_sensor);
  Middle_Tra_Value = analogRead(Middle_sensor);
  Right_Tra_Value = analogRead(Right_sensor);
  
  delay(5);  // Delay for stability in sensor reading

  // If the left and right sensors are on the white line and the middle sensor is on the black line, move forward
  if (Left_Tra_Value < Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Forward, 180);
  }

  // If the left sensor is on the white line and the middle and right sensors are on the black line, move forward
  if (Left_Tra_Value < Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line) {
    Acebott.Move(Forward, 180);
  }

  // If the left sensor is on the black line and the right sensor is on the white line, move forward
  if (Left_Tra_Value >= Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Forward, 180);
  }

  // If the left sensor is on the black line and the middle and right sensors are on the white line, rotate counterclockwise
  else if (Left_Tra_Value >= Black_Line && Middle_Tra_Value < Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Contrarotate, 220);  // Counter-clockwise movement
  } 
  
  // If the right sensor is on the black line and the left and middle sensors are on the white line, rotate clockwise
  else if (Left_Tra_Value < Black_Line && Middle_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line) {
    Acebott.Move(Clockwise, 220);  // Clockwise movement
  }

  // If all sensors detect the road is off-track (values higher than Off_Road threshold), stop the robot
  else if (Left_Tra_Value >= Off_Road && Middle_Tra_Value >= Off_Road && Right_Tra_Value >= Off_Road) {
    Acebott.Move(Stop, 0);  // Stop the robot
  }
}

void model1_func()  // Tracking model1
{
  //fixedServo.write(90); // Servo positioning is disabled

  // Read sensor values from the left and right sensors
  Left_Tra_Value = analogRead(Left_sensor);
  Right_Tra_Value = analogRead(Right_sensor);

  delay(5);  // Delay to stabilize sensor readings

  // If both left and right sensors detect the white line (values less than Black_Line), move forward
  if (Left_Tra_Value < Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Forward, 130);
  } 
  // If only the left sensor detects the black line (value greater than or equal to Black_Line), rotate counterclockwise
  else if (Left_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Contrarotate, 150);
  } 
  // If only the right sensor detects the black line (value greater than or equal to Black_Line), rotate clockwise
  else if (Left_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line) {
    Acebott.Move(Clockwise, 150);
  } 
  // If both sensors are on the white line but within an acceptable range (i.e., not out of bounds), stop the robot
  else if (Left_Tra_Value >= Black_Line && Left_Tra_Value < Off_Road && Right_Tra_Value >= Black_Line && Right_Tra_Value < Off_Road) {
    Acebott.Move(Stop, 0);
  } 
  // If both sensors detect an "off-road" condition (values exceed Off_Road), stop the robot
  else if (Left_Tra_Value >= Off_Road && Right_Tra_Value >= Off_Road) {
    Acebott.Move(Stop, 0);
  }
}

// Function to smoothly move the servo to a specified angle
// The movement speed is adjusted based on the angle difference
// Parameters:
//   angle: The target angle to which the servo should move (between 1 and 180 degrees)
void Servo_MoveBLU(int angle)  // Servo movement function
{
  // Check if the difference between the target angle and the current angle is large
  if (abs(angle - current_angles) > 40) {
    // Map the target angle to a PWM value for the servo (with a different range for larger movements)
    int pwmValue = map(angle, 1, 180, 130, 70);
    int currentPwm = turnServo.read();  // Get the current PWM value of the servo
    // Gradually move the servo in 15 steps for a smooth transition
    for (int j = 0; j < 20; j++) {
      // Calculate the new PWM value incrementally
      int newPwm = currentPwm + (pwmValue - currentPwm) * (j / 20.0);
      turnServo.write(newPwm);  // Write the new PWM value to the servo
      delay(15);                // Wait for 15 milliseconds before the next step
    }
  } else {
    // Map the target angle to a PWM value for the servo (with a different range for smaller movements)
    int pwmValue = map(angle, 1, 180, 130, 70);
    int currentPwm = turnServo.read();  // Get the current PWM value of the servo
    // Gradually move the servo in 10 steps for a smoother transition
    for (int j = 0; j < 15; j++) {
      // Calculate the new PWM value incrementally
      int newPwm = currentPwm + (pwmValue - currentPwm) * (j / 15.0);
      turnServo.write(newPwm);  // Write the new PWM value to the servo
      delay(10);                // Wait for 10 milliseconds before the next step
    }
  }
  current_angles = angle;  // Update the current angle to the target angle
}
void Servo_Move(int angles)  // Accepts an angle value (angles) to control the servo's movement
{
  // Use the map function to map the input angle (1 to 180) to the corresponding PWM value (130 to 70)
  int pwmValue = map(angles, 1, 180, 130, 70);
  
  // Get the current position of the servo, which is its current PWM value
  int currentPwm = turnServo.read();
  
  // If the target PWM value is greater than the current PWM value, the servo needs to move to a higher position
  if (pwmValue > currentPwm) {
    // Gradually transition to the target PWM value in 20 steps
    for (int j = 0; j < 20; j++) {
      // Calculate the incremental PWM value for smooth transition
      int newPwm = currentPwm + (pwmValue - currentPwm) * (j / 20.0);
      // Set the new PWM value to control the servo's movement
      turnServo.write(newPwm);
      // Delay for 20 milliseconds to allow the servo to move smoothly
      delay(20);
    }
  } else {
    // If the target PWM value is less than the current PWM value, the servo needs to move to a lower position
    for (int j = 0; j < 15; j++) {
      // Calculate the incremental PWM value for smooth transition
      int newPwm = currentPwm + (pwmValue - currentPwm) * (j / 15.0);
      // Set the new PWM value to control the servo's movement
      turnServo.write(newPwm);
      // Delay for 20 milliseconds to allow the servo to move smoothly
      delay(20);
    }
  }
}
void Music_a() {  // Play tune a on the buzzer
  for (int x = 0; x < length0; x++) {
    tone(Buzzer, tune0[x]);  // Play the note at the frequency specified in tune0 array
    delay(500 * durt0[x]);   // Delay for a duration based on the durt0 array (500ms per unit)
    noTone(Buzzer);          // Stop the tone
  }
}
void Music_b() {  // Play tune b on the buzzer
  for (int x = 0; x < length1; x++) {
    tone(Buzzer, tune1[x]);  // Play the note at the frequency specified in tune1 array
    delay(500 * durt1[x]);   // Delay for a duration based on the durt1 array (500ms per unit)
    noTone(Buzzer);          // Stop the tone
  }
}
void Music_c() {  // Play tune c on the buzzer
  for (int x = 0; x < length2; x++) {
    tone(Buzzer, tune2[x]);  // Play the note at the frequency specified in tune2 array
    delay(500 * durt2[x]);   // Delay for a duration based on the durt2 array (500ms per unit)
    noTone(Buzzer);          // Stop the tone
  }
}
void Music_d() {  // Play tune d on the buzzer
  for (int x = 0; x < length3; x++) {
    tone(Buzzer, tune3[x]);  // Play the note at the frequency specified in tune3 array
    delay(300 * durt3[x]);   // Delay for a duration based on the durt3 array (300ms per unit)
    noTone(Buzzer);          // Stop the tone
  }
}
void Buzzer_run(int M) {  // Run the buzzer with the specified tune
  switch (M) {
    case 0x01:
      Music_a();  // Play tune a
      break;
    case 0x02:
      Music_b();  // Play tune b
      break;
    case 0x03:
      Music_c();  // Play tune c
      break;
    case 0x04:
      Music_d();  // Play tune d
      break;
    default:
      break;  // Do nothing if the input is not recognized
  }
}

void Buzzer_runBLU()  // Function to play a random song using the buzzer
{
  // Randomly select one of the five songs to play
  int count = rand() % 5;  // Generate a random number between 0 and 4
  switch (count) {
    case 0:
      Music_a();  // Play song A
      break;
    case 1:
      Music_b();  // Play song B
      break;
    case 2:
      Music_c();  // Play song C
      break;
    case 3:
      Music_d();  // Play song D
      break;
    default:
      break;  // No song if count is 4
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
    case 0x02:
      {
        Servo_Move(val);
      }
      break;
    case 0x03:
      {
        Buzzer_run(val);
      }
      break;
    case 0x05:
      {
        digitalWrite(LED_Module1, val);
        digitalWrite(LED_Module2, val);
      }
      break;
    case 0x0D:
      {
        speeds = val;
      }
      break;
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
void Speed_State(int speed) {  // Function to switch the speed based on the input value
  // The input 'speed' determines which speed is selected.
  // The speeds range from 120 to 255.
  switch (speed) {
    case 0:
      speeds = 120;  // Set speed to 120 for 'speed' 0
      delay(200);    // Delay for 200 milliseconds for smooth transition
      break;
    case 1:
      speeds = 150;  // Set speed to 150 for 'speed' 1
      delay(200);    // Delay for 200 milliseconds
      break;
    case 2:
      speeds = 180;  // Set speed to 180 for 'speed' 2
      delay(200);    // Delay for 200 milliseconds
      break;
    case 3:
      speeds = 210;  // Set speed to 210 for 'speed' 3
      delay(200);    // Delay for 200 milliseconds
      break;
    case 4:
      speeds = 255;  // Set speed to 250 for 'speed' 4
      delay(200);    // Delay for 200 milliseconds
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
      fixedServo.write(90);
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
  client = server.available();                  // Waiting for client to connect
  if (client)                                   // If there is a client connection
  {     
    WA_en = true;                               // enable write enable
    ED_client = true;                           // Client connection flag set to true
    Serial.println("[Client connected]");       // Print client connection information
    unsigned long previousMillis = millis();   
    const unsigned long timeoutDuration = 2000; 
    while (client.connected())                  // While the client is still connected
    {
      if ((millis() - previousMillis) > timeoutDuration && client.available() == 0 && st==true)
      {
        break;
      }
      if (client.available())  // If there is data to read
      {
        previousMillis = millis();
        unsigned char c = client.read() & 0xff;  // Read data
        Serial.write(c);                         // Print received data
        st = false;
        if (c == 200)
        {
          st = true;
        }
        if (c == 0x55 && isStart == false)       // If the received data is 0x55 and isStart is false
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
      functionMode();            // Function pattern processing
      if (Serial.available())    // If there is data in the serial port, it can be read
      {
        char c = Serial.read();  // Read data
        sendBuff += c;           // Add data to send buffer
        client.print(sendBuff);  // Send data to client
        Serial.print(sendBuff);  // Print sent data
        sendBuff = "";           // Clear send buffer
      }
    }
    client.stop();    
    Acebott.Move(Stop, 0);                        // Disconnect client
    Serial.println("[Client disconnected]");  // Print client disconnect information
  } else                                      // If no client is connected
  {
    if (ED_client == true)  // If there was a client connection before
    {
      ED_client = false;  // Client connection flag set to false
    }
  }
  if (!Ps3.isConnected()) {
    Ps3.begin(macAddress.c_str());  // PS3 Bluetooth controller begin
    delay(1000);
  } else {
    Ps3_Control();  // ps3 control
  }
}
