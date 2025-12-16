#include <WiFi.h>
#include "esp_camera.h"
#include <vehicle.h>
#include <ultrasonic.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <Ps3Controller.h>
#include <time.h>

#define Yservo_PIN 25 //Ultrasonic chassis

#define LED_Module1 2  //Left LED Pin
#define LED_Module2 12 //Right LED Pin
#define Left_Line 35   //Left   Line patrol Pin
#define Center_Line 36 //center Line patrol Pin
#define Right_Line 39  //Right  Line patrol Pin
#define Buzzer 33      //Buzzer Pin

#define CMD_RUN 1      //Motion marker bit
#define CMD_STANDBY 3  //Task flag bit
#define CMD_TRACK_1 4  //Patrol duty 1 mode
#define CMD_TRACK_2 5  //Patrol duty 2 mode
#define CMD_AVOID 6    //Obstacle avoidance mode
#define CMD_FOLLOW 7   //Following mode

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

const char *ssid = "ESP32-CAR";         //Set WIFI name
const char *password = "12345678";        //Set WIFI password
String macAddress = "20:00:00:00:38:40";  // PS3 Bluetooth controller MAC address
WiFiServer server(100);                   //Setting the server port
WiFiClient client;                        //Client side
vehicle Acebott;                          // Car control object
ultrasonic Ultrasonic;                    //Ultrasonic object
Servo Yservo;                             // Servo object

int Left_Tra_Value;      // Value from the left track sensor
int Center_Tra_Value;    // Value from the center track sensor
int Right_Tra_Value;     // Value from the right track sensor
int Black_Line = 2000;   // Threshold for detecting black line
int Off_Road = 4000;     // Threshold for detecting off-road (outside of
int speeds = 250;        // Default speed of the car
int leftDistance = 0;    // Left ultrasonic distance
int middleDistance = 0;  // Middle ultrasonic distance
int rightDistance = 0;   // Right ultrasonic distance

String sendBuff;  // Buffer for sending data
String Version = "Firmware Version is 2.0.0";
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
bool st = false;              // Record whether the APP is disconnected
int count = 0;                // General-purpose counter
char Buzzer_count ;

char Speed_Set_State = 2;  // PS3 Bluetooth controller speed state

int length0;
int length1;
int length2;
int length3;
/*****app music note*****/
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
/*****app music note*****/

char Ps3_Key_Steta[20];  // Array to store the state of the PS3 keys

unsigned long lastDataTimes = 0;  // Record the last time

// Function to read a byte from the buffer at a specified index
// Parameters:
//   index_r: The index in the buffer from which the byte will be read
// Returns:
//   The byte value at the specified index
unsigned char readBuffer(int index_r) {
  return buffer[index_r];
}

enum FUNCTION_MODE {
  STANDBY,
  FOLLOW,
  TRACK_1,
  TRACK_2,
  AVOID,
} function_mode;

// Function to write a byte to the buffer at a specified index
// Parameters:
// index_w: The index in the buffer where the byte will be written
// c: The byte value to write to the buffer
void writeBuffer(int index_w, unsigned char c) {
  buffer[index_w] = c;
}

void Ps3_Control()  // PS3 Bluetooth controller input and state reading
{
  // Assign a value to Ps3_Key_Steta
  // Check if a button is pressed and store its state
  if (Ps3.event.button_down.cross) strcpy(Ps3_Key_Steta, "cross");        // Cross button pressed
  if (Ps3.event.button_down.square) strcpy(Ps3_Key_Steta, "square");      // Square button pressed
  if (Ps3.event.button_down.triangle) strcpy(Ps3_Key_Steta, "triangle");  // Triangle button pressed
  if (Ps3.event.button_down.circle) strcpy(Ps3_Key_Steta, "circle");      // Circle button pressed
  if (Ps3.event.button_down.up) strcpy(Ps3_Key_Steta, "up");              // Up button pressed
  if (Ps3.event.button_down.down) strcpy(Ps3_Key_Steta, "down");          // Down button pressed
  if (Ps3.event.button_down.left) strcpy(Ps3_Key_Steta, "left");          // Left button pressed
  if (Ps3.event.button_down.right) strcpy(Ps3_Key_Steta, "right");        // Right button pressed
  if (Ps3.event.button_down.select) strcpy(Ps3_Key_Steta, "select");      // select button pressed 
  if (Ps3.event.button_down.ps) strcpy(Ps3_Key_Steta, "Playstation");     // PS button pressed

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

  if (strcmp(Ps3_Key_Steta, "Playstation") == 0) {    // Playstation button pressed
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));  // Clear the button state
    delay(10);                                        // Wait for 10 milliseconds
    Ps3.end(1);                                       // End the connection with the PS3 controller
    Serial.println("disconnect");
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta)); 
  }

  if (strcmp(Ps3_Key_Steta, "cross") == 0) {          // Cross button (X button)
    model3_func();                                    // Switch to following mode
  }

  if (strcmp(Ps3_Key_Steta, "square") == 0) {         // Square button
    model1_func();                                    // Switch to line tracking mode 1
  }

  if (strcmp(Ps3_Key_Steta, "triangle") == 0) {       // Triangle button
    function_mode = AVOID;
    model2_func();                                    // Switch to obstacle avoidance mode
  }

  if (strcmp(Ps3_Key_Steta, "circle") == 0) {         // Circle button
    model4_func();                                    // Switch to line tracking mode 2
  }


  if (strcmp(Ps3_Key_Steta, "down") == 0) {           // Left down (Directional button press)
    int currentState = digitalRead(LED_Module2);      // Read current state of the LED module
    digitalWrite(LED_Module2, !currentState);         // Toggle LED module 2 state
    digitalWrite(LED_Module1, !currentState);         // Toggle LED module 1 state
    delay(250);                                       // Wait for 250 milliseconds
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));  // Clear button state
  }

  if (strcmp(Ps3_Key_Steta, "up") == 0) {             // Left up (Directional button press)
    Buzzer_count++;
    Buzzer_count%=4;
    Buzzer_runBLU(Buzzer_count);                       // Activate the buzzer
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));  // Clear button state
  }

  if (strcmp(Ps3_Key_Steta, "select") == 0) {         // Select button
    Speed_Set_State++;                                // Increment the speed setting state
    Speed_Set_State %= 5;                             // Cycle through speed levels (0-4)
    Speed_State(Speed_Set_State);                     // Apply the new speed setting
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));  // Clear button state
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
void Buzzer_runBLU(char count)  // Function to play a random song using the buzzer
{
  // Randomly select one of the five songs to play
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


void setup() {
  Serial.setTimeout(10);  // Set the serial port timeout to 10 milliseconds
  Serial.begin(115200);   // Serial communication is initialized with a baud rate of 115200

  Acebott.Init();     // Initialize Acebott
  Ultrasonic.Init(13,14);  // Initialize the ultrasonic module

  pinMode(LED_Module1, OUTPUT);  // Set pin of LED module as output
  pinMode(LED_Module2, OUTPUT);
  pinMode(Left_Line, INPUT);    // Set infrared left line pin as input
  pinMode(Center_Line, INPUT);  // Set the infrared middle line pin as input
  pinMode(Right_Line, INPUT);   // Set the right infrared line pin as input

  ESP32PWM::allocateTimer(1);  // Assign timer 1 to the ESP32PWM library
  Yservo.attach(Yservo_PIN);   // Connect the servo to the Yservo_PIN pin
  Yservo.write(angle);         // Set the steering angle as Angle
  Acebott.Move(Stop, 0);       // Stop the Acebott exercise
  delay(3000);

  length0 = sizeof(tune0) / sizeof(tune0[0]);  // Calculate the length of the tune0 array
  length1 = sizeof(tune1) / sizeof(tune1[0]);  // Calculate the length of the tune1 array
  length2 = sizeof(tune2) / sizeof(tune2[0]);  // Calculate the length of the tune2 array
  length3 = sizeof(tune3) / sizeof(tune3[0]);  // Calculate the length of the tune3 array

  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // The Wi-Fi transmit power is set to 19.5dBm
  WiFi.mode(WIFI_AP);                   // Set the Wi-Fi operating mode to access point mode
  WiFi.softAP(ssid, password, 5);       // Create a Wi-Fi access point with SSID as ssid, password as password, and maximum number of connections as 5
  Serial.print("\r\n");
  Serial.print("Camera Ready! Use 'http://");  // Printing a prompt
  Serial.print(WiFi.softAPIP());               // Print the access point IP address
  Serial.println("' to connect");              // Printing a prompt

  delay(100);
  server.begin();  // Starting the server
  delay(1000);

  Ps3.begin(macAddress.c_str());
}

void loop() {
  RXpack_func();
}

void functionMode() {
  switch (function_mode) {
    case FOLLOW:
      {
        model3_func();  // Enter follow mode and call the model3_func() function
      }
      break;
    case TRACK_1:
      {
        model1_func();  // Go into trace mode 1 and call the model1_func() function
      }
      break;
    case TRACK_2:
      {
        model4_func();  // Enter trace mode 2 and call the model4_func() function
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
void Receive_data() {  // Receiving data


  if (client.available())  // If data is available
  {
    unsigned char c = client.read() & 0xff;  // Read a byte of data
    Serial.write(c);                         // Send the received data on the serial port
    // Serial.println(c);
    lastDataTimes = millis();
    if (c == 200) {
      st = false;
    }

    if (c == 0x55 && isStart == false)  // If the start flag 0x55 is received and data has not yet been received
    {
      if (prevc == 0xff)  // If the previous byte is also the start flag 0xff
      {
        index_a = 1;     // The data index is set to 1
        isStart = true;  // Start receiving data
      }
    } else {
      prevc = c;    // Update the previous byte's value
      if (isStart)  // If data has already been received
      {
        if (index_a == 2)  // If it is the second byte, it is the length of the data
        {
          dataLen = c;           // Update data length
        } else if (index_a > 2)  // If it's a subsequent byte
        {
          dataLen--;  // The data length is decremented by one
        }
        writeBuffer(index_a, c);  // Writes data to the buffer
      }
    }
    index_a++;          // Index increase
    if (index_a > 120)  // If the index exceeds the upper limit
    {
      index_a = 0;      // Reset the index to 0
      isStart = false;  // End of data reception
    }
    if (isStart && dataLen == 0 && index_a > 3)  // If the data is received
    {
      isStart = false;  // End of data reception
      parseData();      // Parsing data
      index_a = 0;      // Reset the index to 0
    }
  }

  if (client.available() == 0 && (millis() - lastDataTimes) > 3000) {
    st = true;
  }
}
void model4_func() {  // Tracking model 4 (also referred to as model 2 in the function name)
  // Set servo to the middle position (90 degrees)
  Yservo.write(90);
  // Read the values from the left, center, and right line sensors
  Left_Tra_Value = analogRead(Left_Line);
  Center_Tra_Value = analogRead(Center_Line);
  Right_Tra_Value = analogRead(Right_Line);

  // Small delay to stabilize sensor readings
  delay(5);

  // If the center sensor detects the black line, and both left and right sensors do not detect it, move forward
  if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Forward, 180);  // Move forward with speed 180
  }

  // If the left sensor does not detect the black line, the center sensor detects the black line, and the right sensor does not, move forward
  if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line) {
    Acebott.Move(Forward, 180);  // Move forward with speed 180
  }

  // If the left sensor does not detect the black line, the center sensor detects the black line, and the right sensor detects the black line, move forward
  if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Forward, 180);  // Move forward with speed 180
  }

  // If the left and right sensors are off the black line, but the center sensor is on the black line, perform a counter-rotation to adjust
  else if (Left_Tra_Value >= Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Contrarotate, 220);  // Perform a counter-clockwise rotation with speed 220
  }

  // If the left sensor is off the black line, the center sensor is off the black line, and the right sensor is on the black line, perform a clockwise rotation
  else if (Left_Tra_Value < Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line) {
    Acebott.Move(Clockwise, 220);  // Perform a clockwise rotation with speed 220
  }

  // If all sensors detect values that indicate they are off the black line, stop the robot
  else if (Left_Tra_Value >= Off_Road && Center_Tra_Value >= Off_Road && Right_Tra_Value >= Off_Road) {
    Acebott.Move(Stop, 0);  // Stop the robot
  }
}
void model3_func() {  // follow model

  Yservo.write(90);
  UT_distance = Ultrasonic.Ranging();
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
void model2_func() {  // OA (Obstacle Avoidance) function

  // Set the servo to 90 degrees
  Yservo.write(90);
  // Get the distance from the ultrasonic sensor
  UT_distance = Ultrasonic.Ranging();
  middleDistance = UT_distance;

  // If the distance to the object is less than or equal to 25 cm
  if (middleDistance <= 25) {
    st = false;             // Set state to false (likely a flag to stop other operations)
    Acebott.Move(Stop, 0);  // Stop the robot

    // Wait for 500 ms while checking for incoming data
    for (int i = 0; i < 500; i++) {
      delay(1);        // Delay 1 ms
      Receive_data();  // Check for incoming data
      if (function_mode != AVOID)
        return;  // Exit if the function mode is not AVOID  
    }

    // Move the servo to 45 degrees
    Yservo.write(45);
    // Wait for 300 ms while checking for incoming data
    for (int i = 0; i < 300; i++) {
      delay(1);
      Receive_data();
      if (function_mode != AVOID)
        return;  // Exit if the function mode is not AVOID  
    }

    // Get the distance to the right of the robot
    rightDistance = Ultrasonic.Ranging();

    // Move the servo to 135 degrees (turning right)
    Yservo.write(135);
    // Wait for 300 ms while checking for incoming data
    for (int i = 0; i < 300; i++) {
      delay(1);
      Receive_data();
      if (function_mode != AVOID )
        return;  // Exit if the function mode is not AVOID  
    }

    // Get the distance to the left of the robot
    leftDistance = Ultrasonic.Ranging();

    // Move the servo back to 90 degrees (center position)
    Yservo.write(90);

    // If both the right and left distances are less than 10 cm, move backward and then rotate
    if ((rightDistance < 10) && (leftDistance < 10)) {
      Acebott.Move(Backward, 180);  // Move backward for 180 degrees
      for (int i = 0; i < 1000; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID)
          return;  // Exit if the function mode is not AVOID  
      }

      Acebott.Move(Contrarotate, 180);  // Rotate in the opposite direction for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID  )
          return;  // Exit if the function mode is not AVOID  
      }
    }
    // If the right distance is less than the left, turn left
    else if (rightDistance < leftDistance) {
      Acebott.Move(Backward, 180);  // Move backward for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID  )
          return;  // Exit if the function mode is not AVOID  
      }

      Acebott.Move(Contrarotate, 180);  // Rotate in the opposite direction for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID  )
          return;  // Exit if the function mode is not AVOID  
      }
    }
    // If the right distance is greater than the left, turn right
    else if (rightDistance > leftDistance) {
      Acebott.Move(Backward, 180);  // Move backward for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID  )
          return;  // Exit if the function mode is not AVOID  
      }

      Acebott.Move(Clockwise, 180);  // Rotate clockwise for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID  )
          return;  // Exit if the function mode is not AVOID  
      }
    }
    // If the distances are equal, do the same as turning right
    else {
      Acebott.Move(Backward, 180);  // Move backward for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID )
          return;  // Exit if the function mode is not AVOID  
      }

      Acebott.Move(Clockwise, 180);  // Rotate clockwise for 180 degrees
      for (int i = 0; i < 500; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID)
          return;  // Exit if the function mode is not AVOID  
      }
    }
  } else {
    Acebott.Move(Forward, 150);  // Move forward with speed 150 if no obstacle is detected within 25 cm
  }
}
void model1_func() {
  // Read the analog value from the left tracking sensor
  Left_Tra_Value = analogRead(Left_Line);
  // Read the analog value from the right tracking sensor
  Right_Tra_Value = analogRead(Right_Line);
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
void Servo_Move(int angles) {
  // Set the servo to the specified angle
  Yservo.write(angles);
  // Limit the angle to the range of 1 to 180 degrees
  if (angles >= 180) angles = 180;
  if (angles <= 1) angles = 1;
  // Small delay to allow the servo to reach the position
  delay(10);
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
      Yservo.write(90);
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

void RXpack_func()  //Receiving data
{
  client = server.available();  // Wait for the client to connect
  if (client)                   // If there is a client connection
  {
    WA_en = true;                                // Enable the write enable
    ED_client = true;                            // The client connection flag is set to true
    Serial.println("[Client connected]");        // Print client connection information
    unsigned long previousMillis = millis();     // Last inspection time
    const unsigned long timeoutDuration = 3000;  // Timeout period (3 seconds)
    while (client.connected())                   // While the client is still connected
    {

      if ((millis() - previousMillis) > timeoutDuration && client.available() == 0 && st == true) {
        break;
      }
      if (client.available())  // If there is data to read
      {
        previousMillis = millis();
        unsigned char c = client.read() & 0xff;  // Reading data
        Serial.write(c);                         // Print the received data
        // Serial.println(c);
        st = false;
        if (c == 200) {
          st = true;
        }
        if (c == 0x55 && isStart == false)  // If the data received is 0x55 and isStart is false
        {
          if (prevc == 0xff)  // If the previous byte is 0xff
          {
            index_a = 1;     // The index is set to 1
            isStart = true;  // The data start flag is set to true
          }
        } else {
          prevc = c;    // Update the previous byte's value
          if (isStart)  // If the data start flag is true
          {
            if (index_a == 2)  // If the index is 2
            {
              dataLen = c;           // The data length is set to c
            } else if (index_a > 2)  // If the index is greater than 2
            {
              dataLen--;  // The data length is decremented by 1
            }
            writeBuffer(index_a, c);  // Writes data to the buffer
          }
        }
        index_a++;          // Index incremented by 1
        if (index_a > 120)  // If the index is greater than 120
        {
          index_a = 0;      // The index is reset to 0
          isStart = false;  // The data start flag is set to false
        }
        if (isStart && dataLen == 0 && index_a > 3)  // If the data start flag is true and the data length is 0 and the index is greater than 3
        {
          isStart = false;  // The data start flag is set to false
          parseData();      // Parsing data
          index_a = 0;      // The index is set to 0
        }
      }
      functionMode();  // Function-pattern processing

      if (Serial.available())  // If the serial port has data to read
      {
        char c = Serial.read();  // Reading data
        sendBuff += c;           // Add the data to the send buffer
        client.print(sendBuff);  // Send the data to the client
        // Serial.print(sendBuff);  // Print the data sent
        sendBuff = "";  // Clear the send buffer
      }
    }
    client.stop();  // Disconnect the client
    Acebott.Move(Stop, 0);
    Serial.println("[Client disconnected]");  // Prints client disconnection information
  } else                                      // If there is no client connection
  {
    if (ED_client == true)  // If there was a previous client connection
    {
      ED_client = false;  // The client connection flag is set to false
    }
  }
  if (!Ps3.isConnected()) {
    Ps3.begin(macAddress.c_str());  // PS3 Bluetooth controller begin
    delay(1000);
  } else {
    Ps3_Control();  // ps3 control
  }
}
