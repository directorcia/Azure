#include <WiFi.h>
#include "esp_camera.h"
#include <time.h>
#include <vehicle.h>
#include <ultrasonic.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <Ps3Controller.h>

#define Shoot_PIN 32        //shoot
#define FIXED_SERVO_PIN 25  //Ultrasonic chassis
#define TURN_SERVO_PIN 26   // adjustable servo pin

#define LED_Module1 2       //Left LED Pin
#define LED_Module2 12      //Right LED Pin
#define Left_sensor 35      //Left   Line patrol Pin
#define Middle_sensor 36    //center Line patrol Pin
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

const char *ssid = "ESP32-CAR";           // Set the WIFI name
const char *password = "12345678";        // Set the WIFI password
String macAddress = "20:00:00:00:38:40";  // PS3 Bluetooth controller MAC address
WiFiServer server(100);                   // Set the server port
WiFiClient client;                        // Client

vehicle Acebott;                 // Car control object
ultrasonic Ultrasonic;           // Ultrasonic object
Servo fixedServo;                // No adjustable servo
Servo turnServo;                 // adjustable servo
         
         
int Left_Tra_Value;               // Value from the left track sensor
int Middle_Tra_Value;             // Value from the middle track sensor
int Right_Tra_Value;              // Value from the right track sensor
int Black_Line = 2000;            // Threshold for detecting black line
int Off_Road = 4000;              // Threshold for detecting off-road (outside of
int speeds = 250;                 // Default speed of the car
int leftDistance = 0;             // Left ultrasonic distance
int middleDistance = 0;           // Middle ultrasonic distance
int rightDistance = 0;            // Right ultrasonic distance
char Speed_Set_State = 2;         // Speed shift
bool st = false;                  // Record whether the APP is disconnected
                
String sendBuff;                  // Buffer for sending data
String Version = "Firmware Version is 2.0.0";
byte dataLen, index_a = 0;        // Data length and index
char buffer[52];                  // Buffer for data storage
unsigned char prevc = 0;          // Previous character (used for UART communication)

byte RX_package[17] = { 0 };      // Array for receiving data (17 bytes buffer)
uint16_t angle = 90;              // Robot's initial angle (set to 90 degrees)
byte action = Stop, device;       // Action state (e.g., Stop) and device identifier
byte val = 0;                     // A general-purpose variable, potentially for control or settings
char model_var = 0;               // Model version or specific mode variable
int UT_distance = 0;              // Ultrasonic sensor distance value
unsigned long lastDataTimes = 0;  // Record the last time
char angles = 90;                 // Initial setting of gun Angle
char current_angles = 90;         // Current gun Angle
char Buzzer_count = 0;
char up = 0;
int currentState = 0;

int length0;
int length1;
int length2;
int length3;

bool isStart = false;             // Flag to indicate if the robot has started
bool ED_client = true;            // Flag indicating if the client is active
bool WA_en = false;               // Flag for enabling WA (could be related to
bool rstate = false;              // servo move state
bool lstate = false;              // servo move state
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

void Ps3_Control()       // PS3 Bluetooth controller and read
{
  // Assign a value to Ps3_Key_Steta
  // Check if a button is pressed and store its state
  if (Ps3.event.button_down.cross) strcpy(Ps3_Key_Steta, "cross");        // Cross button pressed
  if (Ps3.event.button_down.square) strcpy(Ps3_Key_Steta, "square");      // Square button pressed
  if (Ps3.event.button_down.triangle) strcpy(Ps3_Key_Steta, "triangle");  // Triangle button pressed
  if (Ps3.event.button_down.circle) strcpy(Ps3_Key_Steta, "circle");      // Circle button pressed
  if (Ps3.event.button_down.down) strcpy(Ps3_Key_Steta, "down");          // Up button pressed
  if (Ps3.event.button_down.left) strcpy(Ps3_Key_Steta, "left");          // Left button pressed
  if (Ps3.event.button_down.right) strcpy(Ps3_Key_Steta, "right");        // Right button pressed
  if (Ps3.event.button_down.select) strcpy(Ps3_Key_Steta, "select");      // select button pressed 

  if (Ps3.event.button_down.up) up = 1;                                   // Down button pressed
  else if (Ps3.event.button_up.up) up = 0;                                // Down button pressed

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

// If the right state is active (rstate is true)
if (rstate) {
    // If the angles are greater than 10, decrease the angle
    if (angles > 10) {
        angles -= 1;  // Decrease the angle by 1
        // If the angle is less than or equal to 10, set the angle to 10 and deactivate the right state (rstate)
        if (angles <= 10) {
            angles = 10;  // Ensure the angle doesn't go below 10
            rstate = false;  // Deactivate the right state
        }
    }
    delay(20);  // Wait for 20 milliseconds before checking again
}

// If the left state is active (lstate is true)
if (lstate) {
    // If the angles are less than 170, increase the angle
    if (angles < 150) {
        angles += 1;  // Increase the angle by 1
        // If the angle is greater than or equal to 170, set the angle to 170 and deactivate the left state (lstate)
        if (angles >= 150) {
            angles = 150;  // Ensure the angle doesn't go above 170
            lstate = false;  // Deactivate the left state
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


/*******************************************************************      END      ***********************************************************************************/

  if (strcmp(Ps3_Key_Steta, "Playstation") == 0) {     // Playstation button pressed
    Ps3.end(0);
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));   
  }
  if (strcmp(Ps3_Key_Steta, "cross") == 0) {           // X button
    model3_func();                                     // Following mode
  }       
  if (strcmp(Ps3_Key_Steta, "square") == 0) {          // Square button
    model1_func();                                     // Line tracking mode 1
  }    
  if (strcmp(Ps3_Key_Steta, "triangle") == 0) {        // Triangle button
    function_mode = AVOID;
    model2_func();                                     // Obstacle avoidance mode
  }    
  if (strcmp(Ps3_Key_Steta, "circle") == 0) {          // Circle button
    model4_func();                                     // Line tracking 2
  }    
  if (strcmp(Ps3_Key_Steta, "down") == 0) {            // Left down (Directional button press)ft down

    if ((Ps3.event.button_up.down) == 0) {                // Determine whether to press or hold
        currentState = digitalRead(LED_Module2);       // Read current state of the LED modulead current LED state
    } else {                                            // Let go and stop
        digitalWrite(LED_Module2, !currentState);          // Toggle LED module 2 stateggle LED state
        digitalWrite(LED_Module1, !currentState);          // Toggle LED module 1 state
        memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));
    }
  }

  switch(up){
      case 0 : 
          // If the value of 'up' is 0, do nothing and exit the switch
          break;
      case 1 : 
          // If the value of 'up' is 1, increment 'Buzzer_count'
          Buzzer_count++;   
          // Ensure 'Buzzer_count' stays within the range 0-3 by using modulo
          Buzzer_count %= 4;   
          // Print the current value of 'Buzzer_count' to the serial monitor
          Serial.println(Buzzer_count + 0x00);  
          // Change the state of 'up' to 2
          up = 2;
          break;
      case 2 : 
          // If the value of 'up' is 2, call the 'Buzzer_runBLU' function with 'Buzzer_count' as argument
          Buzzer_runBLU(Buzzer_count);  
          // Reset 'up' to 0, starting the cycle over
          up = 0;
          break;
  }

  if (strcmp(Ps3_Key_Steta, "select") == 0) {          // Select button
    Speed_Set_State++;                                 // Increment the speed setting state
    Speed_Set_State %= 6;                              // Cycle through speed levels (0-4)
    Speed_State(Speed_Set_State);                      // Apply the new speed setting
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));   // Clear button state
  }
  if (strcmp(Ps3_Key_Steta, "left trigger") == 0) {     // Left 2  point-and-shoot
    digitalWrite(Shoot_PIN, HIGH);                      // Shoot_PIN pulls up when pressed to start firing
    delay(100);                                         // Delay wait 100 milliseconds
    digitalWrite(Shoot_PIN, LOW);                       // When released Shoot_PIN pulls down to stop firing
    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));
  }
  if (strcmp(Ps3_Key_Steta, "right trigger") == 0) {    // Right 2 Long press combo
    if ((Ps3.event.button_up.r2) == 0) {                // Determine whether to press or hold
      digitalWrite(Shoot_PIN, HIGH);                    // Shoot_PIN pulls up when pressed to start firing
    } else {                                            // Let go and stop
      memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta));  // When released Shoot_PIN pulls down to stop firing
      digitalWrite(Shoot_PIN, LOW);
    }
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

void setup() {
  Serial.setTimeout(10);  // Set serial port timeout to 10 milliseconds
  Serial.begin(115200);   // Initialize serial communication with baud rate of 115200

  Acebott.Init();     // Initialize Acebott
  Ultrasonic.Init(13,14);  // Initialize ultrasonic module

  pinMode(LED_Module1, OUTPUT);        // Set LED Module 1 pin as output
  pinMode(LED_Module2, OUTPUT);        // Set LED Module 2 pin as output
  pinMode(Shoot_PIN, OUTPUT);          // Set shoot pin as output
  pinMode(Left_sensor, INPUT);         // Set left infrared sensor pin as input
  pinMode(Middle_sensor, INPUT);       // Set middle infrared sensor pin as input
  pinMode(Right_sensor, INPUT);        // Set right infrared sensor pin as input

  ESP32PWM::allocateTimer(1);          // Allocate timer 1 to the ESP32PWM library
  fixedServo.attach(FIXED_SERVO_PIN);  // Attach servo to FIXED_SERVO_PIN
  fixedServo.write(angle);             // Set servo angle to 'angle'
  turnServo.attach(TURN_SERVO_PIN);    // Attach servo to TURN_SERVO_PIN
  turnServo.write(90);                 // Set servo angle to 'angle'
  Acebott.Move(Stop, 0);               // Stop Acebott movement
  delay(3000);                         // Delay 3 seconds

  length0 = sizeof(tune0) / sizeof(tune0[0]);  // Calculate the length of tune0 array
  length1 = sizeof(tune1) / sizeof(tune1[0]);  // Calculate the length of tune1 array
  length2 = sizeof(tune2) / sizeof(tune2[0]);  // Calculate the length of tune2 array
  length3 = sizeof(tune3) / sizeof(tune3[0]);  // Calculate the length of tune3 array

  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Set Wi-Fi transmit power to 19.5dBm
  WiFi.mode(WIFI_AP);                   // Set Wi-Fi mode to Access Point mode
  WiFi.softAP(ssid, password, 5);       // Create Wi-Fi access point with SSID 'ssid', password 'password', max connections 5
  Serial.print("\r\n");
  Serial.print("Camera Ready! Use 'http://");  // Print prompt message
  Serial.print(WiFi.softAPIP());               // Print access point IP address
  Serial.println("' to connect");              // Print prompt message

  server.begin();           // Start server
  Ps3.attach(Ps3_Control);  // Attach Ps3_Control function to Ps3
  // Ps3.attachOnConnect(onConnect);  // Attach onConnect function to Ps3 (commented out)
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
        model3_func();  // Enter follow mode and call the model3_func() function
      }
      break;
    case TRACK_1:
      {
        model1_func();  // Enter trace mode 1 and call the model1_func() function
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

void Receive_data()  // receive data
{
  if (client.available())  // If data is available
  {
    unsigned char c = client.read() & 0xff;  // Read one byte of data
    Serial.write(c);                         // Send the received data over the serial port
    lastDataTimes = millis();                // Get current time
    if (c == 200) {
      st = false;
    }
    if (c == 0x55 && isStart == false)  // If the start flag 0x55 is received and data has not yet been received
    {
      if (prevc == 0xff)  // If the previous byte is also the start flag 0xff
      {
        index_a = 1;     // Set the data index to 1
        isStart = true;  //Start receiving data
      }
    } else {
      prevc = c;    // Update the value of the previous byte
      if (isStart)  // If you have started receiving data
      {
        if (index_a == 2)  // If it is the second byte, it indicates the length of the data
        {
          dataLen = c;           // Update data length
        } else if (index_a > 2)  // If it is a subsequent byte
        {
          dataLen--;  // Data length minus one
        }
        writeBuffer(index_a, c);  // Write data to the buffer
      }
    }
    index_a++;          // Index increment
    if (index_a > 120)  // If the index exceeds the upper limit
    {
      index_a = 0;      // Reset index to 0
      isStart = false;  // End data reception
    }
    if (isStart && dataLen == 0 && index_a > 3)  // If the data is received
    {
      isStart = false;  // End data reception
      parseData();      // Analytic data
      index_a = 0;      // Reset index to 0
    }
  }

  if (client.available() == 0 && (millis() - lastDataTimes) > 3000) {
    st = true;
  }
}

void model2_func() {
  // Set the servo to the center position (90 degrees)
  fixedServo.write(90);

  // Measure the distance in front of the robot using the ultrasonic sensor
  UT_distance = Ultrasonic.Ranging();
  middleDistance = UT_distance;

  // If an obstacle is detected in front (distance <= 25 cm)
  if (middleDistance <= 25) {
    // Stop the robot
    Acebott.Move(Stop, 0);
    
    // Wait for a short period to ensure all operations complete
    for (int i = 0; i < 300; i++) {
      delay(1); // 500 ms delay
      Receive_data();  // Check if there's any new data received (e.g., command or status)
      
      // If function mode is not AVOID  is triggered, stop further execution
      if (function_mode != AVOID ) return;
    }

    // Move the servo to 45 degrees (right direction) to check for obstacles on the right side
    fixedServo.write(45);

    // Delay for the servo to stabilize and take a reading
    for (int i = 0; i < 300; i++) {
      delay(1); // 300 ms delay
      Receive_data();  // Continuously check for new data during the wait
      if (function_mode != AVOID ) return;
    }

    // Measure the distance on the right side using the ultrasonic sensor
    rightDistance = Ultrasonic.Ranging();

    // Move the servo to 135 degrees (left direction) to check for obstacles on the left side
    fixedServo.write(135);

    // Delay for the servo to stabilize and take a reading
    for (int i = 0; i < 300; i++) {
      delay(1); // 300 ms delay
      Receive_data();  // Continuously check for new data during the wait
      if (function_mode != AVOID ) return;
    }

    // Measure the distance on the left side using the ultrasonic sensor
    leftDistance = Ultrasonic.Ranging();

    // Return the servo to the center (90 degrees)
    fixedServo.write(90);

    // If both sides (left and right) have obstacles less than 10 cm
    if (rightDistance < 10 && leftDistance < 10) {
      // Back up the robot for 1000 ms
      Acebott.Move(Backward, 180);
      for (int i = 0; i < 200; i++) {
        delay(1);
        Receive_data();  // Ensure no interruption or new command
        if (function_mode != AVOID ) return;
      }

      // Perform a counter-clockwise turn to change direction and avoid the obstacle
      Acebott.Move(Contrarotate, 180);
      for (int i = 0; i < 200; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID ) return;
      }
    }
    // If the right side is clearer (lesser distance than the left side)
    else if (rightDistance < leftDistance) {
      // Back up the robot for 500 ms
      Acebott.Move(Backward, 180);
      for (int i = 0; i < 200; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID ) return;
      }

      // Perform a counter-clockwise turn (to move away from the obstacle on the right)
      Acebott.Move(Contrarotate, 180);
      for (int i = 0; i < 200; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID ) return;
      }
    }
    // If the left side is clearer (lesser distance than the right side)
    else {
      // Back up the robot for 500 ms
      Acebott.Move(Backward, 180);
      for (int i = 0; i < 200; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID ) return;
      }

      // Perform a clockwise turn (to move away from the obstacle on the left)
      Acebott.Move(Clockwise, 180);
      for (int i = 0; i < 100; i++) {
        delay(1);
        Receive_data();
        if (function_mode != AVOID ) return;
      }
    }
  } 
  // If no obstacle is detected in front (distance > 25 cm), continue moving forward
  else {
    Acebott.Move(Forward, 150); // Move forward at speed 150
  }
}

void model3_func()  // follow model
{
  fixedServo.write(90);
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

void model4_func()  // tracking model2
{
  fixedServo.write(90);
  Left_Tra_Value = analogRead(Left_sensor);
  Middle_Tra_Value = analogRead(Middle_sensor);
  Right_Tra_Value = analogRead(Right_sensor);
  delay(5);
  if (Left_Tra_Value < Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Forward, 180);
  }
  if (Left_Tra_Value < Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line) {
    Acebott.Move(Forward, 180);
  }
  if (Left_Tra_Value >= Black_Line && Middle_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Forward, 180);
  }

  else if (Left_Tra_Value >= Black_Line && Middle_Tra_Value < Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Contrarotate, 220);
  } else if (Left_Tra_Value < Black_Line && Middle_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line) {
    Acebott.Move(Clockwise, 220);
  }

  else if (Left_Tra_Value >= Off_Road && Middle_Tra_Value >= Off_Road && Right_Tra_Value >= Off_Road) {
    Acebott.Move(Stop, 0);
  }
}

void model1_func()  // Tracking model1
{
  // Read the analog values from the left and right tracking sensors
  Left_Tra_Value = analogRead(Left_sensor);
  Right_Tra_Value = analogRead(Right_sensor);

  // Small delay to ensure the sensors are stable
  delay(5);

  // If both sensors detect the black line (values below the threshold), move forward
  if (Left_Tra_Value < Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Forward, 130);  // Move forward with speed 130
  } 
  // If the left sensor is on the line (value above threshold) and the right sensor detects the black line, rotate counter-clockwise
  else if (Left_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Contrarotate, 150);  // Rotate counter-clockwise with speed 150
  } 
  // If the right sensor is on the line (value above threshold) and the left sensor detects the black line, rotate clockwise
  else if (Left_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line) {
    Acebott.Move(Clockwise, 150);  // Rotate clockwise with speed 150
  } 
  // If both sensors are off the road (values between Black_Line and Off_Road), stop the robot
  else if (Left_Tra_Value >= Black_Line && Left_Tra_Value < Off_Road && Right_Tra_Value >= Black_Line && Right_Tra_Value < Off_Road) {
    Acebott.Move(Stop, 0);  // Stop the robot
  } 
  // If both sensors are off the road (values greater than Off_Road), stop the robot
  else if (Left_Tra_Value >= Off_Road && Right_Tra_Value >= Off_Road) {
    Acebott.Move(Stop, 0);  // Stop the robot
  }
}

void Servo_Move(int angles)  // Servo movement function
{
  // Map the input angles (1 to 180) to corresponding PWM values (115 to 70)
  int pwmValue = map(angles, 1, 180, 115, 70);

  // Get the current position of the servo
  int currentPwm = turnServo.read();

  // If the target PWM value is greater than the current PWM value (servo needs to move to a higher angle)
  if (pwmValue > currentPwm) {
    // Gradually move the servo by 20 steps
    for (int j = 0; j < 20; j++) {
      // Interpolate the PWM value between current position and target position
      int newPwm = currentPwm + (pwmValue - currentPwm) * (j / 20.0);
      
      // Set the servo to the new PWM position
      turnServo.write(newPwm);
      
      // Wait for 20ms before the next adjustment
      delay(20);
    }
  } else {
    // If the target PWM value is less than the current PWM value (servo needs to move to a lower angle)
    // Gradually move the servo by 15 steps
    for (int j = 0; j < 15; j++) {
      // Interpolate the PWM value between current position and target position
      int newPwm = currentPwm + (pwmValue - currentPwm) * (j / 15.0);
      
      // Set the servo to the new PWM position
      turnServo.write(newPwm);
      
      // Wait for 20ms before the next adjustment
      delay(20);
    }
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
    int pwmValue = map(angle, 1, 180, 115, 70);
    int currentPwm = turnServo.read();  // Get the current PWM value of the servo
    // Gradually move the servo in 15 steps for a smooth transition
    for (int j = 0; j < 15; j++) {
      // Calculate the new PWM value incrementally
      int newPwm = currentPwm + (pwmValue - currentPwm) * (j / 15.0);
      turnServo.write(newPwm);  // Write the new PWM value to the servo
      delay(15);                // Wait for 15 milliseconds before the next step
    }
  } else {
    // Map the target angle to a PWM value for the servo (with a different range for smaller movements)
    int pwmValue = map(angle, 1, 180, 115, 70);
    int currentPwm = turnServo.read();  // Get the current PWM value of the servo
    // Gradually move the servo in 10 steps for a smoother transition
    for (int j = 0; j < 10; j++) {
      // Calculate the new PWM value incrementally
      int newPwm = currentPwm + (pwmValue - currentPwm) * (j / 10.0);
      turnServo.write(newPwm);  // Write the new PWM value to the servo
      delay(10);                // Wait for 10 milliseconds before the next step
    }
  }
  current_angles = angle;  // Update the current angle to the target angle
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

void Buzzer_run(int M) {
  switch (M) {
    case 0x01:
      Music_a();
      break;
    case 0x02:
      Music_b();
      break;
    case 0x03:
      Music_c();
      break;
    case 0x04:
      Music_d();
      break;
    default:
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
    case 0x08:
      {
        digitalWrite(Shoot_PIN, HIGH);
        delay(200);
        digitalWrite(Shoot_PIN, LOW);
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

void RXpack_func()  //receive data
{
  client = server.available();  // Wait for the client to connect
  if (client)                   // If a client is connected
  {
    WA_en = true;                                // Enable the write function
    ED_client = true;                            // Client connection flag set to true
    Serial.println("[Client connected]");        // Print client connection information
    unsigned long previousMillis = millis();     // Last check time
    const unsigned long timeoutDuration = 2000;  // Timeout (2 seconds)
    while (client.connected())                   // When the client is still connected
    {
      if ((millis() - previousMillis) > timeoutDuration && client.available() == 0 && st == true) {
        break;
      }
      if (client.available())  // If there is data to read
      {
        previousMillis = millis();
        unsigned char c = client.read() & 0xff;  // read data
        Serial.write(c);
        st = false;
        if (c == 200) {
          st = true;
        }
        if (c == 0x55 && isStart == false)  // If the received data is 0x55 and isStart is false
        {
          if (prevc == 0xff)  // If the previous byte is 0xff
          {
            index_a = 1;     // Index set to 1
            isStart = true;  // Data start flag is set to true
          }
        } else {
          prevc = c;    // Update the value of the previous byte
          if (isStart)  // If the data start flag is true
          {
            if (index_a == 2)  // If the index is 2
            {
              dataLen = c;           // The data length is set to c
            } else if (index_a > 2)  // If the index is greater than 2
            {
              dataLen--;  // Data length minus 1
            }
            writeBuffer(index_a, c);  // Write data to the buffer
          }
        }
        index_a++;          // Index increase by 1
        if (index_a > 120)  // If the index is greater than 120
        {
          index_a = 0;      // Index reset to 0
          isStart = false;  // Start of data flag is set to false
        }
        if (isStart && dataLen == 0 && index_a > 3)  // If the data start flag is true and the data length is 0 and the index is greater than 3
        {
          isStart = false;  // Start of data flag is set to false
          parseData();      // Analytic data
          index_a = 0;      // Index set to 0
        }
      }
      functionMode();          // Functional mode processing
      if (Serial.available())  // If the serial port has data to read
      {
        char c = Serial.read();  // read data
        sendBuff += c;           // Adds data to the send buffer
        client.print(sendBuff);  // Send data to the client
        Serial.print(sendBuff);
        sendBuff = "";  // Clear send buffer
      }
    }
    client.stop();  // Disconnect the client
    Acebott.Move(Stop, 0);
    Serial.println("[Client disconnected]");  // The client disconnection information is displayed
  } else                                      // If there is no client connection
  {
    if (ED_client == true)  // If a client was previously connected
    {
      ED_client = false;  // Client connection flag is set to false
    }
  }
  if (!Ps3.isConnected()) {
    Ps3.begin(macAddress.c_str());  // Reconnection handle
    delay(1000);
  } else { 
    Ps3_Control();      // ps3 control
  }
}