
#include "esp_camera.h"
#include <vehicle.h>
#include <ultrasonic.h>
#include <ESP32Servo.h>
#include <Arduino.h>

#define Shoot_PIN 32  //shoot---200ms
#define FIXED_SERVO_PIN 25
#define TURN_SERVO_PIN 26

#define LED_Module1 2
#define LED_Module2 12
#define Left_sensor 35
#define Middle_sensor 36
#define Right_sensor 39
#define Buzzer 33

#define CMD_RUN 1
#define CMD_GET 2
#define CMD_STANDBY 3
#define CMD_TRACK_1 4
#define CMD_TRACK_2 5
#define CMD_AVOID 6
#define CMD_FOLLOW 7

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


vehicle Acebott;        //car
ultrasonic Ultrasonic;  //Ultrasonic
Servo fixedServo;       //Non-adjustable steering gear
Servo turnServo;       //Adjustable steering gear


int Left_Tra_Value;
int Middle_Tra_Value;
int Right_Tra_Value;
int Black_Line = 2000;
int Off_Road = 4000;
int speeds = 250;
int leftDistance = 0;
int middleDistance = 0;
int rightDistance = 0;

String sendBuff;
String Version = "Firmware Version is 0.12.21";
byte dataLen, index_a = 0;
char buffer[52];
unsigned char prevc = 0;
bool isStart = false;
bool ED_client = true;
bool WA_en = false;
byte RX_package[17] = { 0 };
uint16_t angle = 90;
byte action = Stop, device;
byte val = 0;
char model_var = 0;
int UT_distance = 0;

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

unsigned char readBuffer(int index_r) {
  return buffer[index_r];
}
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

int newRunMode = 0;

void setup() {
  Serial.setTimeout(10);  // Set the serial port timeout to 10 milliseconds
  Serial.begin(115200);   // Initialize serial communication, baud rate is 115200

  Acebott.Init();     // Initialize Acebott
  Ultrasonic.Init(13,14);  // Initialize the ultrasound module

  pinMode(LED_Module1, OUTPUT);   // Set pin 1 of LED module as output
  pinMode(LED_Module2, OUTPUT);   // Set pin 2 of the LED module as output
  pinMode(Shoot_PIN, OUTPUT);     // Set shooting pin as output
  pinMode(Left_sensor, INPUT);    // Set the infrared left line pin as input
  pinMode(Middle_sensor, INPUT);  // Set the infrared middle line pin as input
  pinMode(Right_sensor, INPUT);   // Set the infrared right line pin as input

  ESP32PWM::allocateTimer(1);          // Assign timer 1 to ESP32PWM library
  fixedServo.attach(FIXED_SERVO_PIN);           // Connect the servo to the FIXED_SERVO_PIN pin
  fixedServo.write(angle);                 // Set the servo angle to angle
  turnServo.attach(TURN_SERVO_PIN);  // Connect the servo to the FIXED_SERVO_PIN pin
  turnServo.write(angle);             // Set the servo angle to angle
  Acebott.Move(Stop, 0);               // Stop Acebott Movement
  delay(3000);                         // Delay 3 seconds

  length0 = sizeof(tune0) / sizeof(tune0[0]);  // Calculate the length of tune0 array
  length1 = sizeof(tune1) / sizeof(tune1[0]);  // Calculate the length of tune1 array
  length2 = sizeof(tune2) / sizeof(tune2[0]);  // Calculate the length of tune2 array
  length3 = sizeof(tune3) / sizeof(tune3[0]);  // Calculate the length of tune3 array

  Serial.println("start");
}

void loop() {
  RXpack_func();
  switch (newRunMode) {
    case 4:
      model1_func();
      break;
    case 5:
      model4_func();
      break;
    case 6:
      function_mode = AVOID;
      model2_func();
      break;
    case 7:
      model3_func();
      break;
    case 3:
      function_mode = STANDBY;
      newRunMode = 0;
      break;
  }
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
}

void model2_func()  // OA
{

  fixedServo.write(90);
  UT_distance = Ultrasonic.Ranging();
  //Serial.print("UT_distance:  ");
  //Serial.println(UT_distance);
  middleDistance = UT_distance;

  if (middleDistance <= 25) {
    Acebott.Move(Stop, 0);
    delay(100);
    int randNumber = random(1, 4);
    switch (randNumber) {
      case 1:
        Acebott.Move(Backward, 180);
        delay(500);
        Acebott.Move(Move_Left, 180);
        delay(500);
        break;
      case 2:
        Acebott.Move(Clockwise, 180);
        delay(1000);
        break;
      case 3:
        Acebott.Move(Contrarotate, 180);
        delay(1000);
        break;
      case 4:
        Acebott.Move(Backward, 180);
        delay(500);
        Acebott.Move(Move_Right, 180);
        delay(500);
        break;
    }
  } else {
    Acebott.Move(Forward, 180);
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

void model1_func()  // tracking model1
{
  //MfixedServo.write(90);
  Left_Tra_Value = analogRead(Left_sensor);
  //Middle_Tra_Value = analogRead(Middle_sensor);
  Right_Tra_Value = analogRead(Right_sensor);
  //Serial.println(Left_Tra_Value);
  delay(5);
  if (Left_Tra_Value < Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Forward, 130);
  } else if (Left_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) {
    Acebott.Move(Contrarotate, 150);
  } else if (Left_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line) {
    Acebott.Move(Clockwise, 150);
  } else if (Left_Tra_Value >= Black_Line && Left_Tra_Value < Off_Road && Right_Tra_Value >= Black_Line && Right_Tra_Value < Off_Road) {
    Acebott.Move(Stop, 0);
  } else if (Left_Tra_Value >= Off_Road && Right_Tra_Value >= Off_Road) {
    Acebott.Move(Stop, 0);
  }
}

void Servo_Move(int angles)  //servo
{
  turnServo.write(map(angles, 1, 180, 130, 70));
  delay(10);
}

void Music_a() {
  for (int x = 0; x < length0; x++) {
    tone(Buzzer, tune0[x]);
    delay(500 * durt0[x]);
    noTone(Buzzer);
  }
}
void Music_b() {
  for (int x = 0; x < length1; x++) {
    tone(Buzzer, tune1[x]);
    delay(500 * durt1[x]);
    noTone(Buzzer);
  }
}
void Music_c() {
  for (int x = 0; x < length2; x++) {
    tone(Buzzer, tune2[x]);
    delay(500 * durt2[x]);
    noTone(Buzzer);
  }
}
void Music_d() {
  for (int x = 0; x < length3; x++) {
    tone(Buzzer, tune3[x]);
    delay(300 * durt3[x]);
    noTone(Buzzer);
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
  int index2 = readBuffer(2);

  if (index2 == 7) {
    newRunMode = action;
  }

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
    // case CMD_TRACK_1:
    //   //callOK_Len01();
    //   function_mode = TRACK_1;
    //   //Serial.write(0x01);
    //   break;
    // case CMD_TRACK_2:
    //   //callOK_Len01();
    //   function_mode = TRACK_2;
    //   break;
    // case CMD_AVOID:
    //   //callOK_Len01();
    //   function_mode = AVOID;
    //   break;
    // case CMD_FOLLOW:
    //   //callOK_Len01();
    //   function_mode = FOLLOW;
    //   break;
    default: break;
  }
}

void RXpack_func()  //Receive data
{
  if (Serial.available() > 0) {
    unsigned char c = Serial.read() & 0xff;
    //Serial.write(c);
    if (c == 0x55 && isStart == false) {
      if (prevc == 0xff) {
        index_a = 1;
        isStart = true;
      }
    } else {
      prevc = c;
      if (isStart) {
        if (index_a == 2) {
          dataLen = c;
        } else if (index_a > 2) {
          dataLen--;
        }
        writeBuffer(index_a, c);
      }
    }
    index_a++;
    if (index_a > 120) {
      index_a = 0;
      isStart = false;
    }
    if (isStart && dataLen == 0 && index_a > 3) {
      isStart = false;
      parseData();
      index_a = 0;
    }
    // functionMode();
  }
}
