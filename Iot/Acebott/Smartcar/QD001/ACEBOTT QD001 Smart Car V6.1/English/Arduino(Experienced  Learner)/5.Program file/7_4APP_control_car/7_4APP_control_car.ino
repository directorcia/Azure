#include <WiFi.h>
#include "esp_camera.h"
#include <vehicle.h>
#include <ultrasonic.h>
#include <ESP32Servo.h>
#include <Arduino.h>

#define Shoot_PIN           32//shoot---150ms
#define Yservo_PIN         25

#define LED_Module1         2
#define LED_Module2         12        
#define Left_Line           35
#define Center_Line         36
#define Right_Line          39
#define Buzzer              33

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

const char *ssid = "ESP32-CAR"; //Set WIFI name
const char *password = "12345678"; //Set WIFI password
WiFiServer server(100); //Setting the server port
WiFiClient client; //Client side
vehicle Acebott; //car
ultrasonic Ultrasonic; //Ultrasonic wave
Servo Yservo; //Servo

int Left_Tra_Value;
int Center_Tra_Value;
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
unsigned char prevc=0;
bool isStart = false;
bool ED_client = true;
bool WA_en = false;
byte RX_package[17] = {0};
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
float durt1[] = { 0.49, 0.01, 0.49, 0.01, 0.99, 0.01, 0.49, 0.01, 0.49, 0.01, 0.99, 0.01, 0.5, 0.5, 0.75, 0.25, 1, 2};
// happy new year
int tune2[] = { C5, N, C5, N, C5, G4, E5, N, E5, N, E5, C5, N, C5, E5, G5, N, G5, F5, E5, D5, N };
float durt2[] = { 0.49, 0.01, 0.49, 0.01, 1, 1, 0.49, 0.01, 0.49, 0.01, 1, 0.99, 0.01, 0.5, 0.5,0.99,0.01, 1, 0.5, 0.5, 1, 1 };
// have a farm
int tune3[] = { C4, N, C4, N, C4, G3, A3, N, A3, G3,  E4, N, E4, D4, N, D4, C4 };
float durt3[] = { 0.99, 0.01, 0.99, 0.01, 1, 1, 0.99, 0.01, 1, 2, 0.99, 0.01, 1, 0.99, 0.01, 1, 1 };
/*****app music*****/

unsigned long lastDataTimes = 0;
bool st = false;

unsigned char readBuffer(int index_r)
{
  return buffer[index_r]; 
}
void writeBuffer(int index_w,unsigned char c)
{
  buffer[index_w]=c;
}

enum FUNCTION_MODE
{
  STANDBY,
  FOLLOW,
  TRACK_1,
  TRACK_2,
  AVOID,
} function_mode;

void setup()
{
    Serial.setTimeout(10);  // Set the serial port timeout to 10 milliseconds
    Serial.begin(115200);  // Serial communication is initialized with a baud rate of 115200

    Acebott.Init();  // Initialize Acebott
    Ultrasonic.Init(13,14);  // Initialize the ultrasonic module
    
    pinMode(LED_Module1, OUTPUT);  // Set pin of LED module as output
    pinMode(LED_Module2, OUTPUT); 
    pinMode(Shoot_PIN, OUTPUT);  // Set the shooting pin as the output
    pinMode(Left_Line, INPUT);  // Set infrared left line pin as input
    pinMode(Center_Line, INPUT);  // Set the infrared middle line pin as input
    pinMode(Right_Line, INPUT);  // Set the right infrared line pin as input

    ESP32PWM::allocateTimer(1);  // Assign timer 1 to the ESP32PWM library
    Yservo.attach(Yservo_PIN);  // Connect the servo to the Yservo_PIN pin
    Yservo.write(angle);  // Set the steering angle as Angle
    Acebott.Move(Stop, 0);  // Stop the Acebott exercise
    delay(3000); 

    length0 = sizeof(tune0) / sizeof(tune0[0]);  // Calculate the length of the tune0 array
    length1 = sizeof(tune1) / sizeof(tune1[0]);  // Calculate the length of the tune1 array
    length2 = sizeof(tune2) / sizeof(tune2[0]);  // Calculate the length of the tune2 array
    length3 = sizeof(tune3) / sizeof(tune3[0]);  // Calculate the length of the tune3 array

    WiFi.setTxPower(WIFI_POWER_19_5dBm);  // The Wi-Fi transmit power is set to 19.5dBm
    WiFi.mode(WIFI_AP);  // Set the Wi-Fi operating mode to access point mode
    WiFi.softAP(ssid, password, 5);  // Create a Wi-Fi access point with SSID as ssid, password as password, and maximum number of connections as 5
    Serial.print("\r\n");
    Serial.print("Camera Ready! Use 'http://");  // Printing a prompt
    Serial.print(WiFi.softAPIP());  // Print the access point IP address
    Serial.println("' to connect");  // Printing a prompt

    delay(100);
    server.begin();  // Starting the server
    delay(1000);
}

void loop()
{
    RXpack_func();
    //model4_func();
}

void functionMode()
{
    switch (function_mode)
    {
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

void Receive_data()  // Receiving data
{
    if (client.available())  // If data is available
    {
        unsigned char c = client.read() & 0xff;  // Read a byte of data
        Serial.write(c);  // Send the received data on the serial port
        lastDataTimes = millis(); // 
        if (c == 200)
        {
          st = false;
        }
        if (c == 0x55 && isStart == false)  // If the start flag 0x55 is received and data has not yet been received
        {
            if (prevc == 0xff)  // If the previous byte is also the start flag 0xff
            {
                index_a = 1;  // The data index is set to 1
                isStart = true;  // Start receiving data
            }
        }
        else
        {
            prevc = c;  // Update the previous byte's value
            if (isStart)  // If data has already been received
            {
                if (index_a == 2)  // If it is the second byte, it is the length of the data
                {
                    dataLen = c;  // Update data length
                }
                else if (index_a > 2)  // If it's a subsequent byte
                {
                    dataLen--;  // The data length is decremented by one
                }
                writeBuffer(index_a, c);  // Writes data to the buffer
            }
        }
        index_a++;  // Index increase
        if (index_a > 120)  // If the index exceeds the upper limit
        {
            index_a = 0;  // Reset the index to 0
            isStart = false;  // End of data reception
        }
        if (isStart && dataLen == 0 && index_a > 3)  // If the data is received
        { 
            isStart = false;  // End of data reception
            parseData();  // Parsing data
            index_a = 0;  // Reset the index to 0
        }
    }
    if (client.available() == 0 && (millis() - lastDataTimes)>3000)
    {
      st = true;
    }
}

void model2_func()      // OA
{
    Yservo.write(90);
    UT_distance = Ultrasonic.Ranging();
    //Serial.print("UT_distance:  ");
    //Serial.println(UT_distance);
    middleDistance = UT_distance;

    if (middleDistance <= 25) 
    {
        Acebott.Move(Stop, 0);
        for(int i = 0;i < 500;i++)
        {
          delay(1);
          Receive_data();
          if(function_mode != AVOID)
            return ;
        }
        Yservo.write(45);
        for(int i = 0;i < 300;i++)
        {
          delay(1);
          Receive_data();
          if(function_mode != AVOID)
            return ;
        }
        rightDistance = Ultrasonic.Ranging();
        //Serial.print("rightDistance:  ");
        //Serial.println(rightDistance);
        Yservo.write(135);
        for(int i = 0;i < 300;i++)
        {
          delay(1);
          Receive_data();
          if(function_mode != AVOID)
            return ;
        }
        leftDistance = Ultrasonic.Ranging();
        //Serial.print("leftDistance:  ");
        //Serial.println(leftDistance);
        Yservo.write(90);
        if((rightDistance < 10) && (leftDistance < 10))
        {
            Acebott.Move(Backward, 180);
            for(int i = 0;i < 1000;i++)
            {
                delay(1);
                Receive_data();
                if(function_mode != AVOID)
                    return ;
            }
            Acebott.Move(Contrarotate, 180);//delay(200);
            for(int i = 0;i < 500;i++)
            {
                delay(1);
                Receive_data();
                if(function_mode != AVOID)
                    return ;
            }
        }
        else if(rightDistance < leftDistance) 
        {
            Acebott.Move(Backward, 180);
            for(int i = 0;i < 500;i++)
            {
                delay(1);
                Receive_data();
                if(function_mode != AVOID)
                    return ;
            }
            Acebott.Move(Contrarotate, 180);//delay(200);
            for(int i = 0;i < 500;i++)
            {
                delay(1);
                Receive_data();
                if(function_mode != AVOID)
                    return ;
            }
        }//turn right
        else if(rightDistance > leftDistance)
        {
            Acebott.Move(Backward, 180);
            for(int i = 0;i < 500;i++)
            {
                delay(1);
                Receive_data();
                if(function_mode != AVOID)
                    return ;
            }
            Acebott.Move(Clockwise, 180);//delay(200);
            for(int i = 0;i < 500;i++)
            {
                delay(1);
                Receive_data();
                if(function_mode != AVOID)
                    return ;
            }
        }
        else
        {
            Acebott.Move(Backward, 180);
            for(int i = 0;i < 500;i++)
            {
                delay(1);
                Receive_data();
                if(function_mode != AVOID)
                    return ;
            }
            Acebott.Move(Clockwise, 180);//delay(200); 
            for(int i = 0;i < 500;i++)
            {
                delay(1);
                Receive_data();
                if(function_mode != AVOID)
                    return ;
            }
        }
    }
    else 
    {
        Acebott.Move(Forward, 150);
    }
}

void model3_func()      // follow model
{
    Yservo.write(90);  
    UT_distance = Ultrasonic.Ranging();
    //Serial.println(UT_distance);
    if (UT_distance < 15)
    {
        Acebott.Move(Backward, 200);
    }
    else if (15 <= UT_distance && UT_distance <= 20)
    {
        Acebott.Move(Stop, 0);
    }
    else if (20 <= UT_distance && UT_distance <= 25)
    {
        Acebott.Move(Forward, speeds-70);
    }
    else if (25 <= UT_distance && UT_distance <= 50)
    {
        Acebott.Move(Forward, 220);
    }
    else
    {
        Acebott.Move(Stop, 0);
    }
}

void model4_func()      // tracking model2
{
    Yservo.write(90);
    Left_Tra_Value = analogRead(Left_Line);
    Center_Tra_Value = analogRead(Center_Line);
    Right_Tra_Value = analogRead(Right_Line);
    delay(5);
    if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
    {
        Acebott.Move(Forward, 180);
    }
    if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
    {
        Acebott.Move(Forward, 180);
    }
    if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
    {
        Acebott.Move(Forward, 180);
    }
    /*else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
    {
        Acebott.Move(Contrarotate, 150);
    }*/
    else if (Left_Tra_Value >= Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value < Black_Line)
    {
        Acebott.Move(Contrarotate, 220);
    }
    else if (Left_Tra_Value < Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line)
    {
        Acebott.Move(Clockwise, 220);
    }
    /*else if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
    {
        Acebott.Move(Clockwise, 150);
    }*/
    /*else if (Left_Tra_Value >= Black_Line && Left_Tra_Value < Off_Road && Center_Tra_Value >= Black_Line && Center_Tra_Value < Off_Road && Right_Tra_Value >= Black_Line && Right_Tra_Value < Off_Road)
    {
        Acebott.Move(Forward, 180);
    }*/
    else if (Left_Tra_Value >= Off_Road && Center_Tra_Value >= Off_Road && Right_Tra_Value >= Off_Road)
    {
        Acebott.Move(Stop, 0);
    }
}

void model1_func()      // tracking model1
{
    //MYservo.write(90);
    Left_Tra_Value = analogRead(Left_Line);
    //Center_Tra_Value = analogRead(Center_Line);
    Right_Tra_Value = analogRead(Right_Line);
    //Serial.println(Left_Tra_Value);
    delay(5);
    if (Left_Tra_Value < Black_Line && Right_Tra_Value < Black_Line)
    {
        Acebott.Move(Forward, 130);
    }
    else if (Left_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
    {
        Acebott.Move(Contrarotate, 150);
    }
    else if (Left_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line)
    {
        Acebott.Move(Clockwise, 150);
    }
    else if (Left_Tra_Value >= Black_Line && Left_Tra_Value < Off_Road && Right_Tra_Value >= Black_Line && Right_Tra_Value < Off_Road)
    {
        Acebott.Move(Stop, 0);
    }
    else if (Left_Tra_Value >= Off_Road && Right_Tra_Value >= Off_Road)
    {
        Acebott.Move(Stop, 0);
    }
}

void Servo_Move(int angles)  //servo
{
  Yservo.write(angles);
  if (angles >= 180) angles = 180;
  if (angles <= 1) angles = 1;
  delay(10);
}

void Music_a()
{
    for(int x=0;x<length0;x++) 
    { 
        tone(Buzzer, tune0[x]);
        delay(500 * durt0[x]);
        noTone(Buzzer);
    }
}
void Music_b()
{
    for(int x=0;x<length1;x++) 
    { 
        tone(Buzzer, tune1[x]);
        delay(500 * durt1[x]);
        noTone(Buzzer);
    }
}
void Music_c()
{
    for(int x=0;x<length2;x++) 
    { 
        tone(Buzzer, tune2[x]);
        delay(500 * durt2[x]);
        noTone(Buzzer);
    }
}
void Music_d()
{
    for(int x=0;x<length3;x++) 
    { 
        tone(Buzzer, tune3[x]);
        delay(300 * durt3[x]);
        noTone(Buzzer);
    }
}
void Buzzer_run(int M)
{
    switch (M)
    {
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

void runModule(int device)
{
  val = readBuffer(12);
  switch(device) 
  {
    case 0x0C:
    {   
      switch (val)
      {
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
    }break;
    case 0x02:
    {  
        Servo_Move(val);
    }break;
    case 0x03:
    {  
        Buzzer_run(val);
    }break;
    case 0x05:
    {    
        digitalWrite(LED_Module1,val);
        digitalWrite(LED_Module2,val);
    }break;
    case 0x08:
    {    
        digitalWrite(Shoot_PIN,HIGH);
        delay(150);
        digitalWrite(Shoot_PIN,LOW);
    }break;
    case 0x0D:
    {
        speeds = val;
    }break;
  }   
}
void parseData()
{ 
    isStart = false;
    int action = readBuffer(9);
    int device = readBuffer(10);
    switch (action)
    {
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
        default:break;
    }
}

void RXpack_func()  //Receiving data
{
  client = server.available();  // Wait for the client to connect
  if (client)  // If there is a client connection
  {
    WA_en = true;  // Enable the write enable
    ED_client = true;  // The client connection flag is set to true
    Serial.println("[Client connected]");  // Print client connection information
    unsigned long previousMillis = millis();  // 
    const unsigned long timeoutDuration = 3000;  // 
    while (client.connected())  // While the client is still connected
    {
      if ((millis() - previousMillis) > timeoutDuration && client.available() == 0 && st==true)
      {
        break;
      }
      if (client.available())  // If there is data to read
      {
        previousMillis = millis();
        unsigned char c = client.read() & 0xff;  // Reading data
        Serial.write(c);  // Print the received data
        st = false;
        if (c == 200)
        {
          st = true;
        }
        if (c == 0x55 && isStart == false)  // If the data received is 0x55 and isStart is false
        {
          if (prevc == 0xff)  // If the previous byte is 0xff
          {
            index_a = 1;  // The index is set to 1
            isStart = true;  // The data start flag is set to true
          }
        }
        else
        {
          prevc = c;  // Update the previous byte's value
          if (isStart)  // If the data start flag is true
          {
            if (index_a == 2)  // If the index is 2
            {
              dataLen = c;  // The data length is set to c
            }
            else if (index_a > 2)  // If the index is greater than 2
            {
              dataLen--;  // The data length is decremented by 1
            }
            writeBuffer(index_a, c);  // Writes data to the buffer
          }
        }
        index_a++;  // Index incremented by 1
        if (index_a > 120)  // If the index is greater than 120
        {
          index_a = 0;  // The index is reset to 0
          isStart = false;  // The data start flag is set to false
        }
        if (isStart && dataLen == 0 && index_a > 3)  // If the data start flag is true and the data length is 0 and the index is greater than 3
        {
          isStart = false;  // The data start flag is set to false
          parseData();  // Parsing data
          index_a = 0;  // The index is set to 0
        }
      }
      functionMode();  // Function-pattern processing
      if (Serial.available())  // If the serial port has data to read
      {
        char c = Serial.read();  // Reading data
        sendBuff += c;  // Add the data to the send buffer
        client.print(sendBuff);  // Send the data to the client
        Serial.print(sendBuff);  // Print the data sent
        sendBuff = "";  // Clear the send buffer
      }
    }
    client.stop();  // Disconnect the client
    Acebott.Move(Stop, 0);
    Serial.println("[Client disconnected]");  // Prints client disconnection information
  }
  else  // If there is no client connection
  {
    if (ED_client == true)  // If there was a previous client connection
    {
      ED_client = false;  // The client connection flag is set to false
    }
  }
}

