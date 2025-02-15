/*
  keyestudio sun_follower
  lesson 11
  sun_follower
  http://www.keyestudio.com
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include <BH1750.h>
BH1750 lightMeter;

#include <dht11.h>    //include the library code:
dht11 DHT;
#define DHT11_PIN 7  //define the DHT11 as the digital pin 7

#include <Servo.h>
Servo lr_servo;//define the name of the servo rotating right and left
Servo ud_servo;//efine the name of the servo rotating upwards and downwards

const byte interruptPin = 2;  //the pin of button;the corruption is disrupted

int lr_angle = 90;//set the initial angle to 90 degree
int ud_angle = 10;//set the initial angle to 10 degree;keep the solar panels upright to detect the strongest light
int l_state = A0;//define the analog voltage input of the photoresistors
int r_state = A1;
int u_state = A2;
int d_state = A3;
const byte buzzer = 6;  //set the pin of the buzzer to digital pin 6
const byte lr_servopin = 9;//define the control signal pin of the servo rotating right and lef
const byte ud_servopin = 10;//define the control signal pin of the servo rotating clockwise and anticlockwise 

unsigned int light; //save the variable of light intensity
byte error = 15;//Define the error range to prevent vibration
byte m_speed = 10;//set delay time to adjust the speed of servo;the longer the time, the smaller the speed
byte resolution = 1;   //set the rotation accuracy of the servo, the minimum rotation angle 
int temperature;  //save the variable of temperature
int humidity; //save the variable of humidity

/**********the function of the servo************/
void ServoAction(){
  int L = analogRead(l_state);//read the analog voltage value of the sensor, 0-1023
  int R = analogRead(r_state);
  int U = analogRead(u_state);
  int D = analogRead(d_state);
  /**********************system adjusting left and right序**********************/
  //  abs() is the absolute value function
  if (abs(L - R) > error && L > R) { //Determine whether the error is within the acceptable range, otherwise adjust the steering gear
    lr_angle -= resolution;//reduce the angle
    //    lr_servo.attach(lr_servopin);  // connect servo
    if (lr_angle < 0) { //limit the rotation angle of the servo
      lr_angle = 0;
    }
    lr_servo.write(lr_angle);  //output the angle of the servooutput the angle of servo
    delay(m_speed);

  }
  else if (abs(L - R) > error && L < R) { //Determine whether the error is within the acceptable range, otherwise adjust the steering gear
    lr_angle += resolution;//increase the angle
    //    lr_servo.attach(lr_servopin);    // connect servo
    if (lr_angle > 180) { //limit the rotation angle of servo
      lr_angle = 180;
    }
    lr_servo.write(lr_angle);  //output the angle of servo
    delay(m_speed);

  }
  else if (abs(L - R) <= error) { //Determine whether the error is within the acceptable range, otherwise adjust the steering gear
    //    lr_servo.detach();  //release the pin of servo
    lr_servo.write(lr_angle); //output the angle of servo
  }
  /**********************system adjusting up and down**********************/
  if (abs(U - D) > error && U >= D) { //Determine whether the error is within the acceptable range, otherwise adjust the steering gear
    ud_angle -= resolution;//reduce the angle
    //    ud_servo.attach(ud_servopin);  // connect servo
    if (ud_angle < 10) { //limit the rotation angle of servo
      ud_angle = 10;
    }
    ud_servo.write(ud_angle);  //output the angle of servo
    delay(m_speed);

  }
  else if (abs(U - D) > error && U < D) { //Determine whether the error is within the acceptable range, otherwise adjust the steering gear
    ud_angle += resolution;//increase the angle
    //    ud_servo.attach(ud_servopin);  // connect servo
    if (ud_angle > 90) { //limit the rotation angle of servo
      ud_angle = 90;
    }
    ud_servo.write(ud_angle);  //output the angle of servo
    delay(m_speed);

  }
  else if (abs(U - D) <= error) { //Determine whether the error is within the acceptable range. If it is, keep it stable and make no change in angle
    //    ud_servo.detach();  //release the pin of servo
    ud_servo.write(ud_angle);  //output the angle of servo
  }
}

void LcdShowValue() {
  char str1[5];
  char str2[2];
  char str3[2];
  dtostrf(light, -5, 0, str1); //Format the light value data as a string, left-aligned
  dtostrf(temperature, -2, 0, str2);
  dtostrf(humidity, -2, 0, str3);
  //LCD1602 display
  //display the value of the light intensity
  lcd.setCursor(0, 0);
  lcd.print("Light:");
  lcd.setCursor(6, 0);
  lcd.print(str1);
  lcd.setCursor(11, 0);
  lcd.print("lux");
  
  //display the value of temperature and humidity
  lcd.setCursor(0, 1);
  lcd.print(temperature);
  lcd.setCursor(2, 1);
  lcd.print("C");
  lcd.setCursor(5, 1);
  lcd.print(humidity);
  lcd.setCursor(7, 1);
  lcd.print("%");

  //show the accuracy of rotation
  lcd.setCursor(11, 1);
  lcd.print("res:");
  lcd.setCursor(15, 1);
  lcd.print(resolution);
  /*if (light < 10) {
    lcd.setCursor(7, 0);
    lcd.print("        ");
    lcd.setCursor(6, 0);
    lcd.print(light);
    } else if (light < 100) {
    lcd.setCursor(8, 0);
    lcd.print("       ");
    lcd.setCursor(6, 0);
    lcd.print(light);
    } else if (light < 1000) {
    lcd.setCursor(9, 0);
    lcd.print("      ");
    lcd.setCursor(6, 0);
    lcd.print(light);
    } else if (light < 10000) {
    lcd.setCursor(9, 0);
    lcd.print("      ");
    lcd.setCursor(6, 0);
    lcd.print(light);
    } else if (light < 100000) {
    lcd.setCursor(10, 0);
    lcd.print("     ");
    lcd.setCursor(6, 0);
    lcd.print(light);
    }*/
}

void read_light(){
  light = lightMeter.readLightLevel();  //read the light intensity detected by BH1750
}

void read_dht11(){
  int chk;
  chk = DHT.read(DHT11_PIN);      // read data
  switch (chk) {
    case DHTLIB_OK:
      break;
    case DHTLIB_ERROR_CHECKSUM:   //check and return error
      break;
    case DHTLIB_ERROR_TIMEOUT:    //Timeout and return error
      break;
    default:
      break;
  }
  temperature = DHT.temperature;
  humidity = DHT.humidity;
}

/*********function disrupts service**************/
void adjust_resolution() {
  tone(buzzer, 800, 100);
  delay(10);  //delay to eliminate vibration
  if (!digitalRead(interruptPin)){
    if(resolution < 5){
      resolution++;
    }else{
      resolution = 1;
    }
  }
}

void setup() {
  Serial.begin(9600); //define the serial baud rate
  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();
  lightMeter.begin();

  lr_servo.attach(lr_servopin);  // set the control pin of servo
  ud_servo.attach(ud_servopin);  // set the control pin of servo
  pinMode(l_state, INPUT); //set the mode of pin
  pinMode(r_state, INPUT);
  pinMode(u_state, INPUT);
  pinMode(d_state, INPUT);

  pinMode(interruptPin, INPUT_PULLUP);  //the button pin is set to input pull-up mode
  attachInterrupt(digitalPinToInterrupt(interruptPin), adjust_resolution, FALLING); //xternal interrupt touch type is falling edge; adjust_resolution is interrupt service function ISR

  lcd.init();          // initialize the LCD
  lcd.backlight();     //set LCD backlight

  lr_servo.write(lr_angle);//return to initial angle
  delay(1000);
  ud_servo.write(ud_angle);
  delay(1000);

}

void loop() {
  ServoAction();  //servo performs the action
  read_light();   //read the light intensity of bh1750
  read_dht11();   //read the value of temperature and humidity
  LcdShowValue(); //Lcd shows the values of light intensity, temperature and humidity

  //erial monitor displays the resistance of the photoresistor and the angle of servo
  /*Serial.print(" L ");
  Serial.print(L);
  Serial.print(" R ");
  Serial.print(R);
  Serial.print("  U ");
  Serial.print(U);
  Serial.print(" D ");
  Serial.print(D);
  Serial.print("  ud_angle ");
  Serial.print(ud_angle);
  Serial.print("  lr_angle ");
  Serial.println(lr_angle);*/
  //  delay(1000);//During the test, the serial port data is received too fast, and it can be adjusted by adding delay time */
}

