#define Left_sensor 35 //declare the pin of left tracking sensor
#define Right_sensor 39 //declare the pin of right tracking sensor
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(Left_sensor, INPUT);
  pinMode(Right_sensor, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Left_sensor_value:");
  //Read the value of left tracking sensor
  Serial.println(analogRead(Left_sensor));
  Serial.print("Right_sensor_value:");
  //Read the value of right tracking sensor
  Serial.println(analogRead(Right_sensor));
  delay(1000);
}
