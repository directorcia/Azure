#include <Adafruit_AHTX0.h>

// Require library - Adafruit AHTX0

// DHT20 pinouts
// Pinout (left to right from the front):
//	• VDD: Supply voltage
//	• SDA: Serial Data (I2C)
//	• GND: Ground
//  • SCL: Serial Clock (I2C)

// Huzzah pinouts
// 3V
// GND
// 5 = SCL
// 4 = SDA


Adafruit_AHTX0 aht;
void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit AHT10/AHT20 demo!");
  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");
}

void loop() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  delay(500);
}