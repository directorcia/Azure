// https://blog.ciaopslabs.com
// Capture distance using the Adafruit VL53L0X sensor and display on an LCD as well as serial console
// Tested with ESP32-S2 SparkFun Thing Plus WROOM

#include "Adafruit_VL53L0X.h"   // Distance Sensor
#include <LiquidCrystal_I2C.h>  // LCD display
Adafruit_VL53L0X lox = Adafruit_VL53L0X();  // default address = 0x29
LiquidCrystal_I2C lcd(0x27,16,2);           // default address = 0x27

void setup() {
  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
  lcd.init();
 // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.setCursor(0,0);   // Column 1, Row 1
  lcd.print("VL53L0X test");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false);         // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {           // phase failures have incorrect data
    Serial.print("Distance (mm): "); 
    Serial.println(measure.RangeMilliMeter);
    int distance = measure.RangeMilliMeter; // Save range as an integer
    lcd.setCursor(0,1);                     // Column 1, Row 2
    lcd.print("Dist(mm):     ");            // Clear line 2
    lcd.setCursor(9,1);                     // Column 10, Row 2 
    lcd.print(distance);                    // Display distance in mm
  } else {
    Serial.println(" out of range ");
  }
    
  delay(500);
}
