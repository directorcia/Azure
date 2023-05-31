#include "ciaath.h"
#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;

void ciaaht_init() {
  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");
}
float ciaaht_getTemp() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
//  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
//  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
    return temp.temperature;
}
float ciaaht_getHumidity() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
//  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
//  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
    return humidity.relative_humidity;
}
