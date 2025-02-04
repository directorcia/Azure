#include <Arduino.h>
/*
  keyestudio sun_follower
  lesson 7.1
  DHT11
  http://www.keyestudio.com
*/
#include <dht11.h>  //include the library code:
dht11 DHT;
#define DHT11_PIN 7 //define the DHT11 as the digital port 7
void setup() {
  Serial.begin(115200);
}
void loop() {
  int chk;
  chk = DHT.read(DHT11_PIN);    //read data
  switch (chk) {
    case DHTLIB_OK:
      break;
    case DHTLIB_ERROR_CHECKSUM: //check and return errors
      break;
    case DHTLIB_ERROR_TIMEOUT: //timeout and return errors
      break;
    default:
      break;
  }
  // DISPLAT DATA
  Serial.print("humidity:");
  Serial.print(DHT.humidity);
  Serial.print("   temperature:");
  Serial.println(DHT.temperature);
  delay(200);
}