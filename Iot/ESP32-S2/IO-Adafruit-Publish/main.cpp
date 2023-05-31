#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
// CIA specific headers
#include "ciaath.h"
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME "<Your Adafruit IO username>"
#define AIO_KEY "<Your Adafruit IO key>"
#define WLAN_SSID "<Your Wifi network>"
#define WLAN_PASS "<Your WifI password>"
static int counter = 10;

WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish feed1= Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/feed1");

// connect to adafruit io via MQTT
void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }
    if(ret >= 0)
      mqtt.disconnect();
    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

void setup() {
  int ledflash = 0;
 
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Init serial port");
  ciaaht_init();
  Serial.print("Connecting to wifi");
  WiFi.begin(WLAN_SSID,WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED){
    delay(250);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  // connect to adafruit
  connect();
 
  pinMode(13, OUTPUT);
  for (ledflash = 0; ledflash <=1; ledflash +=1) {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
  }
}
void loop() {
  int ledflash = 0;
    // ping adafruit io a few times to make sure we remain connected
  if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }
  if (!feed1.publish(ciaaht_getTemp())) {
    Serial.println(F("Failed"));
  }
  else {
    Serial.println(F("Success"));
  }
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(2500);
  digitalWrite(13, LOW);
  delay(2500);
}
