
#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME "<Your Adafruit IO username>"
#define AIO_KEY "<Your Adafruit IO key>"
#define WLAN_SSID "<Your Wifi network>"
#define WLAN_PASS "<Your WifI password>"

WiFiClient client;      // declare variable

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoffbutton");

void MQTT_connect();

void setup() {
   // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Init serial port");
  pinMode(13, OUTPUT);
  Serial.print("Connecting to wifi");
  WiFi.begin(WLAN_SSID,WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED){
    delay(250);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  mqtt.subscribe(&onoffbutton);
}

void loop() {
  // connect to adafruit
  MQTT_connect();

  Adafruit_MQTT_Subscribe *subscription;

  Serial.println(F("Subscription"));
  while (subscription = mqtt.readSubscription(2000)) {
    Serial.println(F("Received button data"));
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      String ledstatus = (char *)onoffbutton.lastread;
      if (ledstatus == "ON") {
        digitalWrite(13, HIGH);
      }
      else if (ledstatus == "OFF") {
        digitalWrite(13, LOW);
      }
      Serial.print(ledstatus);
    }
    else {
      Serial.println(F("Failed"));
    }
  }
  Serial.println(F("While-end"));
}

void MQTT_connect() {
  if (mqtt.connected()) 
  {
    return;
  }
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
    delay(5000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}
