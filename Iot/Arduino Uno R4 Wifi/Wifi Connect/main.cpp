/*
  Arduino Uno R4 WiFi - Simple WiFi Connection
  
  This program connects the Arduino Uno R4 WiFi to a WiFi network
  and displays connection information on the Serial Monitor.
*/

#include <WiFiS3.h>
#include "WiFiConnect.h"

// WiFi credentials - replace with your network details
const char* ssid = "YourWiFiName";     // Replace with your WiFi network name
const char* password = "YourPassword";  // Replace with your WiFi password

// Create WiFi connection manager
WiFiConnect wifi(ssid, password);

void connectToWiFi() {
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);
  
  // Begin WiFi connection
  if (wifi.connect(10000)) { // 10 second timeout
    Serial.println("WiFi connected successfully!");
    Serial.print("IP address: ");
    Serial.println(wifi.getIP());
    Serial.print("MAC address: ");
    Serial.println(wifi.getMacAddress());
    Serial.print("Signal strength: ");
    Serial.print(wifi.getSignalStrength());
    Serial.println(" dBm");
  } else {
    Serial.println("Failed to connect to WiFi. Please check your credentials.");
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial && millis() < 5000); // Wait up to 5 seconds for Serial to connect
  
  Serial.println("Arduino Uno R4 WiFi - Simple WiFi Connection");
  
  // Connect to WiFi
  connectToWiFi();
}

void loop() {
  // Check if we're still connected
  if (!wifi.isConnected()) {
    Serial.println("WiFi connection lost. Attempting to reconnect...");
    connectToWiFi();
  }
  
  // Display connection information every 30 seconds
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 30000) { // Every 30 seconds
    Serial.println("WiFi status: Connected");
    Serial.print("IP address: ");
    Serial.println(wifi.getIP());
    Serial.print("Signal strength: ");
    Serial.print(wifi.getSignalStrength());
    Serial.println(" dBm");
    
    lastCheck = millis();
  }
  
  delay(100);
}

