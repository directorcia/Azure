/*
  Arduino Uno R4 WiFi - Connect to WiFi and Display Message on LED Matrix
  
  This program connects the Arduino Uno R4 WiFi to a WiFi network and
  displays a scrolling message on the built-in LED matrix.
*/

#include <WiFiS3.h>
#include "Arduino_LED_Matrix.h"
#include "WiFiConnect.h"

// Create an instance of the LED Matrix
ArduinoLEDMatrix matrix;

// WiFi credentials - replace with your network details
const char* ssid = "YourWiFiName";     // Replace with your WiFi network name
const char* password = "YourPassword";  // Replace with your WiFi password

// Create WiFi connection manager
WiFiConnect wifi(ssid, password);

// Messages to display based on connection status
const uint8_t successPattern[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0 },
  { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

// LED Matrix patterns for different messages
const uint8_t wifiPattern[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

const uint8_t connectedPattern[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0 },
  { 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

const uint8_t disconnectedPattern[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

// Function declarations
void displayScrollingText(String text);
void connectToWiFi();

// Display scrolling text on the LED matrix
void displayScrollingText(String text) {
  // In a real implementation, we would create animated frames for scrolling
  // For simplicity, we'll just display static text with delays between characters
  
  Serial.print("Displaying: ");
  Serial.println(text);
  
  // Clear the matrix
  matrix.clear();
  
  // Display each character of the text with a delay
  for (int i = 0; i < text.length(); i++) {
    // In a real implementation, you would set specific LED patterns for each character
    // Here we're just toggling the LEDs in a pattern to simulate text
    matrix.loadFrame(connectedPattern);
    delay(300);
    matrix.clear();
    delay(200);
  }
  
  // In actual implementation, you would use Arduino_LED_Matrix library's
  // scrolling text capabilities to create a proper scrolling animation
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);
  
  // Begin WiFi connection
  if (wifi.connect(10000)) { // 10 second timeout
    Serial.println("WiFi connected successfully!");
    Serial.print("IP address: ");
    Serial.println(wifi.getIP());
    Serial.print("Signal strength: ");
    Serial.print(wifi.getSignalStrength());
    Serial.println(" dBm");
    
    // Show success checkmark
    matrix.loadFrame(successPattern);
    delay(2000);
    
    // Display IP address
    String ip = wifi.getIP().toString();
    displayScrollingText("IP: " + ip);
  } else {
    Serial.println("Failed to connect to WiFi. Please check your credentials.");
    displayScrollingText("WiFi FAILED");
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial && millis() < 5000); // Wait up to 5 seconds for Serial to connect
  
  Serial.println("Arduino Uno R4 WiFi - LED Matrix Demo");
  
  // Initialize the LED Matrix
  matrix.begin();
  
  // Display WiFi icon on the matrix
  matrix.loadFrame(wifiPattern);
  delay(1000);
  
  // Connect to WiFi
  connectToWiFi();
}

void loop() {
  // Check if we're still connected
  if (!wifi.isConnected()) {
    Serial.println("WiFi connection lost. Attempting to reconnect...");
    matrix.loadFrame(disconnectedPattern);
    delay(1000);
    connectToWiFi();
  }
  
  // Display scrolling message and WiFi signal strength every 10 seconds
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 10000) { // Every 10 seconds
    
    // Display "CONNECTED" text 
    matrix.loadFrame(connectedPattern);
    delay(2000);
    
    // Display WiFi signal strength
    int signalStrength = wifi.getSignalStrength();
    String signalMsg = "WiFi: " + String(signalStrength) + " dBm";
    displayScrollingText(signalMsg);
    
    // Display IP address
    String ip = wifi.getIP().toString();
    displayScrollingText("IP: " + ip);
    
    lastDisplay = millis();
  }
  
  delay(100);
}

