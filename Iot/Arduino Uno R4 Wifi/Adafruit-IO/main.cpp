#include <Arduino.h>
#include <WiFiS3.h>
#include <Arduino_LED_Matrix.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "io_configs.h"

ArduinoLEDMatrix matrix;

// MQTT setup - ONLY communication method
WiFiClient mqttClient;
Adafruit_MQTT_Client mqtt(&mqttClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoffbutton");

// Global variables to track connection status
bool internetConnected = false;
bool wifiConnected = false;
bool mqttConnected = false;
String lastKnownValue = ""; // Cache last value to avoid redundant displays
unsigned long lastMQTTReconnect = 0;
unsigned long lastMQTTActivity = 0; // Track last MQTT message received
const unsigned long MQTT_RECONNECT_INTERVAL = 5000; // Reconnect every 5 seconds if needed

// Function declarations
void printWiFiStatus(int status);
bool testInternetConnectivity();
void showInternetConnectedPattern();
void showWiFiOnlyPattern();
void testLEDMatrix();
void displayPatternFromData(String data);
void displayNumber(int number);
void displayMultiDigitNumber(int number);
void MQTT_connect();
void processMQTTMessage();

void printWiFiStatus(int status) {
  switch (status) {
    case WL_NO_SHIELD:
      Serial.print("WL_NO_SHIELD - WiFi module not found");
      break;
    case WL_IDLE_STATUS:
      Serial.print("WL_IDLE_STATUS - WiFi is idle");
      break;
    case WL_NO_SSID_AVAIL:
      Serial.print("WL_NO_SSID_AVAIL - No SSID available");
      break;
    case WL_SCAN_COMPLETED:
      Serial.print("WL_SCAN_COMPLETED - Scan completed");
      break;
    case WL_CONNECTED:
      Serial.print("WL_CONNECTED - Connected");
      break;
    case WL_CONNECT_FAILED:
      Serial.print("WL_CONNECT_FAILED - Connection failed");
      break;
    case WL_CONNECTION_LOST:
      Serial.print("WL_CONNECTION_LOST - Connection lost");
      break;
    case WL_DISCONNECTED:
      Serial.print("WL_DISCONNECTED - Disconnected");
      break;
    default:
      Serial.print("UNKNOWN STATUS");
      break;
  }
}

bool testInternetConnectivity() {
  Serial.println("Testing connection to Google DNS (8.8.8.8)...");
  
  WiFiClient client;
  
  // Try to connect to Google's public DNS on port 53
  if (client.connect("8.8.8.8", 53)) {
    Serial.println("✅ Successfully connected to 8.8.8.8:53");
    client.stop();
    return true;
  }
  
  Serial.println("❌ Failed to connect to 8.8.8.8:53");
  
  // Try alternative: Cloudflare DNS
  Serial.println("Testing connection to Cloudflare DNS (1.1.1.1)...");
  if (client.connect("1.1.1.1", 53)) {
    Serial.println("✅ Successfully connected to 1.1.1.1:53");
    client.stop();
    return true;
  }
  
  Serial.println("❌ Failed to connect to 1.1.1.1:53");
  return false;
}

void showInternetConnectedPattern() {
  Serial.println("DEBUG: Displaying checkmark on LED matrix...");
  
  // Large checkmark for successful internet connection
  byte checkmark[8][12] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
    {0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
    {0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0}
  };
  
  matrix.renderBitmap(checkmark, 8, 12);
  Serial.println("DEBUG: Checkmark pattern sent to matrix");
  
  // Add a delay to ensure pattern is visible
  delay(1000);
  Serial.println("DEBUG: Checkmark should now be visible on matrix");
}

void showWiFiOnlyPattern() {
  Serial.println("DEBUG: Displaying WiFi-only pattern on LED matrix...");
  
  // WiFi symbol for WiFi connected but no internet
  byte wifiOnly[8][12] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0},
    {0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0},
    {0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0},
    {0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0}
  };
  
  matrix.renderBitmap(wifiOnly, 8, 12);
  Serial.println("DEBUG: WiFi-only pattern sent to matrix");
  
  // Add a delay to ensure pattern is visible
  delay(1000);
  Serial.println("DEBUG: WiFi-only pattern should now be visible on matrix");
}

void testLEDMatrix() {
  Serial.println("DEBUG: Testing LED matrix with simple pattern...");
  
  // Simple test pattern - all LEDs on
  byte testPattern[8][12];
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 12; col++) {
      testPattern[row][col] = 1;
    }
  }
  
  matrix.renderBitmap(testPattern, 8, 12);
  delay(1000);
  matrix.clear();
  delay(500);
  
  Serial.println("DEBUG: LED matrix test complete");
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Reduced from 3000ms
  
  Serial.println("=== Arduino Uno R4 WiFi - MQTT Real-Time System ===");
  Serial.println("Date: July 19, 2025");
  Serial.println("Mode: FAST BOOT + MQTT-ONLY");
  Serial.println("================================================");
  
  // Initialize LED matrix
  matrix.begin();
  Serial.println("✅ LED matrix ready");
  
  // Quick LED test - just flash once
  byte testPattern[8][12];
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 12; col++) {
      testPattern[row][col] = 1;
    }
  }
  matrix.renderBitmap(testPattern, 8, 12);
  delay(200); // Quick flash
  matrix.clear();
  
  Serial.println("🚀 FAST BOOT: Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Fast WiFi connection with timeout
  int connectionAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && connectionAttempts < 15) { // Reduced from 20
    delay(300); // Reduced from 500ms
    Serial.print(".");
    connectionAttempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("✅ WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
    internetConnected = true;
    
    // Immediate MQTT setup - no additional tests
    Serial.println("🚀 MQTT setup...");
    mqtt.subscribe(&onoffbutton);
    MQTT_connect();
    
    if (mqtt.connected()) {
      Serial.println("✅ MQTT ready!");
      mqttConnected = true;
      lastMQTTActivity = millis();
      
      // Show checkmark when ready
      showInternetConnectedPattern();
      
      Serial.println("\n" + String('=', 50));
      Serial.println("🚀 SYSTEM READY - FAST BOOT COMPLETE 🚀");
      Serial.println("Waiting for Adafruit IO feed updates...");
      Serial.println(String('=', 50));
    } else {
      Serial.println("❌ MQTT failed - will retry");
      // Show X pattern for error
      byte errorPattern[8][12] = {
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}
      };
      matrix.renderBitmap(errorPattern, 8, 12);
    }
  } else {
    Serial.println();
    Serial.println("❌ WiFi failed");
    // Show X pattern for WiFi error
    byte errorPattern[8][12] = {
      {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
      {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
      {0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0},
      {0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0},
      {0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0},
      {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}
    };
    matrix.renderBitmap(errorPattern, 8, 12);
  }
}

void loop() {
  static unsigned long loopCounter = 0;
  loopCounter++;
  
  // Very minimal status output - only every 30 seconds
  static unsigned long lastStatusMessage = 0;
  if (mqttConnected && millis() - lastStatusMessage >= 30000) {
    lastStatusMessage = millis();
    Serial.println("✅ MQTT listening...");
  }
  
  // FAST MQTT handling for maximum responsiveness
  if (mqttConnected) {
    // Quick connection check
    if (!mqtt.connected()) {
      Serial.println("⚠️ MQTT lost, reconnecting...");
      mqttConnected = false;
      return;
    }
    
    // Fast message reading with minimal timeout
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(50))) { // Even shorter timeout for speed
      lastMQTTActivity = millis();
      
      if (subscription == &onoffbutton) {
        char* receivedData = (char *)onoffbutton.lastread;
        if (receivedData != NULL) {
          String feedData = String(receivedData);
          feedData.trim();
          
          Serial.print("📡 RECEIVED: '");
          Serial.print(feedData);
          Serial.println("' -> Updating matrix");
          
          if (feedData.length() > 0) {
            // Immediate matrix update - no LED flash delay
            displayPatternFromData(feedData);
            lastKnownValue = feedData;
            Serial.println("✅ Matrix updated");
          }
        }
      }
    }
  } else if (internetConnected && millis() - lastMQTTReconnect >= MQTT_RECONNECT_INTERVAL) {
    // Fast MQTT reconnection
    lastMQTTReconnect = millis();
    Serial.print("Reconnecting MQTT... ");
    
    mqtt.subscribe(&onoffbutton);
    MQTT_connect();
    
    if (mqtt.connected()) {
      Serial.println("✅ Reconnected!");
      mqttConnected = true;
      lastMQTTActivity = millis();
    } else {
      Serial.println("❌ Failed");
      mqttConnected = false;
    }
  }
  
  // Keep WiFi alive
  if (WiFi.status() != WL_CONNECTED && internetConnected) {
    Serial.println("WiFi lost, reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// Function to display custom pattern based on received data
void displayPatternFromData(String data) {
  Serial.println("\n🎯 ==== DISPLAY PATTERN FUNCTION CALLED ====");
  Serial.print("💡 Input data: '");
  Serial.print(data);
  Serial.print("' (length: ");
  Serial.print(data.length());
  Serial.println(")");
  
  int inputValue = data.toInt();
  Serial.print("🔢 Parsed as integer: ");
  Serial.println(inputValue);
  
  // Handle special text inputs first
  if (data.equalsIgnoreCase("on") || data.equalsIgnoreCase("true")) {
    byte onPattern[8][12] = {
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
      {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
      {1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
      {1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1},
      {1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1},
      {1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
      {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };
    matrix.renderBitmap(onPattern, 8, 12);
    // Keep pattern visible - no clear, no delay
    return;
  } 
  
  if (data.equalsIgnoreCase("off") || data.equalsIgnoreCase("false")) {
    byte offPattern[8][12] = {
      {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
      {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
      {0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0},
      {0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0},
      {0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0},
      {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}
    };
    matrix.renderBitmap(offPattern, 8, 12);
    // Keep pattern visible - no clear, no delay
    return;
  }
  
  // Reverse mapping: API input -> Display digit
  // 16->1, 17->2, 18->3, 20->4, 21->5, 22->6, 24->7, 25->8, 26->9, 12->0
  int displayDigit = -1;
  
  switch (inputValue) {
    case 16: displayDigit = 1; break;
    case 17: displayDigit = 2; break;
    case 18: displayDigit = 3; break;
    case 20: displayDigit = 4; break;
    case 21: displayDigit = 5; break;
    case 22: displayDigit = 6; break;
    case 24: displayDigit = 7; break;
    case 25: displayDigit = 8; break;
    case 26: displayDigit = 9; break;
    case 12: displayDigit = 0; break;
    default:
      // If it's a single digit 0-9, display it directly
      if (inputValue >= 0 && inputValue <= 9) {
        displayDigit = inputValue;
      }
      break;
  }
  
  if (displayDigit >= 0 && displayDigit <= 9) {
    Serial.print("Displaying digit: ");
    Serial.println(displayDigit);
    displayNumber(displayDigit);
    // Keep number visible - no clear, no delay
  } else {
    Serial.print("Unknown data, showing question mark for: ");
    Serial.println(data);
    // Unknown data - show question mark pattern
    byte unknownPattern[8][12] = {
      {0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},
      {0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
    matrix.renderBitmap(unknownPattern, 8, 12);
    // Keep pattern visible - no clear, no delay
  }
}

// Function to display numbers 0-9 on the LED matrix
void displayNumber(int number) {
  Serial.print("📱 UPDATING MATRIX: Displaying number ");
  Serial.println(number);
  
  // Define patterns for digits 0-9 (8x12 matrix)
  byte digitPatterns[10][8][12] = {
    // Number 0
    {
      {0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0},
      {0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0},
      {0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0}
    },
    // Number 1
    {
      {0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0},
      {0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
      {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0}
    },
    // Number 2
    {
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
      {0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0},
      {0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0}
    },
    // Number 3
    {
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0}
    },
    // Number 4
    {
      {0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0},
      {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
      {0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0},
      {0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0},
      {0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0},
      {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0}
    },
    // Number 5
    {
      {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0}
    },
    // Number 6
    {
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0}
    },
    // Number 7
    {
      {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0}
    },
    // Number 8
    {
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0}
    },
    // Number 9
    {
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0}
    }
  };
  
  if (number >= 0 && number <= 9) {
    Serial.print("✅ MATRIX UPDATE: Calling renderBitmap for digit ");
    Serial.println(number);
    matrix.renderBitmap(digitPatterns[number], 8, 12);
    Serial.println("✅ MATRIX UPDATE: renderBitmap call completed");
  } else {
    // Show error pattern briefly
    byte errorPattern[8][12] = {
      {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
      {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
      {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
      {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
      {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
      {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
      {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
      {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1}
    };
    matrix.renderBitmap(errorPattern, 8, 12);
    // Keep error pattern visible - no delay, no clear
  }
}

// Function to display multi-digit numbers on the LED matrix
void displayMultiDigitNumber(int number) {
  Serial.print("Displaying multi-digit number: ");
  Serial.println(number);
  
  String numStr = String(number);
  
  if (numStr.length() == 1) {
    // Single digit, just display it
    displayNumber(number);
    delay(3000);
    matrix.clear();
    Serial.println("DEBUG: Matrix cleared after single digit");
  } else {
    // Multi-digit number, show each digit in sequence
    Serial.print("Multi-digit number detected with ");
    Serial.print(numStr.length());
    Serial.println(" digits");
    
    for (int i = 0; i < numStr.length(); i++) {
      int digit = numStr.charAt(i) - '0'; // Convert char to int
      if (digit >= 0 && digit <= 9) {
        Serial.print("Showing digit ");
        Serial.print(i + 1);
        Serial.print(" of ");
        Serial.print(numStr.length());
        Serial.print(": ");
        Serial.println(digit);
        
        displayNumber(digit);
        delay(2000); // Show each digit for 2 seconds
        
        // Brief clear between digits (except after the last digit)
        if (i < numStr.length() - 1) {
          matrix.clear();
          delay(500);
        }
      }
    }
    
    // Keep last digit visible for 1 more second, then clear
    delay(1000);
    matrix.clear();
    Serial.println("DEBUG: Matrix cleared after multi-digit number");
  }
}

// MQTT Helper Functions - Proven approach from working project
void MQTT_connect() {
  if (mqtt.connected()) {
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

void processMQTTMessage() {
  // This function is called in the main loop to check for messages
  // The actual message processing is handled in the loop() function
}