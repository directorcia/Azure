#include <Arduino.h>
#include <WiFiS3.h>
#include <Arduino_LED_Matrix.h>
#include <ArduinoHttpClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "io_configs.h"

ArduinoLEDMatrix matrix;
WiFiClient wifiClient;
HttpClient httpClient = HttpClient(wifiClient, AIO_SERVER, 80);

// MQTT setup
WiFiClient mqttClient;
Adafruit_MQTT_Client mqtt(&mqttClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoffbutton");

// Global variables to track connection status
bool internetConnected = false;
bool wifiConnected = false;
bool adafruitConnected = false;
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
bool testAdafruitIO();
bool createAdafruitFeed(String feedName);
String readAdafruitFeed(String feedName);
bool sendToAdafruitFeed(String feedName, String value);
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
  delay(3000); // Wait for serial connection
  
  // Set HTTP timeout only for initial testing (not used for main operation)
  httpClient.connectionKeepAlive();
  httpClient.setTimeout(5000); // Longer timeout for initial setup only
  
  Serial.println("=== Arduino Uno R4 WiFi - MQTT Real-Time System ===");
  Serial.println("Date: July 13, 2025");
  Serial.println("Mode: MQTT-ONLY for maximum speed");
  Serial.println("================================================");
  
  // Initialize LED matrix
  matrix.begin();
  Serial.println("LED matrix initialized");
  
  // Test LED matrix first
  Serial.println("\nTesting LED matrix functionality...");
  testLEDMatrix();
  
  // Show diagnostic pattern
  byte diag[8][12] = {
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1}
  };
  matrix.renderBitmap(diag, 8, 12);
  
  Serial.println("Step 1: Board Identification");
  Serial.println("----------------------------");
  Serial.print("Board: ");
  #ifdef ARDUINO_UNOR4_WIFI
    Serial.println("Arduino Uno R4 WiFi ✓");
  #else
    Serial.println("NOT Arduino Uno R4 WiFi ✗");
    Serial.println("ERROR: Wrong board type detected!");
    Serial.println("Solution: Check your board selection in platformio.ini");
    Serial.println("Should be: board = uno_r4_wifi");
  #endif
  
  Serial.println("\nStep 2: WiFi Module Detection");
  Serial.println("-----------------------------");
  
  // Test 1: Direct status check
  Serial.print("Test 1 - Initial WiFi.status(): ");
  int status = WiFi.status();
  Serial.print(status);
  Serial.print(" (");
  printWiFiStatus(status);
  Serial.println(")");
  
  if (status == WL_NO_SHIELD) {
    Serial.println("❌ WiFi module not detected!");
    
    // Test 2: Try multiple initialization attempts
    Serial.println("\nTest 2 - Attempting WiFi wake-up...");
    for (int i = 0; i < 5; i++) {
      Serial.print("Attempt ");
      Serial.print(i + 1);
      Serial.print("/5: ");
      
      // Try different initialization methods
      if (i == 0) {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      } else if (i == 1) {
        WiFi.disconnect();
      } else if (i == 2) {
        // Try a different dummy connection
        WiFi.begin("test", "test");
      }
      
      delay(2000);
      status = WiFi.status();
      Serial.print(status);
      Serial.print(" (");
      printWiFiStatus(status);
      Serial.println(")");
      
      if (status != WL_NO_SHIELD) {
        Serial.println("✅ WiFi module responded!");
        break;
      }
    }
  } else {
    Serial.println("✅ WiFi module detected!");
  }
  
  // Test 3: Firmware version check
  Serial.println("\nStep 3: Firmware Version Check");
  Serial.println("------------------------------");
  String firmware = WiFi.firmwareVersion();
  if (firmware.length() > 0 && firmware != "0.0.0") {
    Serial.print("✅ WiFi firmware version: ");
    Serial.println(firmware);
  } else {
    Serial.println("❌ No valid firmware version returned");
  }
  
  // Test 4: Network scan capability
  Serial.println("\nStep 4: Network Scan Test");
  Serial.println("-------------------------");
  if (WiFi.status() != WL_NO_SHIELD) {
    Serial.println("Scanning for networks...");
    int networks = WiFi.scanNetworks();
    if (networks > 0) {
      Serial.print("✅ Found ");
      Serial.print(networks);
      Serial.println(" networks:");
      for (int i = 0; i < min(networks, 5); i++) {
        Serial.print("  ");
        Serial.print(i + 1);
        Serial.print(". ");
        Serial.print(WiFi.SSID(i));
        Serial.print(" (");
        Serial.print(WiFi.RSSI(i));
        Serial.println(" dBm)");
      }
    } else {
      Serial.println("❌ No networks found");
    }
  } else {
    Serial.println("❌ Cannot scan - WiFi module not available");
  }
  
  // Test 5: Actual WiFi Connection
  Serial.println("\nStep 5: WiFi Connection Test");
  Serial.println("----------------------------");
  if (WiFi.status() != WL_NO_SHIELD) {
    Serial.print("Connecting to: ");
    Serial.println(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int connectionAttempts = 0;
    while (WiFi.status() != WL_CONNECTED && connectionAttempts < 20) {
      delay(500);
      Serial.print(".");
      connectionAttempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.println("✅ WiFi connected successfully!");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print("Signal strength: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      
      // Test 6: Internet Connectivity
      Serial.println("\nStep 6: Internet Connectivity Test");
      Serial.println("----------------------------------");
      if (testInternetConnectivity()) {
        Serial.println("✅ Internet connection successful!");
        internetConnected = true;
        
        // Test 7: Adafruit IO Connectivity
        Serial.println("\nStep 7: Adafruit IO Test");
        Serial.println("------------------------");
        if (testAdafruitIO()) {
          Serial.println("✅ Adafruit IO connection successful!");
          adafruitConnected = true;
          
          // Initialize MQTT for real-time updates (PRIMARY COMMUNICATION)
          Serial.println("\nStep 8: Setting up MQTT for INSTANT real-time updates");
          Serial.println("------------------------------------------------------");
          
          // Subscribe to the feed first
          mqtt.subscribe(&onoffbutton);
          Serial.println("✅ Subscribed to onoffbutton feed");
          
          // Connect to MQTT
          MQTT_connect();
          if (mqtt.connected()) {
            Serial.println("✅ MQTT connection successful!");
            Serial.println("🚀 MQTT-ONLY mode activated for maximum speed!");
            mqttConnected = true;
            lastMQTTActivity = millis(); // Initialize activity timer
          } else {
            Serial.println("❌ MQTT connection failed - system will retry automatically");
          }
          
          // Get initial value via HTTP (one-time only)
          Serial.println("Getting initial value via HTTP (one-time setup)...");
          String dummyData = readAdafruitFeed("onoffbutton");
          Serial.print("Initial value: ");
          Serial.println(dummyData);
          lastKnownValue = dummyData; // Cache initial value
          
          // Display initial value
          if (dummyData.length() > 0) {
            displayPatternFromData(dummyData);
            Serial.print("Displaying initial value on matrix: ");
            Serial.println(dummyData);
          } else {
            // No initial data, show a default pattern to test matrix
            Serial.println("No initial data found, displaying test pattern");
            displayPatternFromData("1"); // Display digit 1 as a test
          }
          
          Serial.println("\n🎯 System ready! All updates will now come via MQTT instantly.");
          Serial.println("Monitoring 'onoffbutton' feed via MQTT for instant response...");
          
          Serial.println("\n" + String('=', 60));
          Serial.println("🚀 SYSTEM READY TO RECEIVE COMMANDS 🚀");
          Serial.println("Status: MQTT connected and subscribed to feed");
          Serial.println("Action: Change values in Adafruit IO dashboard now!");
          Serial.println("Expected response: Instant matrix updates");
          Serial.println(String('=', 60));
          Serial.println();
          
          // Don't show checkmark - keep the feed value visible
        } else {
          Serial.println("❌ Adafruit IO connection failed!");
          // Show WiFi success but no feed data
          showInternetConnectedPattern();
        }
      } else {
        Serial.println("❌ Internet connection failed!");
        wifiConnected = true; // WiFi works but no internet
        showWiFiOnlyPattern();
      }
    } else {
      Serial.println();
      Serial.println("❌ WiFi connection failed!");
    }
  } else {
    Serial.println("❌ Cannot connect - WiFi module not available");
  }
  
  Serial.println("\n=== DIAGNOSTIC SUMMARY ===");
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("❌ WiFi MODULE NOT FOUND");
    Serial.println("\nPossible Solutions:");
    Serial.println("1. VERIFY HARDWARE:");
    Serial.println("   - Ensure you have Arduino Uno R4 WIFI (not regular R4)");
    Serial.println("   - Check for ESP32-S3 module on the board");
    Serial.println("   - Look for 'WiFi' text printed on the board");
    Serial.println("");
    Serial.println("2. UPDATE FIRMWARE:");
    Serial.println("   - Use Arduino IDE to update ESP32-S3 firmware");
    Serial.println("   - Go to Tools > WiFi101 / WiFiNINA Firmware Updater");
    Serial.println("");
    Serial.println("3. PLATFORMIO FIXES:");
    Serial.println("   - Update board package: pio pkg update");
    Serial.println("   - Clean build: pio run -t clean");
    Serial.println("   - Try Arduino IDE instead as a test");
    Serial.println("");
    Serial.println("4. RESET PROCEDURES:");
    Serial.println("   - Press and hold reset button for 10 seconds");
    Serial.println("   - Power cycle the board completely");
    Serial.println("   - Try different USB port/cable");
    
    // Show failed pattern
    byte failed[8][12] = {
      {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
      {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
      {0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0},
      {0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0},
      {0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0}
    };
    matrix.renderBitmap(failed, 8, 12);
    
  } else if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi MODULE DETECTED BUT NOT CONNECTED");
    Serial.println("You can now proceed with WiFi connection setup!");
    
    // Show partial success pattern
    byte partial[8][12] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
      {0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
      {0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0},
      {0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0}
    };
    matrix.renderBitmap(partial, 8, 12);
  } else {
    Serial.println("✅ FULL CONNECTIVITY ACHIEVED");
    Serial.println("WiFi connected and internet access confirmed!");
    // LED pattern already set by internet test
  }
  
  Serial.println("\n=== Diagnostic Complete ===");
}

void loop() {
  static unsigned long lastHeartbeat = 0;
  static bool ledState = false;
  static unsigned long loopCounter = 0;
  
  loopCounter++;
  
  // Debug: Show loop activity every 10 seconds
  if (loopCounter % 10000 == 0) {
    Serial.print("Loop running, MQTT connected: ");
    Serial.print(mqttConnected ? "YES" : "NO");
    Serial.print(", last activity: ");
    Serial.print(millis() - lastMQTTActivity);
    Serial.println("ms ago");
  }
  
  // Status message every 30 seconds when connected and ready
  static unsigned long lastStatusMessage = 0;
  if (mqttConnected && millis() - lastStatusMessage >= 30000) {
    lastStatusMessage = millis();
    Serial.println("\n✅ LISTENING FOR COMMANDS - System ready and waiting for Adafruit IO updates");
    Serial.print("Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");
  }
  
  // MQTT-ONLY handling for maximum real-time performance
  if (mqttConnected) {
    // Ensure connection is maintained
    MQTT_connect();
    
    // Simple and reliable message reading (proven approach)
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(100))) { // Short timeout for responsiveness
      lastMQTTActivity = millis(); // Track when we last received activity
      
      Serial.println("\n" + String('=', 50));
      Serial.print("🔔 MQTT MESSAGE RECEIVED at ");
      Serial.print(millis());
      Serial.println("ms");
      
      if (subscription == &onoffbutton) {
        // Get the data from the subscription
        char* receivedData = (char *)onoffbutton.lastread;
        if (receivedData != NULL) {
          String feedData = String(receivedData);
          feedData.trim(); // Remove any whitespace
          
          Serial.println("\n📡 COMMAND RECEIVED:");
          Serial.print("  Raw data: '");
          Serial.print(receivedData);
          Serial.println("'");
          Serial.print("  Processed: '");
          Serial.print(feedData);
          Serial.print("' (len: ");
          Serial.print(feedData.length());
          Serial.println(")");
          Serial.print("  Last known: '");
          Serial.print(lastKnownValue);
          Serial.println("'");
          
          // Always process the command for immediate response
          if (feedData.length() > 0) {
            Serial.print("⚡ PROCESSING COMMAND: ");
            Serial.print(feedData);
            Serial.println(" -> UPDATING MATRIX NOW");
            
            displayPatternFromData(feedData);
            
            // Update lastKnownValue AFTER successful processing
            lastKnownValue = feedData;
            
            Serial.println("✅ COMMAND PROCESSED SUCCESSFULLY");
          } else {
            Serial.println("⚠️  WARNING: Empty value received, ignoring");
          }
        } else {
          Serial.println("❌ ERROR: Received NULL data pointer");
        }
      } else {
        Serial.println("❌ Received message from unknown subscription!");
      }
      Serial.println(String('=', 50));
    }
  } else if (adafruitConnected && millis() - lastMQTTReconnect >= MQTT_RECONNECT_INTERVAL) {
    // Try to reconnect MQTT using the proven approach
    lastMQTTReconnect = millis();
    Serial.print("Attempting MQTT reconnection... ");
    
    // Re-subscribe and connect
    mqtt.subscribe(&onoffbutton);
    MQTT_connect();
    
    if (mqtt.connected()) {
      Serial.println("MQTT reconnected!");
      mqttConnected = true;
      lastMQTTActivity = millis(); // Reset activity timer on reconnection
      
      Serial.println("\n" + String('=', 50));
      Serial.println("🔄 RECONNECTED - READY FOR COMMANDS 🔄");
      Serial.println("MQTT connection restored and subscribed");
      Serial.println(String('=', 50));
      Serial.println();
    } else {
      Serial.println("MQTT reconnection failed, will retry in 5 seconds");
      mqttConnected = false;
    }
  }
  
  // Only show heartbeat if we don't have a successful connection pattern to display
  // and we haven't received Adafruit data recently
  if (!internetConnected && !wifiConnected && !adafruitConnected) {
    // Heartbeat blink to show board is alive when no connection
    if (millis() - lastHeartbeat >= 2000) {
      lastHeartbeat = millis();
      ledState = !ledState;
      
      if (ledState) {
        byte heartbeat[8][12] = {
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
          {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
          {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
        };
        matrix.renderBitmap(heartbeat, 8, 12);
      } else {
        matrix.clear();
      }
    }
  }
  
  // Keep WiFi connection alive
  if (WiFi.status() != WL_CONNECTED && internetConnected) {
    Serial.println("WiFi connection lost, attempting to reconnect...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// Function to test Adafruit IO connectivity
bool testAdafruitIO() {
  Serial.println("Testing Adafruit IO connectivity...");
  
  // Test connection to Adafruit IO
  String path = "/api/v2/" + String(AIO_USERNAME) + "/feeds";
  
  Serial.print("Making GET request to: ");
  Serial.print(AIO_SERVER);
  Serial.println(path);
  
  httpClient.beginRequest();
  httpClient.get(path);
  httpClient.sendHeader("X-AIO-Key", AIO_KEY);
  httpClient.endRequest();
  
  // Wait for response
  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();
  
  Serial.print("HTTP Status Code: ");
  Serial.println(statusCode);
  
  if (statusCode == 200) {
    Serial.println("✅ Successfully connected to Adafruit IO!");
    Serial.println("Available feeds response (first 200 chars):");
    Serial.println(response.substring(0, 200));
    return true;
  } else {
    Serial.println("❌ Failed to connect to Adafruit IO");
    Serial.print("Response: ");
    Serial.println(response);
    return false;
  }
}

// Function to create an Adafruit IO feed
bool createAdafruitFeed(String feedName) {
  Serial.print("Creating Adafruit IO feed: ");
  Serial.println(feedName);
  
  String path = "/api/v2/" + String(AIO_USERNAME) + "/feeds";
  String postData = "{\"name\":\"" + feedName + "\",\"description\":\"Arduino R4 WiFi feed for " + feedName + "\"}";
  
  httpClient.beginRequest();
  httpClient.post(path);
  httpClient.sendHeader("Content-Type", "application/json");
  httpClient.sendHeader("X-AIO-Key", AIO_KEY);
  httpClient.sendHeader("Content-Length", postData.length());
  httpClient.endRequest();
  httpClient.write((const byte*)postData.c_str(), postData.length());
  
  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();
  
  Serial.print("Create Feed Status Code: ");
  Serial.println(statusCode);
  
  if (statusCode == 201 || statusCode == 200) {
    Serial.println("✅ Successfully created feed!");
    return true;
  } else if (statusCode == 422) {
    Serial.println("ℹ️ Feed already exists");
    return true; // Feed exists, which is fine
  } else {
    Serial.println("❌ Failed to create feed");
    Serial.print("Response: ");
    Serial.println(response);
    return false;
  }
}

// Function to read data from a specific Adafruit IO feed
String readAdafruitFeed(String feedName) {
  // Use data endpoint with limit=1 to get latest data faster
  String path = "/api/v2/" + String(AIO_USERNAME) + "/feeds/" + feedName + "/data?limit=1&_t=" + String(millis());
  
  httpClient.beginRequest();
  httpClient.get(path);
  httpClient.sendHeader("X-AIO-Key", AIO_KEY);
  httpClient.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  httpClient.sendHeader("Pragma", "no-cache");
  httpClient.sendHeader("Expires", "0");
  httpClient.sendHeader("If-Modified-Since", "Thu, 01 Jan 1970 00:00:00 GMT");
  httpClient.endRequest();
  
  // Ultra-fast timeout for maximum speed
  unsigned long startTime = millis();
  const unsigned long timeout = 50; // 50ms for ultra-fast response
  
  // Wait for response with timeout
  while (!httpClient.available() && (millis() - startTime < timeout)) {
    delayMicroseconds(10); // Absolute minimal delay
  }
  
  if (millis() - startTime >= timeout) {
    return ""; // Fast fail
  }
  
  int statusCode = httpClient.responseStatusCode();
  
  if (statusCode == 200) {
    String response = httpClient.responseBody();
    
    // Parse array format: [{"value":"26",...}]
    int valueStart = response.indexOf("\"value\":\"");
    if (valueStart >= 0) {
      valueStart += 9;
      int valueEnd = response.indexOf("\"", valueStart);
      if (valueEnd > valueStart && (valueEnd - valueStart) < 20) {
        return response.substring(valueStart, valueEnd);
      }
    }
  } else {
    // Fast fail - consume response buffer quickly
    httpClient.responseBody();
  }
  
  return "";
}

// Function to send data to Adafruit IO feed
bool sendToAdafruitFeed(String feedName, String value) {
  Serial.print("Sending to feed ");
  Serial.print(feedName);
  Serial.print(": ");
  Serial.println(value);
  
  String path = "/api/v2/" + String(AIO_USERNAME) + "/feeds/" + feedName + "/data";
  String postData = "{\"value\":\"" + value + "\"}";
  
  httpClient.beginRequest();
  httpClient.post(path);
  httpClient.sendHeader("Content-Type", "application/json");
  httpClient.sendHeader("X-AIO-Key", AIO_KEY);
  httpClient.sendHeader("Content-Length", postData.length());
  httpClient.endRequest();
  httpClient.write((const byte*)postData.c_str(), postData.length());
  
  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();
  
  Serial.print("Send Status Code: ");
  Serial.println(statusCode);
  
  if (statusCode == 200 || statusCode == 201) {
    Serial.println("✅ Successfully sent data to Adafruit IO!");
    return true;
  } else {
    Serial.println("❌ Failed to send data to Adafruit IO");
    Serial.print("Response: ");
    Serial.println(response);
    return false;
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