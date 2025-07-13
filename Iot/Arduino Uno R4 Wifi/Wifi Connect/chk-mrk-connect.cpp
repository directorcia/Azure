#include <Arduino.h>
#include <WiFiS3.h>
#include <Arduino_LED_Matrix.h>
#include "io_configs.h"

ArduinoLEDMatrix matrix;

// Global variables to track connection status
bool internetConnected = false;
bool wifiConnected = false;

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
  
  Serial.println("=== Arduino Uno R4 WiFi Diagnostic Tool ===");
  Serial.println("Date: July 13, 2025");
  Serial.println("==========================================");
  
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
        showInternetConnectedPattern();
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
  // Only show heartbeat if we don't have a successful connection pattern to display
  if (!internetConnected && !wifiConnected) {
    // Heartbeat blink to show board is alive when no connection
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    
    if (millis() - lastBlink >= 2000) {
      lastBlink = millis();
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
  // If we have internet or WiFi connection, keep showing the success pattern
}