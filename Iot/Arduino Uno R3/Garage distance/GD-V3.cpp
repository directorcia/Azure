#include <Wire.h>
#include "SparkFun_Alphanumeric_Display.h"
#include <VL53L1X.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
/*
 * NeoPixel LED Distance Indicator
 * 
 * This program uses a VL53L1X distance sensor to control a NeoPixel LED strip,
 * providing visual feedback based on distance readings. It includes:
 * 
 * - Distance-based LED indicators (red/yellow/white zones)
 * - Debug mode with cog pattern display and quick LED test
 * - Auto power-off after inactivity
 * - Distance and zone display on alphanumeric display
 * - Potentiometer for adjusting optimal distance
 * 
 * All LEDs are properly tested and handled to ensure consistent operation.
 * The code avoids boot loops by not running long tests during startup.
 */
#define DISPLAY_ADDRESS 0x70 // Default I2C address for the Qwiic Alphanumeric Display
#define NEOPIXEL_PIN 13       // Pin connected to the NeoPixel strip
#define NEOPIXEL_COUNT 40     // Number of NeoPixels in the strip
// Create an instance of the display
HT16K33 display;
// Create an instance of the VL53L1X sensor. Default address = 0x29
VL53L1X sensor;
// Create an instance of the NeoPixel strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
const uint16_t POT_PIN = A0;             // Analog pin for potentiometer
const uint16_t mid_point = 1245;         // Optimal distance from wall
const uint16_t adjust = 334;             // potentiometer midpoint value;
const uint16_t average_number = 10;      // Number for averaging distance readings
int potvalue;
int lastPotValue;                   // Track the last potentiometer value
uint16_t lastDistance = 0;          // Track the last distance reading
int SWITCH_PIN = 8;                 // Debug Switch pin
// Define colors for the NeoPixels
uint32_t COLOR_RED = strip.Color(255, 0, 0);         // Red color
uint32_t COLOR_YELLOW = strip.Color(255, 255, 0);    // Yellow color
uint32_t COLOR_WHITE = strip.Color(255, 255, 255);   // White color
uint32_t COLOR_OFF = strip.Color(0, 0, 0);           // Off (black)
// Define dimmer colors for debug mode
uint32_t DIM_WHITE = strip.Color(50, 50, 50);       // Dimmer white for debug mode
uint32_t DIM_YELLOW = strip.Color(50, 50, 0);       // Dimmer yellow for debug mode
// Add variables to track LED state and inactivity
int currentLedState = 0; // Tracks the current LED state
unsigned long lastActivityTime = 0; // Tracks the last time any activity was detected
const unsigned long inactivityThreshold = 180000; // 3 minutes in milliseconds = 180000
bool isPoweredOff = false; // Tracks if the system is powered off
unsigned int PoweredOffLastReading;
const uint16_t significantDistanceChange = 50; // Distance change that counts as activity (in mm)
const int significantPotChange = 5; // Potentiometer change that counts as activity
// Variables to prevent rapid power cycling
unsigned long powerOffTime = 0; // Tracks when the device was powered off
const unsigned long minPowerOffDuration = 0; // Minimum time to stay off (1 seconds)
const uint16_t wakeUpThreshold = 100; // Minimum sensor reading to wake up (preventing noise)
bool confirmActivation = false; // Flag for confirming activation
// Function to test all LEDs to verify they're working properly
// Returns quickly to avoid boot loops - use isQuickTest=true for a faster test
void testAllLeds(bool isQuickTest = false) {
  uint16_t delayTime = isQuickTest ? 100 : 500; // Shorter delays for quick test mode
  
  // First test - all LEDs RED
  for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
    strip.setPixelColor(i, COLOR_RED);
  }
  strip.show();
  delay(delayTime);
  
  // Second test - all LEDs YELLOW
  for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
    strip.setPixelColor(i, COLOR_YELLOW);
  }
  strip.show();
  delay(delayTime);
  
  // Third test - all LEDs WHITE
  for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
    strip.setPixelColor(i, COLOR_WHITE);
  }
  strip.show();
  delay(delayTime);
  
  // Only do the sequential test in full test mode
  if (!isQuickTest) {
    // Final test - sequential lighting of each LED
    for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
      strip.clear();
      strip.setPixelColor(i, COLOR_WHITE);
      strip.show();
      delay(25); // Even in full mode, we can make this faster
    }
  }
  
  // Turn all LEDs off
  for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
    strip.setPixelColor(i, COLOR_OFF);
  }
  strip.show();
}
void updateActivityTime() {
  lastActivityTime = millis(); // Update the last activity time
}
void powerOff() {
  // Turn off all NeoPixels
  for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
    strip.setPixelColor(i, COLOR_OFF);
  }
  strip.show();
  // Clear the display
  display.clear();
  display.write("    "); // Clear display content
  isPoweredOff = true;
  powerOffTime = millis(); // Record when the device was powered off
  PoweredOffLastReading = sensor.read(); // Store the last reading before powering off
}
void powerOn() {
  isPoweredOff = false;
  confirmActivation = false;
  display.write("On  ");
  delay(250); // Brief notification that device is on
  updateActivityTime(); // Reset the activity timer
}
void setup() {
  // Initialize I2C communication
  Wire.begin();
  // Initialize the display
  if (display.begin(DISPLAY_ADDRESS) == false) {
    Serial.begin(9600);
    Serial.println("Display not found. Check wiring.");
    while (1);
  }
  // Initialize the VL53L1X sensor
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.begin(9600);
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }  sensor.startContinuous(50);
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(33000); // 33ms timing budget (minimum for long mode)
  // Initialize the NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
    // Explicitly turn off all NeoPixels at startup
  for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
    strip.setPixelColor(i, COLOR_OFF);
  }
  strip.show();
  // Clear the display
  display.clear();
  display.write("Boot");
  // pinmodes
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);
  delay(100);
  
  // average pot readings
  uint16_t sum = 0; // Fixed syntax error: unint16_t to uint16_t
  for (uint16_t i = 0; i < average_number; i++) { 
    sum += analogRead(POT_PIN);
    delay(2); 
  }
  potvalue = sum / average_number;
  lastPotValue = potvalue; // Initialize last pot value
  
  updateActivityTime(); // Initialize activity time
}
void loop() {
    char buffer[5];             // for displaying distance 
  if (isPoweredOff) {    // Check for activity to power back on
    // First check if minimum off time has elapsed
    if (millis() - powerOffTime < minPowerOffDuration) {
      delay(100); // Short delay to prevent excessive polling
      return;     // Don't check for power-on events yet
    }
    
    // Get sensor reading
    uint16_t currentReading = sensor.read();
    bool switchPressed = (digitalRead(SWITCH_PIN) == LOW);
    
    // Check for significant activity
    if (switchPressed || (abs((int)currentReading - (int)PoweredOffLastReading) > wakeUpThreshold)) {
        // If switch is pressed, power on immediately
      if (switchPressed) {
        powerOn();
        return;
      }
      
      // For sensor readings, require confirmation to prevent false triggers
      if (!confirmActivation) {
        // First detection, wait for confirmation
        confirmActivation = true;
        display.clear();
        display.write("Wait"); // Optional: show waiting status
        delay(250); // Wait for a second to confirm
        return;
      } else {
        // Confirmation reading
        currentReading = sensor.read();
        if (currentReading > wakeUpThreshold) {
          powerOn();
        } else {
          // False alarm, reset confirmation
          confirmActivation = false;
          display.clear();
          display.write("    "); // Clear the display again
        }
      }
    }    return; // Skip the rest of the loop while powered off
  }
  
  // average distance readings
    uint16_t sum = 0; 
    for (uint16_t i = 0; i < average_number; i++) { 
        sum += sensor.read();
        delay(2); 
    }    uint16_t stableDistance = sum / average_number;
    
    // Check for significant distance change (activity)
    if (abs((int)stableDistance - (int)lastDistance) > significantDistanceChange) {
        updateActivityTime();
    }
    
    lastDistance = stableDistance;    uint16_t midvalue = mid_point + potvalue - adjust;      // Adjusted midpoint value
    
    // Use more precise calculations for thresholds with better precision for floating point
    // Widen the margins slightly to ensure full LED coverage
    uint16_t under_red = (uint16_t)(midvalue * 0.75f);        // Under red zone - expanded
    uint16_t under_yellow = (uint16_t)(midvalue * 0.94f);     // Under yellow zone
    uint16_t over_yellow = (uint16_t)(midvalue * 1.06f);      // Over yellow zone
    uint16_t over_red = (uint16_t)(midvalue * 1.25f);         // Over red zone - expanded    // check for debug switch
    static bool wasInDebugMode = false;  // Track debug mode state changes
    int reading = digitalRead(SWITCH_PIN);
    
    if (reading == LOW) { // if Debug mode
        updateActivityTime(); // Switch activity counts as user interaction
        
        // If we just entered debug mode, do a quick LED test
        if (!wasInDebugMode) {
            display.write("TEST");
            testAllLeds(true);  // Run a quick test
            delay(300);
        }
        wasInDebugMode = true;
        
        // average pot readings
        uint16_t sum = 0;
        for (uint16_t i = 0; i < average_number; i++) { 
            sum += analogRead(POT_PIN);
            delay(2); 
        }
        
        potvalue = sum / average_number;
        
        // Check for significant pot change (activity)
        if (abs(potvalue - lastPotValue) > significantPotChange) {
            updateActivityTime();
        }
        lastPotValue = potvalue;
        
        // Display cog pattern on NeoPixels in debug mode with dimmer lights
        // Clear all pixels first
        for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
          strip.setPixelColor(i, COLOR_OFF);
        }
        
        // Set every 4th LED to create a cog pattern with dimmer colors
        for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
          if (i % 4 == 0) {
            strip.setPixelColor(i, DIM_WHITE);  // Tooth with dimmer white
          } else if (i % 4 == 2) {
            strip.setPixelColor(i, DIM_YELLOW); // Inner part with dimmer yellow
          }
        }
        
        strip.show();
        delay(100);
          // Display the pot value
        snprintf(buffer, sizeof(buffer), "%4d", (potvalue - adjust));
        display.write(buffer);
    } 
    else { // if Normal mode
        wasInDebugMode = false;  // Reset debug mode tracking
        // Read pot value periodically to detect adjustments
        uint16_t potSum = 0;
        for (uint16_t i = 0; i < average_number; i++) { 
            potSum += analogRead(POT_PIN);
            delay(2); 
        }
        int newPotValue = potSum / average_number;
        
        // Check for significant pot change (activity)
        if (abs(newPotValue - lastPotValue) > significantPotChange) {
            updateActivityTime();
            potvalue = newPotValue; // Update potvalue when it changes significantly
        }
        lastPotValue = newPotValue;
        
        // Display the distance
        snprintf(buffer, sizeof(buffer), "%4d", stableDistance);
        display.write(buffer);
        // Determine LED status
        int ledState = 0;
        
        // For additional debugging, display a character indicating which zone we're in
        char zoneIndicator = '-';
        
        if (stableDistance < under_red) {
            ledState = 1; // Too close to the wall
            zoneIndicator = '1';
        } else if (stableDistance < under_yellow) {
            ledState = 2; // Close to the wall
            zoneIndicator = '2';
        } else if (stableDistance < over_yellow) {
            ledState = 3; // Optimal distance zone
            zoneIndicator = '3';
        } else if (stableDistance < over_red) {
            ledState = 4; // Far from the wall
            zoneIndicator = '4';
        } else {
            ledState = 5; // Too far from the wall
            zoneIndicator = '5';
        }        // Display distance with zone indicator
        snprintf(buffer, sizeof(buffer), "%3d%c", stableDistance / 10, zoneIndicator);
        display.write(buffer);
        // Check if the LED state has changed
        if (ledState != currentLedState) {
            updateActivityTime(); // LED state change indicates movement (activity)
            currentLedState = ledState; // Update the current LED state
        }
        
        // Check if inactive for too long
        if (millis() - lastActivityTime > inactivityThreshold) {
            powerOff();
            return;
        }        // First, turn all LEDs off
        for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
            strip.setPixelColor(i, COLOR_OFF);
        }
        
        // Then set the appropriate LEDs based on the state - using a more robust approach
        // Make sure to light ALL LEDs in each zone
        if (ledState == 1) { // Too close - red
            for (uint16_t i = 0; i < 8; i++) {
                strip.setPixelColor(i, COLOR_RED);
            }
        }
        else if (ledState == 2) { // Close - yellow
            for (uint16_t i = 8; i < 16; i++) {
                strip.setPixelColor(i, COLOR_YELLOW);
            }
        }
        else if (ledState == 3) { // Optimal - white
            for (uint16_t i = 16; i < 24; i++) {
                strip.setPixelColor(i, COLOR_WHITE);
            }
        }
        else if (ledState == 4) { // Far - yellow
            for (uint16_t i = 24; i < 32; i++) {
                strip.setPixelColor(i, COLOR_YELLOW);
            }
        }
        else if (ledState == 5) { // Too far - red
            for (uint16_t i = 32; i < NEOPIXEL_COUNT; i++) {
                strip.setPixelColor(i, COLOR_RED);
            }
        }
        else { // Fallback - ensure all LEDs work by lighting them all
            for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
                if (i % 2 == 0) {
                    strip.setPixelColor(i, COLOR_RED);
                } else {
                    strip.setPixelColor(i, COLOR_WHITE);
                }
            }
        }
        
        strip.show();
    }
}



