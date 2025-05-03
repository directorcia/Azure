#include <Wire.h>
#include "SparkFun_Alphanumeric_Display.h"
#include <VL53L1X.h>
#include <Adafruit_NeoPixel.h>

#define DISPLAY_ADDRESS 0x70 // Default I2C address for the Qwiic Alphanumeric Display
#define NEOPIXEL_PIN 9       // Pin connected to the NeoPixel strip
#define NEOPIXEL_COUNT 5     // Number of NeoPixels in the strip

// Create an instance of the display
HT16K33 display;

// Create an instance of the VL53L1X sensor. Default address = 0x29
VL53L1X sensor;

// Create an instance of the NeoPixel strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

const uint16_t POT_PIN = A0;             // Analog pin for potentiometer
const uint16_t mid_point = 1380;         // Optimal distance from wall
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

void updateActivityTime() {
  lastActivityTime = millis(); // Update the last activity time
}
void powerOff() {
  // Turn off all NeoPixels
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
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
  }
  sensor.startContinuous(50);
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(100);

  // Initialize the NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Clear the display
  display.clear();
  display.write("Boot");

  // pinmodes
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);
  delay(100);

  // average pot readings
  uint16_t sum = 0; // Fixed syntax error: unint16_t to uint16_t
  for (int i = 0; i < average_number; i++) { 
    sum += analogRead(POT_PIN);
    delay(2); 
  }
  potvalue = sum / average_number;
  lastPotValue = potvalue; // Initialize last pot value
  
  updateActivityTime(); // Initialize activity time
}
void loop() {
    char buffer[5];             // for displaying distance 
    const uint16_t wallDistance = 1233;
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
    }
    return; // Skip the rest of the loop while powered off
  }
    // average distance readings
    uint16_t sum = 0; 
    for (int i = 0; i < average_number; i++) { 
        sum += sensor.read();
        delay(2); 
    } 
    uint16_t stableDistance = sum / average_number;
    
    // Check for significant distance change (activity)
    if (abs((int)stableDistance - (int)lastDistance) > significantDistanceChange) {
        updateActivityTime();
    }
    lastDistance = stableDistance;
        
    uint16_t midvalue = mid_point + potvalue - adjust;      // Adjusted midpoint value
    uint16_t under_red = 0.8 * midvalue;                    // Under red zone
    uint16_t under_yellow = 0.96 * midvalue;                // Under yellow zone
    uint16_t over_yellow = 1.04 * midvalue;                 // Over yellow zone
    uint16_t over_red = 1.2 * midvalue;                     // Over red zone

    // check for debug switch
    int reading = digitalRead(SWITCH_PIN);
    if (reading == LOW) { // if Debug mode
        updateActivityTime(); // Switch activity counts as user interaction
        
        // average pot readings
        uint16_t sum = 0;
        for (int i = 0; i < average_number; i++) { 
            sum += analogRead(POT_PIN);
            delay(2); 
        }
        potvalue = sum / average_number;
        
        // Check for significant pot change (activity)
        if (abs(potvalue - lastPotValue) > significantPotChange) {
            updateActivityTime();
        }
        lastPotValue = potvalue;

        // Turn on all NeoPixels
        for (int i = 0; i < NEOPIXEL_COUNT; i++) {
            strip.setPixelColor(i, COLOR_WHITE);
        }
        strip.show();
        delay(100);

        // Display the pot value           
        snprintf(buffer, sizeof(buffer), "%4d", (potvalue - adjust));
        display.write(buffer);
    } else { // if Normal mode

        // Read pot value periodically to detect adjustments
        uint16_t potSum = 0;
        for (int i = 0; i < average_number; i++) { 
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
        if (stableDistance < under_red) {
            ledState = 1; // Too close to the wall
        } else if (stableDistance < under_yellow) {
            ledState = 2; // Close to the wall
        } else if (stableDistance < over_yellow) {
            ledState = 3; // Optimal distance zone
        } else if (stableDistance < over_red) {
            ledState = 4; // Far from the wall
        } else {
            ledState = 5; // Too far from the wall
        }

        // Check if the LED state has changed
        if (ledState != currentLedState) {
            updateActivityTime(); // LED state change indicates movement (activity)
            currentLedState = ledState; // Update the current LED state
        }

        // Check if inactive for too long
        if (millis() - lastActivityTime > inactivityThreshold) {
            powerOff();
            return;
        }
        switch (ledState) {
            case 1:
                strip.setPixelColor(0, COLOR_RED);
                strip.setPixelColor(1, COLOR_OFF);
                strip.setPixelColor(2, COLOR_OFF);
                strip.setPixelColor(3, COLOR_OFF);
                strip.setPixelColor(4, COLOR_OFF);
                break;
            case 2:
                strip.setPixelColor(0, COLOR_OFF);
                strip.setPixelColor(1, COLOR_YELLOW);
                strip.setPixelColor(2, COLOR_OFF);
                strip.setPixelColor(3, COLOR_OFF);
                strip.setPixelColor(4, COLOR_OFF);
                break;
            case 3:
                strip.setPixelColor(0, COLOR_OFF);
                strip.setPixelColor(1, COLOR_OFF);
                strip.setPixelColor(2, COLOR_WHITE);
                strip.setPixelColor(3, COLOR_OFF);
                strip.setPixelColor(4, COLOR_OFF);
                break;
            case 4:
                strip.setPixelColor(0, COLOR_OFF);
                strip.setPixelColor(1, COLOR_OFF);
                strip.setPixelColor(2, COLOR_OFF);
                strip.setPixelColor(3, COLOR_YELLOW);
                strip.setPixelColor(4, COLOR_OFF);
                break;
            case 5:
                strip.setPixelColor(0, COLOR_OFF);
                strip.setPixelColor(1, COLOR_OFF);
                strip.setPixelColor(2, COLOR_OFF);
                strip.setPixelColor(3, COLOR_OFF);
                strip.setPixelColor(4, COLOR_RED);
                break;
        }
        strip.show();
    }
}