#include <Wire.h>
#include "SparkFun_Alphanumeric_Display.h"
#include <VL53L1X.h>
#define DISPLAY_ADDRESS 0x70 // Default I2C address for the Qwiic Alphanumeric Display

// Create an instance of the display
HT16K33 display;

// Create an instance of the VL53L1X sensor. Default address = 0x29
VL53L1X sensor;

const uint16_t POT_PIN = A0;             // Analog pin for potentiometer
const uint16_t mid_point = 1380;         // Optimal distance from wall
const uint16_t adjust = 334;             // potentiometer midpoint value;
const uint16_t average_number = 10;      // Number for averaging distance readings
int potvalue;
int lastPotValue;                   // Track the last potentiometer value
uint16_t lastDistance = 0;          // Track the last distance reading
int SWITCH_PIN = 8;                 // Debug Switch pin
int LED_UNDER_RED = 3;              // LED pin for under red
int LED_UNDER_YELLOW = 4;           // LED pin for under yellow
int LED_WHITE = 5;                  // LED pin for white
int LED_OVER_YELLOW = 6;            // LED pin for over yellow
int LED_OVER_RED = 7;               // LED pin for over red

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
  // Turn off all LEDs
  digitalWrite(LED_UNDER_RED, LOW);
  digitalWrite(LED_UNDER_YELLOW, LOW);
  digitalWrite(LED_WHITE, LOW);
  digitalWrite(LED_OVER_YELLOW, LOW);
  digitalWrite(LED_OVER_RED, LOW);
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

  // Clear the display
  display.clear();
  display.write("Boot");

  // pinmodes
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);
  pinMode(LED_UNDER_RED, OUTPUT);
  pinMode(LED_UNDER_YELLOW, OUTPUT);
  pinMode(LED_WHITE, OUTPUT);
  pinMode(LED_OVER_YELLOW, OUTPUT);
  pinMode(LED_OVER_RED, OUTPUT);
  delay(100);
  digitalWrite(LED_UNDER_RED, HIGH);
  delay(100);
  digitalWrite(LED_UNDER_YELLOW, HIGH);
  delay(100);
  digitalWrite(LED_WHITE, HIGH);
  delay(100);
  digitalWrite(LED_OVER_YELLOW, HIGH);
  delay(100);
  digitalWrite(LED_OVER_RED, HIGH);
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

        // Turn on all LEDs
        digitalWrite(LED_UNDER_RED, HIGH);
        digitalWrite(LED_UNDER_YELLOW, HIGH);
        digitalWrite(LED_WHITE, HIGH);
        digitalWrite(LED_OVER_YELLOW, HIGH);
        digitalWrite(LED_OVER_RED, HIGH);
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
                digitalWrite(LED_UNDER_RED, HIGH);
                digitalWrite(LED_UNDER_YELLOW, LOW);
                digitalWrite(LED_WHITE, LOW);
                digitalWrite(LED_OVER_YELLOW, LOW);
                digitalWrite(LED_OVER_RED, LOW);
                break;
            case 2:
                digitalWrite(LED_UNDER_RED, LOW);
                digitalWrite(LED_UNDER_YELLOW, HIGH);
                digitalWrite(LED_WHITE, LOW);
                digitalWrite(LED_OVER_YELLOW, LOW);
                digitalWrite(LED_OVER_RED, LOW);
                break;
            case 3:
                digitalWrite(LED_UNDER_RED, LOW);
                digitalWrite(LED_UNDER_YELLOW, LOW);
                digitalWrite(LED_WHITE, HIGH);
                digitalWrite(LED_OVER_YELLOW, LOW);
                digitalWrite(LED_OVER_RED, LOW);
                break;
            case 4:
                digitalWrite(LED_UNDER_RED, LOW);
                digitalWrite(LED_UNDER_YELLOW, LOW);
                digitalWrite(LED_WHITE, LOW);
                digitalWrite(LED_OVER_YELLOW, HIGH);
                digitalWrite(LED_OVER_RED, LOW);
                break;
            case 5:
                digitalWrite(LED_UNDER_RED, LOW);
                digitalWrite(LED_UNDER_YELLOW, LOW);
                digitalWrite(LED_WHITE, LOW);
                digitalWrite(LED_OVER_YELLOW, LOW);
                digitalWrite(LED_OVER_RED, HIGH);
                break;
        }
    }
}