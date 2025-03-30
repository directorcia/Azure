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
int SWITCH_PIN = 8;                 // Debug Switch pin
int LED_UNDER_RED = 3;              // LED pin for under red
int LED_UNDER_YELLOW = 4;           // LED pin for under yellow
int LED_WHITE = 5;                  // LED pin for white
int LED_OVER_YELLOW = 6;            // LED pin for over yellow
int LED_OVER_RED = 7;               // LED pin for over red

// Add variables to track LED state and inactivity
int currentLedState = 0; // Tracks the current LED state
unsigned long ledStateStartTime = 0; // Tracks when the current LED state started
const unsigned long inactivityThreshold = 180000; // 3 minutes in milliseconds
bool isPoweredOff = false; // Tracks if the system is powered off

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
}

void powerOn() {
  isPoweredOff = false;
  ledStateStartTime = millis(); // Reset the LED state timer
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
  unint16_t sum = 0;
  for (int i = 0; i < average_number; i++) { 
    sum += analogRead(POT_PIN);
    delay(2); 
  }
  potvalue = sum / average_number;
}

void loop() {
    char buffer[5];             // for displaying distance 
    const uint16_t wallDistance = 1233;

  if (isPoweredOff) {    // Check for activity to power back on
    if (digitalRead(SWITCH_PIN) == LOW || sensor.read() != 0) {
      powerOn();
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
        
    uint16_t midvalue = mid_point + potvalue - adjust;      // Adjusted midpoint value
    uint16_t under_red = 0.8 * midvalue;                    // Under red zone
    uint16_t under_yellow = 0.96 * midvalue;                // Under yellow zone
    uint16_t over_yellow = 1.04 * midvalue;                 // Over yellow zone
    uint16_t over_red = 1.2 * midvalue;                     // Over red zone

    // check for debug switch
    int reading = digitalRead(SWITCH_PIN);
    if (reading == LOW) { // if Debug mode
        int adjustvalue;
        
        // average pot readings
        uint16_t sum = 0;
        for (int i = 0; i < average_number; i++) { 
            sum += analogRead(POT_PIN);
            delay(2); 
        }
        potvalue = sum / average_number;

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
            currentLedState = ledState; // Update the current LED state
            ledStateStartTime = millis(); // Reset the timer for the new state
        }

          // Check if the same LED has been on for too long
        if (millis() - ledStateStartTime > inactivityThreshold) {
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