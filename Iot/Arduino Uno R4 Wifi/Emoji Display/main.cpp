
#include "Arduino_LED_Matrix.h"   // Include the LED_Matrix library
ArduinoLEDMatrix matrix;          // Create an instance of the ArduinoLEDMatrix class
void setup() {
  Serial.begin(115200);           // Initialize serial communication at a baud rate of 115200
  matrix.begin();                 // Initialize the LED matrix
}
void loop() {
  // Load and display the basic emoji frame on the LED matrix
  matrix.loadFrame(LEDMATRIX_HEART_BIG);
}

// available emoji frames
// LEDMATRIX_BLUETOOTH
// LEDMATRIX_BOOTLOADER_ON
// LEDMATRIX_CHIP
// LEDMATRIX_CLOUD_WIFI
// LEDMATRIX_DANGER
// LEDMATRIX_EMOJI_BASIC
// LEDMATRIX_EMOJI_HAPPY
// LEDMATRIX_EMOJI_SAD
// LEDMATRIX_HEART_BIG
// LEDMATRIX_HEART_SMALL
// LEDMATRIX_LIKE
// LEDMATRIX_MUSIC_NOTE
// LEDMATRIX_RESISTOR
// LEDMATRIX_UNO