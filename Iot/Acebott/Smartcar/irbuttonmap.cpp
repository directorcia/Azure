#include <Arduino.h>
#include <IRremote.h>

// IR receiver pin
#define IR_RECEIVE_PIN 4

// Function to identify which key was pressed
String getKeyName(uint32_t command) {
  switch (command) {
    case 0x44: return "LEFT ARROW";
    case 0x46: return "UP ARROW";
    case 0x15: return "DOWN ARROW";
    case 0x43: return "RIGHT ARROW";
    case 0x40: return "OK BUTTON";
    case 0x16: return "1";
    case 0x19: return "2";
    case 0xD:  return "3";
    case 0xC:  return "4";
    case 0x18: return "5";
    case 0x5E: return "6";
    case 0x8:  return "7";
    case 0x1C: return "8";
    case 0x5A: return "9";
    case 0x52: return "0";
    case 0x42: return "* (STAR)";
    case 0x4A: return "# (HASH)";
    default:   return "UNKNOWN KEY";
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("IR Remote Code Scanner with Key Identification");
  Serial.println("Point your remote at the IR receiver and press buttons");
  Serial.println("The codes and key names will be displayed below:");
  Serial.println("=================================================");
  
  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
  // Check for IR commands
  if (IrReceiver.decode()) {
    uint32_t command = IrReceiver.decodedIRData.command;
    
    // Display the received IR data with key identification
    Serial.print("KEY: ");
    Serial.print(getKeyName(command));
    Serial.print(" | Code: 0x");
    Serial.print(command, HEX);
    Serial.print(" | Raw: 0x");
    Serial.print(IrReceiver.decodedIRData.decodedRawData, HEX);
    Serial.print(" | Protocol: ");
    Serial.println(getProtocolString(IrReceiver.decodedIRData.protocol));
    
    IrReceiver.resume(); // Ready for next signal
  }
  
  delay(100); // Small delay for stability
}

