#include <Arduino.h>
#include "gemini_connection.h"
#include "wifi_connection.h"

void testGemini() {
  if (WiFi.status() != WL_CONNECTED && !wifi_connection::connect()) {
    Serial.println("Wi-Fi connection failed. Check the local wireless settings.");
    return;
  }
  Serial.print("Wi-Fi connected. IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("Testing Gemini HTTPS API...");
  String reply;
  String error;
  if (gemini_connection::generate("Reply with only: Gemini connection successful", reply, error)) {
    Serial.print("Gemini: ");
    Serial.println(reply);
  } else {
    Serial.println(error);
  }
  Serial.println("Send GEMINI and Enter to retry. No motors are controlled by this program.");
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);
  testGemini();
}

void loop() {
  static String command;
  while (Serial.available()) {
    char character = Serial.read();
    if (character == '\r' || character == '\n') {
      command.trim();
      if (command.equalsIgnoreCase("GEMINI")) testGemini();
      command = "";
    } else if (command.length() < 32) {
      command += character;
    }
  }
}