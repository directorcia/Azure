/*
  WiFiConnect.cpp - Library for managing WiFi connection for Arduino Uno R4 WiFi
*/

#include "WiFiConnect.h"

WiFiConnect::WiFiConnect(const char* ssid, const char* password) {
  _ssid = ssid;
  _password = password;
}

bool WiFiConnect::connect(int timeout) {
  // Begin WiFi connection
  WiFi.begin(_ssid, _password);
  
  // Wait for connection with timeout
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeout) {
    delay(500);
  }
  
  return WiFi.status() == WL_CONNECTED;
}

bool WiFiConnect::isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

IPAddress WiFiConnect::getIP() {
  return WiFi.localIP();
}

String WiFiConnect::getMacAddress() {
  byte mac[6];
  WiFi.macAddress(mac);
  char macStr[18] = {0};
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

int WiFiConnect::getSignalStrength() {
  return WiFi.RSSI();
}

void WiFiConnect::disconnect() {
  WiFi.disconnect();
}