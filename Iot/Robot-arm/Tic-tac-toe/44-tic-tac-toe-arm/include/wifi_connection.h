#pragma once

#include <Arduino.h>
#include <WiFiS3.h>
#include "wifi_secrets.h"

namespace wifi_connection {
inline bool connect(unsigned long timeoutMs = 30000) {
  WiFi.begin(wifi_secrets::ssid, wifi_secrets::password);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeoutMs) {
    delay(250);
  }

  return WiFi.status() == WL_CONNECTED;
}

inline bool internetAvailable(unsigned long timeoutMs = 5000) {
  WiFiClient client;
  if (!client.connect("example.com", 80)) {
    return false;
  }

  client.print("GET / HTTP/1.1\r\nHost: example.com\r\nConnection: close\r\n\r\n");

  unsigned long startTime = millis();
  while (!client.available() && client.connected() && millis() - startTime < timeoutMs) {
    delay(10);
  }

  bool receivedResponse = client.available() > 0;
  client.stop();
  return receivedResponse;
}
}