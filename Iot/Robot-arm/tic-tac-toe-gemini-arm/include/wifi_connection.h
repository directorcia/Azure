#pragma once

// Wi-Fi join helper for the UNO R4 WiFi (WiFiS3).
//
// The radio is not reset when the sketch restarts, so main.cpp calls
// connect() both from a fresh startNetwork(true) and when a later Gemini
// request finds the link down. Credentials are wifi_secrets::ssid and
// wifi_secrets::password, supplied by wifi_secrets.h (gitignored). Copy
// wifi_secrets.example.h and fill it in.
//
// connect() blocks, polling every 250 ms, for up to timeoutMs (30 s by
// default). serviceNetwork() only calls it while the arm is idle, because
// a blocked loop() cannot advance the motion state machine. The 200 Hz
// servo timer keeps running across the wait; no new plan is started.

#include <Arduino.h>
#include <WiFiS3.h>
#include "wifi_secrets.h"

namespace wifi_connection {
// Begins association and returns true only if status is WL_CONNECTED before
// the deadline. A still-associated radio usually returns immediately.
inline bool connect(unsigned long timeoutMs = 30000) {
  WiFi.begin(wifi_secrets::ssid, wifi_secrets::password);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeoutMs) {
    delay(250);
  }

  return WiFi.status() == WL_CONNECTED;
}

// Optional reachability probe: open example.com:80 and wait until at least
// one response byte arrives. The game sketch does not call this; Wi-Fi
// association alone is what gates the Gemini request. Returns false on
// timeout or a refused connection. Blocks for up to timeoutMs.
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
