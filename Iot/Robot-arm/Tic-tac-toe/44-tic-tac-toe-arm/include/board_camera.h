#pragma once

#include <Arduino.h>
#include <WiFiS3.h>
#include <string.h>
#include "camera_config.h"

// One HTTP GET of the board camera.
//
// The arm's FspTimer ISR (200 Hz) refreshes the motion plan and the Servo
// library keeps pulsing the last widths. Nothing in this file stops that timer
// or masks interrupts. `pump`, when set, is called between modem round-trips
// so the "timer unavailable" fallback in loop() can still advance a plan.
// Call this only once the arm is already parked: the WiFiS3 modem waits inside
// its own UART poll, and that poll cannot call `pump` itself.

namespace board_camera {

// Parse an HTTP response whose body is the 9-character board.
// Leading blank lines in the body are ignored. Returns false and sets `error`
// when the status is not 200 or the body is not nine piece characters.
inline bool parseBoardResponse(const char *raw, char out[9], String &error) {
  if (strncmp(raw, "HTTP/", 5) != 0) {
    error = "not an HTTP response";
    return false;
  }
  const char *eol = strchr(raw, '\n');
  const char *space = strchr(raw, ' ');
  if (eol == nullptr || space == nullptr || space >= eol || strncmp(space + 1, "200", 3) != 0) {
    error = "HTTP status not 200";
    return false;
  }
  const char *body = strstr(raw, "\r\n\r\n");
  if (body == nullptr) {
    error = "HTTP headers incomplete";
    return false;
  }
  body += 4;
  while (*body == '\r' || *body == '\n') body++;
  // strlen stops at the terminator, so a short body never reads past `raw`.
  if (strlen(body) < 9) {
    error = "body is not 9 cells of X, O, or .";
    return false;
  }
  for (int i = 0; i < 9; i++) {
    char c = body[i];
    if (c != 'X' && c != 'O' && c != '.') {
      error = "body is not 9 cells of X, O, or .";
      return false;
    }
    out[i] = c;
  }
  return true;
}

// GET /board once. `timeoutMs` is both the TCP connect timeout and the cap on
// the modem command timeout for this attempt (about 3 s). The socket is always
// closed before return, including on failure. The process-wide modem timeout
// is restored to the WiFiS3 default so later web-server requests are unchanged.
inline bool fetchBoard(char out[9], String &error, unsigned long timeoutMs, void (*pump)()) {
  error = "";
  if (timeoutMs < 250) timeoutMs = 250;
  if (WiFi.status() != WL_CONNECTED) {
    error = "Wi-Fi is not connected";
    return false;
  }

  // Stop the socket and put the modem timeout back on every exit path.
  struct Session {
    WiFiClient client;
    ~Session() {
      client.stop();
      modem.timeout(MODEM_TIMEOUT);
    }
  } session;

  modem.timeout(timeoutMs);
  session.client.setConnectionTimeout(timeoutMs > 30000 ? 30000 : int(timeoutMs));

  unsigned long start = millis();
  if (!session.client.connect(CAMERA_HOST, CAMERA_PORT)) {
    error = (millis() - start >= timeoutMs) ? "timed out connecting" : "connection failed";
    return false;
  }

  session.client.print("GET /board HTTP/1.1\r\nHost: ");
  session.client.print(CAMERA_HOST);
  session.client.print("\r\nConnection: close\r\n\r\n");

  char raw[480];
  size_t length = 0;
  raw[0] = '\0';
  while (millis() - start < timeoutMs && length + 1 < sizeof(raw)) {
    // delay() does not stop the servo timer ISR. pump() services the fallback.
    if (pump) pump();
    int waiting = session.client.available();
    if (waiting > 0) {
      int byte = session.client.read();
      if (byte < 0) break;
      raw[length++] = char(byte);
      raw[length] = '\0';
      const char *split = strstr(raw, "\r\n\r\n");
      if (split != nullptr && strlen(split + 4) >= 9) break;
      continue;
    }
    if (!session.client.connected()) break;
    delay(10);
  }

  if (length == 0) {
    error = (millis() - start >= timeoutMs) ? "timed out with no data" : "empty response";
    return false;
  }
  if (!parseBoardResponse(raw, out, error)) {
    if (error.length() == 0) error = "unreadable board response";
    return false;
  }
  return true;
}

}  // namespace board_camera
