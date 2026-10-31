#pragma once

// One-shot Gemini generateContent call for the robot's move choice.
//
// robotMove() in main.cpp has already reduced the decision to a handful of
// equally optimal squares. This helper only transports that prompt and
// returns the model's text. It does not interpret the move: the caller parses
// {"square": N} and rejects anything outside the minimax set.
//
// Credentials live in gemini_config.h (gitignored). Copy
// gemini_config.example.h and set:
//   GEMINI_API_KEY   sent as the x-goog-api-key header, never in the URL
//   GEMINI_URL       full https URL, including the ":generateContent" path
//
// The call blocks (TLS handshake plus up to 20 s for the body). main.cpp
// therefore runs it from loop(), not from the HTTP handler, so the browser
// still gets an immediate reply when the move is queued.

#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <WiFiS3.h>
#include "gemini_config.h"

namespace gemini_connection {
// Posts `prompt` and, on success, appends the non-thought text parts into
// `reply`. On failure `reply` is empty, `error` explains why, and the
// function returns false. `jsonReply` asks Gemini for application/json so
// the caller can deserialize the body directly. `verbose` prints handshake
// progress to Serial only (it does not enter the web log ring).
inline bool generate(const char *prompt, String &reply, String &error, bool jsonReply = false, bool verbose = false) {
  reply = "";
  error = "";
  if (WiFi.status() != WL_CONNECTED) {
    error = "Wi-Fi is not connected.";
    return false;
  }
  // The example file ships with this exact placeholder. Refuse to call out
  // until gemini_config.h has been edited, so a forgotten copy fails closed.
  if (GEMINI_API_KEY == nullptr || GEMINI_API_KEY[0] == '\0' || strcmp(GEMINI_API_KEY, "YOUR_GEMINI_API_KEY") == 0) {
    error = "Set the Gemini API key in gemini_config.h.";
    return false;
  }

  // GEMINI_URL is "https://<host>/<path>". The host is everything between
  // the scheme and the next slash; the path keeps its leading slash and is
  // what HttpClient posts to. The API key is not appended as a query.
  String url = GEMINI_URL;
  if (!url.startsWith("https://")) {
    error = "GEMINI_URL must use HTTPS.";
    return false;
  }
  int pathStart = url.indexOf('/', 8);
  if (pathStart < 0) {
    error = "GEMINI_URL must include an API path.";
    return false;
  }
  String host = url.substring(8, pathStart);
  String path = url.substring(pathStart);

  // generateContent request. One user turn, a short cap (the reply is a
  // single square), and optionally a JSON mime type so the model does not
  // wrap the object in markdown.
  JsonDocument request;
  request["contents"][0]["parts"][0]["text"] = prompt;
  request["generationConfig"]["maxOutputTokens"] = 256;
  if (jsonReply) {
    request["generationConfig"]["responseMimeType"] = "application/json";
  }
  String payload;
  serializeJson(request, payload);

  WiFiSSLClient tls;
  tls.setConnectionTimeout(15000);
  HttpClient http(tls, host.c_str(), 443);
  http.setHttpResponseTimeout(20000);
  if (verbose) Serial.println("[Gemini] Opening HTTPS connection...");
  http.beginRequest();
  int result = http.post(path.c_str());
  if (result != 0) {
    error = "Gemini TLS/HTTP connection failed: ";
    error += String(result);
    http.stop();
    return false;
  }
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", payload.length());
  http.sendHeader("x-goog-api-key", GEMINI_API_KEY);
  http.sendHeader("Connection", "close");
  http.beginBody();
  http.print(payload);
  http.endRequest();

  if (verbose) Serial.println("[Gemini] Request sent. Waiting for HTTP response (timeout 20s)...");
  int status = http.responseStatusCode();
  if (verbose) {
    Serial.print("[Gemini] HTTP status: ");
    Serial.println(status);
  }
  if (status != 200) {
    error = "Gemini HTTP status ";
    error += String(status);
    error += ". Check key, model access, quota and R4 connectivity firmware.";
    http.stop();
    return false;
  }

  String body = http.responseBody();
  http.stop();
  if (verbose) {
    Serial.print("[Gemini] Response body bytes: ");
    Serial.println(body.length());
  }
  JsonDocument response;
  DeserializationError parseError = deserializeJson(response, body);
  if (parseError) {
    error = "Could not parse Gemini response.";
    return false;
  }
  // Thinking models put reasoning in parts flagged "thought". Those are not
  // the move. Concatenate only the visible text parts, in order.
  JsonArrayConst parts = response["candidates"][0]["content"]["parts"].as<JsonArrayConst>();
  for (JsonObjectConst part : parts) {
    if (!(part["thought"] | false)) {
      reply += part["text"] | "";
    }
  }
  if (reply.length() == 0) {
    error = "Gemini returned no text; the response may be blocked or token-limited.";
    return false;
  }
  return true;
}
}
