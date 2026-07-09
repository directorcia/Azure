/**
 * llm-flash.ino
 *
 * ESP32 firmware that periodically queries a local Ollama LLM endpoint and
 * uses the model's JSON reply to control how many times the onboard LED blinks.
 *
 * Workflow:
 *   1. Connect to Wi-Fi.
 *   2. Every REQUEST_INTERVAL_MS milliseconds, pick a random integer 1–4 and
 *      send it to the Ollama /api/generate endpoint as part of a prompt that
 *      instructs the model to echo the value back in JSON form: {"flash": N}.
 *   3. Parse the model's JSON reply and blink the LED that many times.
 *   4. If the reply is missing, malformed, or the value doesn't match what was
 *      sent, blink 5 rapid times to signal an error.
 *
 * LED blink legend:
 *   3 quick blinks  – Wi-Fi connected successfully
 *   5 quick blinks  – Wi-Fi connection failed / HTTP error / JSON parse error
 *   1 long blink    – HTTP request succeeded (Ollama responded 200 OK)
 *   N long blinks   – LLM echoed back the correct flash count N
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_system.h>

// GPIO pin connected to the LED being driven by this sketch.
const int LED_PIN = 4;

// ---------------------------------------------------------------------------
// Network & model configuration — update these for your environment.
// ---------------------------------------------------------------------------

// Wi-Fi credentials.
#define WIFI_SSID "<YOUR_WIFI_SSID>"
#define WIFI_PASSWORD "<YOUR_WIFI_PASSWORD>"

// Full URL of the Ollama REST endpoint on your local network.
const char *OLLAMA_URL = "http://192.168.1.230:11434/api/generate";

// Ollama model tag to use for inference.
const char *OLLAMA_MODEL = "llama3.1:8b";

// Maximum time (ms) to wait for the HTTP response from Ollama.
// LLMs can be slow — 65 s gives the model enough time to finish.
const uint16_t HTTP_TIMEOUT_MS = 65000;

// How often (ms) to send a new request to Ollama.
const unsigned long REQUEST_INTERVAL_MS = 15000;

// Tracks the timestamp (millis()) of the last request so the loop knows when
// REQUEST_INTERVAL_MS has elapsed.
unsigned long lastRequestTime = 0;

// ---------------------------------------------------------------------------
// blink()
//
// Blinks the LED a specified number of times with configurable on/off timing.
//
// Parameters:
//   times  – Number of blink cycles to perform.
//   onMs   – Duration (ms) the LED stays on per cycle.  Default: 120 ms.
//   offMs  – Duration (ms) the LED stays off per cycle. Default: 120 ms.
// ---------------------------------------------------------------------------
void blink(int times, int onMs = 120, int offMs = 120) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    delay(offMs);
  }
}

// ---------------------------------------------------------------------------
// connectWiFi()
//
// Sets the ESP32 to station mode and attempts to join the configured Wi-Fi
// network.  Polls up to 60 times (30 s total) for a connection.
//
// On success: prints the assigned IP address and blinks the LED 3 times.
// On failure: prints an error message and blinks the LED 5 times.
// ---------------------------------------------------------------------------
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");

  int attempts = 0;
  // Poll every 500 ms for up to 30 seconds.
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Wi-Fi connected. IP: ");
    Serial.println(WiFi.localIP());
    blink(3, 80, 80); // 3 quick blinks = connected
  } else {
    Serial.println("Wi-Fi connection failed");
    blink(5, 80, 80); // 5 quick blinks = error
  }
}

// ---------------------------------------------------------------------------
// sendOllamaTest()
//
// Core function that:
//   1. Ensures Wi-Fi is connected (reconnects if needed).
//   2. Generates a random integer 1–4 as the "requested flash count".
//   3. Builds a JSON request body instructing the Ollama model to echo the
//      value back as {"flash": N} — temperature 0 for deterministic output.
//   4. POSTs the request, with one automatic retry on read-timeout.
//   5. Parses the outer Ollama envelope, then the inner model reply JSON.
//   6. Validates that the returned flash count matches what was requested.
//   7. Blinks the LED the correct number of times if everything matches.
//
// Returns:
//   true  – Ollama replied with the correct flash count and the LED blinked.
//   false – Any network, HTTP, JSON, or validation error occurred.
// ---------------------------------------------------------------------------
bool sendOllamaTest() {
  // Ensure Wi-Fi is up before attempting any HTTP work.
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected, reconnecting...");
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) {
      return false; // Still not connected — abort this cycle.
    }
  }

  // Open the HTTP connection to the Ollama endpoint.
  HTTPClient http;
  http.begin(OLLAMA_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(HTTP_TIMEOUT_MS);

  // Pick a random flash count the model must echo back (1, 2, 3, or 4).
  int requestedFlash = random(1, 5);
  Serial.print("Requested flash count: ");
  Serial.println(requestedFlash);

  // Build the Ollama request JSON document.
  JsonDocument req;
  req["model"] = OLLAMA_MODEL;
  // Prompt explicitly instructs the model to return only a JSON object with
  // the provided flash value — no markdown fences, no extra text.
  req["prompt"] = String("Return JSON only using exactly this schema: {\"flash\": ") + requestedFlash + "}. Echo the provided flash value exactly. No markdown, no explanation.";
  req["stream"] = false;      // Receive the full response in one HTTP reply.
  req["format"] = "json";     // Ask Ollama to validate/constrain output as JSON.
  req["keep_alive"] = "30m";  // Keep the model loaded in memory for 30 minutes.

  // Inference options: temperature 0 = deterministic, num_predict caps tokens.
  JsonObject options = req["options"].to<JsonObject>();
  options["temperature"] = 0;   // No randomness — the model must echo exactly.
  options["num_predict"] = 12;  // {"flash":N} is at most ~12 tokens; cap here.

  // Serialize the JSON document to a string for the HTTP POST body.
  String body;
  serializeJson(req, body);

  // Attempt the POST request with one retry on read-timeout.
  int httpCode = -1;
  String postError;
  for (int attempt = 1; attempt <= 2; attempt++) {
    Serial.print("Sending request to Ollama (attempt ");
    Serial.print(attempt);
    Serial.println(")...");

    httpCode = http.POST(body);

    if (httpCode > 0) {
      break; // Received a valid HTTP status code — proceed to response handling.
    }

    // httpCode <= 0 means a transport-level error occurred.
    postError = http.errorToString(httpCode);
    Serial.print("HTTP request failed: ");
    Serial.println(postError);

    // On the first attempt, retry once if the error was a read timeout
    // (the model may have taken too long but is still healthy).
    if (attempt < 2 && postError.indexOf("read Timeout") >= 0) {
      Serial.println("Read timeout detected, retrying once...");
      http.end();
      delay(1000); // Brief pause before re-opening the connection.
      http.begin(OLLAMA_URL);
      http.addHeader("Content-Type", "application/json");
      http.setTimeout(HTTP_TIMEOUT_MS);
      continue;
    }

    // Non-retryable error or second failure — give up this cycle.
    http.end();
    blink(5, 60, 60); // 5 rapid blinks = error
    return false;
  }

  Serial.print("HTTP status: ");
  Serial.println(httpCode);

  // Anything other than 200 OK is treated as an error.
  if (httpCode != HTTP_CODE_OK) {
    String errorBody = http.getString();
    Serial.print("Unexpected status body: ");
    Serial.println(errorBody);
    http.end();
    blink(5, 60, 60); // 5 rapid blinks = error
    return false;
  }

  // 1 blink = message sent and endpoint responded successfully.
  blink(1, 180, 120);

  // Read the full response body from Ollama.
  String responseBody = http.getString();
  http.end();

  // Parse the outer Ollama response envelope.
  // Expected structure: { "model": "...", "response": "<inner JSON string>", ... }
  JsonDocument res;
  DeserializationError err = deserializeJson(res, responseBody);
  if (err) {
    Serial.print("JSON parse failed: ");
    Serial.println(err.c_str());
    Serial.print("Raw body: ");
    Serial.println(responseBody);
    blink(5, 60, 60); // 5 rapid blinks = error
    return false;
  }

  // Extract the model's text reply from the "response" field of the envelope.
  const char *reply = res["response"] | "";
  if (strlen(reply) == 0) {
    Serial.println("No response text from Ollama");
    blink(5, 60, 60); // 5 rapid blinks = error
    return false;
  }

  Serial.print("Ollama reply: ");
  Serial.println(reply);

  // The model's reply should itself be a JSON string — parse it a second time.
  // Expected: {"flash": N}
  JsonDocument replyDoc;
  DeserializationError replyErr = deserializeJson(replyDoc, reply);
  if (replyErr) {
    Serial.print("Reply JSON parse failed: ");
    Serial.println(replyErr.c_str());
    blink(5, 60, 60); // 5 rapid blinks = error
    return false;
  }

  // Extract the flash count from the parsed reply. Default to 0 if missing.
  int flashCount = replyDoc["flash"] | 0;

  // Validate the value is within the expected 1–4 range.
  if (flashCount < 1 || flashCount > 4) {
    Serial.println("Unexpected reply. Expected JSON with flash set to 1 through 4.");
    blink(5, 60, 60); // 5 rapid blinks = error
    return false;
  }

  // Verify the model echoed back exactly what was requested.
  if (flashCount != requestedFlash) {
    Serial.print("Reply mismatch. Expected flash count: ");
    Serial.println(requestedFlash);
    blink(5, 60, 60); // 5 rapid blinks = error
    return false;
  }

  // Success — blink the LED the number of times the model confirmed.
  blink(flashCount, 180, 140);
  return true;
}

// ---------------------------------------------------------------------------
// setup()
//
// Arduino entry point — runs once at power-on or reset.
//   1. Configures the LED pin and ensures it starts off.
//   2. Initialises the Serial console at 115200 baud.
//   3. Seeds the PRNG using the ESP32's hardware random number generator so
//      each power cycle produces a different sequence of flash counts.
//   4. Connects to Wi-Fi and fires an initial Ollama request.
// ---------------------------------------------------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure LED is off at startup.

  Serial.begin(115200);
  delay(500); // Allow the serial monitor time to attach before first output.

  // Use the ESP32's hardware RNG for a truly random seed (not time-based).
  randomSeed(esp_random());

  connectWiFi();

  // Send the first request immediately rather than waiting for the interval.
  sendOllamaTest();
  lastRequestTime = millis();
}

// ---------------------------------------------------------------------------
// loop()
//
// Arduino main loop — runs repeatedly after setup() returns.
// Waits for REQUEST_INTERVAL_MS to elapse, then sends another Ollama request.
// Using millis() avoids blocking the loop with delay().
// ---------------------------------------------------------------------------
void loop() {
  if (millis() - lastRequestTime >= REQUEST_INTERVAL_MS) {
    sendOllamaTest();
    lastRequestTime = millis();
  }
}
