#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
  ESP32 Fortune Teller

  What this sketch does:
  - Connects an ESP32-S2 to Wi-Fi.
  - Calls a local Ollama model over HTTP to generate a short "fortune".
  - Displays status and fortune text on a 128x64 SSD1306 OLED.
  - Uses an LED for simple visual feedback (blink patterns).

  Hardware assumptions:
  - Board: ESP32-S2 (or compatible ESP32 board with matching pin map).
  - OLED: I2C SSD1306 at address 0x3C.
  - LED: Connected to LED_PIN (or built-in LED on that pin).

  Runtime behavior:
  - setup(): Initializes serial, display, Wi-Fi, then fetches and shows one fortune.
  - loop(): Fetches and shows a new fortune every REQUEST_INTERVAL milliseconds.
*/
// =================================================
// WIFI SETTINGS
// =================================================
#define WIFI_SSID "<YOUR_WIFI_SSID>"
#define WIFI_PASSWORD "<YOUR_WIFI_PASSWORD>"
// =================================================
// OLLAMA SETTINGS
// =================================================
// Local Ollama generate endpoint. Ensure this host is reachable from the ESP32.
const char *OLLAMA_URL = "http://192.168.1.230:11434/api/generate";
// Model name/tag installed in local Ollama.
const char *OLLAMA_MODEL = "llama3.1:8b";
// =================================================
// HARDWARE
// =================================================
// LED used for quick status feedback.
#define LED_PIN 4
// I2C pins used by the OLED on this board.
#define OLED_SDA 7
#define OLED_SCL 6
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// =================================================
// Timestamp of the last successful/attempted fortune request.
unsigned long lastRequest = 0;
// Periodic request interval (20 seconds).
const unsigned long REQUEST_INTERVAL = 20000;
// Per-request HTTP timeout in milliseconds.
const uint16_t HTTP_TIMEOUT_MS = 60000;
// Retry count for HTTP/JSON failures.
const int MAX_HTTP_ATTEMPTS = 2;
// =================================================
// Blink helper for short visual cues.
// Typical usage:
// - 3 blinks after Wi-Fi connects.
// - 2 blinks after a fortune is displayed.
void blink(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
}
// =================================================
// Clears the OLED and prints arbitrary text starting at (0, 0).
// This is used for status screens (boot, Wi-Fi, "consulting" message).
void showText(String text) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(text);
  display.display();
}
// =================================================
// Renders the fortune card layout on the OLED.
// The incoming fortune is expected to be short (prompt requests <= 12 words)
// so it can fit cleanly on the 128x64 display.
void showFortune(String fortune) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("YOUR FORTUNE");
  display.println("----------------");
  display.println();
  display.println(fortune);
  display.display();
}
// =================================================
// Connects to Wi-Fi in station mode and blocks until connected.
// Displays progress over serial and then shows the assigned local IP on OLED.
void connectWiFi() {
  showText("Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected");
  showText("Connected\n\nIP:\n" + WiFi.localIP().toString());
  blink(3);
  delay(2000);
}
// =================================================
// Sends a JSON request to Ollama and returns one short fortune string.
//
// Request strategy:
// - Uses non-streaming response for simpler parsing.
// - Sets "keep_alive" to reduce model cold starts on repeated calls.
// - Uses a higher temperature for varied, mystical phrasing.
//
// Reliability strategy:
// - Retries up to MAX_HTTP_ATTEMPTS for HTTP and JSON parse failures.
// - Returns user-friendly fallback strings when all attempts fail.
String getFortune() {
  // Build request payload sent to /api/generate.
  JsonDocument req;
  req["model"] = OLLAMA_MODEL;
  req["prompt"] = "You are a mystical fortune teller. "
                  "Give one short fortune. "
                  "Maximum 12 words. "
                  "No introduction. "
                  "No quotes.";
  req["stream"] = false;
  req["keep_alive"] = "30m";
  JsonObject options = req["options"].to<JsonObject>();
  options["temperature"] = 1.2;
  options["num_predict"] = 32;

  // Serialize payload once and reuse across retry attempts.
  String body;
  serializeJson(req, body);

  // Retry loop handles transient network/model issues.
  for (int attempt = 1; attempt <= MAX_HTTP_ATTEMPTS; attempt++) {
    HTTPClient http;
    http.begin(OLLAMA_URL);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(HTTP_TIMEOUT_MS);
    int httpCode = http.POST(body);
    if (httpCode != HTTP_CODE_OK) {
      Serial.printf("HTTP Error %d (attempt %d/%d)\n", httpCode, attempt, MAX_HTTP_ATTEMPTS);
      Serial.println(http.errorToString(httpCode));
      http.end();
      if (attempt < MAX_HTTP_ATTEMPTS) {
        delay(500);
        continue;
      }
      return "Unable to contact AI";
    }
    String response = http.getString();
    http.end();

    // Parse JSON response and extract generated text from "response".
    JsonDocument res;
    DeserializationError err = deserializeJson(res, response);
    if (err) {
      Serial.printf("JSON parse error (attempt %d/%d): %s\n", attempt, MAX_HTTP_ATTEMPTS,
                    err.c_str());
      if (attempt < MAX_HTTP_ATTEMPTS) {
        delay(300);
        continue;
      }
      return "JSON Error";
    }
    String fortune = res["response"] | "";
    fortune.trim();
    // Remove surrounding quotes if the model still wraps output.
    if (fortune.length() >= 2 &&
        ((fortune.startsWith("\"") && fortune.endsWith("\"")) ||
         (fortune.startsWith("'") && fortune.endsWith("'")))) {
      fortune = fortune.substring(1, fortune.length() - 1);
      fortune.trim();
    }
    return fortune;
  }
  return "Unable to contact AI";
}
// =================================================
// One-time initialization sequence.
// Steps:
// 1) Initialize GPIO/serial/I2C.
// 2) Initialize OLED and halt if not found.
// 3) Connect Wi-Fi.
// 4) Fetch first fortune and render it.
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
    while (true) {
    }
  }
  display.clearDisplay();
  display.display();
  showText("ESP32 Fortune Teller");
  connectWiFi();
  String fortune = getFortune();
  Serial.println(fortune);
  showFortune(fortune);
  blink(2);
  lastRequest = millis();
}
// =================================================
// Main loop polls based on REQUEST_INTERVAL and refreshes the fortune.
// The interval check is non-blocking except while HTTP request is in flight.
void loop() {
  if (millis() - lastRequest > REQUEST_INTERVAL) {
    showText("Consulting the\nAI oracle...");
    String fortune = getFortune();
    Serial.println();
    Serial.println("Fortune:");
    Serial.println(fortune);
    showFortune(fortune);
    blink(2);
    lastRequest = millis();
  }
}

