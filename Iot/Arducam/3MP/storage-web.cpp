/**
 * @file storage-web.cpp
 * @brief Timed image capture from an Arducam Mega (ESP32) and upload to Azure Blob Storage over HTTPS using a SAS token.
 *
 * This sketch configures an ESP32 with an Arducam Mega camera to periodically capture
 * a JPEG image at 3MP (QXGA) resolution and upload it directly to Azure Blob Storage
 * via an HTTP PUT request over TLS (port 443). It can optionally overwrite a fixed
 * blob name (e.g., latest.jpg) for easy consumption by web clients.
 *
 * Hardware
 * - Board: ESP32 (tested on common ESP32 dev boards)
 * - Camera: Arducam Mega connected to VSPI
 *   - MOSI: GPIO 23
 *   - MISO: GPIO 19
 *   - SCK : GPIO 18
 *   - CS  : GPIO 5
 * - Optional noise sensor: GPIO 34 (input only), configured but not used in timed mode
 * - Status LED: GPIO 2 (reserved; not driven)
 *
 * Dependencies
 * - Arduino core for ESP32
 * - Arducam Mega library
 * - WiFi and WiFiClientSecure libraries
 *
 * Configuration (io_config.h)
 * - AZURE_STORAGE_ACCOUNT: Storage account name (e.g., mystorageacct)
 * - AZURE_CONTAINER      : Target container (e.g., images)
 * - AZURE_SAS_TOKEN      : SAS query string (without leading '?'), including permissions and expiry
 * - WIFI_SSID / WIFI_PASSWORD: Network credentials
 *
 * How It Works
 * 1. Initializes SPI and the Arducam Mega, sets image quality.
 * 2. Ensures Wi-Fi connectivity.
 * 3. Every CAPTURE_INTERVAL_MS, captures a JPEG image at QXGA and uploads it to Azure Blob Storage.
 * 4. The HTTP PUT includes a fixed Content-Length equal to camera-reported size.
 * 5. Streams the captured bytes; once the JPEG end marker (0xFF 0xD9) is found, any remaining
 *    bytes up to Content-Length are padded with zeros to honor the declared length.
 * 6. Expects HTTP 201 Created on success.
 *
 * Alternative Flow
 * - The helper `captureImage()` streams the JPEG to Serial with headers so a host can
 *   consume the image from the serial port (SIZE + raw bytes + END_IMAGE_DATA).
 *
 * Notes
 * - GPIO 34 is input-only on ESP32; do not drive it.
 * - `WiFiClientSecure::setInsecure()` is used to skip certificate validation. For production,
 *   consider pinning the certificate or using proper trust anchors.
 *
 * Limitations
 * - Azure SAS must grant `w` (write) permission on the container for PUT operations.
 * - Network instability may cause timeouts; retries are minimal here for simplicity.
 */
#include <Arduino.h>
#include <Arducam_Mega.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <driver/adc.h>
#include "io_config.h"

// Function declarations (defined later)
/**
 * @brief Capture a JPEG and stream it to Serial for host-side retrieval.
 * @param resolution Camera resolution (e.g., `CAM_IMAGE_MODE_QXGA`).
 */
void captureImage(CAM_IMAGE_MODE resolution);
/**
 * @brief Capture a JPEG and upload to Azure Blob Storage via HTTPS.
 * @param resolution    Camera resolution to use for capture.
 * @param useFixedName  When true, uploads to a constant blob name (e.g., latest.jpg);
 *                      otherwise uses a unique, timestamped name.
 */
void captureAndUpload(CAM_IMAGE_MODE resolution, bool useFixedName = false);
/**
 * @brief Ensure Wi-Fi is connected; attempts connection if disconnected.
 * @return true if connected to Wi-Fi; false on timeout/failure.
 */
bool ensureWifi();

/**
 * @section SPI_Pins
 * VSPI pin mapping for camera transport layer:
 * - MOSI: GPIO 23
 * - MISO: GPIO 19
 * - SCK : GPIO 18
 * - CS  : GPIO 5
 */
const int PIN_MOSI = 23;
const int PIN_MISO = 19;
const int PIN_SCK  = 18;
const int CS_PIN   = 5;
/**
 * @brief Global Arducam Mega camera instance using `CS_PIN`.
 */
Arducam_Mega myCAM(CS_PIN);

/**
 * @section Noise_Sensor
 * Optional noise sensor input. In this sketch, noise sensing is configured but not used for
 * capture triggering (timed mode only). If enabling noise-triggered capture, consider using
 * the hysteresis thresholds below to avoid oscillation. GPIO 34 is input-only; do not drive it.
 *
 * Example voltage divider for a 5V sensor output -> ESP32 GPIO34 (max 3.3V):
 *   D0 (5V) --[10k]--+--[10k]-- GND
 *                    |
 *                  GPIO34 (reads ~2.5V when D0 = 5V)
 */
const int NOISE_PIN = 34;
unsigned long lastNoiseTrigger = 0;
const unsigned long NOISE_COOLDOWN_MS = 5000;
const int NOISE_HIGH_SAMPLES = 3; // require N consecutive HIGHs
const int NOISE_LOW_SAMPLES  = 4; // require N consecutive LOWs to re-arm
const bool NOISE_ACTIVE_HIGH = true; // set false if your module idles HIGH and pulses LOW
const bool USE_ANALOG_NOISE  = true; // set true to use ADC threshold instead of D0 digital level
const int NOISE_ANALOG_HIGH  = 2200; // trigger when analogRead >= this (0-4095)
const int NOISE_ANALOG_LOW   = 1800; // re-arm when analogRead <= this (hysteresis)
const bool NOISE_AUTO_BASELINE = true; // if true, derive thresholds from a rolling baseline
const int NOISE_MARGIN = 20;          // margin around baseline for trigger when auto-baseline

// Status LED (on-board): reserved/unused to avoid conflicts
const int STATUS_LED_PIN = 2;

// Capture counter for unique filenames when needed
static uint32_t captureCounter = 0;

/**
 * @brief Timed capture period (milliseconds). Image capture+upload is triggered every interval.
 */
const unsigned long CAPTURE_INTERVAL_MS = 60000; // 60 seconds

/**
 * @brief Azure Blob endpoint host derived from `AZURE_STORAGE_ACCOUNT`.
 *        Example: mystorageacct.blob.core.windows.net
 */
String azureBlobHost = String(AZURE_STORAGE_ACCOUNT) + ".blob.core.windows.net";

/**
 * @brief Arduino setup routine.
 * - Initializes serial @ 115200 bps.
 * - Starts VSPI and Arducam Mega camera, sets high JPEG quality.
 * - Configures ADC resolution and attenuation for optional noise input.
 * - Attempts Wi-Fi connection early to enable first upload.
 */
void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  Serial.println("========================================");
  Serial.println("  Arducam Mega 3MP Camera - Live Test  ");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Initializing camera...");

  // Initialize SPI on the specified VSPI pins
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CS_PIN);
  
  // Initialize camera
  myCAM.begin();

  Serial.println("SUCCESS! Camera initialized!");
  Serial.println();
  Serial.println("========================================");
  Serial.println("Camera is ready!");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Mode: timed capture every 60s at max resolution (3MP)");

  // Noise sensor input (not used in timed mode but kept configured safely)
  pinMode(NOISE_PIN, INPUT);
  analogReadResolution(12); // full 0-4095 range on ESP32
  analogSetPinAttenuation(NOISE_PIN, ADC_0db); // best resolution for 0-3.3V sources

  // Status LED unused; reserved pin
  pinMode(STATUS_LED_PIN, INPUT);

  // Prefer highest quality for uploads
  myCAM.setImageQuality(HIGH_QUALITY);

  // Bring Wi-Fi up on boot so first capture can upload
  ensureWifi();
}

/**
 * @brief Main loop: performs a timed capture and upload.
 *
 * Captures a 3MP (QXGA) JPEG and uploads every `CAPTURE_INTERVAL_MS`. When `useFixedName`
 * is true (as used here), the blob `latest.jpg` is overwritten each cycle.
 */
void loop() {
  static unsigned long lastCaptureMs = 0;
  unsigned long now = millis();

  if (now - lastCaptureMs >= CAPTURE_INTERVAL_MS) {
    lastCaptureMs = now;
    Serial.println();
    Serial.println("[TIMER] Capturing at max resolution and uploading as latest.jpg...");
    captureAndUpload(CAM_IMAGE_MODE_QXGA, true);
  }

  delay(50);
}

/**
 * @brief Ensure Wi-Fi connectivity.
 * Attempts to connect to the configured SSID up to 20 seconds.
 * @return true if `WiFi.status() == WL_CONNECTED`; false otherwise.
 */
bool ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("✗ WiFi connect failed");
    return false;
  }
}

/**
 * @brief Capture a JPEG and upload it to Azure Blob Storage over HTTPS.
 *
 * This function captures a JPEG with the specified resolution, opens a TLS connection
 * to Azure Blob Storage, and issues an HTTP PUT request using a SAS token appended as
 * query parameters. The `Content-Length` header matches the camera-reported size.
 *
 * Data Streaming Strategy
 * - Bytes are read from the camera and buffered to the TLS socket.
 * - When the JPEG end marker (0xFF 0xD9) is encountered, any remaining bytes up to
 *   `Content-Length` are padded with zeros to align with the declared size.
 * - This ensures Azure accepts the payload length even if the camera ends the JPEG early.
 *
 * Response Handling
 * - Waits for server response and checks the HTTP status line.
 * - Treats `HTTP/1.1 201 Created` as success; logs response headers for diagnostics.
 *
 * @param resolution   Camera resolution (e.g., `CAM_IMAGE_MODE_QXGA`).
 * @param useFixedName If true, overwrites `latest.jpg`. If false, generates a unique name
 *                     using `captureCounter` and a millisecond timestamp.
 */
void captureAndUpload(CAM_IMAGE_MODE resolution, bool useFixedName) {
  Serial.println("┌─────────────────────────────────────┐");
  Serial.println("│  CAPTURE + AZURE UPLOAD            │");
  Serial.println("└─────────────────────────────────────┘");

  CamStatus status = myCAM.takePicture(resolution, CAM_IMAGE_PIX_FMT_JPG);
  if (status != CAM_ERR_SUCCESS) {
    Serial.println("✗ Capture FAILED!");
    Serial.print("Error code: ");
    Serial.println(status);
    Serial.println();
    return;
  }

  delay(100);

  uint32_t imageSize = myCAM.getTotalLength();
  Serial.print("Image size: ");
  Serial.print(imageSize);
  Serial.println(" bytes");

  if (imageSize == 0) {
    Serial.println("✗ Image size is 0, cannot upload");
    Serial.println();
    return;
  }

  if (!ensureWifi()) {
    Serial.println("✗ WiFi failed");
    Serial.println();
    return;
  }

  // Build blob path; reuse same name when requested to overwrite
  captureCounter++;
  uint32_t timestamp = millis();
  String blobName = useFixedName ? "latest.jpg" : "image_" + String(captureCounter) + "_" + String(timestamp) + ".jpg";
  String azureBlobPath = "/" + String(AZURE_CONTAINER) + "/" + blobName + "?" + String(AZURE_SAS_TOKEN);

  WiFiClientSecure client;
  client.setInsecure();

  Serial.print("Connecting to Azure host: ");
  Serial.println(azureBlobHost);

  if (!client.connect(azureBlobHost.c_str(), 443)) {
    Serial.println("✗ Connection to Azure failed");
    Serial.println();
    return;
  }
  Serial.println("✓ Connected to Azure");

  // Send HTTP PUT with known Content-Length (SAS grants write permission)
  Serial.println("Sending PUT request...");
  client.print("PUT ");
  client.print(azureBlobPath);
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(azureBlobHost);
  client.println("Content-Type: image/jpeg");
  client.print("Content-Length: ");
  client.println(imageSize);
  client.println("x-ms-blob-type: BlockBlob");
  client.println("Connection: close");
  client.println();

  // Stream exactly imageSize bytes from camera to socket with buffering
  Serial.println("Streaming image bytes...");
  uint32_t sent = 0;
  uint8_t prevByte = 0;
  const uint16_t BUFFER_SIZE = 4096;
  uint8_t buffer[BUFFER_SIZE];
  uint16_t bufferPos = 0;
  
  while (sent < imageSize) {
    uint8_t byte = myCAM.readByte();
    buffer[bufferPos++] = byte;
    sent++;
    
    // When buffer is full or we've read all data, flush to socket
    if (bufferPos >= BUFFER_SIZE || sent >= imageSize) {
      client.write(buffer, bufferPos);
      bufferPos = 0;
    }
    
    // Progress indicator
    if (sent % 5000 == 0) {
      Serial.print(".");
    }
    
    // Stop at JPEG end marker and pad with zeros if needed
    if (prevByte == 0xFF && byte == 0xD9) {
      Serial.println("✓ Found JPEG end marker");
      // Flush any remaining data in buffer
      if (bufferPos > 0) {
        client.write(buffer, bufferPos);
        bufferPos = 0;
      }
      // Pad remaining with zeros
      while (sent < imageSize) {
        buffer[bufferPos++] = 0x00;
        sent++;
        if (bufferPos >= BUFFER_SIZE) {
          client.write(buffer, bufferPos);
          bufferPos = 0;
        }
      }
      break;
    }
    prevByte = byte;
  }
  
  // Flush any remaining buffered data
  if (bufferPos > 0) {
    client.write(buffer, bufferPos);
  }
  
  Serial.println();
  Serial.print("Sent ");
  Serial.print(sent);
  Serial.println(" bytes");

  // Ensure all data is sent to the network stack
  client.flush();
  Serial.println("Flushed data, waiting for response...");

  // Wait for response
  unsigned long timeout = millis();
  while (!client.available() && millis() - timeout < 10000) {
    delay(50);
  }

  if (!client.available()) {
    Serial.println("✗ No response from server (timeout)");
    client.stop();
    Serial.println();
    return;
  }

  // Read HTTP status line (e.g., "HTTP/1.1 201 Created")
  String statusLine = client.readStringUntil('\n');
  statusLine.trim();
  Serial.print("Azure response: ");
  Serial.println(statusLine);

  // Print additional response headers/body for debugging
  while (client.available()) {
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.print("  ");
      Serial.println(line);
    }
  }

  bool success = statusLine.startsWith("HTTP/1.1 201");
  if (!success) {
    Serial.println("✗ Upload did not return 201 Created");
  } else {
    Serial.print("✓ Upload success (201 Created)");
    Serial.println();
    Serial.print("  Filename: ");
    Serial.println(blobName);
  }

  client.stop();
  Serial.println();
}
/**
 * @brief Capture a JPEG and stream it to Serial.
 *
 * Emits a compact header that a host tool can parse:
 * - `SIZE:<bytes>` on a single line
 * - `START_BINARY_DATA` marker
 * - Raw JPEG bytes until the end marker (0xFF 0xD9) or safety limit
 * - `END_IMAGE_DATA` marker
 *
 * This path is useful for debugging or host-side ingestion where the image is
 * retrieved over serial rather than uploaded to a remote service.
 *
 * @param resolution Camera resolution to capture (e.g., `CAM_IMAGE_MODE_VGA`).
 */
void captureImage(CAM_IMAGE_MODE resolution) {
  String resName;
  switch(resolution) {
    case CAM_IMAGE_MODE_QVGA:  resName = "QVGA (320x240)"; break;
    case CAM_IMAGE_MODE_VGA:   resName = "VGA (640x480)"; break;
    case CAM_IMAGE_MODE_FHD:   resName = "1080p (1920x1080)"; break;
    case CAM_IMAGE_MODE_QXGA:  resName = "3MP (2048x1536)"; break;
    default: resName = "Unknown"; break;
  }
  
  Serial.println("┌─────────────────────────────────────┐");
  Serial.print("│  CAPTURING: ");
  Serial.print(resName);
  for(int i = resName.length(); i < 23; i++) Serial.print(" ");
  Serial.println("│");
  Serial.println("└─────────────────────────────────────┘");
  
  unsigned long startTime = millis();

  // Take a picture with JPEG format
  CamStatus status = myCAM.takePicture(resolution, CAM_IMAGE_PIX_FMT_JPG);
  
  if (status != CAM_ERR_SUCCESS) {
    Serial.println("✗ Capture FAILED!");
    Serial.print("Error code: ");
    Serial.println(status);
    Serial.println();
    return;
  }

  // Wait a bit for capture to complete
  delay(100);

  unsigned long captureTime = millis() - startTime;
  uint32_t imageSize = myCAM.getTotalLength();

  Serial.println();
  Serial.println("✓ IMAGE CAPTURED!");
  Serial.println("─────────────────────────────────────");
  Serial.print("Capture time:  ");
  Serial.print(captureTime);
  Serial.println(" ms");
  Serial.print("Image size:    ");
  Serial.print(imageSize);
  Serial.println(" bytes");
  Serial.print("Size (KB):     ");
  Serial.println(imageSize / 1024.0, 2);

  // Use a default size if getTotalLength() returns 0
  // VGA is roughly 30-50KB, estimate 100KB max
  if (imageSize == 0) {
    imageSize = 100000;
    Serial.println("⚠ Using estimated size (getTotalLength = 0)");
  }

  Serial.println();

  // Send image size as first line
  Serial.print("SIZE:");
  Serial.println(imageSize);
  Serial.println("START_BINARY_DATA");

  // Flush serial to ensure headers are sent
  delay(100);

  // Stream image bytes directly
  uint32_t bytesRead = 0;
  uint8_t prevByte = 0;
  bool foundEnd = false;

  while (bytesRead < imageSize) {
    uint8_t byte = myCAM.readByte();
    Serial.write(byte);
    bytesRead++;

    // Check for JPEG end marker
    if (prevByte == 0xFF && byte == 0xD9) {
      foundEnd = true;
      break;
    }
    
    prevByte = byte;

    // Safety check - if we've read way too much, stop
    if (bytesRead > 200000) {
      break;
    }
  }

  // Send end of transmission
  Serial.println();
  Serial.println("END_IMAGE_DATA");
  Serial.println("─────────────────────────────────────");
  Serial.println();
}
