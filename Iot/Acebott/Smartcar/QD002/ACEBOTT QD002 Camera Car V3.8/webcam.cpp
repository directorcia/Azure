// ESP32-CAM WebServer (STA mode) providing near-live MJPEG stream
// This code creates a WiFi-enabled camera webserver that streams video and captures images

#include <Arduino.h>        // Arduino core functionality
#include <WiFi.h>           // WiFi connectivity in Station mode
#include "esp_camera.h"     // ESP32 camera driver library
#include "io_config.h"      // WiFi credentials (WIFI_SSID, WIFI_PASSWORD)

// Serial monitor baud rate for debugging output
static const long MONITOR_BAUD = 115200;

// ========================================================================
// AI-Thinker ESP32-CAM Camera Pin Configuration
// ========================================================================
// These pins connect the ESP32 to the OV2640 camera module
// DO NOT CHANGE unless using different ESP32-CAM board variant

#define PWDN_GPIO_NUM     32  // Camera power down pin (active high)
#define RESET_GPIO_NUM    -1  // Camera reset pin (not connected on AI-Thinker)
#define XCLK_GPIO_NUM      0  // Camera clock signal
#define SIOD_GPIO_NUM     26  // Camera I2C data line (SCCB protocol)
#define SIOC_GPIO_NUM     27  // Camera I2C clock line (SCCB protocol)

// Camera parallel data pins (8-bit bus for image data)
#define Y9_GPIO_NUM       35  // Most significant bit (MSB)
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5  // Least significant bit (LSB)

// Camera timing and sync signals
#define VSYNC_GPIO_NUM    25  // Vertical sync - marks start of new frame
#define HREF_GPIO_NUM     23  // Horizontal reference - marks valid pixel data
#define PCLK_GPIO_NUM     22  // Pixel clock - synchronizes data transfer  // Pixel clock - synchronizes data transfer

// Create HTTP server listening on port 80 (standard HTTP port)
WiFiServer httpServer(80);

// ========================================================================
// HTTP Response Handlers
// ========================================================================

/**
 * Sends the main HTML index page to the client
 * Displays embedded video stream and navigation links
 * @param client - Connected WiFi client to send response to
 */
static void sendIndex(WiFiClient &client) {
  // Send HTTP response headers
  client.print(
    "HTTP/1.1 200 OK\r\n"              // Status: successful request
    "Content-Type: text/html\r\n"      // Content is HTML webpage
    "Connection: close\r\n\r\n"        // Close connection after sending
  );
  
  // Send HTML page body with embedded video stream
  client.print(
    "<html><head><title>QD002 CAM</title></head><body>"
    "<h2>QD002 Camera WebServer (AP)</h2>"
    "<p>Stream: <a href=\"/stream\">/stream</a></p>"           // Link to stream endpoint
    "<img src=\"/stream\" style=\"max-width:100%; height:auto\"/>"  // Embedded live stream
    "<p><a href=\"/capture\">Capture still</a></p>"            // Link to capture endpoint
    "</body></html>"
  );
}

/**
 * Captures and sends a single JPEG image to the client
 * @param client - Connected WiFi client to send image to
 */
static void sendCapture(WiFiClient &client) {
  // Request a frame buffer from the camera driver
  camera_fb_t *fb = esp_camera_fb_get();
  
  // Check if frame capture was successful
  if (!fb) {
    // Send error response if camera failed to capture
    client.print("HTTP/1.1 500 FAIL\r\nConnection: close\r\n\r\n");
    return;
  }
  
  // Send HTTP response headers for JPEG image
  client.print(
    "HTTP/1.1 200 OK\r\n"              // Status: successful request
    "Content-Type: image/jpeg\r\n"     // Content is JPEG image
    "Connection: close\r\n"            // Close after sending
  );
  
  // Send image size in bytes, then blank line to end headers
  client.printf("Content-Length: %u\r\n\r\n", (unsigned)fb->len);
  
  // Send raw JPEG data from frame buffer
  client.write((const char*)fb->buf, fb->len);
  
  // Return frame buffer to camera driver for reuse
  esp_camera_fb_return(fb);
}

/**
 * Sends continuous MJPEG video stream to the client
 * Uses multipart MIME format to send sequential JPEG frames
 * Continues streaming until client disconnects
 * @param client - Connected WiFi client to stream video to
 */
static void sendStream(WiFiClient &client) {
  // Send HTTP headers for multipart MJPEG stream
  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"  // Multipart stream format
    "Cache-Control: no-cache, no-store, must-revalidate\r\n"      // Disable caching
    "Pragma: no-cache\r\n"                                         // HTTP 1.0 cache control
    "Connection: close\r\n\r\n"                                    // Will close when done
  );
  
  // Continuous streaming loop - runs until client disconnects
  while (client.connected()) {
    // Capture a single frame from camera
    camera_fb_t *fb = esp_camera_fb_get();
    
    // If frame capture fails, exit streaming loop
    if (!fb) {
      break;
    }
    
    // Send multipart boundary marker (separates frames)
    client.print("--frame\r\n");
    
    // Send headers for this frame part
    client.print("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %u\r\n\r\n", (unsigned)fb->len);
    
    // Send the actual JPEG image data
    client.write((const char*)fb->buf, fb->len);
    client.print("\r\n");
    
    // Return frame buffer to camera driver for next capture
    esp_camera_fb_return(fb);
    
    // Delay to achieve ~25 FPS (1000ms / 25fps = 40ms per frame)
    delay(40);
  }
}

// ========================================================================
// Arduino Setup Function - Runs once at startup
// ========================================================================
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(MONITOR_BAUD);
  Serial.println();
  Serial.println("========================================");
  Serial.println("   ESP32-CAM WebServer (STA mode)");
  Serial.println("========================================");
  
  // ========================================================================
  // Configure Camera Hardware Settings
  // ========================================================================
  camera_config_t config;  // Camera configuration structure
  
  // LED PWM configuration for camera clock signal
  config.ledc_channel = LEDC_CHANNEL_0;  // Use LED PWM channel 0
  config.ledc_timer = LEDC_TIMER_0;      // Use LED PWM timer 0
  
  // Assign GPIO pins for 8-bit parallel camera data bus (D0-D7)
  config.pin_d0 = Y2_GPIO_NUM;  // Data bit 0 (LSB)
  config.pin_d1 = Y3_GPIO_NUM;  // Data bit 1
  config.pin_d2 = Y4_GPIO_NUM;  // Data bit 2
  config.pin_d3 = Y5_GPIO_NUM;  // Data bit 3
  config.pin_d4 = Y6_GPIO_NUM;  // Data bit 4
  config.pin_d5 = Y7_GPIO_NUM;  // Data bit 5
  config.pin_d6 = Y8_GPIO_NUM;  // Data bit 6
  config.pin_d7 = Y9_GPIO_NUM;  // Data bit 7 (MSB)
  
  // Assign timing and control pins
  config.pin_xclk = XCLK_GPIO_NUM;      // Camera clock output
  config.pin_pclk = PCLK_GPIO_NUM;      // Pixel clock input
  config.pin_vsync = VSYNC_GPIO_NUM;    // Vertical sync input
  config.pin_href = HREF_GPIO_NUM;      // Horizontal reference input
  
  // Assign I2C pins for camera configuration (SCCB protocol)
  config.pin_sccb_sda = SIOD_GPIO_NUM;  // I2C data line
  config.pin_sccb_scl = SIOC_GPIO_NUM;  // I2C clock line
  
  // Power and reset control
  config.pin_pwdn = PWDN_GPIO_NUM;      // Power down control
  config.pin_reset = RESET_GPIO_NUM;    // Reset control (-1 = not used)
  
  // Camera clock frequency: 20 MHz (optimal for OV2640 sensor)
  config.xclk_freq_hz = 20000000;
  
  // Output format: JPEG compressed images (not raw RGB)
  config.pixel_format = PIXFORMAT_JPEG;
  
  // ========================================================================
  // Adaptive Configuration Based on Available Memory
  // ========================================================================
  // Check if external PSRAM (Pseudo-Static RAM) is available
  // PSRAM allows higher resolution and smoother streaming with dual buffers
  bool psram = psramFound();
  Serial.printf("PSRAM available: %s\n", psram ? "YES" : "NO");
  
  if (psram) {
    // High quality settings when PSRAM is available
    config.frame_size = FRAMESIZE_VGA;     // 640x480 pixels
    config.jpeg_quality = 12;              // Lower = better quality (range 0-63)
    config.fb_count = 2;                   // Use 2 frame buffers for smoother streaming
  } else {
    // Reduced quality settings for limited internal RAM
    config.frame_size = FRAMESIZE_QVGA;    // 320x240 pixels (safer without PSRAM)
    config.jpeg_quality = 15;              // Slightly lower quality
    config.fb_count = 1;                   // Single frame buffer only
  }
  
  // ========================================================================
  // Initialize Camera with Error Recovery
  // ========================================================================
  // Attempt to initialize camera with configured settings
  esp_err_t err = esp_camera_init(&config);
  
  if (err != ESP_OK) {
    // First attempt failed - try minimal fallback configuration
    Serial.printf("Camera init failed: 0x%X\n", (unsigned)err);
    Serial.println("Trying fallback: QQVGA 160x120, single frame buffer...");
    
    // Fallback to minimal settings
    config.frame_size = FRAMESIZE_QQVGA;   // 160x120 pixels (minimal resolution)
    config.jpeg_quality = 20;              // Lower quality for reliability
    config.fb_count = 1;                   // Single buffer
    
    // Attempt initialization with fallback settings
    err = esp_camera_init(&config);
    
    if (err != ESP_OK) {
      // Both attempts failed - camera hardware problem
      Serial.printf("Camera fallback failed: 0x%X\n", (unsigned)err);
      Serial.println("Camera sensor may not be detected. Check ribbon cable and power.");
      return;  // Exit setup - system will not function without camera
    }
  }
  
  // ========================================================================
  // Connect to WiFi Network
  // ========================================================================
  // Set WiFi mode to Station (client mode - connects to existing network)
  WiFi.mode(WIFI_STA);
  
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  // Begin connection attempt using credentials from io_config.h
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Wait for connection with timeout (20 attempts × 500ms = 10 seconds)
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);           // Wait 500ms between checks
    Serial.print(".");    // Print progress indicator
    attempts++;
  }
  Serial.println();
  
  // Check if connection was successful
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection failed. Check SSID/password in main.cpp");
    return;  // Exit setup - webserver requires WiFi
  }
  
  // Connection successful - get assigned IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("WiFi connected. IP: ");
  Serial.println(ip);
  
  // ========================================================================
  // Start HTTP Web Server
  // ========================================================================
  // Begin listening for HTTP requests on port 80
  httpServer.begin();
  Serial.println("HTTP server started at http://" + ip.toString());
  Serial.println("Access camera stream at: http://" + ip.toString() + "/stream");
}

// ========================================================================
// Arduino Main Loop - Runs continuously after setup()
// ========================================================================
void loop() {
  // Check if a client has connected to the web server
  WiFiClient client = httpServer.available();
  
  // If no client is waiting, return immediately and check again
  if (!client) return;
  
  // ========================================================================
  // Parse HTTP Request
  // ========================================================================
  // Read the first line of HTTP request (e.g., "GET /stream HTTP/1.1")
  String reqLine = client.readStringUntil('\n');
  reqLine.trim();  // Remove leading/trailing whitespace
  
  // Skip remaining HTTP headers until we find blank line
  // We don't need header data for this simple server
  while (client.connected()) {
    String h = client.readStringUntil('\n');
    if (h.length() <= 2) break;  // Blank line marks end of headers
  }
  
  // ========================================================================
  // Route Request to Appropriate Handler
  // ========================================================================
  // Check request path and call corresponding function
  if (reqLine.startsWith("GET / ")) {
    // Root path: send main HTML page
    sendIndex(client);
    
  } else if (reqLine.startsWith("GET /stream")) {
    // Stream endpoint: send continuous MJPEG video
    sendStream(client);
    
  } else if (reqLine.startsWith("GET /capture")) {
    // Capture endpoint: send single JPEG image
    sendCapture(client);
    
  } else {
    // Unknown path: send 404 error
    client.print("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n");
    client.print("Not Found");
  }
  
  // Close client connection
  // (streaming will have already closed when client disconnected)
  client.stop();
}