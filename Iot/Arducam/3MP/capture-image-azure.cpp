/*
 * Arducam Mega 3MP Camera with Azure Blob Storage Upload
 * 
 * This sketch captures images from an Arducam Mega 3MP camera module connected via SPI
 * to an ESP32 microcontroller. Images can be displayed locally or uploaded directly to
 * Azure Blob Storage over HTTPS.
 * 
 * Hardware:
 *  - Arducam Mega 3MP camera module (3-megapixel, JPEG format)
 *  - ESP32 microcontroller (with WiFi capability)
 *  - SPI communication between ESP32 and camera
 * 
 * Features:
 *  - Multiple resolution support (QVGA, VGA, 1080p, 3MP)
 *  - Adjustable image quality (HIGH, MEDIUM, LOW)
 *  - HTTPS upload to Azure Blob Storage with SAS token authentication
 *  - Real-time progress reporting via serial monitor
 *  - Buffered streaming to prevent memory overflow
 */

// Include necessary libraries for camera, networking, and SPI communication
#include <Arduino.h>           // Arduino core functionality
#include <Arducam_Mega.h>      // Arducam camera driver library
#include <SPI.h>               // SPI protocol for camera communication
#include <WiFi.h>              // WiFi connectivity (ESP32)
#include <WiFiClientSecure.h>  // HTTPS secure connection support
#include "io_config.h"         // External config with WiFi and Azure credentials

// Function declarations (prototypes) - defined later in this file
void captureImage(CAM_IMAGE_MODE resolution);         // Capture and stream image locally
void captureAndUpload(CAM_IMAGE_MODE resolution);    // Capture image and upload to Azure
bool ensureWifi();                                     // Establish WiFi connection if needed
// ==================== HARDWARE PIN CONFIGURATION ====================
// ESP32 VSPI (Variable Speed SPI) pins for Arducam camera communication
// These pins are used for the SPI bus that connects to the camera module
const int PIN_MOSI = 23;  // Master Out Slave In (GPIO 23)  - Data from ESP32 to camera
const int PIN_MISO = 19;  // Master In Slave Out (GPIO 19)  - Data from camera to ESP32
const int PIN_SCK  = 18;  // Serial Clock (GPIO 18)          - Clock signal for synchronization
const int CS_PIN   = 5;   // Chip Select (GPIO 5)             - Enable/disable camera chip

// ==================== CAMERA OBJECT ====================
// Create Arducam Mega camera instance, passing the chip select pin
// This object handles all camera-related operations (capture, settings, etc)
Arducam_Mega myCAM(CS_PIN);

// ==================== GLOBAL VARIABLES ====================
// Counter to generate unique filenames for each captured image
// Incremented after each upload to Azure; used in filename: image_1, image_2, etc
static uint32_t captureCounter = 0;

// Pre-build the Azure Blob Storage hostname from configuration
// Format: {StorageAccountName}.blob.core.windows.net
// Example: mystorageaccount.blob.core.windows.net
String azureBlobHost = String(AZURE_STORAGE_ACCOUNT) + ".blob.core.windows.net";
// ==================== SETUP FUNCTION ====================
// Called once when the ESP32 powers on or resets
// Purpose: Initialize hardware (serial, SPI, camera) and display welcome message
void setup() {
  // Initialize serial communication at 115200 baud rate
  // This allows communication with a computer via USB cable for debugging and commands
  Serial.begin(115200);
  
  // Display welcome banner to the serial monitor
  Serial.println("========================================");
  Serial.println("  Arducam Mega 3MP Camera - Live Test  ");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Initializing camera...");
  
  // Initialize the SPI (Serial Peripheral Interface) bus
  // This sets up the GPIO pins for communication with the camera
  // Parameters: (clock_pin, miso_pin, mosi_pin, chip_select_pin)
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CS_PIN);
  
  // Initialize the camera module
  // This performs handshake with camera and prepares it for operation
  myCAM.begin();
  Serial.println("SUCCESS! Camera initialized!");
  Serial.println();
  
  // Display startup completed message
  Serial.println("========================================");
  Serial.println("Camera is ready!");
  Serial.println("========================================");
  Serial.println();
  
  // Display available commands for the user
  // User sends single characters or short commands via serial monitor
  Serial.println("Commands:");
  Serial.println("  Capture Commands:");
  Serial.println("    'c' - Capture image (VGA - default)");
  Serial.println("    '1' - Capture QVGA (320x240)");
  Serial.println("    '2' - Capture VGA (640x480)");
  Serial.println("    '3' - Capture 1080p (1920x1080)");
  Serial.println("    '4' - Capture 3MP (2048x1536)");
  Serial.println();
  Serial.println("  Capture + Upload to Azure:");
  Serial.println("    'u' - Upload VGA to Azure");
  Serial.println("    'u1' - Upload QVGA to Azure");
  Serial.println("    'u2' - Upload VGA to Azure");
  Serial.println("    'u3' - Upload 1080p to Azure");
  Serial.println("    'u4' - Upload 3MP to Azure");
  Serial.println();
  Serial.println("  Quality Settings:");
  Serial.println("    'q' - Set quality HIGH (smaller files)");
  Serial.println("    'w' - Set quality MEDIUM (default)");
  Serial.println("    'e' - Set quality LOW (larger files)");
  Serial.println();
}
// ==================== MAIN LOOP FUNCTION ====================
// Called repeatedly (roughly 100 times per second at 10ms delay)
// Purpose: Read serial input from user and execute corresponding commands
void loop() {
  // Check if any data is available from the serial port (e.g., from serial monitor)
  if (Serial.available() > 0) {
    // Read one character from the serial buffer
    char cmd = Serial.read();
    
    // Process the command using a switch statement
    // Each case handles a different user input
    switch(cmd) {
      // ===== SIMPLE CAPTURE COMMANDS =====
      case 'c':
      case 'C':
        // Capture image at VGA resolution (640x480)
        captureImage(CAM_IMAGE_MODE_VGA);
        break;
      case '1':
        // Capture image at QVGA resolution (320x240) - lowest resolution
        captureImage(CAM_IMAGE_MODE_QVGA);
        break;
      case '2':
        // Capture image at VGA resolution (640x480)
        captureImage(CAM_IMAGE_MODE_VGA);
        break;
      case '3':
        // Capture image at 1080p resolution (1920x1080) - Full HD
        captureImage(CAM_IMAGE_MODE_FHD);
        break;
      case '4':
        // Capture image at 3MP resolution (2048x1536) - highest resolution
        captureImage(CAM_IMAGE_MODE_QXGA);
        break;
      
      // ===== CAPTURE AND UPLOAD COMMANDS =====
      case 'u':
      case 'U':
        // Check if there's a second character following 'u' (like u1, u2, u3, u4)
        if (Serial.available() > 0) {
          delay(10); // Small delay to ensure next character has arrived
          char subcmd = Serial.read();  // Read the second character
          
          // Process the sub-command (resolution selection for upload)
          switch(subcmd) {
            case '1':
              // Capture and upload at QVGA resolution
              captureAndUpload(CAM_IMAGE_MODE_QVGA);
              break;
            case '2':
              // Capture and upload at VGA resolution
              captureAndUpload(CAM_IMAGE_MODE_VGA);
              break;
            case '3':
              // Capture and upload at 1080p resolution
              captureAndUpload(CAM_IMAGE_MODE_FHD);
              break;
            case '4':
              // Capture and upload at 3MP resolution
              captureAndUpload(CAM_IMAGE_MODE_QXGA);
              break;
            default:
              // If second character is not 1-4, treat just 'u' as VGA upload
              captureAndUpload(CAM_IMAGE_MODE_VGA);
              break;
          }
        } else {
          // No second character available, default to VGA resolution
          captureAndUpload(CAM_IMAGE_MODE_VGA);
        }
        break;
      
      // ===== IMAGE QUALITY COMMANDS =====
      case 'q':
      case 'Q':
        // Set image quality to HIGH (results in smaller file sizes due to more compression)
        myCAM.setImageQuality(HIGH_QUALITY);
        Serial.println("✓ Quality: HIGH (more compression, smaller files)");
        break;
      case 'w':
      case 'W':
        // Set image quality to MEDIUM/DEFAULT (balanced compression and quality)
        myCAM.setImageQuality(DEFAULT_QUALITY);
        Serial.println("✓ Quality: DEFAULT (balanced compression)");
        break;
      case 'e':
      case 'E':
        // Set image quality to LOW (minimal compression, larger file sizes but better quality)
        myCAM.setImageQuality(LOW_QUALITY);
        Serial.println("✓ Quality: LOW (less compression, larger files)");
        break;
    }
  }
  
  // Small delay to prevent the loop from running too fast
  // Also gives the CPU time to handle WiFi and other background tasks
  delay(10);
}
// ==================== ENSURE WIFI FUNCTION ====================
// Purpose: Verify WiFi connection exists; connect if necessary
// Returns: true if WiFi is connected, false if connection failed
bool ensureWifi() {
  // First, check if WiFi is already connected from a previous call
  if (WiFi.status() == WL_CONNECTED) {
    return true;  // Already connected, no need to do anything
  }
  
  // WiFi not connected, attempt to establish connection
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  // Set WiFi mode to Station (STA) - connects to existing network
  WiFi.mode(WIFI_STA);
  
  // Start WiFi connection using SSID and password from io_config.h
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Record the current time to implement a 20-second timeout
  unsigned long start = millis();
  
  // Loop until connected or timeout (20 seconds = 20000 milliseconds)
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);           // Wait half a second between status checks
    Serial.print('.');    // Print a dot to show progress
  }
  
  Serial.println();  // New line after the progress dots
  
  // Check final connection status
  if (WiFi.status() == WL_CONNECTED) {
    // Connection successful - display IP address
    Serial.print("✓ WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    // Connection failed after 20-second timeout
    Serial.println("✗ WiFi connect failed");
    return false;
  }
}
// ==================== CAPTURE AND UPLOAD FUNCTION ====================
// Purpose: Capture an image from camera and upload it to Azure Blob Storage
// Parameters:
//   resolution - Image resolution mode (QVGA, VGA, FHD, or QXGA)
void captureAndUpload(CAM_IMAGE_MODE resolution) {
  // Display banner indicating the operation starting
  Serial.println("┌─────────────────────────────────────┐");
  Serial.println("│  CAPTURE + AZURE UPLOAD            │");
  Serial.println("└─────────────────────────────────────┘");
  
  // STEP 1: CAPTURE IMAGE FROM CAMERA
  // Send command to camera to capture a JPEG image at specified resolution
  CamStatus status = myCAM.takePicture(resolution, CAM_IMAGE_PIX_FMT_JPG);
  
  // Check if capture was successful
  if (status != CAM_ERR_SUCCESS) {
    Serial.println("✗ Capture FAILED!");
    Serial.print("Error code: ");
    Serial.println(status);
    Serial.println();
    return;  // Exit function early - cannot upload without an image
  }
  
  // Small delay to allow camera to finish processing
  delay(100);
  
  // Get the size of the captured image in bytes
  uint32_t imageSize = myCAM.getTotalLength();
  Serial.print("Image size: ");
  Serial.print(imageSize);
  Serial.println(" bytes");
  
  // Validate that image was actually captured
  if (imageSize == 0) {
    Serial.println("✗ Image size is 0, cannot upload");
    Serial.println();
    return;  // Exit function - no image data to send
  }
  
  // STEP 2: ENSURE WIFI CONNECTION
  // We need WiFi to reach Azure cloud storage
  if (!ensureWifi()) {
    Serial.println("✗ WiFi failed");
    Serial.println();
    return;  // Exit function - cannot upload without internet
  }
  
  // STEP 3: BUILD UNIQUE FILENAME FOR AZURE STORAGE
  // Increment counter for unique identifier (image_1, image_2, etc)
  captureCounter++;
  
  // Get current milliseconds as timestamp for additional uniqueness
  uint32_t timestamp = millis();
  
  // Construct the full Azure blob storage path with SAS token
  // Format: /{container}/image_{counter}_{timestamp}.jpg?{SAS_TOKEN}
  String azureBlobPath = "/" + String(AZURE_CONTAINER) + "/image_" + 
                         String(captureCounter) + "_" + String(timestamp) + 
                         ".jpg?" + String(AZURE_SAS_TOKEN);
  
  // STEP 4: ESTABLISH HTTPS CONNECTION TO AZURE
  // Create a secure WiFi client for HTTPS communication
  WiFiClientSecure client;
  
  // Disable certificate verification (for testing/development)
  // WARNING: In production, implement proper certificate validation
  client.setInsecure();
  
  // Display connection attempt message
  Serial.print("Connecting to Azure host: ");
  Serial.println(azureBlobHost);
  
  // Connect to Azure Blob Storage on HTTPS port 443
  if (!client.connect(azureBlobHost.c_str(), 443)) {
    Serial.println("✗ Connection to Azure failed");
    Serial.println();
    return;  // Exit function - cannot reach Azure
  }
  
  Serial.println("✓ Connected to Azure");
  
  // STEP 5: SEND HTTP PUT REQUEST HEADERS
  // HTTP PUT is used to create/upload a blob to Azure storage
  Serial.println("Sending PUT request...");
  
  // Send the request line: PUT {path} HTTP/1.1
  client.print("PUT ");
  client.print(azureBlobPath);
  client.println(" HTTP/1.1");
  
  // Send the Host header (required by HTTP/1.1)
  client.print("Host: ");
  client.println(azureBlobHost);
  
  // Specify JPEG content type
  client.println("Content-Type: image/jpeg");
  
  // Send exact image size so Azure knows how many bytes to expect
  // This is critical - Azure validates the received data matches this length
  client.print("Content-Length: ");
  client.println(imageSize);
  
  // Azure-specific header: specify this is a block blob (as opposed to append blob)
  client.println("x-ms-blob-type: BlockBlob");
  
  // Tell server to close connection after response
  client.println("Connection: close");
  
  // Empty line marks end of headers, beginning of body
  client.println();
  
  // STEP 6: STREAM IMAGE BYTES WITH BUFFERING
  // Don't send all bytes at once - use buffering to avoid memory issues on ESP32
  Serial.println("Streaming image bytes...");
  
  uint32_t sent = 0;                    // Track bytes sent so far
  uint8_t prevByte = 0;                 // Used to detect JPEG end marker (0xFF 0xD9)
  const uint16_t BUFFER_SIZE = 4096;    // 4KB buffer for efficient streaming
  uint8_t buffer[BUFFER_SIZE];          // Temporary buffer
  uint16_t bufferPos = 0;               // Current position in buffer
  
  // Loop until all image bytes are sent
  while (sent < imageSize) {
    // Read one byte from camera's image buffer
    uint8_t byte = myCAM.readByte();
    
    // Store byte in local buffer
    buffer[bufferPos++] = byte;
    sent++;
    
    // When buffer is full OR all data has been read, flush to socket
    if (bufferPos >= BUFFER_SIZE || sent >= imageSize) {
      // Send accumulated bytes to Azure
      client.write(buffer, bufferPos);
      bufferPos = 0;  // Reset buffer position
    }
    
    // Show progress every 5000 bytes sent
    if (sent % 5000 == 0) {
      Serial.print(".");
    }
    
    // JPEG format ends with specific 2-byte marker: 0xFF 0xD9
    // Detect this to find the actual image boundary
    if (prevByte == 0xFF && byte == 0xD9) {
      Serial.println("✓ Found JPEG end marker");
      
      // Flush any remaining buffered data
      if (bufferPos > 0) {
        client.write(buffer, bufferPos);
        bufferPos = 0;
      }
      
      // Azure expects exactly imageSize bytes as declared in Content-Length
      // If we found the JPEG end before imageSize, pad remaining with zeros
      while (sent < imageSize) {
        buffer[bufferPos++] = 0x00;  // Padding byte
        sent++;
        
        // Flush buffer if full
        if (bufferPos >= BUFFER_SIZE) {
          client.write(buffer, bufferPos);
          bufferPos = 0;
        }
      }
      break;  // Exit the reading loop
    }
    
    prevByte = byte;  // Save for next iteration
  }
  
  // Flush any remaining bytes still in the buffer
  if (bufferPos > 0) {
    client.write(buffer, bufferPos);
  }
  
  // Display upload progress summary
  Serial.println();
  Serial.print("Sent ");
  Serial.print(sent);
  Serial.println(" bytes");
  
  // STEP 7: WAIT FOR AZURE RESPONSE
  // Make sure all data is transmitted from ESP32 to Azure
  client.flush();
  Serial.println("Flushed data, waiting for response...");
  
  // Wait up to 10 seconds for Azure to respond
  unsigned long timeout = millis();
  while (!client.available() && millis() - timeout < 10000) {
    delay(50);  // Check every 50ms
  }
  
  // Check if we received a response before timeout
  if (!client.available()) {
    Serial.println("✗ No response from server (timeout)");
    client.stop();  // Close connection
    Serial.println();
    return;  // Exit function
  }
  
  // STEP 8: PARSE AZURE RESPONSE
  // Read the HTTP status line (first line of response)
  String statusLine = client.readStringUntil('\n');
  statusLine.trim();  // Remove whitespace
  Serial.print("Azure response: ");
  Serial.println(statusLine);
  
  // Read and display remaining response headers/body for debugging
  // This helps diagnose any issues with the upload
  while (client.available()) {
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.print("  ");
      Serial.println(line);
    }
  }
  
  // STEP 9: VERIFY UPLOAD SUCCESS
  // Azure returns HTTP 201 (Created) if blob was successfully uploaded
  // Any other code means upload failed
  bool success = statusLine.startsWith("HTTP/1.1 201");
  
  if (!success) {
    Serial.println("✗ Upload did not return 201 Created");
  } else {
    // Success! Display confirmation with the uploaded filename
    Serial.print("✓ Upload success (201 Created)");
    Serial.println();
    Serial.print("  Filename: image_");
    Serial.print(captureCounter);
    Serial.print("_");
    Serial.print(timestamp);
    Serial.println(".jpg");
  }
  
  // Close the HTTPS connection to Azure
  client.stop();
  Serial.println();
}
// ==================== CAPTURE IMAGE FUNCTION ====================
// Purpose: Capture image from camera and stream it to serial port
// Images can be captured and saved locally without uploading to Azure
// Parameters:
//   resolution - Image resolution mode (QVGA, VGA, FHD, or QXGA)
void captureImage(CAM_IMAGE_MODE resolution) {
  // Convert resolution enum to human-readable string
  String resName;
  switch(resolution) {
    case CAM_IMAGE_MODE_QVGA:  resName = "QVGA (320x240)"; break;
    case CAM_IMAGE_MODE_VGA:   resName = "VGA (640x480)"; break;
    case CAM_IMAGE_MODE_FHD:   resName = "1080p (1920x1080)"; break;
    case CAM_IMAGE_MODE_QXGA:  resName = "3MP (2048x1536)"; break;
    default: resName = "Unknown"; break;
  }
  
  // Display banner with resolution information
  Serial.println("┌─────────────────────────────────────┐");
  Serial.print("│  CAPTURING: ");
  Serial.print(resName);
  // Add padding to align the closing bracket
  for(int i = resName.length(); i < 23; i++) Serial.print(" ");
  Serial.println("│");
  Serial.println("└─────────────────────────────────────┘");
  
  // Record the current time to measure capture duration
  unsigned long startTime = millis();
  
  // STEP 1: CAPTURE IMAGE FROM CAMERA
  // Send command to camera to capture a JPEG image at specified resolution
  CamStatus status = myCAM.takePicture(resolution, CAM_IMAGE_PIX_FMT_JPG);
  
  // Check if capture was successful
  if (status != CAM_ERR_SUCCESS) {
    Serial.println("✗ Capture FAILED!");
    Serial.print("Error code: ");
    Serial.println(status);
    Serial.println();
    return;  // Exit function early - no image to display
  }
  
  // Wait for camera processing to complete
  delay(100);
  
  // Calculate how long the capture took
  unsigned long captureTime = millis() - startTime;
  
  // Get the size of the captured image in bytes
  uint32_t imageSize = myCAM.getTotalLength();
  
  // STEP 2: DISPLAY CAPTURE STATISTICS
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
  Serial.println(imageSize / 1024.0, 2);  // Display with 2 decimal places
  
  // Handle case where camera returns size 0 (fallback to estimate)
  // QVGA ~15KB, VGA ~50KB, 1080p ~150KB, 3MP ~200KB
  if (imageSize == 0) {
    imageSize = 100000;  // Use 100KB as conservative estimate
    Serial.println("⚠ Using estimated size (getTotalLength = 0)");
  }
  
  Serial.println();
  
  // STEP 3: STREAM IMAGE DATA TO SERIAL PORT
  // Send markers and size info that external software can parse
  Serial.print("SIZE:");
  Serial.println(imageSize);
  Serial.println("START_BINARY_DATA");
  
  // Small delay to ensure all setup messages have been sent
  delay(100);
  
  // STEP 4: READ AND TRANSMIT IMAGE BYTES
  uint32_t bytesRead = 0;           // Track bytes read from camera
  uint8_t prevByte = 0;             // Used to detect JPEG end marker
  bool foundEnd = false;            // Flag for finding JPEG end
  
  // Read from camera buffer and write directly to serial
  while (bytesRead < imageSize) {
    // Read one byte from camera's image buffer
    uint8_t byte = myCAM.readByte();
    
    // Write byte directly to serial port (for external capture/storage)
    Serial.write(byte);
    bytesRead++;
    
    // Check for JPEG format end marker: 0xFF 0xD9
    // This indicates the end of valid image data
    if (prevByte == 0xFF && byte == 0xD9) {
      foundEnd = true;
      break;  // Stop reading after found JPEG end
    }
    
    prevByte = byte;  // Save for next iteration
    
    // Safety check - prevent infinite loop if image data is corrupted
    // Stop if we've read way more than expected (200KB max)
    if (bytesRead > 200000) {
      break;
    }
  }
  
  // STEP 5: SEND END-OF-DATA MARKERS
  Serial.println();              // Line break after binary data
  Serial.println("END_IMAGE_DATA");
  Serial.println("─────────────────────────────────────");
  Serial.println();
}
