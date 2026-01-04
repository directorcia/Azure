/*
 * ARDUCAM MEGA 3MP CAMERA - IMAGE CAPTURE SKETCH
 * 
 * This sketch demonstrates how to:
 * 1. Initialize the Arducam Mega 3MP camera module via SPI
 * 2. Capture images at various resolutions (QVGA, VGA, FHD, 3MP)
 * 3. Adjust image quality settings (HIGH, MEDIUM, LOW)
 * 4. Stream captured image data over serial connection
 * 5. Display capture statistics (time, file size)
 * 
 * Hardware:
 * - Arduino Leonardo (or compatible with ICSP pins)
 * - Arducam Mega 3MP module
 * - SPI connection via ICSP pins
 */

#include <Arduino.h>
#include <Arducam_Mega.h>
#include <SPI.h>

// Function declarations
void captureImage(CAM_IMAGE_MODE resolution);

/*
 * CAMERA CONFIGURATION
 * Arducam Mega camera object with SPI communication
 * Using default SPI pins for Arduino Leonardo:
 * MOSI: Digital 16 (ICSP-4) - Data from master (Arduino) to slave (Camera)
 * MISO: Digital 14 (ICSP-1) - Data from slave (Camera) to master (Arduino)
 * SCK:  Digital 15 (ICSP-3) - Clock signal for synchronization
 * CS:   Digital 10 - Chip Select pin (active LOW) to select the camera
 */
const int CS_PIN = 10;
Arducam_Mega myCAM(CS_PIN); // Create camera object with CS pin
void setup() {
  /*
   * INITIALIZATION PHASE
   * This function runs once when the Arduino starts up.
   * It configures serial communication and initializes the camera hardware.
   */

  // Initialize serial communication at 115200 baud rate
  // This allows communication with a serial monitor or host computer
  Serial.begin(115200);
  
  // Wait for the serial port to be ready
  // This is particularly important for Arduino Leonardo boards which have built-in USB
  while (!Serial) {
    ; // Wait for serial port to connect (needed for Leonardo)
  }
  
  // Display welcome message with ASCII art border
  Serial.println("========================================");
  Serial.println("  Arducam Mega 3MP Camera - Live Test  ");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Initializing camera...");

  // Initialize SPI (Serial Peripheral Interface) communication
  // This sets up the protocol for communicating with the camera over the ICSP pins
  SPI.begin();
  
  // Initialize the camera hardware and perform startup checks
  // This includes verifying camera communication and setting default configurations
  myCAM.begin();
  
  // Display success message
  Serial.println("SUCCESS! Camera initialized!");
  Serial.println();
  Serial.println("========================================");
  Serial.println("Camera is ready!");
  Serial.println("========================================");
  Serial.println();

  // Display user commands and their descriptions
  // Users can send these characters via serial monitor to control the camera
  Serial.println("Commands:");
  Serial.println("  'c' - Capture image (VGA)");
  Serial.println("  '1' - Capture QVGA (320x240)");
  Serial.println("  '2' - Capture VGA (640x480)");
  Serial.println("  '3' - Capture 1080p (1920x1080)");
  Serial.println("  '4' - Capture 3MP (2048x1536)");
  Serial.println("  'q' - Set quality HIGH");
  Serial.println("  'w' - Set quality MEDIUM");
  Serial.println("  'e' - Set quality LOW");
  Serial.println();
}
void loop() {
  /*
   * MAIN LOOP
   * This function runs continuously after setup() completes.
   * It listens for serial commands and processes them to control the camera.
   */

  // Check if any data has been received from the serial port
  if (Serial.available() > 0) {
    // Read a single character from the serial buffer
    char cmd = Serial.read();
    
    // Use a switch statement to handle different commands
    switch(cmd) {
      // Capture image at VGA resolution (640x480)
      case 'c':
      case 'C':
        captureImage(CAM_IMAGE_MODE_VGA);
        break;

      // Capture image at QVGA resolution (320x240) - Quarter VGA, lowest resolution
      case '1':
        captureImage(CAM_IMAGE_MODE_QVGA);
        break;

      // Capture image at VGA resolution (640x480) - Standard resolution
      case '2':
        captureImage(CAM_IMAGE_MODE_VGA);
        break;

      // Capture image at 1080p resolution (1920x1080) - Full HD
      case '3':
        captureImage(CAM_IMAGE_MODE_FHD);
        break;

      // Capture image at 3MP resolution (2048x1536) - Maximum resolution for this camera
      case '4':
        captureImage(CAM_IMAGE_MODE_QXGA);
        break;

      // Set image quality to HIGH (largest file size, best quality)
      case 'q':
      case 'Q':
        myCAM.setImageQuality(HIGH_QUALITY);
        Serial.println("✓ Quality: HIGH");
        break;

      // Set image quality to DEFAULT/MEDIUM (medium file size and quality)
      case 'w':
      case 'W':
        myCAM.setImageQuality(DEFAULT_QUALITY);
        Serial.println("✓ Quality: DEFAULT");
        break;

      // Set image quality to LOW (smallest file size, lower quality)
      case 'e':
      case 'E':
        myCAM.setImageQuality(LOW_QUALITY);
        Serial.println("✓ Quality: LOW");
        break;
    }
  }
  
  // Small delay to reduce CPU usage and prevent overwhelming the serial buffer
  // Allows time for other processes and prevents the loop from running too fast
  delay(10);
}
void captureImage(CAM_IMAGE_MODE resolution) {
  /*
   * CAPTURE IMAGE FUNCTION
   * 
   * This function handles the complete image capture workflow:
   * 1. Determines the resolution name for display purposes
   * 2. Initiates the camera capture process
   * 3. Monitors capture progress and timing
   * 4. Retrieves the captured image data
   * 5. Streams the image bytes over serial connection
   * 6. Provides feedback on capture success/failure
   * 
   * Parameters:
   *   resolution - The desired image resolution (QVGA, VGA, FHD, or QXGA)
   */

  // Convert resolution enum to human-readable string for display
  String resName;
  switch(resolution) {
    case CAM_IMAGE_MODE_QVGA:  resName = "QVGA (320x240)"; break;
    case CAM_IMAGE_MODE_VGA:   resName = "VGA (640x480)"; break;
    case CAM_IMAGE_MODE_FHD:   resName = "1080p (1920x1080)"; break;
    case CAM_IMAGE_MODE_QXGA:  resName = "3MP (2048x1536)"; break;
    default: resName = "Unknown"; break;
  }
  
  // Display capture header with ASCII art formatting
  Serial.println("┌─────────────────────────────────────┐");
  Serial.print("│  CAPTURING: ");
  Serial.print(resName);
  // Pad the resolution name with spaces for alignment
  for(int i = resName.length(); i < 23; i++) Serial.print(" ");
  Serial.println("│");
  Serial.println("└─────────────────────────────────────┘");
  
  // Record the start time to measure capture duration
  unsigned long startTime = millis();
  
  /*
   * CAPTURE PHASE
   * Take a picture using the specified resolution in JPEG format
   * JPEG is used because it provides good compression while maintaining quality
   * CAM_IMAGE_PIX_FMT_JPG = JPEG compression format for efficient file size
   */
  CamStatus status = myCAM.takePicture(resolution, CAM_IMAGE_PIX_FMT_JPG);
  
  // Check if the capture was successful
  if (status != CAM_ERR_SUCCESS) {
    // Capture failed - display error information
    Serial.println("✗ Capture FAILED!");
    Serial.print("Error code: ");
    Serial.println(status);
    Serial.println();
    return; // Exit the function early on error
  }
  
  // Wait briefly to ensure capture process is fully complete
  delay(100);
  
  // Calculate how long the capture took
  unsigned long captureTime = millis() - startTime;
  
  // Retrieve the total size of the captured image data in bytes
  uint32_t imageSize = myCAM.getTotalLength();
  
  // Display capture statistics
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
  
  /*
   * FALLBACK SIZE HANDLING
   * If getTotalLength() returns 0 (which can happen with some camera firmware versions),
   * use an estimated size. This ensures we still attempt to stream the image data.
   * For VGA, a typical JPEG size is 30-50KB, so we estimate 100KB as safe maximum.
   */
  if (imageSize == 0) {
    imageSize = 100000;
    Serial.println("⚠ Using estimated size (getTotalLength = 0)");
  }
  
  Serial.println();
  
  /*
   * TRANSMISSION HEADER
   * Send metadata before the binary image data to help the receiving system
   * understand what to expect:
   * - SIZE: tells recipient how many bytes to expect
   * - START_BINARY_DATA: marks the beginning of binary image data
   */
  Serial.print("SIZE:");
  Serial.println(imageSize);
  Serial.println("START_BINARY_DATA");
  
  // Give the serial buffer time to flush all header data before sending binary
  delay(100);
  
  /*
   * IMAGE DATA STREAMING PHASE
   * Stream the image bytes one at a time over the serial connection
   * The camera stores the image internally, and we read it byte-by-byte
   */
  uint32_t bytesRead = 0;    // Counter for bytes read so far
  uint8_t prevByte = 0;      // Store previous byte for JPEG end marker detection
  bool foundEnd = false;     // Flag indicating if JPEG end marker was found
  
  // Read and transmit image data until we've sent all bytes
  while (bytesRead < imageSize) {
    // Read one byte from the camera's internal buffer
    uint8_t byte = myCAM.readByte();
    
    // Send this byte directly to serial (binary data, not ASCII)
    Serial.write(byte);
    
    bytesRead++;
    
    /*
     * JPEG END MARKER DETECTION
     * JPEG files always end with the bytes 0xFF 0xD9
     * If we detect this sequence, we know we've reached the end of the image
     * and can stop transmission early (optimization to avoid reading garbage data)
     */
    if (prevByte == 0xFF && byte == 0xD9) {
      foundEnd = true;
      break; // Stop reading - we've found the end of the JPEG file
    }
    
    // Keep track of current byte for next iteration's end-marker check
    prevByte = byte;
    
    /*
     * SAFETY CHECK
     * If we've read way more data than expected (200KB max), stop immediately
     * This prevents infinite loops or reading excessive amounts of garbage data
     * if something goes wrong with the camera or data retrieval
     */
    if (bytesRead > 200000) {
      break;
    }
  }
  
  // Send transmission footer to mark the end of image data
  Serial.println();
  Serial.println("END_IMAGE_DATA");
  Serial.println("─────────────────────────────────────");
  Serial.println();
}
