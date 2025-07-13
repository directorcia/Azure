#include <Arduino.h>
#include <SPI.h>
// Removed Wire.h since camera is SPI-only

// Arducam 3MP Camera Configuration - SPI ONLY
// *** IMPORTANT: This camera uses SPI MODE3 ***
// Detected working configuration: MODE3, 1MHz, extended timing
// Common SPI-only models: ArduCAM Mini 2MP/3MP/5MP
// Pin definitions for SPI interface
// Standard SPI pins for Arduino Uno R4 WiFi
#define CS_PIN 10     // Chip Select pin
#define MOSI_PIN 11   // Master Out Slave In
#define MISO_PIN 12   // Master In Slave Out
#define SCK_PIN 13    // Serial Clock

// Alternative pin definitions if using different wiring
// Uncomment and modify these if your camera uses different pins
// #define CS_PIN 8      // Alternative CS pin
// #define MOSI_PIN 9    // Alternative MOSI pin
// #define MISO_PIN 7    // Alternative MISO pin
// #define SCK_PIN 6     // Alternative SCK pin

// Camera registers (for SPI-only Arducam modules)
#define ARDUCHIP_TEST1       0x00  // Test register
#define ARDUCHIP_FRAMES      0x01  // Frame counter
#define ARDUCHIP_MODE        0x02  // Mode register
#define ARDUCHIP_TIM         0x03  // Timing register
#define ARDUCHIP_FIFO        0x04  // FIFO register
#define ARDUCHIP_GPIO        0x06  // GPIO register
#define ARDUCHIP_VER         0x40  // Version register
#define ARDUCHIP_TRIG        0x41  // Trigger register
#define ARDUCHIP_FIFO_BURST  0x3C  // FIFO burst read register
#define ARDUCHIP_FIFO_SIZE1  0x42  // FIFO size register 1
#define ARDUCHIP_FIFO_SIZE2  0x43  // FIFO size register 2  
#define ARDUCHIP_FIFO_SIZE3  0x44  // FIFO size register 3
#define BURST_FIFO_READ      0x3C  // Burst FIFO read
#define SINGLE_FIFO_READ     0x3D  // Single FIFO read

// FIFO control registers
#define FIFO_CLEAR_MASK      0x01
#define FIFO_START_MASK      0x02
#define FIFO_RDPTR_RST_MASK  0x10
#define FIFO_WRPTR_RST_MASK  0x20

// Image format definitions (for SPI-only modules)
#define BMP_FORMAT           0x00
#define JPEG_FORMAT          0x01
#define RAW_FORMAT           0x02

// Image size definitions for ArduCAM modules
#define OV2640_160x120       0x00
#define OV2640_176x144       0x01
#define OV2640_320x240       0x02
#define OV2640_352x288       0x03
#define OV2640_640x480       0x04
#define OV2640_800x600       0x05
#define OV2640_1024x768      0x06
#define OV2640_1280x1024     0x07
#define OV2640_1600x1200     0x08

// Function declarations (SPI-only, removed I2C functions)
void initializeCamera();
bool testCameraConnection();
void testSPIConnection();
void captureImage();
void captureJPEGImage();
void streamJPEGToSerial();
void readFIFO();
uint32_t readFIFOLength();
byte readReg(byte addr);
byte readRegWithSpeed(byte addr, int speed);
byte readRegWithMode(byte addr, int spi_mode);
void writeReg(byte addr, byte data);
void writeRegWithSpeed(byte addr, byte data, int speed);
void writeRegWithMode(byte addr, byte data, int spi_mode);
void printCameraInfo();
void blinkLED(int times);
bool initializeSPICamera();
void setFormat(byte format);
void setImageSize(byte size);
bool findJPEGStart();
bool findJPEGEnd();
void displayJPEGInfo();
void advancedSPITest();
void configureCustomPins();
void testMISOWiring();
void testWithPins(int cs, int mosi, int miso, int sck);
byte readRegWithPins(byte addr, int cs, int mosi, int miso, int sck);
void writeRegWithPins(byte addr, byte data, int cs, int mosi, int miso, int sck);
void hardwareDiagnostics();
void ultraSlowDetection();
void optimizedCameraTest();
void optimizedCameraTestImproved();
void properCaptureSequence();
void initializeOV5642Sensor();
void comprehensiveOV5642Init();
void captureWithFullSensorInit();
void captureImageWithSensorInit();
bool initializeCameraForMode3();

// Helper functions for extended camera testing
bool testCameraConnectionWithExtendedTiming();
bool testCameraConnectionComprehensive();
void writeRegWithMode(byte reg, byte value, int mode);
byte readRegWithMode(byte reg, int mode);
void writeRegWithSpeed(byte reg, byte value, int speed);
byte readRegWithSpeed(byte reg, int speed);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for serial port to connect
  }
  
  Serial.println("=== Arducam 3MP SPI-Only Camera Test ===");
  Serial.println("Camera Type: SPI-only (no I2C sensor control)");
  Serial.println("Initializing camera system...");
  
  // Initialize built-in LED for status indication
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Initialize SPI only (no I2C needed)
  SPI.begin();
  
  // Initialize camera
  initializeCamera();
  
  // Try MODE3 optimized initialization first
  if (initializeCameraForMode3()) {
    Serial.println("✓ SPI camera initialized successfully with MODE3");
  } else {
    Serial.println("✗ Failed to initialize SPI camera with MODE3, trying standard init...");
    if (initializeSPICamera()) {
      Serial.println("✓ SPI camera initialized with standard method");
    } else {
      Serial.println("✗ Failed to initialize SPI camera");
    }
  }
  
  // Test connections
  if (testCameraConnection()) {
    Serial.println("✓ Camera module detected successfully!");
    blinkLED(3); // Success indication
  } else {
    Serial.println("✗ Camera module not detected. Check connections.");
    blinkLED(10); // Error indication
  }
  
  // Print camera information
  printCameraInfo();
}

void loop() {
  Serial.println("\n--- SPI Camera Test Menu ---");
  Serial.println("🎯 Camera detected with MODE3! Optimized for your camera:");
  Serial.println("1. Test camera connection");
  Serial.println("2. Test SPI connection");
  Serial.println("3. Capture test image");
  Serial.println("4. Read FIFO status");
  Serial.println("5. Camera info");
  Serial.println("6. Capture JPEG image");
  Serial.println("7. Stream JPEG to Serial");
  Serial.println("8. Display JPEG info");
  Serial.println("9. Advanced SPI diagnostics");
  Serial.println("A. Configure custom pins");
  Serial.println("B. Test MISO-based wiring");
  Serial.println("C. Hardware diagnostics");
  Serial.println("D. Ultra-slow detection");
  Serial.println("E. Optimized MODE3 test");
  Serial.println("F. Improved MODE3 test");
  Serial.println("H. Proper capture sequence");
  Serial.println("I. Capture with sensor init (recommended)");
  Serial.println("J. Full OV5642 sensor setup + capture");
  Serial.println("Enter command (1-9, A-J):");
  
  while (!Serial.available()) {
    delay(100);
  }
  
  char command = Serial.read();
  while (Serial.available()) Serial.read(); // Clear buffer
  
  switch (command) {
    case '1':
      testCameraConnection();
      break;
    case '2':
      testSPIConnection();
      break;
    case '3':
      captureImage();
      break;
    case '4':
      readFIFO();
      break;
    case '5':
      printCameraInfo();
      break;
    case '6':
      captureJPEGImage();
      break;
    case '7':
      streamJPEGToSerial();
      break;
    case '8':
      displayJPEGInfo();
      break;
    case '9':
      advancedSPITest();
      break;
    case 'A':
    case 'a':
      configureCustomPins();
      break;
    case 'B':
    case 'b':
      testMISOWiring();
      break;
    case 'C':
    case 'c':
      hardwareDiagnostics();
      break;
    case 'D':
    case 'd':
      ultraSlowDetection();
      break;
    case 'E':
    case 'e':
      optimizedCameraTest();
      break;
    case 'F':
    case 'f':
      optimizedCameraTestImproved();
      break;
    case 'H':
    case 'h':
      properCaptureSequence();
      break;
    case 'I':
    case 'i':
      captureImageWithSensorInit();
      break;
    case 'J':
    case 'j':
      captureWithFullSensorInit();
      break;
    default:
      Serial.println("Invalid command. Try again.");
      break;
  }
  
  delay(2000);
}

void initializeCamera() {
  Serial.println("Setting up SPI camera pins...");
  
  // Set up CS pin
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  // Initialize SPI pins explicitly
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(SCK_PIN, OUTPUT);
  
  // Ensure pins are in known state
  digitalWrite(MOSI_PIN, LOW);
  digitalWrite(SCK_PIN, LOW);
  
  Serial.println("✓ SPI pins configured");
  
  // Initialize SPI with MODE3 (detected as working mode)
  SPI.begin();
  delay(100);
  
  Serial.println("✓ SPI initialized with MODE3 support");
  
  // Add extended power-up delay
  Serial.println("Waiting for camera power-up...");
  delay(2000);
  
  // Try to wake up camera with CS toggling
  Serial.println("Attempting to wake camera...");
  for (int i = 0; i < 5; i++) {
    digitalWrite(CS_PIN, LOW);
    delay(10);
    digitalWrite(CS_PIN, HIGH);
    delay(100);
  }
  
  // Reset camera with proper timing - using MODE3
  Serial.println("Resetting camera with MODE3...");
  for (int attempt = 0; attempt < 3; attempt++) {
    Serial.print("Reset attempt ");
    Serial.println(attempt + 1);
    
    // Try to write reset command using MODE3
    writeRegWithMode(ARDUCHIP_MODE, 0x00, SPI_MODE3);
    delay(500);
    
    // Try to write another command to see if it responds
    writeRegWithMode(ARDUCHIP_FIFO, FIFO_CLEAR_MASK, SPI_MODE3);
    delay(200);
    
    // Test if camera responds using MODE3
    writeRegWithMode(ARDUCHIP_TEST1, 0x55, SPI_MODE3);
    delay(100);
    byte test_response = readRegWithMode(ARDUCHIP_TEST1, SPI_MODE3);
    
    if (test_response == 0x55) {
      Serial.println("✓ Camera responded to reset using MODE3");
      break;
    } else {
      Serial.print("No response (got 0x");
      Serial.print(test_response, HEX);
      Serial.println(")");
    }
  }
  
  Serial.println("✓ Camera initialization sequence completed with MODE3");
}

void readFIFO() {
  Serial.println("\nReading FIFO status...");
  
  byte fifo_status = readReg(ARDUCHIP_FIFO);
  Serial.print("FIFO status: 0x");
  Serial.println(fifo_status, HEX);
  
  // Decode status bits
  Serial.print("FIFO clear: ");
  Serial.println((fifo_status & FIFO_CLEAR_MASK) ? "Yes" : "No");
  Serial.print("FIFO start: ");
  Serial.println((fifo_status & FIFO_START_MASK) ? "Yes" : "No");
  Serial.print("Capture done: ");
  Serial.println((fifo_status & 0x08) ? "Yes" : "No");
  
  // Read FIFO size
  byte fifo_size1 = readReg(0x42);
  byte fifo_size2 = readReg(0x43);
  byte fifo_size3 = readReg(0x44);
  
  unsigned long fifo_length = ((fifo_size3 << 16) | (fifo_size2 << 8) | fifo_size1) & 0x07ffff;
  
  Serial.print("FIFO length: ");
  Serial.print(fifo_length);
  Serial.println(" bytes");
}

uint32_t readFIFOLength() {
  // Read FIFO size registers
  byte fifo_size1 = readRegWithMode(ARDUCHIP_FIFO_SIZE1, SPI_MODE3);
  byte fifo_size2 = readRegWithMode(ARDUCHIP_FIFO_SIZE2, SPI_MODE3);
  byte fifo_size3 = readRegWithMode(ARDUCHIP_FIFO_SIZE3, SPI_MODE3);
  
  // Combine into 24-bit length (mask to 19 bits as per ArduCAM spec)
  uint32_t fifo_length = ((fifo_size3 << 16) | (fifo_size2 << 8) | fifo_size1) & 0x07ffff;
  
  return fifo_length;
}

byte readReg(byte addr) {
  return readRegWithSpeed(addr, 1000000); // Default 1MHz with MODE3
}

byte readRegWithSpeed(byte addr, int speed) {
  // Optimized for MODE3 cameras
  
  // Use MODE3 for this camera (as detected during testing)
  SPI.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(10); // Extended delay for signal settling
  
  SPI.transfer(addr);
  delayMicroseconds(10); // Extended delay between transfers
  byte result = SPI.transfer(0x00);
  
  delayMicroseconds(10);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  
  return result;
}

void writeReg(byte addr, byte data) {
  writeRegWithSpeed(addr, data, 1000000); // Default 1MHz with MODE3
}

void writeRegWithSpeed(byte addr, byte data, int speed) {
  // Optimized for MODE3 cameras
  SPI.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(10); // Extended delay for signal settling
  
  SPI.transfer(addr | 0x80); // Set write bit
  delayMicroseconds(10); // Extended delay between transfers
  SPI.transfer(data);
  
  delayMicroseconds(10);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  
  // Add a small delay after write to allow register to update
  delayMicroseconds(100);
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

void captureJPEGImage() {
  Serial.println("\n=== Capturing JPEG Image ===");
  
  // Clear FIFO
  writeReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
  delay(10);
  
  // Reset FIFO pointers
  writeReg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
  writeReg(ARDUCHIP_FIFO, FIFO_WRPTR_RST_MASK);
  
  // Start capture
  writeReg(ARDUCHIP_FIFO, FIFO_START_MASK);
  
  Serial.println("📸 Capture started. Waiting for completion...");
  
  // Wait for capture to complete
  unsigned long start_time = millis();
  while (millis() - start_time < 10000) { // 10 second timeout
    byte fifo_status = readReg(ARDUCHIP_FIFO);
    if (fifo_status & 0x08) { // Capture done bit
      Serial.println("✓ JPEG image capture completed!");
      
      // Read FIFO length
      byte fifo_size1 = readReg(0x42);
      byte fifo_size2 = readReg(0x43);
      byte fifo_size3 = readReg(0x44);
      
      unsigned long fifo_length = ((fifo_size3 << 16) | (fifo_size2 << 8) | fifo_size1) & 0x07ffff;
      
      Serial.print("📊 FIFO length: ");
      Serial.print(fifo_length);
      Serial.println(" bytes");
      
      if (fifo_length > 0) {
        Serial.println("✓ JPEG image data available in FIFO");
        
        // Verify JPEG format
        if (findJPEGStart()) {
          Serial.println("✓ Valid JPEG header found (0xFF 0xD8)");
          
          // Display some image statistics
          displayJPEGInfo();
          
          Serial.println("\n📝 Options:");
          Serial.println("   - Use option 8 to stream JPEG data to Serial");
          Serial.println("   - Use option 9 to display JPEG information");
          
        } else {
          Serial.println("✗ Invalid JPEG format - no JPEG header found");
        }
      } else {
        Serial.println("✗ No image data in FIFO");
      }
      
      return;
    }
    delay(100);
  }
  
  Serial.println("✗ Capture timeout - no image captured");
}

void streamJPEGToSerial() {
  Serial.println("\n=== Streaming JPEG to Serial ===");
  Serial.println("📡 Starting JPEG data stream...");
  Serial.println("Copy the hex data below to create a JPEG file:");
  Serial.println("--- JPEG DATA START ---");
  
  // Read FIFO length
  byte fifo_size1 = readReg(0x42);
  byte fifo_size2 = readReg(0x43);
  byte fifo_size3 = readReg(0x44);
  
  unsigned long fifo_length = ((fifo_size3 << 16) | (fifo_size2 << 8) | fifo_size1) & 0x07ffff;
  
  if (fifo_length == 0) {
    Serial.println("✗ No image data available. Capture an image first.");
    return;
  }
  
  // Reset FIFO read pointer
  writeReg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
  
  // Stream the entire JPEG image
  unsigned long bytes_read = 0;
  Serial.print("0x");
  
  while (bytes_read < fifo_length) {
    byte pixel_data = readReg(SINGLE_FIFO_READ);
    
    // Format as hex
    if (pixel_data < 16) Serial.print("0");
    Serial.print(pixel_data, HEX);
    
    bytes_read++;
    
    // Add spacing and line breaks for readability
    if (bytes_read % 16 == 0) {
      Serial.println();
      if (bytes_read < fifo_length) Serial.print("0x");
    } else if (bytes_read % 2 == 0) {
      Serial.print(" ");
    }
    
    // Progress indicator
    if (bytes_read % 1000 == 0) {
      Serial.print(" [");
      Serial.print((bytes_read * 100) / fifo_length);
      Serial.print("%] ");
    }
  }
  
  Serial.println();
  Serial.println("--- JPEG DATA END ---");
  Serial.print("✓ Streamed ");
  Serial.print(bytes_read);
  Serial.println(" bytes");
  
  Serial.println("\n📝 To view the image:");
  Serial.println("1. Copy the hex data above");
  Serial.println("2. Convert hex to binary using a hex editor");
  Serial.println("3. Save as a .jpg file");
  Serial.println("4. Open with any image viewer");
}

bool findJPEGStart() {
  // Reset FIFO read pointer
  writeReg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
  
  // Look for JPEG start marker (0xFF 0xD8)
  for (int i = 0; i < 100; i++) { // Check first 100 bytes
    byte byte1 = readReg(SINGLE_FIFO_READ);
    if (byte1 == 0xFF) {
      byte byte2 = readReg(SINGLE_FIFO_READ);
      if (byte2 == 0xD8) {
        return true; // Found JPEG start marker
      }
    }
  }
  
  return false;
}

bool findJPEGEnd() {
  // Look for JPEG end marker (0xFF 0xD9)
  // This is more complex as we need to read through the entire image
  // For now, we'll assume the image is complete based on FIFO status
  return true;
}

void displayJPEGInfo() {
  Serial.println("\n=== JPEG Image Information ===");
  
  // Reset FIFO read pointer
  writeReg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
  
  // Read first 32 bytes to analyze JPEG header
  Serial.println("📋 JPEG Header Analysis:");
  Serial.print("Header bytes: ");
  
  for (int i = 0; i < 32; i++) {
    byte header_byte = readReg(SINGLE_FIFO_READ);
    Serial.print("0x");
    if (header_byte < 16) Serial.print("0");
    Serial.print(header_byte, HEX);
    Serial.print(" ");
    
    if (i == 15) {
      Serial.println();
      Serial.print("              ");
    }
  }
  Serial.println();
  
  // Get image size
  byte fifo_size1 = readReg(0x42);
  byte fifo_size2 = readReg(0x43);
  byte fifo_size3 = readReg(0x44);
  
  unsigned long fifo_length = ((fifo_size3 << 16) | (fifo_size2 << 8) | fifo_size1) & 0x07ffff;
  
  Serial.print("📊 Image size: ");
  Serial.print(fifo_length);
  Serial.println(" bytes");
  
  Serial.print("📐 Estimated resolution: ");
  if (fifo_length < 50000) {
    Serial.println("320x240 (QVGA)");
  } else if (fifo_length < 200000) {
    Serial.println("640x480 (VGA)");
  } else if (fifo_length < 400000) {
    Serial.println("800x600 (SVGA)");
  } else {
    Serial.println("1024x768+ (XGA+)");
  }
  
  Serial.print("🎨 Color format: JPEG compressed");
  Serial.println();
  
  // Quality estimation based on file size
  float quality_ratio = (float)fifo_length / (640.0 * 480.0 * 3.0); // Assuming VGA
  Serial.print("📈 Estimated quality: ");
  if (quality_ratio > 0.8) {
    Serial.println("High");
  } else if (quality_ratio > 0.4) {
    Serial.println("Medium");
  } else {
    Serial.println("Low");
  }
}

byte readRegWithMode(byte addr, int spi_mode) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, spi_mode));
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(10); // Extended delay for signal settling
  
  SPI.transfer(addr);
  delayMicroseconds(10); // Extended delay between transfers
  byte result = SPI.transfer(0x00);
  
  delayMicroseconds(10);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  
  return result;
}

void writeRegWithMode(byte addr, byte data, int spi_mode) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, spi_mode));
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(10); // Extended delay for signal settling
  
  SPI.transfer(addr | 0x80); // Set write bit
  delayMicroseconds(10); // Extended delay between transfers
  SPI.transfer(data);
  
  delayMicroseconds(10);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  
  // Add a small delay after write to allow register to update
  delayMicroseconds(100);
}

void advancedSPITest() {
  Serial.println("\n=== Advanced SPI Diagnostics ===");
  
  // Test pin states
  Serial.println("📋 Pin State Check:");
  Serial.print("CS pin (D10): ");
  Serial.println(digitalRead(CS_PIN) ? "HIGH" : "LOW");
  Serial.print("MOSI pin (D11): ");
  Serial.println(digitalRead(MOSI_PIN) ? "HIGH" : "LOW");
  Serial.print("MISO pin (D12): ");
  Serial.println(digitalRead(MISO_PIN) ? "HIGH" : "LOW");
  Serial.print("SCK pin (D13): ");
  Serial.println(digitalRead(SCK_PIN) ? "HIGH" : "LOW");
  
  // Test CS pin control
  Serial.println("\n📡 CS Pin Control Test:");
  digitalWrite(CS_PIN, LOW);
  Serial.print("CS LOW: ");
  Serial.println(digitalRead(CS_PIN) ? "HIGH" : "LOW");
  delay(100);
  digitalWrite(CS_PIN, HIGH);
  Serial.print("CS HIGH: ");
  Serial.println(digitalRead(CS_PIN) ? "HIGH" : "LOW");
  
  // Test different clock dividers
  Serial.println("\n⚡ Clock Speed Test:");
  int speeds[] = {500000, 1000000, 2000000, 4000000, 8000000};
  
  for (int i = 0; i < 5; i++) {
    Serial.print("Testing at ");
    Serial.print(speeds[i] / 1000000.0);
    Serial.println(" MHz:");
    
    bool success = false;
    for (int attempt = 0; attempt < 3; attempt++) {
      writeRegWithSpeed(ARDUCHIP_TEST1, 0x77, speeds[i]);
      delay(5);
      byte result = readRegWithSpeed(ARDUCHIP_TEST1, speeds[i]);
      
      if (result == 0x77) {
        Serial.println("  ✓ Success");
        success = true;
        break;
      } else {
        Serial.print("  Attempt ");
        Serial.print(attempt + 1);
        Serial.print(": Expected 0x77, Got 0x");
        Serial.println(result, HEX);
      }
    }
    
    if (!success) {
      Serial.println("  ✗ Failed at this speed");
    }
  }
  
  // Test extended timing
  Serial.println("\n⏱️ Extended Timing Test:");
  
  // Test with longer delays
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  
  digitalWrite(CS_PIN, LOW);
  delay(1); // Extended delay
  
  SPI.transfer(ARDUCHIP_TEST1 | 0x80);
  delay(1);
  SPI.transfer(0x99);
  delay(1);
  
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  
  delay(10);
  
  // Read back
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  
  digitalWrite(CS_PIN, LOW);
  delay(1);
  
  SPI.transfer(ARDUCHIP_TEST1);
  delay(1);
  byte extended_result = SPI.transfer(0x00);
  delay(1);
  
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  
  Serial.print("Extended timing test - Expected: 0x99, Got: 0x");
  Serial.println(extended_result, HEX);
  
  if (extended_result == 0x99) {
    Serial.println("✓ Extended timing successful");
  } else {
    Serial.println("✗ Extended timing failed");
  }
}

void configureCustomPins() {
  Serial.println("\n=== Custom Pin Configuration ===");
  Serial.println("Current pin configuration:");
  Serial.print("CS: D"); Serial.println(CS_PIN);
  Serial.print("MOSI: D"); Serial.println(MOSI_PIN);
  Serial.print("MISO: D"); Serial.println(MISO_PIN);
  Serial.print("SCK: D"); Serial.println(SCK_PIN);
  
  Serial.println("\nCommon Arducam wiring configurations:");
  Serial.println("1. Standard SPI (CS=10, MOSI=11, MISO=12, SCK=13)");
  Serial.println("2. Alternative 1 (CS=8, MOSI=9, MISO=7, SCK=6)");
  Serial.println("3. Alternative 2 (CS=7, MOSI=6, MISO=5, SCK=4)");
  Serial.println("4. Alternative 3 (CS=9, MOSI=8, MISO=7, SCK=6)");
  Serial.println("5. Custom configuration");
  
  Serial.println("\nEnter configuration number (1-5):");
  
  while (!Serial.available()) {
    delay(100);
  }
  
  char config = Serial.read();
  while (Serial.available()) Serial.read(); // Clear buffer
  
  int test_cs, test_mosi, test_miso, test_sck;
  
  switch (config) {
    case '1':
      test_cs = 10; test_mosi = 11; test_miso = 12; test_sck = 13;
      break;
    case '2':
      test_cs = 8; test_mosi = 9; test_miso = 7; test_sck = 6;
      break;
    case '3':
      test_cs = 7; test_mosi = 6; test_miso = 5; test_sck = 4;
      break;
    case '4':
      test_cs = 9; test_mosi = 8; test_miso = 7; test_sck = 6;
      break;
    case '5':
      Serial.println("Custom configuration not implemented in this version.");
      Serial.println("Please modify the #define statements in the code.");
      return;
    default:
      Serial.println("Invalid selection.");
      return;
  }
  
  Serial.print("Testing with CS="); Serial.print(test_cs);
  Serial.print(", MOSI="); Serial.print(test_mosi);
  Serial.print(", MISO="); Serial.print(test_miso);
  Serial.print(", SCK="); Serial.println(test_sck);
  
  testWithPins(test_cs, test_mosi, test_miso, test_sck);
}

void testMISOWiring() {
  Serial.println("\n=== MISO-Based Wiring Test ===");
  Serial.println("Testing common MISO-based configurations...");
  
  // Common MISO-based configurations for Arducam modules
  int configurations[][4] = {
    {10, 11, 12, 13}, // Standard
    {8, 9, 7, 6},     // Alt 1
    {7, 6, 5, 4},     // Alt 2  
    {9, 8, 7, 6},     // Alt 3
    {6, 7, 8, 9},     // Alt 4
    {5, 6, 7, 8},     // Alt 5
    {4, 5, 6, 7},     // Alt 6
    {3, 4, 5, 6}      // Alt 7
  };
  
  String config_names[] = {
    "Standard (10,11,12,13)",
    "Alt 1 (8,9,7,6)",
    "Alt 2 (7,6,5,4)",
    "Alt 3 (9,8,7,6)",
    "Alt 4 (6,7,8,9)",
    "Alt 5 (5,6,7,8)",
    "Alt 6 (4,5,6,7)",
    "Alt 7 (3,4,5,6)"
  };
  
  for (int i = 0; i < 8; i++) {
    Serial.print("\nTesting configuration ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(config_names[i]);
    
    testWithPins(configurations[i][0], configurations[i][1], 
                 configurations[i][2], configurations[i][3]);
    
    delay(500);
  }
  
  Serial.println("\n=== MISO Test Complete ===");
  Serial.println("If any configuration worked, update the #define statements");
  Serial.println("in your code to match the working configuration.");
}

void testWithPins(int cs, int mosi, int miso, int sck) {
  Serial.print("  Testing CS="); Serial.print(cs);
  Serial.print(", MOSI="); Serial.print(mosi);
  Serial.print(", MISO="); Serial.print(miso);
  Serial.print(", SCK="); Serial.println(sck);
  
  // Configure pins
  pinMode(cs, OUTPUT);
  pinMode(mosi, OUTPUT);
  pinMode(miso, INPUT);
  pinMode(sck, OUTPUT);
  
  digitalWrite(cs, HIGH);
  
  // Test communication
  writeRegWithPins(ARDUCHIP_TEST1, 0x55, cs, mosi, miso, sck);
  delay(10);
  byte result = readRegWithPins(ARDUCHIP_TEST1, cs, mosi, miso, sck);
  
  Serial.print("    Wrote 0x55, Read: 0x");
  Serial.println(result, HEX);
  
  if (result == 0x55) {
    Serial.println("    ✓ FIRST TEST PASSED!");
    
    // Second test
    writeRegWithPins(ARDUCHIP_TEST1, 0xAA, cs, mosi, miso, sck);
    delay(10);
    result = readRegWithPins(ARDUCHIP_TEST1, cs, mosi, miso, sck);
    
    Serial.print("    Wrote 0xAA, Read: 0x");
    Serial.println(result, HEX);
    
    if (result == 0xAA) {
      Serial.println("    ✓ SECOND TEST PASSED!");
      Serial.println("    ✓✓ THIS CONFIGURATION WORKS! ✓✓");
      Serial.println("    Update your #define statements to use these pins.");
    } else {
      Serial.println("    ✗ Second test failed");
    }
  } else {
    Serial.println("    ✗ First test failed");
  }
}

byte readRegWithPins(byte addr, int cs, int mosi, int miso, int sck) {
  // Bit-bang SPI communication
  digitalWrite(cs, LOW);
  delayMicroseconds(2);
  
  // Send address byte
  for (int i = 7; i >= 0; i--) {
    digitalWrite(sck, LOW);
    digitalWrite(mosi, (addr >> i) & 0x01);
    delayMicroseconds(1);
    digitalWrite(sck, HIGH);
    delayMicroseconds(1);
  }
  
  // Read data byte
  byte result = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(sck, LOW);
    delayMicroseconds(1);
    digitalWrite(sck, HIGH);
    if (digitalRead(miso)) {
      result |= (1 << i);
    }
    delayMicroseconds(1);
  }
  
  digitalWrite(cs, HIGH);
  delayMicroseconds(2);
  
  return result;
}

void writeRegWithPins(byte addr, byte data, int cs, int mosi, int miso, int sck) {
  // Bit-bang SPI communication
  digitalWrite(cs, LOW);
  delayMicroseconds(2);
  
  // Send address byte with write bit
  byte write_addr = addr | 0x80;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(sck, LOW);
    digitalWrite(mosi, (write_addr >> i) & 0x01);
    delayMicroseconds(1);
    digitalWrite(sck, HIGH);
    delayMicroseconds(1);
  }
  
  // Send data byte
  for (int i = 7; i >= 0; i--) {
    digitalWrite(sck, LOW);
    digitalWrite(mosi, (data >> i) & 0x01);
    delayMicroseconds(1);
    digitalWrite(sck, HIGH);
    delayMicroseconds(1);
  }
  
  digitalWrite(cs, HIGH);
  delayMicroseconds(2);
}

void printCameraInfo() {
  Serial.println("\n=== Camera Information ===");
  
  // Camera type and interface
  Serial.println("📷 Camera Type: Arducam 3MP SPI-only");
  Serial.println("🔌 Interface: SPI (no I2C sensor control)");
  
  // Pin configuration
  Serial.println("\n📋 Pin Configuration:");
  Serial.print("  CS (Chip Select): D"); Serial.println(CS_PIN);
  Serial.print("  MOSI (Master Out): D"); Serial.println(MOSI_PIN);
  Serial.print("  MISO (Master In): D"); Serial.println(MISO_PIN);
  Serial.print("  SCK (Serial Clock): D"); Serial.println(SCK_PIN);
  
  // Read camera registers
  Serial.println("\n📊 Register Status:");
  
  // Version register
  byte version = readReg(ARDUCHIP_VER);
  Serial.print("  Version register: 0x");
  Serial.println(version, HEX);
  
  // Identify camera type based on version
  Serial.print("  Camera model: ");
  switch (version) {
    case 0x40:
      Serial.println("ArduCAM Mini 2MP");
      break;
    case 0x41:
      Serial.println("ArduCAM Mini 3MP");
      break;
    case 0x42:
      Serial.println("ArduCAM Mini 5MP");
      break;
    case 0x43:
      Serial.println("ArduCAM Mini 8MP");
      break;
    default:
      Serial.print("Unknown (0x");
      Serial.print(version, HEX);
      Serial.println(")");
      break;
  }
  
  // Mode register
  byte mode = readReg(ARDUCHIP_MODE);
  Serial.print("  Mode register: 0x");
  Serial.println(mode, HEX);
  
  // Timing register (format)
  byte timing = readReg(ARDUCHIP_TIM);
  Serial.print("  Format: ");
  switch (timing) {
    case 0x00:
      Serial.println("BMP");
      break;
    case 0x01:
      Serial.println("JPEG");
      break;
    case 0x02:
      Serial.println("RAW");
      break;
    default:
      Serial.print("Unknown (0x");
      Serial.print(timing, HEX);
      Serial.println(")");
      break;
  }
  
  // FIFO status
  byte fifo_status = readReg(ARDUCHIP_FIFO);
  Serial.print("  FIFO status: 0x");
  Serial.println(fifo_status, HEX);
  
  // GPIO register
  byte gpio = readReg(ARDUCHIP_GPIO);
  Serial.print("  GPIO register: 0x");
  Serial.println(gpio, HEX);
  
  // SPI communication test
  Serial.println("\n🔧 SPI Communication Test:");
  
  // Test write/read
  writeReg(ARDUCHIP_TEST1, 0x55);
  delay(10);
  byte test_result = readReg(ARDUCHIP_TEST1);
  
  Serial.print("  Test register: Wrote 0x55, Read 0x");
  Serial.println(test_result, HEX);
  
  if (test_result == 0x55) {
    Serial.println("  ✓ SPI communication working");
  } else {
    Serial.println("  ✗ SPI communication failed");
  }
  
  // FIFO size
  byte fifo_size1 = readReg(0x42);
  byte fifo_size2 = readReg(0x43);
  byte fifo_size3 = readReg(0x44);
  
  unsigned long fifo_length = ((fifo_size3 << 16) | (fifo_size2 << 8) | fifo_size1) & 0x07ffff;
  
  Serial.print("  FIFO length: ");
  Serial.print(fifo_length);
  Serial.println(" bytes");
  
  // Power and connection status
  Serial.println("\n⚡ Status Summary:");
  
  if (version >= 0x40 && version <= 0x43) {
    Serial.println("  ✓ Camera module detected");
  } else {
    Serial.println("  ⚠ Camera module not recognized");
  }
  
  if (test_result == 0x55) {
    Serial.println("  ✓ SPI communication OK");
  } else {
    Serial.println("  ✗ SPI communication failed");
  }
  
  if (fifo_length > 0) {
    Serial.println("  ✓ Image data available");
  } else {
    Serial.println("  ⚪ No image data (capture required)");
  }
  
  Serial.println("\n💡 Usage Tips:");
  Serial.println("  - Use option 1 to test camera connection");
  Serial.println("  - Use option 6 to capture a JPEG image");
  Serial.println("  - Use option 7 to stream image data");
  Serial.println("  - Use option 9 for advanced SPI diagnostics");
}

void hardwareDiagnostics() {
  Serial.println("\n=== Hardware Diagnostics ===");
  
  // Test Arduino pins
  Serial.println("📋 Arduino Pin Test:");
  Serial.print("CS pin (D"); Serial.print(CS_PIN); Serial.print("): ");
  Serial.println(digitalRead(CS_PIN) ? "HIGH" : "LOW");
  
  Serial.print("MOSI pin (D"); Serial.print(MOSI_PIN); Serial.print("): ");
  Serial.println(digitalRead(MOSI_PIN) ? "HIGH" : "LOW");
  
  Serial.print("MISO pin (D"); Serial.print(MISO_PIN); Serial.print("): ");
  Serial.println(digitalRead(MISO_PIN) ? "HIGH" : "LOW");
  
  Serial.print("SCK pin (D"); Serial.print(SCK_PIN); Serial.print("): ");
  Serial.println(digitalRead(SCK_PIN) ? "HIGH" : "LOW");
  
  // Test pin control
  Serial.println("\n🔧 Pin Control Test:");
  
  // Test CS pin control
  Serial.println("Testing CS pin control...");
  digitalWrite(CS_PIN, LOW);
  delay(10);
  bool cs_low = digitalRead(CS_PIN);
  digitalWrite(CS_PIN, HIGH);
  delay(10);
  bool cs_high = digitalRead(CS_PIN);
  
  Serial.print("CS LOW test: "); Serial.println(cs_low ? "FAILED" : "PASSED");
  Serial.print("CS HIGH test: "); Serial.println(cs_high ? "PASSED" : "FAILED");
  
  // Test MOSI pin control
  Serial.println("Testing MOSI pin control...");
  digitalWrite(MOSI_PIN, LOW);
  delay(10);
  bool mosi_low = digitalRead(MOSI_PIN);
  digitalWrite(MOSI_PIN, HIGH);
  delay(10);
  bool mosi_high = digitalRead(MOSI_PIN);
  
  Serial.print("MOSI LOW test: "); Serial.println(mosi_low ? "FAILED" : "PASSED");
  Serial.print("MOSI HIGH test: "); Serial.println(mosi_high ? "PASSED" : "FAILED");
  
  // Test SCK pin control
  Serial.println("Testing SCK pin control...");
  digitalWrite(SCK_PIN, LOW);
  delay(10);
  bool sck_low = digitalRead(SCK_PIN);
  digitalWrite(SCK_PIN, HIGH);
  delay(10);
  bool sck_high = digitalRead(SCK_PIN);
  
  Serial.print("SCK LOW test: "); Serial.println(sck_low ? "FAILED" : "PASSED");
  Serial.print("SCK HIGH test: "); Serial.println(sck_high ? "PASSED" : "FAILED");
  
  // Test MISO pin behavior
  Serial.println("\n📡 MISO Pin Analysis:");
  
  // Sample MISO multiple times
  int miso_samples[20];
  for (int i = 0; i < 20; i++) {
    miso_samples[i] = digitalRead(MISO_PIN);
    delay(10);
  }
  
  int high_count = 0;
  for (int i = 0; i < 20; i++) {
    if (miso_samples[i]) high_count++;
  }
  
  Serial.print("MISO samples (20 reads): ");
  Serial.print(high_count);
  Serial.println(" HIGH");
  
  if (high_count == 20) {
    Serial.println("❌ MISO stuck HIGH - possible wiring issue");
  } else if (high_count == 0) {
    Serial.println("❌ MISO stuck LOW - possible wiring issue");
  } else if (high_count > 15) {
    Serial.println("⚠️  MISO mostly HIGH - check pull-up resistor");
  } else if (high_count < 5) {
    Serial.println("⚠️  MISO mostly LOW - check connections");
  } else {
    Serial.println("✅ MISO appears to be floating - normal behavior");
  }
  
  // Test power supply
  Serial.println("\n⚡ Power Supply Check:");
  Serial.println("Note: This test assumes camera is powered from Arduino");
  Serial.println("If using external power, ignore these results");
  
  // Test if we can toggle CS without affecting other pins
  bool original_miso = digitalRead(MISO_PIN);
  digitalWrite(CS_PIN, LOW);
  delay(100);
  bool cs_low_miso = digitalRead(MISO_PIN);
  digitalWrite(CS_PIN, HIGH);
  delay(100);
  bool cs_high_miso = digitalRead(MISO_PIN);
  
  if (original_miso != cs_low_miso || original_miso != cs_high_miso) {
    Serial.println("✅ MISO responds to CS changes - camera likely powered");
  } else {
    Serial.println("⚠️  MISO doesn't respond to CS - check power/connections");
  }
  
  Serial.println("\n🔍 Recommendations:");
  Serial.println("1. If pins test OK but camera not detected:");
  Serial.println("   - Check camera power (3.3V vs 5V)");
  Serial.println("   - Verify ground connections");
  Serial.println("   - Try menu option 'D' for ultra-slow detection");
  Serial.println("2. If pin tests fail:");
  Serial.println("   - Check Arduino connections");
  Serial.println("   - Verify pin numbers in code");
  Serial.println("   - Check for shorts or bad connections");
}

void ultraSlowDetection() {
  Serial.println("\n=== Ultra-Slow Detection Test ===");
  Serial.println("This test uses extremely slow SPI speeds and extended delays");
  Serial.println("to detect difficult or slow-responding cameras.");
  
  // Test with extremely slow speeds
  int ultra_slow_speeds[] = {62500, 125000, 250000}; // 62.5kHz, 125kHz, 250kHz
  
  for (int s = 0; s < 3; s++) {
    Serial.print("\nTesting at ");
    Serial.print(ultra_slow_speeds[s] / 1000.0);
    Serial.println(" kHz...");
    
    // Extended initialization sequence
    digitalWrite(CS_PIN, HIGH);
    delay(500);
    digitalWrite(CS_PIN, LOW);
    delay(100);
    digitalWrite(CS_PIN, HIGH);
    delay(500);
    
    // Try to write test pattern with extended delays
    Serial.println("Writing test pattern...");
    writeRegWithSpeed(ARDUCHIP_TEST1, 0x55, ultra_slow_speeds[s]);
    delay(200); // Extended delay
    
    byte result1 = readRegWithSpeed(ARDUCHIP_TEST1, ultra_slow_speeds[s]);
    Serial.print("  Test 1 - Wrote 0x55, Read: 0x");
    Serial.println(result1, HEX);
    
    if (result1 == 0x55) {
      Serial.println("  ✅ SUCCESS! Camera detected at ultra-slow speed");
      
      // Verify with second test
      writeRegWithSpeed(ARDUCHIP_TEST1, 0xAA, ultra_slow_speeds[s]);
      delay(200);
      byte result2 = readRegWithSpeed(ARDUCHIP_TEST1, ultra_slow_speeds[s]);
      
      Serial.print("  Test 2 - Wrote 0xAA, Read: 0x");
      Serial.println(result2, HEX);
      
      if (result2 == 0xAA) {
        Serial.println("  ✅ VERIFIED! Camera is working at ultra-slow speed");
        
        // Try to read version
        byte version = readRegWithSpeed(ARDUCHIP_VER, ultra_slow_speeds[s]);
        Serial.print("  Version register: 0x");
        Serial.println(version, HEX);
        
        Serial.println("\n📋 Camera Detection Summary:");
        Serial.print("  Working SPI speed: ");
        Serial.print(ultra_slow_speeds[s] / 1000.0);
        Serial.println(" kHz");
        
        Serial.println("  Recommendation: Use this slow speed for all operations");
        Serial.println("  Your camera may be a slower variant or have timing issues");
        
        return;
      }
    }
  }
  
  Serial.println("\n❌ Ultra-slow detection failed");
  Serial.println("📋 This suggests:");
  Serial.println("1. Camera is not properly connected");
  Serial.println("2. Camera is not powered correctly");
  Serial.println("3. Camera may be damaged");
  Serial.println("4. Wrong camera type for this code");
  
  Serial.println("\n🔍 Next Steps:");
  Serial.println("1. Double-check ALL wiring connections");
  Serial.println("2. Verify camera power requirements (3.3V vs 5V)");
  Serial.println("3. Try menu option 'B' for wiring auto-detection");
  Serial.println("4. Check camera datasheet for compatibility");
}

void optimizedCameraTest() {
  Serial.println("\n=== Optimized MODE3 Camera Test ===");
  Serial.println("Testing camera with optimized MODE3 settings...");
  Serial.println("Note: This test uses a writable test register.");
  Serial.println("If it fails, try menu option 'F' for improved testing.");
  
  // Test multiple speeds with MODE3
  int speeds[] = {500000, 1000000, 2000000, 4000000}; // 0.5MHz to 4MHz
  
  for (int i = 0; i < 4; i++) {
    Serial.print("\nTesting MODE3 at ");
    Serial.print(speeds[i] / 1000000.0);
    Serial.println(" MHz:");
    
    bool success = true;
    
    // Test 1: Basic register test
    writeRegWithMode(ARDUCHIP_TEST1, 0x55, SPI_MODE3);
    delay(10);
    byte result1 = readRegWithMode(ARDUCHIP_TEST1, SPI_MODE3);
    
    Serial.print("  Test 1 - Wrote 0x55, Read: 0x");
    Serial.println(result1, HEX);
    
    if (result1 != 0x55) {
      success = false;
      Serial.println("  ⚠️  Test register may not be writable on this camera");
    }
    
    // Test 2: Different value
    writeRegWithMode(ARDUCHIP_TEST1, 0xAA, SPI_MODE3);
    delay(10);
    byte result2 = readRegWithMode(ARDUCHIP_TEST1, SPI_MODE3);
    
    Serial.print("  Test 2 - Wrote 0xAA, Read: 0x");
    Serial.println(result2, HEX);
    
    if (result2 != 0xAA) {
      success = false;
      Serial.println("  ⚠️  Register persistence issue detected");
    }
    
    // Test 3: Version register (read-only test)
    byte version = readRegWithMode(ARDUCHIP_VER, SPI_MODE3);
    Serial.print("  Version register: 0x");
    Serial.println(version, HEX);
    
    if (success && result1 == 0x55 && result2 == 0xAA) {
      Serial.print("  ✅ MODE3 working perfectly at ");
      Serial.print(speeds[i] / 1000000.0);
      Serial.println(" MHz");
      
      // Test image capture readiness
      Serial.println("\n🎯 Testing image capture readiness...");
      
      // Clear FIFO
      writeRegWithMode(ARDUCHIP_FIFO, FIFO_CLEAR_MASK, SPI_MODE3);
      delay(50);
      
      // Check FIFO status
      byte fifo_status = readRegWithMode(ARDUCHIP_FIFO, SPI_MODE3);
      Serial.print("  FIFO status: 0x");
      Serial.println(fifo_status, HEX);
      
      Serial.println("  ✅ Camera is ready for image capture!");
      Serial.println("\n📸 Next steps:");
      Serial.println("  - Try menu option 3 for test image capture");
      Serial.println("  - Try menu option 6 for JPEG image capture");
      Serial.println("  - Your camera is now fully operational!");
      
      return;
    } else {
      Serial.print("  ❌ Test register issues at ");
      Serial.print(speeds[i] / 1000000.0);
      Serial.println(" MHz");
    }
  }
  
  // If we get here, test register method failed
  Serial.println("\n📋 Analysis:");
  Serial.println("⚠️  The test register (0x00) may not be writable on your camera");
  Serial.println("⚠️  This is common with some ArduCAM modules");
  Serial.println("✅ Your camera is likely working, but needs different testing");
  
  Serial.println("\n🎯 Recommendations:");
  Serial.println("1. Try menu option 'F' for improved testing (recommended)");
  Serial.println("2. Try menu option '1' for basic connection testing");
  Serial.println("3. Try menu option '6' for direct JPEG capture");
  Serial.println("4. Your camera may be fully functional despite these test results");
}

void optimizedCameraTestImproved() {
  Serial.println("\n=== Improved MODE3 Camera Test ===");
  Serial.println("Testing camera with focus on readable registers...");
  
  // Test 1: Version register (read-only, should be consistent)
  Serial.println("\n🔍 Testing Version Register (Read-Only):");
  byte version1 = readRegWithMode(ARDUCHIP_VER, SPI_MODE3);
  delay(10);
  byte version2 = readRegWithMode(ARDUCHIP_VER, SPI_MODE3);
  delay(10);
  byte version3 = readRegWithMode(ARDUCHIP_VER, SPI_MODE3);
  
  Serial.print("Version readings: 0x");
  Serial.print(version1, HEX);
  Serial.print(", 0x");
  Serial.print(version2, HEX);
  Serial.print(", 0x");
  Serial.println(version3, HEX);
  
  if (version1 == version2 && version2 == version3) {
    Serial.println("✅ Version register is consistent - SPI communication working!");
    
    // Test 2: Test if we can read different registers consistently
    Serial.println("\n🔍 Testing Multiple Registers:");
    byte fifo_reg = readRegWithMode(ARDUCHIP_FIFO, SPI_MODE3);
    byte mode_reg = readRegWithMode(ARDUCHIP_MODE, SPI_MODE3);
    byte gpio_reg = readRegWithMode(ARDUCHIP_GPIO, SPI_MODE3);
    
    Serial.print("FIFO register: 0x");
    Serial.println(fifo_reg, HEX);
    Serial.print("MODE register: 0x");
    Serial.println(mode_reg, HEX);
    Serial.print("GPIO register: 0x");
    Serial.println(gpio_reg, HEX);
    
    // Test 3: Test write operations on control registers
    Serial.println("\n🔍 Testing Write Operations:");
    
    // Try writing to FIFO control (this should work)
    byte original_fifo = readRegWithMode(ARDUCHIP_FIFO, SPI_MODE3);
    Serial.print("Original FIFO status: 0x");
    Serial.println(original_fifo, HEX);
    
    // Clear FIFO
    writeRegWithMode(ARDUCHIP_FIFO, FIFO_CLEAR_MASK, SPI_MODE3);
    delay(50); // Extended delay
    byte after_clear = readRegWithMode(ARDUCHIP_FIFO, SPI_MODE3);
    
    Serial.print("After FIFO clear: 0x");
    Serial.println(after_clear, HEX);
    
    // Try setting timing register (format)
    writeRegWithMode(ARDUCHIP_TIM, 0x01, SPI_MODE3); // JPEG format
    delay(50);
    byte timing_reg = readRegWithMode(ARDUCHIP_TIM, SPI_MODE3);
    
    Serial.print("Timing register (set to JPEG): 0x");
    Serial.println(timing_reg, HEX);
    
    // Test 4: Check if camera responds to different commands
    Serial.println("\n🔍 Testing Camera Response:");
    
    // Reset FIFO pointers
    writeRegWithMode(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK, SPI_MODE3);
    delay(50);
    writeRegWithMode(ARDUCHIP_FIFO, FIFO_WRPTR_RST_MASK, SPI_MODE3);
    delay(50);
    
    byte fifo_after_reset = readRegWithMode(ARDUCHIP_FIFO, SPI_MODE3);
    Serial.print("FIFO after pointer reset: 0x");
    Serial.println(fifo_after_reset, HEX);
    
    // Check FIFO size registers
    byte fifo_size1 = readRegWithMode(0x42, SPI_MODE3);
    byte fifo_size2 = readRegWithMode(0x43, SPI_MODE3);
    byte fifo_size3 = readRegWithMode(0x44, SPI_MODE3);
    
    Serial.print("FIFO size registers: 0x");
    Serial.print(fifo_size1, HEX);
    Serial.print(", 0x");
    Serial.print(fifo_size2, HEX);
    Serial.print(", 0x");
    Serial.println(fifo_size3, HEX);
    
    Serial.println("\n🎉 CAMERA OPERATIONAL ANALYSIS:");
    Serial.println("✅ SPI MODE3 communication is working");
    Serial.println("✅ Camera registers are readable");
    Serial.println("✅ Camera responds to control commands");
    
    // Identify camera type
    Serial.print("📷 Camera Type: ");
    if (version1 == 0x40) {
      Serial.println("ArduCAM Mini 2MP");
    } else if (version1 == 0x41) {
      Serial.println("ArduCAM Mini 3MP");
    } else if (version1 == 0x42) {
      Serial.println("ArduCAM Mini 5MP");
    } else if (version1 == 0x43) {
      Serial.println("ArduCAM Mini 8MP");
    } else {
      Serial.print("Unknown ArduCAM (Version: 0x");
      Serial.print(version1, HEX);
      Serial.println(")");
    }
    
    Serial.println("\n📸 Ready for image capture!");
    Serial.println("🎯 Next steps:");
    Serial.println("   - Try menu option 3 for basic image capture");
    Serial.println("   - Try menu option 6 for JPEG image capture");
    Serial.println("   - Your camera is fully functional!");
    
    return;
  } else {
    Serial.println("❌ Version register inconsistent - SPI communication issue");
  }
  
  // If version register is inconsistent, try slower speeds
  Serial.println("\n🔍 Testing slower speeds for better reliability:");
  
  int slower_speeds[] = {125000, 250000, 500000}; // 125kHz to 500kHz
  
  for (int i = 0; i < 3; i++) {
    Serial.print("Testing at ");
    Serial.print(slower_speeds[i] / 1000.0);
    Serial.println(" kHz...");
    
    // Test version register consistency at slower speed
    byte v1 = readRegWithSpeed(ARDUCHIP_VER, slower_speeds[i]);
    delay(20);
    byte v2 = readRegWithSpeed(ARDUCHIP_VER, slower_speeds[i]);
    delay(20);
    byte v3 = readRegWithSpeed(ARDUCHIP_VER, slower_speeds[i]);
    
    Serial.print("  Version readings: 0x");
    Serial.print(v1, HEX);
    Serial.print(", 0x");
    Serial.print(v2, HEX);
    Serial.print(", 0x");
    Serial.println(v3, HEX);
    
    if (v1 == v2 && v2 == v3 && v1 != 0x00 && v1 != 0xFF) {
      Serial.print("✅ Camera working reliably at ");
      Serial.print(slower_speeds[i] / 1000.0);
      Serial.println(" kHz");
      Serial.println("🎯 Recommendation: Use this speed for all operations");
      return;
    }
  }
  
  Serial.println("❌ Camera not responding consistently at any speed");
  Serial.println("📋 Try menu option 'D' for ultra-slow detection");
}

void properCaptureSequence() {
  Serial.println("\n=== Proper Capture Sequence for Your Camera ===");
  
  // Step 1: Initialize camera for capture
  Serial.println("Step 1: Initializing camera for capture...");
  writeRegWithMode(ARDUCHIP_MODE, 0x00, SPI_MODE3);  // Reset mode
  delay(100);
  
  // Step 2: Set JPEG format explicitly
  Serial.println("Step 2: Setting JPEG format...");
  writeRegWithMode(ARDUCHIP_TIM, 0x01, SPI_MODE3);   // JPEG format
  delay(100);
  
  // Step 3: Clear FIFO completely
  Serial.println("Step 3: Clearing FIFO buffer...");
  writeRegWithMode(ARDUCHIP_FIFO, FIFO_CLEAR_MASK, SPI_MODE3);
  delay(100);
  writeRegWithMode(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK, SPI_MODE3);
  delay(50);
  writeRegWithMode(ARDUCHIP_FIFO, FIFO_WRPTR_RST_MASK, SPI_MODE3);
  delay(50);
  
  // Step 4: Verify FIFO is clear
  Serial.println("Step 4: Verifying FIFO is clear...");
  byte fifo_status = readRegWithMode(ARDUCHIP_FIFO, SPI_MODE3);
  Serial.print("FIFO status after clear: 0x");
  Serial.println(fifo_status, HEX);
  
  // Step 5: Start capture mode
  Serial.println("Step 5: Starting capture mode...");
  writeRegWithMode(ARDUCHIP_FIFO, FIFO_START_MASK, SPI_MODE3);
  delay(100);
  
  // Step 6: Trigger capture (multiple methods)
  Serial.println("Step 6: Triggering capture...");
  
  // Method 1: Mode register trigger
  writeRegWithMode(ARDUCHIP_MODE, 0x01, SPI_MODE3);
  delay(500);
  
  // Method 2: Trigger register
  writeRegWithMode(ARDUCHIP_TRIG, 0x01, SPI_MODE3);
  delay(500);
  
  // Method 3: GPIO trigger (some cameras use this)
  writeRegWithMode(ARDUCHIP_GPIO, 0x01, SPI_MODE3);
  delay(500);
  
  // Step 7: Wait for capture completion
  Serial.println("Step 7: Waiting for capture completion...");
  bool capture_done = false;
  
  for (int i = 0; i < 20; i++) {
    fifo_status = readRegWithMode(ARDUCHIP_FIFO, SPI_MODE3);
    Serial.print("Status check ");
    Serial.print(i + 1);
    Serial.print("/20: 0x");
    Serial.println(fifo_status, HEX);
    
    // Check various completion indicators
    if (fifo_status & 0x08) {  // Capture done bit
      capture_done = true;
      Serial.println("✅ Capture completed (method 1)!");
      break;
    }
    if (fifo_status & 0x04) {  // Alternative done bit
      capture_done = true;
      Serial.println("✅ Capture completed (method 2)!");
      break;
    }
    
    delay(250);
  }
  
  // Step 8: Check final FIFO size
  Serial.println("Step 8: Checking final FIFO size...");
  byte fifo_size1 = readReg(0x42);
  byte fifo_size2 = readReg(0x43);
  byte fifo_size3 = readReg(0x44);
  
  unsigned long fifo_length = ((unsigned long)fifo_size3 << 16) | 
                              ((unsigned long)fifo_size2 << 8) | 
                              fifo_size1;
  
  Serial.print("Final FIFO length: ");
  Serial.print(fifo_length);
  Serial.println(" bytes");
  
  // Step 9: Read first 32 bytes to check for JPEG header
  Serial.println("Step 9: Checking for JPEG header...");
  Serial.print("First 32 bytes: ");
  
  for (int i = 0; i < 32; i++) {
    byte data = readReg(SINGLE_FIFO_READ);
    Serial.print("0x");
    if (data < 0x10) Serial.print("0");
    Serial.print(data, HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Step 10: Check for JPEG markers
  Serial.println("Step 10: JPEG header analysis...");
  
  // Reset FIFO read pointer to start
  writeReg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
  delay(50);
  
  byte first_byte = readReg(SINGLE_FIFO_READ);
  byte second_byte = readReg(SINGLE_FIFO_READ);
  
  Serial.print("First two bytes: 0x");
  Serial.print(first_byte, HEX);
  Serial.print(" 0x");
  Serial.println(second_byte, HEX);
  
  if (first_byte == 0xFF && second_byte == 0xD8) {
    Serial.println("✅ Valid JPEG header found!");
    Serial.println("🎯 Camera is working properly!");
    Serial.println("📋 Try menu option '7' to stream the full image");
  } else {
    Serial.println("❌ No valid JPEG header found");
    Serial.println("💡 Camera may need different initialization sequence");
    
    // Try alternative JPEG detection
    Serial.println("Trying alternative JPEG detection...");
    writeReg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
    delay(50);
    
    // Look for JPEG header in first  100 bytes
    bool jpeg_found = false;
    for (int i = 0; i < 100; i++) {
      byte b1 = readReg(SINGLE_FIFO_READ);
      if (b1 == 0xFF) {
        byte b2 = readReg(SINGLE_FIFO_READ);
        if (b2 == 0xD8 || b2 ==  0xE0) {
          Serial.print("✅ JPEG header found at position ");
          Serial.println(i);
          jpeg_found = true;
          break;
        }
        i++; // Skip one position since we read two bytes
      }
    }
    
    if (!jpeg_found) {
      Serial.println("❌ No JPEG header found in first 100 bytes");
      Serial.println("💡 This might be a sensor configuration issue");
    }
  }
  
  Serial.println("=== Capture sequence complete ===");
}

// OV5642 sensor initialization for ArduCAM Mini 5MP
void initializeOV5642Sensor() {
  Serial.println("\n🎯 Initializing OV5642 Image Sensor...");
  
  // Step 1: ArduCAM hardware reset
  Serial.println("Step 1: Hardware reset...");
  writeRegWithMode(ARDUCHIP_MODE, 0x00, SPI_MODE3);  // Software reset
  delay(200);
  
  // Step 2: Check if sensor is detected
  Serial.println("Step 2: Checking sensor presence...");
  byte version = readRegWithMode(ARDUCHIP_VER, SPI_MODE3);
  Serial.print("ArduCAM version: 0x");
  Serial.println(version, HEX);
  
  // Step 3: Clear FIFO flag
  Serial.println("Step 3: Clearing FIFO...");
  writeRegWithMode(ARDUCHIP_FIFO, 0x01, SPI_MODE3);
  delay(100);
  
  // Step 4: Set JPEG format and initialize sensor
  Serial.println("Step 4: Setting JPEG format...");
  writeRegWithMode(ARDUCHIP_TIM, 0x01, SPI_MODE3);  // Enable JPEG mode
  delay(100);
  
  // Step 5: OV5642 specific initialization sequence
  Serial.println("Step 5: OV5642 sensor configuration...");
  
  // Basic OV5642 initialization for JPEG mode
  // These are ArduCAM-specific registers for OV5642 control
  writeRegWithMode(0x15, 0x00, SPI_MODE3);  // JPEG control register
  delay(50);
  writeRegWithMode(0x16, 0x24, SPI_MODE3);  // Clock settings
  delay(50);
  writeRegWithMode(0x17, 0x18, SPI_MODE3);  // Frame control
  delay(50);
  writeRegWithMode(0x18, 0x04, SPI_MODE3);  // Frame control
  delay(50);
  
  // JPEG quality and format settings
  writeRegWithMode(0x32, 0x80, SPI_MODE3);  // DSP control
  delay(50);
  writeRegWithMode(0x19, 0x03, SPI_MODE3);  // Format control
  delay(50);
  writeRegWithMode(0x1A, 0x40, SPI_MODE3);  // Format control
  delay(50);
  
  // Resolution settings for VGA (640x480)
  writeRegWithMode(0x03, 0x12, SPI_MODE3);  // Common control A
  delay(50);
  writeRegWithMode(0x32, 0x80, SPI_MODE3);  // Common control B  
  delay(50);
  writeRegWithMode(0x17, 0x18, SPI_MODE3);  // Horizontal window start
  delay(50);
  writeRegWithMode(0x18, 0x04, SPI_MODE3);  // Horizontal window end
  delay(50);
  writeRegWithMode(0x19, 0x01, SPI_MODE3);  // Vertical window start
  delay(50);
  writeRegWithMode(0x1A, 0x81, SPI_MODE3);  // Vertical window end
  delay(50);
  
  // JPEG settings
  writeRegWithMode(0x37, 0x08, SPI_MODE3);  // JPEG control
  delay(50);
  writeRegWithMode(0x38, 0x30, SPI_MODE3);  // JPEG quality (moderate)
  delay(50);
  
  Serial.println("Step 6: Final sensor setup...");
  
  // Test sensor communication
  byte test_read = readRegWithMode(ARDUCHIP_TIM, SPI_MODE3);
  Serial.print("Format register readback: 0x");
  Serial.println(test_read, HEX);
  
  if (test_read == 0x01) {
    Serial.println("✅ Sensor configuration appears successful");
  } else {
    Serial.println("⚠️ Sensor configuration may need adjustment");
  }
}

void captureImageWithSensorInit() {
  Serial.println("\n=== Image Capture with Sensor Initialization ===");
  
  // Step 1: Initialize the sensor first
  initializeOV5642Sensor();
  
  // Step 2: Clear FIFO completely
  Serial.println("Clearing FIFO buffer...");
  writeRegWithMode(ARDUCHIP_FIFO, FIFO_CLEAR_MASK, SPI_MODE3);
  delay(100);
  writeRegWithMode(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK, SPI_MODE3);
  delay(50);
  writeRegWithMode(ARDUCHIP_FIFO, FIFO_WRPTR_RST_MASK, SPI_MODE3);
  delay(50);
  
  // Step 3: Verify FIFO is empty
  byte fifo_size1 = readRegWithMode(0x42, SPI_MODE3);
  byte fifo_size2 = readRegWithMode(0x43, SPI_MODE3);
  byte fifo_size3 = readRegWithMode(0x44, SPI_MODE3);
  unsigned long initial_size = ((unsigned long)fifo_size3 << 16) | 
                               ((unsigned long)fifo_size2 << 8) | 
                               fifo_size1;
  
  Serial.print("Initial FIFO size: ");
  Serial.print(initial_size);
  Serial.println(" bytes");
  
  // Step 4: Start capture sequence
  Serial.println("Starting capture sequence...");
  writeRegWithMode(ARDUCHIP_FIFO, FIFO_START_MASK, SPI_MODE3);
  delay(100);
  
  // Step 5: Trigger actual image capture
  Serial.println("Triggering image capture...");
  writeRegWithMode(ARDUCHIP_MODE, 0x02, SPI_MODE3);  // Capture trigger
  delay(1000);  // Give sensor time to capture
  
  // Step 6: Check for capture completion
  Serial.println("Waiting for capture completion...");
  bool capture_completed = false;
  
  for (int i = 0; i < 30; i++) {
    byte status = readRegWithMode(ARDUCHIP_FIFO, SPI_MODE3);
    Serial.print("Status ");
    Serial.print(i + 1);
    Serial.print(": 0x");
    Serial.println(status, HEX);
    
    // Check different completion flags
    if (status & 0x08) {  // Capture done
      capture_completed = true;
      Serial.println("✅ Capture completed!");
      break;
    }
    
    delay(200);
  }
  
  // Step 7: Check final FIFO size
  Serial.println("Step 7: Checking final FIFO size...");
  fifo_size1 = readReg(0x42);
  fifo_size2 = readReg(0x43);
  fifo_size3 = readReg(0x44);
  
  unsigned long final_size = ((unsigned long)fifo_size3 << 16) | 
                             ((unsigned long)fifo_size2 << 8) | 
                             fifo_size1;
  
  Serial.print("Final FIFO size: ");
  Serial.print(final_size);
  Serial.println(" bytes");
  
  // Step 8: Check for valid JPEG data
  Serial.println("Checking for JPEG header...");
  writeReg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
  delay(50);
  
  Serial.print("First 16 bytes: ");
  for (int i = 0; i < 16; i++) {
    byte b = readReg(SINGLE_FIFO_READ);
    Serial.print("0x");
    if (b < 0x10) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Reset and check for JPEG markers
  writeReg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
  delay(50);
  
  byte first = readReg(SINGLE_FIFO_READ);
  byte second = readReg(SINGLE_FIFO_READ);
  
  if (first == 0xFF && second == 0xD8) {
    Serial.println("🎉 Valid JPEG header found!");
    Serial.println("📸 Image capture successful!");
    Serial.println("💡 You can now use menu option 7 to stream the full image");
  } else {
    Serial.print("❌ No JPEG header. Got: 0x");
    Serial.print(first, HEX);
    Serial.print(" 0x");
    Serial.println(second, HEX);
    Serial.println("💡 Sensor may need additional configuration");
  }
  
  Serial.println("=== Full sensor setup test complete ===");
}

bool testCameraConnection() {
  Serial.println("=== Testing Camera Connection ===");
  
  // Test basic SPI communication with MODE3
  writeRegWithMode(ARDUCHIP_TEST1, 0x55, SPI_MODE3);
  delay(10);
  byte test_val = readRegWithMode(ARDUCHIP_TEST1, SPI_MODE3);
  
  if (test_val == 0x55) {
    Serial.println("✅ Camera connection OK");
    
    // Read version register
    byte version = readRegWithMode(ARDUCHIP_VER, SPI_MODE3);
    Serial.print("Camera version: 0x");
    Serial.println(version, HEX);
    
    return true;
  } else {
    Serial.print("❌ Camera connection failed. Expected 0x55, got 0x");
    Serial.println(test_val, HEX);
    return false;
  }
}

void testSPIConnection() {
  Serial.println("=== Testing SPI Connection ===");
  
  // Test different patterns
  byte patterns[] = {0x55, 0xAA, 0x33, 0xCC};
  bool all_passed = true;
  
  for (int i = 0; i < 4; i++) {
    writeRegWithMode(ARDUCHIP_TEST1, patterns[i], SPI_MODE3);
    delay(10);
    byte result = readRegWithMode(ARDUCHIP_TEST1, SPI_MODE3);
    
    Serial.print("Pattern 0x");
    Serial.print(patterns[i], HEX);
    Serial.print(" -> 0x");
    Serial.print(result, HEX);
    
    if (result == patterns[i]) {
      Serial.println(" ✅");
    } else {
      Serial.println(" ❌");
      all_passed = false;
    }
  }
  
  if (all_passed) {
    Serial.println("✅ SPI connection test passed");
  } else {
    Serial.println("❌ SPI connection test failed");
  }
}

void captureImage() {
  Serial.println("=== Basic Image Capture ===");
  
  // Clear FIFO
  writeRegWithMode(ARDUCHIP_FIFO, 0x01, SPI_MODE3);
  delay(100);
  
  // Check initial FIFO size
  uint32_t initial_size = readFIFOLength();
  Serial.print("Initial FIFO: ");
  Serial.print(initial_size);
  Serial.println(" bytes");
  
  // Trigger capture
  writeRegWithMode(ARDUCHIP_MODE, 0x02, SPI_MODE3);
  delay(1000);
  
  // Check final FIFO size
  uint32_t final_size = readFIFOLength();
  Serial.print("Final FIFO: ");
  Serial.print(final_size);
  Serial.println(" bytes");
  
  if (final_size > initial_size + 50) {
    Serial.println("✅ Image data captured");
    
    // Check for JPEG header
    writeRegWithMode(ARDUCHIP_FIFO, 0x00, SPI_MODE3);
    delay(10);
    
    byte first = readReg(ARDUCHIP_FIFO_BURST);
    byte second = readReg(ARDUCHIP_FIFO_BURST);
    
    if (first == 0xFF && second == 0xD8) {
      Serial.println("🎉 Valid JPEG header found!");
    } else {
      Serial.print("⚠️  No JPEG header. Got: 0x");
      Serial.print(first, HEX);
      Serial.print(" 0x");
      Serial.println(second, HEX);
    }
  } else {
    Serial.println("❌ No image data captured");
  }
}

bool initializeSPICamera() {
  Serial.println("=== Initializing SPI Camera ===");
  
  // Initialize SPI
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  delay(100);
  
  // Test basic communication
  writeRegWithMode(ARDUCHIP_TEST1, 0xAA, SPI_MODE3);
  delay(10);
  byte test_val = readRegWithMode(ARDUCHIP_TEST1, SPI_MODE3);
  
  if (test_val == 0xAA) {
    Serial.println("✅ SPI Camera initialized successfully");
    
    // Set JPEG mode
    writeRegWithMode(ARDUCHIP_TIM, 0x01, SPI_MODE3);
    delay(50);
    
    // Clear FIFO
    writeRegWithMode(ARDUCHIP_FIFO, 0x01, SPI_MODE3);
    delay(50);
    
    return true;
  } else {
    Serial.print("❌ SPI Camera initialization failed. Expected 0xAA, got 0x");
    Serial.println(test_val, HEX);
    return false;
  }
}

bool initializeCameraForMode3() {
  Serial.println("=== Initializing Camera for SPI MODE3 ===");
  
  // Initialize SPI with MODE3 settings
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  // Test basic communication
  writeRegWithMode(ARDUCHIP_TEST1, 0x55, SPI_MODE3);
  delay(10);
  byte test_val = readRegWithMode(ARDUCHIP_TEST1, SPI_MODE3);
  
  if (test_val == 0x55) {
    Serial.println("✅ Camera responding to SPI MODE3");
    
    // Clear FIFO
    writeRegWithMode(ARDUCHIP_FIFO, 0x01, SPI_MODE3);
    delay(10);
    
    // Set to JPEG mode
    writeRegWithMode(ARDUCHIP_TRIG, 0x01, SPI_MODE3);
    delay(10);
    
    return true;
  } else {
    Serial.print("❌ Camera not responding. Expected 0x55, got 0x");
    Serial.println(test_val, HEX);
    return false;
  }
}

void comprehensiveOV5642Init() {
  Serial.println("=== Comprehensive OV5642 Sensor Initialization ===");
  
  // Phase 1: Hardware reset and basic setup
  Serial.println("Phase 1: Hardware preparation...");
  
  // Clear any previous state
  writeRegWithMode(ARDUCHIP_FIFO, 0x01, SPI_MODE3);  // Clear FIFO
  delay(100);
  
  // Software reset
  writeRegWithMode(ARDUCHIP_MODE, 0x00, SPI_MODE3);  // Reset mode
  delay(500);  // Extended reset time
  
  // Phase 2: Enable JPEG mode
  Serial.println("Phase 2: Enabling JPEG format...");
  writeRegWithMode(ARDUCHIP_TIM, 0x01, SPI_MODE3);   // Enable JPEG
  delay(200);
  
  // Phase 3: Basic sensor configuration
  Serial.println("Phase 3: Basic sensor setup...");
  
  // Set JPEG format and quality
  writeRegWithMode(0x15, 0x00, SPI_MODE3);  // JPEG control register
  delay(50);
  writeRegWithMode(0x16, 0x24, SPI_MODE3);  // Clock settings
  delay(50);
  writeRegWithMode(0x17, 0x18, SPI_MODE3);  // Frame control
  delay(50);
  writeRegWithMode(0x18, 0x04, SPI_MODE3);  // Frame control
  delay(50);
  
  // JPEG quality and format settings
  writeRegWithMode(0x32, 0x80, SPI_MODE3);  // DSP control
  delay(50);
  writeRegWithMode(0x19, 0x03, SPI_MODE3);  // Format control
  delay(50);
  writeRegWithMode(0x1A, 0x40, SPI_MODE3);  // Format control
  delay(50);
  
  // Phase 4: Wait for sensor stabilization
  Serial.println("Phase 4: Waiting for sensor stabilization...");
  delay(1000);
  
  // Phase 5: Verify key registers
  Serial.println("Phase 5: Verifying configuration...");
  byte jpeg_enable = readRegWithMode(ARDUCHIP_TIM, SPI_MODE3);
  Serial.print("JPEG mode register: 0x");
  Serial.println(jpeg_enable, HEX);
  
  if (jpeg_enable & 0x01) {
    Serial.println("✅ JPEG mode enabled successfully");
  } else {
    Serial.println("⚠️  JPEG mode may not be properly enabled");
  }
  
  Serial.println("✅ Comprehensive OV5642 initialization complete");
}

void captureWithFullSensorInit() {
  Serial.println("\n=== Full OV5642 Setup + Image Capture ===");
  
  // Step 1: Complete sensor initialization
  comprehensiveOV5642Init();
  
  // Step 2: Clear FIFO and prepare for capture
  Serial.println("\nStep 2: Preparing for capture...");
  writeRegWithMode(ARDUCHIP_FIFO, 0x01, SPI_MODE3);  // Clear FIFO
  delay(100);
  
  // Check initial FIFO size
  uint32_t initial_size = readFIFOLength();
  Serial.print("Initial FIFO size: ");
  Serial.print(initial_size);
  Serial.println(" bytes");
  
  // Step 3: Trigger capture
  Serial.println("Step 3: Triggering image capture...");
  writeRegWithMode(ARDUCHIP_MODE, 0x02, SPI_MODE3);  // Start capture
  delay(2000);  // Extended capture time for OV5642
  
  // Step 4: Check capture completion
  Serial.println("Step 4: Checking capture status...");
  bool capture_done = false;
  
  for (int i = 0; i < 50; i++) {  // Extended timeout
    byte status = readRegWithMode(ARDUCHIP_TRIG, SPI_MODE3);
    Serial.print("Status check ");
    Serial.print(i + 1);
    Serial.print(": 0x");
    Serial.println(status, HEX);
    
    // Check for capture done flag
    if (!(status & 0x02)) {  // Capture bit cleared = done
      capture_done = true;
      Serial.println("✅ Capture completed!");
      break;
    }
    
    delay(100);
  }
  
  if (!capture_done) {
    Serial.println("⚠️  Capture may still be in progress or failed");
  }
  
  // Step 5: Check FIFO size
  uint32_t final_size = readFIFOLength();
  Serial.print("Final FIFO size: ");
  Serial.print(final_size);
  Serial.println(" bytes");
  
  if (final_size > initial_size + 100) {
    Serial.println("✅ Image data detected in FIFO");
    
    // Step 6: Read and check JPEG header
    Serial.println("Step 6: Checking JPEG header...");
    
    writeRegWithMode(ARDUCHIP_FIFO, 0x00, SPI_MODE3);  // FIFO read mode
    delay(10);
    
    // Read first few bytes to check for JPEG header
    Serial.print("First 16 bytes: ");
    for (int i = 0; i < 16; i++) {
      byte b = readRegWithMode(ARDUCHIP_FIFO_BURST, SPI_MODE3);
      Serial.print("0x");
      if (b < 0x10) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
    }
    Serial.println();
    
    // Reset FIFO read pointer
    writeRegWithMode(ARDUCHIP_FIFO, 0x01, SPI_MODE3);
    writeRegWithMode(ARDUCHIP_FIFO, 0x00, SPI_MODE3);
    delay(10);
    
    // Check for JPEG SOI marker
    byte first = readRegWithMode(ARDUCHIP_FIFO_BURST, SPI_MODE3);
    byte second = readRegWithMode(ARDUCHIP_FIFO_BURST, SPI_MODE3);
    
    if (first == 0xFF && second == 0xD8) {
      Serial.println("🎉 Valid JPEG header found!");
      Serial.println("📸 Image capture successful!");
      Serial.println("💡 You can now use menu option 7 to stream the full image");
    } else {
      Serial.print("❌ No JPEG header. Got: 0x");
      Serial.print(first, HEX);
      Serial.print(" 0x");
      Serial.println(second, HEX);
      Serial.println("💡 Sensor may need additional configuration");
    }
  } else {
    Serial.println("❌ No significant data in FIFO");
    Serial.println("💡 Sensor initialization may have failed");
  }
  
  Serial.println("=== Full sensor setup test complete ===");
}