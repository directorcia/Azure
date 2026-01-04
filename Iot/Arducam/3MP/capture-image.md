# Arducam Mega 3MP Camera - Image Capture Documentation

## Overview

This Arduino sketch enables image capture and transmission using an **Arducam Mega 3MP camera module**. The script allows real-time image capture at multiple resolutions, quality adjustments, and serial transmission of image data for processing on a host computer.

### Key Features
- **Multiple Resolution Support**: QVGA (320x240), VGA (640x480), 1080p (1920x1080), and 3MP (2048x1536)
- **Adjustable Quality**: HIGH, MEDIUM, and LOW quality settings
- **Real-time Feedback**: Capture statistics and error reporting via serial communication
- **Efficient Data Streaming**: JPEG compression with JPEG end-marker detection
- **Safety Mechanisms**: Timeout protection and data validation

---

## Hardware Requirements

### Microcontroller
- **Arduino Leonardo** (or compatible board with ICSP pins)
- Sufficient RAM for image buffers
- USB connection for serial communication

### Camera Module
- **Arducam Mega 3MP** (OV5642 sensor)
- SPI interface
- 3.3V operation

### Connections (SPI via ICSP)
| Signal | Arduino Pin | ICSP Pin | Description |
|--------|------------|----------|-------------|
| MOSI   | Digital 16 | ICSP-4   | Master Out, Slave In (data from Arduino to camera) |
| MISO   | Digital 14 | ICSP-1   | Master In, Slave Out (data from camera to Arduino) |
| SCK    | Digital 15 | ICSP-3   | Serial Clock (synchronization signal) |
| CS     | Digital 10 | -        | Chip Select (active LOW, selects camera) |
| GND    | GND        | ICSP-6   | Ground reference |
| 3.3V   | 3.3V       | ICSP-2   | Power supply (use voltage regulator if needed) |

### Additional Hardware
- **USB Cable** for serial communication and power
- **Voltage Regulator** (if powering 3.3V camera from 5V Arduino)
- **Jumper Wires** for connections

---

## Software Requirements

### Arduino Libraries
1. **Arduino.h** - Core Arduino functions
2. **Arducam_Mega.h** - Arducam camera control library
3. **SPI.h** - Serial Peripheral Interface communication

### Installation
```bash
# Install via Arduino IDE Library Manager:
# Sketch → Include Library → Manage Libraries
# Search for "Arducam" and install the Arducam Mega library
```

### Compatible Arduino Boards
- Arduino Leonardo
- Arduino Micro
- Arduino Pro Micro
- Other boards with ICSP header support

---

## Setup and Installation

### 1. Hardware Assembly
1. Connect the Arducam module to Arduino using SPI pins (ICSP header)
2. Ensure proper voltage regulation (camera requires 3.3V)
3. Verify all connections are secure and correct polarity

### 2. Library Installation
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for "Arducam" and install the official library
4. Verify installation by checking Arduino libraries folder

### 3. Upload Sketch
1. Connect Arduino to computer via USB
2. Select Board: **Tools → Board → Arduino Leonardo** (or your board)
3. Select Port: **Tools → Port → COM[X]** (where X is your port number)
4. Click **Upload** button or press Ctrl+U

### 4. Verify Communication
1. Open **Tools → Serial Monitor**
2. Set baud rate to **115200**
3. You should see the initialization message:
   ```
   ========================================
     Arducam Mega 3MP Camera - Live Test  
   ========================================
   
   Initializing camera...
   SUCCESS! Camera initialized!
   ```

---

## Operation Guide

### Serial Monitor Setup
1. **Baud Rate**: 115200 bps
2. **Line Ending**: No line ending (or "Both NL & CR")
3. **Keep Serial Monitor open** while testing

### Available Commands

| Command | Function | Resolution | Notes |
|---------|----------|------------|-------|
| `c` or `C` | Capture image | VGA (640x480) | Default capture command |
| `1` | Capture QVGA | 320x240 | Lowest resolution, fastest capture |
| `2` | Capture VGA | 640x480 | Standard resolution |
| `3` | Capture FHD | 1920x1080 | Full HD, larger file size |
| `4` | Capture 3MP | 2048x1536 | Maximum resolution, largest file |
| `q` or `Q` | Set HIGH quality | - | Larger file size, better quality |
| `w` or `W` | Set MEDIUM quality | - | Balanced quality and file size |
| `e` or `E` | Set LOW quality | - | Smallest file size, lower quality |

### Usage Example
```
1. Open Serial Monitor (115200 baud)
2. Wait for initialization message
3. Type: 1
   → Captures 320x240 image
4. Type: q
   → Sets quality to HIGH
5. Type: 4
   → Captures 3MP image at HIGH quality
6. Type: 2
   → Captures 640x480 image at HIGH quality
```

---

## Execution Flow

### Startup Sequence
```
┌─────────────────────────────────┐
│ Arduino Power-On                │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│ Serial Communication (115200)   │
│ SPI Protocol Initialization     │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│ Camera Hardware Initialization  │
│ Verify SPI Communication        │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│ Display Welcome Message         │
│ Show Available Commands         │
│ Ready for User Input            │
└─────────────────────────────────┘
```

### Main Loop Operation
```
┌──────────────────────────────────┐
│ Wait for Serial Input (10ms)     │
└────────────┬─────────────────────┘
             │
             ▼ (Data available?)
         ┌───┴────┐
         │ NO     │ YES
         │        └────────────────┐
         │                         ▼
         │            ┌──────────────────────────┐
         │            │ Read Command Character   │
         │            └────────┬─────────────────┘
         │                     │
         │                     ▼
         │            ┌──────────────────────────┐
         │            │ Process Command          │
         │            │ - Resolution selection   │
         │            │ - Quality adjustment     │
         │            │ - Image capture          │
         │            └────────┬─────────────────┘
         │                     │
         └─────────────────────┘
                     │
                     ▼
         ┌──────────────────────────┐
         │ Loop (10ms delay)        │
         └──────────────────────────┘
```

### Image Capture Process
```
┌──────────────────────────────────────────┐
│ User Command: Capture at Resolution      │
└────────────┬─────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────┐
│ Display Capture Header with Resolution   │
│ ┌──────────────────────────────────────┐ │
│ │ CAPTURING: VGA (640x480)             │ │
│ └──────────────────────────────────────┘ │
└────────────┬─────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────┐
│ Record Start Time (millis())             │
└────────────┬─────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────┐
│ Send takePicture() Command to Camera     │
│ - Specify Resolution                     │
│ - Specify JPEG Format                    │
└────────────┬─────────────────────────────┘
             │
      ┌──────┴──────┐
      ▼             ▼
   SUCCESS      ERROR
      │             │
      │          ┌──┴─────────────────────────┐
      │          │ Display Error Message      │
      │          │ Return from Function       │
      │          └────────────────────────────┘
      │
      ▼
┌──────────────────────────────────────────┐
│ Wait 100ms for Capture to Complete       │
└────────────┬─────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────┐
│ Calculate Capture Duration               │
│ Get Image Size from Camera               │
└────────────┬─────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────┐
│ Display Capture Statistics               │
│ - Capture time (ms)                      │
│ - Image size (bytes and KB)              │
└────────────┬─────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────┐
│ Send Transmission Header                 │
│ SIZE:[bytes]                             │
│ START_BINARY_DATA                        │
└────────────┬─────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────┐
│ Stream Image Data Byte-by-Byte           │
│ Read from camera buffer                  │
│ Send over serial (binary)                │
│ Check for JPEG end marker (0xFF 0xD9)   │
└────────────┬─────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────┐
│ Send Transmission Footer                 │
│ END_IMAGE_DATA                           │
└──────────────────────────────────────────┘
```

---

## Serial Data Format

### Transmission Protocol

#### Header Section
```
┌─────────────────────────────────────────┐
│ Capture Feedback:                       │
│ ┌─────────────────────────────────────┐ │
│ │ CAPTURING: VGA (640x480)            │ │
│ └─────────────────────────────────────┘ │
│                                         │
│ ✓ IMAGE CAPTURED!                       │
│ Capture time: 245 ms                    │
│ Image size: 47834 bytes                 │
│ Size (KB): 46.71                        │
│                                         │
│ SIZE:47834                              │
│ START_BINARY_DATA                       │
└─────────────────────────────────────────┘
```

#### Binary Data Section
```
[47834 bytes of JPEG image data]
- JPEG files begin with: FF D8
- JPEG files end with: FF D9
- Data is raw binary, not ASCII text
```

#### Footer Section
```
┌─────────────────────────────────────────┐
│ END_IMAGE_DATA                          │
│ ─────────────────────────────────────   │
│ (Ready for next command)                │
└─────────────────────────────────────────┘
```

### Example Raw Output
```
┌─────────────────────────────────────┐
│ CAPTURING: VGA (640x480)            │
└─────────────────────────────────────┘

✓ IMAGE CAPTURED!
─────────────────────────────────────
Capture time:  245 ms
Image size:    47834 bytes
Size (KB):     46.71

SIZE:47834
START_BINARY_DATA
[Binary JPEG data: FF D8 FF E0... (47834 bytes) ...FF D9]
END_IMAGE_DATA
─────────────────────────────────────
```

---

## Image Quality Settings

### Quality Levels and Impact

| Quality | File Size | Compression | Best For |
|---------|-----------|-------------|----------|
| HIGH | Largest (~50-80KB for VGA) | Minimal compression | Detail-critical applications |
| MEDIUM (DEFAULT) | Medium (~30-50KB for VGA) | Balanced | General purpose, recommended |
| LOW | Smallest (~15-25KB for VGA) | Heavy compression | Bandwidth-limited scenarios |

### Quality Selection Behavior
```
1. Quality setting is persistent across multiple captures
   - Set quality once, applies to all subsequent captures

2. Changing quality:
   Commands 'q', 'w', 'e' set the quality
   Next capture will use the newly set quality

3. Typical workflow:
   q     → Set HIGH quality
   4     → Capture 3MP (high quality)
   w     → Set MEDIUM quality
   2     → Capture VGA (medium quality)
```

---

## Supported Resolutions

### Resolution Specifications

| Name | Resolution | Dimensions | Typical Size (JPEG) | Notes |
|------|-----------|------------|-------------------|-------|
| QVGA | 320×240 | 320 pixels wide, 240 tall | 10-20KB | Fastest capture, lowest detail |
| VGA | 640×480 | 640 pixels wide, 480 tall | 30-50KB | Standard resolution, balanced |
| FHD (1080p) | 1920×1080 | 1920 pixels wide, 1080 tall | 80-120KB | Full HD, high detail |
| 3MP (QXGA) | 2048×1536 | 2048 pixels wide, 1536 tall | 100-150KB | Maximum resolution, largest file |

### Capture Time Estimates
- QVGA: ~150-200ms
- VGA: ~200-300ms
- FHD: ~400-600ms
- 3MP: ~600-800ms

---

## Code Structure

### Global Variables
```cpp
const int CS_PIN = 10;           // Chip Select pin for SPI
Arducam_Mega myCAM(CS_PIN);     // Camera object instance
```

### Function Breakdown

#### `void setup()`
**Purpose**: Initialize hardware and display welcome message
**Operations**:
1. Initialize serial communication (115200 baud)
2. Wait for serial port connection
3. Initialize SPI protocol
4. Initialize camera hardware
5. Display welcome banner and command reference

#### `void loop()`
**Purpose**: Continuously listen for and process user commands
**Operations**:
1. Check serial buffer for incoming data
2. Read command character
3. Process command via switch statement
4. Execute corresponding action (capture or quality change)
5. Brief delay (10ms) to reduce CPU usage

#### `void captureImage(CAM_IMAGE_MODE resolution)`
**Purpose**: Perform complete image capture and transmission workflow
**Parameters**:
- `resolution`: Target resolution (QVGA, VGA, FHD, QXGA)

**Operations**:
1. Convert resolution to display string
2. Show capture header with resolution info
3. Record capture start time
4. Send capture command to camera
5. Monitor capture status
6. Retrieve image size
7. Send transmission headers
8. Stream image data byte-by-byte
9. Detect JPEG end marker (0xFF 0xD9)
10. Send transmission footer

---

## Troubleshooting Guide

### Issue: "Initializing camera..." but no SUCCESS message

**Cause**: Camera not properly initialized or SPI communication failed

**Solutions**:
1. Verify all SPI pin connections (MOSI, MISO, SCK, CS)
2. Check voltage (camera requires 3.3V)
3. Ensure Arducam library is properly installed
4. Try restarting Arduino IDE and re-uploading sketch
5. Check Arduino board model selection (must support ICSP)

### Issue: Captures start but no data transmitted

**Cause**: Image capture successful but serial transmission issues

**Solutions**:
1. Verify serial baud rate is exactly 115200
2. Check USB cable connection
3. Confirm CS pin (10) is not conflicting with other hardware
4. Increase delay values in `captureImage()` function
5. Try lower resolution captures first

### Issue: "Capture FAILED!" message

**Cause**: Camera returned error status

**Solutions**:
1. Verify camera power supply (3.3V)
2. Check SPI connection integrity
3. Ensure camera is not being accessed by multiple processes
4. Power cycle the Arduino board
5. Try a simple lower-resolution capture (QVGA)

### Issue: Partial or corrupted image data

**Cause**: Serial transmission errors or incomplete data

**Solutions**:
1. Reduce baud rate temporarily for testing
2. Add more delay between data transmission
3. Check for USB cable/connector issues
4. Verify sufficient Arduino RAM for selected resolution
5. Try capturing at lower resolution

### Issue: Serial monitor shows garbage characters

**Cause**: Baud rate mismatch

**Solutions**:
1. Set Serial Monitor baud rate to **exactly 115200**
2. Check that board is selected correctly
3. Verify correct COM port is selected

---

## Performance Considerations

### Capture Time Factors
- **Resolution**: Higher resolution = longer capture time
- **Quality Setting**: Lower quality may actually take longer due to compression
- **Camera Temperature**: Cold sensors may take longer to stabilize
- **Lighting Conditions**: Low light may require longer exposure

### Memory Usage
- Image buffers are allocated internally by Arducam library
- Arduino Leonardo has ~2.5KB of RAM (limited)
- Large resolutions may cause memory issues
- If memory is constrained, use lower resolutions or quality

### Serial Transmission Speed
- **Baud Rate**: 115200 bps = ~14.4 KB/s maximum
- **VGA image (~50KB)**: ~3.5 seconds to transmit
- **3MP image (~150KB)**: ~10 seconds to transmit
- **Faster rates**: Up to 230400 baud (requires code modification)

---

## Extending the Sketch

### Modify Baud Rate
```cpp
// In setup() function:
Serial.begin(230400);  // Change from 115200 to 230400
```

### Add Custom Resolution
```cpp
// Add new case in loop() switch statement:
case '5':
  captureImage(CAM_IMAGE_MODE_CUSTOM);
  break;

// Modify captureImage() to handle new resolution
```

### Implement SD Card Saving
```cpp
// Add after transmission in captureImage():
File imageFile = SD.open("image.jpg", FILE_WRITE);
// Read from camera and write to SD card
imageFile.close();
```

### Auto-Capture Feature
```cpp
// In loop(), add timer:
unsigned long lastCapture = 0;
const unsigned long CAPTURE_INTERVAL = 5000; // 5 seconds

if (millis() - lastCapture >= CAPTURE_INTERVAL) {
  captureImage(CAM_IMAGE_MODE_VGA);
  lastCapture = millis();
}
```

---

## Safety and Best Practices

### Do's ✓
- ✓ Always verify connections before powering on
- ✓ Use proper voltage regulation for 3.3V camera
- ✓ Include delay statements between operations
- ✓ Validate camera initialization before capturing
- ✓ Monitor serial communication for errors
- ✓ Use appropriate baud rate for reliable transmission

### Don'ts ✗
- ✗ Don't use 5V directly on camera (will damage it)
- ✗ Don't remove USB cable during image transmission
- ✗ Don't use incompatible SPI pins for other devices
- ✗ Don't exceed maximum baud rate without testing
- ✗ Don't capture continuously without delays
- ✗ Don't ignore error messages from camera

---

## Reference Material

### JPEG File Format
- **Start Marker**: 0xFF 0xD8 (file begins)
- **End Marker**: 0xFF 0xD9 (file ends)
- **Segments**: SOI, APP0, DQT, DHT, SOF, SOS, etc.

### SPI Protocol
- **Master**: Arduino Leonardo (controls communication)
- **Slave**: Arducam camera (responds to commands)
- **Clock Speed**: Determined by SPI.begin()
- **Data Order**: MSB first (most significant bit)

### Arduino Serial Communication
- **Baud Rate**: Bits per second (115200 = 115,200 bps)
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024-01-04 | Initial release with comprehensive documentation |

---

## Support and Contact

For issues or questions:
1. Verify all hardware connections
2. Check Arduino IDE Console for error messages
3. Review troubleshooting section above
4. Consult Arducam official documentation
5. Test with simplified code examples

---

**Last Updated**: January 4, 2026
**Sketch Name**: capture-image.cpp
**Compatibility**: Arduino Leonardo, Arduino Micro, and compatible boards
