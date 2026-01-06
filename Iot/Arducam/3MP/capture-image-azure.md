# Arducam 3MP Camera - Operation and Execution Guide

## Table of Contents
1. [Startup Sequence](#startup-sequence)
2. [Main Program Loop](#main-program-loop)
3. [Command Processing Flow](#command-processing-flow)
4. [Image Capture Process](#image-capture-process)
5. [Azure Upload Process](#azure-upload-process)
6. [WiFi Connection Flow](#wifi-connection-flow)
7. [Error Handling & Recovery](#error-handling--recovery)
8. [Execution Examples](#execution-examples)
9. [Timing Information](#timing-information)
10. [State Management](#state-management)

---

## Startup Sequence

### Initialization Phase (Power-on or Reset)

When the ESP32 powers on or resets, the `setup()` function executes **once**:

```
POWER ON / RESET
    ↓
setup() function begins
    ↓
Serial communication initialized (115200 baud)
    ↓
Display welcome banner
    ↓
Initialize SPI bus (pins 18, 19, 23, 5)
    ↓
Initialize camera module
    ↓
Display command menu
    ↓
setup() completes
    ↓
Transition to main loop
```

### Serial Output on Startup

```
========================================
  Arducam Mega 3MP Camera - Live Test  
========================================

Initializing camera...
SUCCESS! Camera initialized!

========================================
Camera is ready!
========================================

Commands:
  Capture Commands:
    'c' - Capture image (VGA - default)
    '1' - Capture QVGA (320x240)
    '2' - Capture VGA (640x480)
    '3' - Capture 1080p (1920x1080)
    '4' - Capture 3MP (2048x1536)

  Capture + Upload to Azure:
    'u' - Upload VGA to Azure
    'u1' - Upload QVGA to Azure
    'u2' - Upload VGA to Azure
    'u3' - Upload 1080p to Azure
    'u4' - Upload 3MP to Azure

  Quality Settings:
    'q' - Set quality HIGH (smaller files)
    'w' - Set quality MEDIUM (default)
    'e' - Set quality LOW (larger files)
```

### Post-Startup State

After setup completes, the ESP32 is ready for commands:
- Camera is idle, waiting for commands
- WiFi is disconnected (will connect on-demand)
- Default image quality: MEDIUM
- Capture counter: 0

---

## Main Program Loop

### Loop Execution Cycle

The `loop()` function repeats continuously (~100 times per second):

```
Loop iteration begins
    ↓
Check Serial.available() > 0?
    ├─ NO → delay(10ms) → restart loop
    └─ YES → 
        Read first character from serial buffer
            ↓
        Process command via switch statement
            ↓
        Execute corresponding function
            ↓
        delay(10ms)
            ↓
        Loop restarts
```

### Loop Timing

- **Loop iteration interval**: ~10ms (100 Hz)
- **Actual execution time**: Varies greatly depending on command
  - Simple command processing: ~1-2ms
  - Image capture: 500-2000ms
  - Azure upload: 5-30 seconds
  - During long operations: loop is blocked (unable to process new commands)

### Buffer Behavior

While the loop is blocked during capture/upload:
- New serial input is buffered by the ESP32 UART
- Typically 128-256 byte buffer available
- Commands are processed in the order received
- If buffer fills, oldest commands are lost

---

## Command Processing Flow

### Command Recognition

The program reads **one character at a time** from the serial buffer:

```
Character received from serial
    ↓
Is it 'c'/'C'?     → Capture VGA
Is it '1'?         → Capture QVGA
Is it '2'?         → Capture VGA
Is it '3'?         → Capture 1080p
Is it '4'?         → Capture 3MP
Is it 'u'/'U'?     → Check for second character (u1/u2/u3/u4 or default u)
Is it 'q'/'Q'?     → Set quality HIGH
Is it 'w'/'W'?     → Set quality MEDIUM
Is it 'e'/'E'?     → Set quality LOW
Otherwise          → Ignore unrecognized character
```

### Two-Character Commands (Upload Resolution)

For 'u' or 'U' commands, the program checks if a second character is available:

```
'u' or 'U' received
    ↓
Are more characters available? (within buffer)
    ├─ YES → Wait 10ms (ensure character arrived)
    │   ↓
    │   Read second character
    │   ├─ '1' → captureAndUpload(QVGA)
    │   ├─ '2' → captureAndUpload(VGA)
    │   ├─ '3' → captureAndUpload(1080p)
    │   ├─ '4' → captureAndUpload(3MP)
    │   └─ other → captureAndUpload(VGA) [fallback]
    │
    └─ NO → captureAndUpload(VGA) [default]
```

### Case Insensitivity

Commands 'c' and 'C' are treated identically:
```cpp
case 'c':
case 'C':
    captureImage(CAM_IMAGE_MODE_VGA);
    break;
```

### Quality Setting Execution

When quality command is processed:

```
'q' received → myCAM.setImageQuality(HIGH_QUALITY)
              → Serial: "✓ Quality: HIGH (more compression, smaller files)"

'w' received → myCAM.setImageQuality(DEFAULT_QUALITY)
              → Serial: "✓ Quality: DEFAULT (balanced compression)"

'e' received → myCAM.setImageQuality(LOW_QUALITY)
              → Serial: "✓ Quality: LOW (less compression, larger files)"
```

Quality affects **only future captures**, not previously captured images.

---

## Image Capture Process

### Simple Capture (captureImage function)

Used when commands 'c', '1', '2', '3', or '4' are entered.

#### Step-by-Step Execution

**Phase 1: Display Information**
```
Display: ┌─────────────────────────────────────┐
         │  CAPTURING: QVGA (320x240)          │
         └─────────────────────────────────────┘

Record current time in startTime variable
```

**Phase 2: Send Capture Command to Camera**
```
myCAM.takePicture(resolution, CAM_IMAGE_PIX_FMT_JPG)
    ↓
Send SPI command to camera module
    ↓
Camera sensor captures pixels into its buffer
    ↓
JPEG encoder compresses image data
    ↓
Return status code (CAM_ERR_SUCCESS or error code)
```

**Phase 3: Validate Capture**
```
If status != CAM_ERR_SUCCESS:
    Display: ✗ Capture FAILED!
             Error code: [status_value]
    Return (exit function)
```

**Phase 4: Wait for Processing**
```
delay(100ms) - Allow camera to finalize JPEG encoding
```

**Phase 5: Calculate Metrics**
```
captureTime = current_time - startTime
imageSize = myCAM.getTotalLength()  // Query camera for total bytes

Display statistics:
    ✓ IMAGE CAPTURED!
    ─────────────────────────────────────
    Capture time:   [timing] ms
    Image size:     [size] bytes
    Size (KB):      [size/1024] KB
```

**Phase 6: Handle Zero-Length Response**
```
If imageSize == 0:
    Set imageSize = 100000  (fallback estimate)
    Display: ⚠ Using estimated size (getTotalLength = 0)
```

**Phase 7: Prepare Serial Transmission**
```
Send markers: SIZE:[imageSize]
              START_BINARY_DATA
delay(100ms) - Flush buffers
```

**Phase 8: Stream Image Bytes**
```
Loop while bytesRead < imageSize:
    byte = myCAM.readByte()          // Read from camera buffer
    Serial.write(byte)               // Send to serial port
    bytesRead++
    
    If byte == 0xD9 AND prevByte == 0xFF:
        JPEG end marker found
        break (exit loop)
    
    If bytesRead > 200000:
        Safety limit reached
        break (exit loop)
    
    prevByte = byte
```

**Phase 9: End Transmission**
```
Send markers: END_IMAGE_DATA
              ─────────────────────────────────────
              (blank line)
```

#### Total Execution Time: Simple Capture

```
Display time:        ~5ms
SPI initialization:  ~10ms
Camera capture:      500-2000ms (depends on resolution)
JPEG encoding:       Included in capture time
delay(100):          100ms
Metrics calculation: ~5ms
Byte transmission:   30-200ms (depends on image size and baud rate)
─────────────────────────────────────
Total per image:     650-2300ms (0.65-2.3 seconds)
```

### Capture Process Detailed Example

For a VGA (640×480) capture at MEDIUM quality:

```
Command 'c' received
    ↓ [1ms]
Execute captureImage(CAM_IMAGE_MODE_VGA)
    ↓ [0.5ms]
Display: ┌─────────────────────────────────────┐
         │  CAPTURING: VGA (640x480)           │
         └─────────────────────────────────────┘
    ↓ [800ms - CAMERA CAPTURES AND ENCODES]
Camera returns success status
    ↓ [100ms - delay]
Query imageSize → 45,320 bytes
    ↓ [5ms]
Display statistics
    ↓ [100ms - delay]
Send SIZE:45320
     START_BINARY_DATA
    ↓ [120ms - STREAM BYTES OVER SERIAL]
Serial streams 45,320 bytes at 115200 baud
    ↓ [5ms]
Send END_IMAGE_DATA
    ↓ [10ms loop delay]
Return to main loop

Total time: ~1140ms (1.14 seconds)
```

---

## Azure Upload Process

### Capture and Upload (captureAndUpload function)

Used when commands 'u', 'u1', 'u2', 'u3', or 'u4' are entered.

#### Step-by-Step Execution

**Phase 1: Display Operation Banner**
```
Display: ┌─────────────────────────────────────┐
         │  CAPTURE + AZURE UPLOAD            │
         └─────────────────────────────────────┘
```

**Phase 2: Capture Image (same as captureImage function)**
```
myCAM.takePicture(resolution, CAM_IMAGE_PIX_FMT_JPG)
    ↓
Validate capture success
    ├─ FAIL → Display error and return
    └─ SUCCESS → Continue
    ↓
delay(100ms)
    ↓
Get imageSize from camera
    ↓
Validate imageSize > 0
    ├─ FAIL → Display error and return
    └─ SUCCESS → Continue
```

**Phase 3: Ensure WiFi Connection**
```
Call ensureWifi()
    ├─ If already connected → Return true immediately
    └─ If not connected:
        Display: Connecting to WiFi: [SSID]
            ↓
        WiFi.mode(WIFI_STA)
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD)
            ↓
        Loop for up to 20 seconds:
            Check WiFi.status() == WL_CONNECTED
            Display progress dots (.)
            Wait 500ms between checks
            ↓
        If connected: Return true, display IP
        If timeout: Return false, display error
```

**Phase 4: Build Unique Filename**
```
captureCounter++  (now 1, 2, 3, etc.)
timestamp = millis()
azureBlobPath = "/{CONTAINER}/image_{counter}_{timestamp}.jpg?{SAS_TOKEN}"

Example: /mycontainer/image_5_1234567890.jpg?sv=2023-...&sig=...
```

**Phase 5: Establish HTTPS Connection**
```
Create WiFiClientSecure object
    ↓
Set insecure mode (no certificate verification)
    ↓
client.connect(azureBlobHost, 443)  // Azure blob storage HTTPS
    ├─ SUCCESS → Display: ✓ Connected to Azure
    └─ FAIL → Display: ✗ Connection to Azure failed → Return
```

**Phase 6: Send HTTP PUT Request Headers**
```
Send to socket:
    PUT /mycontainer/image_5_1234567890.jpg?... HTTP/1.1
    Host: mystorageaccount.blob.core.windows.net
    Content-Type: image/jpeg
    Content-Length: 65432
    x-ms-blob-type: BlockBlob
    Connection: close
    [empty line]

(At this point, HTTP headers sent but no body data yet)
```

**Phase 7: Stream Image Bytes with Buffering**
```
Initialize:
    sent = 0
    prevByte = 0
    BUFFER_SIZE = 4096 bytes
    Create buffer array[4096]
    bufferPos = 0

Display: Streaming image bytes...

Loop while sent < imageSize:
    byte = myCAM.readByte()
    buffer[bufferPos++] = byte
    sent++
    
    If bufferPos >= 4096 OR sent >= imageSize:
        client.write(buffer, bufferPos)  // Flush to socket
        bufferPos = 0
    
    If sent % 5000 == 0:
        Display: .  (progress indicator)
    
    If prevByte == 0xFF AND byte == 0xD9:  (JPEG end marker)
        Display: ✓ Found JPEG end marker
        Flush remaining buffer
        Pad rest with zeros to match Content-Length
        break (exit loop)
    
    prevByte = byte
```

**Phase 8: Complete Transmission**
```
Flush any remaining buffer contents
    ↓
Display: Sent [bytes_sent] bytes
    ↓
client.flush()  // Ensure OS sends all data
    ↓
Display: Flushed data, waiting for response...
```

**Phase 9: Wait for Azure Response**
```
Set timeout to current_time + 10 seconds

Loop while !client.available() AND time < timeout:
    delay(50ms)
    Check again

If timeout reached:
    Display: ✗ No response from server (timeout)
    client.stop()
    Return (exit function)
```

**Phase 10: Parse Azure Response**
```
Read first line of HTTP response (status line)
    Example: HTTP/1.1 201 Created

Trim whitespace

Read remaining response headers/body for debugging
    Display each non-empty line with indentation
```

**Phase 11: Verify Upload Success**
```
If statusLine.startsWith("HTTP/1.1 201"):
    Display: ✓ Upload success (201 Created)
             Filename: image_5_1234567890.jpg
    success = true
Else:
    Display: ✗ Upload did not return 201 Created
    success = false
```

**Phase 12: Cleanup**
```
client.stop()  // Close HTTPS connection
    ↓
Display: [blank line]
    ↓
Return to main loop
```

#### Total Execution Time: Azure Upload

```
Display banner:          ~5ms
Camera capture:          500-2000ms
WiFi connection:         0ms (if already connected) or 2-15 seconds (first connection)
Build filename:          ~2ms
HTTPS connection:        200-500ms
Send headers:            ~10ms
Stream image bytes:      200-2000ms (depends on file size)
Wait for response:       1-10 seconds
Parse response:          ~5ms
─────────────────────────────────────
Total per upload:        700-30000ms (0.7-30 seconds)

Typical scenario (WiFi already connected, 45KB VGA):
                         ~2500ms (2.5 seconds)
```

---

## WiFi Connection Flow

### Connection State Management

The ESP32 maintains WiFi connection state across loop iterations:

```
Initial state after power-on: DISCONNECTED

ensureWifi() called:
    ↓
Check WiFi.status() == WL_CONNECTED
    ├─ YES → Return true (instant return)
    └─ NO → Attempt connection

Connection attempt:
    WiFi.mode(WIFI_STA)
    WiFi.begin(SSID, PASSWORD)
    
    Loop for 20 seconds:
        Check status every 500ms
        Display progress dots
    
    If connected before timeout:
        Return true
        State: CONNECTED (remains connected until disconnected)
    Else:
        Return false
        State: FAILED (retry connection on next ensureWifi call)
```

### Connection Persistence

Once WiFi is connected:
- **Remains connected** between uploads
- **No re-authentication required** for subsequent commands
- **Next ensureWifi() call returns immediately**
- If connection drops (signal loss, etc.), reconnection is automatic on next upload

### Connection Timing

```
First upload (WiFi not connected):
    ensureWifi() called
    WiFi connection: 2-15 seconds (depends on signal, distance, interference)
    Upload: 1-30 seconds
    Total: 3-45 seconds
    
Subsequent uploads (WiFi connected):
    ensureWifi() returns immediately
    Upload: 1-30 seconds
    Total: 1-30 seconds (no WiFi delay)
```

### Connection State Display

```
"Connecting to WiFi: MySSID"    // Connection attempt begins
"."                              // Connection in progress (every 500ms)
"."
"."
"✓ WiFi connected. IP: 192.168.1.42"  // Success
```

---

## Error Handling & Recovery

### Camera Capture Errors

**Error Scenario 1: Camera Not Responding**

```
Command: 'c' (capture)
    ↓
myCAM.takePicture() called
    ↓
SPI communication with camera fails
    ↓
Returns error status (not CAM_ERR_SUCCESS)
    ↓
Display: ✗ Capture FAILED!
         Error code: [error_code]
    ↓
Function returns (no image to display)
    ↓
Ready for next command

Recovery: 
    - Check SPI pin connections
    - Verify camera power supply
    - Try capture again
```

**Error Scenario 2: Image Size = 0**

```
Command: 'c' (capture)
    ↓
Capture succeeds
    ↓
myCAM.getTotalLength() returns 0
    ↓
Display: ⚠ Using estimated size (getTotalLength = 0)
    ↓
Set imageSize = 100,000 bytes (estimate)
    ↓
Continue with capture (fallback behavior)

Recovery:
    - Image data may be corrupted
    - Estimated size may be inaccurate
    - Try capturing again
```

### WiFi Connection Errors

**Error Scenario 1: WiFi Not Available**

```
Command: 'u' (upload)
    ↓
ensureWifi() called
    ↓
WiFi.begin(SSID, PASSWORD) executes
    ↓
Device cannot find SSID after 20 seconds
    ↓
Display: ✗ WiFi connect failed
    ↓
ensureWifi() returns false
    ↓
captureAndUpload() detects false return
    ↓
Display: ✗ WiFi failed
    ↓
Function returns (no upload)
    ↓
Ready for next command

Recovery:
    - Check SSID is correct in io_config.h
    - Check WiFi password is correct
    - Verify WiFi router is powered on
    - Check signal strength (move closer to router)
    - Restart device
```

### Azure Upload Errors

**Error Scenario 1: Azure Connection Failed**

```
Command: 'u4' (upload 3MP)
    ↓
Image captured successfully
    ↓
WiFi connected successfully
    ↓
Attempt HTTPS connection to Azure
    ↓
client.connect() fails
    ↓
Display: ✗ Connection to Azure failed
    ↓
Function returns
    ↓
Ready for next command

Recovery:
    - Check storage account name (in io_config.h)
    - Verify Azure storage account exists
    - Check internet connectivity
    - Verify SAS token is valid (not expired)
```

**Error Scenario 2: Upload Response Timeout**

```
Command: 'u2' (upload VGA)
    ↓
Image and headers sent successfully
    ↓
Wait up to 10 seconds for Azure response
    ↓
No response received within 10 seconds
    ↓
Display: ✗ No response from server (timeout)
    ↓
client.stop() (close connection)
    ↓
Function returns
    ↓
Ready for next command

Recovery:
    - Check internet bandwidth
    - Reduce image resolution (try QVGA instead)
    - Try again (may be temporary Azure delay)
    - Check Azure service status
```

**Error Scenario 3: HTTP Error Response**

```
Command: 'u' (upload VGA)
    ↓
Image uploaded successfully
    ↓
Azure responds with: HTTP/1.1 401 Unauthorized
    ↓
Display: Azure response: HTTP/1.1 401 Unauthorized
         ✗ Upload did not return 201 Created
    ↓
Function returns
    ↓
Ready for next command

Common HTTP responses:
    201 Created      → Upload successful
    400 Bad Request  → Invalid request format
    401 Unauthorized → SAS token expired or invalid
    403 Forbidden    → Storage account access denied
    404 Not Found    → Container does not exist
    500 Server Error → Azure service error

Recovery:
    - Check SAS token (io_config.h)
    - Regenerate SAS token if expired
    - Verify container name
    - Check storage account permissions
```

### Progressive Degradation

When errors occur:
1. **Capture errors** → Function returns, no image transmitted
2. **WiFi errors** → Function returns, camera has image but not uploaded
3. **Upload errors** → Image may be partially sent or not sent

**No cascading failures** - error in one function doesn't affect next command.

---

## Execution Examples

### Example 1: Basic Image Capture

**User Action:** Open serial monitor → Type 'c' → Press Enter

```
Serial input: 'c'
    ↓ [via loop()]
Recognized as capture command
    ↓
captureImage(CAM_IMAGE_MODE_VGA) called
    
Serial output:
    ┌─────────────────────────────────────┐
    │  CAPTURING: VGA (640x480)           │
    └─────────────────────────────────────┘
    [~500ms - camera captures]
    
    ✓ IMAGE CAPTURED!
    ─────────────────────────────────────
    Capture time:  582 ms
    Image size:    45682 bytes
    Size (KB):     44.61 KB
    
    SIZE:45682
    START_BINARY_DATA
    [binary JPEG data - 45682 bytes of image data]
    END_IMAGE_DATA
    ─────────────────────────────────────
    
Time elapsed: ~1.1 seconds

User can now:
    - Type 'q' to increase quality
    - Type '4' to capture 3MP instead
    - Type 'u' to upload this image (NO - will capture new image first)
```

### Example 2: Set Quality and Capture 3MP

**User Action:** Type 'q' → Wait → Type '4'

```
Serial input: 'q'
    ↓
Quality set to HIGH
Serial output:
    ✓ Quality: HIGH (more compression, smaller files)

User waits 2 seconds...

Serial input: '4'
    ↓
captureImage(CAM_IMAGE_MODE_QXGA) called
    
Serial output:
    ┌─────────────────────────────────────┐
    │  CAPTURING: 3MP (2048x1536)         │
    └─────────────────────────────────────┘
    [~1500ms - 3MP capture takes longer]
    
    ✓ IMAGE CAPTURED!
    ─────────────────────────────────────
    Capture time:  1523 ms
    Image size:    89456 bytes
    Size (KB):     87.36 KB
    
    SIZE:89456
    START_BINARY_DATA
    [binary JPEG data]
    END_IMAGE_DATA
    ─────────────────────────────────────

Time elapsed: ~2 seconds
Note: File is LARGER with HIGH quality than expected (opposite of typical)
      This can happen with cameras - pixel density and quality settings interact
```

### Example 3: Capture and Upload to Azure

**User Action:** Type 'u3' (capture 1080p and upload)

```
Serial input: 'u3'
    ↓
captureAndUpload(CAM_IMAGE_MODE_FHD) called

Serial output:
    ┌─────────────────────────────────────┐
    │  CAPTURE + AZURE UPLOAD            │
    └─────────────────────────────────────┘
    [~800ms - capture 1080p]
    
    Image size: 67823 bytes
    Connecting to WiFi: MyWiFiNetwork
    ...... [connected on 3rd attempt]
    ✓ WiFi connected. IP: 192.168.1.42
    
    Connecting to Azure host: mystorageaccount.blob.core.windows.net
    ✓ Connected to Azure
    Sending PUT request...
    Streaming image bytes...
    ..... [streaming 67823 bytes]
    ✓ Found JPEG end marker
    
    Sent 67823 bytes
    Flushed data, waiting for response...
    [~3 seconds waiting for Azure response]
    
    Azure response: HTTP/1.1 201 Created
    ✓ Upload success (201 Created)
      Filename: image_3_2847293.jpg

Time elapsed: ~5-6 seconds (includes WiFi connection time)
```

### Example 4: Quick Sequential Uploads

**User Action:** Type 'u1' → Wait → Type 'u2' → Wait → Type 'u3'

```
First command 'u1':
    Image size: 12456 bytes (QVGA - smallest)
    WiFi status: Not connected
    WiFi connection time: ~8 seconds (first connection)
    Upload time: ~1 second
    Total: ~9 seconds
    Result: image_1_2847293.jpg uploaded

Second command 'u2':
    Image size: 45682 bytes (VGA)
    WiFi status: Already connected (instant)
    Upload time: ~2 seconds
    Total: ~2.5 seconds
    Result: image_2_2851234.jpg uploaded

Third command 'u3':
    Image size: 67823 bytes (1080p)
    WiFi status: Still connected (instant)
    Upload time: ~3 seconds
    Total: ~3.5 seconds
    Result: image_3_2855678.jpg uploaded

Total time for 3 uploads: ~15 seconds (instead of ~25 if WiFi reconnected each time)
```

### Example 5: Error Recovery

**User Action:** Type 'u' but WiFi password is wrong in config

```
Serial input: 'u'
    ↓
captureAndUpload(CAM_IMAGE_MODE_VGA) called

Serial output:
    ┌─────────────────────────────────────┐
    │  CAPTURE + AZURE UPLOAD            │
    └─────────────────────────────────────┘
    
    Image size: 45682 bytes
    Connecting to WiFi: MyWiFiNetwork
    ........ [20 attempts over 10 seconds]
    ✗ WiFi connect failed
    
    ✗ WiFi failed
    
[Function returns, no upload attempted]

Recovery action:
    1. Fix WiFi password in io_config.h
    2. Recompile and upload sketch to ESP32
    3. Restart device
    4. Try again: Type 'u'
    
During this time:
    - Image was captured but not uploaded
    - Device is ready for next command
    - Next 'u' command will trigger new capture
```

---

## Timing Information

### Component Timing Breakdown

#### Camera Operations
| Operation | Min | Typ | Max | Notes |
|-----------|-----|-----|-----|-------|
| SPI initialization | 1ms | 2ms | 5ms | One-time setup |
| Camera initialization | 50ms | 100ms | 200ms | One-time setup |
| QVGA capture | 300ms | 500ms | 800ms | Smallest resolution |
| VGA capture | 400ms | 700ms | 1000ms | Common resolution |
| 1080p capture | 800ms | 1200ms | 1500ms | Full HD |
| 3MP capture | 1000ms | 1500ms | 2000ms | Largest resolution |
| JPEG encoding | Included in capture time | | | Real-time during capture |

#### Serial Communication
| Operation | Baud Rate | Speed |
|-----------|-----------|-------|
| Send 1 byte | 115200 | ~87 bytes/ms |
| Send 45KB image | 115200 | ~520ms |
| Send 90KB image | 115200 | ~1040ms |

#### WiFi Operations
| Operation | Condition | Min | Typ | Max | Notes |
|-----------|-----------|-----|-----|-----|-------|
| Already connected check | (instant return) | 0.1ms | 0.1ms | 0.5ms | Negligible |
| First-time connection | Good signal | 2s | 8s | 20s | Timeout |
| First-time connection | Poor signal | 5s | 15s | 20s | May timeout |
| Reconnection | Previous good state | 0.5s | 2s | 5s | Much faster |

#### HTTPS/Azure Operations
| Operation | Condition | Min | Typ | Max | Notes |
|-----------|-----------|-----|-----|-----|-------|
| HTTPS handshake | Good connection | 100ms | 300ms | 1000ms | SSL/TLS negotiation |
| Send headers | (fixed size) | 5ms | 10ms | 20ms | ~300 bytes |
| Stream 12KB | (QVGA) | 200ms | 500ms | 1000ms | Network dependent |
| Stream 45KB | (VGA) | 500ms | 1000ms | 2000ms | Network dependent |
| Stream 67KB | (1080p) | 800ms | 1500ms | 3000ms | Network dependent |
| Stream 90KB | (3MP) | 1000ms | 2000ms | 4000ms | Network dependent |
| Receive response | (depends on Azure) | 500ms | 3s | 10s | Timeout after 10s |

### Typical Complete Workflows

#### Workflow 1: Single Local Capture
```
Time: 0s    - Command 'c' entered
Time: 1.1s  - Capture complete, image displayed
Time: 1.1s+ - Ready for next command
Total: ~1.1 seconds
```

#### Workflow 2: First-Time Upload (WiFi disconnected)
```
Time: 0s    - Command 'u' entered
Time: 0.8s  - Image captured
Time: 0.8s  - WiFi connection starts
Time: 8.8s  - WiFi connected
Time: 9.0s  - HTTPS handshake
Time: 10.0s - Image bytes sent
Time: 13.0s - Response received
Time: 13.1s - Complete

Total: ~13 seconds
```

#### Workflow 3: Subsequent Uploads (WiFi connected)
```
Time: 0s    - Command 'u' entered
Time: 0.8s  - Image captured
Time: 0.8s  - WiFi check (instant - already connected)
Time: 0.9s  - HTTPS handshake
Time: 1.9s  - Image bytes sent
Time: 4.9s  - Response received
Time: 5.0s  - Complete

Total: ~5 seconds
```

#### Workflow 4: Three Rapid Uploads
```
Time: 0s    - Command 'u1'
Time: 2.0s  - Command 'u2'
Time: 4.5s  - Command 'u3'
    ↓
Execution (first upload slower due to WiFi):
Time: 0-12s   - First upload (u1) completes
Time: 12-14s  - Second upload (u2) completes (no WiFi wait)
Time: 14-17s  - Third upload (u3) completes (no WiFi wait)

Total: ~17 seconds for 3 uploads
(vs ~39 seconds if WiFi reconnected each time)
```

---

## State Management

### Global State Variables

#### 1. captureCounter
```
Type: static uint32_t
Initial value: 0
Modified: Incremented before each upload
Purpose: Creates unique filenames (image_1, image_2, etc)
Scope: Persists across all function calls
Behavior: 
    - Only incremented on successful Azure upload
    - NOT incremented on local capture
    - Can overflow (wraps around at max uint32_t)
    - Restarts at 0 on device restart
```

**Example Sequence:**
```
Power on:     captureCounter = 0
Command 'c':  captureCounter = 0 (not incremented)
Command 'u':  captureCounter = 1 (uploaded as image_1)
Command 'u':  captureCounter = 2 (uploaded as image_2)
Command 'c':  captureCounter = 2 (not changed)
Command 'u':  captureCounter = 3 (uploaded as image_3)
Device reset: captureCounter = 0 (resets on power cycle)
```

#### 2. Camera Quality Setting
```
Type: myCAM.setImageQuality() setting
Initial value: DEFAULT_QUALITY
Modified: When user enters 'q', 'w', or 'e'
Purpose: Controls JPEG compression ratio
Options: HIGH_QUALITY, DEFAULT_QUALITY, LOW_QUALITY
Scope: Affects all subsequent captures
Behavior:
    - Persists until changed by user
    - Does NOT affect already-captured images
    - Takes effect immediately on next capture
```

**Example Sequence:**
```
Power on:         Quality = MEDIUM (default)
Command 'q':      Quality = HIGH
Command 'c':      Capture with HIGH quality (smaller file)
Command 'e':      Quality = LOW
Command 'u':      Capture with LOW quality (larger file) then upload
Command 'w':      Quality = MEDIUM
Command 'c':      Capture with MEDIUM quality
```

#### 3. WiFi Connection State
```
Type: ESP32 WiFi module internal state
Managed by: ensureWifi() function
Initial state: Disconnected (after power-on)
State transitions:
    Disconnected → (ensureWifi called) → Connecting → Connected
                                              ↓
                                         (timeout) → Failed
Behavior:
    - Persists across multiple upload commands
    - Only reconnects if current connection lost
    - No user-accessible state variable
    - Can be monitored via WiFi.status()
```

**Example Sequence:**
```
Power on:           WiFi = Disconnected
Command 'u':        ensureWifi() → WiFi = Connected (8 seconds)
Command 'u':        ensureWifi() → WiFi = Connected (instant, no reconnection)
Command 'u':        ensureWifi() → WiFi = Connected (instant)
[After 10 minutes] 
Command 'u':        WiFi might have disconnected → Reconnects (8 seconds)
```

#### 4. Serial Buffer State
```
Type: ESP32 UART hardware buffer
Size: Typically 128-256 bytes
Managed by: Arduino Serial object (automatic)
Behavior:
    - Receives characters from serial port
    - Stored until loop() reads them
    - Characters read one-at-a-time with Serial.read()
    - If buffer fills, oldest characters are lost
    - Not accessible to user code (hardware-managed)
```

**Example of Buffer Overflow:**
```
Scenario: User types very fast without waiting for responses
Input: "u1u2u3u4u1u2u3u4" quickly
Available buffer: 128 bytes

First 128 characters stored
Remaining characters: Lost (buffer full)

Processing:
Command 'u1': Execute (10+ seconds)
Command 'u2': Execute (5 seconds)
... commands continue as they become available
... but some typed commands were lost in buffer overflow
```

### Function State (Execution-Specific)

#### Local Variables in captureImage()
```
resName     - Resolution name string (created, used, discarded)
startTime   - Recorded at function entry
captureTime - Calculated from startTime
imageSize   - Retrieved from camera
bytesRead   - Tracks byte count during streaming
prevByte    - Previous byte (for JPEG end-marker detection)
foundEnd    - Boolean flag (true if JPEG end found)
```

#### Local Variables in captureAndUpload()
```
status      - Camera capture status
imageSize   - Captured image byte count
captureCounter - INCREMENTED globally
timestamp   - millis() at function entry
azureBlobPath - Full Azure path with SAS token
client      - WiFiClientSecure object
sent        - Bytes sent to Azure
prevByte    - Previous byte (for JPEG end-marker detection)
BUFFER_SIZE - 4096 bytes (constant)
buffer      - 4KB temporary buffer for streaming
bufferPos   - Position in temporary buffer
success     - Boolean result of upload
statusLine  - HTTP response status line
```

---

## Advanced Topics

### Memory Considerations

**ESP32 RAM Typical: 4-16 MB**

```
Memory usage by component:
    Sketch code:           ~100KB
    Global variables:      ~1KB
    Image buffer (camera): ~256KB (managed by camera module)
    Streaming buffer:      4KB (in captureAndUpload)
    WiFi stack:            ~80KB (when connected)
    Serial buffer:         256 bytes
    Stack (temporary):     ~50KB used during execution
    
Total typical usage: ~500KB
Available for other uses: 3.5-15.5MB

Memory never required all at once - image is streamed, not stored
```

### Buffer Behavior During Streaming

**Efficient streaming prevents memory exhaustion:**

```
Image = 90KB
Without buffering (inefficient):
    Load entire 90KB into RAM → 90KB memory required
    
With 4KB buffering (actual code):
    Load 4KB chunk → Stream → Discard
    Load 4KB chunk → Stream → Discard
    ... repeat 22 times
    Maximum memory for image: 4KB only!
```

### JPEG End-Marker Detection

**Why detect 0xFF 0xD9?**

```
JPEG Format:
    [SOI marker: 0xFF 0xD8]
    [Image data]
    [Metadata]
    [End marker: 0xFF 0xD9] ← This signals end of JPEG

Camera behavior:
    May return imageSize that includes extra padding bytes
    Actual image data ends at 0xFF 0xD9 marker
    Extra bytes are padding (not part of image)

Code action:
    Detects end marker
    Pads remaining bytes with 0x00 to match Content-Length
    Azure accepts full Content-Length bytes
```

### SAS Token Security

**SAS Token in Code:**
```
Stored in: io_config.h (firmware)
Visible to: Anyone with access to compiled firmware
Expiration: Typically 1-12 months from creation
Scope: Limited to specific container and operations

Security implications:
    ✓ No storage account key exposed (only limited SAS)
    ✓ HTTPS encrypts token during transmission
    ✗ Token embedded in firmware
    ✗ If device stolen, SAS could be extracted
    
Best practices:
    - Set SAS token expiration to ~30 days
    - Regularly regenerate SAS tokens
    - Use most restrictive permissions (write-only for uploads)
    - Monitor Azure storage account access logs
```

---

## Troubleshooting Decision Tree

### Problem: Nothing happens after typing command

```
Possible causes:
    1. Serial monitor not connected
       Solution: Close and reopen serial monitor
    
    2. Baud rate mismatch
       Solution: Set serial monitor to 115200 baud
    
    3. Character didn't transmit (serial issue)
       Solution: Try again, check USB cable
    
    4. Command being processed
       Solution: Wait (capture can take 1-2 seconds)
```

### Problem: Capture shows "FAILED"

```
Possible causes:
    1. Camera not powered
       Solution: Check power connections
    
    2. SPI pins not connected correctly
       Solution: Verify GPIO 18, 19, 23, 5
    
    3. Camera firmware issue
       Solution: Restart ESP32
    
    4. Conflict with other SPI devices
       Solution: Check if other SPI devices exist
```

### Problem: Upload fails "No response from server"

```
Possible causes:
    1. Internet connection dropped
       Solution: Check WiFi signal
    
    2. Azure service unreachable
       Solution: Verify storage account URL
    
    3. Firewall blocking HTTPS
       Solution: Check network firewall rules
    
    4. Image too large (timeout)
       Solution: Use smaller resolution (QVGA instead of 3MP)
```

---

## Summary

The Arducam 3MP camera system follows this execution model:

1. **Startup** → Initialize hardware, display commands (once)
2. **Main Loop** → Read serial input, process commands (forever)
3. **Capture** → Take image, display locally, ready for next command (~1 sec)
4. **Upload** → Capture, connect WiFi (if needed), send to Azure (~5-15 sec)
5. **Error Handling** → Display errors, return to ready state for next command

Key timing characteristics:
- **Simple capture**: 0.5-2 seconds
- **Upload (WiFi connected)**: 3-10 seconds
- **Upload (WiFi first-time)**: 10-30 seconds
- **Quality changes**: Immediate (next capture uses new setting)
- **Multiple uploads**: Reuse WiFi connection for 50% faster execution

The code is designed for reliability and user feedback, with every step displayed on the serial monitor for debugging and monitoring.
