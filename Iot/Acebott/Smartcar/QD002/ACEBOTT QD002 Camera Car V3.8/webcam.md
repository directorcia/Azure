# QD002 ESP32-CAM WebServer Documentation

## Overview
This code implements a web-based camera streaming server for the ESP32-CAM module on the ACEBOTT QD002 Camera Car. It provides real-time MJPEG video streaming and still image capture capabilities through a web interface.

## Hardware Configuration

### ESP32-CAM Module (AI-Thinker)
The code is configured for the AI-Thinker ESP32-CAM board with the following pin mapping:

| Pin Function | GPIO Pin |
|-------------|----------|
| Power Down | 32 |
| Reset | -1 (not used) |
| XCLK | 0 |
| SIOD (SDA) | 26 |
| SIOC (SCL) | 27 |
| Y9 | 35 |
| Y8 | 34 |
| Y7 | 39 |
| Y6 | 36 |
| Y5 | 21 |
| Y4 | 19 |
| Y3 | 18 |
| Y2 | 5 |
| VSYNC | 25 |
| HREF | 23 |
| PCLK | 22 |

## Operation Flow

### 1. System Initialization (`setup()`)

#### Serial Communication Setup
- Baud rate: 115200
- Displays startup banner to Serial Monitor

#### Camera Configuration
The camera is configured with adaptive settings based on available PSRAM:

**With PSRAM:**
- Frame Size: VGA (640x480)
- JPEG Quality: 12 (higher quality, lower value = better)
- Frame Buffers: 2 (for smoother streaming)

**Without PSRAM:**
- Frame Size: QVGA (320x240)
- JPEG Quality: 15
- Frame Buffers: 1

**Fallback Configuration (if initialization fails):**
- Frame Size: QQVGA (160x120)
- JPEG Quality: 20
- Frame Buffers: 1

#### WiFi Connection (STA Mode)
- Operates in Station mode (connects to existing WiFi network)
- Credentials loaded from `io_config.h` (WIFI_SSID and WIFI_PASSWORD)
- Connection timeout: 10 seconds (20 attempts × 500ms)
- Upon success, displays assigned IP address

#### HTTP Server Startup
- Starts HTTP server on port 80
- Makes camera accessible via browser at the device's IP address

### 2. Main Loop (`loop()`)

The main loop continuously listens for HTTP client connections and routes requests to appropriate handlers.

#### Request Processing Flow:
1. Wait for client connection
2. Read HTTP request line (e.g., "GET /stream HTTP/1.1")
3. Skip remaining HTTP headers until blank line
4. Route request based on URL path:
   - `GET /` → Send index page
   - `GET /stream` → Start MJPEG stream
   - `GET /capture` → Capture single image
   - Other paths → Return 404 error
5. Close client connection

## Web Interface Endpoints

### 1. Index Page (`/`)
**Function:** `sendIndex()`

**Purpose:** Displays the main web interface

**Features:**
- Page title: "QD002 CAM"
- Link to streaming endpoint
- Embedded live stream viewer
- Link to capture still images

**HTTP Response:**
```
HTTP/1.1 200 OK
Content-Type: text/html
Connection: close
```

### 2. Live Stream (`/stream`)
**Function:** `sendStream()`

**Purpose:** Provides continuous MJPEG video stream

**Operation:**
1. Sends HTTP headers for multipart MIME stream
2. Enters continuous loop while client is connected
3. For each frame:
   - Captures frame buffer from camera
   - Sends multipart boundary marker
   - Sends JPEG image data
   - Returns frame buffer to pool
   - Delays 40ms (~25 FPS target)
4. Continues until client disconnects

**HTTP Response Headers:**
```
HTTP/1.1 200 OK
Content-Type: multipart/x-mixed-replace; boundary=frame
Cache-Control: no-cache, no-store, must-revalidate
Pragma: no-cache
Connection: close
```

**Stream Format:**
Each frame is sent with:
```
--frame
Content-Type: image/jpeg
Content-Length: <size>

<JPEG data>
```

**Frame Rate:** Approximately 25 FPS (40ms delay between frames)

### 3. Capture Still Image (`/capture`)
**Function:** `sendCapture()`

**Purpose:** Captures and returns a single JPEG image

**Operation:**
1. Requests frame buffer from camera
2. If successful:
   - Sends HTTP 200 OK response
   - Sends JPEG image data
   - Returns frame buffer
3. If failed:
   - Sends HTTP 500 error response

**HTTP Response:**
```
HTTP/1.1 200 OK
Content-Type: image/jpeg
Connection: close
Content-Length: <image size>
```

## Error Handling

### Camera Initialization Failures
1. **First attempt fails:** Tries fallback configuration (QQVGA, single buffer)
2. **Fallback fails:** 
   - Logs error code to Serial Monitor
   - Suggests checking ribbon cable and power
   - System remains in non-operational state

### WiFi Connection Failures
- **Timeout after 10 seconds:** 
  - Logs failure to Serial Monitor
  - Prompts user to check SSID/password configuration
  - System remains in non-operational state

### Frame Capture Failures
- **During streaming:** Breaks stream loop, closes connection
- **During still capture:** Returns HTTP 500 error to client

## Configuration Requirements

### External Dependencies
- **io_config.h** must define:
  - `WIFI_SSID` - WiFi network name
  - `WIFI_PASSWORD` - WiFi password

### Libraries Required
- `Arduino.h` - Arduino framework
- `WiFi.h` - WiFi connectivity
- `esp_camera.h` - ESP32 camera driver

## Usage Instructions

### 1. Setup
1. Ensure WiFi credentials are configured in `io_config.h`
2. Upload code to ESP32-CAM
3. Open Serial Monitor at 115200 baud
4. Note the IP address displayed after WiFi connection

### 2. Accessing the Camera
- **Main interface:** `http://<IP_ADDRESS>/`
- **Direct stream:** `http://<IP_ADDRESS>/stream`
- **Still capture:** `http://<IP_ADDRESS>/capture`

### 3. Viewing the Stream
- Open a web browser
- Navigate to the device's IP address
- The embedded stream will display automatically
- Stream can be accessed from multiple clients simultaneously (bandwidth permitting)

## Performance Characteristics

### Frame Rate
- **Target:** 25 FPS
- **Actual:** Depends on WiFi bandwidth, image quality, and client count

### Image Quality
- **VGA (640×480):** High quality, requires PSRAM
- **QVGA (320×240):** Medium quality, works without PSRAM
- **QQVGA (160×120):** Fallback, lowest quality

### Memory Usage
- **With PSRAM:** Uses 2 frame buffers for smoother streaming
- **Without PSRAM:** Single buffer, may have more frame drops

## Troubleshooting

### Camera Not Working
**Symptoms:** Error message on Serial Monitor during initialization

**Solutions:**
1. Check ribbon cable connection (ensure it's fully seated and not reversed)
2. Verify camera module has power
3. Try reducing frame size or quality settings
4. Check for loose connections

### WiFi Connection Fails
**Symptoms:** "WiFi connection failed" message

**Solutions:**
1. Verify WIFI_SSID and WIFI_PASSWORD in `io_config.h`
2. Ensure WiFi network is 2.4 GHz (ESP32 doesn't support 5 GHz)
3. Check WiFi signal strength
4. Verify router is broadcasting SSID

### Stream Stuttering or Low Frame Rate
**Symptoms:** Choppy video or slow updates

**Solutions:**
1. Reduce frame size (use QVGA instead of VGA)
2. Increase JPEG quality value (lower image quality, faster processing)
3. Reduce number of simultaneous clients
4. Improve WiFi signal strength
5. Check network bandwidth

### Can't Access Web Interface
**Symptoms:** Browser can't connect to IP address

**Solutions:**
1. Verify device is connected to WiFi (check Serial Monitor)
2. Confirm correct IP address
3. Ensure client device is on same WiFi network
4. Try accessing from different browser or device
5. Check firewall settings

## Technical Notes

### MJPEG Streaming
The code uses Motion JPEG (MJPEG) streaming, which sends individual JPEG frames over HTTP using multipart MIME encoding. This is simple and widely supported but uses more bandwidth than modern video codecs.

### Frame Buffer Management
The ESP32 camera driver maintains frame buffers in memory. After capturing or streaming each frame, `esp_camera_fb_return()` must be called to return the buffer to the pool.

### Connection Handling
Each HTTP request opens a new connection. The streaming endpoint keeps the connection open for continuous transmission, while index and capture endpoints close immediately after sending data.

### Clock Frequency
Camera XCLK is set to 20 MHz, which is optimal for most ESP32-CAM modules.

## Modifications and Extensions

### Possible Enhancements
1. **Authentication:** Add password protection for web interface
2. **Settings Page:** Allow runtime adjustment of frame size and quality
3. **Motion Detection:** Add alerts when motion is detected
4. **Recording:** Save images or video to SD card
5. **Pan/Tilt Control:** Add servo controls for camera positioning
6. **Night Vision:** Add IR LED control
7. **WebSocket:** Use WebSocket for more efficient streaming
8. **mDNS:** Add mDNS for friendly hostname (e.g., `http://qd002cam.local`)

### Performance Tuning
- Adjust `delay(40)` in stream loop to change frame rate
- Modify `jpeg_quality` for quality vs. speed tradeoff
- Change `frame_size` based on use case requirements
