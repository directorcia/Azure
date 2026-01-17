# Arducam Mega ESP32 → Azure Blob Storage (storage-web.cpp)

A practical guide to wiring, configuration, build, flashing, execution, and troubleshooting for the `storage-web.cpp` sketch, which captures JPEG images from an Arducam Mega on ESP32 and uploads them to Azure Blob Storage via HTTPS.

## Overview
- Captures a JPEG image at 3MP (QXGA) on a timed interval.
- Uploads the image to Azure Blob Storage using an HTTPS `PUT` request with SAS token.
- Optionally overwrites a fixed blob name (e.g., `latest.jpg`) for easy web access.
- Includes a serial-streaming helper (`captureImage()`) for host-side ingestion.

## Hardware
- Board: ESP32 dev board (e.g., DOIT, NodeMCU-32S)
- Camera: Arducam Mega
- Optional sensor: Noise sensor on GPIO 34 (input-only) — configured but not used in timed mode.
- Status LED: GPIO 2 (reserved/unused)

### Wiring (VSPI)
- MOSI: GPIO 23
- MISO: GPIO 19
- SCK : GPIO 18
- CS  : GPIO 5

### Optional Noise Sensor (GPIO 34)
GPIO 34 is input-only. If your sensor outputs 5V, use a voltage divider:
```
D0 (5V) --[10k]--+--[10k]-- GND
                 |
               GPIO34 (reads ~2.5V when D0 = 5V)
```
This project uses timed capture, so noise input is not used for triggering.

## Software Prerequisites
- Arduino IDE (ESP32 board support) or Arduino CLI
- Arducam Mega library
- ESP32 core libraries (`WiFi`, `WiFiClientSecure`)

## Configuration (io_config.h)
Create or update `io_config.h` (already referenced by the sketch) with:
```c
#define WIFI_SSID        "<your-ssid>"
#define WIFI_PASSWORD    "<your-password>"

#define AZURE_STORAGE_ACCOUNT "<storage-account-name>"  // e.g., mystorageacct
#define AZURE_CONTAINER       "<container-name>"        // e.g., images
#define AZURE_SAS_TOKEN       "<sas-query-string>"      // without leading '?'
```
Notes:
- `AZURE_SAS_TOKEN` should include permissions, expiry, and signature (e.g., `sv=...&ss=b&srt=...&sp=w&se=...&sig=...`).
- For overwriting a fixed blob (e.g., `latest.jpg`), ensure SAS includes write permission `sp=w`.

## Azure Prerequisites
- An Azure Storage account with a Blob container (e.g., `images`).
- A SAS token granting write permission to the target container.
  - Minimum: `sp=w` (Write). If creating new blobs only, `sp=c` (Create) may be needed for certain flows; `sp=w` typically covers create/overwrite with `PUT`.
  - Scope: Container or Blob-level.
  - Include `se` (expiry), `sv` (service version), `sr` (resource), and `sig` (signature).
- Network access from the ESP32 to `https://<account>.blob.core.windows.net` port 443.

## Build & Flash

### Arduino IDE
1. Install ESP32 boards via Board Manager.
2. Install Arducam Mega library.
3. Open `storage-web.cpp` in Arduino IDE or include it in a sketch.
4. Select your ESP32 board and COM port.
5. Verify and Upload.

### Arduino CLI (example)
Assuming you have a sketch folder with `storage-web.cpp` and `io_config.h`:
```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "ArduCAM"
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload  --fqbn esp32:esp32:esp32 -p COM3 .
```
Adjust `--fqbn` and `-p` for your board/port.

## Runtime Operation
- On boot, the sketch initializes SPI and the camera, sets high JPEG quality, configures ADC, and attempts Wi‑Fi connection.
- Every `CAPTURE_INTERVAL_MS` (default 60000 ms), it:
  - Captures JPEG at QXGA (3MP).
  - Builds the blob path:
    - Fixed name mode: `latest.jpg` (overwrite each cycle).
    - Unique name mode: `image_<counter>_<millis>.jpg`.
  - Opens TLS (`WiFiClientSecure`) and sends an HTTP `PUT` with headers:
    - `Content-Type: image/jpeg`
    - `Content-Length: <camera-reported-size>`
    - `x-ms-blob-type: BlockBlob`
  - Streams JPEG bytes; when the JPEG end marker (`0xFF 0xD9`) is found, pads zeros until `Content-Length` is met.
  - Awaits response; expects `HTTP/1.1 201 Created`.

### Serial Monitoring
Use the Serial Monitor at 115200 baud. You will see logs for:
- Camera init and resolution
- Wi‑Fi connection and IP
- Upload connection and PUT request
- Byte streaming progress and response status

### Switching Filename Behavior
- Fixed name (overwrite): The loop calls `captureAndUpload(CAM_IMAGE_MODE_QXGA, true)`, producing `latest.jpg`.
- Unique names: Change this parameter to `false` to keep each image.

## Verifying Uploads
You can check the blob via a browser or `curl` (do not expose SAS publicly):
```bash
curl -I "https://<account>.blob.core.windows.net/<container>/latest.jpg?<sas>"
```
A successful upload should return `HTTP/1.1 200 OK` on `HEAD` and `Content-Type: image/jpeg`.

## Security Notes
- `WiFiClientSecure::setInsecure()` disables certificate validation. For production, consider:
  - Using proper root CA trust or certificate pinning.
  - Managing SAS secrets securely (short expiry, least privileges).
- Avoid printing full SAS tokens in public logs.

## Adjustments & Tunables
- `CAPTURE_INTERVAL_MS`: capture cadence (default 60000 ms).
- Image quality: `myCAM.setImageQuality(HIGH_QUALITY)`; adjust if bandwidth is limited.
- Resolution: use other `CAM_IMAGE_MODE_*` constants (e.g., VGA) if needed.
- Noise sensor thresholds (if later used): `NOISE_ANALOG_HIGH/LOW`, hysteresis, `NOISE_MARGIN`.

## Troubleshooting
- Wi‑Fi fails to connect:
  - Verify SSID/password; check network 2.4GHz availability.
  - Confirm device receives an IP and has Internet connectivity.
- Azure connection fails (`client.connect`):
  - Check firewall rules to `*.blob.core.windows.net:443`.
  - Ensure correct storage account DNS and connectivity.
- `HTTP/1.1 403` (Forbidden):
  - SAS token invalid/expired or missing required permission (`sp=w`).
  - Container name typo or insufficient scope.
- `HTTP/1.1 404` (Not Found):
  - Container does not exist; verify `AZURE_CONTAINER`.
- `HTTP/1.1 400` (Bad Request) or `411 Length Required`:
  - Mismatch between streamed bytes and `Content-Length`. Verify camera length and padding logic.
- `HTTP/1.1 201 Created` not received:
  - Check serial logs for request and ensure SAS query string is appended.

## Maintenance
- Rotate SAS tokens regularly; keep expiry (`se`) short.
- Monitor upload outcomes via Serial and Azure metrics.
- Consider adding retries/backoff for transient network issues.
- For production hardening:
  - Replace `setInsecure()` with proper certificate handling.
  - Add time sync (NTP) for accurate timestamps.
  - Implement error telemetry.

## Alternate Flow: Serial Image Streaming
`captureImage()` emits:
```
SIZE:<bytes>
START_BINARY_DATA
<raw JPEG bytes>
END_IMAGE_DATA
```
This is useful for host tools to read images via serial instead of Azure upload.

## File References
- Code: `storage-web.cpp`
- Config: `io_config.h`
- This guide: `README-storage-web.md`
