# WiFi-Enabled IR-Controlled Acebott Smartcar — Execution & Operation Guide

This document explains the runtime behavior, control flow, hardware interactions, WiFi connectivity, web server operations, and API endpoints of the WiFi-enabled Acebott Smartcar program.

Source: [Iot/Acebott/Smartcar/with-web-serve.cpp](Iot/Acebott/Smartcar/with-web-serve.cpp)

## Overview
This program extends the basic IR-controlled robot with WiFi connectivity and a built-in web server, enabling dual control modes:
1. **IR Remote Control** — Physical infrared remote with button-to-action mappings
2. **Web Browser Control** — HTTP-based control panel accessible from any device on the network

The robot provides real-time status information via web UI, JSON API endpoints, and Serial output.

## Architecture

### Control Flow
```
WiFi Connection → Web Server Start → Main Loop
                                      ├─> Service HTTP Requests
                                      ├─> Monitor WiFi (auto-reconnect)
                                      └─> Process IR Commands
```

### Key Features
- Dual input: IR remote + HTTP/Web interface
- Auto-reconnect WiFi with configurable retry intervals
- Static IP or DHCP support
- Mobile-responsive web UI with real-time device stats
- JSON status endpoint for programmatic access
- Audio feedback on successful WiFi connection
- Non-blocking operation (IR and web requests handled concurrently)

## Hardware & Libraries

### MCU & Connectivity
- MCU: ESP32 (WiFi-capable)
- Libraries: `WiFi.h`, `WebServer.h`, `IRremote`, `ESP32Servo`, `vehicle`, `ultrasonic`
- Configuration: External `io_config.h` for WiFi credentials and network settings

### Pin Assignments
- IR receiver: `IR_RECEIVE_PIN = 4`
- Ultrasonic: `TRIG_PIN = 13`, `ECHO_PIN = 14`
- Servo: `SERVO_PIN = 25`
- LED: `LED_PIN = 2`
- Buzzer: `BUZZER_PIN = 33`

### Required Configuration File: `io_config.h`
Must define:
```cpp
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASSWORD "YourPassword"
#define WEB_SERVER_PORT 80  // or any available port

// Optional:
#define HOSTNAME "acebott-robot"
#define STATIC_IP "192.168.1.100"
#define GATEWAY_IP "192.168.1.1"
#define SUBNET_MASK "255.255.255.0"
#define DNS_SERVER "192.168.1.1"
```

## Global State

### Robot State
- `ledState` (bool): LED on/off status
- `buzzerState` (bool): Buzzer activation edge trigger
- `servoPosition` (int): Current servo angle (0–180°, default 90°)

### WiFi State
- `wifiConnectedOnce` (bool): Tracks if initial connection has succeeded (prevents repeated celebration tunes)
- `lastWifiAttemptMs` (unsigned long): Timestamp of last WiFi connection attempt
- `wifiRetryIntervalMs` (const): 10 seconds between reconnection attempts

### Server Objects
- `myCar` (vehicle): Motor/drivetrain controller
- `sensor` (ultrasonic): Distance sensor
- `scanServo` (Servo): Pan/tilt servo
- `server` (WebServer): HTTP server instance on configured port

## Startup Sequence (`setup()`)

Initialization follows this order:

1. **Serial Communication** — Start at 115200 baud, print startup banner
2. **Robot Hardware**
   - Initialize drivetrain: `myCar.Init()`
   - Initialize ultrasonic sensor: `sensor.Init(TRIG_PIN, ECHO_PIN)`
   - Attach servo to pin 25, center at 90°, wait 1s for settling
   - Configure LED and buzzer pins as outputs, set both LOW
3. **IR Receiver** — Initialize with `IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK)`
4. **WiFi Connection** — Call `tryConnectWiFi()` with 15s timeout
5. **Web Server** — Call `startWebServer()` to register routes and begin listening
6. **Ready State** — Print usage help to Serial

The robot is fully operational even if WiFi fails; IR control remains functional.

## Main Loop (`loop()`)

The loop performs three concurrent tasks:

### 1. WiFi Maintenance (Non-blocking)
- Checks connection status every iteration
- If disconnected and retry interval elapsed (10s), attempts brief reconnection (7s timeout)
- Does not block IR processing or web serving

### 2. HTTP Request Handling
- Calls `server.handleClient()` to process incoming web requests
- Dispatches to registered route handlers (`/`, `/status`, `/cmd`)

### 3. IR Command Processing
- Polls `IrReceiver.decode()` for new IR signals
- Decodes command and dispatches to action functions
- Calls `IrReceiver.resume()` to re-arm receiver
- Prints diagnostics for unmapped keys

### Loop Timing
- 100 ms delay at end for system stability
- All blocking operations (servo moves, buzzer tones) run within action handlers

## WiFi Connection Flow (`tryConnectWiFi()`)

### Configuration Precedence
1. **Hostname** — Sets via `WiFi.setHostname(HOSTNAME)` if defined
2. **Static IP** — If `STATIC_IP` defined, configures static networking with optional gateway/subnet/DNS
3. **DHCP Fallback** — If static config fails or not defined, uses DHCP

### Connection Process
1. Set WiFi mode to station (`WIFI_STA`)
2. Call `WiFi.begin(WIFI_SSID, WIFI_PASSWORD)`
3. Poll `WiFi.status()` every 250 ms until connected or timeout
4. On success:
   - Print IP address and signal strength (RSSI)
   - Play 3-tone ascending chirp if first connection (`wifiConnectedOnce` flag)
5. On timeout:
   - Print timeout message
   - System continues operation without network (IR still works)

### Auto-Reconnect Logic
- Main loop monitors `WiFi.status()` continuously
- If disconnected, waits 10s (`wifiRetryIntervalMs`) before next attempt
- Uses shorter timeout (7s) for retry attempts to minimize disruption
- Reconnection happens in background; does not halt IR or servo operations

## Web Server Architecture

### Server Initialization (`startWebServer()`)
Registers four routes:
- `GET /` → `handleRoot()` — Main control UI (HTML)
- `GET /status` → `handleStatusJson()` — JSON device info
- `GET /cmd?action=<action>` → `handleCommand()` — Execute robot command
- `*` → `handleNotFound()` — 404 handler

Server listens on port specified in `WEB_SERVER_PORT` (typically 80).

### Route: `GET /` (Web UI)

**Handler:** `handleRoot()`

Generates a mobile-responsive HTML5 control panel with:

#### Device Status Section
- Connection state (Connected/Disconnected)
- SSID
- IP address
- MAC address
- WiFi signal strength (RSSI in dBm)
- Hostname (if defined)
- System uptime (formatted as days:hours:minutes:seconds)

#### Control Panel
A 3×3 grid layout for directional control:
```
       ⬆️ Forward
⬅️ Left  ⏹️ Stop  ➡️ Right
       ⬇️ Back
```

#### Additional Controls (Horizontal Rows)
- Rotation: `↶ Rotate L`, `↷ Rotate R`
- Servo: `📷⬅️ Servo L`, `📷⚫ Center`, `📷➡️ Servo R`
- Utilities: `💡 LED`, `🔊 Buzzer`, `📏 Distance`

#### JavaScript Client Logic
- Each button calls `cmd(action)` function
- Sends AJAX `fetch()` to `/cmd?action=<action>`
- Displays response in alert box
- No page refresh required (single-page control)

#### Styling
- Responsive CSS grid layout
- Blue buttons with hover/active states
- Red "Stop" button for visual priority
- Green "Center" button for neutral action
- Mobile-optimized (viewport meta tag)

### Route: `GET /status` (JSON API)

**Handler:** `handleStatusJson()`

Returns real-time device telemetry in JSON format:

```json
{
  "connected": true,
  "ssid": "YourNetwork",
  "ip": "192.168.1.100",
  "mac": "AA:BB:CC:DD:EE:FF",
  "rssi": -45,
  "hostname": "acebott-robot",
  "uptime": "0d 01:23:45",
  "freeHeap": 234560,
  "chipModel": "ESP32-D0WDQ6",
  "chipRevision": 1
}
```

**Use Cases:**
- Programmatic monitoring/integration
- Dashboard applications
- Health checks
- Automation scripts

**Implementation Notes:**
- Manual JSON string composition (avoids JSON library overhead)
- Uses `F()` macro for flash-stored literals (saves RAM)
- Pre-reserves 512 bytes to minimize reallocations

### Route: `GET /cmd?action=<action>` (Command API)

**Handler:** `handleCommand()`

Executes robot commands via HTTP GET parameter.

#### Supported Actions

| Action | Function | Robot Behavior | Speed/Angle |
|--------|----------|----------------|-------------|
| `forward` | `moveForward()` | Move forward | 150 |
| `backward` | `moveBackward()` | Move backward | 150 |
| `left` | `moveLeft()` | Strafe/turn left | 150 |
| `right` | `moveRight()` | Strafe/turn right | 150 |
| `stop` | `stopRobot()` | Stop all motors | 0 |
| `rotleft` | `rotateLeft()` | Rotate CCW | 120 |
| `rotright` | `rotateRight()` | Rotate CW | 120 |
| `servoleft` | `servoLeft()` | Pan servo +30° | Max 180° |
| `servoright` | `servoRight()` | Pan servo −30° | Min 0° |
| `servocenter` | `servoCenter()` | Center servo | 90° |
| `led` | `toggleLED()` | Toggle LED state | — |
| `buzzer` | `toggleBuzzer()` | Play 2-beep tone | 1000 Hz |
| `distance` | `sensor.Ranging()` | Measure distance | Returns cm |

#### Example Requests
```
GET /cmd?action=forward      → "Moving forward"
GET /cmd?action=stop         → "Stopped"
GET /cmd?action=distance     → "Distance: 42.3 cm"
GET /cmd?action=led          → "LED ON" / "LED OFF"
GET /cmd?action=unknown      → "Unknown command: unknown"
```

#### Response Format
- Content-Type: `text/plain`
- HTTP 200 status for all requests (including unknown commands)
- Text response confirms action or reports distance reading

### Route: `*` (404 Handler)

**Handler:** `handleNotFound()`

Returns plaintext message: `"Not found: <requested-uri>"`

## IR Control Mappings

Identical to the base IR controller. See [irmovecontrol.md](irmovecontrol.md#ir-command-mappings) for full key map.

**Quick Reference:**
- Arrows → Movement
- OK → Stop
- 1/2 → Rotate
- 3/4/5 → Servo control
- 6 → LED toggle
- 7 → Buzzer
- 8 → Distance reading

## Robot Action Implementations

### Movement Functions
All movement commands forward to `myCar.Move(direction, speed)`:
- Forward/backward/left/right use speed **150**
- Rotations use speed **120**
- Stop uses speed **0**

Serial logs each action for debugging.

### Servo Control
- **Range:** 0° to 180°, enforced via `min()/max()`
- **Step Size:** 30° increments/decrements
- **Center:** 90° (neutral pan position)
- **Delays:** 500 ms settling time after each move
- **Direction Note:** Implementation has `servoLeft()` increasing angle and `servoRight()` decreasing; adjust if physical mount differs

### LED Toggle
- Flips `ledState` boolean
- Writes to `LED_PIN` directly
- Prints current state to Serial

### Buzzer Operation
Custom tone generation using blocking pin toggle:
1. Saves current servo position
2. Plays two 500 ms beeps at 1000 Hz (via 500 µs HIGH/LOW cycles)
3. 100 ms pause between beeps
4. Forces buzzer LOW
5. Restores servo position
6. Resets `buzzerState` to false

**Note:** Uses `playToneHz()` for WiFi connection tune (ascending 3-tone chirp).

### Ultrasonic Distance Reading
- Calls `sensor.Ranging()` which returns distance in cm
- Prints to Serial
- Web API returns formatted string: `"Distance: X.X cm"`

## Utility Functions

### `formatUptime()`
Converts `millis()` to human-readable format: `"<days>d HH:MM:SS"`

**Example:** `1d 03:42:15` for ~27.7 hours runtime

Used in both HTML UI and JSON status.

### `playToneHz(int pin, int freq, int durationMs)`
Generates square wave tone using blocking microsecond delays:
- Calculates period from frequency
- Toggles pin for calculated number of cycles
- Used for WiFi connection celebration tune

**Implementation:**
```cpp
periodUs = 1000000 / freq
cycles = (durationMs * 1000) / periodUs
for each cycle: HIGH (periodUs/2), LOW (periodUs/2)
```

### `playConnectedTune()`
Plays ascending 3-tone sequence on first WiFi connection:
- 880 Hz (A5) for 130 ms
- 1245 Hz (D#6) for 130 ms
- 1760 Hz (A6) for 200 ms
- 50 ms pauses between tones

Only plays once per boot (`wifiConnectedOnce` flag prevents repeats on reconnect).

### `getKeyName(uint32_t command)`
Maps IR command codes to human-readable names.

Returns descriptive strings like `"UP ARROW"`, `"OK BUTTON"`, `"1"`, or `"UNKNOWN KEY"` for diagnostics.

## Execution Flow Summary

### Startup
1. Initialize all robot hardware (motors, servo, sensors, IR, LED, buzzer)
2. Connect to WiFi with 15s timeout
3. Start web server on configured port
4. Enter main loop

### Runtime Loop (every 100 ms)
1. **WiFi Check** — If disconnected >10s, retry connection (non-blocking)
2. **Web Server** — Process any pending HTTP requests
3. **IR Input** — Decode and execute any remote button presses
4. **Delay** — 100 ms stabilization pause

### Control Paths
- **Web UI** → User clicks button → JavaScript fetch → `/cmd?action=X` → `handleCommand()` → Action function → Motor/servo/sensor
- **IR Remote** → Button press → IR signal → `IrReceiver.decode()` → Command map → Action function → Motor/servo/sensor

Both paths converge on the same action functions, ensuring consistent behavior.

## Network Configuration Examples

### DHCP (Simple)
```cpp
// io_config.h
#define WIFI_SSID "HomeNetwork"
#define WIFI_PASSWORD "SecurePass123"
#define WEB_SERVER_PORT 80
```

### Static IP
```cpp
// io_config.h
#define WIFI_SSID "HomeNetwork"
#define WIFI_PASSWORD "SecurePass123"
#define WEB_SERVER_PORT 80
#define HOSTNAME "acebott-robot"
#define STATIC_IP "192.168.1.100"
#define GATEWAY_IP "192.168.1.1"
#define SUBNET_MASK "255.255.255.0"
#define DNS_SERVER "8.8.8.8"
```

### Custom Port (Non-privileged)
```cpp
#define WEB_SERVER_PORT 8080  // Use http://robot-ip:8080/
```

## Web UI Access Methods

### Local Network
1. Note IP address from Serial output after WiFi connection
2. Open browser on any device connected to same network
3. Navigate to `http://<robot-ip>/` (or `http://<hostname>.local/` on mDNS-capable networks)

### Mobile Device
- UI is fully responsive; works on phones/tablets
- Use landscape orientation for optimal button layout
- Bookmark for quick access

### API Integration
```bash
# Command line control
curl "http://192.168.1.100/cmd?action=forward"
curl "http://192.168.1.100/cmd?action=stop"

# Status monitoring
curl "http://192.168.1.100/status" | jq .

# Automation example (bash)
while true; do
  DIST=$(curl -s "http://192.168.1.100/cmd?action=distance")
  echo $DIST
  sleep 2
done
```

## Troubleshooting

### WiFi Issues
**Symptom:** "Connection timeout" on Serial  
**Solutions:**
- Verify SSID/password in `io_config.h`
- Check 2.4 GHz WiFi availability (ESP32 doesn't support 5 GHz)
- Ensure router allows new device connections
- Try increasing timeout in `tryConnectWiFi()` call
- Check Serial for specific error codes

**Symptom:** Repeated reconnection attempts  
**Solutions:**
- Weak signal → Move closer to router or add antenna
- Router blocking MAC → Whitelist device MAC (printed on Serial)
- Network congestion → Try different WiFi channel

### Web Server Issues
**Symptom:** Cannot access web UI  
**Solutions:**
- Verify `WiFi.status() == WL_CONNECTED` via Serial
- Ping IP address to confirm network routing
- Check firewall settings (especially on Windows)
- Try direct IP instead of hostname
- Ensure port not blocked (try 8080 if 80 fails)

**Symptom:** Commands not executing  
**Solutions:**
- Check Serial output for action logs
- Verify URL format: `/cmd?action=forward` (not `/cmd/forward`)
- Test with minimal command: `/cmd?action=stop`
- Check browser console for JavaScript errors

### IR Control Issues
**Symptom:** No IR response  
**Solutions:**
- Confirm IR receiver powered and oriented toward remote
- Check `IR_RECEIVE_PIN` matches physical wiring
- Use diagnostic output (unmapped key handler) to verify codes
- Replace IR receiver if no signal detected
- Check for IR interference from sunlight/fluorescent lights

**Symptom:** Wrong actions triggered  
**Solutions:**
- Different remote models use different codes
- Read actual codes from Serial diagnostic output
- Update `switch(command)` cases to match your remote

### Robot Motion Issues
See [irmovecontrol.md](irmovecontrol.md#troubleshooting) for motor, servo, and sensor debugging.

## Performance Considerations

### Memory Management
- HTML uses `F()` macro to store strings in flash (saves RAM)
- JSON manually composed (avoids library overhead)
- String pre-reservation reduces heap fragmentation
- Current free heap reported in `/status` endpoint

### Blocking Operations
**During these operations, IR/web are unresponsive:**
- Servo moves: 500 ms
- Buzzer beeps: ~1.2 seconds total
- WiFi connection: up to 15s (startup) or 7s (reconnect)

**Mitigation:**
- Keep servo delays minimal
- Consider async WiFi connection for real-time applications
- Use non-blocking tone generation for production use

### Network Latency
- Web commands: ~50–200 ms typical (LAN)
- IR commands: <10 ms (direct)
- For time-critical control, prefer IR input

## Security Considerations

**⚠️ WARNING:** This is a development/education platform with no authentication.

**Risks:**
- Anyone on the network can control the robot
- No HTTPS (credentials/commands sent in clear text)
- No rate limiting (command spam possible)

**Production Hardening:**
- Add basic authentication to web routes
- Implement HTTPS (ESP32 supports TLS)
- Use WPA2-Enterprise or VPN for network isolation
- Add command rate limiting
- Implement session tokens
- Consider moving to MQTT with ACLs

## Extensibility

### Adding New Commands
1. Create action function (e.g., `void spinCircle()`)
2. Add case to IR `switch(command)` in `loop()`
3. Add button to HTML in `handleRoot()`
4. Add case to `handleCommand()` dispatcher
5. Update this documentation

### Adding Sensors
1. Initialize in `setup()`
2. Create read function
3. Add IR/web command mappings
4. Consider adding to `/status` JSON for monitoring

### Advanced Features
- **Autonomous Mode** — Use ultrasonic in loop to auto-avoid obstacles
- **Path Recording** — Log commands and replay sequence
- **Video Streaming** — Add ESP32-CAM module
- **Websockets** — Replace polling with real-time updates
- **MQTT Integration** — Connect to IoT platform (Azure IoT, AWS IoT)
- **Telemetry** — Log distance/position to cloud database

## Code Organization

### Sections (Line References)
- Hardware definitions: Lines 1–16
- Global objects/state: Lines 18–28
- Audio helpers: Lines 30–59
- Web server handlers: Lines 73–191
- WiFi connection: Lines 194–234
- IR key mapping: Lines 237–254
- Robot actions: Lines 257–372
- Command dispatcher: Lines 374–422
- Setup: Lines 424–471
- Main loop: Lines 473–517

### Key Dependencies
- `io_config.h` — WiFi credentials (must be created by user)
- `vehicle.h` — Motor control abstraction
- `ultrasonic.h` — Distance sensor wrapper
- Arduino ESP32 core libraries

## Build & Upload

### Arduino IDE
1. Install ESP32 board support via Board Manager
2. Install libraries: `IRremote`, `ESP32Servo`
3. Create `io_config.h` with WiFi credentials
4. Select ESP32 board variant
5. Upload and open Serial Monitor (115200 baud)

### PlatformIO
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    IRremote
    ESP32Servo
monitor_speed = 115200
```

### First Run Checklist
1. Create `io_config.h` with valid credentials
2. Upload sketch
3. Open Serial Monitor at 115200 baud
4. Wait for WiFi connection message
5. Note IP address from Serial output
6. Navigate to IP in web browser
7. Test both IR and web controls

## Comparison to Base IR Controller

| Feature | irmovecontrol.cpp | with-web-serve.cpp |
|---------|-------------------|-------------------|
| IR Control | ✅ | ✅ |
| WiFi | ❌ | ✅ |
| Web UI | ❌ | ✅ |
| JSON API | ❌ | ✅ |
| Auto-reconnect | N/A | ✅ |
| Static IP | N/A | ✅ |
| Uptime tracking | ❌ | ✅ |
| Connection tune | ❌ | ✅ |
| Remote access | Physical only | Network-based |
| Complexity | Low | Medium |
| Dependencies | 4 libraries | 6 libraries + config |

Choose base version for simplicity; choose web version for remote control and monitoring.

## Conclusion

This program transforms the Acebott Smartcar into a network-connected robot with dual control interfaces, real-time status monitoring, and extensible API endpoints. The architecture maintains backward compatibility with IR control while adding modern web-based interaction suitable for IoT integration, educational demonstrations, and remote operation scenarios.

For questions or contributions, refer to the inline code comments and Serial diagnostic output during runtime.
