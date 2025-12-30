# ACEBOTT QD010 Smart Car - PS3 Controller with Camera System

## Overview
This code implements a comprehensive robot car control system for the ACEBOTT QD010 platform, combining PS3 wireless controller input, ESP32-CAM integration, ultrasonic sensing, and servo-controlled camera positioning. The system provides real-time video streaming while maintaining responsive remote control.

## System Architecture

### Hardware Components
1. **Main Controller**: ESP32 microcontroller
2. **Motor System**: 4-wheel drive with differential control
3. **Camera Module**: ESP32-CAM (connected via UART)
4. **Distance Sensor**: HC-SR04 ultrasonic sensor
5. **Servo Motors**: 
   - Ultrasonic scanner servo (pin 25)
   - Camera pan/tilt servo (pin 27)
6. **User Feedback**:
   - 2 LEDs (pins 2, 12)
   - Buzzer (pin 33)
7. **Control Interface**: PS3 Controller (Bluetooth)

### Communication Interfaces
- **Bluetooth**: PS3 controller wireless connection
- **WiFi**: Network access for camera streaming
- **UART (Serial1)**: ESP32 ↔ ESP32-CAM communication
- **USB Serial**: Debugging output (115200 baud)

## Execution Flow

### 1. Initialization Sequence (`setup()`)

#### Phase 1: Serial Communication
```
1. Initialize USB serial (115200 baud)
2. Display startup banner
```

#### Phase 2: Motor System
```
1. Initialize vehicle motor controller
2. Configure motor driver pins and PWM channels
3. Set initial motor state to stopped
```

#### Phase 3: Sensors
```
1. Initialize ultrasonic sensor (TRIG=13, ECHO=14)
2. Configure trigger and echo pin modes
```

#### Phase 4: Servo Systems
```
1. Ultrasonic Scanner Servo:
   - Attach to pin 25
   - Move to center (90°)
   - Wait 1000ms for positioning
   
2. Camera Tilt Servo:
   - Attach to pin 27
   - Move to center (90°)
   - Wait 500ms for positioning
```

#### Phase 5: Camera Communication
```
1. Initialize UART1 (115200 baud, 8N1)
2. Configure RX=9, TX=10 for ESP32-CAM
3. Enable AT command interface
```

#### Phase 6: User Interface
```
1. Configure LED pins (2, 12) as outputs
2. Configure buzzer pin (33) as output
3. Set all outputs to LOW (off)
```

#### Phase 7: PS3 Controller
```
1. Register event callback: onPs3Notify()
2. Register connection callback: onPs3Connect()
3. Start Bluetooth with MAC: "20:00:00:00:82:62"
4. Wait for PS button press to connect
```

#### Phase 8: Network
```
1. Attempt WiFi connection (15 second timeout)
2. Configure static IP if defined, else use DHCP
3. Set hostname if defined
4. Display IP address and signal strength
5. Play connection tune on success
```

### 2. Main Loop (`loop()`)

The main loop runs continuously and handles WiFi maintenance:

```
┌─────────────────────────────────────┐
│ Check WiFi Status                   │
├─────────────────────────────────────┤
│ If disconnected:                    │
│   - Check if 10 seconds elapsed     │
│   - Attempt reconnection (7s timeout)│
│   - Update retry timestamp          │
└─────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────┐
│ Delay 100ms                         │
│ (Prevents watchdog reset)           │
└─────────────────────────────────────┘
          │
          ▼
    [Repeat Loop]
```

**Note**: All PS3 controller input is handled asynchronously via the `onPs3Notify()` callback—no polling required in the main loop.

## PS3 Controller Event Processing (`onPs3Notify()`)

This callback is triggered automatically whenever the PS3 controller state changes.

### Input Processing Order

#### 1. Joystick Reading
```
Left Stick:
  - X axis: -128 (left) to +127 (right)
  - Y axis: -128 (up/forward) to +127 (down/back)

Right Stick:
  - Y axis: Speed control
    • Center (±30): Default speed = 200
    • Up (-128): Maximum speed = 255
    • Down (+127): Minimum speed = 150
```

#### 2. Movement Logic Decision Tree

```
┌─────────────────────────────────────────┐
│ Is |JoyY| > deadzone AND (L1 OR R1)?   │
└─────────────────────────────────────────┘
          │ YES                    │ NO
          ▼                        ▼
┌─────────────────────┐   ┌─────────────────────┐
│   ARC TURNING       │   │ L1 pressed alone?   │
│                     │   └─────────────────────┘
│ Forward + L1:       │            │ YES
│  Left slower (70%)  │            ▼
│                     │   ┌─────────────────────┐
│ Forward + R1:       │   │  ROTATE LEFT        │
│  Right slower (70%) │   │  (Counter-clockwise)│
│                     │   └─────────────────────┘
│ Backward + L1/R1:   │            │ NO
│  Same differential  │            ▼
└─────────────────────┘   ┌─────────────────────┐
                          │ R1 pressed alone?   │
                          └─────────────────────┘
                                   │ YES
                                   ▼
                          ┌─────────────────────┐
                          │  ROTATE RIGHT       │
                          │  (Clockwise)        │
                          └─────────────────────┘
                                   │ NO
                                   ▼
                          ┌─────────────────────┐
                          │ Joystick deflected? │
                          └─────────────────────┘
                                   │ YES
                                   ▼
                          ┌─────────────────────┐
                          │ NORMAL MOVEMENT     │
                          │                     │
                          │ Priority: Larger    │
                          │ axis deflection     │
                          └─────────────────────┘
                                   │ NO
                                   ▼
                          ┌─────────────────────┐
                          │      STOP           │
                          └─────────────────────┘
```

#### 3. Button Actions

| Button | Action | Description |
|--------|--------|-------------|
| **D-pad Left** | Servo Left | Rotate ultrasonic servo +30° (max 180°) |
| **D-pad Right** | Servo Right | Rotate ultrasonic servo -30° (min 0°) |
| **D-pad Up** | Camera Up | Tilt camera up -10° |
| **D-pad Down** | Camera Down | Tilt camera down +10° |
| **Circle (○)** | Center Servo | Reset ultrasonic servo to 90° |
| **Square (□)** | Beep | Play two 2000Hz beeps |
| **Triangle (△)** | Flip Camera | Send VFLIP=1, HFLIP=1 to camera |
| **Cross (✕)** | Reset Camera | Send VFLIP=0, HFLIP=0 to camera |
| **L1** | Turn Left | Arc turn or rotate based on joystick |
| **R1** | Turn Right | Arc turn or rotate based on joystick |
| **L2** | Toggle LED | Switch LED1 on/off |
| **Select** | Center Servo | Same as Circle button |

## Movement Systems

### Motor Trim Calibration

The robot uses motor trim to compensate for manufacturing variations:

```
Left Motor Trim:  0.66 (66% power)
Right Motor Trim: 1.0  (100% power)
```

**Why Trim is Needed:**
- DC motors have slight performance differences
- Wheels may have different diameters
- Friction varies between sides
- Without trim, robot veers during straight movement

**Calibration Process:**
1. Robot tested moving forward
2. Observed veering direction
3. Trim adjusted to compensate
4. Tested until straight movement achieved

### Smooth Acceleration System

The code implements gradual speed changes to prevent:
- Wheel slipping
- Jerky motion
- Loss of traction
- Sudden battery drain

**Algorithm:**
```
1. If starting from stop:
   - Jump to 180 PWM (minimum working speed)
   - Ensures motors overcome static friction

2. Every 30ms:
   - Increase/decrease speed by 15 PWM units
   - Approach target speed gradually
   
3. Speed limiting:
   - Don't overshoot target
   - Constrain to 0-255 PWM range
   - Apply trim factors
```

### Movement Types

#### 1. Forward/Backward Movement
```
All wheels rotate in same direction
Speed: Both sides at trimmed speeds
Result: Straight line motion
```

#### 2. Arc Turning (Joystick + L1/R1)
```
Forward + L1 (Arc Left):
  Left wheels:  70% × motorSpeed × 0.66 (trim)
  Right wheels: 100% × motorSpeed × 1.0 (trim)
  Result: Smooth left curve

Forward + R1 (Arc Right):
  Left wheels:  100% × motorSpeed × 0.66 (trim)
  Right wheels: 70% × motorSpeed × 1.0 (trim)
  Result: Smooth right curve
```

#### 3. Point Rotation (L1/R1 alone)
```
Rotate Left (L1 only):
  Left wheels:  Backward
  Right wheels: Forward
  Result: Counter-clockwise spin

Rotate Right (R1 only):
  Left wheels:  Forward
  Right wheels: Backward
  Result: Clockwise spin
```

#### 4. Lateral Movement
```
Move Left:
  Left wheels:  Backward
  Right wheels: Forward
  Result: Crab sideways left

Move Right:
  Left wheels:  Forward
  Right wheels: Backward
  Result: Crab sideways right
```

## Camera Control System

### UART Communication Protocol

The ESP32 communicates with ESP32-CAM using AT commands over UART:

```
Baud Rate: 115200
Format: 8N1 (8 data bits, No parity, 1 stop bit)
TX Pin: GPIO10
RX Pin: GPIO9
```

### Camera Commands

#### Flip Image 180° (Triangle Button)
```
Command Sequence:
1. Serial1.println("AT+VFLIP=1")  → Enable vertical flip
2. delay(50ms)
3. Serial1.println("AT+HFLIP=1")  → Enable horizontal flip
4. delay(100ms)

Use Case: Camera mounted upside-down on robot
```

#### Reset Image Orientation (Cross Button)
```
Command Sequence:
1. Serial1.println("AT+VFLIP=0")  → Disable vertical flip
2. delay(50ms)
3. Serial1.println("AT+HFLIP=0")  → Disable horizontal flip
4. delay(100ms)

Use Case: Camera mounted right-side up
```

### Camera Tilt Servo

```
Pin: 27
Range: 0° to 180°
Step Size: 10° per button press
Center: 90° (level view)

D-pad Up:   Decrease angle (tilt up)
D-pad Down: Increase angle (tilt down)
```

## Audio Feedback System

### Tone Generation

ESP32 doesn't have Arduino's `tone()` function, so tones are generated manually:

```
Algorithm:
1. Calculate wave period: 1,000,000 µs / frequency
2. Calculate required cycles: (duration × 1000) / period
3. For each cycle:
   - Set pin HIGH
   - Delay period/2 microseconds
   - Set pin LOW
   - Delay period/2 microseconds
```

### Sound Patterns

#### Connection Success Tune
```
Note 1: 880 Hz (A5)  - 130ms
Pause:  50ms
Note 2: 1245 Hz (D#6) - 130ms
Pause:  50ms
Note 3: 1760 Hz (A6)  - 200ms

Triggered when:
- PS3 controller connects
- WiFi connects (first time only)
```

#### Buzzer Beep (Square Button)
```
Beep 1: 2000 Hz - 100ms
Pause:  50ms
Beep 2: 2000 Hz - 100ms

Purpose: User confirmation or alert
```

## WiFi Connection Management

### Initial Connection

```
1. Set hostname (if defined in io_config.h)
2. Configure static IP (if defined) or use DHCP
3. Set WiFi mode to STA (Station)
4. Call WiFi.begin(SSID, PASSWORD)
5. Wait up to 15 seconds for connection
6. Display IP address and signal strength
7. Play connection tune (first time only)
```

### Connection Maintenance

```
Main Loop Checks:
- Every cycle: Check WiFi.status()
- If disconnected:
  - Has 10 seconds elapsed since last attempt?
    - YES: Attempt reconnection (7 second timeout)
    - NO: Skip this cycle
```

### Network Configuration Options

The code supports flexible network setup via `io_config.h`:

```cpp
// Required:
WIFI_SSID       - Network name
WIFI_PASSWORD   - Network password

// Optional:
HOSTNAME        - Custom device name
STATIC_IP       - Fixed IP address
GATEWAY_IP      - Router address
SUBNET_MASK     - Network mask
DNS_SERVER      - DNS server address
```

## Sensor Systems

### Ultrasonic Distance Sensor

```
Model: HC-SR04
Trigger Pin: 13
Echo Pin: 14

Operation:
1. Send 10µs pulse to trigger
2. Measure echo pulse duration
3. Calculate: distance = (duration × 0.034) / 2 cm

Range: 2cm to 400cm
Accuracy: ±3mm
```

### Ultrasonic Scanner Servo

```
Pin: 25
Range: 0° to 180°
Step Size: 30° per button press
Center: 90° (forward)

Purpose: Scan environment for obstacles
Control: D-pad Left/Right buttons
```

## System State Variables

### Global State Tracking

| Variable | Purpose | Range/Values |
|----------|---------|--------------|
| `ledState` | LED1 on/off state | true/false |
| `led2State` | LED2 on/off state | true/false |
| `buzzerState` | Buzzer state | true/false |
| `servoPosition` | Scanner servo angle | 0-180° |
| `cameraPanTiltPosition` | Camera servo angle | 0-180° |
| `motorSpeed` | Target motor PWM | 150-255 |
| `currentMotorSpeed` | Current motor PWM | 0-255 |
| `lastAccelTime` | Acceleration timing | milliseconds |
| `wifiConnectedOnce` | First WiFi connection flag | true/false |
| `lastWifiAttemptMs` | Last WiFi retry time | milliseconds |

## Performance Characteristics

### Speed Control

```
Default Speed: 200 PWM (when right stick centered)
Speed Range:   150-255 PWM
Speed Steps:   15 PWM units per 30ms
Acceleration:  ~8 PWM units/second
```

### Control Responsiveness

```
PS3 Callback:      Asynchronous (instant response)
Main Loop Cycle:   ~100ms
Servo Movement:    500ms delay for positioning
Speed Ramp Step:   30ms intervals
WiFi Retry:        10 second intervals
```

### Motor Trim Impact

```
Without Trim:
- Forward motion veers to one side
- Rotation may be off-center
- Inconsistent arc turns

With Trim (0.66 / 1.0):
- Straight forward/backward movement
- Centered rotation
- Predictable arc behavior
- Compensates for ~34% power difference
```

## Error Handling

### WiFi Connection Failures
```
Symptom: Connection timeout message
Action: 
- Robot continues to operate normally
- Retry every 10 seconds
- Camera streaming unavailable until connected
- PS3 control unaffected
```

### Camera UART Communication
```
No error checking implemented
Assumes ESP32-CAM receives and processes AT commands
Delays allow command processing time
```

### PS3 Controller Disconnection
```
Behavior: Robot stops moving
Recovery: Press PS button to reconnect
Audio feedback on successful reconnection
```

## Best Practices and Usage

### Starting the Robot

1. Power on robot
2. Wait for "PS3 Controller ready" message
3. Press PS button on controller
4. Listen for connection tune
5. Check Serial Monitor for WiFi IP address
6. Access camera stream at `http://<IP>/stream`

### Controller Operation Tips

**For Smooth Driving:**
- Keep right stick near center for optimal speed (200 PWM)
- Use L1/R1 for gradual turns while moving
- Use L1/R1 alone for precise rotation

**For Exploring:**
- Use D-pad to scan ultrasonic sensor
- Adjust camera tilt for better viewing angle
- Toggle camera flip if mounting orientation changes

**For Maneuvering:**
- Arc turns are smoother for navigation
- Point rotation for tight spaces
- Lateral movement for precise positioning

### Calibration

**If Robot Veers Left:**
```cpp
// Increase left motor trim
float leftMotorTrim = 0.70;  // Increase from 0.66
```

**If Robot Veers Right:**
```cpp
// Decrease left motor trim
float leftMotorTrim = 0.62;  // Decrease from 0.66
```

**Testing Straight Movement:**
1. Place robot on flat surface
2. Mark starting position
3. Hold joystick forward for 3 seconds
4. Measure lateral drift
5. Adjust trim as needed
6. Repeat until drift < 5cm over 3 meters

### Camera Setup

**For Upside-Down Mounting:**
- Press Triangle button to flip image 180°
- Camera view will appear right-side up

**For Normal Mounting:**
- Press Cross button to reset orientation
- Camera view will be normal

## Troubleshooting Guide

### Issue: Robot doesn't respond to controller

**Check:**
1. Is "PS3 Controller Connected!" displayed?
2. Press PS button to reconnect
3. Check controller battery level
4. Verify Bluetooth MAC address matches

### Issue: Robot moves but not straight

**Solution:**
1. Adjust `leftMotorTrim` value
2. Test on flat, smooth surface
3. Increase value if veering left
4. Decrease value if veering right
5. Make small adjustments (±0.02)

### Issue: Camera image is upside-down

**Solution:**
- Press Triangle button for 180° flip
- Or press Cross button for normal orientation

### Issue: WiFi won't connect

**Check:**
1. SSID and password in `io_config.h`
2. Router is 2.4GHz (ESP32 doesn't support 5GHz)
3. Signal strength adequate
4. DHCP enabled (if not using static IP)

### Issue: Jerky movement

**Possible Causes:**
1. Speed too low (motors stalling)
2. Battery voltage low
3. Friction in wheels/gears
4. Acceleration step too large

**Solution:**
- Increase minimum speed in `moveWithTrim()`
- Charge/replace battery
- Check for mechanical obstructions

### Issue: Servo not centering

**Solution:**
- Press Circle or Select button
- Check servo is properly attached
- Verify servo power supply
- Replace servo if mechanically damaged

## Code Architecture Highlights

### Callback-Driven Design
- PS3 input handled asynchronously
- No polling overhead in main loop
- Instant response to controller changes

### Separation of Concerns
- Movement logic isolated in dedicated functions
- Sensor reading separate from control
- WiFi management non-blocking

### State Management
- Global variables track system state
- Position tracking for servos
- Speed tracking for acceleration
- Connection state for WiFi

### Hardware Abstraction
- Vehicle library handles low-level motor control
- Servo library manages PWM timing
- PS3 library handles Bluetooth protocol

## Future Enhancement Possibilities

### Autonomous Features
- Obstacle avoidance using ultrasonic sensor
- Autonomous scanning and mapping
- Waypoint navigation
- Line following

### Camera Improvements
- Pan servo on second axis (full pan/tilt)
- Computer vision processing
- Object tracking
- Face detection

### Control Enhancements
- Web-based control interface
- Mobile app control
- Waypoint recording and playback
- Speed profiles for different terrain

### Sensor Expansion
- IMU for orientation tracking
- GPS for outdoor navigation
- Additional ultrasonic sensors
- Encoders for odometry

### Communication Features
- WebSocket for real-time camera
- Telemetry data logging
- Remote debugging interface
- Multi-robot coordination

## Technical Specifications Summary

| Component | Specification |
|-----------|--------------|
| **Main MCU** | ESP32 |
| **Secondary MCU** | ESP32-CAM |
| **Motor Channels** | 4 (differential drive) |
| **PWM Range** | 0-255 |
| **Servo Quantity** | 2 (scanner + camera) |
| **Servo Range** | 0-180° |
| **Ultrasonic Range** | 2-400 cm |
| **WiFi** | 2.4 GHz 802.11 b/g/n |
| **Bluetooth** | Classic (PS3 protocol) |
| **UART Baud** | 115200 (camera + debug) |
| **Operating Voltage** | 5-7.4V (typical) |
| **Control Range** | ~10m (Bluetooth) |
| **Camera Stream** | MJPEG via WiFi |

## Safety Considerations

### Electrical
- Ensure proper power supply voltage
- Avoid short circuits on motor connections
- Keep camera module dry
- Check battery polarity

### Mechanical
- Test in open area first
- Keep fingers clear of wheels
- Ensure servos don't over-rotate
- Secure all components properly

### Software
- Don't exceed PWM limits (0-255)
- Verify servo angle constraints
- Test trim values incrementally
- Monitor serial output for errors

## Conclusion

This robot control system demonstrates sophisticated integration of multiple subsystems:
- Wireless control with PS3 Bluetooth
- Live video streaming via WiFi
- Precise motor control with trim compensation
- Smooth acceleration for better traction
- Servo-based sensor positioning
- Audio feedback for user interaction

The callback-driven architecture ensures responsive control while maintaining system flexibility. Motor trim compensation and smooth acceleration provide reliable, predictable movement. The dual-communication design (Bluetooth for control, WiFi for video) optimizes bandwidth usage and maintains low-latency control.
