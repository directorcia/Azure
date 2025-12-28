# Acebott PS3-Controlled Car – Code Walkthrough

Source: [Iot/Acebott/Smartcar/QD010/car-ps3.cpp](Iot/Acebott/Smartcar/QD010/car-ps3.cpp)

## What the sketch does
- Drives the Acebott four-motor car with three control paths: PS3 Bluetooth gamepad, IR remote, and a WiFi web remote.
- Exposes status (WiFi, uptime, heap, chip info) and simple drive/servo/buzzer/LED actions via an HTTP UI and JSON endpoint.
- Adds motor trim and a ramped acceleration profile so the robot tracks straight and avoids stalls at low duty cycles.
- Provides basic peripherals: two LEDs, a piezo buzzer, a single ultrasonic range read, and a pan servo for the sensor/camera.

## Hardware and pins
- IR receiver: pin 4.
- Ultrasonic: trig 13, echo 14.
- Servo: pin 25 (initially centered to 90°).
- LEDs: pin 2 (LED1), pin 12 (LED2).
- Buzzer: pin 33.

## Global runtime state
- `motorSpeed` is the target speed (default 255) and is dynamically set from the PS3 right-stick Y axis; defaults to 200 when the stick is near center.
- `currentMotorSpeed` ramps toward `motorSpeed` every 30 ms; if starting from rest it jumps to 180 to overcome motor deadband.
- Trim factors: left `0.66`, right `1.0` to compensate drift; applied to PWM before issuing motor commands.
- Servo angle tracked in `servoPosition`; LED and buzzer latched states maintained for UI feedback/toggles.

## Startup sequence (`setup()`)
- Initializes serial logging, vehicle driver, ultrasonic sensor, servo (centered), LED/buzzer pins, and IR receiver.
- Registers PS3 callbacks (`onPs3Notify`, `onPs3Connect`) and begins Bluetooth listening with the configured MAC.
- Attempts WiFi STA connection using credentials/macros from `io_config.h`; optional static IP/hostname support.
- Starts the HTTP server on `WEB_SERVER_PORT`, wiring routes for root HTML, JSON status, command endpoint, and 404 handler.

## WiFi and web server
- `tryConnectWiFi(timeoutMs=15000)` handles DHCP/static configuration, connects, prints IP/RSSI, and plays a three-tone chirp the first time a connection succeeds.
- Retry logic in `loop()`: if disconnected, retries every 10 s with a 7 s timeout so control paths continue to respond.
- `handleRoot()` builds a lightweight HTML control page with movement buttons, servo controls, LED/buzzer toggles, and a distance read request.
- `handleStatusJson()` returns a compact JSON snapshot: connectivity, SSID/IP/MAC/RSSI, hostname (if defined), uptime, free heap, chip model/revision.
- `handleCommand()` executes actions based on `action` query param (drive, rotate, servo, LED, buzzer, distance) and returns plain-text feedback.

## PS3 controller behavior
- Callback-driven in `onPs3Notify()`; left stick steers/advances, right stick sets speed.
- Deadzone of 20 on stick axes; speed stick deadzone of 30 uses a default speed of 200.
- Holding L1/R1 while pushing the left stick forward/back causes arcing turns: all wheels move the same direction with one side throttled to 70%.
- L1 or R1 alone spins in place (`rotateLeft` / `rotateRight`).
- Button mappings:
  - Triangle / Circle / Cross: servo left / center / right (±30° steps, clamped 0–180°).
  - Square: buzzer double-beep.
  - L2: toggle LED1; Select: toggle LED2.
- `onPs3Connect()` sets the controller player LED and plays the connect chirp.

## Drive and trim logic
- `moveWithTrim(direction, baseSpeed)` ramps `currentMotorSpeed` toward `baseSpeed` with ±15 PWM steps every 30 ms, applying left/right trim before commanding `myCar.MoveWithTrim`.
- High-level helpers (`moveForward`, `moveBackward`, `moveLeft`, `moveRight`) log the action then call `moveWithTrim` with the current `motorSpeed`.
- `stopRobot()` stops motors and resets `currentMotorSpeed` to 0 so the next movement ramps from rest.
- `rotateLeft`/`rotateRight` call direct rotation modes without ramping.

## Peripheral helpers
- Servo positioning: center at 90°, left/right add/subtract 30° with bounds; 500 ms pauses allow motion to settle.
- LEDs: `toggleLED` and `toggleLED2` flip latch state and drive the respective GPIO.
- Buzzer: `toggleBuzzer` issues two short 2 kHz beeps, then forces the pin low; state resets to off afterward.
- Ultrasonic: `readUltrasonic` prints the distance read from `sensor.Ranging()`.

## IR remote mapping
- Uses `IRremote` decode results inside `loop()`. Key codes: Up/Down/Left/Right → forward/back/left/right; OK → stop; 1 → rotate left; 2 → rotate right; 3 → servo left; 4 → servo right; 5 → servo center; 6 → toggle LED; 7 → buzzer beeps; 8 → ultrasonic read; others log protocol/name.

## Main loop responsibilities
- Maintain WiFi (periodic reconnect attempts) and service HTTP requests (`server.handleClient()`).
- Process IR inputs when available, then `IrReceiver.resume()` to ready the next decode.
- Delay 100 ms to balance responsiveness with IR polling and CPU load; PS3 input remains responsive via callbacks.

## Operational tips
- If the car drifts, adjust `leftMotorTrim` or `rightMotorTrim` in the source to balance straight-line motion.
- Keep right-stick near center for a steady cruising speed; push fully up to reach max PWM (255).
- Web control works only when WiFi is connected; PS3 and IR remain usable offline.
- The acceleration ramp favors smooth starts; expect a brief spin-up before full speed after stopping.
