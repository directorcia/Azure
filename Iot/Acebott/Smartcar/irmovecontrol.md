# IR-Controlled Acebott Smartcar — Execution & Operation Guide

This document explains the runtime behavior, control flow, hardware interactions, and extension points of the IR-controlled Acebott Smartcar program.

Source: Iot/Acebott/Smartcar/irmovecontrol.cpp

## Overview
The program turns an Acebott/ESP32-based robot car into an IR-remote–controlled vehicle with basic movement, obstacle sensing, servo scanning, LED and buzzer control. It:
- Initializes the vehicle drivetrain, ultrasonic distance sensor, a scanning servo, onboard LED, buzzer, and IR receiver.
- Continuously listens for IR remote keypresses and maps each to a robot action.
- Provides feedback over Serial and simple state toggles for LED and buzzer.

## Hardware & Libraries
- MCU: ESP32 (uses `ESP32Servo` and is compatible with `IRremote`’s ESP32 backend)
- Actuation: DC motors (through `vehicle` library), 1x servo on pan mount
- Sensors: Ultrasonic distance sensor (trigger/echo)
- Feedback: Onboard LED, piezo buzzer

### Pin Assignments
- IR receiver: `IR_RECEIVE_PIN = 4`
- Ultrasonic: `TRIG_PIN = 13`, `ECHO_PIN = 14`
- Servo: `SERVO_PIN = 25`
- LED: `LED_PIN = 2`
- Buzzer: `BUZZER_PIN = 33`

### Key Libraries/Classes
- `IRremote` (`IrReceiver`) — decodes IR signals
- `vehicle` (`myCar`) — abstracts motor control; supports motions such as `Forward`, `Backward`, `Move_Left`, `Move_Right`, `Clockwise`, `Contrarotate`, `Stop`
- `ultrasonic` (`sensor`) — provides distance via `Ranging()`
- `ESP32Servo` (`scanServo`) — controls the pan servo

## Global State
- `ledState` (bool): LED on/off
- `buzzerState` (bool): request/edge for buzzer beeps (auto-resets after sounding)
- `servoPosition` (int): tracked servo angle, 0–180 (starts centered at 90)

## Startup Sequence (`setup()`)
1. Start Serial at 115200 and print a banner.
2. Initialize drivetrain via `myCar.Init()`.
3. Initialize ultrasonic with `sensor.Init(TRIG_PIN, ECHO_PIN)`.
4. Attach servo to `SERVO_PIN`, center at 90°, and store `servoPosition = 90`.
5. Configure LED and buzzer as outputs; set both LOW.
6. Start IR receiver with `IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK)`.
7. Print quick usage help over Serial.

This ensures all subsystems are ready before processing input.

## Main Loop (`loop()`)
- Polls `IrReceiver.decode()`;
  - If a key is decoded, reads `IrReceiver.decodedIRData.command` and dispatches to a matching action.
  - If no mapping exists, prints a diagnostic (human-readable key name, command hex, raw data hex, and protocol name).
  - Calls `IrReceiver.resume()` to re-arm the receiver.
- Sleeps for ~100 ms to reduce CPU churn/jitter.

## IR Command Mappings
The following commands are handled directly in the `switch (command)` dispatch:
- UP (0x46): `moveForward()` → `myCar.Move(Forward, 150)`
- DOWN (0x15): `moveBackward()` → `myCar.Move(Backward, 150)`
- LEFT (0x44): `moveLeft()` → `myCar.Move(Move_Left, 150)`
- RIGHT (0x43): `moveRight()` → `myCar.Move(Move_Right, 150)`
- OK (0x40): `stopRobot()` → `myCar.Move(Stop, 0)`
- 1 (0x16): `rotateLeft()` → `myCar.Move(Contrarotate, 120)`
- 2 (0x19): `rotateRight()` → `myCar.Move(Clockwise, 120)`
- 3 (0x0D): Servo Left (−30° step) via `servoLeft()`
- 4 (0x0C): Servo Right (+30° step) via `servoRight()`
- 5 (0x18): Servo Center (90°) via `servoCenter()`
- 6 (0x5E): `toggleLED()`
- 7 (0x08): `toggleBuzzer()` (plays two beeps)
- 8 (0x1C): `readUltrasonic()` (prints distance in cm)

Other keys print a descriptive line using `getKeyName(command)`, plus raw/ protocol diagnostics.

## Action Implementations

### Movement
All movement functions print a log line and forward to `vehicle::Move`:
- `moveForward()` / `moveBackward()` use speed 150
- `moveLeft()` / `moveRight()` use speed 150
- `rotateLeft()` (`Contrarotate`) / `rotateRight()` (`Clockwise`) use speed 120
- `stopRobot()` clears motion via `Stop, 0`

These speeds are relative to the `vehicle` implementation; tune as needed for your chassis/gearbox.

### Servo Scan
- Positions are tracked in `servoPosition` and enforced via `scanServo.write(servoPosition)`.
- `servoLeft()` decreases by 30°, min 0°
- `servoRight()` increases by 30°, max 180°
- `servoCenter()` sets 90°
- Each move includes a `delay(500)` to allow mechanical settling.

Tip: Adjust the step size or delays to suit the physical mount and servo speed.

### LED Toggle
- `toggleLED()` flips `ledState` and writes to `LED_PIN` accordingly, printing the new state.

### Buzzer
- `toggleBuzzer()` uses a high-frequency digital toggle loop to synthesize ~1 kHz tone without PWM.
- Behavior:
  1. Saves current `servoPosition`.
  2. Plays two ~500 ms beeps separated by 100 ms by rapidly toggling `BUZZER_PIN` (500 µs high / 500 µs low).
  3. Forces buzzer LOW and restores servo position (no re-attach needed).
  4. Prints a message and resets `buzzerState` to `false` (so it acts like a “beep now” command rather than a latched toggle).

Note: Because the routine busy-waits with microsecond delays, the MCU is occupied during the beep.

### Ultrasonic Distance
- `readUltrasonic()` calls `sensor.Ranging()` and prints the result in centimeters.
- Use it to quickly sanity-check obstacle proximity while driving or steering the servo.

## IR Diagnostics & Mapping Helper
- `getKeyName(uint32_t command)` returns a human-readable name for common IR remote key codes (arrows, digits, star, hash). Unknown codes return `"UNKNOWN KEY"`.
- On unmapped inputs, diagnostics also include `decodedRawData` and the decoded `protocol` name to aid in extending the command map.

## Execution Flow Summary
1. Initialize all subsystems in `setup()` and print usage help.
2. In `loop()`, wait for `IrReceiver.decode()`.
3. If a known key is detected, run the corresponding action; else, print diagnostics.
4. Re-arm the receiver and idle briefly.

## Extensibility
- Add new IR bindings: Extend the `switch (command)` map.
- Change speeds: Update values in movement helpers.
- Adjust servo behavior: Modify step size or center angle.
- Continuous scanning: Create a mode that sweeps the servo and samples `Ranging()` for simple obstacle mapping.
- Safety: Add minimum distance checks to auto-stop the car if `Ranging()` < threshold.

## Troubleshooting
- No IR response: Confirm `IR_RECEIVE_PIN`, IRremote version, and that the IR receiver is powered and oriented correctly.
- Wrong codes: Use the default diagnostic print (unmapped key) to see the actual `command` and update the map.
- Servo jitter/no motion: Verify 5V supply (if required by the servo), common ground, and `SERVO_PIN`; try increasing delays.
- Buzzer quiet: Some buzzers need different frequencies; adjust the `delayMicroseconds` high/low durations to change pitch.
- Distance reads 0 or constant: Check `TRIG_PIN`/`ECHO_PIN` wiring; ensure no pin conflicts.

## Build & Upload (Typical)
- Board: ESP32-based development board
- Arduino IDE: Install `IRremote` (ESP32-compatible), `ESP32Servo`, and ensure your custom `vehicle` and `ultrasonic` headers/sources are available in the project.
- PlatformIO: Add dependencies to `platformio.ini` accordingly and select an ESP32 platform.

Serial monitor at 115200 shows action logs, distance readings, and unmapped-key diagnostics for quick iteration.
