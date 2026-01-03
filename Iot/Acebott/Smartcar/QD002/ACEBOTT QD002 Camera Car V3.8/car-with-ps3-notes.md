# Acebott QD002 Camera Car (PS3) - Run Notes

## Overview
- Sketch: car-with-ps3.cpp (ESP32-based). Uses a PS3 controller for all motion, camera tilt, LEDs, buzzer, and ultrasonic servo.
- Peripherals: dual motor driver via vehicle lib, ultrasonic on TRIG 13/ECHO 14, scan servo on 25, camera pan/tilt servo on 27, buzzer on 33, LEDs on 2 and 12, camera UART on GPIO9/10.
- Connectivity: optional WiFi STA for telemetry; not required for driving. PS3 connects over Bluetooth directly to ESP32.

## Startup Sequence (setup)
1) Begin Serial at 115200 for logs.
2) Init vehicle and ultrasonic driver; attach scan servo (center at 90°); attach camera pan/tilt servo (center at 90°).
3) Start UART1 for camera commands at 115200 (GPIO9 RX, GPIO10 TX).
4) Configure LED/buzzer pins LOW.
5) Register PS3 callbacks and start pairing with fixed MAC `20:00:00:00:82:62`; buzzer chirp on connect.
6) Attempt WiFi connection using credentials from io_config.h; remember first success to chirp once.

## Control Mapping (PS3)
- Left stick: translation (forward/back/left/right) with deadzone 20.
- Right stick Y: speed scalar (deadzone → ~200 PWM; full range maps 150-255).
- L1/R1 + forward/back: arc turns (differential speeds). L1 alone: rotate left; R1 alone: rotate right.
- D-pad left/right: sweep ultrasonic scan servo (25); Select: center scan servo.
- D-pad up/down: tilt camera servo (27) in 10° steps.
- Triangle: flip camera image (sends AT+VFLIP/HFLIP = 1). X: reset camera orientation.
- Circle: re-center scan servo. Square: short buzzer beep pair.
- L2: toggle LED on pin 2.

## Motion Logic
- All movement goes through moveWithTrim():
  - Applies gradual acceleration/deceleration every 30 ms toward the requested speed.
  - Uses per-side trim (left 0.66, right 1.0) so chassis tracks straight.
  - Sends PWM to vehicle.MoveWithTrim(direction, rightPWM, leftPWM).
- Stops instantly when no stick input; resets currentMotorSpeed for the next ramp.

## Camera and Buzzer
- Camera flip/reset issued via Serial1 AT commands (VFLIP/HFLIP).
- Buzzer helper playToneHz toggles pin manually (ESP32 tone-less); used for connect chirp and Square button beeps.

## WiFi Behavior
- WiFi STA retry loop runs in `loop()` without blocking PS3 handling.
- Attempts reconnect every 10 s when disconnected; initial connect timeout 15 s (7 s on retries).
- PS3 input is callback-driven, so main loop remains light.

## Practical Test Steps
1) Power ESP32 and press PS button on controller; wait for “PS3 Controller Connected!” and chirp.
2) Verify sticks move the chassis; adjust leftMotorTrim/rightMotorTrim in code if it drifts.
3) Test arc turns with L1/R1 + forward, rotations with L1/R1 alone.
4) Check camera tilt (D-pad up/down) and flip/reset (Triangle/X); observe video output.
5) Confirm ultrasonic servo sweeps (D-pad left/right) and centers (Circle/Select).
6) Verify WiFi logs an IP when available; ensure driving still responds if WiFi is down.
