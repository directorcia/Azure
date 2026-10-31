# Tic-tac-toe Gemini arm

PlatformIO sketch for an Arduino UNO R4 WiFi (Arduino framework). The human plays O and places those pieces by hand. The arm plays X: it picks a piece up from a point beside the board and drops it on a square. A minimax search lists every equally best square; Gemini chooses one of them, and any other reply is rejected.

Servos stay detached until you arm them. The first arming move jumps to the home pose. Later moves are eased by a 200 Hz timer.

## Hardware

- Arduino UNO R4 WiFi
- Keyestudio Sensor Shield V5.2
- Six arm servos on D0–D5. M0 (D0) is the gripper
- A separate test servo on D11, used from the calibration page. Its angle is independent of the stored poses
- An external servo supply of about 6 V, into the shield, with its ground tied to the UNO ground

Power the servos from that external supply. The UNO 5 V pin is for logic. The serial monitor is USB Serial at 115200, so D0 and D1 are free as servo signals.

## Setup

From this directory:

```bash
cp include/wifi_secrets.example.h include/wifi_secrets.h
cp include/gemini_config.example.h include/gemini_config.h
```

Edit the two copies. `wifi_secrets.h` defines `wifi_secrets::ssid` and `wifi_secrets::password` (2.4 GHz network). `gemini_config.h` defines `GEMINI_API_KEY` and `GEMINI_URL`. The URL is `https://…` and includes the `generateContent` path. Both real headers are listed in `.gitignore`.

The sketch will not call Gemini while the key is still the placeholder `YOUR_GEMINI_API_KEY`.

```bash
pio run -e uno_r4_wifi
pio run -e uno_r4_wifi -t upload
pio device monitor -b 115200
```

`pio run` fetches Servo (Renesas backend), ArduinoJson, and ArduinoHttpClient. The UNO R4 core supplies WiFiS3, EEPROM, and FspTimer.

Support the arm and clear its travel before arming. Put an X piece on the pickup point before each robot move. Poses ship as compiled defaults and, once you calibrate, are stored in EEPROM.

## Web pages

After Wi-Fi connects, the serial log prints both addresses.

- `http://<device-ip>/` — game. You are O and the arm is X. Arm the servos, choose who starts, place your O on the physical board, then tap that square. The output panel follows the serial log.
- `http://<device-ip>/calibrate` — choose a target (squares 1–9, the off-board pickup, or home), jog M0–M5, save the angles, then use **Store to flash** so they survive a reset. The same page sets how far the gripper closes and can jog or sweep the D11 servo. A full-range sweep stops if this page stops polling or Wi-Fi drops. **Stop motion** leaves the servos holding; cut servo power if a stop cannot reach the board.

## Serial commands

115200 baud. `?` prints this list on the board. Commands are not case-sensitive. A line longer than 32 characters is discarded. `!` stops as soon as it is received.

| Command | What it does |
| --- | --- |
| `ARM` | Attach D0–D5 and command the home pose, gripper open. |
| `FIRST` | The arm (X) moves first. The servos must be armed, and the game waiting to choose. |
| `SECOND` | The human (O) moves first. |
| `1`–`9` | Place O on that square, on the human's turn, when the square is empty. |
| `RETRY` | Ask Gemini again. Used on the robot's turn after a failed request. |
| `NEW` | Clear the board and wait for `FIRST` or `SECOND`. |
| `BOARD` | Print the board. Empty squares show their numbers. |
| `STATUS` | Armed state, whose turn it is, the motion phase, Wi-Fi, and the commanded angles. |
| `TEST` | Run the rules and minimax self-test and print pass or fail. |
| `WEB` | Reset the Wi-Fi radio and restart the pages once the arm is idle. |
| `STOP` or `!` | Cancel motion and the game. Servos stay attached and hold their angle. |
| `RELEASE` | Cancel motion, then detach every servo, including D11. Support the arm first. |
| `?` or `HELP` | Print the command list. |

`STOP`, `RELEASE`, `BOARD`, `STATUS`, `TEST`, `HELP`, and `WEB` are accepted during a move. The other commands run when the arm is idle. If the self-test printed at startup fails, motion stays disabled.
