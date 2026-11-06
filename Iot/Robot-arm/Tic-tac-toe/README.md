# Tic-tac-toe robot arm

Firmware and print files for a tic-tac-toe arm and its overhead board camera. Extract this directory at the repository root so the files land in `Iot/Robot-arm/Tic-tac-toe/`.

The human plays **yellow O**. The robot plays **red X**. Print the pieces in those colours. The camera tells them apart by colour.

Source revision of the firmware and models: `af8aa8e4c23615142922aa867aad2687fc3f2d36`.

## Directory layout

| Path | What it is |
| --- | --- |
| `44-tic-tac-toe-arm/` | Arduino UNO R4 WiFi arm. Direct servo output, motion on an FspTimer interrupt, on-device game page, and the client that reads the camera. |
| `45-board-camera/` | AI-Thinker ESP32-CAM. Separate PlatformIO project. Serves the board reading the arm consumes. |
| `3d-models/tictactoe/` | Board and piece meshes. See that folder's README for which file is the board and which files are the pieces. `two_part/` is the glue-together set. |

Open `44-tic-tac-toe-arm` and `45-board-camera` as separate PlatformIO projects. Each has its own `platformio.ini`.

## Hardware the firmware expects

Arm (`44-tic-tac-toe-arm`, default environment `uno_r4_wifi`):

- Board: Arduino UNO R4 WiFi (`renesas-ra` / `uno_r4_wifi`).
- Six arm servos on pins 0, 1, 2, 3, 4, and 5. Motor 0 is the gripper.
- A stand-alone test servo on D11. It is not part of the arm poses.
- Servo pulses are refreshed from a 200 Hz FspTimer interrupt so Wi-Fi and camera HTTP do not stall the motion plan.
- Wrist compensation values in `src/main.cpp` are the published calibration. They are geometry, not credentials.

Camera (`45-board-camera`, environment `esp32cam`):

- Board: AI-Thinker ESP32-CAM, with PSRAM (`BOARD_HAS_PSRAM`).
- Pin map in `src/main.cpp` is the AI-Thinker camera pinout.
- `fmt2rgb888` writes B, G, R on this board. The firmware reads byte 2 as red. That order was checked on the hardware.
- The camera sees the board rotated 180 degrees. Image index 0 is reported as cell 9 (`cell = 9 - image index`). `GET /board` returns nine characters, cell 1 first: `X` red, `O` yellow, `.` empty.
- The AI-Thinker ESP32-CAM module has no USB connector. Upload with a USB-to-serial adapter. Hold IO0 low while resetting to enter the bootloader.

## Prerequisites

- [PlatformIO](https://platformio.org/) IDE (VS Code extension) or PlatformIO Core (`pio` / `platformio` on the command line).
- A USB cable for the UNO R4 WiFi, and a USB-to-serial adapter for the ESP32-CAM.
- Python is not required to build. PlatformIO downloads the Renesas and ESP32 toolchains on the first build.

Serial monitor speed for both projects is 115200.

## Local configuration

Real Wi-Fi names, Wi-Fi passwords, API keys, and the camera's address stay on your machine. This tree ships only `*_example.h` templates. `.gitignore` in this directory ignores the filled-in headers, `.env` files, and PlatformIO build output.

Arm, copy these three files inside `44-tic-tac-toe-arm/include/` and edit the copies:

| Template | Copy to | What you put in the copy |
| --- | --- | --- |
| `wifi_secrets_example.h` | `wifi_secrets.h` | Your Wi-Fi SSID and password. |
| `gemini_config_example.h` | `gemini_config.h` | Your Gemini API key and the HTTPS generateContent URL you call. The template key and URL are dummies. The default game environment still includes this header, so the file has to exist before that environment will compile. |
| `camera_config_example.h` | `camera_config.h` | The camera's host and port. The template host `192.0.2.10` is a documentation address, not a device. Use the address the camera sketch prints. |

Camera, copy inside `45-board-camera/include/`:

| Template | Copy to | What you put in the copy |
| --- | --- | --- |
| `wifi_secrets_example.h` | `wifi_secrets.h` | The same Wi-Fi network the arm joins, so the arm can reach the camera. |

Do not commit `wifi_secrets.h`, `gemini_config.h`, or `camera_config.h`.

## Build and upload

From `44-tic-tac-toe-arm`, after the three local headers exist:

```bash
pio run -e uno_r4_wifi
pio run -e uno_r4_wifi -t upload
pio device monitor -b 115200
```

`uno_r4_wifi` builds `src/main.cpp`. That is the camera-integrated game: web pages at `/` and `/calibrate`, serial commands, and `GET /board` on the camera while the arm is parked.

Other environments in the same project, same board:

| Environment | Sketch | Local headers it needs |
| --- | --- | --- |
| `cell_sequence` | `src/cell_sequence.cpp` | None. Motion demo only. |
| `mapping` | `src/mapping.cpp` | `wifi_secrets.h` |
| `gemini` | `src/gemini_test.cpp` | `wifi_secrets.h`, `gemini_config.h` |
| `tictactoe` | `src/tic_tac_toe.cpp` | `wifi_secrets.h`, `gemini_config.h`. Serial game, no camera client. |

From `45-board-camera`, after `wifi_secrets.h` exists:

```bash
pio run -e esp32cam
pio run -e esp32cam -t upload
pio device monitor -b 115200
```

The camera sketch prints URLs for `/capture`, `/debug`, `/cells`, and `/board`. Put that host into the arm's `camera_config.h`. Frame crop (`boardX`, `boardY`, `boardW`, `boardH` in the camera sketch) is adjusted after looking at `/capture`.

## Models

`3d-models/tictactoe/README.md` names the board files and the piece files. The board is `board.3mf` and `board.stl` (180×180 mm). Red X and yellow O counters are the `counter_X*` and `counter_O*` files, including tower and `two_part/` variants. `tictactoe.solidcraft` is the editable SolidCraft source.

## What this publication deliberately changes

The firmware behaviour matches the source revision above: UNO R4 WiFi direct-servo / FspTimer motion, AI-Thinker ESP32-CAM, verified BGR conversion, 180-degree cell numbering, human yellow O, robot red X.

These files differ from that source tree so the publication does not carry local settings:

- The arm's camera address is not committed. Copy `camera_config_example.h` to `camera_config.h`.
- The arm gained `wifi_secrets_example.h` and `gemini_config_example.h`. The sketches already included `wifi_secrets.h` and `gemini_config.h`; those filled-in headers were never part of the source tree.
- One calibration note no longer contains a local machine path. Commands are unchanged.
- Print files live in `3d-models/tictactoe/` (including `two_part/`) instead of inside the arm project folder. Meshes, previews, and the SolidCraft source are the same files.
