# 44-tic-tac-toe-arm

UNO R4 WiFi firmware for the tic-tac-toe arm. Open this folder in PlatformIO (it contains `platformio.ini`).

The default environment `uno_r4_wifi` builds `src/main.cpp`: the camera-integrated game. Servos are driven directly. A 200 Hz FspTimer interrupt updates the motion plan so Wi-Fi and the camera HTTP client do not stall the servos. Arm servos use pins 0–5. D11 is a stand-alone test servo and is not stored in the arm poses.

The human piece is yellow O. The robot piece is red X.

## Local headers

Copy the templates in `include/` and edit the copies. Do not commit the copies.

- `wifi_secrets_example.h` → `wifi_secrets.h`
- `gemini_config_example.h` → `gemini_config.h`
- `camera_config_example.h` → `camera_config.h`

`camera_config.h` is the address of the companion sketch in `../45-board-camera/`. The template host is a documentation placeholder.

## Build

```bash
pio run -e uno_r4_wifi
pio run -e uno_r4_wifi -t upload
```

Other environments: `cell_sequence`, `mapping`, `gemini`, `tictactoe`. See `../README.md` for which local headers each one needs.

Board and piece meshes are in `../3d-models/tictactoe/`, not in this folder. Wrist calibration notes in this folder describe the compensation table in `src/main.cpp`.
