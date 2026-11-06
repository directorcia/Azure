# 45-board-camera

AI-Thinker ESP32-CAM firmware. Open this folder in PlatformIO (it contains `platformio.ini`). It is a separate project from the UNO R4 arm.

`src/main.cpp` sets the AI-Thinker camera pins and `BOARD_HAS_PSRAM`. `fmt2rgb888` is B, G, R on this board (checked on the hardware). The camera looks at the board rotated 180 degrees, so the reported cell is `9 - image index`. `GET /board` returns nine characters, cell 1 first: `X` red, `O` yellow, `.` empty.

The module has no USB connector. Upload with a USB-to-serial adapter, holding IO0 low while resetting to enter the bootloader.

## Local header

Copy `include/wifi_secrets_example.h` to `include/wifi_secrets.h` and fill in the Wi-Fi network. Do not commit `wifi_secrets.h`. Use the same network as the arm.

## Build

```bash
pio run -e esp32cam
pio run -e esp32cam -t upload
pio device monitor -b 115200
```

The serial monitor prints the device address and the `/capture`, `/debug`, `/cells`, and `/board` paths. Put that host into the arm's `camera_config.h` (copied from `camera_config_example.h` in the arm project). Adjust `boardX`, `boardY`, `boardW`, and `boardH` after looking at `/capture`.
