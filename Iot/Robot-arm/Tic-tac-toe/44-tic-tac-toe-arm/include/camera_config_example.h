#pragma once

#include <stdint.h>

// Copy this file to camera_config.h (same folder) and set the address of the
// overhead board camera (project 45-board-camera). camera_config.h is
// gitignored. Do not commit it.
//
// 192.0.2.10 is a documentation placeholder (TEST-NET-1). It is not a camera.
// After the camera prints its address on the serial monitor, put that host
// here. Keep Wi-Fi keys and API keys out of this file.
//
// GET http://CAMERA_HOST:CAMERA_PORT/board
// Body is 9 plain-text characters, cell 1 first through cell 9:
//   'X' red piece, 'O' yellow piece, '.' empty.
// Cell numbers match this arm: 1 2 3 / 4 5 6 / 7 8 9.

constexpr char CAMERA_HOST[] = "192.0.2.10";
constexpr uint16_t CAMERA_PORT = 80;
