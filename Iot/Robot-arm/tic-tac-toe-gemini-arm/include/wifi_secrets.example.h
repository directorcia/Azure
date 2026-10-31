#pragma once

// Copy this file to wifi_secrets.h in this same folder, then replace the
// placeholders. wifi_connection.h includes wifi_secrets.h, and that real
// file is gitignored. Do not commit SSIDs or passwords.
//
// Names consumed by wifi_connection.h:
//   wifi_secrets::ssid
//   wifi_secrets::password
//
// The UNO R4 WiFi radio joins 2.4 GHz networks.

namespace wifi_secrets {
inline constexpr const char *ssid = "YOUR_WIFI_SSID";
inline constexpr const char *password = "YOUR_WIFI_PASSWORD";
}
