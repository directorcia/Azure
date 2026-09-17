#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Human-readable name shown to BLE scanners as the device name.
// Keep this short and unique enough to identify this board in a crowded area.
static const char *kBeaconName = "ACEBOTT-Beacon-01";

// Custom 128-bit BLE service UUID used for advertisement filtering.
// Client apps can scan specifically for this UUID to find this beacon quickly.
static const char *kServiceUuid = "7d0b4b0f-3d55-4d48-a8f5-61c2edb0a101";

// Global advertising handle so setup() can configure/start advertising
// and loop() can remain lightweight.
BLEAdvertising *advertising = nullptr;

// Timestamp of the last periodic status message, in milliseconds since boot.
// Used with millis() to print status every 5 seconds without blocking logic.
unsigned long lastStatusPrintMs = 0;

void setup() {
  // Start USB/serial logging so we can observe initialization and runtime status.
  Serial.begin(115200);

  // Give the serial port a short moment to initialize before printing.
  // This helps ensure early boot messages are visible in the serial monitor.
  delay(500);

  // Initialize BLE stack and register this board's advertised device name.
  BLEDevice::init(kBeaconName);

  // Create a BLE server object. Even for simple beacon behavior, many BLE
  // frameworks require a server/service context for service advertisements.
  BLEServer *server = BLEDevice::createServer();

  // Create and start a service so its UUID can be included in advertisements.
  // We are not adding characteristics here; this is a beacon-style presence signal.
  BLEService *service = server->createService(kServiceUuid);
  service->start();

  // Get the global advertising object from the BLE stack.
  advertising = BLEDevice::getAdvertising();

  // Include the service UUID in advertisement packets. This enables scanner
  // apps/clients to filter by UUID and discover this device efficiently.
  advertising->addServiceUUID(kServiceUuid);

  // Enable scan response so additional metadata can be returned when scanners
  // actively request more data.
  advertising->setScanResponse(true);

  // Preferred connection interval hints (common ESP32 BLE compatibility values).
  // Some mobile clients behave better with these preferences set.
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);

  // Begin broadcasting BLE advertisement packets.
  advertising->start();

  // Print startup confirmation and identity details for troubleshooting.
  Serial.println("BLE beacon started");
  Serial.println(kBeaconName);
  Serial.println(kServiceUuid);
}

void loop() {
  // Current uptime from boot in milliseconds (rolls over eventually, which is
  // safe for subtraction-based interval checks like the one below).
  const unsigned long now = millis();

  // Emit a heartbeat every 5 seconds so we know the sketch is alive and still
  // advertising. Uses non-blocking elapsed-time logic for periodic behavior.
  if (now - lastStatusPrintMs >= 5000) {
    // Record this print time before output so timing remains stable even if
    // serial printing takes a small amount of time.
    lastStatusPrintMs = now;

    // Runtime status message: advertised name, service UUID, and uptime.
    Serial.print("Advertising as ");
    Serial.print(kBeaconName);
    Serial.print(" | service UUID: ");
    Serial.print(kServiceUuid);
    Serial.print(" | uptime(ms): ");
    Serial.println(now);
  }

  // Short sleep to reduce CPU usage and serial spam while still checking the
  // heartbeat interval frequently enough for this simple beacon sketch.
  delay(1000);
}
