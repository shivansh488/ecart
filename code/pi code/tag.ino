#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// Define SPI pins for the ESP32
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define DW_CS     4

// Connection pins
const uint8_t PIN_RST = 27;  // Reset pin
const uint8_t PIN_IRQ = 34;  // IRQ pin
const uint8_t PIN_SS  = 4;   // SPI select pin

// Tag antenna delay defaults to 16384 (or as calibrated)
// The leftmost two bytes in this address become the short address.
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize SPI with the specified pins.
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  
  // Initialize DW1000 communication: Reset, CS, and IRQ pin setup.
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  
  // Attach callback handlers for ranging events.
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  
  // Start as a tag. The mode is set to MODE_LONGDATA_RANGE_LOWPOWER.
  // false indicates that we do not assign a random short address.
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void loop() {
  // Process ranging events.
  DW1000Ranging.loop();
}

// Callback: Called when a new range measurement is available.
void newRange() {
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(", ");
  Serial.println(DW1000Ranging.getDistantDevice()->getRange());
}

// Callback: Called when a new device is detected.
void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

// Callback: Called when a device becomes inactive.
void inactiveDevice(DW1000Device *device) {
  Serial.print("Device removed: ");
  Serial.println(device->getShortAddress(), HEX);
}
