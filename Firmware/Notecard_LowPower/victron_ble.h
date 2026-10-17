// victron_ble.h - read the charge controller's "instant readout" over BLE.
//
// A Victron SmartSolar advertises its live state every second or so as an
// encrypted BLE manufacturer-data record. The ESP32-S3 has a BLE radio, so
// the node can read battery voltage, charge current and PV power with no
// extra hardware - the only input is the 16-byte key VictronConnect shows
// under Product info -> Instant readout via Bluetooth.
#pragma once

#include <stdint.h>

namespace victron {

struct Reading {
  bool valid = false;
  uint8_t deviceState = 0;   // 3 bulk, 4 absorption, 5 float, 0 off ...
  uint8_t chargerError = 0;  // 0 = none
  float batteryV = 0;        // V
  float batteryA = 0;        // A, positive = charging
  float yieldTodayKwh = 0;   // kWh since midnight (controller's clock)
  uint16_t pvW = 0;          // W
  int8_t rssi = 0;           // dBm
  uint32_t atMs = 0;         // millis() when decoded
};

// 32 hex characters -> 16 bytes. Returns false if the string is malformed.
bool parseKey(const char* hex, uint8_t out[16]);

// Scan for up to scanSeconds, decode the first solar-charger record that
// decrypts with key, and fill r. Blocks for the scan. BLE is brought up for
// the scan and shut down after it, so it costs nothing while the node sleeps.
bool readSolarCharger(const uint8_t key[16], uint8_t scanSeconds, Reading& r, bool verbose);

const char* deviceStateName(uint8_t state);

}  // namespace victron
