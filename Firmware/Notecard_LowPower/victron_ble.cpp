// victron_ble.cpp - see victron_ble.h.
//
// Advertisement layout, after the 2-byte company id 0x02E1 (0xE1 0x02 on
// the wire). Confirmed against a SmartSolar 75/15 on the bench:
//   [0..1] 0x10 0x02, the "instant readout" record prefix (two bytes)
//   [2..3] product id, little-endian: 0xA075 is a SmartSolar MPPT 75/15
//   [4]    record type; 0x01 is a solar charger
//   [5..6] nonce, little-endian - also the AES-CTR initial counter
//   [7]    first byte of the encryption key, in clear, so a wrong key is
//          caught before any decrypting
//   [8..]  the record, AES-128-CTR encrypted, 12 bytes for a solar charger
//
// Solar charger record, little-endian:
//   [0]    device state      [1]    charger error
//   [2..3] battery V x0.01   [4..5] battery A x0.1 (signed)
//   [6..7] yield today kWh x0.01      [8..9] PV power W
// The record is under 16 bytes, so CTR never has to increment its counter and
// the question of which end it increments from never arises.
#include "victron_ble.h"

#include <Arduino.h>
#include <string.h>

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "mbedtls/aes.h"

namespace victron {

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool parseKey(const char* hex, uint8_t out[16]) {
  if (!hex || strlen(hex) != 32) return false;
  for (int i = 0; i < 16; i++) {
    int hi = hexNibble(hex[2 * i]), lo = hexNibble(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}

const char* deviceStateName(uint8_t s) {
  switch (s) {
    case 0: return "off";
    case 1: return "low power";
    case 2: return "fault";
    case 3: return "bulk";
    case 4: return "absorption";
    case 5: return "float";
    case 6: return "storage";
    case 7: return "equalize";
    case 245: return "starting up";
    case 247: return "auto equalize";
    case 252: return "external control";
    default: return "unknown";
  }
}

static bool decrypt(const uint8_t key[16], uint16_t nonce, const uint8_t* in, size_t len,
                    uint8_t* out) {
  if (len > 16) len = 16;
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  if (mbedtls_aes_setkey_enc(&ctx, key, 128) != 0) {
    mbedtls_aes_free(&ctx);
    return false;
  }
  uint8_t counter[16] = {0};
  counter[0] = (uint8_t)(nonce & 0xFF);
  counter[1] = (uint8_t)(nonce >> 8);
  uint8_t stream[16] = {0};
  size_t off = 0;
  int rc = mbedtls_aes_crypt_ctr(&ctx, len, &off, counter, stream, in, out);
  mbedtls_aes_free(&ctx);
  return rc == 0;
}

struct ScanState {
  const uint8_t* key;
  Reading* out;
  bool verbose;
  bool found;
  uint16_t seen;         // every advertiser the scan heard
  uint16_t victronSeen;  // of those, carrying Victron manufacturer data
};
static const uint16_t VERBOSE_DEVICE_LIMIT = 20;
static ScanState state;

static void hexDump(const uint8_t* p, size_t n) {
  for (size_t i = 0; i < n; i++) Serial.printf("%02x", p[i]);
}

class Callbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (state.found) return;
    state.seen++;
    // std::string on older cores, String on newer; both have c_str()/length().
    auto md = dev.getManufacturerData();
    const uint8_t* p = (const uint8_t*)md.c_str();
    size_t n = dev.haveManufacturerData() ? md.length() : 0;
    bool isVictron = n >= 12 && p[0] == 0xE1 && p[1] == 0x02;
    if (isVictron) state.victronSeen++;
    // In verbose mode name everything heard, so "nothing decoded" can be told
    // apart from "the Victron was not there" and "it was there but silent".
    if (state.verbose && state.seen <= VERBOSE_DEVICE_LIMIT) {
      Serial.printf("[BATT]   heard %s rssi %d", dev.getAddress().toString().c_str(), dev.getRSSI());
      if (dev.haveName()) Serial.printf(" name '%s'", dev.getName().c_str());
      if (n >= 2) Serial.printf(" mfr 0x%02x%02x (%u bytes)", p[1], p[0], (unsigned)n);
      Serial.println(isVictron ? "  <- Victron" : "");
    }
    if (!isVictron) return;
    const uint8_t* d = p + 2;
    size_t dn = n - 2;
    if (d[0] != 0x10) return;
    uint16_t pid = (uint16_t)(d[2] | (d[3] << 8));
    uint8_t rec = d[4];
    uint16_t nonce = (uint16_t)(d[5] | (d[6] << 8));
    uint8_t key0 = d[7];

    if (state.verbose) {
      Serial.printf("[BATT] victron adv from %s rssi %d: product 0x%04x record 0x%02x nonce %u raw ",
                    dev.getAddress().toString().c_str(), dev.getRSSI(), pid, rec, nonce);
      hexDump(p, n);
      Serial.println();
    }
    if (rec != 0x01) {
      if (state.verbose) Serial.println("[BATT]   not a solar charger record, skipped");
      return;
    }
    if (key0 != state.key[0]) {
      Serial.printf("[BATT]   key mismatch: advert says the key starts 0x%02x, config starts 0x%02x\n",
                    key0, state.key[0]);
      return;
    }
    if (dn < 8 + 10) return;
    uint8_t plain[16] = {0};
    if (!decrypt(state.key, nonce, d + 8, dn - 8, plain)) {
      Serial.println("[BATT]   AES failed");
      return;
    }
    Reading& r = *state.out;
    r.deviceState = plain[0];
    r.chargerError = plain[1];
    r.batteryV = (int16_t)(plain[2] | (plain[3] << 8)) * 0.01f;
    r.batteryA = (int16_t)(plain[4] | (plain[5] << 8)) * 0.1f;
    r.yieldTodayKwh = (uint16_t)(plain[6] | (plain[7] << 8)) * 0.01f;
    r.pvW = (uint16_t)(plain[8] | (plain[9] << 8));
    r.rssi = (int8_t)dev.getRSSI();
    r.atMs = millis();
    // A wrong key decrypts to noise; a real pack sits between these.
    r.valid = r.batteryV > 5.0f && r.batteryV < 20.0f;
    if (!r.valid) {
      Serial.print("[BATT]   decoded to nonsense, so the key is probably wrong: ");
      hexDump(plain, 10);
      Serial.println();
      return;
    }
    state.found = true;
    BLEDevice::getScan()->stop();  // got what we came for
  }
};

bool readSolarCharger(const uint8_t key[16], uint8_t scanSeconds, Reading& r, bool verbose) {
  static Callbacks callbacks;  // the scan object only stores the pointer
  r = Reading();
  state.key = key;
  state.out = &r;
  state.verbose = verbose;
  state.found = false;

  state.seen = 0;
  state.victronSeen = 0;

  BLEDevice::init("");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&callbacks);
  scan->setActiveScan(true);  // also collect scan responses, which carry the name
  scan->setInterval(100);
  scan->setWindow(99);
  scan->start(scanSeconds, false);
  scan->clearResults();
  BLEDevice::deinit(true);  // radio off and memory back while the node sleeps

  if (!state.found) {
    Serial.printf("[BATT] scan heard %u device(s), %u with Victron data\n", state.seen,
                  state.victronSeen);
    if (state.seen == 0) {
      Serial.println("[BATT]   nothing at all: BLE scanning itself is not working on this board");
    } else if (state.victronSeen == 0) {
      Serial.println("[BATT]   no Victron packet: Instant Readout is off, VictronConnect is "
                     "connected to it, or it is out of range");
    }
  }
  return state.found;
}

}  // namespace victron
