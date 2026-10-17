// BioBot sensor node firmware - LOW POWER VARIANT (under test, not merged)
//
// Differs from Firmware/Notecard in four ways:
//   - samples once a minute instead of every 30 s, and sends a batch every
//     20 min instead of every 10;
//   - light-sleeps the ESP32 in 5 s ticks between tasks (RAM is kept, so the
//     detector's baselines survive), unless the USB serial monitor is open;
//   - reads battery voltage, charge current and PV power from the Victron
//     charge controller over BLE, with no extra hardware, and adds them to
//     every batch;
//   - duty-cycles the BMV080 at 30 s to match the slower sampling.
// Everything else is the main firmware, line for line.
//
// Reads a Bosch BME680/BME688 (temperature, humidity, pressure, gas
// resistance) and a Bosch BMV080 (PM1 / PM2.5 / PM10) over I2C, runs
// on-device anomaly detection on every sample, and ships data through a
// Blues Notecard cellular modem to Notehub.
//
// Three kinds of notes leave the node:
//   device.qo  once at boot: registers the node with the BioBot API
//   data.qo    every DATA_SEND_INTERVAL_MS: a batch of readings
//   alert.qo   immediately when the anomaly detector raises, escalates,
//              repeats or clears an event (synced right away)
//
// Per-node identity lives in config.h (see config.example.h). The node holds
// no API credentials: the Notehub route adds the server's token when it
// forwards each note.
// The anomaly logic lives in anomaly.h / anomaly.cpp and has host-side tests
// in test/test_anomaly.cpp.

#include <Arduino.h>
#include <Wire.h>
#include <time.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "SparkFun_BMV080_Arduino_Library.h"
#include <Notecard.h>
#include <esp_sleep.h>

#include "config.h"
#include "anomaly.h"
#include "victron_ble.h"

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------
static const uint32_t SERIAL_SPEED = 115200;

static const int I2C_SDA_PIN = A4;
static const int I2C_SCL_PIN = A5;
static const uint32_t I2C_CLOCK_HZ = 100000;          // slow and robust for long leads
static const uint8_t BME_ADDR_PRIMARY = 0x77;
static const uint8_t BME_ADDR_SECONDARY = 0x76;
static const uint8_t BMV080_ADDR = 0x57;
static const uint8_t NOTECARD_ADDR = 0x17;

static const uint32_t SENSOR_READ_INTERVAL_MS = 60UL * 1000UL;        // 60 s
static const uint32_t DATA_SEND_INTERVAL_MS = 20UL * 60UL * 1000UL;   // 20 min
static const uint32_t SEND_RETRY_INTERVAL_MS = 60UL * 1000UL;         // after a failed batch
static const uint32_t REGISTER_RETRY_INTERVAL_MS = 5UL * 1000UL;
static const uint32_t STATUS_PRINT_INTERVAL_MS = 60UL * 1000UL;

// Readings per batch at the intervals above, plus headroom so a failed send
// can be retried without losing samples.
static const uint8_t BATCH_SIZE = DATA_SEND_INTERVAL_MS / SENSOR_READ_INTERVAL_MS;  // 20
static const uint8_t BUFFER_CAPACITY = BATCH_SIZE + BATCH_SIZE / 2;                // 30

// A BME680/688 that fails at boot or stops answering mid-run is retried
// rather than written off: a brown-out during a cellular transmit, or a
// connector nudged on the bench, should not cost the node every later sample.
// Recovery is cheap, so it runs on the first failed read; the interval only
// stops it looping tightly.
static const uint32_t BME_RETRY_INTERVAL_MS = 10UL * 1000UL;

// The gas heater is the only part of a reading that draws real current, so it
// is what fails first on a sensor whose supply cannot hold up - and it is the
// only channel the node can afford to give up. Temperature, humidity, pressure
// and PM2.5 all keep working without it.
static const uint16_t BME_HEATER_C = 320;
static const uint16_t BME_HEATER_MS = 150;
static const uint8_t BME_HEATER_OFF_AFTER_FAILURES = 3;
static const uint32_t BME_HEATER_RETRY_INTERVAL_MS = 60UL * 60UL * 1000UL;

// Continuous mode cost 182 mW to produce a sample every second, 29 of every
// 30 of which were thrown away - a third of the node's whole power budget.
// Duty cycling wakes the sensor once per period instead. 30 s is shorter than
// the 60 s sample interval, so every sample window holds a fresh result no
// matter how the two clocks drift. Results are cached as they arrive and a
// sample reads the cache, so nothing ever waits on the sensor.
static const uint16_t BMV080_DUTY_PERIOD_S = 30;
static const uint32_t PM_SERVICE_INTERVAL_MS = 250;                        // cache refresh
static const uint32_t PM_STALE_MS = 3UL * BMV080_DUTY_PERIOD_S * 1000UL;   // older is a fault

// Its driver is retried the same way the BME's is: a sensor that would not open
// at boot is not necessarily broken.
static const uint8_t BMV080_BOOT_ATTEMPTS = 3;
static const uint32_t BMV080_RETRY_INTERVAL_MS = 60UL * 1000UL;

// Wall-clock time comes from the Notecard over the shared I2C bus, so it is
// anchored once and carried forward on millis() instead of being re-fetched
// for every sample.
static const uint32_t TIME_REFRESH_INTERVAL_MS = 60UL * 60UL * 1000UL;

static const uint8_t SETUP_WARMUP_READS = 5;   // gas heater warmup in setup()
static const uint8_t LOOP_WARMUP_READS = 2;    // early loop samples not buffered

// Sleep. Between tasks the ESP32 light-sleeps in short ticks rather than one
// long stretch, so the BMV080's driver still gets serviced within a few
// seconds of each result it produces. millis() keeps counting through light
// sleep, so every interval above keeps working unchanged.
static const uint32_t TICK_SLEEP_MS = 5000;
static const uint32_t NO_SLEEP_AFTER_BOOT_MS = 90UL * 1000UL;   // let setup settle
static const uint32_t NO_SLEEP_UNREGISTERED_MS = 10UL * 60UL * 1000UL;  // but not forever

// Battery, read from the charge controller over BLE. A scan costs ~90 mA for
// up to VICTRON_SCAN_S, so it is done rarely; the pack's voltage moves slowly.
static const uint32_t BATTERY_READ_INTERVAL_MS = 10UL * 60UL * 1000UL;
static const uint32_t BATTERY_FIRST_READ_MS = 30UL * 1000UL;
static const uint32_t BATTERY_STALE_MS = 30UL * 60UL * 1000UL;  // older is not attached
static const uint8_t VICTRON_SCAN_S = 4;
static const float BATTERY_LOW_V = 10.5f;   // 3S LiPo: 3.5 V/cell, time to worry

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
struct Reading {
  float temperatureC;
  float humidityPct;
  float pressureHpa;
  float gasKOhm;
  float altitudeM;
  float pm1, pm25, pm10;
  bool envValid;   // temperature, humidity, pressure, altitude usable
  bool gasValid;   // gas resistance usable
  bool pmValid;
  bool pmObstructed;
  bool batteryValid;      // a fresh Victron readout was attached
  float batteryV, chargeA, pvW;
  uint8_t chargerState;   // Victron code: 0 off, 3 bulk, 4 absorption, 5 float
  char timestampIso[24];  // "YYYY-MM-DDTHH:MM:SSZ"
  biobot::Severity severity;
  uint8_t anomalyScore;
};

static Adafruit_BME680 bme(&Wire);
static SparkFunBMV080 bmv080;
static Notecard notecard;
// The detector's defaults assume 30 s samples. At 60 s, halve the sample
// counts and double the learning rate so the same minutes of behaviour fall
// out: a 10-minute warmup and a ~100-minute baseline time constant.
static biobot::Config detectorConfig() {
  biobot::Config c;
  c.warmupSamples = 10;
  c.baselineAlpha = 0.01f;
  return c;
}
static biobot::Detector detector(detectorConfig());

static Reading buffer[BUFFER_CAPACITY];
static uint8_t bufferCount = 0;

static bool bmeReady = false;
static uint8_t bmeAddr = 0;         // address it answered on, 0 if never found
static uint8_t bmeFailures = 0;     // consecutive failed reads
static uint32_t lastBmeRetryMs = 0;
static bool bmeHeaterOn = true;
static bool bmv080Ready = false;
static uint32_t lastBmv080RetryMs = 0;

// The particulate sensor's most recent result, filled in by taskPm().
struct PmSample {
  float pm1, pm25, pm10;
  bool obstructed;
  uint32_t atMs;
  bool valid;
};
static PmSample pmCache = {0, 0, 0, false, 0, false};
static uint32_t lastPmServiceMs = 0;

static victron::Reading battery;       // latest decoded readout
static uint32_t lastBatteryReadMs = 0;
static uint8_t victronKey[16];
static bool victronKeyOk = false;
static bool lowBatteryLogged = false;

static uint32_t sleepTicks = 0;        // light-sleep entries
static uint32_t sleptMs = 0;           // time actually spent asleep
static bool sleepRejectedLogged = false;

// Defined with the battery and sleep tasks below; used by the sample and
// status tasks above them. Spelled out rather than left to the IDE's
// prototype generation.
static const char* batteryStatus();
static void attachBattery(Reading& r);
static uint32_t bmeRetries = 0;     // reads that needed a second attempt
static bool pmFaultLogged = false;  // the per-sample PM complaint is printed once
static uint32_t lastHeaterRetryMs = 0;

static bool timeValid = false;       // the Notecard has given us the time once
static time_t timeBaseUnix = 0;      // its answer
static uint32_t timeBaseMs = 0;      // millis() when that answer arrived
static bool deviceRegistered = false;
static uint32_t sampleCount = 0;
static uint32_t droppedReadings = 0;

static uint32_t lastSensorReadMs = 0;
static uint32_t lastDataSendMs = 0;
static uint32_t lastRegisterMs = 0;
static uint32_t lastStatusMs = 0;
static uint32_t nextSendNotBeforeMs = 0;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
static float pressureToAltitudeM(float hPa, float seaLevelHpa = 1013.25f) {
  return 44330.0f * (1.0f - powf(hPa / seaLevelHpa, 0.1903f));
}

// Ask the Notecard for UNIX time. Returns false until it has synced with the
// network. This is I2C traffic on the bus the sensors sit on, so callers go
// through timeRefresh() rather than calling it per sample.
static bool notecardFetchUnix(time_t& out) {
  J* req = notecard.newRequest("card.time");
  if (!req) return false;
  J* rsp = notecard.requestAndResponse(req);
  if (!rsp) return false;
  double unixTime = JGetNumber(rsp, "time");
  notecard.deleteResponse(rsp);
  if (unixTime <= 0) return false;
  out = (time_t)unixTime;
  return true;
}

// Re-anchor the clock at most once an hour, and keep trying until the first
// answer arrives. millis() is good to a few seconds an hour, which is well
// inside what a 30 s sample interval needs.
static void timeRefresh() {
  if (timeValid && (uint32_t)(millis() - timeBaseMs) < TIME_REFRESH_INTERVAL_MS) return;
  time_t fetched;
  if (!notecardFetchUnix(fetched)) return;
  timeBaseUnix = fetched;
  timeBaseMs = millis();
  if (!timeValid) Serial.println("[TIME] network time acquired");
  timeValid = true;
}

// Current time as ISO 8601 UTC, from the cached anchor plus elapsed millis().
static bool nowIso(char* out, size_t outSize) {
  if (!timeValid) return false;
  time_t raw = timeBaseUnix + (time_t)((uint32_t)(millis() - timeBaseMs) / 1000UL);
  struct tm* utc = gmtime(&raw);
  if (!utc) return false;
  strftime(out, outSize, "%Y-%m-%dT%H:%M:%SZ", utc);
  return true;
}

// Queue a note. Ownership of body passes to the request.
static bool noteAdd(const char* file, J* body, bool sync) {
  J* req = notecard.newRequest("note.add");
  if (!req) {
    JDelete(body);
    return false;
  }
  JAddStringToObject(req, "file", file);
  JAddBoolToObject(req, "sync", sync);
  JAddItemToObject(req, "body", body);
  return notecard.sendRequest(req);
}

static void addReading(J* arr, const char* type, const char* unit, float value,
                       const char* timestamp) {
  J* r = JCreateObject();
  if (!r) return;
  JAddStringToObject(r, "readingType", type);
  JAddStringToObject(r, "unit", unit);
  JAddNumberToObject(r, "value", value);
  JAddStringToObject(r, "timestamp", timestamp);
  JAddItemToArray(arr, r);
}

// ---------------------------------------------------------------------------
// Notecard: setup, registration, batches, alerts
// ---------------------------------------------------------------------------
static void notecardBegin() {
  Serial.println("[NC] Initializing Notecard...");
  notecard.begin();
  notecard.setDebugOutputStream(Serial);

  J* ver = notecard.newRequest("card.version");
  J* rsp = ver ? notecard.requestAndResponse(ver) : nullptr;
  if (rsp) {
    Serial.printf("[NC] Notecard %s\n", JGetString(rsp, "version"));
    notecard.deleteResponse(rsp);
  } else {
    Serial.println("[NC] WARNING: no response to card.version");
  }

  J* req = notecard.newRequest("hub.set");
  if (req) {
    JAddStringToObject(req, "product", NOTEHUB_PRODUCT_UID);
    JAddStringToObject(req, "mode", NOTEHUB_MODE);
    notecard.sendRequest(req);
  }
  Serial.println("[NC] Ready");
}

static bool sendRegistration() {
  J* body = JCreateObject();
  if (!body) return false;
  JAddStringToObject(body, "request_type", "create_device");
  JAddStringToObject(body, "deviceId", DEVICE_ID);
  JAddStringToObject(body, "name", DEVICE_NAME);
  JAddStringToObject(body, "type", DEVICE_TYPE);
  JAddStringToObject(body, "status", "active");
  JAddNumberToObject(body, "timestamp", millis());
  return noteAdd("device.qo", body, true);
}

static bool sendBatch(const Reading* readings, uint8_t count) {
  J* body = JCreateObject();
  if (!body) return false;
  J* requests = JAddArrayToObject(body, "requests");
  if (!requests) {
    JDelete(body);
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    const Reading& r = readings[i];
    J* item = JCreateObject();
    if (!item) continue;
    JAddStringToObject(item, "deviceId", DEVICE_ID);

    // Only channels that actually read are sent: a missing channel is better
    // than a zero the dashboard would plot as a real measurement.
    J* arr = JAddArrayToObject(item, "readings");
    if (r.envValid) {
      addReading(arr, "temperature", "°C", r.temperatureC, r.timestampIso);
      addReading(arr, "humidity", "%", r.humidityPct, r.timestampIso);
      addReading(arr, "pressure", "hPa", r.pressureHpa, r.timestampIso);
      addReading(arr, "altitude", "m", r.altitudeM, r.timestampIso);
    }
    if (r.gasValid) addReading(arr, "gasResistance", "kΩ", r.gasKOhm, r.timestampIso);
    if (r.batteryValid) {
      addReading(arr, "batteryVoltage", "V", r.batteryV, r.timestampIso);
      addReading(arr, "chargeCurrent", "A", r.chargeA, r.timestampIso);
      addReading(arr, "solarPower", "W", r.pvW, r.timestampIso);
      // Victron device state code; 3/4/5 mean the panel is charging.
      addReading(arr, "chargerState", "code", r.chargerState, r.timestampIso);
    }
    if (r.pmValid) {
      addReading(arr, "pm1", "μg/m³", r.pm1, r.timestampIso);
      addReading(arr, "pm25", "μg/m³", r.pm25, r.timestampIso);
      addReading(arr, "pm10", "μg/m³", r.pm10, r.timestampIso);
      addReading(arr, "obstructed", "bool", r.pmObstructed ? 1 : 0, r.timestampIso);
    }

    J* anomaly = JAddObjectToObject(item, "anomaly");
    if (anomaly) {
      JAddStringToObject(anomaly, "severity", biobot::Detector::severityName(r.severity));
      JAddNumberToObject(anomaly, "score", r.anomalyScore);
    }
    JAddItemToArray(requests, item);
  }

  return noteAdd("data.qo", body, true);
}

static bool sendAlert(const Reading& r, const biobot::Result& res, const char* event) {
  J* body = JCreateObject();
  if (!body) return false;
  JAddStringToObject(body, "request_type", "anomaly");
  JAddStringToObject(body, "deviceId", DEVICE_ID);
  JAddStringToObject(body, "event", event);
  JAddStringToObject(body, "severity", biobot::Detector::severityName(res.severity));
  JAddNumberToObject(body, "score", res.score);
  if (r.timestampIso[0]) JAddStringToObject(body, "timestamp", r.timestampIso);
  JAddNumberToObject(body, "uptimeMs", millis());

  J* signals = JAddArrayToObject(body, "signals");
  for (uint16_t rest = res.signals; biobot::Signal s = biobot::Detector::nextSignal(rest);) {
    JAddItemToArray(signals, JCreateString(biobot::Detector::signalName(s)));
  }

  J* readings = JAddObjectToObject(body, "readings");
  if (readings) {
    JAddNumberToObject(readings, "temperature", r.temperatureC);
    JAddNumberToObject(readings, "humidity", r.humidityPct);
    JAddNumberToObject(readings, "pressure", r.pressureHpa);
    JAddNumberToObject(readings, "gasResistance", r.gasKOhm);
    if (r.pmValid) {
      JAddNumberToObject(readings, "pm1", r.pm1);
      JAddNumberToObject(readings, "pm25", r.pm25);
      JAddNumberToObject(readings, "pm10", r.pm10);
      JAddBoolToObject(readings, "obstructed", r.pmObstructed);
    }
  }

  J* baseline = JAddObjectToObject(body, "baseline");
  if (baseline) {
    JAddBoolToObject(baseline, "ready", res.baselineReady);
    JAddNumberToObject(baseline, "temperature", res.baselineTempC);
    JAddNumberToObject(baseline, "humidity", res.baselineHumidityPct);
    JAddNumberToObject(baseline, "gasResistance", res.baselineGasKOhm);
    JAddNumberToObject(baseline, "pm25", res.baselinePm25);
  }

  return noteAdd("alert.qo", body, true);
}

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
// Raw probe. 0 means the device acknowledged; anything else is the TwoWire
// error, and which error it is decides the diagnosis:
//   2 - address not acknowledged: the chip is unpowered, or gone
//   3 - data not acknowledged
//   5 - timeout: the bus itself is being held, so no address can answer
static uint8_t i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

static bool i2cPresent(uint8_t addr) { return i2cProbe(addr) == 0; }

// Tear the I2C controller down and bring it back up. A transaction that times
// out can leave the peripheral wedged so every later transfer fails, which
// looks exactly like a sensor that vanished.
static void i2cBusReset() {
  Wire.end();
  delay(10);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(10);
}

// Probe all three known devices and say what the bus looks like. If the BME is
// silent while the Notecard still answers, the sensor lost power or died; if
// nothing answers, the bus is stuck and worth resetting.
static void i2cDiagnose() {
  uint8_t bmeErr = i2cProbe(bmeAddr != 0 ? bmeAddr : BME_ADDR_SECONDARY);
  uint8_t noteErr = i2cProbe(NOTECARD_ADDR);
  uint8_t pmErr = i2cProbe(BMV080_ADDR);
  Serial.printf("[I2C] probe errors - bme %u | notecard %u | bmv080 %u\n", bmeErr, noteErr,
                pmErr);
  if (bmeErr != 0 && noteErr != 0 && pmErr != 0) {
    Serial.println("[I2C] nothing on the bus answers - resetting the controller");
    i2cBusReset();
  }
}

// Print every address that answers, so the boot log carries the same evidence
// an I2C scanner sketch would give. Expect 0x17 (Notecard), 0x57 (BMV080) and
// one of 0x76/0x77 (BME680/688).
static void i2cScan() {
  Serial.print("[SETUP] I2C scan:");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2cPresent(addr)) {
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }
  if (found == 0) Serial.print(" nothing answered");
  Serial.printf("  (%u device%s)\n", found, found == 1 ? "" : "s");
}

// warmup runs the gas heater settling reads; skipped when re-initialising
// mid-run so a recovery does not block the loop for ten seconds.
static bool bmeBegin(bool warmup) {
  bmeAddr = 0;
  // 0x76 first: that is where a BME688 sits with SDO low, which is what rev A
  // of the board leaves it at. Probe before calling begin(), because begin()
  // against an absent address is a guaranteed failed transaction on a bus we
  // already suspect.
  const uint8_t candidates[] = {BME_ADDR_SECONDARY, BME_ADDR_PRIMARY};
  for (uint8_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]) && bmeAddr == 0; i++) {
    uint8_t err = i2cProbe(candidates[i]);
    if (err != 0) {
      Serial.printf("[BME] 0x%02X no answer (I2C error %u)\n", candidates[i], err);
    } else if (bme.begin(candidates[i])) {
      bmeAddr = candidates[i];
    } else {
      Serial.printf("[BME] 0x%02X answers but would not identify itself\n", candidates[i]);
    }
  }
  if (bmeAddr == 0) {
    Serial.println("[BME] unavailable - see the probe errors above");
    return false;
  }
  Serial.printf("[BME] found at 0x%02X\n", bmeAddr);

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  if (bmeHeaterOn) {
    bme.setGasHeater(BME_HEATER_C, BME_HEATER_MS);
  } else {
    bme.setGasHeater(0, 0);  // disabled: see the heater fallback below
  }
  Serial.printf("[BME] gas heater %s\n", bmeHeaterOn ? "on" : "off (running without gas)");

  if (warmup) {
    Serial.println("[BME] Settling...");
    uint8_t ok = 0;
    for (uint8_t i = 0; i < SETUP_WARMUP_READS; i++) {
      if (bme.performReading()) {
        ok++;
        Serial.printf("[BME] Warmup %u/%u - gas %.1f kΩ\n", i + 1, SETUP_WARMUP_READS,
                      bme.gas_resistance / 1000.0f);
      } else {
        Serial.printf("[BME] Warmup %u/%u - read failed\n", i + 1, SETUP_WARMUP_READS);
      }
      delay(2000);
    }
    // Init succeeded but no reading did: the difference between the two is the
    // heater, so drop it now rather than spend the first two minutes failing.
    if (ok == 0 && bmeHeaterOn) {
      Serial.println("[BME] no warmup read succeeded with the gas heater on - disabling it");
      bmeHeaterOn = false;
      bme.setGasHeater(0, 0);
      lastHeaterRetryMs = millis();
    }
  }
  return true;
}

// Throttled recovery, used both when the sensor was missing at boot and when a
// read fails. Diagnoses the bus first, since a wedged controller has to be
// reset before re-initialising the sensor can possibly work.
static void bmeRecover(const char* why) {
  if ((uint32_t)(millis() - lastBmeRetryMs) < BME_RETRY_INTERVAL_MS) return;
  lastBmeRetryMs = millis();
  Serial.printf("[BME] recovering (%s)\n", why);
  i2cDiagnose();
  bmeReady = bmeBegin(false);
  Serial.printf("[BME] recovery %s\n", bmeReady ? "succeeded" : "failed, will retry");
}

// Try the heater again occasionally: whatever starved it may be temporary, and
// gas resistance is a real detection signal worth getting back.
static void bmeHeaterRetry() {
  if (bmeHeaterOn || !bmeReady) return;
  if ((uint32_t)(millis() - lastHeaterRetryMs) < BME_HEATER_RETRY_INTERVAL_MS) return;
  lastHeaterRetryMs = millis();
  Serial.println("[BME] re-enabling the gas heater to see whether it holds up now");
  bmeHeaterOn = true;
  lastBmeRetryMs = millis() - BME_RETRY_INTERVAL_MS;  // let recovery run at once
  bmeRecover("heater re-enabled");
}

// Service the particulate sensor's driver and cache whatever it produces. In
// duty-cycling mode the sensor measures on its own clock and readSensor()
// returns true exactly once per result, so this runs from the loop all the
// time and the sample task reads the cache rather than the sensor.
static void taskPm() {
  if (!bmv080Ready) return;
  if ((uint32_t)(millis() - lastPmServiceMs) < PM_SERVICE_INTERVAL_MS) return;
  lastPmServiceMs = millis();
  if (!bmv080.readSensor()) return;
  pmCache.pm1 = bmv080.PM1();
  pmCache.pm25 = bmv080.PM25();
  pmCache.pm10 = bmv080.PM10();
  pmCache.obstructed = bmv080.isObstructed();
  pmCache.atMs = millis();
  pmCache.valid = true;
}

// Read the sensor, retrying once. Firmware/diagnostics/bme_diag reads this
// sensor 8 times out of 8 with the node's exact settings when nothing else is
// on the bus, so a read that fails here fails because of what else shares the
// bus - and the first transfer after the Notecard has been talking is the one
// that gets lost. A second attempt costs a few hundred milliseconds and tells
// us plainly which of the two it was.
static bool bmeReadWithRetry() {
  if (bme.performReading()) return true;
  uint8_t err = i2cProbe(bmeAddr);
  delay(20);
  if (!bme.performReading()) return false;
  bmeRetries++;
  // Say this once with the reasoning, then keep the count on the status line
  // rather than repeating it every 30 s for the life of the node.
  if (bmeRetries == 1) {
    Serial.printf("[SENS] BME read failed then succeeded on retry (probe between: %u) - the "
                  "first transfer after Notecard traffic gets lost; retries are counted on "
                  "the status line from here on\n", err);
  }
  return true;
}

// Bring the particulate sensor up, naming the stage that failed. Each stage is
// a separate fault: an address that does not answer is wiring, a driver that
// will not open is the Bosch SDK refusing, and a mode that will not set is the
// sensor refusing to start measuring.
static bool bmv080Begin() {
  if (!bmv080.begin(BMV080_ADDR, Wire)) {
    Serial.println("[PM] begin() failed - nothing usable at 0x57");
    return false;
  }
  // These two were called with their results thrown away. init() is what opens
  // the sensor and starts the Bosch driver behind it: if it fails, the sensor
  // sits on the bus acknowledging probes and never produces a single sample,
  // which is exactly what "no sample in 2500 ms, I2C error 0" looks like.
  if (!bmv080.init()) {
    Serial.println("[PM] init() failed - the sensor answers on I2C but its driver "
                   "would not open, so it will never measure");
    return false;
  }
  // Period before mode: the Bosch driver takes its parameters before a
  // measurement starts, not during one.
  if (!bmv080.setDutyCyclingPeriod(BMV080_DUTY_PERIOD_S)) {
    Serial.println("[PM] setDutyCyclingPeriod() failed");
    return false;
  }
  if (!bmv080.setMode(SF_BMV080_MODE_DUTY_CYCLE)) {
    Serial.println("[PM] setMode(duty cycle) failed - driver open but not measuring");
    return false;
  }
  pmCache.valid = false;  // nothing from before a (re)start is trusted
  Serial.printf("[PM] ready - duty cycling, one sample every %u s\n", BMV080_DUTY_PERIOD_S);
  return true;
}

// The BME680/688 turned out to lose its first transfer after other traffic on
// this bus; assume the particulate sensor can too, and do not give up on one
// refusal.
static bool bmv080BeginRetrying(uint8_t attempts) {
  for (uint8_t i = 1; i <= attempts; i++) {
    if (bmv080Begin()) return true;
    if (i < attempts) {
      Serial.printf("[PM] attempt %u of %u failed - retrying\n", i, attempts);
      delay(500);
    }
  }
  return false;
}

static void bmv080Recover() {
  if (bmv080Ready) return;
  if ((uint32_t)(millis() - lastBmv080RetryMs) < BMV080_RETRY_INTERVAL_MS) return;
  lastBmv080RetryMs = millis();
  Serial.println("[PM] retrying the particulate sensor");
  bmv080Ready = bmv080Begin();
  if (bmv080Ready) pmFaultLogged = false;  // complain again if it drops out later
}

// Fill r from the sensors. The two sensors are read independently, so losing
// one does not cost the other: PM2.5 is the primary wildfire signal and must
// keep flowing even with a dead BME. Returns false only when neither sensor
// produced anything, so there is nothing worth recording.
static bool readSensors(Reading& r) {
  memset(&r, 0, sizeof(r));

  // Another library sharing this bus (the Notecard) can leave the clock where
  // it wants it; put it back before touching the sensors.
  Wire.setClock(I2C_CLOCK_HZ);

  // Spend a throwaway transfer first. The transfer that follows Notecard
  // traffic is the one that gets lost, so let it be this one rather than a
  // measurement: it is why the retry below almost always succeeds.
  if (bmeReady) i2cProbe(bmeAddr);

  if (!bmeReady) {
    bmeRecover("never initialised");
  } else if (bmeReadWithRetry()) {
    bmeFailures = 0;
    r.envValid = true;
    r.temperatureC = bme.temperature;
    r.humidityPct = bme.humidity;
    r.pressureHpa = bme.pressure / 100.0f;
    r.altitudeM = pressureToAltitudeM(r.pressureHpa);
    // The library zeroes gas_resistance when the heater did not reach a stable
    // temperature, so a zero here means "no gas reading", not "no resistance".
    r.gasKOhm = bme.gas_resistance / 1000.0f;
    r.gasValid = bmeHeaterOn && r.gasKOhm > 0.0f;
    if (r.gasValid && (r.gasKOhm < 1.0f || r.gasKOhm > 500.0f)) {
      Serial.printf("[SENS] WARNING: gas resistance %.1f kΩ out of expected range\n", r.gasKOhm);
    }
  } else {
    if (bmeFailures < 255) bmeFailures++;
    uint8_t err = i2cProbe(bmeAddr);
    Serial.printf("[SENS] BME680/688 read failed (%u in a row) - 0x%02X %s (I2C error %u)\n",
                  bmeFailures, bmeAddr, err == 0 ? "still answers" : "silent", err);
    // Init keeps succeeding while readings keep failing, and the difference
    // between the two is the heater. Give the heater up rather than the sensor.
    if (bmeHeaterOn && bmeFailures >= BME_HEATER_OFF_AFTER_FAILURES) {
      Serial.println("[BME] reads keep failing with the gas heater on - disabling the heater");
      Serial.println("[BME] if readings now succeed, the heater current is browning the sensor "
                     "out: shorten its lead, decouple its 3V3, or give it its own supply");
      bmeHeaterOn = false;
      lastHeaterRetryMs = millis();
      lastBmeRetryMs = millis() - BME_RETRY_INTERVAL_MS;  // re-init straight away
    }
    bmeRecover("read failed");
  }

  uint32_t pmAgeMs = pmCache.valid ? (uint32_t)(millis() - pmCache.atMs) : 0;
  r.pmValid = bmv080Ready && pmCache.valid && pmAgeMs < PM_STALE_MS;
  if (r.pmValid) {
    r.pm1 = pmCache.pm1;
    r.pm25 = pmCache.pm25;
    r.pm10 = pmCache.pm10;
    r.pmObstructed = pmCache.obstructed;
  } else {
    if (!bmv080Ready) {
      // Once is enough: the minute retry already reports the stage it fails at,
      // so repeating this every sample only buries the readings that do work.
      if (!pmFaultLogged) {
        Serial.println("[SENS] BMV080 unavailable - its driver is not open; see the [PM] lines");
        pmFaultLogged = true;
      }
    } else if (!pmCache.valid) {
      Serial.println("[SENS] BMV080 has not produced its first sample yet");
    } else {
      Serial.printf("[SENS] BMV080 last sample is %lu s old (limit %lu s) - 0x%02X I2C error %u\n",
                    (unsigned long)(pmAgeMs / 1000), (unsigned long)(PM_STALE_MS / 1000),
                    BMV080_ADDR, i2cProbe(BMV080_ADDR));
    }
  }
  return r.envValid || r.pmValid;
}

// ---------------------------------------------------------------------------
// Loop tasks
// ---------------------------------------------------------------------------
static void taskRegister() {
  if (deviceRegistered || (uint32_t)(millis() - lastRegisterMs) < REGISTER_RETRY_INTERVAL_MS) {
    return;
  }
  lastRegisterMs = millis();
  deviceRegistered = sendRegistration();
  Serial.printf("[REG] Registration %s\n", deviceRegistered ? "queued" : "failed, will retry");
}

static void taskSample() {
  if ((uint32_t)(millis() - lastSensorReadMs) < SENSOR_READ_INTERVAL_MS) return;
  lastSensorReadMs = millis();

  Reading r;
  if (!readSensors(r)) return;
  attachBattery(r);
  timeRefresh();
  bool hasTime = nowIso(r.timestampIso, sizeof(r.timestampIso));

  // Detection runs on every sample, even before the clock or buffer are ready.
  biobot::Sample s;
  s.temperatureC = r.temperatureC;
  s.humidityPct = r.humidityPct;
  s.gasResistanceKOhm = r.gasKOhm;
  s.pm25 = r.pm25;
  s.pmValid = r.pmValid;
  s.pmObstructed = r.pmObstructed;
  s.envValid = r.envValid;
  s.gasValid = r.gasValid;
  biobot::Result res = detector.update(s, millis());
  r.severity = res.severity;
  r.anomalyScore = res.score;
  sampleCount++;

  Serial.print("[SENS] ");
  if (r.envValid) {
    Serial.printf("T:%.1f°C H:%.1f%% P:%.1fhPa ", r.temperatureC, r.humidityPct, r.pressureHpa);
    if (r.gasValid) Serial.printf("Gas:%.1fkΩ ", r.gasKOhm);
    else Serial.print("Gas:n/a ");
  } else {
    Serial.print("BME:fault ");
  }
  if (r.pmValid) Serial.printf("PM2.5:%.1f ", r.pm25);
  else Serial.print("PM2.5:n/a ");
  if (r.batteryValid) Serial.printf("Batt:%.2fV ", r.batteryV);
  Serial.printf("%s | %s score:%u%s\n", hasTime ? r.timestampIso : "(no time yet)",
                biobot::Detector::severityName(res.severity), res.score,
                res.baselineReady ? "" : " [baseline learning]");

  if (res.notify) {
    const char* event = res.cleared ? "cleared" : (res.newEvent ? "raised" : "updated");
    Serial.printf("[ALERT] %s: %s (score %u)\n", event,
                  biobot::Detector::severityName(res.severity), res.score);
    if (!sendAlert(r, res, event)) {
      Serial.println("[ALERT] ERROR: failed to queue alert note");
    }
  }

  // Buffer for the periodic batch.
  if (sampleCount <= LOOP_WARMUP_READS) return;
  if (!hasTime) {
    Serial.println("[SENS] No network time yet - reading not buffered");
    return;
  }
  if (bufferCount == BUFFER_CAPACITY) {
    memmove(&buffer[0], &buffer[1], sizeof(Reading) * (BUFFER_CAPACITY - 1));
    bufferCount--;
    droppedReadings++;
    Serial.println("[SENS] WARNING: buffer full, dropped oldest reading");
  }
  buffer[bufferCount++] = r;
}

static void taskSend() {
  if (bufferCount == 0) return;
  uint32_t now = millis();
  bool intervalDue = (uint32_t)(now - lastDataSendMs) >= DATA_SEND_INTERVAL_MS;
  bool batchFull = bufferCount >= BATCH_SIZE;
  if (!intervalDue && !batchFull) return;
  if ((int32_t)(now - nextSendNotBeforeMs) < 0) return;  // backing off after a failure

  Serial.printf("[SEND] Sending %u readings...\n", bufferCount);
  if (sendBatch(buffer, bufferCount)) {
    Serial.printf("[SEND] Queued %u readings\n", bufferCount);
    bufferCount = 0;
    lastDataSendMs = now;
    nextSendNotBeforeMs = now;
  } else {
    Serial.printf("[SEND] Failed - retrying in %lu s\n", (unsigned long)(SEND_RETRY_INTERVAL_MS / 1000));
    nextSendNotBeforeMs = now + SEND_RETRY_INTERVAL_MS;
  }
}

static void taskStatus() {
  if ((uint32_t)(millis() - lastStatusMs) < STATUS_PRINT_INTERVAL_MS) return;
  lastStatusMs = millis();
  uint32_t sinceSend = millis() - lastDataSendMs;
  uint32_t untilSend = sinceSend >= DATA_SEND_INTERVAL_MS ? 0 : DATA_SEND_INTERVAL_MS - sinceSend;
  uint32_t up = millis();
  unsigned sleepPct = up ? (unsigned)((uint64_t)sleptMs * 100 / up) : 0;
  Serial.printf("\n[STATUS] up %lu s | heap %u | registered %s | bme %s | pm %s | batt %s | "
                "buffer %u/%u | dropped %lu | retries %lu | severity %s | next send in %lu s | "
                "asleep %u%% (%lu ticks)\n\n",
                (unsigned long)(up / 1000), (unsigned)ESP.getFreeHeap(),
                deviceRegistered ? "yes" : "no",
                bmeReady ? (bmeHeaterOn ? "ok" : "ok, no gas") : "FAULT",
                bmv080Ready ? (pmCache.valid ? "ok" : "ok, waiting") : "FAULT",
                batteryStatus(),
                bufferCount, BUFFER_CAPACITY,
                (unsigned long)droppedReadings, (unsigned long)bmeRetries,
                biobot::Detector::severityName(detector.severity()),
                (unsigned long)(untilSend / 1000), sleepPct, (unsigned long)sleepTicks);
}

// ---------------------------------------------------------------------------
// Battery, via the charge controller's BLE readout
// ---------------------------------------------------------------------------
static const char* batteryStatus() {
  static char buf[24];
  if (!battery.valid) return VICTRON_BLE_ENABLED ? "none yet" : "off";
  snprintf(buf, sizeof(buf), "%.2fV %s", battery.batteryV, victron::deviceStateName(battery.deviceState));
  return buf;
}

static void taskBattery() {
#if VICTRON_BLE_ENABLED
  if (!victronKeyOk) return;
  uint32_t now = millis();
  if (lastBatteryReadMs == 0) {
    if (now < BATTERY_FIRST_READ_MS) return;  // let the Notecard and sensors come up first
  } else if ((uint32_t)(now - lastBatteryReadMs) < BATTERY_READ_INTERVAL_MS) {
    return;
  }
  lastBatteryReadMs = now;

  victron::Reading r;
  Serial.printf("[BATT] scanning for the Victron for up to %u s...\n", VICTRON_SCAN_S);
  if (!victron::readSolarCharger(victronKey, VICTRON_SCAN_S, r, VICTRON_BLE_VERBOSE)) {
    Serial.println("[BATT] no readout decoded - is Instant Readout on, and the key right?");
    return;
  }
  battery = r;
  Serial.printf("[BATT] %.2f V, %+.1f A, PV %u W, today %.2f kWh, %s, rssi %d\n", r.batteryV,
                r.batteryA, r.pvW, r.yieldTodayKwh, victron::deviceStateName(r.deviceState), r.rssi);
  if (r.batteryV < BATTERY_LOW_V) {
    if (!lowBatteryLogged) {
      Serial.printf("[BATT] LOW: %.2f V is under %.1f V\n", r.batteryV, BATTERY_LOW_V);
      lowBatteryLogged = true;
    }
  } else {
    lowBatteryLogged = false;
  }
#endif
}

// Carry the latest readout on each sample so it lands in the batch with the
// sample's timestamp. Nothing stale rides along.
static void attachBattery(Reading& r) {
  r.batteryValid = battery.valid && (uint32_t)(millis() - battery.atMs) < BATTERY_STALE_MS;
  if (!r.batteryValid) return;
  r.batteryV = battery.batteryV;
  r.chargeA = battery.batteryA;
  r.pvW = battery.pvW;
  r.chargerState = battery.deviceState;
}

// ---------------------------------------------------------------------------
// Sleep
// ---------------------------------------------------------------------------
// Light sleep suspends the USB serial port, so a node on the bench with the
// monitor open would go quiet every five seconds. SLEEP_MODE 1 keeps it awake
// while someone is watching and lets it sleep in the field.
static bool sleepAllowed() {
  uint32_t up = millis();
  if (up < NO_SLEEP_AFTER_BOOT_MS) return false;
  if (!deviceRegistered && up < NO_SLEEP_UNREGISTERED_MS) return false;
#if SLEEP_MODE == 0
  return false;
#elif SLEEP_MODE == 2
  return true;
#else
  return !(bool)Serial;
#endif
}

static void idle() {
  if (!sleepAllowed()) {
    delay(100);
    return;
  }
  Serial.flush();
  uint32_t t0 = millis();
  esp_sleep_enable_timer_wakeup((uint64_t)TICK_SLEEP_MS * 1000ULL);
  esp_err_t rc = esp_light_sleep_start();
  uint32_t slept = millis() - t0;
  if (rc != ESP_OK) {
    if (!sleepRejectedLogged) {
      Serial.printf("[SLEEP] light sleep rejected (err %d) - falling back to delay\n", (int)rc);
      sleepRejectedLogged = true;
    }
    delay(TICK_SLEEP_MS);
    return;
  }
  sleepTicks++;
  sleptMs += slept;
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_SPEED);
  delay(2000);

  Serial.println("\n========================================");
  Serial.println("  BioBot sensor node");
  Serial.println("========================================");
  Serial.printf("Device:        %s (%s)\n", DEVICE_ID, DEVICE_NAME);
  Serial.printf("Read interval: %lu s\n", (unsigned long)(SENSOR_READ_INTERVAL_MS / 1000));
  Serial.printf("Send interval: %lu min\n", (unsigned long)(DATA_SEND_INTERVAL_MS / 60000));
  Serial.printf("Sleep mode:    %d (%s)\n", SLEEP_MODE,
                SLEEP_MODE == 0 ? "never" : SLEEP_MODE == 2 ? "always" : "unless the serial monitor is open");
  Serial.printf("Battery:       %s\n", VICTRON_BLE_ENABLED ? "Victron BLE readout" : "off");
  Serial.printf("Batch size:    %u readings (buffer %u)\n", BATCH_SIZE, BUFFER_CAPACITY);
  Serial.printf("Batches/month: ~%lu\n", (unsigned long)((30UL * 24UL * 3600UL * 1000UL) / DATA_SEND_INTERVAL_MS));
  Serial.printf("Free heap:     %u bytes\n\n", (unsigned)ESP.getFreeHeap());

  Serial.println("[SETUP] Init I2C...");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(100);
  i2cScan();

  Serial.println("[SETUP] Init BME680/688...");
  bmeReady = bmeBegin(true);

  Serial.println("[SETUP] Init BMV080...");
  bmv080Ready = bmv080BeginRetrying(BMV080_BOOT_ATTEMPTS);
  lastBmv080RetryMs = millis();
  if (!bmv080Ready) {
    // Halting used to seem right - PM2.5 is the node's main job - but a halted
    // node reports nothing at all, not even that it is broken. Carry on: the
    // detector raises a sensor fault after ten failed particulate reads, which
    // reaches the operator, and temperature and humidity still get logged.
    Serial.println("[SETUP] WARNING: no particulate sensor - continuing without PM2.5");
  }

  notecardBegin();

#if VICTRON_BLE_ENABLED
  victronKeyOk = victron::parseKey(VICTRON_BLE_KEY, victronKey);
  if (victronKeyOk) {
    Serial.printf("[BATT] key loaded, starts 0x%02x (VictronConnect shows the same two characters first)\n",
                  victronKey[0]);
  }
  if (!victronKeyOk) {
    Serial.println("[BATT] VICTRON_BLE_KEY is not 32 hex characters - battery readout disabled");
  } else if (victronKey[0] == 0 && victronKey[1] == 0 && victronKey[2] == 0 && victronKey[3] == 0) {
    Serial.println("[BATT] VICTRON_BLE_KEY looks like the placeholder - paste the real key from VictronConnect");
  }
#endif

  Serial.println("\n========================================");
  Serial.println("  Setup complete");
  Serial.println("========================================\n");
}

void loop() {
  taskRegister();
  bmeHeaterRetry();
  bmv080Recover();
  taskPm();
  taskSample();
  taskBattery();
  taskSend();
  taskStatus();
  idle();
}
