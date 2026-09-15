// BioBot sensor node firmware
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

#include "config.h"
#include "anomaly.h"

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

static const uint32_t SENSOR_READ_INTERVAL_MS = 30UL * 1000UL;        // 30 s
static const uint32_t DATA_SEND_INTERVAL_MS = 10UL * 60UL * 1000UL;   // 10 min
static const uint32_t SEND_RETRY_INTERVAL_MS = 60UL * 1000UL;         // after a failed batch
static const uint32_t REGISTER_RETRY_INTERVAL_MS = 5UL * 1000UL;
static const uint32_t STATUS_PRINT_INTERVAL_MS = 60UL * 1000UL;

// Readings per batch at the intervals above, plus headroom so a failed send
// can be retried without losing samples.
static const uint8_t BATCH_SIZE = DATA_SEND_INTERVAL_MS / SENSOR_READ_INTERVAL_MS;  // 20
static const uint8_t BUFFER_CAPACITY = BATCH_SIZE + BATCH_SIZE / 2;                // 30

static const uint8_t SETUP_WARMUP_READS = 5;   // gas heater warmup in setup()
static const uint8_t LOOP_WARMUP_READS = 2;    // early loop samples not buffered

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
  bool pmValid;
  bool pmObstructed;
  char timestampIso[24];  // "YYYY-MM-DDTHH:MM:SSZ"
  biobot::Severity severity;
  uint8_t anomalyScore;
};

static Adafruit_BME680 bme(&Wire);
static SparkFunBMV080 bmv080;
static Notecard notecard;
static biobot::Detector detector;

static Reading buffer[BUFFER_CAPACITY];
static uint8_t bufferCount = 0;

static bool bmeReady = false;
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

// Ask the Notecard for wall-clock time as ISO 8601 UTC. Returns false until
// the Notecard has synced time with the network.
static bool notecardTimeIso(char* out, size_t outSize) {
  J* req = notecard.newRequest("card.time");
  if (!req) return false;
  J* rsp = notecard.requestAndResponse(req);
  if (!rsp) return false;
  double unixTime = JGetNumber(rsp, "time");
  notecard.deleteResponse(rsp);
  if (unixTime <= 0) return false;

  time_t raw = (time_t)unixTime;
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

    J* arr = JAddArrayToObject(item, "readings");
    addReading(arr, "temperature", "°C", r.temperatureC, r.timestampIso);
    addReading(arr, "humidity", "%", r.humidityPct, r.timestampIso);
    addReading(arr, "pressure", "hPa", r.pressureHpa, r.timestampIso);
    addReading(arr, "gasResistance", "kΩ", r.gasKOhm, r.timestampIso);
    addReading(arr, "altitude", "m", r.altitudeM, r.timestampIso);
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
static bool bmeBegin() {
  uint8_t found = 0;
  if (bme.begin(BME_ADDR_PRIMARY)) {
    found = BME_ADDR_PRIMARY;
  } else if (bme.begin(BME_ADDR_SECONDARY)) {
    found = BME_ADDR_SECONDARY;
  }
  if (!found) {
    Serial.println("[SETUP] BME680/688 not found - check wiring and address");
    return false;
  }
  Serial.printf("[SETUP] BME680/688 found at 0x%02X\n", found);

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320 °C for 150 ms

  Serial.println("[SETUP] Warming up gas heater...");
  for (uint8_t i = 0; i < SETUP_WARMUP_READS; i++) {
    if (bme.performReading()) {
      Serial.printf("[SETUP] Warmup %u/%u - gas %.1f kΩ\n", i + 1, SETUP_WARMUP_READS,
                    bme.gas_resistance / 1000.0f);
    } else {
      Serial.printf("[SETUP] Warmup %u/%u - read failed\n", i + 1, SETUP_WARMUP_READS);
    }
    delay(2000);
  }
  return true;
}

static bool bmv080Begin() {
  if (!bmv080.begin(BMV080_ADDR, Wire)) {
    Serial.println("[SETUP] BMV080 not found - check wiring");
    return false;
  }
  bmv080.init();
  bmv080.setMode(SF_BMV080_MODE_CONTINUOUS);
  Serial.println("[SETUP] BMV080 OK");
  return true;
}

// Fill r from the sensors. Returns false if the BME680 could not be read.
static bool readSensors(Reading& r) {
  memset(&r, 0, sizeof(r));

  if (!bmeReady || !bme.performReading()) {
    Serial.println("[SENS] BME680/688 read failed");
    return false;
  }
  r.temperatureC = bme.temperature;
  r.humidityPct = bme.humidity;
  r.pressureHpa = bme.pressure / 100.0f;
  r.gasKOhm = bme.gas_resistance / 1000.0f;
  r.altitudeM = pressureToAltitudeM(r.pressureHpa);
  if (r.gasKOhm < 1.0f || r.gasKOhm > 500.0f) {
    Serial.printf("[SENS] WARNING: gas resistance %.1f kΩ out of expected range\n", r.gasKOhm);
  }

  r.pmValid = bmv080.readSensor();
  if (r.pmValid) {
    r.pm1 = bmv080.PM1();
    r.pm25 = bmv080.PM25();
    r.pm10 = bmv080.PM10();
    r.pmObstructed = bmv080.isObstructed();
  } else {
    Serial.println("[SENS] BMV080 read failed");
  }
  return true;
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
  bool hasTime = notecardTimeIso(r.timestampIso, sizeof(r.timestampIso));

  // Detection runs on every sample, even before the clock or buffer are ready.
  biobot::Sample s;
  s.temperatureC = r.temperatureC;
  s.humidityPct = r.humidityPct;
  s.gasResistanceKOhm = r.gasKOhm;
  s.pm25 = r.pm25;
  s.pmValid = r.pmValid;
  s.pmObstructed = r.pmObstructed;
  biobot::Result res = detector.update(s, millis());
  r.severity = res.severity;
  r.anomalyScore = res.score;
  sampleCount++;

  Serial.printf("[SENS] T:%.1f°C H:%.1f%% P:%.1fhPa Gas:%.1fkΩ PM2.5:%.1f %s | %s score:%u%s\n",
                r.temperatureC, r.humidityPct, r.pressureHpa, r.gasKOhm, r.pm25,
                hasTime ? r.timestampIso : "(no time yet)",
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
  Serial.printf("\n[STATUS] up %lu s | heap %u | registered %s | buffer %u/%u | dropped %lu | "
                "severity %s | next send in %lu s\n\n",
                (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap(),
                deviceRegistered ? "yes" : "no", bufferCount, BUFFER_CAPACITY,
                (unsigned long)droppedReadings,
                biobot::Detector::severityName(detector.severity()),
                (unsigned long)(untilSend / 1000));
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
  Serial.printf("Batch size:    %u readings (buffer %u)\n", BATCH_SIZE, BUFFER_CAPACITY);
  Serial.printf("Batches/month: ~%lu\n", (unsigned long)((30UL * 24UL * 3600UL * 1000UL) / DATA_SEND_INTERVAL_MS));
  Serial.printf("Free heap:     %u bytes\n\n", (unsigned)ESP.getFreeHeap());

  Serial.println("[SETUP] Init I2C...");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(100);

  Serial.println("[SETUP] Init BME680/688...");
  bmeReady = bmeBegin();

  Serial.println("[SETUP] Init BMV080...");
  if (!bmv080Begin()) {
    // Without the particulate sensor the node cannot do its main job.
    while (true) {
      Serial.println("[SETUP] HALTED - no BMV080");
      delay(5000);
    }
  }

  notecardBegin();

  Serial.println("\n========================================");
  Serial.println("  Setup complete");
  Serial.println("========================================\n");
}

void loop() {
  taskRegister();
  taskSample();
  taskSend();
  taskStatus();
  delay(100);
}
