// i2c_watch - find an intermittent connector.
//
// A one-shot I2C scan answers "is it there right now", which is the wrong
// question for a damaged connector: a pin that has been rocked loose answers
// most of the time and drops out under the slightest movement. So this probes
// every device ten times a second and keeps score, which turns "it worked when
// I checked" into a number.
//
// How to use it: upload, open the Serial Monitor, and while it runs press,
// wiggle and gently tug each sensor lead in turn. The address whose failure
// count climbs is the one with the bad connection. A good connector stays at
// zero failures no matter what you do to it short of unplugging it.
//
// Needs only Wire - no sensor libraries, no Bosch SDK, no Notecard.

#include <Arduino.h>
#include <Wire.h>

static const uint32_t SERIAL_SPEED = 115200;
static const int I2C_SDA_PIN = A4;
static const int I2C_SCL_PIN = A5;
static const uint32_t I2C_CLOCK_HZ = 100000;

static const uint32_t PROBE_INTERVAL_MS = 100;
static const uint32_t REPORT_INTERVAL_MS = 2000;

struct Watched {
  uint8_t addr;
  const char* name;
  uint32_t probes;
  uint32_t failures;
  uint32_t windowFailures;  // failures since the last report
  uint8_t lastError;
};

static Watched devices[] = {
    {0x17, "Notecard", 0, 0, 0, 0},
    {0x57, "BMV080  ", 0, 0, 0, 0},
    {0x76, "BME 0x76", 0, 0, 0, 0},
    {0x77, "BME 0x77", 0, 0, 0, 0},
};
static const uint8_t DEVICE_COUNT = sizeof(devices) / sizeof(devices[0]);

static uint32_t lastProbeMs = 0;
static uint32_t lastReportMs = 0;
static uint32_t startMs = 0;

static uint8_t probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

static void report() {
  Serial.printf("\n[%lus]\n", (unsigned long)((millis() - startMs) / 1000));
  for (uint8_t i = 0; i < DEVICE_COUNT; i++) {
    Watched& d = devices[i];
    if (d.probes == 0) continue;
    float lossPct = 100.0f * (float)d.failures / (float)d.probes;
    const char* verdict;
    if (d.failures == 0) {
      verdict = "solid";
    } else if (d.failures == d.probes) {
      verdict = "absent - never answered";
    } else {
      verdict = "INTERMITTENT - suspect this connector";
    }
    Serial.printf("  0x%02X %s  %6lu probes  %5lu failures (%5.1f%%)  +%lu just now  "
                  "last error %u  %s\n",
                  d.addr, d.name, (unsigned long)d.probes, (unsigned long)d.failures, lossPct,
                  (unsigned long)d.windowFailures, d.lastError, verdict);
    d.windowFailures = 0;
  }
}

void setup() {
  Serial.begin(SERIAL_SPEED);
  delay(2000);
  Serial.println("\n========================================");
  Serial.println("  I2C connector watch");
  Serial.println("========================================");
  Serial.println("Probing 10x a second. Wiggle one lead at a time and watch");
  Serial.println("which address starts failing. Keep 12 V connected.\n");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(100);
  startMs = millis();
}

void loop() {
  if ((uint32_t)(millis() - lastProbeMs) >= PROBE_INTERVAL_MS) {
    lastProbeMs = millis();
    for (uint8_t i = 0; i < DEVICE_COUNT; i++) {
      Watched& d = devices[i];
      uint8_t err = probe(d.addr);
      d.probes++;
      if (err != 0) {
        d.failures++;
        d.windowFailures++;
        d.lastError = err;
      }
    }
  }
  if ((uint32_t)(millis() - lastReportMs) >= REPORT_INTERVAL_MS) {
    lastReportMs = millis();
    report();
  }
}
