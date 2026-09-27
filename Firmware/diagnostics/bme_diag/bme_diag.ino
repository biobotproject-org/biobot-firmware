// bme_diag - isolate a BME680/688 that answers on I2C but will not produce a
// reading.
//
// This sketch talks to nothing but the BME680/688. No Notecard, no BMV080, no
// cellular, no anomaly detection. It needs only Wire and the Adafruit BME680
// library, so it also compiles on a machine without the proprietary Bosch
// BMV080 SDK. Upload it in place of the node firmware, read the output, then
// put the node firmware back.
//
// Why it exists
// -------------
// On the node, bme.begin() succeeds every single time and
// bme.performReading() fails every single time. In Adafruit's library a
// reading can only fail in two places, and HOW LONG the call takes says which:
//
//   fails in under ~5 ms      bme68x_set_op_mode() failed - an I2C write
//                             problem, so the bus or the chip's interface
//   fails after roughly the   bme68x_get_data() returned "no new data" - the
//   whole measurement window  chip accepted the command but never finished
//                             measuring, which points at power or the chip
//
// So every read is timed, and four configurations are tried in turn to see
// which, if any, produces a reading. The heater phases separate current draw
// from everything else; the 50 kHz phase gives a long sensor lead more timing
// margin.

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

static const uint32_t SERIAL_SPEED = 115200;
static const int I2C_SDA_PIN = A4;
static const int I2C_SCL_PIN = A5;
static const uint8_t READS_PER_PHASE = 8;
static const uint32_t READ_GAP_MS = 500;

static Adafruit_BME680 bme(&Wire);
static uint8_t bmeAddr = 0;

struct Phase {
  const char* name;
  uint32_t clockHz;
  uint8_t osTemp, osHum, osPress, iir;
  uint16_t heaterC, heaterMs;  // 0/0 disables the heater
};

static const Phase PHASES[] = {
    {"as the node runs it: 8x/2x/4x, IIR 3, heater 320C/150ms", 100000, BME680_OS_8X,
     BME680_OS_2X, BME680_OS_4X, BME680_FILTER_SIZE_3, 320, 150},
    {"heater off, same oversampling", 100000, BME680_OS_8X, BME680_OS_2X, BME680_OS_4X,
     BME680_FILTER_SIZE_3, 0, 0},
    {"minimal: 1x oversampling, no IIR, heater off", 100000, BME680_OS_1X, BME680_OS_1X,
     BME680_OS_1X, BME680_FILTER_SIZE_0, 0, 0},
    {"minimal at 50 kHz - extra margin for a long sensor lead", 50000, BME680_OS_1X,
     BME680_OS_1X, BME680_OS_1X, BME680_FILTER_SIZE_0, 0, 0},
};
static const uint8_t PHASE_COUNT = sizeof(PHASES) / sizeof(PHASES[0]);

static uint8_t i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

static void scan() {
  Serial.print("I2C scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cProbe(a) == 0) {
      Serial.printf(" 0x%02X", a);
      found++;
    }
  }
  if (found == 0) Serial.print(" nothing answered");
  Serial.printf("  (%u device%s)\n", found, found == 1 ? "" : "s");
}

static bool findSensor() {
  const uint8_t candidates[] = {0x76, 0x77};
  for (uint8_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
    uint8_t err = i2cProbe(candidates[i]);
    if (err != 0) {
      Serial.printf("  0x%02X no answer (I2C error %u)\n", candidates[i], err);
      continue;
    }
    if (bme.begin(candidates[i])) {
      bmeAddr = candidates[i];
      Serial.printf("  0x%02X answered and initialised\n", bmeAddr);
      return true;
    }
    Serial.printf("  0x%02X answered but would not initialise\n", candidates[i]);
  }
  return false;
}

static void runPhase(const Phase& p) {
  Serial.printf("\n--- %s\n", p.name);

  // Re-init so each phase starts from the library's default register state.
  Wire.setClock(p.clockHz);
  if (!bme.begin(bmeAddr)) {
    Serial.println("  re-init FAILED - skipping this phase");
    return;
  }
  bme.setTemperatureOversampling(p.osTemp);
  bme.setHumidityOversampling(p.osHum);
  bme.setPressureOversampling(p.osPress);
  bme.setIIRFilterSize(p.iir);
  bme.setGasHeater(p.heaterC, p.heaterMs);

  // What window does the library think this configuration needs? A failure
  // that takes about this long is a measurement that never completed; a
  // failure that returns instantly never got started.
  uint32_t end = bme.beginReading();
  int window = bme.remainingReadingMillis();
  Serial.printf("  expected measurement window: %d ms%s\n", window,
                end == 0 ? "  (beginReading FAILED outright)" : "");
  bme.endReading();

  uint8_t ok = 0;
  for (uint8_t i = 0; i < READS_PER_PHASE; i++) {
    uint32_t t0 = micros();
    bool good = bme.performReading();
    uint32_t us = micros() - t0;
    uint8_t err = i2cProbe(bmeAddr);
    uint8_t err2 = i2cProbe(bmeAddr);

    if (good) {
      ok++;
      Serial.printf("  read %u/%u OK   in %6.1f ms  T %.2f C  H %.2f %%  P %.1f hPa  gas %.1f kOhm\n",
                    i + 1, READS_PER_PHASE, us / 1000.0f, bme.temperature, bme.humidity,
                    bme.pressure / 100.0f, bme.gas_resistance / 1000.0f);
    } else {
      Serial.printf("  read %u/%u FAIL in %6.1f ms  probe after: %u then %u\n", i + 1,
                    READS_PER_PHASE, us / 1000.0f, err, err2);
    }
    delay(READ_GAP_MS);
  }
  Serial.printf("  result: %u of %u reads succeeded\n", ok, READS_PER_PHASE);
}

void setup() {
  Serial.begin(SERIAL_SPEED);
  delay(2000);
  Serial.println("\n========================================");
  Serial.println("  BME680/688 diagnostic - this sensor only");
  Serial.println("========================================");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  delay(100);
  scan();

  Serial.println("Looking for the sensor:");
  if (!findSensor()) {
    while (true) {
      Serial.println("No BME680/688 at 0x76 or 0x77 - nothing to test");
      delay(5000);
    }
  }
}

void loop() {
  for (uint8_t i = 0; i < PHASE_COUNT; i++) runPhase(PHASES[i]);
  Serial.println("\n=== all phases done, repeating in 10 s ===");
  delay(10000);
}
