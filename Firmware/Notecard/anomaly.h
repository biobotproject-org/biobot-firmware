// anomaly.h - On-device anomaly detection for the BioBot sensor node.
//
// This module is deliberately free of Arduino dependencies so it can be
// compiled and unit-tested on a desktop machine (see test/test_anomaly.cpp).
//
// How it works
// ------------
// Every sensor sample is compared against two things:
//   1. Absolute limits that are unusual anywhere (very high PM2.5, very
//      high temperature).
//   2. A slow-moving baseline of "normal for this spot" for each channel.
//      Smoke lowers BME680 gas resistance, raises particulates, and a
//      nearby fire raises temperature and drops humidity. Comparing against
//      the local baseline lets a node in a damp valley and a node on a dry
//      ridge use the same firmware.
//
// Each signal that trips is debounced: it must exceed its threshold for
// `confirmSamples` consecutive samples before it counts, and must fall below
// the threshold times `clearRatio` for the same number of samples before it
// releases. Signals carry weights, and the sum of active weights becomes a
// score that maps to a severity level. Multiple weak signals corroborating
// each other are treated as more serious than one strong signal alone.
//
// Baselines only learn while the node is in the NONE state, so an ongoing
// event cannot be "learned away".
#pragma once

#include <stdint.h>

namespace biobot {

// A single reading from the environmental and particulate sensors.
struct Sample {
  float temperatureC;
  float humidityPct;
  float gasResistanceKOhm;  // BME680 gas resistance; drops when VOCs/smoke present
  float pm25;               // ug/m3
  bool pmValid;             // false when the BMV080 read failed
  bool pmObstructed;        // BMV080 optical path blocked
};

// Bit flags describing which signals are currently confirmed.
enum Signal : uint16_t {
  SIG_NONE            = 0,
  SIG_PM25_HIGH       = 1 << 0,  // PM2.5 above the alert limit
  SIG_PM25_CRITICAL   = 1 << 1,  // PM2.5 above the critical limit
  SIG_TEMP_HIGH       = 1 << 2,  // absolute temperature above limit
  SIG_TEMP_RISE       = 1 << 3,  // temperature far above local baseline
  SIG_HUMIDITY_DROP   = 1 << 4,  // humidity far below local baseline
  SIG_GAS_DROP        = 1 << 5,  // gas resistance well below local baseline
  SIG_PM_OBSTRUCTED   = 1 << 6,  // sensor fault: particulate optics blocked
  SIG_PM_SENSOR_FAIL  = 1 << 7,  // sensor fault: particulate reads failing
};

enum class Severity : uint8_t {
  NONE = 0,   // nothing unusual
  WATCH,      // one weak signal; flagged in batch data, no alert note
  ALERT,      // a strong signal or two weak ones; send an alert note
  CRITICAL,   // corroborated by multiple signals; send an alert note
  FAULT,      // sensor problem; the host should check the module
};

struct Config {
  // Absolute limits
  float pm25AlertUgm3 = 55.0f;      // roughly "unhealthy for sensitive groups"
  float pm25CriticalUgm3 = 150.0f;  // heavy smoke
  float tempHighC = 50.0f;          // hotter than a sunny enclosure should get

  // Deviation from local baseline
  float tempRiseC = 10.0f;          // degrees above baseline
  float humidityDropPct = 25.0f;    // percentage points below baseline
  float gasDropRatio = 0.5f;        // gas resistance at or below this fraction of baseline

  // Baseline learning. alpha is the exponential moving average weight per
  // sample. With 30 s samples, 0.005 gives a time constant of about 100 min,
  // slow enough that a fire does not become the new normal before it trips.
  float baselineAlpha = 0.005f;
  uint16_t warmupSamples = 20;      // samples before baseline-relative signals are trusted

  // Debounce and hysteresis
  uint8_t confirmSamples = 3;       // consecutive samples to confirm a signal
  float clearRatio = 0.8f;          // signal releases once below threshold * clearRatio
  uint8_t faultSamples = 10;        // consecutive bad PM reads before FAULT

  // Notification pacing
  uint32_t repeatIntervalMs = 15UL * 60UL * 1000UL;  // re-notify an ongoing event this often
};

struct Result {
  Severity severity = Severity::NONE;
  uint16_t signals = SIG_NONE;  // bitmask of confirmed Signal values
  uint8_t score = 0;            // weighted sum of active signals
  bool baselineReady = false;   // false during warmup

  // True when the caller should send a notification now: a new event, an
  // escalation, a repeat of an ongoing event, or a return to normal.
  bool notify = false;
  bool newEvent = false;        // this notification opens a new event
  bool cleared = false;         // this notification is an all-clear

  // Baselines at the time of the sample, for context in alert payloads.
  float baselineTempC = 0;
  float baselineHumidityPct = 0;
  float baselineGasKOhm = 0;
  float baselinePm25 = 0;
};

class Detector {
public:
  explicit Detector(const Config& cfg = Config());

  // Feed one sample. nowMs is a monotonic millisecond clock (millis()).
  Result update(const Sample& s, uint32_t nowMs);

  // Forget baselines and event state, e.g. after the node is moved.
  void reset();

  const Config& config() const { return cfg_; }
  Severity severity() const { return severity_; }
  uint32_t samplesSeen() const { return samplesSeen_; }

  // Human-readable helpers for payloads and logs.
  static const char* severityName(Severity s);
  static const char* signalName(Signal s);

  // Iterate set bits in a signal mask. Returns SIG_NONE when exhausted.
  // Usage: for (uint16_t rest = mask; Signal s = nextSignal(rest); ) { ... }
  static Signal nextSignal(uint16_t& remaining);

private:
  // Per-signal debounce state.
  struct Gate {
    bool active = false;
    uint8_t onCount = 0;
    uint8_t offCount = 0;
    // Feed whether the raw condition holds (and whether it has released past
    // hysteresis). Returns the debounced state.
    bool feed(bool tripped, bool released, uint8_t confirm);
    void reset() { active = false; onCount = 0; offCount = 0; }
  };

  void learn(const Sample& s);
  uint8_t scoreFor(uint16_t signals) const;
  Severity severityFor(uint16_t signals, uint8_t score) const;

  Config cfg_;
  uint32_t samplesSeen_ = 0;
  bool baselineInit_ = false;
  float bTemp_ = 0, bHum_ = 0, bGas_ = 0, bPm25_ = 0;

  Gate gPmHigh_, gPmCrit_, gTempHigh_, gTempRise_, gHumDrop_, gGasDrop_;
  uint8_t obstructedCount_ = 0;
  uint8_t failCount_ = 0;

  Severity severity_ = Severity::NONE;
  uint16_t signals_ = SIG_NONE;
  uint32_t lastNotifyMs_ = 0;
  bool everNotified_ = false;
};

}  // namespace biobot
