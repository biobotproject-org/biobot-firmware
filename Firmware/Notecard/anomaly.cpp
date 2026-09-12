// anomaly.cpp - see anomaly.h for the design.
#include "anomaly.h"

namespace biobot {

Detector::Detector(const Config& cfg) : cfg_(cfg) {}

void Detector::reset() {
  samplesSeen_ = 0;
  baselineInit_ = false;
  bTemp_ = bHum_ = bGas_ = bPm25_ = 0;
  gPmHigh_.reset(); gPmCrit_.reset(); gTempHigh_.reset();
  gTempRise_.reset(); gHumDrop_.reset(); gGasDrop_.reset();
  obstructedCount_ = 0;
  failCount_ = 0;
  severity_ = Severity::NONE;
  signals_ = SIG_NONE;
  lastNotifyMs_ = 0;
  everNotified_ = false;
}

bool Detector::Gate::feed(bool tripped, bool released, uint8_t confirm) {
  if (!active) {
    offCount = 0;
    onCount = tripped ? (uint8_t)(onCount + 1) : 0;
    if (onCount >= confirm) {
      active = true;
      onCount = 0;
    }
  } else {
    onCount = 0;
    offCount = released ? (uint8_t)(offCount + 1) : 0;
    if (offCount >= confirm) {
      active = false;
      offCount = 0;
    }
  }
  return active;
}

void Detector::learn(const Sample& s) {
  if (!baselineInit_) {
    bTemp_ = s.temperatureC;
    bHum_ = s.humidityPct;
    bGas_ = s.gasResistanceKOhm;
    bPm25_ = s.pmValid ? s.pm25 : 0;
    baselineInit_ = true;
    return;
  }
  // During warmup learn fast so the baseline settles within a few samples;
  // afterwards learn slowly so an event cannot drag the baseline with it.
  const float a = (samplesSeen_ < cfg_.warmupSamples) ? 0.2f : cfg_.baselineAlpha;
  bTemp_ += a * (s.temperatureC - bTemp_);
  bHum_ += a * (s.humidityPct - bHum_);
  bGas_ += a * (s.gasResistanceKOhm - bGas_);
  if (s.pmValid) bPm25_ += a * (s.pm25 - bPm25_);
}

uint8_t Detector::scoreFor(uint16_t sig) const {
  uint8_t score = 0;
  if (sig & SIG_PM25_HIGH)     score += 2;
  if (sig & SIG_PM25_CRITICAL) score += 2;  // stacks with PM25_HIGH: 4 total
  if (sig & SIG_TEMP_HIGH)     score += 2;
  if (sig & SIG_TEMP_RISE)     score += 1;
  if (sig & SIG_HUMIDITY_DROP) score += 1;
  if (sig & SIG_GAS_DROP)      score += 1;
  return score;
}

Severity Detector::severityFor(uint16_t sig, uint8_t score) const {
  if (sig & (SIG_PM_OBSTRUCTED | SIG_PM_SENSOR_FAIL)) {
    // A faulty particulate sensor takes precedence unless the environmental
    // channels alone are already shouting.
    if (score < 4) return Severity::FAULT;
  }
  if (score >= 4) return Severity::CRITICAL;
  if (score >= 2) return Severity::ALERT;
  if (score >= 1) return Severity::WATCH;
  return Severity::NONE;
}

Result Detector::update(const Sample& s, uint32_t nowMs) {
  Result r;
  const bool ready = baselineInit_ && samplesSeen_ >= cfg_.warmupSamples;
  const uint8_t confirm = cfg_.confirmSamples ? cfg_.confirmSamples : 1;
  const float cr = cfg_.clearRatio;

  uint16_t sig = SIG_NONE;

  // --- Sensor health -------------------------------------------------------
  if (s.pmValid) {
    failCount_ = 0;
    obstructedCount_ = s.pmObstructed ? (uint8_t)(obstructedCount_ + 1) : 0;
  } else {
    failCount_ = (uint8_t)(failCount_ + 1);
    obstructedCount_ = 0;
  }
  if (obstructedCount_ >= cfg_.faultSamples) sig |= SIG_PM_OBSTRUCTED;
  if (failCount_ >= cfg_.faultSamples) sig |= SIG_PM_SENSOR_FAIL;
  // Saturate counters so they do not wrap back to zero.
  if (obstructedCount_ > cfg_.faultSamples) obstructedCount_ = cfg_.faultSamples;
  if (failCount_ > cfg_.faultSamples) failCount_ = cfg_.faultSamples;

  // --- Absolute limits (valid from the very first sample) ------------------
  const bool pmUsable = s.pmValid && !s.pmObstructed;
  if (gPmHigh_.feed(pmUsable && s.pm25 >= cfg_.pm25AlertUgm3,
                    !pmUsable || s.pm25 < cfg_.pm25AlertUgm3 * cr, confirm))
    sig |= SIG_PM25_HIGH;
  if (gPmCrit_.feed(pmUsable && s.pm25 >= cfg_.pm25CriticalUgm3,
                    !pmUsable || s.pm25 < cfg_.pm25CriticalUgm3 * cr, confirm))
    sig |= SIG_PM25_CRITICAL;
  if (gTempHigh_.feed(s.temperatureC >= cfg_.tempHighC,
                      s.temperatureC < cfg_.tempHighC - 5.0f, confirm))
    sig |= SIG_TEMP_HIGH;

  // --- Baseline-relative signals (only once the baseline is trusted) -------
  if (ready) {
    const float dT = s.temperatureC - bTemp_;
    if (gTempRise_.feed(dT >= cfg_.tempRiseC, dT < cfg_.tempRiseC * cr, confirm))
      sig |= SIG_TEMP_RISE;

    const float dH = bHum_ - s.humidityPct;
    if (gHumDrop_.feed(dH >= cfg_.humidityDropPct, dH < cfg_.humidityDropPct * cr, confirm))
      sig |= SIG_HUMIDITY_DROP;

    // Guard against a zero baseline (sensor not warmed up) producing nonsense.
    const bool gasOk = bGas_ > 0.5f && s.gasResistanceKOhm > 0.0f;
    const float gasRatio = gasOk ? s.gasResistanceKOhm / bGas_ : 1.0f;
    // Release once the ratio recovers above the drop ratio plus a margin.
    const float gasRelease = cfg_.gasDropRatio + (1.0f - cfg_.gasDropRatio) * (1.0f - cr);
    if (gGasDrop_.feed(gasOk && gasRatio <= cfg_.gasDropRatio, gasRatio > gasRelease, confirm))
      sig |= SIG_GAS_DROP;
  } else {
    gTempRise_.reset(); gHumDrop_.reset(); gGasDrop_.reset();
  }

  // --- Score and severity ----------------------------------------------------
  const uint8_t score = scoreFor(sig);
  const Severity sev = severityFor(sig, score);

  // --- Decide whether to notify ----------------------------------------------
  const bool wasActive = severity_ >= Severity::ALERT;
  const bool isActive = sev >= Severity::ALERT;
  bool notify = false;
  bool newEvent = false;
  bool cleared = false;

  if (isActive && !wasActive) {
    notify = true;                                   // new event
    newEvent = true;
  } else if (isActive && sev != severity_) {
    notify = true;                                   // escalation or de-escalation
  } else if (isActive && sig != signals_) {
    notify = true;                                   // same level, different evidence
  } else if (isActive && everNotified_ &&
             (uint32_t)(nowMs - lastNotifyMs_) >= cfg_.repeatIntervalMs) {
    notify = true;                                   // periodic reminder
  } else if (!isActive && wasActive) {
    notify = true;                                   // back to normal
    cleared = true;
  }

  if (notify) {
    lastNotifyMs_ = nowMs;
    everNotified_ = true;
  }

  // --- Learn only while nothing is going on ----------------------------------
  if (sev == Severity::NONE || !ready) learn(s);
  samplesSeen_++;

  severity_ = sev;
  signals_ = sig;

  r.severity = sev;
  r.signals = sig;
  r.score = score;
  r.baselineReady = ready;
  r.notify = notify;
  r.newEvent = newEvent;
  r.cleared = cleared;
  r.baselineTempC = bTemp_;
  r.baselineHumidityPct = bHum_;
  r.baselineGasKOhm = bGas_;
  r.baselinePm25 = bPm25_;
  return r;
}

const char* Detector::severityName(Severity s) {
  switch (s) {
    case Severity::NONE: return "none";
    case Severity::WATCH: return "watch";
    case Severity::ALERT: return "alert";
    case Severity::CRITICAL: return "critical";
    case Severity::FAULT: return "fault";
  }
  return "unknown";
}

const char* Detector::signalName(Signal s) {
  switch (s) {
    case SIG_PM25_HIGH: return "pm25_high";
    case SIG_PM25_CRITICAL: return "pm25_critical";
    case SIG_TEMP_HIGH: return "temperature_high";
    case SIG_TEMP_RISE: return "temperature_rise";
    case SIG_HUMIDITY_DROP: return "humidity_drop";
    case SIG_GAS_DROP: return "gas_resistance_drop";
    case SIG_PM_OBSTRUCTED: return "pm_sensor_obstructed";
    case SIG_PM_SENSOR_FAIL: return "pm_sensor_failed";
    case SIG_NONE: break;
  }
  return "unknown";
}

Signal Detector::nextSignal(uint16_t& remaining) {
  if (remaining == 0) return SIG_NONE;
  uint16_t bit = remaining & (uint16_t)(-(int16_t)remaining);  // lowest set bit
  remaining &= (uint16_t)~bit;
  return (Signal)bit;
}

}  // namespace biobot
