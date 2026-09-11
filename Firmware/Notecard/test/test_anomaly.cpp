// Host-side unit test for the anomaly detector.
// Build and run from Firmware/Notecard:
//   g++ -std=c++17 -Wall -Wextra -I. anomaly.cpp test/test_anomaly.cpp -o /tmp/test_anomaly && /tmp/test_anomaly
#include "anomaly.h"

#include <cstdio>
#include <cstdlib>

using namespace biobot;

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { \
  std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

static Sample clean() {
  Sample s{};
  s.temperatureC = 18.0f;
  s.humidityPct = 55.0f;
  s.gasResistanceKOhm = 120.0f;
  s.pm25 = 4.0f;
  s.pmValid = true;
  s.pmObstructed = false;
  return s;
}

// Feed n copies of a sample, advancing the clock by stepMs each time.
static Result feed(Detector& d, const Sample& s, int n, uint32_t& now, uint32_t stepMs = 30000) {
  Result r;
  for (int i = 0; i < n; i++) { now += stepMs; r = d.update(s, now); }
  return r;
}

static void test_clean_air_stays_quiet() {
  std::printf("clean air stays quiet\n");
  Detector d;
  uint32_t now = 0;
  Result r = feed(d, clean(), 200, now);
  CHECK(r.severity == Severity::NONE);
  CHECK(r.signals == SIG_NONE);
  CHECK(!r.notify);
  CHECK(r.baselineReady);
  CHECK(r.baselineTempC > 17.9f && r.baselineTempC < 18.1f);
}

static void test_pm25_alert_needs_confirmation() {
  std::printf("pm2.5 alert is debounced and notifies once\n");
  Detector d;
  uint32_t now = 0;
  feed(d, clean(), 50, now);
  Sample smoke = clean();
  smoke.pm25 = 80.0f;
  Result r = feed(d, smoke, 1, now);
  CHECK(r.severity == Severity::NONE);   // 1 of 3
  r = feed(d, smoke, 1, now);
  CHECK(r.severity == Severity::NONE);   // 2 of 3
  r = feed(d, smoke, 1, now);
  CHECK(r.severity == Severity::ALERT);  // confirmed
  CHECK(r.signals & SIG_PM25_HIGH);
  CHECK(r.notify);
  CHECK(r.newEvent);
  CHECK(!r.cleared);
  r = feed(d, smoke, 5, now);
  CHECK(r.severity == Severity::ALERT);
  CHECK(!r.notify);                      // no spam while unchanged
}

static void test_corroboration_escalates() {
  std::printf("multiple signals escalate to critical\n");
  Detector d;
  uint32_t now = 0;
  feed(d, clean(), 50, now);
  Sample fire = clean();
  fire.pm25 = 80.0f;               // +2
  fire.gasResistanceKOhm = 40.0f;  // ratio 0.33 -> +1
  fire.humidityPct = 20.0f;        // drop of 35 -> +1
  Result r = feed(d, fire, 3, now);
  CHECK(r.severity == Severity::CRITICAL);
  CHECK(r.score == 4);
  CHECK(r.newEvent);
  // Escalation of an open event notifies again but is not a new event.
  fire.pm25 = 200.0f;
  r = feed(d, fire, 3, now);
  CHECK(r.notify);
  CHECK(!r.newEvent);
  CHECK(r.signals & SIG_PM25_CRITICAL);
  CHECK(r.signals & SIG_GAS_DROP);
  CHECK(r.signals & SIG_HUMIDITY_DROP);
  CHECK(r.notify);
  // Baseline must not have chased the event.
  CHECK(r.baselineGasKOhm > 110.0f);
  CHECK(r.baselineHumidityPct > 50.0f);
}

static void test_single_weak_signal_is_watch() {
  std::printf("a lone weak signal is a watch, not an alert\n");
  Detector d;
  uint32_t now = 0;
  feed(d, clean(), 50, now);
  Sample s = clean();
  s.gasResistanceKOhm = 50.0f;  // ratio 0.42
  Result r = feed(d, s, 3, now);
  CHECK(r.severity == Severity::WATCH);
  CHECK(!r.notify);
}

static void test_hysteresis_and_all_clear() {
  std::printf("hysteresis holds the alert and the all-clear notifies\n");
  Detector d;
  uint32_t now = 0;
  feed(d, clean(), 50, now);
  Sample smoke = clean();
  smoke.pm25 = 80.0f;
  feed(d, smoke, 3, now);
  // Just under the trip point but above the release point: alert holds.
  Sample lingering = clean();
  lingering.pm25 = 50.0f;  // release point is 55 * 0.8 = 44
  Result r = feed(d, lingering, 10, now);
  CHECK(r.severity == Severity::ALERT);
  CHECK(!r.notify);
  // Now genuinely clean: release after confirmSamples, with a notify.
  r = feed(d, clean(), 2, now);
  CHECK(r.severity == Severity::ALERT);
  r = feed(d, clean(), 1, now);
  CHECK(r.severity == Severity::NONE);
  CHECK(r.notify);
  CHECK(r.cleared);
  r = feed(d, clean(), 5, now);
  CHECK(!r.notify);
}

static void test_periodic_reminder() {
  std::printf("an ongoing event re-notifies after the repeat interval\n");
  Config c;
  c.repeatIntervalMs = 5 * 60 * 1000;
  Detector d(c);
  uint32_t now = 0;
  feed(d, clean(), 50, now);
  Sample smoke = clean();
  smoke.pm25 = 200.0f;
  Result r = feed(d, smoke, 3, now);
  CHECK(r.severity == Severity::CRITICAL);
  CHECK(r.notify);
  int notifies = 0;
  for (int i = 0; i < 20; i++) {          // 10 minutes at 30 s
    r = feed(d, smoke, 1, now);
    if (r.notify) notifies++;
  }
  CHECK(notifies == 2);
}

static void test_temperature_rise_relative_to_baseline() {
  std::printf("temperature rise is measured against the local baseline\n");
  Detector d;
  uint32_t now = 0;
  Sample warm = clean();
  warm.temperatureC = 30.0f;  // a hot day is this node's normal
  feed(d, warm, 50, now);
  Sample hotter = warm;
  hotter.temperatureC = 36.0f;  // +6: below the 10 degree rise threshold
  Result r = feed(d, hotter, 5, now);
  CHECK(r.severity == Severity::NONE);
  hotter.temperatureC = 42.0f;  // +12: rise, but under the 50 C absolute limit
  r = feed(d, hotter, 3, now);
  CHECK(r.severity == Severity::WATCH);
  CHECK(r.signals & SIG_TEMP_RISE);
  hotter.temperatureC = 55.0f;  // absolute limit too
  r = feed(d, hotter, 3, now);
  CHECK(r.severity == Severity::ALERT);
  CHECK(r.signals & SIG_TEMP_HIGH);
}

static void test_sensor_faults() {
  std::printf("persistent obstruction or read failure becomes a fault\n");
  Detector d;
  uint32_t now = 0;
  feed(d, clean(), 50, now);
  Sample blocked = clean();
  blocked.pmObstructed = true;
  blocked.pm25 = 500.0f;  // garbage while obstructed must not become an alert
  Result r = feed(d, blocked, 9, now);
  CHECK(r.severity == Severity::NONE);
  r = feed(d, blocked, 1, now);
  CHECK(r.severity == Severity::FAULT);
  CHECK(r.signals & SIG_PM_OBSTRUCTED);
  CHECK(r.notify);
  // Recovery clears the fault and sends an all-clear.
  r = feed(d, clean(), 1, now);
  CHECK(r.severity == Severity::NONE);
  CHECK(r.cleared);

  Detector d2;
  now = 0;
  feed(d2, clean(), 50, now);
  Sample dead = clean();
  dead.pmValid = false;
  r = feed(d2, dead, 10, now);
  CHECK(r.severity == Severity::FAULT);
  CHECK(r.signals & SIG_PM_SENSOR_FAIL);
  // Counter saturates instead of wrapping.
  r = feed(d2, dead, 300, now);
  CHECK(r.severity == Severity::FAULT);
}

static void test_signal_iteration() {
  std::printf("signal mask iteration and names\n");
  uint16_t rest = SIG_PM25_HIGH | SIG_GAS_DROP;
  int count = 0;
  for (Signal s; (s = Detector::nextSignal(rest)) != SIG_NONE;) {
    count++;
    CHECK(s == SIG_PM25_HIGH || s == SIG_GAS_DROP);
  }
  CHECK(count == 2);
  CHECK(Detector::signalName(SIG_GAS_DROP)[0] == 'g');
  CHECK(Detector::severityName(Severity::CRITICAL)[0] == 'c');
}

int main() {
  test_clean_air_stays_quiet();
  test_pm25_alert_needs_confirmation();
  test_corroboration_escalates();
  test_single_weak_signal_is_watch();
  test_hysteresis_and_all_clear();
  test_periodic_reminder();
  test_temperature_rise_relative_to_baseline();
  test_sensor_faults();
  test_signal_iteration();
  if (failures) { std::printf("%d check(s) failed\n", failures); return 1; }
  std::printf("all checks passed\n");
  return 0;
}
