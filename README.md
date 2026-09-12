# BioBot firmware

Firmware for the BioBot wildfire-watch sensor node: an open, low-cost module
that towns and volunteers place at the forest edge to catch early signs of
smoke and fire. See the mission document for the bigger picture.

## What the node does

Every 30 seconds the node reads:

| Sensor | Bus | Channels |
| --- | --- | --- |
| Bosch BME680 / BME688 | I2C 0x77 (or 0x76) | temperature, humidity, pressure, gas resistance |
| Bosch BMV080 | I2C 0x57 | PM1, PM2.5, PM10, optical obstruction flag |

Each sample goes through the on-device anomaly detector, then into a buffer.
A Blues Notecard cellular modem carries three kinds of notes to Notehub, which
routes them to the BioBot API:

| Note file | When | Contents |
| --- | --- | --- |
| `device.qo` | once at boot | node registration |
| `data.qo` | every 10 minutes | a batch of 20 readings, each tagged with its anomaly severity and score |
| `alert.qo` | immediately | an anomaly raised, updated, repeated, or cleared |

## Anomaly detection

The detector (`Firmware/Notecard/anomaly.h`) runs on the node so an alert can
leave the module the moment it is confirmed, rather than waiting for the next
batch. It looks at two kinds of evidence:

- **Absolute limits.** PM2.5 above 35 µg/m³ (watch), 55 µg/m³ (alert) or 150 µg/m³ (critical),
  temperature above 50 °C.
- **Deviation from this node's own baseline.** A slow exponential moving
  average learns what "normal" looks like at this exact spot. Smoke and
  nearby fire show up as gas resistance falling to half its baseline, humidity
  dropping 25 points, or temperature climbing 10 °C above baseline.

Each signal must hold for 3 consecutive samples before it counts, and must
fall well below its threshold for 3 samples before it releases, so a single
gust does not trip or clear an alert. Signals carry weights and the total
maps to a severity:

| Severity | Meaning | Sent as alert? |
| --- | --- | --- |
| `none` | nothing unusual | no |
| `watch` | one weak signal, e.g. PM2.5 between 35 and 55 or a gas resistance drop alone | no, flagged in batch data |
| `alert` | one strong signal or two weak ones | yes |
| `critical` | several signals corroborating each other, or very heavy smoke | yes |
| `fault` | particulate sensor obstructed or failing for 10 samples | yes |

The baseline stops learning while an event is open, so a fire cannot become
the new normal. An open event re-notifies every 15 minutes, and an all-clear
is sent when it ends. All thresholds live in `biobot::Config` and can be
tuned per deployment.

### Alert payload

```json
{
  "request_type": "anomaly",
  "deviceId": "biobot-001",
  "event": "raised",
  "severity": "critical",
  "score": 5,
  "signals": ["pm25_elevated", "pm25_high", "humidity_drop", "gas_resistance_drop"],
  "timestamp": "2026-08-14T21:07:30Z",
  "readings": { "temperature": 31.2, "humidity": 18.5, "pressure": 912.4,
                "gasResistance": 38.1, "pm1": 52.0, "pm25": 88.3, "pm10": 101.0,
                "obstructed": false },
  "baseline": { "ready": true, "temperature": 24.8, "humidity": 51.0,
                "gasResistance": 121.6, "pm25": 5.2 }
}
```

`event` is one of `raised`, `updated` (severity or evidence changed, or the
periodic reminder), or `cleared`.

## Building

1. Install the Arduino IDE or `arduino-cli` with an ESP32 board package.
2. Install these libraries: Adafruit BME680, Adafruit Unified Sensor,
   SparkFun BMV080, Blues Wireless Notecard.
3. Copy `Firmware/Notecard/config.example.h` to `Firmware/Notecard/config.h`
   and fill in your Notehub product UID and device identity. `config.h` is
   git-ignored. The node holds no API credentials; the Notehub route adds
   the server's token when it forwards notes (see the biobot-cloud README).
4. Open `Firmware/Notecard/Notecard.ino` and upload.

Wiring defaults are at the top of the sketch: I2C on A4/A5 at 100 kHz.

## Continuous integration

Every push and pull request compiles the sketch for the Arduino Nano ESP32
with `arduino-cli` and runs the detector's host tests. See
`.github/workflows/build.yml`.

## Testing the detector on your computer

The anomaly logic has no Arduino dependencies, so it can be tested on a
laptop with any C++ compiler:

```sh
cd Firmware/Notecard
g++ -std=c++17 -Wall -Wextra -I. anomaly.cpp test/test_anomaly.cpp -o test/test_anomaly
./test/test_anomaly
```

The tests cover clean air, debounced smoke alerts, multi-signal escalation,
hysteresis, the all-clear, periodic reminders, baseline-relative temperature
rise, and sensor faults.

## Repository layout

```
Firmware/Notecard/Notecard.ino      main sketch: sensors, buffering, Notecard I/O
Firmware/Notecard/anomaly.h / .cpp  portable anomaly detector
Firmware/Notecard/config.example.h  template for per-node secrets and identity
Firmware/Notecard/test/             host-side unit tests
archive/new.grc/                    earlier UART-based prototype, kept for reference
```

## Power note

The default Notehub mode is `continuous`, which keeps the modem online and is
the hungriest setting. For a solar node, set `NOTEHUB_MODE` to `"periodic"`
in `config.h`. Alert notes are added with `sync:true`, so they still go out
immediately in periodic mode; only routine batches wait for the next sync.
