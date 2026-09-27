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

1. Install the Arduino IDE and, in Boards Manager, **Arduino ESP32 Boards**.
   The target is **Arduino Nano ESP32** (`arduino:esp32:nano_nora`).
2. In Library Manager install, by exact name: **Adafruit BME680 Library**,
   **Adafruit Unified Sensor**, **SparkFun BMV080 Arduino Library** (accept
   its SparkFun Toolkit dependency), and **Blues Wireless Notecard**.
3. Install the Bosch BMV080 SDK into the SparkFun library. See the next
   section; the build fails with `bmv080.h not found` until this is done.
4. Copy `Firmware/Notecard/config.example.h` to `Firmware/Notecard/config.h`
   and fill in your Notehub product UID and device identity. `config.h` is
   git-ignored.
5. Open `Firmware/Notecard/Notecard.ino` and upload. Serial Monitor at
   115200 baud shows every step of startup.

Wiring defaults are at the top of the sketch: I2C on A4/A5 at 100 kHz.

## Bosch BMV080 SDK

The SparkFun library is a thin wrapper around a precompiled library from
Bosch, which Bosch licenses for use but not redistribution. So it is not
in this repository, not in the SparkFun library, and not on the Arduino
package servers. Every machine that builds the firmware needs it once:

1. Download the SDK from Bosch's BMV080 page under Documents ("Download
   the SDK for BMV080"). A Bosch account is required.
2. Unzip it. Find your SparkFun library folder:
   `Documents/Arduino/libraries/SparkFun_BMV080_Arduino_Library` on
   Windows and macOS, `~/Arduino/libraries/...` on Linux.
3. Copy `api/inc/bmv080.h` and `api/inc/bmv080_defs.h` into that
   library's `src/sfTk/` folder.
4. Copy the two `.a` files from
   `api/api/lib/xtensa_esp32s3/xtensa_esp32s3_elf_gcc/release/` into the
   library's `src/esp32s3/` folder. The Nano ESP32 is an ESP32-S3. The
   SparkFun README lists the paths for every other architecture.

**Continuous integration.** The compile job in `.github/workflows/build.yml`
needs the same files. Put the unzipped SDK in a private GitHub repository
that only the team can read, then add two repository secrets to
biobot-firmware: `BMV080_SDK_REPO` (for example
`biobotproject-org/bmv080-sdk`) and `BMV080_SDK_TOKEN` (a fine-grained
personal access token with read access to that private repository).
Until both exist the job skips with a notice rather than failing.

## Bench power

USB alone powers the Nano but not the Notecard: its modem supply comes
from the board's 5 V rail, which the 12 V input feeds. On the bench,
connect a 12 V supply of at least 2 A to the screw terminal as well as
the USB cable. Check polarity before applying power; rev A of the board
has no fuse. Attach the cellular antenna to the Notecard's main u.FL
connector before the first boot.

## Reading the boot log

The sketch scans the I2C bus before it touches a sensor, so the first
useful line of every boot lists what answered:

```
[SETUP] I2C scan: 0x17 0x57 0x76  (3 devices)
```

Expect `0x17` (Notecard), `0x57` (BMV080) and one of `0x76` / `0x77`
(BME680/688 — rev A leaves SDO low, so 0x76). When a read fails, the log
carries the raw TwoWire error code:

| Error | Meaning |
| --- | --- |
| 0 | the chip acknowledged: the bus is fine and the measurement itself failed |
| 2 | address not acknowledged: that chip is unpowered, in reset, or gone |
| 5 | timeout: the bus is being held, so no address can answer |

A failed read triggers a recovery pass, which probes all three devices
first. If every probe is non-zero the bus is wedged and the controller is
torn down and restarted. If only the BME is non-zero while the Notecard
answers, the bus is healthy and the sensor itself dropped out.

### When init succeeds but every reading fails

This is the interesting case, and it points at one thing. `begin()` only
soft-resets the chip and reads its calibration, which costs almost no
current. A reading additionally fires the gas heater — 320 °C for 150 ms —
and that is the only part of the cycle that draws real current. So a
sensor that initialises perfectly and then fails every single reading,
NACKing for a moment afterwards and answering again straight after, is a
sensor whose supply cannot hold up under the heater. Its own brown-out
detector puts it briefly into reset; that is the NACK.

The node handles this itself. If no warmup read succeeds at boot, or three
readings in a row fail, it disables the heater and says so:

```
[BME] reads keep failing with the gas heater on - disabling the heater
[BME] if readings now succeed, the heater current is browning the sensor out:
      shorten its lead, decouple its 3V3, or give it its own supply
```

Readings then resume without gas resistance, and the status line reads
`bme ok, no gas`. The heater is re-tried once an hour in case the cause was
temporary. If readings work with the heater off, the diagnosis is confirmed
and the fix is electrical, not firmware: a shorter lead to the sensor, a
10 µF bulk plus 100 nF ceramic across its 3V3 and GND at the sensor end,
or a supply that is not shared with the Notecard's modem.

### The BMV080 and "read failed"

`readSensor()` returns false both for a real error and for "no new sample
yet", and in continuous mode the sensor produces a sample about once a
second. Asking once every 30 s therefore misses more often than it hits, so
the node polls for up to 2.5 s and only then reports a failure, with how
long it waited and the sensor's I2C error:

```
[SENS] BMV080 gave no sample in 2500 ms - 0x57 I2C error 0
```

Error 0 there means the sensor is on the bus and simply produced nothing.

### Degraded operation

The two sensors are read independently, so one failing does not silence
the other. PM2.5 is the primary wildfire signal and keeps flowing with a
dead BME; temperature, humidity and pressure keep flowing with the heater
off. Channels that did not read are **omitted** from the note rather than
sent as zero, and the detector treats an absent channel as absent — it
raises no signal and does not move its baseline. A humidity reading of
zero from a failed sensor would otherwise look like a 55-point humidity
drop and trip a false alert; there is a host test covering exactly that.

Network time is fetched from the Notecard once and carried forward on
`millis()`, re-anchored hourly, rather than fetched for every sample. That
keeps 20 of every 21 Notecard transactions off the bus the sensors share.

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
rise, sensor faults, and degraded operation with a failed BME or a disabled
gas heater.

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
