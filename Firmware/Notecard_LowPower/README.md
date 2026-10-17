# Notecard_LowPower

A copy of `Firmware/Notecard` with the power work applied, kept as a separate
sketch so the proven firmware stays untouched. Bench-verified on 2026-10-17:
sampling, batching and the Victron BLE battery readout all run; light sleep
(`SLEEP_MODE 2`) has not yet been confirmed on hardware.

## What changed

| | Main firmware | This sketch |
| --- | --- | --- |
| Sample interval | 30 s | **60 s** |
| Batch send | every 10 min, 20 readings | **every 20 min**, 20 readings |
| BMV080 duty period | 20 s | **30 s** |
| Between tasks | `delay(100)`, always awake | **light sleep in 5 s ticks** |
| Battery | not measured | **read from the Victron over BLE** every 10 min: `batteryVoltage`, `chargeCurrent`, `solarPower`, `chargerState` |
| Detector | warmup 20 samples, alpha 0.005 | warmup 10, alpha 0.01 - same minutes |

Expected draw drops from ~24 Wh/day to roughly 6, with the charge controller's
own 3 Wh/day now the largest single item.

## Setting it up

1. Copy `config.example.h` to `config.h`, fill in the usual identity, and set
   `NOTEHUB_MODE "periodic"`.
2. In VictronConnect, open the SmartSolar, gear icon, **Product info**, turn
   on **Instant readout via Bluetooth**, then **Show encryption data** and
   copy the 32-character key into `VICTRON_BLE_KEY`.
3. Leave `SLEEP_MODE 1` and `VICTRON_BLE_VERBOSE 1` for the first run.
4. Upload with the serial monitor open.

## What the first run should show

```
Sleep mode:    1 (unless the serial monitor is open)
Battery:       Victron BLE readout
...
[PM] ready - duty cycling, one sample every 30 s
[BATT] scanning for the Victron for up to 4 s...
[BATT] victron adv from c0:..:.. rssi -61: product 0xa05f record 0x01 nonce 1234 raw e1021...
[BATT] 12.43 V, +0.3 A, PV 6 W, today 0.02 kWh, bulk, rssi -61
[SENS] T:21.4°C H:38.2% P:975.1hPa Gas:31.2kΩ PM2.5:2.1 Batt:12.43V 2026-...
[STATUS] up 120 s | ... | batt 12.43V bulk | ... | asleep 0% (0 ticks)
[SEND] Sending 3 readings...
[NC] note.add data.qo: ok, 1 note(s) waiting in that file
[SEND] Queued 3 readings
[NC] sync requested
[NC] sync status: completed {sync-end} | completed 12 s ago | requested 40 s ago
```

The first batch leaves after three readings (about five minutes from boot, as
the two warm-up samples are not buffered), so a `data.qo` event should appear
in Notehub within six or seven minutes of flashing. After that batches go out
every 20 minutes. The boot banner also prints `Last reset:`; anything marked
CRASH or BROWNOUT means the node restarted on its own, which is worth
reporting along with what it was doing just before.

With the monitor open the node does not sleep (`asleep 0%`), which is the
point of mode 1: everything else is exactly as it will run in the field.

**To prove sleep works**, set `SLEEP_MODE 2` once. The serial port drops at
every sleep; reopen the monitor after a minute or two and the status line
should read `asleep 90%` or better. Then put it back to 1 for deployment.

## If something is wrong

| Log says | Meaning |
| --- | --- |
| `key mismatch: advert says the key starts 0x..` | the key in config.h is not this controller's |
| `decoded to nonsense` | key is the right length but wrong; re-copy it |
| `no readout decoded` and no `victron adv` lines | Instant Readout is off, or the node is out of BLE range (~10 m) |
| `not a solar charger record` on every advert | another Victron product nearby; the solar charger will also show |
| `light sleep rejected` | the core refused light sleep; the node falls back to delay and keeps running |
| `asleep` stays at 0% in mode 2 | sleep is being entered and left instantly; report the ticks count |
| `note.add data.qo: ` followed by an error | the Notecard refused the batch; the text is its reason |
| `sync status: ... ALERT: last sync failed` | the modem could not reach Notehub; check the antenna, signal and the product UID in config.h |
| `device.qo` reaches Notehub but `data.qo` never does | the node is restarting before its first batch; see `Last reset:` in the boot banner |

## What to report back

The first ~3 minutes of serial output, plus one status line from later on.
That is enough to see the battery readout, the sample cadence and whether
sleep is happening.
