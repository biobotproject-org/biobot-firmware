# Diagnostics

Throwaway sketches for isolating a fault on the bench. Upload one in place of
the node firmware, read the output, then put the node firmware back. None of
them is part of the node's normal operation.

## i2c_watch

For an intermittent connector. A one-shot scan answers "is it there right now",
which is the wrong question about a pin that has been rocked loose: it answers
most of the time and drops out on the slightest movement.

This probes 0x17, 0x57, 0x76 and 0x77 ten times a second and keeps score. Run
it, then press, wiggle and gently tug each sensor lead in turn. The address
whose failure count climbs is the bad connection; a sound one stays at zero.

| Verdict | Meaning |
| --- | --- |
| `solid` | no failure in any probe |
| `absent - never answered` | not connected at all, or wrong address |
| `INTERMITTENT - suspect this connector` | answers sometimes: mechanical |

Needs only `Wire`.

## bme_diag

For a BME680/688 that answers on I2C but will not produce a reading.

It talks to nothing but that sensor — no Notecard, no BMV080, no cellular — so
it needs only `Wire` and the Adafruit BME680 library and compiles without the
proprietary Bosch BMV080 SDK.

Every read is timed, because in Adafruit's library a reading can fail in only
two places and the duration says which:

| Duration of a failed read | Where it failed | What it means |
| --- | --- | --- |
| under ~5 ms | `bme68x_set_op_mode()` | the command never landed: an I2C write problem |
| ~the whole measurement window | `bme68x_get_data()` returned "no new data" | the chip took the command but never finished measuring: power, or the chip |

Four configurations run in turn so one run separates the variables:

1. exactly what the node uses (8x/2x/4x, IIR 3, gas heater 320 °C for 150 ms)
2. the same with the heater off — separates heater current from everything else
3. minimal: 1x oversampling, no IIR, heater off
4. minimal at 50 kHz — more timing margin for a long sensor lead

Keep the 12 V supply connected while running it, so the rail is loaded the same
way it is in normal operation.

### Result on the first node, 2026-09-27

Phase 1 — the node's exact configuration, gas heater and all — read the sensor
**8 times out of 8**, each taking 330 ms, which is the full expected
measurement window. So on that node the sensor, its lead, its connector and its
supply are all sound, and the gas heater draws no more current than the rail can
give. Every read failure seen in the node firmware was caused by something else
sharing the I2C bus, not by the sensor.

That is what the sketch is for: it turns "the sensor is broken" into "the sensor
is fine, look elsewhere" in one run.

### Result on the first node, BMV080

Earlier in the same session an I2C scan found 0x57, and the node's own bus probe
reported `bmv080 0` — it answered. Later `begin()` reported nothing usable at
0x57 on the same board, same boot-to-boot wiring. An address that answers
sometimes and not others is not a driver fault and not a firmware fault; it is a
connection. Confirmed against a reported physical cause: a male Qwiic/JST pin on
the particulate sensor lead had been pushed hard enough to rock in its housing.

A pin that visibly rocks has a cracked solder joint or a spread housing contact.
Re-seating the plug may restore it for minutes; the repair is to resolder or
replace that connector.
