# Diagnostics

Throwaway sketches for isolating a fault on the bench. Upload one in place of
the node firmware, read the output, then put the node firmware back. None of
them is part of the node's normal operation.

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
