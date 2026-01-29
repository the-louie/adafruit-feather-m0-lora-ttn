# Sleep and wake counter: persist before standby

**Date:** 2026-01-27

## Symptom

After EV_TXCOMPLETE and RTC standby, the device showed "Wake #0" again instead of an incremented wake index. Sleep/interval behavior looked wrong (e.g. long gap or no visible increment).

## Cause

On SAMD21/Feather M0, waking from RTCZero `standbyMode()` causes a **full reset**; execution does not continue after `standbyMode()`. So after sleep the device runs `setup()` again and reads from FlashStorage. The wake counter was only written in `do_sleep`, which runs 2 s after EV_TXCOMPLETE. If the board reset during that delay, during standby, or on wake before any later logic, the increment was never persisted, so the next boot saw Wake #0 (or whatever was last in flash).

## Change

The next wake index (`wakeCounter + 1`) and magic are now written to FlashStorage in **EV_TXCOMPLETE**, as soon as we know we are going to sleep, and before scheduling the 2 s callback to `do_sleep`. That way the value is in flash before the delay and before `standbyMode()`. On wake (reset), `setup()` reads and shows Wake #1, Wake #2, etc. The write in `do_sleep` is kept as a second safeguard.

## References

- Arduino docs / RTC sleep: device resets when waking from RTC standby on SAMD21.
- FlashStorage (cmaglie): `write()` persists to NVM; survives reset.
