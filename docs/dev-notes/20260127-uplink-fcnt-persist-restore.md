# Uplink frame counter persist/restore for session restore

**Date:** 2026-01-27

## Problem

After a reset, the device restored its session from flash (netid, devaddr, nwkKey, artKey) and sent uplinks. TTN showed no packets. The device log showed EV_TXSTART and EV_TXCOMPLETE and “Session restored from flash”, so the radio path was fine.

The network server (NS) drops uplinks whose uplink frame counter (FCnt) is not strictly greater than the last accepted value. On reset, LMIC restarts with FCnt 0 (or whatever `LMIC_reset` leaves). The NS had already accepted higher FCnts (e.g. 17), so it rejected the new uplinks.

## Solution

Persist and restore the uplink frame counter (`LMIC.seqnoUp`) together with the session. After each accepted uplink, store the current `LMIC.seqnoUp`. On wake/reset, after `LMIC_setSession(...)`, set `LMIC.seqnoUp` from the stored value so the next uplink uses a FCnt greater than the last one the NS accepted.

## Changes

- **PersistentData_t:** New field `seqnoUp` (uint32_t), stored after the session keys.
- **EV_JOINED:** When saving the session, set `persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp` and include it in the same `persistentStore.write(persistentData)`.
- **EV_TXCOMPLETE:** When persisting before scheduling sleep, set `persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp` so the next uplink’s counter is stored, then write.
- **setup():** After `LMIC_setSession(...)`, add `LMIC.seqnoUp = (u4_t)persistentData.seqnoUp` so the restored session uses the correct FCnt.

## Wake counter showing 0, 2, 4, 6, 8 (every even)

TTN payload `analog_in_0` (wake counter) was 0, 2, 4, 6, 8 instead of 0, 1, 2, 3, 4, … because `do_sleep` was updating `persistentData.wakeCounter` again before writing. EV_TXCOMPLETE already sets `persistentData.wakeCounter = wakeCounter + 1` and writes. If `do_sleep` also sets it (e.g. using `persistentData.wakeCounter + 1` or any formula that uses the already-updated value), the next boot reads that double-incremented value and the payload counter skips odds.

**Fix:** In `do_sleep`, do not set `persistentData.wakeCounter` (or `seqnoUp`). Only set `magic` and call `persistentStore.write(persistentData)` so we re-persist exactly what EV_TXCOMPLETE already wrote. After wake, `wakeCounter = persistentData.wakeCounter` then uses that single-incremented value.

## Compatibility

- Uses direct `LMIC.seqnoUp` (Arduino-LMIC / MCCI LMIC). If your fork uses `LMIC_getSeqnoUp()` / `LMIC_setSeqnoUp()`, you can switch to those and keep the same stored layout.
- Old flash images written before this change have no valid `seqnoUp`. Restoring that value can cause rejects until the device re-joins or flash is cleared. After one successful join and send with the new firmware, `seqnoUp` is stored and later restores correctly.
