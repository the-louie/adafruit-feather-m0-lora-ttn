# Bugfix: Decoded payload timestamps all 2026-01-01T00:00:00Z

**Date:** 2026-01-28

## Bug

Decoded uplink `entries[].timestamp` were all `2026-01-01T00:00:00.000Z` because the device was sending `timeTick` 0 for every entry.

## Root cause

RTC is set from network time (DeviceTimeAns) after join. Until that runs, `rtc.getEpoch()` is 0 (or RTC_DEFAULT_EPOCH). In `do_wake()`, `epoch` 0 is clamped to `CUSTOM_EPOCH`, so `ticks = (t - CUSTOM_EPOCH) / SECONDS_PER_TICK` is 0. All log entries were therefore stored and sent with `timeTick == 0`. Entries created before the first time sync (or when restoring a session that had never received time) stayed at zero.

## Fix

**Firmware (m0-lorawan-ttn.ino):** When building the batch payload in `do_send()`, compute current 30-min tick from RTC. Pass it into `buildBatchPayload()`. For any entry with `timeTick == 0`, substitute a plausible tick assuming 30-min spacing: entry index k (0 = oldest) uses `currentTicks - (n - 1 - k)`, clamped so it does not underflow. If RTC is still invalid (epoch &lt; CUSTOM_EPOCH), no substitution (use 0x10000 so condition fails). This fixes batches that contain entries recorded before RTC sync.

**Decoder (ttn-decoder-batch.js):** When `ticks === 0`, derive timestamp from `input.recvTime` (or `input.uplink_message.received_at` if present) and 30-min spacing: entry i gets `received_at - (n - 1 - i) * 30 min`. This gives sensible timestamps for legacy payloads or when the device still sends zeros (e.g. formatter not yet updated).

## Files changed

- `m0-lorawan-ttn/m0-lorawan-ttn.ino`: `buildBatchPayload()` gains `currentTicks` and substitutes for zero ticks; `do_send()` computes current ticks and passes them.
- `m0-lorawan-ttn/ttn-decoder-batch.js`: Fallback timestamp from `recvTime`/`received_at` when tick is 0.
