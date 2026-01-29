# Batched log: RAM buffer, Flash backup, 6 h send

## Summary

The M0 LoRaWAN sketch now uses an advanced batched flow instead of a single Cayenne LPP uplink per wake.

**Measure:** Every 30 min the device wakes, reads DS18B20 temperature and RTC, and appends one 4-byte log entry (2-byte 30-min tick since 2026-01-01 00:00:00 UTC, 2-byte high-res temperature in centidegrees) into a RAM buffer. The buffer holds up to 48 entries (24 h). A `LogEntry` struct keeps timeTick and temperature packed correctly.

**Backup:** Every 6 h (every 12th 30-min wake), before attempting an uplink, the device merges the current RAM buffer with any unsent data already in Flash and writes the combined log back to Flash. This ensures no data is lost if the send fails.

**Send:** The device builds one uplink with battery voltage (2 bytes, centivolts) and the batched entries (count + 4 bytes per entry, big-endian) and attempts the LoRaWAN uplink. Payload size is about 60+ bytes for 12 entries (2+1+4×12); up to 48 entries fit within EU868 limits.

**Confirm:** If the send completes successfully (EV_TXCOMPLETE), the Flash log is cleared. If it fails, the data remains in Flash; at the next 6 h boundary the new RAM entries are merged with that Flash log and the device tries again with a larger batch.

**Schedule:** Production uses 30 min sleep (measure every 30 min) and send every 12th wake (6 h). Development uses 30 s sleep so the send cadence is every 12 wakes (~6 min).

**Persistent state:** `PersistentData_t` now includes `measuresInPeriod` (0..11) so after power loss the device still knows whether the next wake is the 12th. A separate `PersistentLog_t` and Flash storage hold the backup log (magic, count, up to 48 LogEntry). Session, RTC epoch, and network-time throttle remain in the existing persistent struct.

**TTN:** Use a Custom payload formatter. The Readme includes a JavaScript decoder that parses battery_v and an entries array (timestamp ISO, temperature_c or temperature_state). Temperature encoding: 0–3000 = centidegrees (0.00–30.00°C); 0xFFFD = over 30°C, 0xFFFE = under 0°C, 0xFFFF = error.

## Files changed

- **m0-lorawan-ttn.ino:** LogEntry struct and dataBuffer, PersistentLog_t and logStore; measure-on-wake (do_wake), merge-backup-send (mergeBackupAndPrepareSend, do_send builds batch); EV_TXCOMPLETE clears Flash log; do_sleep invokes do_wake after sleep; setup invokes do_wake and restores measuresInPeriod / persistentLog.
- **Readme.md:** Batched flow description, payload format, Custom decoder for battery_v and entries[], and short troubleshooting note.

## Design choices

- **LogEntry:** Colleague-defined struct with timeTick (uint16_t) and temperature (uint16_t) ensures consistent 4-byte packing and a clear contract for decoder and firmware.
- **RAM then Flash:** Keeping the last 24 h in RAM (32 KB is enough) avoids Flash wear on every measure; Flash is written only every 6 h on backup and cleared only on send success.
- **Merge on backup:** Unsent Flash log and current RAM are merged (capped at 48) so repeated send failures accumulate data and the next attempt sends a larger batch without dropping older entries.
