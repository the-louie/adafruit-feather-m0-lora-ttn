# Session persistence with devEuiTag (PERSIST_MAGIC_V2)

Context for PERSIST_MAGIC_V2, devEuiTag, and safety delay on M0. Aligns M0 application-level behavior with pico-lorawan-ttn without changing LMIC/radio.

## Safety delay (DEV only on M0)

Startup delay remains **DEV only** (`#if RUN_MODE == RUN_MODE_DEV`). M0 has no boot selector; the 10 s upload window is only needed in development mode. TEST and PROD have no startup delay.

## PERSIST_MAGIC_V2 and devEuiTag

- **PERSIST_MAGIC_V2** (0x4C4D4944): Persisted session layout includes `devEuiTag` (uint64_t LORAWAN_DEV_EUI). Session is restored only when `magic == PERSIST_MAGIC_V2` and `devEuiTag == LORAWAN_DEV_EUI` (same device).
- **Backward compatibility:** Old flash with **PERSIST_MAGIC** (no tag) is still accepted once for session restore. On first save after that, firmware writes PERSIST_MAGIC_V2 and devEuiTag; thereafter only PERSIST_MAGIC_V2 with matching tag restores.
- LMIC session content (netid, devaddr, keys, seqnoUp) and radio behavior are unchanged; only the restore condition and persisted magic/tag are added.

## File references

- Firmware: `m0-lorawan-ttn/m0-lorawan-ttn.ino` (PERSIST_MAGIC_V2, LORAWAN_DEV_EUI, PersistentData_t.devEuiTag, haveStoredSession, all persist write sites).
- See also: `docs/dev-notes/hist_timecounting.md`, `docs/dev-notes/hist_payloadtimestamp.md`.
