# 1-min timestamp (3-byte) and 5-min interval – implementation

Time-counting was changed from 30-minute periods (2-byte tick) to 1-minute periods (3-byte tick); measure interval is 5 min. Flash layout, uplink payload, and JS decoder use the new format.

**Current behavior:** Epoch 2026-01-01 00:00:00 UTC. One tick = 1 minute (24-bit, 3 bytes big-endian). LogEntry is 5 bytes (3-byte tick + 2-byte temperature). Measure every 5 min; backup+send every 72 wakes (6 h). Up to 43 entries per uplink (EU868 222-byte limit). Flash magic LOG2; old LOG1 flash is ignored. Decoder: 5 bytes per entry, 1-min tick, fallback timestamp from received_at with 5-min spacing when tick is 0.

TODO.md and TODO-summarized.md were not present in this repo; no TODO removal was performed.
