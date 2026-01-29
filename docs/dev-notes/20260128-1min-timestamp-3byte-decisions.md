# Decisions: 1-min timestamp (3-byte) and 5-min interval

- **Roll-over:** 3-byte tick = 2^24 minutes ≈ 31.9 years from 2026; sufficient.
- **Payload size:** EU868 max 222 bytes (DR0). Format 3+5*n → LOG_SEND_CAP = 43 (218 bytes).
- **Send period:** SEND_PERIOD_MEASURES = 72 so backup+send every 72 × 5 min = 6 h.
- **Flash:** New magic LOG2 (0x4C4F4732); old LOG1 flash is ignored (layout incompatible).
