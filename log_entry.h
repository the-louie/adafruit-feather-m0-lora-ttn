/*
 * Log entry type for batched temperature payload. Separate header so the type
 * is defined before any use when the Arduino build merges/includes the sketch.
 * Copyright (c) 2026 the_louie
 */
#ifndef M0_LORAWAN_LOG_ENTRY_H
#define M0_LORAWAN_LOG_ENTRY_H

/** One log entry: 2-byte temperature only. Time is implicit from header baseTick + entry index (5-min spacing). */
struct LogEntryS {
    uint16_t temperature; /* Centidegrees 0-3000 (0.00-30.00°C); 0xFFFD=>30, 0xFFFE=<0, 0xFFFF=error */
} __attribute__((packed));
typedef struct LogEntryS AppLogEntry;
#define LOG_ENTRIES_MAX 48   /* RAM buffer capacity */
#define LOG_SEND_CAP    43   /* max entries per uplink (6+2*43=92 bytes <= EU868 222) */

#endif /* M0_LORAWAN_LOG_ENTRY_H */
