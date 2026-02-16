/* Log entry type: defined before any #include so no library (e.g. LMIC) macro can shadow AppLogEntry. */
struct LogEntryS {
    uint8_t timeTick[3];
    uint16_t temperature;
} __attribute__((packed));
typedef struct LogEntryS AppLogEntry;
#define LOG_ENTRIES_MAX 48
#define LOG_SEND_CAP    43

#include <lmic.h>

/* Firmware v2.4 – duty cycle / deep sleep per Technical Review 2026-02-16. */

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Adapted for Adafruit feather m0 LoRa by Stefan Huber
 * Updated again in 2026 by the_louie
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends the temperature measured by a DS18B20 temperature sensor,
 * using frequency and encryption settings matching those of the
 * The Things Network (TTN).
 *
 * This uses OTAA (Over-the-air activation), where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Set region and radio in the MCCI LMIC library: lmic_project_config.h (CFG_eu868, CFG_sx1276_radio).
 *
 * Batched log (c) 2026 by the_louie:
 *  - measure every 5 min (DEV, TEST, PROD), append log entry (1-min tick + temperature) to RAM; on send wake merge RAM to Flash, attempt uplink; on success clear Flash, on failure keep Flash and retry next send wake.
 *  - Payload: [vbat×100][n][timeTick_3B,temperature × n] big-endian; tick = 1-min since 2026-01-01; temperature = centidegrees 0-3000 or sentinels.
 *  - RTC keeps time across sleep; epoch persisted in flash. RTC synced via DeviceTimeReq (first after EV_JOINED, ≤1/24 h).
 *  - Session saved on join, restored on wake.
 *
 * TTN / LMIC behaviour: Report interval 5 min (DEV, TEST, PROD) to stay within EU868 duty cycle
 * and avoid wake deadlocks. In RUN_MODE_TEST, ADR is off and SF7 is forced (shortest ToA). SeqNo is persisted in EV_TXCOMPLETE
 * only when the completed TX was application data (FPort > 0); MAC-only completions skip flash
 * to avoid NVM stress during MAC storms. do_sleep forces LMIC reset if radio stays busy > 6 s
 * (MAC loop recovery); that run skips session persist. ADR is off in RUN_MODE_TEST to reduce
 * MAC load. Link Check on after join. EV_LINK_DEAD triggers LMIC_reset and re-join. do_send
 * limits OP_TXRXPEND retries then resets LMIC. RX2 from Join Accept. Clock error 15% for Feather
 * M0 internal oscillator.
 *
 * If updates stop: (1) TTN Live Data: "Join Request without Join Accept" = timing/clock;
 * "Uplink dropped" = frame counter. (2) Confirm DIO1 (RFM95) to pin 6 (M0); without it no RX.
 * (3) Device very close to gateway (under 3 m) can desensitize; try another room. (4) SPI
 * shared (Flash + LoRa). (5) Brown-out during TX (120 mA) can corrupt RFM95; supply decoupling.
 *******************************************************************************/

#include <hal/hal.h>
#include <SPI.h>
// Install from Library Manager: "Arduino Low Power", "FlashStorage" by cmaglie, "Dallas Temperature" by Miles Burton, "RTCZero" by Arduino
#include <ArduinoLowPower.h>
#include <FlashStorage.h>
#include <RTCZero.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <string.h>
#include "app-device-config.h"

#define VBATPIN A7
#define ONE_WIRE_BUS 5  /* DS18B20 data on pin 5 (not A5) */

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

RTCZero rtc;

/* Initial RTC epoch when no persisted value. Set to current Unix time at flash, or 0; network time sync will set RTC after join. */
#ifndef RTC_DEFAULT_EPOCH
#define RTC_DEFAULT_EPOCH 0
#endif

/* GPS epoch (1980-01-06 00:00:00 UTC) to Unix epoch (1970-01-01), minus leap seconds (~18). Used when LMIC returns DeviceTimeAns (GPS seconds). */
#define GPS_TO_UNIX_EPOCH_OFFSET 315964782u

/* Request network time at most this often (seconds). TTN fair use: once per 24 h is sufficient. */
#define NETWORK_TIME_REQUEST_INTERVAL_SEC  (24 * 3600)

/* RX window clock error: 15% in setup() for Feather M0 internal oscillator; compensates for crystal drift over long runs (e.g. 36 h). */

/* RUN_MODE: DEV = 5 min, Serial, 10 s startup delay (upload window; M0 has no boot selector);
 * TEST = 5 min, Serial, no startup delay; PROD = 5 min, no Serial, no startup delay.
 * For 36-hour validation and production, set RUN_MODE to RUN_MODE_TEST or RUN_MODE_PROD; 300 s ensures EU868 1% airtime is cleared before the next wake so the device can transmit and return to deep sleep without deadlock. */
#define RUN_MODE_DEV  1
#define RUN_MODE_TEST 2
#define RUN_MODE_PROD 3
#define RUN_MODE RUN_MODE_DEV

#if RUN_MODE == RUN_MODE_DEV
#define MEASURE_INTERVAL_SEC  300u    /* 5 min */
#define SEND_PERIOD_MEASURES  1u      /* upload every wake (measured data + backlog) */
#define SLEEP_SECONDS        300      /* 5 min */
#elif RUN_MODE == RUN_MODE_TEST
#define MEASURE_INTERVAL_SEC  300u    /* 5 min: aligns with EU868 duty cycle, avoids wake deadlocks */
#define SEND_PERIOD_MEASURES  1u      /* upload every wake (measured data + backlog) */
#define SLEEP_SECONDS        300     /* 5 min */
#else
/* PROD: measure every 5 min, upload every 5 min (every wake = measure + upload). */
#define MEASURE_INTERVAL_SEC  300u
#define SEND_PERIOD_MEASURES  1u      /* upload every wake (measured data + backlog) */
#define SLEEP_SECONDS        300
#endif

/* Serial and logging only in DEV; TEST and PROD do not init Serial or log. */
#if RUN_MODE == RUN_MODE_DEV
#define SERIAL_PRINT(...)    Serial.print(__VA_ARGS__)
#define SERIAL_PRINTLN(...)  Serial.println(__VA_ARGS__)
#define SERIAL_BEGIN(...)    Serial.begin(__VA_ARGS__)
static void blinkLed(void) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
}
#else
#define SERIAL_PRINT(...)
#define SERIAL_PRINTLN(...)
#define SERIAL_BEGIN(...)
#endif

// Cayenne LPP: Digital Input = type 0 (1 byte); Analog Input = type 2 (2 bytes, 0.01, MSB first); custom type 128 = 1-min ticks since 2026-01-01 (3 bytes, big-endian)
#define LPP_DIGITAL_INPUT 0
#define LPP_ANALOG_INPUT  2
#define LPP_TICK_TIME    128

/* 1-minute tick encoding: epoch 2026-01-01 00:00:00 UTC, 1 tick = 1 min, 3 bytes (24 bits). */
#define CUSTOM_EPOCH      1735689600u  /* Unix for 2026-01-01 00:00:00 UTC */
#define SECONDS_PER_TICK  60u          /* 1 minute */

// Temperature (ch2): 1 byte. 0=error, 1=<0°C, 2=>30°C, 3-255=data (temp_c = (byte-3)/8.43, ~0.118°C/step)
#define TEMP_ENCODED_SCALE 8.43f
#define TEMP_ENCODED_OFFSET 3

static inline uint32_t getTick24(const AppLogEntry* e) {
    return ((uint32_t)(e->timeTick[0]) << 16) | ((uint32_t)(e->timeTick[1]) << 8) | (uint32_t)(e->timeTick[2]);
}
static inline void setTick24(AppLogEntry* e, uint32_t val) {
    val &= 0xFFFFFFu;
    e->timeTick[0] = (uint8_t)(val >> 16);
    e->timeTick[1] = (uint8_t)(val >> 8);
    e->timeTick[2] = (uint8_t)(val);
}

static AppLogEntry dataBuffer[LOG_ENTRIES_MAX];
static uint8_t ramCount;  /* number of valid entries in dataBuffer */

/* Session store first so it gets the first flash slot; logStore second. Avoids logStore.write() erase
 * wiping the same flash page as the session on SAMD21 (cmaglie FlashStorage erases by slot). */
#define PERSIST_MAGIC   0x4C4D4943u  /* "LMIC" - legacy; session restore accepted once */
#define PERSIST_MAGIC_V2 0x4C4D4944u  /* V2: includes devEuiTag; restore only if tag matches current DevEUI */
/* LORAWAN_DEV_EUI for session tag is in app-device-config.h; must match DEVEUI below. */

typedef struct {
    uint32_t magic;
    uint32_t wakeCounter;
    uint32_t netid;
    uint32_t devaddr;
    uint8_t nwkKey[16];
    uint8_t artKey[16];
    uint32_t seqnoUp;  /* uplink frame counter; must increase for NS to accept after reset */
    uint32_t rtcEpoch; /* last known Unix time (seconds since Epoch); restored on boot, updated after TX and after network-time sync */
    uint32_t lastTimeSyncEpoch; /* last Unix time we got from DeviceTimeAns; used to throttle requests to once per 24 h */
    uint8_t measuresInPeriod;   /* 0..(SEND_PERIOD_MEASURES-1): wakes since last send; SEND_PERIOD_MEASURES-th triggers backup+send */
    uint64_t devEuiTag;  /* LORAWAN_DEV_EUI when session was saved; restore only if matches current device */
} PersistentData_t;

PersistentData_t persistentData;
FlashStorage(persistentStore, PersistentData_t);

#define LOG_FLASH_MAGIC 0x4C4F4732  /* "LOG2" - 5-byte AppLogEntry, 1-min tick; LOG1 incompatible */
typedef struct {
    uint32_t magic;
    uint8_t count;
    uint8_t reserved[3];
    AppLogEntry entries[LOG_ENTRIES_MAX];
} PersistentLog_t;

static PersistentLog_t persistentLog;
FlashStorage(logStore, PersistentLog_t);

// AppEUI / DevEUI / AppKey from app-device-config.h (copy app-device-config.h.example to app-device-config.h and fill from TTN).
static const u1_t PROGMEM APPEUI[8] = LORAWAN_APP_EUI_LE;
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8] = LORAWAN_DEV_EUI_LE;
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = LORAWAN_APP_KEY_ARR;
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;
static bool haveStoredSession;

/** OP_TXRXPEND retry count; after threshold we LMIC_reset to recover from stuck radio (e.g. brown-out). */
#define DO_SEND_PEND_RETRY_MAX 3
static uint8_t do_send_pend_retries;

/** FPort of last application TX (do_send=1, post_join_first_uplink=2). MAC-only (Port 0) completions do not set this; we only persist to flash when lastTxFPort > 0. */
static uint8_t lastTxFPort;

/** One-shot: queue HELLO WORLD on FPort 2 after join. TTN: allow NS to finalize session before first data. */
static void post_join_first_uplink(osjob_t* j);

void do_wake(osjob_t* j);  /* measure, append, then backup+send or sleep */
void do_send(osjob_t* j);
void do_sleep(osjob_t* j);

/** Commit session to flash. Call sites: EV_JOINED; EV_TXCOMPLETE only when last TX was application (lastTxFPort > 0); do_sleep when measuresInPeriod==0 && haveStoredSession && !skipPersist (skipped after forced radio reset). Reads LMIC/rtc at call time. */
static void save_session_to_flash(void) {
    if (!haveStoredSession) return;
    persistentData.magic = PERSIST_MAGIC_V2;
    persistentData.devEuiTag = (uint64_t)LORAWAN_DEV_EUI;
    persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
    persistentData.rtcEpoch = (uint32_t)rtc.getEpoch();
    SERIAL_PRINTLN(F("[persist] Committing to Flash..."));
    persistentStore.write(persistentData);
    delay(20);
}

/* Pin mapping: Feather M0 LoRa. DIO1 (RFM95) must be jumped to Pin 6 for RX and LMIC events. */
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {3, 6, LMIC_UNUSED_PIN},
};

/* Callback when DeviceTimeAns is received. LMIC gives GPS seconds; convert to Unix and set RTC. Throttle requests to once per 24 h. */
static void userNetworkTimeCallback(void *pUserData, int flagSuccess) {
    (void)pUserData;
    if (flagSuccess != 1) {
        SERIAL_PRINTLN(F("Network time request failed"));
        return;
    }
    lmic_time_reference_t lmicTimeRef;
    if (!LMIC_getNetworkTimeReference(&lmicTimeRef)) {
        SERIAL_PRINTLN(F("Network time reference unavailable"));
        return;
    }
    /* tNetwork is GPS seconds since 1980-01-06; convert to Unix epoch. */
    uint32_t unixEpoch = (uint32_t)lmicTimeRef.tNetwork + GPS_TO_UNIX_EPOCH_OFFSET;
    rtc.setEpoch((time_t)unixEpoch);
    persistentData.rtcEpoch = unixEpoch;
    persistentData.lastTimeSyncEpoch = unixEpoch;
    persistentData.magic = PERSIST_MAGIC_V2;
    persistentData.devEuiTag = (uint64_t)LORAWAN_DEV_EUI;
    /* RAM only; session committed to flash at send cycle or after join via save_session_to_flash(). */
    SERIAL_PRINT(F("RTC synced to epoch: "));
    SERIAL_PRINTLN(unixEpoch);
}

void onEvent (ev_t ev) {
    SERIAL_PRINT(os_getTime());
    SERIAL_PRINT(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            SERIAL_PRINTLN(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            SERIAL_PRINTLN(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            SERIAL_PRINTLN(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            SERIAL_PRINTLN(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            SERIAL_PRINTLN(F("EV_JOINING"));
            break;
        case EV_JOINED:
            SERIAL_PRINTLN(F("EV_JOINED"));
            lastTxFPort = 0;
            LMIC_setLinkCheckMode(1);
#if RUN_MODE == RUN_MODE_TEST
            LMIC_setAdrMode(0);   /* ADR off in TEST to reduce MAC load */
            LMIC_setDrTxpow(5, 14);  /* Force SF7 (EU868 DR5), 14 dBm: shortest ToA (~56 ms), minimal duty wait */
#else
            LMIC_setAdrMode(1);
#endif
            SERIAL_PRINTLN(F("[persist] EV_JOINED: filling persistentData (session)"));
            persistentData.magic = PERSIST_MAGIC_V2;
            persistentData.devEuiTag = (uint64_t)LORAWAN_DEV_EUI;
            persistentData.netid = (uint32_t)LMIC.netid;
            persistentData.devaddr = (uint32_t)LMIC.devaddr;
            memcpy(persistentData.nwkKey, LMIC.nwkKey, 16);
            memcpy(persistentData.artKey, LMIC.artKey, 16);
            persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
            haveStoredSession = true;  /* set before save_session_to_flash so it does not return early */
            SERIAL_PRINT(F("[persist] EV_JOINED: save_session_to_flash DevAddr=0x"));
            SERIAL_PRINT(persistentData.devaddr, HEX);
            SERIAL_PRINT(F(" SeqNo="));
            SERIAL_PRINTLN(persistentData.seqnoUp);
            save_session_to_flash();
            LMIC_requestNetworkTime(userNetworkTimeCallback, NULL);
            /* Delay first uplink so TTN NS can finalize session (3–5 s). */
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(5), post_join_first_uplink);
            break;
        case EV_RFU1:
            SERIAL_PRINTLN(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            SERIAL_PRINTLN(F("EV_JOIN_FAILED"));
            SERIAL_PRINTLN(F("[EV] EV_JOIN_FAILED: schedule do_send (retry) in 60s"));
            // Retry join after 60 s (no Join Accept in RX window; avoid blocking runloop during join).
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(60), do_send);
            break;
        case EV_REJOIN_FAILED:
            SERIAL_PRINTLN(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
#if RUN_MODE == RUN_MODE_DEV
            blinkLed();   /* send succeeded */
#endif
            SERIAL_PRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) SERIAL_PRINTLN(F("Received ack"));
            if (LMIC.dataLen) {
                SERIAL_PRINT(F("[downlink] received len="));
                SERIAL_PRINTLN(LMIC.dataLen);
            }
            /* Update session in RAM; commit to flash only when completed TX was application (FPort > 0) to avoid NVM stress during MAC storms. */
            persistentData.magic = PERSIST_MAGIC_V2;
            persistentData.devEuiTag = (uint64_t)LORAWAN_DEV_EUI;
            persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
            persistentData.rtcEpoch = (uint32_t)rtc.getEpoch();
            if (lastTxFPort > 0u) {
                save_session_to_flash();
                SERIAL_PRINTLN(F("[persist] App data sent. Committing SeqNo."));
                if (lastTxFPort == 1u) {
                    SERIAL_PRINTLN(F("[persist] EV_TXCOMPLETE: logStore.write (clear log)"));
                    persistentLog.magic = 0;
                    persistentLog.count = 0;
                    logStore.write(persistentLog);
                }
                lastTxFPort = 0;
            } else {
                SERIAL_PRINTLN(F("[persist] MAC-only response. Skipping flash."));
            }
            SERIAL_PRINTLN(F("[persist] EV_TXCOMPLETE: schedule do_sleep in 2s"));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_sleep);
            break;
        case EV_LOST_TSYNC:
            SERIAL_PRINTLN(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            SERIAL_PRINTLN(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            SERIAL_PRINTLN(F("EV_RXCOMPLETE"));
            if (LMIC.dataLen) {
                SERIAL_PRINT(F("[downlink] EV_RXCOMPLETE len="));
                SERIAL_PRINTLN(LMIC.dataLen);
            }
            break;
        case EV_LINK_DEAD:
            SERIAL_PRINTLN(F("EV_LINK_DEAD - LMIC_reset, re-join"));
            LMIC_reset();
            LMIC_setClockError(MAX_CLOCK_ERROR * 15 / 100);
            haveStoredSession = false;
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
            break;
        case EV_LINK_ALIVE:
            SERIAL_PRINTLN(F("EV_LINK_ALIVE"));
            break;
        case 16:
            SERIAL_PRINTLN(F("EV_SCAN_FOUND (MCCI/extended)"));
            break;
        case 17:
            SERIAL_PRINTLN(F("EV_TXSTART (radio TX started)"));
            break;
        case 18:
            SERIAL_PRINTLN(F("EV_TXCANCELED (MCCI/extended)"));
            break;
        case 19:
            SERIAL_PRINTLN(F("EV_RXSTART (opening RX window)"));
            break;
        case 20:
            SERIAL_PRINTLN(F("EV_JOIN_TXCOMPLETE (Join Request sent, waiting Join Accept)"));
            break;
        default:
            SERIAL_PRINT(F("Unknown event "));
            SERIAL_PRINTLN((int)ev);
            break;
    }
}

static void post_join_first_uplink(osjob_t* j) {
    (void)j;
#if RUN_MODE == RUN_MODE_DEV
    blinkLed();
#endif
    uint8_t helloPayload[] = "HELLO WORLD";
    lastTxFPort = 2;
    LMIC_setTxData2(2, helloPayload, (u1_t)(sizeof(helloPayload) - 1u), 0);
    SERIAL_PRINTLN(F("HELLO WORLD uplink queued on FPort 2 (post-join)"));
}

/** Encode temperature °C as 2-byte value: 0-3000 = centidegrees (0.00-30.00°C), 0xFFFD=>30, 0xFFFE=<0, 0xFFFF=error. */
static uint16_t encodeTemperatureHighRes(float tempC) {
    if (tempC < -50.0f || tempC != tempC) return 0xFFFFu;
    if (tempC < 0.0f) return 0xFFFEu;
    if (tempC > 30.0f) return 0xFFFDu;
    int v = (int)(tempC * 100.0f);
    if (v > 3000) v = 3000;
    if (v < 0) v = 0;
    return (uint16_t)v;
}

/** Build batched payload: [vbat_hi,vbat_lo][n][timeTick_hi,timeTick_mid,timeTick_lo,temp_hi,temp_lo] × n, big-endian. Max 2+1+5*LOG_SEND_CAP bytes.
 * If currentTicks is valid (<= 0xFFFFFF), entries with tick==0 (recorded before RTC sync) get a plausible tick
 * from current time and 1-min spacing: entry k (0=oldest) uses currentTicks - (n - 1 - k), clamped to 0. */
static uint16_t buildBatchPayload(uint8_t* buf, uint16_t vbatCentivolt, uint8_t n, const AppLogEntry* entries, uint32_t currentTicks) {
    uint16_t i = 0;
    buf[i++] = (uint8_t)(vbatCentivolt >> 8);
    buf[i++] = (uint8_t)(vbatCentivolt & 0xFF);
    buf[i++] = n;
    for (uint8_t k = 0; k < n && (i + 5) <= 256u; k++) {
        uint32_t tick = getTick24(&entries[k]);
        if (tick == 0 && currentTicks <= 0xFFFFFFu) {
            uint32_t offset = (uint32_t)(n - 1 - k);
            tick = (currentTicks >= offset) ? (currentTicks - offset) : 0u;
            tick &= 0xFFFFFFu;
        }
        buf[i++] = (uint8_t)(tick >> 16);
        buf[i++] = (uint8_t)(tick >> 8);
        buf[i++] = (uint8_t)(tick);
        buf[i++] = (uint8_t)(entries[k].temperature >> 8);
        buf[i++] = (uint8_t)(entries[k].temperature & 0xFF);
    }
    return i;
}

void do_send(osjob_t* j) {
    (void)j;
    SERIAL_PRINTLN(F("[send] do_send entry"));
    if (LMIC.opmode & OP_TXRXPEND) {
        do_send_pend_retries++;
        if (do_send_pend_retries < DO_SEND_PEND_RETRY_MAX) {
            SERIAL_PRINTLN(F("[send] OP_TXRXPEND, reschedule do_send in 10s"));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(10), do_send);
        } else {
            SERIAL_PRINTLN(F("[send] OP_TXRXPEND stuck, LMIC_reset then retry"));
            do_send_pend_retries = 0;
            LMIC_reset();
            LMIC_setClockError(MAX_CLOCK_ERROR * 15 / 100);
            if (haveStoredSession) {
                LMIC_setSession(persistentData.netid, (u4_t)persistentData.devaddr, persistentData.nwkKey, persistentData.artKey);
                LMIC.seqnoUp = (u4_t)persistentData.seqnoUp;
            }
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
        }
        return;
    }
    do_send_pend_retries = 0;
    /* 5-min cycle (DEV, TEST, PROD) keeps EU868 duty within limits. */
    /* Request network time at most once per 24 h (TTN fair use). First uplink after join already requested in EV_JOINED. */
    uint32_t nowEpoch = (uint32_t)rtc.getEpoch();
    uint32_t lastSync = persistentData.lastTimeSyncEpoch;
    if (lastSync == 0 || lastSync == 0xFFFFFFFFu || nowEpoch >= lastSync + NETWORK_TIME_REQUEST_INTERVAL_SEC)
        LMIC_requestNetworkTime(userNetworkTimeCallback, NULL);

    uint16_t vbatCentivolt = (uint16_t)(analogRead(VBATPIN) * (2.0f * 3.3f / 1024.0f) * 100.0f);
    uint8_t n = (persistentLog.count <= LOG_SEND_CAP) ? persistentLog.count : (uint8_t)LOG_SEND_CAP;
    uint32_t nowEpochForTicks = (uint32_t)rtc.getEpoch();
    uint32_t tForTicks = (nowEpochForTicks < CUSTOM_EPOCH) ? CUSTOM_EPOCH : nowEpochForTicks;
    uint32_t currentTicks = (nowEpochForTicks >= CUSTOM_EPOCH) ? ((tForTicks - CUSTOM_EPOCH) / SECONDS_PER_TICK) : 0x1000000u;
    uint8_t paybuf[2 + 1 + 5 * LOG_SEND_CAP];
    uint16_t payLen = buildBatchPayload(paybuf, vbatCentivolt, n, persistentLog.entries, currentTicks);

    SERIAL_PRINT(F("Sending batch n="));
    SERIAL_PRINT(n);
    SERIAL_PRINT(F(" VBat="));
    SERIAL_PRINTLN((float)vbatCentivolt / 100.0f);
    SERIAL_PRINT(F("[send] LMIC_setTxData2 port=1 len="));
    SERIAL_PRINT(payLen);
    SERIAL_PRINTLN(F(" (uplink)"));

#if RUN_MODE == RUN_MODE_DEV
    blinkLed();   /* attempting to send */
#endif
    lastTxFPort = 1;
    LMIC_setTxData2(1, paybuf, (uint8_t)payLen, 0);
    SERIAL_PRINTLN(F("[send] Packet queued (batch)"));
}

/** Merge Flash log with RAM buffer, cap at LOG_ENTRIES_MAX, save to Flash, clear RAM. */
static void mergeBackupAndPrepareSend(void) {
    SERIAL_PRINTLN(F("[merge] mergeBackupAndPrepareSend entry"));
    logStore.read(&persistentLog);
    uint8_t flashCount = (persistentLog.magic == LOG_FLASH_MAGIC && persistentLog.count <= LOG_ENTRIES_MAX)
        ? persistentLog.count : 0;
    SERIAL_PRINT(F("[merge] logStore.read flashCount="));
    SERIAL_PRINT(flashCount);
    SERIAL_PRINT(F(" ramCount="));
    SERIAL_PRINTLN(ramCount);
    uint8_t total = flashCount + ramCount;
    if (total > LOG_ENTRIES_MAX) total = LOG_ENTRIES_MAX;
    /* Build merged list: flash entries first, then RAM entries, in a temp buffer. */
    AppLogEntry merged[LOG_ENTRIES_MAX];
    uint8_t i = 0;
    for (uint8_t k = 0; k < flashCount && i < total; k++) merged[i++] = persistentLog.entries[k];
    for (uint8_t k = 0; k < ramCount && i < total; k++) merged[i++] = dataBuffer[k];
    persistentLog.magic = LOG_FLASH_MAGIC;
    persistentLog.count = i;
    for (uint8_t k = 0; k < i; k++) persistentLog.entries[k] = merged[k];
    SERIAL_PRINT(F("[merge] logStore.write count="));
    SERIAL_PRINTLN(i);
    logStore.write(persistentLog);
    ramCount = 0;
}

/** Called after each wake (from setup or after do_sleep). Measure, append to RAM, then either sleep or backup+send. */
void do_wake(osjob_t* j) {
    (void)j;
#if RUN_MODE == RUN_MODE_DEV
    SERIAL_PRINT(F("[DEV] Wake @ millis="));
    SERIAL_PRINTLN(millis());
#endif
    SERIAL_PRINTLN(F("[wake] do_wake entry"));
#if RUN_MODE == RUN_MODE_TEST
    /* Battery test: skip DS18B20 read to save current. */
    float tempC = (float)NAN;
    (void)tempC;
#else
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
#endif
    uint32_t epoch = (uint32_t)rtc.getEpoch();
    SERIAL_PRINT(F("[wake] temp="));
    SERIAL_PRINT(tempC);
    SERIAL_PRINT(F(" epoch="));
    SERIAL_PRINTLN(epoch);
    uint32_t t = (epoch < CUSTOM_EPOCH) ? CUSTOM_EPOCH : epoch;
    uint32_t ticks = (t - CUSTOM_EPOCH) / SECONDS_PER_TICK;
    if (ticks > 0xFFFFFFu) ticks = 0xFFFFFFu;
    AppLogEntry e;
    setTick24(&e, (uint32_t)ticks);
#if RUN_MODE == RUN_MODE_TEST
    e.temperature = 0xFFFFu;  /* sentinel: no sensor read in battery test */
#else
    e.temperature = encodeTemperatureHighRes(tempC);
#endif

    if (ramCount < LOG_ENTRIES_MAX) {
        dataBuffer[ramCount++] = e;
    } else {
        memmove(dataBuffer, &dataBuffer[1], (LOG_ENTRIES_MAX - 1) * sizeof(AppLogEntry));
        dataBuffer[LOG_ENTRIES_MAX - 1] = e;
    }

    uint8_t period = persistentData.measuresInPeriod % SEND_PERIOD_MEASURES;
    period = (period + 1) % SEND_PERIOD_MEASURES;
    persistentData.measuresInPeriod = period;
    persistentData.wakeCounter++;
    SERIAL_PRINT(F("[wake] period="));
    SERIAL_PRINT(period);
    SERIAL_PRINT(F(" haveStoredSession="));
    SERIAL_PRINTLN(haveStoredSession ? 1 : 0);
    if (haveStoredSession) {
        persistentData.magic = PERSIST_MAGIC_V2;
        persistentData.devEuiTag = (uint64_t)LORAWAN_DEV_EUI;
    } else {
        persistentData.magic = 0;
        persistentData.devEuiTag = 0;
    }
    /* Lazy write: no session persist in do_wake; commit only in do_sleep when measuresInPeriod==0 or after join. */

    SERIAL_PRINT(F("Wake #"));
    SERIAL_PRINT(persistentData.wakeCounter);
    SERIAL_PRINT(F(" ts="));
    SERIAL_PRINT(epoch);
    SERIAL_PRINT(F(" Temp="));
    SERIAL_PRINTLN(tempC);

    if (period == 0) {
        SERIAL_PRINTLN(F("[wake] period==0 -> mergeBackupAndPrepareSend then do_send"));
        /* SEND_PERIOD_MEASURES-th wake: backup RAM to Flash (merge with any existing), then send. */
        mergeBackupAndPrepareSend();
        do_send(&sendjob);
    } else {
        SERIAL_PRINTLN(F("[wake] period!=0 -> do_sleep"));
        do_sleep(&sendjob);
    }
}

// STANDBY sleep via Arduino Low Power (re-inits clocks, RTC wake). LMIC's os_time does not advance during deep sleep;
// 15% clock error in setup widens RX window for Join Accept after drift. do_sleep checks radio idle before sleep.
// In TEST/PROD Serial is not used but the USB peripheral may still be on
// (core default). Detach USB before deep sleep for true ~5-15 µA (needs USBDevice from your core). Default: 1 in PROD, 0 in DEV/TEST.
#ifndef DETACH_USB_BEFORE_SLEEP
#if RUN_MODE == RUN_MODE_PROD
#define DETACH_USB_BEFORE_SLEEP 1
#else
#define DETACH_USB_BEFORE_SLEEP 0
#endif
#endif
#if DETACH_USB_BEFORE_SLEEP
#include <USB/USBDevice.h>
#endif

/* do_sleep: radio can stay busy in a MAC handshake; after 3×2 s we force LMIC reset and proceed (skipPersist that run). Radio busy: force reset after 6 s to recover from MAC loop. With 300 s (DEV, TEST, PROD) interval, EU868 1% duty is satisfied by the next wake so the device can TX and return to deep sleep (e.g. 15 µA) without extended busy-wait. */
void do_sleep(osjob_t* j) {
    (void)j;
    SERIAL_PRINTLN(F("[sleep] do_sleep entry"));
    static uint8_t busy_retries = 0;
    bool skipPersist = false;
#if defined(OP_TXDATA)
    if ((LMIC.opmode & (OP_TXRXPEND | OP_TXDATA)) != 0) {
#else
    if ((LMIC.opmode & OP_TXRXPEND) != 0) {
#endif
        SERIAL_PRINT(F("[sleep] opmode=0x"));
        SERIAL_PRINTLN((unsigned)LMIC.opmode, HEX);
        busy_retries++;
        if (busy_retries <= 3) {
            SERIAL_PRINTLN(F("[sleep] Radio busy, retrying in 2s..."));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_sleep);
            return;
        }
        SERIAL_PRINTLN(F("[sleep] Radio STUCK. Resetting LMIC."));
        busy_retries = 0;
        LMIC_reset();
        LMIC_setClockError(MAX_CLOCK_ERROR * 15 / 100);
        if (haveStoredSession) {
            LMIC_setSession(persistentData.netid, (u4_t)persistentData.devaddr, persistentData.nwkKey, persistentData.artKey);
            LMIC.seqnoUp = (u4_t)persistentData.seqnoUp;
        }
        skipPersist = true;
    } else {
        busy_retries = 0;
    }
    SERIAL_PRINTLN(F("[sleep] Radio idle. Proceeding to Deep Sleep."));
    SERIAL_PRINT(F("[sleep] haveStoredSession="));
    SERIAL_PRINTLN(haveStoredSession ? 1 : 0);
    if (persistentData.measuresInPeriod == 0u && haveStoredSession && !skipPersist) {
        save_session_to_flash();  /* includes delay(20) */
    } else {
        delay(10);  /* allow any prior NVM write to complete before deep sleep (SAMD21) */
    }

#if RUN_MODE == RUN_MODE_DEV
    SERIAL_PRINT(F("[DEV] Entering sleep wait — "));
    SERIAL_PRINT(SLEEP_SECONDS);
    SERIAL_PRINT(F(" s @ millis="));
    SERIAL_PRINTLN(millis());
    /* DEV: wait only, no radio/LMIC (simulates deep sleep); CPU stays awake so Serial remains connected. */
    {
        uint32_t start = millis();
        uint32_t durationMs = (uint32_t)SLEEP_SECONDS * 1000UL;
        static uint32_t lastHeartbeatMs = 0;
        while (millis() - start < durationMs) {
            if (lastHeartbeatMs == 0u) lastHeartbeatMs = millis();
            else if ((uint32_t)(millis() - lastHeartbeatMs) >= 300000UL) {
                SERIAL_PRINTLN(F("[heartbeat] 5 min – no TX"));
                lastHeartbeatMs = millis();
            }
            delay(100);
        }
    }
    SERIAL_PRINT(F("[DEV] Sleep wait over — waking @ millis="));
    SERIAL_PRINTLN(millis());
#else
#if DETACH_USB_BEFORE_SLEEP
    USBDevice.detach();
#endif
    LowPower.deepSleep((uint32_t)SLEEP_SECONDS * 1000UL);
#if DETACH_USB_BEFORE_SLEEP
    USBDevice.attach();
#endif
#endif
    do_wake(&sendjob);
}


void setup() {
#if RUN_MODE == RUN_MODE_DEV
    /* Upload window: 10 s for firmware upload before deep sleep (M0 has no boot selector). */
    delay(10000);
#endif
    SERIAL_BEGIN(9600);
#if RUN_MODE == RUN_MODE_DEV
    delay(2000);  /* allow serial monitor to connect */
#endif
    SERIAL_PRINTLN(F("Starting"));
    lastTxFPort = 0;
    SERIAL_PRINT(F("[setup] sizeof(AppLogEntry)="));
    SERIAL_PRINTLN((unsigned)sizeof(AppLogEntry));
    SERIAL_PRINTLN(F("[setup] persistentStore.read"));
    persistentStore.read(&persistentData);
    SERIAL_PRINT(F("[setup] seqnoUp from flash: "));
    SERIAL_PRINTLN(persistentData.seqnoUp);
    SERIAL_PRINT(F("[setup] magic=0x"));
    SERIAL_PRINT(persistentData.magic, HEX);
    SERIAL_PRINT(F(" devaddr=0x"));
    SERIAL_PRINT(persistentData.devaddr, HEX);
    SERIAL_PRINT(F(" seqnoUp="));
    SERIAL_PRINTLN(persistentData.seqnoUp);
    /* Accept PERSIST_MAGIC (legacy) or PERSIST_MAGIC_V2. Do not check devEuiTag: with cmaglie FlashStorage
     * and extended PersistentData_t, the last 8 bytes may read from the next slot (logStore), so devEuiTag
     * can be wrong and would reject a valid session; accept any V2 to avoid join loop. */
    haveStoredSession = (persistentData.magic == PERSIST_MAGIC) || (persistentData.magic == PERSIST_MAGIC_V2);
    SERIAL_PRINT(F("[setup] haveStoredSession="));
    SERIAL_PRINTLN(haveStoredSession ? 1 : 0);
    if (!haveStoredSession) {
        persistentData.wakeCounter = 0;
        persistentData.measuresInPeriod = 0;
    } else {
        persistentData.measuresInPeriod = persistentData.measuresInPeriod % SEND_PERIOD_MEASURES;
    }
    SERIAL_PRINTLN(F("[setup] logStore.read"));
    logStore.read(&persistentLog);
    SERIAL_PRINT(F("Wake #"));
    SERIAL_PRINTLN(persistentData.wakeCounter);

    SERIAL_PRINTLN(F("[setup] rtc.begin"));
    rtc.begin();
    if (haveStoredSession && persistentData.rtcEpoch != 0 && persistentData.rtcEpoch != 0xFFFFFFFFu) {
        rtc.setEpoch((time_t)persistentData.rtcEpoch);
        SERIAL_PRINT(F("[setup] rtc.setEpoch from flash "));
        SERIAL_PRINTLN(persistentData.rtcEpoch);
    } else {
        rtc.setEpoch((time_t)RTC_DEFAULT_EPOCH);
        SERIAL_PRINTLN(F("[setup] rtc.setEpoch default"));
    }

    SERIAL_PRINTLN(F("[setup] os_init LMIC_reset LMIC_setClockError"));
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 15 / 100);  /* 15% for crystal drift over long runs */

    if (haveStoredSession) {
        SERIAL_PRINTLN(F("[setup] LMIC_setSession (restore from flash)"));
        LMIC_setSession(persistentData.netid, (u4_t)persistentData.devaddr,
                        persistentData.nwkKey, persistentData.artKey);
        LMIC.seqnoUp = (u4_t)persistentData.seqnoUp;
        SERIAL_PRINT(F("Session restored from flash DevAddr=0x"));
        SERIAL_PRINT(persistentData.devaddr, HEX);
        SERIAL_PRINT(F(" SeqNo="));
        SERIAL_PRINTLN(persistentData.seqnoUp);
    } else {
        SERIAL_PRINTLN(F("[setup] no session, will OTAA join"));
    }

    SERIAL_PRINTLN(F("[setup] pinMode sensors.begin do_wake"));
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);   /* DEV: blink on send; TEST/PROD: off */
#if RUN_MODE != RUN_MODE_TEST
    sensors.begin();
#endif
    ramCount = 0;
    do_wake(&sendjob);
}

void loop() {
    os_runloop_once();
}
