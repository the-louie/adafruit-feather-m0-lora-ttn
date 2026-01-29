#include <lmic.h>

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Adapted for Adafruit feather m0 LoRa by Stefan Huber
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends the actual battery voltage, using frequency and
 * encryption settings matching those of the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
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
 * Do not forget to define the radio type correctly in config.h.
 *
 * Batched log (c) 2026: measure every 5 min, append LogEntry (1-min tick + temperature) to RAM; every 6 h backup
 * RAM to Flash, attempt uplink; on success clear Flash, on failure keep Flash and retry in 6 h with next batch merged.
 * Payload: [vbat×100][n][timeTick_3B,temperature × n] big-endian; tick = 1-min since 2026-01-01; temperature = centidegrees 0-3000 or sentinels.
 * RTC keeps time across sleep; epoch persisted in flash. RTC synced via DeviceTimeReq (first after EV_JOINED, ≤1/24 h).
 * Session saved on join, restored on wake.
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

/* RUN_MODE: DEV = short intervals + Serial; TEST = medium intervals + Serial; PROD = 5 min / 6 h, no Serial. */
#define RUN_MODE_DEV  1
#define RUN_MODE_TEST 2
#define RUN_MODE_PROD 3
#define RUN_MODE RUN_MODE_DEV

#if RUN_MODE == RUN_MODE_DEV
#define MEASURE_INTERVAL_SEC  30u     /* 30 s */
#define SEND_PERIOD_MEASURES  1u      /* upload every wake (measured data + backlog) */
#define SLEEP_SECONDS        30
#elif RUN_MODE == RUN_MODE_TEST
#define MEASURE_INTERVAL_SEC  30u     /* 30 s */
#define SEND_PERIOD_MEASURES  1u      /* upload every wake (measured data + backlog) */
#define SLEEP_SECONDS        30
#else
/* PROD: measure every 5 min, upload every 5 min (every wake = measure + upload). */
#define MEASURE_INTERVAL_SEC  300u
#define SEND_PERIOD_MEASURES  1u      /* upload every wake (measured data + backlog) */
#define SLEEP_SECONDS        300
#endif

#if RUN_MODE == RUN_MODE_PROD
#define SERIAL_PRINT(...)
#define SERIAL_PRINTLN(...)
#define SERIAL_BEGIN(...)
#else
#define SERIAL_PRINT(...)    Serial.print(__VA_ARGS__)
#define SERIAL_PRINTLN(...)  Serial.println(__VA_ARGS__)
#define SERIAL_BEGIN(...)    Serial.begin(__VA_ARGS__)
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

/** One log entry: 3-byte 1-min tick and 2-byte high-res temperature. Packed 5-byte layout. */
struct __attribute__((packed)) LogEntry {
    uint8_t timeTick[3];  /* 1-min tick since 2026-01-01 00:00:00 UTC, big-endian */
    uint16_t temperature; /* Centidegrees 0-3000 (0.00-30.00°C); 0xFFFD=>30, 0xFFFE=<0, 0xFFFF=error */
};
#define LOG_ENTRIES_MAX 48   /* RAM buffer capacity */
#define LOG_SEND_CAP     43  /* max entries per uplink (3+5*43=218 bytes ≤ EU868 222) */

static inline uint32_t getTick24(const LogEntry* e) {
    return ((uint32_t)(e->timeTick[0]) << 16) | ((uint32_t)(e->timeTick[1]) << 8) | (uint32_t)(e->timeTick[2]);
}
static inline void setTick24(LogEntry* e, uint32_t val) {
    val &= 0xFFFFFFu;
    e->timeTick[0] = (uint8_t)(val >> 16);
    e->timeTick[1] = (uint8_t)(val >> 8);
    e->timeTick[2] = (uint8_t)(val);
}

static LogEntry dataBuffer[LOG_ENTRIES_MAX];
static uint8_t ramCount;  /* number of valid entries in dataBuffer */

#define LOG_FLASH_MAGIC 0x4C4F4732  /* "LOG2" – 5-byte LogEntry, 1-min tick; LOG1 incompatible */
typedef struct {
    uint32_t magic;
    uint8_t count;
    uint8_t reserved[3];
    LogEntry entries[LOG_ENTRIES_MAX];
} PersistentLog_t;

static PersistentLog_t persistentLog;
FlashStorage(logStore, PersistentLog_t);

// AppEUI from docs/app-device-config.h (LORAWAN_APP_EUI). Little-endian.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// DevEUI from docs/app-device-config.h (LORAWAN_DEVICE_EUI "70B3D57ED007560E"). Little-endian.
static const u1_t PROGMEM DEVEUI[8] = { 0x0E, 0x56, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// AppKey from docs/app-device-config.h (LORAWAN_APP_KEY). Copied as-is from TTN.
static const u1_t PROGMEM APPKEY[16] = { 0x72, 0xA5, 0x30, 0xB5, 0xB0, 0xE7, 0xC3, 0x0C, 0x65, 0x2D, 0x66, 0xAD, 0x6D, 0xA9, 0x2C, 0xD5 };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

#define PERSIST_MAGIC 0x4C4D4943  // "LMIC"

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
} PersistentData_t;

PersistentData_t persistentData;
FlashStorage(persistentStore, PersistentData_t);

static osjob_t sendjob;
static bool haveStoredSession;

void do_wake(osjob_t* j);  /* measure, append, then backup+send or sleep */

// Pin mapping

const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {3,6,LMIC_UNUSED_PIN},
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
    persistentData.magic = PERSIST_MAGIC;
    persistentStore.write(persistentData);
    SERIAL_PRINT(F("RTC synced to epoch: "));
    SERIAL_PRINTLN(unixEpoch);
}

// Suppress Serial during EV_JOINING..EV_JOINED/FAILED so runloop can service RX windows (~1-2 s after Join Request).
void onEvent (ev_t ev) {
    static bool joining = false;
    bool quiet = joining && (ev != EV_JOINED && ev != EV_JOIN_FAILED);
    if (!quiet) {
        SERIAL_PRINT(os_getTime());
        SERIAL_PRINT(": ");
    }
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            if (!quiet) SERIAL_PRINTLN(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            if (!quiet) SERIAL_PRINTLN(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            if (!quiet) SERIAL_PRINTLN(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            if (!quiet) SERIAL_PRINTLN(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            joining = true;
            if (!quiet) SERIAL_PRINTLN(F("EV_JOINING"));
            break;
        case EV_JOINED:
            joining = false;
            SERIAL_PRINTLN(F("EV_JOINED"));
            LMIC_setLinkCheckMode(0);
            persistentData.magic = PERSIST_MAGIC;
            persistentData.netid = (uint32_t)LMIC.netid;
            persistentData.devaddr = (uint32_t)LMIC.devaddr;
            memcpy(persistentData.nwkKey, LMIC.nwkKey, 16);
            memcpy(persistentData.artKey, LMIC.artKey, 16);
            persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
            persistentStore.write(persistentData);
            LMIC_requestNetworkTime(userNetworkTimeCallback, NULL);
            {
                uint8_t helloPayload[] = "HELLO WORLD";
                LMIC_setTxData2(2, helloPayload, (u1_t)(sizeof(helloPayload) - 1u), 0);
                if (!quiet) SERIAL_PRINTLN(F("HELLO WORLD uplink queued on FPort 2"));
            }
            break;
        case EV_RFU1:
            if (!quiet) SERIAL_PRINTLN(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            joining = false;
            SERIAL_PRINTLN(F("EV_JOIN_FAILED"));
            // Retry join after 60 s (no Join Accept in RX window; avoid blocking runloop during join).
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(60), do_send);
            break;
        case EV_REJOIN_FAILED:
            if (!quiet) SERIAL_PRINTLN(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            if (!quiet) SERIAL_PRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (!quiet && (LMIC.txrxFlags & TXRX_ACK)) SERIAL_PRINTLN(F("Received ack"));
            if (!quiet && LMIC.dataLen) {
                SERIAL_PRINTLN(F("Received "));
                SERIAL_PRINTLN(LMIC.dataLen);
                SERIAL_PRINTLN(F(" bytes of payload"));
            }
            /* Send success: clear Flash log so next backup starts fresh. */
            persistentLog.magic = 0;
            persistentLog.count = 0;
            logStore.write(persistentLog);
            persistentData.magic = PERSIST_MAGIC;
            persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
            persistentData.rtcEpoch = (uint32_t)rtc.getEpoch();
            persistentStore.write(persistentData);
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_sleep);
            break;
        case EV_LOST_TSYNC:
            if (!quiet) SERIAL_PRINTLN(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            if (!quiet) SERIAL_PRINTLN(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            if (!quiet) SERIAL_PRINTLN(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            if (!quiet) SERIAL_PRINTLN(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            if (!quiet) SERIAL_PRINTLN(F("EV_LINK_ALIVE"));
            break;
        case 16:
            if (!quiet) SERIAL_PRINTLN(F("EV_SCAN_FOUND (MCCI/extended)"));
            break;
        case 17:
            if (!quiet) SERIAL_PRINTLN(F("EV_TXSTART (radio TX started)"));
            break;
        case 18:
            if (!quiet) SERIAL_PRINTLN(F("EV_TXCANCELED (MCCI/extended)"));
            break;
        case 19:
            if (!quiet) SERIAL_PRINTLN(F("EV_RXSTART (opening RX window)"));
            break;
        case 20:
            if (!quiet) SERIAL_PRINTLN(F("EV_JOIN_TXCOMPLETE (Join Request sent, waiting Join Accept)"));
            break;
        default:
            if (!quiet) { SERIAL_PRINT(F("Unknown event ")); SERIAL_PRINTLN((int)ev); }
            break;
    }
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
static uint16_t buildBatchPayload(uint8_t* buf, uint16_t vbatCentivolt, uint8_t n, const LogEntry* entries, uint32_t currentTicks) {
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
    if (LMIC.opmode & OP_TXRXPEND) {
        SERIAL_PRINTLN(F("OP_TXRXPEND, not sending"));
        return;
    }
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

    LMIC_setTxData2(1, paybuf, (uint8_t)payLen, 0);
    SERIAL_PRINTLN(F("Packet queued (batch)"));
}

/** Merge Flash log with RAM buffer, cap at LOG_ENTRIES_MAX, save to Flash, clear RAM. */
static void mergeBackupAndPrepareSend(void) {
    logStore.read(&persistentLog);
    uint8_t flashCount = (persistentLog.magic == LOG_FLASH_MAGIC && persistentLog.count <= LOG_ENTRIES_MAX)
        ? persistentLog.count : 0;
    uint8_t total = flashCount + ramCount;
    if (total > LOG_ENTRIES_MAX) total = LOG_ENTRIES_MAX;
    /* Build merged list: flash entries first, then RAM entries, in a temp buffer. */
    LogEntry merged[LOG_ENTRIES_MAX];
    uint8_t i = 0;
    for (uint8_t k = 0; k < flashCount && i < total; k++) merged[i++] = persistentLog.entries[k];
    for (uint8_t k = 0; k < ramCount && i < total; k++) merged[i++] = dataBuffer[k];
    persistentLog.magic = LOG_FLASH_MAGIC;
    persistentLog.count = i;
    for (uint8_t k = 0; k < i; k++) persistentLog.entries[k] = merged[k];
    logStore.write(persistentLog);
    ramCount = 0;
}

/** Called after each wake (from setup or after do_sleep). Measure, append to RAM, then either sleep or backup+send. */
void do_wake(osjob_t* j) {
    (void)j;
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    uint32_t epoch = (uint32_t)rtc.getEpoch();
    uint32_t t = (epoch < CUSTOM_EPOCH) ? CUSTOM_EPOCH : epoch;
    uint32_t ticks = (t - CUSTOM_EPOCH) / SECONDS_PER_TICK;
    if (ticks > 0xFFFFFFu) ticks = 0xFFFFFFu;
    LogEntry e;
    setTick24(&e, (uint32_t)ticks);
    e.temperature = encodeTemperatureHighRes(tempC);

    if (ramCount < LOG_ENTRIES_MAX) {
        dataBuffer[ramCount++] = e;
    } else {
        memmove(dataBuffer, &dataBuffer[1], (LOG_ENTRIES_MAX - 1) * sizeof(LogEntry));
        dataBuffer[LOG_ENTRIES_MAX - 1] = e;
    }

    uint8_t period = persistentData.measuresInPeriod % SEND_PERIOD_MEASURES;
    period = (period + 1) % SEND_PERIOD_MEASURES;
    persistentData.measuresInPeriod = period;
    persistentData.wakeCounter++;
    persistentData.magic = PERSIST_MAGIC;
    persistentStore.write(persistentData);

    SERIAL_PRINT(F("Wake #"));
    SERIAL_PRINT(persistentData.wakeCounter);
    SERIAL_PRINT(F(" ts="));
    SERIAL_PRINT(epoch);
    SERIAL_PRINT(F(" Temp="));
    SERIAL_PRINTLN(tempC);

    if (period == 0) {
        /* SEND_PERIOD_MEASURES-th wake: backup RAM to Flash (merge with any existing), then send. */
        mergeBackupAndPrepareSend();
        do_send(&sendjob);
    } else {
        do_sleep(&sendjob);
    }
}

// STANDBY sleep via Arduino Low Power (re-inits clocks, RTC wake). Optional: detach USB before
// sleep for true ~5-15 µA; set DETACH_USB_BEFORE_SLEEP 1 and provide USBDevice if your core has it.
#define DETACH_USB_BEFORE_SLEEP 0
#if DETACH_USB_BEFORE_SLEEP
#include <USB/USBDevice.h>
#endif

void do_sleep(osjob_t* j) {
    (void)j;
    persistentData.magic = PERSIST_MAGIC;
    persistentStore.write(persistentData);

#if DETACH_USB_BEFORE_SLEEP
    USBDevice.detach();
#endif
    LowPower.deepSleep((uint32_t)SLEEP_SECONDS * 1000UL);
#if DETACH_USB_BEFORE_SLEEP
    USBDevice.attach();
#endif
    do_wake(&sendjob);
}


void setup() {
#if RUN_MODE == RUN_MODE_DEV
    /* Give developer time to upload new code before device enters deep sleep (development phase only). */
    delay(10000);
#endif
    SERIAL_BEGIN(9600);
    delay(2000);
    SERIAL_PRINTLN(F("Starting"));

    persistentStore.read(&persistentData);
    haveStoredSession = (persistentData.magic == PERSIST_MAGIC);
    if (!haveStoredSession) {
        persistentData.wakeCounter = 0;
        persistentData.measuresInPeriod = 0;
    } else {
        persistentData.measuresInPeriod = persistentData.measuresInPeriod % SEND_PERIOD_MEASURES;
    }
    logStore.read(&persistentLog);
    SERIAL_PRINT(F("Wake #"));
    SERIAL_PRINTLN(persistentData.wakeCounter);

    rtc.begin();
    if (haveStoredSession && persistentData.rtcEpoch != 0 && persistentData.rtcEpoch != 0xFFFFFFFFu)
        rtc.setEpoch((time_t)persistentData.rtcEpoch);
    else
        rtc.setEpoch((time_t)RTC_DEFAULT_EPOCH);

    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    if (haveStoredSession) {
        LMIC_setSession(persistentData.netid, (u4_t)persistentData.devaddr,
                        persistentData.nwkKey, persistentData.artKey);
        LMIC.seqnoUp = (u4_t)persistentData.seqnoUp;
        SERIAL_PRINTLN(F("Session restored from flash"));
    }

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    sensors.begin();
    ramCount = 0;
    do_wake(&sendjob);
}

void loop() {
    os_runloop_once();
}
