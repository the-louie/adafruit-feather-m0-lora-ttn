/* Log entry type: defined before any #include so no library (e.g. LMIC) macro can shadow AppLogEntry. v3.0: 2-byte entry; time from gateway received_at − entry index × interval (FPort). */
struct LogEntryS {
    uint16_t temperature;
} __attribute__((packed));
typedef struct LogEntryS AppLogEntry;
#define LOG_ENTRIES_MAX 48
#define LOG_SEND_CAP    43

#include <lmic.h>
/* No-op when LMIC does not provide LMIC_disableDutyCycle. To enable duty-cycle bypass, use a library that provides it and replace with LMIC_disableDutyCycle(). */
#define APP_DISABLE_LMIC_DUTY_CYCLE() ((void)0)

/* Firmware v3.0 – Timeless Resilience: No RTC; time from gateway received_at. FPort 10 = 300 s, FPort 20 = 10800 s. 4-byte header (vbat, flags, sequence). Join-first; 90 s join budget; 1 h hibernation after 3× join watchdog. Zero-Trust watchdog: 30 s (or 90 s join / 15 s low VBat) awake budget; delta sleep. */

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
 * Batched log (c) 2026 by the_louie, v3.0 Timeless:
 *  - On each wake: measure, append to RAM (newest at index 0); on send wake merge RAM to Flash, uplink on FPort 10 (300 s) or 20 (10800 s); on success clear Flash.
 *  - Payload: 4-byte header [vbat×100][flags][sequence] then 2×n temperature. Flags bit0 = watchdog, bit1 = cold boot. Time from gateway received_at; decoder uses FPort for interval.
 *  - Session saved on join, restored on wake.
 *
 * TTN / LMIC behaviour: Duty cycle disabled; 5-min sleep cycle enforces EU868. Zero-Trust watchdog: if awake exceeds budget (30 s data / 90 s join / 15 s low VBat), force LMIC_reset and do_sleep; optional Watchdog Bit in payload (flags bit0) for TTN visibility.
 * SF7 default, ADR off. After 3 consecutive EV_LINK_DEAD the device uses SF8 for one send cycle then reverts to SF7. All uplinks Unconfirmed (no ACK/retries).
 * 20% clock error after every LMIC_reset (Feather M0 oscillator / RX2 window). SeqNo persisted in EV_TXCOMPLETE
 * only when completed TX was application data (FPort > 0); MAC-only completions skip flash. do_sleep does not wait for radio idle (proactive hand-off); session saved when send cycle complete. MAC throttle: >3 consecutive Port 0 downlinks trigger LMIC reset and duty-cycle disable. Link Check on after join. EV_LINK_DEAD triggers LMIC_reset and re-join. do_send allows at most one 10 s reschedule when OP_TXRXPEND; on second blocked attempt forces sleep. RX2 from Join Accept.
 *
 * If updates stop: (1) TTN Live Data: "Join Request without Join Accept" = timing/clock;
 * "Uplink dropped" = frame counter. (2) Confirm DIO1 (RFM95) to pin 6 (M0); without it no RX.
 * (3) Device very close to gateway (under 3 m) can desensitize; try another room. (4) SPI
 * shared (Flash + LoRa). (5) Brown-out during TX (120 mA) can corrupt RFM95; supply decoupling.
 * Runbook: DETACH_USB_BEFORE_SLEEP (PROD default) for ~5–15 µA sleep; DS18B20 4.7 kΩ pull-up; sensor read timeout 2 s; Starting logs VBat; below 3.4 V sleep doubled, below 3.2 V critical (skip TX, 600 s sleep). BOD33 2.7V. Ensure 1000µF capacitor on supply to prevent false resets during SF7 TX spike. Payload 4+2×43 bytes max (v3.0). SF bump: 3× link dead → SF8 one cycle. Cold Boot test: pull battery, reattach; verify device joins at SF7 and resumes correct seqnoUp from flash (no drift); after 3× EV_LINK_DEAD next cycle may use SF8.
 *******************************************************************************/

#include <hal/hal.h>
#include <SPI.h>
// Install from Library Manager: "Arduino Low Power", "FlashStorage" by cmaglie, "Dallas Temperature" by Miles Burton
#include <ArduinoLowPower.h>
#include <FlashStorage.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <string.h>
#include "app-device-config.h"

#define VBATPIN A7
#define VBAT_VOLTS() (analogRead(VBATPIN) * (2.0f * 3.3f / 1024.0f))
#define VBAT_LOW_THRESHOLD_V 3.4f  /* Below this we double sleep interval to preserve capacity. */
#define VBAT_CRITICAL_THRESHOLD_V 3.2f  /* Below this skip TX (brown-out risk), sleep 600 s. */
#define ONE_WIRE_BUS 5  /* DS18B20 data on pin 5 (not A5). 4.7 kΩ pull-up data→3V required to avoid hang. */
#define DS18B20_READ_TIMEOUT_MS 2000  /* Max wait for conversion; on timeout use sentinel (0xFFFF). */

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/* 20% clock error for Feather M0 internal oscillator: Join Accept and MAC downlinks (e.g. ADR, LinkCheckAns) need wider RX window. */

/* RUN_MODE: DEV = 5 min, TEST = 5 min, PROD = 3 h. FPort 10 = 300 s, FPort 20 = 10800 s (decoder interval mapping). */
#define RUN_MODE_DEV  1
#define RUN_MODE_TEST 2
#define RUN_MODE_PROD 3
#define RUN_MODE RUN_MODE_DEV

#define FPORT_INTERVAL_5MIN  10   /* 300 s interval; decoder uses FPort to infer interval. */
#define FPORT_INTERVAL_3H    20   /* 10800 s (3 h) interval. */
#define INTERVAL_SEC_DEV_TEST  300u
#define INTERVAL_SEC_PROD      10800u

#if RUN_MODE == RUN_MODE_DEV
#define MEASURE_INTERVAL_SEC  300u
#define SEND_PERIOD_MEASURES  1u
#define SLEEP_SECONDS        300
#elif RUN_MODE == RUN_MODE_TEST
#define MEASURE_INTERVAL_SEC  300u
#define SEND_PERIOD_MEASURES  1u
#define SLEEP_SECONDS        300
#else
/* PROD: 3 h measure/send interval. */
#define MEASURE_INTERVAL_SEC  10800u
#define SEND_PERIOD_MEASURES  1u
#define SLEEP_SECONDS         10800
#endif

/* EU868 data rate indices: DR5=SF7 (default), DR4=SF8 (one cycle after 3× EV_LINK_DEAD). */
#define SF7_DR_EU868 5
#define SF8_DR_EU868 4

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
    uint8_t measuresInPeriod;   /* 0..(SEND_PERIOD_MEASURES-1): wakes since last send; SEND_PERIOD_MEASURES-th triggers backup+send */
    uint64_t devEuiTag;  /* LORAWAN_DEV_EUI when session was saved; restore only if matches current device */
} PersistentData_t;

PersistentData_t persistentData;
FlashStorage(persistentStore, PersistentData_t);

#define LOG_FLASH_MAGIC 0x4C4F4732  /* "LOG2" - 5-byte AppLogEntry; legacy, not read as entries in v2.6 */
#define LOG_FLASH_MAGIC_V3 0x4C4F4733u  /* "LOG3" - 2-byte AppLogEntry, newest-first; time from gateway received_at */
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

/** OP_TXRXPEND: at most one 10 s reschedule of do_send; on second blocked attempt we force sleep to save battery. */
#define DO_SEND_PEND_RESCHEDULE_MAX 1
static uint8_t do_send_pend_retries;

/** Failure-based SF bump: after 3 consecutive EV_LINK_DEAD use SF8 for one send cycle then back to SF7. RAM-only; not persisted. */
static uint8_t consecutiveLinkDeadCount = 0;
static bool useSf8NextCycle = false;

/** MAC storm recovery: > MAC_STORM_PORT0_THRESHOLD consecutive Port 0 (MAC-only) downlinks triggers LMIC reset to break sync loops. */
#define MAC_STORM_PORT0_THRESHOLD 3
static uint8_t consecutivePort0Downlinks = 0;
static bool skipPersistDueToMacStorm = false;

/** Zero-Trust watchdog: MCU forces sleep when millis() exceeds this deadline regardless of radio state. */
#define MAX_AWAKE_TIME_MS      30000   /* 30 s for data uplinks */
#define MAX_AWAKE_JOIN_MS     90000   /* 90 s for OTAA join (handshake can exceed 60 s in poor conditions) */
#define MAX_AWAKE_LOW_VBAT_MS 15000   /* 15 s when VBat < 3.4 V */
static uint32_t wakeDeadlineMs = 0;
/** Set when watchdog forces sleep; included in next batch payload (flags byte bit0), then cleared. */
static bool watchdogTriggeredLastWake = false;
/** Join failure backoff: after 3 consecutive watchdog triggers during join, one 1-hour hibernation. */
static uint8_t consecutiveWatchdogTriggers = 0;
static bool hibernationRequested = false;

/** Adaptive join backoff: EV_JOIN_FAILED retry delay 10 min / 30 min / 1 h (RAM-only, reset on EV_JOINED). */
static uint8_t consecutiveJoinFailCount = 0;

/** Set wake deadline for this wake cycle (join vs data, VBat-aware). Zero-Trust awake budget. */
static void setWakeDeadline(bool isJoinPhase, float vbatV) {
    uint32_t ms = (vbatV < VBAT_LOW_THRESHOLD_V) ? MAX_AWAKE_LOW_VBAT_MS
                  : (isJoinPhase ? MAX_AWAKE_JOIN_MS : MAX_AWAKE_TIME_MS);
    wakeDeadlineMs = millis() + ms;
}

/** FPort of last application TX (batch=10/20, post_join_first_uplink=2). MAC-only (Port 0) completions do not set this; we only persist to flash when lastTxFPort > 0. */
static uint8_t lastTxFPort;
/** Set true at boot; cleared after first successful batch TX. Flags bit 1 in payload (cold boot/reset). */
static bool coldBootThisRun = true;

/** One-shot: queue HELLO WORLD on FPort 2 after join. TTN: allow NS to finalize session before first data. */
static void post_join_first_uplink(osjob_t* j);

void do_wake(osjob_t* j);  /* measure, append, then backup+send or sleep */
void do_send(osjob_t* j);
void do_sleep(osjob_t* j);

/** Commit session to flash. Call sites: EV_JOINED; EV_TXCOMPLETE when last TX was application (lastTxFPort > 0); do_sleep when measuresInPeriod==0 && haveStoredSession && !skipPersist && seqnoUp increased. Reads LMIC at call time. */
static void save_session_to_flash(void) {
    if (!haveStoredSession) return;
    persistentData.magic = PERSIST_MAGIC_V2;
    persistentData.devEuiTag = (uint64_t)LORAWAN_DEV_EUI;
    persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
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

/** Classify downlink: dataLen==0 = Port 0 (MAC-only); dataLen>0 then FPort from LMIC.frame[LMIC.dataBeg-1]. Update consecutivePort0Downlinks; if > threshold, LMIC_reset + disableDutyCycle + session restore + schedule do_send, return true. */
static bool updatePort0CounterAndMaybeReset(void) {
    if (LMIC.dataLen == 0) {
        consecutivePort0Downlinks++;
    } else if (LMIC.dataBeg >= 1) {
        uint8_t fport = LMIC.frame[LMIC.dataBeg - 1];
        if (fport == 0)
            consecutivePort0Downlinks++;
        else
            consecutivePort0Downlinks = 0;
    } else {
        consecutivePort0Downlinks = 0;
    }
    if (consecutivePort0Downlinks <= MAC_STORM_PORT0_THRESHOLD)
        return false;
    SERIAL_PRINTLN(F("[MAC] >3 consecutive Port 0 downlinks, reset to break storm"));
    consecutivePort0Downlinks = 0;
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);  /* 20% for Feather M0 internal oscillator / RX2 window */
    APP_DISABLE_LMIC_DUTY_CYCLE();
    if (haveStoredSession) {
        LMIC_setSession(persistentData.netid, (u4_t)persistentData.devaddr, persistentData.nwkKey, persistentData.artKey);
        LMIC.seqnoUp = (u4_t)persistentData.seqnoUp;
        LMIC_setAdrMode(0);
        LMIC_setDrTxpow(5, 14);
    }
    skipPersistDueToMacStorm = true;
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
    return true;
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
            consecutiveJoinFailCount = 0;  /* Success resets join backoff. */
            lastTxFPort = 0;
            consecutivePort0Downlinks = 0;  /* Fresh join; clear MAC Port 0 counter. */
            LMIC_setLinkCheckMode(1);
            LMIC_setAdrMode(0);   /* ADR off in all run modes: no SF drift to SF12, multi-year battery */
            LMIC_setDrTxpow(5, 14);  /* SF7 (EU868 DR5), 14 dBm: shortest ToA (~56 ms) */
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
            /* Join-first power isolation: sensors.begin() only after EV_JOINED so OneWire/Pin 5 stay inert during join. */
#if RUN_MODE != RUN_MODE_TEST
            sensors.begin();
#endif
            /* Delay first uplink so TTN NS can finalize session (3–5 s). */
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(5), post_join_first_uplink);
            break;
        case EV_RFU1:
            SERIAL_PRINTLN(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED: {
            SERIAL_PRINTLN(F("EV_JOIN_FAILED"));
            if (consecutiveJoinFailCount < 255u) consecutiveJoinFailCount++;
            if (consecutiveJoinFailCount > 3u) consecutiveJoinFailCount = 3u;
            uint32_t backoffSec = (consecutiveJoinFailCount == 1u) ? 600u : (consecutiveJoinFailCount == 2u) ? 1800u : 3600u;
            SERIAL_PRINT(F("[EV] EV_JOIN_FAILED: retry do_send in "));
            SERIAL_PRINT(backoffSec);
            SERIAL_PRINTLN(F("s (adaptive backoff)"));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(backoffSec), do_send);
            break;
        }
        case EV_REJOIN_FAILED:
            SERIAL_PRINTLN(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            /* Start DS18B20 conversion so it runs during the 2 s pre-sleep delay (overlap). */
#if RUN_MODE != RUN_MODE_TEST
            sensors.setWaitForConversion(false);
            sensors.requestTemperatures();
#endif
#if RUN_MODE == RUN_MODE_DEV
            blinkLed();   /* send succeeded */
#endif
            SERIAL_PRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) SERIAL_PRINTLN(F("Received ack"));
            if (LMIC.dataLen) {
                SERIAL_PRINT(F("[downlink] received len="));
                SERIAL_PRINTLN(LMIC.dataLen);
            }
            if ((LMIC.txrxFlags & TXRX_ACK) || (LMIC.dataLen > 0)) {
                if (updatePort0CounterAndMaybeReset())
                    break;
            }
            /* Update session in RAM; commit to flash only when completed TX was application (FPort > 0) to avoid NVM stress during MAC storms. */
            persistentData.magic = PERSIST_MAGIC_V2;
            persistentData.devEuiTag = (uint64_t)LORAWAN_DEV_EUI;
            persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
            if (lastTxFPort > 0u) {
                consecutiveLinkDeadCount = 0;  /* Success breaks consecutive link-dead streak */
                save_session_to_flash();
                SERIAL_PRINTLN(F("[persist] App data sent. Committing SeqNo."));
                if (lastTxFPort == FPORT_INTERVAL_5MIN || lastTxFPort == FPORT_INTERVAL_3H) {
                    coldBootThisRun = false;
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
            (void)updatePort0CounterAndMaybeReset();
            break;
        case EV_LINK_DEAD:
            SERIAL_PRINTLN(F("EV_LINK_DEAD - LMIC_reset, re-join"));
            LMIC_reset();
            LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);  /* 20% for Feather M0 / RX2 window */
            APP_DISABLE_LMIC_DUTY_CYCLE();
            haveStoredSession = false;
            /* After 3 consecutive EV_LINK_DEAD use SF8 for one cycle. */
            consecutiveLinkDeadCount++;
            if (consecutiveLinkDeadCount >= 3) useSf8NextCycle = true;
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
            SERIAL_PRINT(F("EV_TXSTART datarate="));
            SERIAL_PRINTLN(LMIC.datarate);  /* 5 = DR5/SF7; any other value suggests override, investigate */
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

/** v3.0 payload: 4-byte header [VBat_Hi,VBat_Lo][Flags][Sequence] then 2×n temp. Flags: bit0=watchdog, bit1=cold boot. Sequence = wakeCounter & 0xFF for gap detection. */
static uint16_t buildBatchPayload(uint8_t* buf, uint16_t vbatCentivolt, uint8_t flagsByte, uint8_t sequenceByte, uint8_t n, const AppLogEntry* entries) {
    uint16_t i = 0;
    buf[i++] = (uint8_t)(vbatCentivolt >> 8);
    buf[i++] = (uint8_t)(vbatCentivolt & 0xFF);
    buf[i++] = flagsByte;
    buf[i++] = sequenceByte;
    for (uint8_t k = 0; k < n && (i + 2) <= 256u; k++) {
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
        if (do_send_pend_retries <= DO_SEND_PEND_RESCHEDULE_MAX) {
            SERIAL_PRINTLN(F("[send] OP_TXRXPEND, reschedule do_send in 10s"));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(10), do_send);
        } else {
            SERIAL_PRINTLN(F("[send] Radio busy, forcing sleep to save battery"));
            do_send_pend_retries = 0;  /* next wake gets one reschedule again */
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(0), do_sleep);
        }
        return;
    }
    do_send_pend_retries = 0;
    /* Failure-based SF bump: SF8 one cycle after 3× EV_LINK_DEAD; else SF7. No persistence across reboot. */
    if (useSf8NextCycle) {
        LMIC_setDrTxpow(SF8_DR_EU868, 14);
        useSf8NextCycle = false;
        consecutiveLinkDeadCount = 0;
    } else {
        LMIC_setDrTxpow(SF7_DR_EU868, 14);
    }

    uint16_t vbatCentivolt = (uint16_t)(VBAT_VOLTS() * 100.0f);
    /* Merge RAM into Flash so batch includes backlog; normal path already merged in do_wake. */
    if (ramCount > 0u) mergeBackupAndPrepareSend();
    uint8_t n = (persistentLog.count <= LOG_SEND_CAP) ? persistentLog.count : (uint8_t)LOG_SEND_CAP;
    uint8_t flagsByte = (watchdogTriggeredLastWake ? 1u : 0u) | (coldBootThisRun ? 2u : 0u);
    if (watchdogTriggeredLastWake) watchdogTriggeredLastWake = false;
    uint8_t sequenceByte = (uint8_t)(persistentData.wakeCounter & 0xFF);
    uint8_t paybuf[4 + 2 * LOG_SEND_CAP];
    uint16_t payLen = buildBatchPayload(paybuf, vbatCentivolt, flagsByte, sequenceByte, n, persistentLog.entries);

    uint8_t targetPort = (RUN_MODE == RUN_MODE_PROD) ? FPORT_INTERVAL_3H : FPORT_INTERVAL_5MIN;
    SERIAL_PRINT(F("Sending batch n="));
    SERIAL_PRINT(n);
    SERIAL_PRINT(F(" VBat="));
    SERIAL_PRINTLN((float)vbatCentivolt / 100.0f);
    SERIAL_PRINT(F("[send] LMIC_setTxData2 port="));
    SERIAL_PRINT(targetPort);
    SERIAL_PRINT(F(" len="));
    SERIAL_PRINTLN(payLen);

#if RUN_MODE == RUN_MODE_DEV
    blinkLed();   /* attempting to send */
#endif
    lastTxFPort = targetPort;
    /* All uplinks Unconfirmed (4th arg 0) for battery life; no ACK wait/retries. */
    LMIC_setTxData2(targetPort, paybuf, (uint8_t)payLen, 0);
    SERIAL_PRINTLN(F("[send] Packet queued (batch)"));
}

/** Merge Flash log with RAM buffer, cap at LOG_ENTRIES_MAX, save to Flash, clear RAM. LOG3 only; LOG2 (legacy) treated as empty. */
static void mergeBackupAndPrepareSend(void) {
    SERIAL_PRINTLN(F("[merge] mergeBackupAndPrepareSend entry"));
    logStore.read(&persistentLog);
    uint8_t flashCount = (persistentLog.magic == LOG_FLASH_MAGIC_V3 && persistentLog.count <= LOG_ENTRIES_MAX)
        ? persistentLog.count : 0;
    SERIAL_PRINT(F("[merge] logStore.read flashCount="));
    SERIAL_PRINT(flashCount);
    SERIAL_PRINT(F(" ramCount="));
    SERIAL_PRINTLN(ramCount);
    uint8_t total = flashCount + ramCount;
    if (total > LOG_ENTRIES_MAX) total = LOG_ENTRIES_MAX;
    /* Build merged list newest-first: RAM (already newest-first) then flash (older). Decoder expects entry i = anchor - i*interval. */
    AppLogEntry merged[LOG_ENTRIES_MAX];
    uint8_t i = 0;
    for (uint8_t k = 0; k < ramCount && i < total; k++) merged[i++] = dataBuffer[k];
    for (uint8_t k = 0; k < flashCount && i < total; k++) merged[i++] = persistentLog.entries[k];
    persistentLog.magic = LOG_FLASH_MAGIC_V3;
    persistentLog.count = i;
    for (uint8_t k = 0; k < i; k++) persistentLog.entries[k] = merged[k];
    SERIAL_PRINT(F("[merge] logStore.write count="));
    SERIAL_PRINTLN(i);
    logStore.write(persistentLog);
    ramCount = 0;
}

/** Capture wake start for delta-sleep. Set at very start of do_wake. */
static uint32_t wakeStartMs;

/** Called after each wake (from setup or after do_sleep). Measure, append to RAM (newest at index 0), then either sleep or backup+send. */
void do_wake(osjob_t* j) {
    (void)j;
    wakeStartMs = millis();
    SERIAL_PRINTLN(F("[wake] Standby exit @ millis="));
    SERIAL_PRINTLN(millis());
#if RUN_MODE == RUN_MODE_DEV
    SERIAL_PRINT(F("[DEV] Wake @ millis="));
    SERIAL_PRINTLN(millis());
#endif
    SERIAL_PRINTLN(F("[wake] do_wake entry"));
    float vbatV = VBAT_VOLTS();
    setWakeDeadline(!haveStoredSession, vbatV);  /* Zero-Trust awake budget for this wake cycle */
    if (haveStoredSession) consecutiveWatchdogTriggers = 0;
    if (!haveStoredSession) {
        SERIAL_PRINTLN(F("[wake] No session, join-first: skip sensor, do_send"));
        do_send(&sendjob);
        return;
    }
#if RUN_MODE == RUN_MODE_TEST
    /* Battery test: skip DS18B20 read to save current. */
    float tempC = (float)NAN;
    (void)tempC;
#else
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    uint32_t startMs = millis();
    while (!sensors.isConversionComplete() && (millis() - startMs < (uint32_t)DS18B20_READ_TIMEOUT_MS))
        delay(50);
    float tempC = (millis() - startMs < (uint32_t)DS18B20_READ_TIMEOUT_MS) ? sensors.getTempCByIndex(0) : (float)NAN;
#endif
    SERIAL_PRINT(F("[wake] temp="));
    SERIAL_PRINT(tempC);
    AppLogEntry e;
#if RUN_MODE == RUN_MODE_TEST
    e.temperature = 0xFFFFu;  /* sentinel: no sensor read in battery test */
#else
    e.temperature = encodeTemperatureHighRes(tempC);
#endif

    /* Recent-first: new reading at index 0; shift existing right so decoder sees newest first (anchor - i*interval). */
    if (ramCount < LOG_ENTRIES_MAX) {
        memmove(dataBuffer + 1, dataBuffer, ramCount * sizeof(AppLogEntry));
        dataBuffer[0] = e;
        ramCount++;
    } else {
        memmove(dataBuffer + 1, dataBuffer, (LOG_ENTRIES_MAX - 1) * sizeof(AppLogEntry));
        dataBuffer[0] = e;
    }

    uint8_t period = persistentData.measuresInPeriod % SEND_PERIOD_MEASURES;
    period = (period + 1) % SEND_PERIOD_MEASURES;
    persistentData.measuresInPeriod = period;
    persistentData.wakeCounter++;
    SERIAL_PRINT(F("[wake] period="));
    SERIAL_PRINT(period);
    SERIAL_PRINT(F(" haveStoredSession="));
    SERIAL_PRINTLN(haveStoredSession ? 1 : 0);
    /* Reach here only with haveStoredSession (join-first returns earlier). */
    persistentData.magic = PERSIST_MAGIC_V2;
    persistentData.devEuiTag = (uint64_t)LORAWAN_DEV_EUI;
    /* Lazy write: no session persist in do_wake; commit in do_sleep (when seqnoUp guard passes), EV_TXCOMPLETE, or after join. */

    SERIAL_PRINT(F("Wake #"));
    SERIAL_PRINT(persistentData.wakeCounter);
    SERIAL_PRINT(F(" Temp="));
    SERIAL_PRINTLN(tempC);

    if (vbatV < VBAT_CRITICAL_THRESHOLD_V) {
        SERIAL_PRINTLN(F("[wake] Critical VBat, skip send, 600 s sleep"));
        do_sleep(&sendjob);
    } else if (period == 0) {
        SERIAL_PRINTLN(F("[wake] period==0 -> mergeBackupAndPrepareSend then do_send"));
        /* SEND_PERIOD_MEASURES-th wake: backup RAM to Flash (merge with any existing), then send. */
        mergeBackupAndPrepareSend();
        do_send(&sendjob);
    } else {
        SERIAL_PRINTLN(F("[wake] period!=0 -> do_sleep"));
        do_sleep(&sendjob);
    }
}

// STANDBY sleep via Arduino Low Power. Delta sleep keeps period accurate (interval - awake). do_sleep does not wait for radio idle (proactive hand-off).
// Detach USB before deep sleep so sleep current is ~5–15 µA; otherwise Feather M0 USB draws ~10 mA. Default: 1 in PROD, 0 in DEV/TEST (Serial debugging).
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

/* do_sleep: proactive hand-off; no wait for radio idle. Session persisted here only when seqnoUp increased (see guard below); otherwise EV_TXCOMPLETE or watchdog path persists. */
void do_sleep(osjob_t* j) {
    (void)j;
    SERIAL_PRINTLN(F("[sleep] do_sleep entry"));
    bool skipPersist = false;
    if (skipPersistDueToMacStorm) {
        skipPersist = true;
        skipPersistDueToMacStorm = false;
    }
    SERIAL_PRINTLN(F("[sleep] Standby start @ millis="));
    SERIAL_PRINTLN(millis());
    SERIAL_PRINT(F("[sleep] haveStoredSession="));
    SERIAL_PRINTLN(haveStoredSession ? 1 : 0);
    /* Persist only when seqnoUp increased (avoids redundant write after EV_TXCOMPLETE; avoids overwriting with 0 after watchdog LMIC_reset). */
    if (persistentData.measuresInPeriod == 0u && haveStoredSession && !skipPersist
        && (uint32_t)LMIC.seqnoUp != persistentData.seqnoUp && (uint32_t)LMIC.seqnoUp > persistentData.seqnoUp) {
        save_session_to_flash();  /* includes delay(20) */
    } else {
        delay(10);  /* allow any prior NVM write to complete before deep sleep (SAMD21) */
    }

    float vbatV = VBAT_VOLTS();
    /* Critical: 600 s; low: double interval; hibernation: 1 h. Normal: delta sleep to keep period accurate (interval - awake). */
    uint32_t intervalMs = (RUN_MODE == RUN_MODE_PROD) ? (INTERVAL_SEC_PROD * 1000UL) : (INTERVAL_SEC_DEV_TEST * 1000UL);
    uint32_t effectiveSleepMs;
    if (hibernationRequested) {
        effectiveSleepMs = 3600000UL;  /* 1 h */
        hibernationRequested = false;
        consecutiveWatchdogTriggers = 0;
    } else if (vbatV < VBAT_CRITICAL_THRESHOLD_V) {
        effectiveSleepMs = 600000UL;  /* 10 min */
    } else if (vbatV < VBAT_LOW_THRESHOLD_V) {
        effectiveSleepMs = (uint32_t)SLEEP_SECONDS * 2000UL;
    } else {
        uint32_t awakeDurationMs = (uint32_t)(millis() - wakeStartMs);
        if (awakeDurationMs < intervalMs)
            effectiveSleepMs = intervalMs - awakeDurationMs;
        else
            effectiveSleepMs = 1000UL;  /* floor 1 s if awake exceeded interval */
    }

#if RUN_MODE == RUN_MODE_DEV
    SERIAL_PRINT(F("[DEV] Entering sleep wait — "));
    SERIAL_PRINT(effectiveSleepMs / 1000UL);
    SERIAL_PRINT(F(" s @ millis="));
    SERIAL_PRINTLN(millis());
    /* DEV: wait only, no radio/LMIC (simulates deep sleep); CPU stays awake so Serial remains connected. */
    {
        uint32_t start = millis();
        static uint32_t lastHeartbeatMs = 0;
        while (millis() - start < effectiveSleepMs) {
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
    /* Radio DIO pins (3, 6): LMIC remains idle after reset/sleep; no explicit detach before deepSleep. */
#if DETACH_USB_BEFORE_SLEEP
    USBDevice.detach();
#endif
    LowPower.deepSleep(effectiveSleepMs);
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
#if defined(__SAMD21__)
    /* BOD33 at 2.7V for clean shutdown during LoRa TX surge (120 mA); avoids flash corruption. */
    {
        #define SYSCTRL_BOD33_REG   (*(volatile uint32_t*)(0x40000810u))
        #define SYSCTRL_PCLKSR_REG  (*(volatile uint32_t*)(0x40000808u))
        #define PCLKSR_B33SRDY      (1u << 2)
        SYSCTRL_BOD33_REG = 0;
        while ((SYSCTRL_PCLKSR_REG & PCLKSR_B33SRDY) == 0) {}
        SYSCTRL_BOD33_REG = (32u << 16) | (2u << 3) | 1u;  /* LEVEL 32 ~2.7V, ACTION reset, ENABLE */
    }
#endif
    SERIAL_PRINTLN(F("Starting"));
    SERIAL_PRINT(F("Starting VBat="));
    SERIAL_PRINTLN(VBAT_VOLTS());
    lastTxFPort = 0;
    coldBootThisRun = true;  /* First send after boot sets payload flags bit1 (cold boot); cleared after first batch TX. */
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
    if (haveStoredSession && persistentData.devaddr == 0u) {
        haveStoredSession = false;  /* Invalid session (DevAddr 0); force new join. */
    }
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

    SERIAL_PRINTLN(F("[setup] os_init LMIC_reset LMIC_setClockError"));
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);  /* 20% for Feather M0 internal oscillator / RX2 window */
    APP_DISABLE_LMIC_DUTY_CYCLE();  /* Application enforces 300 s sleep; LMIC duty would cause long awake-waits */

    if (haveStoredSession) {
        SERIAL_PRINTLN(F("[setup] LMIC_setSession (restore from flash)"));
        LMIC_setSession(persistentData.netid, (u4_t)persistentData.devaddr,
                        persistentData.nwkKey, persistentData.artKey);
        LMIC.seqnoUp = (u4_t)persistentData.seqnoUp;
        LMIC_setAdrMode(0);
        LMIC_setDrTxpow(5, 14);  /* SF7 in all run modes */
        SERIAL_PRINT(F("Session restored from flash DevAddr=0x"));
        SERIAL_PRINT(persistentData.devaddr, HEX);
        SERIAL_PRINT(F(" SeqNo="));
        SERIAL_PRINTLN(persistentData.seqnoUp);
    } else {
        SERIAL_PRINTLN(F("[setup] no session, will OTAA join"));
    }

    SERIAL_PRINTLN(F("[setup] pinMode do_wake"));
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);   /* DEV: blink on send; TEST/PROD: off */
    /* Join-first power isolation: sensors.begin() only after EV_JOINED or when restoring session so data path has sensor. */
    if (haveStoredSession) {
#if RUN_MODE != RUN_MODE_TEST
        sensors.begin();
#endif
    }
    ramCount = 0;
    setWakeDeadline(!haveStoredSession, VBAT_VOLTS());
    do_wake(&sendjob);
}

void loop() {
    os_runloop_once();
    /* Rollover-safe: signed delta for 30–90 s windows. */
    if (wakeDeadlineMs != 0 && (int32_t)(millis() - wakeDeadlineMs) >= 0) {
        watchdogTriggeredLastWake = true;
        if (!haveStoredSession) {
            consecutiveWatchdogTriggers++;
            if (consecutiveWatchdogTriggers >= 3) {
                hibernationRequested = true;
                SERIAL_PRINTLN(F("[watchdog] CRITICAL FAILURE: Entering 1-hour hibernation"));
            }
        }
        SERIAL_PRINTLN(F("[watchdog] AWAKE LIMIT REACHED. Forcing sleep."));
#if RUN_MODE == RUN_MODE_DEV
        Serial.flush();  /* DEV only; PROD has Serial compiled out. */
#endif
        if (haveStoredSession && (uint32_t)LMIC.seqnoUp != persistentData.seqnoUp)
            save_session_to_flash();
        consecutivePort0Downlinks = 0;
        LMIC_reset();
        LMIC.opmode = 0;  /* Clear opmode so next wake starts clean. */
        LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);  /* 20% for Feather M0 / RX2 window */
        APP_DISABLE_LMIC_DUTY_CYCLE();
#if RUN_MODE == RUN_MODE_DEV
        if (hibernationRequested) {
            Serial.flush();  /* DEV only; PROD has Serial compiled out. */
            delay(100);
        }
#endif
        os_setTimedCallback(&sendjob, os_getTime(), do_sleep);
        wakeDeadlineMs = millis() + 60000;  /* prevent re-trigger until do_sleep runs */
    }
}
