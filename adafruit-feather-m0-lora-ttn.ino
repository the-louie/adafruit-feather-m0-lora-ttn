/* Log entry type: defined before any #include so no library (e.g. LMIC) macro can shadow AppLogEntry. v3.1: 2-byte entry; time from gateway received_at − entry index × interval (FPort). */
struct LogEntryS {
    uint16_t temperature;
} __attribute__((packed));
typedef struct LogEntryS AppLogEntry;
#define LOG_ENTRIES_MAX 48
#define LOG_SEND_CAP    43

#include <lmic.h>
/* No-op when LMIC does not provide LMIC_disableDutyCycle. To enable duty-cycle bypass, use a library that provides it and replace with LMIC_disableDutyCycle(). */
#define APP_DISABLE_LMIC_DUTY_CYCLE() ((void)0)

/* Firmware v3.5 – Hardware-strapped run mode (pins 11/12 GND-strap). Single binary; 0 µA pin leakage after read; reset-cause filtering (PM->RCAUSE). v3.3.2: Join-first uplink includes one reading; direct-from-RAM when SEND_PERIOD_MEASURES==1 and no backlog. v3.4: Cold boot always uplink with data; if no flash backlog, one temperature measurement is taken and sent; first wake after cold boot may take longer. v3.5: USB cold-boot override – when powered from USB at cold boot, run mode is forced to DEV for that boot without writing to Flash (Live Boot; reverts to straps on battery). */

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
 * Batched log (c) 2026 by the_louie, v3.1 Clean-Join:
 *  - On each wake: measure, append to RAM (newest at index 0); on send wake merge RAM to Flash, uplink on FPort 10 (300 s) or 20 (10800 s); on success clear Flash. After join, only this batch is sent (no post-join HELLO).
 *  - Payload: 4-byte header [vbat×100][flags][sequence] then 2×n temperature. Flags bit0 = watchdog, bit1 = cold boot. Time from gateway received_at; decoder uses FPort for interval.
 *  - Session saved on join, restored on wake.
 *
 * TTN / LMIC behaviour: Duty cycle disabled; 5-min sleep cycle enforces EU868. Zero-Trust watchdog: if awake exceeds budget (30 s data / 90 s join / 15 s low VBat), force LMIC_reset and do_sleep; optional Watchdog Bit in payload (flags bit0) for TTN visibility.
 * App watchdog: 10s after EV_TXSTART (when haveStoredSession; not during join) or 12s after EV_JOIN_TXCOMPLETE (join), if completion never fires, force do_sleep. MAC reset retry: 4s fallback if EV_TXSTART not seen. do_sleep: persist only when lastTxFPort>0 or watchdog (bypass for pure MAC).
 * SF7 default, ADR off. After 3 consecutive EV_LINK_DEAD the device uses SF8 for one send cycle then reverts to SF7. All uplinks Unconfirmed (no ACK/retries).
 * 20% clock error after every LMIC_reset (Feather M0 oscillator / RX2 window). SeqNo persisted in EV_TXCOMPLETE only when completed TX was application data (FPort > 0); MAC-only completions skip flash. do_sleep does not wait for radio idle (proactive hand-off); session saved when send cycle complete. v3.2.1: App watchdog 10s when haveStoredSession (any radio TX); not armed during join. Stay-awake until haveStoredSession && validationCycles>=2; do_send does not force sleep on OP_TXRXPEND when !haveStoredSession. MAC throttle: >3 consecutive Port 0 downlinks trigger LMIC reset and duty-cycle disable. Link Check on after join. EV_LINK_DEAD triggers LMIC_reset and re-join. do_send allows at most one 10 s reschedule when OP_TXRXPEND (with session); on second blocked attempt forces sleep. RX2 from Join Accept.
 *
 * If updates stop: (1) TTN Live Data: "Join Request without Join Accept" = timing/clock;
 * "Uplink dropped" = frame counter. (2) Confirm DIO1 (RFM95) to pin 6 (M0); without it no RX.
 * (3) Device very close to gateway (under 3 m) can desensitize; try another room. (4) SPI
 * shared (Flash + LoRa). (5) Brown-out during TX (120 mA) can corrupt RFM95; supply decoupling.
 * Runbook: DETACH_USB_BEFORE_SLEEP (PROD default) for ~5–15 µA sleep; DS18B20 4.7 kΩ pull-up; sensor read timeout 2 s; Starting logs VBat; below 3.4 V sleep doubled, below 3.2 V critical (skip TX, 600 s sleep). BOD33 2.7V. Ensure 1000µF capacitor on supply to prevent false resets during SF7 TX spike. Payload 4+2×43 bytes max (v3.1). SF bump: 3× link dead → SF8 one cycle. Cold Boot test: pull battery, reattach; verify device joins at SF7 and resumes correct seqnoUp from flash (no drift); after 3× EV_LINK_DEAD next cycle may use SF8.
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
#include "hw-device-config.h"
/* No explicit #include <USB/USBDevice.h>; on SAMD21 the USBDevice object is provided by the core (Arduino.h). */
#if defined(USBCON)
#define HAVE_USB_DEVICE 1
#else
#define HAVE_USB_DEVICE 0
#endif

#define VBATPIN A7
#define VBAT_VOLTS() (analogRead(VBATPIN) * (2.0f * 3.3f / 1024.0f))
#define VBAT_LOW_THRESHOLD_V 3.4f  /* Below this we double sleep interval to preserve capacity. */
#define VBAT_CRITICAL_THRESHOLD_V 3.2f  /* Below this skip TX (brown-out risk), sleep 600 s. */
#define DS18B20_READ_TIMEOUT_MS 2000  /* Max wait for conversion; on timeout use sentinel (0xFFFF). */

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/* 20% clock error for Feather M0 internal oscillator: Join Accept and MAC downlinks (e.g. ADR, LinkCheckAns) need wider RX window. */

/* Run mode from hardware strapping (pins 11/12) at cold boot; DEV/TEST = 5 min FPort 10, PROD = 3 h FPort 20. Senior-dev convention: PROD=1, DEV=2, TEST=3. */
#define RUN_MODE_PROD 1
#define RUN_MODE_DEV  2
#define RUN_MODE_TEST 3

#define FPORT_INTERVAL_5MIN  10   /* 300 s interval; decoder uses FPort to infer interval. */
#define FPORT_INTERVAL_3H    20   /* 10800 s (3 h) interval. */
#define INTERVAL_SEC_DEV_TEST  300u
#define INTERVAL_SEC_PROD      10800u  /* TTN decoder must use same FPort→interval: FPort 10 = 300 s, FPort 20 = 10800 s; change ttn-decoder-batch.js if PROD interval changes. */

#define SEND_PERIOD_MEASURES  1u  /* Same in all modes; interval comes from configureIntervals(). */

/* EU868 data rate indices: DR5=SF7 (default), DR4=SF8 (one cycle after 3× EV_LINK_DEAD). */
#define SF7_DR_EU868 5
#define SF8_DR_EU868 4

/* Application watchdog: if EV_TXCOMPLETE never fires (e.g. MAC/stack absorbs event), force do_sleep. */
#define APP_WATCHDOG_AFTER_TX_SEC 10u  /* Data/MAC TX: 10s after EV_TXSTART. Covers standard RX1 (1s/5s) and RX2 (2s/6s) windows with margin. */
#define APP_WATCHDOG_JOIN_SEC 12u      /* Join: 12s after EV_JOIN_TXCOMPLETE (RX1+RX2+Feather M0 processing margin) */

/* Serial only when runMode == RUN_MODE_DEV (runtime); Serial.begin() called only in setup when DEV. Single binary. */
#define SERIAL_PRINT(...)    do { if (persistentData.runMode == RUN_MODE_DEV) Serial.print(__VA_ARGS__); } while(0)
#define SERIAL_PRINTLN(...)  do { if (persistentData.runMode == RUN_MODE_DEV) Serial.println(__VA_ARGS__); } while(0)
#define SERIAL_BEGIN(...)    do { if (persistentData.runMode == RUN_MODE_DEV) Serial.begin(__VA_ARGS__); } while(0)
static void blinkLed(void) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
}

static AppLogEntry dataBuffer[LOG_ENTRIES_MAX];
static uint8_t ramCount;  /* number of valid entries in dataBuffer */

/* Session store first so it gets the first flash slot; logStore second. Avoids logStore.write() erase
 * wiping the same flash page as the session on SAMD21 (cmaglie FlashStorage erases by slot). */
#define PERSIST_MAGIC   0x4C4D4943u  /* "LMIC" - legacy; not used for session restore (V2 only). */
#define PERSIST_MAGIC_V2 0x4C4D4944u  /* V2: session restore only when magic==V2 and devaddr!=0. */
/* LORAWAN_DEV_EUI for session tag is in app-device-config.h; must match DEVEUI below. */
/** Version marker; mismatch in setup() forces validationCycles=0 (first boot after flash or firmware upgrade). */
#define FIRMWARE_VERSION_V350 0x00030500u

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
    uint8_t validationCycles;   /* Number of completed application TX cycles after join; deep sleep allowed when >= 2. */
    uint8_t runMode;            /* Runtime mode from hardware strapping (pins 11/12); packed with validationCycles. */
    uint8_t reserved_padding[2]; /* Explicit padding for 4-byte alignment of lastFirmwareVersion. */
    uint32_t lastFirmwareVersion;  /* FIRMWARE_VERSION_V350 when saved; mismatch forces validation period. */
} PersistentData_t;

PersistentData_t persistentData;
FlashStorage(persistentStore, PersistentData_t);

#define LOG_FLASH_MAGIC 0x4C4F4732  /* "LOG2" - legacy format; v3.1 uses LOG3 only for entries. */
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
/** MAC storm fallback job; LMIC os_setTimedCallback supports multiple concurrent osjob_t. */
static osjob_t macRetryFallbackJob;
/** Set when MAC storm reset schedules do_send; cleared when EV_TXSTART fires or do_send queues. mac_retry_fallback at 4s forces do_sleep if still set. */
static bool macStormRetryPending = false;
static bool haveStoredSession;

/** OP_TXRXPEND: at most one 10 s reschedule of do_send when session exists; on second blocked attempt we force sleep. v3.2: When !haveStoredSession (commissioning) we reschedule in 10 s without forcing sleep; 90 s Join Watchdog unchanged. */
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

/** FPort of last application TX (batch=10/20). Set in do_send before LMIC_setTxData2; do not clear in EV_JOINED so first post-join TX completion triggers persist and do_sleep. MAC-only (Port 0) completions do not set this; we only persist to flash when lastTxFPort > 0. */
static uint8_t lastTxFPort;
/** Set true at boot; cleared after first successful batch TX. Flags bit 1 in payload (cold boot/reset). */
static bool coldBootThisRun = true;

/* Runtime interval/FPort/USB from configureIntervals(persistentData.runMode); set in setup(). */
static uint32_t currentMeasureInterval = 10800u;   /* seconds; PROD default until configureIntervals runs */
static uint32_t currentSleepMs = 10800000UL;      /* PROD default */
static uint8_t currentFPort = FPORT_INTERVAL_3H;  /* PROD default */
static bool detachUsbBeforeSleep = true;           /* Non-DEV: detach before deepSleep; set in configureIntervals (USB active only in DEV). */
/** Run mode to persist to Flash (strap/version value). Set in setup() before USB override; used in save_session_to_flash() so USB-forced DEV is never written. */
static uint8_t runModeToPersist = RUN_MODE_PROD;
/** True when run mode was forced to DEV this boot because USB was detected (Live Boot). Logged to Serial in setup(). */
static bool usbForcedDevMode = false;

void do_wake(osjob_t* j);  /* measure, append, then backup+send or sleep */
void do_send(osjob_t* j);
void do_sleep(osjob_t* j);
static void app_watchdog_sleep(osjob_t* j);   /* 10s after EV_TXSTART (when haveStoredSession) if EV_TXCOMPLETE never fired */
static void join_watchdog_sleep(osjob_t* j);  /* 12s after EV_JOIN_TXCOMPLETE if EV_JOINED/EV_JOIN_FAILED never fired */
static void mac_retry_fallback(osjob_t* j);   /* 4s after MAC storm reset if EV_TXSTART never fired */

/** Commit session to flash. updateSeqnoFromLmic: when false, do not overwrite seqnoUp from LMIC (use after watchdog so LMIC_reset does not corrupt stored seqnoUp). Call sites: EV_JOINED; EV_TXCOMPLETE when last TX was application; do_sleep when measuresInPeriod==0 && lastTxFPort>0 && seqnoUp increased or when watchdogTriggeredLastWake; app_watchdog_sleep (false). */
static void save_session_to_flash(bool updateSeqnoFromLmic = true) {
    if (!haveStoredSession) return;
    persistentData.magic = PERSIST_MAGIC_V2;
    persistentData.devEuiTag = (uint64_t)LORAWAN_DEV_EUI;
    if (updateSeqnoFromLmic) persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
    SERIAL_PRINTLN(F("[persist] Committing to Flash..."));
    uint8_t r = persistentData.runMode;
    persistentData.runMode = runModeToPersist;  /* Never persist USB-forced DEV to Flash. */
    persistentStore.write(persistentData);
    persistentData.runMode = r;
    delay(20);
}

/* Pin mapping: Feather M0 LoRa. DIO1 (RFM95) must be jumped to Pin 6 for RX and LMIC events. */
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
};

/** SAMD21 RCAUSE: mandatory fallback if toolchain does not define. POR = Bit 1 (0x02), EXT = Bit 4 (0x10). */
#ifndef PM_RCAUSE_POR
#define PM_RCAUSE_POR 0x02
#endif
#ifndef PM_RCAUSE_EXT
#define PM_RCAUSE_EXT 0x10
#endif

/** True if MCU started due to power-on (POR) or external reset (RST button). */
static bool isHardwareReset(void) {
    uint8_t cause = PM->RCAUSE.reg;
    return (cause & PM_RCAUSE_POR) || (cause & PM_RCAUSE_EXT);
}

/** Read pins 11/12 with pull-up, then set INPUT (High-Z) for 0 µA. Call only when version mismatch or isHardwareReset(). */
static void updateHardwareRunMode(void) {
    pinMode(STRAP_DEV_PIN, INPUT_PULLUP);
    pinMode(STRAP_TEST_PIN, INPUT_PULLUP);
    delay(10);
    if (digitalRead(STRAP_DEV_PIN) == LOW)
        persistentData.runMode = RUN_MODE_DEV;
    else if (digitalRead(STRAP_TEST_PIN) == LOW)
        persistentData.runMode = RUN_MODE_TEST;
    else
        persistentData.runMode = RUN_MODE_PROD;
    pinMode(STRAP_DEV_PIN, INPUT);
    pinMode(STRAP_TEST_PIN, INPUT);
}

/** True if VBUS is present (USB cable connected). Used for cold-boot power-source identification only.
 *  Enables the USB APB clock and waits for synchronization before reading.
 *  Does not manage runtime USB transitions. */
bool isUsbCableDetected(void) {
#if defined(__SAMD21__) && defined(USBCON)
    // 1. Enable the APB clock for the USB peripheral if not already active
    if (!(PM->APBBMASK.reg & PM_APBBMASK_USB)) {
        PM->APBBMASK.reg |= PM_APBBMASK_USB;
        // 2. Latency: Wait for the clock to stabilize before register access
        delayMicroseconds(10);
    }
    // 3. Now safe to read the Finite State Machine status
    // Any state != 0 (ON, SUSPEND, SLEEP) indicates 5V on VBUS
    return (USB->DEVICE.FSMSTATUS.bit.FSMSTATE != 0);
#endif
    return false;
}

/** Set runtime interval, FPort, and USB detach from persisted runMode. Call once in setup() after runMode is stable. USB only active in DEV (strapping). */
static void configureIntervals(uint8_t mode) {
    if (mode == RUN_MODE_DEV || mode == RUN_MODE_TEST) {
        currentMeasureInterval = 300u;
        currentSleepMs = 300000UL;
        currentFPort = FPORT_INTERVAL_5MIN;
        detachUsbBeforeSleep = (mode != RUN_MODE_DEV);  /* USB active only in DEV */
    } else {
        currentMeasureInterval = 10800u;
        currentSleepMs = 10800000UL;
        currentFPort = FPORT_INTERVAL_3H;
        detachUsbBeforeSleep = true;
    }
}

/** MAC storm fallback: if do_send at 2s did not trigger EV_TXSTART, force sleep to avoid 90s hang. */
static void mac_retry_fallback(osjob_t* j) {
    (void)j;
    if (!macStormRetryPending) return;
    macStormRetryPending = false;
    SERIAL_PRINTLN(F("[MAC] Retry fallback: EV_TXSTART not seen, forcing sleep"));
    os_setTimedCallback(&sendjob, os_getTime(), do_sleep);
}

/** App-watchdog path: EV_TXCOMPLETE was ghosted. Radio purge (LMIC_reset) prevents OP_TXRXPEND lockup across wake; restore session so device stays joined. Next wake re-sends batch. */
static void app_watchdog_sleep(osjob_t* j) {
    (void)j;
    SERIAL_PRINTLN(F("[watchdog] App watchdog: TX ghosted, purging radio and restoring session"));
    if (haveStoredSession)
        save_session_to_flash(false);
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);  /* 20% for Feather M0 / RX2 window */
    APP_DISABLE_LMIC_DUTY_CYCLE();
    if (haveStoredSession) {
        LMIC_setSession(persistentData.netid, (u4_t)persistentData.devaddr, persistentData.nwkKey, persistentData.artKey);
        LMIC.seqnoUp = (u4_t)persistentData.seqnoUp;
        LMIC_setAdrMode(0);
        LMIC_setDrTxpow(5, 14);  /* SF7 */
    }
    watchdogTriggeredLastWake = true;  /* do_sleep will persist and set payload flag on next TX */
    os_setTimedCallback(&sendjob, os_getTime(), do_sleep);
}

/** Join-phase watchdog: EV_JOINED/EV_JOIN_FAILED never arrived. Do NOT sleep; reset MAC and retry immediately. */
static void join_watchdog_sleep(osjob_t* j) {
    (void)j;
    SERIAL_PRINTLN(F("[join] Join Accept missed (watchdog), retrying immediately..."));
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);
    APP_DISABLE_LMIC_DUTY_CYCLE();
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(5), do_send);
}

/** Classify downlink: dataLen==0 = Port 0 (MAC-only); dataLen>0 then FPort from LMIC.frame[LMIC.dataBeg-1]. Update consecutivePort0Downlinks; if > threshold, LMIC_reset + disableDutyCycle + session restore + schedule do_send, return true.
 * appTxFPort: FPort of completed TX when called from EV_TXCOMPLETE; 0 when from EV_RXCOMPLETE. Used to clear log when app batch was sent before reset. */
static bool updatePort0CounterAndMaybeReset(uint8_t appTxFPort) {
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
    /* Reset path schedules do_send in 2s; that send completes via EV_TXCOMPLETE or app watchdog. Timeout fallback: mac_retry_fallback at 4s forces do_sleep if EV_TXSTART never fires. */
    SERIAL_PRINTLN(F("[MAC] >3 consecutive Port 0 downlinks, reset to break storm"));
    consecutivePort0Downlinks = 0;
    /* Persist current seqnoUp before LMIC_reset; otherwise restore would use stale value and NS rejects next uplink. */
    if (haveStoredSession) {
        persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
        save_session_to_flash();
    }
    if (appTxFPort == FPORT_INTERVAL_5MIN || appTxFPort == FPORT_INTERVAL_3H) {
        persistentLog.magic = 0;
        persistentLog.count = 0;
        logStore.write(persistentLog);
    }
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
    macStormRetryPending = true;
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
    os_setTimedCallback(&macRetryFallbackJob, os_getTime() + sec2osticks(4), mac_retry_fallback);
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
            /* Do not set lastTxFPort = 0: do_send already set it for the queued batch; EV_TXCOMPLETE must see it to persist and clear log. */
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
            /* v3.1: Join-first power isolation. Sensors only after EV_JOINED; no second uplink – FPort 10/20 batch from do_send is the only TX. */
            if (persistentData.runMode != RUN_MODE_TEST) sensors.begin();
            /* Post-join: EV_TXSTART (app watchdog) or EV_TXCOMPLETE will schedule next step; no 90s backstop. */
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
        case EV_REJOIN_FAILED: {
            SERIAL_PRINTLN(F("EV_REJOIN_FAILED"));
            /* Schedule retry so device does not hang; same backoff as EV_JOIN_FAILED. */
            if (consecutiveJoinFailCount < 255u) consecutiveJoinFailCount++;
            if (consecutiveJoinFailCount > 3u) consecutiveJoinFailCount = 3u;
            uint32_t backoffSec = (consecutiveJoinFailCount == 1u) ? 600u : (consecutiveJoinFailCount == 2u) ? 1800u : 3600u;
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(backoffSec), do_send);
            break;
        }
        case EV_TXCOMPLETE:
            /* Start DS18B20 conversion so it runs during the 2 s pre-sleep delay (overlap). */
            if (persistentData.runMode != RUN_MODE_TEST) {
                sensors.setWaitForConversion(false);
                sensors.requestTemperatures();
            }
            if (persistentData.runMode == RUN_MODE_DEV) blinkLed();   /* send succeeded */
            SERIAL_PRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) SERIAL_PRINTLN(F("Received ack"));
            if (LMIC.dataLen) {
                SERIAL_PRINT(F("[downlink] received len="));
                SERIAL_PRINTLN(LMIC.dataLen);
            }
            if ((LMIC.txrxFlags & TXRX_ACK) || (LMIC.dataLen > 0)) {
                if (updatePort0CounterAndMaybeReset(lastTxFPort))
                    break;  /* Schedules do_send at 2s + mac_retry_fallback at 4s; no do_sleep here */
            }
            /* Sleep after any radio completion (data or MAC); persist/log only when lastTxFPort > 0. */
            persistentData.magic = PERSIST_MAGIC_V2;
            persistentData.devEuiTag = (uint64_t)LORAWAN_DEV_EUI;
            persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
            if (lastTxFPort > 0u) {
                consecutiveLinkDeadCount = 0;  /* Success breaks consecutive link-dead streak */
                coldBootThisRun = false;  /* first completed application TX clears boot-event bit */
                if (persistentData.validationCycles < 2u) persistentData.validationCycles++;
                save_session_to_flash();
                SERIAL_PRINTLN(F("[persist] App data sent. Committing SeqNo."));
                SERIAL_PRINT(F("[valid] Cycle count: "));
                SERIAL_PRINTLN(persistentData.validationCycles);
                if (lastTxFPort == FPORT_INTERVAL_5MIN || lastTxFPort == FPORT_INTERVAL_3H) {
                    ramCount = 0;  /* Clear RAM batch so next wake does not resend. */
                    /* Only write Flash when we had valid log (avoids wear on RAM-only sends). */
                    if (persistentLog.magic == LOG_FLASH_MAGIC_V3) {
                        SERIAL_PRINTLN(F("[persist] EV_TXCOMPLETE: logStore.write (clear log)"));
                        persistentLog.magic = 0;
                        persistentLog.count = 0;
                        logStore.write(persistentLog);
                    }
                }
                lastTxFPort = 0;
            } else {
                SERIAL_PRINTLN(F("[persist] MAC-only response. Skipping flash."));
            }
            SERIAL_PRINTLN(F("[persist] EV_TXCOMPLETE: schedule do_sleep in 2s"));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_sleep);  /* Overwrites app watchdog if set */
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
            (void)updatePort0CounterAndMaybeReset(0);
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
        case 17: {
            SERIAL_PRINTLN(F("EV_TXSTART (radio TX started)"));
            SERIAL_PRINT(F("EV_TXSTART datarate="));
            SERIAL_PRINTLN(LMIC.datarate);  /* 5 = DR5/SF7; any other value suggests override, investigate */
            macStormRetryPending = false;   /* MAC storm retry path: TX started, fallback not needed */
            /* Skip app-watchdog during join; allow stack to complete handshake naturally. Hardware watchdog (90s) remains as fail-safe. */
            if (haveStoredSession) {
                /* Application watchdog: if EV_TXCOMPLETE never fires, app_watchdog_sleep runs 10s after EV_TXSTART (any radio TX). */
                os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(APP_WATCHDOG_AFTER_TX_SEC), app_watchdog_sleep);
            }
            break;
        }
        case 18:
            SERIAL_PRINTLN(F("EV_TXCANCELED (MCCI/extended)"));
            break;
        case 19:
            SERIAL_PRINTLN(F("EV_RXSTART (opening RX window)"));
            break;
        case 20:
            SERIAL_PRINTLN(F("EV_JOIN_TXCOMPLETE (Join Request sent, waiting Join Accept)"));
            /* Join Watchdog (12s) armed only here. EV_JOINING has no app watchdog; 1–2 s blind spot with only 90s HW watchdog. */
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(APP_WATCHDOG_JOIN_SEC), join_watchdog_sleep);
            break;
        default:
            SERIAL_PRINT(F("Unknown event "));
            SERIAL_PRINTLN((int)ev);
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

/** Build payload from RAM then Flash (newest-first). 4-byte header [VBat_Hi,VBat_Lo][Flags][Sequence] then 2×n temp. Flags: bit0=watchdog, bit1=cold boot. Sequence = wakeCounter & 0xFF. No local AppLogEntry array; Flash entries only when caller passes flashN from magic-valid log. */
static uint16_t buildBatchPayloadTwoSources(uint8_t* buf, uint16_t vbatCentivolt, uint8_t flagsByte, uint8_t sequenceByte,
    const AppLogEntry* ramEntries, uint8_t ramN, const AppLogEntry* flashEntries, uint8_t flashN) {
    uint16_t i = 0;
    buf[i++] = (uint8_t)(vbatCentivolt >> 8);
    buf[i++] = (uint8_t)(vbatCentivolt & 0xFF);
    buf[i++] = flagsByte;
    buf[i++] = sequenceByte;
    uint8_t n = ramN + flashN;
    if (n > LOG_SEND_CAP) n = (uint8_t)LOG_SEND_CAP;
    uint8_t written = 0;
    for (uint8_t k = 0; k < ramN && written < n && (i + 2) <= 256u; k++, written++) {
        buf[i++] = (uint8_t)(ramEntries[k].temperature >> 8);
        buf[i++] = (uint8_t)(ramEntries[k].temperature & 0xFF);
    }
    for (uint8_t k = 0; k < flashN && written < n && (i + 2) <= 256u; k++, written++) {
        buf[i++] = (uint8_t)(flashEntries[k].temperature >> 8);
        buf[i++] = (uint8_t)(flashEntries[k].temperature & 0xFF);
    }
    return i;
}

void do_send(osjob_t* j) {
    (void)j;
    SERIAL_PRINTLN(F("[send] do_send entry"));
    /* Each path below calls os_setTimedCallback for sendjob or leaves the job to be driven by LMIC events; previous timers on sendjob are overwritten by the next schedule. */
    /* Clean state: after app watchdog the radio was purged (LMIC_reset + session restore) so a re-send will not hang on OP_TXRXPEND. Log cleared only in EV_TXCOMPLETE. */
    if (LMIC.opmode & OP_TXRXPEND) {
        /* Commissioning priority: do not force sleep on busy radio if session is not yet established. */
        if (!haveStoredSession) {
            SERIAL_PRINTLN(F("[send] OP_TXRXPEND, reschedule do_send in 10s (commissioning)"));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(10), do_send);
            return;
        }
        do_send_pend_retries++;
        if (do_send_pend_retries <= DO_SEND_PEND_RESCHEDULE_MAX) {
            SERIAL_PRINTLN(F("[send] OP_TXRXPEND, reschedule do_send in 10s"));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(10), do_send);
        } else {
            SERIAL_PRINTLN(F("[send] Radio busy, forcing sleep to save battery"));
            do_send_pend_retries = 0;  /* next wake gets one reschedule again */
            macStormRetryPending = false;  /* Avoid mac_retry_fallback double-sleep when coming from MAC storm path */
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
    /* Merge RAM into Flash so batch includes backlog; normal path already merged in do_wake. May early-return when Flash empty and SEND_PERIOD_MEASURES==1. */
    if (ramCount > 0u) mergeBackupAndPrepareSend();
    /* Flash only used when magic valid (avoids sending uninitialized memory on fresh device). */
    uint8_t flashCount = (persistentLog.magic == LOG_FLASH_MAGIC_V3 && persistentLog.count <= LOG_ENTRIES_MAX)
        ? persistentLog.count : 0;

    /* Defensive: cold boot with no backlog – take one reading so backend sees boot_event with data (same encode path as do_wake). */
    if (coldBootThisRun && ramCount == 0u && flashCount == 0u) {
        float tempC;
        if (persistentData.runMode == RUN_MODE_TEST) {
            tempC = (float)NAN;
            (void)tempC;
        } else {
            sensors.setWaitForConversion(false);
            sensors.requestTemperatures();
            uint32_t startMs = millis();
            while (!sensors.isConversionComplete() && (millis() - startMs < (uint32_t)DS18B20_READ_TIMEOUT_MS))
                delay(50);
            tempC = (millis() - startMs < (uint32_t)DS18B20_READ_TIMEOUT_MS) ? sensors.getTempCByIndex(0) : (float)NAN;
        }
        AppLogEntry e;
        e.temperature = (persistentData.runMode == RUN_MODE_TEST) ? 0xFFFFu : encodeTemperatureHighRes(tempC);
        dataBuffer[0] = e;
        ramCount = 1u;
    }

    uint8_t flagsByte = (watchdogTriggeredLastWake ? 1u : 0u) | (coldBootThisRun ? 2u : 0u);
    if (watchdogTriggeredLastWake) watchdogTriggeredLastWake = false;
    uint8_t sequenceByte = (uint8_t)(persistentData.wakeCounter & 0xFF);
    uint8_t paybuf[4 + 2 * LOG_SEND_CAP];
    uint16_t payLen = buildBatchPayloadTwoSources(paybuf, vbatCentivolt, flagsByte, sequenceByte,
        dataBuffer, ramCount, persistentLog.entries, flashCount);
    uint8_t n = ramCount + flashCount;
    if (n > LOG_SEND_CAP) n = (uint8_t)LOG_SEND_CAP;

    SERIAL_PRINT(F("Sending batch n="));
    SERIAL_PRINT(n);
    SERIAL_PRINT(F(" VBat="));
    SERIAL_PRINTLN((float)vbatCentivolt / 100.0f);
    SERIAL_PRINT(F("[send] LMIC_setTxData2 port="));
    SERIAL_PRINT(currentFPort);
    SERIAL_PRINT(F(" len="));
    SERIAL_PRINTLN(payLen);

    if (persistentData.runMode == RUN_MODE_DEV) blinkLed();   /* attempting to send */
    /* Set lastTxFPort before LMIC_setTxData2 so EV_TXCOMPLETE and app watchdog can identify application TX even if stack delays or merges events. */
    lastTxFPort = currentFPort;
    macStormRetryPending = false;   /* MAC storm retry: packet queued, EV_TXSTART will fire */
    /* All uplinks Unconfirmed (4th arg 0) for battery life; no ACK wait/retries. */
    LMIC_setTxData2(currentFPort, paybuf, (uint8_t)payLen, 0);
    SERIAL_PRINTLN(F("[send] Packet queued (batch)"));
}

/** Merge Flash log with RAM buffer, cap at LOG_ENTRIES_MAX, save to Flash, clear RAM. LOG3 only; LOG2 (legacy) treated as empty. */
static void mergeBackupAndPrepareSend(void) {
    SERIAL_PRINTLN(F("[merge] mergeBackupAndPrepareSend entry"));
    logStore.read(&persistentLog);
    uint8_t flashCount = (persistentLog.magic == LOG_FLASH_MAGIC_V3 && persistentLog.count <= LOG_ENTRIES_MAX)
        ? persistentLog.count : 0;
    /* Skip Flash write when no backlog and send period is 1 to reduce wear; do_send will build from RAM. */
    if (flashCount == 0 && SEND_PERIOD_MEASURES == 1u) {
        SERIAL_PRINTLN(F("[merge] Flash empty, send from RAM to save wear"));
        return;
    }
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

/** Called after each wake (from setup or after do_sleep). Critical VBat check first; then measure, append to RAM (newest at index 0), then either join-first do_send or session period logic. When coldBootThisRun is true and the device has a session, the send path is forced so the first wake always uplinks data. */
void do_wake(osjob_t* j) {
    (void)j;
    wakeStartMs = millis();
    SERIAL_PRINTLN(F("[wake] Standby exit @ millis="));
    SERIAL_PRINTLN(millis());
    if (persistentData.runMode == RUN_MODE_DEV) {
        SERIAL_PRINT(F("[DEV] Wake @ millis="));
        SERIAL_PRINTLN(millis());
    }
    SERIAL_PRINTLN(F("[wake] do_wake entry"));
    float vbatV = VBAT_VOLTS();
    setWakeDeadline(!haveStoredSession, vbatV);  /* Zero-Trust awake budget for this wake cycle */
    if (haveStoredSession) consecutiveWatchdogTriggers = 0;

    /* Critical VBat check before sensor read: avoid ~750 ms and ~1 mA DS18B20 when already in brown-out danger. */
    if (vbatV < VBAT_CRITICAL_THRESHOLD_V) {
        SERIAL_PRINTLN(F("[wake] Critical VBat, skip sensor and send, 600 s sleep"));
        do_sleep(&sendjob);
        return;
    }

    float tempC;
    if (persistentData.runMode == RUN_MODE_TEST) {
        tempC = (float)NAN;  /* Battery test: skip DS18B20 read to save current. */
        (void)tempC;
    } else {
        sensors.setWaitForConversion(false);
        sensors.requestTemperatures();
        uint32_t startMs = millis();
        while (!sensors.isConversionComplete() && (millis() - startMs < (uint32_t)DS18B20_READ_TIMEOUT_MS))
            delay(50);
        tempC = (millis() - startMs < (uint32_t)DS18B20_READ_TIMEOUT_MS) ? sensors.getTempCByIndex(0) : (float)NAN;
    }
    SERIAL_PRINT(F("[wake] temp="));
    SERIAL_PRINT(tempC);
    AppLogEntry e;
    if (persistentData.runMode == RUN_MODE_TEST)
        e.temperature = 0xFFFFu;  /* sentinel: no sensor read in battery test */
    else
        e.temperature = encodeTemperatureHighRes(tempC);

    /* Recent-first: new reading at index 0; shift existing right so decoder sees newest first (anchor - i*interval). */
    if (ramCount > LOG_ENTRIES_MAX) ramCount = LOG_ENTRIES_MAX;  /* defensive cap */
    if (ramCount < LOG_ENTRIES_MAX) {
        memmove(dataBuffer + 1, dataBuffer, ramCount * sizeof(AppLogEntry));
        dataBuffer[0] = e;
        ramCount++;
    } else {
        memmove(dataBuffer + 1, dataBuffer, (LOG_ENTRIES_MAX - 1) * sizeof(AppLogEntry));
        dataBuffer[0] = e;
    }

    /* Join-first path: wakeCounter++ before do_send so Join-Uplink carries unique sequence (avoids Sequence 0 conflicts on reboot). */
    if (!haveStoredSession) {
        persistentData.wakeCounter++;
        SERIAL_PRINTLN(F("[wake] No session, join-first: do_send with one reading"));
        do_send(&sendjob);
        return;
    }

    /* Session path: period logic, then merge+send or sleep. */
    uint8_t period = persistentData.measuresInPeriod % SEND_PERIOD_MEASURES;
    period = (period + 1) % SEND_PERIOD_MEASURES;
    persistentData.measuresInPeriod = period;
    persistentData.wakeCounter++;
    SERIAL_PRINT(F("[wake] period="));
    SERIAL_PRINT(period);
    SERIAL_PRINT(F(" haveStoredSession="));
    SERIAL_PRINTLN(haveStoredSession ? 1 : 0);
    persistentData.magic = PERSIST_MAGIC_V2;
    persistentData.devEuiTag = (uint64_t)LORAWAN_DEV_EUI;
    /* Lazy write: no session persist in do_wake; commit in do_sleep (when seqnoUp guard passes), EV_TXCOMPLETE, or after join. */

    SERIAL_PRINT(F("Wake #"));
    SERIAL_PRINT(persistentData.wakeCounter);
    SERIAL_PRINT(F(" Temp="));
    SERIAL_PRINTLN(tempC);

    /* Cold boot: always uplink data so backend can verify hardware stack; we already have one reading in RAM. */
    if (coldBootThisRun) {
        SERIAL_PRINTLN(F("[wake] Cold boot: merge then do_send"));
        mergeBackupAndPrepareSend();
        do_send(&sendjob);
        return;
    }

    if (period == 0) {
        SERIAL_PRINTLN(F("[wake] period==0 -> mergeBackupAndPrepareSend then do_send"));
        mergeBackupAndPrepareSend();
        do_send(&sendjob);
    } else {
        SERIAL_PRINTLN(F("[wake] period!=0 -> do_sleep"));
        do_sleep(&sendjob);
    }
}

/* do_sleep: proactive hand-off; no wait for radio idle. Session persisted here only when seqnoUp increased (see guard below); otherwise EV_TXCOMPLETE or watchdog path persists. */
void do_sleep(osjob_t* j) {
    (void)j;
    SERIAL_PRINTLN(F("[sleep] do_sleep entry"));
    bool afterWatchdog = watchdogTriggeredLastWake;  /* capture before persist block may clear it */
    bool skipPersist = false;
    if (skipPersistDueToMacStorm) {
        skipPersist = true;
        skipPersistDueToMacStorm = false;
    }
    SERIAL_PRINTLN(F("[sleep] Standby start @ millis="));
    SERIAL_PRINTLN(millis());
    SERIAL_PRINT(F("[sleep] haveStoredSession="));
    SERIAL_PRINTLN(haveStoredSession ? 1 : 0);
    /* After watchdog: persist wakeCounter (and session) without overwriting seqnoUp from LMIC (LMIC_reset zeroed it). */
    bool didPersist = false;
    if (watchdogTriggeredLastWake && haveStoredSession && !skipPersist) {
        save_session_to_flash(false);
        /* Leave watchdogTriggeredLastWake set so next wake's do_send sets payload watchdog bit; cleared in do_send when building flags. */
        didPersist = true;
    }
    /* Persist only when seqnoUp increased AND lastTxFPort>0 (avoids redundant write; avoids overwriting with 0 after watchdog; bypass for pure MAC completion to save Flash wear). */
    if (!didPersist && persistentData.measuresInPeriod == 0u && haveStoredSession && !skipPersist && lastTxFPort > 0u
        && (uint32_t)LMIC.seqnoUp != persistentData.seqnoUp && (uint32_t)LMIC.seqnoUp > persistentData.seqnoUp) {
        save_session_to_flash();
        didPersist = true;
    }
    if (!didPersist)
        delay(10);  /* allow any prior NVM write to complete before deep sleep (SAMD21) */

    float vbatV = VBAT_VOLTS();
    /* Critical: 600 s; low: double interval with delta-sleep; hibernation: 1 h. Normal: delta sleep (interval - awake). Unjoined: short backoff so we retry quickly. */
    uint32_t intervalMs = currentMeasureInterval * 1000UL;
    uint32_t effectiveSleepMs;
    if (!haveStoredSession) {
        effectiveSleepMs = 15000UL;  /* 15 s: rapid join retry; stay-awake delay uses this instead of 3 h */
    } else if (hibernationRequested) {
        effectiveSleepMs = 3600000UL;  /* 1 h */
        hibernationRequested = false;
        consecutiveWatchdogTriggers = 0;
    } else if (vbatV < VBAT_CRITICAL_THRESHOLD_V) {
        effectiveSleepMs = 600000UL;  /* 10 min */
    } else if (vbatV < VBAT_LOW_THRESHOLD_V) {
        uint32_t lowVbatIntervalMs = intervalMs * 2;
        uint32_t awakeDurationMs = (uint32_t)(millis() - wakeStartMs);
        effectiveSleepMs = (awakeDurationMs < lowVbatIntervalMs) ? (lowVbatIntervalMs - awakeDurationMs) : 1000UL;
    } else if (afterWatchdog) {
        effectiveSleepMs = intervalMs;  /* fixed recovery; wakeStartMs is from previous do_wake and is stale */
    } else {
        uint32_t awakeDurationMs = (uint32_t)(millis() - wakeStartMs);
        if (awakeDurationMs < intervalMs)
            effectiveSleepMs = intervalMs - awakeDurationMs;
        else
            effectiveSleepMs = 1000UL;  /* floor 1 s if awake exceeded interval */
    }
    /* watchdogTriggeredLastWake cleared in do_send when building payload flags so next TX carries watchdog bit. */

    /* Stay-Awake Guard: physically prevent entering standard sleep path when unjoined or not yet validated. When !haveStoredSession, effectiveSleepMs is 15 s above so this delay is short. */
    bool stayAwake = !haveStoredSession || (persistentData.validationCycles < 2u);
    if (stayAwake) {
        SERIAL_PRINTLN(F("[valid] Bench Mode Guard: Using delay() instead of deepSleep"));
        uint32_t elapsed = 0u;
        const uint32_t chunkMs = 5000u;
        while (elapsed < effectiveSleepMs) {
            uint32_t step = (effectiveSleepMs - elapsed >= chunkMs) ? chunkMs : (effectiveSleepMs - elapsed);
            delay(step);
            if (persistentData.runMode == RUN_MODE_DEV && Serial) Serial.flush();
            elapsed += step;
        }
        do_wake(&sendjob);
        return;  /* CRITICAL: Exit so standard sleep path (DEV wait / deepSleep) is never reached. */
    }

    /* Standard sleep logic runs only when stayAwake is false (haveStoredSession && validationCycles >= 2). */
    if (persistentData.runMode == RUN_MODE_DEV) {
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
            if (Serial) Serial.flush();
        }
    }
    SERIAL_PRINT(F("[DEV] Sleep wait over — waking @ millis="));
    SERIAL_PRINTLN(millis());
    } else {
    /* Radio DIO pins (3, 6): LMIC remains idle after reset/sleep. Runtime USB detach for PROD. */
#if HAVE_USB_DEVICE
    if (detachUsbBeforeSleep) USBDevice.detach();
#endif
    LowPower.deepSleep(effectiveSleepMs);
#if HAVE_USB_DEVICE
    if (detachUsbBeforeSleep) USBDevice.attach();
#endif
    }
    do_wake(&sendjob);
}


void setup() {
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
    persistentStore.read(&persistentData);
    bool versionMismatch = (persistentData.lastFirmwareVersion != FIRMWARE_VERSION_V350);
    bool hardwareReset = isHardwareReset();
    if (versionMismatch || hardwareReset) {
        if (versionMismatch) {
            persistentData.runMode = RUN_MODE_PROD;
            updateHardwareRunMode();
            persistentData.lastFirmwareVersion = FIRMWARE_VERSION_V350;
            persistentData.validationCycles = 0;
        } else {
            updateHardwareRunMode();
        }
        persistentStore.write(persistentData);
        delay(20);
    }
    runModeToPersist = persistentData.runMode;  /* Strap/version value; never persist USB-forced DEV. */
    /* v3.5 USB cold-boot override: RAM-only, runs on every boot (including watchdog). */
    if (isUsbCableDetected()) {
        persistentData.runMode = RUN_MODE_DEV;
        usbForcedDevMode = true;
    }
    configureIntervals(persistentData.runMode);
#if HAVE_USB_DEVICE
    /* TEST and PROD: never initialize or use USB; detach immediately so Serial is never started. */
    if (persistentData.runMode != RUN_MODE_DEV) USBDevice.detach();
#endif
    if (persistentData.runMode == RUN_MODE_DEV) {
        delay(10000);
        Serial.begin(9600);
        delay(2000);
    }
    SERIAL_PRINTLN(F("Starting"));
    if (usbForcedDevMode) SERIAL_PRINTLN(F("USB detected: running in DEV (Live Boot)"));
    SERIAL_PRINT(F("Starting VBat="));
    SERIAL_PRINTLN(VBAT_VOLTS());
    lastTxFPort = 0;
    coldBootThisRun = true;
    SERIAL_PRINT(F("[setup] sizeof(AppLogEntry)="));
    SERIAL_PRINTLN((unsigned)sizeof(AppLogEntry));
    SERIAL_PRINT(F("[setup] seqnoUp from flash: "));
    SERIAL_PRINTLN(persistentData.seqnoUp);
    SERIAL_PRINT(F("[setup] magic=0x"));
    SERIAL_PRINT(persistentData.magic, HEX);
    SERIAL_PRINT(F(" devaddr=0x"));
    SERIAL_PRINT(persistentData.devaddr, HEX);
    SERIAL_PRINT(F(" seqnoUp="));
    SERIAL_PRINTLN(persistentData.seqnoUp);
    /* Session restore: only V2 magic and non-zero DevAddr. */
    haveStoredSession = (persistentData.magic == PERSIST_MAGIC_V2) && (persistentData.devaddr != 0u);
    SERIAL_PRINT(F("[setup] haveStoredSession="));
    SERIAL_PRINTLN(haveStoredSession ? 1 : 0);
    if (!haveStoredSession) {
        persistentData.wakeCounter = 0;
        persistentData.measuresInPeriod = 0;
    } else {
        persistentData.measuresInPeriod = persistentData.measuresInPeriod % SEND_PERIOD_MEASURES;
    }
    if (!haveStoredSession)
        persistentData.validationCycles = 0;
    else if (persistentData.validationCycles > 2u)
        persistentData.validationCycles = 2;  /* Backward compat: clamp legacy or corrupted flash. */
    SERIAL_PRINTLN(F("[setup] logStore.read"));
    logStore.read(&persistentLog);
    SERIAL_PRINT(F("Wake #"));
    SERIAL_PRINTLN(persistentData.wakeCounter);

    SERIAL_PRINTLN(F("[setup] os_init LMIC_reset LMIC_setClockError"));
    os_init();
    memset(&macRetryFallbackJob, 0, sizeof(macRetryFallbackJob));  /* Init before first MAC reset can occur */
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
    /* Initialize sensor before first do_wake so join path can read one value for Join uplink. */
    if (persistentData.runMode != RUN_MODE_TEST)
        sensors.begin();
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
        if (persistentData.runMode == RUN_MODE_DEV && Serial) Serial.flush();
        /* Session/wakeCounter persisted in do_sleep with save_session_to_flash(false) to avoid overwriting seqnoUp after LMIC_reset. */
        macStormRetryPending = false;  /* Avoid stale mac_retry_fallback firing after wake */
        consecutivePort0Downlinks = 0;
        LMIC_reset();
        LMIC.opmode = 0;  /* Clear opmode so next wake starts clean. */
        LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);  /* 20% for Feather M0 / RX2 window */
        APP_DISABLE_LMIC_DUTY_CYCLE();
        if (persistentData.runMode == RUN_MODE_DEV && hibernationRequested) {
            if (Serial) Serial.flush();
            delay(100);
        }
        os_setTimedCallback(&sendjob, os_getTime(), do_sleep);
        wakeDeadlineMs = millis() + 60000;  /* prevent re-trigger until do_sleep runs */
    }
}
