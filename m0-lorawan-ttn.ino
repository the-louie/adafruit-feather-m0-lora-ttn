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
 * Battery/sleep/Cayenne LPP (c) 2026: STANDBY deep sleep between sends (Arduino Low Power);
 * payload = Cayenne LPP: ch0 Unix timestamp (seconds since Epoch, RTC), ch1 battery V, ch2 DS18B20 temp (1 byte).
 * RTC keeps time across sleep; epoch persisted in flash for restore after power loss. Set RTC_DEFAULT_EPOCH or time source for initial time.
 * Session saved on join, restored on wake to skip join.
 *******************************************************************************/

#include <hal/hal.h>
#include <SPI.h>
// Install from Library Manager: "Arduino Low Power", "FlashStorage" by cmaglie, "Dallas Temperature" by Miles Burton, "RTCZero" by Arduino
#include <ArduinoLowPower.h>
#include <FlashStorage.h>
#include <RTCZero.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define VBATPIN A7
#define ONE_WIRE_BUS 5  /* DS18B20 data on pin 5 (not A5) */

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

RTCZero rtc;

/* Initial RTC epoch when no persisted value. Set to current Unix time at flash, or 0 to send 0 until time is set (e.g. via downlink). */
#ifndef RTC_DEFAULT_EPOCH
#define RTC_DEFAULT_EPOCH 0
#endif

// 1 = sleep 10 s (development), 0 = sleep 6 h (production)
#define USE_DEV_SLEEP 1
#define SLEEP_SECONDS_DEV  10
#define SLEEP_SECONDS_PROD (6 * 3600)
#define SLEEP_SECONDS      (USE_DEV_SLEEP ? SLEEP_SECONDS_DEV : SLEEP_SECONDS_PROD)

// Cayenne LPP: Digital Input = type 0 (1 byte); Analog Input = type 2 (2 bytes, 0.01, MSB first); custom Unix time = type 128 (4 bytes, MSB first)
#define LPP_DIGITAL_INPUT 0
#define LPP_ANALOG_INPUT  2
#define LPP_UNIX_TIME    128

// Temperature (ch2): 1 byte. 0=error, 1=<0°C, 2=>30°C, 3–255=data (temp_c = (byte-3)/8.43, ~0.118°C/step)
#define TEMP_ENCODED_SCALE 8.43f
#define TEMP_ENCODED_OFFSET 3

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
    uint32_t rtcEpoch; /* last known Unix time (seconds since Epoch); restored on boot, updated after TX; at end for layout compatibility */
} PersistentData_t;

PersistentData_t persistentData;
FlashStorage(persistentStore, PersistentData_t);

static osjob_t sendjob;
static uint32_t wakeCounter;  // value sent this run
static bool haveStoredSession;

// Pin mapping

const lmic_pinmap lmic_pins = {
    .nss = 8,  
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {3,6,LMIC_UNUSED_PIN},
};

// Suppress Serial during EV_JOINING..EV_JOINED/FAILED so runloop can service RX windows (~1–2 s after Join Request).
void onEvent (ev_t ev) {
    static bool joining = false;
    bool quiet = joining && (ev != EV_JOINED && ev != EV_JOIN_FAILED);
    if (!quiet) {
        Serial.print(os_getTime());
        Serial.print(": ");
    }
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            if (!quiet) Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            if (!quiet) Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            if (!quiet) Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            if (!quiet) Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            joining = true;
            if (!quiet) Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            joining = false;
            Serial.println(F("EV_JOINED"));
            LMIC_setLinkCheckMode(0);
            persistentData.magic = PERSIST_MAGIC;
            persistentData.wakeCounter = wakeCounter;
            persistentData.netid = (uint32_t)LMIC.netid;
            persistentData.devaddr = (uint32_t)LMIC.devaddr;
            memcpy(persistentData.nwkKey, LMIC.nwkKey, 16);
            memcpy(persistentData.artKey, LMIC.artKey, 16);
            persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
            persistentStore.write(persistentData);
            break;
        case EV_RFU1:
            if (!quiet) Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            joining = false;
            Serial.println(F("EV_JOIN_FAILED"));
            // Retry join after 60 s (no Join Accept in RX window; avoid blocking runloop during join).
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(60), do_send);
            break;
        case EV_REJOIN_FAILED:
            if (!quiet) Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            if (!quiet) Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (!quiet && (LMIC.txrxFlags & TXRX_ACK)) Serial.println(F("Received ack"));
            if (!quiet && LMIC.dataLen) {
                Serial.println(F("Received "));
                Serial.println(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }
            // Persist next wake index, uplink FCnt, and RTC epoch before delay/standby.
            persistentData.wakeCounter = wakeCounter + 1;
            persistentData.magic = PERSIST_MAGIC;
            persistentData.seqnoUp = (uint32_t)LMIC.seqnoUp;
            persistentData.rtcEpoch = (uint32_t)rtc.getEpoch();
            persistentStore.write(persistentData);
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_sleep);
            break;
        case EV_LOST_TSYNC:
            if (!quiet) Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            if (!quiet) Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            if (!quiet) Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            if (!quiet) Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            if (!quiet) Serial.println(F("EV_LINK_ALIVE"));
            break;
        case 16:
            if (!quiet) Serial.println(F("EV_SCAN_FOUND (MCCI/extended)"));
            break;
        case 17:
            if (!quiet) Serial.println(F("EV_TXSTART (radio TX started)"));
            break;
        case 18:
            if (!quiet) Serial.println(F("EV_TXCANCELED (MCCI/extended)"));
            break;
        case 19:
            if (!quiet) Serial.println(F("EV_RXSTART (opening RX window)"));
            break;
        case 20:
            if (!quiet) Serial.println(F("EV_JOIN_TXCOMPLETE (Join Request sent, waiting Join Accept)"));
            break;
        default:
            if (!quiet) { Serial.print(F("Unknown event ")); Serial.println((int)ev); }
            break;
    }
}

// Cayenne LPP: ch0 = Unix timestamp (type 128, 4 bytes MSB first), ch1 = battery V (Analog Input), ch2 = temperature (Digital Input, 1 byte).
// Temperature encoding: 0=error, 1=<0°C, 2=>30°C, 3–255=(temp*8.43)+3 truncated, ~0.118°C/step.
static uint8_t buildCayenneLPP(uint8_t* buf, uint32_t epoch, float voltageV, float tempC) {
    uint8_t i = 0;
    int16_t val;

    buf[i++] = 0;
    buf[i++] = LPP_UNIX_TIME;
    buf[i++] = (uint8_t)(epoch >> 24);
    buf[i++] = (uint8_t)(epoch >> 16);
    buf[i++] = (uint8_t)(epoch >> 8);
    buf[i++] = (uint8_t)(epoch & 0xff);

    buf[i++] = 1;
    buf[i++] = LPP_ANALOG_INPUT;
    val = (int16_t)(voltageV * 100.0f);
    buf[i++] = (uint8_t)(val >> 8);
    buf[i++] = (uint8_t)(val & 0xff);

    buf[i++] = 2;
    buf[i++] = LPP_DIGITAL_INPUT;
    if (tempC < -50.0f || tempC != tempC) {
        buf[i++] = 0;
    } else if (tempC < 0.0f) {
        buf[i++] = 1;
    } else if (tempC > 30.0f) {
        buf[i++] = 2;
    } else {
        int v = (int)(tempC * TEMP_ENCODED_SCALE + (float)TEMP_ENCODED_OFFSET);
        if (v > 255) v = 255;
        if (v < TEMP_ENCODED_OFFSET) v = TEMP_ENCODED_OFFSET;
        buf[i++] = (uint8_t)v;
    }

    return i;
}

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        return;
    }
    float vbat = analogRead(VBATPIN) * (2.0f * 3.3f / 1024.0f);
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    uint32_t epoch = (uint32_t)rtc.getEpoch();

    Serial.print(F("Sending ts="));
    Serial.print(epoch);
    Serial.print(F(" VBat="));
    Serial.print(vbat);
    Serial.print(F(" Temp="));
    Serial.println(tempC);

    uint8_t lpp[16];
    uint8_t len = buildCayenneLPP(lpp, epoch, vbat, tempC);
    LMIC_setTxData2(1, lpp, len, 0);
    Serial.println(F("Packet queued (Cayenne LPP)"));
}

// STANDBY sleep via Arduino Low Power (re-inits clocks, RTC wake). Optional: detach USB before
// sleep for true ~5–15 µA; set DETACH_USB_BEFORE_SLEEP 1 and provide USBDevice if your core has it.
#define DETACH_USB_BEFORE_SLEEP 0
#if DETACH_USB_BEFORE_SLEEP
#include <USB/USBDevice.h>
#endif

void do_sleep(osjob_t* j) {
    (void)j;
    /* Re-persist state already set in EV_TXCOMPLETE (wakeCounter+1, seqnoUp). Do not
     * update wakeCounter here: use of persistentData.wakeCounter would double-increment
     * and yield payload counters 0,2,4,6,8 instead of 0,1,2,3,4,... */
    persistentData.magic = PERSIST_MAGIC;
    persistentStore.write(persistentData);

#if DETACH_USB_BEFORE_SLEEP
    USBDevice.detach();
#endif
    LowPower.deepSleep((uint32_t)SLEEP_SECONDS * 1000UL);
#if DETACH_USB_BEFORE_SLEEP
    USBDevice.attach();
#endif
    wakeCounter = persistentData.wakeCounter;
    do_send(&sendjob);
}


void setup() {
    Serial.begin(9600);
    delay(2000);
    Serial.println(F("Starting"));

    persistentStore.read(&persistentData);
    haveStoredSession = (persistentData.magic == PERSIST_MAGIC);
    wakeCounter = haveStoredSession ? persistentData.wakeCounter : 0;
    Serial.print(F("Wake #"));
    Serial.println(wakeCounter);

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
        Serial.println(F("Session restored from flash"));
    }

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    sensors.begin();
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
