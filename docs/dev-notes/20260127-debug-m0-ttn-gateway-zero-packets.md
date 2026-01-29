# Debug: M0 LoRaWAN TTN – Gateway Receives 0 Packets

## Resolved

Link is working: gateway reports **RF packets received: 1**, **CRC_OK: 100%**, **RF packets forwarded: 1 (23 bytes)**. Fix was one of: EU868 in LMIC config, antenna (8.2 cm @ 868 MHz), or device/gateway distance. Document here which change fixed it if known.

---

## Situation

- Device: Adafruit Feather M0 + RFM9x, LMIC sketch, OTAA join.
- Serial: EV_JOINING → **EV_TXSTART** → **EV_JOIN_TXCOMPLETE** (repeating); device is clearly transmitting Join Requests.
- Gateway: "RF packets received by concentrator: 0", CRC_OK/CRC_FAIL/NO_CRC all 0.00%.

So the gateway never sees any frame from the device. The device stack and radio TX path are working (EV_TXSTART / EV_JOIN_TXCOMPLETE). The concentrator reports no LoRa energy on its channels — either the device is on a different frequency band, or antenna/distance prevents the gateway from receiving.

## Changes Made in Sketch

1. **Unknown event** now logs the event code: `Unknown event <code>`. Re-flash and note the code(s). In many LMIC ports, 15 = EV_TXSTART (radio TX started). If you see that, the stack is trying to send; the problem is then link/region/hardware.
2. **Duplicate `break`** after EV_REJOIN_FAILED removed (typo).
3. **EV_JOIN_FAILED** schedules a retry in 60 s via `do_send`, so the device keeps trying to join instead of stopping.

## What to Check Next

### 1. Region and frequency (EU868) — prime suspect

Gateway config is EU868 (867.5 / 868.5 MHz, 8 multi-SF channels). The device must transmit in **867–868 MHz**. If it’s on 902–928 MHz (US915) or another band, the gateway will always show 0 packets.

**Confirm device frequency:**

- Open your **huebe/arduino-lmic** clone (e.g. `Documents\Arduino\libraries\arduino-lmic` or `~/Arduino/libraries/arduino-lmic`). Look for:
  - `project_config/lmic_project_config.h` or `src/lmic_project_config.h`
  - or any `config.h` / `hal/hal.h` that sets region
- Search for `CFG_eu868`, `CFG_us915`, `EU868`, `US915`, or "region". Ensure **EU868** (or "TTN EU" / 868) is enabled and US915 is not.
- If you have an **RTL-SDR** or similar: tune to 868 MHz, run the device so it emits EV_TXSTART, and check for a clear spike when the device is transmitting. No spike in 867–868 MHz → device is on another band.

### 2. Antenna and placement

- Readme: ~8.2 cm wire for 868 MHz on the antenna pin.
- Put device and gateway close (e.g. 1–3 m indoors) to rule out range.
- Confirm antenna is connected to the RFM9x antenna pin (or U.FL/ipex if you use an external antenna).

### 3. Wiring (Feather M0 + RFM9x)

- Readme: **DIO1 → D6** — a wire must be soldered from the RFM9x DIO1 pin to Arduino D6. The sketch uses `dio = {3, 6, …}` (DIO0=3, DIO1=6). Without DIO1→D6 the HAL cannot get proper interrupt/timing and uplinks may fail.
- NSS, SCK, MISO, MOSI, RST as per the HAL; sketch pinmap: NSS=8, DIO0=3, DIO1=6.

### 4. Event codes (MCCI/extended LMIC)

If your LMIC is MCCI or an extended fork, event 17 = **EV_TXSTART** (radio TX started) and 20 = **EV_JOIN_TXCOMPLETE** (Join Request sent, waiting for Join Accept). Seeing both in order (EV_JOINING → EV_TXSTART → EV_JOIN_TXCOMPLETE) means the device is transmitting Join Requests; the gateway’s “0 packets” indicate the uplink is not reaching the gateway (region/frequency, antenna, or distance). The sketch now maps 16–20 to readable names (EV_SCAN_FOUND, EV_TXSTART, EV_TXCANCELED, EV_RXSTART, EV_JOIN_TXCOMPLETE).

### 5. TTN and keys

- AppEUI/DevEUI/AppKey must match the device in TTN.
- Same band/region in TTN console as on the device (EU868).

## Readme.md checklist (from m0-lorawan-ttn/Readme.md)

Verify these match your setup:

1. **LMIC library**: Use **huebe/arduino-lmic** only — `git clone https://github.com/huebe/arduino-lmic` into your Arduino `libraries` folder. The Readme says this fork has "a slower SPI speed and the correct radio settings" for Feather M0 + RFM95. Any other LMIC (matthijskooijman, MCCI, etc.) may use wrong pins or SPI timing and can cause 0 packets.
2. **Wiring**: DIO1 → D6 (wire from RFM9x DIO1 to Feather pin D6). See Readme wiring image.
3. **Antenna**: 8.2 cm for 868 MHz on the RFM9x antenna pin (Adafruit recommendation).
4. **Board**: In Arduino IDE, board type must be **Feather M0** (SAMD21).
5. **TTN credentials**: Readme says copy AppEUI/DevEUI in "C-Style lsb format" and App Key in "C-Style msb format" from the TTN device. The sketch uses little-endian EUIs and MSB App Key; we already mapped from `docs/app-device-config.h`.
6. **TTN Console**: Readme links to the legacy TTN v2 console. If you use TTN v3 (console.thethings.network), create the application/device there and ensure frequency plan is **Europe 863–870 MHz (SRD)** so it matches EU868.

## Gateway config (reference)

Gateway uses EU868, SX1302, 867.5 / 868.5 MHz, multichannel. No change needed on gateway side for this debug; focus on device region, antenna, and distance first.

---

## Summary when device shows EV_TXSTART / EV_JOIN_TXCOMPLETE but gateway shows 0 packets

| Check | Action |
|-------|--------|
| **Frequency** | Inspect huebe/arduino-lmic config: EU868 (867–868 MHz) must be set; US915 must not. Optionally verify with RTL-SDR at 868 MHz during EV_TXSTART. |
| **Antenna** | 8.2 cm for 868 MHz on RFM9x antenna pin; confirm connection. |
| **Distance** | Place device and gateway 1–2 m apart, same room, line-of-sight. |
| **Wiring** | DIO1→D6; you already see TX events, so timing is working; focus on frequency/antenna. |
