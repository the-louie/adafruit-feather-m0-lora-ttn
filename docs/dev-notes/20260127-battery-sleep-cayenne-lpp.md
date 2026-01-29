# Battery / deep sleep / Cayenne LPP (step one)

## Summary

Step one of the battery-powered M0 LoRaWAN device:

1. **Wake counter** – Incremented each wake cycle, persisted in flash, sent as Cayenne LPP channel 0 (Analog Input, 0.01 resolution). Decoder shows value/100 (e.g. 5.00 for wake #5). Counter capped at 327 (int16_t range for 0.01 encoding).
2. **Battery voltage** – Read from VBATPIN (A7), sent as Cayenne LPP channel 1 (Analog Input, 0.01 resolution). Value in volts (e.g. 3.45 V).
3. **Cayenne LPP** – Payload format per [Cayenne LPP](https://docs.mydevices.com/docs/lorawan/cayenne-lpp): [ch][type][2 bytes MSB] per sensor. Type 2 = Analog Input, 0.01 resolution.
4. **Sleep** – After TX complete, device saves wake counter +1 and session (if any) to flash, sets RTC alarm, enters `rtc.standbyMode()`. Wake interval: **30 s** when `USE_DEV_SLEEP 1` (development), **6 h** when `USE_DEV_SLEEP 0` (production). RTCZero used for alarm.
5. **Session persistence** – On EV_JOINED, netid, devaddr, nwkKey, artKey are written to flash. On wake, if magic matches, `LMIC_setSession()` is called so the device sends without joining. First boot or after invalid/flushed flash: join as usual, then save on join.

## Required libraries

- **RTCZero** – Usually provided by Arduino SAMD or Adafruit SAMD (RTC alarm for wake).
- **FlashStorage** (cmaglie) – Non-volatile storage for wake counter and session on SAMD21 (no EEPROM). Install from Library Manager if needed.

## Configuration

- `USE_DEV_SLEEP 1` → sleep 30 s (development).
- `USE_DEV_SLEEP 0` → sleep 6 h (production).
- Cayenne LPP ch0 = wake counter, ch1 = battery V. TTN payload decoder example in m0-lorawan-ttn Readme.md.

## Behaviour

- **First run / no valid session**: Join (OTAA), send Cayenne LPP (counter=0, battery), on EV_JOINED save session, on EV_TXCOMPLETE save counter+1 and sleep.
- **Wake with valid session**: Restore session, send Cayenne LPP (counter, battery), on EV_TXCOMPLETE save counter+1 and sleep. No join.
- **Join failed**: Retry do_send in 60 s (no sleep).

Session restore uses `LMIC_setSession(netid, devaddr, nwkKey, artKey)` with data read from flash. Compatible with classic/huebe LMIC layout (LMIC.netid, LMIC.devaddr, LMIC.nwkKey, LMIC.artKey). MCCI LMIC has `LMIC_getSessionKeys`; if you use MCCI, you can switch to that and keep the same stored layout.
