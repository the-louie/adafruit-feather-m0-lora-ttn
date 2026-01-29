This project is based on [werktag/m0-lorawan-ttn](https://github.com/werktag/m0-lorawan-ttn). We are grateful for that work; this code would not exist without it.

---

# Adafruit Feather M0 LoRa on The Things Network

This tutorial describes how to setup the [Adafruit Feather M0 with RFM95 LoRa Radio](https://www.adafruit.com/product/3178) for [The Things Network](https://www.thethingsnetwork.org/) and transmit batched temperature logs and battery voltage.

## Wiring

To work with The Things Network, a wire has to be soldered from DI01 to D6 (blue wire):

![feather wiring](feather-lora-wiring.png)

Adafruit [recommends](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/antenna-options) to solder an antenna with 8.2 cm length (for 868 MHz) to the antenna pin (yellow wire).

### Battery voltage (external battery on VUSB)

If you power the Feather via **VUSB** from an external battery (e.g. USB power bank), the built-in JST battery sense is not used. Add a voltage divider from the battery (or VIN) to **A7** and set the sketch divider factor. See [Battery measurements](docs/battery_measurements.md) for resistor values (e.g. 100 kΩ / 200 kΩ for 5 V max) and sketch changes.

### DS18B20 temperature sensor (optional)

To add a DS18B20 one-wire temperature sensor, use **Pin 5** for the data line. Alternatives: Pin 10, 11, or 12. Pins 3, 4, 6, 8, and 9 are in use by the LoRa radio or battery sense. Wiring: DS18B20 data → chosen pin; **4.7 kΩ** resistor from data to **3V**; GND → GND; VDD → **3V** (3.3 V operation). Install **Dallas Temperature** (Miles Burton) and **OneWire** (Paul Stoffregen) from Library Manager. See [DS18B20 pin and wiring](docs/ds18b20-pin-and-wiring.md) for details.

## How to set up a working development environment for the Feather M0 LoRa

Setup the Arduino IDE with the necessary libraries: Follow the [tutorial](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/overview) on Adafruit, or if your Arduino IDE (or VS Code with Arduino integration) is already setup, do:

1.  Add `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json` to your Additional Boards Manager URLs
2.  Install: `Arduino SAMD Boards` in the Boards Manager
3.  Install: `Adafruit SAMD` in the Boards Manager
4.  Select the Board Type `Feather M0`
5.  Under Linux/Ubuntu, you might have to add your user to the dialout group: `sudo usermod -a -G dialout $USER`, and make sure the changes are applied.
6.  Select the port for your feather.

Clone this Arduino Sketch: `git clone https://github.com/werktag/m0-lorawan-ttn`

Clone the lmic library adjusted for the Feather M0 LoRa into your Arduino library folder: `git clone https://github.com/huebe/arduino-lmic` (The Arduino library folder is usually be found in `Documents\Arduino\libraries` in Windows, and `~/Arduino/libraries` in Linux)

The changes to the original lmic library are a slower SPI speed and the correct radio settings.

For the battery/sleep/Cayenne LPP version, install from **Library Manager** (Sketch → Include Library → Manage Libraries): search **"RTCZero"** → Install (by Arduino); search **"FlashStorage"** → Install (by cmaglie, for SAMD21 flash). Both are required; they are not bundled with the Adafruit SAMD board package.

Now it's time to verify the sketch and upload it onto your feather.
If the Sketch can't be uploaded, a double click on the reset button of the feather might help. The blinking red LED indicates that your feather is in bootloader state and ready to be programmed.

## Getting the credentials form The Things Network

If you don't have login for The Things Network (TTN), create one.

To adjust the TTN credentials, edit **`app-device-config.h`** (the repo ships placeholders only; do not commit this file after putting real keys—add `app-device-config.h` to `.gitignore` locally if you use real keys):

   * In the [TTN Console](https://console.thethingsnetwork.org/), create an application.
   * Add a device to the application. Give it a Device ID of your choice, click the "Generate" button for Device EUI, leave App Key empty and press "Register".
   * In the Device Overview, copy the Application EUI into `LORAWAN_APP_EUI_LE` (8 bytes, little-endian).
   * In the Device Overview, copy the Device EUI into `LORAWAN_DEV_EUI_LE` (8 bytes, little-endian) and `LORAWAN_DEV_EUI` (same value as MSB uint64_t).
   * In the Device Overview, copy the App Key into `LORAWAN_APP_KEY_ARR` (16 bytes, MSB as from TTN).

  ![ttn console](ttn-console-format.png)

## Retrieving data on The Things Network

The sketch uses a **batched log flow**:

- **Measure:** Every 5 min it wakes, reads DS18B20 temperature and RTC, and appends one 5-byte **LogEntry** (3-byte 1-min tick since 2026-01-01 00:00:00 UTC, 2-byte high-res temperature in centidegrees) into a **RAM buffer** (up to 48 entries).
- **Backup:** Every 6 h (every 72nd wake), before sending, it merges the current RAM buffer with any unsent data already in **Flash** and saves the combined log back to Flash.
- **Send:** It builds one uplink with battery voltage plus the batched entries and attempts the LoRaWAN uplink (up to 43 entries per uplink to stay within EU868 payload limit).
- **Confirm:** On success it clears the Flash log; on failure it keeps the data in Flash and retries in 6 h with the next batch merged in.

Time is kept by the RTC (RTCZero) across sleep; epoch is persisted in flash and restored after power loss. **RTC is synced from the network** via LoRaWAN DeviceTimeReq/DeviceTimeAns (first after EV_JOINED, then at most once per 24 h). Session is saved on join and restored on wake. Sleep: 30 s in development (`USE_DEV_SLEEP 1`), 5 min in production (`USE_DEV_SLEEP 0`).

**Payload format** (big-endian): `[vbat_hi, vbat_lo]` (battery × 100, 0.01 V), `[n]` (entry count, 0–43), then for each entry `[timeTick_hi, timeTick_mid, timeTick_lo, temp_hi, temp_lo]` (5 bytes). Tick = 1-min since 2026-01-01 00:00:00 UTC (24-bit). Temperature: 0–3000 = 0.00–30.00°C (centidegrees); 0xFFFD = &gt;30°C, 0xFFFE = &lt;0°C, 0xFFFF = error. Typical size e.g. 2+1+5×12 = 63 bytes for 12 entries.

In [TTN Console](https://console.thethingsnetwork.org/) (or [The Things Stack](https://console.thethings.network/)) → Application → Payload Formats: set to **Custom** and paste the decoder from [`ttn-decoder-batch.js`](ttn-decoder-batch.js), or use this:

```javascript
function decodeUplink(input) {
  var b = input.bytes;
  if (!b || b.length === 0) {
    if (input.frm_payload && typeof atob === 'function') {
      var raw = atob(input.frm_payload);
      b = [];
      for (var i = 0; i < raw.length; i++) b.push(raw.charCodeAt(i));
    }
  }
  if (!b || b.length === 0) return { data: {} };
  if (typeof b.slice === 'function') b = Array.from(b);

  if (input.fPort === 2) {
    var text = '';
    for (var t = 0; t < b.length; t++) text += String.fromCharCode(b[t] & 0xFF);
    return { data: { message: text } };
  }

  if (b.length < 3) return { data: {} };

  var battery_v = (((b[0] << 8) | b[1]) >>> 0) / 100;
  var n = b[2] & 0xFF;
  var maxN = ((b.length - 3) / 5) | 0;
  if (n > maxN) n = maxN;

  var customEpoch = new Date('2026-01-01T00:00:00Z').getTime();
  var receivedAtMs = null;
  if (input.recvTime && typeof input.recvTime.getTime === 'function') {
    receivedAtMs = input.recvTime.getTime();
  } else if (input.uplink_message && input.uplink_message.received_at) {
    receivedAtMs = new Date(input.uplink_message.received_at).getTime();
  }

  var entries = [];
  for (var i = 0, j = 3; i < n && j + 5 <= b.length; i++, j += 5) {
    var ticks = ((b[j] << 16) | (b[j + 1] << 8) | b[j + 2]) >>> 0;
    var temp = ((b[j + 3] << 8) | b[j + 4]) >>> 0;
    var ts;
    if (ticks === 0 && receivedAtMs !== null && n > 0) {
      var offsetMs = (n - 1 - i) * 300 * 1000;
      ts = new Date(receivedAtMs - offsetMs).toISOString();
    } else {
      ts = new Date(customEpoch + ticks * 60 * 1000).toISOString();
    }
    if (temp === 0xFFFF) {
      entries.push({ timestamp: ts, temperature_state: 'error' });
    } else if (temp === 0xFFFE) {
      entries.push({ timestamp: ts, temperature_state: 'under_range' });
    } else if (temp === 0xFFFD) {
      entries.push({ timestamp: ts, temperature_state: 'over_range' });
    } else {
      entries.push({ timestamp: ts, temperature_c: temp / 100 });
    }
  }

  return { data: { battery_v: battery_v, entries: entries } };
}
```

The decoded `battery_v` and `entries[]` (each with `timestamp` and `temperature_c` or `temperature_state`) appear in the application data.

(as you might notice, the battery is charging ;) )

That's it, enjoy LoRaWAN with your feather!

If the gateway receives 0 packets or joins fail, check: DIO1→D6 wiring, 8.2 cm antenna for 868 MHz, use of **huebe/arduino-lmic** (not another LMIC), Feather M0 as board, and EU868 / Europe 863–870 MHz in TTN. See project `docs/dev-notes/` (e.g. `20260127-debug-m0-ttn-gateway-zero-packets.md`) for a debug checklist.

If you see **`as.up.data.decode.fail`**, ensure the application uses **Custom** payload formatter with the batched decoder above (not built-in Cayenne LPP).

Enjoyed this article? Head over to [Werktag Blog](https://blog.werktag.io) for more articles.