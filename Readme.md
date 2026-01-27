# Adafruit Feather M0 LoRa on The Things Network

This tutorial describes how to setup the [Adafruit Feather M0 with RFM95 LoRa Radio](https://www.adafruit.com/product/3178) for [The Things Network](https://www.thethingsnetwork.org/) and transmit batched temperature logs and battery voltage.

## Wiring

To work with The Things Network, a wire has to be soldered from DI01 to D6 (blue wire):

![feather wiring](feather-lora-wiring.png)

Adafruit [recommends](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/antenna-options) to solder an antenna with 8.2 cm length (for 868 MHz) to the antenna pin (yellow wire).

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

To adjust the TTN Credentials in the Sketch:

   * In the [TTN Console](https://console.thethingsnetwork.org/), create an application.
   * Add a device to the application. Give it a Device ID of you choice, click the "Generate" button for Device EUI, leave App Key empty and press "Register".
   * In the Device Overview, copy the Application EUI to the APPEUI variable in the Sketch in the C-Style lsb format.
   * In the Device Overview, copy the Device EUI to the DEVEUI variable in the Sketch in the C-Style lsb format.
   * In the Device Overview, copy the App Key to the APPKEY variable in the Sketch in the C-Style msb format.

  ![ttn console](ttn-console-format.png)

## Retrieving data on The Things Network

The sketch uses a **batched log flow**:

- **Measure:** Every 30 min it wakes, reads DS18B20 temperature and RTC, and appends one 4-byte **LogEntry** (2-byte 30-min tick since 2026-01-01 00:00:00 UTC, 2-byte high-res temperature in centidegrees) into a **RAM buffer** (up to 48 entries = 24 h).
- **Backup:** Every 6 h (every 12th wake), before sending, it merges the current RAM buffer with any unsent data already in **Flash** and saves the combined log back to Flash.
- **Send:** It builds one uplink with battery voltage plus the batched entries and attempts the LoRaWAN uplink.
- **Confirm:** On success it clears the Flash log; on failure it keeps the data in Flash and retries in 6 h with the next batch merged in.

Time is kept by the RTC (RTCZero) across sleep; epoch is persisted in flash and restored after power loss. **RTC is synced from the network** via LoRaWAN DeviceTimeReq/DeviceTimeAns (first after EV_JOINED, then at most once per 24 h). Session is saved on join and restored on wake. Sleep: 30 s in development (`USE_DEV_SLEEP 1`), 30 min in production (`USE_DEV_SLEEP 0`).

**Payload format** (big-endian): `[vbat_hi, vbat_lo]` (battery × 100, 0.01 V), `[n]` (entry count, 0–48), then for each entry `[timeTick_hi, timeTick_lo, temp_hi, temp_lo]`. Temperature: 0–3000 = 0.00–30.00°C (centidegrees); 0xFFFD = &gt;30°C, 0xFFFE = &lt;0°C, 0xFFFF = error. Typical size about 60+ bytes (e.g. 2+1+4×12 = 51 bytes for 12 entries).

In [TTN Console](https://console.thethingsnetwork.org/) (or [The Things Stack](https://console.thethings.network/)) → Application → Payload Formats: set to **Custom** and paste the decoder from [`ttn-decoder-batch.js`](ttn-decoder-batch.js), or use this:

```javascript
function decodeUplink(input) {
  var b = input.bytes;
  if (!b || b.length < 3) {
    if (input.frm_payload && typeof atob === 'function') {
      var raw = atob(input.frm_payload);
      b = [];
      for (var i = 0; i < raw.length; i++) b.push(raw.charCodeAt(i));
    }
  }
  if (!b || b.length < 3) return { data: {} };
  if (typeof b.slice === 'function') b = Array.from(b);

  var battery_v = ((b[0] << 8) | b[1]) / 100;
  var n = b[2] & 0xFF;
  var maxN = (b.length - 3) >> 2;
  if (n > maxN) n = maxN;

  var entries = [];
  var customEpoch = new Date('2026-01-01T00:00:00Z').getTime();
  for (var i = 0, j = 3; i < n && j + 4 <= b.length; i++, j += 4) {
    var ticks = (b[j] << 8) | b[j + 1];
    var temp = (b[j + 2] << 8) | b[j + 3];
    var ts = new Date(customEpoch + ticks * 1800 * 1000).toISOString();
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