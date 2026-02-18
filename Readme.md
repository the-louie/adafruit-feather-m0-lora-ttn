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

To add a DS18B20 one-wire temperature sensor, use **Pin 5** for the data line. Alternative: Pin 10. Pins 3, 4, 6, 8, and 9 are in use by the LoRa radio or battery sense; **pins 11 and 12 are reserved for run-mode strapping** (v3.3.0) and must not be used for the sensor. Wiring: DS18B20 data → chosen pin; **4.7 kΩ** pull-up from data to **3V** (required to avoid sensor hang / -127.00); GND → GND; VDD → **3V** (3.3 V operation). Install **Dallas Temperature** (Miles Burton) and **OneWire** (Paul Stoffregen) from Library Manager. See [DS18B20 pin and wiring](docs/ds18b20-pin-and-wiring.md) for details.

## How to set up a working development environment for the Feather M0 LoRa

Setup the Arduino IDE with the necessary libraries: Follow the [tutorial](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/overview) on Adafruit, or if your Arduino IDE (or VS Code with Arduino integration) is already setup, do:

1.  Add `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json` to your Additional Boards Manager URLs
2.  Install: `Arduino SAMD Boards` in the Boards Manager
3.  Install: `Adafruit SAMD` in the Boards Manager
4.  Select the Board Type `Feather M0`
5.  Under Linux/Ubuntu, you might have to add your user to the dialout group: `sudo usermod -a -G dialout $USER`, and make sure the changes are applied.
6.  Select the port for your feather.

Clone this Arduino Sketch: `git clone https://github.com/werktag/m0-lorawan-ttn`

**LMIC library (MCCI LoRaWAN LMIC):** This sketch uses the **MCCI LoRaWAN LMIC library** (v4.1.1 or newer). Install it via **Library Manager** (Sketch → Include Library → Manage Libraries): search for **"MCCI LoRaWAN LMIC library"** and install. If you previously used "IBM LMIC framework" or "arduino-lmic", uninstall them in Manage Libraries and remove any folders named `arduino-lmic` or `IBM_LMIC` from your Arduino libraries directory (e.g. `Documents\Arduino\libraries` on Windows, `~/Arduino/libraries` on Linux) to avoid header conflicts. After installing MCCI LMIC, you must set the region and radio in the library: open `lmic_project_config.h` inside the library (typically `.../libraries/MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h`) and ensure only **EU868** and **SX1276** are enabled, e.g. `#define CFG_eu868 1` and `#define CFG_sx1276_radio 1`; other region/radio defines should be 0 or commented out. See [MCCI LMIC configuration](docs/mcci-lmic-config.md) for details.

For the battery/sleep version, install from **Library Manager**: search **"FlashStorage"** → Install (by cmaglie, for SAMD21 flash). RTCZero is not used in v3.1 (timeless model).

Now it's time to verify the sketch and upload it onto your feather.
If the Sketch can't be uploaded, a double click on the reset button of the feather might help. The blinking red LED indicates that your feather is in bootloader state and ready to be programmed.

## Getting the credentials from The Things Network

If you don't have login for The Things Network (TTN), create one.

To adjust the TTN credentials, edit **`app-device-config.h`** (the repo ships placeholders only; do not commit this file after putting real keys—add `app-device-config.h` to `.gitignore` locally if you use real keys):

   * In the [TTN Console](https://console.thethingsnetwork.org/), create an application.
   * Add a device to the application. Give it a Device ID of your choice, click the "Generate" button for Device EUI, leave App Key empty and press "Register".
   * In the Device Overview, copy the Application EUI into `LORAWAN_APP_EUI_LE` (8 bytes, little-endian).
   * In the Device Overview, copy the Device EUI into `LORAWAN_DEV_EUI_LE` (8 bytes, little-endian) and `LORAWAN_DEV_EUI` (same value as MSB uint64_t).
   * In the Device Overview, copy the App Key into `LORAWAN_APP_KEY_ARR` (16 bytes, MSB as from TTN).

  ![ttn console](ttn-console-format.png)

## Retrieving data on The Things Network

The sketch uses a **batched log flow** (v3.1 Clean-Join; Timeless Resilience):

- **Measure:** On each wake it reads DS18B20 temperature and appends one **log entry** (2-byte temperature) into a **RAM buffer** (newest at index 0; up to 48 entries).
- **Backup:** On each send wake it merges RAM with any unsent data in **Flash** (newest-first) and saves to Flash.
- **Send:** It builds one uplink with 4-byte header (vbat, flags, sequence) plus batched entries and sends on **FPort 10** (DEV/TEST, 300 s) or **FPort 20** (PROD, 10800 s). Interval is encoded by FPort so the decoder knows the sample spacing.
- **Confirm:** On success it clears the Flash log; on failure it keeps data and retries next send wake.

**v3.1 Timeless:** Time is **not** kept on the device. The gateway **received_at** is the time anchor; the decoder assigns entry *i* the timestamp `received_at - i × interval_seconds`. **FPort 10** = 300 s (5 min), **FPort 20** = 10800 s (3 h). If you change PROD to a different interval, you must update the decoder FPort→interval mapping in [`ttn-decoder-batch.js`](ttn-decoder-batch.js) so timestamps stay correct. **20% clock error** is kept for Feather M0 so Join Accept and MAC downlinks (e.g. ADR, LinkCheckAns) are received reliably. Session is saved on join and restored on wake. **Join-first:** DS18B20/sensors are initialised only after EV_JOINED or when restoring a session. **v3.2 Commissioning:** On first join the device stays awake (delay instead of deep sleep) and does not force sleep when the radio is busy until two successful data cycles after join, so bench commissioning is reliable. **v3.2.1:** App watchdog 10s after EV_TXSTART when session exists (not during join). **v3.3.0:** **Single binary:** run mode is selected at cold boot by **hardware strapping** (GND on **Pin 11** = DEV, **Pin 12** = TEST, no jumper = PROD). Mode is persisted in flash until next hardware reset or reflash; pins are read only at cold boot then set to High-Z for 0 µA. **PROD:** Serial not started; USB detached before radio init. Sleep: 300 s (DEV/TEST) or 10800 s (PROD); **delta sleep** (interval − awake time) keeps the measurement period accurate. **Battery:** `DETACH_USB_BEFORE_SLEEP` in PROD for ~5–15 µA sleep. Below **3.4 V** sleep is doubled; below **3.2 V** skip TX, sleep 600 s. **BOD33** 2.7 V. 1000µF on supply recommended. DS18B20 timeout 2 s → 0xFFFF. **SF:** Default SF7; 3× EV_LINK_DEAD → SF8 one cycle. **Join:** 90 s budget; 3× join watchdog → 1 h hibernation. **EV_JOIN_FAILED** backoff: 10 min, 30 min, 1 h.

**Session resumption:** Enable **Resend Frame Counter** or **Relax Frame Counter** in TTN Console if the NS rejects after restore. DevAddr 0 forces new join. **Sequence** (1 byte in header) allows the decoder/backend to detect lost reporting cycles (e.g. sequence 10 then 12 = one interval lost).

**Payload format (v3.1):** **FPort 10** (300 s) / **FPort 20** (10800 s): 4-byte header `[vbat_hi, vbat_lo]`, `[flags]` (bit0 = watchdog, bit1 = cold boot), `[sequence]` (0–255), then `[temp_hi, temp_lo]`×n. Newest reading first. After join only this batch is sent (no post-join HELLO). **FPort 2** = HELLO WORLD (decoder supports; firmware v3.1 does not send it on first join). **FPort 1** / **FPort 4** = legacy (decoder still supports them). Temperature: 0–3000 = centidegrees; 0xFFFD/0xFFFE/0xFFFF = over/under/error. Use [`ttn-decoder-batch.js`](ttn-decoder-batch.js).

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

  if (input.fPort != null && Number(input.fPort) === 4 && b.length >= 3) {
    var battery_v = (((b[0] << 8) | b[1]) >>> 0) / 100;
    return { data: { battery_v: battery_v, time_request: true } };
  }
  if (input.fPort != null && Number(input.fPort) === 2) {
    var text = '';
    for (var t = 0; t < b.length; t++) text += String.fromCharCode(b[t] & 0xFF);
    return { data: { message: text } };
  }
  if ((input.fPort != null && Number(input.fPort) !== 1) || b.length < 6) return { data: {} };

  var headerLen, battery_v, flags, n, baseTick, entryStart;
  if (b.length >= 7 && (b.length - 7) % 2 === 0) {
    headerLen = 7;
    battery_v = (((b[0] << 8) | b[1]) >>> 0) / 100;
    flags = b[2] & 0xFF;
    n = b[3] & 0xFF;
    baseTick = ((b[4] << 16) | (b[5] << 8) | b[6]) >>> 0;
    entryStart = 7;
  } else if ((b.length - 6) % 2 === 0) {
    headerLen = 6;
    battery_v = (((b[0] << 8) | b[1]) >>> 0) / 100;
    flags = 0;
    n = b[2] & 0xFF;
    baseTick = ((b[3] << 16) | (b[4] << 8) | b[5]) >>> 0;
    entryStart = 6;
  } else {
    return { data: {} };
  }
  var maxN = (b.length - headerLen) >> 1;
  if (n > maxN) n = maxN;

  var customEpoch = 1735689600 * 1000;
  var receivedAtMs = null;
  if (input.recvTime && typeof input.recvTime.getTime === 'function') {
    receivedAtMs = input.recvTime.getTime();
  } else if (input.uplink_message && input.uplink_message.received_at) {
    receivedAtMs = new Date(input.uplink_message.received_at).getTime();
  }

  var entries = [];
  for (var i = 0; i < n; i++) {
    var j = entryStart + i * 2;
    var temp = ((b[j] << 8) | b[j + 1]) >>> 0;
    var ts;
    if (baseTick === 0 && receivedAtMs !== null && n > 0) {
      var offsetMs = (n - 1 - i) * 300 * 1000;
      ts = new Date(receivedAtMs - offsetMs).toISOString();
    } else {
      ts = new Date(customEpoch + (baseTick + i * 5) * 60 * 1000).toISOString();
    }
    if (temp === 0xFFFF) {
      entries.push({ timestamp: ts, temperature_state: 'error' });
    } else if (temp === 0xFFFE) {
      entries.push({ timestamp: ts, temperature_state: 'under_range' });
    } else if (temp === 0xFFFD) {
      entries.push({ timestamp: ts, temperature_state: 'over_range' });
    } else {
      entries.push({ timestamp: ts, temperature_c: Number((temp / 100).toFixed(2)) });
    }
  }
  var data = { battery_v: battery_v, entries: entries };
  if (headerLen === 7 && (flags & 1)) data.watchdog_triggered = true;
  return { data: data };
}
```

The decoded `battery_v` and `entries[]` (each with `timestamp` and `temperature_c` or `temperature_state`) appear in the application data.

(as you might notice, the battery is charging ;) )

That's it, enjoy LoRaWAN with your feather!

If the gateway receives 0 packets or joins fail, check: DIO1→D6 wiring (IO1 to Pin 6), 8.2 cm antenna for 868 MHz, use of **MCCI LoRaWAN LMIC library** with EU868 and SX1276 in `lmic_project_config.h`, Feather M0 as board, and EU868 / Europe 863–870 MHz in TTN. See project `docs/dev-notes/` (e.g. `20260127-debug-m0-ttn-gateway-zero-packets.md`) for a debug checklist.

If you see **`as.up.data.decode.fail`**, ensure the application uses **Custom** payload formatter with the batched decoder above (not built-in Cayenne LPP).

Enjoyed this article? Head over to [Werktag Blog](https://blog.werktag.io) for more articles.