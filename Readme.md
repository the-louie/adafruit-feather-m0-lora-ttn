# Adafruit Feather M0 LoRa on The Things Network

This tutorial describes how to setup the [Adafruit Feather M0 with RFM95 LoRa Radio](https://www.adafruit.com/product/3178) for [The Things Network](https://www.thethingsnetwork.org/) and transmit the battery voltage encoded to reduce the duty cycle.

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

The sketch sends **Cayenne LPP** ([Cayenne Low Power Payload](https://docs.mydevices.com/docs/lorawan/cayenne-lpp)): channel 0 = Unix timestamp (seconds since Epoch, custom type 128, 4 bytes MSB first), channel 1 = battery voltage in V (Analog Input, 0.01 resolution), channel 2 = DS18B20 temperature (Digital Input, 1 byte): 0 = error, 1 = &lt;0°C, 2 = &gt;30°C, 3–255 = temperature °C encoded as `(byte−3)/8.43` (~0.118°C/step over 0–30°C). Time is kept by the RTC (RTCZero) across sleep; epoch is persisted in flash and restored after power loss. **RTC is synced from the network** via LoRaWAN DeviceTimeReq/DeviceTimeAns: the first request is sent after EV_JOINED, then at most once per 24 hours (TTN fair use). This requires an LMIC library that supports LoRaWAN 1.0.3 and DeviceTimeReq (e.g. [MCCI LoRaWAN LMIC](https://github.com/mcci-catena/arduino-lmic)); ensure `LMIC_ENABLE_DeviceTimeReq` is enabled in the LMIC project config. If no network time is available, set `RTC_DEFAULT_EPOCH` in the sketch or leave 0. Session is saved on join and restored on wake so the device can skip join when possible. Sleep interval: 30 s in development (`USE_DEV_SLEEP 1`), 6 h in production (`USE_DEV_SLEEP 0`).

In [TTN Console](https://console.thethingsnetwork.org/) (or [The Things Stack](https://console.thethings.network/)) → Application → Payload Formats: **use Custom** (not the built-in Cayenne LPP). The built-in Cayenne LPP formatter does not support our custom type 128 (Unix timestamp) and will produce `as.up.data.decode.fail` with `pkg/messageprocessors/cayennelpp` / "invalid output". Paste this decoder for ch0 (Unix timestamp), ch1 (battery V), and ch2 (temperature, 1-byte encoding):

```javascript
function decodeUplink(input) {
  var b = input.bytes;
  if (b.length < 4) return { data: {} };
  var i = 0;
  var out = {};
  while (i < b.length) {
    var ch = b[i++], type = b[i++];
    if (type === 128 && i + 4 <= b.length) {
      var ts = (b[i] << 24) | (b[i+1] << 16) | (b[i+2] << 8) | b[i+3]; i += 4;
      if (ch === 0) out.timestamp_utc = ts;
    } else if (type === 2 && i + 2 <= b.length) {
      var val = (b[i] << 8) | b[i+1]; i += 2;
      if (ch === 1) out.battery_v = val / 100;
    } else if (type === 0 && i < b.length) {
      var v = b[i++];
      if (ch === 2) {
        if (v === 0) out.temperature_state = 'error';
        else if (v === 1) out.temperature_state = 'under_range';
        else if (v === 2) out.temperature_state = 'over_range';
        else out.temperature_c = (v - 3) / 8.43;
      }
    } else break;
  }
  return { data: out };
}
```

(Ch0: custom type 128, 4 bytes MSB first = Unix timestamp (seconds since Epoch). Ch1: Cayenne LPP Analog Input type 2, 2-byte value MSB first, 0.01 resolution. Ch2: Digital Input type 0, 1 byte — 0=error, 1=&lt;0°C, 2=&gt;30°C, 3–255=(temp×8.43)+3 truncated.)

The decoded values appear in the application data.

(as you might notice, the battery is charging ;) )

That's it, enjoy LoRaWAN with your feather!

If the gateway receives 0 packets or joins fail, check: DIO1→D6 wiring, 8.2 cm antenna for 868 MHz, use of **huebe/arduino-lmic** (not another LMIC), Feather M0 as board, and EU868 / Europe 863–870 MHz in TTN. See project `docs/dev-notes/` (e.g. `20260127-debug-m0-ttn-gateway-zero-packets.md`) for a debug checklist.

If you see **`as.up.data.decode.fail`** with `namespace: pkg/messageprocessors/cayennelpp` and `message_format: invalid output`, the application is using the built-in Cayenne LPP payload formatter, which does not support our custom type 128 (timestamp). Switch to **Custom** formatter and use the JavaScript decoder above.

Enjoyed this article? Head over to [Werktag Blog](https://blog.werktag.io) for more articles.