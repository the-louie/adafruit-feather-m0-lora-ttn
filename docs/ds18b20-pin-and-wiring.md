# DS18B20 one-wire temperature sensor — pin and wiring

This document describes how to connect a DS18B20 one-wire temperature sensor to the Adafruit Feather M0 LoRa used in the m0-lorawan-ttn project.

## Pin choice

The sketch and board use:

- **LoRa radio:** NSS = 8, DIO0 = 3, DIO1 = 6, Reset = 4
- **Battery sense:** A7 (Pin 9)
- **LED:** Pin 13

So pins **3, 4, 6, 8, 9** are not free for the DS18B20 data line.

**Recommended data pin: Pin 5.** It is a free digital GPIO with no conflict with LoRa, SPI, I2C, or battery.

**Alternatives:** Pin 10, Pin 11, Pin 12 — same reasoning; use one of these if Pin 5 is needed for something else.

Pinout reference: [Adafruit Feather M0 LoRa pinouts](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/pinouts).

## Wiring

- **DS18B20 data** → **Pin 5** (or 10, 11, 12)
- **4.7 kΩ pullup** from the data line to **3V**
- **DS18B20 GND** → **GND**
- **DS18B20 VDD** → **3V** (3.0–5.5 V supported; 3.3 V matches this board)

Use a single 4.7 kΩ resistor between the one-wire data line and 3V. Keep the wiring short to avoid noise.

## Integration

The sketch reads the DS18B20 on pin 5 via DallasTemperature/OneWire and sends temperature on Cayenne LPP channel 2 as one byte (Digital Input): 0 = error (e.g. sensor disconnected), 1 = under-range (&lt;0°C), 2 = over-range (&gt;30°C), 3–255 = data window with ~0.118°C per step, encoded as `(Temperature×8.43)+3` truncated. See the project Readme for the payload decoder (`temperature_c` or `temperature_state` on ch2) and library install (Dallas Temperature, OneWire).
