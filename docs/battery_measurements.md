# Battery measurements with external battery on VUSB

This guide assumes the Feather M0 LoRa is powered from **three 1.5 V alkaline cells in series** (AA/AAA), connected such that their output (about 3.0–4.8 V, worst-case up to ~5.0 V) appears on **VUSB/VIN** or another point you want to measure. In this configuration the **built-in JST LiPo battery sense does not apply**. The onboard divider is wired to the JST LiPo jack (BAT), not to VUSB/VIN. To measure the external 3×AA/AAA battery voltage you must add a **voltage divider** from the battery (or VIN) to **A7**, and optionally adjust the sketch factor.

## Safety

- **A7 must never exceed 3.3 V.** The SAMD21 ADC is not 5 V tolerant. Use a divider so that the voltage at A7 is always ≤ 3.3 V for the maximum battery voltage you expect.
- Use a **common GND** between the battery and the Feather (battery negative to Feather GND).

## Circuit

Connect a two-resistor divider between the voltage you want to measure and A7:

```
  Battery + (or VIN)  ----[ R1 ]----+---- A7 (Feather)
                                    |
                                  [ R2 ]
                                    |
                                   GND  (Battery − and Feather GND)
```

- **R1**: between battery positive (or VIN) and A7.
- **R2**: between A7 and GND.
- **A7**: the junction of R1 and R2 goes to the Feather A7 pin.

Voltage at A7:

`V_A7 = V_battery × R2 / (R1 + R2)`

So:

`V_battery = V_A7 × (R1 + R2) / R2`

The sketch converts the ADC reading to voltage as:

`V_battery = (ADC × 3.3 / 1024) × divider_factor`

So **divider_factor = (R1 + R2) / R2**. The default sketch uses `2.0f` (built-in 2:1 divider). For your divider you must use your actual ratio.

## Resistor values

Use 1% resistors if you want the best accuracy. Standard E24 values are fine.

### 3 × 1.5 V alkaline (AA/AAA, up to ~4.8–5.0 V)

A fresh 3×AA/AAA alkaline pack is typically around 4.5–4.8 V, with an upper bound close to 5.0 V. Choose a ratio so ~5.0 V → ≤ 3.3 V at A7.

- **R1 = 100 kΩ**, **R2 = 200 kΩ**  
  Ratio = (100 + 200) / 200 = **3.0**  
  At 5.0 V: V_A7 ≈ 5.0 / 3 ≈ 1.67 V (safe, well below 3.3 V).  
  At 4.8 V: V_A7 ≈ 4.8 / 3 ≈ 1.60 V.  
  **Sketch factor: 3.0** (use this for 3×alkaline on VUSB)

### 4.2 V max (single LiPo before USB circuitry, optional)

If instead of 3×alkaline you measure a raw LiPo (about 3.2 V–4.2 V) before any USB boost:

- **R1 = 27 kΩ**, **R2 = 100 kΩ**  
  Ratio = (27 + 100) / 100 = **1.27**  
  At 4.2 V: V_A7 = 4.2 / 1.27 ≈ 3.31 V (acceptable).  
  **Sketch factor: 1.27**

- Or **R1 = 33 kΩ**, **R2 = 100 kΩ**  
  Ratio = 1.33 (max ~4.4 V at 3.3 V A7).  
  **Sketch factor: 1.33**

### 3.7 V nominal (single LiPo, headroom; optional)

- **R1 = 10 kΩ**, **R2 = 100 kΩ**  
  Ratio = 1.1 (max ~4.0 V at 3.3 V A7).  
  **Sketch factor: 1.1**

## Updating the sketch

The battery reading is in `do_send()` in `m0-lorawan-ttn.ino`:

```c
uint16_t vbatCentivolt = (uint16_t)(analogRead(VBATPIN) * (2.0f * 3.3f / 1024.0f) * 100.0f);
```

The `2.0f` is the built-in divider factor (R1+R2)/R2. Replace it with your divider factor so that:

`divider_factor = (R1 + R2) / R2`

Example for **R1 = 100 kΩ, R2 = 200 kΩ** (3×1.5 V alkaline, up to ~5 V):

```c
#define VBAT_DIVIDER_FACTOR  3.0f   /* R1=100k, R2=200k for 5V max */
// ...
uint16_t vbatCentivolt = (uint16_t)(analogRead(VBATPIN) * (VBAT_DIVIDER_FACTOR * 3.3f / 1024.0f) * 100.0f);
```

Example for **R1 = 27 kΩ, R2 = 100 kΩ** (4.2 V max):

```c
#define VBAT_DIVIDER_FACTOR  1.27f
```

## Wiring summary

| Battery / max voltage        | R1     | R2      | Sketch factor |
|-----------------------------|--------|---------|----------------|
| 3×1.5 V alkaline (~4.8–5 V) | 100 kΩ | 200 kΩ  | 3.0            |
| 4.2 V (LiPo, optional)      | 27 kΩ  | 100 kΩ  | 1.27           |
| 4.2 V (LiPo, optional)      | 33 kΩ  | 100 kΩ  | 1.33           |
| 3.7 V nominal (LiPo, opt.)  | 10 kΩ  | 100 kΩ  | 1.1            |

- **Battery + or VIN** → R1 → **A7** → R2 → **GND**
- Battery − and Feather GND must be connected.

## Accuracy notes

- The SAMD21 internal reference is about 3.3 V; tolerance is typically a few percent. For better accuracy you could calibrate against a known voltage.
- Resistor tolerance (e.g. 1%) adds error; 1% parts are recommended for consistent readings.
- Keep the wires from the divider to A7 short to reduce noise.
