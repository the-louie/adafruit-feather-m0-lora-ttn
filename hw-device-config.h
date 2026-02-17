/*
 * Hardware pin assignments for Adafruit Feather M0 LoRa.
 * One-wire (DS18B20) and run-mode strapping pins. Change these if you use different pins.
 * Copyright (c) 2026 the_louie
 */
#ifndef M0_LORAWAN_TTN_HW_DEVICE_CONFIG_H
#define M0_LORAWAN_TTN_HW_DEVICE_CONFIG_H

/* DS18B20 one-wire data pin. 4.7 kΩ pull-up data→3V required. */
#define ONE_WIRE_BUS  5

/* Run-mode strapping: GND on pin = active. Read at cold boot only; then set INPUT for 0 µA. */
#define STRAP_DEV_PIN  11
#define STRAP_TEST_PIN 12

#endif /* M0_LORAWAN_TTN_HW_DEVICE_CONFIG_H */
