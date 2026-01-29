/*
 * LoRaWAN OTAA credentials. Replace placeholders with values from TTN Console.
 * Do not commit this file with real keys to a public repo.
 * EU868 / TTN. LMIC expects AppEUI/DevEUI in little-endian; AppKey as from TTN (MSB).
 */
#ifndef M0_LORAWAN_TTN_APP_DEVICE_CONFIG_H
#define M0_LORAWAN_TTN_APP_DEVICE_CONFIG_H

/* AppEUI: 8 bytes, little-endian (LSB first). Paste from TTN as LE or convert MSB hex to LE. */
#define LORAWAN_APP_EUI_LE  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

/* DevEUI: 8 bytes, little-endian (LSB first). Paste from TTN as LE or convert MSB hex to LE. */
#define LORAWAN_DEV_EUI_LE  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

/* Session tag: DevEUI as uint64_t MSB (same as TTN hex). Must match LORAWAN_DEV_EUI_LE. */
#define LORAWAN_DEV_EUI  0x0000000000000000ULL

/* AppKey: 16 bytes, MSB (copy from TTN as-is). */
#define LORAWAN_APP_KEY_ARR  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

#endif
