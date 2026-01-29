// LoRaWAN region (e.g. LORAMAC_REGION_EU868, LORAMAC_REGION_US915)
#define LORAWAN_REGION          LORAMAC_REGION_EU868

// LoRaWAN Device EUI (64-bit), NULL uses flash unique ID. Must match TTN/ChirpStack device.
#define LORAWAN_DEVICE_EUI      "70B3D57ED007560E"

// LoRaWAN Application / Join EUI (64-bit). Must match application in console (e.g. 0000000000000000).
#define LORAWAN_APP_EUI         "0000000000000000"

// LoRaWAN Application Key (128-bit). Must match the device App Key in TTN/ChirpStack.
#define LORAWAN_APP_KEY         "72A530B5B0E7C30C652D66AD6DA92CD5"

// Channel mask for region, NULL uses default
#define LORAWAN_CHANNEL_MASK    "00FF00000000000000000000"