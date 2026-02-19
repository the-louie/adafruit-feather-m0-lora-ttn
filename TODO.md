[watchdog] AWAKE LIMIT REACHED. Forcing sleep
This should never happen until the device have established connection. This should never everhappen when on usb-power.
```log
09:11:41.499 -> --> Freq: 868500000 Hz, DataRate: 5
09:11:47.833 -> 6019981: EV_JOIN_TXCOMPLETE (Join Request sent, no Accept received)
09:11:47.833 -> --> Native LMIC Opmode state: 0x80C
09:12:51.721 -> 10016857: EV_TXSTART (radio TX started)
09:12:51.721 -> --> Freq: 868100000 Hz, DataRate: 5
09:12:58.020 -> 10410642: EV_JOIN_TXCOMPLETE (Join Request sent, no Accept received)
09:12:58.020 -> --> Native LMIC Opmode state: 0x80C
09:14:07.084 -> 14728233: EV_TXSTART (radio TX started)
09:14:07.084 -> --> Freq: 868500000 Hz, DataRate: 4
09:14:13.439 -> 15125233: EV_JOIN_TXCOMPLETE (Join Request sent, no Accept received)
09:14:13.439 -> --> Native LMIC Opmode state: 0x80C
09:15:23.497 -> [watchdog] AWAKE LIMIT REACHED. Forcing sleep.
09:15:23.533 -> [sleep] do_sleep entry
09:15:23.533 -> [sleep] Standby start @ millis=
09:15:23.533 -> 312155
09:15:23.533 -> [sleep] haveStoredSession=0
09:15:23.533 -> [valid] Bench Mode Guard: Using delay() instead of deepSleep
```