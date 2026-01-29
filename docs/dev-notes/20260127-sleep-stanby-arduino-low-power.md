# Sleep: STANDBY via Arduino Low Power (colleague feedback)

**Date:** 2026-01-27

## Colleague notes (summary)

- SAMD21 has **IDLE** (~3–5 mA, CPU stop, peripherals on) and **STANDBY** (~5–15 µA, only RTC/WDT on). For battery we want STANDBY.
- STANDBY is entered by setting SLEEPDEEP and executing WFI; Arduino Low Power handles clock re-init and USB.
- **USB stays on by default (~10 mA)**; for true deep sleep you must detach it (`USBDevice.detach()` before sleep).
- Wake sources: external interrupts (EIC), RTC alarm, WDT.

## Changes made

1. **RTCZero → Arduino Low Power**  
   `do_sleep` now uses `LowPower.deepSleep((uint32_t)SLEEP_SECONDS * 1000UL)` instead of RTCZero `standbyMode()`. Arduino Low Power handles STANDBY, RTC-based wake, and system clock re-initialisation.  
   **Library:** Install “Arduino Low Power” from Library Manager.

2. **Wake behaviour**  
   After `deepSleep(ms)` the MCU resumes execution (no reset). So on wake we set `wakeCounter = persistentData.wakeCounter` and call `do_send(&sendjob)` to run the next TX. The next wake index is still persisted in EV_TXCOMPLETE (and at the start of `do_sleep`) so it survives any future reset.

3. **Optional USB detach**  
   For lowest power, USB should be detached before sleep. A define `DETACH_USB_BEFORE_SLEEP` (default 0) guards calls to `USBDevice.detach()` / `attach()`. Set it to 1 for production/low-power builds only if your board/core provides `USBDevice` (e.g. older Arduino SAMD core with `#include <USB/USBDevice.h>`). Newer cores that use TinyUSB may not offer this API.

## References

- Colleague: SAMD21 IDLE vs STANDBY, SLEEPDEEP/WFI, Arduino Low Power, USB detach.
- Arduino Low Power: `deepSleep(ms)`, RTC wake, resume after wake (no reset).
- FlashStorage and wake-counter persist unchanged (see 20260127-sleep-wake-counter-persist.md).
