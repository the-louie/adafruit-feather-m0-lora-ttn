# Join reliability: Serial blocking and RX windows

**Date:** 2026-01-27

## Symptom

Device log showed EV_JOINING → EV_TXSTART → EV_JOIN_TXCOMPLETE repeatedly and never EV_JOINED, while TTN and the gateway showed successful joins and decoded Cayenne LPP. The network was sending Join Accept; the device was not receiving or processing it in time.

## Cause

LMIC must open RX1 (~1 s) and RX2 (~2 s) after the end of Join Request TX. During that window, `os_runloop_once()` must be called frequently so the radio can open and decode the Join Accept. Serial output in `onEvent` (e.g. timestamp + event string) can block long enough to starve the runloop and miss the downlink.

## Change

`onEvent` now suppresses Serial for all events between EV_JOINING and EV_JOINED/EV_JOIN_FAILED. Only EV_JOINED and EV_JOIN_FAILED are printed during that phase, so the runloop stays responsive for RX windows.

## References

- TTN forum: Feather M0 EV_JOIN_TXCOMPLETE / no Join Accept (blocking code in loop).
- LMIC: `os_runloop_once()` must run without blocking; avoid Serial or other blocking work during join/RX.
