# The Pi 4's unexplained reboots on 20 Nov 2026: what the data shows

Sources: `../2026-11-20-pi4-emulated-io/12-health.log` and `../2026-11-20-pi4-bench/health-log-until-reboot-3.log`
(`scripts/target/health-log.sh`: `vcgencmd get_throttled`, `measure_temp`, `measure_volts core` every 2 s, synced to disk).

| Event | Boot time (UTC) | What was running | Health log |
|---|---|---|---|
| 1 | 07:00:05 | guardian + pilot over USB serial to the Black Pill (emulator), idle after the campaign | not running yet |
| 2 | 07:42:04 | guardian `--learn-from-sim` over USB serial, 1.5 min in | not running yet |
| (gap) | 08:29 -> 11:52:05 | - | log stops 08:28:59 with a clean sample; most likely the Pi was switched off (not confirmed) |
| 3 | 12:59:03 | demo (guardian under HAM) over USB serial | **last sample 12:35:50, 23 min before the boot** |

Facts from 2043 samples (07:53-12:35 UTC): `throttled=0x0` in every sample (no under-voltage, no capping,
no throttling, no soft temperature limit), SoC 44.8-54.0 C, core 0.84 V. After event 3 the Black Pill did not
enumerate on USB at all (`usb` listed no 0483 device) until it was replugged.

**Most likely cause: a hang involving the USB serial path, not heat and not sustained under-voltage.**
- Heat: ruled out (54 C max, never throttled).
- Sustained under-voltage: ruled out at 2 s resolution right up to 12:35:50. A dip shorter than 2 s cannot be
  excluded, and the firmware's flags reset at every boot; but a brownout resets the Pi at once, and event 3 shows
  23 minutes of silence before the next boot, which looks like a hang followed by a power cycle.
- All three events happened while the guardian was talking to the Black Pill through `devc-serusb` (a driver
  copied from the SDP onto an image that does not ship it), and after event 3 the board was gone from the bus.
  No event happened in the hours of simulated-plant runs before or since (2026-11-19, 2026-11-22).

This is an inference from coincidence, not a diagnosis. To pin it down:
1. Tell us whether the Pi was power-cycled by hand at 08:29-11:52 and around 12:59 UTC on 20 Nov.
2. Capture the console: a 3.3 V USB-UART on the Pi's GPIO UART shows kernel messages that die with a hang.
3. Soak test: guardian over USB serial for 2 h with the Black Pill on a powered USB hub, then the same with
   `--backend sim` only. If only the first fails, it is the USB serial path.
4. Keep `health-log.sh` running, at 0.5 s if it helps, and use the official 5.1 V supply.
