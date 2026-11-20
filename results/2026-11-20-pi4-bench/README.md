# 2026-11-20: `rxctl bench` on the Pi 4 (QNX 8.0)

| File | What | Command |
|---|---|---|
| `bench-pi-sim.md/.csv/.png` | simulated plant at 1 kHz on the Pi: AI 0.68 s vs fallback 1.68 s mean settling, RMS 25.0 vs 32.7 mm, 0 interventions, all steps settled, final errors 0.0-0.2 mm | `./reflex-demo.sh start --sim; ./rxctl bench bench-pi-sim.md --csv bench-pi-sim.csv` |
| `health-log-until-reboot-3.log` | the Pi rebooted by itself a third time (up again 12:59:03 UTC). The log's last line is 12:35:50 UTC, throttled=0x0, 52 C, with the demo running on the serial backend. After the reboot the Black Pill did not enumerate on USB at all | `health-log.sh` |

**Not run:** the same benchmark on the emulated board over USB, because the Black Pill was no longer on the Pi's USB bus
after the reboot. The fake-node run (`../2026-11-20-host-bench/04-after-fake.md`) is the 50 Hz hardware-configuration result until then.
