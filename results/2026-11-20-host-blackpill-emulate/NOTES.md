# 2026-11-20: Linux host guardian <-> Black Pill emulator over USB CDC

Target: Linux host (see env.txt for `uname -a` and git hash), I/O node = Black Pill running
`firmware/reflex_io` with EMULATE=1 (plant simulated on the MCU, no sensor, no servo).
Chip read over SWD: STM32F411 rev A (IDCODE 0x10006431), flash-size register 256 KB,
board sold as F401CCU6. Firmware built with BOARD=BLACKPILL_F401CC, flashed with st-flash over ST-LINK/V2.

Commands (in order):
    stty -F /dev/ttyACM0 raw -echo 115200; timeout 3 cat /dev/ttyACM0     # raw check: 150 "P" lines / 3 s = 50 Hz
    ./build/linux/reflex-guardian --backend serial --dev /dev/ttyACM0 --fresh
    ./build/linux/rxctl reset; ./build/linux/rxctl campaign results/2026-11-20-host-blackpill-emulate/campaign.md

Result (campaign.md): ball kept in 8 of 8 guarded fault runs; lost in 7 of 8 without the guardian.
Takeover 14.4-18.6 ms; wake latency max 678.7 us over 10136 ticks at 50 Hz, 0 late ticks;
guardian WCET 641.7 us. These latency numbers are for a stock Linux host, not QNX.

Caveats:
- No independent ground truth: the emulator keeps no loss counter, so "kept"/"LOST" is the
  guardian's own inference (from "P !" runs). make test-serial cross-checks this against the
  fake node; this run does not.
- "Closest to beam end" is the guardian's estimate including +/-3 mm emulated sensor noise.
  Stuck-at-max: 1.4 cm (keep-out strip 10 mm). Consistent with the 13.2 mm seen on the fake node.
- balls lost = 16 in status.txt are all from bypass runs (each lost run can lose the ball
  several times in 4 s, since the emulator re-centres it after 1 s).
- events.txt is the 40-entry ring buffer at the end, not the full log.
