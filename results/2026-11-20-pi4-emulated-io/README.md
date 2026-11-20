# 2026-11-20: Pi 4 (QNX 8.0) guardian <-> Black Pill emulator over USB (devc-serusb)

Roadmap D5 and D6. Target and versions in `env.txt` and `uname.txt`.
All target commands run in `~/reflex` as `qnxuser`; the only root step is starting the driver.

| File | What | Command |
|---|---|---|
| `00-rate-5s.txt` | raw stream after draining: 234 `P` lines in 5 s (50 Hz), 0 `P !` | `cat /dev/serusb1` for 2 s (drain), then 5 s |
| `01-backlog-30s-unread.txt` | 409 lines read in 1.5 s after 30 s unread: ~2 KB queued by the driver | `sleep 30; cat /dev/serusb1` for 1.5 s |
| `02-backlog-after-tilt-30s.txt` | after tilting the emulated beam so the ball falls (then re-centres at 0) and 30 s unread, the backlog **starts with `P !`** from the fall: the driver keeps the oldest data when its buffer is full | 400 x `echo "U 150" > /dev/serusb1`, read 1 s, `sleep 30`, read 1 s |
| `flushprobe.c` | probe: open like rx_io.c, optional `tcflush(TCIFLUSH)`, read after 30 ms. After 30 s unread: no flush 2046 bytes / 370 lines; flush 27 and 29 bytes / 5 lines | `qcc -Vgcc_ntoaarch64le`; `sleep 30; ./flushprobe /dev/serusb1 0` (then `1`, twice) |
| `03-nopilot.txt` | fallback only, 15 s: ball held at +76.7 mm (setpoint +80), 0 interventions, wake max 24.0 us, 0 deadline misses | `./reflex-guardian --backend serial --dev /dev/serusb1 --no-pilot --fresh` |
| `04-pilot-and-campaign.txt`, `05-campaign.md/.log`, `05-events.txt` | frozen pilot in AI mode after ~400 ms; fault campaign: **kept 8/8 with the guardian, lost 5/8 without**; takeover 14-18 ms; wake-up max 24.6 us over 10837 ticks, 0 late; guardian WCET 160.3 us | `./reflex-guardian --backend serial --dev /dev/serusb1 --fresh &`, `./rxctl reset; ./rxctl campaign campaign-serial.md` |
| `06-unplug-replug.txt` | D6: cable pulled ~3 s with the AI in control (setpoint period 60 s, `rxctl period 60000`): `link down` logged, `/dev/serusb1` re-created by devc-serusb, `link up` 3.5 s later, AI back 0.7 s after that; same guardian process (`guardian starts 1`), 0 deadline misses, balls-lost count unchanged | `rxctl period 60000; rxctl release`, then the user pulls and replugs the cable |
| `07-hw-pilot-healthy-and-campaign.txt`, `08-campaign-hw.md/.log`, `08-events-hw.txt` | after a Pi reboot (07:00:05 UTC, cause unknown; devc-serusb restarted by hand), the **hardware-trained pilot** (`reflex-pilot ... --hardware`): 120 s healthy at the default +/-80 mm / 4 s setpoint, **0 interventions, 0 kills**; campaign kept 8/8, lost 5/8 without; 60 s after the campaign no further interventions. Stuck-at-max closest-to-end **estimate 0.2 cm**; fake-node ground truth for the same phase is 10.0-14.1 mm (`../2026-11-20-host-hw-pilot/`) | `./reflex-guardian --backend serial --dev /dev/serusb1 --fresh &`, `rxctl period 4000; rxctl reset; sleep 120`, `rxctl campaign campaign-hw.md`, `sleep 60` |

Fix made from this run: `src/guardian/rx_io.c` now calls `tcflush(fd, TCIFLUSH)` after opening the
port (init and helper thread only, never the control loop). Without it the first tick after an
open acts on a position that can be seconds old. Linux `make test` and `make test-serial` pass with it.

Caveats:
- `mlockall` is refused for `qnxuser` (same as the 2026-11-19 runs).
- Ball kept/lost is the guardian's own inference; the emulator keeps no loss counter.
- Closest-to-end is the guardian's estimate with +/-3 mm emulated noise. Stuck-at-max: 1.3 cm
  (keep-out strip 10 mm), consistent with 1.4 cm on the laptop and 13.2 mm on the fake node.
- Without-guardian losses (5/8 here, 7/8 on the laptop at 50 Hz) vary run to run: the
  slow/NaN cases depend on where the ball is when the fault starts.
- The node watchdog could not be observed in the unplug test: the Black Pill is powered from the USB
  cable, so pulling it also resets the board (make test-serial covers the watchdog with the fake node).
- With the frozen simulator pilot at 50 Hz, every setpoint step is vetoed (envelope); 3 strikes kill it and
  3 kills quarantine it (after the campaign: kills at ticks 11464, 12079, 12678, takeovers 4 s apart = the
  setpoint half-period). The pilot was trained at 250 Hz with the simulator config, not the hardware config.
- devc-serusb is not in the QSTI image and was started by hand; it does not survive a reboot.
