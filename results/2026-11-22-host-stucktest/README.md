# 2026-11-22: `rxctl stuck` - the hardware-day margin check, tried on the simulator and the fake I/O node

`rxctl stuck [N] [file.md]` injects the stuck-at-full-tilt fault N times with the guardian ON, each 1 s after the
setpoint turns positive, and reports the closest approach to the end against the keep-out strip, with PASS/FAIL.

| File | What | Command |
|---|---|---|
| `sim-beam.md` | simulated beam, ground truth: closest 30.8 mm, **PASS** | `reflex-guardian --fresh &; rxctl stuck 3 sim-beam.md` |
| `fake-node.md`, `fake-status.json` | fake I/O node (serial, 50 Hz, hardware pilot after item 1): guardian's estimate 4.1 and -1.2 mm in runs 2-3, **FAIL**; the fake node's ground truth for the session: **4.3 mm, i.e. inside the 10 mm strip** (ball kept) | fake node with the test-serial calibration; `reflex-guardian --backend serial --conf ...`; `rxctl stuck 3 fake-node.md` |
| `fake-node-truth-per-run.txt`, `fake-node-truth-20runs.txt` | the same, ground truth per run (SIGHUP resets the fake node's window): 25 runs, ground truth **12.6-27.5 mm**, none inside the strip; the estimate read 4.4-7.5 mm closer than the truth | `kill -HUP <fake>; rxctl stuck 1` x 5 and x 20 |
| `fake-node-truth-hw-pilot-v2.txt` | aborted: the 20 Nov script starts the guardian without reflex.conf, which protocol v2 refuses | - |

**Finding, not acted on (a safety-margin decision):** in 28 stuck runs on the fake node one entered the keep-out
strip (4.3 mm from the end). On 20 Nov, with the earlier hardware pilot, 5 runs gave 10.0-14.1 mm. The guardian's
bound with `t_delay = 0.12 s` therefore does not fully cover the fake node's delay chain in every case. The ball
was kept every time. Raising `t_delay` (or `edge_margin`) in `rx_cfg_hardware()` is the fix to discuss with data
from the real beam; nothing was changed. On the real beam `rxctl stuck` only has the estimate, which on the fake
node is pessimistic by 4-8 mm, so a FAIL there needs the side-on video to judge.
