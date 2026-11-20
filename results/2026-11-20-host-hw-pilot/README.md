# 2026-11-20: hardware-trained pilot (Linux host, offline + fake I/O node)

Why: on the Pi 4 the simulator-trained frozen pilot was vetoed at nearly every setpoint step under
the serial backend's `rx_cfg_hardware()` and was quarantined about 40 s after the campaign
(`results/2026-11-20-pi4-emulated-io/`, caveats). It was trained at 250 Hz with the simulator config.

| File | What | Command |
|---|---|---|
| `00-train-hardware.txt` | training log; weights reproduce byte for byte; held-out eval (64 episodes x 60 s): old sim pilot under the hardware model 11.38 vetoes/min; hardware pilot 0.00 vetoes/min, cost 0.1701; the fallback law flown as a pilot 0.2187 (the hardware pilot tracks 22% better) | `python3 tools/train_policy.py --hardware`; `python3 -I eval_hw.py tools src/core/policy_weights.h rx_pol`; `python3 -I cmp_hw.py tools eval_hw.py src/core/policy_weights_hw.h` |
| `01-rx_simtest-hw-section.txt` | new `rx_simtest` section (50 Hz, `rx_cfg_hardware()`, +/-3 mm uniform sensor noise): all faults kept outside the 1 cm strip; healthy 120 s of +/-80 mm steps: hardware pilot 0 interventions, 0 strike-outs; sim pilot 29 interventions, 9 strike-outs | `make test` |
| `02-min-edge-random-phase-hw-pilot.txt` | fake-node ground truth, fault at a random setpoint phase: stuck 19.9-24.9 mm, flip 21.2-50.8 mm (old pilot, same script, `../2026-11-20-host-fake-io/min-edge-truth.txt`: stuck 13.2-26.2, flip 17.1-52.1) | `bash results/2026-11-20-host-fake-io/min-edge-truth.sh` |
| `03-min-edge-campaign-phase-hw-pilot.txt` | fake-node ground truth, fault injected at the campaign's phase (1 s after the setpoint goes positive): **stuck 10.0-14.1 mm**, flip 17.8-30.2 mm | `FAULTS="stuck flip" ./min-edge-phase.sh $PWD` |
| `04-min-edge-campaign-phase-sim-pilot.txt` | same for the old simulator pilot (`simpilot.sh` drops `--hardware`): stuck 11.5-17.1 mm | `FAULTS=stuck` with `--pilot simpilot.sh` |
| `05-test-serial.txt` | `make test-serial` with the hardware pilot: all checks pass, ball-lost inference = ground truth (13) | `make test-serial` |

Finding (not changed, needs data from the real rig): stuck-at-max is the adversary the recoverable-set
margin is sized for, so with a perfect model it would stop exactly at the 10 mm keep-out line. On the
fake node the closest true approach is 10.0 mm with the hardware pilot and 11.5 mm with the sim pilot,
i.e. about 0-5 mm of slack. The campaign's "closest to beam end" on the serial backend is the
guardian's noisy estimate (0.2 cm for stuck on both the fake node and the Pi); the ground truth above is
what to compare against the strip. `rx_cfg_hardware().t_delay` (0.12 s) should be checked against the
measured sensor + servo delay of the real rig before relying on this margin.
