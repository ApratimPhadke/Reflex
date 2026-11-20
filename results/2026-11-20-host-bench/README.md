# 2026-11-20: is the AI worth having? (`rxctl bench`, Linux host)

`rxctl bench` flies the same setpoint sequence (+80, -80, +40, -40, 0 mm, 5 s each) twice: with the AI
pilot in charge (guardian on as always), then on the fallback controller alone (`rxctl fallback on`).
Settling = last time outside a band of 10 % of the step (at least 5 mm). Host: see `../2026-11-20-host-hw-pilot/env.txt`.

| File | What | Command |
|---|---|---|
| `01-before-sim.md/.csv/.png` | simulator, 1 kHz, the frozen pilot as it was: AI settles faster (0.76 s vs 1.68 s) but **stops short of every setpoint** (final error -10.2, +11.4, -4.5, +6.4 mm; fallback within 0.2 mm), one step never settles | `reflex-guardian --fresh &`, `rxctl bench 01.md --csv 01.csv` |
| `02-before-fake.md/.csv` | fake I/O node at 50 Hz with the hardware-trained pilot as it was: **not better** - 3 of 5 steps not settled, settling 2.77 s vs 1.82 s, final error -18.3, +19.1, -9.0, +9.7 mm | fake node with the test-serial calibration, `reflex-guardian --backend serial --conf ...`, `rxctl bench` |
| `03-after-sim.md/.csv/.png` | after the fix: AI 0.67 s vs fallback 1.68 s mean settling (-60 %), RMS 24.9 vs 32.8 mm (-24 %), 0 interventions, all steps settled, final error 0.0-0.1 mm | as 01 |
| `04-after-fake.md/.csv` | after the fix, 50 Hz hardware configuration: AI 1.44 s vs 1.87 s (-23 %), RMS 29.2 vs 31.8 mm (-8 %), all steps settled. A modest gain: the hardware envelope (t_delay 0.12 s) caps the ball speed near 0.34 m/s at the centre, so the AI cannot be much faster without being vetoed | as 02 |
| `05-train-*.log` | retraining logs. Sim pilot eval cost 0.1041 (was 0.0981); hardware pilot 0.1835, 0.02 vetoes/min (was 0.1701, 0.00). 80 CEM iterations instead of 40: 0.1830, 0.05 vetoes/min, not kept | `python3 tools/train_policy.py`, `--hardware`, `--hardware --iters 80` |
| `learner-seeds-lr0.02.txt`, `learner-seeds-lr0.0075.txt`, `learnseeds.c` | rx_simtest's learning test (15 simulated min from an untrained network) over 48 seeds: learning rate 0.02 failed on **11 of 48**, 0.0075 on **0 of 48** (median best cost 0.315 vs 0.460) | `gcc ... learnseeds.c src/core/rx_core.c src/core/rx_learner.c -DLR0=...; ./learnseeds 48` |
| `final_error.py` | mean error over the last second of every step, from a bench CSV | `python3 final_error.py bench.csv` |

What changed (pilot side only; guardian, envelope and margins untouched):
1. **Equilibrium-corrected policy** (`rx_policy_with`, `tools/train_policy.py`): u = net(e, v, x) - net(0, 0, x), so the
   command is exactly zero with the ball at rest on the setpoint. Both frozen pilots retrained in this form.
2. **Learner**: `--learn-from-sim` uses the same corrected form; learning from scratch keeps the free form (it reached a
   lower best cost). Its learning rate went from 0.02 to 0.0075 (the seed study above). `make test` failed on the
   learning test with the first change; the cause was that the test was one draw of a learner that failed 23 % of the time.
