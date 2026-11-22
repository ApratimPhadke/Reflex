# 2026-11-22: second plant, simulated robot joint, on the Pi 4 (QNX 8.0)

| File | What | Command |
|---|---|---|
| `campaign.md/.log` | 8/8 kept with the guardian, limit hit in 3/8 without; closest approach 25.4 deg (stuck) | `./reflex-demo.sh start --plant joint; ./rxctl reset; ./rxctl campaign joint-campaign.md` |
| `bench.md/.csv/.png` | AI 0.63 s vs fallback 0.84 s mean settling, RMS 164.7 vs 177.6 mrad, 0 interventions | `./rxctl bench joint-bench.md --csv joint-bench.csv` |
| `latency.txt` | guardian wake-up latency at 1 kHz over the campaign and benchmark: max 31.7 us over 313359 ticks, 0 late | `./rxctl latency` |
| `start.txt`, `status-*.txt`, `events.txt`, `env.txt` | `reflex-demo.sh` picked the simulator (no I/O node; the joint is simulation only), guardian under HAM | |
