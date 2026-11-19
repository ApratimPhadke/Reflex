# Phase 2: Reflex on the Raspberry Pi 4, simulated plant (2026-11-19)

Target: `QNX qnxpi 8.0.0 2026/06/05-16:21:14EDT RaspberryPi4B aarch64le` (quick-start image), see `env.txt`.
Binaries: `PI=qnxpi make deploy` from the host. Builds are named by the commit that contains them.
All commands were run on the target in `~/reflex` (= `/data/home/qnxuser/reflex`) as `qnxuser`; root steps via `sudo` (passwordless on the QSTI).

| File | Step | Build | What it shows |
|---|---|---|---|
| `01-rx_simtest.log` | 1 | c572b96 | `./rx_simtest`: SIMTEST PASSED on the Pi (3 min 14 s) |
| `02-guardian-1khz.txt`, `02-rxdash-api-state.json` | 2 | c572b96 + timer fix | `./reflex-guardian --fresh`, `rxctl status/latency/events`; `rxdash` JSON fetched from the laptop (`curl http://qnxpi.local:8080/api/state`) |
| `03-campaign.md`, `03-campaign-final-status.txt` | 3 | 5794766 | `./rxctl reset; ./rxctl campaign campaign.md` at 1 kHz: kept 8/8 with the guardian, lost 4/8 without |
| `04-pidin.txt` | 4 | timer fix + spawner | guardian 50f RECEIVE, spawner thread 40f, pilot 10r REPLY-blocked on the guardian pid |
| `05-respawn-and-spoof.txt` | 5 | 5794766 (without SIGPIPE fix) | 5x `inject crash`, then `./reflex-pilot $(./rxctl endpoint)`: 5 restarts, 1 spoof refused, exec max 58.3 us |
| `06-sporadic.txt` | 6 | 5794766 | `--sporadic` A/B of `sched_ss_max_repl` and the spin fault |
| `07-rxfs.txt` | 7 | c572b96 + timer fix | `cat /dev/reflex`, `echo "fault nan" > /dev/reflex`, invalid command -> EINVAL |
| `08-ham.txt` | 8 | 5794766 | HAM restart after `sudo slay -s KILL reflex-guardian`, with `ham`'s stdout deliberately broken: exactly one restart, ball kept |
| `09-learning-10min.txt` | 9 | 5794766 | `--fresh --hz 250 --learn`, `rxctl period 2000`, 10 min, learner killed at 5:00: 0 balls lost, resumed at episode 150 |
| `10-trace.txt` | 10 | 5794766 | `tracelogger` + host `traceprinter`: guardian `trace_logf` user events (code 42) present |

## Bugs found and fixed in this phase (see commit 5794766)

Before/after numbers for the first three come from runs during debugging whose raw logs were not kept; the "after" numbers are in the files above.

1. **Latency reference wrong on QNX.** Timer armed relative to "now", while QNX expiries land on 1 ms tick boundaries: every tick read about 612 us late (`deadline-misses=4011` of 4061 ticks), with real jitter of about 7 us. Fixed by arming a tick-aligned absolute first expiry. After: p50 < 8 us, max 25.6 us at 1 kHz (`03-campaign.md`).
2. **`posix_spawn` inside the control tick.** A respawn tick took 783 us (median tick 3 us); the first campaign reported a worst-case exec time of 951.1 us. Moved to a helper thread. After: 40.4 us (`03-campaign.md`).
3. **`SCHED_SPORADIC` replenishment limit.** A healthy pilot ran at priority 1 in 35/100 samples (`06-sporadic.txt`).
4. **HAM restart loop.** A HAM-restarted guardian with a dead stderr died of SIGPIPE on its first line: `guardian starts` went from 3 to 1759. Fixed by ignoring SIGPIPE first (`08-ham.txt` reproduces the condition).
5. `rxham` needed `libham.so.2`, and `ham` itself is not on the image: libham is now linked statically and `make deploy` copies `ham`.

## Observations, not verified further

- `mlockall` fails with EPERM as `qnxuser` (the guardian continues). Not investigated: whether QNX 8.0 needs `PROCMGR_AID_MEM_LOCK` or locks memory by default.
- Without the guardian, the `spin` fault was kept in one campaign run and lost in the other: with the guardian bypassed, a hung pilot leaves its last command on the actuator, so the outcome depends on timing.
- Learning is noisy: smoothed cost 1.98 at 30 s, between 0.70 and 1.46 after 150 s, 0.81 at 600 s. Interventions per 30 s fell from about 14 to 6-10 but did not reach zero in 10 minutes.
