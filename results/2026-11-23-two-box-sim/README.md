# Two boxes: guardian on the Pi 4, AI pilot on a laptop over Wi-Fi (simulated beam)

Run 2026-11-23, 18:49-19:04 IST, commit `c1aea25` (clean tree; deployed with `scripts/deploy.sh`), driven by `scripts/two-box-sim.sh` phase by phase. Environment, `uname -a` of both machines and the exact commands are in `env.txt`, with a timeline in `run.log`.

- **Guardian:** Pi 4, QNX 8.0 QSTI, `./reflex-guardian --fresh --hz 50 --conf two-box.conf`, simulator backend, 50 Hz, run as `qnxuser` (`mlockall` not permitted, see `guardian.log`). `two-box.conf` has `pilot_transport = udp`, `pilot_peer` = the laptop, `guest_timeout_s = 1.0`, `pilot_max_age_ms = 100`. The rig's own `reflex.conf` was not changed.
- **Pilot:** laptop (Ubuntu 24.04, Wi-Fi, power save on), static `reflex-pilot --udp <pi>:47800 --hz 50`. It runs in a loop that restarts it 2 s after it exits, standing in for a guest's init. The Pi cannot restart a laptop, so there is no `guest_restart_cmd`, and "guest restart command issued" events are logged only.
- **Stale timeout:** 100 ms = 5 periods, from the Wi-Fi p99 round trip of 65.4 ms (`results/2026-11-23-wifi-link/`). The reasoning is in `docs/ARCHITECTURE.md`, "Stale timeout over a network". The guardian printed `stale after 5 ticks (100 ms) from pilot_max_age_ms`. No other limit or margin was changed.
- **Checks:** every phase is judged on the simulator's ground truth, every tick (`<phase>.csv`, recorded on the Pi with `rxctl log`). PASS needs no ball lost and the ball never inside the 10 mm keep-out strip at either beam end (half length 200 mm). Analysis: `<phase>.txt`. Guardian events: `<phase>-events.txt`.

| Phase | Duration | What was done | AI in control | Fallback ticks (reason) | Closest to a beam end | Balls lost | Result |
|---|---|---|---|---|---|---|---|
| 1 normal | 120 s | nothing | 93.9 % | stale 23, envelope 7, hold 338 | 117.9 mm | 0 | PASS |
| 2 kill | 40 s | `kill -9` of the laptop pilot at +10 s | 92.6 % | no-pilot 49, stale 48, envelope 1, hold 50 | 117.6 mm | 0 | PASS |
| 3 Wi-Fi off | 240 s | laptop Wi-Fi switched off by hand, then on | 78.1 % | no-pilot 2344, stale 48, envelope 7, hold 225 | 117.7 mm | 0 | PASS |
| 4 flip | 30 s | pilot output sign-inverted from +10 s to +20 s (`rxctl inject flip`) | 76.6 % | envelope 122, hold 229 | 30.9 mm | 0 | PASS |
| 4a flip, limiter exhausted | 30 s | same, right after phase 3 (see below) | 37.4 % | quarantine 861, envelope 28, hold 50 | 38.3 mm | 0 | PASS |
| 5 learning | 300 s | pilot restarted with `--learn` (untrained network), `rxctl train on` | 88.9 % | envelope 209, stale 23, hold 1427 | 36.6 mm | 0 | PASS |

Over the whole run the guardian had 0 balls lost, 84 interventions, worst wake-up latency 46.7 us, worst execution time per tick 41.6 us and 0 deadline misses (`final-status.txt`, QNX on the Pi). It accepted 40 052 datagrams and refused none (`wrong-peer=0 malformed=0 replayed=0`).

## What happened in each phase

**2, kill -9.** The proposals went stale, and the guardian fell back within 100 ms. GUEST DOWN came 1020 ms after the last valid proposal. The loop started a new pilot instance 2 s after the kill, then GUEST UP, and the AI was back in control 0.5 s later (`kill-events.txt`).

**3, Wi-Fi off.** The radio was off for **43 s, not 10 s**. NetworkManager logged the radio disabled at 18:53:14.31 and enabled at 18:53:57.25, and the link was re-associated with the same address at 18:54:01.57 (`wifi-networkmanager.txt`). The guardian saw the guest down for 46.9 s. It flew the fallback the whole time, issued 4 rate-limited restart commands 15 s apart (log only), logged GUEST UP when datagrams returned, and gave the AI control back 0.5 s later. The laptop pilot did not exit while the network was down. In this run the laptop-side script hung on the ssh that started the Pi recording, so it did not log the off/on times itself; the times above come from NetworkManager's journal. The script has been fixed since this run.

**4a, flip with the restart limiter exhausted.** The kill and the long Wi-Fi outage had used all 5 restarts allowed in 10 minutes. The flipped pilot's three envelope strikes asked for a sixth, so the guardian set **GUEST_QUARANTINED** and flew the fallback until an operator `rxctl release`. That is the designed behaviour. Files: `flip-quarantined-*`.

**4, flip after `rxctl release`.** The release cleared the limiter history. Same fault: the guardian took over at each sign-inverted command the envelope refused, the strikes requested one restart (log only), and the AI flew again after `rxctl clear`.

**5, learning.** The pilot started from an untrained network; the learner ran 76 episodes. Per minute (`learn-trend.txt`):

| Minute | AI in control | Envelope takeovers | Closest to a beam end |
|---|---|---|---|
| 1 | 77.5 % | 15 | 42.4 mm |
| 2 | 82.8 % | 17 | 36.6 mm |
| 3 | 91.6 % | 9 | 39.2 mm |
| 4 | 95.7 % | 4 | 48.3 mm |
| 5 | 97.1 % | 3 | 80.9 mm |

## Limits of this run

- The beam is simulated on the Pi. No servo or sensor was involved.
- The guardian ran without `mlockall` (it ran as qnxuser, not root).
- The laptop's Wi-Fi power save was on. The stale timeout comes from one 5-minute link measurement in one room.
- The phases ran one after the other on one guardian instance; counters are cumulative and are compared before and after each phase.
