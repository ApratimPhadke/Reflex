# Wi-Fi link, laptop <-> Pi 4, UDP round trip at the pilot's rate

Measured 2026-11-23 with `rx_udpping` at commit `2dfac91` (clean tree). No guardian or pilot was running; no settings were changed.

- Pi 4 (QNX 8.0 QSTI, `bcm0` Wi-Fi, 10.203.87.190) plays the guardian: one 52-byte probe (size of the state datagram) every 20 ms (50 Hz, the hardware control rate) from UDP 47800, timestamped on the Pi's monotonic clock.
- Laptop (Ubuntu 24.04, `wlp0s20f3`, 10.203.87.207, 5745 MHz, -47 dBm, VHT-MCS 9; **Wi-Fi power save on**) plays the pilot: `rx_udpping echo 47801` answers with 48 bytes (size of a proposal).
- 300 s, 15 000 probes. Commands: `command.txt`. Per-probe data: `rtt.csv`. Tool output: `summary.txt`. Bursts: `bursts.txt`. Environment and `uname -a`: `env.txt`.

| | |
|---|---|
| RTT min / p50 / p90 | 1.98 / 3.32 / 5.31 ms |
| RTT p99 | 65.4 ms |
| RTT p99.9 | 173.9 ms |
| RTT max | 244.5 ms |
| Lost | 6 of 15 000 (0.040 %), one burst of 6 probes (120 ms) |
| Replies later than one period (20 ms) | 527 (3.5 %) |
| Replies later than the guardian's `max_age_s` (2.5 periods = 50 ms) | 198 (1.3 %) |
| Longest run of probes late (> 50 ms) or lost | 16 consecutive (320 ms) |
| Longest run late (> 20 ms) or lost | 29 consecutive (580 ms) |

The tail is not spread evenly: in 10 s windows, the probes > 50 ms or lost were `2 0 0 0 0 0 0 0 0 0 0 87 0 0 0 20 0 0 0 0 0 0 0 23 29 27 14 2 0 0`. Most of the run is clean; a few windows have bursts of hundreds of ms. The cause (power save, interference, scans) was not investigated.

What this means for the pilot over Wi-Fi (inference, not measured with the guardian running): the median round trip fits easily in one 20 ms period, but about 1.3 % of proposals would arrive after the 50 ms staleness deadline and be refused as stale, with the fallback flying for stretches of up to about 0.3 s. The longest outage in this sample (320 ms) is under the default `guest_timeout_s` of 1.0 s.
