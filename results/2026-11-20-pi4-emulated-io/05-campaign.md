# Reflex fault campaign

- backend: `serial`, control rate 50 Hz
- each fault is injected into a healthy system and left active for 4 s
- *without Reflex* = the same pilot, same faults, guardian in bypass (pilot commands go straight to the actuator)

- *closest to beam end* is the guardian's **estimate** (no ground truth on this backend; it includes sensor noise)

| # | Injected pilot fault | Guardian took over after | Closest to beam end | Ball (with Reflex) | Pilot restarts | Ball (without Reflex) |
|---|---|---|---|---|---|---|
| 1 | pilot process killed (SIGKILL) | 14.0 ms | 7.3 cm | kept | 3 | kept |
| 2 | pilot frozen (SIGSTOP), no CPU use | 16.0 ms | 6.8 cm | kept | 2 | **LOST** |
| 3 | pilot stuck in an infinite loop, 100% CPU | 18.0 ms | 8.8 cm | kept | 3 | **LOST** |
| 4 | pilot answers 50 ms late | 17.0 ms | 11.8 cm | kept | 0 | kept |
| 5 | pilot outputs NaN | 15.0 ms | 11.8 cm | kept | 0 | kept |
| 6 | pilot outputs 40 rad (limit is 0.21) | 15.0 ms | 11.8 cm | kept | 0 | **LOST** |
| 7 | pilot output stuck at +max | 14.0 ms | 1.3 cm | kept | 2 | **LOST** |
| 8 | pilot output sign-inverted (drives the ball off) | 15.0 ms | 4.2 cm | kept | 0 | **LOST** |

**With Reflex: ball kept in 8 of 8 fault scenarios. Without Reflex: ball lost in 5 of 8.**

```
guardian wake-up latency, 10837 ticks at 50 Hz (period 20000 us)
         4 - 8        us |########################################| 9259
         8 - 16       us |################################        | 1572
        16 - 32       us |########                                | 6
  p50 < 8 us   p99 < 16 us   p99.99 < 32 us   max = 24.6 us   late ticks (> period/2) = 0
```

Guardian worst-case execution time per tick: 160.3 us. Spoofed proposals refused: 0.
