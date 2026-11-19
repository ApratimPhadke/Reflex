# Reflex fault campaign

- backend: `sim`, control rate 1000 Hz
- each fault is injected into a healthy system and left active for 4 s
- *without Reflex* = the same pilot, same faults, guardian in bypass (pilot commands go straight to the actuator)

| # | Injected pilot fault | Guardian took over after | Closest to beam end | Ball (with Reflex) | Pilot restarts | Ball (without Reflex) |
|---|---|---|---|---|---|---|
| 1 | pilot process killed (SIGKILL) | 1.0 ms | 12.5 cm | kept | 1 | kept |
| 2 | pilot frozen (SIGSTOP), no CPU use | 6.0 ms | 12.3 cm | kept | 2 | kept |
| 3 | pilot stuck in an infinite loop, 100% CPU | 6.0 ms | 12.0 cm | kept | 5 | **LOST** |
| 4 | pilot answers 50 ms late | 6.0 ms | 12.0 cm | kept | 0 | kept |
| 5 | pilot outputs NaN | 2.0 ms | 12.0 cm | kept | 0 | kept |
| 6 | pilot outputs 40 rad (limit is 0.21) | 2.0 ms | 12.0 cm | kept | 0 | **LOST** |
| 7 | pilot output stuck at +max | 244.0 ms | 3.4 cm | kept | 2 | **LOST** |
| 8 | pilot output sign-inverted (drives the ball off) | 514.0 ms | 3.4 cm | kept | 2 | **LOST** |

**With Reflex: ball kept in 8 of 8 fault scenarios. Without Reflex: ball lost in 4 of 8.**

```
guardian wake-up latency, 254524 ticks at 1000 Hz (period 1000 us)
         4 - 8        us |########################################| 231703
         8 - 16       us |################################        | 22808
        16 - 32       us |########                                | 13
  p50 < 8 us   p99 < 16 us   p99.99 < 16 us   max = 25.6 us   late ticks (> period/2) = 0
```

Guardian worst-case execution time per tick: 40.4 us. Spoofed proposals refused: 0.
