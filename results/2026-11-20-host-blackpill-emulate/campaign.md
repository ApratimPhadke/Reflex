# Reflex fault campaign

- backend: `serial`, control rate 50 Hz
- each fault is injected into a healthy system and left active for 4 s
- *without Reflex* = the same pilot, same faults, guardian in bypass (pilot commands go straight to the actuator)

- *closest to beam end* is the guardian's **estimate** (no ground truth on this backend; it includes sensor noise)

| # | Injected pilot fault | Guardian took over after | Closest to beam end | Ball (with Reflex) | Pilot restarts | Ball (without Reflex) |
|---|---|---|---|---|---|---|
| 1 | pilot process killed (SIGKILL) | 18.6 ms | 7.8 cm | kept | 1 | kept |
| 2 | pilot frozen (SIGSTOP), no CPU use | 14.8 ms | 8.3 cm | kept | 2 | **LOST** |
| 3 | pilot stuck in an infinite loop, 100% CPU | 14.4 ms | 9.0 cm | kept | 3 | **LOST** |
| 4 | pilot answers 50 ms late | 14.7 ms | 8.7 cm | kept | 0 | **LOST** |
| 5 | pilot outputs NaN | 18.4 ms | 7.8 cm | kept | 0 | **LOST** |
| 6 | pilot outputs 40 rad (limit is 0.21) | 15.0 ms | 7.6 cm | kept | 0 | **LOST** |
| 7 | pilot output stuck at +max | 14.9 ms | 1.4 cm | kept | 2 | **LOST** |
| 8 | pilot output sign-inverted (drives the ball off) | 16.7 ms | 3.8 cm | kept | 2 | **LOST** |

**With Reflex: ball kept in 8 of 8 fault scenarios. Without Reflex: ball lost in 7 of 8.**

```
guardian wake-up latency, 10136 ticks at 50 Hz (period 20000 us)
         8 - 16       us |#######                                 | 4
        16 - 32       us |###################                     | 59
        32 - 64       us |################################        | 926
        64 - 128      us |##################################      | 1712
       128 - 256      us |########################################| 5079
       256 - 512      us |####################################    | 2355
       512 - 1024     us |###                                     | 1
  p50 < 256 us   p99 < 512 us   p99.99 < 512 us   max = 678.7 us   late ticks (> period/2) = 0
```

Guardian worst-case execution time per tick: 641.7 us. Spoofed proposals refused: 0.
