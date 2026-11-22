# Reflex fault campaign

- backend: `sim`, control rate 1000 Hz
- each fault is injected into a healthy system and left active for 4 s
- *without Reflex* = the same pilot, same faults, guardian in bypass (pilot commands go straight to the actuator)

- plant: **robot joint** (simulated); "limit hit" = the joint reached its hard stop

| # | Injected pilot fault | Guardian took over after | Closest to the stop | Joint (with Reflex) | Pilot restarts | Joint (without Reflex) |
|---|---|---|---|---|---|---|
| 1 | pilot process killed (SIGKILL) | 1.0 ms | 34.4 deg | kept | 1 | kept |
| 2 | pilot frozen (SIGSTOP), no CPU use | 6.0 ms | 34.4 deg | kept | 2 | kept |
| 3 | pilot stuck in an infinite loop, 100% CPU | 6.0 ms | 34.4 deg | kept | 5 | kept |
| 4 | pilot answers 50 ms late | 6.0 ms | 34.4 deg | kept | 0 | kept |
| 5 | pilot outputs NaN | 2.0 ms | 34.4 deg | kept | 0 | kept |
| 6 | pilot outputs 40 N m (limit is 4) | 2.0 ms | 34.4 deg | kept | 0 | **LIMIT HIT** |
| 7 | pilot output stuck at full torque | 86.0 ms | 25.4 deg | kept | 4 | **LIMIT HIT** |
| 8 | pilot output sign-inverted (drives the joint into a stop) | 209.0 ms | 35.4 deg | kept | 2 | **LIMIT HIT** |

**With Reflex: joint kept off its stops in 8 of 8 fault scenarios. Without Reflex: limit hit in 3 of 8.**

```
guardian wake-up latency, 255742 ticks at 1000 Hz (period 1000 us)
         4 - 8        us |########################################| 236233
         8 - 16       us |###############################         | 19506
        16 - 32       us |####                                    | 3
  p50 < 8 us   p99 < 16 us   p99.99 < 16 us   max = 31.7 us   late ticks (> period/2) = 0
```

Guardian worst-case execution time per tick: 47.3 us. Spoofed proposals refused: 0.
