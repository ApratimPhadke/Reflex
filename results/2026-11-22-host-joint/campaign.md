# Reflex fault campaign

- backend: `sim`, control rate 1000 Hz
- each fault is injected into a healthy system and left active for 4 s
- *without Reflex* = the same pilot, same faults, guardian in bypass (pilot commands go straight to the actuator)

- plant: **robot joint** (simulated); "limit hit" = the joint reached its hard stop

| # | Injected pilot fault | Guardian took over after | Closest to the stop | Joint (with Reflex) | Pilot restarts | Joint (without Reflex) |
|---|---|---|---|---|---|---|
| 1 | pilot process killed (SIGKILL) | 1.4 ms | 34.4 deg | kept | 1 | kept |
| 2 | pilot frozen (SIGSTOP), no CPU use | 5.4 ms | 34.4 deg | kept | 2 | kept |
| 3 | pilot stuck in an infinite loop, 100% CPU | 5.4 ms | 34.4 deg | kept | 5 | kept |
| 4 | pilot answers 50 ms late | 5.8 ms | 34.4 deg | kept | 0 | kept |
| 5 | pilot outputs NaN | 1.4 ms | 34.4 deg | kept | 0 | kept |
| 6 | pilot outputs 40 N m (limit is 4) | 1.6 ms | 34.4 deg | kept | 0 | **LIMIT HIT** |
| 7 | pilot output stuck at full torque | 85.0 ms | 25.4 deg | kept | 4 | **LIMIT HIT** |
| 8 | pilot output sign-inverted (drives the joint into a stop) | 208.3 ms | 35.5 deg | kept | 2 | **LIMIT HIT** |

**With Reflex: joint kept off its stops in 8 of 8 fault scenarios. Without Reflex: limit hit in 3 of 8.**

```
guardian wake-up latency, 248551 ticks at 1000 Hz (period 1000 us)
         4 - 8        us |####                                    | 3
         8 - 16       us |################################        | 13035
        16 - 32       us |########################################| 115975
        32 - 64       us |####################################### | 89912
        64 - 128      us |##################################      | 23486
       128 - 256      us |#############################           | 5630
       256 - 512      us |####################                    | 348
       512 - 1024     us |################                        | 109
      1024 - 2048     us |############                            | 38
      2048 - 4096     us |#########                               | 15
  p50 < 32 us   p99 < 256 us   p99.99 < 2048 us   max = 3592.6 us   late ticks (> period/2) = 167
```

Guardian worst-case execution time per tick: 2138.0 us. Spoofed proposals refused: 0.
