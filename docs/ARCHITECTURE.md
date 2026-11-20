# Reflex architecture

## The problem

A learned controller is attractive because it performs better than a hand-tuned one, and unacceptable in a safety-relevant machine because nobody can state what it will do in every situation. The standard answer is the Simplex architecture (Sha, 2001), today usually called runtime assurance: run the unverified high-performance controller, but put a small verified monitor in front of the actuator with the authority to switch to a simple controller whose behaviour is fully understood.

The monitor is only as good as its timing. If it can be delayed, starved or blocked by the thing it monitors, it guarantees nothing. That is an operating-system property, and it is the reason Reflex is built on QNX mechanisms instead of application-level timeouts.

## Three rings

1. **Pilot (untrusted).** `reflex-pilot`, a 3-12-1 tanh network trained in simulation (`tools/train_policy.py`). Two frozen weight sets are compiled in: one for the simulator backend, and one trained with `--hardware` for the serial backend (50 Hz, `rx_cfg_hardware()`, noisy sensor, one tick of pilot latency, the guardian's own envelope check in the training loop). The guardian passes `--hardware` to the pilot on the serial backend; that hint only changes how well the pilot flies, never how its commands are judged. A guardian running as root (under HAM) spawns the pilot with the credentials of the pilot binary's owner (`posix_spawnattr_setcred`, `--pilot-user UID:GID` to override) and refuses to run it as root. Low priority, optionally CPU-budgeted. Allowed to fail in any way.
2. **Guardian (trusted, hard real time).** `reflex-guardian`. One control thread, fixed high priority, memory locked, no allocation, no locks, no blocking call except the single `MsgReceive()`. Starting a pilot process is a blocking request to the process manager (up to about 0.8 ms on the Pi 4, measured in Phase 2), so it is done by a helper thread at a lower priority (40 by default); the control thread only raises a request flag and later picks up the new pid, both through atomics, and never waits for the helper.
3. **I/O node (hardware).** A microcontroller that owns the sensor and servo and levels the beam by itself if the guardian goes silent for 150 ms.

On top of ring 2, the QNX High Availability Manager restarts the guardian if it dies. Plant state and counters live in shared memory, so the new guardian continues where the old one stopped.

## One control period

```
timer pulse ──> MsgReceive() returns
                 read sensor, update estimator
                 judge the pilot's last proposal      (rx_guard_step)
                 command = proposal  or  safe controller
                 write actuator
                 MsgReply(state) ──> pilot wakes, computes, MsgSend(proposal), blocks again
                 supervise pilot process, push telemetry record
```

The pilot is REPLY-blocked between ticks. It does not poll and cannot run ahead: it gets exactly one state per period and its answer is expected before the next one. The guardian never issues a blocking call toward the pilot. `MsgReply()` does not block.

## The safety kernel (`src/core/rx_core.c`)

Pure functions, about 150 lines, no OS dependency, unit-tested by `rx_simtest`.

**Plant.** Ball position `x`, velocity `v`, beam angle `u`: `x'' = k sin(u)`, `|u| <= u_max`, beam half-length `L`.

**Recoverable set.** A state is recoverable if, for *each* end of the beam, the following worst case still stops short of the keep-out strip: for a reaction delay `t_d` the actuator pushes the ball toward that end at full authority, and after that braking achieves only a fraction `eta` of the ideal deceleration. For the end in direction `s` (+1 or -1):

```
a_max   = k sin(u_max)
x_s     = s x,  v_s = s v                          (position, velocity toward that end)
x_d     = x_s + v_s t_d + a_max t_d^2 / 2
v_d     = v_s + a_max t_d
reach_s = max( x_s, x_d, x_d + v_d^2 / (2 eta a_max) if v_d > 0 )
margin  = (L - strip) - max(reach_+, reach_-)
```

`margin > 0` means recoverable. `make test` checks this claim by Monte Carlo: random recoverable states, one more adversarial tick, then the fallback alone, across perturbed plant parameters. An earlier version only looked at the end the ball was moving toward; the test found states near the opposite end that violated the strip, which is why the bound is two-sided now.

**Decision.** A proposal is accepted only if all of these hold:

1. a pilot is connected and not quarantined,
2. the proposal was computed from the previous tick's state (deadline),
3. it is finite,
4. it is inside the actuator range,
5. the state predicted one step ahead under that command is still recoverable.

Otherwise the guardian switches to SAFE. The fallback controller pushes away from whichever end has less than 1 cm of margin, at full authority, and is a critically damped PD law elsewhere. Control returns to the pilot after a hold time, once its proposals are valid again and the state is back inside an inner margin.

**Escalation.** Three unsafe-command interventions inside ten seconds: the pilot is killed and respawned. Three kills inside thirty seconds: quarantine. The fallback keeps flying until an operator releases the pilot.

Rule 5 is what makes this more than a watchdog. A stuck or sign-inverted output is a perfectly valid number, on time, in range. It is only caught because the guardian reasons about where the command leads.

## Learning under the guardian

`reflex-pilot --learn` flies a 3-12-1 tanh network whose weights are improved online with an antithetic evolution strategy (`src/core/rx_learner.c`). One episode is one setpoint segment. Two consecutive episodes fly `mean + sigma*eps` and `mean - sigma*eps`; the difference of their costs gives a gradient estimate along `eps`. The cost is squared tracking error plus a heavy charge for every tick the guardian had to be in control, which the guardian reports to the pilot in each reply. The best weights seen are kept and restored if learning diverges.

Three properties make this safe to run on real hardware:

- The guardian's decision does not depend on what the pilot is. An exploring network is treated exactly like a faulty one.
- In training mode the guardian still overrides every unsafe command but does not count them as strikes, so the learner is not killed for exploring. Hangs and crashes are still handled as usual.
- Weights are written to disk by a helper thread (file I/O in the loop would make the pilot miss its deadline) with an atomic rename. Kill the learner mid-training and its replacement resumes from the last snapshot.

The learning curve shown on the dashboard combines the pilot's self-reported cost (untrusted, display only) with intervention counts measured by the guardian.

## Why each QNX mechanism is there

- **Send/Receive/Reply instead of a queue or socket.** The direction of blocking is the safety argument: the client (pilot) blocks, the server (guardian) does not. With a queue the guardian would need its own timeout logic and a second wake-up source.
- **Pulses.** The periodic timer delivers a pulse to the same channel pilot messages arrive on, so a single-threaded state machine handles both with no shared state and no lock. QNX timers expire on clock-tick boundaries (multiples of `ClockPeriod()`, 1 ms on the Pi 4 quick-start image), so the first expiry is armed at an absolute, tick-aligned time; wake-up latency is then measured from the instant the timer was really programmed to fire, not from an arbitrary phase.
- **`_NTO_CHF_FIXED_PRIORITY`.** A QNX server normally runs at the priority of the client it serves. That is the right default for drivers and the wrong one here, because the client is the untrusted party.
- **The pilot's interface is one channel.** The guardian passes its process id and channel id on the pilot's command line. The pilot does not map the telemetry or control memory.
- **Sender identity.** `MsgReceive()` reports the sending process. The guardian accepts proposals only from the process it spawned, and from nobody before its first pilot exists. The pilot is created stopped (`POSIX_SPAWN_HOLD`), registered as the allowed sender, and only then released with `SIGCONT`, so its first message can never be mistaken for a spoof. Any other process that attaches to the channel is answered with `EPERM` and counted (`spoofed senders` in the status).
- **Disconnect pulse.** A dead pilot is reported by the kernel, so crash detection is immediate and costs nothing while the pilot is alive.
- **`SCHED_SPORADIC`.** The policy is set in the spawn attributes, so it applies before the pilot runs. `sched_ss_max_repl` is sized to the control rate (twice the pilot's activations per 10 ms period): with the original fixed value of 8, a healthy pilot at 1 kHz was seen at its low priority 1 in 35 of 100 `pidin` samples on the Pi 4, and in 0 of 100 with the sized value. QNX 8.0 no longer has the adaptive partitioning scheduler. The budget mechanism that remains is sporadic scheduling: the pilot runs at its normal priority for 3 ms in every 10 ms and at priority 1 after that, so a spinning pilot cannot starve the dashboard, the shell or the network stack.
- **Resource manager.** `/dev/reflex` makes the guardian scriptable from the shell with `cat` and `echo`. It is a separate process reading shared memory, so killing it has no effect on control.
- **HAM.** Restarting the guardian is delegated to the component QNX provides for exactly that.

## I/O node link: protocol v2 and `reflex.conf`

The Black Pill is deliberately dumb: it reports the **raw** sensor distance and applies a **raw** servo pulse. Every conversion to metres and radians happens on the Pi, from a calibration file the `rxcal` wizard writes, so calibrating a rig never means editing firmware or reflashing. Parser and formatter: `src/core/rx_link.c` (pure, fuzzed by `rx_linktest`).

| Direction | Line | Meaning |
|---|---|---|
| node → host | `M <raw_mm>` / `M !` | raw distance 0..2000 mm, 50 Hz; `!` = no valid reading |
| | `I reflex_io <proto> <board> <emulated>` | identity, on reset and on `?` |
| | `T <theta_mrad>` | emulated boards only: true beam angle (stands in for the inclinometer when `rxcal` is tested; never used by the guardian) |
| | `# …` | comment |
| host → node | `S <pulse_us>` | servo pulse; the node clamps it to its compile-time 1000..2000 µs and parks the servo at the level pulse if none arrives for 150 ms |
| | `L <pulse_us>` | level pulse for that watchdog; written to flash only on change and only while parked (an STM32 sector erase stalls the CPU) |
| | `?` | request the `I` line |
| | `E pin <mm>` / `E free` / `E remove` | emulator only: place the simulated ball (lets `rxcal --auto` run without a person) |

A protocol-v1 line (`P …`) is recognised only so the guardian can log "firmware speaks protocol v1: reflash it". On every link-up the guardian sends `L <level_us>` and `?`.

`reflex.conf` (`src/core/rx_conf.h`) is `key = value` text with ten required keys: `level_us`, `us_per_mrad`, `servo_dir`, `center_mm`, `sensor_dir`, `range_min_mm`, `range_max_mm`, `half_len_m`, `calibrated_at`, `board`. The serial backend converts with

- position: `x = sensor_dir · (raw − center_mm) / 1000` m, valid only inside `[range_min_mm, range_max_mm]`;
- command: `pulse = level_us + servo_dir · u · 1000 · us_per_mrad` µs, after clamping `u` to the kernel's `theta_max`, then to 1000..2000 µs.

The parser rejects a missing, unknown or duplicated key and any value out of range, and `--backend serial` refuses to start without a valid file (`--uncalibrated` for debugging). `half_len_m` may only shorten the kernel's beam (0.10..0.20 m), never lengthen it. Unit tests: `rx_linktest`; the wizard end to end against a fake rig with a deliberately wrong hidden calibration: `make test-cal`.

## Telemetry

The guardian never prints from its control loop: a console write can block for milliseconds. Everything it has to say goes into a wait-free event log in shared memory (`rxctl events`, dashboard, `cat /dev/reflex`). `rxctl incidents <dir>` is a flight recorder that saves one second before and after every intervention.

The guardian writes one record per tick into a ring buffer in POSIX shared memory and publishes the head index atomically. Readers copy a record and re-check the head, so they never block the writer. Counters are C11 atomics. Every tool (`rxctl`, `rxdash`, `rxfs`) is a reader of this block and a writer only of the small control section.

## Porting to another plant

Replace three things in `src/core/rx_core.c`: the model in `rx_plant_step()`, the recoverable-set bound in `rx_margin()`, and the fallback in `rx_safe_u()`. Add an I/O backend in `src/guardian/rx_io.c`. The IPC, supervision, escalation, telemetry and tools are plant-independent.
