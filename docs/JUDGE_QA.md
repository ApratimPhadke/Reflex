# Judge Q&A: the 20 hardest questions, with short answers that point at the code

References are `file: function`. Every number comes from a file in `results/`, named next to it.

**1. What exactly does the guardian decide each tick?**
`src/core/rx_core.c: rx_guard_step`. It gives the newest proposal one verdict, in order: quarantined, no pilot, stale
(older than `max_age_s`), not finite, out of range, or outside the envelope. Any verdict sends the fallback command
and holds SAFE for `hold_s`. The AI gets control back only after the hold, with `inner_margin` of room.

**2. What is "the envelope"? Derive it.**
`rx_core.c: beam_margin_side` (and `joint_margin_side`). Take the worst case toward one end: for `t_delay` the actuator
pushes at full authority (a_max = k sin(theta_max) for the beam), then braking delivers only eta = 0.7 of a_max.
reach = max(x, x_d, x_d + v_d^2 / (2 eta a_max)), with x_d = x + v t_d + a_max t_d^2 / 2 and v_d = v + a_max t_d.
margin = (half_len - edge_margin) - reach, the smaller over both ends. The proposal is checked one step ahead
(`rx_guard_step`, "one-step lookahead"): x1, v1 under the proposed command over max(dt, 20 ms), and refused if
margin(x1, v1) <= 0.

**3. Why should that bound hold?**
Because `t_delay` covers a full actuator reversal plus sensing (beam: 0.42 rad at 6 rad/s is 70 ms, `t_delay`
80 ms in simulation, 120 ms on the rig), friction only helps, and only 70 % of the ideal braking is assumed. It is
analytic and conservative for this model. It is not a formal proof, and it is only as good as the model.

**4. What does the Monte Carlo test prove, and what not?**
`src/tools/rx_simtest.c: envelope_mc`, `envelope_mc_joint`. About 120,000 random states per case that the kernel
calls recoverable get one extra adversarial tick at full authority, then only the fallback. The ball or joint must
never enter the keep-out strip. Cases: nominal and perturbed plants (±15 % gain or inertia, +15 % gravity, 20 %
slower actuator), 0 violations (`make test`). It does **not** prove all states (it samples), it uses the true state
(not the estimator), and it uses the model, not the real delay chain. On the fake I/O node one stuck run in 28
entered the strip, at 4.3 mm, with the ball kept (`results/2026-11-22-host-stucktest/`).

**5. Why is the AI the client and the guardian the server?**
`src/ipc/rx_ipc_qnx.c: rx_server_wait`. `MsgSend` blocks the sender; `MsgReply` never blocks the receiver. So the
pilot waits for the guardian, never the reverse, and the guardian replies on the next tick, which paces the pilot.
The codelab demonstrates it: 150 proposals, last reply at tick 150 (`results/2026-11-22-pi4-codelab/run.txt`).

**6. Why `_NTO_CHF_FIXED_PRIORITY`?**
Without it, a QNX server inherits the priority of the client it is serving, so a priority-10 pilot would pull the
guardian down to 10 while its message is handled. Measured in the codelab: priority 30 with the flag, 10 without.

**7. QNX 8.0 has no adaptive partitioning. What stops a pilot that spins at 100 % CPU?**
The guardian runs at a higher fixed priority (50) than the pilot (10). With `--sporadic` the pilot runs under
`SCHED_SPORADIC` (`src/guardian/main.c: spawn_pilot`): 3 ms in every 10 ms at its normal priority, then priority 1.
A spinning pilot stops answering and is caught as stale. Over USB on the Pi: taken over after 76 ms
(`results/2026-11-20-pi4-emulated-io/15-campaign-v2-pilot-unprivileged.txt`).

**8. Why not just a watchdog?**
A watchdog sees liveness and timing. The worst faults in the campaign are valid numbers, on time and in range:
stuck at full tilt, and sign-inverted. Only the one-step prediction catches them. Without the guardian those rows
lose the ball (`results/2026-11-20-pi4-emulated-io/05-campaign.md`).

**9. How do you know who sent a proposal?**
The kernel fills `_msg_info.pid`, and `rx_server_wait` refuses any other sender with `EPERM`. The pilot is spawned
with `POSIX_SPAWN_HOLD` and released only after its pid is registered, so its first message is never refused.

**10. What happens when the pilot crashes, hangs or misbehaves?**
Crash: a disconnect pulse (the `scoid` is checked) and `waitpid` in the supervisor, then a respawn with backoff.
Hang: no proposal for 0.4 s, so it is killed. Unsafe commands: 3 strikes in 10 s means a kill, and 3 kills in 30 s
means quarantine (`main.c`, pilot supervision). In every case the fallback flies.

**11. What if the guardian itself dies?**
HAM restarts it (`src/qnx/rxham.c`: `ham_action_restart`), and it resumes from shared memory. Shown with the ball
kept (`results/2026-11-20-pi4-emulated-io/11-reflex-demo.txt`). If the whole computer goes silent, the I/O node
parks the servo at the stored level pulse after 150 ms (`firmware/reflex_io/reflex_io.ino`, the watchdog in `loop`).

**12. Where can the control loop block?**
Only in `rx_server_wait` (invariant 1, `CLAUDE.md`). Spawning the pilot runs in a helper thread, and so does
reopening the serial port. The loop does not allocate, lock or print; diagnostics go to the shared-memory event
log. Serial reads are non-blocking and bounded (`src/guardian/rx_io.c: ser_read`).

**13. What latency did you measure, and where?**
On the Pi 4 under QNX 8.0: worst wake-up latency 31.7 µs over 313,359 ticks at 1 kHz with 0 late ticks
(`results/2026-11-22-pi4-joint/latency.txt`), and 43.2 µs at 50 Hz over USB serial
(`results/2026-11-20-pi4-emulated-io/15-...txt`). A load test on this branch and a Linux A/B on the same Pi are
still to come (Roadmap H). Until then, no comparison with Linux is claimed.

**14. Is learning on the real machine safe?**
Training mode only stops counting strikes (invariant 4). The guardian's decision is the same. Ten minutes of
`--learn-from-sim` over USB on the Pi lost 0 balls (`results/2026-11-20-pi4-emulated-io/10-learn-from-sim-10min.txt`).

**15. Is the AI worth having if the fallback is safe anyway?**
`rxctl bench` flies the same setpoints with the AI and with the fallback alone. On the simulated beam the AI
settled 60 % faster with 24 % lower RMS error (`results/2026-11-20-host-bench/03-after-sim.md`, and the same on the
Pi). With the hardware configuration it was 23 % faster. The first bench showed the AI was *not* better (a
steady-state bias), and that is how the bias was found and fixed.

**16. Do you trust the sensor?**
No more than you must. A reading outside the calibrated window, or none for 200 ms, counts as "no ball", and the
beam is levelled. The serial parser is strict and fuzzed (`src/core/rx_link.c`, `rx_linktest`). A real product needs
redundant, diagnosed sensing; this is listed in `README.md` (Limits) and `docs/IMPACT.md`.

**17. What does the pilot get to see or touch?**
The guardian's pid, its channel id and the control rate: nothing else. It never maps the guardian's shared
memory (invariant 3). A guardian running as root (HAM) spawns it as an ordinary user (`posix_spawnattr_setcred`).

**18. Is this tied to the ball and beam?**
No. A plant is one `rx_plant_ops_t` table (`rx_core.c: rx_plant_ops`): lookahead dynamics, bound, fallback,
simulator and holding command. A second plant, a robot joint with a gravity load, runs the same campaign: 8/8 kept,
and 3/8 limit hits without the guardian (`results/2026-11-22-pi4-joint/campaign.md`).

**19. How would this scale to a real multi-joint robot?**
Per-joint bounds that ignore coupling are not enough. A real arm needs a recoverable set for its coupled dynamics,
for example worst-case coupling torques in each joint's bound, or a reachability or control-barrier formulation. It
also needs higher control rates, independent sensors, a certified RTOS and timing analysis. The QNX pattern itself
(one channel, pulses, a fixed-priority server, kernel sender identity, HAM) carries over unchanged.

**20. What is the weakest point you know of?**
Three: (a) the stuck fault's margin is thin, with one run in 28 into the strip on the fake node, so `t_delay` must
come from the real beam's measured delay, and `rxctl stuck` exists to measure it; (b) the Pi rebooted three times
while driving the Black Pill over USB, with heat and sustained under-voltage ruled out
(`results/2026-11-22-reboot-analysis/`); (c) nothing is certified (`docs/IMPACT.md`).
