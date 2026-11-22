# Submission kit

Category: **Robotics & Medical**. Bonus prize to target: **Best use of QNX-specific Technologies**. Confirm on the official Rules tab how many prizes one project may take.

## The one-sentence pitch

A neural network learns to control an unstable machine from scratch, on the real hardware, and never breaks it once, because a QNX guardian it cannot delay, starve, block or impersonate overrules every unsafe command within one control period.

## Video (under 3:00, public on YouTube, no copyrighted music, no third-party logos in frame)

The video is a script: `./rxctl script demo/video.rxs` (then `demo/video-joint.rxs`) plays the faults and puts
the captions on the dashboard; open it in presentation mode (`http://<pi>:8080/?present`, or press P). Before
recording, run `./rxctl bench` once so the "Is the AI worth having?" card shows numbers measured on the plant in
the shot. Record the screen with OBS and the beam with a phone on a tripod. Time-lapses must be labelled.

| Time | Shot (from `demo/video.rxs`) | Say |
|---|---|---|
| 0:00-0:15 | **Hook.** Flip injected with the guardian on: badge goes green, ball stays. Same flip with the guardian off: ball rolls off. | "This AI just output the exact wrong command. Guardian on: nothing happens. Guardian off: it's over." |
| 0:15-0:35 | Fallback controller alone, then the AI on the same setpoints; the benchmark card. | "Why have the AI at all? It is faster. Measured on this machine: [numbers from the card]. Reflex lets you keep that and still never crash." |
| 0:35-1:05 | Crash, freeze, NaN. Event log scrolls. | "Kill it: the kernel tells the guardian that tick. Freeze it: its deadline passes, the fallback flies. Garbage: refused." |
| 1:05-1:35 | Terminal: `pidin` (pilot REPLY-blocked, priorities), `cat /dev/reflex`, the codelab output. | "The AI is a client that blocks on the guardian, never the reverse. One MsgReceive gets the timer pulse and the AI's message: one thread, no locks. Fixed-priority channel, kernel-checked sender, disconnect pulses." |
| 1:35-2:00 | `slay -s KILL reflex-guardian`: HAM restarts it, ball stays. Pull the USB cable: the I/O node levels the beam. | "Who guards the guardian? HAM restarts it from shared memory. If the computer goes silent, the microcontroller levels the beam. Three rings." |
| 2:00-2:35 | `demo/video-joint.rxs`: the robot joint, stuck torque with the guardian on, then off. | "Same guardian, different machine: one plant table swapped. A joint lifting a load, like an exoskeleton knee." |
| 2:35-3:00 | Campaign table and latency histogram from `results/`; repo URL. | "Every number is measured on the target and in the repository. Open source." |

If the hardware is not ready, use the dashboard's plant view and say that the plant is simulated.
30-second social clip for Community Choice: `docs/SOCIAL_CLIP.md`.

## Devpost text (edit to match what you built and measured; the bracketed parts must be in your own words)

**Tagline:** An AI learns to control a real machine and is never allowed to break it.

**Inspiration.** Robot learning has a hardware problem: a policy that is still learning will, sooner or later, command something destructive. Researchers work around it with simulators, cages and people holding emergency stops. Industry cannot ship a learned controller in a vehicle, a medical device or a factory robot at all, because it cannot be certified. The accepted way out is runtime assurance: a small verified monitor in front of the actuator (ASTM F3269 in aviation; "safety filters" in robotics research). Almost all of that work assumes the monitor always runs on time. That assumption is an operating-system property, and it is what QNX is for.

**What it does.** Reflex puts a hard-real-time guardian between an untrusted neural-network controller and the actuator. Each control period it checks: is the AI alive, on time, sane, and does its command keep the system inside the set of states from which a verified fallback controller can still recover. If not, the fallback takes over within that period, and the AI is told. Because of that, the AI can learn online on the physical plant. Misbehaving pilots are restarted, then quarantined. Around it: a fault campaign, an AI-vs-fallback benchmark, a calibration wizard so a new rig needs no reflashing, a live dashboard with a presentation mode, a `/dev/reflex` resource manager, HAM restart and a flight recorder. Two plants: a ball on a beam (the real rig) and a simulated robot joint with a gravity load, behind one plant interface.

**How I built it.** [Your own account: what you designed, the tools you used including AI assistance (disclose it as the rules require), what you had to fix to get it running on QNX, how you built and tuned the beam.]

**How it uses QNX 8.0** (also paste under "Evidence of QNX use"):
- `src/ipc/rx_ipc_qnx.c`: `ChannelCreate(_NTO_CHF_FIXED_PRIORITY | _NTO_CHF_DISCONNECT)`, timer pulses via `SIGEV_PULSE_INIT`, `MsgReceive`/`MsgReply`/`MsgError`, sender check via `_msg_info.pid`, disconnect pulses matched by `scoid`.
- `src/guardian/main.c`: pilot spawned with `POSIX_SPAWN_HOLD` (pid registered before it runs), `posix_spawnattr_setcred` (never as root), `SCHED_SPORADIC` budget; `trace_logf()` kernel trace events.
- `src/qnx/rxfs.c`: resource manager at `/dev/reflex` (`resmgr_attach`, `iofunc_*`, custom `io_read`/`io_write`).
- `src/qnx/rxham.c`: High Availability Manager entity, death condition and restart action. The Pi 4 quick-start image has no `ham`; `make deploy` copies the SDP's, and HAM restarts were run on the Pi (`results/2026-11-20-pi4-emulated-io/11-reflex-demo.txt`).
- USB serial: `devc-serusb` from the SDP started at boot by a hook (`scripts/install-autostart.sh`); the guardian flushes its stale buffer on open (`src/guardian/rx_io.c`).
- `docs/codelab/`: a standalone tutorial program of the same pattern, built with `qcc` and run on the Pi.
- `Makefile`: `ntoaarch64-gcc` / `ntox86_64-gcc` from SDP 8.0, zero warnings. `results/`: everything measured on the QNX target.

**Why a real-time OS.** [Not yet measured on this project: the Linux A/B on the same Pi (Roadmap H3). Fill in only from `results/`.]

**Challenges.** [In your own words. Documented ones: the QNX USB serial driver kept seconds-old data while the port was closed, so the guardian could act on a stale position (fixed with `tcflush`, `results/2026-11-20-pi4-emulated-io/`); under HAM the untrusted pilot ran as root, which silently broke fault injection (fixed with `posix_spawnattr_setcred`); the benchmark showed the AI was *not* better at first, because of a steady-state bias (fixed in the policy, `results/2026-11-20-host-bench/`); the online learner failed on 11 of 48 random seeds (fixed by its learning rate); QNX 8.0 has no adaptive partitioning, so CPU isolation uses sporadic scheduling.]

**What's next.** [Candidates: measure and set `t_delay` from the real beam (the stuck-fault margin is thin, `results/2026-11-22-host-stucktest/`); pilot in a Linux guest under the QNX Hypervisor so it can be a full PyTorch stack; a QNX security policy for the pilot; a coupled multi-joint plant; formal verification of the envelope function.]

## Final checklist

- [ ] Repository is public, new, created during the submission period, MIT licence visible in the About box
- [ ] README has build and run instructions naming QNX SDP 8.0, the target, and the toolchain
- [ ] README Results section contains only numbers measured on your target
- [ ] `results/` committed
- [ ] Every QNX-only file compiles and has been run at least once on the target
- [ ] Video under 3:00, public, shows it running on the device, time-lapse labelled
- [ ] Category selected: Robotics & Medical
- [ ] Devpost description lists the QNX evidence with file names
- [ ] You can answer the questions at the end of `docs/GUIDE.md`
- [ ] Submitted at least two days before the deadline
