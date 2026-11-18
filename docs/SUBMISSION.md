# Submission kit

Category: **Robotics & Medical**. Bonus prize to target: **Best use of QNX-specific Technologies**. Confirm on the official Rules tab how many prizes one project may take.

## The one-sentence pitch

A neural network learns to control an unstable machine from scratch, on the real hardware, and never breaks it once, because a QNX guardian it cannot delay, starve, block or impersonate overrules every unsafe command within one control period.

## Video (under 3:00, public on YouTube, no copyrighted music, no third-party logos in frame)

Record the screen with OBS and the beam with a phone on a tripod. Speak plainly. No intro animation. Time-lapse sections must be labelled as such on screen.

| Time | Picture | Say |
|---|---|---|
| 0:00-0:20 | Beam with ball, untrained network: the beam twitches, guardian badge flashes green, ball stays on. Dashboard beside it. | "This neural network has never seen this machine. Its weights are random. It has full control of an unstable system, and it is about to learn on the real hardware. Normally that means a broken robot. Here it means nothing, because of the process in the middle. This is Reflex, on QNX 8.0 on a Raspberry Pi 4." |
| 0:20-0:50 | Time-lapse of the learning panel: green bars thin out, blue cost line falls. Counter "Balls lost: 0" stays in frame. | "Every command the AI proposes is checked one step ahead. If that step would leave the set of states the fallback controller can recover from, the guardian takes over, and tells the AI it had to. That is the learning signal. [N] minutes, [N] interventions, zero balls dropped. [Your numbers.]" |
| 0:50-1:10 | Click *Guardian bypass*, then *Flip*. Ball rolls off. Re-enable, *Flip* again, ball stays. | "Without the guardian, one bad output and it is over. With it, the same fault is caught, even though the command is a valid number, on time, in range. A watchdog cannot see that. A prediction can." |
| 1:10-1:35 | Click Crash, Freeze, NaN. Event log scrolls. Kill the learner mid-training: 'resumed at episode N'. Start a second pilot by hand: 'spoofed sender refused'. | "Kill the AI: the kernel tells the guardian that tick. Freeze it: deadline missed, fallback flies. It resumes learning where it stopped. A second process tries to send commands: refused by sender identity." |
| 1:35-2:10 | Terminal: `pidin` showing priorities and the pilot REPLY-blocked; `cat /dev/reflex`; the 20 lines of `rx_server_wait()`. | "None of this is application-level timeouts. The AI is a client that blocks on the guardian, never the reverse. One MsgReceive gets the timer pulse and the AI's message, so the guardian is one thread with no locks. The channel has fixed priority, so the AI cannot drag it down. The kernel names the sender. Status is a resource manager." |
| 2:10-2:30 | `slay -s KILL reflex-guardian`; HAM restarts it; ball still there. Pull the USB cable; MCU levels the beam. | "Who guards the guardian? The High Availability Manager restarts it and it resumes from shared memory. If the whole computer dies, the I/O node levels the beam by itself. Three rings." |
| 2:30-2:55 | Campaign table, QNX latency histogram under full CPU load, same test on Linux beside it. | "Eight faults, with and without Reflex, measured on the target. Worst-case wake-up latency at [rate] under full load: [your number] on QNX, [your number] on stock Linux on the same board." |
| 2:55-3:00 | Repo URL | "Open source. Runs in QEMU with no hardware. The safety kernel ports to any plant by replacing three functions." |

If the hardware is not ready, use the dashboard's plant view and say that the plant is simulated.

## Devpost text (edit to match what you built and measured)

**Tagline:** An AI learns to control a real machine from scratch and is never allowed to break it.

**Inspiration.** Robot learning has a hardware problem: a policy that is still learning will, sooner or later, command something destructive. Researchers work around it with simulators, cages and people holding emergency stops. Industry cannot ship a learned controller in a vehicle, a medical device or a factory robot at all, because it cannot be certified. The accepted way out is runtime assurance: a small verified monitor in front of the actuator (ASTM F3269 in aviation; "safety filters" in robotics research). Almost all of that work assumes the monitor always runs on time. That assumption is an operating-system property, and it is what QNX is for.

**What it does.** Reflex puts a hard-real-time guardian between an untrusted neural-network controller and the actuator. Each control period it checks: is the AI alive, on time, sane, and does its command keep the system inside the set of states from which a verified fallback controller can still recover. If not, the fallback takes over within that period, and the AI is told. Because of that, the AI can learn online, from an untrained network, on the physical plant. Misbehaving pilots are restarted, then quarantined. It ships with a fault injector, a live web dashboard, a `/dev/reflex` resource manager, a flight recorder and a one-command fault campaign. The demo plant is a ball on a beam.

**How I built it.** [Your own account: what you designed, the tools you used including any AI assistance, what you had to fix to get it running on QNX, how you built and tuned the beam.]

**How it uses QNX 8.0** (also paste under "Evidence of QNX use"):
- `src/ipc/rx_ipc_qnx.c`: `ChannelCreate(_NTO_CHF_FIXED_PRIORITY | _NTO_CHF_DISCONNECT)`, timer pulses via `SIGEV_PULSE_INIT`, `MsgReceive`/`MsgReply`/`MsgError`, sender check via `_msg_info.pid`, disconnect pulses.
- `src/guardian/main.c`: `SCHED_SPORADIC` budget applied to the pilot (`spawn_pilot()`), `trace_logf()` kernel trace events on mode changes.
- `src/qnx/rxfs.c`: resource manager (`resmgr_attach`, `iofunc_*`, custom `io_read`/`io_write`) at `/dev/reflex`.
- `src/qnx/rxham.c`: High Availability Manager entity, death condition and restart action.
- `Makefile`: builds with `ntoaarch64-gcc` / `ntox86_64-gcc` from SDP 8.0.
- `results/`: campaign, latency and learning reports produced on the QNX target; the video shows it on a Raspberry Pi 4.

**Why a real-time OS.** [Your measured QNX vs Linux latency under load, and what a late guardian tick means physically: at 1.2 m/s^2 of authority, every 10 ms of delay near the edge costs X mm of margin.]

**Challenges.** [Real ones only. Candidates: QNX 8.0 dropped adaptive partitioning, so CPU isolation was redesigned around sporadic scheduling; a Monte Carlo test showed the first recoverable-set bound was one-sided and wrong near the far end of the beam; learning is noisy and needed elitism to stop it diverging; sensor noise on the real beam.]

**What's next.** [Candidates: pilot in a Linux guest under the QNX Hypervisor so it can be a full PyTorch stack; QNX security policy to drop the pilot's abilities; a second plant; formal verification of the 40-line envelope function.]

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
