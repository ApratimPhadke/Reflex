# Prompt for Claude Code: take Reflex to plug-and-play

## How to use this (for you, Apratim)

Do these once, in a terminal, before starting Claude Code:

```sh
cd ~/reflex                                   # the project folder (unzipped v4)
git init -q 2>/dev/null; git add -A; git commit -qm "Reflex v4 baseline" || true
source ~/qnx800/qnxsdp-env.sh                 # Claude Code inherits this environment
ssh-copy-id qnxuser@<pi-ip>                   # password-less SSH to the Pi (password: qnxuser)
export PI=qnxuser@<pi-ip>
claude                                        # start Claude Code in the project folder
```

Then type this one line into Claude Code:

> Read docs/CLAUDE_CODE_PROMPT.md, section "The prompt", and carry it out.

Or paste everything under "The prompt" yourself. `CLAUDE.md` in the project root gives Claude Code the project rules automatically.

Keep the Pi powered and on the network, and the Black Pill plugged into the laptop or the Pi, as each phase asks. Claude Code will stop at every checkpoint and before anything that needs your hands (DFU buttons, servo power) or that is risky (sudo, pushing to GitHub).

---

## The prompt

You are working on **Reflex**, my entry for the QNX Global Hackathon 2026 (deadline 16 Dec 2026, 00:30 IST; I want to submit by 13 Dec). Read `CLAUDE.md` first and follow its invariants and "ask before" rules throughout.

**Goal.** Make the software side complete, verified and boringly reliable, so that the day I finish assembling the beam I only have to plug it in, run one calibration wizard, and start the demo. Hardware is not assembled yet; the Raspberry Pi 4 (QNX 8.0 quick-start image) and the STM32 Black Pill are available. My host is Ubuntu 24.04.

**How to work.**
- Keep a task list. Work through the phases in order. Inside a phase, work autonomously.
- At the end of each phase, stop and give me a checkpoint report: what you changed, what you verified and on which machine (host / QEMU / Pi / Black Pill), the evidence (command + output file), and what is still unverified. Then wait for me to say "continue".
- Commit at least once per phase with a clear message.
- When a QNX API, a QSTI detail, or a board option is uncertain, look it up (headers in `$QNX_TARGET/usr/include`, the QNX 8.0 docs, `arduino-cli board details`) or inspect the target over SSH. Do not guess and do not leave guesses in code or docs.
- If something cannot be done (missing library, hardware not present), say so plainly, implement the best fallback, and mark it in the report.
- Never fabricate measurements. Numbers in docs come only from files in `results/`.

### Phase 0. Orient (read-only)

1. Read `README.md`, `docs/ARCHITECTURE.md`, `docs/GUIDE.md`, `docs/HARDWARE.md`, then skim every file in `src/`, `firmware/`, `scripts/`, `tools/`.
2. Run `make doctor`, `make && make test`, and `PI=$PI make doctor`.
3. Report: the state of the host and target, anything in the code that looks wrong or risky to you, and your plan for Phases 1-8 with a rough time estimate each.

**Checkpoint 0.**

### Phase 1. QNX build: zero errors, zero warnings

1. `make QNX=1 ARCH=aarch64` and `make QNX=1 ARCH=x86_64`. The QNX-only files (`src/ipc/rx_ipc_qnx.c`, `src/qnx/rxfs.c`, `src/qnx/rxham.c`, the `__QNXNTO__` parts of `src/guardian/main.c`) have never been compiled. Fix every error and warning against the real SDP headers.
2. Use `HAM=0` or `TRACE=0` only if `make doctor` / the SDP proves the library is absent; if so, document it and keep the code path buildable when it is present.
3. Keep the Linux build and `make test` green.

**Done when:** both QNX architectures and Linux build with no warnings; `make test` passes. **Checkpoint 1.**

### Phase 2. Run everything on the Pi with the simulated plant

Use `PI=$PI make deploy`. On the target, work through every step of `docs/GUIDE.md` Part 4 and make each one actually behave as described:

1. `rx_simtest` passes on the Pi.
2. Guardian at 1 kHz with the simulator, `rxdash` reachable from the laptop browser, `rxctl status/events/latency` sane.
3. Every fault: `crash freeze spin slow nan huge stuck flip`, with guardian on (ball kept) and off (ball lost where expected). `rxctl campaign` completes.
4. `pidin` confirms guardian priority 50, pilot 10, pilot REPLY-blocked on the guardian.
5. Spoof test: a hand-started pilot is refused and logged.
6. `--sporadic`: `rxctl inject spin` does not starve the shell; confirm with `pidin`/`hogs`.
7. `rxfs` as root: `cat /dev/reflex`, `echo "fault nan" > /dev/reflex`.
8. HAM: find or copy `ham` onto the target, register with `rxham`, kill the guardian with SIGKILL, show it restarts and the simulated ball survives. Check `rxham -r`.
9. Learning from scratch (`--learn --hz 250`, `rxctl period 2000`) for 10 minutes: improves, zero balls lost; kill the learner mid-run and show it resumes.
10. System Profiler: confirm the guardian's `trace_logf` events appear in a kernel trace (`tracelogger` on the target), or document why not.

Fix every bug you find in the code, not in the docs. Save raw logs to `results/<date>-pi4-sim/`.

**Done when:** all ten work on the Pi and logs are saved. **Checkpoint 2.**

### Phase 3. Prove the serial path before the mechanics exist

The weakest untested link is Pi ↔ USB ↔ Black Pill. Make it testable without the beam.

1. **Host-side fake I/O node.** `tools/fake_io_node.py`: opens a pty, speaks the I/O node protocol, simulates the ball-and-beam with the same physics as `rx_plant_step()`, VL53L0X-like noise (±3 mm) and 1 mm quantisation, 50 Hz, servo slew, and the 150 ms watchdog. Add a `make test-serial` target that runs the Linux guardian with `--backend serial --dev <pty>` against it and runs the fault campaign. Wire it into CI.
2. **On-board emulator firmware.** A build flag (`make fw EMULATE=1`) that makes the Black Pill run the same simulated plant internally instead of reading the sensor and driving the servo, speaking the identical protocol over USB. No sensor or servo attached.
3. **Firmware compile.** Get `make fw` to compile for the real board (ask me which chip is printed on it). Then ask me to put the board in DFU mode and flash the emulator build.
4. **On the Pi.** Plug the Black Pill into the Pi. Find out how the QSTI exposes a USB CDC-ACM device (`/dev/serusb*` via `devc-serusb`, or something else). Make it work, and put the exact steps into `docs/HARDWARE.md` Part 8 and a target script. Run the guardian with `--backend serial` against the emulator board: fallback-only, then the pilot, then the fault campaign, then 10 minutes of `--learn-from-sim`. Save logs to `results/<date>-pi4-emulated-io/`.
5. Make the serial reader robust: garbage lines, partial lines, line bursts after a stall, a disconnected and re-plugged cable (guardian survives, levels the beam, reconnects). Add a fuzz test for the parser.

**Done when:** the full chain Pi → USB → Black Pill (emulated plant) runs the campaign; unplugging the cable is handled; tests cover the parser. **Checkpoint 3.**

### Phase 4. Calibration without reflashing

Today every calibration step means editing `#define`s and reflashing. Replace that with a host-side calibration file and a wizard.

1. **Protocol v2** (document it in `docs/ARCHITECTURE.md`):
   - node → host: `M <raw_mm>` raw sensor reading or `M !` for no valid reading; `# ...` comment lines; `I reflex_io <proto> <board> <emulated 0|1>` on reset and on request.
   - host → node: `S <pulse_us>` servo pulse; `L <pulse_us>` level pulse used by the watchdog (store it in flash with the STM32duino EEPROM emulation so the beam levels correctly even before the Pi has connected); `?` to request the `I` line.
   - The firmware keeps hard limits on the pulse (compile-time `SERVO_MIN_US`/`SERVO_MAX_US`, default 1000-2000 µs) that the host cannot override, and the 150 ms watchdog.
2. **`reflex.conf`** (plain `key = value` text, next to the binaries on the Pi): `level_us`, `us_per_mrad`, `servo_dir`, `center_mm`, `sensor_dir`, `range_min_mm`, `range_max_mm`, `half_len_m`, `calibrated_at`, `board`. The guardian's serial backend converts between metres/radians and raw mm/µs using it. Without a valid file, `--backend serial` refuses to start and tells me to run the wizard (`--uncalibrated` overrides for debugging).
3. **`rxcal`**, an interactive terminal wizard (runs on the Pi; also on the laptop against `/dev/ttyACM0`), one screen per step with a clear instruction, live readout and keys:
   1. link check (`I` line, sensor present);
   2. level: arrow keys nudge `level_us` while I read a phone inclinometer;
   3. direction, automatic: ball near centre, small tilt each way, measure which way the reading moves, set `sensor_dir`; ask me to confirm that the ball visibly rolled;
   4. centre: hold the ball at the pivot mark, average 2 s;
   5. range: ball at near stop, at far stop, no ball; compute the window and warn if far stop and "no ball" are within 30 mm;
   6. angle scale: command ±200 mrad, I type the inclinometer readings, compute `us_per_mrad`, warn on asymmetry > 1°;
   7. validation: 20 s fallback-only closed loop, ball must stay inside ±150 mm; print pass/fail;
   8. write `reflex.conf`, keep a timestamped backup of the old one.
   The wizard must say "keep hands clear" before any step that moves the servo and wait for Enter.
4. Make `rxcal` testable against `tools/fake_io_node.py` (give the fake node configurable wrong signs and offsets and check the wizard finds them).
5. Update `docs/HARDWARE.md` Parts 6-7 to use `rxcal` instead of editing constants.

**Done when:** against the fake node and the emulator board, `rxcal` recovers deliberately wrong calibration values, and the guardian runs from `reflex.conf`. **Checkpoint 4.**

### Phase 5. One command to run the demo

1. `reflex-demo.sh start|stop|status` for the target: starts `ham` + `rxham` + guardian, `rxfs`, `rxdash`; picks `--backend serial` if an I/O node and a valid `reflex.conf` are present, otherwise the simulator; prints the dashboard URL.
2. Optional start at boot: inspect how the QSTI runs startup scripts and propose the least invasive way. Ask me before changing anything on the image.
3. Dashboard: show the backend (sim / serial / emulated board), calibration date, I/O node link state, and a large readable status for the video.
4. `rxctl script <file>`: run a timed sequence of commands (for example, wait 5 s, guard off, inject flip, wait 3 s, clear, guard on). Write `demo/video.rxs` that plays the storyline in `docs/SUBMISSION.md`, with captions shown in a dashboard banner, so the video can be recorded in one take.

**Done when:** a cold-booted Pi plus `./reflex-demo.sh start` gives a working demo in either backend, and the video script plays end to end. **Checkpoint 5.**

### Phase 6. Evidence, generated not typed

1. `make evidence PI=...`: on the target, run the campaign, a 60 s idle latency run, a 60 s run under `rxctl hog 40 60`, and a 15-minute learning run; collect into `results/<date>-<target>-<backend>/` with `uname -a`, SDP version, git hash, exact commands and raw CSVs.
2. `tools/report.py`: turn a results folder into `report.md` plus PNG plots (latency histogram idle vs loaded, learning curve with interventions per episode, campaign table) using matplotlib from apt.
3. Linux A/B on the same Pi: `scripts/linux-ab.sh` that, on Raspberry Pi OS booted from a second SD card, builds the Linux version natively and runs the same latency and campaign commands, saving to `results/<date>-pi4-linux/`. Write the instructions; I will boot the card. Report both systems honestly, including where Linux does fine.
4. README "Results" section generated from the latest results folders, with links to the raw files.

**Done when:** `make evidence` on the Pi with the simulated plant produces a complete report with plots. **Checkpoint 6.**

### Phase 7. Quality pass

1. Review the whole codebase against the `CLAUDE.md` invariants and fix violations. Look especially at: atomics and memory ordering in `rx_shm.c`, integer overflow in tick arithmetic, every error path of the QNX IPC code, signal handling, file-descriptor and process leaks in supervision.
2. Unit tests: learner save/load round-trip and corrupt-file handling, `rx_command` parsing, the serial protocol, `reflex.conf` parsing (missing keys, bad values).
3. Run the guardian under the Linux sanitizers (ASan/UBSan build target) through the campaign; fix everything they find.
4. CI: host build, `make test`, `make test-serial`, sanitizer build, shellcheck on `scripts/`.
5. Docs: make every command in `docs/*.md` match reality; remove anything that is now wrong. Add a section to `docs/GUIDE.md` listing what was verified on which machine and date.

**Done when:** CI is green, sanitizers are clean, docs match the code. **Checkpoint 7.**

### Phase 8. Hardware-day kit

Write `docs/HARDWARE_DAY.md`, one page, the exact sequence for the day the beam is assembled, assuming all earlier phases are done:

1. multimeter checks (from `docs/HARDWARE.md` 4.4);
2. `make fw && make flash` (real build, not emulator);
3. bench check on the laptop with `scripts/fw.sh monitor`;
4. plug into the Pi; `./rxcal`;
5. `./reflex-demo.sh start`; fallback-only first, then pilot, then learning;
6. `make evidence` with the real beam;
7. record the video with `rxctl script demo/video.rxs`.

Include a "when it misbehaves" table that points into `docs/HARDWARE.md` Part 10.

**Final report.** List every file you added or changed, what is verified (with evidence paths), what is not, and the three biggest remaining risks before submission. Remind me to: create the public GitHub repository myself, check the Devpost rules on AI-assisted work and disclose it if required, and record numbers only from `results/`.
