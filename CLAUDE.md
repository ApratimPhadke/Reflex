# Reflex: project notes for Claude Code

Reflex is an entry for the QNX Global Hackathon 2026 (deadline 16 Dec 2026, 00:30 IST; aim to submit by 13 Dec). It is a hard-real-time guardian on QNX 8.0 that lets an untrusted neural-network pilot control an unstable plant (ball on a tilting beam), checks every command one control period ahead, and takes over with a verified fallback controller before the plant can fail. The pilot can learn online on the real hardware because the guardian makes failure impossible.

Read `README.md`, `docs/ARCHITECTURE.md`, `docs/GUIDE.md` and `docs/HARDWARE.md` before changing anything substantial.

## Setup you are working in

- Host: Ubuntu 24.04 LTS, x86_64. QNX SDP 8.0.x (8.0.2 or newer) in `~/qnx800`. The QNX environment must be loaded in the shell: `source ~/qnx800/qnxsdp-env.sh` (check `$QNX_TARGET`).
- Target: Raspberry Pi 4 running the QNX 8.0 quick-start image (QSTI). Login `qnxuser`/`qnxuser`, root/`root`. Address in `$PI` (for example `qnxuser@qnxpi.local`). Second target: QNX in QEMU (`ARCH=x86_64`).
- I/O node: STM32 Black Pill **F401CCU6** (`BOARD=BLACKPILL_F401CC`, the default) on USB, running `firmware/reflex_io`. Sensor VL53L0X on I2C (PB7 SDA, PB6 SCL), servo MG996R on PA8 with its own 5 V supply.
- The Pi Zero 2 W cannot run QNX 8.0 and is not part of this project.

## Commands

```sh
make && make test                      # Linux reference build + full offline test suite (must always pass)
make test-serial                       # serial path end to end against tools/fake_io_node.py (~5 min, includes test-cal)
make test-cal                          # rxcal wizard against a fake rig with a wrong hidden calibration (~35 s)
make QNX=1 ARCH=aarch64                # QNX build for the Pi 4 (HAM=0 / TRACE=0 only if those libs are truly missing)
make doctor [PI=...]                   # host + target readiness check
PI=qnxuser@<ip> make deploy            # build for QNX and copy to ~/reflex on the target (+ reflex-demo.sh, demo/)
PI=qnxpi scripts/install-autostart.sh  # target boot hook: USB serial driver at boot (--demo / --remove / --status)
make fw / make flash [EMULATE=1]       # firmware via arduino-cli + dfu-util (BOARD=BLACKPILL_F401CC default)
python3 tools/train_policy.py          # retrain the simulator pilot (numpy from apt)
python3 tools/train_policy.py --hardware   # retrain the serial-backend pilot (src/core/policy_weights_hw.h)
```

## Map

```
src/core/rx_core.c     safety kernel: recoverable-set bound, fallback controller, guard decision, plant model, frozen policy
src/core/rx_learner.c  online evolution-strategy learner for the pilot
src/core/rx_shm.c      shared-memory telemetry, event log, text commands (used by rxctl/rxdash/rxfs)
src/core/rx_link.c     I/O node line protocol v2: strict parser + command formatter (pure, fuzzed by rx_linktest)
src/core/rx_conf.c     reflex.conf: parse/check/format, raw mm/us <-> m/rad (pure except load/save)
src/ipc/rx_ipc_qnx.c   QNX channel/pulse/Send-Receive-Reply backend  (QNX only)
src/ipc/rx_ipc_posix.c Linux timerfd + SOCK_SEQPACKET backend (A/B baseline, host tests)
src/guardian/main.c    the real-time guardian process
src/guardian/rx_io.c   plant I/O: simulator and serial link to the I/O node
src/pilot/main.c       untrusted pilot process (frozen or learning) with fault-injection hooks
src/tools/             rxctl (CLI, campaign, script, flight recorder), rxdash (web UI), rxcal (calibration wizard), rx_simtest/rx_linktest (tests)
src/qnx/               rxfs (resource manager /dev/reflex), rxham (High Availability Manager registration)
firmware/reflex_io/    I/O node sketch (Black Pill / Arduino)
tools/fake_io_node.py  fake I/O node on a pty (same plant as rx_plant_step), used by make test-serial
scripts/               Ubuntu 24.04 setup, doctor, firmware build/flash, deploy, test-serial/test-cal, install-autostart
scripts/target/        runs on the Pi: reflex-demo.sh (start|stop|status), boot hook, health-log.sh
demo/video.rxs         the video as an rxctl script
```

## Invariants: never break these

1. The guardian's control loop never blocks on the pilot, never allocates, never takes a lock, never prints, and has exactly one blocking call (`rx_server_wait`). Diagnostics go to the shared-memory event log.
2. `src/core/rx_core.c` stays pure: no OS calls, no globals with state, unit-testable offline.
3. The pilot is untrusted. It receives only the guardian's pid and channel id; it never maps the guardian's shared memory. Nothing it reports is used for a safety decision.
4. The guardian does not change its decision because the pilot is learning. Training mode only suppresses strike counting.
5. `make test` must pass. Never weaken a test, threshold or margin to make a test pass; find the real cause. If a safety margin must change, explain why with data and ask first.
6. Both builds (Linux gcc and QNX qcc/ntoaarch64-gcc) compile with zero warnings under `-Wall -Wextra`.
7. No invented numbers. Any latency, success rate, learning time or other measurement in a document must come from a file in `results/` produced on the named target, with the command that produced it. If something was not run, say "not verified".
8. The I/O node levels the beam by itself if commands stop for 150 ms, and clamps the servo to a hard range independent of the host. Keep both.

## Ask the user before

- flashing firmware, or putting the Black Pill in DFU mode (they must press buttons);
- any command that can move the servo while the beam is assembled (keep hands clear);
- anything with `sudo` on the host or root on the target that changes system configuration;
- changing safety-kernel thresholds (`rx_cfg_default`, `rx_cfg_hardware`, `rx_margin`, `rx_safe_u`);
- `git push`, creating the GitHub repository, or anything public.

## Conventions

- C11 (`-std=gnu11`), 4-space indent, `rx_` prefix, MIT SPDX header on every source file.
- QNX-specific code stays behind the IPC/IO interfaces or in `src/qnx/`; the Linux build must keep working.
- When a QNX API is in doubt, read the header in `$QNX_TARGET/usr/include` and the QNX 8.0 documentation; do not guess signatures.
- Small commits with messages that say what changed and why. Keep `docs/` in step with the code.
- Raw logs of every on-target run go to `results/<YYYY-MM-DD>-<target>/` with `uname -a`, the git hash and the exact command.
