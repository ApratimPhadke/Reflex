# Reflex: from zero to submitted

Follow the parts in order. Each part ends with a check. Do not start the next part until the check passes.

**Deadline: 16 Dec 2026, 12:30 am IST.** Plan to submit by 13 Dec. Suggested calendar:

| Dates | Goal |
|---|---|
| 17-19 Nov | Parts 0-2: accounts, toolchain, QNX booted, repo created |
| 20-24 Nov | Part 3-4: Reflex compiles for QNX and runs with the simulator (this alone is a valid entry) |
| 25-27 Nov | Part 5: measurements on QNX, optional Linux comparison |
| 28 Nov - 6 Dec | Part 6: physical ball-and-beam |
| 7-10 Dec | Part 7: video, README results, Devpost text |
| 11-13 Dec | Buffer, submit |

What has and has not been verified before you start:

- Verified on Ubuntu 24.04 (gcc 13): the safety kernel, guardian, pilot, online learning, `rxctl`, `rxdash`, the fault campaign, the flight recorder, the test suite.
- Compiled, not yet run: the QNX-only files (`src/ipc/rx_ipc_qnx.c`, `src/qnx/rxfs.c`, `src/qnx/rxham.c`, and the `__QNXNTO__` blocks in `src/guardian/main.c`) build with zero warnings under `-Wall -Wextra` with SDP 8.0 (`ntoaarch64-gcc` and `ntox86_64-gcc` 12.2.0) on 2026-11-19, `make QNX=1 ARCH=aarch64` and `ARCH=x86_64`, including `rxfs` and `rxham` (HAM=1, TRACE=1). Running them on the target is Part 4. If your SDP lacks a library, Part 3 says what to switch off.
- Not verified: anything about your specific hardware.

---

## Part 0. Your target (5 minutes)

You have a Raspberry Pi 4: that is a supported QNX 8.0 target. Use `ARCH=aarch64` everywhere below. If you get a Pi 5 later, nothing in the software changes except the image you flash.

1. You need a micro-SD card of 32 GB or more, a 5 V 3 A USB-C supply, and either Ethernet to your router or a monitor and keyboard for first boot.
2. The Pi Zero 2 W and the spare phone are not used. QNX 8.0 images exist only for Pi 4, Pi 5 and x86_64 QEMU.
3. QEMU on your laptop is useful as a second target for quick tests, and the rules accept it, but record the video on the Pi.

**Check:** SD card and power supply in hand.

## Part 1. Accounts and toolchain (2-3 hours, mostly downloads)

Use a laptop with **Ubuntu 24.04 LTS** on an x86_64 CPU (native or dual boot is best; a VM works, but USB passthrough to the Black Pill is fiddly). Every command below assumes it.

Ubuntu 24.04 is an officially supported QNX host from **SDP 8.0.2** onward, so install the latest 8.0.x release, not the original 8.0.0.

1. Read the official rules on the Devpost page (Rules tab) once, completely. Note anything about eligibility, AI-tool use and what "created during the submission period" requires. If the rules ask you to disclose AI assistance, disclose it.
2. Install the host tools in one go (asks for your sudo password, takes 10-20 minutes):
   ```sh
   cd ~/reflex          # wherever you unzipped / cloned the project
   make setup           # = ./scripts/setup-host-ubuntu24.sh
   ```
   It installs: compilers, `python3-numpy` from apt, QEMU, `rpi-imager`, `dfu-util`, `picocom`, `arduino-cli` with the STM32 core and the VL53L0X library, USB rules for the Black Pill, and adds you to the `dialout`, `plugdev` and `kvm` groups. **Log out and back in** afterwards.

   Why a script and not the usual instructions, on 24.04 specifically:
   - `pip install numpy` fails with "externally-managed-environment" (PEP 668). The script uses `apt install python3-numpy` instead. If you ever need pip, use a venv: `python3 -m venv ~/.venv && ~/.venv/bin/pip install ...`.
   - The Arduino IDE 2 AppImage needs `libfuse2t64` (24.04's new name for libfuse2), and on 24.04 it often refuses to start because AppArmor restricts unprivileged user namespaces ("SUID sandbox helper" error; workaround `./arduino-ide*.AppImage --no-sandbox`). Reflex avoids the IDE: firmware is built with `arduino-cli` and flashed with `dfu-util`.
   - ModemManager probes new USB serial ports for several seconds and garbles the first lines. The udev rule tells it to ignore the Black Pill.
3. Go to `https://www.qnx.com/getqnx`. Create a myQNX account. Request the free non-commercial QNX SDP 8.0 licence. Accept it and deploy it to yourself in the licence portal.
4. Download **QNX Software Center** for Linux (a `.run` file). Install it as your normal user, never with sudo:
   ```sh
   chmod +x ~/Downloads/qnx-setup-*-linux.run
   ~/Downloads/qnx-setup-*-linux.run
   ```
5. In QNX Software Center: sign in, *Add Installation*, pick **QNX Software Development Platform 8.0** (latest 8.0.x), install to `~/qnx800`.
6. In the *Available* tab, search for and install:
   - Pi 4: `com.qnx.qnx800.quickstart.rpi4`
   - QEMU (optional second target): `com.qnx.qnx800.quickstart.qemu`
7. Load the environment and check everything:
   ```sh
   source ~/qnx800/qnxsdp-env.sh
   make doctor
   ```
   Add `source ~/qnx800/qnxsdp-env.sh` to the end of `~/.bashrc` so every new terminal has it.

**Check:** `make doctor` shows no FAIL lines in the *host*, *QNX SDP* and *firmware tools* sections. WARN lines about HAM or a missing Black Pill are fine for now.

## Part 2. Boot QNX (1-2 hours)

### 2A. Raspberry Pi 4 / 5

1. Find the image: `find ~/qnx800 -name "*quickstart*.img*"`. Unzip it if it is compressed.
2. Raspberry Pi Imager was installed by `make setup` (`rpi-imager`). Choose your Pi model, *Use custom* for the OS, select the `.img`, select the SD card, write. Decline OS customisation.
3. The official QSTI guide describes how to put Wi-Fi credentials on the card before first boot. Follow it: search "QNX Quick Start Target Image Raspberry Pi guide". Ethernet to your router is simpler if you have a cable.
4. Insert the card, power the Pi. With a monitor attached you will see a desktop and the IP address. Without one, try `ping qnxpi.local`.
5. From the laptop: `ssh qnxuser@qnxpi.local` (password `qnxuser`). Root password is `root`.

### 2B. QEMU

1. Follow the official "Quick Start Target Image for QEMU" page for the exact launch command of the package you installed.
2. If that gives trouble, the SDP has its own generator that always works:
   ```sh
   # QEMU and bridge-utils were installed by make setup
   mkdir ~/qnxvm && cd ~/qnxvm
   mkqnximage --type=qemu --arch=x86_64 --hostname=reflex --run
   ```
   It prints the VM's IP address. Log in as `root` (password `root`).

**Check:** on the target, `uname -a` prints a line containing `QNX` and `8.0`.

### 2C. Create the repository (must be done now, inside the submission period)

1. On GitHub create a **new public** repository named `reflex`, with the MIT licence selected. (If "Reflex" is taken or you prefer another name, rename freely; nothing in the code depends on it.)
2. On the laptop:
   ```sh
   git clone https://github.com/<you>/reflex.git
   cd reflex
   unzip ~/Downloads/reflex.zip -d .      # the project files; keep GitHub's LICENSE or the included one
   git add -A && git commit -m "Reflex: runtime assurance for physical AI on QNX 8.0" && git push
   ```
3. On GitHub, check that the *About* box on the right shows "MIT license".
4. Commit often from here on. Judges look at history for real engineering effort.

## Part 3. Build (30 minutes to half a day)

1. Sanity check on Linux first:
   ```sh
   make && make test
   ```
   Expect `SIMTEST PASSED`.
2. Build for QNX:
   ```sh
   source ~/qnx800/qnxsdp-env.sh
   make QNX=1 ARCH=aarch64          # x86_64 for QEMU
   ```
3. If it compiles: go to Part 4.
4. If it does not:
   - Error mentions `ham` (`ha/ham.h` or `-lham`): build with `make QNX=1 ARCH=... HAM=0`. Come back to HAM later.
   - Error mentions `trace_logf` or `sys/trace.h`: add `TRACE=0`.
   - Error is in `src/qnx/rxfs.c`: temporarily remove `$(OUT)/rxfs` from the `EXTRA :=` line in the Makefile to get the core running, fix `rxfs` afterwards.
   - Error is in `src/ipc/rx_ipc_qnx.c` or `src/guardian/main.c`: this is the core. Copy the full compiler output and the file name into a chat with me and I will fix it. Typical causes are a renamed macro or a missing header. Also open the function's page in the QNX 8.0 C Library Reference and compare the signature.
5. Every fix you make: commit it with a message that says what was wrong.

**Check:** `ls build/qnx-<arch>/` shows `reflex-guardian reflex-pilot rxctl rxdash rx_simtest` (plus `rxfs`, `rxham` if enabled).

## Part 4. Run on QNX with the simulated plant (1 hour)

1. Set up password-less SSH once (`ssh-copy-id qnxuser@<target>`), then build and copy in one step:
   ```sh
   PI=qnxuser@<target> make deploy                  # Pi 4
   PI=root@<qemu-ip> ARCH=x86_64 make deploy        # QEMU
   ```
2. On the target:
   ```sh
   cd ~/reflex
   ./rx_simtest                     # must print SIMTEST PASSED
   ./reflex-guardian --fresh &
   ```
   It prints two start-up lines and then stays silent on purpose (a real-time loop does not print). Look inside with the tools in the next step. Within about a second the mode becomes `AI`.
3. In a second SSH session:
   ```sh
   cd ~/reflex
   ./rxctl status
   ./rxdash &
   ```
   Open `http://<target-ip>:8080` in your laptop's browser. You should see the ball tracking a setpoint that jumps every 4 seconds.
4. Break things. Watch the dashboard while you run each line:
   ```sh
   ./rxctl inject crash
   ./rxctl inject flip ; sleep 5 ; ./rxctl clear ; ./rxctl release
   ./rxctl guard off ; ./rxctl inject flip     # ball is lost: this is the system without Reflex
   ./rxctl clear ; ./rxctl guard on
   ```
5. Resource manager (needs root):
   ```sh
   su root -c "./rxfs &"
   cat /dev/reflex
   echo "fault nan" > /dev/reflex ; cat /dev/reflex ; echo "fault clear" > /dev/reflex
   ```
6. Look at it the QNX way: `pidin | grep reflex` shows the guardian at priority 50, the pilot at 10 and REPLY-blocked on the guardian.
7. Spoof test. Start a second pilot by hand: `./reflex-pilot $(./rxctl endpoint)`. It must exit immediately, `./rxctl status` must show `spoofed senders 1 refused`, and `./rxctl events` must list it.
8. Sporadic budget: stop the guardian (`slay reflex-guardian`), restart with `./reflex-guardian --fresh --sporadic &`. `pidin -P reflex-pilot sched` shows the healthy pilot at `10s`. Run `./rxctl inject spin`: the pilot alternates between `10s` and `1s` while spinning, the shell stays responsive, and the guardian kills it as unresponsive after about 1 s.
9. HAM. The Pi 4 quick-start image has no `ham`; `make deploy` copies the SDP's into `~/reflex`. `qnxuser` may use `sudo` on the QSTI.
   ```sh
   slay reflex-guardian
   sudo sh -c "./ham > ham.log 2>&1 < /dev/null &"   # redirect: see the note below
   sudo ./rxham $PWD/reflex-guardian                 # HAM starts the guardian (as root)
   sudo slay -s KILL reflex-guardian                 # kill the guardian itself
   ./rxctl status                                    # "guardian starts" +1, ball still on the beam
   sudo ./rxham -r                                   # stop monitoring
   ```
   Notes from the Pi 4 (Phase 2): a guardian started by HAM runs as root, so stop it with `sudo slay`. `ham` runs as two processes (manager and a watcher that restarts it) and does not stop on `slay`; use `sudo kill -9` on both pids from `sudo pidin -f an | grep ham` (as `qnxuser`, `pidin` does not list root's processes). Always start `ham` with its output redirected: a `ham` left behind by a closed ssh session hands its dead stdout to every guardian it restarts. The guardian now ignores `SIGPIPE` from its first line, so this no longer kills it, but the log would be lost.

10. Learning from nothing. This is the centrepiece of the demo:
    ```sh
    slay reflex-guardian ; rm -f reflex-policy.bin
    ./reflex-guardian --fresh --hz 250 --learn &
    ./rxctl period 2000
    ```
    Watch the dashboard's *Learning on the live plant* panel for 5-15 minutes. The green bars (interventions per episode) should thin out and the blue cost line should fall while *Balls lost* stays 0. Learning is noisy: expect relapses. Then:
    ```sh
    ./rxctl inject crash        # kill the learner mid-training: it resumes from its last snapshot
    ./rxctl train off           # stop exploring, fly the best weights found
    ```
11. Flight recorder: `mkdir incidents ; ./rxctl incidents incidents &` then inject a fault and open the CSV it writes.

If something behaves differently from this description, that is a bug to fix now, not later. Note what you saw and bring it to me.

**Check:** dashboard works, `flip` with the guardian on keeps the ball, `flip` with the guardian off loses it, and a `--learn` run improves with zero balls lost.

At this point you have a complete, valid submission. Everything after this makes it stronger.

## Part 5. Collect the evidence (half a day)

Numbers you measure yourself on QNX are the core of the "why a real-time OS" argument. Never copy numbers from anywhere else.

1. Clean run:
   ```sh
   ./reflex-guardian --fresh &
   sleep 5 ; ./rxctl reset
   ./rxctl campaign campaign-qnx.md
   ```
2. Latency under load. In one session `./rxctl reset ; ./rxctl hog 40 60` (every core spinning at priority 40, below the guardian's 50). When it ends: `./rxctl latency > latency-qnx-loaded.txt`.
3. Long soak: leave it running for an hour with a fault script, then `./rxctl latency` and `./rxctl status`:
   ```sh
   while true; do for f in crash flip nan freeze stuck; do ./rxctl inject $f; sleep 6; ./rxctl clear; ./rxctl release; sleep 6; done; done
   ```
4. Learning curve: run step 10 of Part 4 for 20 minutes with `./rxctl log learn.csv 1200` running, screenshot the learning panel at the start, middle and end, and note total interventions and balls lost from `./rxctl status`. Repeat once more from scratch so you can say it was not a lucky run.
5. Copy the files back: `scp qnxuser@<target>:reflex/*.md qnxuser@<target>:reflex/*.txt results/`. Commit them under `results/`.
6. Strongly recommended: the Linux comparison on the same Pi 4. Flash Raspberry Pi OS on a second SD card, `make` on the Pi, run the same steps 1-2 with `sudo ./reflex-guardian --fresh` and `sudo ./rxctl hog 40 60`. Report both honestly, including where Linux does fine. State kernel version and that it is a stock (non-PREEMPT_RT) kernel.
7. Put the campaign table, the latency histograms, the learning-curve screenshots and one dashboard screenshot into the README's Results section.

**Check:** `results/` in your repo contains files produced on your QNX target.

## Part 6. The physical beam (about a week of evenings)

Everything about the hardware is in **[HARDWARE.md](HARDWARE.md)**: geometry, mechanical build, wiring, firmware, calibration, connecting to QNX, first closed loop and troubleshooting. Follow it part by part.

Keep the simulator demo as your fallback for the video if the hardware is not solid by 8 Dec.

**Check:** the real ball survives `./rxctl inject flip` with the guardian on, and falls with the guardian off.

## Part 7. Submit

Work through `docs/SUBMISSION.md`: video script, Devpost text, evidence-of-QNX-use list, final checklist.

## Understand what you are submitting

Judges score "genuine understanding of the problem space" and may ask questions at the coffee chat. Before you submit, be able to explain without notes:

1. Why the pilot is the *client* and the guardian the *server*, and what would break if it were the other way round.
2. What `_NTO_CHF_FIXED_PRIORITY` changes, and what priority inheritance would do without it.
3. The `rx_margin()` formula term by term.
4. Why a stuck or flipped output is not caught by a watchdog, and which rule catches it.
5. Why QNX 8.0 has no adaptive partitioning and what `SCHED_SPORADIC` gives you instead.
6. How the learner estimates a gradient from two episodes, and why the guardian does not need to know the pilot is learning.
7. What the Monte Carlo test proves and what it does not.

Read `src/core/rx_core.c`, `src/core/rx_learner.c` and `src/ipc/rx_ipc_qnx.c` line by line until you can. Change something (a gain, a threshold, a new fault type) and see the effect; that is the fastest way to own the code.
