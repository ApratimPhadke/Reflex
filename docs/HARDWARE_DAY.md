# Hardware day: from an assembled beam to the recorded demo

One page, in order. Everything before this (Pi, QNX, USB serial at boot, the wizard, the demo script) was
prepared and tested without the beam; see "What is already verified" at the end. Keep hands clear of
the push-rod whenever the servo has power.

**1. Power and wiring checks (multimeter, nothing plugged into USB yet).** `docs/HARDWARE.md` 4.4:
4.9-5.3 V at the servo adapter, 3.2-3.4 V at the Black Pill's 3V3, adapter − continuous with Black Pill
GND, adapter + not continuous with 3V3. Servo red/brown go to the adapter only, never to the Black Pill.

**2. Real firmware (laptop).** The board currently runs the v2 emulator build.
```sh
make fw                                        # real build: sensor + servo (BOARD=BLACKPILL_F401CC)
# with the ST-LINK on the SWD header (no buttons), servo supply OFF:
st-flash --reset write build/firmware/reflex_io.ino.bin 0x08000000
# or DFU: hold BOOT0, tap NRST, release BOOT0, then: make flash
```

**3. Bench check on the laptop** (unplug the ST-LINK first, then USB-C to the laptop; servo supply ON).
`scripts/fw.sh monitor`: `I reflex_io 2 BLACKPILL_F401CC 0`, `# reflex_io v2 sensor=ok` every 2 s,
`M …` 50 times a second that follow the ball. Type `S 1600` / `S 1400`: the beam tilts both ways and
re-levels 150 ms later. Details: `docs/HARDWARE.md` Part 6.

**4. Plug into the Pi and calibrate.** USB-C from the Black Pill to any Pi USB port. The Pi starts the USB
serial driver at boot by itself (`scripts/install-autostart.sh`), so `/dev/serusb1` appears.
```sh
ssh qnxpi
cd ~/reflex && ./reflex-demo.sh stop      # rxcal will not run while a guardian owns the servo
./rxcal                                   # 8 steps, about 5 minutes, phone inclinometer ready
```
Step 7 must say PASS. If it does not, the wizard tells you which step to look at (`docs/HARDWARE.md` Part 7).

**5. Start the demo in stages.**
```sh
./reflex-demo.sh start --no-pilot         # fallback controller only: ball settles at the setpoint
./reflex-demo.sh stop; ./reflex-demo.sh start                    # + the hardware-trained AI pilot
./reflex-demo.sh stop; ./reflex-demo.sh start --learn-from-sim   # + learning on the real beam (15-30 min)
```
The dashboard address is printed by `start` (`http://qnxpi.local:8080/`). Check, in this order: ball
settles; `./rxctl inject flip` with the guardian on keeps the ball; pulling the USB cable levels the beam
within 150 ms; `sudo slay -s KILL reflex-guardian` is restarted by HAM.

**6. Measure the margin before trusting it.** The stuck-at-full-tilt fault is the worst case the safety
margin is sized for; on the fake node it came within 10.0-14.1 mm of the end against the 10 mm keep-out
strip (`results/2026-11-20-host-hw-pilot/`). On the real beam, film `./rxctl inject stuck` a few times
side-on with a ruler in frame. If the ball enters the last 10 mm, stop and bring the footage: the
actuator delay `t_delay` in `rx_cfg_hardware()` (0.12 s) then needs to be set from the measured delay,
and that is a safety-margin change to decide with data, not to tweak.

**7. Evidence.** Raw logs of every run go to `results/<date>-pi4-real/` with `uname -a`, the git hash and
the exact commands (`./rxctl campaign campaign.md`, `./rxctl latency`, `./rxctl events`).

**8. Record the video.** Dashboard full screen beside the beam, phone on a tripod. Check the script, then play it:
```sh
./rxctl script demo/video.rxs --check
./rxctl script demo/video.rxs             # captions appear on the dashboard; "say" cues appear here
```

## When it misbehaves

| Symptom | Where to look |
|---|---|
| `reflex-demo.sh` says "no I/O node" | `ls /dev/serusb*`; `cat /dev/shmem/reflex_startup.log`; `sudo /system/etc/startup/reflex_startup.sh` |
| "no valid calibration" | run `./rxcal` (HARDWARE.md Part 7) |
| Event log: "firmware speaks protocol v1" | step 2 above |
| Ball races to one end every time | a sign: rerun `./rxcal` |
| Ball oscillates, growing | HARDWARE.md Part 9.1 table |
| Guardian intervenes all the time, reason `envelope`, ball calm | real plant slower than the model: HARDWARE.md 9.2 (a margin change: decide with data) |
| Servo twitches, Black Pill resets, readings jump | HARDWARE.md Part 10 |
| The Pi reboots by itself | `~/reflex/health-log.sh &` then look at the last line of `health.log` after the reboot (under-voltage bits); use the official 5.1 V 3 A supply |

## What is already verified (2026-11-20)

| What | Where | Evidence |
|---|---|---|
| Pi ↔ USB ↔ Black Pill (emulated plant), campaign 8/8, unplug/replug | Pi 4, QNX 8.0 | `results/2026-11-20-pi4-emulated-io/` |
| USB serial driver started at boot | Pi 4 | `results/2026-11-20-pi4-emulated-io/09-autostart-after-reboot.txt` |
| `rxcal --auto` recovers a wrong-in-every-way calibration; guardian refuses to run without one; campaign 8/8 on that rig | laptop, fake I/O node | `make test-cal`, `make test-serial` |
| `reflex-demo.sh start/stop/status`, HAM restart, captions | Pi 4, simulated plant | this session; see `results/2026-11-20-pi4-emulated-io/README.md` |
| Firmware v2 (real and emulator) compiles for the F401CC | laptop | `make fw`, `make fw EMULATE=1` |
| Firmware v2 (emulator) on the board; `rxcal --auto` over QNX recovers its hidden calibration exactly; `reflex-demo.sh` picks the serial backend; campaign 8/8 with all faults injected | Pi 4 + Black Pill | `results/2026-11-20-pi4-emulated-io/` 13-15 |
| **Not verified:** the real (sensor + servo) firmware on the board, and anything with the real sensor and servo | | |
