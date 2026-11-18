# Reflex roadmap: from nothing to submitted

The same steps as the Whiteboard flowchart, as a checklist. Deadline 16 Dec 2026 00:30 IST; aim to submit by 13 Dec. A check that fails sends you to its fix; then retry the step before it.

## A · Accounts, parts and laptop · 18-20 Nov

- [ ] **A1** Read the Devpost **Rules** tab fully: eligibility, AI use, new-repo rule
- [ ] **A2** Confirm you are registered at qnx.devpost.com
- [ ] **A3** Order the BOM **today**: Probots, Robocraze, local hardware shop
- [ ] **A4** Unzip Reflex v4 to `~/reflex`, then `git init` and first commit
- [ ] **A5** `make setup` (Ubuntu 24.04 script), then log out and back in
- [ ] **A6** Create a myQNX account at qnx.com/getqnx
- [ ] **A7** Request the free non-commercial SDP 8.0 licence, deploy it to yourself
- [ ] **A8** Install QNX Software Center (`.run` file, normal user, no sudo)
- [ ] **A9** Install QNX SDP 8.0, latest 8.0.x (8.0.2+), to `~/qnx800`
- [ ] **A10** Install packages `quickstart.rpi4` and `quickstart.qemu`
- [ ] **A11** `source ~/qnx800/qnxsdp-env.sh`, add it to `~/.bashrc`
- [ ] **Check:** `make doctor`: no FAIL?  
      *If not:* Read the FAIL line, fix that item or rerun `make setup`
- [ ] **Milestone: Laptop ready**

## B · QNX on the Raspberry Pi 4 · 20-22 Nov

- [ ] **B1** Find the image: `find ~/qnx800 -name "*quickstart*.img*"`
- [ ] **B2** `rpi-imager`: Use custom, pick the `.img`, write to the 32 GB SD card
- [ ] **B3** Wi-Fi: follow the QSTI guide, or just use an Ethernet cable
- [ ] **B4** SD card in, Ethernet in, power with the official 5.1 V 3 A supply
- [ ] **B5** Find the Pi's IP: router page, monitor, or `ping qnxpi.local`
- [ ] **B6** `ssh qnxuser@<ip>` (password `qnxuser`)
- [ ] **Check:** `uname -a` says QNX 8.0?  
      *If not:* Reflash. Check it is the rpi4 image and the supply is official
- [ ] **B7** `ssh-copy-id qnxuser@<ip>`, then `export PI=qnxuser@<ip>`
- [ ] **Check:** `PI=$PI make doctor` SSH passes?  
      *If not:* Check IP, network, ssh-copy-id; try Ethernet
- [ ] **B8** On GitHub: new **public** repo `reflex`, MIT licence
- [ ] **B9** `git remote add origin ...` and push the baseline
- [ ] **Milestone: QNX target ready**

## C · Software on QNX with Claude Code · 21-25 Nov

- [ ] **C1** In `~/reflex`: source env, `export PI`, run `claude`
- [ ] **C2** Tell it: *Read docs/CLAUDE_CODE_PROMPT.md and carry it out*
- [ ] **C3** Phase 0: read its report, answer its questions, say *continue*
- [ ] **C4** Phase 1: it builds for QNX and fixes every error
- [ ] **Check:** QNX build: 0 errors, 0 warnings?  
      *If not:* Let it fix; paste errors; HAM=0 only if the lib is missing
- [ ] **C5** Phase 2: it runs all ten checks on the Pi, simulated plant
- [ ] **C6** You: open `http://<pi-ip>:8080`, inject flip, guard on vs off
- [ ] **Check:** Campaign: ball kept 8/8 guarded?  
      *If not:* Claude Code debugs on the target; do not edit tests
- [ ] **C7** Learning from scratch, 10 min, 0 balls lost
- [ ] **C8** Logs in `results/...-pi4-sim/`, commit and push
- [ ] **Milestone: A valid entry exists: sim on real QNX**

## D · Serial path, calibration, demo tooling · 25-30 Nov

- [ ] **D1** Read the chip on the Black Pill (F411CE or F401CC), tell Claude Code
- [ ] **D2** Phase 3: fake I/O node, `make test-serial` on the laptop
- [ ] **D3** Black Pill to laptop: hold BOOT0, tap NRST, release = DFU
- [ ] **D4** `make fw EMULATE=1 && make flash` (board simulates the beam)
- [ ] **D5** Plug the Black Pill into the Pi; look for `/dev/serusb*`
- [ ] **Check:** Guardian runs over serial vs emulated board?  
      *If not:* USB serial driver, or UART fallback: HARDWARE Part 8
- [ ] **D6** Unplug and replug the cable: guardian survives, reconnects
- [ ] **D7** Phase 4: protocol v2, `reflex.conf`, `rxcal` wizard
- [ ] **Check:** `rxcal` finds wrong values on the fake node?  
      *If not:* Claude Code fixes rxcal before going on
- [ ] **D8** Phase 5: `reflex-demo.sh`, dashboard status, `video.rxs` script
- [ ] **D9** Cold-boot the Pi, `./reflex-demo.sh start` works
- [ ] **Milestone: Software is plug-and-play**

## E · Mechanical build (parts have arrived) · 26 Nov-1 Dec

- [ ] **E1** Unbox, tick every BOM line, keep the spare sensor
- [ ] **E2** Cut the channel to 500 mm, centre-punch the pivot mark
- [ ] **E3** Cut base 550x150, two uprights 150x60, block 40x30x20
- [ ] **E4** 16 mm holes at 120 mm height, press in the 625ZZ bearings
- [ ] **E5** Uprights on the base 35 mm apart, M5 bolt through both
- [ ] **E6** 5 mm hole in the block, fix the block under the pivot mark
- [ ] **E7** Clamp the block to the axle with a nut each side
- [ ] **Check:** Beam turns freely, about balanced?  
      *If not:* Shift the pivot or add a small counterweight
- [ ] **E8** Centre the servo (send `U 0`), fit the metal horn horizontal
- [ ] **E9** Mount the servo below the link point, 60 mm from the pivot
- [ ] **E10** Bracket under the beam at 60 mm; horn hole at 20 mm
- [ ] **E11** M3 push-rod with lock-nuts on both ends
- [ ] **Check:** Horn horizontal = beam level ±1°?  
      *If not:* Adjust the push-rod length with its nuts
- [ ] **E12** Sensor bracket: face at -280 mm, centre 17 mm above edges
- [ ] **E13** Low end stops at ±240 mm
- [ ] **E14** Route the sensor wires with a slack loop
- [ ] **Check:** Beam swings ±15°, ball rolls end to end?  
      *If not:* Fix whatever catches or binds
- [ ] **Milestone: Rig built**

## F · Electronics and firmware · 1-4 Dec

- [ ] **F1** Everything unpowered
- [ ] **F2** Sensor: VIN→3V3, GND→GND, SDA→PB7, SCL→PB6
- [ ] **F3** Servo orange (signal) → PA8
- [ ] **F4** Servo red/brown → 5 V 3 A adapter terminals, never to the Black Pill
- [ ] **F5** Common ground: adapter − → Black Pill GND
- [ ] **F6** 1000 µF across adapter + and −, stripe to −
- [ ] **F7** Meter: 4.9-5.3 V at adapter, 3.2-3.4 V at 3V3
- [ ] **F8** Continuity: GND to adapter − beeps; adapter + to 3V3 does not
- [ ] **Check:** All four meter checks pass?  
      *If not:* Rewire before powering anything
- [ ] **F9** DFU mode, then `make fw && make flash` (real firmware)
- [ ] **F10** `scripts/fw.sh monitor`: ready line, P values follow the ball
- [ ] **Check:** Sensor found and servo obeys `U 100`?  
      *If not:* HARDWARE Part 10 troubleshooting table
- [ ] **Milestone: I/O node alive**

## G · Calibration and first closed loop · 4-7 Dec

- [ ] **G1** Plug the Black Pill into the Pi
- [ ] **G2** On the Pi: `./rxcal` (hands clear when it says so)
- [ ] **G3** rxcal: level, direction, centre, range, angle scale
- [ ] **Check:** rxcal validation: ball within ±150 mm?  
      *If not:* Fix sign or linkage: HARDWARE 7.2 and 9.1
- [ ] **G4** Guardian serial, `--no-pilot`: ball settles at setpoint
- [ ] **G5** Add the AI: `--learn-from-sim`, `rxctl period 3000`, 15-30 min
- [ ] **Check:** Interventions falling, 0 balls lost?  
      *If not:* Ask Claude Code: raise t_delay, retune, retest
- [ ] **G6** Flip with guardian on: ball stays. Off: ball falls
- [ ] **G7** Pull the USB cable: beam levels within 150 ms
- [ ] **G8** HAM: kill the guardian, it restarts, ball stays
- [ ] **G9** Optional: learn from scratch on the real beam
- [ ] **Check:** Real demo solid by 8 Dec?  
      *If not:* Record the video with sim + emulated board, and say so
- [ ] **Milestone: Real demo works**

## H · Evidence · 7-9 Dec

- [ ] **H1** `make evidence PI=$PI` with the real beam
- [ ] **H2** Same with the simulated plant at 1 kHz
- [ ] **H3** Linux A/B: boot Raspberry Pi OS card, `scripts/linux-ab.sh`
- [ ] **H4** `tools/report.py`: report.md and plots
- [ ] **H5** README Results filled from `results/` only
- [ ] **Check:** Every number traceable to a results file?  
      *If not:* Delete any number that is not
- [ ] **H6** Commit and push `results/`
- [ ] **Milestone: Evidence committed**

## I · The video · 9-11 Dec

- [ ] **I1** Reread the storyline in `docs/SUBMISSION.md`
- [ ] **I2** Phone on tripod facing the beam; OBS records the dashboard
- [ ] **I3** Dry run: `rxctl script demo/video.rxs`
- [ ] **I4** Record; label any time-lapse on screen
- [ ] **I5** Edit: under 3:00, no copyrighted music or logos
- [ ] **Check:** Under 3:00 and shows the device running?  
      *If not:* Re-cut; cut words before cutting the demo
- [ ] **I6** Upload to YouTube as **Public**, check it plays logged out
- [ ] **Milestone: Video public**

## J · Write-up and submit · 11-13 Dec

- [ ] **J1** Devpost text from `SUBMISSION.md`, in your own words
- [ ] **J2** Disclose AI assistance if the rules ask for it
- [ ] **J3** Evidence of QNX use: list files and functions
- [ ] **J4** Repo: public, MIT in About, build/run steps name SDP 8.0, Pi 4, toolchain
- [ ] **J5** Category: Robotics & Medical; fill bonus-prize fields
- [ ] **J6** Rehearse the 7 questions at the end of GUIDE.md
- [ ] **Check:** SUBMISSION.md checklist all ticked?  
      *If not:* Fix each open item first
- [ ] **J7** Submit on Devpost **by 13 Dec**
- [ ] **J8** Check the submission page and confirmation email
- [ ] **Milestone: SUBMITTED**
