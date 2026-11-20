# Reflex hardware guide: mechanics, electronics and bring-up

This guide takes you from a bag of parts to a ball balancing under the Reflex guardian on QNX. Do the parts in order. Each part ends with a **Check**; do not move on until it passes.

Time: about 2 evenings for the mechanics, 1 for the electronics and firmware, 1-3 for calibration and tuning.

Status: the firmware compiles for the Black Pill F401CC (`make fw`, STM32duino core 3.1.0, 2026-11-20). The emulator build (`EMULATE=1`) has run on the project's board against the Linux guardian over USB CDC (`results/2026-11-20-host-blackpill-emulate/`); the sensor/servo build has not yet run on a board, and the rig described here has not been built. The numbers below come from geometry and datasheet values, so treat them as starting points that you confirm with a ruler and the serial monitor.

---

## Part 0. How the system fits together

```
   230 V ──[5 V 3 A adapter]──(+5V, GND)──────────────┐
                                                      │ servo power only
                                                      ▼
 ┌────────────────┐  USB-C data   ┌───────────────┐  PA8 signal  ┌──────────┐
 │ Raspberry Pi 4 │──────────────▶│  Black Pill   │─────────────▶│  MG996R  │──push-rod──▶ beam
 │ QNX 8.0        │◀──────────────│  (I/O node)   │              └──────────┘
 │ guardian+pilot │  "P <mm>" up  │  firmware:    │  I2C PB6/PB7  ┌──────────┐
 └────────────────┘  "U <mrad>" dn│  reflex_io    │◀─────────────│ VL53L0X  │◀── laser ── ball
        ▲                         └───────────────┘              └──────────┘
        │ Ethernet
     laptop (dashboard in a browser, SSH)
```

- The **Pi** runs everything that thinks: the guardian, the AI pilot, the dashboard.
- The **Black Pill** is a dumb, fast I/O node. It reads the sensor 50 times a second, sends the position up the USB cable, and moves the servo to whatever angle comes back down. If commands stop for 150 ms, it levels the beam by itself.
- The **servo** has its own power supply. It never draws power from the Black Pill or the Pi.
- Nothing connects to the Pi's GPIO header in the default setup.

---

## Part 1. Parts and tools

### Parts (see the BOM for links)

| Ref | Part | Qty |
|---|---|---|
| P1 | Raspberry Pi 4 + official 5.1 V 3 A USB-C supply + 32 GB micro-SD | 1 |
| P2 | STM32 Black Pill (F401 or F411) | 1 |
| P3 | VL53L0X module (GY-530 / VL53L0XV2 type, with on-board regulator) | 1 + 1 spare |
| P4 | MG996R servo, **180°** version, with metal horn | 1 |
| P5 | 5 V 3 A wall adapter with 5.5 mm barrel plug | 1 |
| P6 | Female barrel jack to screw-terminal adapter (match the plug's inner diameter, 2.1 or 2.5 mm) | 1 |
| P7 | 1000 µF 16 V electrolytic capacitor | 1 + 1 spare |
| P8 | Aluminium U-channel, inner width 15-20 mm, cut to 500 mm | 1 |
| P9 | 625ZZ bearings (5 mm bore, 16 mm OD) | 2 |
| P10 | M5 × 50 mm bolt, 4 nuts, 4 washers | 1 set |
| P11 | M3 push-rod 100 mm + 4 M3 nuts (2 per end, locked against each other) | 1 set |
| P12 | 12 mm plywood/MDF: base 550 × 150 mm, two uprights 150 × 60 mm, one block 40 × 30 × 20 mm | — |
| P13 | M3 screws/nuts/spacers kit, a few wood screws | 1 |
| P14 | Ping-pong balls (40 mm) | 3-4 |
| P15 | Breadboard, male-female and male-male jumpers, USB-C data cable, Ethernet cable | — |

The BOM earlier said a 45 cm channel. Buy 55 cm and cut it to 500 mm: the extra length gives the sensor room to see the whole travel (Part 2.3).

### Tools

Drill with 3, 5 and 16 mm bits (16 mm Forstner or spade bit for the bearings), hacksaw, file, screwdriver set, steel rule, multimeter, hot glue or epoxy, a phone with a free inclinometer app, and soldering iron only if your modules come with loose header pins.

---

## Part 2. Geometry (read this before cutting anything)

All dimensions are in mm, measured along the beam from the pivot (0). Positive is toward the far end.

```
 sensor            channel                      pivot                      channel
 bracket   -250 ◀───────────────────────────────── 0 ──────────────────────────────▶ +250
   │         │stop -240                                                  stop +240│
   ▼         ▼                                                                    ▼
  [S]=======[|===================================(●)====================================|]
 -280                    ball travel used by the software: -200 … +200
```

### 2.1 Beam and stops

- Channel length 500 mm, pivot exactly at its centre (±1 mm; mark it with a centre punch).
- End stops at ±240 mm: an M3 screw through each side wall, or a strip of foam glued across. Keep them **low**: no higher than the channel walls, so the sensor beam passes over them as much as possible.
- The software treats |x| > 200 mm as "ball lost" and keeps a 10 mm keep-out strip inside that. The stops at 240 mm are only there to catch the ball.

### 2.2 How the ball sits

In a U-channel narrower than the ball, the ball rides on the two top edges like rails, not on the floor. With 20 mm between the edges, the ball's centre sits about 17 mm above the top edges (√(20² − 10²) ≈ 17.3). That is good: rolling is smooth and the sensor can look over the walls.

### 2.3 Sensor position

The VL53L0X measures to the first surface it sees, so it must sit **outboard** of the beam end and tilt **with** the beam:

- Mount it on an L-bracket fixed to the channel, so the sensor face is at −280 mm (30 mm beyond the channel end).
- Its optical centre must be at the height of the ball's centre: about 17 mm above the channel top edges.
- Aim it straight down the beam's axis, parallel to the channel.

With that geometry the sensor reads the distance to the near surface of the ball:

| Ball centre at | Expected reading |
|---|---|
| −200 mm (near end of travel) | about 60 mm |
| 0 (centre) | about 260 mm |
| +200 mm (far end of travel) | about 460 mm |
| no ball (sees the far stop at +240) | about 520 mm or a timeout |

These are the defaults now in the firmware (`CENTER_MM = 260`, `RANGE_MIN_MM = 40`, `RANGE_MAX_MM = 495`). You will correct them with real measurements in Part 7.

The VL53L0X's field of view is about 25°, so at 40-50 cm its cone is wider than the ball. That is why the sensor height matters and why the walls and stops should stay low. If readings near the far end are noisy, move the sensor 5 mm higher or put a strip of matte black tape on the far stop.

### 2.4 Pivot height and servo linkage

```
 side view (not to scale)

          beam ───────────────(●)pivot──────────[bracket]──────────
                               ║                     │ push-rod (vertical)
                               ║  upright            │ ~ 60 mm from pivot
                               ║  (two, one          │
                               ║  each side)     ┌───●───┐ horn tip, 20 mm from servo shaft
                               ║                 │ MG996R│
          ═════════════════════╩═════════════════╧═══════╧═══════ base
```

- Pivot axle height above the base: about 120 mm (the exact value only has to fit your servo).
- Push-rod attaches to the beam **60 mm** from the pivot, on the side away from the sensor or toward it; either works.
- Push-rod attaches to the servo horn **20 mm** from the servo shaft.
- With the horn horizontal at the servo's centre position, the push-rod must be vertical and the beam level. Adjust the rod length with its nuts until this is true.

Why these numbers: the beam turns by roughly (horn radius ÷ beam lever) × servo angle = 20/60 = 1/3 of the servo angle. The software asks for at most ±0.21 rad (12°) of beam, which is ±36° at the servo: well inside its range, with good resolution. The firmware's `US_PER_MILLIRAD = 1.9` is derived from the same 1/3 ratio and a servo that moves about 90° per 1000 µs. You will measure the real value in Part 7.

---

## Part 3. Mechanical build

### 3.1 Base and uprights

1. Cut the base (550 × 150 mm) and two uprights (150 × 60 mm).
2. In each upright, 120 mm above its bottom edge and centred, drill a 16 mm hole for a bearing. Drill slowly; you want a tight fit.
3. Press a 625ZZ bearing into each hole. If it is loose, a ring of epoxy around it (keep glue out of the bearing).
4. Screw the uprights to the base, facing each other, about 35 mm apart (wide enough for the channel plus the block below). Both bearing holes must line up: push the M5 bolt through both to check.

### 3.2 Beam and axle

1. Drill a 5 mm hole sideways through the 40 × 30 × 20 mm block, 10 mm from one face.
2. Glue and screw the block centred under the channel at the pivot mark.
3. Pass the M5 bolt through upright, washer, block, washer, upright. Clamp the block to the bolt with a nut on each side, so the bolt turns **with** the beam and rotates in the bearings.
4. **Check:** the beam rotates freely and stays roughly where you leave it. If it always falls to one side, the pivot is off centre; shift weight (or the pivot) until it is close to balanced. A balanced beam makes the servo's job much easier.

### 3.3 Servo

1. Before mounting, centre the servo: power it (Part 4) and send `U 0` (Part 6), or use any servo tester. Then fit the metal horn horizontal.
2. Mount the servo on the base with M3 screws and spacers, so its horn tip sits directly below the beam's link point, 60 mm from the pivot.
3. Drill a 3 mm hole in the horn 20 mm from the shaft centre (most metal horns already have holes; use the one closest to 20 mm and note the real value).
4. Fix a small L-bracket (or a block with a 3 mm hole) under the channel 60 mm from the pivot.
5. Fit the push-rod between horn and bracket with two nuts locked against each other on each side of each hole, so the rod can pivot but not slide.
6. **Check:** with the horn horizontal, the beam is level (inclinometer reads 0 ± 1°). Turn the horn by hand through ±40°: nothing binds or hits the base.

### 3.4 Sensor

1. Make an L-bracket from a strip of aluminium or a 3D-printed part. Fix it to the near end of the channel so the sensor face ends up at −280 mm.
2. Mount the VL53L0X with M2/M3 screws or hot glue, lens facing down the beam, centre about 17 mm above the channel edges.
3. Route the four sensor wires along the beam and down near the pivot, with a slack loop so the beam can move ±15° without pulling them.
4. Fit the end stops at ±240 mm.

**Check (mechanical):** the whole beam swings ±15° freely, no wire pulls, the ball rolls end to end without catching.

---

## Part 4. Electrical wiring

**Power off everything while wiring. Never connect the servo's red wire to the Black Pill.**

### 4.1 Connection table

| From | To | Wire |
|---|---|---|
| VL53L0X **VIN** | Black Pill **3V3** | red |
| VL53L0X **GND** | Black Pill **GND** | black |
| VL53L0X **SDA** | Black Pill **PB7** | blue |
| VL53L0X **SCL** | Black Pill **PB6** | yellow |
| VL53L0X XSHUT, GPIO1 | not connected | — |
| Servo **signal** (orange) | Black Pill **PA8** | orange |
| Servo **+** (red) | screw terminal **+** of the barrel adapter | red, short and thick |
| Servo **−** (brown) | screw terminal **−** of the barrel adapter | brown, short and thick |
| Barrel adapter **−** | Black Pill **GND** | black: **common ground, required** |
| 1000 µF capacitor | across the barrel adapter **+** and **−** | long leg (+) to +, striped side (−) to − |
| Black Pill USB-C | Raspberry Pi 4 USB-A port (any) | data cable |
| Pi Ethernet | your router or laptop | — |
| Pi USB-C | official Pi power supply | — |

### 4.2 Schematic

```
                        5 V 3 A adapter
                         │+        │−
                    ┌────┴────┐    │
                    │ 1000 µF │    │
                    └────┬────┘    │
                         │         │
          servo red ─────┘         ├──────── servo brown
                                   │
                                   └──────── Black Pill GND   (common ground)

  Black Pill                       VL53L0X
  ┌──────────┐                     ┌──────────┐
  │ 3V3  ────┼─────────────────────┤ VIN      │
  │ GND  ────┼─────────────────────┤ GND      │
  │ PB7  ────┼─────────────────────┤ SDA      │
  │ PB6  ────┼─────────────────────┤ SCL      │
  │          │                     └──────────┘
  │ PA8  ────┼───────────────────── servo orange (signal)
  │ USB-C ───┼───────────────────── Raspberry Pi 4 USB
  └──────────┘
```

### 4.3 Why each rule is there

- **Separate servo supply:** an MG996R can draw more than 1 A when it starts or stalls. From the Black Pill or the Pi's USB that causes brown-outs: the board resets, or the Pi throttles and corrupts timing measurements.
- **Common ground:** the servo reads its signal relative to its own ground. Without the link from adapter − to Black Pill GND the servo twitches randomly or does not move.
- **Capacitor:** absorbs the current spikes of the servo so the 5 V line does not dip.
- **3V3 for the sensor:** the GY-530 style modules accept 2.8-5 V and have their own I2C pull-ups. Powering from 3V3 keeps the I2C lines at the Black Pill's logic level.
- **Signal at 3.3 V:** the MG996R accepts a 3.3 V control signal in practice. If it jitters or ignores commands, that is the first suspect: see Troubleshooting.

### 4.4 Power-on checks (multimeter)

Do these with the servo signal wire **disconnected** from PA8.

1. Adapter plugged in, nothing else: + to − at the screw terminal reads 4.9-5.3 V.
2. Black Pill on USB from your laptop: 3V3 pin to GND reads 3.2-3.4 V.
3. Continuity: Black Pill GND to adapter − beeps.
4. Continuity: nothing beeps between adapter + and Black Pill 3V3 (no short between the two supplies).

**Check:** all four pass. Now connect the servo signal to PA8.

---

## Part 5. Firmware

Built from the command line with `arduino-cli` and flashed with `dfu-util`. Both were installed by `make setup` (Ubuntu 24.04). You do not need the Arduino IDE or STM32CubeProgrammer.

### 5.1 Pick your board variant

Read the chip on the Black Pill: **STM32F411CEU6** (most common) or **STM32F401CCU6 / F401CEU6**.

| Chip | `BOARD=` |
|---|---|
| STM32F411CEU6 | `BLACKPILL_F411CE` |
| STM32F401CCU6 | `BLACKPILL_F401CC` (default: the board this project uses) |
| STM32F401CEU6 | `BLACKPILL_F401CE` |

The marking can lie. The project's board is sold and marked as F401CCU6, but over SWD it reads IDCODE `0x10006431` (DEV_ID 0x431 = **STM32F411**, rev A) with a flash-size register of 256 KB. If you have an ST-LINK, check yours with `st-info --probe` (and `st-flash read id.bin 0xE0042000 4` for the IDCODE). When the flash-size register says 256 KB, build `BLACKPILL_F401CC` whatever the chip ID says: it assumes 256 KB flash and 64 KB RAM, which is a safe subset of an F411, while `BLACKPILL_F411CE` assumes 512 KB of flash the chip does not report.

### 5.2 Build

```sh
make fw                         # F401CC (default)
make fw BOARD=BLACKPILL_F411CE  # F411CE
make fw EMULATE=1               # plant emulator: no sensor/servo needed (build/firmware-emulate/)
```

The binary ends up in `build/firmware/reflex_io.ino.bin`.

If it fails with an unknown board option, the STM32 core renamed a menu value. Run `scripts/fw.sh options`, find the right `pnum=` / `usb=` value, and set `FQBN=` accordingly. The only compiler warnings come from the Pololu VL53L0X library header (deprecated `boolean`).

**Emulator build.** `make fw EMULATE=1 && make flash EMULATE=1` turns the board into a simulated ball-and-beam (same model as the guardian's simulator, sensor-like noise) speaking the same protocol, with the same watchdog and clamps. Nothing needs to be wired to it. Use it to test the Pi ↔ USB ↔ board chain before the mechanics exist; `tools/fake_io_node.py` does the same on the laptop without any board.

### 5.3 Flash

1. Connect the Black Pill to the laptop with the USB-C data cable.
2. Put it in DFU mode: hold **BOOT0**, tap **NRST**, release **BOOT0**.
3. `lsusb | grep 0483:df11` must show "STMicroelectronics STM Device in DFU Mode".
4. `make flash`
5. Tap **NRST** if it does not restart by itself.

If DFU mode is not detected: try another cable (charge-only cables are common), a USB 2.0 port, or hold BOOT0 for a full second while tapping NRST. Some F411 boards only enter DFU reliably with the board warm or after a few tries; that is a known quirk of the ROM bootloader.

**Alternative: ST-LINK over SWD (no buttons).** Wire an ST-LINK/V2 to the Black Pill's 4-pin SWD header (SWDIO, SWCLK, GND, 3.3V; NRST is not needed) and run

```sh
st-flash read backup.bin 0x08000000 0x40000                                   # optional: keep the old image
st-flash --reset write build/firmware-emulate/reflex_io.ino.bin 0x08000000    # or build/firmware/...
```

`st-flash` verifies the write and resets the chip in software. Before plugging the USB-C cable in for the serial port, unplug the ST-LINK or pull its 3.3V wire, so the board is not powered from two sources.

**Check:** `lsusb | grep 0483:5740` shows the board as a serial device, `/dev/ttyACM0` exists, and the LED blinks fast (nobody is sending commands yet, so the watchdog is active).

## Part 6. Bench test with the laptop

Open a serial console:

```sh
scripts/fw.sh monitor          # picocom at 115200; quit with Ctrl-A then Ctrl-X
```

Type commands and press Enter.

1. On reset you see `# reflex_io ready`. If you see `VL53L0X NOT FOUND`, check the four sensor wires, especially SDA/SCL swapped.
2. A stream of `P …` lines appears, 50 per second. Put the ball on the beam: the number changes as you roll it. Remove the ball: `P !` or a large number.
3. Type `U 0` + Enter. The beam goes level and the LED stops blinking for 150 ms, then blinks again (the watchdog levels the beam because you are not sending commands continuously). That is correct behaviour.
4. Send `U 100`, then `U -100`. The beam tilts one way, then the other.

**Check:** sensor values change smoothly with the ball's position; the servo responds to `U`.

---

## Part 7. Calibration

Change the constants at the top of `reflex_io.ino`, then `make fw && make flash` (DFU mode first), repeat. Write the final values down; they go into your README.

### 7.1 Level (SERVO_CENTER_US)

Type `U 0` + Enter several times in quick succession (paste it ten times in a row). Read the inclinometer on the beam. Adjust `SERVO_CENTER_US` in steps of 10 µs until the beam reads 0.0 ± 0.3°.

### 7.2 Direction (SERVO_DIR, SENSOR_DIR)

This is the most important step. A wrong sign makes every controller push the ball off the beam.

1. Ball near the centre. Send `U 100` several times.
2. The ball rolls one way. Watch the `P` value:
   - it **increases**: correct, change nothing;
   - it **decreases**: set `SENSOR_DIR = -1`.

Rule: a positive command must make the reported position increase. Fix it with **one** of the two constants, never both.

### 7.3 Centre (CENTER_MM)

1. Set `SENSOR_DIR` correctly first.
2. Temporarily set `CENTER_MM = 0`, rebuild and flash.
3. Hold the ball still at the pivot mark. Average the `P` readings over a few seconds (for example 258).
4. Set `CENTER_MM` to that number (258), rebuild and flash. The reading at the centre is now about 0.

### 7.4 Range window (RANGE_MIN_MM, RANGE_MAX_MM)

1. With `CENTER_MM = 0` still set, read the value with the ball against the near stop and against the far stop, and with no ball at all.
2. `RANGE_MIN_MM`: about 20 below the near-stop reading.
3. `RANGE_MAX_MM`: halfway between the far-stop reading and the no-ball reading.
4. If the far-stop and no-ball readings are within 30 mm of each other, the sensor is seeing the stop: lower the stop or tape it black (2.3).

### 7.5 Angle scale (US_PER_MILLIRAD)

1. Send `U 200` repeatedly. That should be 0.200 rad = 11.5° of beam.
2. Read the inclinometer. If it says, for example, 9.0°, multiply `US_PER_MILLIRAD` by 11.5 / 9.0.
3. Repeat with `U -200`. If the two sides differ by more than 1°, the linkage is not symmetric: the horn is not horizontal at centre. Fix the mechanics (3.3), not the constant.

### 7.6 Beam length in software

If your usable travel is not ±200 mm, change `half_len` in `rx_cfg_default()` in `src/core/rx_core.c` (metres), rebuild, and run `make test` on the laptop before copying to the Pi.

**Check:** `U 200` gives 11.5 ± 0.5° and `U -200` gives −11.5 ± 0.5°; the ball at the pivot reads 0 ± 3 mm; positive command means increasing position.

---

## Part 8. Connect to QNX

1. Plug the Black Pill into the Pi 4 USB port. SSH into the Pi.
2. `ls /dev/ser*` before and after plugging in. A new device such as `/dev/serusb1` is the Black Pill.
3. If nothing appears, start the USB serial driver. The QNX 8.0 quick-start image for the Pi 4 does **not** include `devc-serusb` (checked 2026-11-20), but the USB host stack (`io-usb-otg`) is running and the board enumerates (`usb` lists `0483:5740 BLACKPILL_F401CC CDC in FS Mode`). Copy the driver from the SDP and start it:
   ```sh
   scp $QNX_TARGET/aarch64le/sbin/devc-serusb qnxpi:reflex/      # on the laptop
   cd ~/reflex && sudo sh -c 'nohup ./devc-serusb > /dev/null 2>&1 &'   # on the Pi (sudo is passwordless on the QSTI)
   ```
   `/dev/serusb1` appears at once. This does not survive a reboot.
4. `cat /dev/serusb1`: you should see the `P …` stream at 50 per second. Ctrl-C.

Note: `devc-serusb` keeps reading the board while nobody has the port open and holds about 2 KB, keeping the *oldest* data when full. So the first lines `cat` prints after a pause can be seconds old. The guardian discards them on open (`tcflush` in `src/guardian/rx_io.c`); measurements in `results/2026-11-20-pi4-emulated-io/`.

**If USB serial cannot be made to work** on QNX, use the hardware UART fallback:

| Black Pill | Raspberry Pi 4 header |
|---|---|
| PA9 (TX) | GPIO15 / RXD, pin 10 |
| PA10 (RX) | GPIO14 / TXD, pin 8 |
| GND | GND, pin 6 |

Set `#define LINK_UART 1` in the firmware, `make fw && make flash`. On the Pi, start the PL011 UART driver as described in the QNX "Configuring UART" page for the Raspberry Pi quick-start image, and use that device name with `--dev`. Both sides are 3.3 V, so no level shifter is needed. Keep the USB cable connected for power, or power the Black Pill from the Pi's 5 V pin (pin 2) to its 5V pin.

**Check:** the `P` stream is visible on the Pi.

---

## Part 9. First closed loop

Do this in stages. Keep a hand near the beam the first few times.

### 9.1 Fallback controller only

```sh
cd ~/reflex
./reflex-guardian --backend serial --dev /dev/serusb1 --no-pilot --fresh &
./rxdash &
```

Open the dashboard on the laptop: `http://<pi-ip>:8080`. The guardian flies the beam with its simple, verified controller.

| What you see | Cause | Fix |
|---|---|---|
| Ball settles at the setpoint, slowly | Correct | Go to 9.2 |
| Ball races to one end, every time | A sign is wrong | Part 7.2 |
| Ball oscillates around the setpoint, growing | Plant slower than the model | In `rx_cfg_hardware()`: raise `est_tau` to 0.12, or lower `wn` to 1.8 in `rx_cfg_default()` |
| Ball oscillates, small and steady | Backlash or sensor noise | Tighten the linkage nuts; check sensor height |
| Dashboard says "sensor lost" often | Range window or sensor sees walls | Part 7.4, 2.3 |
| Beam jerks and Black Pill resets | Power | Capacitor, common ground, separate supply (Part 4) |

### 9.2 Add the AI

```sh
slay reflex-guardian
./reflex-guardian --backend serial --dev /dev/serusb1 --learn-from-sim --fresh &
./rxctl period 3000
```

The pilot starts from the simulator-trained network and adapts to your real beam. Leave it 15-30 minutes. Watch the learning panel: interventions should become rarer. Then `./rxctl train off` to fly the best weights.

If the guardian intervenes constantly with reason `envelope` even when the ball is calm, the real plant reacts more slowly than the model: raise `t_delay` in `rx_cfg_hardware()` to 0.16 and rebuild.

### 9.3 The demo checks

1. `./rxctl inject flip`: the guardian takes over, the ball stays on.
2. `./rxctl guard off ; ./rxctl inject flip`: the ball rolls off. Then `./rxctl clear ; ./rxctl guard on`.
3. Pull the USB cable: within 150 ms the Black Pill levels the beam and blinks.
4. `slay -s KILL reflex-guardian` with HAM running: the guardian comes back, the ball stays.

**Check:** all four behave as described. Record each one on video.

---

## Part 10. Troubleshooting

| Symptom | Likely cause | What to do |
|---|---|---|
| Servo twitches randomly | No common ground; noisy supply | Adapter − to Black Pill GND; capacitor; short servo wires |
| Servo ignores commands but beam is stiff | 3.3 V signal not enough for this servo | Try another MG996R, or add a 74HCT125 buffer powered from 5 V between PA8 and the servo |
| Servo hums at rest | Mechanical load or binding | Balance the beam (3.2), loosen the push-rod nuts slightly |
| Black Pill resets when the servo moves | Servo powered from the board, or supply too weak | Separate 5 V 3 A supply |
| `VL53L0X NOT FOUND` | SDA/SCL swapped, loose wire, no 3V3 | Re-check 4.1; try the spare sensor |
| Readings jump by 20-50 mm | Sensor sees the channel walls or a stop | Raise the sensor, lower the walls/stops, matte tape |
| Readings fine on the laptop, nothing on the Pi | USB serial driver | Part 8, or the UART fallback |
| LED keeps blinking while the guardian runs | Commands do not arrive | Wrong `--dev`; baud mismatch; check `cat` on the device |
| Ball lost at one end only | Asymmetric linkage or off-centre pivot | Part 3.3 check; 7.5 step 3 |

---

## Part 11. Safety

- The MG996R is strong enough to pinch fingers in the linkage. Keep hands out of the push-rod area while it is powered.
- Unplug the servo supply before working on the mechanics.
- A ping-pong ball leaving the beam is harmless, which is why this plant was chosen. Do not scale the rig up without rethinking that.
