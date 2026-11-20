#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Fake Reflex I/O node: a ball-and-beam on a pseudo-terminal.

Speaks protocol v2 like firmware/reflex_io (see src/core/rx_link.h) and simulates the same plant
as rx_plant_step() in src/core/rx_core.c, so the guardian's serial backend and the rxcal wizard can
be tested end to end without hardware:

    tools/fake_io_node.py --link /tmp/reflex-fake-io &
    build/linux/rxcal --dev /tmp/reflex-fake-io --auto          # writes reflex.conf
    build/linux/reflex-guardian --backend serial --dev /tmp/reflex-fake-io

The rig has a hidden calibration (--level-us, --us-per-mrad, --servo-dir, --center-mm, --sensor-dir,
--noball-mm) that rxcal has to find: beam angle = servo_dir * (pulse - level_us) / us_per_mrad,
raw reading = center_mm + sensor_dir * x_mm. Plant: x'' = k sin(theta) - 0.15 v, the servo slews at
6 rad/s, the pulse is clamped to 1000..2000 us and the beam to +/-230 mrad. Sensor: 50 Hz "M <raw>",
uniform noise +/-noise_mm, 1 mm quantisation; with no ball it reads --noball-mm ("M !" if 0).
Watchdog: no "S" line for 150 ms -> the stored level pulse ("L", initially --stored-level-us).
A ball that falls off is replaced at the centre after 1 s. As an emulated node it identifies with
"I reflex_io 2 FAKE_IO_NODE 1", sends the true angle as "T <mrad>" and obeys the emulator-only
"E pin <mm>" / "E free" / "E remove".

Fault knobs for robustness tests: --garbage P (fraction of garbage lines), SIGUSR1 = unplug for
--replug-gap seconds (the pty disappears and a new one appears behind the same link), SIGUSR2 =
stall sensor output for --stall seconds, then send everything buffered at once. SIGHUP = restart the
min_edge_mm ground-truth window.

Ground truth goes to --status FILE as JSON (rewritten every 0.5 s and on exit).
"""
import argparse
import json
import math
import os
import random
import select
import signal
import sys
import time
import tty

K_ACC, THETA_MAX_NODE, SLEW, ROLL_LOSS, HALF_LEN = 5.886, 0.230, 6.0, 0.15, 0.20
PULSE_MIN, PULSE_MAX, PROTO = 1000, 2000, 2
WATCHDOG_S, SENSOR_HZ, SUBSTEP = 0.150, 50.0, 0.0005


class Pty:
    def __init__(self, link):
        self.link = link
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        tmp = link + ".new"
        try:
            os.unlink(tmp)
        except FileNotFoundError:
            pass
        os.symlink(os.ttyname(self.slave), tmp)
        os.replace(tmp, link)          # atomic: the guardian never sees a missing link mid-swap

    def close(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="/tmp/reflex-fake-io", help="symlink to the pty slave (use as --dev)")
    ap.add_argument("--status", default="", help="write ground truth JSON here")
    ap.add_argument("--noise-mm", type=float, default=3.0)
    ap.add_argument("--garbage", type=float, default=0.0, help="fraction of output lines replaced by garbage")
    ap.add_argument("--replug-gap", type=float, default=1.0, help="seconds unplugged after SIGUSR1")
    ap.add_argument("--stall", type=float, default=0.3, help="seconds of sensor stall after SIGUSR2")
    ap.add_argument("--x0-mm", type=float, default=30.0, help="initial ball position")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--seconds", type=float, default=0, help="exit after this long (0 = run until killed)")
    # the rig's hidden calibration (what rxcal has to find)
    ap.add_argument("--level-us", type=int, default=1500, help="pulse at which the beam is truly level")
    ap.add_argument("--us-per-mrad", type=float, default=1.9)
    ap.add_argument("--servo-dir", type=int, default=1, choices=(-1, 1))
    ap.add_argument("--center-mm", type=int, default=260, help="raw reading with the ball at the pivot")
    ap.add_argument("--sensor-dir", type=int, default=1, choices=(-1, 1))
    ap.add_argument("--noball-mm", type=int, default=0, help="raw reading with no ball (0 = report 'M !')")
    ap.add_argument("--stored-level-us", type=int, default=1500, help="level pulse in the node's flash at start")
    ap.add_argument("--real", action="store_true", help="pose as real hardware: emulated=0, no T lines, E commands ignored")
    a = ap.parse_args()
    rnd = random.Random(a.seed)

    st = dict(balls_lost=0, min_edge_mm=1e9, watchdog_trips=0, commands=0, bad_commands=0,
              samples_sent=0, samples_dropped=0, garbage_sent=0, replugs=0, stalls=0,
              max_abs_cmd_mrad=0, t=0.0, x_mm=0.0, theta_mrad=0.0, link=a.link,
              level_writes=0, stored_level_us=a.stored_level_us, pulse_us=a.stored_level_us, queries=0,
              truth=dict(level_us=a.level_us, us_per_mrad=a.us_per_mrad, servo_dir=a.servo_dir,
                         center_mm=a.center_mm, sensor_dir=a.sensor_dir, noball_mm=a.noball_mm))
    x, v, th, cmd = a.x0_mm / 1000.0, 0.0, 0.0, 0.0
    stored_level = a.stored_level_us
    pinned, removed = None, False        # emulator test hooks ("E pin <mm>", "E remove")

    HELLO = f"I reflex_io {PROTO} FAKE_REAL 0" if a.real else f"I reflex_io {PROTO} FAKE_IO_NODE 1"

    def angle_of(pulse):
        pulse = max(PULSE_MIN, min(PULSE_MAX, pulse))
        th_ = a.servo_dir * (pulse - a.level_us) / a.us_per_mrad / 1000.0
        return max(-THETA_MAX_NODE, min(THETA_MAX_NODE, th_)), pulse
    lost_until = 0.0
    last_cmd_t = -1e9
    in_watchdog = True
    req = {"replug": False, "stall": False, "stop": False}
    signal.signal(signal.SIGUSR1, lambda *_: req.__setitem__("replug", True))
    signal.signal(signal.SIGUSR2, lambda *_: req.__setitem__("stall", True))
    signal.signal(signal.SIGHUP, lambda *_: st.__setitem__("min_edge_mm", 1e9))   # restart the min-edge window
    signal.signal(signal.SIGTERM, lambda *_: req.__setitem__("stop", True))
    signal.signal(signal.SIGINT, lambda *_: req.__setitem__("stop", True))

    p = Pty(a.link)
    print(f"fake_io_node: {a.link} -> {os.ttyname(p.slave)}", flush=True)
    cmd, st["pulse_us"] = angle_of(stored_level)
    rx = b""
    t0 = time.monotonic()
    t_sim = 0.0
    next_sample = 0.0
    next_status = 0.0
    stall_until = -1.0
    stalled = []
    unplugged_until = -1.0

    def dump():
        if a.status:
            tmp = a.status + ".tmp"
            with open(tmp, "w") as f:
                json.dump(st, f)
            os.replace(tmp, a.status)

    def send(line):
        if p is None:
            return
        if a.garbage and rnd.random() < a.garbage:
            line = rnd.choice(["M 12abc", "M", "xx M 5", "M 99999", "\x00\xff garbage", "M -", "# noise", "I reflex_io"])
            st["garbage_sent"] += 1
        try:
            os.write(p.master, (line + "\n").encode("latin-1"))
        except BlockingIOError:
            st["samples_dropped"] += 1        # like the firmware: never block on output
        except OSError:
            pass

    send(HELLO)                  # hello on reset, like the firmware
    while not req["stop"]:
        now = time.monotonic() - t0
        if a.seconds and now > a.seconds:
            break

        # ---- unplug / replug
        if req["replug"] and p is not None:
            req["replug"] = False
            p.close()
            p = None
            try:
                os.unlink(a.link)
            except FileNotFoundError:
                pass
            unplugged_until = now + a.replug_gap
            st["replugs"] += 1
        if p is None and now >= unplugged_until:
            p = Pty(a.link)
            rx = b""
            send(HELLO)          # a replugged board resets and says hello
        if req["stall"]:
            req["stall"] = False
            stall_until = now + a.stall
            st["stalls"] += 1

        # ---- commands from the guardian
        if p is not None:
            try:
                r, _, _ = select.select([p.master], [], [], 0.0005)
                if r:
                    rx += os.read(p.master, 4096)
            except OSError:
                r = []
            while b"\n" in rx or b"\r" in rx:
                i = min(j for j in (rx.find(b"\n"), rx.find(b"\r")) if j >= 0)
                line, rx = rx[:i].decode("latin-1").strip(), rx[i + 1:]
                parts = line.split()
                try:
                    if len(parts) == 2 and parts[0] == "S":
                        cmd, st["pulse_us"] = angle_of(int(parts[1]))      # hard limits, independent of host
                        st["commands"] += 1
                        st["max_abs_cmd_mrad"] = max(st["max_abs_cmd_mrad"], round(abs(cmd) * 1000))
                        last_cmd_t = now
                        in_watchdog = False
                    elif len(parts) == 2 and parts[0] == "L":
                        lv = max(PULSE_MIN, min(PULSE_MAX, int(parts[1])))
                        if lv != stored_level:
                            stored_level = lv
                            st["level_writes"] += 1                    # the firmware writes flash only on change
                        st["stored_level_us"] = stored_level
                    elif parts == ["?"]:
                        st["queries"] += 1
                        send(HELLO)
                    elif parts and parts[0] == "E" and a.real:
                        pass                                       # real firmware ignores emulator hooks
                    elif len(parts) == 3 and parts[:2] == ["E", "pin"]:
                        pinned, removed = int(parts[2]) / 1000.0, False
                        x, v, lost_until = pinned, 0.0, 0.0
                    elif parts == ["E", "free"]:
                        if removed or lost_until:
                            x, v, lost_until = 0.0, 0.0, 0.0
                        pinned, removed = None, False
                    elif parts == ["E", "remove"]:
                        pinned, removed = None, True
                    elif line:
                        st["bad_commands"] += 1
                except ValueError:
                    st["bad_commands"] += 1
            if len(rx) > 256:
                rx = b""
        else:
            time.sleep(0.0005)

        # ---- watchdog
        if now - last_cmd_t > WATCHDOG_S:
            if not in_watchdog:
                st["watchdog_trips"] += 1
                in_watchdog = True
            cmd, st["pulse_us"] = angle_of(stored_level)

        # ---- physics up to `now`
        while t_sim + SUBSTEP <= now:
            t_sim += SUBSTEP
            d = cmd - th
            lim = SLEW * SUBSTEP
            th += max(-lim, min(lim, d))
            if removed or t_sim < lost_until:
                continue
            if pinned is not None:
                x, v = pinned, 0.0
                continue
            if lost_until and t_sim >= lost_until:
                x, v, lost_until = 0.0, 0.0, 0.0
            a_ = K_ACC * math.sin(th) - ROLL_LOSS * v
            v += a_ * SUBSTEP
            x += v * SUBSTEP
            st["min_edge_mm"] = min(st["min_edge_mm"], (HALF_LEN - abs(x)) * 1000.0)
            if abs(x) > HALF_LEN:
                st["balls_lost"] += 1
                lost_until = t_sim + 1.0
                x = math.copysign(HALF_LEN, x)
                v = 0.0

        # ---- sensor at 50 Hz
        if now >= next_sample:
            next_sample += 1.0 / SENSOR_HZ
            if next_sample < now:
                next_sample = now + 1.0 / SENSOR_HZ
            if lost_until or removed:
                line = f"M {max(0, round(a.noball_mm + rnd.uniform(-a.noise_mm, a.noise_mm)))}" if a.noball_mm else "M !"
            else:
                raw = round(a.center_mm + a.sensor_dir * x * 1000.0 + rnd.uniform(-a.noise_mm, a.noise_mm))
                line = f"M {max(0, min(2000, raw))}"
            if p is not None and not (now < stall_until) and not a.real:
                send(f"T {round(th * 1000)}")
            st["samples_sent"] += 1
            if now < stall_until:
                stalled.append(line)
            else:
                for s in stalled:
                    send(s)
                stalled = []
                send(line)

        if now >= next_status:
            next_status = now + 0.5
            st.update(t=round(now, 3), x_mm=round(x * 1000, 2), theta_mrad=round(th * 1000, 1))
            dump()

    st.update(t=round(time.monotonic() - t0, 3), x_mm=round(x * 1000, 2))
    dump()
    if p is not None:
        p.close()
    try:
        os.unlink(a.link)
    except FileNotFoundError:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
