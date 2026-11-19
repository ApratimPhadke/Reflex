#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Fake Reflex I/O node: a ball-and-beam on a pseudo-terminal.

Speaks the same line protocol as firmware/reflex_io (see src/core/rx_link.h) and simulates the
same plant as rx_plant_step() in src/core/rx_core.c, so the guardian's serial backend can be
tested end to end without hardware:

    tools/fake_io_node.py --link /tmp/reflex-fake-io &
    build/linux/reflex-guardian --backend serial --dev /tmp/reflex-fake-io

Plant: x'' = k sin(theta) - 0.15 v, servo slews at `slew` rad/s toward the commanded angle, which
the node clamps to +/-230 mrad (the firmware's hard limit). Sensor: 50 Hz, uniform noise
+/-noise_mm, 1 mm quantisation, "P !" while the ball is off the beam. Watchdog: no "U" line for
150 ms -> command 0 (level). A lost ball is replaced at the centre after 1 s.

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
    a = ap.parse_args()
    rnd = random.Random(a.seed)

    st = dict(balls_lost=0, min_edge_mm=1e9, watchdog_trips=0, commands=0, bad_commands=0,
              samples_sent=0, samples_dropped=0, garbage_sent=0, replugs=0, stalls=0,
              max_abs_cmd_mrad=0, t=0.0, x_mm=0.0, theta_mrad=0.0, link=a.link)
    x, v, th, cmd = a.x0_mm / 1000.0, 0.0, 0.0, 0.0
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
            line = rnd.choice(["P 12abc", "P", "xx P 5", "P 99999", "\x00\xff garbage", "P -", "# noise"])
            st["garbage_sent"] += 1
        try:
            os.write(p.master, (line + "\n").encode("latin-1"))
        except BlockingIOError:
            st["samples_dropped"] += 1        # like the firmware: never block on output
        except OSError:
            pass

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
                if line.startswith("U "):
                    try:
                        m = int(line[2:])
                    except ValueError:
                        st["bad_commands"] += 1
                        continue
                    st["commands"] += 1
                    st["max_abs_cmd_mrad"] = max(st["max_abs_cmd_mrad"], abs(m))
                    cmd = max(-THETA_MAX_NODE, min(THETA_MAX_NODE, m / 1000.0))   # hard limit, independent of host
                    last_cmd_t = now
                    in_watchdog = False
                elif line:
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
            cmd = 0.0

        # ---- physics up to `now`
        while t_sim + SUBSTEP <= now:
            t_sim += SUBSTEP
            d = cmd - th
            lim = SLEW * SUBSTEP
            th += max(-lim, min(lim, d))
            if t_sim < lost_until:
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
            if lost_until:
                line = "P !"
            else:
                mm = round(x * 1000.0 + rnd.uniform(-a.noise_mm, a.noise_mm))
                line = f"P {mm}"
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
