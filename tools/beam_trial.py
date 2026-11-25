#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Summary of one beam-direct trial (scripts/beam-trial.sh): ball position, mode, tilt, current, supply.
import csv, math, sys
base = sys.argv[1]
rec = list(csv.DictReader(open(base + '.rec.csv')))
beam = list(csv.DictReader(open(base + '.beam.csv')))
st = open(base + '.status.txt').read()
def f(x):
    try: return float(x)
    except ValueError: return math.nan
xs = [f(r['x_mm']) for r in rec]; sps = [f(r['sp_mm']) for r in rec]; u = [f(r['u_out_rad']) * 1000 for r in rec]
lostf = [r['lost'] not in ('0', '0.0') for r in rec]
seen = [x for x, l in zip(xs, lostf) if not l and not math.isnan(x)]
n = len(rec)
print(f"ticks {n} ({n / 50:.1f} s)")
if seen:
    m = sum(seen) / len(seen); sd = (sum((x - m) ** 2 for x in seen) / len(seen)) ** .5
    err = [x - s for x, s, l in zip(xs, sps, lostf) if not l and not math.isnan(x)]
    rms = (sum(e * e for e in err) / len(err)) ** .5
    print(f"ball x: mean {m:+.1f} mm, sd {sd:.1f}, min {min(seen):+.1f}, max {max(seen):+.1f}; error to setpoint rms {rms:.1f} mm")
    # oscillation: zero crossings of (x - mean)
    c = sum(1 for a, b in zip(seen, seen[1:]) if (a - m) * (b - m) < 0)
    print(f"crossings of the mean: {c} ({c / 2 / (n / 50):.2f} Hz if periodic)")
lost = sum(1 for r in rec if r['lost'] not in ('0', '0.0'))
modes = {}
for r in rec: modes[r['mode']] = modes.get(r['mode'], 0) + 1
reasons = {}
for r in rec: reasons[r['reason']] = reasons.get(r['reason'], 0) + 1
print(f"sensor lost {100 * lost / n:.0f} % of ticks; modes {modes}; reasons {reasons}")
print(f"command: {min(u):+.1f}..{max(u):+.1f} mrad; at the limits (|u| >= 99 % theta_max) {sum(1 for v in u if abs(v) >= 0.99 * max(abs(min(u)), abs(max(u)), 1e-9)) * 100 // n} % of ticks")
cur = [f(r['current_ma']) for r in beam]; vol = [f(r['supply_v']) for r in beam]; til = [f(r['mpu_tilt_deg']) for r in beam if not math.isnan(f(r['mpu_tilt_deg']))]
acc = sum(1 for r in beam if r['accepted'] == '1')
print(f"sonar accepted {100 * acc / max(1, len(beam)):.0f} %; current mean {sum(cur) / len(cur):.0f} mA peak {max(cur):.0f} mA; supply min {min(vol):.2f} V; MPU tilt {min(til):+.1f}..{max(til):+.1f} deg")
kt = [f(r['t_s']) for r in beam if r.get('kick') == '1']
if kt:
    t_end = max(kt); rec_t = [f(r['t_s']) for r in rec]
    print(f"kick: {len(kt)} ticks, {min(kt):.2f}..{t_end:.2f} s (I/O-helper clock)")
    # ball after the kick (rec rows are in order; map by fraction of the trial)
    post = [x for x, l in zip(xs, lostf) if not l and not math.isnan(x)]
    if post: print(f"ball over the whole trial: farthest {max(post, key=abs):+.1f} mm; last 5 s mean {sum(post[-250:]) / len(post[-250:]):+.1f} mm")
for k in ('balls lost', 'interventions'):
    for line in st.splitlines():
        if line.startswith(k): print(line)
