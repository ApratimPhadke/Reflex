#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Reflex: calibration wizard end to end against tools/fake_io_node.py (Linux build).
#
#   make test-cal               (or: scripts/test-cal.sh [outdir])
#
# The fake rig gets a hidden calibration that is wrong in every way the defaults could be wrong
# (level off by 30 us, a different servo gain, both signs flipped, centre and no-ball reading moved).
# rxcal --auto, starting from no file, must recover it; the guardian must refuse to start without a
# valid reflex.conf; and rxcal must refuse --auto on a node that is not an emulator.
# Writes <outdir>/reflex.conf, which make test-serial then flies with (FAKE_CAL below).
set -uo pipefail
cd "$(dirname "$0")/.."
OUT="${1:-build/test-cal}"
B=build/linux
LINK="/tmp/reflex-fake-cal-$$"
# hidden calibration of the fake rig (keep in step with scripts/test-serial.sh)
FAKE_CAL="--level-us 1530 --us-per-mrad 2.1 --servo-dir -1 --sensor-dir -1 --center-mm 247 --noball-mm 560"
mkdir -p "$OUT"; rm -f "$OUT"/reflex.conf* "$OUT"/rxcal.log "$OUT"/fake.json
fail=0
ok()  { printf '  PASS  %s\n' "$*"; }
bad() { printf '  FAIL  %s\n' "$*"; fail=1; }
cleanup() { [ -n "${FPID:-}" ] && kill "$FPID" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT

make -s all >/dev/null || { echo "build failed"; exit 1; }
if pgrep -x reflex-guardian >/dev/null; then echo "a reflex-guardian is running on this host; stop it first"; exit 1; fi

echo "== guardian refuses the serial backend without a calibration"
"$B"/reflex-guardian --backend serial --dev /dev/null --conf "$OUT/missing.conf" > "$OUT/refuse.log" 2>&1
rc=$?
[ "$rc" = 2 ] && grep -q 'rxcal' "$OUT/refuse.log" && ok "exit 2 and points at rxcal" || { bad "guardian started without reflex.conf (rc=$rc)"; cat "$OUT/refuse.log"; }

echo "== rxcal --auto against a fake rig with a wrong-in-every-way hidden calibration"
# shellcheck disable=SC2086
python3 tools/fake_io_node.py --link "$LINK" --status "$OUT/fake.json" $FAKE_CAL > "$OUT/fake.log" 2>&1 &
FPID=$!
for _ in $(seq 50); do [ -e "$LINK" ] && break; sleep 0.1; done
timeout 120 "$B"/rxcal --dev "$LINK" --conf "$OUT/reflex.conf" --auto > "$OUT/rxcal.log" 2>&1
rc=$?
[ "$rc" = 0 ] && ok "rxcal finished (validation passed)" || { bad "rxcal failed (rc=$rc)"; tail -20 "$OUT/rxcal.log"; }
if [ -f "$OUT/reflex.conf" ]; then
    python3 - "$OUT/reflex.conf" "$OUT/fake.json" <<'EOF' || fail=1
import json, sys
conf = {}
for line in open(sys.argv[1]):
    line = line.split("#")[0].strip()
    if "=" in line:
        k, v = (s.strip() for s in line.split("=", 1)); conf[k] = v
truth = json.load(open(sys.argv[2]))["truth"]
checks = [
    ("level_us",    abs(int(conf["level_us"]) - truth["level_us"]) <= 3,                      f'{conf["level_us"]} vs {truth["level_us"]} (+/-3 us)'),
    ("us_per_mrad", abs(float(conf["us_per_mrad"]) / truth["us_per_mrad"] - 1) <= 0.03,       f'{conf["us_per_mrad"]} vs {truth["us_per_mrad"]} (+/-3 %)'),
    ("servo_dir",   int(conf["servo_dir"]) == truth["servo_dir"],                             f'{conf["servo_dir"]} vs {truth["servo_dir"]}'),
    ("sensor_dir",  int(conf["sensor_dir"]) == truth["sensor_dir"],                           f'{conf["sensor_dir"]} vs {truth["sensor_dir"]}'),
    ("center_mm",   abs(int(conf["center_mm"]) - truth["center_mm"]) <= 2,                    f'{conf["center_mm"]} vs {truth["center_mm"]} (+/-2 mm)'),
    ("range",       int(conf["range_min_mm"]) < truth["center_mm"] - 195 and
                    truth["center_mm"] + 195 < int(conf["range_max_mm"]) < truth["noball_mm"], f'{conf["range_min_mm"]}..{conf["range_max_mm"]} (beam {truth["center_mm"]-200}..{truth["center_mm"]+200}, no ball {truth["noball_mm"]})'),
    ("half_len_m",  0.19 <= float(conf["half_len_m"]) <= 0.20,                                conf["half_len_m"]),
]
bad = 0
for name, good, what in checks:
    print(f'  {"PASS" if good else "FAIL"}  recovered {name}: {what}')
    bad |= not good
sys.exit(bad)
EOF
fi
kill "$FPID" 2>/dev/null; wait "$FPID" 2>/dev/null; FPID=

echo "== rxcal refuses --auto on a node that is not an emulator"
python3 tools/fake_io_node.py --link "$LINK" --real > "$OUT/fake-real.log" 2>&1 &
FPID=$!
for _ in $(seq 50); do [ -e "$LINK" ] && break; sleep 0.1; done
timeout 20 "$B"/rxcal --dev "$LINK" --conf "$OUT/should-not-exist.conf" --auto > "$OUT/rxcal-real.log" 2>&1
rc=$?
[ "$rc" != 0 ] && grep -q 'only allowed on an emulated node' "$OUT/rxcal-real.log" && [ ! -e "$OUT/should-not-exist.conf" ] \
    && ok "refused, nothing written" || bad "rxcal --auto ran against a real-looking node (rc=$rc)"

echo
if [ "$fail" = 0 ]; then echo "CALTEST PASSED"; else echo "CALTEST FAILED"; fi
exit "$fail"
