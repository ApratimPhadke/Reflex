#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Reflex: end-to-end test of the serial path against tools/fake_io_node.py (Linux build).
#
#   make test-serial                 (or: scripts/test-serial.sh [outdir])
#
# 0. scripts/test-cal.sh: rxcal --auto recovers the fake rig's wrong-in-every-way calibration
# 1. fake I/O node on a pty with that hidden calibration, 2% of its lines replaced by garbage
# 2. guardian --backend serial --conf <the wizard's reflex.conf>, frozen pilot, full fault campaign
#    pass: ball kept in 8/8 guarded runs; the guardian's ball-lost count equals the fake
#    node's ground truth
# 3. unplug the "cable" for 1 s: the guardian logs link down/up, keeps running, and AI is back
# 4. sensor stall of 0.3 s followed by a burst: no crash, no ball lost
set -uo pipefail
cd "$(dirname "$0")/.."
OUT="${1:-build/test-serial}"
B=build/linux
LINK="/tmp/reflex-fake-io-$$"
mkdir -p "$OUT"
rm -f "$OUT"/fake.json "$OUT"/fake-final.json "$OUT"/campaign.md "$OUT"/campaign.log "$OUT"/fake.log "$OUT"/guardian.log "$OUT"/status.txt "$OUT"/events.txt

fail=0
say()  { printf '\n== %s\n' "$*"; }
ok()   { printf '  PASS  %s\n' "$*"; }
bad()  { printf '  FAIL  %s\n' "$*"; fail=1; }
jget() { python3 -c "import json,sys; print(json.load(open('$OUT/fake.json'))['$1'])"; }
cleanup() {
    [ -n "${GPID:-}" ] && kill "$GPID" 2>/dev/null
    sleep 0.3
    [ -n "${FPID:-}" ] && kill "$FPID" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT

make -s all >/dev/null || { echo "build failed"; exit 1; }
if pgrep -x reflex-guardian >/dev/null; then echo "a reflex-guardian is already running on this host; stop it first"; exit 1; fi
# the same hidden calibration as scripts/test-cal.sh, which writes the reflex.conf we fly with
FAKE_CAL="--level-us 1530 --us-per-mrad 2.1 --servo-dir -1 --sensor-dir -1 --center-mm 247 --noball-mm 560"
say "calibrate the fake rig with rxcal (scripts/test-cal.sh)"
./scripts/test-cal.sh "$OUT/cal" > "$OUT/cal.log" 2>&1; calrc=$?
grep -E 'PASS|FAIL' "$OUT/cal.log" | grep -v TEST
[ "$calrc" = 0 ] && ok "calibration test passed" || bad "calibration test failed (see $OUT/cal.log)"
if [ -f "$OUT/cal/reflex.conf" ]; then ok "reflex.conf written by rxcal"; else bad "rxcal wrote no reflex.conf"; echo "SERIALTEST FAILED"; exit 1; fi

say "fake I/O node + guardian (serial, 50 Hz, frozen pilot)"
# shellcheck disable=SC2086
python3 tools/fake_io_node.py --link "$LINK" --status "$OUT/fake.json" --garbage 0.02 $FAKE_CAL > "$OUT/fake.log" 2>&1 &
FPID=$!
for _ in $(seq 50); do [ -e "$LINK" ] && break; sleep 0.1; done
[ -e "$LINK" ] || { cat "$OUT/fake.log"; echo "fake node did not start"; exit 1; }
"$B"/reflex-guardian --backend serial --dev "$LINK" --conf "$OUT/cal/reflex.conf" --fresh > "$OUT/guardian.log" 2>&1 &
GPID=$!
ai=0
for _ in $(seq 100); do
    "$B"/rxctl status 2>/dev/null | grep -q '^mode *AI' && { ai=1; break; }
    sleep 0.1
done
[ "$ai" = 1 ] && ok "pilot in control" || bad "pilot never reached AI mode"

say "fault campaign over the serial link (about 4 minutes)"
"$B"/rxctl reset; sleep 0.3
lost0=$(jget balls_lost)
"$B"/rxctl campaign "$OUT/campaign.md" 2> "$OUT/campaign.log" >/dev/null
lost1=$(jget balls_lost)
cat "$OUT/campaign.md"
grep -q 'With Reflex: ball kept in 8 of 8' "$OUT/campaign.md" && ok "ball kept in 8/8 guarded runs" || bad "guarded run lost the ball"
truth=$((lost1 - lost0))
seen=$("$B"/rxctl status | awk '/^balls lost/{print $3}')
[ "$truth" = "$seen" ] && ok "ball-lost inference matches ground truth ($seen)" \
                       || bad "guardian counted $seen balls lost, fake node lost $truth"
garb=$(jget garbage_sent); [ "$garb" -gt 0 ] && ok "$garb garbage lines sent and survived" || bad "no garbage was sent"

say "unplug the cable for 1 s"
"$B"/rxctl reset; sleep 3
lost0=$(jget balls_lost)
kill -USR1 "$FPID"; sleep 3
kill -0 "$GPID" 2>/dev/null && ok "guardian survived" || bad "guardian died"
ev=$("$B"/rxctl events)
echo "$ev" | tail -8
echo "$ev" | grep -q 'link down' && ok "link down logged" || bad "no link-down event"
echo "$ev" | grep -q 'link up'   && ok "link up logged (reconnected)" || bad "no link-up event"
[ "$(jget watchdog_trips)" -ge 1 ] && ok "node watchdog levelled the beam while unplugged" || bad "watchdog did not trip"
ai=0; for _ in $(seq 60); do "$B"/rxctl status | grep -q '^mode *AI' && { ai=1; break; }; sleep 0.1; done
[ "$ai" = 1 ] && ok "AI back in control after replug" || bad "AI not back after replug"
[ "$(jget balls_lost)" = "$lost0" ] && ok "no ball lost across the unplug" || bad "ball lost across the unplug"

say "sensor stall 0.3 s, then a burst"
lost0=$(jget balls_lost)
kill -USR2 "$FPID"; sleep 3
kill -0 "$GPID" 2>/dev/null && ok "guardian survived" || bad "guardian died"
[ "$(jget balls_lost)" = "$lost0" ] && ok "no ball lost across the stall" || bad "ball lost across the stall"

"$B"/rxctl status > "$OUT/status.txt"
"$B"/rxctl events > "$OUT/events.txt"
cp "$OUT/fake.json" "$OUT/fake-final.json"
echo
if [ "$fail" = 0 ]; then echo "SERIALTEST PASSED"; else echo "SERIALTEST FAILED"; fi
exit "$fail"
