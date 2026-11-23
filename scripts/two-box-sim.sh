#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Reflex, two boxes: the guardian on the Pi 4 (QNX 8.0, sim backend, 50 Hz, pilot_transport = udp) and the
# static Linux pilot on this laptop, over the Wi-Fi measured in results/2026-11-23-wifi-link/.
#
#   PI=qnxpi scripts/two-box-sim.sh start            guardian on the Pi, pilot (respawn loop) here
#   PI=qnxpi scripts/two-box-sim.sh normal [120]     record a clean run
#   PI=qnxpi scripts/two-box-sim.sh kill             kill -9 the laptop pilot 10 s into a 40 s recording
#   PI=qnxpi scripts/two-box-sim.sh wifi [150]       record on the Pi (detached) while YOU turn the laptop
#                                                    Wi-Fi off for ~10 s and back on; logs when it went down/up
#   PI=qnxpi scripts/two-box-sim.sh flip             sign-inverted pilot from 10 s to 20 s of a 30 s recording
#   PI=qnxpi scripts/two-box-sim.sh learn [300]      pilot restarted learning from scratch, training on
#   PI=qnxpi scripts/two-box-sim.sh stop             stop both sides, collect logs, write summary.md
#
# OUT (default results/<date>-two-box-sim) gets per phase: <phase>.csv (Pi telemetry, simulator ground truth),
# <phase>-status-{before,after}.txt, <phase>-events.txt, <phase>.txt (analysis). The laptop pilot is restarted 2 s
# after it exits, standing in for a guest's init; the Pi cannot restart a laptop, so no guest_restart_cmd.
set -uo pipefail
cd "$(dirname "$0")/.."
PI="${PI:?set PI=user@host of the QNX Pi}"
OUT="${OUT:-results/$(date +%F)-two-box-sim}"
RD="${REMOTE_DIR:-reflex}"
LP=build/linux-pilot-x86_64/reflex-pilot
HZ=50
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI")
mkdir -p "$OUT"

pi_ip()     { "${SSH[@]}" "ifconfig bcm0" | sed -n 's/.*inet \([0-9.]*\) .*/\1/p' | head -1; }
PI_IP="${PI_IP:-$(cat "$OUT/.pi_ip" 2>/dev/null || pi_ip)}"
[ -n "$PI_IP" ] || { echo "could not find the Pi's Wi-Fi address (set PI_IP)"; exit 1; }
echo "$PI_IP" > "$OUT/.pi_ip"
MY_IF=$(ip -4 route get "$PI_IP" | sed -n 's/.* dev \([^ ]*\).*/\1/p')
MY_IP=$(ip -4 route get "$PI_IP" | sed -n 's/.* src \([0-9.]*\).*/\1/p')

pi()    { "${SSH[@]}" "cd $RD && $*"; }
stamp() { date +%T.%3N; }
note()  { echo "[$(stamp)] $*" | tee -a "$OUT/run.log"; }

pilot_loop() {   # $@: extra pilot options
    stop_pilot
    nohup sh -c "while :; do '$PWD/$LP' --udp $PI_IP:47800 --bind 0.0.0.0:47801 --hz $HZ $* >> '$OUT/pilot.log' 2>&1;
                 echo \"[loop] \$(date +%T.%3N) pilot exited (\$?), restarting in 2 s\" >> '$OUT/pilot.log'; sleep 2; done" \
        > /dev/null 2>&1 < /dev/null &
    echo $! > "$OUT/.pilot_loop.pid"
    note "laptop pilot loop started: $LP --udp $PI_IP:47800 --bind 0.0.0.0:47801 --hz $HZ $*"
}
stop_pilot() {
    [ -f "$OUT/.pilot_loop.pid" ] && kill "$(cat "$OUT/.pilot_loop.pid")" 2>/dev/null
    rm -f "$OUT/.pilot_loop.pid"; pkill -x reflex-pilot 2>/dev/null; sleep 0.3; pkill -9 -x reflex-pilot 2>/dev/null; true
}
wait_ai() { for _ in $(seq "${1:-300}"); do pi ./rxctl status 2>/dev/null | grep -q '^mode  *AI' && return 0; sleep 0.5; done; return 1; }

analyse() {   # $1 phase
    python3 - "$OUT/$1.csv" "$OUT/$1-status-before.txt" "$OUT/$1-status-after.txt" <<'EOF' | tee "$OUT/$1.txt"
import csv, sys, collections, re
rows = list(csv.DictReader(open(sys.argv[1])))
half, strip = 200.0, 10.0                                    # rx_cfg_default: half_len 0.20 m, edge_margin 10 mm
def num(f, k):
    m = re.search(k + r'\s+(\d+)', open(f).read()); return int(m.group(1)) if m else None
def guest(f):
    m = re.search(r'^guest\s+(\S+).*restarts=(\d+)', open(f).read(), re.M); return m.groups() if m else ('?', '?')
n = len(rows); ai = sum(r['mode'] == 'AI' for r in rows)
reasons = collections.Counter(r['reason'] for r in rows if r['mode'] != 'AI')
edges = [half - abs(float(r['x_true_mm'])) for r in rows if r['x_true_mm'] not in ('', 'nan')]
lost_rows = sum(int(r['lost']) for r in rows)
b0, b1 = num(sys.argv[2], 'balls lost'), num(sys.argv[3], 'balls lost')
seqs = [int(r['seq']) for r in rows]
print(f"ticks recorded {n} ({n / 50:.1f} s at 50 Hz), gaps in the record {sum(1 for a, b in zip(seqs, seqs[1:]) if b != a + 1)}")
print(f"AI in control {100 * ai / max(n, 1):.1f} % of ticks; fallback ticks by reason: {dict(reasons)}")
print(f"closest approach to a beam end (ground truth): {min(edges):.1f} mm (keep-out strip {strip:.0f} mm)")
print(f"balls lost: {b1 - b0 if b0 is not None and b1 is not None else '?'} (counter {b0} -> {b1}); ticks with no ball reading: {lost_rows}")
print(f"guest at end: {guest(sys.argv[3])[0]}, guardian-side guest restarts counter {guest(sys.argv[2])[1]} -> {guest(sys.argv[3])[1]}")
ok = (b0 is not None and b1 == b0) and min(edges) > strip and lost_rows == 0
print("PASS: ball kept, never inside the keep-out strip" if ok else "FAIL: ball lost or inside the keep-out strip")
EOF
}

record() {   # $1 phase, $2 seconds, then "at:secs:command" actions run on this laptop during the recording
    local ph=$1 secs=$2; shift 2
    pi ./rxctl status > "$OUT/$ph-status-before.txt"
    note "phase $ph: recording $secs s on the Pi"
    pi ./rxctl log "two-box/$ph.csv" "$secs" > /dev/null &
    local rec=$! t0=$SECONDS
    for a in "$@"; do
        local at=${a%%:*} cmd=${a#*:}
        while [ $((SECONDS - t0)) -lt "$at" ]; do sleep 0.1; done
        note "phase $ph +${at}s: $cmd"; eval "$cmd"
    done
    wait $rec
    pi ./rxctl status > "$OUT/$ph-status-after.txt"; pi ./rxctl events > "$OUT/$ph-events.txt"
    scp -q "$PI:$RD/two-box/$ph.csv" "$OUT/$ph.csv"
    analyse "$ph"
}

case "${1:-}" in
start)
    [ -x "$LP" ] || make -s linux-pilot LINUX_ARCH=x86_64
    cat > "$OUT/two-box.conf" <<EOF
# reflex.conf keys for the two-box run (sim backend: transport keys only). Passed with --conf so the rig's own
# reflex.conf (serial calibration) is not changed.
pilot_transport   = udp
pilot_bind        = $PI_IP:47800
pilot_peer        = $MY_IP:47801
guest_timeout_s   = 1.0
# Wi-Fi round trip at 50 Hz, results/2026-11-23-wifi-link: p99 65.4 ms -> 100 ms = 5 periods (docs/ARCHITECTURE.md)
pilot_max_age_ms  = 100
EOF
    scp -q "$OUT/two-box.conf" "$PI:$RD/two-box.conf"
    pi "pidin -F '%N' 2>/dev/null | grep -q reflex-guardian" && { echo "a reflex-guardian is already running on $PI"; exit 1; }
    pi "mkdir -p two-box && rm -f two-box/*.csv && ./rxctl wipe > /dev/null 2>&1; nohup ./reflex-guardian --fresh --hz $HZ --conf two-box.conf > two-box/guardian.log 2>&1 < /dev/null &"
    sleep 1.5; pi cat two-box/guardian.log | tee -a "$OUT/run.log"
    : > "$OUT/pilot.log"; pilot_loop
    wait_ai 60 && note "AI in control over UDP" || { note "AI never took control"; pi ./rxctl status; exit 1; }
    {
        echo "date: $(date -Iseconds)"
        echo "commit: $(git rev-parse HEAD) (working tree clean: $(git diff --quiet HEAD && echo yes || echo no))"
        echo "laptop: $(uname -a)"; echo "pi: $("${SSH[@]}" uname -a)"
        echo "laptop $MY_IF $MY_IP -> pi bcm0 $PI_IP"; iw dev "$MY_IF" link 2>/dev/null; iw dev "$MY_IF" get power_save 2>/dev/null
        echo "guardian: cd ~/$RD && ./reflex-guardian --fresh --hz $HZ --conf two-box.conf   (as $("${SSH[@]}" id -un))"
        echo "pilot: $LP --udp $PI_IP:47800 --bind 0.0.0.0:47801 --hz $HZ   (respawned 2 s after it exits)"
    } > "$OUT/env.txt"
    ;;
normal) record normal "${2:-120}" ;;
kill)   record kill 40 "10:pkill -9 -x reflex-pilot" ;;
flip)   record flip 30 "10:pi ./rxctl inject flip >/dev/null" "20:pi ./rxctl clear >/dev/null" ;;
learn)
    pilot_loop --learn --weights "$PWD/$OUT/learn-weights.bin"
    pi ./rxctl train on > /dev/null; wait_ai 120 || note "learning pilot not in control after 60 s (recording anyway)"
    record learn "${2:-300}"
    pi ./rxctl train off > /dev/null; pi ./rxctl status | grep -E '^learning' | tee -a "$OUT/learn.txt"
    ;;
wifi)
    secs=${2:-150}
    pi ./rxctl status > "$OUT/wifi-status-before.txt"
    pi "nohup ./rxctl log two-box/wifi.csv $secs > /dev/null 2>&1 < /dev/null &"
    note "phase wifi: recording $secs s on the Pi (detached). Turn the laptop Wi-Fi OFF now, wait ~10 s, turn it back ON."
    t0=$SECONDS; down=""; up=""
    while [ $((SECONDS - t0)) -lt $((secs - 5)) ]; do
        s=$(cat "/sys/class/net/$MY_IF/operstate" 2>/dev/null || echo missing)
        if [ -z "$down" ] && [ "$s" != up ]; then down=$SECONDS; note "laptop $MY_IF went $s (+$((down - t0)) s)"; fi
        if [ -n "$down" ] && [ -z "$up" ] && [ "$s" = up ]; then up=$SECONDS; note "laptop $MY_IF up again (+$((up - t0)) s), address $(ip -4 -br addr show "$MY_IF" | awk '{print $3}')"; fi
        sleep 0.2
    done
    [ -n "$down" ] || note "Wi-Fi was never turned off during the recording"
    while [ $((SECONDS - t0)) -lt $((secs + 3)) ]; do sleep 0.5; done
    for _ in $(seq 60); do "${SSH[@]}" true 2>/dev/null && break; sleep 1; done
    pi ./rxctl status > "$OUT/wifi-status-after.txt"; pi ./rxctl events > "$OUT/wifi-events.txt"
    scp -q "$PI:$RD/two-box/wifi.csv" "$OUT/wifi.csv"
    analyse wifi
    ;;
stop)
    stop_pilot
    pi ./rxctl status > "$OUT/final-status.txt"; pi ./rxctl events > "$OUT/final-events.txt"
    pi "slay -s INT reflex-guardian" 2>/dev/null; sleep 1
    scp -q "$PI:$RD/two-box/guardian.log" "$OUT/guardian.log"
    note "stopped"
    ;;
*) sed -n 3,20p "$0"; exit 2 ;;
esac
