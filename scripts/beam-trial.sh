#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# One trial on the beam-direct rig: start the guardian (--backend beam), set the setpoint, record, stop, fetch
# and analyse. The person puts the ball on the beam first.
#
#   scripts/beam-trial.sh <name> <secs> [setpoint: <mm> | steps:<mm>:<period_s>] [guardian options...]
#   e.g. scripts/beam-trial.sh t1 30 0 --no-pilot
#
# Files in results/2026-11-24-beam-direct/step4-<name>.*: guardian.log, events.txt, rec.csv (guardian
# telemetry, rxctl log), beam.csv (I/O helper: raw sonar, pulse, current, supply, MPU tilt), summary.txt.
set -uo pipefail
cd "$(dirname "$0")/.."
NAME=${1:?name}; SECS=${2:?secs}; SP=${3:-0}; shift 3 2>/dev/null || shift $#
OUT="${OUT:-results/2026-11-24-beam-direct}/step4-$NAME"
PI_IP="${PI_IP:-10.203.87.190}"
SSH=(ssh -o HostName="$PI_IP" -o HostKeyAlias=qnxpi.local -o BatchMode=yes -o ConnectTimeout=5 qnxpi)
SCP_OPTS=(-o HostName="$PI_IP" -o HostKeyAlias=qnxpi.local -o BatchMode=yes)
pi() { "${SSH[@]}" "cd reflex && $*"; }
mkdir -p "$(dirname "$OUT")"
pi "pidin -F '%N' 2>/dev/null | grep -q reflex-guardian" && { echo "a guardian is already running on the Pi"; exit 1; }
KICK_ENV=""; [ -n "${KICK:-}" ] && KICK_ENV="RX_BEAM_KICK=$KICK"           # KICK=mrad:ms (test disturbance)
pi "./rxctl wipe >/dev/null 2>&1; rm -f beam.csv; ($KICK_ENV nohup ./reflex-guardian --backend beam --fresh $* > beam-guardian.log 2>&1 < /dev/null &)"
sleep 1.5
case "$SP" in
    steps:*) IFS=: read -r _ AMP PER <<< "$SP"; pi "./rxctl period $((PER * 1000)) >/dev/null; ./rxctl sp auto >/dev/null" ;;
    *)       pi "./rxctl sp $SP >/dev/null" ;;
esac
sleep 0.5
pi "./rxctl log trial.csv $SECS >/dev/null"
pi "./rxctl events > trial-events.txt; ./rxctl status > trial-status.txt; slay -s INT reflex-guardian; sleep 1"
for f in beam-guardian.log:guardian.log trial-events.txt:events.txt trial-status.txt:status.txt trial.csv:rec.csv beam.csv:beam.csv; do
    scp -q "${SCP_OPTS[@]}" "qnxpi:reflex/${f%%:*}" "$OUT.${f##*:}"
done
{
    echo "trial $NAME, $(date -Iseconds), commit $(git rev-parse --short HEAD)$(git diff --quiet HEAD -- src || echo '-dirty')"
    echo "command: ${KICK:+KICK=$KICK }scripts/beam-trial.sh $NAME $SECS $SP $*"
    grep beam-direct "$OUT.guardian.log"
    python3 tools/beam_trial.py "$OUT"
} | tee "$OUT.summary.txt"
