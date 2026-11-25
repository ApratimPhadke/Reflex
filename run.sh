#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Reflex, beam-direct rig: one command from the laptop.
#
#   ./run.sh                 start: guardian (QNX, beam backend) + AI pilot + dashboard on the Pi, open the dashboard
#   ./run.sh --fallback      guardian only, no AI: the verified fallback controller balances the ball
#   ./run.sh --learn         the AI pilot learns online (starting from the simulator weights), guardian on
#   ./run.sh --deploy [...]  build for QNX first and copy the binaries to the Pi, then start
#   ./run.sh stop            stop guardian, pilot and dashboard (the servo goes limp)
#   ./run.sh status          the guardian's status
#   ./run.sh logs            guardian log + the last events
#
# Environment: PI_IP (default 10.203.87.190), PI (ssh host alias, default qnxpi), PORT (dashboard, default 8080),
#              NO_BROWSER=1 to not open a browser.
set -uo pipefail
cd "$(dirname "$0")"
PI_IP="${PI_IP:-10.203.87.190}"; PI="${PI:-qnxpi}"; PORT="${PORT:-8080}"
SSH=(ssh -o HostName="$PI_IP" -o HostKeyAlias=qnxpi.local -o BatchMode=yes -o ConnectTimeout=5 "$PI")
SCP=(scp -q -o HostName="$PI_IP" -o HostKeyAlias=qnxpi.local -o BatchMode=yes)
URL="http://$PI_IP:$PORT/"
say()  { printf '\033[1m%s\033[0m\n' "$*"; }
die()  { printf '\033[31m%s\033[0m\n' "$*" >&2; exit 1; }
pi()   { "${SSH[@]}" "cd reflex && $*"; }

stop_all() {
    pi 'slay -s INT reflex-guardian 2>/dev/null; slay -s TERM rxdash rxhw 2>/dev/null; sleep 1.5; slay -s KILL reflex-guardian reflex-pilot 2>/dev/null; true'
}

deploy() {
    [ -n "${QNX_TARGET:-}" ] || { [ -f ~/qnx800/qnxsdp-env.sh ] && source ~/qnx800/qnxsdp-env.sh >/dev/null; }
    [ -n "${QNX_TARGET:-}" ] || die "QNX SDP not found (source ~/qnx800/qnxsdp-env.sh)"
    say "building for QNX (aarch64)..."
    make -s QNX=1 ARCH=aarch64 >/dev/null || die "build failed"
    say "copying to the Pi..."
    stop_all
    "${SCP[@]}" build/qnx-aarch64/{reflex-guardian,reflex-pilot,rxctl,rxdash,rxhw} "$PI:reflex/" || die "copy failed"
}

MODE=ai; DEPLOY=0; CMD=start
for a in "$@"; do
    case "$a" in
        --fallback) MODE=fallback ;; --learn) MODE=learn ;; --ai) MODE=ai ;; --deploy) DEPLOY=1 ;;
        stop|status|logs|start) CMD=$a ;;
        -h|--help) sed -n 3,16p "$0"; exit 0 ;;
        *) die "unknown option $a (./run.sh --help)" ;;
    esac
done

"${SSH[@]}" true 2>/dev/null || die "cannot reach the Pi at $PI_IP (Wi-Fi? PI_IP=...)"
case "$CMD" in
    stop)   stop_all; say "stopped: guardian, pilot and dashboard; the servo is limp"; exit 0 ;;
    status) pi './rxctl status'; echo "dashboard: $URL"; exit 0 ;;
    logs)   pi 'cat beam-guardian.log; echo; ./rxctl events | tail -25'; exit 0 ;;
esac

[ "$DEPLOY" = 1 ] && deploy
pi './reflex-guardian --backend beam --check-conf' >/dev/null 2>&1 || die "reflex.conf on the Pi is not a valid beam-direct calibration (steps 2-3)"
say "starting the beam-direct rig ($MODE)..."
stop_all
case "$MODE" in
    fallback) GOPT="--no-pilot" ;;
    learn)    GOPT="--learn-from-sim --weights reflex-policy-beam.bin" ;;
    *)        GOPT="" ;;
esac
pi "./rxctl wipe >/dev/null 2>&1; (nohup ./reflex-guardian --backend beam --fresh $GOPT > beam-guardian.log 2>&1 < /dev/null &)"
for _ in $(seq 30); do pi './rxctl status >/dev/null 2>&1' && break; sleep 0.3; done
pi './rxctl status >/dev/null 2>&1' || { pi 'cat beam-guardian.log'; die "the guardian did not start"; }
pi './rxctl sp 0 >/dev/null'
pi '(nohup ./rxdash '"$PORT"' > rxdash.log 2>&1 < /dev/null &)'
for _ in $(seq 30); do curl -s --max-time 1 "$URL/api/state" >/dev/null && break; sleep 0.3; done
curl -s --max-time 2 "$URL/api/state" >/dev/null || die "the dashboard did not come up ($URL)"
pi 'head -1 beam-guardian.log'
pi './rxctl status | sed -n 2,3p'
if [ -z "${NO_BROWSER:-}" ] && command -v xdg-open >/dev/null; then (xdg-open "$URL" >/dev/null 2>&1 &); fi
say "running. Dashboard: $URL"
echo "  put the ball on the beam (~9 cm from the sensor), click a target on the dashboard"
echo "  e-stop: the button on the rig (beam goes level, AI stops), release it on the dashboard"
echo "  ./run.sh stop   to stop everything"
