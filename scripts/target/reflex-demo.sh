#!/bin/sh
# SPDX-License-Identifier: MIT
# Reflex: the whole demo on the target with one command (copied to ~/reflex by make deploy).
#
#   ./reflex-demo.sh start [--sim | --serial] [guardian options, e.g. --learn-from-sim | --learn | --no-pilot]
#   ./reflex-demo.sh stop
#   ./reflex-demo.sh status
#
# start picks the plant: the serial backend if an I/O node is plugged in ($REFLEX_DEV, default
# /dev/serusb1) AND reflex.conf passes the guardian's own check; otherwise the simulator, and it
# says why. It starts the guardian under the High Availability Manager (ham + rxham) when both are
# here and sudo works, else on its own; rxfs (/dev/reflex) when sudo works; and rxdash on
# $REFLEX_PORT (default 8080). Logs go to ./logs/.
cd "$(dirname "$0")" || exit 1
DIR=$(pwd)
DEV=${REFLEX_DEV:-/dev/serusb1}
PORT=${REFLEX_PORT:-8080}
LOGS=$DIR/logs

running()   { slay -p "$1" > /dev/null 2>&1; [ $? -ne 0 ]; }    # slay -p: exit status = number of matches
have_sudo() { sudo -n true > /dev/null 2>&1; }
as_root()   { if have_sudo; then sudo -n "$@"; else "$@"; fi; }
ip_addr()   { ifconfig 2>/dev/null | awk '/inet / && $2 != "127.0.0.1" { print $2; exit }'; }

do_status() {
    for p in ham reflex-guardian reflex-pilot rxfs rxdash devc-serusb; do
        if running "$p"; then printf '  %-16s running (pid %s)\n' "$p" "$(slay -p "$p" 2>/dev/null | tr '\n' ' ')"
        else printf '  %-16s -\n' "$p"; fi
    done
    [ -c "$DEV" ] && echo "  I/O node         $DEV present" || echo "  I/O node         $DEV absent"
    if running reflex-guardian; then
        ./rxctl status 2>/dev/null | sed -n '1,4p;/balls lost/p' | sed 's/^/  /'
        echo "  dashboard        http://$(ip_addr):$PORT/   (or http://$(hostname).local:$PORT/)"
    fi
}

do_start() {
    if running reflex-guardian; then echo "reflex-demo: already running (./reflex-demo.sh stop first)"; do_status; return 1; fi
    mkdir -p "$LOGS"
    backend=auto
    case "${1:-}" in --sim) backend=sim; shift ;; --serial) backend=serial; shift ;; esac
    if [ "$backend" = auto ]; then
        if [ ! -c "$DEV" ]; then
            backend=sim; echo "reflex-demo: no I/O node at $DEV -> simulated plant"
        elif ! ./reflex-guardian --check-conf > "$LOGS/check-conf.txt" 2>&1; then
            backend=sim; echo "reflex-demo: I/O node present but no valid calibration ($(tail -1 "$LOGS/check-conf.txt")) -> simulated plant"
            echo "             calibrate with: ./rxcal --dev $DEV"
        else
            backend=serial; echo "reflex-demo: I/O node at $DEV, $(cat "$LOGS/check-conf.txt") -> real plant"
        fi
    fi
    if [ "$backend" = serial ]; then
        [ -c "$DEV" ] || { echo "reflex-demo: --serial but $DEV does not exist"; return 1; }
        ./reflex-guardian --check-conf > "$LOGS/check-conf.txt" 2>&1 || { cat "$LOGS/check-conf.txt"; echo "run ./rxcal --dev $DEV first"; return 1; }
        set -- --backend serial --dev "$DEV" "$@"
    fi
    set -- --weights "$DIR/reflex-policy.bin" "$@"          # absolute: HAM's working directory is not ours
    ./rxctl wipe > /dev/null 2>&1      # a demo start gets a fresh world and zeroed counters (HAM restarts still resume)

    if have_sudo && [ -x ./ham ] && [ -x ./rxham ]; then
        running ham || { sudo -n sh -c "cd '$DIR'; ./ham > '$LOGS/ham.log' 2>&1 < /dev/null &" > /dev/null 2>&1 < /dev/null; sleep 1; }
        if sudo -n ./rxham "$DIR/reflex-guardian" "$@" > "$LOGS/rxham.log" 2>&1 < /dev/null; then how="under HAM (restarted if it dies)"
        else echo "reflex-demo: rxham failed ($(tail -1 "$LOGS/rxham.log")), starting the guardian on its own"; how=""; fi
    fi
    if [ -z "${how:-}" ]; then
        nohup ./reflex-guardian "$@" > "$LOGS/guardian.log" 2>&1 < /dev/null &
        how="on its own (no HAM)"
    fi
    i=0; until ./rxctl status > /dev/null 2>&1; do i=$((i+1)); [ $i -gt 50 ] && { echo "reflex-demo: guardian did not come up; see $LOGS"; return 1; }; sleep 0.1; done
    if have_sudo && [ -x ./rxfs ] && ! running rxfs; then sudo -n sh -c "cd '$DIR'; ./rxfs > '$LOGS/rxfs.log' 2>&1 < /dev/null &" > /dev/null 2>&1 < /dev/null; fi
    if ! running rxdash; then nohup ./rxdash "$PORT" > "$LOGS/rxdash.log" 2>&1 < /dev/null & fi
    sleep 0.5
    echo "reflex-demo: guardian started $how, plant: $backend"
    do_status
}

do_stop() {
    [ -x ./rxham ] && have_sudo && sudo -n ./rxham -r > /dev/null 2>&1      # first: stop HAM from restarting it
    for p in reflex-guardian; do running "$p" && as_root slay -s INT "$p" > /dev/null 2>&1; done
    i=0; while running reflex-guardian && [ $i -lt 30 ]; do sleep 0.1; i=$((i+1)); done
    running reflex-guardian && as_root slay -s KILL reflex-guardian > /dev/null 2>&1
    for p in reflex-pilot rxdash; do running "$p" && as_root slay "$p" > /dev/null 2>&1; done
    running rxfs && as_root slay rxfs > /dev/null 2>&1
    # ham is left running on purpose: it supervises itself, and idle it costs nothing
    sleep 0.3
    echo "reflex-demo: stopped (the I/O node levels the beam by itself; ham stays up, idle)"
    do_status
}

case "${1:-}" in
start)  shift; do_start "$@" ;;
stop)   do_stop ;;
status) do_status ;;
*) echo "usage: $0 start [--sim|--serial] [guardian options] | stop | status"; exit 2 ;;
esac
