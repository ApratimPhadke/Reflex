#!/usr/bin/env bash
# Ground-truth closest approach for the stuck fault injected at the campaign's phase
# (1 s after the setpoint square wave goes positive), fake I/O node, serial backend, 50 Hz.
cd "$1" || exit 1; B=build/linux; L=/tmp/reflex-phase; ST=/tmp/reflex-phase.json
python3 tools/fake_io_node.py --link $L --status $ST >/dev/null 2>&1 & F=$!; sleep 1
$B/reflex-guardian --backend serial --dev $L --fresh >/dev/null 2>&1 & G=$!; sleep 4
$B/rxctl period 4000
sp() { $B/rxctl status | awk '/^ball/{for(i=1;i<=NF;i++) if($i ~ /^setpoint=/){sub("setpoint=","",$i); sub("mm","",$i); print $i}}'; }
for f in ${FAULTS:-stuck}; do for i in 1 2 3 4 5; do
  $B/rxctl clear; $B/rxctl release; sleep 3
  prev=$(sp); while :; do cur=$(sp); [ "${prev%%.*}" -lt 0 ] 2>/dev/null && [ "${cur%%.*}" -gt 0 ] 2>/dev/null && break; prev=$cur; sleep 0.02; done
  sleep 1; kill -HUP $F; $B/rxctl inject $f >/dev/null; sleep 4.6
  python3 -c "import json;d=json.load(open('$ST'));print('$f (campaign phase) run $i: true min edge %.1f mm, balls lost %d'%(d['min_edge_mm'],d['balls_lost']))"
done; done
kill $G; sleep 0.3; kill $F; wait
