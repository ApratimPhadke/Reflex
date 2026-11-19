#!/usr/bin/env bash
# Ground-truth closest approach to the beam end for the stuck and flip faults, guardian ON, serial
# backend at 50 Hz against tools/fake_io_node.py (SIGHUP restarts its min-edge window). Linux host.
cd "$(dirname "$0")/../.." || exit 1; B=build/linux; L=/tmp/reflex-stuck; S=/tmp/reflex-stuck.json
python3 tools/fake_io_node.py --link $L --status $S >/dev/null 2>&1 & F=$!; sleep 1
$B/reflex-guardian --backend serial --dev $L --fresh >/dev/null 2>&1 & G=$!; sleep 4
for f in stuck flip; do for i in 1 2 3 4 5; do
  $B/rxctl clear; $B/rxctl release; sleep 3; kill -HUP $F; sleep 0.6; e0=$(python3 -c "import json;print(json.load(open('$S'))['min_edge_mm'])")
  $B/rxctl inject $f >/dev/null; sleep 4; sleep 0.6
  python3 -c "import json;d=json.load(open('$S'));print('$f run $i: true min edge %.1f mm, balls lost %d'%(d['min_edge_mm'],d['balls_lost']))"
done; done
kill $G; sleep 0.3; kill $F; wait
