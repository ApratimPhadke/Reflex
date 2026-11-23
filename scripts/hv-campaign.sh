#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Reflex-HV: run the fault campaign with the guest stage on the QNX Hypervisor host and file the
# evidence under results/<YYYY-MM-DD>-hv/ (or the directory given), with the commit, uname -a of the QNX
# host and of the Linux guest, and the exact commands.
#
#   PI=root@qnxpi scripts/hv-campaign.sh [outdir]
#
# Needs on the target: a running reflex-guardian with pilot_transport = udp and guest_ssh in ~/reflex/reflex.conf
# (docs/HYPERVISOR.md), the guest's pilot up, and key-based ssh from the target to the guest for the user in $PI.
# The guest stage panics the guest kernel once per pass: the guest must come back by itself (panic=N on its
# kernel command line) or through guest_restart_cmd. Takes about 10 minutes.
set -euo pipefail
cd "$(dirname "$0")/.."
PI="${PI:?set PI=user@host of the QNX Hypervisor host}"
OUT="${1:-results/$(date +%F)-hv}"
REMOTE_DIR="${REMOTE_DIR:-reflex}"
mkdir -p "$OUT"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI")

hash=$(git rev-parse HEAD); dirty=$(git diff --quiet HEAD && echo "" || echo " (working tree had uncommitted changes)")
guest=$("${SSH[@]}" "sed -n 's/^ *guest_ssh *= *\\([^ #]*\\).*/\\1/p' $REMOTE_DIR/reflex.conf" || true)
[ -n "$guest" ] || { echo "no guest_ssh in $PI:$REMOTE_DIR/reflex.conf"; exit 1; }
"${SSH[@]}" "cd $REMOTE_DIR && ./rxctl status" > "$OUT/status-before.txt" || { echo "no guardian running on $PI"; exit 1; }
grep -q '^pilot link *udp' "$OUT/status-before.txt" || { echo "the guardian on $PI is not on pilot_transport = udp"; cat "$OUT/status-before.txt"; exit 1; }

{
    echo "# Reflex-HV fault campaign, $(date -Iseconds)"
    echo
    echo "- commit: \`$hash\`$dirty"
    echo "- QNX host: \`$("${SSH[@]}" 'uname -a')\`"
    echo "- Linux guest ($guest): \`$("${SSH[@]}" "ssh -o BatchMode=yes -o ConnectTimeout=3 $guest 'uname -a'" 2>/dev/null || echo 'not reachable')\`"
    echo "- command (on the host, in ~/$REMOTE_DIR): \`./rxctl campaign campaign-hv.md --guest\`"
    echo "- collected by: \`PI=$PI scripts/hv-campaign.sh $OUT\`"
} > "$OUT/README.md"

"${SSH[@]}" "cd $REMOTE_DIR && ./rxctl reset && ./rxctl campaign campaign-hv.md --guest" 2> "$OUT/campaign.log"
scp -q "$PI:$REMOTE_DIR/campaign-hv.md" "$OUT/campaign.md"
"${SSH[@]}" "cd $REMOTE_DIR && ./rxctl status && ./rxctl events && ./rxctl latency" > "$OUT/status-after.txt"
echo "evidence in $OUT/"
grep -E '^\*\*' "$OUT/campaign.md" || true
