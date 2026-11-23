#!/usr/bin/env bash
# Reflex: build for QNX and copy to the target.
#   PI=qnxuser@qnxpi.local scripts/deploy.sh            build aarch64 + copy to ~/reflex on the Pi
#   PI=root@<qemu-ip> ARCH=x86_64 scripts/deploy.sh     same for a QEMU target
#   ... scripts/deploy.sh --test                         also run rx_simtest on the target
set -euo pipefail
cd "$(dirname "$0")/.."
PI="${PI:?set PI, e.g. PI=qnxuser@qnxpi.local}"
ARCH="${ARCH:-aarch64}"
[ -n "${QNX_TARGET:-}" ] || { echo "run: source ~/qnx800/qnxsdp-env.sh" >&2; exit 1; }
make QNX=1 ARCH="$ARCH" ${MAKEFLAGS_EXTRA:-}
ssh "$PI" 'mkdir -p reflex'
scp -q build/qnx-"$ARCH"/* "$PI":reflex/
scp -q scripts/target/reflex-demo.sh scripts/target/health-log.sh "$PI":reflex/
ssh "$PI" 'mkdir -p reflex/demo'
scp -q demo/*.rxs "$PI":reflex/demo/
# The Pi 4 quick-start image has no High Availability Manager; ship the SDP's copy next to the binaries.
HAM_BIN="$QNX_TARGET/${ARCH/aarch64/aarch64le}/usr/sbin/ham"
[ -f "$HAM_BIN" ] && scp -q "$HAM_BIN" "$PI":reflex/ham || echo "note: $HAM_BIN not found, HAM demo needs ham on the target" >&2
ssh "$PI" 'cd reflex && chmod +x reflex-guardian reflex-pilot rxctl rxdash rxcal rx_simtest rx_linktest rx_udpping rxfs rxham ham reflex-demo.sh health-log.sh 2>/dev/null; ls -l'
if [ "${1:-}" = "--test" ]; then
    ssh "$PI" 'cd reflex && ./rx_simtest | tail -3'
fi
