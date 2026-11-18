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
ssh "$PI" 'cd reflex && chmod +x reflex-guardian reflex-pilot rxctl rxdash rx_simtest rxfs rxham 2>/dev/null; ls -l'
if [ "${1:-}" = "--test" ]; then
    ssh "$PI" 'cd reflex && ./rx_simtest | tail -3'
fi
