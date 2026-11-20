#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Reflex: make the Pi 4 (QNX 8.0 quick-start image) plug-and-play at boot.
#
#   PI=qnxpi scripts/install-autostart.sh            install: USB serial driver started at every boot
#   PI=qnxpi scripts/install-autostart.sh --demo     also start ~/reflex/reflex-demo.sh (as qnxuser) at boot
#   PI=qnxpi scripts/install-autostart.sh --no-demo  keep the driver, stop autostarting the demo
#   PI=qnxpi scripts/install-autostart.sh --remove   undo (restores the original post_startup.sh)
#   PI=qnxpi scripts/install-autostart.sh --status   show what is installed and the last boot log
#
# Changes on the target (root via the QSTI's passwordless sudo; logic in scripts/target/reflex_install.sh):
#   /system/bin/devc-serusb                   the SDP's USB serial driver (the QSTI does not ship it)
#   /system/etc/startup/reflex_startup.sh     scripts/target/reflex_startup.sh, root-owned 0555
#   /system/etc/startup/post_startup.sh       ONE guarded line before its final "exit 0";
#                                             original kept as post_startup.sh.reflex-backup-<timestamp>
#   /system/etc/reflex/autostart              flag file, only with --demo
# Nothing user-writable is executed as root.
set -euo pipefail
cd "$(dirname "$0")/.."
PI="${PI:?set PI, e.g. PI=qnxpi}"
case "${1:-}" in
"")          mode=install ;;
--demo)      mode=demo ;;
--no-demo)   mode=no-demo ;;
--remove)    mode=remove ;;
--status)    mode=status ;;
*) echo "usage: PI=... $0 [--demo|--no-demo|--remove|--status]" >&2; exit 2 ;;
esac
STAGE=/tmp/reflex-install
ssh "$PI" "rm -rf $STAGE && mkdir -p $STAGE"
scp -q scripts/target/reflex_install.sh "$PI":$STAGE/
if [ "$mode" = install ] || [ "$mode" = demo ] || [ "$mode" = no-demo ]; then
    [ -n "${QNX_TARGET:-}" ] || { echo "run: source ~/qnx800/qnxsdp-env.sh" >&2; exit 1; }
    DRV="$QNX_TARGET/aarch64le/sbin/devc-serusb"
    [ -f "$DRV" ] || { echo "$DRV not found in the SDP" >&2; exit 1; }
    scp -q "$DRV" scripts/target/reflex_startup.sh "$PI":$STAGE/
fi
ssh "$PI" "cd $STAGE && sudo -n sh ./reflex_install.sh $mode; rc=\$?; cd / && rm -rf $STAGE; exit \$rc"
[ "$mode" = install ] || [ "$mode" = demo ] && echo "Takes effect at the next boot. To start it now: ssh $PI 'sudo /system/etc/startup/reflex_startup.sh'"
exit 0
