#!/bin/sh
# SPDX-License-Identifier: MIT
# Reflex: target side of scripts/install-autostart.sh. Run as root on the Pi:
#   sh reflex_install.sh install|demo|no-demo|remove|status   [files staged in the current directory]
# install expects ./devc-serusb and ./reflex_startup.sh next to it.
PS=/system/etc/startup/post_startup.sh
HOOKS=/system/etc/startup/reflex_startup.sh
MARK='# reflex: USB serial + optional demo (scripts/install-autostart.sh)'
HOOK='[ -x /system/etc/startup/reflex_startup.sh ] && /system/etc/startup/reflex_startup.sh &'
mode="${1:-status}"

add_hook() {
    if grep -q 'reflex_startup.sh' "$PS"; then echo "post_startup.sh already has the reflex hook"; return 0; fi
    ts=$(date +%Y%m%d-%H%M%S)
    cp -p "$PS" "$PS.reflex-backup-$ts" || return 1
    # insert before the first line that is exactly "exit 0" (the QSTI script ends with one), else append
    awk -v mark="$MARK" -v hook="$HOOK" '
        $0 == "exit 0" && !done { print mark; print hook; done = 1 }
        { print }
        END { if (!done) { print mark; print hook } }' "$PS.reflex-backup-$ts" > "$PS.reflex-new" || return 1
    cat "$PS.reflex-new" > "$PS" && rm -f "$PS.reflex-new"
    echo "hook added to post_startup.sh; backup: $PS.reflex-backup-$ts"
}

case "$mode" in
install|demo|no-demo)
    [ -f ./devc-serusb ] && [ -f ./reflex_startup.sh ] || { echo "stage devc-serusb and reflex_startup.sh first"; exit 1; }
    if cmp -s ./devc-serusb /system/bin/devc-serusb; then echo "devc-serusb already installed"
    else cp ./devc-serusb /system/bin/devc-serusb && chown root:root /system/bin/devc-serusb && chmod 0555 /system/bin/devc-serusb || exit 1; fi
    cp ./reflex_startup.sh "$HOOKS" && chown root:root "$HOOKS" && chmod 0555 "$HOOKS" || exit 1
    add_hook || exit 1
    mkdir -p /system/etc/reflex
    [ "$mode" = demo ] && touch /system/etc/reflex/autostart && echo "demo autostart ON"
    [ "$mode" = no-demo ] && rm -f /system/etc/reflex/autostart && echo "demo autostart OFF"
    ;;
remove)
    b=$(ls "$PS".reflex-backup-* 2>/dev/null | sort | head -1)       # the oldest backup = the original
    if [ -n "$b" ]; then cat "$b" > "$PS"; echo "restored post_startup.sh from $b"
    else grep -v -e 'reflex_startup.sh' -e '^# reflex:' "$PS" > "$PS.reflex-new" && cat "$PS.reflex-new" > "$PS" && rm -f "$PS.reflex-new"; echo "hook lines removed"; fi
    rm -f "$HOOKS" /system/etc/reflex/autostart; rmdir /system/etc/reflex 2>/dev/null
    echo "removed $HOOKS and the autostart flag (/system/bin/devc-serusb left in place)"
    ;;
status) ;;
*) echo "usage: $0 install|demo|no-demo|remove|status"; exit 2 ;;
esac
echo "--- status"
ls -l /system/bin/devc-serusb "$HOOKS" 2>&1
[ -f /system/etc/reflex/autostart ] && echo "demo autostart: ON" || echo "demo autostart: off"
grep -n 'reflex' "$PS" || echo "post_startup.sh: no reflex hook"
tail -3 "$PS"
[ -f /dev/shmem/reflex_startup.log ] && { echo "--- last boot hook log"; cat /dev/shmem/reflex_startup.log; }
exit 0
