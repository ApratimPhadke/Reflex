#!/bin/sh
# SPDX-License-Identifier: MIT
# Reflex: boot hook for the Raspberry Pi 4 QNX 8.0 quick-start image.
#
# Installed root-owned as /system/etc/startup/reflex_startup.sh by scripts/install-autostart.sh and
# called (in the background) from one guarded line at the end of /system/etc/startup/post_startup.sh.
# Runs as root, so it never executes anything from a user-writable directory.
#
# 1. USB serial: the QSTI does not ship devc-serusb, so the Black Pill (USB CDC-ACM) would not appear.
#    Start the copy installed in /system/bin; it attaches to the Black Pill whenever it is plugged in
#    and re-creates /dev/serusb1 on a replug.
# 2. Optional demo autostart: only if /system/etc/reflex/autostart exists (root-owned, created by
#    install-autostart.sh --demo). The demo itself runs as qnxuser, never as root.

LOG=/dev/shmem/reflex_startup.log
echo "reflex_startup: $(date)" > $LOG

if [ -x /system/bin/devc-serusb ]; then
    waitfor /dev/usb/io-usb-otg 20 >> $LOG 2>&1
    slay -p devc-serusb > /dev/null 2>&1        # exit status = number of matching processes
    if [ $? -ne 0 ]; then
        echo "devc-serusb already running" >> $LOG
    else
        /system/bin/devc-serusb >> $LOG 2>&1 &
        echo "devc-serusb started" >> $LOG
    fi
else
    echo "/system/bin/devc-serusb missing: run scripts/install-autostart.sh" >> $LOG
fi

if [ -f /system/etc/reflex/autostart ]; then
    DEMO=/data/home/qnxuser/reflex/reflex-demo.sh
    if [ -x "$DEMO" ]; then
        waitfor /dev/serusb1 5 >> $LOG 2>&1          # give a plugged-in Black Pill a chance to appear
        on -u 1000:1000 sh -c "cd /data/home/qnxuser/reflex && HOME=/data/home/qnxuser ./reflex-demo.sh start" >> $LOG 2>&1 &
        echo "demo autostart requested (as qnxuser)" >> $LOG
    else
        echo "autostart flag set but $DEMO missing" >> $LOG
    fi
fi
exit 0
