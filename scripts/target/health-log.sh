#!/bin/sh
# SPDX-License-Identifier: MIT
# Reflex: log the Pi's power/thermal state every 2 s to a file that survives a reset
# (each line is synced to disk), to diagnose unexplained reboots.
#   ./health-log.sh [file]  (default ~/reflex/health.log); stop with: slay health-log.sh
# throttled bits (Raspberry Pi firmware): 0x1 under-voltage now, 0x2 arm freq capped, 0x4 throttled,
# 0x8 soft temp limit; 0x10000.. the same "has occurred since boot". The flags reset at every boot,
# so the LAST line before a reboot is the evidence.
F="${1:-$HOME/reflex/health.log}"
echo "# boot $(pidin info | head -1)" >> "$F"
while :; do
    echo "$(date -u +%H:%M:%S) $(vcgencmd get_throttled) $(vcgencmd measure_temp) $(vcgencmd measure_volts core) procs=$(pidin ar 2>/dev/null | grep -cE 'reflex-(guardian|pilot)')" >> "$F"
    sync
    sleep 2
done
