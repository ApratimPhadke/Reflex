#!/usr/bin/env bash
# Reflex: check that the host is ready. Prints PASS/FAIL per item and never changes anything.
#   make doctor            (or: PI=qnxuser@<ip> make doctor to also check the Pi)
set -u
pass=0; fail=0; warnc=0
ok()   { printf '  \033[32mPASS\033[0m  %s\n' "$*"; pass=$((pass+1)); }
bad()  { printf '  \033[31mFAIL\033[0m  %s\n' "$*"; fail=$((fail+1)); }
note() { printf '  \033[33mWARN\033[0m  %s\n' "$*"; warnc=$((warnc+1)); }

echo "host"
. /etc/os-release 2>/dev/null || true
[ "${VERSION_ID:-}" = "24.04" ] && ok "Ubuntu 24.04" || note "OS is ${PRETTY_NAME:-unknown} (guides assume Ubuntu 24.04)"
[ "$(uname -m)" = "x86_64" ] && ok "x86_64 CPU" || bad "QNX SDP needs an x86_64 host"
command -v gcc >/dev/null && ok "gcc $(gcc -dumpversion)" || bad "gcc missing (sudo apt install build-essential)"
python3 -c 'import numpy' 2>/dev/null && ok "python3 numpy" || bad "numpy missing (sudo apt install python3-numpy)"
python3 -c 'import matplotlib' 2>/dev/null && ok "python3 matplotlib" || note "matplotlib missing (sudo apt install python3-matplotlib), needed for evidence plots"

echo "QNX SDP"
if [ -n "${QNX_HOST:-}" ] && [ -n "${QNX_TARGET:-}" ]; then
    ok "QNX environment loaded (QNX_TARGET=$QNX_TARGET)"
    for cc in ntoaarch64-gcc ntox86_64-gcc; do
        command -v "$cc" >/dev/null && ok "$cc $($cc -dumpversion 2>/dev/null)" || bad "$cc not on PATH"
    done
    [ -f "$QNX_TARGET/usr/include/sys/neutrino.h" ] && ok "QNX headers present" || bad "QNX headers not found under QNX_TARGET"
    [ -f "$QNX_TARGET/usr/include/ha/ham.h" ] && ok "HAM headers present (rxham will build)" || note "ha/ham.h not found: build with HAM=0"
    ls "$QNX_TARGET"/aarch64le/usr/lib/libham* >/dev/null 2>&1 && ok "libham for aarch64" || note "libham for aarch64 not found: build with HAM=0"
else
    bad "QNX environment not loaded: run  source ~/qnx800/qnxsdp-env.sh"
fi

echo "firmware tools"
AC="$(command -v arduino-cli || echo "$HOME/.local/bin/arduino-cli")"
if [ -x "$AC" ]; then
    ok "arduino-cli $("$AC" version 2>/dev/null | awk '{print $3}')"
    "$AC" core list 2>/dev/null | grep -q '^STMicroelectronics:stm32' && ok "STM32 core installed" || bad "STM32 core missing (scripts/setup-host-ubuntu24.sh)"
    "$AC" lib list 2>/dev/null | grep -q '^VL53L0X' && ok "VL53L0X library installed" || bad "VL53L0X library missing (arduino-cli lib install VL53L0X)"
else
    bad "arduino-cli missing (scripts/setup-host-ubuntu24.sh)"
fi
command -v dfu-util >/dev/null && ok "dfu-util" || bad "dfu-util missing (sudo apt install dfu-util)"
[ -f /etc/udev/rules.d/50-reflex-blackpill.rules ] && ok "Black Pill udev rules" || bad "udev rules missing (scripts/setup-host-ubuntu24.sh)"
id -nG | grep -qw dialout && ok "user in dialout" || bad "user not in dialout: sudo usermod -aG dialout \$USER, then log out/in"

echo "devices (only needed when plugged in)"
if command -v lsusb >/dev/null; then
    lsusb | grep -qi '0483:df11' && ok "Black Pill in DFU mode detected"
    lsusb | grep -qi '0483:5740' && ok "Black Pill running Reflex firmware detected ($(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' '))"
    lsusb | grep -qiE '0483:(df11|5740)' || note "no Black Pill on USB right now"
fi

if [ -n "${PI:-}" ]; then
    echo "target $PI"
    if ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI" 'uname -a' 2>/dev/null | grep -q QNX; then
        ok "SSH to QNX target works ($(ssh -o BatchMode=yes "$PI" 'uname -r' 2>/dev/null))"
        ssh -o BatchMode=yes "$PI" 'ls /dev/ser* 2>/dev/null' | grep -q serusb && ok "USB serial device on target" || note "no /dev/serusb* on target (I/O node unplugged, or devc-serusb not running)"
    else
        bad "cannot SSH to $PI without a password (run: ssh-copy-id $PI)"
    fi
fi

echo
echo "$pass passed, $fail failed, $warnc warnings"
[ "$fail" -eq 0 ]
