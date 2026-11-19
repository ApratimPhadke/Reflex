#!/usr/bin/env bash
# Reflex I/O node firmware: build and flash with arduino-cli + dfu-util (no IDE, no STM32CubeProgrammer).
#
#   scripts/fw.sh build            compile firmware/reflex_io
#   scripts/fw.sh flash            flash over USB in DFU mode (hold BOOT0, tap NRST, release BOOT0 first)
#   scripts/fw.sh monitor [dev]    serial console at 115200 (Ctrl-A Ctrl-X to quit)
#   scripts/fw.sh options          list the board's menu options (if the FQBN below needs fixing)
#
# BOARD=BLACKPILL_F401CC | BLACKPILL_F401CE | BLACKPILL_F411CE   (read the chip; default F401CC,
#       the board this project is built with)
# EMULATE=1  build the plant emulator instead (no sensor, no servo; same protocol and watchdog)
set -euo pipefail
cd "$(dirname "$0")/.."
AC="$(command -v arduino-cli || echo "$HOME/.local/bin/arduino-cli")"
BOARD="${BOARD:-BLACKPILL_F401CC}"
EMULATE="${EMULATE:-0}"
FQBN="${FQBN:-STMicroelectronics:stm32:GenF4:pnum=${BOARD},usb=CDCgen,xserial=generic}"
if [ "$EMULATE" = 1 ]; then OUT=build/firmware-emulate; else OUT=build/firmware; fi
case "${1:-build}" in
build)
    mkdir -p "$OUT"
    "$AC" compile --fqbn "$FQBN" --output-dir "$OUT" --warnings default \
        --build-property "compiler.cpp.extra_flags=-DREFLEX_EMULATE=$EMULATE" firmware/reflex_io
    echo "$BOARD EMULATE=$EMULATE" > "$OUT/variant.txt"
    ls -l "$OUT"/reflex_io.ino.bin
    ;;
flash)
    [ -f "$OUT/reflex_io.ino.bin" ] || "$0" build
    if ! lsusb | grep -qi '0483:df11'; then
        echo "Black Pill is not in DFU mode. Hold BOOT0, tap NRST, release BOOT0, then run this again." >&2; exit 1
    fi
    dfu-util -a 0 -s 0x08000000:leave -D "$OUT/reflex_io.ino.bin"
    echo "flashed. If it does not start by itself, tap NRST."
    ;;
monitor)
    dev="${2:-$(ls /dev/ttyACM* 2>/dev/null | head -1)}"
    [ -n "$dev" ] || { echo "no /dev/ttyACM* found" >&2; exit 1; }
    picocom -b 115200 --imap lfcrlf --omap crlf "$dev"
    ;;
options)
    "$AC" board details -b STMicroelectronics:stm32:GenF4
    ;;
*) sed -n '2,10p' "$0"; exit 2 ;;
esac
