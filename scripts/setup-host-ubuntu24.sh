#!/usr/bin/env bash
# Reflex: one-time setup of an Ubuntu 24.04 LTS development host.
#
#   ./scripts/setup-host-ubuntu24.sh
#
# Installs everything Reflex needs EXCEPT the QNX SDP itself (that needs your myQNX
# login and goes through the QNX Software Center, see docs/GUIDE.md part 1).
# Safe to run again. Asks for your sudo password.
set -euo pipefail

ARDUINO_CLI_VERSION="${ARDUINO_CLI_VERSION:-1.1.1}"
STM32_INDEX="https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json"

say()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33mWARNING: %s\033[0m\n' "$*"; }

. /etc/os-release
if [ "${ID:-}" != "ubuntu" ] || [ "${VERSION_ID:-}" != "24.04" ]; then
    warn "this script is written for Ubuntu 24.04, you have ${PRETTY_NAME:-unknown}. Continuing anyway."
fi
if [ "$(uname -m)" != "x86_64" ]; then
    warn "QNX SDP only runs on x86_64 hosts. You have $(uname -m)."
fi

say "1/6 system packages"
# python3-numpy comes from apt: on 24.04 'pip install' outside a venv is blocked (PEP 668).
# libfuse2t64 is the 24.04 name of libfuse2 (needed only if you also want the Arduino IDE AppImage).
sudo apt-get update
sudo apt-get install -y \
    build-essential make git curl wget unzip xz-utils ca-certificates \
    python3 python3-numpy python3-matplotlib python3-venv \
    openssh-client \
    qemu-system-x86 qemu-utils bridge-utils net-tools \
    dfu-util picocom usbutils \
    rpi-imager \
    libfuse2t64

say "2/6 groups: dialout (serial ports), plugdev (USB), kvm (fast QEMU)"
for g in dialout plugdev kvm; do
    if getent group "$g" >/dev/null && ! id -nG "$USER" | grep -qw "$g"; then
        sudo usermod -aG "$g" "$USER"; NEED_RELOGIN=1
    fi
done

say "3/6 udev rules for the Black Pill (DFU flashing without sudo; keep ModemManager off the serial port)"
sudo tee /etc/udev/rules.d/50-reflex-blackpill.rules >/dev/null <<'RULES'
# STM32 ROM bootloader in DFU mode (BOOT0 held at reset)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666", TAG+="uaccess"
# STM32duino USB CDC serial: Ubuntu's ModemManager probes new ttyACM devices and corrupts the
# first seconds of traffic. Tell it to leave this one alone.
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_IGNORE}="1", MODE="0666", TAG+="uaccess"
RULES
sudo udevadm control --reload-rules
sudo udevadm trigger || true

say "4/6 arduino-cli ${ARDUINO_CLI_VERSION} into ~/.local/bin"
mkdir -p "$HOME/.local/bin"
if ! "$HOME/.local/bin/arduino-cli" version 2>/dev/null | grep -q "$ARDUINO_CLI_VERSION"; then
    tmp="$(mktemp -d)"
    curl -fsSL -o "$tmp/ac.tgz" \
        "https://github.com/arduino/arduino-cli/releases/download/v${ARDUINO_CLI_VERSION}/arduino-cli_${ARDUINO_CLI_VERSION}_Linux_64bit.tar.gz"
    tar -xzf "$tmp/ac.tgz" -C "$tmp" arduino-cli
    install -m 0755 "$tmp/arduino-cli" "$HOME/.local/bin/arduino-cli"
    rm -rf "$tmp"
fi
case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *)
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
    export PATH="$HOME/.local/bin:$PATH"; NEED_RELOGIN=1 ;;
esac
AC="$HOME/.local/bin/arduino-cli"

say "5/6 STM32 Arduino core + VL53L0X library (a few hundred MB, first time only)"
"$AC" config init --overwrite >/dev/null
"$AC" config add board_manager.additional_urls "$STM32_INDEX"
"$AC" core update-index
"$AC" core install STMicroelectronics:stm32
"$AC" lib install "VL53L0X"

say "6/6 QNX SDP check"
if [ -f "$HOME/qnx800/qnxsdp-env.sh" ]; then
    echo "found ~/qnx800/qnxsdp-env.sh"
else
    warn "QNX SDP 8.0 not found in ~/qnx800. Install it with the QNX Software Center (docs/GUIDE.md part 1).
         On Ubuntu 24.04 you need SDP 8.0.2 or newer: install the latest 8.0.x baseline."
fi

say "done"
if [ "${NEED_RELOGIN:-0}" = 1 ]; then
    warn "log out and back in (or reboot) so the new groups and PATH take effect."
fi
echo "next:  source ~/qnx800/qnxsdp-env.sh && make doctor"
