#!/bin/bash
#
# Reflash the TNC over the Pi's serial port without touching the board.
#
#   ./flash-pi.sh [hexfile] [port]
#
# Defaults to bin/mmdvm_f4.hex on /dev/ttyAMA0.
#
# The running firmware is asked, with a magic KISS frame, to reboot into the
# STM32's factory ROM bootloader, which listens on the same USART; then
# stm32flash writes the image and starts it. No BOOT0 or reset lines are
# involved, so no unplugging and no capacitor lottery.
#
# This only works once a firmware that understands the request (2026-07-31
# or later) is on the board. For a board running older firmware, or one too
# broken to answer, the old way in still applies:
#
#   while ! sudo stm32flash -v -w bin/mmdvm_f4.hex -R \
#       -i 20,-21,21:-20,-21,21 /dev/ttyAMA0; do echo Again; done
#
# ...plugging the board into the header while the loop runs.

set -e

HEX=${1:-bin/mmdvm_f4.hex}
PORT=${2:-/dev/ttyAMA0}

if [ ! -f "$HEX" ]; then
    echo "$HEX not found; build the firmware first (make pi)" >&2
    exit 1
fi

if ! command -v stm32flash >/dev/null 2>&1; then
    echo "stm32flash is not installed (apt install stm32flash)" >&2
    exit 1
fi

if ! [ -w "$PORT" ]; then
    echo "$PORT is not writable; run with sudo, or free it if kissattach or" >&2
    echo "MMDVMHost has it open" >&2
    exit 1
fi

# The magic KISS frame: FEND, type 0x0B, "BOOT", FEND.
stty -F "$PORT" 115200 raw -echo -crtscts
printf '\300\013BOOT\300' > "$PORT"
echo "Asked the firmware to reboot into the bootloader..."
sleep 1

# The ROM autobauds on stm32flash's first byte; -g 0x0 starts the new
# firmware afterwards.
stm32flash -v -w "$HEX" -g 0x0 "$PORT"
