#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC/bluetooth_uart"
ADDRESS="48:87:2D:73:E1:B0"
UART_CHAR="0000ffe1-0000-1000-8000-00805f9b34fb"

if (( $# > 1 )); then
    echo "Usage: $0 [legacy-seconds-ignored]" >&2
    exit 2
fi

if (( $# == 1 )); then
    if ! [[ "$1" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "Usage: $0 [legacy-seconds-ignored]" >&2
        exit 2
    fi
    echo "Note: the old ${1}-second limit is ignored; listening is now continuous."
fi

echo "BT24: ${ADDRESS}"
echo "MCU <-> BT24 UART setting: 9600 8N1, no flow control"
echo "Mode: listen only (no data will be sent to the MCU)"
echo "Incoming bytes are printed as hexadecimal. Press Ctrl+C to stop."

exec "${ROOT}/host/ble_tool.sh" listen \
    "${ADDRESS}" \
    --notify-char "${UART_CHAR}" \
    --reconnect-delay 2
