#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC/bluetooth_uart"
ADDRESS="48:87:2D:73:E1:B0"
UART_CHAR="0000ffe1-0000-1000-8000-00805f9b34fb"

if [[ "${1:-}" != "--arm" ]]; then
    cat >&2 <<EOF
Safety lock: this program can send control frames that cause the car to move.

Lift the wheels or clear the test area, then run:
  $0 --arm

Additional bridge arguments may follow --arm.
EOF
    exit 2
fi
shift

echo "BT24: ${ADDRESS}"
echo "MCU <-> BT24 UART setting: 9600 8N1, no flow control"
echo "Mission: MCU forward 1 m -> PC ACK 0x10 -> MCU left 90 deg -> PC ACK 0x11 -> MCU forward 0.5 m"
echo "Safety lock released."

exec "${ROOT}/host/run_ble_xml_mission.sh" \
    --address "${ADDRESS}" \
    --write-char "${UART_CHAR}" \
    --notify-char "${UART_CHAR}" \
    "$@"
