#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC/bluetooth_uart"
VENDOR_DIR="${ROOT}/vendor"

if [[ ! -d "${VENDOR_DIR}/bleak" ]]; then
    echo "BLE environment is missing. Run: ${ROOT}/setup_ble_env.sh" >&2
    exit 2
fi

export PYTHONPATH="${VENDOR_DIR}"
exec /usr/bin/python3 -u "${ROOT}/host/ble_tool.py" "$@"
