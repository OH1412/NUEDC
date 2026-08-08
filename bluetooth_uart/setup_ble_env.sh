#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC/bluetooth_uart"
VENDOR_DIR="${ROOT}/vendor"

mkdir -p "${VENDOR_DIR}"

/usr/bin/python3 -m pip install --upgrade \
    --target "${VENDOR_DIR}" \
    -r "${ROOT}/requirements-ble.txt"

PYTHONPATH="${VENDOR_DIR}" /usr/bin/python3 - <<'PY'
import bleak
print("BLE environment ready; bleak:", getattr(bleak, "__version__", "0.22.3"))
PY
