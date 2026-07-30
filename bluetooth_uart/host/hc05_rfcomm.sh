#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  hc05_rfcomm.sh scan [seconds]
  hc05_rfcomm.sh pair MAC
  hc05_rfcomm.sh info MAC
  hc05_rfcomm.sh channels MAC
  hc05_rfcomm.sh bind MAC [channel]
  hc05_rfcomm.sh release
EOF
}

valid_mac() {
    [[ "$1" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]]
}

command_name="${1:-}"
case "${command_name}" in
    scan)
        seconds="${2:-15}"
        bluetoothctl power on >/dev/null
        bluetoothctl --timeout "${seconds}" scan on
        ;;
    pair)
        mac="${2:-}"
        valid_mac "${mac}" || { echo "Error: invalid MAC: ${mac}" >&2; exit 2; }
        bluetoothctl power on >/dev/null
        bluetoothctl --agent KeyboardDisplay --timeout 40 pair "${mac}"
        bluetoothctl trust "${mac}"
        bluetoothctl info "${mac}"
        ;;
    info)
        mac="${2:-}"
        valid_mac "${mac}" || { echo "Error: invalid MAC: ${mac}" >&2; exit 2; }
        bluetoothctl info "${mac}"
        ;;
    channels)
        mac="${2:-}"
        valid_mac "${mac}" || { echo "Error: invalid MAC: ${mac}" >&2; exit 2; }
        sdptool browse "${mac}" | sed -n '/Service Name: Serial Port/,+12p'
        ;;
    bind)
        mac="${2:-}"
        channel="${3:-1}"
        valid_mac "${mac}" || { echo "Error: invalid MAC: ${mac}" >&2; exit 2; }
        [[ "${channel}" =~ ^[0-9]+$ ]] || {
            echo "Error: channel must be an integer" >&2
            exit 2
        }
        sudo rfcomm release 0 >/dev/null 2>&1 || true
        sudo rfcomm bind 0 "${mac}" "${channel}"
        echo "Bound ${mac} channel ${channel} as /dev/rfcomm0"
        echo "Open /dev/rfcomm0 to establish the SPP connection."
        ;;
    release)
        sudo rfcomm release 0
        ;;
    *)
        usage
        exit 2
        ;;
esac
