#!/usr/bin/env bash
set -euo pipefail

CONNECTION_NAME="NUEDC-H-OFFLINE"
RESTORE_CONNECTION="${1:-}"

if nmcli -t -f NAME connection show --active |
    grep -Fxq "${CONNECTION_NAME}"; then
    nmcli connection down "${CONNECTION_NAME}"
    echo "离线热点已停止。"
else
    echo "离线热点当前未运行。"
fi

if [[ -n "${RESTORE_CONNECTION}" ]]; then
    nmcli connection up "${RESTORE_CONNECTION}"
    echo "已恢复网络连接：${RESTORE_CONNECTION}"
else
    echo "如需恢复校园网：./H/stop_offline_hotspot.sh SCUNET"
fi

