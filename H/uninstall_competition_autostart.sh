#!/usr/bin/env bash
set -euo pipefail

UNIT_TARGET="/etc/systemd/system/nuedc-h-competition.service"
HOTSPOT_TARGET="/etc/systemd/system/nuedc-h-hotspot.service"

sudo systemctl disable --now nuedc-h-competition.service 2>/dev/null || true
sudo systemctl disable --now nuedc-h-hotspot.service 2>/dev/null || true
sudo rm -f "${UNIT_TARGET}"
sudo rm -f "${HOTSPOT_TARGET}"
sudo systemctl daemon-reload
sudo systemctl reset-failed nuedc-h-competition.service 2>/dev/null || true

echo "开机自启动服务已卸载。项目代码和参数文件均未删除。"
