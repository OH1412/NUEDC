#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC"
UNIT_SOURCE="${ROOT}/H/nuedc-h-competition.service"
UNIT_TARGET="/etc/systemd/system/nuedc-h-competition.service"
HOTSPOT_SOURCE="${ROOT}/H/nuedc-h-hotspot.service"
HOTSPOT_TARGET="/etc/systemd/system/nuedc-h-hotspot.service"

if [[ ! -f "${UNIT_SOURCE}" ]]; then
    echo "错误：找不到服务单元：${UNIT_SOURCE}" >&2
    exit 2
fi

echo "安装开机自启动服务（本次只启用，不会立即启动电机和相机）……"
sudo install -m 0644 "${UNIT_SOURCE}" "${UNIT_TARGET}"
sudo install -m 0644 "${HOTSPOT_SOURCE}" "${HOTSPOT_TARGET}"
sudo systemctl daemon-reload
sudo systemctl enable nuedc-h-hotspot.service
sudo systemctl enable nuedc-h-competition.service

echo
echo "安装完成：热点与控制服务已经分离；控制服务重启不会重启热点。"
echo "开机自启动采用完全静默模式，不向终端或journal输出运行日志。"
echo "下次开机将自动启动热点、视觉、推流和UART监听。"
echo "现在立即启动：sudo systemctl start nuedc-h-competition.service"
echo "查看状态：sudo systemctl status nuedc-h-competition.service"
echo "查看日志：journalctl -u nuedc-h-competition.service -f"
