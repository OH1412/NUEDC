#!/usr/bin/env bash
set -euo pipefail

# Jetson离线热点：不依赖路由器、校园网或互联网。
INTERFACE="wlan0"
CONNECTION_NAME="NUEDC-H-OFFLINE"
SSID="${1:-NUEDC-H}"
PASSWORD="${2:-NUEDC2026}"
JETSON_ADDRESS="192.168.50.1/24"

if [[ ${#PASSWORD} -lt 8 || ${#PASSWORD} -gt 63 ]]; then
    echo "错误：热点密码长度必须为8～63个字符。" >&2
    exit 2
fi

if ! nmcli -t -f DEVICE,TYPE device status |
    grep -q "^${INTERFACE}:wifi"; then
    echo "错误：未找到无线网卡 ${INTERFACE}。" >&2
    exit 3
fi

if nmcli -t -f NAME connection show | grep -Fxq "${CONNECTION_NAME}"; then
    nmcli connection modify "${CONNECTION_NAME}" \
        connection.interface-name "${INTERFACE}" \
        connection.autoconnect no \
        802-11-wireless.ssid "${SSID}" \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        802-11-wireless-security.key-mgmt wpa-psk \
        802-11-wireless-security.psk "${PASSWORD}" \
        ipv4.method shared \
        ipv4.addresses "${JETSON_ADDRESS}" \
        ipv6.method disabled
else
    nmcli connection add \
        type wifi \
        ifname "${INTERFACE}" \
        con-name "${CONNECTION_NAME}" \
        autoconnect no \
        ssid "${SSID}"
    nmcli connection modify "${CONNECTION_NAME}" \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        802-11-wireless-security.key-mgmt wpa-psk \
        802-11-wireless-security.psk "${PASSWORD}" \
        ipv4.method shared \
        ipv4.addresses "${JETSON_ADDRESS}" \
        ipv6.method disabled
fi

echo "即将断开当前Wi-Fi并启动离线热点：${SSID}" >&2
nmcli connection up "${CONNECTION_NAME}"

echo
echo "热点已启动"
echo "  SSID：${SSID}"
echo "  密码：${PASSWORD}"
echo "  Jetson固定地址：192.168.50.1"
echo
echo "PC连接热点后，用 ipconfig（Windows）或 ip addr（Linux）查看PC地址。"
echo "再将该地址传给视觉程序的 --stream-host。"
echo "查看已连接设备：ip neigh show dev ${INTERFACE}"

