#!/usr/bin/env bash
set -u

ROOT="/home/pangolin/NUEDC"
SERVICE_ENV="${ROOT}/H/competition.env"
if [[ -f "${SERVICE_ENV}" ]]; then
    source "${SERVICE_ENV}"
fi
STREAM_HOST="${COMPETITION_STREAM_HOST:-192.168.50.199}"
STREAM_PORT="${COMPETITION_STREAM_PORT:-5600}"
STREAM_FPS="${COMPETITION_STREAM_FPS:-20}"
STREAM_BITRATE="${COMPETITION_STREAM_BITRATE:-1200000}"

show_status() {
    clear 2>/dev/null || true
    echo "========== NUEDC-H 比赛状态 $(date '+%H:%M:%S') =========="
    echo "配置的PC接收地址：${STREAM_HOST}:${STREAM_PORT}"
    echo "视频参数：640x480，${STREAM_FPS} FPS，${STREAM_BITRATE} bit/s"
    echo "PC接收端窗口显示的地址必须与上面完全相同。"
    echo
    HOTSPOT_UNIT="$(systemctl is-active nuedc-h-hotspot.service 2>/dev/null || true)"
    COMPETITION_UNIT="$(systemctl is-active nuedc-h-competition.service 2>/dev/null || true)"
    WLAN_CONNECTION="$(nmcli -g GENERAL.CONNECTION device show wlan0 2>/dev/null || true)"
    WLAN_ADDRESS="$(ip -o -4 address show dev wlan0 2>/dev/null | awk '{print $4}' | head -1)"
    echo "热点服务：${HOTSPOT_UNIT:-unknown}；wlan0连接：${WLAN_CONNECTION:-无}；地址：${WLAN_ADDRESS:-无}"
    if [[ "${WLAN_CONNECTION}" != "NUEDC-H-OFFLINE" || "${WLAN_ADDRESS}" != "192.168.50.1/24" ]]; then
        echo "  [异常] wlan0现在不是NUEDC-H热点，即使systemd显示active也无法推流。"
    fi
    echo "比赛服务：${COMPETITION_UNIT:-unknown}"
    echo
    echo "当前热点客户端/邻居："
    awk 'NR > 1 && $1 ~ /^192\.168\.50\./ {printf "  IP=%s  MAC=%s  网卡=%s\n", $1, $4, $6}' /proc/net/arp
    if ! awk 'NR > 1 && $1 ~ /^192\.168\.50\./ {found=1} END {exit !found}' /proc/net/arp; then
        echo "  暂未发现（让PC保持连接并运行接收器；也可能尚未产生网络通信）"
    fi
    echo
    echo "最近运行信息："
    journalctl -u nuedc-h-competition.service -n 35 --no-pager 2>/dev/null | \
        grep -E '状态：|YOLO 权重|RealSense：|视频推流：|H\.264|推流错误|视觉自检|UART2|警告：|运行错误' | tail -18
    echo
    echo "持续查看：./H/competition_status.sh --watch    退出：Ctrl+C"
}

if [[ "${1:-}" == "--watch" ]]; then
    while true; do
        show_status
        sleep 2
    done
else
    show_status
fi
