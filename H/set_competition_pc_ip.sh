#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC"
CONFIG="${ROOT}/H/competition.env"
PC_IP="${1:-}"

if [[ ! "${PC_IP}" =~ ^192\.168\.50\.([2-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-4])$ ]]; then
    echo "用法：$0 PC在NUEDC-H热点中的地址" >&2
    echo "示例：$0 192.168.50.115" >&2
    exit 2
fi

sed -i "s/^COMPETITION_STREAM_HOST=.*/COMPETITION_STREAM_HOST=${PC_IP}/" "${CONFIG}"
echo "已把比赛视频目标改为 ${PC_IP}:5600。"
echo "热点不会重启；现在只重启比赛进程使地址生效："
echo "  sudo systemctl restart nuedc-h-competition.service"
