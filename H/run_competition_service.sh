#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC"

echo "[1/2] 启动NUEDC-H离线热点……" >&2
"${ROOT}/H/start_offline_hotspot.sh"

echo "[2/2] 初始化YOLO、RealSense、视频推流和UART2比赛服务……" >&2
exec "${ROOT}/H/run_competition_core.sh" "$@"
