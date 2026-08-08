#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC"
PYTHON="${ROOT}/.conda/envs/yolo-steel-ball/bin/python"
LOCK_FILE="/tmp/yolo-steel-ball-camera.lock"
SERVICE_ENV="${ROOT}/H/competition.env"
MODE5_EQUILIBRIUM_FILE="${ROOT}/H/mode5_equilibrium_points.json"

if [[ ! -x "${PYTHON}" ]]; then
    echo "错误：视觉环境不存在：${PYTHON}" >&2
    exit 2
fi

export PYTHONNOUSERSITE=1
unset PYTHONPATH

if [[ -f "${SERVICE_ENV}" ]]; then
    # 该文件只允许简单的KEY=VALUE比赛配置，由项目所有者维护。
    source "${SERVICE_ENV}"
fi
STREAM_HOST="${COMPETITION_STREAM_HOST:-192.168.50.199}"
STREAM_PORT="${COMPETITION_STREAM_PORT:-5600}"
STREAM_FPS="${COMPETITION_STREAM_FPS:-20}"
STREAM_BITRATE="${COMPETITION_STREAM_BITRATE:-1200000}"

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    echo "错误：已有程序正在占用RealSense相机。请先关闭旧视觉程序。" >&2
    exit 4
fi

exec "${PYTHON}" -u "${ROOT}/H/competition_runtime.py" \
    --stream-host "${STREAM_HOST}" \
    --stream-port "${STREAM_PORT}" \
    --stream-fps "${STREAM_FPS}" \
    --stream-bitrate "${STREAM_BITRATE}" \
    --mode5-equilibrium-file "${MODE5_EQUILIBRIUM_FILE}" \
    "$@"
