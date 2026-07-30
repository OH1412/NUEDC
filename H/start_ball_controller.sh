#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC"
PYTHON="${ROOT}/.conda/envs/yolo-steel-ball/bin/python"
LOCK_FILE="/tmp/yolo-steel-ball-camera.lock"

if [[ ! -x "${PYTHON}" ]]; then
    echo "错误：视觉环境不存在：${PYTHON}" >&2
    exit 2
fi

export PYTHONNOUSERSITE=1
unset PYTHONPATH

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    echo "错误：已有程序正在占用 RealSense 相机。" >&2
    exit 4
fi

exec "${PYTHON}" -u "${ROOT}/H/ball_control_runtime.py" "$@"
