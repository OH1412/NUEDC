#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC"
ENV_DIR="${ROOT}/.conda/envs/yolo-steel-ball"
SCRIPT="${ROOT}/H/ball_depth_tracker.py"
LOCK_FILE="/tmp/yolo-steel-ball-camera.lock"

if [[ ! -x "${ENV_DIR}/bin/python" ]]; then
    echo "错误：YOLO 环境不存在：${ENV_DIR}" >&2
    exit 2
fi

export PYTHONNOUSERSITE=1
unset PYTHONPATH

# 与原有 YOLO 相机程序共用锁，避免两个进程同时占用 D435。
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    echo "错误：已有程序正在占用钢珠相机。" >&2
    exit 4
fi

exec "${ENV_DIR}/bin/python" -u "${SCRIPT}" "$@"
