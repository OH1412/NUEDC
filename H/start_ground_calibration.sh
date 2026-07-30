#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC"
ENV_DIR="${ROOT}/.conda/envs/yolo-steel-ball"
SCRIPT="${ROOT}/H/calibrate_camera_ground.py"
LOCK_FILE="/tmp/yolo-steel-ball-camera.lock"

if [[ ! -x "${ENV_DIR}/bin/python" ]]; then
    echo "错误：视觉环境不存在：${ENV_DIR}" >&2
    exit 2
fi

export PYTHONNOUSERSITE=1
unset PYTHONPATH

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    echo "错误：已有程序正在占用 RealSense 相机，请先停止识别、拍照或推流。" >&2
    exit 4
fi

exec "${ENV_DIR}/bin/python" -u "${SCRIPT}" "$@"

