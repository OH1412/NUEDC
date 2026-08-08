#!/usr/bin/env bash
set -euo pipefail

ENV_DIR="/home/pangolin/NUEDC/.conda/envs/yolo-steel-ball"
SCRIPT="/home/pangolin/NUEDC/yolo_camera.py"
LOCK_FILE="/tmp/yolo-steel-ball-camera.lock"

export PYTHONNOUSERSITE=1
unset PYTHONPATH

# Keep the lock across exec so V4L2 and librealsense launchers cannot compete
# for the same physical camera.
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    echo "Error: another YOLO camera process is already running." >&2
    echo "Stop its window with q/Esc or Ctrl+C, then try again." >&2
    exit 4
fi

exec "${ENV_DIR}/bin/python" -u "${SCRIPT}" "$@"
