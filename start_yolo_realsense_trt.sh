#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC"
ENGINE="/home/pangolin/Downloads/best.engine"

if [[ ! -f "${ENGINE}" ]]; then
    echo "Error: TensorRT engine not found: ${ENGINE}" >&2
    exit 2
fi

# Use librealsense's explicit RGB/BGR8 stream. This avoids accidentally
# selecting one of the D435 depth/infrared /dev/video nodes.
exec "${ROOT}/start_yolo_camera.sh" \
    --weights "${ENGINE}" \
    --source realsense \
    --device 0 \
    --imgsz 320 \
    --width 640 \
    --height 480 \
    --fps 30 \
    --conf 0.45 \
    --iou 0.35 \
    --no-cpu-fallback \
    "$@"
