#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC"
ENGINE="/home/pangolin/Downloads/best.engine"

if [[ ! -f "${ENGINE}" ]]; then
    echo "Error: TensorRT engine not found: ${ENGINE}" >&2
    exit 2
fi

# best.engine is a fixed-shape 320x320 FP16 engine built specifically for
# this Jetson Xavier and its installed TensorRT 8.5.2 runtime.
exec "${ROOT}/start_yolo_camera.sh" \
    --weights "${ENGINE}" \
    --source auto \
    --device 0 \
    --imgsz 320 \
    --conf 0.45 \
    --iou 0.35 \
    --no-cpu-fallback \
    "$@"
