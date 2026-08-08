#!/usr/bin/env bash
set -euo pipefail

ENV_DIR="/home/pangolin/NUEDC/.conda/envs/yolo-steel-ball"
SCRIPT="/home/pangolin/NUEDC/realsense_test.py"

export PYTHONNOUSERSITE=1
unset PYTHONPATH

exec "${ENV_DIR}/bin/python" -u "${SCRIPT}" --fps 60 "$@"
