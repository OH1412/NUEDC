#!/usr/bin/env bash
set -euo pipefail

# 兼容简写名称；默认推送到192.168.50.43:5600，额外参数可覆盖默认值。
exec /home/pangolin/NUEDC/H/start_camera_video_stream.sh "$@"

