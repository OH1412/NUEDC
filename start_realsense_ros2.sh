#!/usr/bin/env bash
set -euo pipefail

source /home/pangolin/NUEDC/ros2_env.sh

exec ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    serial_no:=_254522076117 \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=640x480x30 \
    enable_color:=true \
    enable_depth:=true \
    enable_sync:=false \
    align_depth.enable:=false \
    pointcloud.enable:=false \
    "$@"
