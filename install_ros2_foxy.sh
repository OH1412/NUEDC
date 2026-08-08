#!/usr/bin/env bash
set -euo pipefail

if [[ "$(lsb_release -sc)" != "focal" ]]; then
    echo "Error: this installer is only for Ubuntu 20.04 (focal)." >&2
    exit 2
fi

# The USTC/TUNA/BFSU ROS 2 mirrors currently publish stale Foxy indexes whose
# referenced arm64 .deb files return 404. Keep the working ROS 1 source, disable
# only stale ROS 2 mirror entries, and use the signed upstream repository.
for source_file in /etc/apt/sources.list.d/*.list; do
    [[ -f "${source_file}" ]] || continue
    if grep -qE '^[[:space:]]*deb .*mirrors\\..*/ros2/ubuntu' "${source_file}"; then
        sudo sed -i -E \
            '\|^[[:space:]]*deb .*mirrors\\..*/ros2/ubuntu| s|^[[:space:]]*deb |# disabled-stale-ros2-mirror deb |' \
            "${source_file}"
    fi
done

echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu focal main" \
    | sudo tee /etc/apt/sources.list.d/ros2-official.list >/dev/null

sudo apt-get clean
sudo apt-get update
sudo apt-get install -y \
    ros-foxy-ros-base \
    ros-foxy-rmw-fastrtps-cpp \
    ros-foxy-image-tools \
    ros-foxy-image-transport-plugins \
    ros-foxy-tf2-tools \
    ros-foxy-rqt-image-view \
    ros-foxy-rviz2 \
    ros-foxy-xacro \
    ros-foxy-diagnostic-updater \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    build-essential \
    cmake \
    git

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update --include-eol-distros

source /home/pangolin/NUEDC/ros2_env.sh
cd /home/pangolin/NUEDC/ros2_ws

rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro foxy \
    --skip-keys librealsense2 \
    -r -y

colcon build \
    --symlink-install \
    --cmake-clean-cache \
    --executor sequential \
    --packages-up-to realsense2_camera \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DPYTHON_EXECUTABLE=/usr/bin/python3

echo
echo "ROS 2 Foxy and the RealSense ROS wrapper installation completed."
echo "Activate it with: source /home/pangolin/NUEDC/ros2_env.sh"
