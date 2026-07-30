#!/usr/bin/env bash
# Source this file from a clean shell to use ROS 2 Foxy without mixing ROS 1.

unset ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ETC_DIR
unset ROS_MASTER_URI ROS_ROOT ROS_DISTRO
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset PYTHONHOME PYTHONPATH

# ROS 2 Foxy Debian packages are built for Ubuntu's /usr/bin/python3. A conda
# environment (including base) can otherwise make CMake select conda Python,
# where ROS generators such as empy are unavailable. Remove only conda and
# previously sourced ROS bin entries; leave the rest of the user's PATH intact.
ROS2_ENV_CONDA_PREFIX="${CONDA_PREFIX:-}"
ROS2_ENV_CLEAN_PATH=""
IFS=: read -r -a ROS2_ENV_PATH_PARTS <<< "${PATH}"
for ROS2_ENV_PATH_PART in "${ROS2_ENV_PATH_PARTS[@]}"; do
    if [[ -n "${ROS2_ENV_CONDA_PREFIX}" ]] && {
        [[ "${ROS2_ENV_PATH_PART}" == "${ROS2_ENV_CONDA_PREFIX}/bin" ]] ||
        [[ "${ROS2_ENV_PATH_PART}" == "${ROS2_ENV_CONDA_PREFIX}/condabin" ]];
    }; then
        continue
    fi
    [[ "${ROS2_ENV_PATH_PART}" == /opt/ros/*/bin ]] && continue
    if [[ -z "${ROS2_ENV_CLEAN_PATH}" ]]; then
        ROS2_ENV_CLEAN_PATH="${ROS2_ENV_PATH_PART}"
    else
        ROS2_ENV_CLEAN_PATH="${ROS2_ENV_CLEAN_PATH}:${ROS2_ENV_PATH_PART}"
    fi
done
export PATH="${ROS2_ENV_CLEAN_PATH}"
unset ROS2_ENV_CONDA_PREFIX ROS2_ENV_CLEAN_PATH
unset ROS2_ENV_PATH_PART ROS2_ENV_PATH_PARTS
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
hash -r

if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
    echo "ROS 2 Foxy is not installed. Run: /home/pangolin/NUEDC/install_ros2_foxy.sh" >&2
    return 1 2>/dev/null || exit 1
fi

# Foxy's generated setup files predate widespread `set -u` usage and read a
# few optional variables before checking whether they exist. Temporarily turn
# nounset off while sourcing, then restore the caller's shell option.
case "$-" in
    *u*) ROS2_ENV_RESTORE_NOUNSET=1; set +u ;;
    *) ROS2_ENV_RESTORE_NOUNSET=0 ;;
esac
AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

source /opt/ros/foxy/setup.bash

ROS2_WS="/home/pangolin/NUEDC/ros2_ws"
if [[ -f "${ROS2_WS}/install/local_setup.bash" ]]; then
    source "${ROS2_WS}/install/local_setup.bash"
fi

if [[ "${ROS2_ENV_RESTORE_NOUNSET}" == "1" ]]; then
    set -u
fi
unset ROS2_ENV_RESTORE_NOUNSET

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo "ROS 2 ${ROS_DISTRO} ready; Python: $(command -v python3); workspace: ${ROS2_WS}"
