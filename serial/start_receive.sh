#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC"
PYTHON="${ROOT}/.conda/envs/yolo-steel-ball/bin/python"
SCRIPT="${ROOT}/serial/receive.py"

if [[ ! -x "${PYTHON}" ]]; then
    echo "错误：项目Python环境不存在：${PYTHON}" >&2
    exit 2
fi

export PYTHONNOUSERSITE=1
unset PYTHONPATH

exec "${PYTHON}" -u "${SCRIPT}" "$@"
