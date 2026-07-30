#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC/mission_bt"
exec /usr/bin/python3 -u "${ROOT}/run_mission.py" \
    --transport mock \
    "$@"
