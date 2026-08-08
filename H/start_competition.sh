#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/pangolin/NUEDC"
RUNNER="${ROOT}/H/run_competition_service.sh"
LOG_FILE="${ROOT}/H/output/competition.log"
PID_FILE="/tmp/nuedc-competition.pid"

if [[ "${1:-}" == "--foreground" ]]; then
    shift
    exec "${RUNNER}" "$@"
fi

if [[ -f "${PID_FILE}" ]]; then
    EXISTING_PID="$(tr -d '[:space:]' < "${PID_FILE}")"
    if [[ "${EXISTING_PID}" =~ ^[0-9]+$ ]] && kill -0 "${EXISTING_PID}" 2>/dev/null; then
        EXISTING_COMMAND="$(ps -p "${EXISTING_PID}" -o args= 2>/dev/null || true)"
        if [[ "${EXISTING_COMMAND}" == *"competition_runtime.py"* || "${EXISTING_COMMAND}" == *"run_competition_service.sh"* ]]; then
            echo "比赛服务已经运行，PID=${EXISTING_PID}" >&2
            echo "查看日志：tail -f ${LOG_FILE}" >&2
            exit 0
        fi
    fi
fi

mkdir -p "$(dirname "${LOG_FILE}")"
nohup "${RUNNER}" "$@" >>"${LOG_FILE}" 2>&1 </dev/null &
SERVICE_PID=$!
echo "${SERVICE_PID}" >"${PID_FILE}"

sleep 1
if ! kill -0 "${SERVICE_PID}" 2>/dev/null; then
    echo "比赛服务启动失败，请查看：${LOG_FILE}" >&2
    exit 1
fi

echo "比赛服务已在后台启动，PID=${SERVICE_PID}。切换热点导致SSH断线也不会退出。"
echo "日志：tail -f ${LOG_FILE}"
echo "停止：./H/stop_competition.sh"
