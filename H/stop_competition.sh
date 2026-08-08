#!/usr/bin/env bash
set -euo pipefail

PID_FILE="/tmp/nuedc-competition.pid"

if [[ ! -f "${PID_FILE}" ]]; then
    echo "比赛服务没有PID记录，可能尚未启动。"
    exit 0
fi

SERVICE_PID="$(tr -d '[:space:]' < "${PID_FILE}")"
if [[ ! "${SERVICE_PID}" =~ ^[0-9]+$ ]]; then
    echo "错误：PID文件内容无效：${SERVICE_PID}" >&2
    exit 2
fi

if ! kill -0 "${SERVICE_PID}" 2>/dev/null; then
    rm -f "${PID_FILE}"
    echo "比赛服务已经退出，已清理旧PID记录。"
    exit 0
fi

SERVICE_COMMAND="$(ps -p "${SERVICE_PID}" -o args= 2>/dev/null || true)"
if [[ "${SERVICE_COMMAND}" != *"competition_runtime.py"* && "${SERVICE_COMMAND}" != *"run_competition_service.sh"* ]]; then
    echo "错误：PID=${SERVICE_PID}不是比赛服务，拒绝发送停止信号。" >&2
    echo "实际进程：${SERVICE_COMMAND}" >&2
    exit 4
fi

echo "正在安全停止比赛服务PID=${SERVICE_PID}……"
kill -TERM "${SERVICE_PID}"
for _attempt in $(seq 1 50); do
    if ! kill -0 "${SERVICE_PID}" 2>/dev/null; then
        rm -f "${PID_FILE}"
        echo "比赛服务已停止；活动控制任务已结束并发送0位。"
        exit 0
    fi
    sleep 0.1
done

echo "警告：服务在5秒内没有退出，请查看进程状态：ps -p ${SERVICE_PID}" >&2
exit 3
