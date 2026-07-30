#!/usr/bin/env python3
"""以子进程复用现有钢珠识别器，并逐行读取结构化结果。"""

import json
from pathlib import Path
import subprocess
import sys
from typing import Any, Dict, Iterator, Optional, Sequence


H_DIR = Path(__file__).resolve().parent
TRACKER = H_DIR / "ball_depth_tracker.py"


def normalize_tracker_args(arguments: Sequence[str]) -> list:
    args = list(arguments)
    if args and args[0] == "--":
        args = args[1:]
    return args


class BallTrackerSource:
    def __init__(self, tracker_args: Sequence[str] = ()) -> None:
        extra = normalize_tracker_args(tracker_args)
        # 放在最后，保证控制/标定程序总能收到逐帧JSON。
        self.command = [
            sys.executable,
            "-u",
            str(TRACKER),
            *extra,
            "--print-every",
            "1",
        ]
        self.process: Optional[subprocess.Popen] = None

    def start(self) -> None:
        if self.process is not None:
            raise RuntimeError("钢珠识别子进程已经启动。")
        self.process = subprocess.Popen(
            self.command,
            stdout=subprocess.PIPE,
            stderr=None,
            text=True,
            bufsize=1,
        )

    def records(self) -> Iterator[Dict[str, Any]]:
        if self.process is None or self.process.stdout is None:
            raise RuntimeError("钢珠识别子进程尚未启动。")
        for line in self.process.stdout:
            text = line.strip()
            if not text:
                continue
            try:
                record = json.loads(text)
            except json.JSONDecodeError:
                print(
                    "忽略识别器非JSON输出：{}".format(text[:160]),
                    file=sys.stderr,
                )
                continue
            if isinstance(record, dict):
                yield record

    def wait(self, timeout: float = 1.0) -> int:
        """等待识别器退出并返回真实退出码。"""

        if self.process is None:
            raise RuntimeError("钢珠识别子进程尚未启动。")
        return int(self.process.wait(timeout=max(0.0, float(timeout))))

    def poll(self) -> Optional[int]:
        """返回识别器退出码；仍在运行时返回None。"""

        if self.process is None:
            raise RuntimeError("钢珠识别子进程尚未启动。")
        result = self.process.poll()
        return None if result is None else int(result)

    def close(self) -> None:
        process = self.process
        self.process = None
        if process is None:
            return
        if process.stdout is not None:
            process.stdout.close()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)

    def __enter__(self) -> "BallTrackerSource":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def base_point_from_record(record: Dict[str, Any]) -> Optional[list]:
    if not record.get("valid"):
        return None
    point = record.get("camera_base_point_m")
    if not isinstance(point, dict):
        return None
    try:
        return [float(point["x"]), float(point["y"]), float(point["z"])]
    except (KeyError, TypeError, ValueError):
        return None
