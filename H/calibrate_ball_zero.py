#!/usr/bin/env python3
"""把球心放在管道零位，稳健估计其底座坐标并写入控制配置。"""

import argparse
import json
from pathlib import Path
import queue
import sys
import threading
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np

from ball_tracker_source import BallTrackerSource, base_point_from_record


H_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = H_DIR / "ball_control_config.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="标定管道零位对应的钢珠球心底座坐标"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--samples", type=int, default=50)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument(
        "--startup-timeout-s",
        type=float,
        default=60.0,
        help="等待YOLO产生第一条识别结果的最长时间，默认60秒",
    )
    parser.add_argument("--no-prompt", action="store_true")
    parser.add_argument(
        "tracker_args",
        nargs=argparse.REMAINDER,
        help="写在 -- 后，原样传给ball_depth_tracker.py",
    )
    return parser.parse_args()


def robust_zero_point(points: np.ndarray) -> tuple:
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) < 10:
        raise ValueError("至少需要10个三维球心样本。")
    center = np.median(values, axis=0)
    distances = np.linalg.norm(values - center, axis=1)
    median_distance = float(np.median(distances))
    mad = float(np.median(np.abs(distances - median_distance)))
    threshold = max(0.003, median_distance + 4.0 * max(mad, 1e-6))
    inliers = values[distances <= threshold]
    if len(inliers) < max(10, int(0.7 * len(values))):
        raise ValueError("零点样本离散过大，请固定钢珠并重新标定。")
    zero = np.median(inliers, axis=0)
    residuals = np.linalg.norm(inliers - zero, axis=1)
    return zero, residuals, len(inliers)


def update_zero_config(path: Path, zero: np.ndarray) -> None:
    data: Dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    data["zero_point_base_m"] = [
        round(float(value), 6) for value in zero
    ]
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def tracker_failure_message(exit_code: Optional[int]) -> str:
    if exit_code is None:
        return "识别器在启动超时内没有输出任何帧。"
    if exit_code < 0:
        signal_number = -exit_code
        detail = "识别子进程被信号{}终止".format(signal_number)
        if signal_number == 9:
            detail += "（通常是Jetson内存不足，被系统强制杀死）"
        return detail + "。"
    return "识别子进程提前退出，退出码为{}。".format(exit_code)


def collect_points(
    source: BallTrackerSource,
    sample_count: int,
    startup_timeout_s: float,
    sample_timeout_s: float,
) -> Tuple[list, bool]:
    """在独立读线程中读取JSON，使启动和采样超时都能真正生效。"""

    records: "queue.Queue[object]" = queue.Queue()
    finished = object()

    def read_records() -> None:
        try:
            for record in source.records():
                records.put(record)
        except (OSError, ValueError):
            # 主线程超时关闭stdout时，读线程可能得到已关闭文件错误。
            pass
        finally:
            records.put(finished)

    reader = threading.Thread(
        target=read_records,
        name="zero-calibration-tracker-reader",
        daemon=True,
    )
    reader.start()
    points = []
    first_record_received = False
    startup_deadline = time.monotonic() + startup_timeout_s
    sample_deadline: Optional[float] = None

    while len(points) < sample_count:
        deadline = (
            sample_deadline
            if sample_deadline is not None
            else startup_deadline
        )
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            item = records.get(timeout=remaining)
        except queue.Empty:
            break
        if item is finished:
            break
        if not first_record_received:
            first_record_received = True
            sample_deadline = time.monotonic() + sample_timeout_s
        point = base_point_from_record(item)  # type: ignore[arg-type]
        if point is not None:
            points.append(point)
            print(
                "零点有效样本：{}/{}".format(len(points), sample_count),
                end="\r",
                file=sys.stderr,
                flush=True,
            )
    return points, first_record_received


def main() -> int:
    args = parse_args()
    if (
        args.samples < 10
        or args.timeout_s <= 0
        or args.startup_timeout_s <= 0
    ):
        print(
            "配置错误：samples至少10，timeout-s和startup-timeout-s必须为正。",
            file=sys.stderr,
        )
        return 2
    config_path = args.config.expanduser().resolve()
    if not config_path.is_file():
        print("配置错误：配置文件不存在：{}".format(config_path), file=sys.stderr)
        return 2

    source = BallTrackerSource(args.tracker_args)
    try:
        if not args.no_prompt:
            input(
                "请把钢珠球心固定在你定义的0 cm位置，保持不动后按Enter开始采样："
            )
        source.start()
        print(
            "开始采集零点，共需{}个有效样本……".format(args.samples),
            file=sys.stderr,
        )
        points, first_record_received = collect_points(
            source,
            sample_count=args.samples,
            startup_timeout_s=args.startup_timeout_s,
            sample_timeout_s=args.timeout_s,
        )
        print(file=sys.stderr)
        if len(points) < 10:
            if not first_record_received:
                raise RuntimeError(tracker_failure_message(source.poll()))
            raise ValueError(
                "识别器已经运行，但采样超时前仅取得{}个有效球心样本；"
                "请检查钢珠检测框和深度是否有效。".format(len(points))
            )
        zero, residuals, inlier_count = robust_zero_point(
            np.asarray(points)
        )
        update_zero_config(config_path, zero)
        print("零点标定完成：")
        print(
            json.dumps(
                {
                    "zero_point_base_m": [
                        round(float(value), 6) for value in zero
                    ],
                    "inlier_samples": inlier_count,
                    "sample_count": len(points),
                    "rms_spread_mm": round(
                        float(np.sqrt(np.mean(residuals ** 2))) * 1000.0,
                        3,
                    ),
                    "max_spread_mm": round(
                        float(np.max(residuals)) * 1000.0, 3
                    ),
                    "updated_config": str(config_path),
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0
    except (RuntimeError, ValueError, OSError, json.JSONDecodeError) as error:
        print("零点标定失败：{}".format(error), file=sys.stderr)
        return 3
    except (KeyboardInterrupt, EOFError):
        return 130
    finally:
        source.close()


if __name__ == "__main__":
    sys.exit(main())
