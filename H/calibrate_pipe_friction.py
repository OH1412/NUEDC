#!/usr/bin/env python3
"""固定倾角滚落实验：从视觉轨迹估计净加速度和集总滚动参数。"""

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np
from scipy.signal import savgol_filter

from ball_control import GRAVITY_M_S2, ball_position_from_zero
from ball_tracker_source import BallTrackerSource, base_point_from_record


H_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = H_DIR / "ball_control_config.json"
DEFAULT_OUTPUT = H_DIR / "output"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="让钢珠在固定倾角管道上自由滚动，辨识集总滚动参数"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--angle-deg",
        type=float,
        default=10.0,
        help="实验实际倾角；正角度应使球滚向零点",
    )
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--min-speed-m-s", type=float, default=0.025)
    parser.add_argument("--min-fit-samples", type=int, default=10)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--update-model", action="store_true")
    parser.add_argument("--no-prompt", action="store_true")
    parser.add_argument(
        "tracker_args",
        nargs=argparse.REMAINDER,
        help="写在 -- 后，原样传给ball_depth_tracker.py",
    )
    return parser.parse_args()


def load_experiment_config(path: Path) -> Dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    zero = data.get("zero_point_base_m")
    if not isinstance(zero, list) or len(zero) != 3:
        raise ValueError(
            "尚未标定zero_point_base_m，请先运行"
            "./H/start_ball_zero_calibration.sh。"
        )
    return data


def longest_true_segment(mask: np.ndarray) -> np.ndarray:
    indices = np.flatnonzero(mask)
    if len(indices) == 0:
        return indices
    split_locations = np.where(np.diff(indices) > 1)[0] + 1
    groups = np.split(indices, split_locations)
    return max(groups, key=len)


def robust_quadratic_fit(
    times_s: Sequence[float], positions_m: Sequence[float]
) -> Tuple[np.ndarray, np.ndarray, float]:
    times = np.asarray(times_s, dtype=np.float64)
    positions = np.asarray(positions_m, dtype=np.float64)
    if len(times) < 6:
        raise ValueError("二次轨迹拟合至少需要6个样本。")
    centered_time = times - times[0]
    design = np.column_stack(
        [np.ones(len(times)), centered_time, 0.5 * centered_time ** 2]
    )
    mask = np.ones(len(times), dtype=bool)
    coefficients = np.zeros(3)
    for _ in range(5):
        if np.count_nonzero(mask) < 6:
            break
        coefficients, _, _, _ = np.linalg.lstsq(
            design[mask], positions[mask], rcond=None
        )
        residuals = positions - design.dot(coefficients)
        center = float(np.median(residuals[mask]))
        mad = float(np.median(np.abs(residuals[mask] - center)))
        threshold = max(0.0025, 4.0 * 1.4826 * max(mad, 1e-7))
        new_mask = np.abs(residuals - center) <= threshold
        if np.array_equal(new_mask, mask):
            break
        mask = new_mask
    residuals = positions[mask] - design[mask].dot(coefficients)
    rmse = float(np.sqrt(np.mean(residuals ** 2)))
    return coefficients, mask, rmse


def analyze_roll_down(
    times_s: Sequence[float],
    positions_m: Sequence[float],
    angle_deg: float,
    min_speed_m_s: float,
    min_fit_samples: int,
) -> Dict[str, Any]:
    times = np.asarray(times_s, dtype=np.float64)
    positions = np.asarray(positions_m, dtype=np.float64)
    if len(times) < max(min_fit_samples, 7):
        raise ValueError("有效轨迹样本不足。")
    order = np.argsort(times)
    times = times[order]
    positions = positions[order]
    unique = np.concatenate([[True], np.diff(times) > 1e-5])
    times = times[unique]
    positions = positions[unique]
    if len(times) < max(min_fit_samples, 7):
        raise ValueError("去除重复时间戳后样本不足。")

    median_dt = float(np.median(np.diff(times)))
    window = min(11, len(times) if len(times) % 2 else len(times) - 1)
    window = max(window, 5)
    smoothed = savgol_filter(
        positions, window_length=window, polyorder=2, mode="interp"
    )
    velocities = np.gradient(smoothed, times)
    expected_velocity_sign = -1.0 if angle_deg > 0 else 1.0
    away_from_ends = (positions > 0.01) & (positions < 0.24)
    moving = (
        expected_velocity_sign * velocities >= min_speed_m_s
    ) & away_from_ends
    segment = longest_true_segment(moving)
    if len(segment) < min_fit_samples:
        raise ValueError(
            "连续滚动段仅{}个样本，至少需要{}个；请提高视觉帧率、"
            "从远端静止释放并避免端部碰撞。".format(
                len(segment), min_fit_samples
            )
        )

    fit_times = times[segment]
    fit_positions = positions[segment]
    coefficients, inlier_mask, rmse_m = robust_quadratic_fit(
        fit_times, fit_positions
    )
    acceleration = float(coefficients[2])
    if expected_velocity_sign * acceleration <= 0:
        raise ValueError(
            "拟合加速度方向与倾角符号不一致；请确认+角度确实让球向零点滚。"
        )
    net_acceleration = abs(acceleration)
    sine = abs(math.sin(math.radians(angle_deg)))
    if sine < 1e-4:
        raise ValueError("实验倾角太小，无法计算角度增益。")
    empirical_gain = net_acceleration / sine
    return {
        "sample_count": int(len(times)),
        "moving_segment_samples": int(len(segment)),
        "fit_inlier_samples": int(np.count_nonzero(inlier_mask)),
        "median_sample_rate_hz": round(1.0 / median_dt, 3),
        "fit_duration_s": round(float(fit_times[-1] - fit_times[0]), 4),
        "initial_velocity_m_s": round(float(coefficients[1]), 6),
        "signed_acceleration_m_s2": round(acceleration, 6),
        "net_acceleration_m_s2": round(net_acceleration, 6),
        "empirical_acceleration_gain_m_s2": round(empirical_gain, 6),
        "fit_rmse_mm": round(rmse_m * 1000.0, 3),
        "_fit_indices": segment,
        "_smoothed_positions": smoothed,
        "_velocities": velocities,
    }


def write_raw_csv(
    path: Path,
    times: Sequence[float],
    positions: Sequence[float],
    points: Sequence[Sequence[float]],
    confidences: Sequence[float],
) -> None:
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_s",
                "position_m",
                "base_x_m",
                "base_y_m",
                "base_z_m",
                "confidence",
            ]
        )
        origin = times[0]
        for timestamp, position, point, confidence in zip(
            times, positions, points, confidences
        ):
            writer.writerow(
                [
                    "{:.9f}".format(timestamp - origin),
                    "{:.9f}".format(position),
                    "{:.9f}".format(point[0]),
                    "{:.9f}".format(point[1]),
                    "{:.9f}".format(point[2]),
                    "{:.6f}".format(confidence),
                ]
            )


def update_motion_gain(
    path: Path, acceleration_gain_m_s2: float
) -> None:
    data = json.loads(path.read_text(encoding="utf-8"))
    data["motion_model"]["acceleration_gain_m_s2"] = round(
        float(acceleration_gain_m_s2), 9
    )
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main() -> int:
    args = parse_args()
    if (
        not 0 < abs(args.angle_deg) <= 30
        or args.duration_s <= 0
        or args.min_speed_m_s <= 0
        or args.min_fit_samples < 6
    ):
        print("配置错误：请检查倾角、时长、速度和样本数。", file=sys.stderr)
        return 2
    config_path = args.config.expanduser().resolve()
    try:
        config = load_experiment_config(config_path)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print("配置错误：{}".format(error), file=sys.stderr)
        return 2
    zero = config["zero_point_base_m"]
    pipe_length = float(config["pipe_length_m"])
    mass_kg = float(config["ball_mass_kg"])
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    source = BallTrackerSource(args.tracker_args)
    times: List[float] = []
    positions: List[float] = []
    points: List[List[float]] = []
    confidences: List[float] = []
    try:
        source.start()
        records = source.records()
        print("等待钢珠识别器给出首个有效位置……", file=sys.stderr)
        ready = False
        for record in records:
            if base_point_from_record(record) is not None:
                ready = True
                break
        if not ready:
            raise ValueError("识别器退出前没有取得有效钢珠位置。")
        if not args.no_prompt:
            input(
                "把管道稳定在{:+.2f}°，将球静止放在高端；"
                "按Enter后立即无初速度释放：".format(args.angle_deg)
            )
        deadline = time.monotonic() + args.duration_s
        for record in records:
            point = base_point_from_record(record)
            if point is not None:
                try:
                    position = ball_position_from_zero(
                        point, zero, pipe_length, tolerance_m=0.04
                    )
                except ValueError:
                    continue
                timestamp = float(
                    record.get(
                        "capture_monotonic_ms",
                        record.get("timestamp_ms", 0.0),
                    )
                ) / 1000.0
                if math.isfinite(timestamp):
                    times.append(timestamp)
                    positions.append(position)
                    points.append(point)
                    confidences.append(float(record.get("confidence", 0.0)))
            if time.monotonic() >= deadline:
                break
        if len(times) < args.min_fit_samples:
            raise ValueError(
                "只取得{}个有效位置样本。".format(len(times))
            )
        result = analyze_roll_down(
            times,
            positions,
            args.angle_deg,
            args.min_speed_m_s,
            args.min_fit_samples,
        )
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = output_dir / "friction_{}_{:+.1f}deg.csv".format(
            stamp, args.angle_deg
        )
        write_raw_csv(csv_path, times, positions, points, confidences)

        net_accel = float(result["net_acceleration_m_s2"])
        downhill_point_mass = GRAVITY_M_S2 * abs(
            math.sin(math.radians(args.angle_deg))
        )
        downhill_solid_sphere = (
            5.0 / 7.0 * GRAVITY_M_S2
            * abs(math.sin(math.radians(args.angle_deg)))
        )
        result.update(
            {
                "angle_deg": args.angle_deg,
                "ball_mass_kg": mass_kg,
                "point_mass_equivalent_disturbance_accel_m_s2": round(
                    downhill_point_mass - net_accel, 6
                ),
                "point_mass_equivalent_disturbance_force_N": round(
                    mass_kg * (downhill_point_mass - net_accel), 8
                ),
                "solid_sphere_residual_loss_accel_m_s2": round(
                    downhill_solid_sphere - net_accel, 6
                ),
                "solid_sphere_residual_loss_force_N": round(
                    mass_kg * (downhill_solid_sphere - net_accel), 8
                ),
                "raw_csv": str(csv_path),
                "model_updated": bool(args.update_model),
                "identifiability_note": (
                    "单次固定角单方向实验只能辨识净加速度/集总增益，"
                    "不能唯一分离重力增益、库仑阻力和黏性阻力。"
                ),
            }
        )
        result.pop("_fit_indices", None)
        result.pop("_smoothed_positions", None)
        result.pop("_velocities", None)
        if args.update_model:
            update_motion_gain(
                config_path,
                float(result["empirical_acceleration_gain_m_s2"]),
            )
        json_path = csv_path.with_suffix(".json")
        json_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (ValueError, RuntimeError, OSError) as error:
        print("摩擦标定失败：{}".format(error), file=sys.stderr)
        return 3
    except (KeyboardInterrupt, EOFError):
        return 130
    finally:
        source.close()


if __name__ == "__main__":
    sys.exit(main())
