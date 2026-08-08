#!/usr/bin/env python3
"""只含串级 PID 本体的钢珠位置调参程序。"""

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any, Dict

from angle_serial import (
    AngleEncodingError,
    PeriodicAngleSender,
    angle_to_serial_displacement_mm,
)
from ball_control import (
    KinematicKalmanFilter,
    PureCascadePIDController,
    ball_position_from_zero,
)
from ball_tracker_source import BallTrackerSource, base_point_from_record
from control_curve_ui import ControlCurveUI
from pure_pid_tuning_ui import PurePIDTuningUI, validate_pure_values
from pure_pid_profiles import (
    DEFAULT_PURE_PROFILE_DIR,
    list_pure_profiles,
    load_active_pure_profile,
    load_pure_profile,
    rename_pure_profile,
    save_pure_profile,
    set_active_pure_profile,
)


H_DIR = Path(__file__).resolve().parent
DEFAULT_MAIN_CONFIG = H_DIR / "ball_control_config.json"
DEFAULT_PID_CONFIG = H_DIR / "pure_pid_tuning.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "纯串级PID调参：位置PID输出目标速度，速度PID输出倾角；"
            "不使用限幅、死区、预停、补偿、局部零点或输出平滑"
        )
    )
    parser.add_argument("--target-cm", type=float, default=0.0)
    parser.add_argument("--pid-config", type=Path, default=DEFAULT_PID_CONFIG)
    parser.add_argument("--config", type=Path, default=DEFAULT_MAIN_CONFIG)
    parser.add_argument("--enable-serial", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int)
    parser.add_argument("--no-ui", action="store_true", help="不打开实时PID调参窗口")
    parser.add_argument(
        "--no-plot-ui",
        action="store_true",
        help="不打开目标/实际速度和位置实时曲线窗口",
    )
    parser.add_argument(
        "tracker_args",
        nargs=argparse.REMAINDER,
        help="写在 -- 后并原样传给ball_depth_tracker.py",
    )
    return parser.parse_args()


def load_pid_config(path: Path) -> Dict[str, float]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("纯PID配置的根节点必须是对象。")
    required = set(PureCascadePIDController.REQUIRED_KEYS)
    actual = set(raw)
    if actual != required:
        missing = sorted(required - actual)
        extra = sorted(actual - required)
        raise ValueError(
            "纯PID配置必须且只能包含六个PID参数；缺少={}，多余={}。"
            .format(missing, extra)
        )
    return validate_pure_values(raw)


def save_pid_config(path: Path, values: Dict[str, float]) -> None:
    validated = validate_pure_values(values)
    resolved = path.expanduser().resolve()
    temporary = resolved.with_name(resolved.name + ".tmp")
    temporary.write_text(
        json.dumps(validated, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(resolved)


def load_main_config(path: Path) -> Dict[str, Any]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("主配置的根节点必须是对象。")
    zero = raw.get("zero_point_base_m")
    if not isinstance(zero, list) or len(zero) != 3:
        raise ValueError("主配置缺少有效的zero_point_base_m。")
    return raw


def internal_target_m(target_cm: float, config: Dict[str, Any]) -> float:
    centered_m = float(target_cm) / 100.0
    if not math.isfinite(centered_m):
        raise ValueError("目标位置必须是有限数。")
    pipe_length_m = float(config["pipe_length_m"])
    ball_radius_m = float(config["zero_calibration_ball_radius_m"])
    half_span_m = pipe_length_m / 2.0 - ball_radius_m
    if not -half_span_m <= centered_m <= half_span_m:
        raise ValueError(
            "目标位置必须在{:+.1f}～{:+.1f} cm。".format(
                -half_span_m * 100.0, half_span_m * 100.0
            )
        )
    return float(config["target_coordinate_center_m"]) + centered_m


def compact_line(
    target_cm: float,
    position_cm: float,
    velocity_target_cm_s: float,
    velocity_cm_s: float,
    angle_deg: float,
    motor_mm: Any,
) -> str:
    return json.dumps(
        {
            "tgt": round(target_cm, 3),
            "pos": round(position_cm, 3),
            "err": round(target_cm - position_cm, 3),
            "v_tgt": round(velocity_target_cm_s, 3),
            "vel": round(velocity_cm_s, 3),
            "deg": round(angle_deg, 3),
            "mm": None if motor_mm is None else round(float(motor_mm), 2),
        },
        ensure_ascii=False,
        separators=(",", ":"),
    )


def main() -> int:
    args = parse_args()
    sender = None
    source = None
    tuning_ui = None
    curve_ui = None
    try:
        main_config = load_main_config(args.config.expanduser().resolve())
        pid_values = load_pid_config(args.pid_config.expanduser().resolve())
        active_profile, active_values = load_active_pure_profile(
            DEFAULT_PURE_PROFILE_DIR
        )
        if active_values is not None:
            pid_values = active_values
        target_m = internal_target_m(args.target_cm, main_config)
        center_m = float(main_config["target_coordinate_center_m"])
        controller = PureCascadePIDController(pid_values)
        estimator = KinematicKalmanFilter(**main_config["estimator"])
        pipe_length_m = float(main_config["pipe_length_m"])
        ball_radius_m = float(main_config["zero_calibration_ball_radius_m"])
        target_limit_cm = (pipe_length_m / 2.0 - ball_radius_m) * 100.0

        print(
            "纯串级PID：位置P/I/D={:.6g}/{:.6g}/{:.6g}，"
            "速度P/I/D={:.6g}/{:.6g}/{:.6g}，目标{:+.3f}cm。"
            .format(
                pid_values["position_kp_s_inv"],
                pid_values["position_ki_s2_inv"],
                pid_values["position_kd"],
                pid_values["velocity_kp_deg_per_m_s"],
                pid_values["velocity_ki_deg_per_m"],
                pid_values["velocity_kd"],
                args.target_cm,
            ),
            file=sys.stderr,
        )
        print(
            "控制器未启用软件限幅、死区、预停、静摩擦补偿、"
            "局部零点、积分限幅或输出平滑。",
            file=sys.stderr,
        )

        if args.enable_serial:
            scale = float(main_config.get("motor_displacement_scale", 1.0))
            sender = PeriodicAngleSender.open(
                port=args.port or main_config["serial_port"],
                baudrate=args.baud or int(main_config["serial_baud"]),
                rate_hz=float(main_config["serial_rate_hz"]),
                initial_angle_deg=0.0,
                motor_displacement_scale=scale,
                negative_motor_displacement_scale=scale,
            )
            sender.start()
            print("串口已启用；每个视觉帧更新PID，串口独立周期重发。", file=sys.stderr)
        else:
            print("试运行：不发送串口；真实驱动请加--enable-serial。", file=sys.stderr)

        if not args.no_ui:
            tuning_ui = PurePIDTuningUI(
                pid_values,
                args.target_cm,
                -target_limit_cm,
                target_limit_cm,
                list_pure_profiles(DEFAULT_PURE_PROFILE_DIR),
                active_profile,
            )
            tuning_ui.start()
            print("纯PID实时调参UI已启动；参数点击应用后立即生效。", file=sys.stderr)
        if not args.no_plot_ui:
            curve_ui = ControlCurveUI("position")
            curve_ui.start()
            print(
                "实时曲线UI已启动：速度、位置两个标签页，按有效视觉帧更新。",
                file=sys.stderr,
            )

        source = BallTrackerSource(args.tracker_args)
        source.start()
        previous_timestamp_s = None
        for record in source.records():
            if tuning_ui is not None:
                for message in tuning_ui.poll():
                    try:
                        message_type = message.get("type")
                        if message_type in ("parameters", "save"):
                            pid_values = validate_pure_values(message.get("values", {}))
                            controller.set_config(pid_values)
                            if message_type == "save":
                                save_pid_config(args.pid_config, pid_values)
                                status = "六个PID参数已保存到JSON并实时应用。"
                            else:
                                status = "六个PID参数已实时应用（未保存）。"
                            tuning_ui.set_status(status)
                            print(status, file=sys.stderr)
                        elif message_type == "save_profile":
                            pid_values = validate_pure_values(message.get("values", {}))
                            selected = str(message.get("name", ""))
                            save_pure_profile(
                                selected, pid_values, DEFAULT_PURE_PROFILE_DIR
                            )
                            active_profile = set_active_pure_profile(
                                selected, DEFAULT_PURE_PROFILE_DIR
                            )
                            controller.set_config(pid_values)
                            tuning_ui.set_profiles(
                                list_pure_profiles(DEFAULT_PURE_PROFILE_DIR),
                                active_profile,
                                pid_values,
                            )
                            status = (
                                "已保存并应用{}.json，设为下次启动默认方案。"
                                .format(active_profile)
                            )
                            tuning_ui.set_status(status)
                            print(status, file=sys.stderr)
                        elif message_type == "load_profile":
                            selected = str(message.get("name", ""))
                            pid_values = load_pure_profile(
                                selected, DEFAULT_PURE_PROFILE_DIR
                            )
                            active_profile = set_active_pure_profile(
                                selected, DEFAULT_PURE_PROFILE_DIR
                            )
                            controller.set_config(pid_values)
                            tuning_ui.set_profiles(
                                list_pure_profiles(DEFAULT_PURE_PROFILE_DIR),
                                active_profile,
                                pid_values,
                            )
                            status = (
                                "已实时应用{}.json，设为下次启动默认方案。"
                                .format(active_profile)
                            )
                            tuning_ui.set_status(status)
                            print(status, file=sys.stderr)
                        elif message_type == "rename_profile":
                            renamed = rename_pure_profile(
                                str(message.get("old_name", "")),
                                str(message.get("new_name", "")),
                                DEFAULT_PURE_PROFILE_DIR,
                            )
                            active_profile, _unused = load_active_pure_profile(
                                DEFAULT_PURE_PROFILE_DIR
                            )
                            tuning_ui.set_profiles(
                                list_pure_profiles(DEFAULT_PURE_PROFILE_DIR),
                                active_profile,
                            )
                            tuning_ui.set_status(
                                "已重命名为{}.json。".format(renamed)
                            )
                        elif message_type == "setpoint":
                            new_target_cm = float(message["value"])
                            target_m = internal_target_m(new_target_cm, main_config)
                            args.target_cm = new_target_cm
                            controller.reset(0.0)
                            previous_timestamp_s = None
                            status = (
                                "目标点已实时切换为{:+.2f}cm；PID积分和微分历史已清除。"
                                .format(args.target_cm)
                            )
                            tuning_ui.set_status(status)
                            print(status, file=sys.stderr)
                        elif message_type == "error":
                            print("调参UI错误：{}".format(message.get("message")), file=sys.stderr)
                    except (KeyError, TypeError, ValueError, OSError) as error:
                        tuning_ui.set_status("应用失败：{}".format(error))
            point = base_point_from_record(record)
            if point is None:
                continue
            try:
                timestamp_s = float(record["capture_monotonic_ms"]) / 1000.0
                if not math.isfinite(timestamp_s):
                    raise ValueError("视觉时间戳无效。")
                measured_position_m = ball_position_from_zero(
                    point,
                    main_config["zero_point_base_m"],
                    float(main_config["pipe_length_m"]),
                    tolerance_m=0.03,
                )
                estimate = estimator.update(measured_position_m, timestamp_s)
                if not estimate.measurement_accepted:
                    continue
                if previous_timestamp_s is None:
                    previous_timestamp_s = timestamp_s
                    controller.reset(0.0)
                    continue
                dt_s = timestamp_s - previous_timestamp_s
                if dt_s <= 0.0:
                    continue
                previous_timestamp_s = timestamp_s
                angle_deg = controller.update(
                    estimate.position_m,
                    estimate.velocity_m_s,
                    target_m,
                    dt_s,
                )

                motor_mm = None
                try:
                    motor_mm = angle_to_serial_displacement_mm(
                        angle_deg,
                        float(main_config.get("motor_displacement_scale", 1.0)),
                    )
                except AngleEncodingError:
                    # 纯PID不偷偷裁剪；超出物理协议时保留原始角度并明确报错。
                    pass
                if sender is not None:
                    sender.set_angle(angle_deg)
                    sender.raise_if_failed()
                if curve_ui is not None:
                    curve_ui.offer(
                        time_s=estimate.timestamp_s,
                        target_velocity_cm_s=(
                            controller.last_velocity_reference_m_s * 100.0
                        ),
                        velocity_cm_s=estimate.velocity_m_s * 100.0,
                        target_position_cm=args.target_cm,
                        position_cm=(estimate.position_m - center_m) * 100.0,
                    )
                print(
                    compact_line(
                        args.target_cm,
                        (estimate.position_m - center_m) * 100.0,
                        controller.last_velocity_reference_m_s * 100.0,
                        estimate.velocity_m_s * 100.0,
                        angle_deg,
                        motor_mm,
                    ),
                    flush=True,
                )
            except (KeyError, TypeError, ValueError) as error:
                print("忽略无效视觉记录：{}".format(error), file=sys.stderr)
        return source.wait(timeout=1.0)
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, ValueError, KeyError, json.JSONDecodeError) as error:
        print("运行错误：{}".format(error), file=sys.stderr)
        return 2
    finally:
        if source is not None:
            source.close()
        if tuning_ui is not None:
            tuning_ui.close()
        if curve_ui is not None:
            curve_ui.close()
        if sender is not None:
            sender.close()


if __name__ == "__main__":
    sys.exit(main())
