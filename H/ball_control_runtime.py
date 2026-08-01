#!/usr/bin/env python3
"""视觉闭环控制：新视觉帧触发倾角更新，独立50 Hz串口重发。"""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any, Dict, Optional, Tuple

from angle_serial import (
    LINKAGE_LENGTH_MM,
    MAX_ANGLE_DEG,
    PeriodicAngleSender,
    encode_angle,
)
from ball_control import (
    CascadePIDController,
    CompetitionTargetMonitor,
    ConstrainedMPCController,
    KinematicEstimate,
    KinematicKalmanFilter,
    VelocityLowPassFilter,
    ball_position_from_zero,
)
from ball_tracker_source import BallTrackerSource, base_point_from_record
from control_profiles import (
    DEFAULT_PROFILE_DIR,
    list_profiles,
    load_active_profile,
    load_profile,
    rename_profile,
    save_profile,
    set_active_profile,
)
from control_curve_ui import ControlCurveUI
from control_tuning_ui import (
    ControlTuningUI,
    PARAMETER_SPECS,
    validate_parameter_values,
)
from longitudinal_acceleration import (
    LongitudinalAccelerationSource,
    ManualAccelerationSource,
    ReservedAccelerationSource,
    acceleration_feedforward_angle_deg,
)
from tuning_diagnostics import (
    TuningDebugReporter,
    TuningDiagnostics,
    TuningHealthSample,
    TuningSample,
)


H_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = H_DIR / "ball_control_config.json"
DEFAULT_STREAM_HOST = "192.168.50.199"
VELOCITY_TUNING_LIMIT_CM_S = 5.0
VELOCITY_EDGE_MARGIN_M = 0.005
SPECIAL_TASK_NONE = "none"
SPECIAL_TASK_MINUS4P5_THEN_PLUS5 = "minus4p5_then_plus5"
SPECIAL_TASK_INITIAL_LIMIT_CM = 1.0
SPECIAL_TASK_WAYPOINT_CM = -4.5
SPECIAL_TASK_WAYPOINT_TOLERANCE_CM = 0.05
SPECIAL_TASK_RETURN_TRIGGER_CM = 3.0
SPECIAL_TASK_RETURN_TOLERANCE_CM = 0.05
SPECIAL_TASK_FINAL_CM = 5.0
SPECIAL_TASK_OPEN_LOOP_ANGLE_DEG = 2.0
SPECIAL_TASK_DEFAULTS = {
    "first_point_cm": -3.0,
    "second_point_cm": 5.0,
    "first_angle_deg": 2.43,
    "positive_motor_scale": 0.2,
    "negative_motor_scale": 0.7,
}
SPECIAL_TASK_CONTROL_PROFILE = "my_pos"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="识别钢珠并输出目标管道倾角；默认只计算，不发送串口"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--target-cm",
        type=float,
        default=0.0,
        help=(
            "以管道几何中心为0的球心目标；原零点端为负，"
            "原25cm端为正，球心可达范围-12～+12cm；默认0"
        ),
    )
    parser.add_argument(
        "--control-mode",
        choices=("position", "velocity"),
        default="position",
        help="position为普通位置闭环；velocity绕过位置环单独调速度环",
    )
    parser.add_argument(
        "--target-speed-cm-s",
        type=float,
        default=0.0,
        help="velocity模式的初始目标速度，正值朝电机端，默认0 cm/s",
    )
    parser.add_argument(
        "--controller",
        choices=("cascade_pid", "mpc"),
        default="cascade_pid",
    )
    parser.add_argument(
        "--special-task",
        choices=(SPECIAL_TASK_NONE, SPECIAL_TASK_MINUS4P5_THEN_PLUS5),
        default=SPECIAL_TASK_NONE,
        help=(
            "启用UI可重复特殊位置任务；五个任务参数在UI底部设置，"
            "点击启动且钢珠位于中心±1cm才执行；默认none"
        ),
    )
    parser.add_argument(
        "--working-angle-limit-deg",
        type=float,
        default=None,
        help=(
            "临时覆盖参数文件中的工作限角；协议最大{}度"
            .format(MAX_ANGLE_DEG)
        ),
    )
    parser.add_argument(
        "--equilibrium-angle-bias-deg",
        type=float,
        help=(
            "本次目标位置的非零平衡保持角初值；不同目标可给不同"
            "小角度，省略时读取配置默认值"
        ),
    )
    parser.add_argument(
        "--enable-serial",
        action="store_true",
        help="真实打开串口驱动电机；默认安全试运行",
    )
    parser.add_argument("--port", help="覆盖配置文件串口")
    parser.add_argument("--baud", type=int, help="覆盖配置文件波特率")
    parser.add_argument(
        "--print-every",
        type=int,
        default=25,
        help="每N个25Hz监督周期输出一次状态JSON；默认25即约每秒1行，0关闭",
    )
    parser.add_argument(
        "--telemetry",
        choices=("compact", "full"),
        default="compact",
        help="终端状态字段：默认compact精简，full显示全部调试字段",
    )
    parser.add_argument(
        "--enable-mpc-dob",
        action="store_true",
        help="实验性：允许MPC使用视觉加速度更新常值扰动估计",
    )
    parser.add_argument(
        "--enable-acceleration-feedforward",
        action="store_true",
        help=(
            "启用车辆纵向加速度前馈；默认关闭。正方向为固定端到"
            "电机端，当前只保留数据接口，尚未连接加速度串口"
        ),
    )
    parser.add_argument(
        "--test-cart-acceleration-m-s2",
        type=float,
        help=(
            "仅用于无加速度串口时的软件验证；提供恒定纵向加速度，"
            "必须与--enable-acceleration-feedforward一起使用"
        ),
    )
    parser.add_argument(
        "--stop-on-competition-failure",
        action="store_true",
        help=(
            "越过目标另一侧1cm后立即归零并退出；默认只记录失败，"
            "继续控制钢珠返回目标"
        ),
    )
    parser.add_argument(
        "--tuning-debug",
        action="store_true",
        help=(
            "开启串级PID实机只读调参诊断；后台限频输出建议，"
            "不自动修改参数或控制量"
        ),
    )
    parser.add_argument(
        "--stream-host",
        default=DEFAULT_STREAM_HOST,
        help=(
            "控制器默认视频接收PC地址；默认{}，写在--之前"
            .format(DEFAULT_STREAM_HOST)
        ),
    )
    parser.add_argument(
        "--no-stream",
        action="store_true",
        help="关闭控制器默认视频推流",
    )
    parser.add_argument(
        "--no-control-ui",
        action="store_true",
        help="不打开串级PID实时调参和参数文件管理窗口",
    )
    parser.add_argument(
        "--no-plot-ui",
        action="store_true",
        help="不打开独立的目标/实际位置与速度实时曲线窗口",
    )
    parser.add_argument(
        "tracker_args",
        nargs=argparse.REMAINDER,
        help="写在 -- 后，原样传给ball_depth_tracker.py",
    )
    return parser.parse_args()


def load_config(path: Path) -> Dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    zero = data.get("zero_point_base_m")
    if not isinstance(zero, list) or len(zero) != 3:
        raise ValueError(
            "尚未标定零点。请先运行 ./H/start_ball_zero_calibration.sh"
        )
    required_positive = (
        "pipe_length_m",
        "control_rate_hz",
        "serial_rate_hz",
        "measurement_timeout_s",
        "velocity_filter_time_constant_s",
    )
    for key in required_positive:
        if float(data[key]) <= 0:
            raise ValueError("{}必须大于0。".format(key))
    for key in (
        "velocity_stationary_window_s",
        "velocity_stationary_position_span_m",
        "velocity_stationary_threshold_m_s",
    ):
        if not math.isfinite(float(data[key])) or float(data[key]) <= 0.0:
            raise ValueError("{}必须为有限正数。".format(key))
    if float(data["serial_rate_hz"]) < 20.0:
        raise ValueError("串口发送频率必须至少20 Hz。")
    center_m = float(data.get("target_coordinate_center_m", 0.0))
    ball_radius_m = float(data.get("zero_calibration_ball_radius_m", 0.0))
    if (
        not math.isfinite(center_m)
        or center_m <= 0.0
        or center_m >= float(data["pipe_length_m"])
    ):
        raise ValueError(
            "target_coordinate_center_m必须位于(0, pipe_length_m)范围内。"
        )
    if (
        not math.isfinite(ball_radius_m)
        or ball_radius_m <= 0.0
        or 2.0 * ball_radius_m >= float(data["pipe_length_m"])
    ):
        raise ValueError(
            "zero_calibration_ball_radius_m必须为合理的正数。"
        )
    feedforward = data.get("acceleration_feedforward")
    if not isinstance(feedforward, dict):
        raise ValueError("缺少acceleration_feedforward配置。")
    for key in (
        "gravity_m_s2",
        "measurement_timeout_s",
        "max_abs_acceleration_m_s2",
    ):
        value = float(feedforward[key])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(
                "acceleration_feedforward.{}必须为有限正数。".format(
                    key
                )
            )
    equilibrium_bias_deg = float(
        data.get("equilibrium_angle_bias_deg", 0.0)
    )
    if not math.isfinite(equilibrium_bias_deg):
        raise ValueError("equilibrium_angle_bias_deg必须是有限值。")
    motor_displacement_scale = float(
        data.get("motor_displacement_scale", 1.0)
    )
    if (
        not math.isfinite(motor_displacement_scale)
        or not 0.0 < motor_displacement_scale <= 2.0
    ):
        raise ValueError("motor_displacement_scale必须在(0,2.0]范围内。")
    data["motor_displacement_scale"] = motor_displacement_scale
    special_task_values = dict(SPECIAL_TASK_DEFAULTS)
    raw_special_task = data.get("special_task", {})
    if not isinstance(raw_special_task, dict):
        raise ValueError("special_task必须是对象。")
    special_task_values.update(raw_special_task)
    physical_angle_limit = min(
        abs(float(data["angle_min_deg"])),
        abs(float(data["angle_max_deg"])),
    )
    data["special_task"] = validate_special_task_settings(
        special_task_values,
        centered_target_limits_cm(data),
        physical_angle_limit,
    )
    return data


def save_special_task_config(
    config_path: Path, settings: Dict[str, float]
) -> None:
    """只原子覆盖主配置中的special_task，不写入当前PID方案值。"""

    path = Path(config_path).expanduser().resolve()
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("主配置文件根节点必须是对象。")
    raw["special_task"] = {
        key: float(settings[key]) for key in SPECIAL_TASK_DEFAULTS
    }
    temporary_path = path.with_name(path.name + ".special-task.tmp")
    temporary_path.write_text(
        json.dumps(raw, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary_path.replace(path)


_CONTROL_UI_GLOBAL_KEYS = {
    "working_angle_limit_deg",
    "max_angle_step_deg",
    "motor_displacement_scale",
    "equilibrium_angle_bias_deg",
    "velocity_filter_time_constant_s",
}


def control_ui_values(
    config: Dict[str, Any],
    working_angle_limit_deg: float,
    equilibrium_angle_bias_deg: float,
) -> Dict[str, float]:
    values: Dict[str, Any] = {
        "working_angle_limit_deg": working_angle_limit_deg,
        "max_angle_step_deg": config["max_angle_step_deg"],
        "motor_displacement_scale": config.get(
            "motor_displacement_scale", 1.0
        ),
        "equilibrium_angle_bias_deg": equilibrium_angle_bias_deg,
        "velocity_filter_time_constant_s": config[
            "velocity_filter_time_constant_s"
        ],
    }
    for spec in PARAMETER_SPECS:
        if spec.key not in _CONTROL_UI_GLOBAL_KEYS:
            values[spec.key] = config["cascade_pid"][spec.key]
    return validate_parameter_values(values)


def overlay_control_parameters(
    config: Dict[str, Any], values: Dict[str, Any]
) -> Dict[str, float]:
    validated = validate_parameter_values(values)
    config["working_angle_limit_deg"] = validated[
        "working_angle_limit_deg"
    ]
    config["max_angle_step_deg"] = validated["max_angle_step_deg"]
    config["motor_displacement_scale"] = validated[
        "motor_displacement_scale"
    ]
    config["equilibrium_angle_bias_deg"] = validated[
        "equilibrium_angle_bias_deg"
    ]
    config["velocity_filter_time_constant_s"] = validated[
        "velocity_filter_time_constant_s"
    ]
    for key, value in validated.items():
        if key not in _CONTROL_UI_GLOBAL_KEYS:
            config["cascade_pid"][key] = value
    return validated


def apply_control_parameters(
    controller: CascadePIDController,
    config: Dict[str, Any],
    values: Dict[str, Any],
    current_angle_deg: float,
) -> Tuple[float, float, float]:
    """实时应用参数，清除旧积分并从当前安全倾角继续。"""

    validated = overlay_control_parameters(config, values)
    limit = validated["working_angle_limit_deg"]
    current = max(-limit, min(limit, float(current_angle_deg)))
    controller.angle_min_deg = -limit
    controller.angle_max_deg = limit
    controller.rate_limiter.minimum_deg = -limit
    controller.rate_limiter.maximum_deg = limit
    controller.rate_limiter.max_step_deg = validated[
        "max_angle_step_deg"
    ]
    controller.reset(current)
    return (
        limit,
        validated["equilibrium_angle_bias_deg"],
        current,
    )


def reset_velocity_state_preserving_local_zero(
    controller: CascadePIDController, angle_deg: float
) -> None:
    """清速度积分/停滞计时，但保留当前管段已经学习的角度零点。"""

    local_zero_deg = controller.local_zero_angle_deg
    local_zero_position_m = controller.local_zero_position_m
    controller.reset(angle_deg)
    controller.local_zero_angle_deg = local_zero_deg
    controller.local_zero_position_m = local_zero_position_m


def predicted_local_zero_from_target_cm(
    centered_target_cm: float, config: Dict[str, Any]
) -> Tuple[float, float]:
    """按标定点线性预测目标附近的局部零点，返回(角度, 电机mm)。"""

    prior = config.get("position_local_zero_prior", {})
    if not isinstance(prior, dict) or not bool(prior.get("enabled", False)):
        return 0.0, 0.0
    raw_points = prior.get("points", [])
    if not isinstance(raw_points, list) or len(raw_points) < 2:
        raise ValueError("position_local_zero_prior.points至少需要两个标定点。")
    points = sorted(
        (
            float(point["position_cm"]),
            float(point["motor_mm"]),
        )
        for point in raw_points
    )
    if any(not all(math.isfinite(value) for value in point) for point in points):
        raise ValueError("局部零点先验标定点必须是有限数。")
    if any(right[0] <= left[0] for left, right in zip(points, points[1:])):
        raise ValueError("局部零点先验的position_cm必须严格递增且不能重复。")

    target_cm = float(centered_target_cm)
    if not math.isfinite(target_cm):
        raise ValueError("局部零点先验目标位置必须是有限数。")
    if target_cm <= points[0][0]:
        motor_mm = points[0][1]
    elif target_cm >= points[-1][0]:
        motor_mm = points[-1][1]
    else:
        motor_mm = points[0][1]
        for left, right in zip(points, points[1:]):
            if left[0] <= target_cm <= right[0]:
                ratio = (target_cm - left[0]) / (right[0] - left[0])
                motor_mm = left[1] + ratio * (right[1] - left[1])
                break
    angle_deg = math.degrees(
        math.atan(float(motor_mm) / LINKAGE_LENGTH_MM)
    )
    return angle_deg, float(motor_mm)


def apply_position_local_zero_prior(
    controller: Any,
    centered_target_cm: float,
    internal_target_m: float,
    config: Dict[str, Any],
) -> Tuple[float, float]:
    """把位置目标先验写入PID；停滞学习仍可在之后覆盖。"""

    if not isinstance(controller, CascadePIDController):
        return 0.0, 0.0
    angle_deg, motor_mm = predicted_local_zero_from_target_cm(
        centered_target_cm, config
    )
    controller.local_zero_angle_deg = min(
        max(angle_deg, controller.angle_min_deg), controller.angle_max_deg
    )
    controller.local_zero_position_m = float(internal_target_m)
    return controller.local_zero_angle_deg, motor_mm


def control_target_from_centered(
    centered_target_cm: float, config: Dict[str, Any]
) -> float:
    """把以管中心为0的目标换成以标定球心零位为0的内部目标。"""

    centered_target_m = float(centered_target_cm) / 100.0
    if not math.isfinite(centered_target_m):
        raise ValueError("目标位置必须是有限值。")
    center_m = float(config["target_coordinate_center_m"])
    pipe_length_m = float(config["pipe_length_m"])
    ball_radius_m = float(config["zero_calibration_ball_radius_m"])
    reachable_half_span_m = pipe_length_m / 2.0 - ball_radius_m
    minimum_m = -reachable_half_span_m
    maximum_m = reachable_half_span_m
    if not minimum_m <= centered_target_m <= maximum_m:
        raise ValueError(
            "中心坐标目标必须在{:+.1f}～{:+.1f} cm。".format(
                minimum_m * 100.0,
                maximum_m * 100.0,
            )
        )
    return centered_target_m + center_m


def special_waypoint_reached(centered_position_cm: float) -> bool:
    """特殊任务从0向负方向运动，进入-4.5cm的0.05cm带即算到达。"""

    return float(centered_position_cm) <= (
        SPECIAL_TASK_WAYPOINT_CM + SPECIAL_TASK_WAYPOINT_TOLERANCE_CM
    )


def special_return_trigger_reached(centered_position_cm: float) -> bool:
    """回程开环到达+3cm的0.05cm带后交给最终位置闭环。"""

    return float(centered_position_cm) >= (
        SPECIAL_TASK_RETURN_TRIGGER_CM
        - SPECIAL_TASK_RETURN_TOLERANCE_CM
    )


def directed_point_reached(
    position_cm: float,
    target_cm: float,
    direction: int,
    tolerance_cm: float = SPECIAL_TASK_WAYPOINT_TOLERANCE_CM,
) -> bool:
    """判断沿指定方向运动时是否进入或越过目标容差边界。"""

    if direction not in (-1, 1):
        raise ValueError("特殊任务运动方向必须是-1或+1。")
    position = float(position_cm)
    target = float(target_cm)
    tolerance = float(tolerance_cm)
    if not all(math.isfinite(value) for value in (position, target, tolerance)):
        raise ValueError("特殊任务到达判断必须使用有限数。")
    if tolerance < 0.0:
        raise ValueError("特殊任务到达容差不能为负。")
    return (
        position >= target - tolerance
        if direction > 0
        else position <= target + tolerance
    )


def validate_special_task_settings(
    raw: Any,
    target_limits_cm: Tuple[float, float],
    working_angle_limit_deg: float,
) -> Dict[str, float]:
    """验证两段式特殊任务参数。"""

    if not isinstance(raw, dict):
        raise ValueError("特殊任务参数格式无效。")
    settings: Dict[str, float] = {}
    for key in SPECIAL_TASK_DEFAULTS:
        value = float(raw.get(key, SPECIAL_TASK_DEFAULTS[key]))
        if not math.isfinite(value):
            raise ValueError("特殊任务参数{}必须是有限数。".format(key))
        settings[key] = value
    minimum_cm, maximum_cm = map(float, target_limits_cm)
    for key in ("first_point_cm", "second_point_cm"):
        if not minimum_cm <= settings[key] <= maximum_cm:
            raise ValueError(
                "{}必须在{:+.2f}到{:+.2f}cm内。".format(
                    key, minimum_cm, maximum_cm
                )
            )
    first = settings["first_point_cm"]
    second = settings["second_point_cm"]
    if math.isclose(first, second, abs_tol=1e-9):
        raise ValueError("第一个到达点和第二个到达点不能相同。")
    limit = float(working_angle_limit_deg)
    for key in ("first_angle_deg",):
        if abs(settings[key]) > limit:
            raise ValueError(
                "{}超过当前工作限角±{:.2f}°。".format(key, limit)
            )
    for key in ("positive_motor_scale", "negative_motor_scale"):
        if not 0.0 < settings[key] <= 2.0:
            raise ValueError("{}必须在(0,2.0]范围内。".format(key))
    return settings


def centered_target_limits_cm(config: Dict[str, Any]) -> Tuple[float, float]:
    """返回受管长和球半径限制的界面目标范围。"""

    half_span_cm = (
        float(config["pipe_length_m"]) / 2.0
        - float(config["zero_calibration_ball_radius_m"])
    ) * 100.0
    return -half_span_cm, half_span_cm


def velocity_edge_trigger_cm(config: Dict[str, Any]) -> float:
    """速度调试的球心端点阈值：可达端点内再留0.5cm检测余量。"""

    reachable_half_span_m = (
        float(config["pipe_length_m"]) / 2.0
        - float(config["zero_calibration_ball_radius_m"])
    )
    trigger_m = reachable_half_span_m - VELOCITY_EDGE_MARGIN_M
    if trigger_m <= 0.0:
        raise ValueError("管道可达范围不足以设置速度模式端点保护。")
    return trigger_m * 100.0


def outward_velocity_edge_reached(
    centered_position_cm: float,
    target_velocity_cm_s: float,
    trigger_abs_cm: float,
) -> bool:
    """只在目标速度继续朝当前端点外侧时触发锁存。"""

    position = float(centered_position_cm)
    velocity_target = float(target_velocity_cm_s)
    trigger = abs(float(trigger_abs_cm))
    if not all(math.isfinite(value) for value in (position, velocity_target, trigger)):
        raise ValueError("速度端点判断输入必须是有限值。")
    return (
        velocity_target > 0.0 and position >= trigger
    ) or (
        velocity_target < 0.0 and position <= -trigger
    )


def create_target_monitor(
    target_m: float, config: Dict[str, Any]
) -> CompetitionTargetMonitor:
    """为当前目标创建全新的到达与比赛越界判定状态。"""

    return CompetitionTargetMonitor(
        target_position_m=target_m,
        internal_tolerance_m=float(
            config["safety"]["internal_tolerance_m"]
        ),
        competition_tolerance_m=float(
            config["safety"]["competition_tolerance_m"]
        ),
        settle_velocity_m_s=float(
            config["safety"]["settle_velocity_m_s"]
        ),
        settle_time_s=float(config["safety"]["settle_time_s"]),
    )


class LatestRecord:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.record: Optional[Dict[str, Any]] = None
        self.received_monotonic_s: Optional[float] = None
        self.sequence = 0
        self.finished = False
        self.reader_error: Optional[BaseException] = None

    def put(self, record: Dict[str, Any]) -> None:
        with self.lock:
            self.record = record
            self.received_monotonic_s = time.perf_counter()
            self.sequence += 1

    def mark_finished(
        self, error: Optional[BaseException] = None
    ) -> None:
        with self.lock:
            self.finished = True
            self.reader_error = error

    def get(
        self,
    ) -> Tuple[
        Optional[Dict[str, Any]],
        Optional[float],
        int,
        bool,
        Optional[BaseException],
    ]:
        with self.lock:
            return (
                self.record,
                self.received_monotonic_s,
                self.sequence,
                self.finished,
                self.reader_error,
            )


def read_tracker(source: BallTrackerSource, latest: LatestRecord) -> None:
    error: Optional[BaseException] = None
    try:
        for record in source.records():
            latest.put(record)
    except BaseException as caught:
        error = caught
    finally:
        latest.mark_finished(error)


def point_timestamp_s(record: Dict[str, Any]) -> float:
    if "capture_monotonic_ms" not in record:
        raise ValueError("识别记录缺少capture_monotonic_ms，不能用于实时控制。")
    timestamp_s = float(record["capture_monotonic_ms"]) / 1000.0
    if not math.isfinite(timestamp_s):
        raise ValueError("识别记录采集时间无效。")
    return timestamp_s


def optional_finite_float(
    record: Dict[str, Any], key: str
) -> Optional[float]:
    value = record.get(key)
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def optional_nonnegative_int(
    record: Dict[str, Any], key: str
) -> Optional[int]:
    value = record.get(key)
    if isinstance(value, bool):
        return None
    try:
        number = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if number >= 0 else None


def stop_on_competition_failure(
    args: argparse.Namespace, config: Dict[str, Any]
) -> bool:
    """命令行可强制停止，同时兼容配置文件中的停止开关。"""

    return bool(
        args.stop_on_competition_failure
        or config.get("safety", {}).get(
            "stop_on_competition_failure", False
        )
    )


def predict_to_now(
    estimate: KinematicEstimate, now_s: float, max_age_s: float
) -> Tuple[float, float, float]:
    actual_age = now_s - estimate.timestamp_s
    prediction_age = max(0.0, min(actual_age, max_age_s))
    # 加速度由位置二阶信息得到，视觉噪声下不适合直接参与短时外推。
    # 控制只做常速度延迟补偿；加速度保留给辨识和可选DOB。
    position = (
        estimate.position_m + prediction_age * estimate.velocity_m_s
    )
    velocity = estimate.velocity_m_s
    return position, velocity, actual_age


def telemetry_line(
    cycle: int,
    controller_name: str,
    target_m: float,
    estimate: Optional[KinematicEstimate],
    position_now: Optional[float],
    velocity_now: Optional[float],
    measurement_age_s: Optional[float],
    angle_deg: float,
    serial_enabled: bool,
    controller: Any,
    valid_control: bool,
    control_updated: bool,
    target_status: Optional[Any],
    mode: str = "compact",
    coordinate_center_m: float = 0.0,
    acceleration_feedforward_enabled: bool = False,
    acceleration_sample_valid: bool = False,
    cart_acceleration_m_s2: float = 0.0,
    acceleration_feedforward_deg: float = 0.0,
    equilibrium_angle_bias_deg: float = 0.0,
    control_mode: str = "position",
    motor_displacement_scale: float = 1.0,
    negative_motor_displacement_scale: Optional[float] = None,
) -> str:
    encoded_angle = encode_angle(
        angle_deg,
        motor_displacement_scale,
        negative_motor_displacement_scale,
    )
    motor_displacement_mm = (
        int(encoded_angle[2]) + int(encoded_angle[3]) / 100.0
    )
    if encoded_angle[1] == 0x01:
        motor_displacement_mm = -motor_displacement_mm
    if mode == "compact":
        position_cm = (
            None
            if position_now is None
            else round(
                (float(position_now) - coordinate_center_m) * 100.0,
                3,
            )
        )
        error_cm = (
            None
            if position_now is None
            else round(
                (float(target_m) - float(position_now)) * 100.0, 3
            )
        )
        speed_payload: Dict[str, Any] = {
            "v_tgt": (
                round(controller.last_velocity_reference_m_s * 100.0, 3)
                if valid_control
                and isinstance(controller, CascadePIDController)
                else None
            ),
            "vel": (
                None
                if velocity_now is None
                else round(float(velocity_now) * 100.0, 3)
            ),
            "deg": round(float(angle_deg), 2),
            "mm": motor_displacement_mm,
        }
        if control_mode == "velocity":
            compact_payload = {"pos": position_cm, **speed_payload}
        else:
            compact_payload = {
                "tgt": round(
                    (target_m - coordinate_center_m) * 100.0, 3
                ),
                "pos": position_cm,
                "err": error_cm,
                **speed_payload,
            }
        return json.dumps(
            compact_payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )

    payload: Dict[str, Any] = {
        "control_mode": control_mode,
        "cycle": cycle,
        "valid_control": valid_control,
        "control_updated_from_new_vision": control_updated,
        "controller": controller_name,
        "target_cm": (
            None
            if control_mode == "velocity"
            else round((target_m - coordinate_center_m) * 100.0, 3)
        ),
        "angle_command_deg": round(angle_deg, 3),
        "motor_displacement_mm": round(motor_displacement_mm, 2),
        "motor_displacement_positive_scale": round(
            motor_displacement_scale, 4
        ),
        "motor_displacement_negative_scale": round(
            motor_displacement_scale
            if negative_motor_displacement_scale is None
            else negative_motor_displacement_scale,
            4,
        ),
        "serial_payload_hex": encode_angle(
            angle_deg,
            motor_displacement_scale,
            negative_motor_displacement_scale,
        ).hex(" "),
        "serial_enabled": serial_enabled,
        "acceleration_feedforward_enabled": (
            acceleration_feedforward_enabled
        ),
        "acceleration_sample_valid": acceleration_sample_valid,
        "cart_acceleration_m_s2": round(
            cart_acceleration_m_s2, 4
        ),
        "acceleration_feedforward_deg": round(
            acceleration_feedforward_deg, 4
        ),
        "equilibrium_angle_bias_deg": round(
            equilibrium_angle_bias_deg, 4
        ),
    }
    if (
        estimate is not None
        and position_now is not None
        and velocity_now is not None
        and measurement_age_s is not None
    ):
        payload.update(
            {
                "position_cm": round(
                    (float(position_now) - coordinate_center_m)
                    * 100.0,
                    3,
                ),
                "velocity_cm_s": round(float(velocity_now) * 100.0, 3),
                "acceleration_cm_s2": round(
                    estimate.acceleration_m_s2 * 100.0, 3
                ),
                "measurement_age_ms": round(
                    float(measurement_age_s) * 1000.0, 2
                ),
                "measurement_accepted": estimate.measurement_accepted,
            }
        )
    if isinstance(controller, CascadePIDController):
        payload["velocity_reference_cm_s"] = round(
            controller.last_velocity_reference_m_s * 100.0, 3
        )
        payload["active_target_offset_cm"] = round(
            controller.active_target_offset_m * 100.0, 3
        )
        payload["static_compensation_deg"] = round(
            controller.static_compensation_deg, 3
        )
        payload["stall_drive_boost_deg"] = round(
            controller.stall_drive_boost_angle_deg, 3
        )
        payload["temporary_local_zero_deg"] = round(
            controller.local_zero_angle_deg, 3
        )
        payload["temporary_local_zero_position_cm"] = (
            None
            if controller.local_zero_position_m is None
            else round(
                (
                    controller.local_zero_position_m
                    - coordinate_center_m
                )
                * 100.0,
                3,
            )
        )
    if isinstance(controller, ConstrainedMPCController):
        payload.update(
            {
                "mpc_solve_success": controller.last_solve_success,
                "mpc_solve_ms": round(controller.last_solve_ms, 3),
                "disturbance_accel_m_s2": round(
                    controller.disturbance_accel_m_s2, 5
                ),
            }
        )
    if target_status is not None:
        payload.update(
            {
                "target_error_cm": round(
                    target_status.error_m * 100.0, 3
                ),
                "within_internal_tolerance": (
                    target_status.within_internal_tolerance
                ),
                "settled": target_status.settled,
                "settled_duration_s": round(
                    target_status.settled_duration_s, 3
                ),
                "competition_failed": (
                    target_status.competition_failed
                ),
                "failure_boundary_cm": (
                    None
                    if target_status.failure_boundary_m is None
                    else round(
                        (
                            target_status.failure_boundary_m
                            - coordinate_center_m
                        )
                        * 100.0,
                        3,
                    )
                ),
            }
        )
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def main() -> int:
    args = parse_args()
    config_path = args.config.expanduser().resolve()
    active_profile: Optional[str] = None
    try:
        config = load_config(config_path)
        try:
            active_profile, active_values = load_active_profile(
                DEFAULT_PROFILE_DIR
            )
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
            print(
                "默认参数方案读取失败，改用基础配置：{}".format(error),
                file=sys.stderr,
            )
            active_profile, active_values = None, None
        if active_values is not None:
            overlay_control_parameters(config, active_values)
        if args.special_task != SPECIAL_TASK_NONE:
            if args.control_mode != "position":
                raise ValueError("特殊任务只支持position位置控制模式。")
            if args.controller != "cascade_pid":
                raise ValueError("特殊任务当前只支持cascade_pid控制器。")
            if args.no_control_ui:
                raise ValueError("特殊任务依赖UI启动，不能使用--no-control-ui。")
            args.target_cm = 0.0
            special_defaults = config["special_task"]
            print(
                "特殊任务已启用但尚未启动：UI底部五个任务参数来自配置文件；"
                "默认第一点{:+.2f}cm、第二点{:+.2f}cm，第一段开环倾角"
                "{:+.2f}°，任务正/负比例{:.2f}/{:.2f}。"
                "到达第一点后直接使用{}.json闭环到第二点。"
                "点击启动时钢珠必须位于中心±{:.1f}cm。".format(
                    special_defaults["first_point_cm"],
                    special_defaults["second_point_cm"],
                    special_defaults["first_angle_deg"],
                    special_defaults["positive_motor_scale"],
                    special_defaults["negative_motor_scale"],
                    SPECIAL_TASK_CONTROL_PROFILE,
                    SPECIAL_TASK_INITIAL_LIMIT_CM,
                ),
                file=sys.stderr,
            )
        expected_vision = config.get("expected_vision_rate_hz")
        if (
            not isinstance(expected_vision, list)
            or len(expected_vision) != 2
            or float(expected_vision[0]) <= 0.0
            or float(expected_vision[1]) < float(expected_vision[0])
        ):
            raise ValueError(
                "expected_vision_rate_hz必须是递增的两个正数。"
            )
        target_m = control_target_from_centered(
            args.target_cm, config
        )
        target_velocity_m_s = float(args.target_speed_cm_s) / 100.0
        if (
            not math.isfinite(target_velocity_m_s)
            or abs(float(args.target_speed_cm_s))
            > VELOCITY_TUNING_LIMIT_CM_S
        ):
            raise ValueError(
                "速度模式目标速度必须在±{:.1f} cm/s内。".format(
                    VELOCITY_TUNING_LIMIT_CM_S
                )
            )
        if args.control_mode == "velocity" and args.controller != "cascade_pid":
            raise ValueError("velocity模式只支持cascade_pid速度环。")
        physical_limit = min(
            abs(float(config["angle_min_deg"])),
            abs(float(config["angle_max_deg"])),
        )
        args.working_angle_limit_deg = float(
            config.get("working_angle_limit_deg", 2.0)
            if args.working_angle_limit_deg is None
            else args.working_angle_limit_deg
        )
        if not 0 < args.working_angle_limit_deg <= physical_limit:
            raise ValueError(
                "工作限角必须在(0,{:.1f}]度。".format(physical_limit)
            )
        equilibrium_angle_bias_deg = float(
            config.get("equilibrium_angle_bias_deg", 0.0)
            if args.equilibrium_angle_bias_deg is None
            else args.equilibrium_angle_bias_deg
        )
        if (
            not math.isfinite(equilibrium_angle_bias_deg)
            or abs(equilibrium_angle_bias_deg)
            > args.working_angle_limit_deg
        ):
            raise ValueError(
                "平衡偏置角必须在当前工作限角±{:.2f}°内。".format(
                    args.working_angle_limit_deg
                )
            )
        if args.print_every < 0:
            raise ValueError("print-every不能为负。")
        if args.tuning_debug and args.controller != "cascade_pid":
            raise ValueError(
                "--tuning-debug当前只为cascade_pid提供参数建议；"
                "MPC请先保持只试算。"
            )
        if args.tuning_debug and args.control_mode == "velocity":
            raise ValueError(
                "--tuning-debug当前按位置闭环诊断，不用于velocity模式。"
            )
        if (
            args.test_cart_acceleration_m_s2 is not None
            and not args.enable_acceleration_feedforward
        ):
            raise ValueError(
                "--test-cart-acceleration-m-s2必须与"
                "--enable-acceleration-feedforward一起使用。"
            )
        if args.test_cart_acceleration_m_s2 is not None:
            test_acceleration = float(
                args.test_cart_acceleration_m_s2
            )
            max_test_acceleration = float(
                config["acceleration_feedforward"][
                    "max_abs_acceleration_m_s2"
                ]
            )
            if (
                not math.isfinite(test_acceleration)
                or abs(test_acceleration) > max_test_acceleration
            ):
                raise ValueError(
                    "测试纵向加速度必须在±{:.3f} m/s²内。".format(
                        max_test_acceleration
                    )
                )
        stop_after_competition_failure = stop_on_competition_failure(
            args, config
        )
    except (
        OSError,
        TypeError,
        ValueError,
        KeyError,
        json.JSONDecodeError,
    ) as error:
        print("配置错误：{}".format(error), file=sys.stderr)
        return 2

    estimator = KinematicKalmanFilter(**config["estimator"])
    velocity_filter = VelocityLowPassFilter(
        time_constant_s=float(config["velocity_filter_time_constant_s"]),
        stationary_window_s=float(config["velocity_stationary_window_s"]),
        stationary_position_span_m=float(
            config["velocity_stationary_position_span_m"]
        ),
        stationary_velocity_threshold_m_s=float(
            config["velocity_stationary_threshold_m_s"]
        ),
    )
    coordinate_center_m = float(
        config["target_coordinate_center_m"]
    )
    if args.control_mode == "position":
        print(
            "位置模式：中心坐标目标{:+.3f} cm；内部控制目标"
            " = {:+.3f} + {:.3f} = {:.3f} cm。".format(
                args.target_cm,
                args.target_cm,
                coordinate_center_m * 100.0,
                target_m * 100.0,
            ),
            file=sys.stderr,
        )
    else:
        print(
            "速度环独立调试模式：目标速度{:+.3f} cm/s；"
            "不使用目标位置、位置环、预停点和位置死区。".format(
                args.target_speed_cm_s
            ),
            file=sys.stderr,
        )
        print(
            "注意：管长有限，非零目标速度不会自动停在边缘；"
            "请在钢珠接近端点前从UI设为0或反向。",
            file=sys.stderr,
        )
    print(
        "平衡保持角偏置：{:+.3f}°；PID积分仍可在此基础上"
        "自动形成非零保持角。".format(equilibrium_angle_bias_deg),
        file=sys.stderr,
    )
    print(
        "普通任务倾角到电机升降比例：{:.3f}；"
        "串口发送 h=比例×250×tan(theta)。"
        .format(
            float(config["motor_displacement_scale"]),
        ),
        file=sys.stderr,
    )
    if active_profile is not None:
        print(
            "默认参数文件：{}/{}.json".format(
                DEFAULT_PROFILE_DIR, active_profile
            ),
            file=sys.stderr,
        )
    angle_min = -args.working_angle_limit_deg
    angle_max = args.working_angle_limit_deg
    if args.controller == "cascade_pid":
        controller: Any = CascadePIDController(
            config["cascade_pid"],
            angle_min,
            angle_max,
            float(config["max_angle_step_deg"]),
            config["safety"],
            coordinate_center_m=coordinate_center_m,
        )
    else:
        controller = ConstrainedMPCController(
            config["mpc"],
            config["motion_model"],
            angle_min,
            angle_max,
            float(config["max_angle_step_deg"]),
            config["safety"],
        )
    if args.control_mode == "position":
        prior_angle_deg, prior_motor_mm = apply_position_local_zero_prior(
            controller, args.target_cm, target_m, config
        )
        if isinstance(controller, CascadePIDController):
            print(
                "目标点局部零点先验：{:+.2f}cm -> {:+.3f}mm "
                "({:+.3f}°)；停滞达到阈值后由实测覆盖。".format(
                    args.target_cm, prior_motor_mm, prior_angle_deg
                ),
                file=sys.stderr,
            )
    target_monitor = create_target_monitor(target_m, config)
    tuning_reporter: Optional[TuningDebugReporter] = None
    if args.tuning_debug:
        tuning_reporter = TuningDebugReporter(
            TuningDiagnostics(
                config["cascade_pid"],
                config["safety"],
                args.working_angle_limit_deg,
                enabled=True,
                expected_fps_min=float(
                    config["expected_vision_rate_hz"][0]
                ),
            )
        )

    sender: Optional[PeriodicAngleSender] = None
    control_ui: Optional[ControlTuningUI] = None
    curve_ui: Optional[ControlCurveUI] = None
    acceleration_source: LongitudinalAccelerationSource
    if (
        args.enable_acceleration_feedforward
        and args.test_cart_acceleration_m_s2 is not None
    ):
        acceleration_source = ManualAccelerationSource(
            args.test_cart_acceleration_m_s2
        )
    else:
        acceleration_source = ReservedAccelerationSource()
    tracker_args = list(args.tracker_args)
    if (
        not args.no_stream
        and "--stream-host" not in tracker_args
    ):
        tracker_args.extend(["--stream-host", args.stream_host])
    source = BallTrackerSource(tracker_args)
    latest = LatestRecord()
    reader: Optional[threading.Thread] = None
    angle_deg = 0.0
    competition_failure_reported = False
    last_local_zero_update_count = 0
    cart_acceleration_m_s2 = 0.0
    acceleration_feedforward_deg = 0.0
    acceleration_sample_valid = False
    velocity_tracking_active = args.control_mode != "velocity"
    velocity_waiting_for_detection = False
    velocity_edge_hold = False
    velocity_edge_abs_cm = velocity_edge_trigger_cm(config)
    special_task_settings = dict(config["special_task"])
    special_task_phase = "idle" if args.special_task != SPECIAL_TASK_NONE else "inactive"
    special_manual_zero_hold = False
    special_first_direction = -1
    active_positive_motor_scale = float(config["motor_displacement_scale"])
    active_negative_motor_scale = float(config["motor_displacement_scale"])
    try:
        if args.enable_serial:
            sender = PeriodicAngleSender.open(
                port=args.port or config["serial_port"],
                baudrate=args.baud or int(config["serial_baud"]),
                rate_hz=float(config["serial_rate_hz"]),
                initial_angle_deg=0.0,
                motor_displacement_scale=float(
                    config["motor_displacement_scale"]
                ),
                negative_motor_displacement_scale=float(
                    config["motor_displacement_scale"]
                ),
            )
            sender.start()
            print(
                "真实串口已启用：{} @ {} baud，{:.1f} Hz".format(
                    args.port or config["serial_port"],
                    args.baud or int(config["serial_baud"]),
                    sender.rate_hz,
                ),
                file=sys.stderr,
            )
        else:
            print(
                "安全试运行：只计算倾角，不发送串口；确认方向后加"
                " --enable-serial。",
                file=sys.stderr,
            )
        if args.control_mode == "velocity":
            controller.last_velocity_reference_m_s = target_velocity_m_s
            print(
                "速度环当前暂停并保持0°；请设置目标速度后点击UI中的"
                "“启动速度环”。端点锁存阈值：球心±{:.2f} cm。".format(
                    velocity_edge_abs_cm
                ),
                file=sys.stderr,
            )
        if not args.no_control_ui:
            if isinstance(controller, CascadePIDController):
                initial_ui_values = control_ui_values(
                    config,
                    args.working_angle_limit_deg,
                    equilibrium_angle_bias_deg,
                )
                control_ui = ControlTuningUI(
                    initial_ui_values,
                    list_profiles(DEFAULT_PROFILE_DIR),
                    active_profile,
                    initial_target_cm=(
                        args.target_speed_cm_s
                        if args.control_mode == "velocity"
                        else args.target_cm
                    ),
                    target_min_cm=(
                        -VELOCITY_TUNING_LIMIT_CM_S
                        if args.control_mode == "velocity"
                        else centered_target_limits_cm(config)[0]
                    ),
                    target_max_cm=(
                        VELOCITY_TUNING_LIMIT_CM_S
                        if args.control_mode == "velocity"
                        else centered_target_limits_cm(config)[1]
                    ),
                    setpoint_mode=args.control_mode,
                    special_task_enabled=(
                        args.special_task != SPECIAL_TASK_NONE
                    ),
                    special_task_initial=special_task_settings,
                )
                control_ui.start()
                print(
                    "PID实时调参窗口已启动；关闭界面可用"
                    " --no-control-ui。",
                    file=sys.stderr,
                )
            else:
                print(
                    "控制参数UI当前只支持cascade_pid，MPC运行不打开窗口。",
                    file=sys.stderr,
                )
        if not args.no_plot_ui:
            if isinstance(controller, CascadePIDController):
                curve_ui = ControlCurveUI(args.control_mode)
                curve_ui.start()
                tab_description = (
                    "速度曲线1个标签页"
                    if args.control_mode == "velocity"
                    else "速度、位置曲线2个标签页"
                )
                print(
                    "独立实时曲线窗口已启动：{}；关闭可用 --no-plot-ui。"
                    .format(tab_description),
                    file=sys.stderr,
                )
            else:
                print(
                    "实时曲线窗口当前只支持cascade_pid。",
                    file=sys.stderr,
                )
        if args.enable_acceleration_feedforward:
            if args.test_cart_acceleration_m_s2 is None:
                print(
                    "车辆加速度前馈已启用，但当前加速度串口尚未接入；"
                    "没有有效样本时前馈为0°。",
                    file=sys.stderr,
                )
            else:
                print(
                    "车辆加速度前馈软件测试：a={:+.3f} m/s²，"
                    "正方向为固定端->电机端。".format(
                        args.test_cart_acceleration_m_s2
                    ),
                    file=sys.stderr,
                )
        else:
            print("车辆加速度前馈：关闭。", file=sys.stderr)
        print(
            "时序：视觉新帧约{:.0f}～{:.0f} FPS才更新控制器；"
            "{:.0f} Hz监督超时；串口独立{:.0f} Hz重发最新升降位移。"
            .format(
                float(expected_vision[0]),
                float(expected_vision[1]),
                float(config["control_rate_hz"]),
                float(config["serial_rate_hz"]),
            ),
            file=sys.stderr,
        )
        if tuning_reporter is not None:
            print(
                "调参诊断已开启：只观察实际运动，不修改配置或控制量；"
                "每2秒后台输出状态，异常建议会去重限频。"
                "普通--print-every JSON已由调参监视替代。",
                file=sys.stderr,
            )
            tuning_reporter.start()

        source.start()
        reader = threading.Thread(
            target=read_tracker,
            args=(source, latest),
            name="ball-tracker-json-reader",
            daemon=True,
        )
        reader.start()

        control_period = 1.0 / float(config["control_rate_hz"])
        timeout_s = float(config["measurement_timeout_s"])
        next_deadline = time.perf_counter()
        last_sequence = -1
        last_valid_estimate: Optional[KinematicEstimate] = None
        last_valid_receive_s: Optional[float] = None
        last_control_measurement_timestamp_s: Optional[float] = None
        last_seen_capture_timestamp_s: Optional[float] = None
        control_was_valid = False
        accepted_control_updates = 0
        cycle = 0
        exit_code = 0
        target_status = None
        while True:
            now = time.perf_counter()
            delay = next_deadline - now
            if delay > 0:
                time.sleep(delay)
            now = time.perf_counter()
            next_deadline += control_period
            if next_deadline <= now:
                next_deadline = now + control_period

            if control_ui is not None:
                for ui_message in control_ui.poll():
                    message_type = ui_message.get("type")
                    try:
                        values_to_apply: Optional[Dict[str, float]] = None
                        selected_name: Optional[str] = None
                        if message_type == "special_task_save":
                            if args.special_task == SPECIAL_TASK_NONE:
                                raise ValueError("当前没有启用特殊任务模式。")
                            physical_angle_limit = min(
                                abs(float(config["angle_min_deg"])),
                                abs(float(config["angle_max_deg"])),
                            )
                            saved_special_settings = validate_special_task_settings(
                                ui_message.get("settings"),
                                centered_target_limits_cm(config),
                                physical_angle_limit,
                            )
                            save_special_task_config(
                                config_path, saved_special_settings
                            )
                            config["special_task"] = dict(
                                saved_special_settings
                            )
                            status_message = (
                                "特殊任务参数已直接覆盖到{}；"
                                "未修改PID参数方案。"
                            ).format(config_path)
                            control_ui.set_status(status_message)
                            print(status_message, file=sys.stderr)
                            continue
                        if message_type == "special_task_zero":
                            if args.special_task == SPECIAL_TASK_NONE:
                                raise ValueError("当前没有启用特殊任务模式。")
                            special_task_phase = "idle"
                            special_manual_zero_hold = True
                            active_positive_motor_scale = float(
                                config["motor_displacement_scale"]
                            )
                            active_negative_motor_scale = active_positive_motor_scale
                            angle_deg = 0.0
                            args.target_cm = 0.0
                            target_m = control_target_from_centered(0.0, config)
                            target_monitor = create_target_monitor(target_m, config)
                            target_status = None
                            competition_failure_reported = False
                            controller.reset(0.0)
                            last_control_measurement_timestamp_s = None
                            if sender is not None:
                                sender.set_max_angle_command_step_deg(None)
                                sender.set_motor_displacement_scales(
                                    active_positive_motor_scale,
                                    active_negative_motor_scale,
                                )
                                sender.set_angle(0.0)
                                sender.resume_sending()
                            status_message = (
                                "特殊任务已取消，倾角已给0°；把钢珠放回中心"
                                "±1.00cm后可再次启动。"
                            )
                            control_ui.set_target(0.0)
                            control_ui.set_status(status_message)
                            print(status_message, file=sys.stderr)
                            continue
                        if message_type == "special_task_start":
                            if args.special_task == SPECIAL_TASK_NONE:
                                raise ValueError("当前没有启用特殊任务模式。")
                            requested_special_settings = validate_special_task_settings(
                                ui_message.get("settings"),
                                centered_target_limits_cm(config),
                                args.working_angle_limit_deg,
                            )
                            fresh_measurement = (
                                last_valid_estimate is not None
                                and last_valid_receive_s is not None
                                and now - last_valid_receive_s <= timeout_s
                                and now - last_valid_estimate.timestamp_s <= timeout_s
                                and now - last_valid_estimate.timestamp_s >= -0.02
                            )
                            if not fresh_measurement:
                                raise ValueError(
                                    "没有新鲜的钢珠位置，不能启动特殊任务。"
                                )
                            assert last_valid_estimate is not None
                            start_position_m, _start_velocity, _start_age = predict_to_now(
                                last_valid_estimate, now, timeout_s
                            )
                            start_position_cm = (
                                start_position_m - coordinate_center_m
                            ) * 100.0
                            if abs(start_position_cm) > SPECIAL_TASK_INITIAL_LIMIT_CM:
                                raise ValueError(
                                    "钢珠当前{:+.3f}cm，超出中心±{:.2f}cm，"
                                    "任务未启动。".format(
                                        start_position_cm,
                                        SPECIAL_TASK_INITIAL_LIMIT_CM,
                                    )
                                )
                            task_profile_values = load_profile(
                                SPECIAL_TASK_CONTROL_PROFILE,
                                DEFAULT_PROFILE_DIR,
                            )
                            (
                                args.working_angle_limit_deg,
                                equilibrium_angle_bias_deg,
                                angle_deg,
                            ) = apply_control_parameters(
                                controller,
                                config,
                                task_profile_values,
                                angle_deg,
                            )
                            active_profile = set_active_profile(
                                SPECIAL_TASK_CONTROL_PROFILE,
                                DEFAULT_PROFILE_DIR,
                            )
                            velocity_filter.time_constant_s = float(
                                config["velocity_filter_time_constant_s"]
                            )
                            control_ui.set_profiles(
                                list_profiles(DEFAULT_PROFILE_DIR),
                                active_profile,
                                task_profile_values,
                            )
                            special_task_settings = requested_special_settings
                            first_point = special_task_settings["first_point_cm"]
                            second_point = special_task_settings["second_point_cm"]
                            special_first_direction = (
                                1 if first_point >= start_position_cm else -1
                            )
                            special_task_phase = "to_first"
                            special_manual_zero_hold = False
                            active_positive_motor_scale = special_task_settings[
                                "positive_motor_scale"
                            ]
                            active_negative_motor_scale = special_task_settings[
                                "negative_motor_scale"
                            ]
                            args.target_cm = first_point
                            target_m = control_target_from_centered(first_point, config)
                            target_monitor = create_target_monitor(target_m, config)
                            target_status = None
                            competition_failure_reported = False
                            angle_deg = special_task_settings["first_angle_deg"]
                            controller.reset(angle_deg)
                            last_control_measurement_timestamp_s = None
                            if sender is not None:
                                sender.set_max_angle_command_step_deg(None)
                                sender.set_motor_displacement_scales(
                                    active_positive_motor_scale,
                                    active_negative_motor_scale,
                                )
                                sender.set_angle(angle_deg)
                                sender.resume_sending()
                            status_message = (
                                "特殊任务启动检查通过：钢珠{:+.3f}cm；"
                                "当前给{:+.2f}°，驶向第一点{:+.2f}cm；"
                                "任务正/负位移比例={:.2f}/{:.2f}；"
                                "闭环阶段固定使用{}.json。"
                            ).format(
                                start_position_cm,
                                angle_deg,
                                first_point,
                                active_positive_motor_scale,
                                active_negative_motor_scale,
                                SPECIAL_TASK_CONTROL_PROFILE,
                            )
                            control_ui.set_target(first_point)
                            control_ui.set_status(status_message)
                            print(status_message, file=sys.stderr)
                            continue
                        if message_type == "velocity_zero":
                            if args.control_mode != "velocity":
                                raise ValueError("返回0按钮只用于velocity模式。")
                            velocity_tracking_active = False
                            velocity_waiting_for_detection = False
                            velocity_edge_hold = False
                            angle_deg = 0.0
                            reset_velocity_state_preserving_local_zero(
                                controller, 0.0
                            )
                            controller.last_velocity_reference_m_s = (
                                target_velocity_m_s
                            )
                            if sender is not None:
                                sender.set_angle(0.0)
                            status_message = (
                                "倾斜角已返回0°，速度环暂停；"
                                "局部角度零点保留。请再点击“启动速度环”。"
                            )
                            control_ui.set_status(status_message)
                            print(status_message, file=sys.stderr)
                            continue
                        if message_type == "velocity_start":
                            if args.control_mode != "velocity":
                                raise ValueError("启动速度环按钮只用于velocity模式。")
                            velocity_tracking_active = False
                            velocity_waiting_for_detection = True
                            velocity_edge_hold = False
                            angle_deg = 0.0
                            reset_velocity_state_preserving_local_zero(
                                controller, 0.0
                            )
                            controller.last_velocity_reference_m_s = (
                                target_velocity_m_s
                            )
                            estimator.reset()
                            velocity_filter.reset()
                            last_valid_estimate = None
                            last_valid_receive_s = None
                            last_control_measurement_timestamp_s = None
                            control_was_valid = False
                            _discard_record, _discard_time, last_sequence, _done, _error = (
                                latest.get()
                            )
                            if sender is not None:
                                sender.set_angle(0.0)
                            status_message = (
                                "速度环已请求启动：旧视觉状态已清除，"
                                "等待下一次有效钢珠检测。"
                            )
                            control_ui.set_status(status_message)
                            print(status_message, file=sys.stderr)
                            continue
                        if message_type == "setpoint":
                            if ui_message.get("mode") != args.control_mode:
                                raise ValueError("UI设定类型与当前控制模式不一致。")
                            if args.special_task != SPECIAL_TASK_NONE:
                                status_message = (
                                    "特殊任务正在接管目标点，忽略UI手动目标；"
                                    "当前阶段目标{:+.2f} cm。"
                                ).format(args.target_cm)
                                control_ui.set_target(args.target_cm)
                                control_ui.set_status(status_message)
                                print(status_message, file=sys.stderr)
                                continue
                            new_setpoint = float(ui_message.get("value"))
                            if args.control_mode == "velocity":
                                if (
                                    not math.isfinite(new_setpoint)
                                    or abs(new_setpoint)
                                    > VELOCITY_TUNING_LIMIT_CM_S
                                ):
                                    raise ValueError(
                                        "目标速度必须在±{:.1f} cm/s内。"
                                        .format(VELOCITY_TUNING_LIMIT_CM_S)
                                    )
                                args.target_speed_cm_s = new_setpoint
                                target_velocity_m_s = new_setpoint / 100.0
                                status_message = (
                                    "目标速度已实时切换为{:+.2f} cm/s；"
                                    "旧速度积分已清除，局部角度零点保留。"
                                ).format(new_setpoint)
                            else:
                                target_m = control_target_from_centered(
                                    new_setpoint, config
                                )
                                args.target_cm = new_setpoint
                                target_monitor = create_target_monitor(
                                    target_m, config
                                )
                                target_status = None
                                competition_failure_reported = False
                                status_message = (
                                    "目标点已实时切换为{:+.2f} cm；"
                                    "旧目标积分和到达状态已清除。"
                                ).format(args.target_cm)
                            if (
                                args.control_mode == "velocity"
                                and isinstance(controller, CascadePIDController)
                            ):
                                reset_velocity_state_preserving_local_zero(
                                    controller, angle_deg
                                )
                                controller.last_velocity_reference_m_s = (
                                    target_velocity_m_s
                                )
                            else:
                                controller.reset(angle_deg)
                                if args.control_mode == "position":
                                    prior_angle_deg, prior_motor_mm = (
                                        apply_position_local_zero_prior(
                                            controller,
                                            args.target_cm,
                                            target_m,
                                            config,
                                        )
                                    )
                                    status_message += (
                                        " 局部零点先验{:+.3f}mm({:+.3f}°)，"
                                        "停滞后实测覆盖。"
                                    ).format(prior_motor_mm, prior_angle_deg)
                            control_ui.set_target(new_setpoint)
                            control_ui.set_status(status_message)
                            print(status_message, file=sys.stderr)
                            continue
                        if message_type == "parameters":
                            values_to_apply = validate_parameter_values(
                                ui_message.get("values", {})
                            )
                        elif message_type == "save_profile":
                            selected_name = str(ui_message.get("name", ""))
                            values_to_apply = validate_parameter_values(
                                ui_message.get("values", {})
                            )
                            save_profile(
                                selected_name,
                                values_to_apply,
                                DEFAULT_PROFILE_DIR,
                            )
                            selected_name = set_active_profile(
                                selected_name, DEFAULT_PROFILE_DIR
                            )
                            active_profile = selected_name
                        elif message_type == "load_profile":
                            selected_name = str(ui_message.get("name", ""))
                            values_to_apply = load_profile(
                                selected_name, DEFAULT_PROFILE_DIR
                            )
                            selected_name = set_active_profile(
                                selected_name, DEFAULT_PROFILE_DIR
                            )
                            active_profile = selected_name
                        elif message_type == "rename_profile":
                            selected_name = rename_profile(
                                str(ui_message.get("old_name", "")),
                                str(ui_message.get("new_name", "")),
                                DEFAULT_PROFILE_DIR,
                            )
                            active_profile, _unused = load_active_profile(
                                DEFAULT_PROFILE_DIR
                            )
                            control_ui.set_profiles(
                                list_profiles(DEFAULT_PROFILE_DIR),
                                active_profile,
                            )
                            control_ui.set_status(
                                "已重命名为{}.json。".format(selected_name)
                            )
                            continue
                        elif message_type == "error":
                            print(
                                "PID调参窗口启动失败：{}".format(
                                    ui_message.get("message", "未知错误")
                                ),
                                file=sys.stderr,
                            )
                            continue
                        else:
                            continue

                        if values_to_apply is not None:
                            if special_task_phase == "to_first":
                                requested_limit = float(
                                    values_to_apply["working_angle_limit_deg"]
                                )
                                active_open_loop_angle = special_task_settings[
                                    "first_angle_deg"
                                ]
                                if abs(active_open_loop_angle) > requested_limit:
                                    raise ValueError(
                                        "特殊任务开环倾角{:+.2f}°正在生效，"
                                        "工作限角不能降到{:.2f}°。".format(
                                            active_open_loop_angle,
                                            requested_limit,
                                        )
                                    )
                            (
                                args.working_angle_limit_deg,
                                equilibrium_angle_bias_deg,
                                angle_deg,
                            ) = apply_control_parameters(
                                controller,
                                config,
                                values_to_apply,
                                angle_deg,
                            )
                            if args.control_mode == "position":
                                apply_position_local_zero_prior(
                                    controller,
                                    args.target_cm,
                                    target_m,
                                    config,
                                )
                            velocity_filter.time_constant_s = float(
                                config["velocity_filter_time_constant_s"]
                            )
                            if special_task_phase in ("idle", "inactive"):
                                active_positive_motor_scale = float(
                                    config["motor_displacement_scale"]
                                )
                                active_negative_motor_scale = (
                                    active_positive_motor_scale
                                )
                            if sender is not None:
                                sender.set_motor_displacement_scales(
                                    active_positive_motor_scale,
                                    active_negative_motor_scale,
                                )
                            if tuning_reporter is not None:
                                tuning_reporter.diagnostics.working_angle_limit_deg = (
                                    args.working_angle_limit_deg
                                )
                            if selected_name is not None:
                                control_ui.set_profiles(
                                    list_profiles(DEFAULT_PROFILE_DIR),
                                    active_profile,
                                    values_to_apply,
                                )
                                status_message = (
                                    "已应用{}.json，并设为下次启动默认参数。"
                                    .format(selected_name)
                                )
                            else:
                                status_message = (
                                    "参数已应用到当前控制器（未保存到文件）。"
                                )
                            control_ui.set_status(status_message)
                            print(status_message, file=sys.stderr)
                    except (
                        OSError,
                        TypeError,
                        ValueError,
                        KeyError,
                        json.JSONDecodeError,
                    ) as error:
                        operation_name = (
                            "特殊任务操作"
                            if str(message_type).startswith("special_task_")
                            else "参数操作"
                        )
                        message = "{}失败：{}".format(operation_name, error)
                        control_ui.set_status(message)
                        print(message, file=sys.stderr)

            acceleration_sample_valid = False
            cart_acceleration_m_s2 = 0.0
            acceleration_feedforward_deg = 0.0
            if args.enable_acceleration_feedforward:
                acceleration_sample = (
                    acceleration_source.latest_sample()
                )
                if acceleration_sample is not None:
                    acceleration_age_s = (
                        now - acceleration_sample.monotonic_s
                    )
                    acceleration_limit = float(
                        config["acceleration_feedforward"][
                            "max_abs_acceleration_m_s2"
                        ]
                    )
                    if (
                        -0.02 <= acceleration_age_s
                        <= float(
                            config["acceleration_feedforward"][
                                "measurement_timeout_s"
                            ]
                        )
                        and math.isfinite(
                            acceleration_sample.acceleration_m_s2
                        )
                        and abs(
                            acceleration_sample.acceleration_m_s2
                        )
                        <= acceleration_limit
                    ):
                        acceleration_sample_valid = True
                        cart_acceleration_m_s2 = float(
                            acceleration_sample.acceleration_m_s2
                        )
                        acceleration_feedforward_deg = (
                            acceleration_feedforward_angle_deg(
                                cart_acceleration_m_s2,
                                float(
                                    config[
                                        "acceleration_feedforward"
                                    ]["gravity_m_s2"]
                                ),
                            )
                        )

            (
                record,
                record_received_s,
                sequence,
                tracker_finished,
                reader_error,
            ) = latest.get()
            new_measurement_accepted = False
            if sequence != last_sequence and record is not None:
                last_sequence = sequence
                tuning_health_reason = str(
                    record.get("reason") or "识别记录没有有效球心坐标"
                )
                point = base_point_from_record(record)
                if point is not None:
                    try:
                        capture_timestamp_s = point_timestamp_s(record)
                        if (
                            last_seen_capture_timestamp_s is not None
                            and capture_timestamp_s
                            <= last_seen_capture_timestamp_s
                        ):
                            raise ValueError("识别记录采集时间重复或倒退。")
                        last_seen_capture_timestamp_s = capture_timestamp_s
                        capture_age_s = now - capture_timestamp_s
                        if capture_age_s < -0.02:
                            raise ValueError("识别记录采集时间晚于当前单调时钟。")
                        if capture_age_s > timeout_s:
                            raise ValueError(
                                "识别记录已过期{:.1f} ms。".format(
                                    capture_age_s * 1000.0
                                )
                            )
                        position = ball_position_from_zero(
                            point,
                            config["zero_point_base_m"],
                            float(config["pipe_length_m"]),
                            tolerance_m=0.03,
                        )
                        estimate = estimator.update(
                            position, capture_timestamp_s
                        )
                        if estimate.measurement_accepted:
                            filtered_velocity_m_s = velocity_filter.update(
                                estimate.velocity_m_s,
                                estimate.timestamp_s,
                                estimate.position_m,
                            )
                            last_valid_estimate = KinematicEstimate(
                                position_m=estimate.position_m,
                                velocity_m_s=filtered_velocity_m_s,
                                acceleration_m_s2=estimate.acceleration_m_s2,
                                timestamp_s=estimate.timestamp_s,
                                measurement_accepted=True,
                            )
                            last_valid_receive_s = (
                                now
                                if record_received_s is None
                                else record_received_s
                            )
                            new_measurement_accepted = True
                            accepted_control_updates += 1
                        else:
                            tuning_health_reason = (
                                "Kalman离群门控拒绝本帧位置"
                            )
                    except ValueError as error:
                        tuning_health_reason = "位置异常：{}".format(
                            error
                        )
                        if tuning_reporter is None:
                            print(
                                "忽略位置异常值：{}".format(error),
                                file=sys.stderr,
                            )
                if (
                    tuning_reporter is not None
                    and not new_measurement_accepted
                ):
                    tuning_reporter.offer_health(
                        TuningHealthSample(
                            monotonic_s=now,
                            reason=tuning_health_reason,
                            frame=optional_nonnegative_int(
                                record, "frame"
                            ),
                            processing_latency_ms=(
                                optional_finite_float(
                                    record,
                                    "processing_latency_ms",
                                )
                            ),
                        )
                    )

            if (
                args.control_mode == "velocity"
                and velocity_waiting_for_detection
                and new_measurement_accepted
            ):
                velocity_waiting_for_detection = False
                velocity_tracking_active = True
                status_message = (
                    "已重新检测到有效钢珠，速度环开始跟踪"
                    "{:+.2f} cm/s。"
                ).format(args.target_speed_cm_s)
                if control_ui is not None:
                    control_ui.set_status(status_message)
                print(status_message, file=sys.stderr)

            valid_control = (
                last_valid_estimate is not None
                and last_valid_receive_s is not None
                and now - last_valid_receive_s <= timeout_s
                and now - last_valid_estimate.timestamp_s <= timeout_s
                and now - last_valid_estimate.timestamp_s >= -0.02
            )
            position_now: Optional[float] = None
            velocity_now: Optional[float] = None
            measurement_age: Optional[float] = None
            if valid_control:
                assert last_valid_estimate is not None
                position_now, velocity_now, measurement_age = predict_to_now(
                    last_valid_estimate, now, timeout_s
                )
                if not control_was_valid:
                    recovery_angle_deg = (
                        angle_deg
                        if args.special_task != SPECIAL_TASK_NONE
                        and not special_manual_zero_hold
                        else 0.0
                    )
                    controller.reset(recovery_angle_deg)
                    if args.control_mode == "position":
                        apply_position_local_zero_prior(
                            controller,
                            args.target_cm,
                            target_m,
                            config,
                        )
                if new_measurement_accepted:
                    centered_position_cm = (
                        position_now - coordinate_center_m
                    ) * 100.0
                    if (
                        special_task_phase == "to_first"
                        and directed_point_reached(
                            centered_position_cm,
                            special_task_settings["first_point_cm"],
                            special_first_direction,
                        )
                    ):
                        special_task_phase = "to_final"
                        args.target_cm = special_task_settings[
                            "second_point_cm"
                        ]
                        target_m = control_target_from_centered(
                            args.target_cm, config
                        )
                        target_monitor = create_target_monitor(
                            target_m, config
                        )
                        target_status = None
                        competition_failure_reported = False
                        controller.reset(angle_deg)
                        prior_angle_deg, prior_motor_mm = (
                            apply_position_local_zero_prior(
                                controller,
                                args.target_cm,
                                target_m,
                                config,
                            )
                        )
                        status_message = (
                            "特殊任务第一点到达：位置{:+.3f}cm；"
                            "不等待稳定，立即使用{}.json闭环驶向第二点"
                            "{:+.2f}cm；局部零点先验{:+.3f}mm({:+.3f}°)。"
                        ).format(
                            centered_position_cm,
                            SPECIAL_TASK_CONTROL_PROFILE,
                            args.target_cm,
                            prior_motor_mm,
                            prior_angle_deg,
                        )
                        if control_ui is not None:
                            control_ui.set_target(args.target_cm)
                            control_ui.set_status(status_message)
                        print(status_message, file=sys.stderr)
                    if last_control_measurement_timestamp_s is None:
                        controller_dt = 1.0 / 17.5
                    else:
                        controller_dt = (
                            last_valid_estimate.timestamp_s
                            - last_control_measurement_timestamp_s
                        )
                    last_control_measurement_timestamp_s = (
                        last_valid_estimate.timestamp_s
                    )
                    if (
                        isinstance(controller, ConstrainedMPCController)
                        and args.enable_mpc_dob
                    ):
                        controller.observe_acceleration(
                            last_valid_estimate.acceleration_m_s2,
                            velocity_now,
                            cart_acceleration_m_s2,
                            float(
                                config["acceleration_feedforward"][
                                    "gravity_m_s2"
                                ]
                            ),
                            equilibrium_angle_bias_deg,
                        )
                    # 只在新视觉测量到达时更新控制器；两帧之间保持角度，
                    # 50Hz串口线程只负责重复最新指令。
                    if isinstance(controller, CascadePIDController):
                        combined_feedforward_deg = (
                            acceleration_feedforward_deg
                            + equilibrium_angle_bias_deg
                        )
                        if args.control_mode == "velocity":
                            if (
                                velocity_tracking_active
                                and outward_velocity_edge_reached(
                                    centered_position_cm,
                                    args.target_speed_cm_s,
                                    velocity_edge_abs_cm,
                                )
                            ):
                                velocity_tracking_active = False
                                velocity_waiting_for_detection = False
                                velocity_edge_hold = True
                                status_message = (
                                    "速度模式到达{:+.2f} cm端点保护区："
                                    "锁存最后倾角{:+.3f}°。请先点击"
                                    "“倾斜角返回0”，再点击“启动速度环”。"
                                ).format(centered_position_cm, angle_deg)
                                if control_ui is not None:
                                    control_ui.set_status(status_message)
                                print(status_message, file=sys.stderr)
                            if velocity_tracking_active:
                                angle_deg = controller.update_velocity(
                                    position_now,
                                    velocity_now,
                                    target_velocity_m_s,
                                    controller_dt,
                                    combined_feedforward_deg,
                                )
                            elif not velocity_edge_hold:
                                angle_deg = 0.0
                                controller.rate_limiter.reset(0.0)
                            controller.last_velocity_reference_m_s = (
                                target_velocity_m_s
                            )
                        else:
                            if special_task_phase == "to_first":
                                angle_deg = special_task_settings[
                                    "first_angle_deg"
                                ]
                                controller.rate_limiter.reset(angle_deg)
                                controller.last_velocity_reference_m_s = 0.0
                            elif special_task_phase == "to_final" or (
                                args.special_task == SPECIAL_TASK_NONE
                            ):
                                angle_deg = controller.update(
                                    position_now,
                                    velocity_now,
                                    target_m,
                                    controller_dt,
                                    combined_feedforward_deg,
                                )
                            else:
                                angle_deg = 0.0
                                controller.rate_limiter.reset(0.0)
                        if (
                            controller.local_zero_update_count
                            != last_local_zero_update_count
                        ):
                            last_local_zero_update_count = (
                                controller.local_zero_update_count
                            )
                            calibrated_snap = (
                                controller.last_calibrated_stall_snap_angle_deg
                            )
                            if calibrated_snap is not None:
                                print(
                                    "持续停滞标定角快速刷新：目标{:+.3f}cm，"
                                    "驱动力一次提高到至少{:+.3f}°；若仍停滞"
                                    "将继续按增驱速度增加。".format(
                                        args.target_cm, calibrated_snap
                                    ),
                                    file=sys.stderr,
                                )
                            else:
                                print(
                                    "临时局部角度零点刷新：位置{:+.3f}cm，"
                                    "局部保持偏置{:+.3f}°；静摩擦补偿继续叠加，"
                                    "持续停滞增驱{:+.3f}°。".format(
                                        (
                                            position_now
                                            - coordinate_center_m
                                        )
                                        * 100.0,
                                        controller.local_zero_angle_deg,
                                        controller.stall_drive_boost_angle_deg,
                                    ),
                                    file=sys.stderr,
                                )
                    else:
                        angle_deg = controller.update(
                            position_now,
                            velocity_now,
                            target_m,
                            controller_dt,
                            cart_acceleration_m_s2,
                            float(
                                config["acceleration_feedforward"][
                                    "gravity_m_s2"
                                ]
                            ),
                            equilibrium_angle_bias_deg,
                        )
                    if (
                        args.control_mode == "position"
                        and (
                            args.special_task == SPECIAL_TASK_NONE
                            or special_task_phase == "to_final"
                        )
                    ):
                        target_status = target_monitor.update(
                            position_now, velocity_now, now
                        )
                    else:
                        target_status = None
                    tuning_control_error = 0.0
                    tuning_velocity_reference = 0.0
                    if isinstance(controller, CascadePIDController):
                        tuning_direction = int(
                            controller.approach_direction or 0
                        )
                        tuning_control_target = (
                            target_m
                            - tuning_direction
                            * controller.active_target_offset_m
                        )
                        tuning_control_error = (
                            tuning_control_target - position_now
                        )
                        tuning_velocity_reference = (
                            controller.last_velocity_reference_m_s
                        )
                    if (
                        target_status is not None
                        and target_status.competition_failed
                    ):
                        if not competition_failure_reported:
                            action = (
                                "立即输出0°并退出。"
                                if stop_after_competition_failure
                                else "已锁存失败记录，但继续控制返回目标。"
                            )
                            print(
                                "competition_failure：位置越过比赛±1 cm"
                                "底线，{}当前={:.3f}cm，目标={:.3f}cm。"
                                .format(
                                    action,
                                    (
                                        position_now
                                        - coordinate_center_m
                                    )
                                    * 100.0,
                                    (
                                        target_m
                                        - coordinate_center_m
                                    )
                                    * 100.0,
                                ),
                                file=sys.stderr,
                            )
                            competition_failure_reported = True
                        if stop_after_competition_failure:
                            angle_deg = 0.0
                            controller.reset(0.0)
                            exit_code = 5
                    if (
                        curve_ui is not None
                        and isinstance(controller, CascadePIDController)
                    ):
                        curve_ui.offer(
                            time_s=last_valid_estimate.timestamp_s,
                            target_velocity_cm_s=(
                                controller.last_velocity_reference_m_s * 100.0
                            ),
                            velocity_cm_s=velocity_now * 100.0,
                            target_position_cm=(
                                None
                                if args.control_mode == "velocity"
                                else args.target_cm
                            ),
                            position_cm=(
                                None
                                if args.control_mode == "velocity"
                                else (
                                    position_now - coordinate_center_m
                                )
                                * 100.0
                            ),
                        )
                    if tuning_reporter is not None:
                        tuning_reporter.offer(
                            TuningSample(
                                monotonic_s=now,
                                capture_timestamp_s=(
                                    last_valid_estimate.timestamp_s
                                ),
                                position_m=(
                                    position_now
                                    - coordinate_center_m
                                ),
                                velocity_m_s=velocity_now,
                                target_position_m=(
                                    target_m
                                    - coordinate_center_m
                                ),
                                control_error_m=(
                                    tuning_control_error
                                ),
                                velocity_reference_m_s=(
                                    tuning_velocity_reference
                                ),
                                angle_command_deg=angle_deg,
                                measurement_age_s=measurement_age,
                                processing_latency_ms=(
                                    optional_finite_float(
                                        record,
                                        "processing_latency_ms",
                                    )
                                ),
                                approach_direction=(
                                    target_status.approach_direction
                                ),
                                within_internal_tolerance=(
                                    target_status
                                    .within_internal_tolerance
                                ),
                                settled=target_status.settled,
                                frame=optional_nonnegative_int(
                                    record, "frame"
                                ),
                                accepted_sequence=(
                                    accepted_control_updates
                                ),
                                serial_enabled=sender is not None,
                            )
                        )
            else:
                if (
                    args.special_task != SPECIAL_TASK_NONE
                    and not special_manual_zero_hold
                ):
                    if control_was_valid:
                        print(
                            "特殊任务视觉测量超时；保持最后倾角"
                            "{:+.2f}°，不注入固定偏置。".format(angle_deg),
                            file=sys.stderr,
                        )
                elif args.control_mode == "velocity" and velocity_edge_hold:
                    if control_was_valid:
                        print(
                            "边缘倾角锁存期间视觉测量超时；"
                            "继续保持最后倾角，等待UI返回0°。",
                            file=sys.stderr,
                        )
                else:
                    if control_was_valid:
                        print(
                            "视觉测量超时，控制失效并回到0°。",
                            file=sys.stderr,
                        )
                        # 所有位置闭环在视觉暂时超时时保留稳定计时；
                        # 恢复后的首个有效测量若不稳定，监视器会自行清零。
                    angle_deg = 0.0
                    if (
                        args.control_mode == "velocity"
                        and isinstance(controller, CascadePIDController)
                    ):
                        reset_velocity_state_preserving_local_zero(
                            controller, 0.0
                        )
                        controller.last_velocity_reference_m_s = (
                            target_velocity_m_s
                        )
                    else:
                        controller.reset(angle_deg)
                if (
                    last_valid_receive_s is not None
                    and now - last_valid_receive_s > 1.0
                ):
                    estimator.reset()
                    velocity_filter.reset()
                    last_valid_estimate = None
                    last_valid_receive_s = None
                    last_control_measurement_timestamp_s = None
            control_was_valid = valid_control

            if sender is not None:
                if not valid_control and args.special_task != SPECIAL_TASK_NONE:
                    sender.force_angle(angle_deg)
                else:
                    sender.set_angle(angle_deg)
                sender.raise_if_failed()
            cycle += 1
            if (
                tuning_reporter is None
                and args.print_every > 0
                and cycle % args.print_every == 0
            ):
                print(
                    telemetry_line(
                        cycle,
                        args.controller,
                        target_m,
                        last_valid_estimate,
                        position_now,
                        velocity_now,
                        measurement_age,
                        (
                            sender.latest_angle_deg
                            if sender is not None
                            else angle_deg
                        ),
                        sender is not None,
                        controller,
                        valid_control,
                        new_measurement_accepted and valid_control,
                        target_status,
                        args.telemetry,
                        coordinate_center_m,
                        args.enable_acceleration_feedforward,
                        acceleration_sample_valid,
                        cart_acceleration_m_s2,
                        acceleration_feedforward_deg,
                        equilibrium_angle_bias_deg,
                        args.control_mode,
                        active_positive_motor_scale,
                        active_negative_motor_scale,
                    ),
                    flush=True,
                )
            if tracker_finished:
                if reader_error is not None:
                    print(
                        "钢珠识别读取线程异常：{}".format(reader_error),
                        file=sys.stderr,
                    )
                    exit_code = 4
                else:
                    try:
                        tracker_exit_code = source.wait(timeout=1.0)
                    except (RuntimeError, subprocess.TimeoutExpired) as error:
                        print(
                            "无法确认钢珠识别进程状态：{}".format(error),
                            file=sys.stderr,
                        )
                        exit_code = 4
                    else:
                        if tracker_exit_code != 0:
                            print(
                                "钢珠识别进程异常退出，退出码{}。".format(
                                    tracker_exit_code
                                ),
                                file=sys.stderr,
                            )
                            exit_code = 4
                break
            if (
                target_status is not None
                and target_status.competition_failed
                and stop_after_competition_failure
            ):
                break
        return exit_code
    except (RuntimeError, OSError, ValueError) as error:
        print("控制运行错误：{}".format(error), file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        return 130
    finally:
        if sender is not None:
            try:
                # 先把后台待发内容改为0并完全停线程，再同步发最后一帧，
                # 防止“主线程发0后，后台又补发旧非零角”的退出竞争。
                sender.force_angle(0.0)
                sender.stop()
                sender.send_once()
            except Exception as error:
                print("退出归零发送失败：{}".format(error), file=sys.stderr)
            try:
                sender.close()
            except Exception as error:
                print("关闭串口失败：{}".format(error), file=sys.stderr)
        try:
            source.close()
        except Exception as error:
            print("关闭钢珠识别进程失败：{}".format(error), file=sys.stderr)
        try:
            acceleration_source.close()
        except Exception as error:
            print("关闭加速度输入失败：{}".format(error), file=sys.stderr)
        if reader is not None:
            try:
                reader.join(timeout=1.0)
            except Exception as error:
                print(
                    "等待识别读取线程失败：{}".format(error),
                    file=sys.stderr,
                )
        if tuning_reporter is not None:
            try:
                tuning_reporter.close(timeout_s=0.5)
            except Exception as error:
                print(
                    "关闭调参诊断线程失败：{}".format(error),
                    file=sys.stderr,
                )
        if control_ui is not None:
            try:
                control_ui.close(timeout_s=1.0)
            except Exception as error:
                print(
                    "关闭PID调参窗口失败：{}".format(error),
                    file=sys.stderr,
                )
        if curve_ui is not None:
            try:
                curve_ui.close(timeout_s=1.0)
            except Exception as error:
                print(
                    "关闭实时曲线窗口失败：{}".format(error),
                    file=sys.stderr,
                )


if __name__ == "__main__":
    sys.exit(main())
