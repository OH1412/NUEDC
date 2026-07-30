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

from angle_serial import PeriodicAngleSender, encode_angle
from ball_control import (
    CascadePIDController,
    CompetitionTargetMonitor,
    ConstrainedMPCController,
    KinematicEstimate,
    KinematicKalmanFilter,
    ball_position_from_zero,
)
from ball_tracker_source import BallTrackerSource, base_point_from_record
from tuning_diagnostics import (
    TuningDebugReporter,
    TuningDiagnostics,
    TuningHealthSample,
    TuningSample,
)


H_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = H_DIR / "ball_control_config.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="识别钢珠并输出目标管道倾角；默认只计算，不发送串口"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--target-cm",
        type=float,
        required=True,
        help="目标位置，0为固定端，25为电机端",
    )
    parser.add_argument(
        "--controller",
        choices=("cascade_pid", "mpc"),
        default="cascade_pid",
    )
    parser.add_argument(
        "--working-angle-limit-deg",
        type=float,
        default=10.0,
        help="初次调试工作限角；验证后可逐步增至物理上限30度",
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
    )
    for key in required_positive:
        if float(data[key]) <= 0:
            raise ValueError("{}必须大于0。".format(key))
    if float(data["serial_rate_hz"]) < 20.0:
        raise ValueError("串口发送频率必须至少20 Hz。")
    return data


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
) -> str:
    encoded_angle = encode_angle(angle_deg)
    quantized_angle_deg = (
        int(encoded_angle[2]) + int(encoded_angle[3]) / 100.0
    )
    if encoded_angle[1] == 0x01:
        quantized_angle_deg = -quantized_angle_deg
    if mode == "compact":
        position_cm = (
            None
            if position_now is None
            else round(float(position_now) * 100.0, 3)
        )
        error_cm = (
            None
            if position_now is None
            else round(
                (float(target_m) - float(position_now)) * 100.0, 3
            )
        )
        compact_payload: Dict[str, Any] = {
            "valid": valid_control,
            "target_cm": round(target_m * 100.0, 3),
            "position_cm": position_cm,
            "error_cm": error_cm,
            "velocity_cm_s": (
                None
                if velocity_now is None
                else round(float(velocity_now) * 100.0, 3)
            ),
            "command_deg": round(quantized_angle_deg, 2),
        }
        return json.dumps(
            compact_payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )

    payload: Dict[str, Any] = {
        "cycle": cycle,
        "valid_control": valid_control,
        "control_updated_from_new_vision": control_updated,
        "controller": controller_name,
        "target_cm": round(target_m * 100.0, 3),
        "angle_command_deg": round(angle_deg, 3),
        "serial_payload_hex": encode_angle(angle_deg).hex(" "),
        "serial_enabled": serial_enabled,
    }
    if (
        estimate is not None
        and position_now is not None
        and velocity_now is not None
        and measurement_age_s is not None
    ):
        payload.update(
            {
                "position_cm": round(float(position_now) * 100.0, 3),
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
                        target_status.failure_boundary_m * 100.0, 3
                    )
                ),
            }
        )
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def main() -> int:
    args = parse_args()
    config_path = args.config.expanduser().resolve()
    try:
        config = load_config(config_path)
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
        target_m = args.target_cm / 100.0
        pipe_length_m = float(config["pipe_length_m"])
        if not 0.0 <= target_m <= pipe_length_m:
            raise ValueError(
                "目标位置必须在0～{:.1f} cm。".format(
                    pipe_length_m * 100.0
                )
            )
        physical_limit = min(
            abs(float(config["angle_min_deg"])),
            abs(float(config["angle_max_deg"])),
        )
        if not 0 < args.working_angle_limit_deg <= physical_limit:
            raise ValueError(
                "工作限角必须在(0,{:.1f}]度。".format(physical_limit)
            )
        if args.print_every < 0:
            raise ValueError("print-every不能为负。")
        if args.tuning_debug and args.controller != "cascade_pid":
            raise ValueError(
                "--tuning-debug当前只为cascade_pid提供参数建议；"
                "MPC请先保持只试算。"
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
    angle_min = -args.working_angle_limit_deg
    angle_max = args.working_angle_limit_deg
    if args.controller == "cascade_pid":
        controller: Any = CascadePIDController(
            config["cascade_pid"],
            angle_min,
            angle_max,
            float(config["max_angle_step_deg"]),
            config["safety"],
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
    target_monitor = CompetitionTargetMonitor(
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
    source = BallTrackerSource(args.tracker_args)
    latest = LatestRecord()
    reader: Optional[threading.Thread] = None
    angle_deg = 0.0
    competition_failure_reported = False
    try:
        if args.enable_serial:
            sender = PeriodicAngleSender.open(
                port=args.port or config["serial_port"],
                baudrate=args.baud or int(config["serial_baud"]),
                rate_hz=float(config["serial_rate_hz"]),
                initial_angle_deg=0.0,
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
        print(
            "时序：视觉新帧约{:.0f}～{:.0f} FPS才更新控制器；"
            "{:.0f} Hz监督超时；串口独立{:.0f} Hz重发最新倾角。"
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
                            pipe_length_m,
                            tolerance_m=0.03,
                        )
                        estimate = estimator.update(
                            position, capture_timestamp_s
                        )
                        if estimate.measurement_accepted:
                            last_valid_estimate = estimate
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
                    controller.reset(0.0)
                if new_measurement_accepted:
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
                        )
                    # 只在新视觉测量到达时更新控制器；两帧之间保持角度，
                    # 50Hz串口线程只负责重复最新指令。
                    angle_deg = controller.update(
                        position_now,
                        velocity_now,
                        target_m,
                        controller_dt,
                    )
                    target_status = target_monitor.update(
                        position_now, velocity_now, now
                    )
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
                    if target_status.competition_failed:
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
                                    position_now * 100.0,
                                    target_m * 100.0,
                                ),
                                file=sys.stderr,
                            )
                            competition_failure_reported = True
                        if stop_after_competition_failure:
                            angle_deg = 0.0
                            controller.reset(0.0)
                            exit_code = 5
                    if tuning_reporter is not None:
                        tuning_reporter.offer(
                            TuningSample(
                                monotonic_s=now,
                                capture_timestamp_s=(
                                    last_valid_estimate.timestamp_s
                                ),
                                position_m=position_now,
                                velocity_m_s=velocity_now,
                                target_position_m=target_m,
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
                if control_was_valid:
                    print(
                        "视觉测量超时，控制失效并回到0°。",
                        file=sys.stderr,
                    )
                    target_monitor.clear_settle_timer()
                angle_deg = 0.0
                controller.reset(0.0)
                if (
                    last_valid_receive_s is not None
                    and now - last_valid_receive_s > 1.0
                ):
                    estimator.reset()
                    last_valid_estimate = None
                    last_valid_receive_s = None
                    last_control_measurement_timestamp_s = None
            control_was_valid = valid_control

            if sender is not None:
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
                        angle_deg,
                        sender is not None,
                        controller,
                        valid_control,
                        new_measurement_accepted and valid_control,
                        target_status,
                        args.telemetry,
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
                sender.set_angle(0.0)
                sender.stop()
                sender.send_once(0.0)
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


if __name__ == "__main__":
    sys.exit(main())
