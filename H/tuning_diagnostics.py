#!/usr/bin/env python3
"""钢珠闭环实机调参诊断；只观察，不修改控制输出或配置。"""

from collections import deque
from dataclasses import dataclass
import math
import statistics
import sys
import threading
import time
from typing import (
    Any,
    Callable,
    Deque,
    Dict,
    List,
    Optional,
    Sequence,
    Tuple,
)


@dataclass(frozen=True)
class TuningSample:
    monotonic_s: float
    capture_timestamp_s: float
    position_m: float
    velocity_m_s: float
    target_position_m: float
    control_error_m: float
    velocity_reference_m_s: float
    angle_command_deg: float
    measurement_age_s: float
    processing_latency_ms: Optional[float]
    approach_direction: int
    within_internal_tolerance: bool
    settled: bool
    frame: Optional[int] = None
    accepted_sequence: Optional[int] = None
    serial_enabled: bool = False


@dataclass(frozen=True)
class TuningHealthSample:
    """没有形成控制测量时，供后台输出视觉健康心跳。"""

    monotonic_s: float
    reason: str
    frame: Optional[int] = None
    processing_latency_ms: Optional[float] = None


@dataclass(frozen=True)
class DiagnosticEvent:
    code: str
    severity: str
    message: str


@dataclass(frozen=True)
class DiagnosticReport:
    status_line: str
    events: Tuple[DiagnosticEvent, ...]


def percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    index = max(
        0,
        min(
            len(ordered) - 1,
            int(math.ceil(fraction * len(ordered))) - 1,
        ),
    )
    return ordered[index]


def sign_crossings(values: Sequence[float], deadband: float) -> int:
    previous = 0
    crossings = 0
    for value in values:
        sign = 1 if value > deadband else -1 if value < -deadband else 0
        if sign == 0:
            continue
        if previous != 0 and sign != previous:
            crossings += 1
        previous = sign
    return crossings


class TuningDiagnostics:
    """根据短时间运动窗口给出限频、去重的中文调参建议。"""

    def __init__(
        self,
        pid_config: Dict[str, Any],
        safety_config: Dict[str, Any],
        working_angle_limit_deg: float,
        enabled: bool = False,
        expected_fps_min: float = 15.0,
        report_interval_s: float = 2.0,
        window_s: float = 6.0,
        suggestion_cooldown_s: float = 6.0,
    ) -> None:
        if (
            expected_fps_min <= 0.0
            or report_interval_s <= 0.0
            or window_s < report_interval_s
            or suggestion_cooldown_s < 0.0
            or working_angle_limit_deg <= 0.0
        ):
            raise ValueError("调参诊断时序或角度配置无效。")
        self.pid = pid_config
        self.safety = safety_config
        self.working_angle_limit_deg = float(working_angle_limit_deg)
        self.enabled = bool(enabled)
        self.expected_fps_min = float(expected_fps_min)
        self.report_interval_s = float(report_interval_s)
        self.window_s = float(window_s)
        self.suggestion_cooldown_s = float(suggestion_cooldown_s)
        self.samples = deque()  # type: Deque[TuningSample]
        self.last_report_s: Optional[float] = None
        self.last_event_s = {}  # type: Dict[str, float]
        self.event_counts = {}  # type: Dict[str, int]
        self.total_samples = 0
        self.first_capture_s: Optional[float] = None
        self.last_capture_s: Optional[float] = None
        self.first_frame: Optional[int] = None
        self.last_frame: Optional[int] = None
        self.first_accepted_sequence: Optional[int] = None
        self.last_accepted_sequence: Optional[int] = None
        self.minimum_position_m = math.inf
        self.maximum_position_m = -math.inf
        self.maximum_wrong_side_m = 0.0
        self.maximum_measurement_age_s = 0.0
        self.settled_seen = False

    def _event(
        self,
        code: str,
        severity: str,
        message: str,
        now_s: float,
    ) -> Optional[DiagnosticEvent]:
        last = self.last_event_s.get(code)
        if (
            last is not None
            and now_s - last < self.suggestion_cooldown_s
        ):
            return None
        self.last_event_s[code] = now_s
        self.event_counts[code] = self.event_counts.get(code, 0) + 1
        return DiagnosticEvent(code, severity, message)

    def update(
        self, sample: TuningSample
    ) -> Optional[DiagnosticReport]:
        if not self.enabled:
            return None
        values = (
            sample.monotonic_s,
            sample.capture_timestamp_s,
            sample.position_m,
            sample.velocity_m_s,
            sample.target_position_m,
            sample.control_error_m,
            sample.velocity_reference_m_s,
            sample.angle_command_deg,
            sample.measurement_age_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("调参诊断样本必须是有限值。")
        if sample.frame is not None and sample.frame < 0:
            raise ValueError("调参诊断帧号不能为负。")
        if (
            sample.accepted_sequence is not None
            and sample.accepted_sequence < 0
        ):
            raise ValueError("调参诊断有效测量序号不能为负。")

        self.samples.append(sample)
        self.total_samples += 1
        if self.first_capture_s is None:
            self.first_capture_s = sample.capture_timestamp_s
        if self.first_frame is None and sample.frame is not None:
            self.first_frame = sample.frame
        if (
            self.first_accepted_sequence is None
            and sample.accepted_sequence is not None
        ):
            self.first_accepted_sequence = sample.accepted_sequence
        self.last_capture_s = sample.capture_timestamp_s
        if sample.frame is not None:
            self.last_frame = sample.frame
        if sample.accepted_sequence is not None:
            self.last_accepted_sequence = sample.accepted_sequence
        self.minimum_position_m = min(
            self.minimum_position_m, sample.position_m
        )
        self.maximum_position_m = max(
            self.maximum_position_m, sample.position_m
        )
        self.maximum_measurement_age_s = max(
            self.maximum_measurement_age_s,
            max(0.0, sample.measurement_age_s),
        )
        if sample.approach_direction != 0:
            wrong_side = sample.approach_direction * (
                sample.position_m - sample.target_position_m
            )
            self.maximum_wrong_side_m = max(
                self.maximum_wrong_side_m, wrong_side
            )
        self.settled_seen = self.settled_seen or sample.settled

        cutoff = sample.monotonic_s - self.window_s
        while self.samples and self.samples[0].monotonic_s < cutoff:
            self.samples.popleft()
        if self.last_report_s is None:
            self.last_report_s = sample.monotonic_s
            return None
        if (
            sample.monotonic_s - self.last_report_s
            < self.report_interval_s
        ):
            return None
        self.last_report_s = sample.monotonic_s
        return self._build_report(sample.monotonic_s)

    def _build_report(self, now_s: float) -> DiagnosticReport:
        window = list(self.samples)
        latest = window[-1]
        capture_span = (
            window[-1].capture_timestamp_s
            - window[0].capture_timestamp_s
        )
        first_accepted_sequence = window[0].accepted_sequence
        last_accepted_sequence = window[-1].accepted_sequence
        accepted_delta = (
            None
            if first_accepted_sequence is None
            or last_accepted_sequence is None
            or last_accepted_sequence < first_accepted_sequence
            else last_accepted_sequence - first_accepted_sequence
        )
        accepted_count_delta = (
            len(window) - 1
            if accepted_delta is None
            else accepted_delta
        )
        accepted_fps = (
            0.0
            if capture_span <= 0.0 or len(window) < 2
            else accepted_count_delta / capture_span
        )
        first_frame = window[0].frame
        last_frame = window[-1].frame
        frame_delta = (
            None
            if first_frame is None
            or last_frame is None
            or last_frame < first_frame
            else last_frame - first_frame
        )
        tracker_fps = (
            accepted_fps
            if frame_delta is None or frame_delta == 0
            else frame_delta / capture_span
        )
        acceptance_ratio = (
            1.0
            if frame_delta is None or frame_delta == 0
            else min(1.0, accepted_count_delta / frame_delta)
        )
        ages_ms = [
            max(0.0, item.measurement_age_s) * 1000.0
            for item in window
        ]
        processing_ms = [
            float(item.processing_latency_ms)
            for item in window
            if item.processing_latency_ms is not None
            and math.isfinite(float(item.processing_latency_ms))
        ]
        median_age_ms = statistics.median(ages_ms)
        p95_age_ms = percentile(ages_ms, 0.95)
        p95_processing_ms = percentile(processing_ms, 0.95)
        actual_error_m = (
            latest.target_position_m - latest.position_m
        )
        status = (
            "已稳定"
            if latest.settled
            else "内部精度带"
            if latest.within_internal_tolerance
            else "调节中"
        )
        status_line = (
            "[调参监视] 处理/有效FPS={:.1f}/{:.1f} 接受率={:.0%}"
            " 测量年龄={:.0f}/{:.0f}ms"
            " 处理P95={:.0f}ms 位置={:.3f}cm 目标={:.3f}cm"
            " 实际误差={:+.3f}cm 内部误差={:+.3f}cm"
            " 速度={:+.3f}cm/s v_ref={:+.3f}cm/s"
            " 角度={:+.2f}° 状态={}"
        ).format(
            tracker_fps,
            accepted_fps,
            acceptance_ratio,
            median_age_ms,
            p95_age_ms,
            p95_processing_ms,
            latest.position_m * 100.0,
            latest.target_position_m * 100.0,
            actual_error_m * 100.0,
            latest.control_error_m * 100.0,
            latest.velocity_m_s * 100.0,
            latest.velocity_reference_m_s * 100.0,
            latest.angle_command_deg,
            status,
        )

        events = []  # type: List[DiagnosticEvent]

        def add(event: Optional[DiagnosticEvent]) -> None:
            if event is not None:
                events.append(event)

        low_tracker_fps = (
            len(window) >= 8
            and tracker_fps < 0.90 * self.expected_fps_min
        )
        low_acceptance = (
            len(window) >= 8
            and frame_delta is not None
            and frame_delta >= 8
            and acceptance_ratio < 0.70
        )
        high_delay = p95_age_ms > 140.0 or p95_processing_ms > 120.0
        if low_tracker_fps:
            add(
                self._event(
                    "low_vision_fps",
                    "warning",
                    (
                        "识别程序处理仅{:.1f} FPS，先不要提高PID增益。"
                        "优先确认GPU/TensorRT、减小imgsz并使用--no-display；"
                        "控制器会继续按真实帧间隔运行。"
                    ).format(tracker_fps),
                    now_s,
                )
            )
        if low_acceptance:
            add(
                self._event(
                    "low_vision_acceptance",
                    "warning",
                    (
                        "识别处理{:.1f} FPS，但只有{:.1f} FPS进入控制"
                        "（接受率{:.0%}）。检查漏检、深度无效和Kalman"
                        "离群拒绝；不要把它误判为YOLO推理帧率低。"
                    ).format(
                        tracker_fps, accepted_fps, acceptance_ratio
                    ),
                    now_s,
                )
            )
        if high_delay:
            safer_velocity = max(
                0.01,
                float(self.pid["max_velocity_m_s"]) * 0.85,
            )
            add(
                self._event(
                    "high_vision_delay",
                    "warning",
                    (
                        "视觉延迟偏高（测量年龄P95={:.0f}ms，处理P95="
                        "{:.0f}ms）。先优化推理；临时保守调试可把"
                        " max_velocity_m_s 从{:.3f}降到约{:.3f}，"
                        "不要靠加大Kp补延迟。"
                    ).format(
                        p95_age_ms,
                        p95_processing_ms,
                        float(self.pid["max_velocity_m_s"]),
                        safer_velocity,
                    ),
                    now_s,
                )
            )
        capture_gaps = []
        for first, second in zip(window, window[1:]):
            if (
                first.accepted_sequence is not None
                and second.accepted_sequence is not None
                and second.accepted_sequence
                - first.accepted_sequence
                != 1
            ):
                # 后台只保留最新诊断快照；若中间快照被丢弃，不能把
                # 诊断自身丢样误报成视觉断帧。
                continue
            capture_gaps.append(
                second.capture_timestamp_s
                - first.capture_timestamp_s
            )
        if capture_gaps and max(capture_gaps) > 0.18:
            add(
                self._event(
                    "vision_gaps",
                    "warning",
                    (
                        "最近有效控制测量最大间隔{:.0f}ms，已接近"
                        "0.25s归零门限。先排查漏检、GPU内存、CPU回退"
                        "和相机占用，不建议此时调PID。"
                    ).format(max(capture_gaps) * 1000.0),
                    now_s,
                )
            )
        vision_unhealthy = (
            low_tracker_fps
            or low_acceptance
            or high_delay
            or (capture_gaps and max(capture_gaps) > 0.18)
        )

        duration_s = window[-1].monotonic_s - window[0].monotonic_s
        desired_direction = (
            1
            if latest.velocity_reference_m_s > 0.0
            else -1
            if latest.velocity_reference_m_s < 0.0
            else latest.approach_direction
        )
        strong_intent_directions = [
            1
            if item.velocity_reference_m_s > 0.0
            else -1
            for item in window
            if abs(item.velocity_reference_m_s) >= 0.008
        ]
        consistent_motion_intent = (
            desired_direction != 0
            and len(strong_intent_directions) >= 6
            and sum(
                value == desired_direction
                for value in strong_intent_directions
            )
            / len(strong_intent_directions)
            >= 0.80
        )
        progress_direction = (
            desired_direction
            if consistent_motion_intent
            else 1
            if window[0].control_error_m > 0.0
            else -1
            if window[0].control_error_m < 0.0
            else 0
        )
        progress_m = progress_direction * (
            window[-1].position_m - window[0].position_m
        )
        progress_speeds = [
            progress_direction * item.velocity_m_s for item in window
        ]
        median_progress_speed = statistics.median(progress_speeds)
        median_speed_magnitude = statistics.median(
            [abs(value) for value in progress_speeds]
        )
        median_vref = statistics.median(
            [abs(item.velocity_reference_m_s) for item in window]
        )
        max_angle = max(abs(item.angle_command_deg) for item in window)
        min_control_error = min(
            abs(item.control_error_m) for item in window
        )
        static_now = float(
            self.pid["static_friction_compensation_deg"]
        )
        ramp_now = float(
            self.pid["static_compensation_ramp_deg_s"]
        )
        no_roll_wait_s = max(
            1.5,
            static_now / max(ramp_now, 1e-9) + 0.5,
        )
        if (
            latest.serial_enabled
            and not vision_unhealthy
            and consistent_motion_intent
            and duration_s >= no_roll_wait_s
            and min_control_error
            > max(
                0.012,
                float(self.pid["static_compensation_min_error_m"]),
            )
            and median_vref >= 0.008
            and median_speed_magnitude <= 0.005
            and abs(progress_m) <= 0.002
            and max_angle >= 1.5
        ):
            static_ceiling = max(
                0.0, self.working_angle_limit_deg - 1.0
            )
            static_next = min(static_ceiling, static_now + 0.5)
            if static_next <= static_now:
                static_suggestion = (
                    "当前工作限角内没有继续增加静摩擦补偿的安全余量，"
                    "先做固定小角度起滚试验"
                )
            else:
                static_suggestion = (
                    "static_friction_compensation_deg "
                    "{:.1f}→{:.1f}"
                ).format(static_now, static_next)
            add(
                self._event(
                    "stiction",
                    "warning",
                    (
                        "{:.1f}s内球仅沿目标方向移动{:.2f}cm，疑似起滚"
                        "不足。先确认管道和符号；若机械正常，每次只改一项："
                        " {}，或 ramp {:.1f}→{:.1f}°/s。"
                    ).format(
                        duration_s,
                        progress_m * 100.0,
                        static_suggestion,
                        ramp_now,
                        ramp_now + 1.0,
                    ),
                    now_s,
                )
            )
        if (
            latest.serial_enabled
            and not vision_unhealthy
            and consistent_motion_intent
            and duration_s >= 1.5
            and abs(window[0].velocity_m_s)
            <= float(self.safety["settle_velocity_m_s"])
            and median_vref >= 0.008
            and median_progress_speed < -0.004
            and progress_m < -0.003
        ):
            add(
                self._event(
                    "direction_mismatch",
                    "danger",
                    (
                        "球持续向目标反方向移动{:.2f}cm。立即停止实机，"
                        "不要调增益；检查“正角使x减小”、零点和串口符号。"
                    ).format(abs(progress_m) * 100.0),
                    now_s,
                )
            )

        requested_error_m = abs(actual_error_m)
        speed_toward_failure = max(
            0.0,
            latest.approach_direction * latest.velocity_m_s,
        )
        signed_wrong_side = latest.approach_direction * (
            latest.position_m - latest.target_position_m
        )
        remaining_to_failure_m = (
            float(self.safety["competition_tolerance_m"])
            - signed_wrong_side
        )
        assumed_brake = max(
            0.1, float(self.pid["braking_accel_m_s2"])
        )
        predicted_travel_m = (
            speed_toward_failure
            * (max(0.0, latest.measurement_age_s) + 0.10)
            + speed_toward_failure ** 2 / (2.0 * assumed_brake)
        )
        approach_too_fast = (
            requested_error_m <= 0.015
            and speed_toward_failure >= 0.025
        )
        boundary_risk = (
            speed_toward_failure > 0.0
            and remaining_to_failure_m > 0.0
            and predicted_travel_m >= 0.80 * remaining_to_failure_m
        )
        if latest.serial_enabled and (
            approach_too_fast or boundary_risk
        ):
            velocity_now = float(self.pid["max_velocity_m_s"])
            velocity_next = max(0.01, velocity_now * 0.85)
            brake_now = float(self.pid["braking_accel_m_s2"])
            add(
                self._event(
                    "approach_too_fast",
                    "danger",
                    (
                        "接近目标速度{:.2f}cm/s，按当前延迟估计仍会前进"
                        "{:.2f}cm，1cm边界余量{:.2f}cm。下一次试验先把"
                        " max_velocity_m_s {:.3f}→约{:.3f}；若仍快，再将"
                        " braking_accel_m_s2 {:.2f}下调10%，"
                        "每次只改一项。当前预停偏置已等于二阶段解锁"
                        "误差，不能继续增加。"
                    ).format(
                        speed_toward_failure * 100.0,
                        predicted_travel_m * 100.0,
                        remaining_to_failure_m * 100.0,
                        velocity_now,
                        velocity_next,
                        brake_now,
                    ),
                    now_s,
                )
            )

        oscillation_window = [
            item
            for item in window
            if item.monotonic_s >= now_s - 5.0
        ]
        oscillation_errors = [
            item.control_error_m for item in oscillation_window
        ]
        oscillation_deadband_m = max(
            float(self.safety["internal_tolerance_m"]),
            float(self.pid["position_deadband_m"]),
        )
        crossings = sign_crossings(
            oscillation_errors, oscillation_deadband_m
        )
        position_span_m = max(
            item.position_m for item in oscillation_window
        ) - min(item.position_m for item in oscillation_window)
        midpoint = max(1, len(oscillation_errors) // 2)
        first_amplitude_m = max(
            [abs(value) for value in oscillation_errors[:midpoint]]
            or [0.0]
        )
        second_amplitude_m = max(
            [abs(value) for value in oscillation_errors[midpoint:]]
            or [0.0]
        )
        amplitude_ratio = second_amplitude_m / max(
            first_amplitude_m, 1e-9
        )
        oscillation_duration_s = (
            oscillation_window[-1].monotonic_s
            - oscillation_window[0].monotonic_s
        )
        if (
            latest.serial_enabled
            and oscillation_duration_s >= 4.0
            and crossings >= 4
            and max(oscillation_errors) > oscillation_deadband_m
            and min(oscillation_errors) < -oscillation_deadband_m
            and position_span_m >= 0.006
            and amplitude_ratio > 0.75
            and not latest.settled
            and not vision_unhealthy
        ):
            position_kp = float(self.pid["position_kp_s_inv"])
            velocity_ki = float(self.pid["velocity_ki_deg_per_m"])
            add(
                self._event(
                    "target_oscillation",
                    "warning",
                    (
                        "{:.1f}s内内部误差换向{}次、摆幅{:.2f}cm，存在"
                        "持续振荡（后/前振幅比{:.0%}）。先把"
                        " position_kp_s_inv {:.2f}→约{:.2f}；"
                        "若角度仍频繁换向，再把 velocity_ki_deg_per_m"
                        " {:.1f}下调约15%，不要同时修改。"
                    ).format(
                        oscillation_duration_s,
                        crossings,
                        position_span_m * 100.0,
                        amplitude_ratio,
                        position_kp,
                        position_kp * 0.85,
                        velocity_ki,
                    ),
                    now_s,
                )
            )

        if (
            latest.serial_enabled
            and not vision_unhealthy
            and consistent_motion_intent
            and duration_s >= 3.0
            and min_control_error >= 0.010
            and progress_m > 0.002
            and median_vref >= 0.010
            and median_progress_speed > 0.0
            and median_progress_speed < 0.35 * median_vref
            and max_angle >= 1.0
        ):
            velocity_ki = float(self.pid["velocity_ki_deg_per_m"])
            add(
                self._event(
                    "response_slow",
                    "info",
                    (
                        "运动方向正确但实际速度仅约参考速度的{:.0%}。"
                        "先做±角度滚动辨识；若确认是稳定动摩擦，可将"
                        " velocity_ki_deg_per_m {:.1f}提高约10%，"
                        "每次试验后重新检查过冲。"
                    ).format(
                        median_progress_speed
                        / max(median_vref, 1e-9),
                        velocity_ki,
                    ),
                    now_s,
                )
            )
        return DiagnosticReport(status_line, tuple(events))

    def summary_line(self) -> str:
        if not self.enabled:
            return "[调参汇总] 调参诊断未启用。"
        if (
            self.total_samples < 2
            or self.first_capture_s is None
            or self.last_capture_s is None
            or self.last_capture_s <= self.first_capture_s
        ):
            return "[调参汇总] 有效视觉样本不足，无法形成调参结论。"
        accepted_count_delta = self.total_samples - 1
        if (
            self.first_accepted_sequence is not None
            and self.last_accepted_sequence is not None
            and self.last_accepted_sequence
            >= self.first_accepted_sequence
        ):
            accepted_count_delta = (
                self.last_accepted_sequence
                - self.first_accepted_sequence
            )
        accepted_fps = accepted_count_delta / (
            self.last_capture_s - self.first_capture_s
        )
        tracker_fps = accepted_fps
        acceptance_ratio = 1.0
        if (
            self.first_frame is not None
            and self.last_frame is not None
            and self.last_frame > self.first_frame
        ):
            frame_delta = self.last_frame - self.first_frame
            tracker_fps = frame_delta / (
                self.last_capture_s - self.first_capture_s
            )
            acceptance_ratio = min(
                1.0, accepted_count_delta / frame_delta
            )
        event_text = (
            "无重复告警"
            if not self.event_counts
            else ",".join(
                "{}×{}".format(code, count)
                for code, count in sorted(self.event_counts.items())
            )
        )
        return (
            "[调参汇总] 样本={} 处理/有效FPS={:.1f}/{:.1f}"
            " 接受率={:.0%} 位置范围="
            "{:.2f}～{:.2f}cm 最大错误侧越过={:.3f}cm"
            " 最大测量年龄={:.0f}ms 曾稳定={} 建议事件={}"
        ).format(
            self.total_samples,
            tracker_fps,
            accepted_fps,
            acceptance_ratio,
            self.minimum_position_m * 100.0,
            self.maximum_position_m * 100.0,
            self.maximum_wrong_side_m * 100.0,
            self.maximum_measurement_age_s * 1000.0,
            "是" if self.settled_seen else "否",
            event_text,
        )


class TuningDebugReporter:
    """后台处理和打印诊断，终端再慢也不反压控制线程。"""

    def __init__(
        self,
        diagnostics: TuningDiagnostics,
        emit: Optional[Callable[[str], None]] = None,
    ) -> None:
        if not diagnostics.enabled:
            raise ValueError("后台调参输出器只应在诊断启用时创建。")
        self.diagnostics = diagnostics
        self.emit = emit or self._emit_stderr
        self.condition = threading.Condition()
        self.pending: Optional[Any] = None
        self.stopping = False
        self.started = False
        self.dropped_samples = 0
        self.debug_errors = 0
        self.start_monotonic_s = time.perf_counter()
        self.last_observation_s: Optional[float] = None
        self.last_valid_sample_s: Optional[float] = None
        self.last_health_emit_s = self.start_monotonic_s
        self.last_health_reason = "尚未收到识别记录"
        self.last_health_frame: Optional[int] = None
        self.last_health_processing_ms: Optional[float] = None
        self.thread = threading.Thread(
            target=self._run,
            name="ball-tuning-debug",
            daemon=True,
        )

    @staticmethod
    def _emit_stderr(line: str) -> None:
        print(line, file=sys.stderr, flush=True)

    def start(self) -> None:
        with self.condition:
            if self.started:
                return
            self.started = True
        self.thread.start()

    def _offer_latest(self, sample: Any) -> bool:
        try:
            with self.condition:
                if self.stopping:
                    return False
                if self.pending is not None:
                    self.dropped_samples += 1
                self.pending = sample
                self.condition.notify()
            return True
        except Exception:
            with self.condition:
                self.debug_errors += 1
            return False

    def offer(self, sample: TuningSample) -> bool:
        """提交有效控制样本；只替换最新值，不排队。"""

        return self._offer_latest(sample)

    def offer_health(self, sample: TuningHealthSample) -> bool:
        """提交无效识别状态，供后台心跳解释为何没有控制样本。"""

        return self._offer_latest(sample)

    def _safe_emit(self, line: str) -> None:
        try:
            self.emit(line)
        except Exception:
            with self.condition:
                self.debug_errors += 1

    def _run(self) -> None:
        while True:
            with self.condition:
                while self.pending is None and not self.stopping:
                    self.condition.wait(
                        timeout=min(
                            0.5, self.diagnostics.report_interval_s
                        )
                    )
                    if self.pending is None:
                        break
                if self.pending is None and self.stopping:
                    break
                sample = self.pending
                self.pending = None
            if isinstance(sample, TuningSample):
                self.last_observation_s = sample.monotonic_s
                self.last_valid_sample_s = sample.monotonic_s
                try:
                    report = self.diagnostics.update(sample)
                except Exception:
                    with self.condition:
                        self.debug_errors += 1
                    report = None
                if report is not None:
                    self._safe_emit(report.status_line)
                    severity_names = {
                        "danger": "严重",
                        "warning": "警告",
                        "info": "提示",
                    }
                    for event in sorted(
                        report.events,
                        key=lambda item: {
                            "danger": 0,
                            "warning": 1,
                            "info": 2,
                        }.get(item.severity, 3),
                    ):
                        self._safe_emit(
                            "[调参建议][{}][{}] {}".format(
                                severity_names.get(
                                    event.severity,
                                    event.severity,
                                ),
                                event.code,
                                event.message,
                            )
                        )
            elif isinstance(sample, TuningHealthSample):
                self.last_observation_s = sample.monotonic_s
                self.last_health_reason = sample.reason
                self.last_health_frame = sample.frame
                self.last_health_processing_ms = (
                    sample.processing_latency_ms
                )
            elif sample is not None:
                with self.condition:
                    self.debug_errors += 1

            current_s = time.perf_counter()
            last_valid_s = (
                self.start_monotonic_s
                if self.last_valid_sample_s is None
                else self.last_valid_sample_s
            )
            if (
                not self.stopping
                and current_s - last_valid_s
                >= self.diagnostics.report_interval_s
                and current_s - self.last_health_emit_s
                >= self.diagnostics.report_interval_s
            ):
                self.last_health_emit_s = current_s
                silence_s = (
                    current_s - self.start_monotonic_s
                    if self.last_observation_s is None
                    else current_s - self.last_observation_s
                )
                frame_text = (
                    "未知"
                    if self.last_health_frame is None
                    else str(self.last_health_frame)
                )
                processing_text = (
                    "未知"
                    if self.last_health_processing_ms is None
                    else "{:.0f}ms".format(
                        self.last_health_processing_ms
                    )
                )
                reason = (
                    "{}；识别进程已{:.1f}s没有新记录".format(
                        self.last_health_reason, silence_s
                    )
                    if silence_s
                    >= self.diagnostics.report_interval_s
                    else self.last_health_reason
                )
                self._safe_emit(
                    "[调参监视] 暂无有效球位置，最近帧={} 原因={} "
                    "处理耗时={}；控制器保持/回到0°，先解决视觉。"
                    .format(frame_text, reason, processing_text)
                )
        self._safe_emit(
            "{} 后台丢弃旧样本={} 诊断异常={}".format(
                self.diagnostics.summary_line(),
                self.dropped_samples,
                self.debug_errors,
            )
        )

    def close(self, timeout_s: float = 0.5) -> bool:
        """请求汇总并限时等待；返回后台线程是否已正常退出。"""

        with self.condition:
            if not self.started:
                return True
            self.stopping = True
            self.condition.notify()
        self.thread.join(timeout=max(0.0, float(timeout_s)))
        return not self.thread.is_alive()
