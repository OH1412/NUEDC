#!/usr/bin/env python3
"""钢珠一维状态估计、串级PID和轻量约束MPC。"""

from dataclasses import dataclass
import math
import time
from typing import Any, Dict, Optional, Sequence

import numpy as np


GRAVITY_M_S2 = 9.81


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(float(value), float(lower)), float(upper))


def ball_position_from_zero(
    point_base_m: Sequence[float],
    zero_point_base_m: Sequence[float],
    pipe_length_m: float = 0.25,
    tolerance_m: float = 0.03,
) -> float:
    """返回球心与固定零点的三维距离，即沿管位置。"""
    point = np.asarray(point_base_m, dtype=np.float64)
    zero = np.asarray(zero_point_base_m, dtype=np.float64)
    if point.shape != (3,) or zero.shape != (3,):
        raise ValueError("球心和零点都必须是3元素坐标。")
    if not np.all(np.isfinite(point)) or not np.all(np.isfinite(zero)):
        raise ValueError("球心和零点坐标必须是有限值。")
    if pipe_length_m <= 0 or tolerance_m < 0:
        raise ValueError("管长必须为正，容差不能为负。")
    position = float(np.linalg.norm(point - zero))
    if position > pipe_length_m + tolerance_m:
        raise ValueError(
            "球心距零点{:.3f}m，超过管长和容差之和{:.3f}m。".format(
                position, pipe_length_m + tolerance_m
            )
        )
    return clamp(position, 0.0, pipe_length_m)


@dataclass(frozen=True)
class KinematicEstimate:
    position_m: float
    velocity_m_s: float
    acceleration_m_s2: float
    timestamp_s: float
    measurement_accepted: bool


@dataclass(frozen=True)
class TargetStatus:
    error_m: float
    approach_direction: int
    failure_boundary_m: Optional[float]
    competition_failed: bool
    within_internal_tolerance: bool
    settled: bool
    settled_duration_s: float


class CompetitionTargetMonitor:
    """区分更严格的内部目标和不可越过的比赛±1cm底线。"""

    def __init__(
        self,
        target_position_m: float,
        internal_tolerance_m: float = 0.003,
        competition_tolerance_m: float = 0.01,
        settle_velocity_m_s: float = 0.008,
        settle_time_s: float = 0.5,
    ) -> None:
        if (
            internal_tolerance_m <= 0
            or competition_tolerance_m <= internal_tolerance_m
            or settle_velocity_m_s <= 0
            or settle_time_s <= 0
        ):
            raise ValueError("目标精度或稳定判据配置无效。")
        self.target_position_m = float(target_position_m)
        self.internal_tolerance_m = float(internal_tolerance_m)
        self.competition_tolerance_m = float(competition_tolerance_m)
        self.settle_velocity_m_s = float(settle_velocity_m_s)
        self.settle_time_s = float(settle_time_s)
        self.approach_direction: Optional[int] = None
        self.competition_failed = False
        self.settle_started_s: Optional[float] = None

    def reset(self) -> None:
        self.approach_direction = None
        self.competition_failed = False
        self.settle_started_s = None

    def clear_settle_timer(self) -> None:
        """失去可信测量时取消“连续稳定”，保留方向和失败锁存。"""

        self.settle_started_s = None

    def update(
        self, position_m: float, velocity_m_s: float, timestamp_s: float
    ) -> TargetStatus:
        position = float(position_m)
        velocity = float(velocity_m_s)
        timestamp = float(timestamp_s)
        if not all(math.isfinite(v) for v in (position, velocity, timestamp)):
            raise ValueError("目标监视器输入必须是有限值。")
        error = self.target_position_m - position
        if self.approach_direction is None:
            if error > self.internal_tolerance_m:
                self.approach_direction = 1
            elif error < -self.internal_tolerance_m:
                self.approach_direction = -1
            else:
                self.approach_direction = 0

        boundary: Optional[float]
        if self.approach_direction > 0:
            boundary = (
                self.target_position_m + self.competition_tolerance_m
            )
            if position > boundary:
                self.competition_failed = True
        elif self.approach_direction < 0:
            boundary = (
                self.target_position_m - self.competition_tolerance_m
            )
            if position < boundary:
                self.competition_failed = True
        else:
            boundary = None
            if abs(error) > self.competition_tolerance_m:
                self.competition_failed = True

        within = abs(error) <= self.internal_tolerance_m
        stable_now = within and abs(velocity) <= self.settle_velocity_m_s
        if stable_now:
            if self.settle_started_s is None:
                self.settle_started_s = timestamp
        else:
            self.settle_started_s = None
        settled_duration = (
            0.0
            if self.settle_started_s is None
            else max(0.0, timestamp - self.settle_started_s)
        )
        return TargetStatus(
            error_m=error,
            approach_direction=int(self.approach_direction),
            failure_boundary_m=boundary,
            competition_failed=self.competition_failed,
            within_internal_tolerance=within,
            settled=settled_duration >= self.settle_time_s,
            settled_duration_s=settled_duration,
        )


class KinematicKalmanFilter:
    """常加速度Kalman滤波器，视觉仅测位置，输出x/v/a。"""

    def __init__(
        self,
        measurement_std_m: float = 0.003,
        jerk_std_m_s3: float = 2.0,
        outlier_gate_sigma: float = 5.0,
    ) -> None:
        if measurement_std_m <= 0 or jerk_std_m_s3 <= 0:
            raise ValueError("估计器噪声参数必须大于0。")
        if outlier_gate_sigma < 2:
            raise ValueError("异常值门限至少为2 sigma。")
        self.measurement_variance = measurement_std_m ** 2
        self.jerk_variance = jerk_std_m_s3 ** 2
        self.outlier_gate_sigma = outlier_gate_sigma
        self.state: Optional[np.ndarray] = None
        self.covariance: Optional[np.ndarray] = None
        self.timestamp_s: Optional[float] = None

    def reset(self) -> None:
        self.state = None
        self.covariance = None
        self.timestamp_s = None

    def update(
        self, position_m: float, timestamp_s: float
    ) -> KinematicEstimate:
        position_m = float(position_m)
        timestamp_s = float(timestamp_s)
        if not math.isfinite(position_m) or not math.isfinite(timestamp_s):
            raise ValueError("位置和时间戳必须是有限值。")
        if self.state is None:
            self.state = np.array([position_m, 0.0, 0.0])
            self.covariance = np.diag([self.measurement_variance, 0.25, 4.0])
            self.timestamp_s = timestamp_s
            return self.estimate(True)

        assert self.covariance is not None
        assert self.timestamp_s is not None
        dt = timestamp_s - self.timestamp_s
        if dt <= 0:
            return self.estimate(False)
        # 防止暂停后用过大的dt把协方差和状态一次推飞。
        dt = clamp(dt, 0.005, 0.20)
        transition = np.array(
            [
                [1.0, dt, 0.5 * dt * dt],
                [0.0, 1.0, dt],
                [0.0, 0.0, 1.0],
            ]
        )
        # 连续白噪声jerk的精确离散过程噪声。
        process_noise = self.jerk_variance * np.array(
            [
                [dt ** 5 / 20.0, dt ** 4 / 8.0, dt ** 3 / 6.0],
                [dt ** 4 / 8.0, dt ** 3 / 3.0, dt ** 2 / 2.0],
                [dt ** 3 / 6.0, dt ** 2 / 2.0, dt],
            ]
        )
        process_noise += np.diag([1e-10, 1e-8, 1e-6])
        self.state = transition.dot(self.state)
        self.covariance = (
            transition.dot(self.covariance).dot(transition.T)
            + process_noise
        )
        self.timestamp_s = timestamp_s

        innovation = position_m - float(self.state[0])
        innovation_variance = (
            float(self.covariance[0, 0]) + self.measurement_variance
        )
        gate_m = max(
            0.012,
            self.outlier_gate_sigma * math.sqrt(innovation_variance),
        )
        if abs(innovation) > gate_m:
            return self.estimate(False)

        gain = self.covariance[:, 0] / innovation_variance
        self.state = self.state + gain * innovation
        identity = np.eye(3)
        observation = np.array([[1.0, 0.0, 0.0]])
        # Joseph形式避免浮点误差使协方差失去半正定性。
        correction = identity - np.outer(gain, observation[0])
        self.covariance = (
            correction.dot(self.covariance).dot(correction.T)
            + self.measurement_variance * np.outer(gain, gain)
        )
        return self.estimate(True)

    def estimate(self, accepted: bool) -> KinematicEstimate:
        if self.state is None or self.timestamp_s is None:
            raise RuntimeError("估计器尚未初始化。")
        return KinematicEstimate(
            position_m=float(self.state[0]),
            velocity_m_s=float(self.state[1]),
            acceleration_m_s2=float(self.state[2]),
            timestamp_s=float(self.timestamp_s),
            measurement_accepted=accepted,
        )


class AngleRateLimiter:
    def __init__(
        self,
        minimum_deg: float,
        maximum_deg: float,
        max_step_deg: float,
    ) -> None:
        if not minimum_deg < maximum_deg or max_step_deg <= 0:
            raise ValueError("角度或角度变化限制无效。")
        self.minimum_deg = float(minimum_deg)
        self.maximum_deg = float(maximum_deg)
        self.max_step_deg = float(max_step_deg)
        self.value_deg = 0.0

    def reset(self, value_deg: float = 0.0) -> None:
        self.value_deg = clamp(
            value_deg, self.minimum_deg, self.maximum_deg
        )

    def update(self, requested_deg: float) -> float:
        requested = clamp(
            requested_deg, self.minimum_deg, self.maximum_deg
        )
        change = clamp(
            requested - self.value_deg,
            -self.max_step_deg,
            self.max_step_deg,
        )
        self.value_deg = clamp(
            self.value_deg + change, self.minimum_deg, self.maximum_deg
        )
        return self.value_deg


class CascadePIDController:
    """外环位置到速度、内环速度到角度的串级PI控制器。"""

    def __init__(
        self,
        config: Dict[str, Any],
        angle_min_deg: float,
        angle_max_deg: float,
        max_angle_step_deg: float,
        safety_config: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.config = config
        self.safety = safety_config or {}
        self.angle_min_deg = float(angle_min_deg)
        self.angle_max_deg = float(angle_max_deg)
        self.rate_limiter = AngleRateLimiter(
            angle_min_deg, angle_max_deg, max_angle_step_deg
        )
        self.outer_integral = 0.0
        self.inner_integral = 0.0
        self.last_velocity_reference_m_s = 0.0
        self.last_raw_angle_deg = 0.0
        self.static_compensation_deg = 0.0
        self.approach_direction: Optional[int] = None
        self.active_target_offset_m = 0.0
        self.target_refinement_dwell_s = 0.0
        self.target_refinement_unlocked = False

    def reset(self, angle_deg: float = 0.0) -> None:
        self.outer_integral = 0.0
        self.inner_integral = 0.0
        self.last_velocity_reference_m_s = 0.0
        self.last_raw_angle_deg = 0.0
        self.static_compensation_deg = 0.0
        self.approach_direction = None
        self.active_target_offset_m = 0.0
        self.target_refinement_dwell_s = 0.0
        self.target_refinement_unlocked = False
        self.rate_limiter.reset(angle_deg)

    def update(
        self,
        position_m: float,
        velocity_m_s: float,
        target_position_m: float,
        dt_s: float,
    ) -> float:
        dt = clamp(dt_s, 0.005, 0.20)
        requested_position_error = float(
            target_position_m - position_m
        )
        if self.approach_direction is None:
            direction_threshold = float(
                self.safety.get("internal_tolerance_m", 0.003)
            )
            if requested_position_error > direction_threshold:
                self.approach_direction = 1
            elif requested_position_error < -direction_threshold:
                self.approach_direction = -1
            else:
                self.approach_direction = 0
            if self.approach_direction != 0:
                self.active_target_offset_m = float(
                    self.safety.get("approach_target_offset_m", 0.0)
                )
        refinement_condition = (
            self.approach_direction not in (None, 0)
            and abs(requested_position_error)
            <= float(
                self.safety.get(
                    "target_refinement_error_m", 0.0
                )
            )
            and abs(float(velocity_m_s))
            <= float(
                self.safety.get(
                    "target_refinement_speed_m_s", 0.0
                )
            )
        )
        if not self.target_refinement_unlocked:
            if refinement_condition:
                self.target_refinement_dwell_s += dt
                if self.target_refinement_dwell_s >= float(
                    self.safety.get(
                        "target_refinement_dwell_s", math.inf
                    )
                ):
                    self.target_refinement_unlocked = True
            else:
                self.target_refinement_dwell_s = 0.0
        if self.target_refinement_unlocked and refinement_condition:
            final_offset = max(
                0.0,
                float(
                    self.safety.get(
                        "final_target_offset_m",
                        self.active_target_offset_m,
                    )
                ),
            )
            refinement_rate = max(
                0.0,
                float(
                    self.safety.get(
                        "target_refinement_rate_m_s", 0.0
                    )
                ),
            )
            self.active_target_offset_m = max(
                final_offset,
                self.active_target_offset_m - refinement_rate * dt,
            )
        target_offset = (
            int(self.approach_direction or 0)
            * self.active_target_offset_m
        )
        # 先停在来向侧的安全预停点；连续低速稳定后才缓慢推进到
        # 0.3cm内部目标带，不能把1cm判负线当作控制目标。
        control_target_position_m = (
            float(target_position_m) - target_offset
        )
        position_error = float(
            control_target_position_m - position_m
        )
        if abs(position_error) < self.config["position_deadband_m"]:
            position_error = 0.0

        self.outer_integral = clamp(
            self.outer_integral + position_error * dt,
            -self.config["outer_integral_limit_m_s"],
            self.config["outer_integral_limit_m_s"],
        )
        velocity_reference = (
            self.config["position_kp_s_inv"] * position_error
            + self.config["position_ki_s2_inv"] * self.outer_integral
        )
        braking_distance = max(
            abs(position_error) - self.config["braking_margin_m"], 0.0
        )
        braking_speed_limit = math.sqrt(
            2.0
            * self.config["braking_accel_m_s2"]
            * braking_distance
        )
        velocity_limit = min(
            self.config["max_velocity_m_s"], braking_speed_limit
        )
        velocity_reference = clamp(
            velocity_reference,
            -velocity_limit,
            velocity_limit,
        )
        velocity_error = velocity_reference - velocity_m_s
        if (
            position_error == 0.0
            and abs(velocity_error) < self.config["velocity_deadband_m_s"]
        ):
            velocity_error = 0.0

        candidate_inner = clamp(
            self.inner_integral + velocity_error * dt,
            -self.config["inner_integral_limit_deg"]
            / max(self.config["velocity_ki_deg_per_m"], 1e-9),
            self.config["inner_integral_limit_deg"]
            / max(self.config["velocity_ki_deg_per_m"], 1e-9),
        )
        # 正角度让球向零点运动，即产生-x方向加速度，因此有负号。
        raw_angle = -(
            self.config["velocity_kp_deg_per_m_s"] * velocity_error
            + self.config["velocity_ki_deg_per_m"] * candidate_inner
        )
        # 低速静摩擦会使小角度命令完全不起作用。补偿从0缓慢爬升，并
        # 只作为同向“最小起滚角”，不与PID角度相加；检测到运动或进入
        # 目标附近后立即撤销，避免视觉速度滞后时再额外推一脚。
        static_compensation_limit = float(
            self.config.get("static_friction_compensation_deg", 0.0)
        )
        desired_motion_direction = (
            1 if velocity_reference > 0.0 else -1
        )
        static_compensation_eligible = (
            static_compensation_limit > 0.0
            and abs(position_error)
            >= float(
                self.config.get(
                    "static_compensation_min_error_m", math.inf
                )
            )
            and abs(velocity_m_s)
            <= float(
                self.config.get(
                    "static_compensation_max_speed_m_s", 0.0
                )
            )
            and abs(velocity_reference) > 1e-9
        )
        if static_compensation_eligible:
            ramp_rate = max(
                0.0,
                float(
                    self.config.get(
                        "static_compensation_ramp_deg_s", 0.0
                    )
                ),
            )
            self.static_compensation_deg = min(
                static_compensation_limit,
                self.static_compensation_deg + ramp_rate * dt,
            )
            # v_ref>0需要负角，v_ref<0需要正角。
            angle_direction = -float(desired_motion_direction)
            raw_angle = angle_direction * max(
                angle_direction * raw_angle,
                self.static_compensation_deg,
            )
        else:
            self.static_compensation_deg = 0.0

        # 实机远距离驱动力增强：误差仍很大时持续使用指定的同方向角度，
        # 不再受球速限制；进入目标附近后恢复普通PID与制动。
        far_drive_angle = max(
            0.0, float(self.config.get("far_drive_angle_deg", 0.0))
        )
        far_drive_eligible = (
            far_drive_angle > 0.0
            and abs(position_error)
            >= float(
                self.config.get("far_drive_min_error_m", math.inf)
            )
            and abs(velocity_reference) > 1e-9
        )
        if far_drive_eligible:
            angle_direction = -float(desired_motion_direction)
            raw_angle = angle_direction * max(
                angle_direction * raw_angle,
                far_drive_angle,
            )

        saturated = clamp(
            raw_angle, self.angle_min_deg, self.angle_max_deg
        )
        # 只有未饱和或积分有助于退出饱和时才接受积分，防止wind-up。
        if (
            raw_angle == saturated
            or (
                raw_angle > self.angle_max_deg
                and velocity_error > 0
            )
            or (
                raw_angle < self.angle_min_deg
                and velocity_error < 0
            )
        ):
            self.inner_integral = candidate_inner

        self.last_velocity_reference_m_s = velocity_reference
        self.last_raw_angle_deg = raw_angle
        return self.rate_limiter.update(saturated)


class ConstrainedMPCController:
    """离散动作束搜索非线性MPC，计算时间固定且不依赖外部QP求解器。"""

    def __init__(
        self,
        mpc_config: Dict[str, Any],
        motion_model: Dict[str, Any],
        angle_min_deg: float,
        angle_max_deg: float,
        max_angle_step_deg: float,
        safety_config: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.config = mpc_config
        self.model = motion_model
        self.angle_min_rad = math.radians(angle_min_deg)
        self.angle_max_rad = math.radians(angle_max_deg)
        self.max_step_rad = math.radians(max_angle_step_deg)
        self.safety = safety_config or {}
        self.previous_angle_rad = 0.0
        self.disturbance_accel_m_s2 = 0.0
        self.last_solve_success = False
        self.last_solve_ms = 0.0
        self.last_solver_message = "not_started"

    def reset(self, angle_deg: float = 0.0) -> None:
        self.previous_angle_rad = clamp(
            math.radians(angle_deg),
            self.angle_min_rad,
            self.angle_max_rad,
        )
        self.disturbance_accel_m_s2 = 0.0
        self.last_solve_success = False

    def _friction_acceleration(self, velocity_m_s: float) -> float:
        smooth_velocity = max(
            float(self.model["friction_smoothing_velocity_m_s"]), 1e-4
        )
        return (
            -float(self.model["coulomb_accel_m_s2"])
            * math.tanh(velocity_m_s / smooth_velocity)
            - float(self.model["viscous_drag_s_inv"]) * velocity_m_s
        )

    def observe_acceleration(
        self, measured_acceleration_m_s2: float, velocity_m_s: float
    ) -> None:
        if not math.isfinite(measured_acceleration_m_s2):
            return
        predicted = (
            -float(self.model["acceleration_gain_m_s2"])
            * math.sin(self.previous_angle_rad)
            + self._friction_acceleration(velocity_m_s)
        )
        residual = measured_acceleration_m_s2 - predicted
        limit = float(self.config["max_disturbance_accel_m_s2"])
        residual = clamp(residual, -limit, limit)
        alpha = clamp(
            self.config["disturbance_observer_alpha"], 0.0, 1.0
        )
        self.disturbance_accel_m_s2 = (
            (1.0 - alpha) * self.disturbance_accel_m_s2
            + alpha * residual
        )

    def _fallback_angle(
        self,
        position_m: float,
        velocity_m_s: float,
        target_position_m: float,
    ) -> float:
        gain = max(float(self.model["acceleration_gain_m_s2"]), 0.1)
        desired_acceleration = (
            8.0 * (target_position_m - position_m)
            - 3.0 * velocity_m_s
        )
        requested = -math.asin(clamp(desired_acceleration / gain, -0.5, 0.5))
        return clamp(
            requested,
            self.previous_angle_rad - self.max_step_rad,
            self.previous_angle_rad + self.max_step_rad,
        )

    def update(
        self,
        position_m: float,
        velocity_m_s: float,
        target_position_m: float,
        dt_s: float,
    ) -> float:
        dt = clamp(dt_s, 0.01, 0.10)
        started = time.perf_counter()
        action_levels = int(self.config.get("action_levels", 5))
        if action_levels < 3 or action_levels % 2 == 0:
            raise ValueError("MPC action_levels必须是至少3的奇数。")
        beam_width = max(1, int(self.config.get("beam_width", 96)))
        horizon = max(1, int(self.config["horizon_steps"]))
        actions = np.linspace(
            -self.max_step_rad, self.max_step_rad, action_levels
        )
        positions = np.array([float(position_m)])
        velocities = np.array([float(velocity_m_s)])
        angles = np.array([self.previous_angle_rad])
        costs = np.zeros(1)
        first_angles = np.array([self.previous_angle_rad])
        gain = float(self.model["acceleration_gain_m_s2"])
        coulomb = float(self.model["coulomb_accel_m_s2"])
        viscous = float(self.model["viscous_drag_s_inv"])
        smooth_velocity = max(
            float(self.model["friction_smoothing_velocity_m_s"]), 1e-4
        )
        internal_tolerance = float(
            self.safety.get("internal_tolerance_m", 0.003)
        )
        if target_position_m - position_m > internal_tolerance:
            approach_direction = 1
        elif target_position_m - position_m < -internal_tolerance:
            approach_direction = -1
        else:
            approach_direction = 0

        for depth in range(horizon):
            candidate_angles = np.clip(
                angles[:, None] + actions[None, :],
                self.angle_min_rad,
                self.angle_max_rad,
            )
            actual_changes = candidate_angles - angles[:, None]
            candidate_velocities_old = np.repeat(
                velocities[:, None], action_levels, axis=1
            )
            acceleration = (
                -gain * np.sin(candidate_angles)
                - coulomb
                * np.tanh(candidate_velocities_old / smooth_velocity)
                - viscous * candidate_velocities_old
                + self.disturbance_accel_m_s2
            )
            candidate_positions = (
                positions[:, None]
                + candidate_velocities_old * dt
                + 0.5 * acceleration * dt ** 2
            )
            candidate_velocities = (
                candidate_velocities_old + acceleration * dt
            )
            errors = candidate_positions - target_position_m
            candidate_costs = costs[:, None] + (
                self.config["position_weight"] * errors ** 2
                + self.config["velocity_weight"]
                * candidate_velocities ** 2
                + self.config["angle_weight"] * candidate_angles ** 2
                + self.config["angle_change_weight"]
                * actual_changes ** 2
            )
            candidate_costs += 1e6 * (
                np.minimum(candidate_positions, 0.0) ** 2
                + np.maximum(candidate_positions - 0.25, 0.0) ** 2
            )
            overshoot_weight = float(
                self.config.get("overshoot_weight", 1e6)
            )
            if approach_direction > 0:
                overshoot = np.maximum(
                    candidate_positions
                    - (target_position_m + internal_tolerance),
                    0.0,
                )
                candidate_costs += overshoot_weight * overshoot ** 2
            elif approach_direction < 0:
                overshoot = np.maximum(
                    (target_position_m - internal_tolerance)
                    - candidate_positions,
                    0.0,
                )
                candidate_costs += overshoot_weight * overshoot ** 2
            if depth == 0:
                candidate_first_angles = candidate_angles
            else:
                candidate_first_angles = np.repeat(
                    first_angles[:, None], action_levels, axis=1
                )

            flat_costs = candidate_costs.ravel()
            keep_count = min(beam_width, len(flat_costs))
            if keep_count < len(flat_costs):
                keep = np.argpartition(flat_costs, keep_count - 1)[
                    :keep_count
                ]
            else:
                keep = np.arange(len(flat_costs))
            positions = candidate_positions.ravel()[keep]
            velocities = candidate_velocities.ravel()[keep]
            angles = candidate_angles.ravel()[keep]
            costs = flat_costs[keep]
            first_angles = candidate_first_angles.ravel()[keep]

        terminal_costs = costs + self.config[
            "terminal_position_weight"
        ] * (positions - target_position_m) ** 2
        best = int(np.argmin(terminal_costs))
        requested = float(first_angles[best])
        self.last_solve_ms = (time.perf_counter() - started) * 1000.0
        self.last_solve_success = math.isfinite(requested)
        self.last_solver_message = (
            "beam_search_ok" if self.last_solve_success else "non_finite"
        )
        if not self.last_solve_success:
            requested = self._fallback_angle(
                position_m, velocity_m_s, target_position_m
            )
        requested = clamp(
            requested, self.angle_min_rad, self.angle_max_rad
        )
        requested = clamp(
            requested,
            self.previous_angle_rad - self.max_step_rad,
            self.previous_angle_rad + self.max_step_rad,
        )
        self.previous_angle_rad = requested
        return math.degrees(requested)
