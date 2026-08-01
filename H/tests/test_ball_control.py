#!/usr/bin/env python3
"""钢珠位置、状态估计、串级PID和约束MPC测试。"""

import copy
import math
import json
from pathlib import Path
import sys
import time
import unittest

import numpy as np


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from ball_control import (  # noqa: E402
    CascadePIDController,
    CompetitionTargetMonitor,
    ConstrainedMPCController,
    KinematicKalmanFilter,
    VelocityLowPassFilter,
    ball_position_from_zero,
)


class BallPositionTests(unittest.TestCase):
    def test_three_dimensional_distance_from_fixed_zero(self) -> None:
        position = ball_position_from_zero(
            [0.06, -0.08, 0.262249],
            [0.0, 0.0, 0.262249],
            pipe_length_m=0.25,
        )
        self.assertAlmostEqual(position, 0.10)

    def test_out_of_pipe_measurement_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            ball_position_from_zero(
                [0.40, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                pipe_length_m=0.25,
                tolerance_m=0.03,
            )


class StateEstimatorTests(unittest.TestCase):
    def test_noisy_constant_acceleration_track(self) -> None:
        rng = np.random.default_rng(5)
        estimator = KinematicKalmanFilter(
            measurement_std_m=0.0015,
            jerk_std_m_s3=0.3,
            outlier_gate_sigma=5.0,
        )
        actual_acceleration = -0.6
        estimate = None
        for index in range(101):
            timestamp = index * 0.02
            actual_position = (
                0.8
                + 0.1 * timestamp
                + 0.5 * actual_acceleration * timestamp ** 2
            )
            measurement = actual_position + rng.normal(0.0, 0.0015)
            estimate = estimator.update(measurement, timestamp)
        assert estimate is not None
        self.assertAlmostEqual(
            estimate.acceleration_m_s2,
            actual_acceleration,
            delta=0.12,
        )
        self.assertAlmostEqual(
            estimate.velocity_m_s,
            0.1 + actual_acceleration * 2.0,
            delta=0.05,
        )

    def test_single_large_position_jump_is_rejected(self) -> None:
        estimator = KinematicKalmanFilter(
            measurement_std_m=0.001,
            jerk_std_m_s3=0.5,
        )
        estimator.update(0.10, 1.0)
        estimator.update(0.101, 1.04)
        estimate = estimator.update(0.23, 1.08)
        self.assertFalse(estimate.measurement_accepted)
        self.assertLess(estimate.position_m, 0.12)

    def test_velocity_low_pass_reduces_jitter_and_resets(self) -> None:
        low_pass = VelocityLowPassFilter(time_constant_s=0.12)
        outputs = []
        raw = []
        for index in range(40):
            value = 0.10 if index % 2 else -0.10
            raw.append(value)
            outputs.append(low_pass.update(value, index * 0.05))
        self.assertLess(np.std(outputs[10:]), np.std(raw[10:]) * 0.35)

        for index in range(40, 70):
            value = low_pass.update(0.05, index * 0.05)
        self.assertAlmostEqual(value, 0.05, delta=0.002)

        low_pass.reset()
        self.assertAlmostEqual(low_pass.update(-0.03, 10.0), -0.03)

    def test_velocity_filter_clamps_stationary_jitter_but_keeps_motion(self) -> None:
        stationary = VelocityLowPassFilter(
            time_constant_s=0.12,
            stationary_window_s=0.4,
            stationary_position_span_m=0.0008,
            stationary_velocity_threshold_m_s=0.001,
        )
        filtered = None
        for index in range(20):
            filtered = stationary.update(
                0.0009 if index % 2 else -0.0009,
                index * 0.05,
                0.10 + (0.0001 if index % 3 else -0.0001),
            )
        self.assertEqual(filtered, 0.0)

        moving = VelocityLowPassFilter(
            time_constant_s=0.12,
            stationary_window_s=0.4,
            stationary_position_span_m=0.0008,
            stationary_velocity_threshold_m_s=0.001,
        )
        for index in range(20):
            filtered = moving.update(
                0.005,
                index * 0.05,
                0.10 + 0.005 * index * 0.05,
            )
        self.assertGreater(filtered, 0.004)


class CompetitionTargetMonitorTests(unittest.TestCase):
    def make_monitor(self) -> CompetitionTargetMonitor:
        return CompetitionTargetMonitor(
            target_position_m=0.15,
            internal_tolerance_m=0.003,
            competition_tolerance_m=0.01,
            settle_velocity_m_s=0.008,
            settle_time_s=0.5,
        )

    def test_positive_direction_one_centimeter_boundary_latches(self) -> None:
        monitor = self.make_monitor()

        initial = monitor.update(0.10, 0.0, 0.0)
        self.assertEqual(initial.approach_direction, 1)
        self.assertAlmostEqual(initial.failure_boundary_m, 0.16)

        still_valid = monitor.update(0.1599, 0.0, 0.2)
        self.assertFalse(still_valid.competition_failed)
        crossed = monitor.update(0.1601, 0.0, 0.3)
        self.assertTrue(crossed.competition_failed)

        # 回到目标点也不能清除失败；只能由显式 reset 开始新任务。
        returned = monitor.update(0.15, 0.0, 0.4)
        self.assertTrue(returned.competition_failed)
        monitor.reset()
        after_reset = monitor.update(0.10, 0.0, 0.5)
        self.assertFalse(after_reset.competition_failed)

    def test_negative_direction_one_centimeter_boundary_latches(self) -> None:
        monitor = self.make_monitor()

        initial = monitor.update(0.20, 0.0, 0.0)
        self.assertEqual(initial.approach_direction, -1)
        self.assertAlmostEqual(initial.failure_boundary_m, 0.14)

        still_valid = monitor.update(0.1401, 0.0, 0.2)
        self.assertFalse(still_valid.competition_failed)
        crossed = monitor.update(0.1399, 0.0, 0.3)
        self.assertTrue(crossed.competition_failed)

        returned = monitor.update(0.15, 0.0, 0.4)
        self.assertTrue(returned.competition_failed)

    def test_three_millimeter_tolerance_requires_continuous_stability(self) -> None:
        monitor = self.make_monitor()

        entered = monitor.update(0.1529, 0.0079, 1.0)
        self.assertTrue(entered.within_internal_tolerance)
        self.assertFalse(entered.settled)
        self.assertAlmostEqual(entered.settled_duration_s, 0.0)

        almost_long_enough = monitor.update(0.1471, -0.0079, 1.49)
        self.assertTrue(almost_long_enough.within_internal_tolerance)
        self.assertFalse(almost_long_enough.settled)
        self.assertAlmostEqual(
            almost_long_enough.settled_duration_s,
            0.49,
        )

        settled = monitor.update(0.15, 0.0, 1.50)
        self.assertTrue(settled.settled)
        self.assertAlmostEqual(settled.settled_duration_s, 0.5)

        # 速度超限会立即打断连续计时，下一次稳定从零重新累计。
        moving = monitor.update(0.15, 0.0081, 1.60)
        self.assertFalse(moving.settled)
        restarted = monitor.update(0.15, 0.0, 1.70)
        self.assertFalse(restarted.settled)
        self.assertAlmostEqual(restarted.settled_duration_s, 0.0)

        outside = monitor.update(0.1531, 0.0, 1.80)
        self.assertFalse(outside.within_internal_tolerance)
        self.assertFalse(outside.settled)

    def test_measurement_loss_clears_only_settle_timer(self) -> None:
        monitor = self.make_monitor()
        monitor.update(0.10, 0.0, 0.0)
        monitor.update(0.15, 0.0, 1.0)
        monitor.clear_settle_timer()
        restarted = monitor.update(0.15, 0.0, 2.0)
        self.assertFalse(restarted.settled)
        self.assertEqual(restarted.approach_direction, 1)
        self.assertFalse(restarted.competition_failed)


class ControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = json.loads(
            (H_DIR / "ball_control_config.json").read_text(
                encoding="utf-8"
            )
        )

    def test_cascade_pid_sign_and_rate_limit(self) -> None:
        controller = CascadePIDController(
            self.config["cascade_pid"], -10.0, 10.0, 1.5
        )
        # 目标在电机端方向，需负角让球远离零点。
        toward_motor = controller.update(0.10, 0.0, 0.20, 0.04)
        self.assertLess(toward_motor, 0.0)
        self.assertLessEqual(abs(toward_motor), 1.5)
        controller.reset()
        # 目标在零点方向，需正角让球向零点滚。
        toward_zero = controller.update(0.20, 0.0, 0.10, 0.04)
        self.assertGreater(toward_zero, 0.0)
        self.assertLessEqual(abs(toward_zero), 1.5)

    def test_cascade_pid_never_exceeds_working_limit(self) -> None:
        controller = CascadePIDController(
            self.config["cascade_pid"], -8.0, 8.0, 1.0
        )
        outputs = [
            controller.update(0.25, 0.2, 0.0, 0.04)
            for _ in range(100)
        ]
        self.assertTrue(all(-8.0 <= value <= 8.0 for value in outputs))
        self.assertTrue(
            all(
                abs(second - first) <= 1.000001
                for first, second in zip(outputs, outputs[1:])
            )
        )

    def test_cascade_pid_braking_speed_limit_is_symmetric(self) -> None:
        config = copy.deepcopy(self.config["cascade_pid"])
        config.update(
            {
                "position_kp_s_inv": 100.0,
                "position_ki_s2_inv": 0.0,
                "max_velocity_m_s": 1.0,
                "braking_accel_m_s2": 0.5,
                "braking_margin_m": 0.003,
                "position_deadband_m": 0.0,
            }
        )
        controller = CascadePIDController(config, -30.0, 30.0, 30.0)
        distance = 0.013
        effective_distance = distance ** 2 / (distance + 0.003)
        expected_speed = math.sqrt(2.0 * 0.5 * effective_distance)

        controller.update(0.137, 0.0, 0.15, 0.04)
        self.assertAlmostEqual(
            controller.last_velocity_reference_m_s,
            expected_speed,
            places=8,
        )

        controller.reset()
        controller.update(0.163, 0.0, 0.15, 0.04)
        self.assertAlmostEqual(
            controller.last_velocity_reference_m_s,
            -expected_speed,
            places=8,
        )

        # 进入3mm软制动区后仍保留连续的小速度，不能提前停死；
        # 只有误差真正为0时才归零。
        controller.reset()
        controller.update(0.147, 0.0, 0.15, 0.04)
        near_distance = 0.003
        near_effective_distance = (
            near_distance ** 2 / (near_distance + 0.003)
        )
        self.assertAlmostEqual(
            controller.last_velocity_reference_m_s,
            math.sqrt(2.0 * 0.5 * near_effective_distance),
            places=8,
        )

    def test_cascade_pid_braking_angle_opposes_incoming_velocity(self) -> None:
        controller = CascadePIDController(
            self.config["cascade_pid"], -30.0, 30.0, 30.0
        )

        # x正方向运动过快：正角产生-x加速度进行制动。
        brake_positive_velocity = controller.update(
            0.149,
            0.05,
            0.15,
            0.04,
        )
        self.assertGreater(brake_positive_velocity, 0.0)

        controller.reset()
        # x负方向运动过快：负角产生+x加速度进行制动。
        brake_negative_velocity = controller.update(
            0.151,
            -0.05,
            0.15,
            0.04,
        )
        self.assertLess(brake_negative_velocity, 0.0)

    def test_positive_cart_acceleration_adds_negative_feedforward(self) -> None:
        controller = CascadePIDController(
            self.config["cascade_pid"], -10.0, 10.0, 10.0
        )
        without_feedforward = controller.update(
            0.15, 0.0, 0.15, 0.04
        )
        controller.reset()
        with_feedforward = controller.update(
            0.15, 0.0, 0.15, 0.04, -3.0
        )
        self.assertAlmostEqual(without_feedforward, 0.0)
        self.assertAlmostEqual(with_feedforward, -3.0)

        controller = CascadePIDController(
            self.config["cascade_pid"], -5.0, 5.0, 10.0
        )
        combined_command = controller.update(
            0.15, 0.0, 0.15, 0.04, 9.0
        )
        self.assertAlmostEqual(combined_command, 5.0)

    def test_static_friction_compensation_has_drive_sign(self) -> None:
        """静止启动补偿必须与所需驱动角同号。"""

        baseline_config = copy.deepcopy(self.config["cascade_pid"])
        baseline_config.update(
            {
                "position_kp_s_inv": 1.0,
                "position_ki_s2_inv": 0.0,
                "max_velocity_m_s": 1.0,
                "velocity_kp_deg_per_m_s": 10.0,
                "velocity_ki_deg_per_m": 0.0,
                "position_deadband_m": 0.0,
                "velocity_deadband_m_s": 0.0,
                "static_friction_compensation_deg": 0.0,
                "far_drive_angle_deg": 0.0,
            }
        )
        compensated_config = copy.deepcopy(baseline_config)
        compensated_config.update(
            {
                "static_friction_compensation_deg": 1.2,
                "static_compensation_ramp_deg_s": 10.0,
                "static_compensation_max_speed_m_s": 0.01,
                "static_compensation_min_error_m": 0.003,
            }
        )

        baseline = CascadePIDController(
            baseline_config, -30.0, 30.0, 30.0
        )
        compensated = CascadePIDController(
            compensated_config, -30.0, 30.0, 30.0
        )
        baseline_toward_motor = 0.0
        compensated_toward_motor = 0.0
        for _ in range(10):
            baseline_toward_motor = baseline.update(
                0.10, 0.0, 0.15, 0.04
            )
            compensated_toward_motor = compensated.update(
                0.10, 0.0, 0.15, 0.04
            )

        baseline.reset()
        compensated.reset()
        baseline_toward_zero = 0.0
        compensated_toward_zero = 0.0
        for _ in range(10):
            baseline_toward_zero = baseline.update(
                0.20, 0.0, 0.15, 0.04
            )
            compensated_toward_zero = compensated.update(
                0.20, 0.0, 0.15, 0.04
            )

        self.assertGreater(
            abs(compensated_toward_motor),
            abs(baseline_toward_motor),
        )
        self.assertLessEqual(compensated_toward_motor, -1.2)
        self.assertGreater(
            abs(compensated_toward_zero),
            abs(baseline_toward_zero),
        )
        self.assertGreaterEqual(compensated_toward_zero, 1.2)

    def test_default_far_drive_is_disabled_and_compensation_is_small(self) -> None:
        """远距离不强制大角度，静止时仅缓升到0.8°起滚补偿。"""

        toward_motor = CascadePIDController(
            self.config["cascade_pid"],
            -10.0,
            10.0,
            float(self.config["max_angle_step_deg"]),
            self.config["safety"],
        )
        toward_zero = CascadePIDController(
            self.config["cascade_pid"],
            -10.0,
            10.0,
            float(self.config["max_angle_step_deg"]),
            self.config["safety"],
        )
        motor_angle = 0.0
        zero_angle = 0.0
        for _ in range(20):
            motor_angle = toward_motor.update(
                0.02, 0.0, 0.125, 1.0 / 17.5
            )
            zero_angle = toward_zero.update(
                0.23, 0.0, 0.125, 1.0 / 17.5
            )
        self.assertAlmostEqual(motor_angle, -0.8, places=6)
        self.assertAlmostEqual(zero_angle, 0.8, places=6)

        moving_far = CascadePIDController(
            self.config["cascade_pid"],
            -10.0,
            10.0,
            float(self.config["max_angle_step_deg"]),
            self.config["safety"],
        )
        moving_angle = 0.0
        for _ in range(20):
            moving_angle = moving_far.update(
                0.02, 0.03, 0.125, 1.0 / 17.5
            )
        # 钢珠已经以3cm/s朝目标运动，超过1.2cm/s参考速度，
        # 此时应反向制动，而不是继续朝目标驱动。
        self.assertGreater(moving_angle, 0.0)
        self.assertLess(abs(moving_angle), 0.8)

        near_target = CascadePIDController(
            self.config["cascade_pid"],
            -10.0,
            10.0,
            float(self.config["max_angle_step_deg"]),
            self.config["safety"],
        )
        near_angle = 0.0
        for _ in range(20):
            near_angle = near_target.update(
                0.105, 0.0, 0.125, 1.0 / 17.5
            )
        self.assertLessEqual(abs(near_angle), 0.800001)

    def test_low_speed_inside_deadband_holds_motor_position(self) -> None:
        controller = CascadePIDController(
            self.config["cascade_pid"],
            -2.0,
            2.0,
            float(self.config["max_angle_step_deg"]),
            self.config["safety"],
        )
        controller.reset(0.4)
        controller.inner_integral = 0.2
        held = controller.update(0.1025, 0.0, 0.10, 0.05)
        self.assertAlmostEqual(held, 0.4)
        self.assertAlmostEqual(controller.inner_integral, 0.2)

        # 高速经过0.3cm死区仍必须制动，不能盲目保持电机位置。
        braking = controller.update(0.1025, -0.02, 0.10, 0.05)
        self.assertNotAlmostEqual(braking, held)

    def test_stalled_nonzero_command_learns_temporary_local_zero(self) -> None:
        controller = CascadePIDController(
            self.config["cascade_pid"],
            -2.0,
            2.0,
            float(self.config["max_angle_step_deg"]),
            self.config["safety"],
        )
        for _ in range(90):
            controller.update(0.05, 0.0, 0.15, 0.05)
        self.assertGreater(controller.local_zero_update_count, 0)
        self.assertLess(controller.local_zero_angle_deg, -0.1)
        learned_position = controller.local_zero_position_m
        self.assertIsNotNone(learned_position)

        # 运动中必须携带当前偏置，不能因跨过距离阈值突然清零并回弹。
        controller.update(float(learned_position) + 0.02, 0.02, 0.15, 0.05)
        self.assertLess(controller.local_zero_angle_deg, -0.1)
        self.assertEqual(controller.local_zero_position_m, learned_position)

        # 只有安全重置才清除运行期临时值。
        controller.reset(0.0)
        self.assertEqual(controller.local_zero_angle_deg, 0.0)
        self.assertIsNone(controller.local_zero_position_m)

    def test_local_zero_stall_time_uses_profile_parameter(self) -> None:
        config = copy.deepcopy(self.config["cascade_pid"])
        config["local_zero_stall_time_s"] = 0.1
        controller = CascadePIDController(
            config,
            -2.0,
            2.0,
            float(self.config["max_angle_step_deg"]),
            self.config["safety"],
        )
        for _ in range(8):
            controller.update(0.05, 0.0, 0.15, 0.05)
        self.assertTrue(controller.stall_drive_adaptation_active)
        self.assertLess(controller.stall_drive_boost_angle_deg, 0.0)

    def test_local_zero_refresh_does_not_double_static_compensation(self) -> None:
        config = copy.deepcopy(self.config["cascade_pid"])
        config.update(
            {
                "local_zero_stall_time_s": 0.1,
                "velocity_kp_deg_per_m_s": 20.0,
                "velocity_ki_deg_per_m": 3.0,
                "static_friction_compensation_deg": 0.85,
                "static_compensation_ramp_deg_s": 100.0,
                "stall_drive_boost_max_deg": 1.0,
                "stall_drive_boost_ramp_deg_s": 0.25,
            }
        )
        controller = CascadePIDController(config, -3.0, 3.0, 0.25)
        controller.rate_limiter.reset(-1.35)
        controller.static_compensation_deg = 0.85
        controller.local_zero_stall_reference_m = 0.12
        controller.local_zero_stall_duration_s = 0.1

        command = controller.update_velocity(0.12, 0.0, 0.01, 0.05)

        # 清积分后的P仅-0.2°，静摩擦下限仍为-0.85°；刷新应把剩余
        # -0.50°转入局部零点，再正常叠加-0.85°，不能变成-2.20°。
        self.assertAlmostEqual(controller.local_zero_angle_deg, -0.5, places=6)
        self.assertAlmostEqual(controller.static_compensation_deg, 0.85)
        self.assertGreater(command, -1.5)
        self.assertLess(command, -1.35)

    def test_local_zero_still_gets_pid_increment_before_safe_pretarget(self) -> None:
        """0.3cm最终死区不能吞掉安全预停点前的剩余误差。"""

        controller = CascadePIDController(
            self.config["cascade_pid"],
            -2.0,
            2.0,
            float(self.config["max_angle_step_deg"]),
            self.config["safety"],
        )
        learned_zero = -0.576
        controller.local_zero_angle_deg = learned_zero
        controller.local_zero_position_m = 0.1415
        controller.rate_limiter.reset(learned_zero)

        # 请求目标15cm；从正方向接近时安全预停点为14.4cm。
        # 当前14.15cm距预停点仅0.25cm，旧代码错误地被0.3cm死区清零。
        command = controller.update(0.1415, 0.0, 0.15, 0.05)
        self.assertLess(command, learned_zero)

    def test_pretarget_stall_gets_static_compensation_and_noise_margin(self) -> None:
        """最终误差约0.72cm时不能卡在0.6cm安全预停点。"""

        controller = CascadePIDController(
            self.config["cascade_pid"],
            -2.0,
            2.0,
            float(self.config["max_angle_step_deg"]),
            self.config["safety"],
        )
        # 先从右侧建立朝负方向接近；目标7cm的安全预停点是7.6cm。
        controller.update(0.12, 0.0, 0.07, 0.05)
        # 当前7.72cm：最终误差0.72cm，但距预停点只有0.12cm，小于
        # 原静摩擦补偿门槛0.6cm。补偿仍应开始缓慢建立。
        command = controller.update(0.0772, 0.0, 0.07, 0.05)
        self.assertGreater(controller.static_compensation_deg, 0.0)
        self.assertGreater(command, 0.0)

        # 0.6cm解除阈值加0.15cm视觉噪声余量后，持续低速0.6秒
        # 应允许二阶段推进，不再要求测量必须严格小于0.600cm。
        for _ in range(12):
            controller.update(0.0772, 0.0, 0.07, 0.05)
        self.assertTrue(controller.target_refinement_unlocked)
        self.assertLess(controller.active_target_offset_m, 0.006)

    def test_direct_velocity_mode_tracks_speed_and_keeps_local_zero(self) -> None:
        driving = CascadePIDController(
            self.config["cascade_pid"], -2.0, 2.0, 0.25, self.config["safety"]
        )
        drive_angle = driving.update_velocity(0.12, 0.0, 0.01, 0.05)
        self.assertLess(drive_angle, 0.0)
        self.assertAlmostEqual(driving.last_velocity_reference_m_s, 0.01)
        self.assertEqual(driving.outer_integral, 0.0)

        braking = CascadePIDController(
            self.config["cascade_pid"], -2.0, 2.0, 0.25, self.config["safety"]
        )
        brake_angle = braking.update_velocity(0.12, 0.02, 0.01, 0.05)
        self.assertGreater(brake_angle, 0.0)
        self.assertEqual(braking.static_compensation_deg, 0.0)

        stalled = CascadePIDController(
            self.config["cascade_pid"], -2.0, 2.0, 0.25, self.config["safety"]
        )
        for _ in range(90):
            stalled.update_velocity(0.12, 0.0, 0.01, 0.05)
        self.assertGreater(stalled.local_zero_update_count, 0)
        self.assertLess(stalled.local_zero_angle_deg, -0.1)
        self.assertAlmostEqual(stalled.local_zero_position_m, 0.12)

    def test_safe_pretarget_requires_continuous_low_speed_dwell(self) -> None:
        config = copy.deepcopy(self.config["cascade_pid"])
        config["static_friction_compensation_deg"] = 0.0
        safety = copy.deepcopy(self.config["safety"])
        controller = CascadePIDController(
            config, -10.0, 10.0, 10.0, safety
        )

        controller.update(0.10, 0.0, 0.20, 0.10)
        self.assertAlmostEqual(controller.active_target_offset_m, 0.006)
        # 预停点附近偶尔低速不足0.6秒，不能提前向真实目标推进。
        for _ in range(5):
            controller.update(0.1941, 0.0, 0.20, 0.10)
        self.assertFalse(controller.target_refinement_unlocked)
        self.assertAlmostEqual(controller.active_target_offset_m, 0.006)

        controller.update(0.1941, 0.0, 0.20, 0.10)
        self.assertTrue(controller.target_refinement_unlocked)
        self.assertLess(controller.active_target_offset_m, 0.006)

    def test_beam_mpc_sign_constraints_and_runtime(self) -> None:
        controller = ConstrainedMPCController(
            self.config["mpc"],
            self.config["motion_model"],
            -10.0,
            10.0,
            1.5,
        )
        started = time.perf_counter()
        toward_zero = controller.update(0.20, 0.0, 0.10, 0.04)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.assertGreater(toward_zero, 0.0)
        self.assertLessEqual(abs(toward_zero), 1.500001)
        self.assertTrue(controller.last_solve_success)
        # 宽松门限用于发现意外退化；实机基准通常约10～15ms。
        self.assertLess(elapsed_ms, 100.0)

        controller.reset()
        toward_motor = controller.update(0.10, 0.0, 0.20, 0.04)
        self.assertLess(toward_motor, 0.0)
        self.assertLessEqual(abs(toward_motor), 1.500001)

        controller.reset()
        compensate_positive_cart_acceleration = controller.update(
            0.15,
            0.0,
            0.15,
            0.04,
            cart_acceleration_m_s2=1.0,
        )
        self.assertLess(compensate_positive_cart_acceleration, 0.0)

        controller.reset()
        target_specific_holding_angle = controller.update(
            0.15,
            0.0,
            0.15,
            0.04,
            equilibrium_angle_bias_deg=2.0,
        )
        self.assertGreater(target_specific_holding_angle, 0.0)


if __name__ == "__main__":
    unittest.main()
