#!/usr/bin/env python3
"""实时控制时间戳与视觉新鲜度辅助逻辑测试。"""

from pathlib import Path
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from ball_control import CascadePIDController, KinematicEstimate  # noqa: E402
from ball_control_runtime import (  # noqa: E402
    LatestRecord,
    centered_target_limits_cm,
    control_target_from_centered,
    create_target_monitor,
    directed_point_reached,
    optional_finite_float,
    optional_nonnegative_int,
    outward_velocity_edge_reached,
    parse_args,
    point_timestamp_s,
    predicted_local_zero_from_target_cm,
    predict_to_now,
    save_special_task_config,
    SPECIAL_TASK_MINUS4P5_THEN_PLUS5,
    special_return_trigger_reached,
    special_waypoint_reached,
    stop_on_competition_failure,
    telemetry_line,
    validate_special_task_settings,
    velocity_edge_trigger_cm,
)


class RuntimeTimestampTests(unittest.TestCase):
    def test_special_task_argument_and_waypoint_threshold(self) -> None:
        with mock.patch.object(
            sys,
            "argv",
            [
                "ball_control_runtime.py",
                "--special-task",
                SPECIAL_TASK_MINUS4P5_THEN_PLUS5,
                "--no-stream",
            ],
        ):
            args = parse_args()
        self.assertEqual(
            args.special_task, SPECIAL_TASK_MINUS4P5_THEN_PLUS5
        )
        self.assertFalse(special_waypoint_reached(-4.44))
        self.assertTrue(special_waypoint_reached(-4.45))
        self.assertTrue(special_waypoint_reached(-4.8))
        self.assertFalse(special_return_trigger_reached(2.94))
        self.assertTrue(special_return_trigger_reached(2.95))
        self.assertTrue(special_return_trigger_reached(3.2))

    def test_special_task_ui_settings_and_bidirectional_reach(self) -> None:
        settings = validate_special_task_settings(
            {
                "first_point_cm": -4.5,
                "second_point_cm": 5.0,
                "first_angle_deg": 1.8,
                "positive_motor_scale": 0.2,
                "negative_motor_scale": 0.7,
            },
            (-12.0, 12.0),
            2.0,
        )
        self.assertEqual(settings["first_point_cm"], -4.5)
        self.assertEqual(settings["second_point_cm"], 5.0)
        self.assertEqual(settings["positive_motor_scale"], 0.2)
        self.assertEqual(settings["negative_motor_scale"], 0.7)
        self.assertTrue(directed_point_reached(-4.45, -4.5, -1))
        self.assertFalse(directed_point_reached(-4.44, -4.5, -1))
        self.assertTrue(directed_point_reached(2.95, 3.0, 1))
        self.assertFalse(directed_point_reached(2.94, 3.0, 1))

    def test_special_task_rejects_equal_points_or_invalid_angle(self) -> None:
        base = {
            "first_point_cm": -4.5,
            "second_point_cm": 5.0,
            "first_angle_deg": 2.0,
        }
        equal_points = dict(base, second_point_cm=-4.5)
        with self.assertRaisesRegex(ValueError, "不能相同"):
            validate_special_task_settings(
                equal_points, (-12.0, 12.0), 3.0
            )
        invalid_angle = dict(base, first_angle_deg=3.1)
        with self.assertRaisesRegex(ValueError, "工作限角"):
            validate_special_task_settings(
                invalid_angle, (-12.0, 12.0), 3.0
            )

    def test_special_task_save_only_overwrites_special_section(self) -> None:
        original = {
            "working_angle_limit_deg": 3.0,
            "cascade_pid": {"position_kp_s_inv": 0.4},
            "special_task": {"first_point_cm": -9.0},
        }
        settings = {
            "first_point_cm": -3.0,
            "second_point_cm": 5.0,
            "first_angle_deg": 2.43,
            "positive_motor_scale": 0.2,
            "negative_motor_scale": 0.7,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps(original), encoding="utf-8")
            save_special_task_config(path, settings)
            saved = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(saved["special_task"], settings)
        self.assertEqual(saved["cascade_pid"], original["cascade_pid"])
        self.assertEqual(saved["working_angle_limit_deg"], 3.0)

    def test_position_local_zero_prior_interpolates_motor_mm(self) -> None:
        config = json.loads(
            (H_DIR / "ball_control_config.json").read_text(encoding="utf-8")
        )
        plus_angle, plus_mm = predicted_local_zero_from_target_cm(5.0, config)
        minus_angle, minus_mm = predicted_local_zero_from_target_cm(-5.0, config)
        center_angle, center_mm = predicted_local_zero_from_target_cm(0.0, config)
        self.assertAlmostEqual(plus_mm, -1.5)
        self.assertAlmostEqual(minus_mm, 0.65)
        self.assertAlmostEqual(center_mm, -0.425)
        self.assertLess(plus_angle, 0.0)
        self.assertGreater(minus_angle, 0.0)
        self.assertLess(center_angle, 0.0)

        # 标定区间外只采用最近端点，不做危险外推。
        self.assertAlmostEqual(
            predicted_local_zero_from_target_cm(12.0, config)[1], -1.5
        )
        self.assertAlmostEqual(
            predicted_local_zero_from_target_cm(-12.0, config)[1], 0.65
        )

    def test_live_record_must_have_capture_monotonic_time(self) -> None:
        with self.assertRaises(ValueError):
            point_timestamp_s({"valid": True})
        self.assertAlmostEqual(
            point_timestamp_s({"capture_monotonic_ms": 1234.5}),
            1.2345,
        )

    def test_prediction_is_bounded_but_reports_true_age(self) -> None:
        estimate = KinematicEstimate(
            position_m=0.10,
            velocity_m_s=0.05,
            acceleration_m_s2=10.0,
            timestamp_s=1.0,
            measurement_accepted=True,
        )
        position, velocity, actual_age = predict_to_now(
            estimate, now_s=1.5, max_age_s=0.20
        )
        self.assertAlmostEqual(position, 0.11)
        self.assertAlmostEqual(velocity, 0.05)
        self.assertAlmostEqual(actual_age, 0.50)

    def test_latest_record_keeps_reader_receive_time(self) -> None:
        latest = LatestRecord()
        latest.put({"frame": 7})
        record, received_s, sequence, finished, error = latest.get()
        self.assertEqual(record, {"frame": 7})
        self.assertIsNotNone(received_s)
        self.assertEqual(sequence, 1)
        self.assertFalse(finished)
        self.assertIsNone(error)

    def test_optional_tracker_metrics_never_raise(self) -> None:
        self.assertAlmostEqual(
            optional_finite_float({"latency": "12.5"}, "latency"),
            12.5,
        )
        for value in (None, "bad", float("nan"), float("inf")):
            self.assertIsNone(
                optional_finite_float({"latency": value}, "latency")
            )
        self.assertEqual(
            optional_nonnegative_int({"frame": "17"}, "frame"), 17
        )
        for value in (None, True, -1, "bad", float("inf")):
            self.assertIsNone(
                optional_nonnegative_int({"frame": value}, "frame")
            )

    def test_tuning_debug_flag_must_be_before_tracker_remainder(self) -> None:
        with mock.patch.object(
            sys,
            "argv",
            [
                "ball_control_runtime.py",
                "--target-cm",
                "10",
                "--tuning-debug",
                "--",
                "--no-display",
            ],
        ):
            args = parse_args()
        self.assertTrue(args.tuning_debug)
        self.assertEqual(args.tracker_args, ["--", "--no-display"])
        self.assertEqual(args.stream_host, "192.168.50.199")
        self.assertFalse(args.no_stream)
        self.assertFalse(args.enable_acceleration_feedforward)
        self.assertIsNone(args.test_cart_acceleration_m_s2)
        self.assertIsNone(args.equilibrium_angle_bias_deg)
        self.assertIsNone(args.working_angle_limit_deg)
        self.assertFalse(args.no_control_ui)
        self.assertFalse(args.no_plot_ui)
        self.assertEqual(args.control_mode, "position")
        self.assertEqual(args.target_speed_cm_s, 0.0)

    def test_position_target_defaults_to_center_zero(self) -> None:
        with mock.patch.object(
            sys, "argv", ["ball_control_runtime.py", "--no-stream"]
        ):
            args = parse_args()
        self.assertEqual(args.target_cm, 0.0)
        self.assertEqual(args.control_mode, "position")

    def test_centered_target_maps_to_internal_pipe_coordinate(self) -> None:
        config = {
            "pipe_length_m": 0.25,
            "target_coordinate_center_m": 0.12,
            "zero_calibration_ball_radius_m": 0.005,
        }
        self.assertAlmostEqual(
            control_target_from_centered(-12.0, config), 0.0
        )
        self.assertAlmostEqual(
            control_target_from_centered(0.0, config), 0.12
        )
        self.assertAlmostEqual(
            control_target_from_centered(12.0, config), 0.24
        )
        for invalid in (-12.01, 12.01, float("nan")):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    control_target_from_centered(invalid, config)

    def test_ui_target_limits_and_new_monitor_follow_selected_target(self) -> None:
        config = {
            "pipe_length_m": 0.25,
            "target_coordinate_center_m": 0.12,
            "zero_calibration_ball_radius_m": 0.005,
            "safety": {
                "internal_tolerance_m": 0.003,
                "competition_tolerance_m": 0.01,
                "settle_velocity_m_s": 0.008,
                "settle_time_s": 0.5,
            },
        }
        self.assertEqual(centered_target_limits_cm(config), (-12.0, 12.0))
        target_m = control_target_from_centered(5.0, config)
        monitor = create_target_monitor(target_m, config)
        self.assertAlmostEqual(monitor.target_position_m, 0.17)
        self.assertFalse(monitor.competition_failed)
        self.assertIsNone(monitor.approach_direction)

    def test_competition_failure_continues_by_default(self) -> None:
        args = mock.Mock(stop_on_competition_failure=False)
        config = {"safety": {"stop_on_competition_failure": False}}
        self.assertFalse(stop_on_competition_failure(args, config))

    def test_competition_failure_can_explicitly_stop(self) -> None:
        args = mock.Mock(stop_on_competition_failure=True)
        config = {"safety": {"stop_on_competition_failure": False}}
        self.assertTrue(stop_on_competition_failure(args, config))

    def test_compact_telemetry_contains_only_operator_fields(self) -> None:
        status = SimpleNamespace(settled=False, competition_failed=True)
        config = json.loads(
            (H_DIR / "ball_control_config.json").read_text(encoding="utf-8")
        )
        controller = CascadePIDController(
            config["cascade_pid"], -2.0, 2.0, 0.25, config["safety"]
        )
        controller.last_velocity_reference_m_s = 0.012
        payload = json.loads(
            telemetry_line(
                cycle=100,
                controller_name="cascade_pid",
                target_m=0.15,
                estimate=None,
                position_now=0.14282,
                velocity_now=0.00144,
                measurement_age_s=0.10,
                angle_deg=-5.006,
                serial_enabled=False,
                controller=controller,
                valid_control=True,
                control_updated=True,
                target_status=status,
                mode="compact",
            )
        )
        self.assertEqual(
            set(payload),
            {
                "tgt",
                "pos",
                "err",
                "v_tgt",
                "vel",
                "deg",
                "mm",
            },
        )
        self.assertEqual(payload["pos"], 14.282)
        self.assertEqual(payload["err"], 0.718)
        self.assertEqual(payload["v_tgt"], 1.2)
        self.assertEqual(payload["vel"], 0.144)
        self.assertEqual(payload["deg"], -5.01)
        self.assertEqual(payload["mm"], -21.9)

    def test_compact_telemetry_uses_centered_coordinate(self) -> None:
        status = SimpleNamespace(settled=False, competition_failed=False)
        payload = json.loads(
            telemetry_line(
                cycle=1,
                controller_name="cascade_pid",
                target_m=0.12,
                estimate=None,
                position_now=0.0,
                velocity_now=0.0,
                measurement_age_s=0.01,
                angle_deg=0.0,
                serial_enabled=False,
                controller=None,
                valid_control=True,
                control_updated=True,
                target_status=status,
                mode="compact",
                coordinate_center_m=0.12,
            )
        )
        self.assertEqual(payload["tgt"], 0.0)
        self.assertEqual(payload["pos"], -12.0)
        self.assertEqual(payload["err"], 12.0)
        self.assertEqual(payload["vel"], 0.0)

    def test_compact_telemetry_mm_uses_motor_displacement_scale(self) -> None:
        payload = json.loads(
            telemetry_line(
                cycle=1,
                controller_name="cascade_pid",
                target_m=0.12,
                estimate=None,
                position_now=0.12,
                velocity_now=0.0,
                measurement_age_s=0.01,
                angle_deg=10.0,
                serial_enabled=False,
                controller=None,
                valid_control=True,
                control_updated=True,
                target_status=None,
                motor_displacement_scale=0.8,
            )
        )
        self.assertEqual(payload["deg"], 10.0)
        self.assertEqual(payload["mm"], 35.27)

        negative_payload = json.loads(
            telemetry_line(
                cycle=2,
                controller_name="cascade_pid",
                target_m=0.12,
                estimate=None,
                position_now=0.12,
                velocity_now=0.0,
                measurement_age_s=0.01,
                angle_deg=-10.0,
                serial_enabled=False,
                controller=None,
                valid_control=True,
                control_updated=True,
                target_status=None,
                motor_displacement_scale=0.8,
                negative_motor_displacement_scale=0.5,
            )
        )
        self.assertEqual(negative_payload["mm"], -22.04)

    def test_velocity_mode_compact_telemetry_has_no_position_target(self) -> None:
        config = json.loads(
            (H_DIR / "ball_control_config.json").read_text(encoding="utf-8")
        )
        controller = CascadePIDController(
            config["cascade_pid"], -2.0, 2.0, 0.25, config["safety"]
        )
        controller.last_velocity_reference_m_s = -0.0075
        payload = json.loads(
            telemetry_line(
                cycle=1,
                controller_name="cascade_pid",
                target_m=0.12,
                estimate=None,
                position_now=0.10,
                velocity_now=-0.006,
                measurement_age_s=0.01,
                angle_deg=0.2,
                serial_enabled=False,
                controller=controller,
                valid_control=True,
                control_updated=True,
                target_status=None,
                mode="compact",
                coordinate_center_m=0.12,
                control_mode="velocity",
            )
        )
        self.assertEqual(
            set(payload), {"pos", "v_tgt", "vel", "deg", "mm"}
        )
        self.assertEqual(payload["pos"], -2.0)
        self.assertEqual(payload["v_tgt"], -0.75)
        self.assertEqual(payload["vel"], -0.6)

    def test_velocity_edge_guard_only_latches_outward_motion(self) -> None:
        config = {
            "pipe_length_m": 0.25,
            "zero_calibration_ball_radius_m": 0.005,
        }
        trigger = velocity_edge_trigger_cm(config)
        self.assertAlmostEqual(trigger, 11.5)
        self.assertTrue(outward_velocity_edge_reached(11.5, 0.5, trigger))
        self.assertTrue(outward_velocity_edge_reached(-11.6, -0.5, trigger))
        self.assertFalse(outward_velocity_edge_reached(11.6, -0.5, trigger))
        self.assertFalse(outward_velocity_edge_reached(-11.6, 0.5, trigger))
        self.assertFalse(outward_velocity_edge_reached(10.0, 0.5, trigger))


if __name__ == "__main__":
    unittest.main()
