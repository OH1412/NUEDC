#!/usr/bin/env python3
"""实时控制时间戳与视觉新鲜度辅助逻辑测试。"""

from pathlib import Path
import json
import sys
from types import SimpleNamespace
import unittest
from unittest import mock


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from ball_control import KinematicEstimate  # noqa: E402
from ball_control_runtime import (  # noqa: E402
    LatestRecord,
    optional_finite_float,
    optional_nonnegative_int,
    parse_args,
    point_timestamp_s,
    predict_to_now,
    stop_on_competition_failure,
    telemetry_line,
)


class RuntimeTimestampTests(unittest.TestCase):
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
                controller=None,
                valid_control=True,
                control_updated=True,
                target_status=status,
                mode="compact",
            )
        )
        self.assertEqual(
            set(payload),
            {
                "valid",
                "target_cm",
                "position_cm",
                "error_cm",
                "velocity_cm_s",
                "command_deg",
            },
        )
        self.assertEqual(payload["position_cm"], 14.282)
        self.assertEqual(payload["error_cm"], 0.718)
        self.assertEqual(payload["command_deg"], -5.01)


if __name__ == "__main__":
    unittest.main()
