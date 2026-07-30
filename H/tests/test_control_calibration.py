#!/usr/bin/env python3
"""零点稳健统计和固定倾角轨迹辨识测试。"""

import sys
from pathlib import Path
import unittest

import numpy as np


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from calibrate_ball_zero import (  # noqa: E402
    robust_zero_point,
    tracker_failure_message,
)
from calibrate_pipe_friction import analyze_roll_down  # noqa: E402


class ZeroCalibrationTests(unittest.TestCase):
    def test_robust_zero_rejects_outliers(self) -> None:
        rng = np.random.default_rng(11)
        expected = np.array([0.12, -0.04, 0.03])
        points = expected + rng.normal(0.0, 0.0005, (60, 3))
        points[:4] += np.array([0.04, -0.03, 0.02])
        zero, residuals, inliers = robust_zero_point(points)
        np.testing.assert_allclose(zero, expected, atol=0.0003)
        self.assertGreaterEqual(inliers, 55)
        self.assertLess(float(np.sqrt(np.mean(residuals ** 2))), 0.002)

    def test_tracker_sigkill_reports_probable_memory_shortage(self) -> None:
        message = tracker_failure_message(-9)
        self.assertIn("信号9", message)
        self.assertIn("内存不足", message)

    def test_tracker_exit_code_is_preserved(self) -> None:
        self.assertIn("退出码为3", tracker_failure_message(3))


class FrictionCalibrationTests(unittest.TestCase):
    def test_recovers_constant_downhill_acceleration(self) -> None:
        rng = np.random.default_rng(12)
        sample_rate = 60.0
        times = np.arange(0.0, 0.62, 1.0 / sample_rate)
        acceleration = -1.0
        positions = (
            0.235
            - 0.005 * times
            + 0.5 * acceleration * times ** 2
            + rng.normal(0.0, 0.0005, len(times))
        )
        result = analyze_roll_down(
            times,
            positions,
            angle_deg=10.0,
            min_speed_m_s=0.02,
            min_fit_samples=10,
        )
        self.assertAlmostEqual(
            result["signed_acceleration_m_s2"],
            acceleration,
            delta=0.08,
        )
        self.assertAlmostEqual(
            result["empirical_acceleration_gain_m_s2"],
            1.0 / np.sin(np.deg2rad(10.0)),
            delta=0.5,
        )
        self.assertGreater(result["median_sample_rate_hz"], 55.0)


if __name__ == "__main__":
    unittest.main()
