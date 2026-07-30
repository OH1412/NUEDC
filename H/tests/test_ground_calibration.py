#!/usr/bin/env python3
"""地面平面拟合和外参建议测试。"""

import sys
import unittest
from pathlib import Path

import numpy as np


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from calibrate_camera_ground import fit_ground_plane, suggested_transform


class GroundCalibrationTests(unittest.TestCase):
    def test_height_and_pitch_from_synthetic_ground(self) -> None:
        rng = np.random.default_rng(7)
        pitch_deg = 30.0
        height_m = 0.25
        angle = np.deg2rad(pitch_deg)
        normal = np.array([0.0, -np.cos(angle), -np.sin(angle)])

        x_values = rng.uniform(-0.4, 0.4, 5000)
        z_values = rng.uniform(0.35, 1.2, 5000)
        y_values = (
            -height_m - normal[2] * z_values
        ) / normal[1]
        points = np.column_stack([x_values, y_values, z_values])
        points += rng.normal(0.0, 0.001, points.shape)
        outliers = rng.uniform(-1.0, 1.0, (300, 3))
        points = np.vstack([points, outliers])

        estimate = fit_ground_plane(
            points,
            threshold_m=0.008,
            iterations=120,
            min_inlier_ratio=0.80,
            rng=np.random.default_rng(8),
        )
        self.assertAlmostEqual(estimate.height_m, height_m, delta=0.002)
        self.assertAlmostEqual(
            estimate.pitch_down_deg, pitch_deg, delta=0.3
        )
        self.assertLess(abs(estimate.roll_deg), 0.3)

    def test_suggested_transform_matches_axis_rule(self) -> None:
        rotation, translation = suggested_transform(0.25, 30.0)
        np.testing.assert_allclose(
            rotation,
            [
                [0.0, -0.5, np.sqrt(3.0) / 2.0],
                [-1.0, 0.0, 0.0],
                [0.0, -np.sqrt(3.0) / 2.0, -0.5],
            ],
            atol=1e-9,
        )
        np.testing.assert_allclose(translation, [0.0, 0.0, 0.25])
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0)

    def test_roll_is_included_in_rotation(self) -> None:
        rotation, _ = suggested_transform(0.25, 24.0, 7.0)
        expected_normal = np.array(
            [
                np.cos(np.deg2rad(24.0)) * np.sin(np.deg2rad(7.0)),
                -np.cos(np.deg2rad(24.0)) * np.cos(np.deg2rad(7.0)),
                -np.sin(np.deg2rad(24.0)),
            ]
        )
        np.testing.assert_allclose(rotation[2], expected_normal, atol=1e-9)
        np.testing.assert_allclose(
            rotation.dot(rotation.T), np.eye(3), atol=1e-9
        )
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0)

if __name__ == "__main__":
    unittest.main()
