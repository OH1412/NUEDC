#!/usr/bin/env python3
"""车辆纵向加速度接口与前馈符号测试。"""

from pathlib import Path
import math
import sys
import unittest


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from longitudinal_acceleration import (  # noqa: E402
    ManualAccelerationSource,
    ReservedAccelerationSource,
    acceleration_feedforward_angle_deg,
)


class LongitudinalAccelerationTests(unittest.TestCase):
    def test_reserved_source_does_not_access_hardware(self) -> None:
        source = ReservedAccelerationSource()
        self.assertIsNone(source.latest_sample())
        source.close()

    def test_positive_acceleration_produces_negative_angle(self) -> None:
        positive = acceleration_feedforward_angle_deg(1.0)
        negative = acceleration_feedforward_angle_deg(-1.0)
        self.assertLess(positive, 0.0)
        self.assertGreater(negative, 0.0)
        self.assertAlmostEqual(positive, -negative)
        self.assertAlmostEqual(
            positive,
            -math.degrees(math.asin(1.0 / 9.80665)),
        )

    def test_manual_source_is_available_for_software_test(self) -> None:
        source = ManualAccelerationSource(0.75)
        sample = source.latest_sample()
        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertAlmostEqual(sample.acceleration_m_s2, 0.75)

    def test_invalid_values_are_rejected(self) -> None:
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    acceleration_feedforward_angle_deg(value)


if __name__ == "__main__":
    unittest.main()
