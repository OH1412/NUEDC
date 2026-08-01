#!/usr/bin/env python3
"""实时曲线窗口的无GUI数学测试。"""

from pathlib import Path
import sys
import unittest


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from control_curve_ui import axis_limits  # noqa: E402


class ControlCurveUITests(unittest.TestCase):
    def test_axis_limits_keep_minimum_span_and_include_curves(self) -> None:
        low, high = axis_limits([0.4, 0.5, 0.45], 1.0)
        self.assertGreaterEqual(high - low, 1.0)
        self.assertLessEqual(low, 0.4)
        self.assertGreaterEqual(high, 0.5)

        low, high = axis_limits([-3.0, 5.0], 1.0)
        self.assertLess(low, -3.0)
        self.assertGreater(high, 5.0)

    def test_empty_axis_is_symmetric(self) -> None:
        self.assertEqual(axis_limits([], 2.0), (-1.0, 1.0))


if __name__ == "__main__":
    unittest.main()
