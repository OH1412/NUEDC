#!/usr/bin/env python3
"""PPR 管道识别的合成图测试。"""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from ppr_pipe_detector import (
    DetectorConfig,
    LightDetectorConfig,
    detect_ppr_pipe,
    detect_ppr_pipe_lightweight,
    line_angle_difference_deg,
)


def synthetic_pipe(
    center=(320, 240),
    size=(360, 32),
    angle_deg=20.0,
    add_clutter=True,
) -> np.ndarray:
    image = np.full((480, 640, 3), 35, dtype=np.uint8)
    box = cv2.boxPoints((center, size, angle_deg)).astype(np.int32)
    cv2.fillConvexPoly(image, box, (225, 225, 225))
    cv2.polylines(image, [box], True, (250, 250, 250), 2)
    if add_clutter:
        cv2.line(image, (30, 80), (120, 80), (220, 220, 220), 3)
        cv2.line(image, (520, 100), (560, 180), (200, 200, 200), 4)
        cv2.rectangle(image, (30, 350), (110, 420), (160, 160, 160), 2)
    return image


class PprPipeDetectorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = DetectorConfig(
            min_length_px=180,
            min_width_px=5,
            max_width_px=80,
            min_aspect_ratio=5,
            hough_min_line_length=130,
        )

    def test_rotated_pipe_center_axis_and_angle(self) -> None:
        image = synthetic_pipe(angle_deg=20.0)
        detection, _debug, _counts = detect_ppr_pipe(image, self.config)
        self.assertIsNotNone(detection)
        self.assertLess(
            line_angle_difference_deg(detection.angle_deg, 20.0), 3.0
        )
        self.assertLess(
            np.linalg.norm(np.asarray(detection.center) - [320, 240]), 8.0
        )
        self.assertGreater(detection.length_px, 330)
        self.assertGreater(detection.aspect_ratio, 7)

    def test_horizontal_pipe(self) -> None:
        image = synthetic_pipe(
            center=(300, 220), size=(320, 28), angle_deg=0.0
        )
        detection, _debug, _counts = detect_ppr_pipe(image, self.config)
        self.assertIsNotNone(detection)
        self.assertLess(
            line_angle_difference_deg(detection.angle_deg, 0.0), 2.0
        )
        self.assertLess(
            np.linalg.norm(np.asarray(detection.center) - [300, 220]), 8.0
        )

    def test_short_clutter_is_rejected(self) -> None:
        image = np.full((480, 640, 3), 30, dtype=np.uint8)
        cv2.line(image, (40, 100), (130, 100), (230, 230, 230), 8)
        cv2.rectangle(image, (400, 300), (470, 350), (220, 220, 220), 3)
        detection, _debug, _counts = detect_ppr_pipe(image, self.config)
        self.assertIsNone(detection)

    def test_light_mode_contour_only(self) -> None:
        image = synthetic_pipe(angle_deg=-18.0)
        light_config = LightDetectorConfig(
            process_scale=0.5,
            min_length_px=180,
            min_width_px=5,
            max_width_px=80,
            min_aspect_ratio=5,
        )
        detection, _debug, counts = detect_ppr_pipe_lightweight(
            image, light_config
        )
        self.assertIsNotNone(detection)
        self.assertEqual(counts["hough_candidates"], 0)
        self.assertEqual(detection.method, "binary_fitline")
        self.assertLess(
            line_angle_difference_deg(detection.angle_deg, -18.0), 4.0
        )

    def test_light_mode_detects_dark_pipe(self) -> None:
        image = np.full((480, 640, 3), 225, dtype=np.uint8)
        box = cv2.boxPoints(((320, 240), (350, 30), 12.0)).astype(
            np.int32
        )
        cv2.fillConvexPoly(image, box, (25, 25, 25))
        detection, _debug, counts = detect_ppr_pipe_lightweight(
            image,
            LightDetectorConfig(process_scale=0.5, max_width_px=80),
        )
        self.assertIsNotNone(detection)
        self.assertEqual(counts["binary_polarity"], "dark")
        self.assertLess(
            line_angle_difference_deg(detection.angle_deg, 12.0), 4.0
        )


if __name__ == "__main__":
    unittest.main()
