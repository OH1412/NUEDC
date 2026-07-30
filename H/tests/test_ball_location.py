#!/usr/bin/env python3
"""使用模拟检测框验证像素、深度、反投影和底座变换全链路。"""

import sys
import unittest
from pathlib import Path

import numpy as np
import pyrealsense2 as rs


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from ball_depth_tracker import locate_ball
from geometry import load_transform


class FakeTensor:
    def __init__(self, values: object) -> None:
        self.values = np.asarray(values)

    def detach(self) -> "FakeTensor":
        return self

    def cpu(self) -> "FakeTensor":
        return self

    def numpy(self) -> np.ndarray:
        return self.values


class FakeBoxes:
    def __init__(
        self,
        boxes: object = ((40.0, 40.0, 60.0, 60.0),),
        confidences: object = (0.95,),
        classes: object = (0,),
    ) -> None:
        self.xyxy = FakeTensor(boxes)
        self.conf = FakeTensor(confidences)
        self.cls = FakeTensor(classes)

    def __len__(self) -> int:
        return len(self.xyxy.values)


class FakeResult:
    def __init__(self, boxes: FakeBoxes = None) -> None:
        self.boxes = boxes if boxes is not None else FakeBoxes()
        self.names = {0: "steel_ball"}


class BallLocationTests(unittest.TestCase):
    def test_complete_camera_to_base_transform_chain(self) -> None:
        intrinsics = rs.intrinsics()
        intrinsics.width = 640
        intrinsics.height = 480
        intrinsics.fx = 600.0
        intrinsics.fy = 600.0
        intrinsics.ppx = 320.0
        intrinsics.ppy = 240.0
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = [0.0] * 5

        depth_raw = np.full((480, 640), 1000, dtype=np.uint16)
        transform = load_transform(H_DIR / "camera_to_base.json")
        record = locate_ball(
            result=FakeResult(
                FakeBoxes(boxes=((300.0, 220.0, 340.0, 260.0),))
            ),
            depth_raw=depth_raw,
            depth_scale_m=0.001,
            color_intrinsics=intrinsics,
            transform=transform,
            frame_index=1,
            timestamp_ms=123.0,
            required_class_id=None,
            depth_roi_scale=0.45,
            min_depth_m=0.08,
            max_depth_m=3.0,
            min_depth_samples=8,
            max_bbox_area_ratio=0.03,
            max_bbox_width_ratio=0.25,
            max_bbox_height_ratio=0.25,
        )

        self.assertTrue(record["valid"])
        self.assertEqual(record["pixel"], {"u": 320.0, "v": 240.0})
        self.assertAlmostEqual(record["surface_depth_m"], 1.0)
        self.assertAlmostEqual(record["ball_radius_m"], 0.005)
        self.assertAlmostEqual(record["depth_m"], 1.005)
        self.assertEqual(
            record["camera_point_m"], {"x": 0.0, "y": 0.0, "z": 1.005}
        )
        self.assertEqual(
            record["camera_base_point_m"],
            {"x": 0.792204, "y": 0.0, "z": -0.356167},
        )

    def test_huge_box_is_filtered_and_not_returned_for_drawing(self) -> None:
        intrinsics = rs.intrinsics()
        intrinsics.width = 640
        intrinsics.height = 480
        intrinsics.fx = 600.0
        intrinsics.fy = 600.0
        intrinsics.ppx = 320.0
        intrinsics.ppy = 240.0
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = [0.0] * 5
        result = FakeResult(
            FakeBoxes(
                boxes=((3.5, 0.0, 640.0, 480.0),),
                confidences=(0.99,),
            )
        )
        record = locate_ball(
            result=result,
            depth_raw=np.full((480, 640), 500, dtype=np.uint16),
            depth_scale_m=0.001,
            color_intrinsics=intrinsics,
            transform=load_transform(H_DIR / "camera_to_base.json"),
            frame_index=2,
            timestamp_ms=456.0,
            required_class_id=None,
            depth_roi_scale=0.45,
            min_depth_m=0.08,
            max_depth_m=3.0,
            min_depth_samples=8,
            max_bbox_area_ratio=0.03,
            max_bbox_width_ratio=0.25,
            max_bbox_height_ratio=0.25,
        )
        self.assertFalse(record["valid"])
        self.assertEqual(record["reason"], "detection_filtered")
        self.assertNotIn("bbox_xyxy", record)

    def test_small_box_wins_over_higher_confidence_huge_box(self) -> None:
        intrinsics = rs.intrinsics()
        intrinsics.width = 640
        intrinsics.height = 480
        intrinsics.fx = 600.0
        intrinsics.fy = 600.0
        intrinsics.ppx = 320.0
        intrinsics.ppy = 240.0
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = [0.0] * 5
        result = FakeResult(
            FakeBoxes(
                boxes=(
                    (3.5, 0.0, 640.0, 480.0),
                    (440.0, 280.0, 480.0, 324.0),
                ),
                confidences=(0.99, 0.80),
                classes=(0, 0),
            )
        )
        record = locate_ball(
            result=result,
            depth_raw=np.full((480, 640), 500, dtype=np.uint16),
            depth_scale_m=0.001,
            color_intrinsics=intrinsics,
            transform=load_transform(H_DIR / "camera_to_base.json"),
            frame_index=3,
            timestamp_ms=789.0,
            required_class_id=None,
            depth_roi_scale=0.45,
            min_depth_m=0.08,
            max_depth_m=3.0,
            min_depth_samples=8,
            max_bbox_area_ratio=0.03,
            max_bbox_width_ratio=0.25,
            max_bbox_height_ratio=0.25,
        )
        self.assertTrue(record["valid"])
        self.assertEqual(record["bbox_xyxy"], [440.0, 280.0, 480.0, 324.0])

    def test_only_highest_confidence_valid_ball_is_output(self) -> None:
        intrinsics = rs.intrinsics()
        intrinsics.width = 640
        intrinsics.height = 480
        intrinsics.fx = 600.0
        intrinsics.fy = 600.0
        intrinsics.ppx = 320.0
        intrinsics.ppy = 240.0
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = [0.0] * 5
        result = FakeResult(
            FakeBoxes(
                boxes=(
                    (100.0, 100.0, 140.0, 140.0),
                    (300.0, 220.0, 340.0, 260.0),
                ),
                confidences=(0.70, 0.92),
                classes=(0, 0),
            )
        )
        record = locate_ball(
            result=result,
            depth_raw=np.full((480, 640), 500, dtype=np.uint16),
            depth_scale_m=0.001,
            color_intrinsics=intrinsics,
            transform=load_transform(H_DIR / "camera_to_base.json"),
            frame_index=4,
            timestamp_ms=900.0,
            required_class_id=None,
            depth_roi_scale=0.45,
            min_depth_m=0.08,
            max_depth_m=3.0,
            min_depth_samples=8,
            max_bbox_area_ratio=0.03,
            max_bbox_width_ratio=0.25,
            max_bbox_height_ratio=0.25,
        )
        self.assertTrue(record["valid"])
        self.assertEqual(record["bbox_xyxy"], [300.0, 220.0, 340.0, 260.0])
        self.assertEqual(record["pixel"], {"u": 320.0, "v": 240.0})
        self.assertAlmostEqual(record["confidence"], 0.92)


if __name__ == "__main__":
    unittest.main()
