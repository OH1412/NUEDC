#!/usr/bin/env python3
"""H 题坐标变换和深度采样的无相机单元测试。"""

import json
import tempfile
import unittest
import sys
from pathlib import Path

import numpy as np

H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from depth_utils import DepthError, sample_depth
from geometry import load_transform


class TransformTests(unittest.TestCase):
    def test_current_camera_to_base_axis_mapping(self) -> None:
        transform = load_transform(H_DIR / "camera_to_base.json")
        np.testing.assert_allclose(
            transform.transform_point([0.0, 0.0, 0.0]),
            [0.0, 0.0, 0.262249],
        )
        np.testing.assert_allclose(
            transform.transform_point([1.0, 0.0, 0.0]),
            [-0.101179197, -0.986389035, 0.132636342],
        )
        np.testing.assert_allclose(
            transform.transform_point([0.0, 1.0, 0.0]),
            [-0.606963894, 0.164428316, -0.515284382],
        )
        np.testing.assert_allclose(
            transform.transform_point([0.0, 0.0, 1.0]),
            [0.788262394, 0.0, -0.353090255],
        )
        np.testing.assert_allclose(
            transform.transform_point([0.12, -0.03, 0.80]),
            [0.63667732838, -0.12329953368, -0.2222499215],
        )

    def test_rotation_and_translation(self) -> None:
        config = {
            "rotation_matrix": [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
            "translation_m": [1, 2, 3],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "transform.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            transform = load_transform(path)
        actual = transform.transform_point([1, 0, 2])
        np.testing.assert_allclose(actual, [1, 3, 5])


class DepthTests(unittest.TestCase):
    def test_center_roi_rejects_bbox_background(self) -> None:
        depth = np.full((100, 100), 2000, dtype=np.uint16)
        depth[40:60, 40:60] = 800
        sample = sample_depth(
            depth,
            depth_scale_m=0.001,
            detection_bbox_xyxy=[30, 30, 70, 70],
            roi_scale=0.45,
        )
        self.assertAlmostEqual(sample.depth_m, 0.8)

    def test_invalid_depth_is_reported(self) -> None:
        depth = np.zeros((50, 50), dtype=np.uint16)
        with self.assertRaises(DepthError):
            sample_depth(
                depth,
                depth_scale_m=0.001,
                detection_bbox_xyxy=[10, 10, 40, 40],
            )


if __name__ == "__main__":
    unittest.main()
