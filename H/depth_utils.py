#!/usr/bin/env python3
"""在 YOLO 检测框中心区域内稳健提取 RealSense 深度。"""

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np


class DepthError(ValueError):
    """深度图或采样参数无效。"""


@dataclass(frozen=True)
class DepthSample:
    depth_m: float
    valid_count: int
    candidate_count: int
    sample_bbox_xyxy: Tuple[int, int, int, int]
    spread_m: float


def _clipped_center_roi(
    image_shape: Sequence[int],
    detection_bbox_xyxy: Sequence[float],
    roi_scale: float,
) -> Tuple[int, int, int, int]:
    if len(image_shape) < 2:
        raise DepthError("深度图至少需要二维形状。")
    height, width = int(image_shape[0]), int(image_shape[1])
    if height <= 0 or width <= 0:
        raise DepthError("深度图尺寸无效。")
    if len(detection_bbox_xyxy) != 4:
        raise DepthError("检测框必须是 [x1, y1, x2, y2]。")
    if not 0.0 < roi_scale <= 1.0:
        raise DepthError("roi_scale 必须位于 (0, 1]。")

    x1, y1, x2, y2 = [float(value) for value in detection_bbox_xyxy]
    if not np.all(np.isfinite([x1, y1, x2, y2])) or x2 <= x1 or y2 <= y1:
        raise DepthError("检测框坐标无效。")
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    half_width = max(1.0, (x2 - x1) * roi_scale / 2.0)
    half_height = max(1.0, (y2 - y1) * roi_scale / 2.0)

    roi_x1 = max(0, int(np.floor(center_x - half_width)))
    roi_y1 = max(0, int(np.floor(center_y - half_height)))
    roi_x2 = min(width, int(np.ceil(center_x + half_width)))
    roi_y2 = min(height, int(np.ceil(center_y + half_height)))
    if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
        raise DepthError("检测框中心区域落在深度图之外。")
    return roi_x1, roi_y1, roi_x2, roi_y2


def sample_depth(
    depth_raw: np.ndarray,
    depth_scale_m: float,
    detection_bbox_xyxy: Sequence[float],
    roi_scale: float = 0.45,
    min_depth_m: float = 0.08,
    max_depth_m: float = 3.0,
    min_valid_samples: int = 8,
) -> DepthSample:
    """返回检测框中心椭圆区域的中值深度。

    中心区域可减少检测框边缘背景对钢珠深度的污染。无效深度为 0，会被过滤；
    有效样本不足时明确抛出异常，而不是把背景深度伪装成钢珠深度。
    """
    if not isinstance(depth_raw, np.ndarray) or depth_raw.ndim != 2:
        raise DepthError("depth_raw 必须是二维 numpy 数组。")
    if not np.isfinite(depth_scale_m) or depth_scale_m <= 0:
        raise DepthError("depth_scale_m 必须大于 0。")
    if min_depth_m <= 0 or max_depth_m <= min_depth_m:
        raise DepthError("深度范围无效。")
    if min_valid_samples < 1:
        raise DepthError("min_valid_samples 至少为 1。")

    x1, y1, x2, y2 = _clipped_center_roi(
        depth_raw.shape, detection_bbox_xyxy, roi_scale
    )
    crop_m = depth_raw[y1:y2, x1:x2].astype(np.float64) * depth_scale_m

    crop_height, crop_width = crop_m.shape
    yy, xx = np.ogrid[:crop_height, :crop_width]
    radius_x = max(crop_width / 2.0, 1.0)
    radius_y = max(crop_height / 2.0, 1.0)
    ellipse = (
        ((xx - (crop_width - 1) / 2.0) / radius_x) ** 2
        + ((yy - (crop_height - 1) / 2.0) / radius_y) ** 2
        <= 1.0
    )
    candidates = crop_m[ellipse]
    valid = candidates[
        np.isfinite(candidates)
        & (candidates >= min_depth_m)
        & (candidates <= max_depth_m)
    ]
    if valid.size < min_valid_samples:
        raise DepthError(
            "钢珠框中心只有 {} 个有效深度点，少于阈值 {}。"
            .format(valid.size, min_valid_samples)
        )

    median = float(np.median(valid))
    absolute_deviation = np.abs(valid - median)
    mad = float(np.median(absolute_deviation))
    # 只在存在明显离群值时二次过滤。2 mm 是 D435 近距离抖动的保守下限。
    gate = max(0.002, 3.5 * 1.4826 * mad)
    inliers = valid[absolute_deviation <= gate]
    if inliers.size >= min_valid_samples:
        valid = inliers

    depth_m = float(np.median(valid))
    spread_m = float(np.median(np.abs(valid - depth_m)))
    return DepthSample(
        depth_m=depth_m,
        valid_count=int(valid.size),
        candidate_count=int(candidates.size),
        sample_bbox_xyxy=(x1, y1, x2, y2),
        spread_m=spread_m,
    )
