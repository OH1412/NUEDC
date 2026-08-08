#!/usr/bin/env python3
"""H 题 PPR 管道识别：轮廓筛选 + 概率霍夫双边线校正。"""

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs

from udp_video_stream import StreamConfig, StreamError, UdpH264Streamer


@dataclass(frozen=True)
class DetectorConfig:
    process_scale: float = 0.5
    blur_kernel: int = 5
    canny_low: int = 50
    canny_high: int = 150
    close_kernel: int = 9
    close_iterations: int = 2
    min_length_px: float = 180.0
    min_width_px: float = 5.0
    max_width_px: float = 100.0
    min_aspect_ratio: float = 5.0
    min_contour_area: float = 300.0
    hough_threshold: int = 70
    hough_min_line_length: float = 140.0
    hough_max_line_gap: float = 30.0
    max_parallel_angle_deg: float = 6.0
    min_line_overlap_ratio: float = 0.45
    max_hough_segments: int = 24
    enable_hough: bool = True


@dataclass(frozen=True)
class LightDetectorConfig:
    """独立轻量方案：二值分割 + 连通轮廓 + fitLine。"""

    process_scale: float = 0.5
    blur_kernel: int = 5
    morph_kernel: int = 5
    morph_iterations: int = 1
    min_length_px: float = 180.0
    min_width_px: float = 5.0
    max_width_px: float = 100.0
    min_aspect_ratio: float = 5.0
    min_contour_area: float = 300.0
    min_fill_ratio: float = 0.35
    max_area_ratio: float = 0.35
    max_contours: int = 64


@dataclass(frozen=True)
class PipeDetection:
    center: Tuple[float, float]
    endpoint_1: Tuple[float, float]
    endpoint_2: Tuple[float, float]
    angle_deg: float
    length_px: float
    width_px: float
    aspect_ratio: float
    score: float
    method: str


@dataclass(frozen=True)
class LineSegment:
    p1: np.ndarray
    p2: np.ndarray
    direction: np.ndarray
    length: float
    angle_deg: float


def normalized_angle_deg(angle_deg: float) -> float:
    """把无方向直线角度归一化到 [-90°, 90°)。"""
    value = (angle_deg + 90.0) % 180.0 - 90.0
    return 0.0 if abs(value) < 1e-9 else value


def line_angle_difference_deg(angle_1: float, angle_2: float) -> float:
    return abs(normalized_angle_deg(angle_1 - angle_2))


def direction_from_angle(angle_deg: float) -> np.ndarray:
    radians = math.radians(angle_deg)
    return np.array([math.cos(radians), math.sin(radians)], dtype=np.float64)


def preprocess(
    frame_bgr: np.ndarray, config: DetectorConfig
) -> Dict[str, np.ndarray]:
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3:
        raise ValueError("输入必须是 BGR 彩色图像。")
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(
        gray, (config.blur_kernel, config.blur_kernel), 0
    )
    edges = cv2.Canny(blurred, config.canny_low, config.canny_high)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (config.close_kernel, config.close_kernel)
    )
    closed = cv2.morphologyEx(
        edges,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=config.close_iterations,
    )
    return {
        "gray": gray,
        "blurred": blurred,
        "edges": edges,
        "closed": closed,
    }


def scaled_odd_kernel(value: int, scale: float) -> int:
    scaled = max(3, int(round(value * scale)))
    return scaled if scaled % 2 == 1 else scaled + 1


def processing_config(config: DetectorConfig) -> DetectorConfig:
    """将以原图像素定义的阈值换算到降采样处理图。"""
    scale = config.process_scale
    if scale == 1.0:
        return config
    return replace(
        config,
        process_scale=1.0,
        blur_kernel=scaled_odd_kernel(config.blur_kernel, scale),
        close_kernel=scaled_odd_kernel(config.close_kernel, scale),
        min_length_px=config.min_length_px * scale,
        min_width_px=config.min_width_px * scale,
        max_width_px=config.max_width_px * scale,
        min_contour_area=config.min_contour_area * scale * scale,
        hough_min_line_length=config.hough_min_line_length * scale,
        hough_max_line_gap=config.hough_max_line_gap * scale,
    )


def rescale_detection(
    detection: Optional[PipeDetection], inverse_scale: float
) -> Optional[PipeDetection]:
    if detection is None or inverse_scale == 1.0:
        return detection

    def scaled_point(point: Tuple[float, float]) -> Tuple[float, float]:
        return point[0] * inverse_scale, point[1] * inverse_scale

    return PipeDetection(
        center=scaled_point(detection.center),
        endpoint_1=scaled_point(detection.endpoint_1),
        endpoint_2=scaled_point(detection.endpoint_2),
        angle_deg=detection.angle_deg,
        length_px=detection.length_px * inverse_scale,
        width_px=detection.width_px * inverse_scale,
        aspect_ratio=detection.aspect_ratio,
        score=detection.score * inverse_scale,
        method=detection.method,
    )


def light_processing_config(
    config: LightDetectorConfig,
) -> LightDetectorConfig:
    scale = config.process_scale
    if scale == 1.0:
        return config
    return replace(
        config,
        process_scale=1.0,
        blur_kernel=scaled_odd_kernel(config.blur_kernel, scale),
        morph_kernel=scaled_odd_kernel(config.morph_kernel, scale),
        min_length_px=config.min_length_px * scale,
        min_width_px=config.min_width_px * scale,
        max_width_px=config.max_width_px * scale,
        min_contour_area=config.min_contour_area * scale * scale,
    )


def light_contour_detection(
    contour: np.ndarray,
    image_area: float,
    config: LightDetectorConfig,
) -> Optional[PipeDetection]:
    """从二值连通区域中筛出连续的细长矩形并拟合中轴。"""
    contour_area = float(cv2.contourArea(contour))
    if (
        contour_area < config.min_contour_area
        or contour_area / max(image_area, 1.0) > config.max_area_ratio
    ):
        return None

    rectangle = cv2.minAreaRect(contour)
    rectangle_width, rectangle_height = rectangle[1]
    long_side = max(float(rectangle_width), float(rectangle_height))
    short_side = min(float(rectangle_width), float(rectangle_height))
    if short_side < config.min_width_px or short_side > config.max_width_px:
        return None
    aspect_ratio = long_side / max(short_side, 1e-6)
    if (
        long_side < config.min_length_px
        or aspect_ratio < config.min_aspect_ratio
    ):
        return None
    fill_ratio = contour_area / max(long_side * short_side, 1e-6)
    if fill_ratio < config.min_fill_ratio:
        return None

    points = contour.reshape(-1, 2).astype(np.float64)
    vx, vy, x0, y0 = [
        float(value)
        for value in cv2.fitLine(
            points.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01
        ).reshape(-1)
    ]
    direction = np.array([vx, vy], dtype=np.float64)
    direction /= max(float(np.linalg.norm(direction)), 1e-12)
    if direction[0] < 0:
        direction = -direction
    origin = np.array([x0, y0], dtype=np.float64)
    projections = (points - origin).dot(direction)
    endpoint_1 = origin + direction * float(np.min(projections))
    endpoint_2 = origin + direction * float(np.max(projections))
    length = float(np.linalg.norm(endpoint_2 - endpoint_1))
    center = (endpoint_1 + endpoint_2) / 2.0
    angle = normalized_angle_deg(
        math.degrees(math.atan2(direction[1], direction[0]))
    )
    score = length * min(aspect_ratio, 25.0) * fill_ratio
    return PipeDetection(
        center=(float(center[0]), float(center[1])),
        endpoint_1=(float(endpoint_1[0]), float(endpoint_1[1])),
        endpoint_2=(float(endpoint_2[0]), float(endpoint_2[1])),
        angle_deg=angle,
        length_px=length,
        width_px=short_side,
        aspect_ratio=aspect_ratio,
        score=score,
        method="binary_fitline",
    )


def detect_ppr_pipe_lightweight(
    frame_bgr: np.ndarray,
    config: LightDetectorConfig,
) -> Tuple[Optional[PipeDetection], Dict[str, np.ndarray], Dict[str, int]]:
    """完全独立的轻量检测，不调用Canny、Hough或重方案候选逻辑。"""
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3:
        raise ValueError("输入必须是 BGR 彩色图像。")
    if not 0.0 < config.process_scale <= 1.0:
        raise ValueError("process_scale 必须位于 (0, 1]。")
    active = light_processing_config(config)
    if config.process_scale < 1.0:
        processing_frame = cv2.resize(
            frame_bgr,
            None,
            fx=config.process_scale,
            fy=config.process_scale,
            interpolation=cv2.INTER_AREA,
        )
    else:
        processing_frame = frame_bgr

    gray = cv2.cvtColor(processing_frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(
        gray, (active.blur_kernel, active.blur_kernel), 0
    )
    _threshold, binary_bright = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    binary_dark = cv2.bitwise_not(binary_bright)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (active.morph_kernel, active.morph_kernel)
    )
    masks = []
    candidates = []
    image_area = float(gray.shape[0] * gray.shape[1])
    for polarity, binary in (
        ("bright", binary_bright),
        ("dark", binary_dark),
    ):
        cleaned = cv2.morphologyEx(
            binary,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=active.morph_iterations,
        )
        masks.append((polarity, cleaned))
        contours, _hierarchy = cv2.findContours(
            cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = sorted(
            contours, key=cv2.contourArea, reverse=True
        )[: active.max_contours]
        for contour in contours:
            detection = light_contour_detection(
                contour, image_area, active
            )
            if detection is not None:
                candidates.append((polarity, detection))

    selected_pair = (
        max(candidates, key=lambda item: item[1].score)
        if candidates
        else None
    )
    selected = selected_pair[1] if selected_pair is not None else None
    selected = rescale_detection(selected, 1.0 / config.process_scale)
    selected_polarity = (
        selected_pair[0] if selected_pair is not None else "none"
    )
    debug_images = {
        "gray": gray,
        "binary_bright": masks[0][1],
        "binary_dark": masks[1][1],
    }
    counts = {
        "contour_candidates": len(candidates),
        "hough_candidates": 0,
        "binary_polarity": selected_polarity,
    }
    return selected, debug_images, counts


def contour_detection(
    contour: np.ndarray, config: DetectorConfig
) -> Optional[PipeDetection]:
    if cv2.contourArea(contour) < config.min_contour_area:
        return None
    rectangle = cv2.minAreaRect(contour)
    rectangle_width, rectangle_height = rectangle[1]
    long_side = max(float(rectangle_width), float(rectangle_height))
    short_side = min(float(rectangle_width), float(rectangle_height))
    if short_side < config.min_width_px or short_side > config.max_width_px:
        return None
    aspect_ratio = long_side / max(short_side, 1e-6)
    if (
        long_side < config.min_length_px
        or aspect_ratio < config.min_aspect_ratio
    ):
        return None

    points = contour.reshape(-1, 2).astype(np.float64)
    vx, vy, x0, y0 = [
        float(value) for value in cv2.fitLine(
            points.astype(np.float32),
            cv2.DIST_L2,
            0,
            0.01,
            0.01,
        ).reshape(-1)
    ]
    direction = np.array([vx, vy], dtype=np.float64)
    direction /= max(np.linalg.norm(direction), 1e-12)
    if direction[0] < 0:
        direction = -direction
    origin = np.array([x0, y0], dtype=np.float64)
    projections = (points - origin).dot(direction)
    endpoint_1 = origin + direction * float(np.min(projections))
    endpoint_2 = origin + direction * float(np.max(projections))
    length = float(np.linalg.norm(endpoint_2 - endpoint_1))
    center = (endpoint_1 + endpoint_2) / 2.0
    angle = normalized_angle_deg(
        math.degrees(math.atan2(direction[1], direction[0]))
    )
    score = length * min(aspect_ratio, 25.0)
    return PipeDetection(
        center=(float(center[0]), float(center[1])),
        endpoint_1=(float(endpoint_1[0]), float(endpoint_1[1])),
        endpoint_2=(float(endpoint_2[0]), float(endpoint_2[1])),
        angle_deg=angle,
        length_px=length,
        width_px=short_side,
        aspect_ratio=aspect_ratio,
        score=score,
        method="contour",
    )


def find_contour_candidates(
    closed_edges: np.ndarray, config: DetectorConfig
) -> List[PipeDetection]:
    contours, _ = cv2.findContours(
        closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    candidates = []
    for contour in contours:
        detection = contour_detection(contour, config)
        if detection is not None:
            candidates.append(detection)
    return candidates


def hough_segments(
    edges: np.ndarray, config: DetectorConfig
) -> List[LineSegment]:
    raw_lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=config.hough_threshold,
        minLineLength=config.hough_min_line_length,
        maxLineGap=config.hough_max_line_gap,
    )
    if raw_lines is None:
        return []

    segments = []
    for line in raw_lines.reshape(-1, 4):
        p1 = np.array(line[:2], dtype=np.float64)
        p2 = np.array(line[2:], dtype=np.float64)
        delta = p2 - p1
        length = float(np.linalg.norm(delta))
        if length < config.hough_min_line_length:
            continue
        direction = delta / length
        if direction[0] < 0:
            direction = -direction
            p1, p2 = p2, p1
        angle = normalized_angle_deg(
            math.degrees(math.atan2(direction[1], direction[0]))
        )
        segments.append(LineSegment(p1, p2, direction, length, angle))
    segments.sort(key=lambda segment: segment.length, reverse=True)
    return segments[: config.max_hough_segments]


def paired_line_detection(
    first: LineSegment,
    second: LineSegment,
    config: DetectorConfig,
) -> Optional[PipeDetection]:
    if (
        line_angle_difference_deg(first.angle_deg, second.angle_deg)
        > config.max_parallel_angle_deg
    ):
        return None

    direction = first.direction + second.direction
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        return None
    direction /= norm
    if direction[0] < 0:
        direction = -direction
    normal = np.array([-direction[1], direction[0]], dtype=np.float64)

    first_points = np.vstack((first.p1, first.p2))
    second_points = np.vstack((second.p1, second.p2))
    first_t = sorted(first_points.dot(direction))
    second_t = sorted(second_points.dot(direction))
    overlap = max(
        0.0, min(first_t[1], second_t[1]) - max(first_t[0], second_t[0])
    )
    overlap_ratio = overlap / max(min(first.length, second.length), 1e-6)
    if overlap_ratio < config.min_line_overlap_ratio:
        return None

    first_normal = float(np.mean(first_points.dot(normal)))
    second_normal = float(np.mean(second_points.dot(normal)))
    width = abs(second_normal - first_normal)
    if width < config.min_width_px or width > config.max_width_px:
        return None

    all_points = np.vstack((first_points, second_points))
    all_t = all_points.dot(direction)
    start_t = float(np.min(all_t))
    end_t = float(np.max(all_t))
    center_normal = (first_normal + second_normal) / 2.0
    endpoint_1 = direction * start_t + normal * center_normal
    endpoint_2 = direction * end_t + normal * center_normal
    length = float(end_t - start_t)
    aspect_ratio = length / max(width, 1e-6)
    if (
        length < config.min_length_px
        or aspect_ratio < config.min_aspect_ratio
    ):
        return None

    center = (endpoint_1 + endpoint_2) / 2.0
    angle = normalized_angle_deg(
        math.degrees(math.atan2(direction[1], direction[0]))
    )
    parallel_quality = max(
        0.0,
        1.0
        - line_angle_difference_deg(first.angle_deg, second.angle_deg)
        / max(config.max_parallel_angle_deg, 1e-6),
    )
    score = (
        length
        * min(aspect_ratio, 25.0)
        * (0.5 + 0.5 * overlap_ratio)
        * (0.7 + 0.3 * parallel_quality)
    )
    return PipeDetection(
        center=(float(center[0]), float(center[1])),
        endpoint_1=(float(endpoint_1[0]), float(endpoint_1[1])),
        endpoint_2=(float(endpoint_2[0]), float(endpoint_2[1])),
        angle_deg=angle,
        length_px=length,
        width_px=width,
        aspect_ratio=aspect_ratio,
        score=score,
        method="hough_pair",
    )


def find_hough_candidates(
    edges: np.ndarray, config: DetectorConfig
) -> List[PipeDetection]:
    segments = hough_segments(edges, config)
    candidates = []
    for first_index in range(len(segments)):
        for second_index in range(first_index + 1, len(segments)):
            detection = paired_line_detection(
                segments[first_index], segments[second_index], config
            )
            if detection is not None:
                candidates.append(detection)
    return candidates


def candidate_center_distance(
    first: PipeDetection, second: PipeDetection
) -> float:
    return float(
        np.linalg.norm(np.asarray(first.center) - np.asarray(second.center))
    )


def choose_detection(
    contour_candidates: Sequence[PipeDetection],
    hough_candidates: Sequence[PipeDetection],
) -> Optional[PipeDetection]:
    if not contour_candidates and not hough_candidates:
        return None
    if not contour_candidates:
        return max(hough_candidates, key=lambda candidate: candidate.score)

    best_contour = max(
        contour_candidates, key=lambda candidate: candidate.score
    )
    matching_hough = [
        candidate
        for candidate in hough_candidates
        if line_angle_difference_deg(
            candidate.angle_deg, best_contour.angle_deg
        )
        <= 8.0
        and candidate_center_distance(candidate, best_contour)
        <= max(60.0, 2.5 * best_contour.width_px)
        and 0.55
        <= candidate.length_px / max(best_contour.length_px, 1e-6)
        <= 1.45
    ]
    if not matching_hough:
        return best_contour

    best_hough = max(matching_hough, key=lambda candidate: candidate.score)
    return PipeDetection(
        center=best_hough.center,
        endpoint_1=best_hough.endpoint_1,
        endpoint_2=best_hough.endpoint_2,
        angle_deg=best_hough.angle_deg,
        length_px=best_hough.length_px,
        width_px=best_hough.width_px,
        aspect_ratio=best_hough.aspect_ratio,
        score=max(best_contour.score, best_hough.score),
        method="contour+hough",
    )


def detect_ppr_pipe(
    frame_bgr: np.ndarray, config: DetectorConfig
) -> Tuple[Optional[PipeDetection], Dict[str, np.ndarray], Dict[str, int]]:
    if not 0.0 < config.process_scale <= 1.0:
        raise ValueError("process_scale 必须位于 (0, 1]。")
    active_config = processing_config(config)
    if config.process_scale < 1.0:
        processing_frame = cv2.resize(
            frame_bgr,
            None,
            fx=config.process_scale,
            fy=config.process_scale,
            interpolation=cv2.INTER_AREA,
        )
    else:
        processing_frame = frame_bgr

    debug_images = preprocess(processing_frame, active_config)
    contour_candidates = find_contour_candidates(
        debug_images["closed"], active_config
    )
    hough_candidates = (
        find_hough_candidates(debug_images["edges"], active_config)
        if active_config.enable_hough
        else []
    )
    detection = choose_detection(contour_candidates, hough_candidates)
    detection = rescale_detection(
        detection, 1.0 / config.process_scale
    )
    counts = {
        "contour_candidates": len(contour_candidates),
        "hough_candidates": len(hough_candidates),
    }
    return detection, debug_images, counts


def detection_record(
    frame_index: int,
    timestamp_ms: float,
    detection: Optional[PipeDetection],
    counts: Dict[str, int],
) -> Dict[str, Any]:
    record: Dict[str, Any] = {
        "frame": frame_index,
        "timestamp_ms": round(timestamp_ms, 3),
        "valid": detection is not None,
        **counts,
    }
    if detection is None:
        record["reason"] = "ppr_pipe_not_detected"
        return record
    values = asdict(detection)
    for point_name in ("center", "endpoint_1", "endpoint_2"):
        point = values[point_name]
        values[point_name] = {
            "u": round(float(point[0]), 3),
            "v": round(float(point[1]), 3),
        }
    for name in (
        "angle_deg",
        "length_px",
        "width_px",
        "aspect_ratio",
        "score",
    ):
        values[name] = round(float(values[name]), 4)
    record.update(values)
    return record


def draw_detection(
    frame_bgr: np.ndarray,
    detection: Optional[PipeDetection],
    fps: float,
    counts: Dict[str, int],
) -> np.ndarray:
    annotated = frame_bgr.copy()
    if detection is not None:
        p1 = tuple(int(round(value)) for value in detection.endpoint_1)
        p2 = tuple(int(round(value)) for value in detection.endpoint_2)
        center = tuple(int(round(value)) for value in detection.center)
        cv2.line(annotated, p1, p2, (0, 255, 0), 3, cv2.LINE_AA)
        cv2.circle(annotated, p1, 6, (255, 0, 255), -1)
        cv2.circle(annotated, p2, 6, (255, 0, 255), -1)
        cv2.drawMarker(
            annotated, center, (0, 255, 255), cv2.MARKER_CROSS, 22, 2
        )
        label = "{} angle={:.1f} L={:.0f} W={:.1f}".format(
            detection.method,
            detection.angle_deg,
            detection.length_px,
            detection.width_px,
        )
        cv2.putText(
            annotated,
            label,
            (10, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    status = "FPS {:.1f} | contour {} | hough {}".format(
        fps, counts["contour_candidates"], counts["hough_candidates"]
    )
    cv2.putText(
        annotated,
        status,
        (10, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return annotated


def odd_positive(value: int, name: str) -> int:
    if value <= 0 or value % 2 == 0:
        raise ValueError("{} 必须是正奇数。".format(name))
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="H 题 PPR 细长管道识别")
    parser.add_argument(
        "--mode",
        choices=("full", "light"),
        default="full",
        help="full=轮廓+霍夫校正；light=轻量轮廓方案，默认full",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument(
        "--process-scale",
        type=float,
        default=None,
        help=(
            "检测图缩放比例；不指定时full使用0.5，light使用0.35，"
            "结果均映射回原图"
        ),
    )
    parser.add_argument("--blur-kernel", type=int, default=5)
    parser.add_argument("--light-morph-kernel", type=int, default=5)
    parser.add_argument("--light-min-fill-ratio", type=float, default=0.35)
    parser.add_argument("--max-light-contours", type=int, default=64)
    parser.add_argument("--canny-low", type=int, default=50)
    parser.add_argument("--canny-high", type=int, default=150)
    parser.add_argument("--close-kernel", type=int, default=9)
    parser.add_argument("--close-iterations", type=int, default=2)
    parser.add_argument("--min-length", type=float, default=180.0)
    parser.add_argument("--min-width", type=float, default=5.0)
    parser.add_argument("--max-width", type=float, default=100.0)
    parser.add_argument("--min-aspect-ratio", type=float, default=5.0)
    parser.add_argument("--min-contour-area", type=float, default=300.0)
    parser.add_argument("--hough-threshold", type=int, default=70)
    parser.add_argument("--hough-min-line-length", type=float, default=140.0)
    parser.add_argument("--hough-max-line-gap", type=float, default=30.0)
    parser.add_argument("--max-parallel-angle", type=float, default=6.0)
    parser.add_argument("--min-line-overlap-ratio", type=float, default=0.45)
    parser.add_argument("--max-hough-segments", type=int, default=24)
    parser.add_argument("--no-hough", action="store_true")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument(
        "--display-every",
        type=int,
        default=2,
        help="主窗口每N帧刷新一次，默认2，降低桌面渲染压力",
    )
    parser.add_argument(
        "--opencv-threads",
        type=int,
        default=2,
        help="OpenCV最多使用的CPU线程数，默认2",
    )
    parser.add_argument("--show-debug", action="store_true")
    parser.add_argument(
        "--debug-every",
        type=int,
        default=5,
        help="调试边缘窗口每N帧刷新一次，默认5",
    )
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument(
        "--print-every",
        type=int,
        default=0,
        help="每N帧打印JSON；默认0，不向终端连续刷屏",
    )
    parser.add_argument("--jsonl", type=Path)
    parser.add_argument(
        "--stream-host",
        help="可选：接收视频的PC局域网IP；不指定则不推流",
    )
    parser.add_argument("--stream-port", type=int, default=5600)
    parser.add_argument("--stream-fps", type=int, default=30)
    parser.add_argument("--stream-bitrate", type=int, default=2_000_000)
    return parser.parse_args()


def config_from_args(args: argparse.Namespace) -> Any:
    odd_positive(args.blur_kernel, "blur-kernel")
    odd_positive(args.close_kernel, "close-kernel")
    odd_positive(args.light_morph_kernel, "light-morph-kernel")
    process_scale = (
        args.process_scale
        if args.process_scale is not None
        else 0.5
    )
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise ValueError("图像尺寸和帧率必须大于 0。")
    if not 0.1 <= process_scale <= 1.0:
        raise ValueError("process-scale 必须位于 [0.1, 1]。")
    if not 0 <= args.canny_low < args.canny_high <= 255:
        raise ValueError("Canny阈值必须满足 0 <= low < high <= 255。")
    if args.close_iterations < 1:
        raise ValueError("close-iterations 至少为 1。")
    if args.min_length <= 0 or not 0 < args.min_width < args.max_width:
        raise ValueError("长度或宽度阈值无效。")
    if args.min_aspect_ratio <= 1.0:
        raise ValueError("min-aspect-ratio 必须大于 1。")
    if not 0.0 < args.light_min_fill_ratio <= 1.0:
        raise ValueError("light-min-fill-ratio 必须位于 (0, 1]。")
    if args.print_every < 0 or args.max_frames < 0:
        raise ValueError("print-every 和 max-frames 不能为负数。")
    if args.debug_every < 1 or args.max_hough_segments < 2:
        raise ValueError("debug-every至少为1，max-hough-segments至少为2。")
    if (
        args.display_every < 1
        or args.opencv_threads < 1
        or args.max_light_contours < 1
    ):
        raise ValueError(
            "display-every、opencv-threads和max-light-contours至少为1。"
        )
    if args.mode == "light":
        return LightDetectorConfig(
            process_scale=process_scale,
            blur_kernel=args.blur_kernel,
            morph_kernel=args.light_morph_kernel,
            morph_iterations=1,
            min_length_px=args.min_length,
            min_width_px=args.min_width,
            max_width_px=args.max_width,
            min_aspect_ratio=args.min_aspect_ratio,
            min_contour_area=args.min_contour_area,
            min_fill_ratio=args.light_min_fill_ratio,
            max_contours=args.max_light_contours,
        )

    return DetectorConfig(
        process_scale=process_scale,
        blur_kernel=args.blur_kernel,
        canny_low=args.canny_low,
        canny_high=args.canny_high,
        close_kernel=args.close_kernel,
        close_iterations=args.close_iterations,
        min_length_px=args.min_length,
        min_width_px=args.min_width,
        max_width_px=args.max_width,
        min_aspect_ratio=args.min_aspect_ratio,
        min_contour_area=args.min_contour_area,
        hough_threshold=args.hough_threshold,
        hough_min_line_length=args.hough_min_line_length,
        hough_max_line_gap=args.hough_max_line_gap,
        max_parallel_angle_deg=args.max_parallel_angle,
        min_line_overlap_ratio=args.min_line_overlap_ratio,
        max_hough_segments=args.max_hough_segments,
        enable_hough=not args.no_hough,
    )


def main() -> int:
    args = parse_args()
    try:
        detector_config = config_from_args(args)
    except ValueError as error:
        print("配置错误：{}".format(error), file=sys.stderr)
        return 2
    cv2.setNumThreads(args.opencv_threads)

    pipeline = rs.pipeline()
    stream_config = rs.config()
    stream_config.enable_stream(
        rs.stream.color,
        args.width,
        args.height,
        rs.format.bgr8,
        args.fps,
    )
    pipeline_started = False
    jsonl_file = None
    streamer = None
    try:
        if args.stream_host:
            streamer = UdpH264Streamer(
                StreamConfig(
                    host=args.stream_host,
                    port=args.stream_port,
                    width=args.width,
                    height=args.height,
                    fps=args.stream_fps,
                    bitrate=args.stream_bitrate,
                )
            )
            print(
                "视频推流：rtp://{}:{}，H.264 {} FPS，{} bit/s"
                .format(
                    args.stream_host,
                    args.stream_port,
                    args.stream_fps,
                    args.stream_bitrate,
                ),
                file=sys.stderr,
            )
        if args.jsonl is not None:
            output_path = args.jsonl.expanduser().resolve()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            jsonl_file = output_path.open("a", encoding="utf-8")

        profile = pipeline.start(stream_config)
        pipeline_started = True
        device = profile.get_device()
        print(
            "RealSense：{}，序列号 {}，彩色流 {}x{}@{}，模式 {}，处理比例 {}"
            .format(
                device.get_info(rs.camera_info.name),
                device.get_info(rs.camera_info.serial_number),
                args.width,
                args.height,
                args.fps,
                args.mode,
                detector_config.process_scale,
            ),
            file=sys.stderr,
        )
        for _ in range(15):
            pipeline.wait_for_frames(5000)

        frame_index = 0
        last_time = time.perf_counter()
        smoothed_fps = 0.0
        while args.max_frames == 0 or frame_index < args.max_frames:
            frames = pipeline.wait_for_frames(5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
            processing_start = time.perf_counter()
            if args.mode == "light":
                detection, debug_images, counts = (
                    detect_ppr_pipe_lightweight(frame, detector_config)
                )
            else:
                detection, debug_images, counts = detect_ppr_pipe(
                    frame, detector_config
                )
            processing_ms = (
                time.perf_counter() - processing_start
            ) * 1000.0
            counts["detector_mode"] = args.mode
            counts["processing_ms"] = round(processing_ms, 3)

            frame_index += 1
            now = time.perf_counter()
            instant_fps = 1.0 / max(now - last_time, 1e-6)
            smoothed_fps = (
                instant_fps
                if smoothed_fps == 0.0
                else 0.9 * smoothed_fps + 0.1 * instant_fps
            )
            last_time = now
            record = detection_record(
                frame_index, time.time() * 1000.0, detection, counts
            )
            line = json.dumps(
                record, ensure_ascii=False, separators=(",", ":")
            )
            if args.print_every > 0 and frame_index % args.print_every == 0:
                print(line, flush=True)
            if jsonl_file is not None:
                jsonl_file.write(line + "\n")
                jsonl_file.flush()

            if not args.no_display:
                if frame_index % args.display_every == 0:
                    annotated = draw_detection(
                        frame, detection, smoothed_fps, counts
                    )
                    cv2.imshow("H - PPR Pipe Detector", annotated)
                    if (
                        args.show_debug
                        and frame_index % args.debug_every == 0
                    ):
                        if args.mode == "light":
                            cv2.imshow(
                                "PPR Binary - Bright",
                                debug_images["binary_bright"],
                            )
                            cv2.imshow(
                                "PPR Binary - Dark",
                                debug_images["binary_dark"],
                            )
                        else:
                            cv2.imshow(
                                "PPR Edges", debug_images["edges"]
                            )
                            cv2.imshow(
                                "PPR Closed Edges",
                                debug_images["closed"],
                            )
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            if streamer is not None:
                # 局域网只发送纯摄像头画面，不叠加管道轴线或检测信息。
                streamer.send(frame)
                if streamer.error is not None:
                    print(
                        "推流错误：{}".format(streamer.error),
                        file=sys.stderr,
                    )
                    streamer.close()
                    streamer = None
    except (RuntimeError, StreamError) as error:
        print("运行错误：{}".format(error), file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        pass
    finally:
        if pipeline_started:
            pipeline.stop()
        if jsonl_file is not None:
            jsonl_file.close()
        if streamer is not None:
            streamer.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
