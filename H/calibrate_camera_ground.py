#!/usr/bin/env python3
"""通过RealSense观测水平地面，估计相机高度、俯仰角和横滚角。"""

import argparse
import json
import math
import sys
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs


@dataclass(frozen=True)
class GroundEstimate:
    normal_camera: np.ndarray
    height_m: float
    pitch_down_deg: float
    roll_deg: float
    inlier_ratio: float
    rmse_m: float
    point_count: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="用水平地面反推RealSense安装高度、俯仰角和横滚角",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--min-depth", type=float, default=0.15)
    parser.add_argument("--max-depth", type=float, default=3.0)
    parser.add_argument("--ransac-threshold", type=float, default=0.01)
    parser.add_argument("--ransac-iterations", type=int, default=100)
    parser.add_argument("--max-points", type=int, default=12000)
    parser.add_argument("--min-inlier-ratio", type=float, default=0.55)
    parser.add_argument("--estimate-every", type=int, default=10)
    parser.add_argument("--history-size", type=int, default=10)
    parser.add_argument("--display-every", type=int, default=10)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument(
        "--auto-frames",
        type=int,
        default=0,
        help="非零时采集到指定帧数后自动输出建议并退出",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise ValueError("图像尺寸和帧率必须大于0。")
    if not 0 < args.min_depth < args.max_depth:
        raise ValueError("深度范围无效。")
    if args.ransac_threshold <= 0 or args.ransac_iterations < 10:
        raise ValueError("RANSAC阈值必须大于0，迭代次数至少为10。")
    if args.max_points < 100 or not 0 < args.min_inlier_ratio <= 1:
        raise ValueError("max-points至少100，min-inlier-ratio应位于(0,1]。")
    if (
        args.estimate_every < 1
        or args.history_size < 3
        or args.display_every < 1
        or args.auto_frames < 0
    ):
        raise ValueError("帧间隔至少1、历史长度至少3，auto-frames不能为负。")


def depth_points(
    depth_frame: rs.depth_frame,
    pointcloud: rs.pointcloud,
    min_depth_m: float,
    max_depth_m: float,
    max_points: int,
    rng: np.random.Generator,
) -> np.ndarray:
    points = pointcloud.calculate(depth_frame)
    vertices = np.asanyarray(points.get_vertices())
    xyz = vertices.view(np.float32).reshape(-1, 3).astype(np.float64)
    valid = (
        np.all(np.isfinite(xyz), axis=1)
        & (xyz[:, 2] >= min_depth_m)
        & (xyz[:, 2] <= max_depth_m)
    )
    xyz = xyz[valid]
    if len(xyz) > max_points:
        indices = rng.choice(len(xyz), size=max_points, replace=False)
        xyz = xyz[indices]
    return xyz


def fit_ground_plane(
    points_camera: np.ndarray,
    threshold_m: float = 0.01,
    iterations: int = 100,
    min_inlier_ratio: float = 0.55,
    rng: Optional[np.random.Generator] = None,
) -> GroundEstimate:
    points = np.asarray(points_camera, dtype=np.float64)
    valid = np.all(np.isfinite(points), axis=1)
    points = points[valid]
    if len(points) < 100:
        raise ValueError("有效地面点不足100个。")
    if rng is None:
        rng = np.random.default_rng()

    best_mask = None
    best_count = 0
    for _ in range(iterations):
        sample = points[rng.choice(len(points), size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal /= norm
        offset = -float(np.dot(normal, sample[0]))
        mask = np.abs(points.dot(normal) + offset) <= threshold_m
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_count = count
            best_mask = mask

    if best_mask is None or best_count < 100:
        raise ValueError("RANSAC未找到有效地面平面。")
    inlier_ratio = best_count / len(points)
    if inlier_ratio < min_inlier_ratio:
        raise ValueError(
            "地面内点比例{:.1%}低于阈值{:.1%}，请让画面只包含地面。"
            .format(inlier_ratio, min_inlier_ratio)
        )

    inliers = points[best_mask]
    centroid = np.mean(inliers, axis=0)
    _, _, vh = np.linalg.svd(inliers - centroid, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    # 地面朝上的法向量在RealSense光学系中应主要指向画面上方，即-Y。
    if normal[1] > 0:
        normal = -normal
    offset = -float(np.dot(normal, centroid))
    if offset <= 0:
        raise ValueError("估计高度非正，请确认镜头朝向地面且画面没有墙面。")

    residuals = inliers.dot(normal) + offset
    rmse_m = float(np.sqrt(np.mean(residuals ** 2)))
    pitch_down_deg = math.degrees(
        math.atan2(-float(normal[2]), -float(normal[1]))
    )
    # 地面法向量可确定倾斜的两个自由度。roll按画面横向倾斜定义：
    # roll=atan2(n_x, -n_y)，pitch为光轴相对水平面的下俯角。
    roll_deg = math.degrees(
        math.atan2(float(normal[0]), -float(normal[1]))
    )
    return GroundEstimate(
        normal_camera=normal,
        height_m=offset,
        pitch_down_deg=pitch_down_deg,
        roll_deg=roll_deg,
        inlier_ratio=inlier_ratio,
        rmse_m=rmse_m,
        point_count=len(points),
    )


def suggested_transform(
    height_m: float,
    pitch_down_deg: float,
    roll_deg: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray]:
    pitch = math.radians(pitch_down_deg)
    roll = math.radians(roll_deg)
    # 底座+Z在相机光学系中的方向，即旋转矩阵第三行。
    normal_camera = np.array(
        [
            math.cos(pitch) * math.sin(roll),
            -math.cos(pitch) * math.cos(roll),
            -math.sin(pitch),
        ]
    )
    # yaw=0时，将相机光轴在地面上的投影定义为底座+X。
    camera_forward = np.array([0.0, 0.0, 1.0])
    base_x_camera = (
        camera_forward
        - float(np.dot(camera_forward, normal_camera)) * normal_camera
    )
    base_x_camera /= np.linalg.norm(base_x_camera)
    base_y_camera = np.cross(normal_camera, base_x_camera)
    rotation = np.stack([base_x_camera, base_y_camera, normal_camera])
    translation = np.array([0.0, 0.0, height_m], dtype=np.float64)
    return rotation, translation


def robust_estimate(history: Deque[GroundEstimate]) -> GroundEstimate:
    if len(history) < 3:
        raise ValueError("至少需要3次有效地面估计。")
    normals = np.stack([item.normal_camera for item in history])
    normal = np.median(normals, axis=0)
    normal /= np.linalg.norm(normal)
    heights = np.array([item.height_m for item in history])
    pitches = np.array([item.pitch_down_deg for item in history])
    rolls = np.array([item.roll_deg for item in history])
    ratios = np.array([item.inlier_ratio for item in history])
    rmses = np.array([item.rmse_m for item in history])
    counts = np.array([item.point_count for item in history])
    return GroundEstimate(
        normal_camera=normal,
        height_m=float(np.median(heights)),
        pitch_down_deg=float(np.median(pitches)),
        roll_deg=float(np.median(rolls)),
        inlier_ratio=float(np.median(ratios)),
        rmse_m=float(np.median(rmses)),
        point_count=int(np.median(counts)),
    )


def print_recommendation(estimate: GroundEstimate) -> None:
    rotation, translation = suggested_transform(
        estimate.height_m,
        estimate.pitch_down_deg,
        estimate.roll_deg,
    )
    homogeneous = np.eye(4)
    homogeneous[:3, :3] = rotation
    homogeneous[:3, 3] = translation
    payload = {
        "_description": (
            "地面平面估计建议；假设底座+X前、+Y左、+Z上，"
            "镜头朝底座正前方，水平平移为零。"
        ),
        "source_frame": "camera_optical",
        "target_frame": "camera_base",
        "rotation_matrix": np.round(rotation, 9).tolist(),
        "translation_m": np.round(translation, 6).tolist(),
    }
    print("\n========== 地面标定建议 ==========")
    print("相机高度：{:.4f} m（{:.2f} cm）".format(
        estimate.height_m, estimate.height_m * 100.0
    ))
    print("镜头下俯角：{:.2f} deg".format(estimate.pitch_down_deg))
    print("横滚角：{:.2f} deg".format(estimate.roll_deg))
    print(
        "安装建议：z={:.4f} m，roll={:.2f} deg，pitch={:.2f} deg，"
        "yaw固定0 deg".format(
            estimate.height_m,
            estimate.roll_deg,
            estimate.pitch_down_deg,
        )
    )
    print("地面法向量（相机系）：{}".format(
        np.round(estimate.normal_camera, 6).tolist()
    ))
    print("内点比例：{:.1%}，平面RMSE：{:.2f} mm，点数：{}".format(
        estimate.inlier_ratio, estimate.rmse_m * 1000.0, estimate.point_count
    ))
    print("\n完整齐次矩阵 T_base_from_camera：")
    print(np.array2string(
        homogeneous,
        precision=9,
        suppress_small=True,
    ))
    print("\n建议 camera_to_base.json：")
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    print("==================================\n")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except ValueError as error:
        print("配置错误：{}".format(error), file=sys.stderr)
        return 2

    cv2.setNumThreads(1)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.depth,
        args.width,
        args.height,
        rs.format.z16,
        args.fps,
    )
    pointcloud = rs.pointcloud()
    colorizer = rs.colorizer()
    rng = np.random.default_rng()
    history: Deque[GroundEstimate] = deque(maxlen=args.history_size)
    pipeline_started = False

    try:
        profile = pipeline.start(config)
        pipeline_started = True
        device = profile.get_device()
        print(
            "RealSense地面标定：{}，序列号 {}，深度流 {}x{}@{}".format(
                device.get_info(rs.camera_info.name),
                device.get_info(rs.camera_info.serial_number),
                args.width,
                args.height,
                args.fps,
            ),
            file=sys.stderr,
        )
        print(
            "让画面只包含水平地面；按空格/S输出建议，Q/Esc退出。",
            file=sys.stderr,
        )
        for _ in range(15):
            pipeline.wait_for_frames(5000)

        frame_index = 0
        last_error = ""
        while args.auto_frames == 0 or frame_index < args.auto_frames:
            frames = pipeline.wait_for_frames(5000)
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            frame_index += 1

            if frame_index % args.estimate_every == 0:
                try:
                    points = depth_points(
                        depth_frame,
                        pointcloud,
                        args.min_depth,
                        args.max_depth,
                        args.max_points,
                        rng,
                    )
                    estimate = fit_ground_plane(
                        points,
                        threshold_m=args.ransac_threshold,
                        iterations=args.ransac_iterations,
                        min_inlier_ratio=args.min_inlier_ratio,
                        rng=rng,
                    )
                    history.append(estimate)
                    last_error = ""
                    print(
                        (
                            "地面估计：高度={:.2f}cm 下俯={:.2f}deg "
                            "横滚={:.2f}deg 内点={:.1%} RMSE={:.2f}mm "
                            "稳定样本={}/{}"
                        ).format(
                            estimate.height_m * 100.0,
                            estimate.pitch_down_deg,
                            estimate.roll_deg,
                            estimate.inlier_ratio,
                            estimate.rmse_m * 1000.0,
                            len(history),
                            args.history_size,
                        ),
                        flush=True,
                    )
                except ValueError as error:
                    last_error = str(error)
                    print("本次估计无效：{}".format(error), file=sys.stderr)

            key = 255
            if not args.no_display and frame_index % args.display_every == 0:
                preview = np.asanyarray(
                    colorizer.colorize(depth_frame).get_data()
                )
                preview = cv2.resize(
                    preview, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA
                )
                status = (
                    "SPACE/S: result | Q/ESC: quit | valid {}/{}"
                    .format(len(history), args.history_size)
                )
                cv2.putText(
                    preview,
                    status,
                    (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                if last_error:
                    cv2.putText(
                        preview,
                        last_error[:50],
                        (8, 44),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.40,
                        (0, 0, 255),
                        1,
                        cv2.LINE_AA,
                    )
                cv2.imshow("H - Ground Calibration", preview)
                key = cv2.pollKey() & 0xFF

            if key in (ord("q"), 27):
                break
            if key in (ord("s"), ord("S"), 32):
                try:
                    print_recommendation(robust_estimate(history))
                except ValueError as error:
                    print("暂不能输出建议：{}".format(error), file=sys.stderr)

        if args.auto_frames > 0:
            print_recommendation(robust_estimate(history))
    except (RuntimeError, ValueError) as error:
        print("运行错误：{}".format(error), file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        if len(history) >= 3:
            print_recommendation(robust_estimate(history))
    finally:
        if pipeline_started:
            pipeline.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
