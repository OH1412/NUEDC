#!/usr/bin/env python3
"""YOLO + RealSense：输出钢珠像素、深度、摄像头坐标和底座坐标。"""

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from ultralytics import YOLO

from depth_utils import DepthError, sample_depth
from geometry import RigidTransform, TransformError, load_transform
from udp_video_stream import StreamConfig, StreamError, UdpH264Streamer


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_TRANSFORM = SCRIPT_DIR / "camera_to_base.json"
DEFAULT_ENGINE = SCRIPT_DIR / "weights" / "steel_ball_best_legacy.engine"
DEFAULT_PT = SCRIPT_DIR / "weights" / "steel_ball_v6_server_real_env.pt"
DEFAULT_BALL_RADIUS_M = 0.0084


def default_weights() -> Path:
    return DEFAULT_PT


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "使用 YOLO 识别钢珠，从对齐后的 RealSense 深度图读取深度，"
            "并将三维点从摄像头坐标系变换到底座坐标系。"
        )
    )
    parser.add_argument("--weights", type=Path, default=default_weights())
    parser.add_argument("--transform", type=Path, default=DEFAULT_TRANSFORM)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.45)
    parser.add_argument("--iou", type=float, default=0.35)
    parser.add_argument("--device", default="0")
    parser.add_argument(
        "--class-id",
        type=int,
        default=None,
        help="只使用指定类别；默认从全部类别中选置信度最高的检测框",
    )
    parser.add_argument(
        "--max-bbox-area-ratio",
        type=float,
        default=0.03,
        help="检测框面积占整幅画面的最大比例，默认 0.03",
    )
    parser.add_argument(
        "--max-bbox-width-ratio",
        type=float,
        default=0.25,
        help="检测框宽度占画面宽度的最大比例，默认 0.25",
    )
    parser.add_argument(
        "--max-bbox-height-ratio",
        type=float,
        default=0.25,
        help="检测框高度占画面高度的最大比例，默认 0.25",
    )
    parser.add_argument("--depth-roi-scale", type=float, default=0.45)
    parser.add_argument("--min-depth", type=float, default=0.08)
    parser.add_argument("--max-depth", type=float, default=3.0)
    parser.add_argument("--min-depth-samples", type=int, default=8)
    parser.add_argument(
        "--ball-radius-m",
        type=float,
        default=DEFAULT_BALL_RADIUS_M,
        help="钢珠半径，RealSense球面深度加此值后作为球心深度，默认0.0084m",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=15,
        help="启动后用于自动曝光稳定的帧数",
    )
    parser.add_argument(
        "--jsonl",
        type=Path,
        help="可选：把每帧结构化结果追加写入 JSON Lines 文件",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=1,
        help="每 N 帧向标准输出打印一次 JSON；0 表示不打印",
    )
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument(
        "--no-cpu-fallback",
        action="store_true",
        help="GPU或TensorRT加载失败时，不回退到项目默认PT权重的CPU推理",
    )
    parser.add_argument(
        "--stream-host",
        help="可选：接收视频的PC局域网IP；不指定则不推流",
    )
    parser.add_argument("--stream-port", type=int, default=5600)
    parser.add_argument("--stream-fps", type=int, default=30)
    parser.add_argument("--stream-bitrate", type=int, default=2_000_000)
    return parser.parse_args()


def point_dict(point: Sequence[float]) -> Dict[str, float]:
    return {
        "x": round(float(point[0]), 6),
        "y": round(float(point[1]), 6),
        "z": round(float(point[2]), 6),
    }


def intrinsics_dict(intrinsics: Any) -> Dict[str, Any]:
    """把 RealSense 内参对象转换为可打印、可记录的普通字典。"""
    return {
        "width": int(intrinsics.width),
        "height": int(intrinsics.height),
        "fx": float(intrinsics.fx),
        "fy": float(intrinsics.fy),
        "ppx": float(intrinsics.ppx),
        "ppy": float(intrinsics.ppy),
        "distortion_model": str(intrinsics.model),
        "coeffs": [float(value) for value in intrinsics.coeffs],
    }


def class_name(result: Any, class_id: int) -> str:
    names = result.names
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    if 0 <= class_id < len(names):
        return str(names[class_id])
    return str(class_id)


def select_detection(
    result: Any,
    required_class_id: Optional[int],
    image_width: int,
    image_height: int,
    max_area_ratio: float,
    max_width_ratio: float,
    max_height_ratio: float,
) -> Tuple[Optional[Tuple[np.ndarray, float, int]], int]:
    if result.boxes is None or len(result.boxes) == 0:
        return None, 0
    xyxy = result.boxes.xyxy.detach().cpu().numpy()
    confidences = result.boxes.conf.detach().cpu().numpy()
    classes = result.boxes.cls.detach().cpu().numpy().astype(int)

    candidate_indices = []
    filtered_count = 0
    for index, detected_class in enumerate(classes):
        if required_class_id is not None and detected_class != required_class_id:
            continue
        x1, y1, x2, y2 = [float(value) for value in xyxy[index]]
        bbox_width = max(0.0, x2 - x1)
        bbox_height = max(0.0, y2 - y1)
        width_ratio = bbox_width / image_width
        height_ratio = bbox_height / image_height
        area_ratio = bbox_width * bbox_height / (image_width * image_height)
        if (
            area_ratio > max_area_ratio
            or width_ratio > max_width_ratio
            or height_ratio > max_height_ratio
        ):
            filtered_count += 1
            continue
        candidate_indices.append(index)

    if not candidate_indices:
        return None, filtered_count
    best_index = max(candidate_indices, key=lambda index: float(confidences[index]))
    return (
        (
            xyxy[best_index].astype(np.float64),
            float(confidences[best_index]),
            int(classes[best_index]),
        ),
        filtered_count,
    )


def make_base_record(frame_index: int, timestamp_ms: float) -> Dict[str, Any]:
    return {
        "frame": frame_index,
        "timestamp_ms": round(timestamp_ms, 3),
        "valid": False,
    }


def locate_ball(
    result: Any,
    depth_raw: np.ndarray,
    depth_scale_m: float,
    color_intrinsics: Any,
    transform: RigidTransform,
    frame_index: int,
    timestamp_ms: float,
    required_class_id: Optional[int],
    depth_roi_scale: float,
    min_depth_m: float,
    max_depth_m: float,
    min_depth_samples: int,
    max_bbox_area_ratio: float,
    max_bbox_width_ratio: float,
    max_bbox_height_ratio: float,
    ball_radius_m: float = DEFAULT_BALL_RADIUS_M,
) -> Dict[str, Any]:
    """把YOLO和球面深度转换为钢珠球心的三维定位结果。"""
    record = make_base_record(frame_index, timestamp_ms)
    image_height, image_width = depth_raw.shape
    selected, filtered_count = select_detection(
        result=result,
        required_class_id=required_class_id,
        image_width=image_width,
        image_height=image_height,
        max_area_ratio=max_bbox_area_ratio,
        max_width_ratio=max_bbox_width_ratio,
        max_height_ratio=max_bbox_height_ratio,
    )
    if selected is None:
        if filtered_count > 0:
            record.update(
                {
                    "reason": "detection_filtered",
                    "filtered_detections": filtered_count,
                    "detail": (
                        "检测框超过尺寸阈值，已忽略且不在窗口绘制。"
                    ),
                }
            )
        else:
            record["reason"] = "ball_not_detected"
        return record

    bbox, confidence, detected_class = selected
    center_u = float((bbox[0] + bbox[2]) / 2.0)
    center_v = float((bbox[1] + bbox[3]) / 2.0)
    record.update(
        {
            "class_id": detected_class,
            "class_name": class_name(result, detected_class),
            "confidence": round(confidence, 6),
            "pixel": {"u": round(center_u, 3), "v": round(center_v, 3)},
            "bbox_xyxy": [round(float(value), 3) for value in bbox],
        }
    )

    try:
        depth = sample_depth(
            depth_raw=depth_raw,
            depth_scale_m=depth_scale_m,
            detection_bbox_xyxy=bbox,
            roi_scale=depth_roi_scale,
            min_depth_m=min_depth_m,
            max_depth_m=max_depth_m,
            min_valid_samples=min_depth_samples,
        )
    except DepthError as error:
        record["reason"] = "depth_invalid"
        record["detail"] = str(error)
        return record

    # RealSense返回检测区域可见球面的Z深度。沿相机Z方向增加钢珠半径，
    # 再用同一中心像素反投影，使后续相机/底座坐标都表示球心。
    surface_depth_m = depth.depth_m
    center_depth_m = surface_depth_m + ball_radius_m
    camera_point = np.asarray(
        rs.rs2_deproject_pixel_to_point(
            color_intrinsics,
            [center_u, center_v],
            center_depth_m,
        ),
        dtype=np.float64,
    )
    base_point = transform.transform_point(camera_point)
    record.update(
        {
            "valid": True,
            "surface_depth_m": round(surface_depth_m, 6),
            "ball_radius_m": round(ball_radius_m, 6),
            "depth_m": round(center_depth_m, 6),
            "depth_valid_samples": depth.valid_count,
            "depth_candidate_samples": depth.candidate_count,
            "depth_spread_m": round(depth.spread_m, 6),
            "depth_sample_bbox_xyxy": list(depth.sample_bbox_xyxy),
            "camera_point_m": point_dict(camera_point),
            "camera_base_point_m": point_dict(base_point),
        }
    )
    return record


def draw_overlay(
    frame: np.ndarray,
    record: Dict[str, Any],
    fps: float,
    transform: RigidTransform,
) -> np.ndarray:
    annotated = frame.copy()
    if "bbox_xyxy" in record:
        x1, y1, x2, y2 = [
            int(round(value)) for value in record["bbox_xyxy"]
        ]
        color = (0, 220, 0) if record["valid"] else (0, 0, 255)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        pixel = record["pixel"]
        center = (int(round(pixel["u"])), int(round(pixel["v"])))
        cv2.drawMarker(
            annotated, center, (0, 255, 255), cv2.MARKER_CROSS, 16, 2
        )
        if record["valid"]:
            base = record["camera_base_point_m"]
            label = "d={:.3f}m base=({:.3f},{:.3f},{:.3f})m".format(
                record["depth_m"], base["x"], base["y"], base["z"]
            )
        else:
            label = str(record.get("reason", "invalid"))
        cv2.putText(
            annotated,
            label,
            (max(5, x1), max(25, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            color,
            2,
            cv2.LINE_AA,
        )

    status = "FPS {:.1f} | {} -> {}".format(
        fps, transform.source_frame, transform.target_frame
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


def emit_record(record: Dict[str, Any], jsonl_file: Optional[Any]) -> None:
    line = json.dumps(record, ensure_ascii=False, separators=(",", ":"))
    print(line, flush=True)
    if jsonl_file is not None:
        jsonl_file.write(line + "\n")
        jsonl_file.flush()


def validate_args(args: argparse.Namespace) -> None:
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise ValueError("图像尺寸和帧率必须大于 0。")
    if not 0.0 < args.conf <= 1.0 or not 0.0 < args.iou <= 1.0:
        raise ValueError("conf 和 iou 必须位于 (0, 1]。")
    if args.print_every < 0 or args.max_frames < 0:
        raise ValueError("print-every 和 max-frames 不能为负数。")
    if args.ball_radius_m < 0:
        raise ValueError("ball-radius-m不能为负数。")
    for name, value in (
        ("max-bbox-area-ratio", args.max_bbox_area_ratio),
        ("max-bbox-width-ratio", args.max_bbox_width_ratio),
        ("max-bbox-height-ratio", args.max_bbox_height_ratio),
    ):
        if not 0.0 < value <= 1.0:
            raise ValueError("{} 必须位于 (0, 1]。".format(name))


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        weights = args.weights.expanduser().resolve()
        if not weights.is_file():
            raise ValueError("YOLO 权重不存在：{}".format(weights))
        transform = load_transform(args.transform)
    except (ValueError, TransformError) as error:
        print("配置错误：{}".format(error), file=sys.stderr)
        return 2

    print("YOLO 权重：{}".format(weights), file=sys.stderr)
    print(
        "坐标变换 {} -> {}：\n{}".format(
            transform.source_frame, transform.target_frame, transform.matrix4x4()
        ),
        file=sys.stderr,
    )

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.depth, args.width, args.height, rs.format.z16, args.fps
    )
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )

    jsonl_file = None
    pipeline_started = False
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
            jsonl_path = args.jsonl.expanduser().resolve()
            jsonl_path.parent.mkdir(parents=True, exist_ok=True)
            jsonl_file = jsonl_path.open("a", encoding="utf-8")

        profile = pipeline.start(config)
        pipeline_started = True
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        depth_scale_m = float(depth_sensor.get_depth_scale())
        align_to_color = rs.align(rs.stream.color)
        model = YOLO(str(weights), task="detect")
        is_engine = weights.suffix.lower() == ".engine"
        inference_device = args.device
        use_half = (
            not is_engine
            and inference_device != "cpu"
            and torch.cuda.is_available()
        )
        fallback_used = False

        print(
            "RealSense：{}，序列号 {}，depth_scale={} m/unit".format(
                device.get_info(rs.camera_info.name),
                device.get_info(rs.camera_info.serial_number),
                depth_scale_m,
            ),
            file=sys.stderr,
        )
        for _ in range(args.warmup_frames):
            pipeline.wait_for_frames(5000)

        frame_index = 0
        smoothed_fps = 0.0
        last_time = time.perf_counter()
        intrinsics_reported = False
        while args.max_frames == 0 or frame_index < args.max_frames:
            frames = align_to_color.process(pipeline.wait_for_frames(5000))
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            # 在任何图像转换和YOLO推理之前记录采集时刻。控制器使用该单调
            # 时钟估计速度/加速度，避免把推理耗时抖动误当成小球运动。
            capture_monotonic_ms = time.perf_counter() * 1000.0
            realsense_timestamp_ms = float(color_frame.get_timestamp())
            color = np.asanyarray(color_frame.get_data())
            depth_raw = np.asanyarray(depth_frame.get_data())
            # 直接读取 RealSense 设备随当前彩色流发布的出厂标定内参。
            # 等价信息在 ROS 中位于 /camera/color/camera_info；此处直连 SDK，
            # 无需手填 fx、fy、cx、cy，也避免流分辨率变化后继续使用旧内参。
            color_intrinsics = (
                color_frame.profile.as_video_stream_profile().get_intrinsics()
            )
            if not intrinsics_reported:
                print(
                    "彩色相机内参：{}".format(
                        json.dumps(
                            intrinsics_dict(color_intrinsics),
                            ensure_ascii=False,
                            separators=(",", ":"),
                        )
                    ),
                    file=sys.stderr,
                )
                intrinsics_reported = True
            try:
                result = model.predict(
                    source=color,
                    conf=args.conf,
                    iou=args.iou,
                    imgsz=args.imgsz,
                    device=inference_device,
                    half=use_half,
                    verbose=False,
                )[0]
            except (RuntimeError, TypeError) as inference_error:
                can_fallback = (
                    not args.no_cpu_fallback
                    and not fallback_used
                    and inference_device != "cpu"
                    and DEFAULT_PT.is_file()
                )
                if not can_fallback:
                    raise
                print(
                    "GPU/TensorRT推理启动失败：{}；改用 {} 在CPU上继续。"
                    .format(inference_error, DEFAULT_PT),
                    file=sys.stderr,
                )
                del model
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
                model = YOLO(str(DEFAULT_PT), task="detect")
                is_engine = False
                inference_device = "cpu"
                use_half = False
                fallback_used = True
                result = model.predict(
                    source=color,
                    conf=args.conf,
                    iou=args.iou,
                    imgsz=args.imgsz,
                    device=inference_device,
                    half=False,
                    verbose=False,
                )[0]

            frame_index += 1
            now = time.perf_counter()
            instant_fps = 1.0 / max(now - last_time, 1e-6)
            smoothed_fps = (
                instant_fps
                if smoothed_fps == 0.0
                else 0.90 * smoothed_fps + 0.10 * instant_fps
            )
            last_time = now
            record = locate_ball(
                result=result,
                depth_raw=depth_raw,
                depth_scale_m=depth_scale_m,
                color_intrinsics=color_intrinsics,
                transform=transform,
                frame_index=frame_index,
                timestamp_ms=time.time() * 1000.0,
                required_class_id=args.class_id,
                depth_roi_scale=args.depth_roi_scale,
                min_depth_m=args.min_depth,
                max_depth_m=args.max_depth,
                min_depth_samples=args.min_depth_samples,
                max_bbox_area_ratio=args.max_bbox_area_ratio,
                max_bbox_width_ratio=args.max_bbox_width_ratio,
                max_bbox_height_ratio=args.max_bbox_height_ratio,
                ball_radius_m=args.ball_radius_m,
            )
            record["capture_monotonic_ms"] = round(
                capture_monotonic_ms, 3
            )
            record["realsense_timestamp_ms"] = round(
                realsense_timestamp_ms, 3
            )
            record["processing_latency_ms"] = round(
                time.perf_counter() * 1000.0 - capture_monotonic_ms, 3
            )
            # 常驻比赛服务用此字段区分“视觉推理正常”和“视频编码仍在线”。
            # 推流异常会在本帧发送后被关闭，下一帧即可报告false。
            record["video_stream_active"] = streamer is not None
            record["video_stream_encoder"] = (
                None if streamer is None else streamer.active_encoder
            )

            if args.print_every > 0 and frame_index % args.print_every == 0:
                emit_record(record, jsonl_file)
            elif jsonl_file is not None:
                line = json.dumps(
                    record, ensure_ascii=False, separators=(",", ":")
                )
                jsonl_file.write(line + "\n")
                jsonl_file.flush()

            if streamer is not None:
                # 局域网只发送纯摄像头画面，不叠加识别框、坐标或深度信息。
                streamer.send(color)
                if streamer.error is not None:
                    print(
                        "推流错误：{}".format(streamer.error),
                        file=sys.stderr,
                    )
                    streamer.close()
                    streamer = None
            if not args.no_display:
                annotated = draw_overlay(
                    color, record, smoothed_fps, transform
                )
                cv2.imshow("H - Steel Ball RGB-D Tracker", annotated)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    except (RuntimeError, TypeError, StreamError) as error:
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
