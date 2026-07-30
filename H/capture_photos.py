#!/usr/bin/env python3
"""使用RealSense采集用于标注的原始彩色照片。"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


PROJECT_ROOT = Path("/home/pangolin/NUEDC")
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "H" / "captured_photos"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RealSense手动拍照工具",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="照片保存目录",
    )
    parser.add_argument("--width", type=int, default=640, help="照片宽度")
    parser.add_argument("--height", type=int, default=480, help="照片高度")
    parser.add_argument("--fps", type=int, default=30, help="相机采集帧率")
    parser.add_argument(
        "--display-every",
        type=int,
        default=15,
        help="每N个相机帧刷新一次预览窗口",
    )
    parser.add_argument(
        "--preview-scale",
        type=float,
        default=0.5,
        help="预览窗口相对原图的缩放比例，不影响照片尺寸",
    )
    parser.add_argument(
        "--opencv-threads",
        type=int,
        default=1,
        help="OpenCV最多使用的CPU线程数",
    )
    parser.add_argument(
        "--prefix",
        default="ppr",
        help="照片文件名前缀",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help="JPEG质量，范围1～100",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="采集帧数上限；0表示持续运行",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise ValueError("宽度、高度和帧率必须大于0。")
    if args.display_every < 1 or args.opencv_threads < 1:
        raise ValueError("display-every和opencv-threads至少为1。")
    if not 0.25 <= args.preview_scale <= 1.0:
        raise ValueError("preview-scale必须位于0.25～1.0。")
    if args.max_frames < 0:
        raise ValueError("max-frames不能为负数。")
    if not 1 <= args.jpeg_quality <= 100:
        raise ValueError("jpeg-quality必须位于1～100。")
    if not args.prefix or any(
        character in args.prefix for character in ("/", "\\", "\0")
    ):
        raise ValueError("prefix不能为空，也不能包含路径分隔符。")


def photo_path(output_dir: Path, prefix: str, sequence: int) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    return output_dir / "{}_{}_{:06d}.jpg".format(
        prefix, timestamp, sequence
    )


def draw_preview(
    frame: np.ndarray,
    saved_count: int,
    fps: float,
    last_filename: str,
) -> np.ndarray:
    preview = frame.copy()
    lines = [
        "SPACE/S: save | Q/ESC: quit",
        "Saved: {} | Preview FPS: {:.1f}".format(saved_count, fps),
    ]
    if last_filename:
        lines.append("Last: {}".format(last_filename))
    for index, text in enumerate(lines):
        y = 28 + index * 27
        cv2.putText(
            preview,
            text,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            preview,
            text,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return preview


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        output_dir = args.output_dir.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        cv2.setNumThreads(args.opencv_threads)
    except (OSError, ValueError) as error:
        print("配置错误：{}".format(error), file=sys.stderr)
        return 2

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.color,
        args.width,
        args.height,
        rs.format.bgr8,
        args.fps,
    )

    pipeline_started = False
    saved_count = 0
    try:
        profile = pipeline.start(config)
        pipeline_started = True
        device = profile.get_device()
        print(
            "RealSense：{}，序列号 {}，彩色流 {}x{}@{}".format(
                device.get_info(rs.camera_info.name),
                device.get_info(rs.camera_info.serial_number),
                args.width,
                args.height,
                args.fps,
            ),
            file=sys.stderr,
        )
        print("照片目录：{}".format(output_dir), file=sys.stderr)
        print("按空格或S拍照，按Q或Esc退出。", file=sys.stderr)

        for _ in range(15):
            pipeline.wait_for_frames(5000)

        sequence = len(list(output_dir.glob("{}_*.jpg".format(args.prefix))))
        last_filename = ""
        smoothed_fps = 0.0
        last_frame_time = time.perf_counter()
        frame_index = 0

        while args.max_frames == 0 or frame_index < args.max_frames:
            frames = pipeline.wait_for_frames(5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
            frame_index += 1

            now = time.perf_counter()
            instant_fps = 1.0 / max(now - last_frame_time, 1e-6)
            smoothed_fps = (
                instant_fps
                if smoothed_fps == 0.0
                else 0.9 * smoothed_fps + 0.1 * instant_fps
            )
            last_frame_time = now

            key = 255
            if frame_index % args.display_every == 0:
                if args.preview_scale < 1.0:
                    preview_source = cv2.resize(
                        frame,
                        None,
                        fx=args.preview_scale,
                        fy=args.preview_scale,
                        interpolation=cv2.INTER_AREA,
                    )
                else:
                    preview_source = frame
                preview = draw_preview(
                    preview_source,
                    saved_count,
                    smoothed_fps,
                    last_filename,
                )
                cv2.imshow("H - Photo Capture", preview)
                # pollKey只处理GUI事件、不等待，避免X11拖慢相机取帧。
                key = cv2.pollKey() & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("s"), ord("S"), 32):
                sequence += 1
                path = photo_path(output_dir, args.prefix, sequence)
                success = cv2.imwrite(
                    str(path),
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
                )
                if not success:
                    print("保存失败：{}".format(path), file=sys.stderr)
                    continue
                saved_count += 1
                last_filename = path.name
                print("已保存：{}".format(path), flush=True)
    except RuntimeError as error:
        print("运行错误：{}".format(error), file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        pass
    finally:
        if pipeline_started:
            pipeline.stop()
        cv2.destroyAllWindows()

    print("拍照结束，共保存 {} 张。".format(saved_count), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
