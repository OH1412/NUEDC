#!/usr/bin/env python3
"""Real-time YOLOv8 steel-ball detection for a USB/V4L2 camera."""

import argparse
import sys
import time
from pathlib import Path
from typing import Union

import cv2
import numpy as np
import psutil
import torch
from ultralytics import YOLO


DEFAULT_WEIGHTS = Path("/home/pangolin/Downloads/best.pt")
DEFAULT_OUTPUT = Path("/home/pangolin/NUEDC/yolo_camera_captures")


def parse_source(value: str) -> Union[int, str]:
    if value == "auto":
        paths = sorted(Path("/dev").glob("video*"))

        def camera_priority(path: Path):
            # A D435 exposes depth/infrared and RGB as separate V4L2 nodes.
            # Its RGB Camera interface is USB interface 03, so try that first.
            interface_file = (
                Path("/sys/class/video4linux") / path.name / "device/bInterfaceNumber"
            )
            try:
                is_realsense_rgb = interface_file.read_text().strip() == "03"
            except OSError:
                is_realsense_rgb = False
            suffix = path.name[5:]
            index = int(suffix) if suffix.isdigit() else 9999
            return (not is_realsense_rgb, index)

        for path in sorted(paths, key=camera_priority):
            suffix = path.name[5:]
            if not suffix.isdigit():
                continue
            index = int(suffix)
            probe = cv2.VideoCapture(index, cv2.CAP_V4L2)
            ok = probe.isOpened() and probe.read()[0]
            probe.release()
            if ok:
                print(f"Auto-selected camera: {path}")
                return index
        raise RuntimeError("No readable V4L2 camera was found under /dev/video*.")
    return int(value) if value.isdigit() else value


class RealSenseColorCapture:
    """Small VideoCapture-compatible wrapper for the D435 RGB stream."""

    def __init__(self, width: int, height: int, fps: int):
        import pyrealsense2 as rs

        self.rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(
            rs.stream.color, width, height, rs.format.bgr8, fps
        )
        self.profile = self.pipeline.start(config)
        self.width = width
        self.height = height
        self.fps = fps
        self.opened = True
        device = self.profile.get_device()
        self.device_name = device.get_info(rs.camera_info.name)
        self.serial = device.get_info(rs.camera_info.serial_number)

        # Let automatic exposure and white balance settle before inference.
        for _ in range(15):
            self.pipeline.wait_for_frames(5000)

    def isOpened(self) -> bool:
        return self.opened

    def read(self):
        try:
            frames = self.pipeline.wait_for_frames(5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return False, None
            return True, np.asanyarray(color_frame.get_data())
        except RuntimeError:
            return False, None

    def release(self) -> None:
        if self.opened:
            self.pipeline.stop()
            self.opened = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOv8 steel-ball camera inference")
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument(
        "--source",
        default="auto",
        help="'auto', a camera index, /dev/videoN, or a video path",
    )
    parser.add_argument("--conf", type=float, default=0.45)
    parser.add_argument("--iou", type=float, default=0.35)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="auto", help="auto, cpu, 0, cuda:0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--no-display", action="store_true", help="run without a GUI window")
    parser.add_argument("--max-frames", type=int, default=0, help="0 means unlimited")
    parser.add_argument("--save-every", type=int, default=0, help="save every Nth annotated frame")
    parser.add_argument(
        "--min-cuda-memory-gb",
        type=float,
        default=2.0,
        help="minimum available unified memory required by auto CUDA selection",
    )
    parser.add_argument(
        "--no-cpu-fallback",
        action="store_true",
        help="do not fall back to CPU after a CUDA out-of-memory error",
    )
    return parser.parse_args()


def choose_device(
    requested: str, min_cuda_memory_gb: float, allow_cpu_fallback: bool
) -> str:
    wants_cuda = requested != "cpu" and (
        requested == "auto" or requested == "0" or requested.startswith("cuda")
    )
    if not wants_cuda:
        return requested
    if not torch.cuda.is_available():
        if requested != "auto":
            print("CUDA was requested but is unavailable; using CPU.", file=sys.stderr)
        return "cpu"

    available_gb = psutil.virtual_memory().available / (1024**3)
    if available_gb < min_cuda_memory_gb:
        message = (
            f"Only {available_gb:.2f} GiB unified memory is available; "
            f"CUDA needs at least {min_cuda_memory_gb:.2f} GiB for this script."
        )
        if allow_cpu_fallback:
            print(
                f"{message} Using CPU to avoid Jetson NvMap allocation failures. "
                "Pass --no-cpu-fallback only after freeing memory.",
                file=sys.stderr,
            )
            return "cpu"
        print(f"Warning: {message} Trying CUDA because fallback is disabled.", file=sys.stderr)

    return "0" if requested == "auto" else requested


def is_cuda_runtime_failure(exc: RuntimeError, device: str) -> bool:
    if device == "cpu":
        return False
    message = str(exc).lower()
    return any(
        marker in message
        for marker in (
            "out of memory",
            "unable to find an engine",
            "cudnn_status_alloc_failed",
            "cuda error",
            "nvmap",
        )
    )


def open_capture(source, width: int, height: int, fps: int) -> cv2.VideoCapture:
    if source == "realsense":
        return RealSenseColorCapture(width, height, fps)

    backend = cv2.CAP_V4L2 if isinstance(source, int) else cv2.CAP_ANY
    capture = cv2.VideoCapture(source, backend)
    if not capture.isOpened() and backend == cv2.CAP_V4L2:
        capture.release()
        capture = cv2.VideoCapture(source)

    if isinstance(source, int):
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def capture_description(capture: cv2.VideoCapture) -> str:
    if isinstance(capture, RealSenseColorCapture):
        return (
            f"{capture.device_name} serial {capture.serial}, "
            f"RGB/BGR8 {capture.width}x{capture.height}@{capture.fps}"
        )

    fourcc_value = int(capture.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((fourcc_value >> (8 * i)) & 0xFF) for i in range(4))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    reported_fps = capture.get(cv2.CAP_PROP_FPS)
    return f"{fourcc} {width}x{height}, driver-reported {reported_fps:.1f} FPS"


def save_frame(frame, output_dir: Path, frame_index: int) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = output_dir / f"steel_ball_{stamp}_{frame_index:06d}.jpg"
    if not cv2.imwrite(str(path), frame):
        raise RuntimeError(f"Failed to save frame: {path}")
    return path


def main() -> int:
    args = parse_args()
    try:
        source = parse_source(args.source)
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 3
    if isinstance(source, int) and not Path(f"/dev/video{source}").exists():
        print(
            f"Error: /dev/video{source} does not exist. The camera is not "
            "currently enumerated by Linux; reconnect its USB 3.x cable and "
            "run with --source auto.",
            file=sys.stderr,
        )
        return 3
    weights = args.weights.expanduser().resolve()
    if not weights.is_file():
        print(f"Error: weights not found: {weights}", file=sys.stderr)
        return 2
    is_tensorrt = weights.suffix.lower() == ".engine"

    device = choose_device(
        args.device,
        args.min_cuda_memory_gb,
        allow_cpu_fallback=not args.no_cpu_fallback and not is_tensorrt,
    )
    if is_tensorrt and device == "cpu":
        print(
            "Error: a TensorRT .engine model requires the Jetson GPU; "
            "use the original best.pt file for CPU fallback.",
            file=sys.stderr,
        )
        return 2
    use_half = device != "cpu" and torch.cuda.is_available()
    print(f"Weights: {weights}")
    print(f"Device: {device} (FP16={use_half})")
    print(f"Thresholds: conf={args.conf}, iou={args.iou}, imgsz={args.imgsz}")

    model = YOLO(str(weights), task="detect")
    try:
        capture = open_capture(source, args.width, args.height, args.fps)
    except RuntimeError as exc:
        print(
            f"Error: cannot start camera source {source}: {exc}",
            file=sys.stderr,
        )
        return 3
    if not capture.isOpened():
        print(f"Error: cannot open camera/video source: {source}", file=sys.stderr)
        return 3
    print(f"Camera: {capture_description(capture)}")

    frame_index = 0
    consecutive_read_failures = 0
    smoothed_fps = 0.0
    last_time = time.perf_counter()
    benchmark_start = None
    benchmark_end = None
    fallback_used = False

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                consecutive_read_failures += 1
                if consecutive_read_failures >= 30:
                    print(
                        "Error: camera failed to deliver 30 consecutive frames. "
                        "Check the USB connection and rerun with --source auto.",
                        file=sys.stderr,
                    )
                    return 4
                print("Warning: camera frame read failed.", file=sys.stderr)
                time.sleep(0.05)
                continue
            consecutive_read_failures = 0

            try:
                result = model.predict(
                    source=frame,
                    conf=args.conf,
                    iou=args.iou,
                    imgsz=args.imgsz,
                    device=device,
                    half=use_half,
                    verbose=False,
                )[0]
            except RuntimeError as exc:
                if (
                    not is_cuda_runtime_failure(exc, device)
                    or args.no_cpu_fallback
                    or is_tensorrt
                    or fallback_used
                ):
                    raise
                print(
                    f"CUDA inference failed ({exc}); retrying on CPU.",
                    file=sys.stderr,
                )
                model.to("cpu")
                model.predictor = None
                torch.cuda.empty_cache()
                device, use_half, fallback_used = "cpu", False, True
                continue

            frame_index += 1
            now = time.perf_counter()
            if frame_index == 1:
                # Exclude model initialization and the first inference from the
                # end-to-end average reported when the program exits.
                benchmark_start = now
            benchmark_end = now
            instant_fps = 1.0 / max(now - last_time, 1e-6)
            smoothed_fps = instant_fps if smoothed_fps == 0 else 0.9 * smoothed_fps + 0.1 * instant_fps
            last_time = now
            detections = 0 if result.boxes is None else len(result.boxes)
            status = f"FPS {smoothed_fps:.1f} | detections {detections} | device {device}"
            should_save = args.save_every > 0 and frame_index % args.save_every == 0
            annotated = None
            if not args.no_display or should_save:
                annotated = result.plot()
                cv2.putText(
                    annotated,
                    status,
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            if should_save:
                print(f"Saved: {save_frame(annotated, args.output_dir, frame_index)}")

            if not args.no_display:
                cv2.imshow("YOLOv8 Steel Ball Detection", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    print(f"Saved: {save_frame(annotated, args.output_dir, frame_index)}")

            if args.max_frames > 0 and frame_index >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        capture.release()
        cv2.destroyAllWindows()

    average_fps = 0.0
    if frame_index > 1 and benchmark_start is not None and benchmark_end is not None:
        average_fps = (frame_index - 1) / max(benchmark_end - benchmark_start, 1e-6)
    print(
        f"Stopped after {frame_index} frames. "
        f"End-to-end average FPS: {average_fps:.1f}; final smoothed FPS: {smoothed_fps:.1f}."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
