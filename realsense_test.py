#!/usr/bin/env python3
"""RealSense D435 color/depth smoke test with live FPS and center distance."""

import argparse
import time

import cv2
import numpy as np
import pyrealsense2 as rs


def parse_args():
    parser = argparse.ArgumentParser(description="Test an Intel RealSense D435")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--frames", type=int, default=0, help="0 means unlimited")
    parser.add_argument("--no-display", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.depth, args.width, args.height, rs.format.z16, args.fps
    )
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )

    profile = pipeline.start(config)
    device = profile.get_device()
    print(f"Device: {device.get_info(rs.camera_info.name)}")
    print(f"Serial: {device.get_info(rs.camera_info.serial_number)}")
    print(f"Firmware: {device.get_info(rs.camera_info.firmware_version)}")
    print(f"Streams: color+depth {args.width}x{args.height}@{args.fps}")

    align = rs.align(rs.stream.color)
    colorizer = rs.colorizer()
    count = 0
    first_time = None
    last_time = time.perf_counter()
    smoothed_fps = 0.0

    try:
        while args.frames == 0 or count < args.frames:
            frames = align.process(pipeline.wait_for_frames(5000))
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            count += 1
            now = time.perf_counter()
            if first_time is None:
                first_time = now
            instant_fps = 1.0 / max(now - last_time, 1e-6)
            smoothed_fps = (
                instant_fps
                if smoothed_fps == 0
                else 0.9 * smoothed_fps + 0.1 * instant_fps
            )
            last_time = now

            center_x, center_y = args.width // 2, args.height // 2
            distance_m = depth_frame.get_distance(center_x, center_y)

            if args.no_display:
                continue

            depth_color = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            cv2.circle(color, (center_x, center_y), 5, (0, 255, 255), 2)
            cv2.putText(
                color,
                f"FPS {smoothed_fps:.1f} | center {distance_m:.3f} m",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            view = np.hstack((color, depth_color))
            cv2.imshow("RealSense D435 Color + Depth", view)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    average_fps = 0.0
    if count > 1 and first_time is not None:
        average_fps = (count - 1) / max(last_time - first_time, 1e-6)
    print(
        f"Stopped after {count} frames. "
        f"Average FPS: {average_fps:.1f}; smoothed FPS: {smoothed_fps:.1f}."
    )


if __name__ == "__main__":
    main()
