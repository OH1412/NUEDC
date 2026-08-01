#!/usr/bin/env python3
"""RealSense纯彩色视频推流，不加载YOLO、深度处理或识别算法。"""

import argparse
import sys
import time

import numpy as np
import pyrealsense2 as rs

from udp_video_stream import StreamConfig, StreamError, UdpH264Streamer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="将RealSense纯彩色画面通过H.264/RTP/UDP推送到PC",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--stream-host",
        default="192.168.50.199",
        help="接收视频的PC局域网IP",
    )
    parser.add_argument(
        "--stream-port",
        type=int,
        default=5600,
        help="PC接收UDP端口",
    )
    parser.add_argument("--width", type=int, default=640, help="画面宽度")
    parser.add_argument("--height", type=int, default=480, help="画面高度")
    parser.add_argument(
        "--fps",
        type=int,
        default=60,
        help="RealSense采集帧率",
    )
    parser.add_argument(
        "--stream-fps",
        type=int,
        default=30,
        help="发送和H.264时间戳帧率",
    )
    parser.add_argument(
        "--stream-bitrate",
        type=int,
        default=3_000_000,
        help="H.264码率bit/s",
    )
    parser.add_argument(
        "--status-every",
        type=int,
        default=300,
        help="每多少帧输出一次实际采集FPS；0表示不输出",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="处理帧数上限；0表示持续运行",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if (
        args.width <= 0
        or args.height <= 0
        or args.fps <= 0
        or args.stream_fps <= 0
    ):
        raise ValueError("采集宽度、高度和帧率必须大于0。")
    if args.stream_fps > args.fps:
        raise ValueError("stream-fps不能高于相机fps。")
    if args.status_every < 0 or args.max_frames < 0:
        raise ValueError("status-every和max-frames不能为负数。")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        stream_config = StreamConfig(
            host=args.stream_host,
            port=args.stream_port,
            width=args.width,
            height=args.height,
            fps=args.stream_fps,
            bitrate=args.stream_bitrate,
        )
    except (ValueError, StreamError) as error:
        print("配置错误：{}".format(error), file=sys.stderr)
        return 2

    pipeline = rs.pipeline()
    realsense_config = rs.config()
    realsense_config.enable_stream(
        rs.stream.color,
        args.width,
        args.height,
        rs.format.bgr8,
        args.fps,
    )

    pipeline_started = False
    streamer = None
    try:
        profile = pipeline.start(realsense_config)
        pipeline_started = True
        device = profile.get_device()
        print(
            "RealSense纯视频：{}，序列号 {}，{}x{}@{}".format(
                device.get_info(rs.camera_info.name),
                device.get_info(rs.camera_info.serial_number),
                args.width,
                args.height,
                args.fps,
            ),
            file=sys.stderr,
        )

        # 丢弃自动曝光刚启动时的过渡帧。
        for _ in range(10):
            pipeline.wait_for_frames(5000)

        streamer = UdpH264Streamer(stream_config)
        print(
            "纯视频推流：rtp://{}:{}，采集 {} FPS，发送 {} FPS，{} bit/s"
            .format(
                args.stream_host,
                args.stream_port,
                args.fps,
                args.stream_fps,
                args.stream_bitrate,
            ),
            file=sys.stderr,
        )
        print(
            "未加载YOLO、TensorRT、Torch或深度流；按 Ctrl+C 停止。",
            file=sys.stderr,
        )

        frame_index = 0
        sent_frames = 0
        status_start = time.perf_counter()
        status_sent_start = 0
        status_written_start = 0
        status_repeated_start = 0
        send_interval = 1.0 / args.stream_fps
        next_send_time = status_start
        while args.max_frames == 0 or frame_index < args.max_frames:
            frames = pipeline.wait_for_frames(5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            frame_index += 1
            now = time.perf_counter()
            if now >= next_send_time:
                streamer.send(frame)
                sent_frames += 1
                # 以单调时钟维持稳定RTP节拍；落后时跳过旧时刻，不突发补帧。
                next_send_time += send_interval
                if next_send_time <= now:
                    skipped_intervals = int(
                        (now - next_send_time) / send_interval
                    ) + 1
                    next_send_time += skipped_intervals * send_interval
            if streamer.error is not None:
                raise StreamError(streamer.error)

            if (
                args.status_every > 0
                and frame_index % args.status_every == 0
            ):
                fps = args.status_every / max(now - status_start, 1e-6)
                send_fps = (sent_frames - status_sent_start) / max(
                    now - status_start, 1e-6
                )
                output_fps = (
                    streamer.written_frames - status_written_start
                ) / max(now - status_start, 1e-6)
                repeated = (
                    streamer.repeated_frames - status_repeated_start
                )
                status_start = now
                status_sent_start = sent_frames
                status_written_start = streamer.written_frames
                status_repeated_start = streamer.repeated_frames
                print(
                    (
                        "已采集 {} 帧，采集 {:.1f} FPS，最新帧刷新 "
                        "{:.1f} FPS，编码输出 {:.1f} FPS，本段重复 {} 帧，"
                        "队列替换 {} 帧"
                    ).format(
                        frame_index,
                        fps,
                        send_fps,
                        output_fps,
                        repeated,
                        streamer.dropped_frames,
                    ),
                    file=sys.stderr,
                )
    except (RuntimeError, StreamError) as error:
        print("运行错误：{}".format(error), file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        pass
    finally:
        if streamer is not None:
            streamer.close()
        if pipeline_started:
            pipeline.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
