#!/usr/bin/env python3
"""通过GStreamer发送低延迟H.264/RTP视频，硬编失败时自动软编。"""

import ipaddress
import queue
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np


class StreamError(RuntimeError):
    """推流配置或GStreamer启动失败。"""


@dataclass(frozen=True)
class StreamConfig:
    host: str
    port: int = 5600
    width: int = 640
    height: int = 480
    fps: int = 30
    bitrate: int = 2_000_000
    encoder: str = "auto"


def validate_stream_config(config: StreamConfig) -> None:
    try:
        ipaddress.ip_address(config.host)
    except ValueError as error:
        raise StreamError(
            "stream-host 必须是PC的IPv4或IPv6地址：{}".format(error)
        )
    if not 1 <= config.port <= 65535:
        raise StreamError("stream-port 必须位于 1～65535。")
    if config.width <= 0 or config.height <= 0 or config.fps <= 0:
        raise StreamError("推流尺寸和帧率必须大于0。")
    if config.bitrate < 100_000:
        raise StreamError("stream-bitrate 不能低于100000 bit/s。")
    if config.encoder not in ("auto", "hardware", "software"):
        raise StreamError("encoder 必须是 auto、hardware 或 software。")
    if shutil.which("gst-launch-1.0") is None:
        raise StreamError("系统未安装 gst-launch-1.0。")


def gstreamer_command(config: StreamConfig, encoder: str = "hardware") -> list:
    frame_size = config.width * config.height * 3
    command = [
        "gst-launch-1.0",
        "-q",
        "fdsrc",
        "fd=0",
        "blocksize={}".format(frame_size),
        "!",
        "videoparse",
        "format=bgr",
        "width={}".format(config.width),
        "height={}".format(config.height),
        "framerate={}/1".format(config.fps),
        "!",
        "videoconvert",
        "!",
    ]
    if encoder == "hardware":
        command.extend([
            "nvvidconv", "!", "video/x-raw(memory:NVMM),format=NV12", "!",
            "nvv4l2h264enc",
            "bitrate={}".format(config.bitrate),
            "control-rate=1", "insert-sps-pps=true",
            "iframeinterval={}".format(config.fps), "preset-level=1", "!",
        ])
    elif encoder == "software":
        if shutil.which("gst-inspect-1.0") is None:
            raise StreamError("系统未安装 gst-inspect-1.0。")
        command.extend([
            "video/x-raw,format=I420", "!", "x264enc",
            "bitrate={}".format(max(100, config.bitrate // 1000)),
            "tune=zerolatency", "speed-preset=ultrafast",
            "key-int-max={}".format(config.fps), "byte-stream=true", "!",
        ])
    else:
        raise StreamError("未知H.264编码器：{}".format(encoder))
    command.extend([
        "h264parse", "config-interval=1", "!",
        "rtph264pay", "pt=96", "config-interval=1", "mtu=1200", "!",
        "udpsink", "host={}".format(config.host),
        "port={}".format(config.port), "sync=false", "async=false",
    ])
    return command


class UdpH264Streamer:
    """后台发送最新帧；队列满时丢弃旧帧，永不阻塞视觉主循环。"""

    def __init__(self, config: StreamConfig) -> None:
        validate_stream_config(config)
        self.config = config
        self._frames: "queue.Queue[Optional[np.ndarray]]" = queue.Queue(
            maxsize=1
        )
        self._process: Optional[subprocess.Popen] = None
        self._error: Optional[str] = None
        self._stopping = False
        self._submitted_frames = 0
        self._dropped_frames = 0
        self._written_frames = 0
        self._repeated_frames = 0
        self._active_encoder: Optional[str] = None
        self._fallback_reason: Optional[str] = None
        self._thread = threading.Thread(
            target=self._worker,
            name="h264-udp-streamer",
            daemon=True,
        )
        self._thread.start()

    @property
    def error(self) -> Optional[str]:
        return self._error

    @property
    def submitted_frames(self) -> int:
        return self._submitted_frames

    @property
    def dropped_frames(self) -> int:
        return self._dropped_frames

    @property
    def written_frames(self) -> int:
        return self._written_frames

    @property
    def repeated_frames(self) -> int:
        return self._repeated_frames

    @property
    def active_encoder(self) -> Optional[str]:
        return self._active_encoder

    @property
    def fallback_reason(self) -> Optional[str]:
        return self._fallback_reason

    def send(self, frame_bgr: np.ndarray) -> bool:
        if self._stopping or self._error is not None:
            return False
        if (
            not isinstance(frame_bgr, np.ndarray)
            or frame_bgr.shape
            != (self.config.height, self.config.width, 3)
            or frame_bgr.dtype != np.uint8
        ):
            raise StreamError(
                "推流帧必须是 {}x{} BGR uint8。".format(
                    self.config.width, self.config.height
                )
            )
        frame = np.ascontiguousarray(frame_bgr).copy()
        self._submitted_frames += 1
        try:
            self._frames.put_nowait(frame)
        except queue.Full:
            self._dropped_frames += 1
            try:
                self._frames.get_nowait()
            except queue.Empty:
                pass
            try:
                self._frames.put_nowait(frame)
            except queue.Full:
                return False
        return True

    def _worker(self) -> None:
        encoders = (
            ["hardware", "software"]
            if self.config.encoder == "auto"
            else [self.config.encoder]
        )
        errors = []
        for encoder_index, encoder in enumerate(encoders):
            if self._stopping:
                break
            try:
                self._run_encoder(encoder)
                return
            except (BrokenPipeError, OSError, StreamError) as error:
                errors.append("{}: {}".format(encoder, error))
                self._cleanup_process()
                if encoder_index + 1 < len(encoders) and not self._stopping:
                    self._fallback_reason = str(error)
                    print(
                        "H.264硬件编码失败：{}；自动切换x264软件编码，视觉和控制不中断。"
                        .format(error),
                        file=sys.stderr,
                    )
                    continue
                if not self._stopping:
                    self._error = "；".join(errors)
                return
        self._cleanup_process()

    def _run_encoder(self, encoder: str) -> None:
        self._active_encoder = encoder
        try:
            self._process = subprocess.Popen(
                gstreamer_command(self.config, encoder),
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
            )
            frame_interval = 1.0 / self.config.fps
            next_write_time = None
            latest_frame = None
            latest_version = 0
            written_version = -1
            while True:
                if latest_frame is None:
                    frame = self._frames.get()
                else:
                    timeout = max(
                        0.0, next_write_time - time.perf_counter()
                    )
                    try:
                        frame = self._frames.get(timeout=timeout)
                    except queue.Empty:
                        frame = ...

                if frame is None:
                    break
                if frame is not ...:
                    latest_frame = frame
                    latest_version += 1
                    if next_write_time is None:
                        next_write_time = time.perf_counter()

                now = time.perf_counter()
                if latest_frame is None or now < next_write_time:
                    continue
                if self._process.poll() is not None:
                    raise StreamError(
                        "GStreamer推流进程提前退出，返回码 {}。".format(
                            self._process.returncode
                        )
                    )
                if self._process.stdin is None:
                    raise StreamError("GStreamer标准输入不可用。")
                self._process.stdin.write(latest_frame.tobytes())
                self._process.stdin.flush()
                self._written_frames += 1
                if latest_version == written_version:
                    self._repeated_frames += 1
                written_version = latest_version
                next_write_time += frame_interval
                # 编码器启动或系统调度落后时直接恢复当前节拍，不突发补写。
                if next_write_time <= time.perf_counter():
                    next_write_time = time.perf_counter() + frame_interval
        finally:
            self._cleanup_process()

    def _cleanup_process(self) -> None:
        process = self._process
        self._process = None
        if process is None:
            return
        if process.stdin is not None:
            try:
                process.stdin.close()
            except OSError:
                pass
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()

    def close(self) -> None:
        if self._stopping:
            return
        self._stopping = True
        try:
            self._frames.put_nowait(None)
        except queue.Full:
            try:
                self._frames.get_nowait()
            except queue.Empty:
                pass
            try:
                self._frames.put_nowait(None)
            except queue.Full:
                pass
        self._thread.join(timeout=4.0)
