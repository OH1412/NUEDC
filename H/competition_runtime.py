#!/usr/bin/env python3
"""H题比赛常驻服务：一次加载视觉，按UART2请求切换控制任务。"""

import argparse
from collections import deque
import errno
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time
from typing import Any, Dict, List, Optional

import numpy as np

from angle_serial import (
    MOTOR_INITIAL_ZERO_FRAME,
    encode_angle,
    open_serial_port,
)
from ball_control import ball_position_from_zero
from ball_tracker_source import base_point_from_record
from calibrate_ball_zero import robust_zero_point
from mode5_equilibrium import (
    DEFAULT_MODE5_EQUILIBRIUM_FILE,
    nearest_mode5_equilibrium,
)


H_DIR = Path(__file__).resolve().parent
TRACKER = H_DIR / "ball_depth_tracker.py"
CONTROLLER = H_DIR / "ball_control_runtime.py"
CONFIG = H_DIR / "ball_control_config.json"

REQUEST_HEADER = 0x76
REQUEST_TAIL = 0x67
RESPONSE_HEADER = 0x92
RESPONSE_TAIL = 0x29
FRAME_LENGTH = 8

WEIGHT_QUERY = bytes((0x76, 0x57, 0x4C, 0, 0, 0, 0, 0x67))
MODE2_BEGIN = bytes((0x76, 0x01, 0, 0, 0, 0, 0, 0x67))
MODE2_END = bytes((0x76, 0x01, 0, 0x01, 0, 0, 0, 0x67))
MODE3_BEGIN = bytes((0x76, 0x02, 0x03, 0, 0, 0, 0, 0x67))
MODE3_END = bytes((0x76, 0x02, 0x03, 0x01, 0, 0, 0, 0x67))
MODE4_BEGIN = bytes((0x76, 0x02, 0x04, 0, 0, 0, 0, 0x67))
MODE4_END = bytes((0x76, 0x02, 0x04, 0x01, 0, 0, 0, 0x67))
BALL_RECOGNITION_REQUEST = bytes((0x76, 0x03, 0, 0, 0, 0, 0, 0x67))
MODE5_BEGIN = bytes((0x76, 0x03, 0x05, 0, 0, 0, 0, 0x67))
MODE5_END = bytes((0x76, 0x03, 0x05, 0x01, 0, 0, 0, 0x67))

OK_RESPONSE = bytes((0x92, 0x4F, 0x4B, 0, 0, 0, 0, 0x29))
WEIGHT_LOADED = bytes((0x92, 0x57, 0x4C, 0x59, 0, 0, 0, 0x29))
WEIGHT_FAILED = bytes((0x92, 0x57, 0x4C, 0x4E, 0, 0, 0, 0x29))
REJECT_RESPONSE = bytes(8)
BALL_RECOGNIZED_RESPONSE = bytes((0x92, 0x6F, 0x6B, 0, 0, 0, 0, 0x29))
MODE34_EQUILIBRIUM_BIAS_DEG = -0.35
MODE2_CONTROL_PROFILE = "my_pos"
MODE34_CONTROL_PROFILE = "my_zero"
MODE5_CONTROL_PROFILE = "my_zero"


def format_frame(frame: bytes) -> str:
    return " ".join("{:02X}".format(value) for value in frame)


class RequestFrameParser:
    """从任意分片串口字节流中提取0x76...0x67固定帧。"""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, chunk: bytes) -> List[bytes]:
        self.buffer.extend(chunk)
        frames: List[bytes] = []
        while True:
            try:
                start = self.buffer.index(REQUEST_HEADER)
            except ValueError:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < FRAME_LENGTH:
                break
            if self.buffer[FRAME_LENGTH - 1] != REQUEST_TAIL:
                del self.buffer[0]
                continue
            frames.append(bytes(self.buffer[:FRAME_LENGTH]))
            del self.buffer[:FRAME_LENGTH]
        return frames


def centered_target_from_stable_points(
    points: Any,
    config: Dict[str, Any],
    max_rms_spread_mm: float,
) -> tuple:
    """用零点标定同款稳健中位数估计模式5记录位置。"""

    center, residuals, inlier_count = robust_zero_point(
        np.asarray(points, dtype=np.float64)
    )
    rms_spread_mm = float(np.sqrt(np.mean(residuals ** 2))) * 1000.0
    if rms_spread_mm > float(max_rms_spread_mm):
        raise ValueError(
            "钢珠尚未稳定：球心RMS离散{:.3f}mm，要求不超过{:.3f}mm。"
            .format(rms_spread_mm, float(max_rms_spread_mm))
        )
    position_m = ball_position_from_zero(
        center,
        config["zero_point_base_m"],
        float(config["pipe_length_m"]),
    )
    centered_cm = (
        position_m - float(config["target_coordinate_center_m"])
    ) * 100.0
    endpoint_offset_m = float(config["zero_calibration_ball_radius_m"])
    half_span_cm = (
        float(config["pipe_length_m"]) / 2.0 - endpoint_offset_m
    ) * 100.0
    if not -half_span_cm <= centered_cm <= half_span_cm:
        raise ValueError(
            "模式5记录位置{:+.3f}cm超出允许范围±{:.3f}cm。"
            .format(centered_cm, half_span_cm)
        )
    return centered_cm, rms_spread_mm, inlier_count


class PersistentVision:
    """持有唯一YOLO/RealSense进程并向当前控制器广播JSON。"""

    def __init__(
        self,
        stream_host: str,
        stream_port: int,
        stream_fps: int,
        stream_bitrate: int,
        tracker_fifo_path: str,
        extra_tracker_args: List[str],
    ) -> None:
        self.stream_host = stream_host
        self.stream_port = int(stream_port)
        self.stream_fps = int(stream_fps)
        self.stream_bitrate = int(stream_bitrate)
        self.tracker_fifo_path = Path(tracker_fifo_path)
        self.fifo_fd: Optional[int] = None
        self.extra_tracker_args = list(extra_tracker_args)
        self.process: Optional[subprocess.Popen] = None
        self.reader: Optional[threading.Thread] = None
        self.latest_record: Optional[Dict[str, Any]] = None
        self.latest_received_s: Optional[float] = None
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.stop_requested = threading.Event()
        self.generation = 0

    def start(self) -> None:
        self.stop()
        if self.tracker_fifo_path.exists() and not self.tracker_fifo_path.is_fifo():
            self.tracker_fifo_path.unlink()
        if not self.tracker_fifo_path.exists():
            os.mkfifo(str(self.tracker_fifo_path), 0o600)
        self.stop_requested.clear()
        self.ready.clear()
        with self.lock:
            self.latest_record = None
            self.latest_received_s = None
            self.generation += 1
            generation = self.generation
        command = [
            sys.executable,
            "-u",
            str(TRACKER),
            "--no-display",
            "--print-every",
            "1",
            "--stream-host",
            self.stream_host,
            "--stream-port",
            str(self.stream_port),
            "--stream-fps",
            str(self.stream_fps),
            "--stream-bitrate",
            str(self.stream_bitrate),
            *self.extra_tracker_args,
        ]
        print("启动常驻视觉：{}".format(" ".join(command)), file=sys.stderr)
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=None,
            text=True,
            bufsize=1,
        )
        self.reader = threading.Thread(
            target=self._read_records,
            args=(self.process, generation),
            name="competition-vision-reader",
            daemon=True,
        )
        self.reader.start()

    def _read_records(
        self, process: subprocess.Popen, generation: int
    ) -> None:
        if process.stdout is None:
            return
        for line in process.stdout:
            if self.stop_requested.is_set():
                break
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(record, dict):
                continue
            now = time.monotonic()
            with self.lock:
                if generation != self.generation:
                    break
                self.latest_record = record
                self.latest_received_s = now
            self.ready.set()
            payload = (json.dumps(
                record, ensure_ascii=False, separators=(",", ":")
            ) + "\n").encode("utf-8")
            try:
                if self.fifo_fd is None:
                    self.fifo_fd = os.open(
                        str(self.tracker_fifo_path),
                        os.O_WRONLY | os.O_NONBLOCK,
                    )
                written = os.write(self.fifo_fd, payload)
                if written != len(payload):
                    raise OSError("共享视觉FIFO发生短写")
            except OSError as error:
                if error.errno not in (errno.ENXIO, errno.EPIPE):
                    print("共享视觉FIFO写入失败：{}".format(error), file=sys.stderr)
                if self.fifo_fd is not None:
                    os.close(self.fifo_fd)
                    self.fifo_fd = None

    def is_process_alive(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def wait_loaded(self, timeout_s: float) -> bool:
        if not self.ready.wait(max(0.0, float(timeout_s))):
            return False
        return self.is_process_alive()

    def reload_and_check(self, timeout_s: float) -> bool:
        self.start()
        return self.wait_loaded(timeout_s)

    def latest(self, max_age_s: float) -> Optional[Dict[str, Any]]:
        with self.lock:
            if self.latest_record is None or self.latest_received_s is None:
                return None
            if time.monotonic() - self.latest_received_s > float(max_age_s):
                return None
            return dict(self.latest_record)

    def stream_active(self) -> bool:
        with self.lock:
            return bool(
                self.latest_record is not None
                and self.latest_record.get("video_stream_active", False)
            )

    def collect_stable_centered_target(
        self,
        config: Dict[str, Any],
        sample_count: int,
        timeout_s: float,
        max_record_age_s: float,
        max_rms_spread_mm: float,
    ) -> tuple:
        """从常驻视觉收集不重复的新帧，直到球心窗口稳定。"""

        points = deque(maxlen=int(sample_count))
        deadline = time.monotonic() + float(timeout_s)
        last_record_key: Any = None
        last_error: Optional[Exception] = None
        while time.monotonic() < deadline:
            if not self.is_process_alive():
                raise RuntimeError("常驻YOLO/RealSense进程已经退出。")
            record = self.latest(max_record_age_s)
            if record is None:
                time.sleep(0.005)
                continue
            record_key = (
                record.get("capture_monotonic_ms"),
                record.get("frame"),
            )
            if record_key == last_record_key:
                time.sleep(0.005)
                continue
            last_record_key = record_key
            point = base_point_from_record(record)
            if point is None:
                continue
            points.append(point)
            if len(points) < int(sample_count):
                continue
            try:
                return centered_target_from_stable_points(
                    list(points), config, max_rms_spread_mm
                )
            except ValueError as error:
                # 钢珠仍在运动时保留滑动窗口，下一帧继续判断，而不是把
                # 一次不稳定直接当成识别失败。
                last_error = error
        if len(points) < 10:
            raise ValueError(
                "模式5识别超时：仅取得{}个有效球心样本。".format(
                    len(points)
                )
            )
        if last_error is not None:
            raise ValueError("模式5识别超时：{}".format(last_error))
        raise ValueError("模式5识别超时，未得到稳定球心位置。")

    def stop(self) -> None:
        process = self.process
        self.process = None
        if process is None:
            return
        self.stop_requested.set()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if process.stdout is not None:
            process.stdout.close()
        reader = self.reader
        self.reader = None
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=1.0)
        if self.fifo_fd is not None:
            os.close(self.fifo_fd)
            self.fifo_fd = None

    def close(self) -> None:
        self.stop()
        try:
            self.tracker_fifo_path.unlink()
        except FileNotFoundError:
            pass


class ControlSession:
    """通过PTY运行现有控制器，由主服务独占并代理物理UART。"""

    def __init__(
        self,
        mode: str,
        tracker_fifo_path: str,
        serial_write: Any,
        start_position_cm: float = 0.0,
        expected_initial_target: Optional[bytes] = None,
        target_position_cm: float = 0.0,
        equilibrium_bias_deg: Optional[float] = None,
    ) -> None:
        self.mode = mode
        self.tracker_fifo_path = str(tracker_fifo_path)
        self.serial_write = serial_write
        self.start_position_cm = float(start_position_cm)
        self.target_position_cm = float(target_position_cm)
        self.equilibrium_bias_deg = (
            None
            if equilibrium_bias_deg is None
            else float(equilibrium_bias_deg)
        )
        self.expected_initial_target = expected_initial_target
        self.process: Optional[subprocess.Popen] = None
        self.master_fd: Optional[int] = None
        self.reader: Optional[threading.Thread] = None
        self.log_reader: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.target_ready = threading.Event()
        self.lock = threading.Lock()
        self.authorized = False
        self.latest_target: Optional[bytes] = None

    def _command(self, slave_path: str) -> List[str]:
        common = [
            sys.executable,
            "-u",
            str(CONTROLLER),
            "--controller",
            "cascade_pid",
            "--enable-serial",
            "--port",
            slave_path,
            "--no-stream",
            "--no-control-ui",
            "--no-plot-ui",
            "--shared-tracker-fifo",
            self.tracker_fifo_path,
            "--print-every",
            "0",
        ]
        if self.mode == "mode2":
            common.extend(
                [
                    "--control-profile",
                    MODE2_CONTROL_PROFILE,
                    "--special-task",
                    "minus4p5_then_plus5",
                    "--auto-start-special-task",
                    "--auto-special-start-position-cm",
                    str(self.start_position_cm),
                ]
            )
        elif self.mode == "mode5":
            if self.equilibrium_bias_deg is None:
                raise ValueError("mode5缺少按记录位置选择的平衡基准角。")
            common.extend(
                [
                    "--control-profile",
                    MODE5_CONTROL_PROFILE,
                    "--target-cm",
                    str(self.target_position_cm),
                    "--equilibrium-angle-bias-deg",
                    str(self.equilibrium_bias_deg),
                    "--no-position-local-zero-prior",
                ]
            )
        elif self.mode in ("mode3", "mode4"):
            common.extend(
                [
                    "--control-profile",
                    MODE34_CONTROL_PROFILE,
                    "--target-cm",
                    "0",
                    "--equilibrium-angle-bias-deg",
                    str(MODE34_EQUILIBRIUM_BIAS_DEG),
                    "--no-position-local-zero-prior",
                ]
            )
        else:
            common.extend(["--target-cm", "0"])
        return common

    def start(self) -> None:
        master_fd, slave_fd = os.openpty()
        slave_path = os.ttyname(slave_fd)
        os.close(slave_fd)
        os.set_blocking(master_fd, False)
        self.master_fd = master_fd
        command = self._command(slave_path)
        print(
            "启动{}控制器：{}".format(self.mode, " ".join(command)),
            file=sys.stderr,
        )
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.reader = threading.Thread(
            target=self._read_targets,
            name="competition-target-proxy",
            daemon=True,
        )
        self.reader.start()
        self.log_reader = threading.Thread(
            target=self._read_logs,
            name="competition-controller-log",
            daemon=True,
        )
        self.log_reader.start()

    def _read_targets(self) -> None:
        assert self.master_fd is not None
        buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                readable, _, _ = select.select([self.master_fd], [], [], 0.1)
                if not readable:
                    continue
                chunk = os.read(self.master_fd, 256)
            except OSError as error:
                # 子进程尚未打开PTY从端时，主端短暂返回EIO；继续等待。
                if error.errno == errno.EIO and self.is_alive():
                    time.sleep(0.01)
                    continue
                break
            except ValueError:
                break
            if not chunk:
                continue
            buffer.extend(chunk)
            while len(buffer) >= FRAME_LENGTH:
                try:
                    start = buffer.index(RESPONSE_HEADER)
                except ValueError:
                    buffer.clear()
                    break
                if start:
                    del buffer[:start]
                if len(buffer) < FRAME_LENGTH:
                    break
                if buffer[FRAME_LENGTH - 1] != RESPONSE_TAIL:
                    del buffer[0]
                    continue
                frame = bytes(buffer[:FRAME_LENGTH])
                del buffer[:FRAME_LENGTH]
                # 子控制器旧的主动OK帧在代理层丢弃；只转发绝对位置帧。
                if frame[1] not in (0x00, 0x01):
                    continue
                with self.lock:
                    self.latest_target = frame
                    authorized = self.authorized
                if (
                    self.expected_initial_target is None
                    or frame == self.expected_initial_target
                ):
                    self.target_ready.set()
                if authorized:
                    self.serial_write(frame)

    def _read_logs(self) -> None:
        process = self.process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            print("[{}] {}".format(self.mode, line.rstrip()), flush=True)

    def is_alive(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def wait_ready(self, timeout_s: float) -> bool:
        return self.target_ready.wait(timeout_s) and self.is_alive()

    def authorize_after_ok(self) -> None:
        # OK必须先于第一帧电机目标到达MCU。
        self.serial_write(OK_RESPONSE)
        with self.lock:
            self.authorized = True
            latest = self.latest_target
        if latest is not None:
            self.serial_write(latest)

    def stop(self, send_zero: bool = True) -> None:
        with self.lock:
            self.authorized = False
        self.stop_event.set()
        process = self.process
        self.process = None
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        master_fd = self.master_fd
        self.master_fd = None
        if master_fd is not None:
            try:
                os.close(master_fd)
            except OSError:
                pass
        for thread in (self.reader, self.log_reader):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=1.0)
        # 模式2/3/4沿用旧协议由Jetson发送一次0mm；新版模式5结束后由
        # MCU自行回M6上零位，因此正常结束模式5时不能再发送0mm竞争。
        if send_zero:
            self.serial_write(MOTOR_INITIAL_ZERO_FRAME)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="H题比赛一键常驻服务")
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--port", default=None, help="UART2设备，默认读取控制配置")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--stream-host", default="192.168.50.199")
    parser.add_argument("--stream-port", type=int, default=5600)
    parser.add_argument("--stream-fps", type=int, default=20)
    parser.add_argument("--stream-bitrate", type=int, default=1_200_000)
    parser.add_argument(
        "--debug-uart",
        action="store_true",
        help="打印原始UART字节和未知帧；开机自启动默认关闭",
    )
    parser.add_argument(
        "--tracker-fifo",
        default="/tmp/nuedc-ball-tracker.fifo",
        help="常驻视觉与控制器之间的内核FIFO",
    )
    parser.add_argument("--vision-load-timeout-s", type=float, default=30.0)
    parser.add_argument("--ball-max-age-s", type=float, default=0.5)
    parser.add_argument("--mode5-recognition-samples", type=int, default=50)
    parser.add_argument("--mode5-recognition-timeout-s", type=float, default=20.0)
    parser.add_argument("--mode5-stable-rms-mm", type=float, default=3.0)
    parser.add_argument(
        "--mode5-equilibrium-file",
        type=Path,
        default=DEFAULT_MODE5_EQUILIBRIUM_FILE,
    )
    parser.add_argument(
        "tracker_args",
        nargs=argparse.REMAINDER,
        help="写在--后并传给常驻ball_depth_tracker.py",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.mode5_recognition_samples < 10:
        raise ValueError("mode5-recognition-samples不能少于10。")
    if args.mode5_recognition_timeout_s <= 0.0:
        raise ValueError("mode5-recognition-timeout-s必须大于0。")
    if args.mode5_stable_rms_mm <= 0.0:
        raise ValueError("mode5-stable-rms-mm必须大于0。")

    def request_shutdown(_signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    config = json.loads(args.config.expanduser().read_text(encoding="utf-8"))
    serial_port_name = args.port or str(config["serial_port"])
    serial_lock = threading.Lock()
    last_target_log_s = 0.0
    last_raw_rx_log_s = 0.0
    suppressed_raw_rx = 0
    last_unknown_rx_log_s = 0.0
    physical: Optional[Any] = None

    def serial_write(frame: bytes) -> None:
        nonlocal last_target_log_s
        if physical is None:
            raise IOError("UART2尚未连接")
        with serial_lock:
            written = physical.write(frame)
            if written != len(frame):
                raise IOError("UART2短写：{}/{}字节".format(written, len(frame)))
            physical.flush()
        is_target = (
            len(frame) == FRAME_LENGTH
            and frame[0] == RESPONSE_HEADER
            and frame[-1] == RESPONSE_TAIL
            and frame[1] in (0x00, 0x01)
        )
        now = time.monotonic()
        if not is_target or now - last_target_log_s >= 1.0:
            label = "UART2目标" if is_target else "UART2应答"
            print("{} TX {}".format(label, format_frame(frame)), file=sys.stderr)
            if is_target:
                last_target_log_s = now

    vision = PersistentVision(
        args.stream_host,
        args.stream_port,
        args.stream_fps,
        args.stream_bitrate,
        args.tracker_fifo,
        list(args.tracker_args[1:] if args.tracker_args[:1] == ["--"] else args.tracker_args),
    )
    parser = RequestFrameParser()
    session: Optional[ControlSession] = None
    active_mode: Optional[str] = None
    mode5_recorded_target_cm: Optional[float] = None
    mode5_equilibrium_bias_deg: Optional[float] = None
    last_vision_restart_s = time.monotonic()
    vision_ready_announced = False
    stream_failure_announced = False
    last_health_log_s = 0.0
    try:
        vision.start()
        print(
            "YOLO、RealSense和推流正在后台初始化；同时等待UART2设备"
            "{}出现。".format(
                serial_port_name
            ),
            file=sys.stderr,
        )
        serial_retry_count = 0
        while physical is None:
            try:
                physical = open_serial_port(
                    serial_port_name, args.baud, write_timeout=0.2
                )
            except OSError as error:
                if serial_retry_count % 10 == 0:
                    print(
                        "状态：视觉={}；小球={}；推流={}({}) -> {}:{}；"
                        "UART2未连接({})；任务=等待。".format(
                            (
                                "正常" if vision.ready.is_set()
                                else "初始化中" if vision.is_process_alive()
                                else "异常"
                            ),
                            "已识别" if bool(
                                vision.latest(2.0)
                                and vision.latest(2.0).get("valid", False)
                            ) else "未识别",
                            "正常" if vision.stream_active() else "初始化中/异常",
                            (
                                (vision.latest(2.0) or {}).get(
                                    "video_stream_encoder", "未知"
                                )
                            ),
                            args.stream_host,
                            args.stream_port,
                            error,
                        ),
                        file=sys.stderr,
                    )
                serial_retry_count += 1
                if not vision.is_process_alive():
                    print("等待串口期间视觉退出，立即重新加载。", file=sys.stderr)
                    vision.start()
                time.sleep(1.0)
        print(
            "UART2已开始监听：{} @ {} baud；等待比赛请求。".format(
                serial_port_name, args.baud
            ),
            file=sys.stderr,
        )
        while True:
            chunk = physical.read(256)
            if not chunk:
                now = time.monotonic()
                if vision.ready.is_set() and vision.is_process_alive():
                    if not vision_ready_announced:
                        print(
                            "视觉自检通过：默认YOLO权重和RealSense已逐帧运行。",
                            file=sys.stderr,
                        )
                        vision_ready_announced = True
                    if vision.stream_active():
                        if stream_failure_announced:
                            print("H.264视频推流已经恢复。", file=sys.stderr)
                        stream_failure_announced = False
                    elif not stream_failure_announced:
                        print(
                            "警告：YOLO/RealSense正常，但H.264推流当前未在线。",
                            file=sys.stderr,
                        )
                        stream_failure_announced = True
                if now - last_health_log_s >= 10.0:
                    record = vision.latest(2.0)
                    ball_ok = bool(record and record.get("valid", False))
                    print(
                        "状态：热点保持独立运行；视觉={}；小球={}；推流={}({}) -> {}:{}；"
                        "UART=已连接；任务={}".format(
                            (
                                "正常" if vision.ready.is_set()
                                else "初始化中" if vision.is_process_alive()
                                else "异常"
                            ),
                            "已识别" if ball_ok else "未识别",
                            "正常" if vision.stream_active() else "异常/初始化中",
                            (record or {}).get("video_stream_encoder", "未知"),
                            args.stream_host,
                            args.stream_port,
                            active_mode or "等待",
                        ),
                        file=sys.stderr,
                    )
                    last_health_log_s = now
                if session is not None and not session.is_alive():
                    print(
                        "{}控制器异常退出，发送0.00mm并返回等待状态。"
                        .format(active_mode),
                        file=sys.stderr,
                    )
                    session.stop()
                    session = None
                    if active_mode == "mode5":
                        mode5_recorded_target_cm = None
                        mode5_equilibrium_bias_deg = None
                    active_mode = None
                if (
                    not vision.is_process_alive()
                    and now - last_vision_restart_s >= 2.0
                ):
                    print("常驻视觉异常退出，后台自动重新加载。", file=sys.stderr)
                    vision.start()
                    vision_ready_announced = False
                    stream_failure_announced = False
                    last_vision_restart_s = now
                time.sleep(0.005)
                continue
            # 比赛请求很稀疏，保留原始收包日志便于现场区分“没有电气数据”与
            # “数据已到达但帧头/帧尾/长度不符合协议”。控制目标TX仍限速打印。
            now = time.monotonic()
            if args.debug_uart and now - last_raw_rx_log_s >= 1.0:
                suffix = (
                    "（此前抑制{}个高频收包日志）".format(suppressed_raw_rx)
                    if suppressed_raw_rx else ""
                )
                print(
                    "UART2 RAW RX({}B) {}{}".format(
                        len(chunk), format_frame(chunk), suffix
                    ),
                    file=sys.stderr,
                )
                last_raw_rx_log_s = now
                suppressed_raw_rx = 0
            elif args.debug_uart:
                suppressed_raw_rx += 1
            for frame in parser.feed(chunk):
                recognized = frame in (
                    WEIGHT_QUERY, MODE2_BEGIN, MODE2_END,
                    MODE3_BEGIN, MODE3_END, MODE4_BEGIN, MODE4_END,
                    BALL_RECOGNITION_REQUEST, MODE5_BEGIN, MODE5_END,
                )
                if recognized:
                    print("UART2 RX {}".format(format_frame(frame)), file=sys.stderr)
                elif args.debug_uart and now - last_unknown_rx_log_s >= 1.0:
                    print("UART2 RX {}".format(format_frame(frame)), file=sys.stderr)
                if frame == WEIGHT_QUERY:
                    loaded = vision.is_process_alive() and vision.latest(2.0) is not None
                    if not loaded:
                        if vision.is_process_alive():
                            print("权重仍在加载，等待当前加载结果。", file=sys.stderr)
                            loaded = vision.wait_loaded(args.vision_load_timeout_s)
                        if not loaded:
                            print("权重加载失败，立即重新加载视觉。", file=sys.stderr)
                            loaded = vision.reload_and_check(args.vision_load_timeout_s)
                            vision_ready_announced = False
                            stream_failure_announced = False
                    serial_write(WEIGHT_LOADED if loaded else WEIGHT_FAILED)
                    continue

                if frame == BALL_RECOGNITION_REQUEST:
                    if session is not None:
                        print(
                            "已有{}运行，拒绝模式5球位识别。".format(active_mode),
                            file=sys.stderr,
                        )
                        serial_write(REJECT_RESPONSE)
                        continue
                    # 每次C5B都重新采集，防止沿用上一轮任务留下的位置。
                    mode5_recorded_target_cm = None
                    mode5_equilibrium_bias_deg = None
                    print(
                        "收到模式5球位识别请求：开始收集{}个稳定球心样本。"
                        .format(args.mode5_recognition_samples),
                        file=sys.stderr,
                    )
                    try:
                        (
                            recorded_cm,
                            rms_spread_mm,
                            inlier_count,
                        ) = vision.collect_stable_centered_target(
                            config=config,
                            sample_count=args.mode5_recognition_samples,
                            timeout_s=args.mode5_recognition_timeout_s,
                            max_record_age_s=args.ball_max_age_s,
                            max_rms_spread_mm=args.mode5_stable_rms_mm,
                        )
                        (
                            selected_bias_deg,
                            selected_position_cm,
                            selected_height_mm,
                        ) = nearest_mode5_equilibrium(
                            recorded_cm, args.mode5_equilibrium_file
                        )
                    except (KeyError, TypeError, ValueError, RuntimeError) as error:
                        print(
                            "模式5球位识别失败：{}".format(error),
                            file=sys.stderr,
                        )
                        serial_write(REJECT_RESPONSE)
                        continue
                    mode5_recorded_target_cm = float(recorded_cm)
                    mode5_equilibrium_bias_deg = float(selected_bias_deg)
                    print(
                        "模式5球位识别成功：记录目标{:+.3f}cm，"
                        "最近标定点{:+.1f}cm、高度{:+.2f}mm -> 基准角"
                        "{:+.4f}°；RMS={:.3f}mm，内点={}/{}；"
                        "回复小写ok。".format(
                            mode5_recorded_target_cm,
                            selected_position_cm,
                            selected_height_mm,
                            mode5_equilibrium_bias_deg,
                            rms_spread_mm,
                            inlier_count,
                            args.mode5_recognition_samples,
                        ),
                        file=sys.stderr,
                    )
                    serial_write(BALL_RECOGNIZED_RESPONSE)
                    continue

                begin_mode = None
                if frame == MODE2_BEGIN:
                    begin_mode = "mode2"
                elif frame == MODE3_BEGIN:
                    begin_mode = "mode3"
                elif frame == MODE4_BEGIN:
                    begin_mode = "mode4"
                elif frame == MODE5_BEGIN:
                    begin_mode = "mode5"
                if begin_mode is not None:
                    if session is not None:
                        print("已有{}运行，拒绝{}。".format(active_mode, begin_mode), file=sys.stderr)
                        serial_write(REJECT_RESPONSE)
                        continue
                    if begin_mode == "mode5" and (
                        mode5_recorded_target_cm is None
                        or mode5_equilibrium_bias_deg is None
                    ):
                        print(
                            "mode5启动检查失败：尚未通过C5B识别并记录稳定球位。",
                            file=sys.stderr,
                        )
                        serial_write(REJECT_RESPONSE)
                        continue
                    record = vision.latest(args.ball_max_age_s)
                    if record is not None:
                        try:
                            capture_age_s = time.perf_counter() - (
                                float(record["capture_monotonic_ms"]) / 1000.0
                            )
                        except (KeyError, TypeError, ValueError):
                            record = None
                        else:
                            if not 0.0 <= capture_age_s <= args.ball_max_age_s:
                                record = None
                    point = None if record is None else base_point_from_record(record)
                    centered_cm = None
                    if point is not None:
                        try:
                            position_m = ball_position_from_zero(
                                point,
                                config["zero_point_base_m"],
                                float(config["pipe_length_m"]),
                            )
                            centered_cm = (
                                position_m - float(config["target_coordinate_center_m"])
                            ) * 100.0
                        except ValueError:
                            centered_cm = None
                    position_ok = centered_cm is not None
                    if begin_mode == "mode2":
                        position_ok = position_ok and abs(centered_cm) <= 1.0
                    if not position_ok:
                        print(
                            "{}启动检查失败：没有新鲜有效球心{}。".format(
                                begin_mode,
                                "或钢珠不在中心±1cm" if begin_mode == "mode2" else "",
                            ),
                            file=sys.stderr,
                        )
                        serial_write(REJECT_RESPONSE)
                        continue
                    expected_initial_target = MOTOR_INITIAL_ZERO_FRAME
                    if begin_mode == "mode2":
                        task = config["special_task"]
                        expected_initial_target = encode_angle(
                            task["first_angle_deg"],
                            task["positive_motor_scale"],
                            task["negative_motor_scale"],
                        )
                    candidate = ControlSession(
                        begin_mode,
                        args.tracker_fifo,
                        serial_write,
                        start_position_cm=centered_cm,
                        expected_initial_target=expected_initial_target,
                        target_position_cm=(
                            mode5_recorded_target_cm
                            if begin_mode == "mode5"
                            else 0.0
                        ),
                        equilibrium_bias_deg=(
                            mode5_equilibrium_bias_deg
                            if begin_mode == "mode5"
                            else None
                        ),
                    )
                    candidate.start()
                    if not candidate.wait_ready(5.0):
                        print("{}控制器初始化失败。".format(begin_mode), file=sys.stderr)
                        candidate.stop()
                        serial_write(REJECT_RESPONSE)
                        continue
                    session = candidate
                    active_mode = begin_mode
                    print(
                        (
                            "{}准备完成，当前钢珠{:+.3f}cm，记录目标{:+.3f}cm，"
                            "发送OK后开始控制。"
                            if begin_mode == "mode5"
                            else "{}准备完成，钢珠{:+.3f}cm，发送OK后开始控制。"
                        ).format(
                            begin_mode,
                            centered_cm,
                            *(
                                (mode5_recorded_target_cm,)
                                if begin_mode == "mode5"
                                else ()
                            ),
                        ),
                        file=sys.stderr,
                    )
                    session.authorize_after_ok()
                    continue

                ending_mode = None
                if frame == MODE2_END:
                    ending_mode = "mode2"
                elif frame == MODE3_END:
                    ending_mode = "mode3"
                elif frame == MODE4_END:
                    ending_mode = "mode4"
                elif frame == MODE5_END:
                    ending_mode = "mode5"
                if ending_mode is not None:
                    if session is not None and active_mode == ending_mode:
                        if ending_mode == "mode5":
                            print(
                                "结束mode5：停止闭环；由MCU回M6上零位，"
                                "常驻视觉和推流继续运行。",
                                file=sys.stderr,
                            )
                            session.stop(send_zero=False)
                            mode5_recorded_target_cm = None
                            mode5_equilibrium_bias_deg = None
                        else:
                            print(
                                "结束{}：停止闭环并发送一次0.00mm。"
                                .format(ending_mode),
                                file=sys.stderr,
                            )
                            session.stop()
                        session = None
                        active_mode = None
                    else:
                        print("忽略非当前模式的结束帧：{}。".format(ending_mode), file=sys.stderr)
                        if ending_mode == "mode5" and active_mode is None:
                            mode5_recorded_target_cm = None
                            mode5_equilibrium_bias_deg = None
                    continue
                if args.debug_uart and now - last_unknown_rx_log_s >= 1.0:
                    print("未知UART2请求，忽略。", file=sys.stderr)
                    last_unknown_rx_log_s = now
    except KeyboardInterrupt:
        return 130
    finally:
        if session is not None:
            session.stop()
        vision.close()
        if physical is not None:
            physical.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
