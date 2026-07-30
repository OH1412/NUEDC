#!/usr/bin/env python3
"""管道倾角的 8 字节串口协议和持续发送器。

线上的数据固定为 8 字节：

    [0x92, 符号, 整数部分, 两位小数部分, 0x00, 0x00, 0x00, 0x29]

符号 0x00 表示正数，0x01 表示负数。整数和小数部分均为二进制
字节值，例如 -12.34 度编码为 ``92 01 0C 22 00 00 00 29``。

本模块故意延迟导入 pyserial。项目内已有名为 ``serial`` 的工具
目录，而 pyserial 的导入名同样是 ``serial``；控制器应直接从 H
目录导入本模块，不要尝试 ``from serial.send import ...``。
"""

import math
import threading
import time
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from typing import Any, Callable, Optional, Protocol


HEADER = 0x92
FOOTER = 0x29
PAYLOAD_LENGTH = 8
MOTOR_ENABLE_FRAME = bytes(
    (HEADER, ord("O"), ord("K"), 0x00, 0x00, 0x00, 0x00, FOOTER)
)
MOTOR_INITIAL_ZERO_FRAME = bytes(
    (HEADER, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, FOOTER)
)
MIN_RATE_HZ = 20.0
DEFAULT_RATE_HZ = 50.0
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 9600
MAX_ANGLE_DEG = Decimal("30")
_HUNDREDTHS_SCALE = Decimal("100")
_INTEGER_QUANTUM = Decimal("1")


class AngleEncodingError(ValueError):
    """倾角不能安全编码到线协议时抛出。"""


class SerialDependencyError(RuntimeError):
    """pyserial 未安装或导入到了错误的同名模块。"""


class WritableSerial(Protocol):
    """发送器实际使用的最小串口接口，便于注入假串口测试。"""

    def write(self, data: bytes) -> int:
        ...

    def flush(self) -> None:
        ...

    def close(self) -> None:
        ...


SerialFactory = Callable[..., WritableSerial]


def encode_angle(angle_deg: Any) -> bytes:
    """把 [-30, 30] 度的倾角编码为精确的 8 字节帧。

    输入按十进制四舍五入到 0.01 度。量化后为零时统一使用正号，
    因此 -0.0 和 -0.004 都不会产生“负零”。
    """

    if isinstance(angle_deg, bool):
        raise AngleEncodingError("倾角必须是数值，不能是布尔值")

    try:
        value = Decimal(str(angle_deg))
    except (InvalidOperation, TypeError, ValueError) as exc:
        raise AngleEncodingError("倾角必须是有效数值") from exc

    if not value.is_finite():
        raise AngleEncodingError("倾角必须是有限值")
    if abs(value) > MAX_ANGLE_DEG:
        raise AngleEncodingError("倾角必须在 [-30, 30] 度范围内")

    hundredths = int(
        (abs(value) * _HUNDREDTHS_SCALE).quantize(
            _INTEGER_QUANTUM,
            rounding=ROUND_HALF_UP,
        )
    )
    integer_part, fractional_part = divmod(hundredths, 100)
    sign = 0x01 if hundredths != 0 and value < 0 else 0x00

    return bytes(
        (
            HEADER,
            sign,
            integer_part,
            fractional_part,
            0x00,
            0x00,
            0x00,
            FOOTER,
        )
    )


def validate_rate_hz(rate_hz: Any) -> float:
    """校验并返回周期发送频率。"""

    if isinstance(rate_hz, bool):
        raise ValueError("发送频率必须是数值")
    try:
        rate = float(rate_hz)
    except (TypeError, ValueError) as exc:
        raise ValueError("发送频率必须是数值") from exc
    if not math.isfinite(rate):
        raise ValueError("发送频率必须是有限值")
    if rate < MIN_RATE_HZ:
        raise ValueError(
            "发送频率不得低于 {:.1f} Hz".format(MIN_RATE_HZ)
        )
    return rate


def open_serial_port(
    port: str = DEFAULT_PORT,
    baudrate: int = DEFAULT_BAUD,
    write_timeout: float = 0.1,
    serial_factory: Optional[SerialFactory] = None,
) -> WritableSerial:
    """打开一个供发送器长期复用的串口。

    ``serial_factory`` 用于测试或集成其他串口实现；为空时才导入
    pyserial，从而避免纯协议测试依赖硬件和 pyserial。
    """

    factory = serial_factory
    if factory is None:
        try:
            import serial as pyserial
        except ImportError as exc:
            raise SerialDependencyError(
                "未安装 pyserial；请使用项目环境并安装 pyserial==3.5"
            ) from exc

        factory = getattr(pyserial, "Serial", None)
        if factory is None:
            raise SerialDependencyError(
                "导入到的 serial 不是 pyserial；请从 H/angle_serial.py "
                "导入控制接口，不要把项目 serial/ 目录作为 Python 包"
            )

    return factory(
        port=port,
        baudrate=int(baudrate),
        timeout=0,
        write_timeout=float(write_timeout),
        exclusive=True,
    )


class PeriodicAngleSender:
    """在后台按固定频率重发最近一次倾角命令。

    控制器调用 :meth:`set_angle` 更新目标即可，后台线程默认以
    50 Hz 发送。传入的串口对象在发送器生命周期内始终保持打开。
    """

    def __init__(
        self,
        serial_port: WritableSerial,
        rate_hz: float = DEFAULT_RATE_HZ,
        initial_angle_deg: Any = 0.0,
        close_port: bool = False,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._serial_port = serial_port
        self._rate_hz = validate_rate_hz(rate_hz)
        self._period_s = 1.0 / self._rate_hz
        self._latest_frame = encode_angle(initial_angle_deg)
        self._close_port = bool(close_port)
        self._clock = clock

        self._frame_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._motor_enable_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None  # type: Optional[threading.Thread]
        self._closed = False
        self._started = False
        self._error = None  # type: Optional[BaseException]
        self._frames_sent = 0
        self._motor_enable_sent = False

    @classmethod
    def open(
        cls,
        port: str = DEFAULT_PORT,
        baudrate: int = DEFAULT_BAUD,
        rate_hz: float = DEFAULT_RATE_HZ,
        initial_angle_deg: Any = 0.0,
        write_timeout: float = 0.1,
        serial_factory: Optional[SerialFactory] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> "PeriodicAngleSender":
        """打开串口并返回拥有该连接的发送器。"""

        # 在打开硬件前先完成所有纯参数校验，避免参数错误时泄漏串口。
        validated_rate = validate_rate_hz(rate_hz)
        encode_angle(initial_angle_deg)
        serial_port = open_serial_port(
            port=port,
            baudrate=baudrate,
            write_timeout=write_timeout,
            serial_factory=serial_factory,
        )
        try:
            return cls(
                serial_port=serial_port,
                rate_hz=validated_rate,
                initial_angle_deg=initial_angle_deg,
                close_port=True,
                clock=clock,
            )
        except Exception:
            serial_port.close()
            raise

    @property
    def rate_hz(self) -> float:
        return self._rate_hz

    @property
    def frames_sent(self) -> int:
        with self._state_lock:
            return self._frames_sent

    @property
    def error(self) -> Optional[BaseException]:
        with self._state_lock:
            return self._error

    @property
    def is_running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    @property
    def latest_frame(self) -> bytes:
        with self._frame_lock:
            return self._latest_frame

    def set_angle(self, angle_deg: Any) -> bytes:
        """原子更新下一周期开始发送的倾角，并返回编码结果。"""

        frame = encode_angle(angle_deg)
        with self._state_lock:
            if self._closed:
                raise RuntimeError("发送器已经关闭")
            error = self._error
        if error is not None:
            raise RuntimeError("周期串口发送已经失败") from error
        with self._frame_lock:
            self._latest_frame = frame
        return frame

    def _write_frame(self, frame: bytes) -> None:
        self.send_motor_enable()
        with self._write_lock:
            written = self._serial_port.write(frame)
            if written != PAYLOAD_LENGTH:
                raise IOError(
                    "串口短写：应发送 {} 字节，实际发送 {} 字节".format(
                        PAYLOAD_LENGTH,
                        written,
                    )
                )
            self._serial_port.flush()
        with self._state_lock:
            self._frames_sent += 1

    def send_motor_enable(self) -> bytes:
        """首次角度命令前依次发送一次大写 ``OK`` 使能帧和0°帧。"""

        with self._motor_enable_lock:
            if self._motor_enable_sent:
                return MOTOR_ENABLE_FRAME
            with self._state_lock:
                if self._closed:
                    raise RuntimeError("发送器已经关闭")
            with self._write_lock:
                written = self._serial_port.write(MOTOR_ENABLE_FRAME)
                if written != PAYLOAD_LENGTH:
                    raise IOError(
                        "电机使能帧短写：应发送 {} 字节，实际发送 {} 字节"
                        .format(PAYLOAD_LENGTH, written)
                    )
                self._serial_port.flush()
                written = self._serial_port.write(
                    MOTOR_INITIAL_ZERO_FRAME
                )
                if written != PAYLOAD_LENGTH:
                    raise IOError(
                        "电机初始化0°帧短写：应发送 {} 字节，实际发送 {} 字节"
                        .format(PAYLOAD_LENGTH, written)
                    )
                self._serial_port.flush()
            self._motor_enable_sent = True
            return MOTOR_ENABLE_FRAME

    def send_once(self, angle_deg: Optional[Any] = None) -> bytes:
        """同步发送一帧；可选地先更新倾角。"""

        if angle_deg is not None:
            frame = self.set_angle(angle_deg)
        else:
            with self._state_lock:
                if self._closed:
                    raise RuntimeError("发送器已经关闭")
            frame = self.latest_frame
        self._write_frame(frame)
        return frame

    def _run(self) -> None:
        next_deadline = self._clock()
        try:
            while not self._stop_event.is_set():
                frame = self.latest_frame
                self._write_frame(frame)

                next_deadline += self._period_s
                now = self._clock()
                if next_deadline <= now:
                    # 已错过周期时从当前时刻重新排程，避免突发补发。
                    next_deadline = now + self._period_s
                self._stop_event.wait(next_deadline - now)
        except Exception as exc:
            with self._state_lock:
                self._error = exc
            self._stop_event.set()

    def start(self) -> "PeriodicAngleSender":
        """启动后台周期发送；同一实例不允许停止后重新启动。"""

        with self._state_lock:
            if self._closed:
                raise RuntimeError("发送器已经关闭")
            if self._started:
                if self.is_running:
                    return self
                raise RuntimeError("发送器停止后不能重新启动")
            self._started = True
            self._thread = threading.Thread(
                target=self._run,
                name="angle-serial-sender",
                daemon=True,
            )
            thread = self._thread
        thread.start()
        return self

    def raise_if_failed(self) -> None:
        """若后台发送失败，在控制线程中重新抛出可见异常。"""

        error = self.error
        if error is not None:
            raise RuntimeError("周期串口发送失败") from error

    def stop(self, timeout: float = 1.0) -> None:
        """停止后台线程，但不关闭外部传入的串口。"""

        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0.0, float(timeout)))
            if thread.is_alive():
                raise RuntimeError("周期串口发送线程未能及时停止")

    def stop_and_send_zero(self, timeout: float = 1.0) -> bytes:
        """停止周期发送后同步发送最终0°，避免旧非零帧覆盖归零帧。"""

        self.set_angle(0.0)
        self.stop(timeout=timeout)
        return self.send_once(0.0)

    def close(self) -> None:
        """停止发送，并关闭由 :meth:`open` 创建的串口。"""

        with self._state_lock:
            if self._closed:
                return
        self.stop()
        if self._close_port:
            self._serial_port.close()
        with self._state_lock:
            self._closed = True

    def __enter__(self) -> "PeriodicAngleSender":
        return self.start()

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()
