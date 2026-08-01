#!/usr/bin/env python3
"""把管道倾角换算为电机升降位移并按 8 字节协议持续发送。

线上的数据固定为 8 字节：

    [0x92, 符号, 整数部分, 两位小数部分, 0x00, 0x00, 0x00, 0x29]

符号 0x00 表示正数，0x01 表示负数。整数和小数部分均为二进制
字节值，线上数值的单位是 mm。控制器内部仍输出倾角，发送前按
``height_mm = 250 * tan(angle_deg)`` 换算；正值表示电机抬高，
负值表示电机下降。

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
LINKAGE_LENGTH_MM = 250.0
MAX_DISPLACEMENT_MM = Decimal("99.99")
MAX_ANGLE_DEG = Decimal("21.80")
# 串口位移符号与控制器定义一致；保留单点开关便于接线测试。
SERIAL_SIGN_INVERTED = False
_HUNDREDTHS_SCALE = Decimal("100")
_INTEGER_QUANTUM = Decimal("1")


class AngleEncodingError(ValueError):
    """倾角或换算后的电机位移不能安全编码时抛出。"""


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


def _finite_decimal(value: Any, name: str) -> Decimal:
    if isinstance(value, bool):
        raise AngleEncodingError("{}必须是数值，不能是布尔值".format(name))
    try:
        decimal_value = Decimal(str(value))
    except (InvalidOperation, TypeError, ValueError) as exc:
        raise AngleEncodingError("{}必须是有效数值".format(name)) from exc
    if not decimal_value.is_finite():
        raise AngleEncodingError("{}必须是有限值".format(name))
    return decimal_value


def validate_motor_displacement_scale(scale: Any) -> float:
    """校验倾角到电机升降位移的独立比例系数。"""

    value = _finite_decimal(scale, "电机升降比例系数")
    if value <= 0 or value > Decimal("2.0"):
        raise AngleEncodingError("电机升降比例系数必须在(0, 2.0]范围内")
    return float(value)


def angle_to_motor_displacement_mm(
    angle_deg: Any,
    motor_displacement_scale: Any = 1.0,
    negative_motor_displacement_scale: Any = None,
) -> float:
    """按倾角正负选择独立比例，再换算电机端升降位移。"""

    angle = _finite_decimal(angle_deg, "倾角")
    if abs(angle) > MAX_ANGLE_DEG:
        raise AngleEncodingError(
            "倾角必须在 [-{0}, +{0}] 度范围内".format(
                MAX_ANGLE_DEG
            )
        )
    positive_scale = validate_motor_displacement_scale(
        motor_displacement_scale
    )
    negative_scale = validate_motor_displacement_scale(
        motor_displacement_scale
        if negative_motor_displacement_scale is None
        else negative_motor_displacement_scale
    )
    scale = positive_scale if angle >= 0 else negative_scale
    displacement_mm = scale * LINKAGE_LENGTH_MM * math.tan(
        math.radians(float(angle))
    )
    if not math.isfinite(displacement_mm):
        raise AngleEncodingError("倾角换算得到的电机位移不是有限值")
    return displacement_mm


def angle_to_serial_displacement_mm(
    angle_deg: Any,
    motor_displacement_scale: Any = 1.0,
    negative_motor_displacement_scale: Any = None,
) -> float:
    """返回实际写入串口帧的毫米数，包含临时符号反转。"""

    displacement_mm = angle_to_motor_displacement_mm(
        angle_deg,
        motor_displacement_scale,
        negative_motor_displacement_scale,
    )
    return (
        -displacement_mm
        if SERIAL_SIGN_INVERTED
        else displacement_mm
    )


def encode_motor_displacement_mm(displacement_mm: Any) -> bytes:
    """把[-99.99,+99.99] mm位移编码为8字节帧。"""

    value = _finite_decimal(displacement_mm, "电机位移")
    hundredths = int(
        (abs(value) * _HUNDREDTHS_SCALE).quantize(
            _INTEGER_QUANTUM,
            rounding=ROUND_HALF_UP,
        )
    )
    if hundredths > int(MAX_DISPLACEMENT_MM * _HUNDREDTHS_SCALE):
        raise AngleEncodingError(
            "电机位移必须在 [-99.99, +99.99] mm范围内"
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


def encode_angle(
    angle_deg: Any,
    motor_displacement_scale: Any = 1.0,
    negative_motor_displacement_scale: Any = None,
) -> bytes:
    """把倾角换算成电机升降毫米数后编码。

    倾角只存在于控制器与终端；线协议的数值始终是毫米位移。
    """

    return encode_motor_displacement_mm(
        angle_to_serial_displacement_mm(
            angle_deg,
            motor_displacement_scale,
            negative_motor_displacement_scale,
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
        motor_displacement_scale: Any = 1.0,
        negative_motor_displacement_scale: Any = None,
        close_port: bool = False,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._serial_port = serial_port
        self._rate_hz = validate_rate_hz(rate_hz)
        self._period_s = 1.0 / self._rate_hz
        self._motor_displacement_scale = validate_motor_displacement_scale(
            motor_displacement_scale
        )
        self._negative_motor_displacement_scale = (
            validate_motor_displacement_scale(
                motor_displacement_scale
                if negative_motor_displacement_scale is None
                else negative_motor_displacement_scale
            )
        )
        self._latest_angle_deg = float(
            _finite_decimal(initial_angle_deg, "倾角")
        )
        self._target_angle_deg = self._latest_angle_deg
        self._max_angle_command_step_deg = None  # type: Optional[float]
        self._latest_displacement_mm = angle_to_serial_displacement_mm(
            self._latest_angle_deg,
            self._motor_displacement_scale,
            self._negative_motor_displacement_scale,
        )
        self._latest_frame = encode_motor_displacement_mm(
            self._latest_displacement_mm
        )
        self._close_port = bool(close_port)
        self._clock = clock

        self._frame_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._periodic_gate_lock = threading.Lock()
        self._motor_enable_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sending_enabled = threading.Event()
        self._sending_enabled.set()
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
        motor_displacement_scale: Any = 1.0,
        negative_motor_displacement_scale: Any = None,
        write_timeout: float = 0.1,
        serial_factory: Optional[SerialFactory] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> "PeriodicAngleSender":
        """打开串口并返回拥有该连接的发送器。"""

        # 在打开硬件前先完成所有纯参数校验，避免参数错误时泄漏串口。
        validated_rate = validate_rate_hz(rate_hz)
        validated_scale = validate_motor_displacement_scale(
            motor_displacement_scale
        )
        validated_negative_scale = validate_motor_displacement_scale(
            motor_displacement_scale
            if negative_motor_displacement_scale is None
            else negative_motor_displacement_scale
        )
        encode_angle(
            initial_angle_deg, validated_scale, validated_negative_scale
        )
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
                motor_displacement_scale=validated_scale,
                negative_motor_displacement_scale=validated_negative_scale,
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

    @property
    def latest_angle_deg(self) -> float:
        with self._frame_lock:
            return self._latest_angle_deg

    @property
    def target_angle_deg(self) -> float:
        with self._frame_lock:
            return self._target_angle_deg

    @property
    def motor_displacement_scale(self) -> float:
        with self._frame_lock:
            return self._motor_displacement_scale

    @property
    def negative_motor_displacement_scale(self) -> float:
        with self._frame_lock:
            return self._negative_motor_displacement_scale

    @property
    def latest_displacement_mm(self) -> float:
        with self._frame_lock:
            return self._latest_displacement_mm

    def set_max_angle_command_step_deg(
        self, maximum_step_deg: Optional[Any]
    ) -> None:
        """设置相邻目标帧的最大角度差；None表示关闭。"""

        if maximum_step_deg is None:
            validated = None
        else:
            validated = float(
                _finite_decimal(maximum_step_deg, "平滑指数")
            )
            if not 0.0 < validated <= float(MAX_ANGLE_DEG):
                raise AngleEncodingError(
                    "平滑指数必须在(0,21.80] deg/次范围内"
                )
        with self._frame_lock:
            self._max_angle_command_step_deg = validated
            if validated is None:
                self._frame_for_angle_locked(
                    self._target_angle_deg, bypass_step_limit=True
                )

    def _frame_for_angle_locked(
        self, angle_deg: float, bypass_step_limit: bool = False
    ) -> bytes:
        command_angle_deg = angle_deg
        step = self._max_angle_command_step_deg
        if step is not None and not bypass_step_limit:
            difference = command_angle_deg - self._latest_angle_deg
            if difference > step:
                command_angle_deg = self._latest_angle_deg + step
            elif difference < -step:
                command_angle_deg = self._latest_angle_deg - step
        self._latest_angle_deg = command_angle_deg
        self._latest_displacement_mm = angle_to_serial_displacement_mm(
            command_angle_deg,
            self._motor_displacement_scale,
            self._negative_motor_displacement_scale,
        )
        self._latest_frame = encode_motor_displacement_mm(
            self._latest_displacement_mm
        )
        return self._latest_frame

    def set_motor_displacement_scale(self, scale: Any) -> bytes:
        """兼容旧接口：把正、负比例同时设为同一个值。"""

        return self.set_motor_displacement_scales(scale, scale)

    def set_motor_displacement_scales(
        self, positive_scale: Any, negative_scale: Any
    ) -> bytes:
        """实时独立修改正、负指令比例，并重建下一发送帧。"""

        validated_positive = validate_motor_displacement_scale(positive_scale)
        validated_negative = validate_motor_displacement_scale(negative_scale)
        with self._state_lock:
            if self._closed:
                raise RuntimeError("发送器已经关闭")
        with self._frame_lock:
            self._motor_displacement_scale = validated_positive
            self._negative_motor_displacement_scale = validated_negative
            self._latest_displacement_mm = angle_to_serial_displacement_mm(
                self._latest_angle_deg,
                self._motor_displacement_scale,
                self._negative_motor_displacement_scale,
            )
            self._latest_frame = encode_motor_displacement_mm(
                self._latest_displacement_mm
            )
            return self._latest_frame

    def set_angle(self, angle_deg: Any) -> bytes:
        """原子更新下一周期开始发送的倾角，并返回编码结果。"""

        with self._state_lock:
            if self._closed:
                raise RuntimeError("发送器已经关闭")
            error = self._error
        if error is not None:
            raise RuntimeError("周期串口发送已经失败") from error
        with self._frame_lock:
            validated_angle = float(_finite_decimal(angle_deg, "倾角"))
            # 只保留最新目标。启用平滑时不在这里生成中间队列，50 Hz
            # 发送线程会从当前已发送角直接追踪这个最新值。
            angle_to_serial_displacement_mm(
                validated_angle,
                self._motor_displacement_scale,
                self._negative_motor_displacement_scale,
            )
            self._target_angle_deg = validated_angle
            if self._max_angle_command_step_deg is None:
                return self._frame_for_angle_locked(
                    validated_angle, bypass_step_limit=True
                )
            return self._latest_frame

    def force_angle(self, angle_deg: Any) -> bytes:
        """绕过平滑限制立即设置角度，用于强制归零。"""

        with self._state_lock:
            if self._closed:
                raise RuntimeError("发送器已经关闭")
        with self._frame_lock:
            validated_angle = float(_finite_decimal(angle_deg, "倾角"))
            self._target_angle_deg = validated_angle
            return self._frame_for_angle_locked(
                validated_angle, bypass_step_limit=True
            )

    def _next_transmit_frame(self) -> bytes:
        """生成下一串口帧，只向当前最新目标移动一步。"""

        with self._frame_lock:
            return self._frame_for_angle_locked(self._target_angle_deg)

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
            self.set_angle(angle_deg)
            frame = self._next_transmit_frame()
        else:
            with self._state_lock:
                if self._closed:
                    raise RuntimeError("发送器已经关闭")
            frame = self._next_transmit_frame()
        self._write_frame(frame)
        return frame

    def _run(self) -> None:
        next_deadline = self._clock()
        try:
            while not self._stop_event.is_set():
                # 检查“允许发送”和实际写串口放在同一把锁内，保证
                # pause_sending返回后绝不会再漏出一帧周期指令。
                with self._periodic_gate_lock:
                    if self._sending_enabled.is_set():
                        frame = self._next_transmit_frame()
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

    def pause_sending(self) -> None:
        """暂停周期串口写入，并等待已经开始的一帧发送完成。"""

        self._sending_enabled.clear()
        with self._periodic_gate_lock:
            pass

    def resume_sending(self) -> None:
        """恢复周期串口写入。"""

        with self._state_lock:
            if self._closed:
                raise RuntimeError("发送器已经关闭")
        self._sending_enabled.set()

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

        self.force_angle(0.0)
        self.stop(timeout=timeout)
        return self.send_once()

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
