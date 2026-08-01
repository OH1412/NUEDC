#!/usr/bin/env python3
"""车辆纵向加速度输入接口和理想前馈换算。

正方向固定为：从管道固定端（零点标定端）指向电机抬升端。
当前只保留接口，不打开串口；后续串口接收器只需实现
``latest_sample()`` 即可接入控制循环。
"""

from dataclasses import dataclass
import math
import time
from typing import Optional, Protocol


@dataclass(frozen=True)
class LongitudinalAccelerationSample:
    acceleration_m_s2: float
    monotonic_s: float


class LongitudinalAccelerationSource(Protocol):
    def latest_sample(
        self,
    ) -> Optional[LongitudinalAccelerationSample]:
        """返回最新样本；尚无数据时返回None。"""

    def close(self) -> None:
        """释放数据源资源。"""


class ReservedAccelerationSource:
    """尚未连接串口时使用的空数据源。"""

    def latest_sample(
        self,
    ) -> Optional[LongitudinalAccelerationSample]:
        return None

    def close(self) -> None:
        return None


class ManualAccelerationSource:
    """纯软件测试数据源，不访问串口或其他硬件。"""

    def __init__(self, acceleration_m_s2: float) -> None:
        value = float(acceleration_m_s2)
        if not math.isfinite(value):
            raise ValueError("测试纵向加速度必须是有限值。")
        self._sample = LongitudinalAccelerationSample(
            acceleration_m_s2=value,
            monotonic_s=time.monotonic(),
        )

    def latest_sample(
        self,
    ) -> Optional[LongitudinalAccelerationSample]:
        return LongitudinalAccelerationSample(
            acceleration_m_s2=self._sample.acceleration_m_s2,
            monotonic_s=time.monotonic(),
        )

    def close(self) -> None:
        return None


def acceleration_feedforward_angle_deg(
    acceleration_m_s2: float,
    gravity_m_s2: float = 9.80665,
) -> float:
    """返回抵消车辆纵向加速度的理想管道倾角。

    正车辆加速度使钢球朝固定端运动，因此需要负倾角使电机端下降。
    使用非线性重力关系 ``theta_ff=-asin(a/g)``。
    """

    acceleration = float(acceleration_m_s2)
    gravity = float(gravity_m_s2)
    if not math.isfinite(acceleration):
        raise ValueError("车辆纵向加速度必须是有限值。")
    if not math.isfinite(gravity) or gravity <= 0.0:
        raise ValueError("重力加速度必须是有限正数。")
    ratio = max(-1.0, min(1.0, acceleration / gravity))
    return -math.degrees(math.asin(ratio))
