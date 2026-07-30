"""Serial and simulated communication backends."""

from __future__ import annotations

import os
import select
import termios
import time
from abc import ABC, abstractmethod
from collections import deque
from typing import Deque, List

from .protocol import (
    CAR_MOTION_COMPLETE,
    PC_ACK_1M,
    PC_ACK_TURN,
    FrameParser,
    validate_frame,
)


class Transport(ABC):
    @abstractmethod
    def send(self, frame: bytes) -> None:
        raise NotImplementedError

    @abstractmethod
    def receive(self, timeout_s: float) -> List[bytes]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class PosixSerialTransport(Transport):
    """Dependency-free Linux UART backend using termios."""

    BAUD_RATES = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }

    def __init__(self, port: str, baud: int = 9600):
        if baud not in self.BAUD_RATES:
            raise ValueError(f"unsupported baud rate: {baud}")
        self.port = port
        self.parser = FrameParser()
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = self.BAUD_RATES[baud]
        attrs[5] = self.BAUD_RATES[baud]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def send(self, frame: bytes) -> None:
        validate_frame(frame)
        view = memoryview(frame)
        while view:
            _, writable, _ = select.select([], [self.fd], [], 1.0)
            if not writable:
                raise TimeoutError(f"serial write timeout: {self.port}")
            written = os.write(self.fd, view)
            view = view[written:]
        termios.tcdrain(self.fd)

    def receive(self, timeout_s: float) -> List[bytes]:
        readable, _, _ = select.select([self.fd], [], [], max(timeout_s, 0.0))
        if not readable:
            return []
        try:
            chunk = os.read(self.fd, 4096)
        except BlockingIOError:
            return []
        return self.parser.feed(chunk)

    def close(self) -> None:
        if getattr(self, "fd", -1) >= 0:
            os.close(self.fd)
            self.fd = -1


class MockCarTransport(Transport):
    """Simulate the MCU completing 1 m, then completing the 90-degree turn."""

    def __init__(self, car_delay_s: float = 0.02):
        self.car_delay_s = car_delay_s
        self.sent: List[bytes] = []
        self.pending: Deque[tuple[float, bytes]] = deque()
        # The MCU starts the first forward movement by itself.
        self.pending.append(
            (time.monotonic() + self.car_delay_s, CAR_MOTION_COMPLETE)
        )

    def send(self, frame: bytes) -> None:
        validate_frame(frame)
        self.sent.append(frame)
        # After 0x10 the MCU turns left, then reports completion again.
        if frame == PC_ACK_1M and len(self.sent) == 1:
            self.pending.append(
                (time.monotonic() + self.car_delay_s, CAR_MOTION_COMPLETE)
            )
        # After 0x11 the MCU moves forward 0.5 m. No third completion
        # frame was specified, so the simulator has nothing more to send.
        elif frame == PC_ACK_TURN and len(self.sent) == 2:
            return

    def receive(self, timeout_s: float) -> List[bytes]:
        deadline = time.monotonic() + max(timeout_s, 0.0)
        while True:
            if self.pending and self.pending[0][0] <= time.monotonic():
                return [self.pending.popleft()[1]]
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return []
            time.sleep(min(remaining, 0.005))


class SilentTransport(Transport):
    """Test backend that never acknowledges."""

    def __init__(self):
        self.sent: List[bytes] = []

    def send(self, frame: bytes) -> None:
        validate_frame(frame)
        self.sent.append(frame)

    def receive(self, timeout_s: float) -> List[bytes]:
        time.sleep(max(timeout_s, 0.0))
        return []
