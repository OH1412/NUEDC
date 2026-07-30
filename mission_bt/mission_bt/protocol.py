"""Eight-byte UART protocol constants and a streaming frame parser."""

from __future__ import annotations

from typing import Iterable, List

FRAME_LENGTH = 8
CAR_TO_PC_HEADER = 0x76
CAR_TO_PC_FOOTER = 0x67
PC_TO_CAR_HEADER = 0x92
PC_TO_CAR_FOOTER = 0x29

# 小车 MCU -> 小电脑：当前运动阶段已经完成。
CAR_MOTION_COMPLETE = bytes((0x76, 0x01, 0, 0, 0, 0, 0, 0x67))

# 小电脑 -> 小车 MCU：允许进入下一运动阶段。
PC_ACK_1M = bytes((0x92, 0x10, 0, 0, 0, 0, 0, 0x29))
PC_ACK_TURN = bytes((0x92, 0x11, 0, 0, 0, 0, 0, 0x29))


def hex_frame(frame: bytes) -> str:
    return " ".join(f"0x{byte:02X}" for byte in frame)


def validate_frame(frame: bytes) -> None:
    if len(frame) != FRAME_LENGTH:
        raise ValueError(f"frame must contain {FRAME_LENGTH} bytes")
    valid_footer = {
        CAR_TO_PC_HEADER: CAR_TO_PC_FOOTER,
        PC_TO_CAR_HEADER: PC_TO_CAR_FOOTER,
    }.get(frame[0])
    if valid_footer is None or frame[-1] != valid_footer:
        raise ValueError(f"invalid frame: {hex_frame(frame)}")


class FrameParser:
    """Recover fixed-size frames from partial reads and line noise."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: Iterable[int]) -> List[bytes]:
        self.buffer.extend(data)
        frames: List[bytes] = []
        while self.buffer:
            while self.buffer and self.buffer[0] not in (
                CAR_TO_PC_HEADER,
                PC_TO_CAR_HEADER,
            ):
                del self.buffer[0]
            if len(self.buffer) < FRAME_LENGTH:
                break
            candidate = bytes(self.buffer[:FRAME_LENGTH])
            expected_footer = (
                CAR_TO_PC_FOOTER
                if candidate[0] == CAR_TO_PC_HEADER
                else PC_TO_CAR_FOOTER
            )
            if candidate[-1] == expected_footer:
                frames.append(candidate)
                del self.buffer[:FRAME_LENGTH]
            else:
                del self.buffer[0]
        return frames
