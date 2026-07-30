"""Serial protocol behavior-node plugin."""

from __future__ import annotations

import time
from typing import List, Mapping

from mission_bt.behavior import Node, Status
from mission_bt.plugin_api import (
    NodeRegistry,
    RuntimeContext,
    float_attribute,
    reject_unknown_attributes,
    require_attribute,
)
from mission_bt.protocol import hex_frame, validate_frame


def _parse_frame(text: str) -> bytes:
    normalized = text.replace(",", " ").replace("-", " ")
    tokens = normalized.split()
    if not tokens:
        raise ValueError("frame cannot be empty")
    try:
        frame = bytes(int(token, 16) for token in tokens)
    except ValueError as exc:
        raise ValueError(
            f"frame must be hexadecimal bytes, got {text!r}"
        ) from exc
    validate_frame(frame)
    return frame


class SendSerialFrame(Node):
    def __init__(self, name: str, context: RuntimeContext, frame: bytes):
        super().__init__(name)
        if context.transport is None:
            raise ValueError("SendSerialFrame requires a transport")
        self.transport = context.transport
        self.frame = frame

    def tick(self) -> Status:
        if self.status in (Status.SUCCESS, Status.FAILURE):
            return self.status
        print(f"\n▶ {self.name}")
        try:
            print(f"小电脑 TX: {hex_frame(self.frame)}")
            self.transport.send(self.frame)
        except Exception as exc:
            print(f"✗ {self.name}: {exc}")
            self.status = Status.FAILURE
        else:
            print(f"✓ {self.name}")
            self.status = Status.SUCCESS
        return self.status


class WaitSerialFrame(Node):
    def __init__(
        self,
        name: str,
        context: RuntimeContext,
        expected: bytes,
        timeout_s: float,
    ):
        super().__init__(name)
        if context.transport is None:
            raise ValueError("WaitSerialFrame requires a transport")
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.transport = context.transport
        self.expected = expected
        self.timeout_s = timeout_s
        self.deadline = None

    def tick(self) -> Status:
        if self.status in (Status.SUCCESS, Status.FAILURE):
            return self.status
        if self.deadline is None:
            print(f"\n▶ {self.name}")
            print(
                f"等待小车 RX: {hex_frame(self.expected)} "
                f"(超时 {self.timeout_s:.1f}s)"
            )
            self.deadline = time.monotonic() + self.timeout_s
            self.status = Status.RUNNING
        for frame in self.transport.receive(0.05):
            print(f"小电脑 RX: {hex_frame(frame)}")
            if frame == self.expected:
                print(f"✓ {self.name}")
                self.status = Status.SUCCESS
                return self.status
            print("  忽略：不是当前 XML 节点期望的帧")
        if time.monotonic() >= self.deadline:
            print(f"✗ {self.name}: 等待串口帧超时")
            self.status = Status.FAILURE
        return self.status

    def reset(self) -> None:
        super().reset()
        self.deadline = None


def _send_builder(
    name: str,
    attributes: Mapping[str, str],
    children: List[Node],
    context: RuntimeContext,
) -> Node:
    if children:
        raise ValueError("SendSerialFrame is an action and cannot have children")
    reject_unknown_attributes(attributes, ("frame",))
    return SendSerialFrame(
        name,
        context,
        _parse_frame(require_attribute(attributes, "frame")),
    )


def _wait_builder(
    name: str,
    attributes: Mapping[str, str],
    children: List[Node],
    context: RuntimeContext,
) -> Node:
    if children:
        raise ValueError("WaitSerialFrame is an action and cannot have children")
    reject_unknown_attributes(attributes, ("frame", "timeout_s"))
    return WaitSerialFrame(
        name,
        context,
        _parse_frame(require_attribute(attributes, "frame")),
        float_attribute(attributes, "timeout_s", 30.0),
    )


def register_plugin(registry: NodeRegistry) -> None:
    registry.register("SendSerialFrame", _send_builder)
    registry.register("WaitSerialFrame", _wait_builder)
