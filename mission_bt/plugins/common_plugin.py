"""Generic utility behavior-node plugin."""

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


class Log(Node):
    def __init__(self, name: str, message: str, context: RuntimeContext):
        super().__init__(name)
        self.message = message
        self.context = context

    def tick(self) -> Status:
        if self.status is Status.IDLE:
            try:
                message = self.message.format_map(self.context.blackboard)
            except KeyError as exc:
                print(f"✗ {self.name}: blackboard key not found: {exc}")
                self.status = Status.FAILURE
            else:
                print(f"[LOG] {message}")
                self.status = Status.SUCCESS
        return self.status


class Delay(Node):
    def __init__(self, name: str, seconds: float):
        super().__init__(name)
        if seconds < 0:
            raise ValueError("seconds cannot be negative")
        self.seconds = seconds
        self.deadline = None

    def tick(self) -> Status:
        if self.status in (Status.SUCCESS, Status.FAILURE):
            return self.status
        if self.deadline is None:
            self.deadline = time.monotonic() + self.seconds
            self.status = Status.RUNNING
        if time.monotonic() >= self.deadline:
            self.status = Status.SUCCESS
        return self.status

    def reset(self) -> None:
        super().reset()
        self.deadline = None


class SetBlackboard(Node):
    def __init__(
        self,
        name: str,
        key: str,
        value: str,
        context: RuntimeContext,
    ):
        super().__init__(name)
        self.key = key
        self.value = value
        self.context = context

    def tick(self) -> Status:
        self.context.blackboard[self.key] = self.value
        self.status = Status.SUCCESS
        return self.status


class CheckBlackboard(Node):
    def __init__(
        self,
        name: str,
        key: str,
        expected: str,
        context: RuntimeContext,
    ):
        super().__init__(name)
        self.key = key
        self.expected = expected
        self.context = context

    def tick(self) -> Status:
        actual = self.context.blackboard.get(self.key)
        self.status = (
            Status.SUCCESS if str(actual) == self.expected else Status.FAILURE
        )
        return self.status


def _leaf(children: List[Node], node_id: str) -> None:
    if children:
        raise ValueError(f"{node_id} cannot have children")


def _log_builder(name, attributes, children, context):
    _leaf(children, "Log")
    reject_unknown_attributes(attributes, ("message",))
    return Log(name, require_attribute(attributes, "message"), context)


def _delay_builder(name, attributes, children, context):
    _leaf(children, "Delay")
    reject_unknown_attributes(attributes, ("seconds",))
    return Delay(name, float_attribute(attributes, "seconds"))


def _set_builder(name, attributes, children, context):
    _leaf(children, "SetBlackboard")
    reject_unknown_attributes(attributes, ("key", "value"))
    return SetBlackboard(
        name,
        require_attribute(attributes, "key"),
        require_attribute(attributes, "value"),
        context,
    )


def _check_builder(name, attributes, children, context):
    _leaf(children, "CheckBlackboard")
    reject_unknown_attributes(attributes, ("key", "equals"))
    return CheckBlackboard(
        name,
        require_attribute(attributes, "key"),
        require_attribute(attributes, "equals"),
        context,
    )


def register_plugin(registry: NodeRegistry) -> None:
    registry.register("Log", _log_builder)
    registry.register("Delay", _delay_builder)
    registry.register("SetBlackboard", _set_builder)
    registry.register("CheckBlackboard", _check_builder)
