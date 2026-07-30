"""Minimal tick-based behavior tree primitives."""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum, auto
import time
from typing import Iterable, List


class Status(Enum):
    IDLE = auto()
    RUNNING = auto()
    SUCCESS = auto()
    FAILURE = auto()


class Node(ABC):
    def __init__(self, name: str):
        self.name = name
        self.status = Status.IDLE

    @abstractmethod
    def tick(self) -> Status:
        raise NotImplementedError

    def reset(self) -> None:
        self.status = Status.IDLE

    def halt(self) -> None:
        """Stop a running node and return it to IDLE."""
        self.reset()


class Sequence(Node):
    """Run children in order; stop on RUNNING or FAILURE."""

    def __init__(self, name: str, children: Iterable[Node]):
        super().__init__(name)
        self.children: List[Node] = list(children)
        self.index = 0

    def tick(self) -> Status:
        self.status = Status.RUNNING
        while self.index < len(self.children):
            child = self.children[self.index]
            child_status = child.tick()
            if child_status is Status.SUCCESS:
                self.index += 1
                continue
            if child_status is Status.FAILURE:
                self.status = Status.FAILURE
                return self.status
            return self.status
        self.status = Status.SUCCESS
        return self.status

    def reset(self) -> None:
        super().reset()
        self.index = 0
        for child in self.children:
            child.reset()


class Fallback(Node):
    """Try children in order until one succeeds or remains running."""

    def __init__(self, name: str, children: Iterable[Node]):
        super().__init__(name)
        self.children: List[Node] = list(children)
        self.index = 0

    def tick(self) -> Status:
        self.status = Status.RUNNING
        while self.index < len(self.children):
            child_status = self.children[self.index].tick()
            if child_status is Status.FAILURE:
                self.index += 1
                continue
            if child_status is Status.SUCCESS:
                self.status = Status.SUCCESS
                return self.status
            return self.status
        self.status = Status.FAILURE
        return self.status

    def reset(self) -> None:
        super().reset()
        self.index = 0
        for child in self.children:
            child.reset()


class Inverter(Node):
    """Decorator that swaps SUCCESS and FAILURE."""

    def __init__(self, name: str, child: Node):
        super().__init__(name)
        self.child = child

    def tick(self) -> Status:
        child_status = self.child.tick()
        if child_status is Status.SUCCESS:
            self.status = Status.FAILURE
        elif child_status is Status.FAILURE:
            self.status = Status.SUCCESS
        else:
            self.status = child_status
        return self.status

    def reset(self) -> None:
        super().reset()
        self.child.reset()


class BehaviorTree:
    """Own and tick one root node."""

    def __init__(self, root: Node):
        self.root = root

    def tick_once(self) -> Status:
        return self.root.tick()

    def run(self, tick_hz: float = 20.0) -> Status:
        if tick_hz <= 0:
            raise ValueError("tick_hz must be positive")
        period = 1.0 / tick_hz
        while True:
            status = self.tick_once()
            if status is not Status.RUNNING:
                return status
            time.sleep(period)

    def reset(self) -> None:
        self.root.reset()

    def halt(self) -> None:
        self.root.halt()
