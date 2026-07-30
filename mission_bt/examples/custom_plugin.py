"""Example external plugin loaded with --plugin /path/custom_plugin.py."""

from typing import List, Mapping

from mission_bt.behavior import Node, Status
from mission_bt.plugin_api import (
    NodeRegistry,
    RuntimeContext,
    reject_unknown_attributes,
    require_attribute,
)


class BlackboardEquals(Node):
    """Return SUCCESS when blackboard[key] equals the configured value."""

    def __init__(
        self,
        name: str,
        context: RuntimeContext,
        key: str,
        expected: str,
    ):
        super().__init__(name)
        self.context = context
        self.key = key
        self.expected = expected

    def tick(self) -> Status:
        actual = self.context.blackboard.get(self.key)
        self.status = (
            Status.SUCCESS if str(actual) == self.expected else Status.FAILURE
        )
        return self.status


def _builder(
    name: str,
    attributes: Mapping[str, str],
    children: List[Node],
    context: RuntimeContext,
) -> Node:
    if children:
        raise ValueError("BlackboardEquals cannot have children")
    reject_unknown_attributes(attributes, ("key", "value"))
    return BlackboardEquals(
        name,
        context,
        require_attribute(attributes, "key"),
        require_attribute(attributes, "value"),
    )


def register_plugin(registry: NodeRegistry) -> None:
    """Required entry point for every plugin module."""

    registry.register("BlackboardEquals", _builder)
