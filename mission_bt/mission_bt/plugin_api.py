"""Plugin registry and runtime context for XML behavior trees."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Mapping, MutableMapping, Optional

from .behavior import Node
from .transport import Transport

NodeBuilder = Callable[
    [str, Mapping[str, str], List[Node], "RuntimeContext"],
    Node,
]


@dataclass
class RuntimeContext:
    """Resources shared by behavior nodes without coupling them to the core."""

    transport: Optional[Transport] = None
    blackboard: MutableMapping[str, Any] = field(default_factory=dict)


class NodeRegistry:
    """Map XML node IDs to builders supplied by built-ins or plugins."""

    def __init__(self):
        self._builders: Dict[str, NodeBuilder] = {}

    def register(
        self,
        node_id: str,
        builder: NodeBuilder,
        *,
        replace: bool = False,
    ) -> None:
        if not node_id or not node_id.isidentifier():
            raise ValueError(f"invalid node ID: {node_id!r}")
        if node_id in self._builders and not replace:
            raise ValueError(f"node ID already registered: {node_id}")
        self._builders[node_id] = builder

    def create(
        self,
        node_id: str,
        name: str,
        attributes: Mapping[str, str],
        children: List[Node],
        context: RuntimeContext,
    ) -> Node:
        try:
            builder = self._builders[node_id]
        except KeyError as exc:
            available = ", ".join(self.registered_ids())
            raise ValueError(
                f"XML node {node_id!r} is not registered; available: {available}"
            ) from exc
        return builder(name, attributes, children, context)

    def registered_ids(self) -> List[str]:
        return sorted(self._builders)


def require_attribute(attributes: Mapping[str, str], key: str) -> str:
    try:
        value = attributes[key].strip()
    except KeyError as exc:
        raise ValueError(f"missing required XML attribute: {key}") from exc
    if not value:
        raise ValueError(f"XML attribute {key!r} cannot be empty")
    return value


def reject_unknown_attributes(
    attributes: Mapping[str, str],
    allowed,
) -> None:
    unknown = sorted(set(attributes) - set(allowed))
    if unknown:
        raise ValueError(
            "unknown XML attribute(s): " + ", ".join(unknown)
        )


def float_attribute(
    attributes: Mapping[str, str],
    key: str,
    default: Optional[float] = None,
) -> float:
    if key not in attributes:
        if default is None:
            raise ValueError(f"missing required XML attribute: {key}")
        return default
    try:
        return float(attributes[key])
    except ValueError as exc:
        raise ValueError(
            f"XML attribute {key!r} must be a number, got {attributes[key]!r}"
        ) from exc
