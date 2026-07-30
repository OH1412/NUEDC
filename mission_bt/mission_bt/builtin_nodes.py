"""Registration of generic control-flow nodes."""

from __future__ import annotations

from typing import List, Mapping

from .behavior import Fallback, Inverter, Node, Sequence
from .plugin_api import (
    NodeRegistry,
    RuntimeContext,
    reject_unknown_attributes,
)


def _require_children(node_id: str, children: List[Node], count=None) -> None:
    if count is not None and len(children) != count:
        raise ValueError(
            f"{node_id} requires exactly {count} child node(s), "
            f"got {len(children)}"
        )
    if count is None and not children:
        raise ValueError(f"{node_id} requires at least one child")


def _sequence(
    name: str,
    attributes: Mapping[str, str],
    children: List[Node],
    context: RuntimeContext,
) -> Node:
    reject_unknown_attributes(attributes, ())
    _require_children("Sequence", children)
    return Sequence(name, children)


def _fallback(
    name: str,
    attributes: Mapping[str, str],
    children: List[Node],
    context: RuntimeContext,
) -> Node:
    reject_unknown_attributes(attributes, ())
    _require_children("Fallback", children)
    return Fallback(name, children)


def _inverter(
    name: str,
    attributes: Mapping[str, str],
    children: List[Node],
    context: RuntimeContext,
) -> Node:
    reject_unknown_attributes(attributes, ())
    _require_children("Inverter", children, count=1)
    return Inverter(name, children[0])


def register_builtin_nodes(registry: NodeRegistry) -> None:
    registry.register("Sequence", _sequence)
    registry.register("Fallback", _fallback)
    registry.register("Inverter", _inverter)
