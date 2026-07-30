"""BehaviorTree.CPP-inspired XML loader."""

from __future__ import annotations

import importlib
import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import ModuleType
from typing import Dict, List, Set

from .behavior import Node
from .plugin_api import NodeRegistry, RuntimeContext


def load_plugin(reference: str, registry: NodeRegistry) -> ModuleType:
    """Load a Python module/path exposing register_plugin(registry)."""

    path = Path(reference).expanduser()
    if path.is_file():
        module_name = f"mission_bt_external_{abs(hash(path.resolve()))}"
        spec = importlib.util.spec_from_file_location(module_name, str(path))
        if spec is None or spec.loader is None:
            raise ImportError(f"cannot load plugin file: {path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
    else:
        module = importlib.import_module(reference)
    register = getattr(module, "register_plugin", None)
    if not callable(register):
        raise ValueError(
            f"plugin {reference!r} must define register_plugin(registry)"
        )
    register(registry)
    return module


class XmlTreeLoader:
    def __init__(self, registry: NodeRegistry):
        self.registry = registry
        self.tree_definitions: Dict[str, ET.Element] = {}

    def load_file(self, path: Path, context: RuntimeContext) -> Node:
        try:
            root = ET.parse(str(path)).getroot()
        except (ET.ParseError, OSError) as exc:
            raise ValueError(f"cannot read behavior tree XML {path}: {exc}") from exc
        return self._load_root(root, context)

    def load_string(self, xml_text: str, context: RuntimeContext) -> Node:
        try:
            root = ET.fromstring(xml_text)
        except ET.ParseError as exc:
            raise ValueError(f"invalid behavior tree XML: {exc}") from exc
        return self._load_root(root, context)

    def _load_root(self, root: ET.Element, context: RuntimeContext) -> Node:
        if root.tag == "BehaviorTree":
            tree_id = root.attrib.get("ID", "MainTree")
            self.tree_definitions = {tree_id: root}
            return self._build_tree(tree_id, context, set())
        if root.tag != "root":
            raise ValueError("XML top-level element must be <root>")

        definitions = [
            element for element in root if element.tag == "BehaviorTree"
        ]
        self.tree_definitions = {}
        for definition in definitions:
            tree_id = definition.attrib.get("ID", "").strip()
            if not tree_id:
                raise ValueError("<BehaviorTree> requires a non-empty ID")
            if tree_id in self.tree_definitions:
                raise ValueError(f"duplicate BehaviorTree ID: {tree_id}")
            self.tree_definitions[tree_id] = definition
        if not self.tree_definitions:
            raise ValueError("<root> contains no <BehaviorTree>")

        main_id = root.attrib.get("main_tree_to_execute", "").strip()
        if not main_id:
            if len(self.tree_definitions) != 1:
                raise ValueError(
                    "main_tree_to_execute is required when multiple trees exist"
                )
            main_id = next(iter(self.tree_definitions))
        return self._build_tree(main_id, context, set())

    def _build_tree(
        self,
        tree_id: str,
        context: RuntimeContext,
        stack: Set[str],
    ) -> Node:
        if tree_id in stack:
            raise ValueError(f"recursive SubTree reference detected: {tree_id}")
        try:
            definition = self.tree_definitions[tree_id]
        except KeyError as exc:
            raise ValueError(f"unknown BehaviorTree ID: {tree_id}") from exc
        children = list(definition)
        if len(children) != 1:
            raise ValueError(
                f"BehaviorTree {tree_id!r} must contain exactly one root node"
            )
        return self._build_element(children[0], context, stack | {tree_id})

    def _build_element(
        self,
        element: ET.Element,
        context: RuntimeContext,
        stack: Set[str],
    ) -> Node:
        if element.tag == "SubTree":
            subtree_id = element.attrib.get("ID", "").strip()
            if not subtree_id:
                raise ValueError("<SubTree> requires an ID")
            if list(element):
                raise ValueError("<SubTree> cannot contain child nodes")
            return self._build_tree(subtree_id, context, stack)

        child_nodes = [
            self._build_element(child, context, stack) for child in element
        ]
        attributes = dict(element.attrib)
        name = attributes.pop("name", element.tag)
        try:
            return self.registry.create(
                element.tag,
                name,
                attributes,
                child_nodes,
                context,
            )
        except ValueError as exc:
            raise ValueError(
                f"error building <{element.tag} name={name!r}>: {exc}"
            ) from exc
