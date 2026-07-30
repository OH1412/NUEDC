#!/usr/bin/env python3
"""Run an XML-defined NUEDC behavior tree."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from mission_bt.behavior import BehaviorTree, Status
from mission_bt.builtin_nodes import register_builtin_nodes
from mission_bt.plugin_api import NodeRegistry, RuntimeContext
from mission_bt.transport import MockCarTransport, PosixSerialTransport
from mission_bt.xml_loader import XmlTreeLoader, load_plugin

DEFAULT_TREE = SCRIPT_DIR / "config/simple_ack.xml"
DEFAULT_PLUGINS = ("plugins.common_plugin", "plugins.serial_plugin")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="NUEDC XML 行为树运行器")
    parser.add_argument(
        "--tree",
        type=Path,
        default=DEFAULT_TREE,
        help=f"行为树 XML（默认: {DEFAULT_TREE}）",
    )
    parser.add_argument(
        "--plugin",
        action="append",
        default=[],
        help="附加 Python 插件模块名或 .py 路径，可重复指定",
    )
    parser.add_argument(
        "--transport",
        choices=("mock", "serial"),
        default="mock",
        help="小车 MCU 通信后端",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--mock-car-delay", type=float, default=0.02)
    parser.add_argument("--tick-hz", type=float, default=20.0)
    parser.add_argument(
        "--set",
        dest="blackboard_items",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="初始化黑板变量，可重复指定",
    )
    parser.add_argument(
        "--list-nodes",
        action="store_true",
        help="列出注册节点后退出",
    )
    return parser.parse_args()


def parse_blackboard(items):
    blackboard = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"--set requires KEY=VALUE, got {item!r}")
        key, value = item.split("=", 1)
        key = key.strip()
        if not key:
            raise ValueError("blackboard key cannot be empty")
        blackboard[key] = value
    return blackboard


def make_transport(args):
    if args.transport == "mock":
        return MockCarTransport(args.mock_car_delay)
    return PosixSerialTransport(args.port, args.baud)


def build_registry(plugin_references) -> NodeRegistry:
    registry = NodeRegistry()
    register_builtin_nodes(registry)
    for reference in (*DEFAULT_PLUGINS, *plugin_references):
        load_plugin(reference, registry)
    return registry


def main() -> int:
    args = parse_args()
    transport = None
    tree = None
    try:
        registry = build_registry(args.plugin)
        if args.list_nodes:
            print("\n".join(registry.registered_ids()))
            return 0

        transport = make_transport(args)
        context = RuntimeContext(
            transport=transport,
            blackboard=parse_blackboard(args.blackboard_items),
        )
        root = XmlTreeLoader(registry).load_file(
            args.tree.expanduser().resolve(),
            context,
        )
        tree = BehaviorTree(root)

        print("NUEDC XML 行为树启动")
        print(f"tree={args.tree.expanduser().resolve()}")
        print(f"transport={args.transport}")
        print(f"nodes={', '.join(registry.registered_ids())}")
        status = tree.run(args.tick_hz)
        if status is Status.SUCCESS:
            print("\n✅ XML 行为树执行成功")
            return 0
        print("\n❌ XML 行为树执行失败")
        return 1
    except KeyboardInterrupt:
        if tree is not None:
            tree.halt()
        print("\n用户中断，行为树已停止")
        return 130
    except Exception as exc:
        print(f"\n❌ 启动/运行错误: {exc}", file=sys.stderr)
        return 2
    finally:
        if transport is not None:
            transport.close()


if __name__ == "__main__":
    raise SystemExit(main())
