#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mission_bt.behavior import BehaviorTree, Status
from mission_bt.builtin_nodes import register_builtin_nodes
from mission_bt.plugin_api import NodeRegistry, RuntimeContext
from mission_bt.protocol import PC_ACK_1M, PC_ACK_TURN
from mission_bt.transport import MockCarTransport
from mission_bt.xml_loader import XmlTreeLoader, load_plugin


def standard_registry():
    registry = NodeRegistry()
    register_builtin_nodes(registry)
    load_plugin("plugins.common_plugin", registry)
    load_plugin("plugins.serial_plugin", registry)
    return registry


class RegistryTest(unittest.TestCase):
    def test_standard_node_ids(self):
        self.assertEqual(
            standard_registry().registered_ids(),
            [
                "CheckBlackboard",
                "Delay",
                "Fallback",
                "Inverter",
                "Log",
                "SendSerialFrame",
                "Sequence",
                "SetBlackboard",
                "WaitSerialFrame",
            ],
        )

    def test_duplicate_registration_is_rejected(self):
        registry = NodeRegistry()
        register_builtin_nodes(registry)
        with self.assertRaisesRegex(ValueError, "already registered"):
            register_builtin_nodes(registry)


class XmlTreeTest(unittest.TestCase):
    def test_simple_ack_xml_end_to_end(self):
        transport = MockCarTransport(car_delay_s=0)
        context = RuntimeContext(transport=transport)
        root = XmlTreeLoader(standard_registry()).load_file(
            ROOT / "config/simple_ack.xml", context
        )
        status = BehaviorTree(root).run(tick_hz=500)
        self.assertIs(status, Status.SUCCESS)
        self.assertEqual(transport.sent, [PC_ACK_1M, PC_ACK_TURN])
        self.assertEqual(context.blackboard["mission_result"], "success")

    def test_subtree_and_fallback(self):
        xml = """
        <root main_tree_to_execute="Main">
          <BehaviorTree ID="Main">
            <Sequence>
              <SubTree ID="Worker"/>
              <CheckBlackboard key="route" equals="B"/>
            </Sequence>
          </BehaviorTree>
          <BehaviorTree ID="Worker">
            <Fallback>
              <CheckBlackboard key="route" equals="A"/>
              <SetBlackboard key="route" value="B"/>
            </Fallback>
          </BehaviorTree>
        </root>
        """
        context = RuntimeContext()
        root = XmlTreeLoader(standard_registry()).load_string(xml, context)
        self.assertIs(BehaviorTree(root).run(tick_hz=500), Status.SUCCESS)
        self.assertEqual(context.blackboard["route"], "B")

    def test_unknown_node_and_attribute_are_rejected(self):
        unknown_node = """
        <BehaviorTree ID="Main"><NotRegistered/></BehaviorTree>
        """
        with self.assertRaisesRegex(ValueError, "not registered"):
            XmlTreeLoader(standard_registry()).load_string(
                unknown_node, RuntimeContext()
            )

        typo = """
        <BehaviorTree ID="Main">
          <Log mesage="typo"/>
        </BehaviorTree>
        """
        with self.assertRaisesRegex(ValueError, "unknown XML attribute"):
            XmlTreeLoader(standard_registry()).load_string(
                typo, RuntimeContext()
            )

    def test_external_plugin_file(self):
        registry = standard_registry()
        load_plugin(str(ROOT / "examples/custom_plugin.py"), registry)
        context = RuntimeContext()
        root = XmlTreeLoader(registry).load_file(
            ROOT / "config/plugin_demo.xml", context
        )
        self.assertIs(BehaviorTree(root).run(tick_hz=500), Status.SUCCESS)


if __name__ == "__main__":
    unittest.main(verbosity=2)
