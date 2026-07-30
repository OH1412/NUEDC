#!/usr/bin/env python3

import os
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mission_bt.behavior import Status
from mission_bt.protocol import (
    CAR_MOTION_COMPLETE,
    PC_ACK_1M,
    PC_ACK_TURN,
    FrameParser,
)
from mission_bt.transport import (
    PosixSerialTransport,
)


class ProtocolTest(unittest.TestCase):
    def test_exact_directional_frames(self):
        self.assertEqual(
            CAR_MOTION_COMPLETE.hex(" "), "76 01 00 00 00 00 00 67"
        )
        self.assertEqual(PC_ACK_1M.hex(" "), "92 10 00 00 00 00 00 29")
        self.assertEqual(PC_ACK_TURN.hex(" "), "92 11 00 00 00 00 00 29")

    def test_parser_recovers_partial_car_frame_and_noise(self):
        parser = FrameParser()
        self.assertEqual(parser.feed(b"\x00\xff\x76\x01"), [])
        self.assertEqual(
            parser.feed(b"\x00\x00\x00\x00\x00\x67"),
            [CAR_MOTION_COMPLETE],
        )

    def test_posix_serial_pc_tx_and_car_rx(self):
        master, slave = os.openpty()
        slave_path = os.ttyname(slave)
        os.close(slave)
        transport = PosixSerialTransport(slave_path, 9600)
        try:
            # Small computer sends 0x92.
            transport.send(PC_ACK_1M)
            self.assertEqual(os.read(master, 64), PC_ACK_1M)

            # MCU sends fragmented 0x76, preceded by line noise.
            os.write(master, b"\x00\xff" + CAR_MOTION_COMPLETE[:3])
            self.assertEqual(transport.receive(0.05), [])
            os.write(master, CAR_MOTION_COMPLETE[3:])
            self.assertEqual(
                transport.receive(0.1), [CAR_MOTION_COMPLETE]
            )
        finally:
            transport.close()
            os.close(master)

if __name__ == "__main__":
    unittest.main(verbosity=2)
