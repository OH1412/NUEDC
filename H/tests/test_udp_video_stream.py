#!/usr/bin/env python3
"""UDP推流配置与GStreamer命令测试。"""

import sys
import unittest
from pathlib import Path


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from udp_video_stream import (
    StreamConfig,
    StreamError,
    gstreamer_command,
    validate_stream_config,
)


class UdpVideoStreamTests(unittest.TestCase):
    def test_valid_config_and_hardware_encoder_command(self) -> None:
        config = StreamConfig(
            host="192.168.1.20",
            port=5600,
            width=640,
            height=480,
            fps=30,
            bitrate=2_000_000,
        )
        validate_stream_config(config)
        command = gstreamer_command(config)
        self.assertIn("nvv4l2h264enc", command)
        self.assertIn("rtph264pay", command)
        self.assertIn("host=192.168.1.20", command)
        self.assertIn("port=5600", command)

    def test_software_encoder_command(self) -> None:
        config = StreamConfig(host="192.168.50.115")
        command = gstreamer_command(config, "software")
        self.assertIn("x264enc", command)
        self.assertIn("speed-preset=ultrafast", command)
        self.assertNotIn("nvv4l2h264enc", command)

    def test_invalid_host_is_rejected(self) -> None:
        with self.assertRaises(StreamError):
            validate_stream_config(StreamConfig(host="not an ip"))


if __name__ == "__main__":
    unittest.main()
