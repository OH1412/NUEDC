import sys
import json
import math
import os
import tempfile
import threading
import unittest
from pathlib import Path


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from competition_runtime import (  # noqa: E402
    BALL_RECOGNITION_REQUEST,
    BALL_RECOGNIZED_RESPONSE,
    MODE2_BEGIN,
    MODE2_END,
    MODE3_BEGIN,
    MODE4_END,
    MODE34_EQUILIBRIUM_BIAS_DEG,
    MODE2_CONTROL_PROFILE,
    MODE34_CONTROL_PROFILE,
    MODE5_CONTROL_PROFILE,
    MODE5_BEGIN,
    MODE5_END,
    ControlSession,
    RequestFrameParser,
    centered_target_from_stable_points,
)
from ball_tracker_source import FifoBallTrackerSource  # noqa: E402
from mode5_equilibrium import (  # noqa: E402
    DEFAULT_MODE5_EQUILIBRIUM_FILE,
    load_mode5_equilibrium_points,
    nearest_mode5_equilibrium,
)


class RequestFrameParserTests(unittest.TestCase):
    def test_fragmented_frame(self) -> None:
        parser = RequestFrameParser()
        self.assertEqual(parser.feed(MODE2_BEGIN[:3]), [])
        self.assertEqual(parser.feed(MODE2_BEGIN[3:]), [MODE2_BEGIN])

    def test_multiple_frames_and_noise(self) -> None:
        parser = RequestFrameParser()
        self.assertEqual(
            parser.feed(b"\x00\x11" + MODE3_BEGIN + MODE4_END),
            [MODE3_BEGIN, MODE4_END],
        )

    def test_mode5_protocol_frames(self) -> None:
        parser = RequestFrameParser()
        self.assertEqual(
            parser.feed(BALL_RECOGNITION_REQUEST + MODE5_BEGIN + MODE5_END),
            [BALL_RECOGNITION_REQUEST, MODE5_BEGIN, MODE5_END],
        )
        self.assertEqual(
            BALL_RECOGNIZED_RESPONSE,
            bytes((0x92, 0x6F, 0x6B, 0, 0, 0, 0, 0x29)),
        )

    def test_mode5_stable_center_converts_to_centered_target(self) -> None:
        config = {
            "zero_point_base_m": [0.0, 0.0, 0.0],
            "pipe_length_m": 0.25,
            "target_coordinate_center_m": 0.1234,
            "zero_calibration_ball_radius_m": 0.0016,
        }
        points = [
            [0.1734 + ((index % 5) - 2) * 0.00005, 0.0, 0.0]
            for index in range(50)
        ]
        target_cm, rms_mm, inliers = centered_target_from_stable_points(
            points, config, max_rms_spread_mm=3.0
        )
        self.assertAlmostEqual(target_cm, 5.0, places=3)
        self.assertLess(rms_mm, 0.1)
        self.assertEqual(inliers, 50)

    def test_mode5_control_session_uses_recorded_target(self) -> None:
        session = ControlSession(
            "mode5",
            "/tmp/test-mode5-tracker.fifo",
            lambda _frame: None,
            target_position_cm=-4.375,
            equilibrium_bias_deg=0.125,
        )
        command = session._command("/dev/pts/123")
        index = command.index("--target-cm")
        self.assertEqual(command[index + 1], "-4.375")
        profile_index = command.index("--control-profile")
        self.assertEqual(command[profile_index + 1], MODE5_CONTROL_PROFILE)
        bias_index = command.index("--equilibrium-angle-bias-deg")
        self.assertEqual(float(command[bias_index + 1]), 0.125)
        self.assertIn("--no-position-local-zero-prior", command)

    def test_mode5_nearest_height_is_converted_to_angle(self) -> None:
        angle_deg, selected_cm, height_mm = (
            nearest_mode5_equilibrium(10.62)
        )
        self.assertEqual(selected_cm, 10.5)
        self.assertAlmostEqual(
            height_mm,
            250.0 * math.tan(math.radians(angle_deg)),
            places=8,
        )

    def test_mode5_equilibrium_table_covers_half_centimeter_grid(self) -> None:
        self.assertTrue(DEFAULT_MODE5_EQUILIBRIUM_FILE.is_file())
        points = load_mode5_equilibrium_points()
        self.assertEqual(len(points), 49)
        self.assertEqual([point["position_cm"] for point in points], [
            12.0 - 0.5 * index for index in range(49)
        ])
        for point in points:
            self.assertAlmostEqual(
                point["equivalent_height_mm"],
                250.0
                * math.tan(
                    math.radians(point["equilibrium_angle_bias_deg"])
                ),
                places=8,
            )

    def test_mode3_and_mode4_use_fixed_equilibrium_bias(self) -> None:
        for mode in ("mode3", "mode4"):
            session = ControlSession(
                mode,
                "/tmp/test-mode34-tracker.fifo",
                lambda _frame: None,
            )
            command = session._command("/dev/pts/123")
            bias_index = command.index("--equilibrium-angle-bias-deg")
            self.assertEqual(
                float(command[bias_index + 1]),
                MODE34_EQUILIBRIUM_BIAS_DEG,
            )
            self.assertIn("--no-position-local-zero-prior", command)
            profile_index = command.index("--control-profile")
            self.assertEqual(
                command[profile_index + 1], MODE34_CONTROL_PROFILE
            )

    def test_mode2_uses_position_profile(self) -> None:
        session = ControlSession(
            "mode2",
            "/tmp/test-mode2-tracker.fifo",
            lambda _frame: None,
        )
        command = session._command("/dev/pts/123")
        profile_index = command.index("--control-profile")
        self.assertEqual(command[profile_index + 1], MODE2_CONTROL_PROFILE)

    def test_bad_tail_resynchronizes(self) -> None:
        parser = RequestFrameParser()
        bad = bytearray(MODE2_END)
        bad[-1] = 0
        self.assertEqual(parser.feed(bytes(bad) + MODE2_END), [MODE2_END])

    def test_fifo_tracker_shares_record_without_disk_payload(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "tracker.fifo")
            source = FifoBallTrackerSource(path)
            source.start()
            received = []

            def consume() -> None:
                received.append(next(source.records()))

            thread = threading.Thread(target=consume)
            thread.start()
            writer = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
            payload = {"frame": 7, "valid": True}
            os.write(writer, (json.dumps(payload) + "\n").encode("utf-8"))
            os.close(writer)
            thread.join(timeout=1.0)
            source.close()
            self.assertFalse(thread.is_alive())
            self.assertEqual(received, [payload])


if __name__ == "__main__":
    unittest.main()
