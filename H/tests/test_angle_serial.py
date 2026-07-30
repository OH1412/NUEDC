#!/usr/bin/env python3
"""不接真实串口验证倾角编码和周期发送。"""

import math
import sys
import threading
import time
import unittest
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from angle_serial import (  # noqa: E402
    AngleEncodingError,
    MOTOR_ENABLE_FRAME,
    MOTOR_INITIAL_ZERO_FRAME,
    PeriodicAngleSender,
    encode_angle,
    open_serial_port,
    validate_rate_hz,
)


class FakeSerial:
    def __init__(self, write_result: Optional[int] = None) -> None:
        self.write_result = write_result
        self.writes = []  # type: List[bytes]
        self.flush_count = 0
        self.closed = False
        self._condition = threading.Condition()

    def write(self, data: bytes) -> int:
        with self._condition:
            self.writes.append(bytes(data))
            self._condition.notify_all()
        if self.write_result is not None:
            return self.write_result
        return len(data)

    def flush(self) -> None:
        self.flush_count += 1

    def close(self) -> None:
        self.closed = True

    def wait_until(
        self,
        predicate: Callable[[List[bytes]], bool],
        timeout: float = 1.0,
    ) -> bool:
        deadline = time.monotonic() + timeout
        with self._condition:
            while not predicate(self.writes):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
            return True


class FakeSerialFactory:
    def __init__(self) -> None:
        self.calls = []  # type: List[Dict[str, Any]]
        self.serial_port = FakeSerial()

    def __call__(self, **kwargs: Any) -> FakeSerial:
        self.calls.append(kwargs)
        return self.serial_port


class AngleEncodingTests(unittest.TestCase):
    def test_exact_wire_examples(self) -> None:
        cases = (
            (0, bytes((0x92, 0x00, 0x00, 0x00, 0, 0, 0, 0x29))),
            (10, bytes((0x92, 0x00, 0x0A, 0x00, 0, 0, 0, 0x29))),
            (12.34, bytes((0x92, 0x00, 0x0C, 0x22, 0, 0, 0, 0x29))),
            (-12.34, bytes((0x92, 0x01, 0x0C, 0x22, 0, 0, 0, 0x29))),
            (30, bytes((0x92, 0x00, 0x1E, 0x00, 0, 0, 0, 0x29))),
            (-30, bytes((0x92, 0x01, 0x1E, 0x00, 0, 0, 0, 0x29))),
        )
        for angle, expected in cases:
            with self.subTest(angle=angle):
                self.assertEqual(encode_angle(angle), expected)

    def test_protocol_has_header_six_data_bytes_and_footer(self) -> None:
        frame = encode_angle("12.34")
        self.assertEqual(len(frame), 8)
        self.assertEqual(frame, b"\x92\x00\x0c\x22\x00\x00\x00\x29")
        self.assertEqual(frame[0], 0x92)
        self.assertEqual(frame[-1], 0x29)
        self.assertEqual(len(frame[1:7]), 6)

    def test_rounds_half_up_and_carries_into_integer_part(self) -> None:
        self.assertEqual(
            encode_angle("1.005"),
            bytes((0x92, 0x00, 0x01, 0x01, 0, 0, 0, 0x29)),
        )
        self.assertEqual(
            encode_angle("9.995"),
            bytes((0x92, 0x00, 0x0A, 0x00, 0, 0, 0, 0x29)),
        )
        self.assertEqual(
            encode_angle("29.999"),
            bytes((0x92, 0x00, 0x1E, 0x00, 0, 0, 0, 0x29)),
        )

    def test_quantized_zero_never_has_negative_sign(self) -> None:
        zero = bytes((0x92, 0x00, 0x00, 0x00, 0, 0, 0, 0x29))
        self.assertEqual(encode_angle(-0.0), zero)
        self.assertEqual(encode_angle("-0.004"), zero)
        self.assertEqual(
            encode_angle("-0.005"),
            bytes((0x92, 0x01, 0x00, 0x01, 0, 0, 0, 0x29)),
        )

    def test_rejects_out_of_range_nonfinite_and_non_numeric_values(self) -> None:
        invalid_values = (
            30.001,
            -30.001,
            float("nan"),
            float("inf"),
            float("-inf"),
            True,
            None,
            "not-a-number",
        )
        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(AngleEncodingError):
                    encode_angle(value)

    def test_rate_must_be_finite_and_at_least_twenty_hertz(self) -> None:
        self.assertEqual(validate_rate_hz(20), 20.0)
        self.assertEqual(validate_rate_hz("50"), 50.0)
        for value in (19.999, 0, -20, math.nan, math.inf, True, "bad"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    validate_rate_hz(value)


class PeriodicSenderTests(unittest.TestCase):
    def test_default_periodic_rate_is_fifty_hertz(self) -> None:
        sender = PeriodicAngleSender(FakeSerial())
        self.assertEqual(sender.rate_hz, 50.0)

    def test_send_once_writes_and_flushes_exact_payload(self) -> None:
        serial_port = FakeSerial()
        sender = PeriodicAngleSender(serial_port, initial_angle_deg=-8.5)

        frame = sender.send_once()

        self.assertEqual(frame, b"\x92\x01\x08\x32\x00\x00\x00\x29")
        self.assertEqual(
            serial_port.writes,
            [MOTOR_ENABLE_FRAME, MOTOR_INITIAL_ZERO_FRAME, frame],
        )
        self.assertEqual(serial_port.flush_count, 3)
        self.assertEqual(sender.frames_sent, 1)

    def test_enable_then_zero_are_sent_once_before_angle_frames(self) -> None:
        serial_port = FakeSerial()
        sender = PeriodicAngleSender(serial_port)

        sender.send_once(1.0)
        sender.send_once(2.0)

        self.assertEqual(
            MOTOR_ENABLE_FRAME,
            bytes((0x92, 0x4F, 0x4B, 0, 0, 0, 0, 0x29)),
        )
        self.assertEqual(
            MOTOR_INITIAL_ZERO_FRAME,
            bytes((0x92, 0x00, 0x00, 0, 0, 0, 0, 0x29)),
        )
        self.assertEqual(serial_port.writes[0], MOTOR_ENABLE_FRAME)
        self.assertEqual(
            serial_port.writes[1], MOTOR_INITIAL_ZERO_FRAME
        )
        self.assertEqual(
            serial_port.writes.count(MOTOR_ENABLE_FRAME), 1
        )
        self.assertEqual(
            serial_port.writes[2:],
            [encode_angle(1.0), encode_angle(2.0)],
        )

    def test_set_angle_updates_payload_repeated_by_background_thread(self) -> None:
        serial_port = FakeSerial()
        sender = PeriodicAngleSender(
            serial_port,
            rate_hz=100,
            initial_angle_deg=1.25,
        )
        self.addCleanup(sender.close)

        sender.start()
        self.assertTrue(
            serial_port.wait_until(lambda writes: len(writes) >= 3)
        )
        first_payload = encode_angle(1.25)
        self.assertEqual(serial_port.writes[0], MOTOR_ENABLE_FRAME)
        self.assertEqual(
            serial_port.writes[1], MOTOR_INITIAL_ZERO_FRAME
        )
        self.assertTrue(
            all(
                frame == first_payload
                for frame in serial_port.writes[2:]
            )
        )

        previous_count = len(serial_port.writes)
        updated_payload = sender.set_angle(-2.5)
        self.assertTrue(
            serial_port.wait_until(
                lambda writes: updated_payload in writes[previous_count:]
            )
        )

        sender.stop()
        self.assertFalse(sender.is_running)
        self.assertFalse(serial_port.closed)

    def test_stop_and_send_zero_makes_zero_the_final_frame(self) -> None:
        serial_port = FakeSerial()
        sender = PeriodicAngleSender(
            serial_port,
            rate_hz=100,
            initial_angle_deg=8.0,
        )
        self.addCleanup(sender.close)

        sender.start()
        self.assertTrue(
            serial_port.wait_until(lambda writes: len(writes) >= 2)
        )
        sender.stop_and_send_zero()

        self.assertFalse(sender.is_running)
        self.assertEqual(serial_port.writes[-1], encode_angle(0.0))

    def test_short_write_stops_thread_and_exposes_failure(self) -> None:
        serial_port = FakeSerial(write_result=5)
        sender = PeriodicAngleSender(serial_port, rate_hz=50)
        self.addCleanup(sender.close)

        sender.start()
        self.assertTrue(
            serial_port.wait_until(lambda writes: len(writes) >= 1)
        )
        deadline = time.monotonic() + 1.0
        while sender.error is None and time.monotonic() < deadline:
            time.sleep(0.001)

        self.assertIsInstance(sender.error, IOError)
        with self.assertRaises(RuntimeError):
            sender.raise_if_failed()

    def test_open_uses_injected_factory_and_owns_resulting_port(self) -> None:
        factory = FakeSerialFactory()
        sender = PeriodicAngleSender.open(
            port="/dev/fake-angle",
            baudrate=115200,
            rate_hz=50,
            initial_angle_deg=3.21,
            write_timeout=0.25,
            serial_factory=factory,
        )

        sender.send_once()
        sender.close()

        self.assertEqual(
            factory.calls,
            [
                {
                    "port": "/dev/fake-angle",
                    "baudrate": 115200,
                    "timeout": 0,
                    "write_timeout": 0.25,
                    "exclusive": True,
                }
            ],
        )
        self.assertEqual(
            factory.serial_port.writes,
            [
                MOTOR_ENABLE_FRAME,
                MOTOR_INITIAL_ZERO_FRAME,
                encode_angle(3.21),
            ],
        )
        self.assertTrue(factory.serial_port.closed)

    def test_open_serial_port_can_be_tested_without_pyserial(self) -> None:
        factory = FakeSerialFactory()
        serial_port = open_serial_port(
            port="/dev/fake",
            baudrate=9600,
            serial_factory=factory,
        )
        self.assertIs(serial_port, factory.serial_port)

    def test_open_validates_before_touching_serial_factory(self) -> None:
        factory = FakeSerialFactory()
        with self.assertRaises(ValueError):
            PeriodicAngleSender.open(
                rate_hz=19,
                serial_factory=factory,
            )
        with self.assertRaises(AngleEncodingError):
            PeriodicAngleSender.open(
                initial_angle_deg=30.1,
                serial_factory=factory,
            )
        self.assertEqual(factory.calls, [])

    def test_closed_sender_rejects_new_commands(self) -> None:
        serial_port = FakeSerial()
        sender = PeriodicAngleSender(serial_port)
        sender.close()
        with self.assertRaises(RuntimeError):
            sender.set_angle(1.0)


if __name__ == "__main__":
    unittest.main()
