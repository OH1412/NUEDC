#!/usr/bin/env python3
"""Send AT commands through a Linux UART without pyserial."""

import argparse
import os
import select
import termios
import time


BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}


def open_uart(port: str, baud: int) -> int:
    if baud not in BAUD_RATES:
        raise ValueError(f"unsupported baud rate: {baud}")
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = BAUD_RATES[baud]
    attrs[5] = BAUD_RATES[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_response(fd: int, timeout_s: float, quiet_s: float = 0.15) -> bytes:
    deadline = time.monotonic() + timeout_s
    quiet_deadline = None
    data = bytearray()
    while time.monotonic() < deadline:
        wait_until = deadline
        if quiet_deadline is not None:
            wait_until = min(wait_until, quiet_deadline)
        remaining = max(0.0, wait_until - time.monotonic())
        readable, _, _ = select.select([fd], [], [], remaining)
        if readable:
            chunk = os.read(fd, 4096)
            if chunk:
                data.extend(chunk)
                quiet_deadline = time.monotonic() + quiet_s
                continue
        if quiet_deadline is not None and time.monotonic() >= quiet_deadline:
            break
    return bytes(data)


def main() -> int:
    parser = argparse.ArgumentParser(description="HC-05/BT04 AT 命令工具")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument(
        "--ending",
        choices=("none", "cr", "lf", "crlf"),
        default="crlf",
    )
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("commands", nargs="+")
    args = parser.parse_args()

    endings = {
        "none": b"",
        "cr": b"\r",
        "lf": b"\n",
        "crlf": b"\r\n",
    }
    fd = open_uart(args.port, args.baud)
    try:
        print(f"UART: {args.port} @ {args.baud} 8N1")
        for command in args.commands:
            payload = command.encode("ascii") + endings[args.ending]
            print(f"TX ASCII: {command!r}")
            os.write(fd, payload)
            termios.tcdrain(fd)
            response = read_response(fd, args.timeout)
            if response:
                print(f"RX HEX  : {response.hex(' ')}")
                print(
                    "RX ASCII:",
                    response.decode("ascii", errors="backslashreplace").rstrip(),
                )
            else:
                print("RX      : <timeout>")
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
