#!/usr/bin/env python3
"""Bridge a BLE UART characteristic pair to the XML mission through a PTY."""

import argparse
import asyncio
import os
import pty
import signal
import subprocess
import tty
from pathlib import Path

from bleak import BleakClient

ROOT = Path("/home/pangolin/NUEDC")
MISSION_RUNNER = ROOT / "mission_bt/run_mission.py"
DEFAULT_TREE = ROOT / "mission_bt/config/simple_ack.xml"
BT24_UART_BAUD = 9600


def parse_args():
    parser = argparse.ArgumentParser(
        description="BT04 BLE -> PTY -> XML behavior-tree bridge"
    )
    parser.add_argument("--address", required=True)
    parser.add_argument("--write-char", required=True)
    parser.add_argument("--notify-char", required=True)
    parser.add_argument("--tree", type=Path, default=DEFAULT_TREE)
    parser.add_argument("--chunk-size", type=int, default=20)
    parser.add_argument("--response", action="store_true")
    parser.add_argument("--tick-hz", type=float, default=20.0)
    return parser.parse_args()


async def run_bridge(args) -> int:
    if args.chunk_size <= 0:
        raise ValueError("chunk-size must be positive")

    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    slave_path = os.ttyname(slave_fd)
    print(f"BLE bridge PTY: {slave_path}")

    child_command = [
        "/usr/bin/python3",
        "-u",
        str(MISSION_RUNNER),
        "--tree",
        str(args.tree.expanduser().resolve()),
        "--transport",
        "serial",
        "--port",
        slave_path,
        "--baud",
        str(BT24_UART_BAUD),
        "--tick-hz",
        str(args.tick_hz),
    ]
    child = subprocess.Popen(child_command)
    loop = asyncio.get_running_loop()
    tx_queue = asyncio.Queue()

    def pty_readable():
        try:
            data = os.read(master_fd, 4096)
        except OSError:
            return
        if data:
            tx_queue.put_nowait(data)

    try:
        loop.add_reader(master_fd, pty_readable)
        async with BleakClient(args.address, timeout=20.0) as client:
            print(f"BLE connected: {args.address}")

            def notification(sender, data: bytearray):
                payload = bytes(data)
                print(f"BLE -> PTY: {payload.hex(' ')}")
                try:
                    os.write(master_fd, payload)
                except OSError as exc:
                    print(f"PTY write failed: {exc}")

            await client.start_notify(args.notify_char, notification)
            print(f"Notify characteristic: {args.notify_char}")
            print(f"Write characteristic : {args.write_char}")

            async def transmit_worker():
                while child.poll() is None and client.is_connected:
                    try:
                        data = await asyncio.wait_for(
                            tx_queue.get(), timeout=0.2
                        )
                    except asyncio.TimeoutError:
                        continue
                    print(f"PTY -> BLE: {data.hex(' ')}")
                    for offset in range(0, len(data), args.chunk_size):
                        await client.write_gatt_char(
                            args.write_char,
                            data[offset : offset + args.chunk_size],
                            response=args.response,
                        )

            await transmit_worker()
            await client.stop_notify(args.notify_char)
    finally:
        loop.remove_reader(master_fd)
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.terminate()
                child.wait(timeout=3)
        os.close(master_fd)
        os.close(slave_fd)
    return child.returncode or 0


def main() -> int:
    return asyncio.run(run_bridge(parse_args()))


if __name__ == "__main__":
    raise SystemExit(main())
