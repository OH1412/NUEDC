#!/usr/bin/env python3
"""Scan and inspect BLE UART modules with Bleak."""

import argparse
import asyncio
from datetime import datetime

from bleak import BleakClient, BleakScanner


def parse_hex(text: str) -> bytes:
    tokens = text.replace(",", " ").replace("-", " ").split()
    return bytes(int(token, 16) for token in tokens)


async def scan(timeout: float) -> None:
    discovered = await BleakScanner.discover(
        timeout=timeout,
        return_adv=True,
    )
    if not discovered:
        print("No BLE devices found.")
        return
    for address in sorted(discovered):
        device, advertisement = discovered[address]
        name = advertisement.local_name or device.name
        print(
            f"{device.address}  RSSI={advertisement.rssi}  name={name!r}"
        )


async def inspect_device(address: str) -> None:
    async with BleakClient(address, timeout=20.0) as client:
        print(f"Connected: {client.is_connected}")
        for service in client.services:
            print(f"SERVICE {service.uuid}  {service.description}")
            for characteristic in service.characteristics:
                properties = ",".join(characteristic.properties)
                print(
                    f"  CHAR {characteristic.uuid} "
                    f"handle={characteristic.handle} [{properties}]"
                )


async def exchange(args) -> None:
    received = asyncio.Event()

    def notification(sender, data: bytearray):
        print(f"NOTIFY {sender.uuid}: {bytes(data).hex(' ')}")
        received.set()

    async with BleakClient(args.address, timeout=20.0) as client:
        await client.start_notify(args.notify_char, notification)
        print("Notifications enabled.")
        if args.send_hex:
            payload = parse_hex(args.send_hex)
            await client.write_gatt_char(
                args.write_char,
                payload,
                response=args.response,
            )
            print(f"WRITE: {payload.hex(' ')}")
        try:
            await asyncio.wait_for(received.wait(), timeout=args.wait)
        except asyncio.TimeoutError:
            print("No notification received before timeout.")
        await client.stop_notify(args.notify_char)


async def listen_forever(args) -> None:
    """Continuously print notifications and reconnect after a BLE disconnect."""
    while True:
        disconnected = asyncio.Event()

        def on_disconnect(_client):
            disconnected.set()

        def notification(sender, data: bytearray):
            now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            payload = bytes(data)
            print(
                f"[{now}] RX {len(payload):3d} bytes: "
                f"{payload.hex(' ')}",
                flush=True,
            )

        try:
            print(f"Connecting to {args.address} ...", flush=True)
            async with BleakClient(
                args.address,
                timeout=20.0,
                disconnected_callback=on_disconnect,
            ) as client:
                await client.start_notify(args.notify_char, notification)
                print(
                    "Connected. Listening continuously; press Ctrl+C to stop.",
                    flush=True,
                )
                await disconnected.wait()
                print("BT24 disconnected.", flush=True)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print(f"BLE error: {exc}", flush=True)

        print(f"Reconnect in {args.reconnect_delay:g} seconds ...", flush=True)
        await asyncio.sleep(args.reconnect_delay)


def parse_args():
    parser = argparse.ArgumentParser(description="BT04 BLE diagnostic tool")
    subparsers = parser.add_subparsers(dest="command", required=True)

    scan_parser = subparsers.add_parser("scan")
    scan_parser.add_argument("--timeout", type=float, default=10.0)

    inspect_parser = subparsers.add_parser("inspect")
    inspect_parser.add_argument("address")

    exchange_parser = subparsers.add_parser("exchange")
    exchange_parser.add_argument("address")
    exchange_parser.add_argument("--write-char", required=True)
    exchange_parser.add_argument("--notify-char", required=True)
    exchange_parser.add_argument("--send-hex")
    exchange_parser.add_argument("--wait", type=float, default=10.0)
    exchange_parser.add_argument("--response", action="store_true")

    listen_parser = subparsers.add_parser("listen")
    listen_parser.add_argument("address")
    listen_parser.add_argument("--notify-char", required=True)
    listen_parser.add_argument("--reconnect-delay", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command == "scan":
        asyncio.run(scan(args.timeout))
    elif args.command == "inspect":
        asyncio.run(inspect_device(args.address))
    elif args.command == "exchange":
        asyncio.run(exchange(args))
    else:
        try:
            asyncio.run(listen_forever(args))
        except KeyboardInterrupt:
            print("\nStopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
