#!/usr/bin/env python3
"""持续向单片机发送管道目标倾角。

协议为恰好 8 个二进制字节：

    [0x92, 符号, 整数部分, 两位小数部分, 0, 0, 0, 0x29]

示例：

    python3 serial/send.py 12.34
    python3 serial/send.py -8.50 --rate-hz 50
    python3 serial/send.py 10 --once
    python3 serial/send.py 0 --port /dev/ttyUSB1 --baud 9600

未指定 ``--once`` 时保持串口打开，并默认以 50 Hz 重复发送。
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Optional, Sequence


# 本目录名为 serial，与 pyserial 的导入名冲突。协议实现放在 H 中，
# 这里按文件位置显式加入路径，避免把本地 serial/ 当作 Python 包。
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
H_DIR = REPOSITORY_ROOT / "H"
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from angle_serial import (  # noqa: E402
    DEFAULT_BAUD,
    DEFAULT_PORT,
    DEFAULT_RATE_HZ,
    MOTOR_ENABLE_FRAME,
    MOTOR_INITIAL_ZERO_FRAME,
    AngleEncodingError,
    PeriodicAngleSender,
    SerialDependencyError,
    encode_angle,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="按 0x92 + 6字节数据 + 0x29 协议持续发送管道目标倾角",
        epilog=(
            "示例: python3 serial/send.py -12.34 --rate-hz 50；"
            "输出为 92 01 0C 22 00 00 00 29"
        ),
    )
    parser.add_argument(
        "angle",
        help="目标倾角（度），范围 [-30, 30]；正角表示电机端抬升",
    )
    parser.add_argument(
        "--port",
        "-p",
        default=DEFAULT_PORT,
        help="串口设备（默认: %(default)s）",
    )
    parser.add_argument(
        "--baud",
        "-b",
        type=int,
        default=DEFAULT_BAUD,
        help="串口波特率（默认: %(default)s）",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=DEFAULT_RATE_HZ,
        help="连续发送频率，不得低于 20 Hz（默认: %(default)s）",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="只发送一次，供协议和接线调试",
    )
    return parser


def format_hex(data: bytes) -> str:
    return " ".join("0x{:02X}".format(value) for value in data)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        frame = encode_angle(args.angle)
        sender = PeriodicAngleSender.open(
            port=args.port,
            baudrate=args.baud,
            rate_hz=args.rate_hz,
            initial_angle_deg=args.angle,
        )
    except (AngleEncodingError, SerialDependencyError, ValueError, OSError) as exc:
        parser.error(str(exc))

    try:
        # 给 USB-UART 和可能因开串口复位的单片机留出稳定时间。
        time.sleep(0.1)
        print(
            "串口 {} @ {} baud | 电机使能 {} | 初始化0° {} | "
            "倾角 {}° | 数据 {}"
            .format(
                args.port,
                args.baud,
                format_hex(MOTOR_ENABLE_FRAME),
                format_hex(MOTOR_INITIAL_ZERO_FRAME),
                args.angle,
                format_hex(frame),
            )
        )

        if args.once:
            sender.send_once()
            print("发送完成（单次）")
            return 0

        sender.start()
        print(
            "正在以 {:.2f} Hz 持续发送，按 Ctrl+C 停止。".format(
                sender.rate_hz
            )
        )
        while sender.is_running:
            sender.raise_if_failed()
            time.sleep(0.1)
        sender.raise_if_failed()
        return 0
    except KeyboardInterrupt:
        print("\n已停止发送。")
        return 0
    except Exception as exc:
        print("串口发送失败: {}".format(exc), file=sys.stderr)
        return 1
    finally:
        if not args.once:
            try:
                # 后台线程可能已经缓存了一帧旧角度。必须先把待发值改成0，
                # 再停止线程，最后同步发0；否则旧非零帧可能排在0°之后。
                sender.stop_and_send_zero()
                print("退出前已发送 0°。")
            except Exception as exc:
                print(
                    "警告：退出归零发送失败: {}".format(exc),
                    file=sys.stderr,
                )
        sender.close()


if __name__ == "__main__":
    sys.exit(main())
