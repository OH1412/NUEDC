#!/usr/bin/env python3
"""
串口发送程序
协议格式: 0x92 + 6字节数据 + 0x29

用法:
    python3 send.py                        # 发送默认数据 0x06 0x07 0x08 0x09 0x0a 0x0b
    python3 send.py 0x01 0x02 0x03 0x04 0x05 0x06   # 发送指定6字节数据
    python3 send.py 1 2 3 4 5 6           # 支持十进制
    python3 send.py --port /dev/ttyUSB1   # 指定串口
    python3 send.py --baud 9600           # 指定波特率
"""

import serial
import argparse
import sys
import time

# ======================== 默认配置 ========================
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 9600
DEFAULT_DATA = [0x01, 0x03, 0x05, 0x07, 0x09, 0x11]

HEADER = 0x92
FOOTER = 0x29


def parse_byte(val: str) -> int:
    """解析字节值，支持 0x 前缀（十六进制）或纯十进制"""
    val = val.strip()
    if val.lower().startswith("0x"):
        return int(val, 16)
    return int(val)


def build_frame(data: list[int]) -> bytes:
    """构建发送帧: HEADER + 6字节数据 + FOOTER"""
    if len(data) != 6:
        print(f"错误: 数据必须是6字节，当前为 {len(data)} 字节", file=sys.stderr)
        sys.exit(1)
    for b in data:
        if not (0 <= b <= 255):
            print(f"错误: 数据值 {b} 超出范围 (0-255)", file=sys.stderr)
            sys.exit(1)

    frame = bytes([HEADER] + data + [FOOTER])
    return frame


def main():
    parser = argparse.ArgumentParser(
        description="串口发送程序 — 向单片机发送 8 字节帧",
        epilog="示例: python3 send.py 0x01 0x02 0x03 0x04 0x05 0x06",
    )
    parser.add_argument(
        "data",
        nargs="*",
        help="6字节数据 (十六进制如 0x06 或十进制如 6)",
    )
    parser.add_argument("--port", "-p", default=DEFAULT_PORT, help=f"串口设备 (默认: {DEFAULT_PORT})")
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD, help=f"波特率 (默认: {DEFAULT_BAUD})")
    args = parser.parse_args()

    # 解析数据
    if args.data:
        if len(args.data) != 6:
            print(f"错误: 必须提供恰好6字节数据，当前为 {len(args.data)} 字节", file=sys.stderr)
            sys.exit(1)
        try:
            data = [parse_byte(v) for v in args.data]
        except ValueError as e:
            print(f"错误: 无法解析数据值: {e}", file=sys.stderr)
            sys.exit(1)
    else:
        data = DEFAULT_DATA
        print(f"未指定数据，使用默认值: {[f'0x{b:02X}' for b in data]}")

    # 构建帧
    frame = build_frame(data)

    # 打印即将发送的内容
    print(f"串口: {args.port} | 波特率: {args.baud}")
    print(f"发送帧: {' '.join(f'0x{b:02X}' for b in frame)}")
    print()

    # 打开串口并发送
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
        # 等待串口就绪
        time.sleep(0.1)
        ser.write(frame)
        ser.flush()
        print("✅ 发送成功!")
        ser.close()
    except serial.SerialException as e:
        print(f"❌ 串口错误: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
