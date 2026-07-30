#!/usr/bin/env python3
"""
串口接收程序 — 持续监听串口，把收到的数据打印到终端
协议格式: 0x76 + 6字节数据 + 0x67 (共8字节)

用法:
    python3 receive.py                     # 持续监听，打印所有收到的字节
    python3 receive.py --port /dev/ttyUSB1 # 指定串口
    python3 receive.py --baud 9600         # 指定波特率
    python3 receive.py --hex-only          # 只显示十六进制
    python3 receive.py --framed            # 按帧解析 (0x76 开头, 0x67 结尾)
    python3 receive.py --timeout 10        # 10秒无数据后自动退出
"""

import argparse
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    print(
        "缺少pyserial。请从项目根目录运行："
        "./serial/start_receive.sh",
        file=sys.stderr,
    )
    sys.exit(2)

# ======================== 默认配置 ========================
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 9600

HEADER = 0x76
FOOTER = 0x67
FRAME_LEN = 8  # 帧总长度


def timestamp() -> str:
    """当前时间戳字符串"""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def print_hex(data: bytes, label: str = ""):
    """以十六进制格式打印字节"""
    hex_str = " ".join(f"0x{b:02X}" for b in data)
    if label:
        print(f"[{timestamp()}] {label}: {hex_str}")
    else:
        print(f"[{timestamp()}] {hex_str}")


def receive_raw(ser: serial.Serial, timeout: float = 0):
    """原始模式: 收到什么打印什么"""
    print(f"开始监听 {ser.port} (原始模式) ...")
    print("按 Ctrl+C 停止\n")

    last_data_time = time.time()

    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                last_data_time = time.time()
                print_hex(data, f"收到 {len(data)} 字节")
            else:
                time.sleep(0.01)  # 避免空转占满CPU

            if timeout > 0 and (time.time() - last_data_time) > timeout:
                print(f"\n{timeout}秒无数据，自动退出。")
                break
    except KeyboardInterrupt:
        print("\n用户中断，退出。")


def receive_framed(ser: serial.Serial, timeout: float = 0):
    """帧解析模式: 按 HEADER + 6数据 + FOOTER 解析"""
    print(f"开始监听 {ser.port} (帧解析模式) ...")
    print(f"帧格式: 0x76 + [6字节数据] + 0x67")
    print("按 Ctrl+C 停止\n")

    buf = bytearray()
    last_data_time = time.time()

    try:
        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buf.extend(chunk)
                last_data_time = time.time()

                # 尝试从缓冲区中提取完整帧
                while len(buf) >= FRAME_LEN:
                    header_idx = buf.find(bytes((HEADER,)))
                    if header_idx == -1:
                        # 没找到帧头，清空缓冲区（或保留最后 FRAME_LEN-1 字节以防帧头被截断）
                        print_hex(bytes(buf[:header_idx]) if header_idx > 0 else b"", "丢弃")
                        buf = bytearray()
                        break

                    # 丢弃帧头之前的杂散数据
                    if header_idx > 0:
                        print_hex(bytes(buf[:header_idx]), "丢弃")
                        del buf[:header_idx]

                    # 检查剩余数据是否够一帧
                    if len(buf) < FRAME_LEN:
                        break

                    # 验证帧尾
                    if buf[FRAME_LEN - 1] == FOOTER:
                        frame = bytes(buf[:FRAME_LEN])
                        data_bytes = frame[1:7]  # 中间6字节数据
                        print_hex(frame, f"完整帧 | 数据={data_bytes.hex(' ')}")
                        del buf[:FRAME_LEN]
                    else:
                        # 帧尾不匹配，丢弃第1字节（假帧头），继续搜索
                        print_hex(bytes([buf[0]]), "丢弃(假帧头)")
                        del buf[0]
            else:
                time.sleep(0.01)

            if timeout > 0 and (time.time() - last_data_time) > timeout:
                print(f"\n{timeout}秒无数据，自动退出。")
                break
    except KeyboardInterrupt:
        print("\n用户中断，退出。")


def main():
    parser = argparse.ArgumentParser(
        description="串口接收程序 — 监听串口并打印收到的数据",
        epilog="示例: python3 receive.py --framed",
    )
    parser.add_argument("--port", "-p", default=DEFAULT_PORT, help=f"串口设备 (默认: {DEFAULT_PORT})")
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD, help=f"波特率 (默认: {DEFAULT_BAUD})")
    parser.add_argument("--framed", "-f", action="store_true",
                        help="按帧解析模式 (识别 0x76...0x67 帧)")
    parser.add_argument("--timeout", "-t", type=float, default=0,
                        help="N秒无数据后自动退出 (0=永不超时)")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"串口已打开: {args.port} @ {args.baud} baud")
    except serial.SerialException as e:
        print(f"❌ 无法打开串口: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        if args.framed:
            receive_framed(ser, args.timeout)
        else:
            receive_raw(ser, args.timeout)
    finally:
        ser.close()
        print("串口已关闭。")


if __name__ == "__main__":
    main()
