#!/usr/bin/env python3
"""交互或命令行测试管道电机目标倾角，不启动相机和控制器。"""

import argparse
from decimal import Decimal, InvalidOperation
import sys
import time
from typing import Optional, Sequence

from angle_serial import (
    DEFAULT_BAUD,
    DEFAULT_PORT,
    DEFAULT_RATE_HZ,
    MAX_ANGLE_DEG,
    MOTOR_ENABLE_FRAME,
    MOTOR_INITIAL_ZERO_FRAME,
    AngleEncodingError,
    PeriodicAngleSender,
    SerialDependencyError,
    angle_to_serial_displacement_mm,
    encode_angle,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "直接向电机发送管道目标倾角；默认交互输入，退出自动发送0°"
        )
    )
    parser.add_argument(
        "angle",
        nargs="?",
        help="可选：直接持续发送该角度，按Ctrl+C归零退出",
    )
    parser.add_argument("--port", "-p", default=DEFAULT_PORT)
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=DEFAULT_RATE_HZ,
        help="周期发送频率，默认%(default)s Hz且不得低于20 Hz",
    )
    parser.add_argument(
        "--limit-deg",
        type=Decimal,
        default=Decimal("10"),
        help=(
            "本测试允许的绝对角度，默认±%(default)s°，"
            "最大{}°".format(MAX_ANGLE_DEG)
        ),
    )
    return parser


def format_frame(frame: bytes) -> str:
    return " ".join("0x{:02X}".format(value) for value in frame)


def validate_test_angle(text: str, limit_deg: Decimal) -> Decimal:
    try:
        value = Decimal(text.strip())
    except (InvalidOperation, AttributeError) as error:
        raise ValueError("请输入有效角度数字") from error
    if not value.is_finite():
        raise ValueError("角度必须是有限值")
    if abs(value) > limit_deg:
        raise ValueError(
            "测试角度超出当前±{}°限制；确需扩大时使用"
            " --limit-deg，最大只能{}°".format(
                limit_deg, MAX_ANGLE_DEG
            )
        )
    encode_angle(value)
    return value


def set_and_report(
    sender: PeriodicAngleSender, value: Decimal
) -> None:
    frame = sender.set_angle(value)
    displacement_mm = angle_to_serial_displacement_mm(value)
    print(
        "目标倾角：{}° | 电机升降：{:+.2f} mm | 发送：{}".format(
            value,
            displacement_mm,
            format_frame(frame),
        ),
        flush=True,
    )


def interactive_loop(
    sender: PeriodicAngleSender, limit_deg: Decimal
) -> None:
    print(
        "交互模式：输入角度后按Enter；输入 0 归零，q 归零退出，"
        "当前限制±{}°。".format(limit_deg)
    )
    while True:
        try:
            text = input("angle> ").strip()
        except EOFError:
            return
        if text.lower() in ("q", "quit", "exit"):
            return
        if not text:
            continue
        try:
            value = validate_test_angle(text, limit_deg)
        except (ValueError, AngleEncodingError) as error:
            print("输入错误：{}".format(error), file=sys.stderr)
            continue
        set_and_report(sender, value)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if (
        not args.limit_deg.is_finite()
        or args.limit_deg <= 0
        or args.limit_deg > MAX_ANGLE_DEG
    ):
        parser.error(
            "--limit-deg必须位于(0, {}]度".format(MAX_ANGLE_DEG)
        )
    initial_angle: Optional[Decimal] = None
    if args.angle is not None:
        try:
            initial_angle = validate_test_angle(
                args.angle, args.limit_deg
            )
        except (ValueError, AngleEncodingError) as error:
            parser.error(str(error))

    sender: Optional[PeriodicAngleSender] = None
    try:
        sender = PeriodicAngleSender.open(
            port=args.port,
            baudrate=args.baud,
            rate_hz=args.rate_hz,
            initial_angle_deg=0.0,
        )
        # 某些USB-UART打开后会让单片机复位，先等待并明确发送0°。
        time.sleep(0.1)
        sender.send_once(0.0)
        sender.start()
        print(
            "串口：{} @ {} baud，电机使能：{}，初始化0°：{}，"
            "发送频率：{:.1f} Hz。"
            .format(
                args.port,
                args.baud,
                format_frame(MOTOR_ENABLE_FRAME),
                format_frame(MOTOR_INITIAL_ZERO_FRAME),
                sender.rate_hz,
            )
        )
        if initial_angle is None:
            interactive_loop(sender, args.limit_deg)
        else:
            set_and_report(sender, initial_angle)
            print("正在持续发送；按Ctrl+C后归零退出。")
            while sender.is_running:
                sender.raise_if_failed()
                time.sleep(0.1)
            sender.raise_if_failed()
        return 0
    except KeyboardInterrupt:
        print("\n收到退出请求。")
        return 0
    except (
        AngleEncodingError,
        SerialDependencyError,
        RuntimeError,
        ValueError,
        OSError,
    ) as error:
        print("电机角度测试失败：{}".format(error), file=sys.stderr)
        return 1
    finally:
        if sender is not None:
            try:
                sender.stop_and_send_zero()
                print("已停止周期发送，并同步发送最终0°。")
            except Exception as error:
                print(
                    "警告：最终归零失败：{}".format(error),
                    file=sys.stderr,
                )
            finally:
                sender.close()


if __name__ == "__main__":
    sys.exit(main())
