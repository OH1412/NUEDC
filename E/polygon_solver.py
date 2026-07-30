#!/usr/bin/env python3
"""由多边形的已知边长和角度求一个未知量。

支持两种角度定义：

1. direction：每条边相对 +x 轴的绝对方向角，逆时针为正；
2. interior：每条边终点处的内角，按逆时针方向遍历多边形。

输入中必须恰好有一个 x，x 可以代替一个边长或一个角度。
"""

import argparse
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


EPSILON = 1e-9
DEFAULT_PLOT = Path(__file__).resolve().parent / "output" / "polygon.png"


class SolveError(ValueError):
    """输入数据不足以得到唯一、有效的解。"""


@dataclass
class Edge:
    length: Optional[float]
    angle: Optional[float]


@dataclass
class Solution:
    unknown_kind: str
    unknown_edge: int
    value: float
    edges: List[Edge]
    directions: List[float]
    vertices: List[Tuple[float, float]]
    residual: float
    tolerance: float
    consistent: bool
    note: str


def normalize_angle(angle: float) -> float:
    """将角度归一化到 [0, 360)。"""
    value = angle % 360.0
    return 0.0 if abs(value - 360.0) < EPSILON else value


def parse_value(text: str) -> Optional[float]:
    if text.strip().lower() == "x":
        return None
    value = float(text)
    if not math.isfinite(value):
        raise SolveError("输入必须是有限数值或 x。")
    return value


def parse_edges(side_count: int, tokens: Sequence[str]) -> List[Edge]:
    if side_count < 3:
        raise SolveError("多边形边数至少为 3。")
    expected = 2 * side_count
    if len(tokens) != expected:
        raise SolveError(
            "需要输入 {} 个值（每条边依次为“边长 角度”），实际输入 {} 个。".format(
                expected, len(tokens)
            )
        )

    edges = []
    unknown_count = 0
    for index in range(side_count):
        length = parse_value(tokens[2 * index])
        angle = parse_value(tokens[2 * index + 1])
        unknown_count += int(length is None) + int(angle is None)
        if length is not None and length <= 0:
            raise SolveError("第 {} 条边的边长必须大于 0。".format(index + 1))
        edges.append(Edge(length, angle))

    if unknown_count != 1:
        raise SolveError(
            "必须恰好输入一个 x；当前检测到 {} 个。".format(unknown_count)
        )
    return edges


def vector(length: float, direction_deg: float) -> Tuple[float, float]:
    radians = math.radians(direction_deg)
    return length * math.cos(radians), length * math.sin(radians)


def closure_tolerance(edges: Sequence[Edge]) -> float:
    perimeter = sum(edge.length or 0.0 for edge in edges)
    return max(1e-6, perimeter * 1e-6)


def build_vertices(
    edges: Sequence[Edge], directions: Sequence[float]
) -> List[Tuple[float, float]]:
    x_coord = 0.0
    y_coord = 0.0
    vertices = [(x_coord, y_coord)]
    for edge, direction in zip(edges, directions):
        if edge.length is None:
            raise AssertionError("求解后仍存在未知边长。")
        dx, dy = vector(edge.length, direction)
        x_coord += dx
        y_coord += dy
        vertices.append((x_coord, y_coord))
    return vertices


def residual_from_vertices(vertices: Sequence[Tuple[float, float]]) -> float:
    return math.hypot(vertices[-1][0] - vertices[0][0],
                      vertices[-1][1] - vertices[0][1])


def solve_missing_length(
    edges: List[Edge], directions: Sequence[float], unknown_edge: int
) -> Tuple[float, str]:
    known_x = 0.0
    known_y = 0.0
    for index, (edge, direction) in enumerate(zip(edges, directions)):
        if index == unknown_edge:
            continue
        if edge.length is None:
            raise AssertionError("存在多个未知边长。")
        dx, dy = vector(edge.length, direction)
        known_x += dx
        known_y += dy

    target_x = -known_x
    target_y = -known_y
    radians = math.radians(directions[unknown_edge])
    unit_x = math.cos(radians)
    unit_y = math.sin(radians)

    # 将待闭合向量投影到未知边的已知方向上。
    length = target_x * unit_x + target_y * unit_y
    perpendicular_error = abs(target_x * unit_y - target_y * unit_x)
    if length <= EPSILON:
        raise SolveError(
            "按给定方向求得的未知边长为 {:.9g}，不是正数；这些数据无法组成该多边形。"
            .format(length)
        )

    note = "闭合向量在未知边方向上的垂直误差为 {:.9g}。".format(
        perpendicular_error
    )
    return length, note


def directions_from_interiors(edges: Sequence[Edge]) -> List[float]:
    """以第一条边方向为 0°，由各顶点内角推导其余边方向。"""
    directions = [0.0]
    for index in range(len(edges) - 1):
        interior = edges[index].angle
        if interior is None:
            raise AssertionError("求解后仍存在未知内角。")
        # 逆时针遍历时，外转角 = 180° - 内角。
        directions.append(directions[-1] + 180.0 - interior)
    return directions


def solve_direction(edges: List[Edge]) -> Solution:
    unknown_edge = next(
        index
        for index, edge in enumerate(edges)
        if edge.length is None or edge.angle is None
    )
    unknown_kind = "length" if edges[unknown_edge].length is None else "angle"
    note = ""

    if unknown_kind == "length":
        directions = [
            edge.angle if edge.angle is not None else 0.0 for edge in edges
        ]
        value, note = solve_missing_length(edges, directions, unknown_edge)
        edges[unknown_edge].length = value
    else:
        known_x = 0.0
        known_y = 0.0
        for index, edge in enumerate(edges):
            if index == unknown_edge:
                continue
            if edge.length is None or edge.angle is None:
                raise AssertionError("存在多个未知量。")
            dx, dy = vector(edge.length, edge.angle)
            known_x += dx
            known_y += dy

        target_x = -known_x
        target_y = -known_y
        target_length = math.hypot(target_x, target_y)
        length = edges[unknown_edge].length
        if length is None:
            raise AssertionError("未知量类型判断错误。")
        if target_length < EPSILON:
            raise SolveError(
                "其余边已经自行闭合，未知边的方向角没有唯一解。"
            )
        value = normalize_angle(math.degrees(math.atan2(target_y, target_x)))
        edges[unknown_edge].angle = value
        note = (
            "其余边所需的闭合长度为 {:.9g}，未知边给定长度为 {:.9g}。"
            .format(target_length, length)
        )

    directions = [
        edge.angle if edge.angle is not None else 0.0 for edge in edges
    ]
    vertices = build_vertices(edges, directions)
    residual = residual_from_vertices(vertices)
    tolerance = closure_tolerance(edges)
    return Solution(
        unknown_kind,
        unknown_edge,
        value,
        edges,
        directions,
        vertices,
        residual,
        tolerance,
        residual <= tolerance,
        note,
    )


def solve_interior(edges: List[Edge]) -> Solution:
    unknown_edge = next(
        index
        for index, edge in enumerate(edges)
        if edge.length is None or edge.angle is None
    )
    unknown_kind = "length" if edges[unknown_edge].length is None else "angle"
    note = ""

    if unknown_kind == "angle":
        known_sum = sum(
            edge.angle for edge in edges if edge.angle is not None
        )
        value = (len(edges) - 2) * 180.0 - known_sum
        if value <= 0.0 or value >= 360.0:
            raise SolveError(
                "由内角和求得 x = {:.9g}°，不是有效的简单多边形内角。"
                .format(value)
            )
        edges[unknown_edge].angle = value
        note = "使用内角和 (n - 2) × 180° 求解。"
    else:
        directions = directions_from_interiors(edges)
        value, note = solve_missing_length(edges, directions, unknown_edge)
        edges[unknown_edge].length = value

    directions = directions_from_interiors(edges)
    vertices = build_vertices(edges, directions)
    residual = residual_from_vertices(vertices)
    tolerance = closure_tolerance(edges)
    return Solution(
        unknown_kind,
        unknown_edge,
        value,
        edges,
        directions,
        vertices,
        residual,
        tolerance,
        residual <= tolerance,
        note,
    )


def solve(mode: str, edges: List[Edge]) -> Solution:
    copied_edges = [Edge(edge.length, edge.angle) for edge in edges]
    if mode == "direction":
        return solve_direction(copied_edges)
    if mode == "interior":
        return solve_interior(copied_edges)
    raise SolveError("不支持的角度模式：{}".format(mode))


def print_solution(solution: Solution, mode: str) -> None:
    kind_name = "边长" if solution.unknown_kind == "length" else "角度"
    unit = "" if solution.unknown_kind == "length" else "°"
    print("\n计算结果")
    print("x 位于第 {} 条边的{}".format(solution.unknown_edge + 1, kind_name))
    print("x = {:.10g}{}".format(solution.value, unit))
    print(solution.note)
    print("闭合残差 = {:.9g}（容差 {:.9g}）".format(
        solution.residual, solution.tolerance
    ))
    if solution.consistent:
        print("校验：通过，输入数据可以闭合。")
    else:
        print("校验：未通过，x 是最接近的候选值，但其余已知数据彼此不相容。")

    angle_name = "方向角" if mode == "direction" else "内角"
    print("\n完整边数据")
    for index, edge in enumerate(solution.edges):
        print(
            "  边 {:>2}: 长度 = {:>12.8g}, {} = {:>12.8g}°"
            .format(index + 1, edge.length, angle_name, edge.angle)
        )

    print("\n重建顶点（第一点和最后一点应重合）")
    for index, (x_coord, y_coord) in enumerate(solution.vertices):
        print("  P{:>2} = ({:>12.8g}, {:>12.8g})".format(
            index, x_coord, y_coord
        ))


def plot_solution(
    solution: Solution, mode: str, output_path: Path, show: bool
) -> Path:
    """绘制求解后的多边形，保存 PNG，并按需弹出窗口。"""
    try:
        import matplotlib

        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise SolveError(
            "绘图需要 matplotlib，请先安装：python3 -m pip install matplotlib"
        ) from error

    # 当前环境提供 Noto CJK；后面的字体是其他系统的回退选项。
    plt.rcParams["font.sans-serif"] = [
        "Noto Sans CJK JP",
        "Noto Sans CJK SC",
        "Microsoft YaHei",
        "SimHei",
        "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False

    fig, axis = plt.subplots(figsize=(8, 7))
    vertices = solution.vertices
    polygon_vertices = vertices[:-1]

    if solution.consistent:
        fill_x = [point[0] for point in polygon_vertices]
        fill_y = [point[1] for point in polygon_vertices]
        axis.fill(fill_x, fill_y, color="#66b3ff", alpha=0.20, zorder=1)

    for index, edge in enumerate(solution.edges):
        start = vertices[index]
        end = vertices[index + 1]
        is_unknown_edge = index == solution.unknown_edge
        color = "#d62728" if is_unknown_edge else "#1769aa"
        axis.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color=color,
            linewidth=3.2 if is_unknown_edge else 2.0,
            marker="o",
            markersize=4,
            zorder=3,
        )

        middle_x = (start[0] + end[0]) / 2.0
        middle_y = (start[1] + end[1]) / 2.0
        angle_symbol = "θ" if mode == "direction" else "α"
        angle_value = edge.angle if edge.angle is not None else float("nan")
        label = "边{}  L={:.4g}, {}={:.4g}°".format(
            index + 1, edge.length, angle_symbol, angle_value
        )
        if is_unknown_edge:
            label += "  [x]"
        axis.annotate(
            label,
            (middle_x, middle_y),
            xytext=(0, 8),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=9,
            color=color,
            bbox={
                "boxstyle": "round,pad=0.25",
                "facecolor": "white",
                "edgecolor": color,
                "alpha": 0.85,
            },
            zorder=5,
        )

    for index, (x_coord, y_coord) in enumerate(polygon_vertices):
        axis.scatter([x_coord], [y_coord], color="#111111", s=22, zorder=6)
        axis.annotate(
            "P{}".format(index),
            (x_coord, y_coord),
            xytext=(6, -12),
            textcoords="offset points",
            fontsize=10,
            fontweight="bold",
            zorder=7,
        )

    if not solution.consistent:
        end_x, end_y = vertices[-1]
        start_x, start_y = vertices[0]
        axis.plot(
            [end_x, start_x],
            [end_y, start_y],
            "--",
            color="#ff7f0e",
            linewidth=2,
            label="未闭合残差 = {:.4g}".format(solution.residual),
            zorder=2,
        )
        axis.scatter([end_x], [end_y], marker="x", color="#ff7f0e", s=70, zorder=6)
        axis.annotate(
            "计算终点 P{}".format(len(solution.edges)),
            (end_x, end_y),
            xytext=(6, 8),
            textcoords="offset points",
            color="#ff7f0e",
        )
        axis.legend(loc="best")

    kind_name = "边长" if solution.unknown_kind == "length" else "角度"
    unit = "" if solution.unknown_kind == "length" else "°"
    status = "闭合校验通过" if solution.consistent else "数据不相容"
    axis.set_title(
        "多边形求解：第{}条边{} x={:.6g}{}（{}）".format(
            solution.unknown_edge + 1,
            kind_name,
            solution.value,
            unit,
            status,
        ),
        fontsize=13,
    )
    axis.set_xlabel("x")
    axis.set_ylabel("y")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, linestyle=":", alpha=0.5)
    axis.margins(0.20)
    fig.tight_layout()

    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=180, bbox_inches="tight")
    print("\n图形已保存：{}".format(output_path))

    if show:
        try:
            plt.show()
        except Exception as error:
            print(
                "警告：无法打开绘图窗口，但 PNG 已保存：{}".format(error),
                file=sys.stderr,
            )
    plt.close(fig)
    return output_path


def interactive_input() -> Tuple[str, int, List[str]]:
    print("多边形单未知量求解器")
    print("1: direction——角度是每条边相对 +x 轴的绝对方向角")
    print("2: interior ——角度是每条边终点处的内角")
    choice = input("请选择角度模式 [1/2，默认 1]: ").strip() or "1"
    mode_map = {"1": "direction", "2": "interior",
                "direction": "direction", "interior": "interior"}
    if choice.lower() not in mode_map:
        raise SolveError("角度模式只能选 1、2、direction 或 interior。")
    mode = mode_map[choice.lower()]

    try:
        side_count = int(input("请输入多边形边数: ").strip())
    except ValueError:
        raise SolveError("边数必须是整数。")

    print("请依次输入“边长 角度”，共 {} 组；未知量写 x。".format(side_count))
    print("可以写在同一行，例如四边形：3 0  4 90  x 180  4 270")
    tokens = input("数据: ").replace(",", " ").split()
    return mode, side_count, tokens


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="由多边形已知边长和角度求一个未知量 x"
    )
    parser.add_argument(
        "--mode",
        choices=("direction", "interior"),
        help="direction=边的绝对方向角；interior=边终点处的内角",
    )
    parser.add_argument("--sides", type=int, help="多边形边数")
    parser.add_argument(
        "--data",
        nargs="+",
        help="交替输入边长和角度，必须恰好有一个 x",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_PLOT,
        help="绘图保存路径，默认：{}".format(DEFAULT_PLOT),
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="只保存图片，不弹出绘图窗口",
    )
    return parser


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()
    try:
        supplied = (args.mode is not None, args.sides is not None,
                    args.data is not None)
        if any(supplied) and not all(supplied):
            raise SolveError(
                "命令行模式必须同时提供 --mode、--sides 和 --data。"
            )
        if all(supplied):
            mode = args.mode
            side_count = args.sides
            tokens = args.data
        else:
            mode, side_count, tokens = interactive_input()

        edges = parse_edges(side_count, tokens)
        solution = solve(mode, edges)
        print_solution(solution, mode)
        display_available = bool(
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        )
        show = not args.no_show and display_available
        plot_solution(solution, mode, args.output, show)
        if not display_available and not args.no_show:
            print("当前没有图形显示环境，已仅保存 PNG 文件。")
        return 0 if solution.consistent else 2
    except (SolveError, ValueError) as error:
        print("错误：{}".format(error), file=sys.stderr)
        return 2
    except (EOFError, KeyboardInterrupt):
        print("\n已取消。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
