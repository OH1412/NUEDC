#!/usr/bin/env python3
"""多边形单未知量求解器的简易图形界面。"""

import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import List, Optional, Tuple

from matplotlib import rcParams
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from polygon_solver import Edge, Solution, SolveError, parse_edges, solve


rcParams["font.sans-serif"] = [
    "Noto Sans CJK JP",
    "Noto Sans CJK SC",
    "Microsoft YaHei",
    "SimHei",
    "DejaVu Sans",
]
rcParams["axes.unicode_minus"] = False

DEFAULT_OUTPUT = Path(__file__).resolve().parent / "output" / "polygon.png"


class PolygonSolverApp:
    MODE_LABELS = {
        "每条边的绝对方向角": "direction",
        "每个顶点的内角": "interior",
    }

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("E 题工具——多边形单未知量求解器")
        self.root.geometry("1240x780")
        self.root.minsize(980, 650)

        self.mode_var = tk.StringVar(value="每条边的绝对方向角")
        self.side_count_var = tk.IntVar(value=4)
        self.status_var = tk.StringVar(value="请填写数据，未知量输入 x。")
        self.entries: List[Tuple[ttk.Entry, ttk.Entry]] = []
        self.solution: Optional[Solution] = None
        self.solution_mode = "direction"

        self._build_layout()
        self.rebuild_edge_inputs()
        self.load_example()

    def _build_layout(self) -> None:
        controls = ttk.Frame(self.root, padding=(12, 10))
        controls.pack(fill="x")

        ttk.Label(controls, text="角度类型：").pack(side="left")
        mode_box = ttk.Combobox(
            controls,
            textvariable=self.mode_var,
            values=list(self.MODE_LABELS.keys()),
            state="readonly",
            width=22,
        )
        mode_box.pack(side="left", padx=(0, 14))
        mode_box.bind("<<ComboboxSelected>>", self._update_angle_heading)

        ttk.Label(controls, text="边数：").pack(side="left")
        side_spin = ttk.Spinbox(
            controls,
            from_=3,
            to=20,
            textvariable=self.side_count_var,
            width=5,
        )
        side_spin.pack(side="left", padx=(0, 5))
        ttk.Button(
            controls, text="生成输入框", command=self.rebuild_edge_inputs
        ).pack(side="left", padx=(0, 14))
        ttk.Button(
            controls, text="载入示例", command=self.load_example
        ).pack(side="left", padx=(0, 8))
        ttk.Button(
            controls, text="计算并绘图", command=self.calculate,
            style="Accent.TButton",
        ).pack(side="left", padx=(0, 8))
        ttk.Button(
            controls, text="保存 PNG", command=self.save_plot
        ).pack(side="left")

        ttk.Label(
            controls,
            text="规则：全部输入框中必须恰好有一个 x",
            foreground="#555555",
        ).pack(side="right")

        body = ttk.PanedWindow(self.root, orient="horizontal")
        body.pack(fill="both", expand=True, padx=12, pady=(0, 8))

        left = ttk.Frame(body, width=400)
        right = ttk.Frame(body)
        body.add(left, weight=1)
        body.add(right, weight=3)

        input_group = ttk.LabelFrame(left, text="逐边输入", padding=8)
        input_group.pack(fill="both", expand=False)

        self.input_canvas = tk.Canvas(
            input_group, height=300, highlightthickness=0
        )
        scrollbar = ttk.Scrollbar(
            input_group, orient="vertical", command=self.input_canvas.yview
        )
        self.edge_frame = ttk.Frame(self.input_canvas)
        self.edge_window = self.input_canvas.create_window(
            (0, 0), window=self.edge_frame, anchor="nw"
        )
        self.input_canvas.configure(yscrollcommand=scrollbar.set)
        self.input_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        self.edge_frame.bind("<Configure>", self._sync_scroll_region)
        self.input_canvas.bind("<Configure>", self._resize_edge_frame)

        result_group = ttk.LabelFrame(left, text="计算结果", padding=6)
        result_group.pack(fill="both", expand=True, pady=(10, 0))
        self.result_text = tk.Text(
            result_group,
            height=16,
            width=44,
            wrap="none",
            font=("Noto Sans Mono CJK SC", 10),
            state="disabled",
        )
        result_y_scroll = ttk.Scrollbar(
            result_group, orient="vertical", command=self.result_text.yview
        )
        result_x_scroll = ttk.Scrollbar(
            result_group, orient="horizontal", command=self.result_text.xview
        )
        self.result_text.configure(
            yscrollcommand=result_y_scroll.set,
            xscrollcommand=result_x_scroll.set,
        )
        self.result_text.grid(row=0, column=0, sticky="nsew")
        result_y_scroll.grid(row=0, column=1, sticky="ns")
        result_x_scroll.grid(row=1, column=0, sticky="ew")
        result_group.rowconfigure(0, weight=1)
        result_group.columnconfigure(0, weight=1)

        plot_group = ttk.LabelFrame(right, text="多边形图形", padding=6)
        plot_group.pack(fill="both", expand=True)
        self.figure = Figure(figsize=(8, 7), dpi=100)
        self.axis = self.figure.add_subplot(111)
        self.axis.set_title("输入数据后点击“计算并绘图”")
        self.axis.set_aspect("equal", adjustable="datalim")
        self.axis.grid(True, linestyle=":", alpha=0.5)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_group)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.draw()

        status = ttk.Label(
            self.root,
            textvariable=self.status_var,
            anchor="w",
            relief="sunken",
            padding=(8, 4),
        )
        status.pack(fill="x", side="bottom")

    def _sync_scroll_region(self, _event: tk.Event) -> None:
        self.input_canvas.configure(
            scrollregion=self.input_canvas.bbox("all")
        )

    def _resize_edge_frame(self, event: tk.Event) -> None:
        self.input_canvas.itemconfigure(self.edge_window, width=event.width)

    def _update_angle_heading(self, _event: Optional[tk.Event] = None) -> None:
        mode = self.MODE_LABELS[self.mode_var.get()]
        heading = "方向角 (°)" if mode == "direction" else "内角 (°)"
        if hasattr(self, "angle_heading"):
            self.angle_heading.configure(text=heading)

    def rebuild_edge_inputs(self) -> None:
        try:
            side_count = int(self.side_count_var.get())
        except (ValueError, tk.TclError):
            messagebox.showerror("输入错误", "边数必须是 3～20 的整数。")
            return
        if side_count < 3 or side_count > 20:
            messagebox.showerror("输入错误", "边数必须在 3～20 之间。")
            return

        old_values = [
            (length.get(), angle.get()) for length, angle in self.entries
        ]
        for child in self.edge_frame.winfo_children():
            child.destroy()
        self.entries = []

        ttk.Label(self.edge_frame, text="边").grid(
            row=0, column=0, padx=5, pady=4
        )
        ttk.Label(self.edge_frame, text="边长").grid(
            row=0, column=1, padx=5, pady=4
        )
        self.angle_heading = ttk.Label(self.edge_frame)
        self.angle_heading.grid(row=0, column=2, padx=5, pady=4)
        self._update_angle_heading()

        for index in range(side_count):
            ttk.Label(
                self.edge_frame, text=str(index + 1), width=4, anchor="center"
            ).grid(row=index + 1, column=0, padx=4, pady=3)
            length_entry = ttk.Entry(self.edge_frame, width=15)
            angle_entry = ttk.Entry(self.edge_frame, width=15)
            length_entry.grid(
                row=index + 1, column=1, padx=4, pady=3, sticky="ew"
            )
            angle_entry.grid(
                row=index + 1, column=2, padx=4, pady=3, sticky="ew"
            )
            if index < len(old_values):
                length_entry.insert(0, old_values[index][0])
                angle_entry.insert(0, old_values[index][1])
            self.entries.append((length_entry, angle_entry))

        self.edge_frame.columnconfigure(1, weight=1)
        self.edge_frame.columnconfigure(2, weight=1)
        self.solution = None
        self.status_var.set("输入框已生成；未知量请填写 x。")

    def load_example(self) -> None:
        self.mode_var.set("每条边的绝对方向角")
        self.side_count_var.set(4)
        self.rebuild_edge_inputs()
        values = [("3", "0"), ("4", "90"), ("x", "180"), ("4", "270")]
        for entries, values_for_edge in zip(self.entries, values):
            for entry, value in zip(entries, values_for_edge):
                entry.delete(0, "end")
                entry.insert(0, value)
        self.status_var.set("已载入示例：第三条边长为未知量 x。")

    def _tokens_from_entries(self) -> List[str]:
        tokens = []
        for index, (length_entry, angle_entry) in enumerate(self.entries):
            length = length_entry.get().strip()
            angle = angle_entry.get().strip()
            if not length or not angle:
                raise SolveError("第 {} 条边有空输入框。".format(index + 1))
            tokens.extend((length, angle))
        return tokens

    def calculate(self) -> None:
        for length_entry, angle_entry in self.entries:
            length_entry.configure(style="TEntry")
            angle_entry.configure(style="TEntry")

        try:
            side_count = len(self.entries)
            tokens = self._tokens_from_entries()
            edges = parse_edges(side_count, tokens)
            mode = self.MODE_LABELS[self.mode_var.get()]
            solution = solve(mode, edges)
        except (SolveError, ValueError) as error:
            self.solution = None
            self.status_var.set("计算失败：{}".format(error))
            messagebox.showerror("计算失败", str(error))
            return

        for entry_pair in self.entries:
            for entry in entry_pair:
                if entry.get().strip().lower() == "x":
                    entry.configure(style="Unknown.TEntry")

        self.solution = solution
        self.solution_mode = mode
        self._show_result_text(solution, mode)
        self._draw_solution(solution, mode)
        if solution.consistent:
            self.status_var.set(
                "计算完成：x = {:.10g}，闭合校验通过。".format(solution.value)
            )
        else:
            self.status_var.set(
                "已得到候选 x，但数据不相容；请查看橙色闭合残差。"
            )

    def _show_result_text(self, solution: Solution, mode: str) -> None:
        kind_name = "边长" if solution.unknown_kind == "length" else "角度"
        unit = "" if solution.unknown_kind == "length" else "°"
        lines = [
            "x 位于第 {} 条边的{}".format(
                solution.unknown_edge + 1, kind_name
            ),
            "x = {:.10g}{}".format(solution.value, unit),
            "",
            solution.note,
            "闭合残差 = {:.9g}".format(solution.residual),
            "闭合容差 = {:.9g}".format(solution.tolerance),
            "校验 = {}".format("通过" if solution.consistent else "未通过"),
            "",
            "完整边数据",
        ]
        angle_name = "方向角" if mode == "direction" else "内角"
        for index, edge in enumerate(solution.edges):
            lines.append(
                "边 {}: 长度={:.8g}, {}={:.8g}°".format(
                    index + 1, edge.length, angle_name, edge.angle
                )
            )
        lines.extend(("", "重建顶点"))
        for index, (x_coord, y_coord) in enumerate(solution.vertices):
            lines.append(
                "P{} = ({:.8g}, {:.8g})".format(index, x_coord, y_coord)
            )

        self.result_text.configure(state="normal")
        self.result_text.delete("1.0", "end")
        self.result_text.insert("1.0", "\n".join(lines))
        self.result_text.configure(state="disabled")

    def _draw_solution(self, solution: Solution, mode: str) -> None:
        axis = self.axis
        axis.clear()
        vertices = solution.vertices
        polygon_vertices = vertices[:-1]

        if solution.consistent:
            axis.fill(
                [point[0] for point in polygon_vertices],
                [point[1] for point in polygon_vertices],
                color="#66b3ff",
                alpha=0.20,
            )

        for index, edge in enumerate(solution.edges):
            start = vertices[index]
            end = vertices[index + 1]
            is_unknown = index == solution.unknown_edge
            color = "#d62728" if is_unknown else "#1769aa"
            axis.plot(
                [start[0], end[0]],
                [start[1], end[1]],
                color=color,
                linewidth=3.2 if is_unknown else 2.0,
                marker="o",
                markersize=4,
            )
            middle = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
            symbol = "θ" if mode == "direction" else "α"
            label = "边{}  L={:.4g}, {}={:.4g}°{}".format(
                index + 1,
                edge.length,
                symbol,
                edge.angle,
                " [x]" if is_unknown else "",
            )
            axis.annotate(
                label,
                middle,
                xytext=(0, 8),
                textcoords="offset points",
                ha="center",
                fontsize=9,
                color=color,
                bbox={
                    "boxstyle": "round,pad=0.2",
                    "facecolor": "white",
                    "edgecolor": color,
                    "alpha": 0.85,
                },
            )

        for index, point in enumerate(polygon_vertices):
            axis.annotate(
                "P{}".format(index),
                point,
                xytext=(6, -12),
                textcoords="offset points",
                fontweight="bold",
            )

        if not solution.consistent:
            end = vertices[-1]
            start = vertices[0]
            axis.plot(
                [end[0], start[0]],
                [end[1], start[1]],
                "--",
                color="#ff7f0e",
                linewidth=2,
                label="未闭合残差={:.4g}".format(solution.residual),
            )
            axis.scatter(
                [end[0]], [end[1]], marker="x", color="#ff7f0e", s=70
            )
            axis.legend(loc="best")

        kind_name = "边长" if solution.unknown_kind == "length" else "角度"
        unit = "" if solution.unknown_kind == "length" else "°"
        axis.set_title(
            "第{}条边{}：x={:.6g}{}（{}）".format(
                solution.unknown_edge + 1,
                kind_name,
                solution.value,
                unit,
                "闭合" if solution.consistent else "不相容",
            )
        )
        axis.set_xlabel("x")
        axis.set_ylabel("y")
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(True, linestyle=":", alpha=0.5)
        axis.margins(0.20)
        self.figure.tight_layout()
        self.canvas.draw_idle()

    def save_plot(self) -> None:
        if self.solution is None:
            messagebox.showinfo("尚无图形", "请先点击“计算并绘图”。")
            return
        DEFAULT_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        selected = filedialog.asksaveasfilename(
            title="保存多边形图形",
            initialdir=str(DEFAULT_OUTPUT.parent),
            initialfile=DEFAULT_OUTPUT.name,
            defaultextension=".png",
            filetypes=[("PNG 图片", "*.png"), ("所有文件", "*.*")],
        )
        if not selected:
            return
        try:
            self.figure.savefig(selected, dpi=180, bbox_inches="tight")
        except OSError as error:
            messagebox.showerror("保存失败", str(error))
            return
        self.status_var.set("图形已保存：{}".format(Path(selected).resolve()))


def main() -> int:
    root = tk.Tk()
    style = ttk.Style(root)
    style.configure("Unknown.TEntry", fieldbackground="#ffe4e1")
    if "clam" in style.theme_names():
        style.theme_use("clam")
        style.configure("Unknown.TEntry", fieldbackground="#ffe4e1")
        style.configure("Accent.TButton", font=("", 10, "bold"))
    PolygonSolverApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
