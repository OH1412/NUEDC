#!/usr/bin/env python3
"""独立进程实时曲线窗口；绘图拥塞不能阻塞视觉控制线程。"""

from collections import deque
import math
import multiprocessing
from queue import Empty, Full
from typing import Any, Deque, Dict, Iterable, List, Optional, Tuple


PLOT_WINDOW_S = 30.0
MAX_SAMPLES = 900


def axis_limits(values: Iterable[float], minimum_span: float) -> Tuple[float, float]:
    """生成带余量且不会退化为零高度的纵轴范围。"""

    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        half = max(float(minimum_span), 1e-6) / 2.0
        return -half, half
    low = min(finite)
    high = max(finite)
    span = max(high - low, float(minimum_span))
    center = (low + high) / 2.0
    half = span * 0.6
    return center - half, center + half


def _curve_process_main(sample_queue: Any, control_mode: str) -> None:
    import tkinter as tk
    from tkinter import font as tkfont
    from tkinter import ttk

    root = tk.Tk()
    root.title("H题钢珠控制实时曲线")
    root.geometry("1100x700")
    root.minsize(820, 520)
    root.configure(background="#eef2f6")

    cjk_font = "Noto Sans CJK SC"
    for font_name in ("TkDefaultFont", "TkTextFont", "TkMenuFont"):
        tkfont.nametofont(font_name).configure(family=cjk_font, size=11)

    header = tk.Frame(root, background="#174f7d", padx=16, pady=10)
    header.pack(fill="x")
    tk.Label(
        header,
        text="钢珠闭环实时曲线",
        background="#174f7d",
        foreground="#ffffff",
        font=(cjk_font, 17, "bold"),
    ).pack(side="left")
    mode_text = "速度环独立模式" if control_mode == "velocity" else "位置闭环模式"
    tk.Label(
        header,
        text=mode_text + " · 最近30秒 · 视觉新帧更新",
        background="#174f7d",
        foreground="#dcecff",
        font=(cjk_font, 11),
    ).pack(side="right")

    notebook = ttk.Notebook(root)
    notebook.pack(fill="both", expand=True, padx=12, pady=12)

    samples: Deque[Dict[str, Optional[float]]] = deque(maxlen=MAX_SAMPLES)
    origin_s: Optional[float] = None
    canvases: Dict[str, tk.Canvas] = {}

    def add_tab(key: str, title: str) -> None:
        frame = ttk.Frame(notebook)
        canvas = tk.Canvas(
            frame,
            background="#111820",
            highlightthickness=0,
        )
        canvas.pack(fill="both", expand=True)
        notebook.add(frame, text=title)
        canvases[key] = canvas

    add_tab("velocity", "速度：目标 / 实际")
    if control_mode == "position":
        add_tab("position", "位置：目标 / 实际")

    def draw_chart(
        canvas: tk.Canvas,
        target_key: str,
        actual_key: str,
        unit: str,
        minimum_span: float,
    ) -> None:
        canvas.delete("all")
        width = max(canvas.winfo_width(), 320)
        height = max(canvas.winfo_height(), 240)
        left, right, top, bottom = 76, 22, 28, 50
        plot_w = max(1, width - left - right)
        plot_h = max(1, height - top - bottom)
        if not samples or origin_s is None:
            canvas.create_text(
                width / 2,
                height / 2,
                text="等待有效视觉数据……",
                fill="#aab8c5",
                font=(cjk_font, 15),
            )
            return

        newest = float(samples[-1]["time_s"] or origin_s) - origin_s
        x_max = max(PLOT_WINDOW_S, newest)
        x_min = max(0.0, x_max - PLOT_WINDOW_S)
        visible = [
            sample
            for sample in samples
            if float(sample["time_s"] or origin_s) - origin_s >= x_min
        ]
        y_values: List[float] = []
        for sample in visible:
            for key in (target_key, actual_key):
                value = sample.get(key)
                if value is not None and math.isfinite(float(value)):
                    y_values.append(float(value))
        y_min, y_max = axis_limits(y_values, minimum_span)

        for index in range(6):
            fraction = index / 5.0
            x = left + fraction * plot_w
            y = top + fraction * plot_h
            canvas.create_line(x, top, x, top + plot_h, fill="#293643")
            canvas.create_line(left, y, left + plot_w, y, fill="#293643")
            canvas.create_text(
                x,
                top + plot_h + 22,
                text="{:.0f}".format(x_min + fraction * (x_max - x_min)),
                fill="#9aabba",
                font=(cjk_font, 9),
            )
            canvas.create_text(
                left - 10,
                top + plot_h - fraction * plot_h,
                text="{:.2f}".format(y_min + fraction * (y_max - y_min)),
                fill="#9aabba",
                anchor="e",
                font=(cjk_font, 9),
            )

        canvas.create_rectangle(
            left, top, left + plot_w, top + plot_h, outline="#607080"
        )
        canvas.create_text(
            left + plot_w / 2,
            height - 12,
            text="时间 / s",
            fill="#c5d0da",
            font=(cjk_font, 10),
        )
        canvas.create_text(
            12,
            top - 12,
            text=unit,
            fill="#c5d0da",
            anchor="w",
            font=(cjk_font, 10),
        )

        def points_for(key: str) -> List[float]:
            points: List[float] = []
            for sample in visible:
                value = sample.get(key)
                if value is None or not math.isfinite(float(value)):
                    continue
                elapsed = float(sample["time_s"] or origin_s) - origin_s
                x = left + (elapsed - x_min) / max(x_max - x_min, 1e-9) * plot_w
                y = top + (y_max - float(value)) / max(y_max - y_min, 1e-9) * plot_h
                points.extend((x, y))
            return points

        target_points = points_for(target_key)
        actual_points = points_for(actual_key)
        if len(target_points) >= 4:
            canvas.create_line(*target_points, fill="#ffb020", width=2)
        if len(actual_points) >= 4:
            canvas.create_line(*actual_points, fill="#28c8ff", width=2)

        latest_target = samples[-1].get(target_key)
        latest_actual = samples[-1].get(actual_key)
        canvas.create_line(left + 12, top + 12, left + 42, top + 12, fill="#ffb020", width=3)
        canvas.create_text(
            left + 48,
            top + 12,
            text="目标 {}".format("--" if latest_target is None else "{:.3f}".format(latest_target)),
            fill="#ffcf70",
            anchor="w",
            font=(cjk_font, 11, "bold"),
        )
        canvas.create_line(left + 190, top + 12, left + 220, top + 12, fill="#28c8ff", width=3)
        canvas.create_text(
            left + 226,
            top + 12,
            text="实际 {}".format("--" if latest_actual is None else "{:.3f}".format(latest_actual)),
            fill="#7edfff",
            anchor="w",
            font=(cjk_font, 11, "bold"),
        )

    def poll_and_draw() -> None:
        nonlocal origin_s
        try:
            while True:
                message = sample_queue.get_nowait()
                if message.get("type") == "shutdown":
                    root.destroy()
                    return
                if message.get("type") == "sample":
                    sample = dict(message["sample"])
                    if origin_s is None:
                        origin_s = float(sample["time_s"])
                    samples.append(sample)
        except Empty:
            pass
        draw_chart(
            canvases["velocity"],
            "target_velocity_cm_s",
            "velocity_cm_s",
            "速度 / cm/s",
            1.0,
        )
        if control_mode == "position":
            draw_chart(
                canvases["position"],
                "target_position_cm",
                "position_cm",
                "位置 / cm",
                2.0,
            )
        root.after(100, poll_and_draw)

    root.after(100, poll_and_draw)
    root.mainloop()


class ControlCurveUI:
    """父进程非阻塞绘图接口；队列满时丢最旧绘图点。"""

    def __init__(self, control_mode: str) -> None:
        if control_mode not in ("position", "velocity"):
            raise ValueError("曲线窗口模式必须是position或velocity。")
        context = multiprocessing.get_context("spawn")
        self._queue = context.Queue(maxsize=64)
        self._process = context.Process(
            target=_curve_process_main,
            args=(self._queue, control_mode),
            name="ball-control-curve-ui",
            daemon=True,
        )

    def start(self) -> None:
        self._process.start()

    def offer(
        self,
        time_s: float,
        target_velocity_cm_s: float,
        velocity_cm_s: float,
        target_position_cm: Optional[float],
        position_cm: Optional[float],
    ) -> None:
        sample = {
            "time_s": float(time_s),
            "target_velocity_cm_s": float(target_velocity_cm_s),
            "velocity_cm_s": float(velocity_cm_s),
            "target_position_cm": (
                None if target_position_cm is None else float(target_position_cm)
            ),
            "position_cm": None if position_cm is None else float(position_cm),
        }
        message = {"type": "sample", "sample": sample}
        try:
            self._queue.put_nowait(message)
        except Full:
            try:
                self._queue.get_nowait()
            except Empty:
                pass
            try:
                self._queue.put_nowait(message)
            except Full:
                pass

    def close(self, timeout_s: float = 1.0) -> None:
        try:
            self._queue.put_nowait({"type": "shutdown"})
        except Full:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait({"type": "shutdown"})
            except (Empty, Full):
                pass
        self._process.join(timeout=max(0.0, timeout_s))
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=0.5)
        self._queue.close()
