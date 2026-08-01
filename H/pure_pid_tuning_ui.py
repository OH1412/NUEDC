#!/usr/bin/env python3
"""纯串级PID的独立进程实时调参界面。"""

from dataclasses import dataclass
import math
import multiprocessing
from multiprocessing.connection import Connection
from typing import Any, Dict, List, Mapping, Optional, Tuple


@dataclass(frozen=True)
class PureParameterSpec:
    key: str
    label: str
    minimum: float
    maximum: float
    increment: float
    unit: str


PURE_PARAMETER_SPECS: Tuple[PureParameterSpec, ...] = (
    PureParameterSpec("position_kp_s_inv", "位置环 Kp", 0.0, 10.0, 0.01, "1/s"),
    PureParameterSpec("position_ki_s2_inv", "位置环 Ki", 0.0, 10.0, 0.01, "1/s²"),
    PureParameterSpec("position_kd", "位置环 Kd", 0.0, 10.0, 0.01, "无量纲"),
    PureParameterSpec("velocity_kp_deg_per_m_s", "速度环 Kp", 0.0, 500.0, 0.5, "deg/(m/s)"),
    PureParameterSpec("velocity_ki_deg_per_m", "速度环 Ki", 0.0, 500.0, 0.5, "deg/m"),
    PureParameterSpec("velocity_kd", "速度环 Kd", 0.0, 100.0, 0.01, "deg/(m/s²)"),
)


def validate_pure_values(values: Mapping[str, Any]) -> Dict[str, float]:
    result: Dict[str, float] = {}
    actual = set(values)
    required = {spec.key for spec in PURE_PARAMETER_SPECS}
    if actual != required:
        raise ValueError("纯PID必须且只能包含六个参数。")
    for spec in PURE_PARAMETER_SPECS:
        value = float(values[spec.key])
        if not math.isfinite(value):
            raise ValueError("{}必须是有限数。".format(spec.label))
        if not spec.minimum <= value <= spec.maximum:
            raise ValueError(
                "{}必须在{}到{}之间。".format(
                    spec.label, spec.minimum, spec.maximum
                )
            )
        result[spec.key] = value
    return result


def _ui_process_main(
    connection: Connection,
    initial_values: Dict[str, float],
    initial_target_cm: float,
    target_min_cm: float,
    target_max_cm: float,
    profile_names: List[str],
    active_profile: Optional[str],
) -> None:
    try:
        import tkinter as tk
        from tkinter import font as tkfont
        from tkinter import messagebox, simpledialog, ttk

        root = tk.Tk()
        root.title("H题纯串级PID实时调参")
        root.geometry("1120x720")
        root.minsize(900, 620)
        root.configure(background="#eef2f6")

        cjk_font = "Noto Sans CJK SC"
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont"):
            tkfont.nametofont(name).configure(family=cjk_font, size=12)
        tkfont.nametofont("TkHeadingFont").configure(
            family=cjk_font, size=13, weight="bold"
        )
        style = ttk.Style(root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background="#eef2f6")
        style.configure("Card.TFrame", background="#ffffff")
        style.configure("TLabel", background="#eef2f6", foreground="#17212b")
        style.configure("Card.TLabel", background="#ffffff")
        style.configure(
            "Section.TLabel",
            background="#dce8f5",
            foreground="#164b78",
            font=(cjk_font, 13, "bold"),
            padding=(12, 8),
        )
        style.configure("TLabelframe", background="#eef2f6", padding=8)
        style.configure(
            "TLabelframe.Label",
            background="#eef2f6",
            foreground="#164b78",
            font=(cjk_font, 13, "bold"),
        )
        style.configure("TButton", font=(cjk_font, 12, "bold"), padding=(12, 8))
        style.configure("Primary.TButton", foreground="#ffffff", background="#1769aa")
        style.map("Primary.TButton", background=[("active", "#0f568d")])
        style.configure("TSpinbox", arrowsize=18, padding=5)
        style.configure("Horizontal.TScale", sliderlength=28, troughcolor="#d8e0e8")

        header = tk.Frame(root, background="#174f7d", padx=18, pady=12)
        header.pack(fill="x")
        tk.Label(
            header,
            text="H题纯串级 PID 实时调参",
            background="#174f7d",
            foreground="#ffffff",
            font=(cjk_font, 18, "bold"),
        ).pack(anchor="w")
        tk.Label(
            header,
            text=(
                "只有位置P/I/D和速度P/I/D参与控制。滑条、输入框和上下箭头联动；"
                "点击应用立即生效；参数文件可另存、选择应用和重命名。"
            ),
            background="#174f7d",
            foreground="#e9f3fb",
            font=(cjk_font, 12),
            wraplength=1060,
        ).pack(fill="x", anchor="w", pady=(4, 0))

        status = tk.StringVar(value="窗口已就绪；当前显示启动时参数。")
        profile_bar = ttk.LabelFrame(root, text="纯PID参数文件")
        profile_bar.pack(fill="x", padx=16, pady=(12, 8))
        selected_profile = tk.StringVar(value=active_profile or "")
        profile_combo = ttk.Combobox(
            profile_bar,
            textvariable=selected_profile,
            values=profile_names,
            state="readonly",
            width=28,
        )
        profile_combo.pack(side="left", padx=8, pady=8)
        active_text = tk.StringVar(
            value="当前默认：{}".format(active_profile or "未选择")
        )
        ttk.Label(profile_bar, textvariable=active_text).pack(side="right", padx=8)

        target_bar = ttk.LabelFrame(root, text="运行目标点（不写入PID参数文件）")
        target_bar.pack(fill="x", padx=16, pady=(0, 8))
        target_text = tk.StringVar(value="{:.2f}".format(initial_target_cm))
        target_slider = tk.DoubleVar(value=initial_target_cm)

        def target_scale_changed(raw: str) -> None:
            value = round(float(raw) * 10.0) / 10.0
            target_text.set("{:.2f}".format(value))

        def normalize_target() -> Optional[float]:
            try:
                value = float(target_text.get())
            except ValueError:
                status.set("目标点必须是数字。")
                return None
            if not math.isfinite(value) or not target_min_cm <= value <= target_max_cm:
                status.set(
                    "目标点必须在{:+.1f}到{:+.1f}cm之间。".format(
                        target_min_cm, target_max_cm
                    )
                )
                return None
            target_text.set("{:.2f}".format(value))
            target_slider.set(value)
            return value

        def apply_target() -> None:
            value = normalize_target()
            if value is None:
                return
            try:
                connection.send({"type": "setpoint", "value": value})
                status.set("目标点已提交，等待控制进程确认……")
            except (BrokenPipeError, EOFError, OSError):
                status.set("控制进程已经结束。")

        ttk.Label(target_bar, text="目标点", width=9).pack(side="left", padx=(8, 4), pady=8)
        ttk.Scale(
            target_bar,
            from_=target_min_cm,
            to=target_max_cm,
            variable=target_slider,
            orient="horizontal",
            command=target_scale_changed,
        ).pack(side="left", fill="x", expand=True, padx=8, pady=8)
        target_spin = ttk.Spinbox(
            target_bar,
            from_=target_min_cm,
            to=target_max_cm,
            increment=0.1,
            textvariable=target_text,
            width=10,
            command=normalize_target,
        )
        target_spin.pack(side="left", padx=8, pady=8)
        target_spin.bind("<Return>", lambda _event: apply_target())
        target_spin.bind("<FocusOut>", lambda _event: normalize_target())
        ttk.Label(target_bar, text="cm").pack(side="left", padx=4)
        ttk.Button(
            target_bar,
            text="应用目标点",
            command=apply_target,
            style="Primary.TButton",
        ).pack(side="left", padx=12, pady=6)

        card = ttk.Frame(root, style="Card.TFrame")
        card.pack(fill="both", expand=True, padx=16, pady=(0, 8))
        variables: Dict[str, tk.StringVar] = {}
        sliders: Dict[str, tk.DoubleVar] = {}
        specs = {spec.key: spec for spec in PURE_PARAMETER_SPECS}

        def decimals(step: float) -> int:
            text = "{:.8f}".format(step).rstrip("0")
            return len(text.split(".", 1)[1]) if "." in text else 0

        def formatted(spec: PureParameterSpec, value: float) -> str:
            return ("{:.%df}" % decimals(spec.increment)).format(value)

        def set_value(key: str, raw: Any) -> None:
            spec = specs[key]
            value = max(spec.minimum, min(spec.maximum, float(raw)))
            variables[key].set(formatted(spec, value))
            sliders[key].set(value)

        def scale_changed(key: str, raw: str) -> None:
            spec = specs[key]
            value = float(raw)
            steps = round((value - spec.minimum) / spec.increment)
            value = spec.minimum + steps * spec.increment
            variables[key].set(formatted(spec, value))

        def entry_changed(key: str) -> None:
            try:
                set_value(key, variables[key].get())
            except ValueError:
                status.set("{}必须是数字。".format(specs[key].label))

        row = 0
        for index, spec in enumerate(PURE_PARAMETER_SPECS):
            if index in (0, 3):
                ttk.Label(
                    card,
                    text="位置环：位置误差 → 目标速度" if index == 0 else "速度环：速度误差 → 倾角",
                    style="Section.TLabel",
                ).grid(row=row, column=0, columnspan=4, sticky="ew", padx=8, pady=(8, 5))
                row += 1
            ttk.Label(card, text=spec.label, width=20, style="Card.TLabel").grid(
                row=row, column=0, sticky="w", padx=(16, 8), pady=11
            )
            variables[spec.key] = tk.StringVar(value=formatted(spec, initial_values[spec.key]))
            sliders[spec.key] = tk.DoubleVar(value=initial_values[spec.key])
            ttk.Scale(
                card,
                from_=spec.minimum,
                to=spec.maximum,
                variable=sliders[spec.key],
                orient="horizontal",
                command=lambda raw, key=spec.key: scale_changed(key, raw),
            ).grid(row=row, column=1, sticky="ew", padx=10, pady=11)
            spin = ttk.Spinbox(
                card,
                from_=spec.minimum,
                to=spec.maximum,
                increment=spec.increment,
                textvariable=variables[spec.key],
                width=14,
                command=lambda key=spec.key: entry_changed(key),
            )
            spin.grid(row=row, column=2, padx=8, pady=11)
            spin.bind("<Return>", lambda _event, key=spec.key: entry_changed(key))
            spin.bind("<FocusOut>", lambda _event, key=spec.key: entry_changed(key))
            ttk.Label(
                card,
                text="{}  [{} 到 {}]".format(spec.unit, spec.minimum, spec.maximum),
                style="Card.TLabel",
                foreground="#555555",
            ).grid(row=row, column=3, sticky="w", padx=(6, 16), pady=11)
            row += 1
        card.columnconfigure(1, weight=1)

        def collect() -> Optional[Dict[str, float]]:
            try:
                return validate_pure_values(
                    {key: variable.get() for key, variable in variables.items()}
                )
            except (TypeError, ValueError) as error:
                status.set("参数错误：{}".format(error))
                return None

        def send_values(message_type: str) -> None:
            values = collect()
            if values is None:
                return
            try:
                connection.send({"type": message_type, "values": values})
                status.set(
                    "正在覆盖JSON并实时应用……"
                    if message_type == "save"
                    else "参数已提交，等待实时应用……"
                )
            except (BrokenPipeError, EOFError, OSError):
                status.set("控制进程已经结束。")

        def save_as_profile() -> None:
            values = collect()
            if values is None:
                return
            name = simpledialog.askstring(
                "保存纯PID参数方案",
                "请输入参数文件名称（无需输入.json）：",
                parent=root,
            )
            if name is None:
                return
            normalized = name.strip()
            if normalized.lower().endswith(".json"):
                normalized = normalized[:-5].strip()
            if normalized in profile_combo["values"] and not messagebox.askyesno(
                "覆盖确认",
                "参数文件已经存在，是否覆盖并设为默认？",
                parent=root,
            ):
                return
            try:
                connection.send(
                    {"type": "save_profile", "name": normalized, "values": values}
                )
                status.set("正在保存并应用纯PID参数方案……")
            except (BrokenPipeError, EOFError, OSError):
                status.set("控制进程已经结束。")

        def load_selected_profile() -> None:
            name = selected_profile.get().strip()
            if not name:
                status.set("请先选择一个纯PID参数文件。")
                return
            try:
                connection.send({"type": "load_profile", "name": name})
                status.set("正在加载并实时应用参数方案……")
            except (BrokenPipeError, EOFError, OSError):
                status.set("控制进程已经结束。")

        def rename_selected_profile() -> None:
            old_name = selected_profile.get().strip()
            if not old_name:
                status.set("请先选择需要重命名的参数文件。")
                return
            new_name = simpledialog.askstring(
                "重命名纯PID参数方案",
                "请输入新名称（无需输入.json）：",
                initialvalue=old_name,
                parent=root,
            )
            if new_name is None:
                return
            try:
                connection.send(
                    {
                        "type": "rename_profile",
                        "old_name": old_name,
                        "new_name": new_name.strip(),
                    }
                )
                status.set("正在重命名参数方案……")
            except (BrokenPipeError, EOFError, OSError):
                status.set("控制进程已经结束。")

        ttk.Button(
            profile_bar,
            text="选择并应用",
            command=load_selected_profile,
            style="Primary.TButton",
        ).pack(side="left", padx=4, pady=6)
        ttk.Button(
            profile_bar, text="当前参数另存为", command=save_as_profile
        ).pack(side="left", padx=4, pady=6)
        ttk.Button(
            profile_bar, text="重命名所选", command=rename_selected_profile
        ).pack(side="left", padx=4, pady=6)

        ttk.Label(root, textvariable=status, font=(cjk_font, 12, "bold")).pack(
            fill="x", padx=18, pady=(4, 4)
        )
        buttons = ttk.Frame(root)
        buttons.pack(fill="x", padx=16, pady=(4, 14))
        ttk.Button(
            buttons,
            text="应用到当前控制器（不保存）",
            command=lambda: send_values("parameters"),
            style="Primary.TButton",
        ).pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="关闭窗口", command=root.destroy).pack(side="right")

        def poll_parent() -> None:
            try:
                while connection.poll():
                    message = connection.recv()
                    if message.get("type") == "shutdown":
                        root.destroy()
                        return
                    if message.get("type") == "status":
                        status.set(str(message.get("message", "")))
                    if message.get("type") == "setpoint_value":
                        value = float(message["value"])
                        target_text.set("{:.2f}".format(value))
                        target_slider.set(value)
                    if message.get("type") == "profile_list":
                        names = list(message.get("profiles", []))
                        profile_combo.configure(values=names)
                        active = message.get("active_profile")
                        if active:
                            selected_profile.set(str(active))
                        active_text.set("当前默认：{}".format(active or "未选择"))
                    if message.get("type") == "load_values":
                        values = message.get("values", {})
                        for key, value in values.items():
                            if key in variables:
                                set_value(key, value)
            except (EOFError, OSError):
                root.destroy()
                return
            root.after(100, poll_parent)

        root.after(100, poll_parent)
        root.mainloop()
    except Exception as error:
        try:
            connection.send({"type": "error", "message": str(error)})
        except Exception:
            pass
    finally:
        try:
            connection.close()
        except Exception:
            pass


class PurePIDTuningUI:
    def __init__(
        self,
        initial_values: Mapping[str, Any],
        initial_target_cm: float,
        target_min_cm: float,
        target_max_cm: float,
        profile_names: Optional[List[str]] = None,
        active_profile: Optional[str] = None,
    ) -> None:
        values = validate_pure_values(initial_values)
        context = multiprocessing.get_context("spawn")
        self._connection, child = context.Pipe(duplex=True)
        self._process = context.Process(
            target=_ui_process_main,
            args=(
                child,
                values,
                initial_target_cm,
                target_min_cm,
                target_max_cm,
                list(profile_names or []),
                active_profile,
            ),
            name="pure-pid-tuning-ui",
            daemon=True,
        )

    def start(self) -> None:
        self._process.start()

    def poll(self) -> List[Dict[str, Any]]:
        messages: List[Dict[str, Any]] = []
        try:
            while self._connection.poll():
                message = self._connection.recv()
                if isinstance(message, dict):
                    messages.append(message)
        except (EOFError, OSError):
            pass
        return messages

    def set_status(self, message: str) -> None:
        try:
            self._connection.send({"type": "status", "message": message})
        except (BrokenPipeError, EOFError, OSError):
            pass

    def set_profiles(
        self,
        profile_names: List[str],
        active_profile: Optional[str],
        values: Optional[Mapping[str, Any]] = None,
    ) -> None:
        try:
            self._connection.send(
                {
                    "type": "profile_list",
                    "profiles": list(profile_names),
                    "active_profile": active_profile,
                }
            )
            if values is not None:
                self._connection.send(
                    {"type": "load_values", "values": dict(values)}
                )
        except (BrokenPipeError, EOFError, OSError):
            pass

    def close(self, timeout_s: float = 1.0) -> None:
        try:
            if self._process.is_alive():
                self._connection.send({"type": "shutdown"})
        except (BrokenPipeError, EOFError, OSError):
            pass
        self._process.join(timeout=max(0.0, timeout_s))
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=0.5)
        try:
            self._connection.close()
        except Exception:
            pass
