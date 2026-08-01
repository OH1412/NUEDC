#!/usr/bin/env python3
"""独立进程Tk界面：实时编辑串级PID参数，不阻塞控制循环。"""

from dataclasses import dataclass
import math
import multiprocessing
from multiprocessing.connection import Connection
from typing import Any, Dict, List, Mapping, Optional, Tuple


@dataclass(frozen=True)
class ParameterSpec:
    key: str
    label: str
    minimum: float
    maximum: float
    increment: float
    unit: str = ""


PARAMETER_SPECS: Tuple[ParameterSpec, ...] = (
    ParameterSpec(
        "working_angle_limit_deg", "工作倾角绝对值上限", 0.1, 21.8, 0.1, "deg"
    ),
    ParameterSpec(
        "max_angle_step_deg", "每个视觉帧最大角度变化", 0.01, 5.0, 0.01, "deg"
    ),
    ParameterSpec(
        "motor_displacement_scale", "倾角到电机升降比例", 0.1, 2.0, 0.01, "x"
    ),
    ParameterSpec(
        "velocity_filter_time_constant_s",
        "速度低通时间常数",
        0.02,
        1.0,
        0.01,
        "s",
    ),
    ParameterSpec(
        "equilibrium_angle_bias_deg", "当前位置平衡角初值", -10.0, 10.0, 0.05, "deg"
    ),
    ParameterSpec("position_kp_s_inv", "位置环 Kp", 0.0, 5.0, 0.05, "1/s"),
    ParameterSpec("position_ki_s2_inv", "位置环 Ki", 0.0, 5.0, 0.05, "1/s^2"),
    ParameterSpec("position_kd", "位置环 Kd", 0.0, 10.0, 0.01, "无量纲"),
    ParameterSpec("max_velocity_m_s", "最大目标速度", 0.001, 0.20, 0.001, "m/s"),
    ParameterSpec("braking_accel_m_s2", "制动加速度估计", 0.01, 5.0, 0.01, "m/s^2"),
    ParameterSpec("braking_margin_m", "提前制动余量", 0.0, 0.05, 0.0005, "m"),
    ParameterSpec(
        "velocity_kp_deg_per_m_s", "速度环 Kp", 0.0, 300.0, 1.0, "deg/(m/s)"
    ),
    ParameterSpec("velocity_ki_deg_per_m", "速度环 Ki", 0.0, 200.0, 1.0, "deg/m"),
    ParameterSpec(
        "velocity_kd", "速度环 Kd", 0.0, 100.0, 0.01, "deg/(m/s^2)"
    ),
    ParameterSpec(
        "static_friction_compensation_deg", "静摩擦起滚补偿", 0.0, 10.0, 0.05, "deg"
    ),
    ParameterSpec(
        "static_compensation_ramp_deg_s", "起滚补偿建立速度", 0.0, 20.0, 0.1, "deg/s"
    ),
    ParameterSpec(
        "static_compensation_max_speed_m_s", "起滚补偿允许最大球速", 0.0, 0.10, 0.001, "m/s"
    ),
    ParameterSpec(
        "static_compensation_min_error_m", "起滚补偿最小位置误差", 0.0, 0.10, 0.001, "m"
    ),
    ParameterSpec(
        "local_zero_stall_time_s", "局部零点停滞更新时间", 0.1, 10.0, 0.1, "s"
    ),
    ParameterSpec(
        "stall_drive_boost_max_deg", "持续停滞追加倾角上限", 0.0, 10.0, 0.05, "deg"
    ),
    ParameterSpec(
        "stall_drive_boost_ramp_deg_s", "持续停滞追加倾角速度", 0.0, 10.0, 0.05, "deg/s"
    ),
    ParameterSpec("far_drive_angle_deg", "远距离强制倾角（0为关闭）", 0.0, 15.0, 0.1, "deg"),
    ParameterSpec("far_drive_min_error_m", "远距离策略最小误差", 0.0, 0.25, 0.005, "m"),
    ParameterSpec(
        "position_deadband_m",
        "电机保持死区（0.003m=0.3cm）",
        0.0,
        0.03,
        0.0005,
        "m",
    ),
    ParameterSpec("velocity_deadband_m_s", "速度死区", 0.0, 0.10, 0.001, "m/s"),
    ParameterSpec("outer_integral_limit_m_s", "位置积分限幅", 0.0, 0.50, 0.005, "m/s"),
    ParameterSpec("inner_integral_limit_deg", "速度积分输出限幅", 0.0, 30.0, 0.25, "deg"),
)

BACKWARD_COMPATIBLE_DEFAULTS = {
    "motor_displacement_scale": 1.0,
    "position_kd": 0.0,
    "velocity_kd": 0.0,
    "local_zero_stall_time_s": 2.0,
    "stall_drive_boost_max_deg": 1.0,
    "stall_drive_boost_ramp_deg_s": 0.25,
}


def validate_parameter_values(values: Mapping[str, Any]) -> Dict[str, float]:
    """检查UI提交值，拒绝缺项、非有限数和危险范围。"""

    result: Dict[str, float] = {}
    for spec in PARAMETER_SPECS:
        if spec.key in values:
            raw_value = values[spec.key]
        elif spec.key in BACKWARD_COMPATIBLE_DEFAULTS:
            raw_value = BACKWARD_COMPATIBLE_DEFAULTS[spec.key]
        else:
            raise ValueError("缺少参数{}".format(spec.key))
        value = float(raw_value)
        if not math.isfinite(value):
            raise ValueError("{}必须是有限数".format(spec.label))
        if not spec.minimum <= value <= spec.maximum:
            raise ValueError(
                "{}必须在{}到{}之间".format(
                    spec.label, spec.minimum, spec.maximum
                )
            )
        result[spec.key] = value
    if abs(result["equilibrium_angle_bias_deg"]) > result[
        "working_angle_limit_deg"
    ]:
        raise ValueError("平衡角初值不能超过工作倾角上限")
    return result


def _ui_process_main(
    connection: Connection,
    initial: Dict[str, float],
    profile_names: List[str],
    active_profile: Optional[str],
    initial_target_cm: float,
    target_min_cm: float,
    target_max_cm: float,
    setpoint_mode: str,
    special_task_enabled: bool,
    special_task_initial: Dict[str, float],
) -> None:
    try:
        import tkinter as tk
        from tkinter import font as tkfont
        from tkinter import messagebox, simpledialog, ttk

        root = tk.Tk()
        root.title("H题钢珠串级PID实时调参")
        root.geometry("1220x820")
        root.minsize(980, 650)
        root.configure(background="#eef2f6")

        cjk_font = "Noto Sans CJK SC"
        for font_name in ("TkDefaultFont", "TkTextFont", "TkMenuFont"):
            tkfont.nametofont(font_name).configure(
                family=cjk_font, size=12
            )
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
        style.configure(
            "TButton", font=(cjk_font, 12, "bold"), padding=(12, 8)
        )
        style.configure("Primary.TButton", foreground="#ffffff", background="#1769aa")
        style.map("Primary.TButton", background=[("active", "#0f568d")])
        style.configure("TEntry", padding=5)
        style.configure("TSpinbox", arrowsize=18, padding=5)
        style.configure("Horizontal.TScale", sliderlength=28, troughcolor="#d8e0e8")

        header = tk.Frame(root, background="#174f7d", padx=18, pady=12)
        header.pack(fill="x")
        tk.Label(
            header,
            text="H题钢珠串级 PID 实时调参",
            background="#174f7d",
            foreground="#ffffff",
            font=(cjk_font, 18, "bold"),
        ).pack(anchor="w")
        heading = tk.Label(
            header,
            text=(
                "拖动条、直接输入和上下箭头三种方式会保持联动。修改后必须点击应用；"
                "真实串口启用时会立即影响电机。"
            ),
            background="#174f7d",
            foreground="#e9f3fb",
            font=(cjk_font, 12),
            wraplength=1160,
        )
        heading.pack(fill="x", anchor="w", pady=(4, 0))

        profile_bar = ttk.LabelFrame(root, text="参数文件")
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
            value="当前默认：{}".format(active_profile or "基础配置（尚未选择方案）")
        )
        ttk.Label(profile_bar, textvariable=active_text).pack(
            side="right", padx=8
        )

        velocity_mode = setpoint_mode == "velocity"
        setpoint_name = "目标速度" if velocity_mode else "目标点"
        setpoint_unit = "cm/s" if velocity_mode else "cm"
        target_bar = ttk.LabelFrame(
            root,
            text="运行{}（不写入PID参数文件）".format(setpoint_name),
        )
        target_bar.pack(fill="x", padx=16, pady=(0, 8))
        target_variable = tk.StringVar(value="{:.2f}".format(initial_target_cm))
        target_slider_variable = tk.DoubleVar(value=float(initial_target_cm))

        def target_scale_changed(raw_value: str) -> None:
            numeric = round(float(raw_value) * 10.0) / 10.0
            numeric = max(target_min_cm, min(target_max_cm, numeric))
            target_variable.set("{:.2f}".format(numeric))

        def normalize_target_entry() -> Optional[float]:
            try:
                numeric = float(target_variable.get())
            except ValueError:
                status.set("{}错误：请输入有效数字。".format(setpoint_name))
                return None
            if not math.isfinite(numeric):
                status.set("{}错误：必须是有限数。".format(setpoint_name))
                return None
            if not target_min_cm <= numeric <= target_max_cm:
                status.set(
                    "{}必须在{:+.1f}到{:+.1f} {}之间。".format(
                        setpoint_name,
                        target_min_cm,
                        target_max_cm,
                        setpoint_unit,
                    )
                )
                return None
            target_variable.set("{:.2f}".format(numeric))
            target_slider_variable.set(numeric)
            return numeric

        def submit_target() -> None:
            numeric = normalize_target_entry()
            if numeric is None:
                return
            try:
                connection.send(
                    {
                        "type": "setpoint",
                        "mode": setpoint_mode,
                        "value": numeric,
                    }
                )
            except (BrokenPipeError, EOFError, OSError):
                status.set(
                    "控制进程已经结束，不能修改{}。".format(setpoint_name)
                )
                return
            status.set("{}已提交，等待控制进程确认……".format(setpoint_name))

        ttk.Label(target_bar, text=setpoint_name, width=10).pack(
            side="left", padx=(8, 4), pady=8
        )
        target_scale = ttk.Scale(
            target_bar,
            from_=target_min_cm,
            to=target_max_cm,
            variable=target_slider_variable,
            orient="horizontal",
            command=target_scale_changed,
        )
        target_scale.pack(side="left", fill="x", expand=True, padx=8, pady=8)
        target_spinbox = ttk.Spinbox(
            target_bar,
            from_=target_min_cm,
            to=target_max_cm,
            increment=0.1,
            textvariable=target_variable,
            width=10,
            command=normalize_target_entry,
        )
        target_spinbox.pack(side="left", padx=8, pady=8)
        target_spinbox.bind("<Return>", lambda _event: submit_target())
        target_spinbox.bind("<FocusOut>", lambda _event: normalize_target_entry())
        ttk.Label(
            target_bar,
            text="{}  [{:+.1f} 到 {:+.1f}]".format(
                setpoint_unit, target_min_cm, target_max_cm
            ),
        ).pack(side="left", padx=6, pady=8)
        ttk.Button(
            target_bar,
            text="应用{}".format(setpoint_name),
            command=submit_target,
            style="Primary.TButton",
        ).pack(side="left", padx=(8, 12), pady=6)

        if velocity_mode:
            velocity_actions = ttk.LabelFrame(root, text="速度环运行控制")
            velocity_actions.pack(fill="x", padx=16, pady=(0, 8))

            def send_velocity_action(action: str, pending_text: str) -> None:
                try:
                    connection.send({"type": action})
                    status.set(pending_text)
                except (BrokenPipeError, EOFError, OSError):
                    status.set("控制进程已经结束，操作不能执行。")

            ttk.Button(
                velocity_actions,
                text="倾斜角返回0",
                command=lambda: send_velocity_action(
                    "velocity_zero",
                    "正在返回0度并暂停速度环……",
                ),
            ).pack(side="left", padx=10, pady=8)
            ttk.Button(
                velocity_actions,
                text="启动速度环",
                command=lambda: send_velocity_action(
                    "velocity_start",
                    "正在清除旧视觉状态，等待重新检测钢珠……",
                ),
                style="Primary.TButton",
            ).pack(side="left", padx=10, pady=8)
            ttk.Label(
                velocity_actions,
                text=(
                    "端点锁存后先返回0度，再启动；启动后取得新的有效"
                    "钢珠测量才会恢复速度跟踪。"
                ),
            ).pack(side="left", padx=12, pady=8)

        status = tk.StringVar(value="窗口已就绪；当前显示启动参数。")
        outer = ttk.Frame(root, style="Card.TFrame")
        outer.pack(fill="both", expand=True, padx=16, pady=(0, 8))
        canvas = tk.Canvas(
            outer, highlightthickness=0, background="#ffffff"
        )
        scrollbar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        form = ttk.Frame(canvas, style="Card.TFrame")
        form.bind(
            "<Configure>",
            lambda _event: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        window_id = canvas.create_window((0, 0), window=form, anchor="nw")
        canvas.bind(
            "<Configure>",
            lambda event: canvas.itemconfigure(window_id, width=event.width),
        )
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        variables: Dict[str, tk.StringVar] = {}
        slider_variables: Dict[str, tk.DoubleVar] = {}
        spec_by_key = {spec.key: spec for spec in PARAMETER_SPECS}

        def decimal_places(increment: float) -> int:
            text = "{:.8f}".format(float(increment)).rstrip("0")
            return len(text.split(".", 1)[1]) if "." in text else 0

        def format_for_spec(spec: ParameterSpec, value: float) -> str:
            return ("{:.%df}" % decimal_places(spec.increment)).format(
                float(value)
            )

        def set_control_value(key: str, value: Any) -> None:
            spec = spec_by_key[key]
            numeric = max(spec.minimum, min(spec.maximum, float(value)))
            variables[key].set(format_for_spec(spec, numeric))
            slider_variables[key].set(numeric)

        def scale_changed(key: str, raw_value: str) -> None:
            spec = spec_by_key[key]
            numeric = float(raw_value)
            steps = round((numeric - spec.minimum) / spec.increment)
            numeric = spec.minimum + steps * spec.increment
            numeric = max(spec.minimum, min(spec.maximum, numeric))
            variables[key].set(format_for_spec(spec, numeric))

        def entry_changed(key: str) -> None:
            try:
                numeric = float(variables[key].get())
            except ValueError:
                status.set("参数错误：{}不是有效数字。".format(spec_by_key[key].label))
                return
            set_control_value(key, numeric)

        section_starts = {
            "working_angle_limit_deg": "安全限制与执行器",
            "velocity_filter_time_constant_s": "视觉速度滤波",
            "position_kp_s_inv": "位置环与制动规划",
            "velocity_kp_deg_per_m_s": "速度环",
            "static_friction_compensation_deg": "起滚与远距离策略",
            "position_deadband_m": "死区与积分限幅",
        }
        row = 0
        for spec in PARAMETER_SPECS:
            if spec.key in section_starts:
                ttk.Label(
                    form,
                    text=section_starts[spec.key],
                    style="Section.TLabel",
                ).grid(
                    row=row,
                    column=0,
                    columnspan=4,
                    sticky="ew",
                    padx=8,
                    pady=(10 if row else 4, 6),
                )
                row += 1
            ttk.Label(
                form, text=spec.label, width=27, style="Card.TLabel"
            ).grid(
                row=row, column=0, sticky="w", padx=(16, 8), pady=7
            )
            variable = tk.StringVar(
                value=format_for_spec(spec, initial[spec.key])
            )
            variables[spec.key] = variable
            slider_variable = tk.DoubleVar(value=float(initial[spec.key]))
            slider_variables[spec.key] = slider_variable
            scale = ttk.Scale(
                form,
                from_=spec.minimum,
                to=spec.maximum,
                variable=slider_variable,
                orient="horizontal",
                command=lambda raw, key=spec.key: scale_changed(key, raw),
            )
            scale.grid(row=row, column=1, sticky="ew", padx=10, pady=7)
            spinbox = ttk.Spinbox(
                form,
                from_=spec.minimum,
                to=spec.maximum,
                increment=spec.increment,
                textvariable=variable,
                width=14,
                command=lambda key=spec.key: entry_changed(key),
            )
            spinbox.grid(row=row, column=2, sticky="ew", padx=8, pady=7)
            spinbox.bind(
                "<Return>", lambda _event, key=spec.key: entry_changed(key)
            )
            spinbox.bind(
                "<FocusOut>", lambda _event, key=spec.key: entry_changed(key)
            )
            ttk.Label(
                form,
                text="{}  [{} 到 {}]".format(
                    spec.unit, spec.minimum, spec.maximum
                ),
                foreground="#555555",
                style="Card.TLabel",
            ).grid(row=row, column=3, sticky="w", padx=(6, 16), pady=7)
            row += 1
        form.columnconfigure(1, weight=1)

        def mouse_wheel(event: Any) -> None:
            if getattr(event, "num", None) == 4:
                canvas.yview_scroll(-3, "units")
            elif getattr(event, "num", None) == 5:
                canvas.yview_scroll(3, "units")
            elif getattr(event, "delta", 0):
                canvas.yview_scroll(int(-event.delta / 120) * 3, "units")

        canvas.bind_all("<MouseWheel>", mouse_wheel)
        canvas.bind_all("<Button-4>", mouse_wheel)
        canvas.bind_all("<Button-5>", mouse_wheel)

        if special_task_enabled:
            special_frame = ttk.LabelFrame(
                root,
                text="特殊任务（独立参数，不保存到PID参数文件）",
            )
            special_frame.pack(fill="x", padx=16, pady=(0, 8))
            special_variables: Dict[str, tk.StringVar] = {}
            special_fields = (
                ("first_point_cm", "第一个到达点", "cm", target_min_cm, target_max_cm, 0.05),
                ("second_point_cm", "第二个到达点", "cm", target_min_cm, target_max_cm, 0.05),
                ("first_angle_deg", "到第一点倾角", "deg", -21.8, 21.8, 0.05),
                ("positive_motor_scale", "任务正指令比例", "x", 0.1, 2.0, 0.01),
                ("negative_motor_scale", "任务负指令比例", "x", 0.1, 2.0, 0.01),
            )
            for column, (key, label, unit, minimum, maximum, increment) in enumerate(
                special_fields
            ):
                field = ttk.Frame(special_frame)
                field.grid(row=0, column=column, padx=6, pady=8, sticky="w")
                ttk.Label(field, text=label).pack(anchor="w")
                variable = tk.StringVar(
                    value="{:.2f}".format(float(special_task_initial[key]))
                )
                special_variables[key] = variable
                line = ttk.Frame(field)
                line.pack(anchor="w", pady=(3, 0))
                ttk.Spinbox(
                    line,
                    from_=minimum,
                    to=maximum,
                    increment=increment,
                    textvariable=variable,
                    width=8,
                ).pack(side="left")
                ttk.Label(line, text=unit).pack(side="left", padx=(4, 0))

            def send_special_action(message_type: str) -> None:
                message: Dict[str, Any] = {"type": message_type}
                if message_type in (
                    "special_task_start",
                    "special_task_save",
                ):
                    try:
                        settings = {
                            key: float(variable.get())
                            for key, variable in special_variables.items()
                        }
                    except ValueError:
                        status.set("特殊任务参数错误：请输入有效数字。")
                        return
                    if not all(math.isfinite(value) for value in settings.values()):
                        status.set("特殊任务参数错误：所有数值必须有限。")
                        return
                    message["settings"] = settings
                try:
                    connection.send(message)
                except (BrokenPipeError, EOFError, OSError):
                    status.set("控制进程已经结束，特殊任务操作不能执行。")
                    return
                status.set(
                    "正在检查钢珠中心位置并启动特殊任务……"
                    if message_type == "special_task_start"
                    else (
                        "正在直接覆盖特殊任务配置……"
                        if message_type == "special_task_save"
                        else "正在取消特殊任务并让倾角返回0°……"
                    )
                )

            actions = ttk.Frame(special_frame)
            actions.grid(row=1, column=0, columnspan=5, sticky="ew", padx=6, pady=(0, 8))
            ttk.Button(
                actions,
                text="倾角给0 / 取消任务",
                command=lambda: send_special_action("special_task_zero"),
            ).pack(side="left", padx=(0, 10))
            ttk.Button(
                actions,
                text="检查中心并启动任务",
                command=lambda: send_special_action("special_task_start"),
                style="Primary.TButton",
            ).pack(side="left")
            ttk.Button(
                actions,
                text="保存并覆盖任务配置",
                command=lambda: send_special_action("special_task_save"),
            ).pack(side="left", padx=(10, 0))
            ttk.Label(
                actions,
                text="只有最新钢珠位置在中心±1.00 cm内才会启动；完成后可归0、放回中心并再次启动。",
            ).pack(side="left", padx=14)

        status_label = ttk.Label(
            root,
            textvariable=status,
            wraplength=1160,
            font=(cjk_font, 12, "bold"),
        )
        status_label.pack(fill="x", padx=18, pady=(6, 4))

        buttons = ttk.Frame(root)
        buttons.pack(fill="x", padx=16, pady=(4, 14))

        def collect() -> Optional[Dict[str, float]]:
            try:
                raw = {key: variable.get() for key, variable in variables.items()}
                return validate_parameter_values(raw)
            except (TypeError, ValueError) as error:
                status.set("参数错误：{}".format(error))
                return None

        def submit() -> None:
            values = collect()
            if values is None:
                return
            try:
                connection.send(
                    {"type": "parameters", "values": values}
                )
            except (BrokenPipeError, EOFError, OSError):
                status.set("控制进程已经结束，不能继续应用参数。")
                return
            status.set("参数已提交，等待控制进程确认……")

        def save_as_profile() -> None:
            values = collect()
            if values is None:
                return
            name = simpledialog.askstring(
                "保存参数方案",
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
                    {
                        "type": "save_profile",
                        "name": normalized,
                        "values": values,
                    }
                )
                status.set("正在保存并应用参数方案……")
            except (BrokenPipeError, EOFError, OSError):
                status.set("控制进程已经结束，不能保存参数。")

        def load_selected_profile() -> None:
            name = selected_profile.get().strip()
            if not name:
                status.set("请先选择一个参数文件。")
                return
            try:
                connection.send({"type": "load_profile", "name": name})
                status.set("正在加载并应用参数方案……")
            except (BrokenPipeError, EOFError, OSError):
                status.set("控制进程已经结束，不能加载参数。")

        def rename_selected_profile() -> None:
            old_name = selected_profile.get().strip()
            if not old_name:
                status.set("请先选择一个需要重命名的参数文件。")
                return
            new_name = simpledialog.askstring(
                "重命名参数方案",
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
                status.set("控制进程已经结束，不能重命名参数。")

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

        def restore() -> None:
            for key, value in initial.items():
                if key in variables:
                    set_control_value(key, value)
            status.set("已恢复启动时数值；点击应用后才会生效。")

        ttk.Button(
            buttons,
            text="应用到当前控制器（不保存）",
            command=submit,
            style="Primary.TButton",
        ).pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="恢复启动值", command=restore).pack(
            side="left", padx=8
        )
        ttk.Button(buttons, text="关闭窗口", command=root.destroy).pack(
            side="right"
        )

        def poll_parent() -> None:
            try:
                while connection.poll():
                    message = connection.recv()
                    if message.get("type") == "shutdown":
                        root.destroy()
                        return
                    if message.get("type") == "status":
                        status.set(str(message.get("message", "")))
                    if message.get("type") == "profile_list":
                        names = list(message.get("profiles", []))
                        profile_combo.configure(values=names)
                        active = message.get("active_profile")
                        if active:
                            selected_profile.set(str(active))
                        active_text.set(
                            "当前默认：{}".format(
                                active or "基础配置（尚未选择方案）"
                            )
                        )
                    if message.get("type") == "load_values":
                        values = message.get("values", {})
                        for key, value in values.items():
                            if key in variables:
                                set_control_value(key, value)
                    if message.get("type") == "setpoint_value":
                        numeric = float(message.get("value"))
                        target_variable.set("{:.2f}".format(numeric))
                        target_slider_variable.set(numeric)
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


class ControlTuningUI:
    """父进程侧非阻塞接口。"""

    def __init__(
        self,
        initial_values: Mapping[str, Any],
        profile_names: Optional[List[str]] = None,
        active_profile: Optional[str] = None,
        initial_target_cm: float = 0.0,
        target_min_cm: float = -12.0,
        target_max_cm: float = 12.0,
        setpoint_mode: str = "position",
        special_task_enabled: bool = False,
        special_task_initial: Optional[Mapping[str, Any]] = None,
    ) -> None:
        self.initial_values = validate_parameter_values(initial_values)
        initial_target_cm = float(initial_target_cm)
        target_min_cm = float(target_min_cm)
        target_max_cm = float(target_max_cm)
        if setpoint_mode not in ("position", "velocity"):
            raise ValueError("控制UI设定类型必须是position或velocity。")
        if not (
            math.isfinite(initial_target_cm)
            and math.isfinite(target_min_cm)
            and math.isfinite(target_max_cm)
            and target_min_cm < target_max_cm
            and target_min_cm <= initial_target_cm <= target_max_cm
        ):
            raise ValueError("目标点初值或范围无效。")
        special_defaults = {
            "first_point_cm": -3.0,
            "second_point_cm": 5.0,
            "first_angle_deg": 2.43,
            "positive_motor_scale": 0.2,
            "negative_motor_scale": 0.7,
        }
        if special_task_initial is not None:
            special_defaults.update(
                {key: float(value) for key, value in special_task_initial.items()}
            )
        context = multiprocessing.get_context("spawn")
        self._connection, child = context.Pipe(duplex=True)
        self._process = context.Process(
            target=_ui_process_main,
            args=(
                child,
                self.initial_values,
                list(profile_names or []),
                active_profile,
                initial_target_cm,
                target_min_cm,
                target_max_cm,
                setpoint_mode,
                bool(special_task_enabled),
                special_defaults,
            ),
            name="ball-control-tuning-ui",
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

    def set_target(self, target_cm: float) -> None:
        try:
            self._connection.send(
                {"type": "setpoint_value", "value": float(target_cm)}
            )
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
