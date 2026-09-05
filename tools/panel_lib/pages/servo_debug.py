"""Servo output debug page and the BUS/PWM host controls."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from ..proto import (
    PROTO_REQ_SERVO_ACTION,
    PROTO_REQ_SERVO_ENABLE,
    PROTO_REQ_SERVO_ID,
    PROTO_REQ_SERVO_MODE,
    PROTO_REQ_SERVO_MOVE,
    PROTO_REQ_SERVO_MOVE_ALL,
    PROTO_REQ_SERVO_RAW,
    PROTO_REQ_SERVO_SETID,
    parse_kv,
    safe_int,
)
from .servo_type_controls import ServoTypeControlsMixin


class ServoDebugPageMixin(ServoTypeControlsMixin):
    def _build_servo_page(
        self, parent: ttk.Frame, *, fixed_parent: ttk.Frame | None = None
    ) -> None:
        self._servo_bus_widgets: list[tk.Widget] = []
        self._servo_raw_widgets: list[tk.Widget] = []
        self._servo_input_widgets: dict[tuple[int, str], tk.Widget] = {}
        self.servo_validation_vars: list[tk.StringVar] = []
        if fixed_parent is not None:
            stop_box = ttk.LabelFrame(fixed_parent, text="固定操作区 · 点动停止", padding=6)
            stop_box.pack(fill=tk.X)
            ttk.Label(stop_box, text="停止只交还稳定环，不断开通信：").pack(
                side=tk.LEFT, padx=(0, 8)
            )
            for index in range(2):
                button = ttk.Button(
                    stop_box, text="停止",
                    command=lambda i=index: self._servo_cmd(i, "DST"),
                    style="Danger.TButton",
                )
                button.pack(side=tk.LEFT, padx=(0, 6))
                self._servo_bus_widgets.append(button)
        self._build_servo_type_controls(parent)

        servo_notebook = ttk.Notebook(parent, style="TNotebook")
        servo_notebook.pack(fill=tk.BOTH, expand=True)
        for index in range(2):
            frame = ttk.Frame(servo_notebook, padding=10)
            servo_notebook.add(frame, text=f"舵机 {index}")
            self._build_servo_tab(frame, index)

        raw = ttk.LabelFrame(parent, text="手动原始舵机指令（仅总线模式）", padding=10)
        raw.pack(fill=tk.X, pady=(10, 0))
        self.raw_var = tk.StringVar(value="{#001P1500T0500!#002P1500T0500!}")
        raw_entry = ttk.Entry(raw, textvariable=self.raw_var, style="Numeric.TEntry")
        raw_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        raw_button = ttk.Button(raw, text="发送原始指令", command=self._send_raw)
        raw_button.pack(side=tk.LEFT, padx=(6, 0))
        self._servo_raw_widgets.extend((raw_entry, raw_button))
        self._refresh_servo_output_controls()
    def _build_servo_tab(self, parent: ttk.Frame, index: int) -> None:
        values: dict[str, tk.Variable] = {
            "id": tk.IntVar(value=index + 1),
            "pulse": tk.IntVar(value=1500),
            "time": tk.IntVar(value=500),
            "mode": tk.IntVar(value=1),
            "enabled": tk.IntVar(value=1),
            "new_id": tk.IntVar(value=index + 1),
            "baud": tk.IntVar(value=4),
        }
        self.servo_widgets.append(values)

        row = 0
        enabled = ttk.Checkbutton(
            parent,
            text="启用此舵机槽位",
            variable=values["enabled"],
            command=lambda i=index: self._servo_enable(i),
        )
        enabled.grid(row=row, column=0, sticky=tk.W)
        self._servo_bus_widgets.append(enabled)
        row += 1
        self._servo_spin(parent, index, "id", row, "当前舵机 ID", values["id"], 0, 255, lambda i=index: self._servo_set_id(i))
        row += 1
        self._scale(parent, row, "目标位置 us", values["pulse"], 500, 2500)
        row += 1
        self._servo_spin(parent, index, "time", row, "运行时间 ms", values["time"], 0, 9999, None)
        row += 1
        self._servo_spin(parent, index, "mode", row, "模式 1-8", values["mode"], 1, 8, lambda i=index: self._servo_mode(i))
        row += 1
        validation_var = tk.StringVar(value="")
        self.servo_validation_vars.append(validation_var)
        ttk.Label(parent, textvariable=validation_var, style="Fail.TLabel").grid(
            row=row, column=0, columnspan=3, sticky=tk.W, pady=(0, 4)
        )
        row += 1
        ttk.Button(parent, text="移动此舵机", command=lambda i=index: self._servo_move(i)).grid(
            row=row, column=0, pady=6, sticky=tk.EW
        )
        # PWM 模式下走 _servo_move_pwm_immediate 逐路即时下发，不再是总线专属控件。
        move_all = ttk.Button(
            parent,
            text="按配置同时移动两路",
            command=self._servo_move_all,
        )
        move_all.grid(row=row, column=1, pady=6, sticky=tk.EW)
        row += 1

        id_box = ttk.LabelFrame(parent, text="修改实体舵机 ID", padding=8)
        id_box.grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=(8, 4))
        new_id = ttk.Spinbox(
            id_box, from_=0, to=255, textvariable=values["new_id"], width=8,
            style="Numeric.TSpinbox",
        )
        new_id.pack(side=tk.LEFT)
        self._servo_input_widgets[(index, "new_id")] = new_id
        set_id = ttk.Button(id_box, text="写入新 ID", command=lambda i=index: self._servo_set_physical_id(i))
        set_id.pack(side=tk.LEFT, padx=6)
        self._servo_bus_widgets.extend((new_id, set_id))
        row += 1

        actions = ttk.LabelFrame(parent, text="众灵手册动作指令", padding=8)
        actions.grid(row=row, column=0, columnspan=2, sticky=tk.EW)
        action_names = [
            ("读取版本", "VER"),
            ("检测 ID", "PID"),
            ("读取位置", "RAD"),
            ("读取模式", "MOD?"),
            ("释放扭力", "ULK"),
            ("恢复扭力", "ULR"),
            ("暂停", "DPT"),
            ("继续", "DCT"),
            ("当前位置设中位", "SCK"),
            ("设置启动位置", "CSD"),
            ("清除启动位置", "CSM"),
            ("恢复启动位置", "CSR"),
            ("设置最小值", "SMI"),
            ("设置最大值", "SMX"),
            ("半恢复出厂", "CLEO"),
            ("全恢复出厂", "CLE"),
        ]
        for n, (label, command) in enumerate(action_names):
            action = ttk.Button(actions, text=label, command=lambda i=index, c=command: self._servo_cmd(i, c))
            action.grid(row=n // 2, column=n % 2, padx=3, pady=3, sticky=tk.EW)
            self._servo_bus_widgets.append(action)
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)

        baud_box = ttk.Frame(parent)
        baud_box.grid(row=row + 1, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))
        ttk.Label(baud_box, text="波特率代码").pack(side=tk.LEFT)
        baud = ttk.Spinbox(
            baud_box, from_=0, to=7, textvariable=values["baud"], width=5,
            style="Numeric.TSpinbox",
        )
        baud.pack(side=tk.LEFT, padx=6)
        self._servo_input_widgets[(index, "baud")] = baud
        baud_button = ttk.Button(baud_box, text="设置波特率", command=lambda i=index: self._servo_baud(i))
        baud_button.pack(side=tk.LEFT)
        self._servo_bus_widgets.extend((baud, baud_button))

        parent.columnconfigure(1, weight=1)

    def _servo_spin(self, parent: ttk.Frame, index: int, field: str, row: int, label: str,
                    variable: tk.Variable, minimum: int, maximum: int, command) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        box = ttk.Spinbox(
            parent, from_=minimum, to=maximum, textvariable=variable, width=10,
            style="Numeric.TSpinbox",
        )
        box.grid(row=row, column=1, sticky=tk.EW, pady=4)
        self._servo_input_widgets[(index, field)] = box
        self._servo_bus_widgets.append(box)
        if command is not None:
            button = ttk.Button(parent, text="应用", command=command)
            button.grid(row=row, column=2, padx=(6, 0))
            self._servo_bus_widgets.append(button)

    def _refresh_servo_output_controls(self) -> None:
        pwm = self._servo_output_is_pwm()
        state = "disabled" if pwm else "normal"
        for widget in getattr(self, "_servo_bus_widgets", ()):
            widget.configure(state=state)
        for widget in getattr(self, "_servo_raw_widgets", ()):
            widget.configure(state=state)

    def _send_raw(self) -> None:
        if self._servo_output_is_pwm():
            return
        payload = f"SERVO RAW {self.raw_var.get().strip()}"
        self._send_proto(PROTO_REQ_SERVO_RAW, payload, payload)

    def _servo_validation_error(self, index: int, field: str, message: str) -> None:
        labels = {
            "id": "当前舵机 ID", "pulse": "目标位置 us", "time": "运行时间 ms",
            "mode": "模式 1-8", "enabled": "启用此舵机槽位", "new_id": "新 ID",
            "baud": "波特率代码",
        }
        validation_vars = getattr(self, "servo_validation_vars", ())
        if index < len(validation_vars):
            validation_vars[index].set(f"{labels.get(field, field)}：{message}")
        widget = getattr(self, "_servo_input_widgets", {}).get((index, field))
        if widget is not None:
            widget.focus_set()
            try:
                widget.selection_range(0, tk.END)
            except tk.TclError:
                pass

    def _servo_clear_validation_error(self, index: int) -> None:
        validation_vars = getattr(self, "servo_validation_vars", ())
        if index < len(validation_vars):
            validation_vars[index].set("")

    def _servo_values(self, index: int, fields: tuple[str, ...] | None = None) -> dict[str, int] | None:
        widgets = self.servo_widgets[index]
        bounds = {
            "id": (0, 255), "pulse": (500, 2500), "time": (0, 9999),
            "mode": (1, 8), "enabled": (0, 1), "new_id": (0, 255), "baud": (0, 7),
        }
        result: dict[str, int] = {}
        for field in fields or tuple(widgets):
            variable = widgets[field]
            try:
                raw = str(variable.get()).strip()
            except tk.TclError:
                # IntVar/DoubleVar validate on ``get()`` and would turn a
                # user-entered partial value into the old global TclError.
                # Read the edit buffer as text so the field-level path below
                # can report it without sending anything.
                raw = str(variable._tk.globalgetvar(variable._name)).strip()
            if not raw:
                self._servo_validation_error(index, field, "值不能为空")
                return None
            if not raw.lstrip("+-").isdigit():
                self._servo_validation_error(index, field, "请输入整数")
                return None
            try:
                value = int(raw)
            except ValueError:
                self._servo_validation_error(index, field, "数值过大")
                return None
            minimum, maximum = bounds[field]
            if value < minimum or value > maximum:
                self._servo_validation_error(index, field, f"范围应为 {minimum}..{maximum}")
                return None
            result[field] = value
        self._servo_clear_validation_error(index)
        return result

    def _servo_move_pwm_immediate(self, index: int, pulse_us: int) -> None:
        # 裸 `SERVO JOG` 是 mechanical.py 拆桨标定的 500µs/s 慢速斜坡（1000µs 行程要 2s），
        # 调试页要的是与总线 SERVO MOVE 同量级的响应，因此走 NOW 即时变体。
        payload = f"SERVO JOG {index} {pulse_us} NOW"
        self._send_proto(PROTO_REQ_SERVO_MOVE, payload, payload)

    def _servo_move(self, index: int) -> None:
        values = self._servo_values(index, ("pulse", "time"))
        if values is None:
            return
        if self._servo_output_is_pwm():
            self._servo_move_pwm_immediate(index, values["pulse"])
            return
        payload = f"SERVO MOVE {index} {values['pulse']} {values['time']}"
        self._send_proto(PROTO_REQ_SERVO_MOVE, payload, payload)

    def _servo_move_all(self) -> None:
        if self._servo_output_is_pwm():
            pulses: list[int] = []
            for index in range(len(self.servo_widgets)):
                values = self._servo_values(index, ("pulse",))
                if values is None:
                    return
                pulses.append(values["pulse"])
            for index, pulse in enumerate(pulses):
                self._servo_move_pwm_immediate(index, pulse)
            return
        self._send_proto(PROTO_REQ_SERVO_MOVE_ALL, "SERVO MOVEALL")

    def _servo_mode(self, index: int) -> None:
        if self._servo_output_is_pwm():
            return
        values = self._servo_values(index, ("mode",))
        if values is None:
            return
        payload = f"SERVO MODE {index} {values['mode']}"
        self._send_proto(PROTO_REQ_SERVO_MODE, payload, payload)

    def _servo_enable(self, index: int) -> None:
        if self._servo_output_is_pwm():
            return
        values = self._servo_values(index, ("enabled",))
        if values is None:
            return
        payload = f"SERVO ENABLE {index} {values['enabled']}"
        self._send_proto(PROTO_REQ_SERVO_ENABLE, payload, payload)

    def _servo_set_id(self, index: int) -> None:
        if self._servo_output_is_pwm():
            return
        values = self._servo_values(index, ("id",))
        if values is None:
            return
        payload = f"SERVO ID {index} {values['id']}"
        self._send_proto(PROTO_REQ_SERVO_ID, payload, payload)

    def _servo_set_physical_id(self, index: int) -> None:
        if self._servo_output_is_pwm():
            return
        values = self._servo_values(index, ("new_id",))
        if values is None:
            return
        payload = f"SERVO SETID {index} {values['new_id']}"
        self._send_proto(PROTO_REQ_SERVO_SETID, payload, payload)

    def _servo_cmd(self, index: int, command: str) -> None:
        if self._servo_output_is_pwm():
            return
        payload = f"SERVO CMD {index} {command}"
        self._send_proto(PROTO_REQ_SERVO_ACTION, payload, payload)

    def _servo_baud(self, index: int) -> None:
        if self._servo_output_is_pwm():
            return
        values = self._servo_values(index, ("baud",))
        if values is None:
            return
        payload = f"SERVO CMD {index} BD {values['baud']}"
        self._send_proto(PROTO_REQ_SERVO_ACTION, payload, payload)

    def _update_servo_ok_line(self, line: str) -> None:
        values = parse_kv(line)
        parts = line.split()
        if len(parts) < 2 or not parts[1].startswith("servo"):
            return
        index = safe_int(parts[1].replace("servo", ""), -1)
        if index < 0:
            return

        field_map = {
            "id": "id",
            "enabled": "enabled",
            "mode": "mode",
            "pulse": "pulse",
            "time": "time",
        }
        for src, dst in field_map.items():
            if src in values:
                self._set_param(f"servo{index}.{dst}", values[src], "OK", dirty=False)
        if 0 <= index < len(self.servo_widgets):
            widgets = self.servo_widgets[index]
            for src, dst in field_map.items():
                if src in values and dst in widgets:
                    widgets[dst].set(safe_int(values[src]))


__all__ = ["ServoDebugPageMixin"]
