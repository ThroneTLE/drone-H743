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
    def _build_servo_page(self, parent: ttk.Frame) -> None:
        self._servo_bus_widgets: list[tk.Widget] = []
        self._servo_raw_widgets: list[tk.Widget] = []
        self._build_servo_type_controls(parent)

        servo_notebook = ttk.Notebook(parent)
        servo_notebook.pack(fill=tk.BOTH, expand=True)
        for index in range(2):
            frame = ttk.Frame(servo_notebook, padding=10)
            servo_notebook.add(frame, text=f"舵机 {index}")
            self._build_servo_tab(frame, index)

        raw = ttk.LabelFrame(parent, text="手动原始舵机指令（仅总线模式）", padding=10)
        raw.pack(fill=tk.X, pady=(10, 0))
        self.raw_var = tk.StringVar(value="{#001P1500T0500!#002P1500T0500!}")
        raw_entry = ttk.Entry(raw, textvariable=self.raw_var)
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
        self._servo_spin(parent, row, "当前舵机 ID", values["id"], 0, 255, lambda i=index: self._servo_set_id(i))
        row += 1
        self._scale(parent, row, "目标位置 us", values["pulse"], 500, 2500)
        row += 1
        self._servo_spin(parent, row, "运行时间 ms", values["time"], 0, 9999, None)
        row += 1
        self._servo_spin(parent, row, "模式 1-8", values["mode"], 1, 8, lambda i=index: self._servo_mode(i))
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
        new_id = ttk.Spinbox(id_box, from_=0, to=255, textvariable=values["new_id"], width=8)
        new_id.pack(side=tk.LEFT)
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
            ("停止", "DST"),
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
        baud = ttk.Spinbox(baud_box, from_=0, to=7, textvariable=values["baud"], width=5)
        baud.pack(side=tk.LEFT, padx=6)
        baud_button = ttk.Button(baud_box, text="设置波特率", command=lambda i=index: self._servo_baud(i))
        baud_button.pack(side=tk.LEFT)
        self._servo_bus_widgets.extend((baud, baud_button))

        parent.columnconfigure(1, weight=1)

    def _servo_spin(self, parent: ttk.Frame, row: int, label: str, variable: tk.Variable,
                    minimum: int, maximum: int, command) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        box = ttk.Spinbox(parent, from_=minimum, to=maximum, textvariable=variable, width=10)
        box.grid(row=row, column=1, sticky=tk.EW, pady=4)
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

    def _servo_values(self, index: int) -> dict[str, int]:
        widgets = self.servo_widgets[index]
        return {key: int(var.get()) for key, var in widgets.items()}

    def _servo_move_pwm_immediate(self, index: int, pulse_us: int) -> None:
        # 裸 `SERVO JOG` 是 mechanical.py 拆桨标定的 500µs/s 慢速斜坡（1000µs 行程要 2s），
        # 调试页要的是与总线 SERVO MOVE 同量级的响应，因此走 NOW 即时变体。
        payload = f"SERVO JOG {index} {pulse_us} NOW"
        self._send_proto(PROTO_REQ_SERVO_MOVE, payload, payload)

    def _servo_move(self, index: int) -> None:
        values = self._servo_values(index)
        if self._servo_output_is_pwm():
            self._servo_move_pwm_immediate(index, values["pulse"])
            return
        payload = f"SERVO MOVE {index} {values['pulse']} {values['time']}"
        self._send_proto(PROTO_REQ_SERVO_MOVE, payload, payload)

    def _servo_move_all(self) -> None:
        if self._servo_output_is_pwm():
            for index in range(len(self.servo_widgets)):
                self._servo_move_pwm_immediate(index, self._servo_values(index)["pulse"])
            return
        self._send_proto(PROTO_REQ_SERVO_MOVE_ALL, "SERVO MOVEALL")

    def _servo_mode(self, index: int) -> None:
        if self._servo_output_is_pwm():
            return
        values = self._servo_values(index)
        payload = f"SERVO MODE {index} {values['mode']}"
        self._send_proto(PROTO_REQ_SERVO_MODE, payload, payload)

    def _servo_enable(self, index: int) -> None:
        if self._servo_output_is_pwm():
            return
        values = self._servo_values(index)
        payload = f"SERVO ENABLE {index} {values['enabled']}"
        self._send_proto(PROTO_REQ_SERVO_ENABLE, payload, payload)

    def _servo_set_id(self, index: int) -> None:
        if self._servo_output_is_pwm():
            return
        values = self._servo_values(index)
        payload = f"SERVO ID {index} {values['id']}"
        self._send_proto(PROTO_REQ_SERVO_ID, payload, payload)

    def _servo_set_physical_id(self, index: int) -> None:
        if self._servo_output_is_pwm():
            return
        values = self._servo_values(index)
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
        values = self._servo_values(index)
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
