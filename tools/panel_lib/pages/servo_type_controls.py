"""BUS/PWM servo type transaction controls for the host panel."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from ..proto import PROTO_REQ_SERVOTYPE, parse_kv


class ServoTypeControlsMixin:
    def _refresh_servo_output_controls(self) -> None:
        """Optional hook overridden by pages with mode-specific widgets."""

    def _build_servo_type_controls(self, parent: ttk.Frame) -> None:
        # The combobox is a candidate only.  Safety gating follows the last
        # board-confirmed active type until an APPLY readback arrives.
        self.servo_type_var = tk.StringVar(value="bus")
        self.servo_type_active_var = tk.StringVar(value="bus")
        self.servo_type_status_var = tk.StringVar(value="尚未查询")

        type_box = ttk.LabelFrame(parent, text="舵机输出类型", padding=8)
        type_box.pack(fill=tk.X)
        ttk.Label(type_box, text="当前类型").pack(side=tk.LEFT)
        selector = ttk.Combobox(
            type_box,
            textvariable=self.servo_type_var,
            values=("bus", "pwm"),
            state="readonly",
            width=8,
        )
        selector.pack(side=tk.LEFT, padx=6)
        selector.bind("<<ComboboxSelected>>", lambda _event: self._refresh_servo_output_controls())
        ttk.Button(type_box, text="查询", command=self._servo_type_query).pack(side=tk.LEFT)
        ttk.Button(type_box, text="应用", command=self._servo_type_apply).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(type_box, text="撤销", command=self._servo_type_revert).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(type_box, text="提交", command=self._servo_type_commit).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Label(type_box, textvariable=self.servo_type_status_var).pack(side=tk.LEFT, padx=(12, 0))

    def _servo_type_query(self) -> None:
        self._send_proto(PROTO_REQ_SERVOTYPE, "SERVOTYPE?")

    def _servo_type_apply(self) -> None:
        selected = self.servo_type_var.get().strip().lower()
        if selected not in {"bus", "pwm"}:
            return
        self._send_proto(PROTO_REQ_SERVOTYPE, f"SERVOTYPE APPLY type={selected}")

    def _servo_type_revert(self) -> None:
        self._send_proto(PROTO_REQ_SERVOTYPE, "SERVOTYPE REVERT")

    def _servo_type_commit(self) -> None:
        self._send_proto(PROTO_REQ_SERVOTYPE, "SERVOTYPE COMMIT")

    def _servo_output_is_pwm(self) -> bool:
        variable = getattr(self, "servo_type_active_var", None)
        return variable is not None and variable.get().strip().lower() == "pwm"

    def _update_servo_type_line(self, line: str) -> None:
        values = parse_kv(line)
        active = values.get("active", "").strip().lower()
        if active in {"bus", "pwm"} and hasattr(self, "servo_type_var"):
            self.servo_type_var.set(active)
        if active in {"bus", "pwm"} and hasattr(self, "servo_type_active_var"):
            self.servo_type_active_var.set(active)
        fields = ("state", "active", "persisted", "dirty", "valid", "explicit",
                  "generation", "record_generation", "request")
        status = " ".join(f"{field}={values.get(field, '-')}" for field in fields)
        if hasattr(self, "servo_type_status_var"):
            self.servo_type_status_var.set(status)
        self._refresh_servo_output_controls()


__all__ = ["ServoTypeControlsMixin"]
