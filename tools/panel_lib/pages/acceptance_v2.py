"""Propeller-off V2A control-chain acceptance page."""

from __future__ import annotations

import json
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, ttk

from ..proto import PROTO_REQ_ACCEPTANCE, parse_kv, safe_int

try:
    from ...project_paths import (
        FLIGHT_ACCEPTANCE_CALIBRATION_DIR,
        dated_directory,
        ensure_directory,
    )
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools.project_paths import (
            FLIGHT_ACCEPTANCE_CALIBRATION_DIR,
            dated_directory,
            ensure_directory,
        )
    except ImportError:
        from project_paths import (
            FLIGHT_ACCEPTANCE_CALIBRATION_DIR,
            dated_directory,
            ensure_directory,
        )


class AcceptanceV2PageMixin:
    def _build_v2_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="GROUND ACCEPTANCE  /  验证控制链与执行方向", style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text="无桨控制链安全验收", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=("这是拆桨地面模式：500 ms 租约，USB/链路停止续租即自动退出；进入模式后电机硬锁且 ESC CCR=0。"
                  "当前版本用于 RC、导航、恢复方向和舵机 ±50 µs 的可执行初始化，不构成自由飞行放行。"),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))
        safety = ttk.LabelFrame(parent, text="必要安全门", padding=8); safety.pack(fill=tk.X)
        ttk.Checkbutton(safety, text="已拆除全部桨叶", variable=self.validation_props_removed_var).pack(side=tk.LEFT)
        ttk.Checkbutton(safety, text="动力已隔离或机体已可靠固定", variable=self.validation_power_safe_var).pack(side=tk.LEFT, padx=(16, 0))
        controls = ttk.Frame(parent); controls.pack(fill=tk.X, pady=(10, 0))
        ttk.Button(controls, text="启动无桨安全模式", command=self._v2_start,
                   style="Warning.TButton").pack(anchor=tk.W)
        step_row = ttk.Frame(controls)
        step_row.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(step_row, text="步骤").pack(side=tk.LEFT, padx=(0, 4))
        stages = (
            "rc_center", "rc_positive_roll", "rc_positive_pitch", "rc_positive_yaw",
            "nav_static", "nav_forward", "nav_left", "restore_positive_roll",
            "restore_positive_pitch", "servo_alpha_positive_50us",
            "servo_alpha_negative_50us", "servo_beta_positive_50us",
            "servo_beta_negative_50us", "failsafe",
        )
        ttk.Combobox(step_row, textvariable=self.v2_stage_var, values=stages,
                     state="readonly", width=31).pack(anchor=tk.W)
        ttk.Button(controls, text="切换并观察步骤", command=self._v2_set_stage,
                   style="Primary.TButton").pack(anchor=tk.W, pady=(8, 0))
        ttk.Button(controls, text="停止无桨验收", command=self._v2_stop,
                   style="Danger.TButton").pack(anchor=tk.W, pady=(8, 0))
        ttk.Button(controls, text="读取快照", command=lambda: self._send_proto(
            PROTO_REQ_ACCEPTANCE, "ACCEPT?", "ACCEPT?"),
            style="Secondary.TButton").pack(anchor=tk.W, pady=(8, 0))
        ttk.Label(parent, textvariable=self.v2_status_var, style="Guide.TLabel",
                  wraplength=1120).pack(fill=tk.X, pady=(12, 8))
        live = ttk.LabelFrame(parent, text="目标端只读快照", padding=10); live.pack(fill=tk.X)
        ttk.Label(live, textvariable=self.v2_live_var, font=("Consolas", 10),
                  wraplength=1080).pack(fill=tk.X)
        ttk.Label(
            parent,
            text=("舵机步骤会命令中心值 ±50 µs，但电机始终禁用。请按机体标记目视确认机械方向；"
                  "电机旋向、偏航反扭矩和任何带桨叶旋转项目当前明确不执行。"),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(10, 0))

    def _v2_start(self) -> None:
        gate_ok, reason = self._validation_live_safety_gate()
        if not gate_ok or not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("无桨控制链安全门未通过", reason if not gate_ok else "必须拆桨并隔离动力")
            return
        if not self.transport.send_line("ACCEPT V2 START props=1"):
            messagebox.showerror("无桨控制链验收", "命令发送失败")
            return
        stamp = datetime.now()
        self.v2_session_dir = ensure_directory(
            dated_directory(FLIGHT_ACCEPTANCE_CALIBRATION_DIR, stamp) /
            ("v2a-" + stamp.strftime("%Y%m%d-%H%M%S")))
        (self.v2_session_dir / "session.json").write_text(
            json.dumps({
                "format": "drone-h743-v2a-live-session", "schema": 1,
                "created_at": stamp.astimezone().isoformat(),
                "props_removed": True, "power_safe_confirmed": True,
                "evidence_only": True, "flight_release": False,
                "raw_log": "acceptance_raw.log",
            }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.v2_status_var.set("正在等待目标端确认 ESC CCR=0 和 500 ms 租约…")

    def _v2_set_stage(self) -> None:
        if not self.v2_active or self.v2_lease_id == 0:
            messagebox.showwarning("无桨控制链验收", "请先启动安全模式")
            return
        self.transport.send_line(
            f"ACCEPT V2 STAGE name={self.v2_stage_var.get()} lease={self.v2_lease_id}")

    def _v2_stop(self) -> None:
        if self.v2_lease_id:
            self.transport.send_line(f"ACCEPT V2 STOP lease={self.v2_lease_id}")
        self.v2_active = False; self.v2_lease_id = 0
        self.v2_status_var.set("无桨控制链验收已停止；目标端保持 ESC 禁用")

    def _v2_tick(self) -> None:
        if self.v2_active and self.v2_lease_id:
            if self.transport.is_connected:
                self.transport.send_line(
                    f"ACCEPT V2 KEEPALIVE lease={self.v2_lease_id}")
                self.v2_tick_count += 1
                if (self.v2_tick_count % 4) == 0:
                    self.transport.send_line("ACCEPT?")
            else:
                self.v2_active = False; self.v2_lease_id = 0
                self.v2_status_var.set("连接已断开；500 ms 租约将自动退出并保持 ESC 禁用")
        self.after(250, self._v2_tick)

    def _v2_handle_line(self, line: str) -> None:
        if self.v2_session_dir is not None:
            try:
                with (self.v2_session_dir / "acceptance_raw.log").open(
                    "a", encoding="utf-8") as stream:
                    stream.write(f"{datetime.now().astimezone().isoformat()} {line}\n")
            except OSError as exc:
                self.v2_status_var.set(f"无桨验收日志写入失败：{exc}")
        values = parse_kv(line)
        if values.get("event") == "started":
            lease = safe_int(values.get("lease", "0"), 0)
            if lease > 0 and values.get("esc") == "0,0":
                self.v2_lease_id = lease; self.v2_active = True
                self.v2_status_var.set(
                    f"无桨验收已启动：lease={lease}，ESC CCR=0；自动续租；证据={self.v2_session_dir}")
        elif values.get("event") == "stopped":
            self.v2_active = False; self.v2_lease_id = 0
            self.v2_status_var.set("无桨控制链验收已停止")
        elif "active" in values:
            if values.get("active") != "1" and self.v2_active:
                self.v2_active = False; self.v2_lease_id = 0
                self.v2_status_var.set("无桨验收租约已失效，目标端已自动退出")
            else:
                self.v2_status_var.set(
                    f"无桨验收 active={values.get('active')} stage={values.get('stage')} "
                    f"lease={values.get('lease')} sample={values.get('sample')} seq={values.get('seq')}")
        if line.startswith(("ACCEPT context", "ACCEPT motion", "ACCEPT control", "ACCEPT servo")):
            self.v2_live_var.set(line)


__all__ = ["AcceptanceV2PageMixin"]
