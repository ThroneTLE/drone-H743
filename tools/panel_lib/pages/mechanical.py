"""Servo-mechanical calibration page builder and handlers."""

from __future__ import annotations

import json
import time
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, ttk

from ..proto import (
    PROTO_REQ_IMU,
    PROTO_REQ_SERVO_CAL,
    PROTO_REQ_SERVO_MOVE,
    parse_kv,
    safe_int,
)

try:
    from ...ground_calibration import GroundCalibrationError, validate_servo_geometry
    from ...project_paths import (
        SERVO_MECHANICAL_CALIBRATION_DIR,
        dated_directory,
        ensure_directory,
    )
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools.ground_calibration import GroundCalibrationError, validate_servo_geometry
        from tools.project_paths import (
            SERVO_MECHANICAL_CALIBRATION_DIR,
            dated_directory,
            ensure_directory,
        )
    except ImportError:
        from ground_calibration import GroundCalibrationError, validate_servo_geometry
        from project_paths import (
            SERVO_MECHANICAL_CALIBRATION_DIR,
            dated_directory,
            ensure_directory,
        )


class MechanicalPageMixin:
    def _build_mechanical_calibration_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="ACTUATOR GEOMETRY  /  拆桨地面校准", style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text="舵机机械中心、方向与安全行程", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=(
                "本页用于逐路小步移动、记录真正的机械中立位、脉宽方向和不干涉行程。"
                "确认后可先应用到 RAM 做 A/B 验证，再写入 FCAL 双槽参数 Flash；"
                "预览和写入期间飞控保持硬解锁锁定，重启后必须回读一致才算闭环。"
                "点动为保持型（SERVO JOG）：舵机匀速走到目标后停住，直到点「结束点动」、"
                "超时 120 s 或飞控解锁才交还稳定环。坐标基准 FLU：+X 机头、+Y 左、+Z 上。"
            ),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))

        safety = ttk.LabelFrame(parent, text="动作安全门", padding=8)
        safety.pack(fill=tk.X)
        ttk.Checkbutton(
            safety, text="已拆除全部桨叶",
            variable=self.validation_props_removed_var,
        ).pack(side=tk.LEFT)
        ttk.Checkbutton(
            safety, text="电机不会启动，机体已固定；仅给舵机保留必要电源",
            variable=self.validation_power_safe_var,
        ).pack(side=tk.LEFT, padx=(16, 0))
        ttk.Button(
            safety, text="读取实时安全快照",
            command=lambda: self._send_proto(PROTO_REQ_IMU, "IMU?"),
            style="Secondary.TButton",
        ).pack(side=tk.RIGHT)
        ttk.Button(
            safety, text="结束点动（交还稳定环）",
            command=self._mechanical_jog_stop,
            style="Danger.TButton",
        ).pack(side=tk.RIGHT, padx=(0, 8))

        table = ttk.LabelFrame(parent, text="两路机械记录", padding=8)
        table.pack(fill=tk.X, pady=(8, 0))
        headings = ("通道", "中心 µs", "最小 µs", "最大 µs", "增大脉宽对应", "动作")
        for column, label in enumerate(headings):
            ttk.Label(table, text=label, style="Muted.TLabel").grid(
                row=0, column=column, sticky=tk.W, padx=4, pady=(0, 5)
            )

        for index, (axis, title, flu_guide) in enumerate((
            (
                "alpha", "Alpha · 左右倾转（总线舵机 ID1）",
                "FLU 判向：点「+50」后旋翼轴向机体左侧(+Y)倾 → 选「机构标记正向」；"
                "向右侧(−Y)倾 → 「机构标记反向」。",
            ),
            (
                "beta", "Beta · 前后倾转（总线舵机 ID2）",
                "FLU 判向：点「+50」后旋翼轴向机尾(−X)倾 → 选「机构标记正向」；"
                "向机头(+X)倾 → 「机构标记反向」。",
            ),
        )):
            row = index * 3 + 1
            values: dict[str, tk.Variable] = {
                "axis": tk.StringVar(value=axis),
                "center": tk.IntVar(value=1500),
                "minimum": tk.IntVar(value=1000),
                "maximum": tk.IntVar(value=2000),
                "polarity": tk.StringVar(value="未确认"),
                "center_confirmed": tk.BooleanVar(value=False),
                "direction_confirmed": tk.BooleanVar(value=False),
                "travel_confirmed": tk.BooleanVar(value=False),
            }
            self.mechanical_rows.append(values)
            ttk.Label(table, text=title).grid(row=row, column=0, sticky=tk.W, padx=4, pady=4)
            ttk.Spinbox(table, from_=500, to=2500, increment=1, width=8,
                        textvariable=values["center"]).grid(row=row, column=1, padx=4)
            ttk.Spinbox(table, from_=500, to=2500, increment=1, width=8,
                        textvariable=values["minimum"]).grid(row=row, column=2, padx=4)
            ttk.Spinbox(table, from_=500, to=2500, increment=1, width=8,
                        textvariable=values["maximum"]).grid(row=row, column=3, padx=4)
            ttk.Combobox(
                table, textvariable=values["polarity"], state="readonly", width=18,
                values=("未确认", "机构标记正向", "机构标记反向"),
            ).grid(row=row, column=4, sticky=tk.W, padx=4)
            actions = ttk.Frame(table)
            actions.grid(row=row, column=5, sticky=tk.W, padx=4)
            for label, target in (
                ("中心", "center"), ("-50", "negative"), ("+50", "positive"),
                ("最小", "minimum"), ("最大", "maximum"),
            ):
                ttk.Button(
                    actions, text=label,
                    command=lambda i=index, t=target: self._mechanical_move(i, t),
                    style="Secondary.TButton" if target == "center" else "Warning.TButton",
                ).pack(side=tk.LEFT, padx=2)
            ttk.Label(actions, text="中点微调", style="Muted.TLabel").pack(
                side=tk.LEFT, padx=(10, 2))
            for label, delta in (("-10", -10), ("-2", -2), ("+2", 2), ("+10", 10)):
                ttk.Button(
                    actions, text=label,
                    command=lambda i=index, d=delta: self._mechanical_nudge_center(i, d),
                    style="Secondary.TButton",
                ).pack(side=tk.LEFT, padx=2)

            # 深色主题下不用 Guide 高亮条（作者实测发白刺眼），平铺亮字即可。
            ttk.Label(
                table, text=flu_guide, wraplength=1080,
            ).grid(row=row + 1, column=0, columnspan=6, sticky=tk.W, padx=4, pady=(0, 2))

            confirms = ttk.Frame(table)
            confirms.grid(row=row + 2, column=0, columnspan=6, sticky=tk.W, padx=4, pady=(0, 7))
            ttk.Checkbutton(
                confirms, text="机械中立位已对正", variable=values["center_confirmed"]
            ).pack(side=tk.LEFT)
            ttk.Checkbutton(
                confirms, text="±50 µs 方向已按机体标记确认", variable=values["direction_confirmed"]
            ).pack(side=tk.LEFT, padx=(14, 0))
            ttk.Checkbutton(
                confirms, text="最小/最大位无干涉、无堵转", variable=values["travel_confirmed"]
            ).pack(side=tk.LEFT, padx=(14, 0))

        target = ttk.LabelFrame(parent, text="应用、撤销与持久化", padding=8)
        target.pack(fill=tk.X, pady=(10, 0))
        target_actions = ttk.Frame(target)
        target_actions.pack(fill=tk.X)
        ttk.Button(
            target_actions, text="读取飞控机械参数",
            command=self._mechanical_read_target,
            style="Secondary.TButton",
        ).pack(side=tk.LEFT)
        self.mechanical_apply_button = ttk.Button(
            target_actions, text="应用到 RAM",
            command=self._mechanical_apply_target,
            style="Warning.TButton",
        )
        self.mechanical_apply_button.pack(side=tk.LEFT, padx=(8, 0))
        self.mechanical_revert_button = ttk.Button(
            target_actions, text="撤销 RAM 预览",
            command=self._mechanical_revert_target,
            state=tk.DISABLED, style="Danger.TButton",
        )
        self.mechanical_revert_button.pack(side=tk.LEFT, padx=(8, 0))
        self.mechanical_commit_button = ttk.Button(
            target_actions, text="写入参数 Flash",
            command=self._mechanical_commit_target,
            state=tk.DISABLED, style="Warning.TButton",
        )
        self.mechanical_commit_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(
            target_actions, text="重启后核对",
            command=lambda: self._mechanical_read_target(reboot_check=True),
            style="Secondary.TButton",
        ).pack(side=tk.LEFT, padx=(16, 0))
        ttk.Label(
            target, textvariable=self.mechanical_target_var,
            style="Guide.TLabel", wraplength=1080,
        ).pack(fill=tk.X, pady=(7, 0))

        footer = ttk.Frame(parent)
        footer.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(
            footer, text="保存机械校准证据", command=self._mechanical_save_evidence,
            style="Primary.TButton",
        ).pack(side=tk.LEFT)
        ttk.Label(
            footer, text="只有 persisted 回读与本页数值完全一致，报告才记录 target_parameters_written=true。",
            style="Muted.TLabel",
        ).pack(side=tk.LEFT, padx=(16, 0))
        ttk.Label(
            parent, textvariable=self.mechanical_status_var,
            style="Guide.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(8, 0))
        ttk.Label(
            parent, textvariable=self.mechanical_last_report_var,
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 0))

    def _mechanical_row_values(self, index: int) -> tuple[int, int, int]:
        row = self.mechanical_rows[index]
        center = int(row["center"].get())
        minimum = int(row["minimum"].get())
        maximum = int(row["maximum"].get())
        validate_servo_geometry(center, minimum, maximum)
        return center, minimum, maximum

    def _mechanical_local_target(self, *, require_confirmations: bool) -> dict[str, int]:
        result: dict[str, int] = {}
        prefixes = ("alpha", "beta")
        for index, row in enumerate(self.mechanical_rows):
            center, minimum, maximum = self._mechanical_row_values(index)
            polarity = str(row["polarity"].get())
            if polarity == "未确认":
                raise GroundCalibrationError(f"{prefixes[index]} 方向尚未确认")
            if require_confirmations and not all(bool(row[name].get()) for name in (
                "center_confirmed", "direction_confirmed", "travel_confirmed"
            )):
                raise GroundCalibrationError(f"{prefixes[index]} 的中心、方向或行程确认不完整")
            result[f"{prefixes[index]}_center"] = center
            result[f"{prefixes[index]}_min"] = minimum
            result[f"{prefixes[index]}_max"] = maximum
            result[f"{prefixes[index]}_sign"] = (
                1 if polarity == "机构标记正向" else -1
            )
        return result

    def _mechanical_target_matches_local(self, scope: str) -> bool:
        record = self.mechanical_target_records.get(scope)
        if not record or record.get("valid") != "1":
            return False
        try:
            local = self._mechanical_local_target(require_confirmations=False)
        except (GroundCalibrationError, ValueError, tk.TclError):
            return False
        return all(safe_int(record.get(name), 0) == value for name, value in local.items())

    def _mechanical_read_target(self, *, reboot_check: bool = False) -> None:
        if reboot_check:
            current_generation = self.serial_transport.connection_generation
            if (
                self.mechanical_commit_connection_generation is not None
                and current_generation <= self.mechanical_commit_connection_generation
            ):
                messagebox.showwarning(
                    "尚未检测到重启重连",
                    "写入后请让飞控重启并重新连接，再执行重启后核对。",
                )
                return
            self.mechanical_reboot_check_pending = True
        self._send_proto(PROTO_REQ_SERVO_CAL, "SERVOCAL?", "SERVOCAL?")
        self.mechanical_target_var.set("正在读取飞控 active / persisted 机械参数…")

    def _mechanical_apply_target(self) -> None:
        if not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("舵机机械校准安全门", "必须拆桨、固定机体，并确认电机不会启动。")
            return
        gate_ok, reason = self._validation_live_safety_gate()
        if not gate_ok:
            messagebox.showwarning("舵机机械校准安全门", reason)
            return
        try:
            values = self._mechanical_local_target(require_confirmations=True)
        except (GroundCalibrationError, ValueError, tk.TclError) as exc:
            messagebox.showwarning("机械校准证据不完整", str(exc))
            return
        command = (
            "SERVOCAL APPLY "
            f"ac={values['alpha_center']} an={values['alpha_min']} ax={values['alpha_max']} as={values['alpha_sign']} "
            f"bc={values['beta_center']} bn={values['beta_min']} bx={values['beta_max']} bs={values['beta_sign']}"
        )
        self._send_proto(PROTO_REQ_SERVO_CAL, command, command)
        self.mechanical_target_var.set("已请求应用到 RAM；等待飞控回报 applied=1 和硬解锁锁定")

    def _mechanical_revert_target(self) -> None:
        self._send_proto(PROTO_REQ_SERVO_CAL, "SERVOCAL REVERT", "SERVOCAL REVERT")
        self.mechanical_target_var.set("正在撤销 RAM 机械参数预览…")

    def _mechanical_commit_target(self) -> None:
        if not messagebox.askyesno(
            "写入舵机机械参数",
            "确认 RAM 预览下中心、方向和端点都正确吗？\n\n"
            "写入后仍需重启并点击“重启后核对”。",
        ):
            return
        self._send_proto(PROTO_REQ_SERVO_CAL, "SERVOCAL COMMIT", "SERVOCAL COMMIT")
        self.mechanical_commit_connection_generation = (
            self.serial_transport.connection_generation
        )
        self.mechanical_reboot_verified = False
        self.mechanical_target_var.set("已请求写入参数 Flash；等待 dirty=0 后再重启核对")

    def _mechanical_handle_target_line(self, line: str) -> None:
        values = parse_kv(line)
        scope = values.get("scope")
        if scope in {"active", "persisted"}:
            self.mechanical_target_records[scope] = dict(values)
        event = values.get("event")
        if event:
            self.mechanical_target_applied = values.get("applied") == "1"
            self.mechanical_target_commit_pending = values.get("commit_pending") == "1"
            if event in {"applied", "status", "commit_queued"}:
                pass
            elif event in {"reverted", "committed"}:
                self.mechanical_target_applied = False
            elif "rejected" in event or event == "commit_failed":
                self.mechanical_status_var.set(
                    f"飞控拒绝机械参数操作：event={event} reason={values.get('reason', '-')}"
                )

        persisted_match = self._mechanical_target_matches_local("persisted")
        active_match = self._mechanical_target_matches_local("active")
        if persisted_match and self.mechanical_reboot_check_pending:
            self.mechanical_reboot_verified = True
            self.mechanical_reboot_check_pending = False
        state = (
            f"active={'匹配' if active_match else '未匹配'} · "
            f"persisted={'匹配' if persisted_match else '未匹配'} · "
            f"RAM预览={'是' if self.mechanical_target_applied else '否'} · "
            f"Flash写入中={'是' if self.mechanical_target_commit_pending else '否'} · "
            f"重启复验={'PASS' if self.mechanical_reboot_verified else '未完成'}"
        )
        self.mechanical_target_var.set(state)
        if hasattr(self, "mechanical_revert_button"):
            self.mechanical_apply_button.configure(
                state=(tk.DISABLED if self.mechanical_target_applied or
                       self.mechanical_target_commit_pending else tk.NORMAL)
            )
            self.mechanical_revert_button.configure(
                state=tk.NORMAL if self.mechanical_target_applied else tk.DISABLED
            )
            self.mechanical_commit_button.configure(
                state=(tk.NORMAL if self.mechanical_target_applied and active_match and
                       not self.mechanical_target_commit_pending else tk.DISABLED)
            )
        self.last_reply_rx = time.monotonic()

    def _mechanical_move(self, index: int, target_name: str) -> None:
        if not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("舵机机械校准安全门", "必须拆桨、固定机体，并确认电机不会启动。")
            return
        gate_ok, reason = self._validation_live_safety_gate()
        if not gate_ok:
            messagebox.showwarning("舵机机械校准安全门", reason)
            return
        try:
            center, minimum, maximum = self._mechanical_row_values(index)
        except (GroundCalibrationError, ValueError, tk.TclError) as exc:
            messagebox.showerror("舵机机械参数无效", str(exc))
            return
        target = {
            "center": center,
            "negative": max(minimum, center - 50),
            "positive": min(maximum, center + 50),
            "minimum": minimum,
            "maximum": maximum,
        }.get(target_name)
        if target is None:
            return
        payload = f"SERVO JOG {index} {target}"
        self._send_proto(PROTO_REQ_SERVO_MOVE, payload, payload)
        axis = str(self.mechanical_rows[index]["axis"].get()).upper()
        self.mechanical_status_var.set(
            f"已命令 {axis} 舵机匀速点动到 {target} µs 并保持（500 µs/s，超时 120 s 自动交还）；"
            "目视检查机构，异常立即断开舵机电源"
        )

    def _mechanical_nudge_center(self, index: int, delta_us: int) -> None:
        if not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("舵机机械校准安全门", "必须拆桨、固定机体，并确认电机不会启动。")
            return
        gate_ok, reason = self._validation_live_safety_gate()
        if not gate_ok:
            messagebox.showwarning("舵机机械校准安全门", reason)
            return
        row = self.mechanical_rows[index]
        try:
            minimum = int(row["minimum"].get())
            maximum = int(row["maximum"].get())
            center = int(row["center"].get())
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("舵机机械参数无效", str(exc))
            return
        # 微调后的中心仍须满足 validate_servo_geometry 的两侧各 ≥50 µs 约束。
        center = max(minimum + 50, min(maximum - 50, center + delta_us))
        row["center"].set(center)
        payload = f"SERVO JOG {index} {center}"
        self._send_proto(PROTO_REQ_SERVO_MOVE, payload, payload)
        axis = str(row["axis"].get()).upper()
        self.mechanical_status_var.set(
            f"{axis} 中点微调至 {center} µs，舵机实时跟随；"
            "对正后勾选确认并「应用到 RAM」→「写入参数 Flash」"
        )

    def _mechanical_jog_stop(self) -> None:
        self._send_proto(PROTO_REQ_SERVO_MOVE, "SERVO JOG STOP", "SERVO JOG STOP")
        self.mechanical_status_var.set("已请求结束点动，舵机输出交还稳定环")

    def _mechanical_save_evidence(self) -> None:
        axes: list[dict[str, object]] = []
        try:
            for index, row in enumerate(self.mechanical_rows):
                center, minimum, maximum = self._mechanical_row_values(index)
                polarity = str(row["polarity"].get())
                confirmations = {
                    "center": bool(row["center_confirmed"].get()),
                    "direction": bool(row["direction_confirmed"].get()),
                    "travel": bool(row["travel_confirmed"].get()),
                }
                if polarity == "未确认" or not all(confirmations.values()):
                    raise GroundCalibrationError(
                        f"第 {index + 1} 路尚未完成中心、方向、行程三项人工确认"
                    )
                axes.append({
                    "axis": str(row["axis"].get()),
                    "servo_index": index,
                    "center_us": center,
                    "minimum_us": minimum,
                    "maximum_us": maximum,
                    "positive_pulse_direction": polarity,
                    "physical_confirmations": confirmations,
                })
        except (GroundCalibrationError, ValueError, tk.TclError) as exc:
            messagebox.showwarning("机械校准证据不完整", str(exc))
            return

        stamp = datetime.now().astimezone()
        path = ensure_directory(dated_directory(SERVO_MECHANICAL_CALIBRATION_DIR, stamp)) / (
            "servo_mechanical_" + stamp.strftime("%Y%m%d_%H%M%S") + ".json"
        )
        persisted_match = self._mechanical_target_matches_local("persisted")
        report = {
            "format": "drone-h743-servo-mechanical-evidence",
            "schema": 1,
            "created_at": stamp.isoformat(),
            "body_frame": "FLU",
            "props_removed_confirmed": bool(self.validation_props_removed_var.get()),
            "motor_safe_confirmed": bool(self.validation_power_safe_var.get()),
            "axes": axes,
            "target_parameters_written": persisted_match,
            "target_persisted_readback": self.mechanical_target_records.get("persisted"),
            "reboot_verification_passed": self.mechanical_reboot_verified,
            "flight_release": False,
            "status": (
                "PERSISTED_REBOOT_VERIFIED" if self.mechanical_reboot_verified else
                ("PERSISTED_READBACK_MATCH" if persisted_match else "EVIDENCE_ONLY")
            ),
        }
        try:
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("机械校准证据保存失败", str(exc))
            return
        self.mechanical_last_report_var.set(f"机械测量证据已保存：{path}")
        self.mechanical_status_var.set(
            "机械参数与重启后 Flash 回读均已闭环" if self.mechanical_reboot_verified else
            "机械测量已记录；还需完成 Flash 回读及重启复验"
        )


__all__ = ["MechanicalPageMixin"]
