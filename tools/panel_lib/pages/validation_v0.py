"""V0 coordinate-frame and polarity validation page."""

from __future__ import annotations

import math

from ..connection_state import snapshot_receipt, snapshot_is_current
import statistics
import time
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, ttk

from ..evidence import (
    VALIDATION_HEALTH_FRESH_S,
    VALIDATION_MOTOR_SAFE_MAX_US,
    VALIDATION_ROTATION_TARGET_SAMPLES,
    VALIDATION_SAMPLE_FRESH_S,
    VALIDATION_STATIC_TARGET_SAMPLES,
    VALIDATION_UI_STAGES,
    signed_permutation_descriptor,
    validation_sample_from_snapshot,
)
from ..proto import PROTO_REQ_IMU, PROTO_REQ_IMU_FRAME, parse_kv, safe_int

try:
    from ...flight_validation import (
        STAGE_DEFINITIONS,
        ImuSample,
        ValidationSession,
        ValidationStage,
        ValidationStatus,
        analyze_rotation_stage,
        analyze_six_face,
        analyze_static_stage,
        build_validation_report,
        load_session,
        write_csv_report,
        write_json_report,
        write_session,
    )
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools.flight_validation import (
            STAGE_DEFINITIONS,
            ImuSample,
            ValidationSession,
            ValidationStage,
            ValidationStatus,
            analyze_rotation_stage,
            analyze_six_face,
            analyze_static_stage,
            build_validation_report,
            load_session,
            write_csv_report,
            write_json_report,
            write_session,
        )
    except ImportError:
        from flight_validation import (
            STAGE_DEFINITIONS,
            ImuSample,
            ValidationSession,
            ValidationStage,
            ValidationStatus,
            analyze_rotation_stage,
            analyze_six_face,
            analyze_static_stage,
            build_validation_report,
            load_session,
            write_csv_report,
            write_json_report,
            write_session,
        )


UI_MONO = "Consolas"
UI_SIZE = 10
UI_PALETTE = {
    "green": "#4ADE97",
    "red": "#FF8B82",
    "amber": "#F2B441",
    "accent": "#4DA3F5",
    "muted": "#8D97A6",
    "panel": "#262C36",
    "ink_dim": "#C6CEDA",
    "border": "#39414E",
}


def v0_workflow_guidance(
    *,
    candidate_available: bool,
    candidate_applied: bool,
    verification_mode: bool,
    candidate_verified: bool,
    candidate_committed: bool,
    runtime_ready: bool,
) -> tuple[str, str]:
    """Explain V0 discovery versus post-apply verification without ambiguity."""

    if candidate_committed:
        return (
            "坐标系校准已完成 · FLU 映射已写入参数 Flash",
            "当前坐标映射已经持久化。下一步进入“IMU 零偏与比例”；这仍不代表允许自由飞行。",
        )
    if candidate_verified:
        return (
            "阶段 3/3 · RAM 映射下的 6 步复验已经 PASS",
            "下一步：点击 C 写入 Flash；写入前继续保持拆桨、未解锁和安全输出。",
        )
    if verification_mode:
        return (
            "阶段 2/3 · 正在验证 RAM 中的 FLU 映射",
            "当前表格只看重新采集的 canonical FLU 样本；依次完成 6 个必要步骤，全部 PASS 后进入阶段 3。",
        )
    if candidate_applied:
        return (
            "阶段 2/3 · 候选已临时应用到 RAM，尚未写 Flash",
            "旧 FAIL 是映射前的发现证据，不会自动变成 PASS。下一步：点击 B 清空旧样本，并按新 FLU 坐标重新采集 6 步。",
        )
    if candidate_available:
        if runtime_ready:
            next_action = (
                "旧 FAIL 说明 legacy 轴与 FLU 不一致，它们用于推导候选，不是最终验收。"
                "下一步：完成两项物理确认，然后点击 A 临时应用到 RAM。"
            )
        else:
            next_action = (
                "旧 FAIL 说明 legacy 轴与 FLU 不一致，它们用于推导候选，不是最终验收。"
                "下一步：连接当前飞控；若加载的是历史会话，点击“继续已加载验收”；"
                "勾选拆桨/动力安全门并等待新鲜快照，再完成两项物理确认和 A。"
            )
        return "阶段 1/3 · 已发现 FLU 映射候选，尚未应用", next_action
    return (
        "阶段 1/3 · 采集姿态与正向旋转，发现坐标映射",
        "下一步：按步骤条依次完成必要动作。此阶段出现 FAIL 可以是坐标未映射的预期现象。",
    )


class ValidationV0PageMixin:
    VALIDATION_STEPS = ("坐标发现", "RAM 复验", "写入 Flash")

    _STAGE_ROW_TAGS = {
        "PASS": "pass",
        "FAIL": "fail",
        "WARN": "warn",
        "COLLECTING": "active",
    }

    _GATE_INDICATORS = {
        "ok": ("●", "Pass.TLabel"),
        "wait": ("◐", "Warn.TLabel"),
        "bad": ("●", "Fail.TLabel"),
    }

    _ORIENTATION_EVENT_STYLES = {
        "ok": "Pass.TLabel",
        "wait": "Warn.TLabel",
        "bad": "Fail.TLabel",
    }

    IMU_HEALTH_LABELS = {0: "正常", 1: "降级", 2: "失效"}

    def _build_validation_stepper(self, parent: ttk.Frame) -> None:
        """三段流程用步骤条表达顺序，因此按钮文案里不再需要 1/2/3/4 编号。"""
        strip = ttk.Frame(parent)
        strip.pack(fill=tk.X, pady=(0, 8))
        self.validation_step_dots = []
        for index, name in enumerate(self.VALIDATION_STEPS):
            if index:
                ttk.Label(strip, text="────", style="Muted.TLabel").pack(
                    side=tk.LEFT, padx=7)
            dot = ttk.Label(strip, text=f"○  {name}", style="Idle.TLabel")
            dot.pack(side=tk.LEFT)
            self.validation_step_dots.append(dot)
        self._validation_refresh_stepper()

    def _validation_current_step(self) -> int:
        """0-based 当前阶段；返回 3 表示三段全部完成。"""
        text = self.validation_phase_var.get()
        for index in range(1, len(self.VALIDATION_STEPS) + 1):
            if text.startswith(f"阶段 {index}/{len(self.VALIDATION_STEPS)}"):
                return index - 1
        return len(self.VALIDATION_STEPS) if text.startswith("坐标系校准已完成") else 0

    def _validation_refresh_stepper(self) -> None:
        current = self._validation_current_step()
        for index, label in enumerate(self.validation_step_dots):
            name = self.VALIDATION_STEPS[index]
            if index < current:
                label.configure(text=f"●  {name}", style="Pass.TLabel")
            elif index == current:
                label.configure(text=f"●  {name}", style="Active.TLabel")
            else:
                label.configure(text=f"○  {name}", style="Idle.TLabel")

    def _validation_recolor_stage_rows(self) -> None:
        """按状态列给步骤表上色。

        状态由十来处调用点写入，逐处打标签容易漏；这里统一按当前状态列回填，
        因此新增写入点不会退化成"没颜色"。
        """
        tree = getattr(self, "validation_stage_tree", None)
        if tree is None:
            return
        for item in tree.get_children():
            values = tree.item(item, "values")
            status = str(values[0]).upper() if values else ""
            tree.item(item, tags=(self._STAGE_ROW_TAGS.get(status, "muted"),))

    def _validation_set_gate(self, key: str, state: str, value: str) -> None:
        """状态用指示灯圆点，文字栏只放可读的实测值。"""
        dot = self.validation_gate_dots.get(key)
        if dot is not None:
            glyph, style = self._GATE_INDICATORS.get(state, ("○", "Idle.TLabel"))
            dot.configure(text=glyph, style=style)
        variable = {
            "conn": self.validation_connection_gate_var,
            "snapshot": self.validation_snapshot_gate_var,
            "bias": self.validation_bias_gate_var,
            "output": self.validation_output_gate_var,
            "health": self.validation_health_gate_var,
        }.get(key)
        if variable is not None:
            variable.set(value)

    def _build_validation_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="IMU FRAME  /  建立唯一坐标与极性", style="Eyebrow.TLabel").pack(anchor=tk.W)
        header = ttk.Frame(parent)
        header.pack(fill=tk.X)
        ttk.Label(header, text="IMU 坐标系与极性校准", style="PageTitle.TLabel").pack(side=tk.LEFT)
        self.validation_status_label = ttk.Label(
            header,
            textvariable=self.validation_session_var,
            style="Warn.TLabel",
        )
        self.validation_status_label.pack(side=tk.RIGHT)

        ttk.Label(
            parent,
            text=(
                "采集阶段只读；只有下方 A/B/C 映射流程可在双重确认后改 RAM 或写参数 Flash，"
                "不会重新烧录程序。支持 USB CDC；传感器映射通过仍不等于整机允许自由飞行。"
            ),
            style="Muted.TLabel",
            wraplength=1080,
        ).pack(fill=tk.X, pady=(3, 8))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))
        self._build_validation_stepper(parent)

        safety = ttk.LabelFrame(parent, text="开始前安全门", padding=8)
        safety.pack(fill=tk.X)
        ttk.Checkbutton(
            safety,
            text="已拆除全部桨叶",
            variable=self.validation_props_removed_var,
            command=self._validation_update_safety_text,
        ).pack(side=tk.LEFT)
        ttk.Checkbutton(
            safety,
            text="电调动力已断开或机体已可靠固定",
            variable=self.validation_power_safe_var,
            command=self._validation_update_safety_text,
        ).pack(side=tk.LEFT, padx=(16, 0))
        ttk.Label(safety, textvariable=self.validation_safety_var).pack(side=tk.RIGHT)

        # 采集操作与会话管理分两行：同一行里只允许存在一个实心主操作按钮，
        # 其余一律描边，避免出现"三色按钮并排"的玩具感。
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill=tk.X, pady=(8, 0))
        self.validation_begin_button = ttk.Button(
            toolbar,
            text="开始采集当前步骤",
            command=self._validation_begin_stage,
            state=tk.DISABLED,
            style="Primary.TButton",
        )
        self.validation_begin_button.pack(side=tk.LEFT)
        self.validation_finish_button = ttk.Button(
            toolbar,
            text="停止并分析",
            command=self._validation_finish_stage,
            state=tk.DISABLED,
            style="Secondary.TButton",
        )
        self.validation_finish_button.pack(side=tk.LEFT, padx=(6, 0))
        self.validation_report_button = ttk.Button(
            toolbar,
            text="生成报告",
            command=self._validation_write_report,
            state=tk.DISABLED,
            style="Secondary.TButton",
        )
        self.validation_report_button.pack(side=tk.LEFT, padx=(18, 0))
        ttk.Button(toolbar, text="报告目录", command=self._validation_open_report_dir,
                   style="Link.TButton").pack(side=tk.LEFT, padx=(4, 0))
        ttk.Label(toolbar, text="固件备注", style="Muted.TLabel").pack(side=tk.LEFT, padx=(18, 6))
        ttk.Entry(toolbar, textvariable=self.validation_firmware_hash_var, width=30).pack(
            side=tk.LEFT)

        session_bar = ttk.Frame(parent)
        session_bar.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(session_bar, text="会话", style="Eyebrow.TLabel").pack(side=tk.LEFT, padx=(0, 8))
        self.validation_session_button = ttk.Button(
            session_bar,
            text="新建验收",
            command=self._validation_start_session,
            style="Primary.TButton",
        )
        self.validation_session_button.pack(side=tk.LEFT)
        self.validation_resume_button = ttk.Button(
            session_bar,
            text="恢复/继续",
            command=self._validation_resume_session,
            state=tk.DISABLED,
            style="Secondary.TButton",
        )
        self.validation_resume_button.pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(session_bar, text="暂停", command=self._validation_stop_session,
                   style="Danger.TButton").pack(side=tk.LEFT, padx=(6, 0))
        self.validation_history_combo = ttk.Combobox(
            session_bar, textvariable=self.validation_history_var,
            values=(), state="readonly", width=40,
        )
        self.validation_history_combo.pack(side=tk.LEFT, padx=(18, 6))
        ttk.Button(
            session_bar, text="载入",
            command=self._validation_load_selected_history,
            style="Secondary.TButton",
        ).pack(side=tk.LEFT)
        ttk.Button(
            session_bar, text="刷新",
            command=self._validation_refresh_history_choices,
            style="Link.TButton",
        ).pack(side=tk.LEFT, padx=(4, 0))

        readiness = ttk.LabelFrame(parent, text="就绪条件", padding=(10, 7))
        readiness.pack(fill=tk.X, pady=(9, 0))
        # 两组条件左对齐成 2×2，右侧留一列吸收富余宽度，避免被拉到屏幕两端。
        readiness.columnconfigure(2, minsize=210)
        readiness.columnconfigure(6, minsize=210)
        readiness.columnconfigure(8, weight=1)
        ttk.Frame(readiness).grid(row=0, column=8, sticky=tk.EW)
        gates = (
            ("conn", "连接", self.validation_connection_gate_var),
            ("snapshot", "快照", self.validation_snapshot_gate_var),
            ("bias", "陀螺零偏", self.validation_bias_gate_var),
            ("output", "安全输出", self.validation_output_gate_var),
        )
        for index, (key, name, variable) in enumerate(gates):
            row, base = index % 2, (index // 2) * 4
            dot = ttk.Label(readiness, text="○", style="Idle.TLabel")
            dot.grid(row=row, column=base, sticky=tk.W, pady=1)
            self.validation_gate_dots[key] = dot
            ttk.Label(readiness, text=name, style="Muted.TLabel", width=9).grid(
                row=row, column=base + 1, sticky=tk.W, padx=(5, 0))
            ttk.Label(readiness, textvariable=variable, style="Mono.TLabel").grid(
                row=row, column=base + 2, sticky=tk.W, padx=(4, 26))
        # 采样链单独一行：它一旦降级，上面四项全绿也不代表数据可用。
        health_dot = ttk.Label(readiness, text="○", style="Idle.TLabel")
        health_dot.grid(row=2, column=0, sticky=tk.W, pady=1)
        self.validation_gate_dots["health"] = health_dot
        ttk.Label(readiness, text="采样链", style="Muted.TLabel", width=9).grid(
            row=2, column=1, sticky=tk.W, padx=(5, 0))
        ttk.Label(
            readiness, textvariable=self.validation_health_gate_var, style="Mono.TLabel"
        ).grid(row=2, column=2, columnspan=6, sticky=tk.W, padx=(4, 26))
        ttk.Label(
            readiness,
            textvariable=self.validation_next_action_var,
            style="Guide.TLabel",
            wraplength=1050,
        ).grid(row=3, column=0, columnspan=9, sticky=tk.EW, pady=(8, 0))

        phase_banner = ttk.Frame(parent)
        phase_banner.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(
            phase_banner, textvariable=self.validation_phase_var,
            style="SectionTitle.TLabel", wraplength=1080,
        ).pack(fill=tk.X)
        ttk.Label(
            phase_banner, textvariable=self.validation_phase_action_var,
            style="Muted.TLabel", wraplength=1080,
        ).pack(fill=tk.X, pady=(3, 0))

        mapping = ttk.LabelFrame(parent, text="坐标映射：A 临时应用 → B 重采复验 → C 写入 Flash", padding=8)
        mapping.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(
            mapping,
            textvariable=self.validation_orientation_var,
            wraplength=1040,
            style="SectionTitle.TLabel",
        ).grid(row=0, column=0, columnspan=6, sticky=tk.W)
        ttk.Checkbutton(
            mapping,
            text="我已确认机头、左侧、上方的物理标记",
            variable=self.validation_axis_labels_confirmed_var,
            command=self._validation_refresh_orientation_controls,
        ).grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=(6, 0))
        ttk.Checkbutton(
            mapping,
            text="我已核对图示轴映射且桨叶已拆除",
            variable=self.validation_mapping_confirmed_var,
            command=self._validation_refresh_orientation_controls,
        ).grid(row=1, column=2, columnspan=2, sticky=tk.W, padx=(16, 0), pady=(6, 0))
        self.validation_apply_button = ttk.Button(
            mapping,
            text="A · 临时应用到 RAM",
            command=self._validation_apply_candidate,
            state=tk.DISABLED,
            style="Warning.TButton",
        )
        self.validation_apply_button.grid(row=2, column=0, sticky=tk.W, pady=(7, 0))
        self.validation_revert_button = ttk.Button(
            mapping,
            text="撤销临时映射",
            command=self._validation_revert_candidate,
            state=tk.DISABLED,
            style="Danger.TButton",
        )
        self.validation_revert_button.grid(row=2, column=1, sticky=tk.W, padx=(6, 0), pady=(7, 0))
        self.validation_verify_button = ttk.Button(
            mapping,
            text="B · 清空旧样本并重采 6 步",
            command=self._validation_begin_candidate_verification,
            state=tk.DISABLED,
            style="Primary.TButton",
        )
        self.validation_verify_button.grid(row=2, column=2, sticky=tk.W, padx=(16, 0), pady=(7, 0))
        self.validation_commit_button = ttk.Button(
            mapping,
            text="C · 复验通过后写入 Flash",
            command=self._validation_commit_candidate,
            state=tk.DISABLED,
            style="Warning.TButton",
        )
        self.validation_commit_button.grid(row=2, column=3, sticky=tk.W, padx=(6, 0), pady=(7, 0))
        ttk.Button(
            mapping,
            text="读取飞机映射状态",
            command=lambda: self._send_proto(PROTO_REQ_IMU_FRAME, "IMUFRAME?"),
            style="Secondary.TButton",
        ).grid(row=2, column=4, sticky=tk.W, padx=(16, 0), pady=(7, 0))
        ttk.Label(mapping, text="飞机回报", style="Muted.TLabel").grid(
            row=3, column=0, sticky=tk.W, pady=(8, 0))
        ttk.Label(
            mapping,
            textvariable=self.validation_orientation_target_var,
            style="Mono.TLabel",
        ).grid(row=3, column=1, columnspan=5, sticky=tk.W, pady=(8, 0))
        ttk.Label(mapping, text="最近操作", style="Muted.TLabel").grid(
            row=4, column=0, sticky=tk.W, pady=(2, 0))
        self.validation_orientation_event_label = ttk.Label(
            mapping,
            textvariable=self.validation_orientation_event_var,
            style="Idle.TLabel",
            wraplength=980,
        )
        self.validation_orientation_event_label.grid(
            row=4, column=1, columnspan=5, sticky=tk.W, pady=(2, 0))
        ttk.Label(
            mapping,
            text="首次使用仍需烧录一次支持 IMUFRAME 的固件；之后改映射无需重新编程。",
            style="Muted.TLabel",
        ).grid(row=5, column=0, columnspan=6, sticky=tk.W, pady=(5, 0))

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        steps_box = ttk.LabelFrame(
            panes, text="坐标发现样本（旧 FAIL 用于推导映射）", padding=8)
        self.validation_steps_box = steps_box
        detail_box = ttk.LabelFrame(panes, text="当前步骤与只读证据", padding=10)
        panes.add(steps_box, weight=2)
        panes.add(detail_box, weight=5)

        self.validation_stage_tree = ttk.Treeview(
            steps_box,
            columns=("status", "samples"),
            show="tree headings",
            height=12,
            selectmode="browse",
        )
        self.validation_stage_tree.heading("#0", text="步骤")
        self.validation_stage_tree.heading("status", text="状态")
        self.validation_stage_tree.heading("samples", text="样本")
        self.validation_stage_tree.column("#0", width=170, anchor=tk.W)
        self.validation_stage_tree.column("status", width=100, anchor=tk.CENTER)
        self.validation_stage_tree.column("samples", width=70, anchor=tk.CENTER)
        self.validation_stage_tree.pack(fill=tk.BOTH, expand=True)
        for tag, color in (
            ("pass", UI_PALETTE["green"]),
            ("fail", UI_PALETTE["red"]),
            ("warn", UI_PALETTE["amber"]),
            ("active", UI_PALETTE["accent"]),
            ("muted", UI_PALETTE["muted"]),
        ):
            self.validation_stage_tree.tag_configure(tag, foreground=color)
        for stage in VALIDATION_UI_STAGES:
            definition = STAGE_DEFINITIONS[stage]
            self.validation_stage_tree.insert(
                "",
                tk.END,
                iid=stage.value,
                text=definition.label_zh,
                values=(ValidationStatus.NOT_RUN.value, 0),
            )
        first_iid = VALIDATION_UI_STAGES[0].value
        self.validation_stage_tree.selection_set(first_iid)
        self.validation_stage_tree.focus(first_iid)
        self.validation_stage_tree.bind("<<TreeviewSelect>>", self._on_validation_stage_select)

        ttk.Label(detail_box, text="动作说明", style="SectionTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            detail_box,
            textvariable=self.validation_instruction_var,
            wraplength=720,
        ).pack(fill=tk.X, pady=(2, 8))
        ttk.Label(detail_box, textvariable=self.validation_source_var, style="Muted.TLabel").pack(anchor=tk.W)
        ttk.Label(
            detail_box,
            textvariable=self.validation_candidate_var,
            style="Muted.TLabel",
            wraplength=720,
        ).pack(fill=tk.X, pady=(4, 0))
        ttk.Label(detail_box, textvariable=self.validation_sample_var).pack(anchor=tk.W, pady=(4, 0))
        self.validation_progress = ttk.Progressbar(detail_box, mode="determinate", maximum=100.0)
        self.validation_progress.pack(fill=tk.X, pady=(4, 8))

        live_box = ttk.LabelFrame(detail_box, text="最新完整快照", padding=8)
        live_box.pack(fill=tk.X)
        ttk.Label(
            live_box,
            textvariable=self.validation_live_var,
            font=("Consolas", 10),
            wraplength=720,
        ).pack(fill=tk.X)

        ttk.Label(detail_box, text="步骤判定", style="SectionTitle.TLabel").pack(anchor=tk.W, pady=(10, 3))
        self.validation_result_text = tk.Text(detail_box, height=10, wrap=tk.WORD)
        self.validation_result_text.configure(
            font=(UI_MONO, UI_SIZE), state=tk.DISABLED,
            background=UI_PALETTE["panel"], foreground=UI_PALETTE["ink_dim"],
            relief=tk.FLAT, highlightthickness=1,
            highlightbackground=UI_PALETTE["border"],
            padx=10, pady=8,
        )
        self.validation_result_text.pack(fill=tk.BOTH, expand=True)
        ttk.Label(detail_box, textvariable=self.validation_report_var, style="Muted.TLabel").pack(fill=tk.X, pady=(6, 0))

        self._validation_update_safety_text()
        self._validation_refresh_history_choices()
        self._validation_refresh_readiness()

    def _validation_update_safety_text(self) -> None:
        ready = self.validation_props_removed_var.get() and self.validation_power_safe_var.get()
        self.validation_safety_var.set(
            "安全门：人工确认完成" if ready else "安全门：必须拆桨并隔离动力"
        )
        if hasattr(self, "validation_begin_button"):
            self._validation_refresh_readiness()

    def _validation_refresh_readiness(self) -> None:
        values = self.validation_latest_values
        now = time.monotonic()
        connected = self._transport_connected()
        fresh = (
            self.validation_latest_host_time > 0.0
            and snapshot_is_current(self)
            and (now - self.validation_latest_host_time) <= VALIDATION_SAMPLE_FRESH_S
        )
        snapshot_ok = (
            fresh
            and values.get("valid") == "1"
            and values.get("source") == "stabilizer_snapshot"
            and (
                values.get("frame", "").startswith("canonical_flu")
                if self.validation_candidate_applied
                else values.get("frame") == "legacy_intermediate"
            )
            and values.get("units") == "mg_mdps_cdeg"
            and safe_int(values.get("contract"), -1) == 1
            and safe_int(values.get("migration"), -1) == 0
            and (
                values.get("orientation")
                == self.validation_orientation_values.get("active_code")
                if self.validation_candidate_applied
                else True
            )
        )
        bias_ok = snapshot_ok and values.get("bias") == "1"
        motor_1 = safe_int(values.get("m1"), 65535)
        motor_2 = safe_int(values.get("m2"), 65535)
        output_ok = (
            snapshot_ok
            and safe_int(values.get("armed"), 1) == 0
            and motor_1 <= VALIDATION_MOTOR_SAFE_MAX_US
            and motor_2 <= VALIDATION_MOTOR_SAFE_MAX_US
        )
        manual_safe = (
            self.validation_props_removed_var.get()
            and self.validation_power_safe_var.get()
        )

        self._validation_set_gate(
            "conn",
            "ok" if connected else "bad",
            f"{self._transport_label()} 已连接" if connected else "未连接 · 点顶部“启动连接”",
        )
        if snapshot_ok:
            self._validation_set_gate(
                "snapshot", "ok", f"seq={self.validation_latest_sequence}")
        elif fresh:
            self._validation_set_gate(
                "snapshot", "bad", "provenance 不兼容 · 需烧录最新固件")
        else:
            self._validation_set_gate("snapshot", "wait", "自动请求中…")
        self._validation_set_gate(
            "bias",
            "ok" if bias_ok else "wait",
            "ready" if bias_ok else "保持机体静止…",
        )
        self._validation_set_gate(
            "output",
            "ok" if output_ok else "bad",
            f"armed=0  m1={motor_1}  m2={motor_2}"
            if output_ok
            else f"armed={values.get('armed', '-')}  m1={values.get('m1', '-')}  m2={values.get('m2', '-')}",
        )

        health_state, health_text, health_ok = self._validation_health_state()
        self._validation_set_gate("health", health_state, health_text)

        stage = self._validation_selected_stage()
        self.validation_begin_button.configure(
            text=f"开始采集 · {STAGE_DEFINITIONS[stage].label_zh}"
        )
        ready_to_collect = (
            self.validation_session_active
            and connected
            and snapshot_ok
            and bias_ok
            and output_ok
            and manual_safe
            and health_ok
            and self.validation_active_stage is None
        )
        self.validation_begin_button.configure(
            state=tk.NORMAL if ready_to_collect else tk.DISABLED
        )
        self.validation_finish_button.configure(
            state=tk.NORMAL if self.validation_active_stage is not None else tk.DISABLED
        )
        has_restorable_state = any(self.validation_samples.values()) or bool(
            self.validation_skipped_stages or self.validation_unsupported_stages
        )
        self.validation_resume_button.configure(
            state=tk.NORMAL
            if has_restorable_state and not self.validation_session_active
            else tk.DISABLED
        )
        has_evidence = any(self.validation_samples.values()) or bool(
            self.validation_unsupported_stages
        )
        self.validation_report_button.configure(
            state=tk.NORMAL
            if has_evidence and self.validation_active_stage is None
            else tk.DISABLED
        )

        if not self.validation_session_active and self.validation_loaded_history and has_restorable_state:
            next_action = (
                "已加载历史样本：若只查看可直接生成报告；若要补测，勾选安全门、连接飞控，"
                "再点击“继续已加载验收”"
            )
        elif not self.validation_session_active:
            next_action = "下一步：恢复已有历史，或勾选两项安全确认后点击“新建验收”"
        elif not connected:
            next_action = "下一步：点击顶部“启动连接”，坐标系校准页会自动轮询 IMU"
        elif not snapshot_ok:
            if fresh:
                next_action = "当前固件快照格式不兼容；请烧录最新固件后重新连接"
            else:
                next_action = "正在自动请求有效快照，请保持连接；超过2秒仍无数据请检查COM是否为飞控USB CDC"
        elif not bias_ok:
            next_action = "下一步：保持飞机完全静止，等待陀螺零偏ready"
        elif not output_ok:
            next_action = "安全输出未通过：确认未解锁，并断开电调动力"
        elif not health_ok:
            next_action = (
                f"采样链异常（{health_text}）：IMU DRDY 中断很可能失效，采样已退到 20ms 轮询。"
                "此状态下滤波与抗混叠假设均不成立，采到的数据不能用于标定，飞控也已禁止解锁。"
                "请检查 PC0 中断线与 IMU 排针，恢复后本项会自动转正常。"
            )
        elif not manual_safe:
            next_action = "下一步：勾选“已拆桨”和“动力已隔离/机体固定”"
        elif self.validation_active_stage is not None:
            next_action = (
                f"正在采集“{STAGE_DEFINITIONS[self.validation_active_stage].label_zh}”；"
                "保持动作，样本达到建议值后点击“停止并分析”"
            )
        else:
            next_action = (
                f"准备完成：按右侧说明摆放/转动飞机，然后点击“开始采集 · "
                f"{STAGE_DEFINITIONS[stage].label_zh}”"
            )
        self.validation_next_action_var.set(next_action)
        self._validation_refresh_orientation_controls()

    def _validation_set_status(self, status: ValidationStatus, text: str) -> None:
        self.validation_session_var.set(f"{status.value} · {text}")
        style = {
            ValidationStatus.PASS: "Pass.TLabel",
            ValidationStatus.WARN: "Warn.TLabel",
            ValidationStatus.SKIPPED: "Muted.TLabel",
            ValidationStatus.FAIL: "Fail.TLabel",
            ValidationStatus.UNSUPPORTED: "Fail.TLabel",
            ValidationStatus.NOT_RUN: "Warn.TLabel",
        }[status]
        self.validation_status_label.configure(style=style)

    def _validation_selected_stage(self) -> ValidationStage:
        selected = self.validation_stage_tree.selection()
        value = selected[0] if selected else self.validation_stage_var.get()
        try:
            return ValidationStage(value)
        except ValueError:
            return VALIDATION_UI_STAGES[0]

    def _on_validation_stage_select(self, _event: tk.Event | None = None) -> None:
        stage = self._validation_selected_stage()
        self.validation_stage_var.set(stage.value)
        self.validation_instruction_var.set(STAGE_DEFINITIONS[stage].prompt_zh)
        self._validation_refresh_stage_view(stage)
        self._validation_refresh_readiness()

    def _validation_refresh_stage_view(self, stage: ValidationStage | None = None) -> None:
        stage = stage or self._validation_selected_stage()
        samples = self.validation_samples.get(stage, [])
        target = (
            VALIDATION_STATIC_TARGET_SAMPLES
            if STAGE_DEFINITIONS[stage].kind == "static"
            else VALIDATION_ROTATION_TARGET_SAMPLES
        )
        self.validation_sample_var.set(f"当前步骤样本：{len(samples)} / 建议 {target}")
        self.validation_progress.configure(value=min(100.0, len(samples) * 100.0 / target))
        result = self.validation_stage_results.get(stage)
        if stage in self.validation_skipped_stages:
            text = "该旧版可选步骤已跳过；不影响必要姿态与旋转验收。"
        elif stage in self.validation_unsupported_stages:
            text = "该步骤的数据源不受支持，已有样本不能作为通过证据。"
        elif result is None:
            text = "尚未分析。开始步骤后按提示缓慢操作，采集完成再点击“停止并分析”。"
        else:
            lines = [
                f"状态：{getattr(result, 'status').value}",
                f"样本：{getattr(result, 'sample_count', len(samples))}",
            ]
            if hasattr(result, "capture_quality_status"):
                lines.append(
                    f"采集质量：{getattr(result, 'capture_quality_status').value}"
                )
            if hasattr(result, "canonical_match_status"):
                match_status = getattr(result, "canonical_match_status")
                lines.append(f"当前数据直接符合FLU：{match_status.value}")
                if (
                    getattr(result, "capture_quality_status", None)
                    is ValidationStatus.PASS
                    and match_status is ValidationStatus.FAIL
                ):
                    lines.append(
                        "解释：采集本身稳定，但当前legacy轴与目标FLU不一致；"
                        "这通常用于推导候选映射，不代表你摆放错误。"
                    )
                if self.validation_verification_mode:
                    consistency, _error, reason = self._validation_static_fusion_consistency(stage)
                    lines.append(f"加速度倾角 ↔ Fusion：{consistency.value} · {reason}")
            if getattr(result, "accel_mean_g", None) is not None:
                lines.append(f"加速度均值 g：{getattr(result, 'accel_mean_g')}")
                lines.append(f"陀螺均值 dps：{getattr(result, 'gyro_mean_dps')}")
                lines.append(f"|a| 均值 g：{getattr(result, 'accel_norm_mean_g'):.4f}")
            if getattr(result, "integrated_angle_deg", None) is not None:
                lines.append(f"陀螺积分 deg：{getattr(result, 'integrated_angle_deg')}")
                lines.append(f"主轴：{getattr(result, 'dominant_axis', '-')}")
            findings = getattr(result, "findings", ())
            if findings:
                lines.append("\n".join(f"• {finding}" for finding in findings))
            text = "\n".join(lines)
        self.validation_result_text.configure(state=tk.NORMAL)
        self.validation_result_text.delete("1.0", tk.END)
        self.validation_result_text.insert("1.0", text)
        self.validation_result_text.configure(state=tk.DISABLED)

    def _validation_refresh_candidate_summary(self) -> None:
        stage_inputs = {
            stage: None if stage in self.validation_unsupported_stages else tuple(samples)
            for stage, samples in self.validation_samples.items()
            if samples or stage in self.validation_unsupported_stages
        }
        result = analyze_six_face(stage_inputs)
        candidate = result.candidate
        if self.validation_verification_mode:
            completed = sum(
                1 for stage in VALIDATION_UI_STAGES if stage in self.validation_finished_stages
            )
            self.validation_candidate_verified = self._validation_candidate_verification_passed()
            message = (
                f"映射后复验：必要步骤 {completed}/{len(VALIDATION_UI_STAGES)} · "
                f"当前数据直接符合FLU={result.canonical_match_status.value} · "
                f"{'已满足写入条件' if self.validation_candidate_verified else '尚未满足写入条件'}"
            )
        elif candidate.complete and candidate.matrix_flu_from_observed is not None:
            matrix = candidate.matrix_flu_from_observed
            descriptor = signed_permutation_descriptor(matrix)
            mapping = ", ".join(
                f"{key}={value}" for key, value in candidate.axis_mapping.items()
            )
            message = (
                f"轴映射候选：{candidate.status.value} · R_FLU←observed={matrix} · "
                f"det={candidate.determinant} · confidence={candidate.confidence:.2%} · {mapping} · 未应用"
            )
            if result.canonical_match_status is ValidationStatus.FAIL:
                message += "。采集质量可用但当前运行轴不直接符合FLU；请在下方二次确认后先做RAM试用。"
            if descriptor is not None:
                self.validation_candidate_matrix = tuple(tuple(row) for row in matrix)
                self.validation_candidate_descriptor = descriptor
                self.validation_candidate_confidence = candidate.confidence
                self.validation_candidate_source_path = (
                    self.validation_loaded_report_path or self.validation_session_path
                )
        else:
            required_static = tuple(
                stage
                for stage in VALIDATION_UI_STAGES
                if STAGE_DEFINITIONS[stage].kind == "static"
            )
            completed = sum(1 for stage in required_static if self.validation_samples.get(stage))
            message = f"轴映射候选：必要静态姿态已完成 {completed}/{len(required_static)}。"
        self.validation_candidate_var.set(message)
        if hasattr(self, "validation_apply_button"):
            self._validation_refresh_orientation_controls()

    def _validation_candidate_verification_passed(self) -> bool:
        if not self.validation_verification_mode or not self.validation_candidate_applied:
            return False
        for stage in VALIDATION_UI_STAGES:
            if stage not in self.validation_finished_stages:
                return False
            result = self.validation_stage_results.get(stage)
            if result is None or getattr(result, "status", None) is not ValidationStatus.PASS:
                return False
            if (
                hasattr(result, "canonical_match_status")
                and getattr(result, "canonical_match_status") is not ValidationStatus.PASS
            ):
                return False
            if STAGE_DEFINITIONS[stage].kind == "static":
                consistency, _error, _reason = self._validation_static_fusion_consistency(stage)
                if consistency is not ValidationStatus.PASS:
                    return False
            else:
                if getattr(result, "gyro_direction_matches_flu", None) is not True:
                    return False
                if getattr(result, "attitude_direction_matches_flu", None) is not True:
                    return False
        return True

    def _validation_static_fusion_consistency(
        self,
        stage: ValidationStage,
    ) -> tuple[ValidationStatus, float | None, str]:
        samples = self.validation_samples.get(stage, [])
        errors: list[float] = []

        def wrapped_error(actual_deg: float, expected_deg: float) -> float:
            return abs((actual_deg - expected_deg + 180.0) % 360.0 - 180.0)

        for sample in samples:
            horizontal = math.sqrt(
                sample.accel_y_g * sample.accel_y_g
                + sample.accel_z_g * sample.accel_z_g
            )
            roll_acc = math.degrees(math.atan2(sample.accel_y_g, sample.accel_z_g))
            pitch_acc = math.degrees(math.atan2(-sample.accel_x_g, horizontal))
            if stage is ValidationStage.LEVEL:
                if sample.roll_deg is None or sample.pitch_deg is None:
                    continue
                errors.extend(
                    (
                        wrapped_error(sample.roll_deg, roll_acc),
                        wrapped_error(sample.pitch_deg, pitch_acc),
                    )
                )
            elif stage is ValidationStage.NOSE_UP:
                if sample.pitch_deg is None:
                    continue
                errors.append(wrapped_error(sample.pitch_deg, pitch_acc))
            elif stage is ValidationStage.LEFT_SIDE_UP:
                if sample.roll_deg is None:
                    continue
                errors.append(wrapped_error(sample.roll_deg, roll_acc))

        minimum = 40 if stage is ValidationStage.LEVEL else 20
        if len(errors) < minimum:
            return ValidationStatus.FAIL, None, "缺少足够的Fusion姿态样本"
        median_error = statistics.median(errors)
        if median_error <= 8.0:
            status = ValidationStatus.PASS
        elif median_error <= 15.0:
            status = ValidationStatus.WARN
        else:
            status = ValidationStatus.FAIL
        return (
            status,
            median_error,
            f"加速度倾角与Fusion中位误差={median_error:.2f}°（PASS≤8°）",
        )

    def _validation_candidate_level_ready(self) -> tuple[bool, str]:
        matrix = self.validation_candidate_matrix
        values = self.validation_latest_values
        if matrix is None:
            return False, "候选矩阵尚未形成"
        try:
            observed = (
                float(values["ax_mg"]) * 0.001,
                float(values["ay_mg"]) * 0.001,
                float(values["az_mg"]) * 0.001,
            )
            gyro = tuple(
                float(values[name]) * 0.001
                for name in ("gx_mdps", "gy_mdps", "gz_mdps")
            )
        except (KeyError, ValueError):
            return False, "等待完整实时IMU快照"
        predicted = tuple(
            sum(matrix[row][column] * observed[column] for column in range(3))
            for row in range(3)
        )
        if (
            abs(predicted[0]) > 0.15
            or abs(predicted[1]) > 0.15
            or not 0.85 <= predicted[2] <= 1.15
        ):
            return False, (
                "临时应用前请把飞机水平放稳；候选换算后的比力应接近[0,0,+1]g，"
                f"当前为[{predicted[0]:+.3f},{predicted[1]:+.3f},{predicted[2]:+.3f}]g"
            )
        if max(abs(value) for value in gyro) > 3.0:
            return False, "临时应用前请保持飞机静止（gyro需小于3 dps）"
        return True, ""

    def _validation_refresh_orientation_controls(self) -> None:
        if not hasattr(self, "validation_apply_button"):
            return
        manual_confirmed = (
            self.validation_axis_labels_confirmed_var.get()
            and self.validation_mapping_confirmed_var.get()
            and self.validation_props_removed_var.get()
            and self.validation_power_safe_var.get()
        )
        connected = self._transport_connected()
        candidate_ready = bool(
            self.validation_candidate_descriptor
            and self.validation_candidate_matrix is not None
            and self.validation_candidate_confidence >= 0.75
        )
        safe, _reason = self._validation_latest_is_safe()
        level_ready, level_reason = self._validation_candidate_level_ready()
        can_apply = (
            candidate_ready
            and manual_confirmed
            and connected
            and safe
            and level_ready
            and not self.validation_candidate_applied
            and not self.validation_orientation_pending
        )
        self.validation_apply_button.configure(state=tk.NORMAL if can_apply else tk.DISABLED)
        self.validation_revert_button.configure(
            state=tk.NORMAL
            if self.validation_candidate_applied and connected and safe and not self.validation_orientation_pending
            else tk.DISABLED
        )
        self.validation_verify_button.configure(
            state=tk.NORMAL
            if self.validation_candidate_applied
            and not self.validation_verification_mode
            and not self.validation_orientation_pending
            else tk.DISABLED
        )
        self.validation_candidate_verified = self._validation_candidate_verification_passed()
        self.validation_commit_button.configure(
            state=tk.NORMAL
            if self.validation_candidate_verified
            and not self.validation_candidate_committed
            and connected
            and safe
            and not self.validation_orientation_pending
            else tk.DISABLED
        )
        if not candidate_ready:
            text = "应用状态：必要静态姿态尚未形成合法 det=+1 候选。"
        else:
            matrix = self.validation_candidate_matrix
            text = (
                f"候选图示：FLU[X前,Y左,Z上] ← observed[{self.validation_candidate_descriptor}] · "
                f"R={matrix} · confidence={self.validation_candidate_confidence:.2%}"
            )
            if self.validation_candidate_committed:
                text += " · 已由目标确认写入 Flash；仍受后续整机验收门限制"
            elif self.validation_candidate_verified:
                text += " · RAM复验全部PASS，可执行C写入Flash"
            elif self.validation_candidate_applied:
                text += " · 已临时应用到RAM，断电恢复；点击B开始必要步骤复验"
            else:
                text += " · 尚未应用"
                if not level_ready:
                    text += f" · {level_reason}"
        self.validation_orientation_var.set(text)
        phase, action = v0_workflow_guidance(
            candidate_available=candidate_ready,
            candidate_applied=self.validation_candidate_applied,
            verification_mode=self.validation_verification_mode,
            candidate_verified=self.validation_candidate_verified,
            candidate_committed=self.validation_candidate_committed,
            runtime_ready=(
                connected and safe and level_ready
                and self.validation_props_removed_var.get()
                and self.validation_power_safe_var.get()
            ),
        )
        self.validation_phase_var.set(phase)
        self.validation_phase_action_var.set(action)
        self._validation_refresh_stepper()
        self._validation_recolor_stage_rows()
        if hasattr(self, "validation_steps_box"):
            self.validation_steps_box.configure(
                text=(
                    "RAM 映射后复验步骤（只看重新采集的新样本）"
                    if self.validation_verification_mode
                    else "坐标发现样本（旧 FAIL 用于推导映射，不是最终失败）"
                )
            )

    def _validation_apply_candidate(self) -> None:
        self._validation_refresh_orientation_controls()
        if str(self.validation_apply_button["state"]) != str(tk.NORMAL):
            return
        descriptor = self.validation_candidate_descriptor
        if not messagebox.askyesno(
            "临时应用坐标映射",
            "将只在 RAM 中临时应用下列映射，断电会恢复；飞控必须保持拆桨/动力隔离。\n\n"
            f"FLU [X前,Y左,Z上] ← observed [{descriptor}]\n"
            f"置信度 {self.validation_candidate_confidence:.2%}\n\n继续？",
        ):
            return
        self.validation_orientation_pending = "apply"
        self._validation_send_orientation_command(f"IMUFRAME APPLY {descriptor}")
        self._validation_note_orientation_event("wait", "已发出 IMUFRAME APPLY；等待飞机确认 RAM 映射")
        self._validation_refresh_orientation_controls()

    def _validation_revert_candidate(self) -> None:
        if not messagebox.askyesno("撤销临时映射", "恢复飞机 Flash 中原有映射并清除本次复验状态？"):
            return
        self.validation_orientation_pending = "revert"
        self._validation_send_orientation_command("IMUFRAME REVERT")
        self._validation_refresh_orientation_controls()

    def _validation_begin_candidate_verification(self) -> None:
        if not self.validation_candidate_applied:
            return
        self._validation_autosave_session(force=True)
        self.validation_verification_mode = True
        self.validation_candidate_verified = False
        self.validation_session_active = True
        self.validation_poll_requested = True
        self.validation_active_stage = None
        self.validation_samples = {stage: [] for stage in STAGE_DEFINITIONS}
        self.validation_stage_results.clear()
        self.validation_unsupported_stages.clear()
        self.validation_skipped_stages.clear()
        self.validation_finished_stages.clear()
        self.validation_raw_rows.clear()
        self.validation_session_path = None
        self.validation_session_created_at = datetime.now().astimezone().isoformat()
        self.validation_report_paths = None
        for stage in VALIDATION_UI_STAGES:
            self.validation_stage_tree.item(
                stage.value,
                values=(ValidationStatus.NOT_RUN.value, 0),
            )
        first_stage = VALIDATION_UI_STAGES[0]
        self.validation_stage_tree.selection_set(first_stage.value)
        self.validation_stage_tree.focus(first_stage.value)
        self.validation_stage_var.set(first_stage.value)
        self.validation_instruction_var.set(STAGE_DEFINITIONS[first_stage].prompt_zh)
        self._validation_set_status(
            ValidationStatus.NOT_RUN,
            "RAM候选复验已开始；只采集6个必要步骤，全部PASS后才能写Flash",
        )
        self._validation_refresh_stage_view(first_stage)
        self._validation_refresh_candidate_summary()
        self._validation_refresh_readiness()

    def _validation_commit_candidate(self) -> None:
        self.validation_candidate_verified = self._validation_candidate_verification_passed()
        if not self.validation_candidate_verified:
            messagebox.showwarning("复验未通过", "6 个必要步骤必须全部 PASS，不能写入 Flash。")
            return
        if not messagebox.askyesno(
            "写入飞机参数 Flash",
            "RAM 映射已通过必要步骤复验。现在只持久化 IMU 坐标映射参数，"
            "不重新烧录固件；写入完成前请勿断电。继续？",
        ):
            return
        self.validation_orientation_pending = "commit"
        self.validation_orientation_poll_count = 0
        self._validation_send_orientation_command("IMUFRAME COMMIT")
        self._validation_note_orientation_event("wait", "已发出 IMUFRAME COMMIT；等待目标回报 dirty=0")
        self._validation_refresh_orientation_controls()

    def _validation_note_orientation_event(self, state: str, text: str) -> None:
        """写映射操作回执。

        带时间戳，因此即使连续两次回包内容相同，也能看出"刚刚确实收到了"。
        这个变量不参与 _validation_refresh_orientation_controls 的重算，
        否则提示会在 0.5s 内被候选摘要覆盖掉。
        """
        self.validation_orientation_event_var.set(f"{time.strftime('%H:%M:%S')} · {text}")
        label = getattr(self, "validation_orientation_event_label", None)
        if label is not None:
            label.configure(
                style=self._ORIENTATION_EVENT_STYLES.get(state, "Idle.TLabel"))

    def _validation_send_orientation_command(self, command: str) -> None:
        normalized = " ".join(command.strip().upper().split())
        self.validation_authorized_orientation_commands.add(normalized)
        self._send_proto(PROTO_REQ_IMU_FRAME, command, command)

    def _validation_poll_orientation_commit(self) -> None:
        if self.validation_orientation_pending != "commit":
            return
        self.validation_orientation_poll_count += 1
        if self.validation_orientation_poll_count > 20:
            self.validation_orientation_pending = ""
            self._validation_note_orientation_event(
                "bad",
                "Flash 写入确认超时：10 秒内没有收到 dirty=0；保持拆桨，点击“读取飞机映射状态”复查",
            )
            self._validation_refresh_orientation_controls()
            return
        self._send_proto_silent(PROTO_REQ_IMU_FRAME, "IMUFRAME?")
        self.after(500, self._validation_poll_orientation_commit)

    def _validation_handle_imu_frame_line(self, line: str) -> None:
        values = parse_kv(line)
        self.validation_orientation_values = dict(values)
        self.last_reply_rx = time.monotonic()
        active = values.get("active", "")
        persisted = values.get("persisted", "")
        event = values.get("event", "status")
        dirty = values.get("dirty", "-")
        base = values.get("base", "")
        commit_was_pending = self.validation_orientation_pending == "commit"
        apply_was_pending = self.validation_orientation_pending == "apply"
        # 无论后面走哪条分支，先把飞机原原本本回报的事实显示出来：点“读取飞机
        # 映射状态”至少要看得见这一行变化。
        self.validation_orientation_target_var.set(
            f"active={active or '-'}  persisted={persisted or '-'}  "
            f"dirty={dirty}  event={event}"
        )
        if base and base != "legacy_intermediate_v1":
            self.validation_orientation_pending = ""
            self._validation_note_orientation_event(
                "bad", f"拒绝使用目标映射：base={base}，期望 legacy_intermediate_v1")
            self._validation_refresh_orientation_controls()
            return

        reported = False
        candidate_matches_active = bool(
            self.validation_candidate_descriptor
            and active == self.validation_candidate_descriptor
        )
        failure_events = {
            "safety_blocked": "飞机安全门拒绝：必须 disarmed 且电机输出≤1100us",
            "apply_busy": "参数后台写入中，暂不能切换 RAM 映射",
            "revert_busy": "参数后台写入中，暂不能撤销映射",
            "commit_busy": "已有另一映射正在写入，拒绝覆盖",
            "param_not_ready": "参数服务尚未就绪",
            "invalid_descriptor": "飞机拒绝了非法/镜像轴映射",
            "invalid_usage": "IMUFRAME 命令格式不兼容",
            "revert_failed": "飞机无法恢复已持久化映射",
            "commit_failed": "飞机无法准备映射参数",
            "commit_queue_failed": "后台 Flash 请求排队失败；尚未持久化，可稍后重试",
        }
        if event in failure_events:
            self.validation_authorized_orientation_commands.clear()
            self.validation_orientation_pending = ""
            self._validation_note_orientation_event("bad", f"映射操作未完成：{failure_events[event]}")
            self._validation_set_status(ValidationStatus.FAIL, failure_events[event])
            self._validation_refresh_orientation_controls()
            return
        if self.validation_orientation_pending == "apply" and candidate_matches_active:
            self.validation_authorized_orientation_commands.discard(
                " ".join(
                    f"IMUFRAME APPLY {self.validation_candidate_descriptor}".upper().split()
                )
            )
        elif self.validation_orientation_pending == "revert" and event == "reverted":
            self.validation_authorized_orientation_commands.discard("IMUFRAME REVERT")
        elif self.validation_orientation_pending == "commit" and event in {
            "commit_queued",
            "committed",
            "status",
        }:
            self.validation_authorized_orientation_commands.discard("IMUFRAME COMMIT")
        if event == "reverted" or (
            self.validation_orientation_pending == "revert" and not candidate_matches_active
        ):
            self.validation_candidate_applied = False
            self.validation_candidate_verified = False
            self.validation_candidate_committed = False
            self.validation_verification_mode = False
            self.validation_orientation_pending = ""
            self._validation_write_orientation_audit("reverted", flash_writes=0)
            self._validation_set_status(ValidationStatus.WARN, "飞机已撤销 RAM 候选并恢复持久映射")
            self._validation_note_orientation_event(
                "wait", "飞机已撤销 RAM 候选并恢复持久映射")
            reported = True
        elif candidate_matches_active:
            first_apply = not self.validation_candidate_applied
            self.validation_candidate_applied = True
            if first_apply and apply_was_pending and event == "applied":
                self._validation_write_orientation_audit("ram_applied", flash_writes=0)
            if self.validation_orientation_pending == "apply":
                self.validation_orientation_pending = ""
                self._validation_set_status(
                    ValidationStatus.WARN,
                    "飞机已确认 RAM 映射；现在必须执行6步复验",
                )
                self._validation_note_orientation_event(
                    "ok", f"飞机已确认 RAM 映射 active={active}；接下来点 B 重采 6 步复验")
                reported = True
            if persisted == self.validation_candidate_descriptor and dirty == "0":
                first_commit = not self.validation_candidate_committed
                self.validation_candidate_committed = True
                self.validation_orientation_pending = ""
                if first_commit and commit_was_pending:
                    audit_path = self._validation_write_orientation_audit(
                        "flash_committed", flash_writes=1
                    )
                    self.validation_report_var.set(f"映射写入审计：{audit_path}")
                    self._validation_note_orientation_event(
                        "ok",
                        f"写入参数 Flash 成功并已读回确认：persisted={persisted}，dirty=0。"
                        f"审计已保存：{audit_path}",
                    )
                    # 写 Flash 是一次性里程碑，页头状态条在滚动后会移出视野，
                    # 因此这里额外弹一次确认，确保不会"写完了却不知道写没写"。
                    messagebox.showinfo(
                        "映射已写入参数 Flash",
                        f"飞机已读回确认：persisted={persisted}，dirty=0。\n\n"
                        f"审计文件：{audit_path}\n\n"
                        "注意：这只持久化了 IMU 坐标映射，flight_release 仍为 false，"
                        "不代表允许自由飞行。",
                    )
                else:
                    self._validation_note_orientation_event(
                        "ok", f"飞机映射已持久化：persisted={persisted}，dirty=0")
                self._validation_set_status(
                    ValidationStatus.PASS,
                    "目标确认映射已持久化；flight_release仍为false",
                )
                reported = True
            elif self.validation_orientation_pending == "commit" and dirty == "1":
                self._validation_note_orientation_event(
                    "wait", "后台正在写参数 Flash（dirty=1），请勿断电；正在轮询确认…")
                reported = True
                if self.validation_orientation_poll_count == 0:
                    self.after(500, self._validation_poll_orientation_commit)
        if not reported:
            # 例如手动点“读取飞机映射状态”：也必须看得见回包到达。
            self._validation_note_orientation_event(
                "wait" if dirty == "1" else "ok",
                f"已读取飞机映射状态：active={active or '-'}，"
                f"persisted={persisted or '-'}，dirty={dirty}",
            )
        self._validation_refresh_orientation_controls()

    def _validation_stage_inputs(self) -> dict[ValidationStage, tuple[ImuSample, ...] | None]:
        return {
            stage: None if stage in self.validation_unsupported_stages else tuple(samples)
            for stage, samples in self.validation_samples.items()
            if (
                (samples and stage not in self.validation_skipped_stages)
                or stage in self.validation_unsupported_stages
            )
        }

    def _validation_rebuild_results(self) -> None:
        self.validation_stage_results.clear()
        for stage in VALIDATION_UI_STAGES:
            definition = STAGE_DEFINITIONS[stage]
            samples = self.validation_samples.get(stage, [])
            label = definition.label_zh if definition.required else f"{definition.label_zh}（可选）"
            if stage in self.validation_skipped_stages:
                status_text = ValidationStatus.SKIPPED.value
            elif stage in self.validation_unsupported_stages:
                status_text = ValidationStatus.UNSUPPORTED.value
            elif stage in self.validation_finished_stages and samples:
                result = (
                    analyze_static_stage(stage, tuple(samples))
                    if definition.kind == "static"
                    else analyze_rotation_stage(stage, tuple(samples))
                )
                self.validation_stage_results[stage] = result
                status_text = result.status.value
            elif samples:
                status_text = "PARTIAL"
            else:
                status_text = ValidationStatus.NOT_RUN.value
            self.validation_stage_tree.item(
                stage.value,
                text=label,
                values=(status_text, len(samples)),
            )

    def _validation_skip_stage(self) -> None:
        if not self.validation_session_active or self.validation_active_stage is not None:
            return
        stage = self._validation_selected_stage()
        definition = STAGE_DEFINITIONS[stage]
        if definition.required:
            messagebox.showwarning("必需步骤", f"“{definition.label_zh}”不能跳过。")
            return
        self.validation_samples[stage] = []
        self.validation_stage_results.pop(stage, None)
        self.validation_unsupported_stages.discard(stage)
        self.validation_skipped_stages.add(stage)
        self.validation_finished_stages.add(stage)
        self.validation_stage_tree.item(
            stage.value,
            values=(ValidationStatus.SKIPPED.value, 0),
        )
        self._validation_set_status(
            ValidationStatus.SKIPPED,
            f"已跳过旧版可选步骤：{definition.label_zh}",
        )
        self._validation_refresh_stage_view(stage)
        self._validation_refresh_candidate_summary()
        self._validation_autosave_session(force=True)
        if self.validation_candidate_descriptor and self.validation_candidate_source_path is None:
            self.validation_candidate_source_path = self.validation_session_path
        self._validation_refresh_readiness()

    def _validation_start_session(self) -> None:
        if self.validation_loaded_history and any(self.validation_samples.values()):
            if not messagebox.askyesno(
                "新建独立验收",
                "当前已加载一份历史验收。新建会清空当前页面工作区，但不会覆盖或删除历史报告；"
                "之后仍可从“历史验收”下拉框恢复。确定新建？",
            ):
                return
        if not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("安全门未通过", "请先确认已拆桨，并断开电调动力或可靠固定机体。")
            return
        if not self._transport_connected():
            messagebox.showwarning("尚未连接", "请先建立到飞控的连接。")
            return
        for transport in (self.tcp_transport, self.udp_transport, self.serial_transport):
            transport.cancel_pending_sends()
        self.validation_session_active = True
        self.validation_poll_requested = True
        self.validation_active_stage = None
        self.validation_samples = {stage: [] for stage in STAGE_DEFINITIONS}
        self.validation_stage_results.clear()
        self.validation_unsupported_stages.clear()
        self.validation_skipped_stages.clear()
        self.validation_finished_stages.clear()
        self.validation_raw_rows.clear()
        self.validation_report_paths = None
        self.validation_loaded_report_path = None
        self.validation_loaded_history = False
        self.validation_candidate_matrix = None
        self.validation_candidate_descriptor = ""
        self.validation_candidate_confidence = 0.0
        self.validation_candidate_source_path = None
        self.validation_candidate_applied = False
        self.validation_candidate_verified = False
        self.validation_candidate_committed = False
        self.validation_verification_mode = False
        self.validation_orientation_pending = ""
        self.validation_session_created_at = datetime.now().astimezone().isoformat()
        self.validation_session_path = None
        self.validation_last_autosave_monotonic = 0.0
        self.validation_latest_sequence = None
        self.validation_latest_values.clear()
        self.validation_latest_host_time = 0.0
        self.validation_stage_last_sequence = None
        first_stage = VALIDATION_UI_STAGES[0]
        self.validation_stage_tree.selection_set(first_stage.value)
        self.validation_stage_tree.focus(first_stage.value)
        self.validation_stage_var.set(first_stage.value)
        for stage in VALIDATION_UI_STAGES:
            definition = STAGE_DEFINITIONS[stage]
            self.validation_stage_tree.item(
                stage.value,
                text=definition.label_zh,
                values=(ValidationStatus.NOT_RUN.value, 0),
            )
        self._validation_set_status(ValidationStatus.NOT_RUN, "只读会话已开始，等待有效目标快照")
        self.validation_report_var.set("报告：尚未生成")
        self._validation_refresh_candidate_summary()
        self._send_proto(PROTO_REQ_IMU, "IMU?")
        self._send_proto(PROTO_REQ_IMU_FRAME, "IMUFRAME?")
        self._validation_refresh_stage_view()
        self._validation_refresh_readiness()

    def _validation_stop_session(self) -> None:
        if self.validation_active_stage is not None:
            self._validation_finish_stage()
        self.validation_session_active = False
        self.validation_poll_requested = False
        self.validation_active_stage = None
        self._validation_autosave_session(force=True)
        self._validation_set_status(ValidationStatus.WARN, "只读会话已结束；结果仍不构成飞行放行")
        self._validation_refresh_readiness()

    def _validation_latest_is_safe(self) -> tuple[bool, str]:
        values = self.validation_latest_values
        if not values or not snapshot_is_current(self) or (time.monotonic() - self.validation_latest_host_time) > VALIDATION_SAMPLE_FRESH_S:
            return False, "没有新鲜的完整目标快照"
        if values.get("source") != "stabilizer_snapshot" or values.get("valid") != "1":
            return False, "固件不支持坐标系校准快照来源标识；请重新编译并烧写本次固件"
        if safe_int(values.get("contract"), -1) != 1:
            return False, f"FLU contract version 不支持：{values.get('contract', '-')}"
        frame = values.get("frame", "")
        expected_frame = (
            frame.startswith("canonical_flu")
            if self.validation_candidate_applied
            else frame == "legacy_intermediate"
        )
        if not expected_frame:
            return False, f"frame provenance 不支持：{frame or '-'}"
        if self.validation_candidate_applied:
            expected_code = self.validation_orientation_values.get("active_code")
            if expected_code is None or values.get("orientation") != expected_code:
                return False, (
                    "orientation provenance 不匹配："
                    f"sample={values.get('orientation', '-')} target={expected_code or '-'}"
                )
        if safe_int(values.get("migration"), -1) != 0:
            return False, f"migration provenance 不支持：{values.get('migration', '-')}"
        if values.get("bias") != "1":
            return False, "陀螺零偏尚未完成"
        if safe_int(values.get("armed"), 1) != 0:
            return False, "目标报告 armed=1，禁止采集"
        motor_1 = safe_int(values.get("m1"), 65535)
        motor_2 = safe_int(values.get("m2"), 65535)
        if motor_1 > VALIDATION_MOTOR_SAFE_MAX_US or motor_2 > VALIDATION_MOTOR_SAFE_MAX_US:
            return False, f"电机输出不安全：m1={motor_1} m2={motor_2}"
        return True, ""

    def _validation_begin_stage(self) -> None:
        if not self.validation_session_active:
            messagebox.showwarning("尚未开始", "请先恢复历史验收，或点击“新建独立验收”。")
            return
        safe, reason = self._validation_latest_is_safe()
        if not safe:
            self._validation_set_status(ValidationStatus.UNSUPPORTED, reason)
            messagebox.showwarning("数据或安全门未通过", reason)
            return
        stage = self._validation_selected_stage()
        self.validation_samples[stage] = []
        self.validation_stage_results.pop(stage, None)
        self.validation_unsupported_stages.discard(stage)
        self.validation_skipped_stages.discard(stage)
        self.validation_finished_stages.discard(stage)
        self.validation_active_stage = stage
        self.validation_stage_last_sequence = self.validation_latest_sequence
        self.validation_stage_tree.item(stage.value, values=("COLLECTING", 0))
        self._validation_set_status(ValidationStatus.WARN, f"正在采集：{STAGE_DEFINITIONS[stage].label_zh}")
        self._validation_refresh_stage_view(stage)
        self._validation_refresh_readiness()

    def _validation_finish_stage(self) -> None:
        stage = self.validation_active_stage
        if stage is None:
            return
        samples = tuple(self.validation_samples[stage])
        if STAGE_DEFINITIONS[stage].kind == "static":
            result = analyze_static_stage(stage, samples)
        else:
            result = analyze_rotation_stage(stage, samples)
        self.validation_stage_results[stage] = result
        self.validation_finished_stages.add(stage)
        self.validation_active_stage = None
        self.validation_stage_last_sequence = None
        self.validation_stage_tree.item(
            stage.value,
            values=(result.status.value, result.sample_count),
        )
        self._validation_set_status(result.status, f"{STAGE_DEFINITIONS[stage].label_zh} 分析完成")
        self._validation_refresh_stage_view(stage)
        self._validation_refresh_candidate_summary()
        self._validation_autosave_session(force=True)
        if self.validation_candidate_descriptor and self.validation_candidate_source_path is None:
            self.validation_candidate_source_path = self.validation_session_path
        stages = list(VALIDATION_UI_STAGES)
        current_index = stages.index(stage)
        if current_index + 1 < len(stages):
            next_stage = stages[current_index + 1]
            self.validation_stage_tree.selection_set(next_stage.value)
            self.validation_stage_tree.focus(next_stage.value)
            self.validation_stage_tree.see(next_stage.value)
            self.validation_stage_var.set(next_stage.value)
            self.validation_instruction_var.set(STAGE_DEFINITIONS[next_stage].prompt_zh)
        self._validation_refresh_readiness()

    def _validation_abort_active_stage(
        self,
        status: ValidationStatus,
        reason: str,
    ) -> None:
        stage = self.validation_active_stage
        if stage is None:
            return
        self.validation_unsupported_stages.add(stage)
        self.validation_finished_stages.add(stage)
        self.validation_active_stage = None
        self.validation_stage_last_sequence = None
        self.validation_stage_tree.item(
            stage.value,
            values=(status.value, len(self.validation_samples[stage])),
        )
        self.validation_raw_rows.append(
            {
                "stage": stage.value,
                "event": reason,
                "source": self.validation_latest_values.get("source", ""),
                "frame": self.validation_latest_values.get("frame", ""),
                "contract": self.validation_latest_values.get("contract", ""),
                "migration": self.validation_latest_values.get("migration", ""),
                "bias": self.validation_latest_values.get("bias", ""),
                "armed": self.validation_latest_values.get("armed", ""),
                "m1": self.validation_latest_values.get("m1", ""),
                "m2": self.validation_latest_values.get("m2", ""),
            }
        )
        self._validation_set_status(status, reason)
        self.validation_result_text.configure(state=tk.NORMAL)
        self.validation_result_text.delete("1.0", tk.END)
        self.validation_result_text.insert("1.0", reason)
        self.validation_result_text.configure(state=tk.DISABLED)
        self._validation_refresh_candidate_summary()
        self._validation_autosave_session(force=True)
        self._validation_refresh_readiness()

    def _validation_accept_imu_health(self, values: dict[str, str]) -> None:
        self.validation_imu_health = dict(values)
        self.validation_imu_health_time = time.monotonic()
        self.last_board_rx = time.monotonic()
        self._validation_refresh_readiness()

    def _validation_health_state(self) -> tuple[str, str, bool]:
        """返回 (指示灯状态, 显示文本, 是否允许采集)。"""
        health = self.validation_imu_health
        if not health or self.validation_imu_health_time == 0.0:
            return "wait", "等待固件上报（旧固件不带此项）", True
        if (time.monotonic() - self.validation_imu_health_time) > VALIDATION_HEALTH_FRESH_S:
            return "wait", "上报已过期", True
        level = safe_int(health.get("level"), -1)
        rate = safe_int(health.get("rate_hz"), 0)
        fault = safe_int(health.get("fault"), 0)
        fault_ever = safe_int(health.get("fault_ever"), 0)
        label = self.IMU_HEALTH_LABELS.get(level, f"未知({level})")
        text = f"{label}  {rate} Hz"
        if fault != 0:
            return "bad", f"{text} · DRDY 失效，已禁止解锁", False
        if level >= 1:
            return "bad", f"{text} · 采样链降级，标定数据不可信", False
        if fault_ever != 0:
            return "wait", f"{text} · 本次上电曾降级，建议重新采集", True
        return "ok", text, True

    def _validation_target_restarted(self, timestamp_ms: int) -> bool:
        """ts_ms 是飞控上电以来的毫秒数，一次上电内严格单调；倒退即重启。"""
        previous = self.validation_latest_timestamp_ms
        return previous is not None and timestamp_ms < previous

    def _validation_accept_imu_values(self, values: dict[str, str]) -> None:
        sequence = safe_int(values.get("seq"), -1)
        if sequence < 0:
            if values.get("valid") == "0" and values.get("source") == "stabilizer_snapshot":
                self.validation_latest_values.clear()
                self.validation_latest_host_time = 0.0
                self.validation_latest_transport_generation = None
                self.validation_source_var.set("数据源：UNSUPPORTED · 目标尚无有效 stabilizer snapshot")
                self._validation_refresh_readiness()
                self._firmware_refresh_safety()
            return
        receipt = snapshot_receipt(self, sequence)
        cached_sequence = safe_int(self.validation_latest_values.get("seq"), -1)
        if cached_sequence != sequence:
            self.validation_latest_values = {"seq": str(sequence)}
            self.validation_latest_transport_generation = None
        self.validation_latest_values.update({key: value for key, value in values.items() if value != "-"})
        merged = self.validation_latest_values
        for short_name, explicit_name in (
            ("ax", "ax_mg"), ("ay", "ay_mg"), ("az", "az_mg"),
            ("gx", "gx_mdps"), ("gy", "gy_mdps"), ("gz", "gz_mdps"),
            ("roll", "roll_cdeg"), ("pitch", "pitch_cdeg"), ("yaw", "yaw_cdeg"),
        ):
            if short_name in merged and explicit_name not in merged:
                merged[explicit_name] = merged[short_name]
        required = (
            "valid", "source", "frame", "units", "contract", "migration", "ts_ms", "seq",
            "bias", "armed", "m1", "m2", "ax_mg", "ay_mg", "az_mg",
            "gx_mdps", "gy_mdps", "gz_mdps",
        )
        if any(key not in merged for key in required):
            return
        if merged.get("valid") != "1" or merged.get("source") != "stabilizer_snapshot":
            return
        if merged.get("units") != "mg_mdps_cdeg":
            self.validation_source_var.set(f"数据源：UNSUPPORTED units={merged.get('units', '-')}")
            return
        sample, parse_error = validation_sample_from_snapshot(merged)
        if sample is None:
            if parse_error is not None and not parse_error.startswith("incomplete:"):
                self.validation_source_var.set(f"数据源：UNSUPPORTED · {parse_error}")
            return
        timestamp_ms = int(merged["ts_ms"], 0)
        if self._validation_target_restarted(timestamp_ms):
            # 目标重启后 seqlock 序号从头开始。不重置基线的话，每一帧都会被下面的
            # 单调守卫当成重放丢掉，validation_latest_host_time 永远停在 0，界面显示
            # "快照已过期（inf）"且断开重连都救不回来——只能重启上位机。
            # 判据用板子自己的 ts_ms 倒退，一次上电内它严格单调，比猜序号阈值可靠。
            self.validation_latest_sequence = None
        self.validation_latest_timestamp_ms = timestamp_ms
        if self.validation_latest_sequence is not None and sequence <= self.validation_latest_sequence:
            return
        self.validation_latest_sequence = sequence
        self.validation_latest_host_time = receipt.received_at
        self.validation_latest_transport_generation = receipt.generation
        self.validation_source_var.set(
            f"数据源：snapshot · frame={merged['frame']} contract={merged['contract']} "
            f"migration={merged['migration']} seq={sequence}"
        )
        accel_norm = math.sqrt(sum(value * value for value in sample.accel_g))
        self.validation_live_var.set(
            f"a=({sample.accel_x_g:+.3f}, {sample.accel_y_g:+.3f}, {sample.accel_z_g:+.3f}) g  "
            f"|a|={accel_norm:.3f} g\n"
            f"gyro=({sample.gyro_x_dps:+.2f}, {sample.gyro_y_dps:+.2f}, {sample.gyro_z_dps:+.2f}) dps  "
            f"armed={merged['armed']} m1={merged['m1']} m2={merged['m2']} bias={merged['bias']}"
        )
        self._validation_refresh_readiness()
        self._firmware_refresh_safety()
        if self.validation_active_stage is None:
            return
        sample_safe, safety_reason = self._validation_latest_is_safe()
        if (
            not sample_safe
            or not self.validation_props_removed_var.get()
            or not self.validation_power_safe_var.get()
        ):
            if sample_safe:
                safety_reason = "人工安全确认在采集中被撤销"
            self._validation_abort_active_stage(
                ValidationStatus.FAIL,
                f"逐样本安全门触发：{safety_reason}",
            )
            return
        if self.validation_stage_last_sequence is not None and sequence <= self.validation_stage_last_sequence:
            return
        self.validation_stage_last_sequence = sequence
        stage = self.validation_active_stage
        self.validation_samples[stage].append(sample)
        self.validation_raw_rows.append(
            {
                "stage": stage.value,
                "target_timestamp_ms": timestamp_ms,
                "sequence": sequence,
                "accel_x_g": sample.accel_x_g,
                "accel_y_g": sample.accel_y_g,
                "accel_z_g": sample.accel_z_g,
                "gyro_x_dps": sample.gyro_x_dps,
                "gyro_y_dps": sample.gyro_y_dps,
                "gyro_z_dps": sample.gyro_z_dps,
                "roll_deg": sample.roll_deg,
                "pitch_deg": sample.pitch_deg,
                "yaw_deg": sample.yaw_deg,
                **{key: merged.get(key, "") for key in ("source", "frame", "units", "contract", "migration", "orientation", "bias", "armed", "m1", "m2")},
            }
        )
        self.validation_stage_tree.item(stage.value, values=("COLLECTING", len(self.validation_samples[stage])))
        target = (
            VALIDATION_STATIC_TARGET_SAMPLES
            if STAGE_DEFINITIONS[stage].kind == "static"
            else VALIDATION_ROTATION_TARGET_SAMPLES
        )
        sample_count = len(self.validation_samples[stage])
        self.validation_sample_var.set(f"当前步骤样本：{sample_count} / 建议 {target}")
        self.validation_progress.configure(value=min(100.0, sample_count * 100.0 / target))
        self._validation_autosave_session()


__all__ = ["ValidationV0PageMixin", "v0_workflow_guidance"]
