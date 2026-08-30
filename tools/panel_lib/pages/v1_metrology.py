"""V1 IMU metrology page builder and handlers."""

from __future__ import annotations

import json
import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

from ..proto import PROTO_REQ_IMU_CAL

try:
    from ...imu_metrology import DEFAULT_THRESHOLDS, MetrologyStage, MetrologyStatus
    from ...imucal_protocol import (
        EncodedV1Candidate,
        commit_candidate as imucal_commit_candidate,
        load_and_encode_v1_candidate,
        parse_imucal_line,
        revert_candidate as imucal_revert_candidate,
        upload_and_apply as imucal_upload_and_apply,
        write_transaction_record as write_imucal_transaction_record,
    )
    from ...imu_vibration_capture import CaptureLink
    from ...v1_metrology_session import (
        CAPTURE_PLANS,
        DISCARDED_DIRNAME,
        RETIRED_STAGE_LABELS,
        V1Session,
        analyze_session,
        context_conflicts as v1_context_conflicts,
        discard_capture as discard_v1_capture,
        latest_session_manifest,
        load_session as load_v1_session,
        load_session_samples as load_v1_session_samples,
        minimum_samples as v1_minimum_samples,
        new_session as new_v1_session,
        persist_capture as persist_v1_capture,
        probe_capture as probe_v1_capture,
    )
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools.imu_metrology import DEFAULT_THRESHOLDS, MetrologyStage, MetrologyStatus
        from tools.imucal_protocol import (
            EncodedV1Candidate,
            commit_candidate as imucal_commit_candidate,
            load_and_encode_v1_candidate,
            parse_imucal_line,
            revert_candidate as imucal_revert_candidate,
            upload_and_apply as imucal_upload_and_apply,
            write_transaction_record as write_imucal_transaction_record,
        )
        from tools.imu_vibration_capture import CaptureLink
        from tools.v1_metrology_session import (
            CAPTURE_PLANS, DISCARDED_DIRNAME, RETIRED_STAGE_LABELS, V1Session, analyze_session,
            context_conflicts as v1_context_conflicts,
            discard_capture as discard_v1_capture,
            latest_session_manifest,
            load_session as load_v1_session,
            load_session_samples as load_v1_session_samples,
            minimum_samples as v1_minimum_samples,
            new_session as new_v1_session,
            persist_capture as persist_v1_capture,
            probe_capture as probe_v1_capture,
        )
    except ImportError:
        from imu_metrology import DEFAULT_THRESHOLDS, MetrologyStage, MetrologyStatus
        from imucal_protocol import (
            EncodedV1Candidate,
            commit_candidate as imucal_commit_candidate,
            load_and_encode_v1_candidate,
            parse_imucal_line,
            revert_candidate as imucal_revert_candidate,
            upload_and_apply as imucal_upload_and_apply,
            write_transaction_record as write_imucal_transaction_record,
        )
        from imu_vibration_capture import CaptureLink
        from v1_metrology_session import (
            CAPTURE_PLANS, DISCARDED_DIRNAME, RETIRED_STAGE_LABELS, V1Session, analyze_session,
            context_conflicts as v1_context_conflicts,
            discard_capture as discard_v1_capture,
            latest_session_manifest,
            load_session as load_v1_session,
            load_session_samples as load_v1_session_samples,
            minimum_samples as v1_minimum_samples,
            new_session as new_v1_session,
            persist_capture as persist_v1_capture,
            probe_capture as probe_v1_capture,
        )


V1_CAPTURE_PREP_SECONDS = 5
V1_FACE_RESIDUAL_WARN_G = DEFAULT_THRESHOLDS.accel_corrected_rms_max_g
V1_FACE_RESIDUAL_FAIL_G = DEFAULT_THRESHOLDS.accel_corrected_rms_fail_g
V1_STAGE_LABELS = {
    "accel_pos_x": "+X 机头朝上", "accel_neg_x": "-X 机头朝下",
    "accel_pos_y": "+Y 左侧朝上", "accel_neg_y": "-Y 右侧朝上",
    "accel_pos_z": "+Z 水平", "accel_neg_z": "-Z 倒置",
}
UI_PALETTE = {
    "green": "#4ADE97",
    "red": "#FF8B82",
    "amber": "#F2B441",
    "muted": "#8D97A6",
}


class V1PageMixin:
    def _build_v1_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="IMU CALIBRATION  /  修正传感器连续误差", style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text="IMU 零偏、比例与正交性校准", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=("IMUCAP v4 采集：六面各 2000 样本、陀螺静止 4500 样本，共 7 步。"
                  "六面解出 3x3 校正矩阵与零偏，静止段解出陀螺零偏；两组都通过即可应用室温基础参数。"
                  "手转 +360° 与温度平台已从流程移除：前者 6.1s 窗口内手转精度反而不如不标，"
                  "后者需要温箱凑齐 3 个温点。"),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))
        safety = ttk.LabelFrame(parent, text="复用坐标系校准 / USB 安全门", padding=8)
        safety.pack(fill=tk.X)
        ttk.Checkbutton(safety, text="已拆除全部桨叶", variable=self.validation_props_removed_var, command=self._v1_refresh_controls).pack(side=tk.LEFT)
        ttk.Checkbutton(safety, text="电调动力已断开或机体已可靠固定", variable=self.validation_power_safe_var, command=self._v1_refresh_controls).pack(side=tk.LEFT, padx=(14, 0))
        ttk.Checkbutton(safety, text="未知 USB 身份人工确认", variable=self.firmware_unknown_usb_override_var, command=self._v1_refresh_controls).pack(side=tk.LEFT, padx=(14, 0))

        controls = ttk.Frame(parent); controls.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(controls, text="新建 IMU 校准会话", command=self._v1_new_session,
                   style="Primary.TButton").pack(side=tk.LEFT)
        ttk.Label(controls, text="步骤").pack(side=tk.LEFT, padx=(14, 4))
        self.v1_stage_combo = ttk.Combobox(controls, textvariable=self.v1_stage_var, values=tuple(self.v1_stage_by_label), state="readonly", width=27)
        self.v1_stage_combo.pack(side=tk.LEFT)
        capture_actions = ttk.Frame(parent); capture_actions.pack(fill=tk.X, pady=(7, 0))
        self.v1_start_button = ttk.Button(capture_actions, text="开始采集", command=self._v1_start_capture,
                                          state=tk.DISABLED, style="Primary.TButton")
        self.v1_start_button.pack(side=tk.LEFT)
        self.v1_finish_button = ttk.Button(capture_actions, text="完成本步并导出", command=self._v1_finish_capture,
                                           state=tk.DISABLED, style="Warning.TButton")
        self.v1_finish_button.pack(side=tk.LEFT, padx=(6, 0))
        self.v1_analyze_button = ttk.Button(capture_actions, text="分析并保存候选", command=self._v1_start_analysis,
                                            state=tk.DISABLED, style="Secondary.TButton")
        self.v1_analyze_button.pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(
            capture_actions,
            text=(f"点开始后有 {V1_CAPTURE_PREP_SECONDS} 秒准备时间（按“完成本步”可立即开录），"
                  "录制期间机体必须绝对静止；分析不会直接改飞机参数。"),
            style="Muted.TLabel", wraplength=760,
        ).pack(side=tk.LEFT, padx=(16, 0))

        ttk.Label(parent, textvariable=self.v1_status_var, style="Guide.TLabel", wraplength=1120).pack(fill=tk.X, pady=(8, 6))
        ttk.Label(parent, textvariable=self.v1_counts_var, style="Muted.TLabel").pack(fill=tk.X)
        ttk.Label(parent, textvariable=self.v1_analysis_var, wraplength=1120).pack(fill=tk.X, pady=(4, 8))
        self.v1_progress = ttk.Progressbar(parent, mode="determinate", maximum=100.0)
        self.v1_progress.pack(fill=tk.X)
        table = ttk.LabelFrame(parent, text="本会话步骤与证据", padding=8); table.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.v1_capture_tree = ttk.Treeview(
            table, columns=("platform", "samples", "rate", "quality", "verdict", "file"),
            show="tree headings", height=12, selectmode="browse")
        self.v1_capture_tree.heading("#0", text="步骤")
        self.v1_capture_tree.column("#0", width=200, anchor=tk.W)
        for name, label, width, anchor in (
            ("platform", "温度平台", 80, tk.CENTER),
            ("samples", "样本", 110, tk.CENTER),
            ("rate", "采样率", 80, tk.CENTER),
            ("quality", "采集质量", 260, tk.W),
            ("verdict", "分析结论", 210, tk.W),
            ("file", "CSV", 230, tk.W),
        ):
            self.v1_capture_tree.heading(name, text=label)
            self.v1_capture_tree.column(name, width=width, anchor=anchor)
        for tag, color in (
            ("pass", UI_PALETTE["green"]), ("fail", UI_PALETTE["red"]),
            ("warn", UI_PALETTE["amber"]), ("muted", UI_PALETTE["muted"]),
        ):
            self.v1_capture_tree.tag_configure(tag, foreground=color)
        self.v1_capture_tree.bind("<<TreeviewSelect>>", self._v1_on_row_selected)
        self.v1_capture_tree.pack(fill=tk.BOTH, expand=True)
        row_actions = ttk.Frame(table); row_actions.pack(fill=tk.X, pady=(7, 0))
        self.v1_discard_button = ttk.Button(
            row_actions, text="删除选中采集", command=self._v1_discard_selected,
            state=tk.DISABLED, style="Danger.TButton")
        self.v1_discard_button.pack(side=tk.LEFT)
        ttk.Label(
            row_actions,
            text=("选中一行即切到该步骤，点“开始采集”会自动替换旧数据；"
                  "删除只是把 CSV 移到 discarded/ 子目录，不会真的销毁。"),
            style="Muted.TLabel", wraplength=880,
        ).pack(side=tk.LEFT, padx=(12, 0))
        self._build_drift_page(parent)

        pending = ttk.Frame(parent); pending.pack(fill=tk.X, pady=(8, 0))
        self.v1_apply_button = ttk.Button(
            pending, text="应用基础/完整候选到 RAM",
            command=self._v1_apply_candidate, state=tk.DISABLED, style="Warning.TButton")
        self.v1_apply_button.pack(side=tk.LEFT)
        self.v1_revert_button = ttk.Button(
            pending, text="撤销 RAM 候选",
            command=self._v1_revert_candidate, state=tk.DISABLED, style="Danger.TButton")
        self.v1_revert_button.pack(side=tk.LEFT, padx=(8, 0))
        self.v1_commit_button = ttk.Button(
            pending, text="确认后写入参数 Flash",
            command=self._v1_commit_candidate, state=tk.DISABLED, style="Warning.TButton")
        self.v1_commit_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(
            pending, text="读取飞机 IMU 校准状态",
            command=lambda: self._send_proto(PROTO_REQ_IMU_CAL, "IMUCAL?"),
            style="Secondary.TButton",
        ).pack(side=tk.LEFT, padx=(16, 0))
        self._v1_refresh_controls()

    def _v1_selected_plan(self):
        return self.v1_stage_by_label[self.v1_stage_var.get()]

    def _v1_refresh_controls(self) -> None:
        # 用整页最后建出来的控件做守卫，页面半成品时不会被定时器提前调进来。
        if not hasattr(self, "v1_discard_button"):
            return
        gate_ok, _reason = self._validation_live_safety_gate()
        safe = gate_ok and self.validation_props_removed_var.get() and self.validation_power_safe_var.get()
        busy = self.v1_worker is not None and self.v1_worker.is_alive()
        self.v1_start_button.configure(state=tk.NORMAL if safe and self.v1_session is not None and not busy else tk.DISABLED)
        self.v1_finish_button.configure(state=tk.NORMAL if busy else tk.DISABLED)
        self.v1_analyze_button.configure(state=tk.NORMAL if self.v1_session is not None and bool(self.v1_session.captures) and not busy else tk.DISABLED)
        selected = self.v1_row_record.get(self.v1_capture_tree.focus())
        self.v1_discard_button.configure(state=tk.NORMAL if selected is not None and not busy else tk.DISABLED)
        # WARN = 六面摆得不够一致（折合几度），数据本身没问题。拿它去标仍然远好于
        # 不标，代价已经写在结论里，收不收由人决定 —— PX4 连这道检查都没有。
        usable = {MetrologyStatus.PASS, MetrologyStatus.WARN}
        candidate_pass = bool(
            self.v1_analysis_summary is not None
            and self.v1_analysis_summary.accelerometer_status in usable
            and self.v1_analysis_summary.gyro_static_status in usable
        )
        self.v1_apply_button.configure(
            state=tk.NORMAL if safe and candidate_pass and not busy and not self.v1_candidate_applied else tk.DISABLED)
        self.v1_revert_button.configure(
            state=tk.NORMAL if safe and self.v1_candidate_applied and not busy else tk.DISABLED)
        self.v1_commit_button.configure(
            state=tk.NORMAL if safe and self.v1_candidate_applied and not busy else tk.DISABLED)

    def _v1_new_session(self) -> None:
        if self.v1_worker is not None and self.v1_worker.is_alive():
            return
        self.v1_session, self.v1_manifest_path = new_v1_session()
        self.v1_analysis_summary = None
        self.v1_encoded_candidate = None
        self.v1_candidate_applied = False
        self.v1_probe_cache.clear()
        self.v1_status_var.set("IMU 校准会话已建立：选择姿态步骤，然后开始采集")
        self._v1_render_session()

    def _v1_load_latest_session(self) -> None:
        path = latest_session_manifest()
        if path is None:
            return
        try:
            self.v1_session = load_v1_session(path); self.v1_manifest_path = path
            self.v1_probe_cache.clear()
            self.v1_status_var.set("已恢复最近一次 IMU 校准会话：可继续未完成的采集步骤")
            self._v1_render_session()
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            self.v1_status_var.set(f"IMU 校准会话恢复失败：{exc}")

    def _v1_probe(self, record):
        """带 mtime 缓存的单条体检：同一份 CSV 只读一次。"""
        assert self.v1_manifest_path is not None
        path = self.v1_manifest_path.parent / record.csv_path
        try:
            key = (record.csv_path, path.stat().st_mtime_ns)
        except OSError:
            key = (record.csv_path, -1)
        if key not in self.v1_probe_cache:
            self.v1_probe_cache[key] = probe_v1_capture(self.v1_manifest_path, record)
        return self.v1_probe_cache[key]

    @staticmethod
    def _v1_describe_analysis(summary) -> str:
        """只报流程里真有的两组，并把 findings 原样带出来。

        原来只显示 accel=FAIL，用户看不到"为什么 FAIL"，只能六面全部重采。
        """
        parts = [f"六面 accel={summary.accelerometer_status.value}",
                 f"陀螺静止={summary.gyro_static_status.value}"]
        for label, status in (("手动 +360°", summary.gyro_rotation_status),
                              ("温度平台", summary.temperature_status)):
            if status is not MetrologyStatus.NOT_RUN:
                parts.append(f"{label}（已移除，仅存档）={status.value}")
        text = " · ".join(parts)
        if summary.findings:
            text += "\n原因：" + "；".join(summary.findings)
        worst = sorted(summary.face_residuals.items(), key=lambda item: -item[1][0])[:2]
        if worst and worst[0][1][0] > V1_FACE_RESIDUAL_WARN_G:
            names = "、".join(
                f"{V1_STAGE_LABELS.get(stage, stage)} 残差 {rms:.3f}g/倾角 {tilt:.1f}°"
                for stage, (rms, tilt) in worst)
            blocking = worst[0][1][0] > V1_FACE_RESIDUAL_FAIL_G
            lead = "必须重采" if blocking else "想更准就重采"
            text += (f"\n{lead}：{names}。"
                     "3x3 能吸收一对正负面的平均倾角（单面歪多少无所谓），但吸收不了同一对"
                     "内部的差值。让这一对靠同一个基准面（书立/纸盒角）翻转 180° 各采一次，"
                     "差值自然接近 0。")
        text += f"\ncandidate={summary.candidate_path}"
        return text

    def _v1_stage_verdict(self, plan):
        """把整组分析结论落回它所属的每一步 —— 六面是一起判的，不存在单面结论。"""
        summary = self.v1_analysis_summary
        if summary is None or plan is None:
            return None
        if plan.stage is MetrologyStage.GYRO_STATIC:
            return summary.gyro_static_status
        if plan.stage is MetrologyStage.TEMPERATURE_STATIC:
            return summary.temperature_status
        if plan.stage.value.startswith("gyro_pos_360"):
            return summary.gyro_rotation_status
        return summary.accelerometer_status

    def _v1_session_rows(self):
        """固定按 CAPTURE_PLANS 顺序展开；未采集的步骤也占一行，进度一眼可见。"""
        records = list(self.v1_session.captures) if self.v1_session is not None else []
        rows = []
        for plan in CAPTURE_PLANS:
            matching = [record for record in records if record.stage == plan.stage.value]
            if matching:
                rows.extend((plan, record) for record in matching)
            else:
                rows.append((plan, None))
        known = {plan.stage.value for plan in CAPTURE_PLANS}
        rows.extend((None, record) for record in records if record.stage not in known)
        return rows

    def _v1_render_session(self) -> None:
        if self.v1_session is None:
            return
        for item in self.v1_capture_tree.get_children():
            self.v1_capture_tree.delete(item)
        self.v1_row_plan.clear(); self.v1_row_record.clear()
        conflicts = v1_context_conflicts(self.v1_session, self.v1_manifest_path)
        seen_stage: set[tuple[str, str | None]] = set()
        captured = blocking = 0
        for index, (plan, record) in enumerate(self._v1_session_rows()):
            iid = f"v1-{index}"
            if plan is not None:
                label = plan.label
            else:
                stage = MetrologyStage(record.stage)
                label = "已移除：" + RETIRED_STAGE_LABELS.get(stage, record.stage)
            if record is None:
                self.v1_capture_tree.insert(
                    "", tk.END, iid=iid, text=label,
                    values=("-", f"0 / {v1_minimum_samples(plan.stage)}", "-", "未采集", "-", "-"),
                    tags=("muted",))
                self.v1_row_plan[iid] = plan
                continue
            health = self._v1_probe(record)
            key = (record.stage, record.temperature_platform)
            duplicate = key in seen_stage
            seen_stage.add(key)
            verdict = self._v1_stage_verdict(plan)
            verdict_text = verdict.value if verdict is not None else "-"
            measured = (self.v1_analysis_summary.face_residuals.get(record.stage)
                        if self.v1_analysis_summary is not None else None)
            if measured is not None:
                rms, tilt = measured
                verdict_text = f"{verdict_text} 残差{rms:.3f}g 倾角{tilt:.1f}°"
            detail = health.detail
            tag = {"ok": "pass", "short": "warn", "angle": "warn",
                   "rate": "fail", "unreadable": "fail"}[health.level]
            if duplicate:
                # 老会话里可能有同一步骤的两份数据；重复的必然进不了分析，直接标红提示删除。
                tag, detail = "fail", "同一步骤重复采集，请删除其中一条"
            elif record.csv_path in conflicts:
                # 换过固件或标定代次的那几条会让整份候选被拒，必须在点分析之前看见。
                tag, detail = "fail", conflicts[record.csv_path]
            if tag == "fail":
                blocking += 1
            elif health.level == "ok":
                captured += 1
            # 采集本身没毛病、是这一面摆歪了：采集质量列保持"可用"，由分析结论列点名。
            # 只有超过 FAIL 档才标红，超过 PASS 档标黄 —— 徒手摆六面本来就到不了 PASS 档。
            if tag != "fail" and measured is not None:
                if measured[0] > V1_FACE_RESIDUAL_FAIL_G:
                    tag = "fail"
                elif measured[0] > V1_FACE_RESIDUAL_WARN_G and tag == "pass":
                    tag = "warn"
            self.v1_capture_tree.insert(
                "", tk.END, iid=iid, text=label,
                values=(
                    record.temperature_platform or "-",
                    f"{health.sample_count} / {health.required_samples}",
                    f"{health.rate_hz:.0f} Hz" if health.rate_hz > 0.0 else "-",
                    detail,
                    verdict_text,
                    record.csv_path,
                ),
                tags=(tag,))
            self.v1_row_plan[iid] = plan
            self.v1_row_record[iid] = record
        blocker = f" · {blocking} 条必须删除重采" if blocking else ""
        self.v1_counts_var.set(
            f"session={self.v1_session.session_id} · 可用证据 {captured}/{len(CAPTURE_PLANS)} 步"
            f"{blocker} · manifest={self.v1_manifest_path}")
        self._v1_refresh_controls()

    def _v1_on_row_selected(self, _event=None) -> None:
        iid = self.v1_capture_tree.focus()
        plan = self.v1_row_plan.get(iid)
        if plan is not None:
            self.v1_stage_var.set(plan.label)
            record = self.v1_row_record.get(iid)
        self._v1_refresh_controls()

    def _v1_discard_selected(self) -> None:
        iid = self.v1_capture_tree.focus()
        record = self.v1_row_record.get(iid)
        if record is None or self.v1_session is None or self.v1_manifest_path is None:
            return
        if not messagebox.askyesno(
            "删除采集",
            f"删除 {record.stage}（{record.sample_count} 样本）？\n"
            f"原始 CSV 会移到 {DISCARDED_DIRNAME}/ 子目录，可手工找回。",
        ):
            return
        try:
            self.v1_session, removed = discard_v1_capture(
                self.v1_session, self.v1_manifest_path, csv_path=record.csv_path)
        except (OSError, ValueError) as exc:
            messagebox.showerror("IMU 校准", f"删除失败：{exc}")
            return
        # 证据变了，之前那份分析结论就不再对应当前会话，必须作废。
        self.v1_analysis_summary = None
        self.v1_encoded_candidate = None
        self.v1_analysis_var.set("证据已变更：请重新执行“分析并保存候选”")
        self.v1_status_var.set(f"已删除 {removed.stage}：原始 CSV 移入 {DISCARDED_DIRNAME}/，可重新采集本步")
        self._v1_render_session()

    def _v1_start_capture(self) -> None:
        if self.v1_session is None or self.v1_manifest_path is None:
            self._v1_new_session()
        gate_ok, reason = self._validation_live_safety_gate()
        if not gate_ok or not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("IMU 校准安全门未通过", reason if not gate_ok else "必须拆桨并隔离动力")
            return
        port = self.serial_transport.active_port
        if not port:
            messagebox.showwarning("IMU 校准 USB CDC", "没有当前 active USB CDC")
            return
        plan = self._v1_selected_plan()
        # 温度平台步骤已从 CAPTURE_PLANS 移除，采集流程里不再有需要标签的步骤。
        platform = None
        self.v1_finish_event = threading.Event(); self.v1_cancel_event = threading.Event(); self.v1_progress.configure(value=0)
        baud = int(self.serial_baud_var.get())
        self.serial_transport.stop(); self._refresh_serial_selection_lock()
        self._serial_port_combo.configure(state=tk.DISABLED)
        self._serial_refresh_button.configure(state=tk.DISABLED)
        self.v1_status_var.set(f"IMU 校准采集中：{plan.label}；USB CDC 已由面板安全让渡给 IMUCAP")
        self.v1_worker = threading.Thread(target=self._v1_capture_worker, args=(port, baud, plan, platform), daemon=True)
        self.v1_worker.start(); self._v1_refresh_controls()

    def _v1_finish_capture(self) -> None:
        self.v1_finish_event.set(); self.v1_status_var.set("正在 STOP/DUMP 并保存原始证据…")

    def _v1_capture_worker(self, port: str, baud: int, plan, platform: str | None) -> None:
        link = None
        try:
            time.sleep(0.25); link = CaptureLink(port, baud=baud, timeout=0.25)
            # 采集窗口只有 requested_samples/1kHz 秒，按下按钮就开录的话，手还没扶稳
            # 窗口已经用掉一截。先给一段准备倒计时，"完成本步"可以提前开始。
            window_s = plan.requested_samples / 1000.0
            hint = f"，全程保持静止（{window_s:.1f}s）"
            for remaining in range(V1_CAPTURE_PREP_SECONDS, 0, -1):
                if self.v1_cancel_event.is_set():
                    raise RuntimeError("V1 capture cancelled")
                self.v1_event_queue.put((
                    "status", f"{plan.label}：{remaining} 秒后开始录制{hint}"))
                if self.v1_finish_event.wait(1.0):
                    self.v1_finish_event.clear()
                    break
            link.send(f"IMUCAP START {plan.requested_samples}")
            deadline = time.monotonic() + 2.0
            started = False
            while time.monotonic() < deadline:
                line = link.read_text_line(deadline)
                if line and "IMUCAP START" in line and "ok" in line.lower():
                    started = True
                    break
            if not started:
                raise RuntimeError("IMUCAP START did not return an explicit ok")
            capture_deadline = time.monotonic() + plan.maximum_samples / 1000.0 + 0.75
            while time.monotonic() < capture_deadline and not self.v1_finish_event.is_set() and not self.v1_cancel_event.is_set():
                remaining = capture_deadline - time.monotonic()
                # 录制中必须有秒数在跳，否则 6 秒的窗口在体感上就是"愣一下就结束了"。
                self.v1_event_queue.put((
                    "progress",
                    (f"● 正在录制 {plan.label}：剩余 {max(remaining, 0.0):.1f}s{hint}",
                     1.0 - max(remaining, 0.0) / (plan.maximum_samples / 1000.0 + 0.75))))
                self.v1_cancel_event.wait(0.1)
            if self.v1_cancel_event.is_set(): raise RuntimeError("V1 capture cancelled")
            link.send("IMUCAP STOP")
            ready_deadline = time.monotonic() + 2.0
            capture_ready = False
            while time.monotonic() < ready_deadline:
                link.send("IMUCAP?")
                line = link.read_text_line(min(ready_deadline, time.monotonic() + 0.25))
                if line and "IMUCAP" in line and "state=ready" in line:
                    capture_ready = True
                    break
            if not capture_ready:
                raise RuntimeError("IMUCAP did not finish timestamp-matched annotations")
            link.send("IMUCAP DUMP")
            samples, header = link.read_blocks(timeout_s=45.0, verbose=False, progress=lambda text, fraction: self.v1_event_queue.put(("progress", (text, fraction))))
            assert self.v1_session is not None and self.v1_manifest_path is not None
            replaced = any(
                item.stage == plan.stage.value and item.temperature_platform == platform
                for item in self.v1_session.captures)
            updated, record = persist_v1_capture(self.v1_session, self.v1_manifest_path, stage=plan.stage, samples=samples, header=header, temperature_platform=platform)
            self.v1_event_queue.put(("capture", (updated, record, replaced)))
        except Exception as exc:
            self.v1_event_queue.put(("error", f"IMU 校准采集失败：{exc}"))
        finally:
            if link is not None:
                try: link.close()
                except Exception: pass
            self.v1_event_queue.put(("reconnect", (port, baud)))

    def _v1_start_analysis(self) -> None:
        if self.v1_session is None or self.v1_manifest_path is None: return
        self.v1_status_var.set("正在离线分析 IMU 原始校准证据…")
        self.v1_worker = threading.Thread(target=self._v1_analysis_worker, args=(self.v1_session, self.v1_manifest_path), daemon=True)
        self.v1_worker.start(); self._v1_refresh_controls()

    def _v1_analysis_worker(self, session: V1Session, manifest: Path) -> None:
        try: self.v1_event_queue.put(("analysis", analyze_session(session, manifest)))
        except Exception as exc: self.v1_event_queue.put(("error", f"IMU 校准分析失败：{exc}"))

    def _v1_begin_target_action(self, action: str) -> None:
        if self.v1_session is None or self.v1_manifest_path is None:
            return
        gate_ok, reason = self._validation_live_safety_gate()
        if not gate_ok or not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("IMU 校准安全门未通过", reason if not gate_ok else "必须拆桨并隔离动力")
            return
        port = self.serial_transport.active_port
        if not port:
            messagebox.showwarning("IMU 校准 USB CDC", "没有当前 active USB CDC")
            return
        try:
            if action == "apply":
                if self.v1_analysis_summary is None:
                    raise ValueError("请先完成步骤 4 离线分析")
                samples = load_v1_session_samples(self.v1_session, self.v1_manifest_path)
                encoded = load_and_encode_v1_candidate(
                    self.v1_analysis_summary.candidate_path, samples)
            else:
                if not self.v1_candidate_applied:
                    raise ValueError("飞控 RAM 里没有候选；先点“读取飞机 IMU 校准状态”确认")
                encoded = self.v1_encoded_candidate
                if encoded is None:
                    # 候选是上一次面板会话（或别的进程）应用的：从同一份候选文件重新
                    # 编码，撤销/写盘就不必先拔电重来。编码里带完整重放校验，
                    # 对不上号会在这里报错，不会拿错的东西去写 Flash。
                    if self.v1_analysis_summary is None:
                        raise ValueError(
                            "飞控 RAM 里有候选，但本次会话没有对应的分析结果；"
                            "请先“分析并保存候选”，或直接“撤销 RAM 候选”")
                    samples = load_v1_session_samples(self.v1_session, self.v1_manifest_path)
                    encoded = load_and_encode_v1_candidate(
                        self.v1_analysis_summary.candidate_path, samples)
        except (OSError, ValueError, TypeError) as exc:
            messagebox.showerror("IMU 校准候选不允许应用", str(exc))
            return
        if action == "commit" and not messagebox.askyesno(
            "写入参数 Flash",
            "确认 RAM 候选的姿态、静止输出和方向均已二次验证？\n\n"
            "此操作只写外部参数双槽，不烧录程序固件。",
        ):
            return
        baud = int(self.serial_baud_var.get())
        self.serial_transport.stop(); self._refresh_serial_selection_lock()
        self._serial_port_combo.configure(state=tk.DISABLED)
        self._serial_refresh_button.configure(state=tk.DISABLED)
        self.v1_status_var.set({
            "apply": "正在重放证据并上传基础/完整候选到 RAM…",
            "revert": "正在撤销 RAM 候选…",
            "commit": "正在确认新代次并写入参数 Flash 双槽…",
        }[action])
        self.v1_worker = threading.Thread(
            target=self._v1_target_worker,
            args=(port, baud, action, encoded), daemon=True)
        self.v1_worker.start(); self._v1_refresh_controls()

    def _v1_sync_target_state(self, line: str) -> None:
        """从飞控的 IMUCAL 回包同步"RAM 里有没有候选"。

        原来这个状态只存在面板内存里：面板一重启（或候选是别的进程应用的），
        撤销/写 Flash 两个按钮就永远是灰的，飞控里挂着候选却谁也动不了它，
        只能拔电。飞控自己知道答案，问它就是了。
        """
        values = parse_imucal_line(line)
        if "candidate" not in values or "applied" not in values:
            return
        applied = values.get("applied") == "1" or values.get("candidate") == "1"
        if applied == self.v1_candidate_applied:
            return
        self.v1_candidate_applied = applied
        if applied:
            self.v1_status_var.set(
                "飞控 RAM 里已有候选（电机保持锁定）：可直接“撤销”或“确认后写入参数 Flash”")
        else:
            self.v1_encoded_candidate = None
        self._v1_refresh_controls()

    def _v1_apply_candidate(self) -> None:
        self._v1_begin_target_action("apply")

    def _v1_revert_candidate(self) -> None:
        self._v1_begin_target_action("revert")

    def _v1_commit_candidate(self) -> None:
        self._v1_begin_target_action("commit")

    def _v1_target_worker(self, port: str, baud: int, action: str,
                          encoded: EncodedV1Candidate) -> None:
        link = None
        try:
            time.sleep(0.25); link = CaptureLink(port, baud=baud, timeout=0.25)
            if action == "apply":
                result = imucal_upload_and_apply(link, encoded)
            elif action == "revert":
                result = imucal_revert_candidate(link)
            elif action == "commit":
                result = imucal_commit_candidate(link)
            else:
                raise ValueError(f"unknown V1 target action {action}")
            assert self.v1_manifest_path is not None
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            record_path = write_imucal_transaction_record(
                self.v1_manifest_path.parent / f"imucal_{action}_{stamp}.json",
                encoded=encoded, result=result)
            self.v1_event_queue.put(("target", (action, encoded, record_path)))
        except Exception as exc:
            self.v1_event_queue.put(("error", f"IMU 校准 {action} 失败：{exc}"))
        finally:
            if link is not None:
                try: link.close()
                except Exception: pass
            self.v1_event_queue.put(("reconnect", (port, baud)))

    def _v1_drain_events(self) -> None:
        try:
            while True:
                kind, payload = self.v1_event_queue.get_nowait()
                if kind == "status": self.v1_status_var.set(str(payload))
                elif kind == "progress":
                    text_value, fraction = payload; self.v1_status_var.set(str(text_value)); self.v1_progress.configure(value=float(fraction) * 100.0)
                elif kind == "capture":
                    self.v1_session, record, replaced = payload
                    self.v1_progress.configure(value=100.0)
                    # 新证据进来，旧分析结论立刻作废，避免拿过期结论去 apply。
                    self.v1_analysis_summary = None
                    self.v1_encoded_candidate = None
                    self.v1_analysis_var.set("证据已变更：请重新执行“分析并保存候选”")
                    self.v1_status_var.set(
                        f"{record.stage} 已保存 {record.sample_count} 样本"
                        + ("，并替换了同步骤的旧数据（移入 discarded/）" if replaced else "")
                        + "：可继续下一步或执行离线分析")
                    self._v1_render_session()
                elif kind == "analysis":
                    summary = payload
                    self.v1_analysis_summary = summary
                    self.v1_encoded_candidate = None
                    self.v1_candidate_applied = False
                    self.v1_status_var.set(summary.status_line)
                    self.v1_analysis_var.set(self._v1_describe_analysis(summary))
                    # 结论出来了必须重绘，否则"分析结论"那一列永远停在 "-"。
                    self._v1_render_session()
                elif kind == "target":
                    action, encoded, record_path = payload
                    if action == "apply":
                        self.v1_encoded_candidate = encoded; self.v1_candidate_applied = True
                        self.v1_status_var.set(f"RAM 候选已应用且电机保持锁定；请二次验证。记录：{record_path}")
                    elif action == "revert":
                        self.v1_encoded_candidate = None; self.v1_candidate_applied = False
                        self.v1_status_var.set(f"RAM 候选已撤销，已恢复 Flash 确认参数。记录：{record_path}")
                    else:
                        self.v1_encoded_candidate = None; self.v1_candidate_applied = False
                        self.v1_status_var.set(f"IMU 参数已写入并由 dirty=0 确认。记录：{record_path}")
                elif kind == "error": self.v1_status_var.set(str(payload)); messagebox.showerror("IMU 校准", str(payload))
                elif kind == "reconnect":
                    port, baud = payload
                    if self.transport is self.serial_transport:
                        self.serial_transport.start(port, baud); self._refresh_serial_selection_lock()
                if kind in {"capture", "analysis", "target", "error"}: self.v1_worker = None
        except queue.Empty: pass
        self._v1_refresh_controls(); self.after(100, self._v1_drain_events)


__all__ = ["V1PageMixin"]
