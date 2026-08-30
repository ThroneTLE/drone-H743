"""Optical-flow / combined-ranging ground calibration page and its handlers."""

from __future__ import annotations

import json
import statistics
import time
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, ttk

from ..proto import PROTO_REQ_IMU, parse_kv, safe_float, safe_int

try:
    from ...ground_calibration import (
        FlowRangeSample,
        GroundCalibrationError,
        analyze_flow_axis,
        analyze_flow_zero,
        analyze_rotation_compensation,
        fit_range_two_point,
    )
    from ...project_paths import (
        FLOW_RANGE_CALIBRATION_DIR,
        dated_directory,
        ensure_directory,
    )
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools.ground_calibration import (
            FlowRangeSample,
            GroundCalibrationError,
            analyze_flow_axis,
            analyze_flow_zero,
            analyze_rotation_compensation,
            fit_range_two_point,
        )
        from tools.project_paths import (
            FLOW_RANGE_CALIBRATION_DIR,
            dated_directory,
            ensure_directory,
        )
    except ImportError:
        from ground_calibration import (
            FlowRangeSample,
            GroundCalibrationError,
            analyze_flow_axis,
            analyze_flow_zero,
            analyze_rotation_compensation,
            fit_range_two_point,
        )
        from project_paths import (
            FLOW_RANGE_CALIBRATION_DIR,
            dated_directory,
            ensure_directory,
        )


FLOW_CALIBRATION_STAGES = {
    "static_zero": "静止零偏与噪声",
    "forward_x": "沿机头方向移动（+X）",
    "left_y": "向机体左侧移动（+Y）",
    "range_near": "组合测距近距离",
    "range_far": "组合测距远距离",
    "yaw_rotation": "原地偏航旋转补偿",
}


class FlowRangingPageMixin:
    def _build_flow_range_calibration_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="FLOW / RANGE  /  只读地面采样", style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text="光流与组合测距坐标、比例和零偏", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=(
                "依次做静止、机头方向 +X、机体左侧 +Y、近/远两点测距和原地偏航。"
                "飞行高度当前来自光流模块内的组合测距，独立 RANGE? 只作旁路诊断。"
                "分析结果先保存为证据，不会直接改飞控参数。"
            ),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))

        live = ttk.LabelFrame(parent, text="实时诊断", padding=8)
        live.pack(fill=tk.X)
        actions = ttk.Frame(live)
        actions.pack(fill=tk.X)
        ttk.Button(
            actions, text="读取 FLOW / RANGE", command=self._flow_range_request_once,
            style="Secondary.TButton",
        ).pack(side=tk.LEFT)
        ttk.Label(
            actions,
            text="要求纹理清晰、光照稳定；组合测距质量不足时不要拟合比例。",
            style="Muted.TLabel",
        ).pack(side=tk.LEFT, padx=(14, 0))
        ttk.Label(
            live, textvariable=self.flow_cal_live_var, font=("Consolas", 9),
            wraplength=1080, justify=tk.LEFT,
        ).pack(fill=tk.X, pady=(7, 0))

        capture = ttk.LabelFrame(parent, text="分步骤采样", padding=8)
        capture.pack(fill=tk.X, pady=(8, 0))
        row = ttk.Frame(capture)
        row.pack(fill=tk.X)
        ttk.Label(row, text="当前步骤").pack(side=tk.LEFT)
        self.flow_cal_stage_combo = ttk.Combobox(
            row, textvariable=self.flow_cal_stage_var,
            values=tuple(self.flow_cal_stage_by_label), state="readonly", width=30,
        )
        self.flow_cal_stage_combo.pack(side=tk.LEFT, padx=(5, 12))
        ttk.Label(row, text="水平参考位移 m").pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=self.flow_cal_reference_distance_var, width=7).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(row, text="近点 m").pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=self.flow_cal_near_height_var, width=7).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(row, text="远点 m").pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=self.flow_cal_far_height_var, width=7).pack(side=tk.LEFT, padx=(4, 12))
        self.flow_cal_start_button = ttk.Button(
            row, text="开始本步采样", command=self._flow_cal_start,
            style="Primary.TButton",
        )
        self.flow_cal_start_button.pack(side=tk.LEFT)
        self.flow_cal_stop_button = ttk.Button(
            row, text="停止并分析", command=self._flow_cal_stop,
            state=tk.DISABLED, style="Warning.TButton",
        )
        self.flow_cal_stop_button.pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(
            capture, textvariable=self.flow_cal_status_var,
            style="Guide.TLabel", wraplength=1080,
        ).pack(fill=tk.X, pady=(7, 0))

        evidence = ttk.LabelFrame(parent, text="证据与结论", padding=8)
        evidence.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.flow_cal_tree = ttk.Treeview(
            evidence, columns=("samples", "result"), show="tree headings", height=7,
        )
        self.flow_cal_tree.heading("#0", text="检查项目")
        self.flow_cal_tree.column("#0", width=260, anchor=tk.W)
        self.flow_cal_tree.heading("samples", text="样本")
        self.flow_cal_tree.column("samples", width=90, anchor=tk.CENTER)
        self.flow_cal_tree.heading("result", text="结论")
        self.flow_cal_tree.column("result", width=700, anchor=tk.W)
        for stage, label in FLOW_CALIBRATION_STAGES.items():
            self.flow_cal_tree.insert("", tk.END, iid=stage, text=label, values=(0, "未采样"))
        self.flow_cal_tree.pack(fill=tk.BOTH, expand=True)
        ttk.Label(
            evidence, textvariable=self.flow_cal_result_var,
            font=("Consolas", 9), wraplength=1080, justify=tk.LEFT,
        ).pack(fill=tk.X, pady=(7, 0))
        save_row = ttk.Frame(evidence)
        save_row.pack(fill=tk.X, pady=(7, 0))
        ttk.Button(
            save_row, text="保存光流 / 测距证据", command=self._flow_cal_save_report,
            style="Primary.TButton",
        ).pack(side=tk.LEFT)
        ttk.Label(
            save_row,
            text="旋转补偿验收使用目标端同一样本的补偿前/后 FLU 速度：偏航均值 ≥15 dps 且残余 ≤0.08 m/s 才 PASS。",
            style="Muted.TLabel",
        ).pack(side=tk.LEFT, padx=(16, 0))

    def _flow_range_request_once(self) -> None:
        self._send("FLOW?")
        self._send("RANGE?")
        self._send_proto(PROTO_REQ_IMU, "IMU?")

    def _flow_cal_start(self) -> None:
        if not self._transport_connected():
            messagebox.showwarning("光流 / 测距校准", "请先连接飞控。")
            return
        stage = self.flow_cal_stage_by_label.get(self.flow_cal_stage_var.get())
        if stage is None:
            return
        self.flow_cal_samples[stage] = []
        self.flow_cal_results.pop(stage, None)
        self.flow_cal_active_stage = stage
        self.flow_cal_last_target_sample_ms = -1
        self.flow_cal_collecting = True
        self.flow_cal_start_button.configure(state=tk.DISABLED)
        self.flow_cal_stop_button.configure(state=tk.NORMAL)
        self.flow_cal_status_var.set(
            f"正在采集“{FLOW_CALIBRATION_STAGES[stage]}”：按页面提示完成动作后点击停止并分析"
        )

    def _flow_cal_stop(self) -> None:
        if not self.flow_cal_collecting or self.flow_cal_active_stage is None:
            return
        stage = self.flow_cal_active_stage
        self.flow_cal_collecting = False
        self.flow_cal_active_stage = None
        self.flow_cal_start_button.configure(state=tk.NORMAL)
        self.flow_cal_stop_button.configure(state=tk.DISABLED)
        try:
            result = self._flow_cal_analyze_stage(stage)
        except (GroundCalibrationError, ValueError) as exc:
            self.flow_cal_status_var.set(f"本步证据不足：{exc}")
            self._flow_cal_refresh_tree(stage, f"证据不足：{exc}")
            return
        self.flow_cal_results[stage] = result
        rendered = json.dumps(result, ensure_ascii=False, sort_keys=True)
        self.flow_cal_result_var.set(rendered)
        self.flow_cal_status_var.set(f"“{FLOW_CALIBRATION_STAGES[stage]}”分析完成；结果尚未写入飞控")
        self._flow_cal_refresh_tree(stage, self._flow_cal_result_summary(stage, result))

    def _flow_cal_analyze_stage(self, stage: str) -> dict[str, object]:
        samples = self.flow_cal_samples[stage]
        if stage == "static_zero":
            return dict(analyze_flow_zero(samples))
        if stage in {"forward_x", "left_y"}:
            return dict(analyze_flow_axis(
                samples,
                expected_axis="x" if stage == "forward_x" else "y",
                reference_distance_m=safe_float(self.flow_cal_reference_distance_var.get(), -1.0),
            ))
        if stage == "yaw_rotation":
            return dict(analyze_rotation_compensation(samples))
        if stage in {"range_near", "range_far"}:
            heights = [sample.height_raw_m for sample in samples if sample.height_raw_m is not None]
            if len(heights) < 5:
                raise GroundCalibrationError("至少需要 5 个有效组合测距样本")
            result: dict[str, object] = {
                "sample_count": len(heights),
                "measured_mean_m": statistics.fmean(heights),
                "measured_std_m": statistics.pstdev(heights),
            }
            near = self.flow_cal_samples["range_near"]
            far = self.flow_cal_samples["range_far"]
            if len(near) >= 5 and len(far) >= 5:
                result["two_point_fit"] = fit_range_two_point(
                    near, far,
                    near_reference_m=safe_float(self.flow_cal_near_height_var.get(), -1.0),
                    far_reference_m=safe_float(self.flow_cal_far_height_var.get(), -1.0),
                )
            else:
                result["two_point_fit"] = "等待近、远两组都采集完成"
            return result
        raise GroundCalibrationError(f"未知步骤 {stage}")

    def _flow_cal_result_summary(self, stage: str, result: dict[str, object]) -> str:
        if stage == "static_zero":
            return f"零偏 vx={float(result['vx_mean_m_s']):+.3f} vy={float(result['vy_mean_m_s']):+.3f} m/s"
        if stage in {"forward_x", "left_y"}:
            sign = "正确" if result.get("positive_sign_ok") else "错误"
            scale = result.get("diagnostic_scale")
            scale_text = "-" if scale is None else f"{float(scale):.4f}"
            return f"正向符号={sign} 主轴/串轴={float(result['axis_dominance_ratio']):.2f} 诊断比例={scale_text}"
        if stage == "yaw_rotation":
            return str(result.get("reason", "未形成结论"))
        fit = result.get("two_point_fit")
        if isinstance(fit, dict):
            return f"两点拟合 scale={float(fit['range_scale']):.5f} offset={float(fit['range_offset_m']):+.4f} m"
        return f"均值={float(result['measured_mean_m']):.4f} m；{fit}"

    def _flow_cal_refresh_tree(self, stage: str, result: str) -> None:
        if hasattr(self, "flow_cal_tree"):
            self.flow_cal_tree.item(
                stage,
                values=(len(self.flow_cal_samples[stage]), result),
            )

    def _flow_cal_save_report(self) -> None:
        if not self.flow_cal_results:
            messagebox.showwarning("光流 / 测距校准", "至少先完成一个采样步骤。")
            return
        stamp = datetime.now().astimezone()
        path = ensure_directory(dated_directory(FLOW_RANGE_CALIBRATION_DIR, stamp)) / (
            "flow_range_" + stamp.strftime("%Y%m%d_%H%M%S") + ".json"
        )
        sample_payload = {
            stage: [
                {
                    "host_time_s": sample.host_time_s,
                    "vx_m_s": sample.vx_m_s,
                    "vy_m_s": sample.vy_m_s,
                    "height_raw_m": sample.height_raw_m,
                    "height_m": sample.height_m,
                    "gyro_z_dps": sample.gyro_z_dps,
                    "vx_compensated_m_s": sample.vx_compensated_m_s,
                    "vy_compensated_m_s": sample.vy_compensated_m_s,
                    "quality": sample.quality,
                    "frame_contract": sample.frame_contract,
                    "orientation_code": sample.orientation_code,
                }
                for sample in samples
            ]
            for stage, samples in self.flow_cal_samples.items() if samples
        }
        report = {
            "format": "drone-h743-flow-range-ground-evidence",
            "schema": 1,
            "created_at": stamp.isoformat(),
            "body_frame": "FLU",
            "stages": self.flow_cal_results,
            "samples": sample_payload,
            "target_parameters_written": False,
            "rotation_compensation_accepted": bool(
                self.flow_cal_results.get("yaw_rotation", {}).get("passed", False)
            ),
            "flight_release": False,
        }
        try:
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("光流 / 测距证据保存失败", str(exc))
            return
        self.flow_cal_status_var.set(f"地面采样证据已保存：{path}")

    def _update_flow_line(self, line: str) -> None:
        values = parse_kv(line)
        self.flow_diag_values.update(values)
        ok = self.flow_diag_values.get("ok") == "1"
        valid = self.flow_diag_values.get("valid", "0") not in {"0", "-"}
        quality = safe_int(self.flow_diag_values.get("quality"), 0)
        min_quality = safe_int(self.flow_diag_values.get("min_q"), 80)
        velocity_valid = self.flow_diag_values.get("vel_valid") == "1"
        height_valid = self.flow_diag_values.get("height_valid") == "1"
        self._update_module(
            "FLOW",
            state="正常" if ok and valid else ("等待有效数据" if ok else "异常"),
            stage="ready" if ok and valid else "data",
            value=(
                f"q={quality}/{min_quality} vel={int(velocity_valid)} height={int(height_valid)} "
                f"vx={self.flow_diag_values.get('vx_mm_s', '-')} vy={self.flow_diag_values.get('vy_mm_s', '-')} mm/s"
            ),
            code=(
                f"frames={self.flow_diag_values.get('frames', '-')} "
                f"cksum={self.flow_diag_values.get('cksum', '-')} err={self.flow_diag_values.get('frame_err', '-')}"
            ),
            hint="质量不足时先改善纹理/光照；校准页可做坐标和比例采样",
            line=line,
        )
        self.flow_cal_live_var.set(
            "FLOW "
            f"valid={self.flow_diag_values.get('valid', '-')} "
            f"q={quality}/{min_quality} "
            f"raw_v=({self.flow_diag_values.get('flow_vx', '-')},{self.flow_diag_values.get('flow_vy', '-')}) "
            f"v=({self.flow_diag_values.get('vx_mm_s', '-')},{self.flow_diag_values.get('vy_mm_s', '-')})mm/s "
            f"comp_flu=({self.flow_diag_values.get('corr_vx_mm_s', '-')},{self.flow_diag_values.get('corr_vy_mm_s', '-')})mm/s "
            f"orientation={self.flow_diag_values.get('orientation', '-')} "
            f"height_raw={self.flow_diag_values.get('height_raw_mm', '-')}mm "
            f"height={self.flow_diag_values.get('height_mm', '-')}mm "
            f"RANGE={getattr(self, 'range_diag_summary', '尚无回包')}"
        )
        self.last_reply_rx = time.monotonic()

        if not self.flow_cal_collecting or self.flow_cal_active_stage is None:
            return
        if "corr_vx_mm_s" not in values or "corr_vy_mm_s" not in values:
            return
        target_sample_ms = safe_int(values.get("sample_ms"), -1)
        orientation_code = safe_int(self.flow_diag_values.get("orientation"), 255)
        frame_contract = safe_int(self.flow_diag_values.get("contract"), 0)
        if (
            self.flow_diag_values.get("export") != "canonical_flu"
            or frame_contract != 1
            or not (0 <= orientation_code < 24)
        ):
            self.flow_cal_status_var.set(
                "目标尚未提供带有效 FLU 方向来源的补偿后速度；请先完成坐标系校准并更新固件"
            )
            return
        if target_sample_ms < 0 or target_sample_ms == self.flow_cal_last_target_sample_ms:
            return
        self.flow_cal_last_target_sample_ms = target_sample_ms
        now = time.monotonic()
        samples = self.flow_cal_samples[self.flow_cal_active_stage]
        if samples and now <= samples[-1].host_time_s:
            now = samples[-1].host_time_s + 1.0e-6
        raw_height = (
            safe_int(self.flow_diag_values.get("height_raw_mm"), 0) * 0.001
            if "height_raw_mm" in self.flow_diag_values else None
        )
        filtered_height = (
            safe_int(self.flow_diag_values.get("height_mm"), 0) * 0.001
            if "height_mm" in self.flow_diag_values else None
        )
        samples.append(FlowRangeSample(
            host_time_s=now,
            vx_m_s=safe_int(values.get("sensor_vx_mm_s"), 0) * 0.001,
            vy_m_s=safe_int(values.get("sensor_vy_mm_s"), 0) * 0.001,
            height_raw_m=raw_height,
            height_m=filtered_height,
            gyro_z_dps=self.flow_latest_gyro_z_dps,
            vx_compensated_m_s=safe_int(values.get("corr_vx_mm_s"), 0) * 0.001,
            vy_compensated_m_s=safe_int(values.get("corr_vy_mm_s"), 0) * 0.001,
            quality=quality,
            frame_contract=frame_contract,
            orientation_code=orientation_code,
        ))
        self.flow_cal_status_var.set(
            f"正在采集“{FLOW_CALIBRATION_STAGES[self.flow_cal_active_stage]}”：{len(samples)} 个样本"
        )
        self._flow_cal_refresh_tree(self.flow_cal_active_stage, "采集中")

    def _update_range_line(self, line: str) -> None:
        values = parse_kv(line)
        ok = values.get("ok") == "1"
        strength = safe_int(values.get("strength"), 0)
        minimum = safe_int(values.get("min_strength"), 80)
        self.range_diag_summary = (
            f"ok={int(ok)} raw={values.get('raw_mm', '-')}mm "
            f"strength={strength}/{minimum} age={values.get('age_ms', '-')}ms"
        )
        self._update_module(
            "RANGE",
            state="正常" if ok else "等待有效数据",
            stage="ready" if ok else "data",
            value=f"raw={values.get('raw_mm', '-')} mm strength={strength}/{minimum}",
            code=f"frames={values.get('frames', '-')} cksum={values.get('cksum', '-')} err={values.get('frame_err', '-')}",
            hint="当前飞行高度不用此独立测距；这里只做旁路健康诊断",
            line=line,
        )
        self.last_reply_rx = time.monotonic()


__all__ = ["FLOW_CALIBRATION_STAGES", "FlowRangingPageMixin"]
