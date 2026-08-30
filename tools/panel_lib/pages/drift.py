"""Stationary-drift page builder and handlers."""

from __future__ import annotations

import time
import tkinter as tk
from tkinter import messagebox, ttk

from ..proto import safe_int

try:
    from ... import stationary_drift as drift
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools import stationary_drift as drift
    except ImportError:
        import stationary_drift as drift


class DriftPageMixin:
    def _build_drift_page(self, parent: ttk.Frame) -> None:
        drift_box = ttk.LabelFrame(parent, text="静止漂移自检（应用前后各做一次，对比收益）", padding=8)
        drift_box.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(
            drift_box,
            text=("飞机放在不会晃的桌面上别碰，录 30~60 秒。不动 = 真实角速度为 0、真实比力为 1g，"
                  "所以不需要转台，读数偏多少就是误差多少。"),
            style="Muted.TLabel", wraplength=1100,
        ).pack(fill=tk.X, pady=(0, 6))
        drift_actions = ttk.Frame(drift_box); drift_actions.pack(fill=tk.X)
        self.drift_start_button = ttk.Button(
            drift_actions, text="开始静止录制", command=self._drift_start, style="Primary.TButton")
        self.drift_start_button.pack(side=tk.LEFT)
        self.drift_stop_button = ttk.Button(
            drift_actions, text="提前结束", command=self._drift_stop,
            state=tk.DISABLED, style="Warning.TButton")
        self.drift_stop_button.pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(drift_actions, text="时长(秒)").pack(side=tk.LEFT, padx=(14, 4))
        ttk.Spinbox(drift_actions, from_=30, to=300, increment=10, width=6,
                    textvariable=self.drift_duration_var).pack(side=tk.LEFT)
        self.drift_baseline_button = ttk.Button(
            drift_actions, text="设为对照组", command=self._drift_set_baseline,
            state=tk.DISABLED, style="Secondary.TButton")
        self.drift_baseline_button.pack(side=tk.LEFT, padx=(14, 0))
        ttk.Button(drift_actions, text="载入上一次录制", command=self._drift_load_previous,
                   style="Secondary.TButton").pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(drift_box, textvariable=self.drift_status_var,
                  style="Guide.TLabel", wraplength=1100).pack(fill=tk.X, pady=(7, 0))
        ttk.Label(drift_box, textvariable=self.drift_result_var,
                  font=("Consolas", 9), wraplength=1100, justify=tk.LEFT).pack(fill=tk.X, pady=(4, 0))
        ttk.Label(drift_box, textvariable=self.drift_compare_var,
                  wraplength=1100, justify=tk.LEFT).pack(fill=tk.X, pady=(4, 0))

    def _drift_start(self) -> None:
        if self.drift_recording:
            return
        if not self._transport_connected():
            messagebox.showwarning("静止漂移自检", "先连接飞控")
            return
        try:
            seconds = max(float(self.drift_duration_var.get()), drift.MIN_DURATION_S)
        except ValueError:
            seconds = drift.RECOMMENDED_DURATION_S
        self.drift_samples = []
        self.drift_last_sequence = -1
        self.drift_recording = True
        self.drift_deadline = time.monotonic() + seconds
        self.drift_report = None
        self.drift_compare_var.set("")
        self.drift_result_var.set("")
        self.drift_start_button.configure(state=tk.DISABLED)
        self.drift_stop_button.configure(state=tk.NORMAL)
        self.drift_baseline_button.configure(state=tk.DISABLED)
        self.drift_status_var.set(f"录制中：剩余 {seconds:.0f} 秒，别碰飞机也别碰桌子")

    def _drift_stop(self) -> None:
        if not self.drift_recording:
            return
        self.drift_recording = False
        self.drift_start_button.configure(state=tk.NORMAL)
        self.drift_stop_button.configure(state=tk.DISABLED)
        context = {
            "cal_generation": self.imu_calibration_generation,
            "firmware_crc32": self.imu_calibration_firmware_crc32,
            "sample_source": "IMU telemetry stream",
        }
        report = drift.analyze_drift(self.drift_samples, context=context)
        self.drift_report = report
        self.drift_result_var.set(
            drift.summarise(report) + "\n" + "\n".join("· " + item for item in report.findings))
        try:
            path = drift.write_report(report)
            self.drift_status_var.set(f"录制结束，已存档：{path}")
        except OSError as exc:
            self.drift_status_var.set(f"录制结束，但存档失败：{exc}")
        self.drift_baseline_button.configure(state=tk.NORMAL)
        self._drift_render_comparison()

    def _drift_set_baseline(self) -> None:
        if self.drift_report is None:
            return
        self.drift_baseline = self.drift_report
        self.drift_compare_var.set(
            "已把这次结果设为对照组。现在去应用/写入标定，再录一次就能看到差值。")

    def _drift_load_previous(self) -> None:
        recent = drift.recent_reports(limit=2)
        if not recent:
            self.drift_compare_var.set("还没有历史录制可以对比。")
            return
        # 最近一条通常就是刚存的这次，所以对照组取次新的那条。
        index = 1 if (len(recent) > 1 and self.drift_report is not None) else 0
        self.drift_baseline_path, self.drift_baseline = recent[index]
        self._drift_render_comparison()

    def _drift_render_comparison(self) -> None:
        if self.drift_baseline is None or self.drift_report is None:
            return
        lines = drift.compare(self.drift_baseline, self.drift_report)
        header = "A/B 对比（对照组 → 本次）"
        if self.drift_baseline_path is not None:
            header += f"，对照组来自 {self.drift_baseline_path.name}"
        self.drift_compare_var.set(header + "\n" + "\n".join("  " + item for item in lines))

    def _drift_accept_sample(self, values: dict[str, str]) -> None:
        """从 IMU 遥测流里取一帧。

        飞控一次回两行、同一个 seq：先是带 ts_ms/temp_cdeg 的那行，随后才是带
        ax/gx/roll 的读数行。所以先缓存表头行，等读数行到了再合成一帧；seq 对不上
        就丢弃，宁可少一帧也不要把两次快照拼在一起。
        """
        if not self.drift_recording:
            return
        sequence = safe_int(values.get("seq"), -1)
        if sequence < 0:
            return
        if "ts_ms" in values:
            self._drift_pending_motion = {
                "seq": sequence,
                "timestamp_s": safe_int(values.get("ts_ms"), 0) / 1000.0,
                "temperature_c": safe_int(values.get("temp_cdeg"), 2500) / 100.0,
            }
            return
        if "gx" not in values:
            return
        header = self._drift_pending_motion
        if header is None or header["seq"] != sequence or sequence == self.drift_last_sequence:
            return
        self._drift_pending_motion = None
        self.drift_last_sequence = sequence
        self.drift_samples.append(drift.DriftSample(
            timestamp_s=header["timestamp_s"],
            gyro_dps=tuple(safe_int(values.get(k), 0) / 1000.0 for k in ("gx", "gy", "gz")),
            accel_g=tuple(safe_int(values.get(k), 0) / 1000.0 for k in ("ax", "ay", "az")),
            attitude_deg=tuple(safe_int(values.get(k), 0) / 100.0 for k in ("roll", "pitch", "yaw")),
            temperature_c=header["temperature_c"],
            sequence=sequence,
        ))

    def _drift_tick(self) -> None:
        if self.drift_recording:
            remaining = self.drift_deadline - time.monotonic()
            if remaining <= 0.0:
                self._drift_stop()
            else:
                self.drift_status_var.set(
                    f"录制中：剩余 {remaining:.0f} 秒，已采 {len(self.drift_samples)} 帧"
                    "，别碰飞机也别碰桌子")
        self.after(500, self._drift_tick)


__all__ = ["DriftPageMixin", "drift"]
