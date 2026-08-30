"""Vibration/filter placeholder page."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk


class VibrationPageMixin:
    def _build_vibration_filter_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="VIBRATION / FILTER  /  预留页面", style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text="振动检测与 IMU 滤波", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=(
                "按当前阶段要求，本页只建立入口，不启动带桨叶旋转、不发电机命令、也不重新拟合滤波器。"
                "以后接入时仍复用已有全速 IMU 采集和频谱报告链路。"
            ),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))
        current = ttk.LabelFrame(parent, text="当前固件滤波基线（保持不变）", padding=10)
        current.pack(fill=tk.X)
        for label, value in (
            ("采样率", "1000 Hz"),
            ("陀螺仪", "二阶 Butterworth 低通 · 80 Hz"),
            ("加速度计", "二阶 Butterworth 低通 · 40 Hz"),
            ("设计依据", "沿用此前实机采集；已测桨频约 56–180 Hz"),
        ):
            row = ttk.Frame(current)
            row.pack(fill=tk.X, pady=3)
            ttk.Label(row, text=label, width=14, style="Muted.TLabel").pack(side=tk.LEFT)
            ttk.Label(row, text=value).pack(side=tk.LEFT)
        pending = ttk.LabelFrame(parent, text="页面状态", padding=10)
        pending.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(
            pending,
            text="未实现 · 不提供开始采集按钮 · 不执行带桨动力测试 · 不改当前滤波参数",
            style="Warn.TLabel",
        ).pack(anchor=tk.W)


__all__ = ["VibrationPageMixin"]
