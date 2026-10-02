"""「偏航（吊绳）」页自己的图：四层叠放，共用时间轴。

* 一：偏航角 ψ（陀螺 z 对时间积分，开跑清零；不用会饱和的 height）[°]，点线是绞绳上限 ±twist_deg；
* 二：偏航角速度 ω（原始陀螺 z，实线）与参考 r（虚线，diff 注入为 0 不画）[rad/s]；
* 三：差速推力指令 ΔT（实线，左轴 [N]）与偏航力矩指令 M（右轴 [mN·m]，饱和前）；
* 四：上桨 / 下桨 eRPM（实际差速）。

记录里的尾字段在 `tools/sysid/yaw_analysis.rename_samples` 里改成 YAW 含义的名字后才画，
所以这张图和它的图例里不出现 "height"。预览时画激励剖面（单位随注入类型）。
"""
from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

from ._core import yaw_rename_samples
from .waveform import _HAVE_PLOT
from .xy_plot import series

if _HAVE_PLOT:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    from ...theme import UI_PALETTE, apply_matplotlib_theme


class YawPlot:
    def __init__(self, panel, parent: ttk.Frame, erpm_var: tk.Variable) -> None:
        box = ttk.LabelFrame(parent, text="波形（实线=实测，虚线=参考）", padding=6)
        box.pack(fill=tk.BOTH, expand=True, pady=(0, 8))
        self.figure = self.canvas = None
        self.psi_axis = self.omega_axis = self.delta_axis = self.erpm_axis = self.moment_axis = None
        if not _HAVE_PLOT:
            ttk.Label(box, text="matplotlib 不可用，本次不画波形。").pack(anchor=tk.W)
            return
        palette = getattr(panel, "ui_palette", None) or UI_PALETTE
        self.figure = Figure(figsize=(6, 6.4), dpi=100)
        self.psi_axis = self.figure.add_subplot(411)
        self.omega_axis = self.figure.add_subplot(412, sharex=self.psi_axis)
        self.delta_axis = self.figure.add_subplot(413, sharex=self.psi_axis)
        self.erpm_axis = self.figure.add_subplot(414, sharex=self.psi_axis)
        self.moment_axis = self.delta_axis.twinx()
        apply_matplotlib_theme(self.figure, palette)
        self.moment_axis.grid(False)
        self.canvas = FigureCanvasTkAgg(self.figure, master=box)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        ttk.Label(box, textvariable=erpm_var, style="Muted.TLabel", wraplength=660,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))

    def _reset(self) -> None:
        for axis in (self.psi_axis, self.omega_axis, self.delta_axis, self.erpm_axis):
            axis.clear()
        for line in list(self.moment_axis.lines):
            line.remove()
        legend = self.moment_axis.get_legend()
        if legend is not None:
            legend.remove()

    def preview(self, spec, unit: str, inject: str = "diff") -> None:
        """激励预览：只在对应的一层画剖面形状（diff 画 ΔT 层，rate 画角速度层），其余层清空。"""
        if self.figure is None:
            return
        t_s, shape, _alpha = spec.preview(step_ms=4)
        self._reset()
        axis = self.delta_axis if inject == "diff" else self.omega_axis
        axis.plot(t_s, shape, linestyle="--", label=f"偏航激励（{unit}）")
        axis.set_ylabel(f"激励 [{unit}]")
        axis.legend(loc="upper right", fontsize=8)
        self.erpm_axis.set_xlabel("时间 [s]")
        self.canvas.draw_idle()

    def measured(self, times, samples, snapshot) -> None:
        """实测四层；rate 才画参考 r（diff 的参考字段为 0，表示不适用）。"""
        if self.figure is None:
            return
        rows = yaw_rename_samples(samples, times)       # ψ 由陀螺 z 积分（height 会饱和）
        request = (snapshot or {}).get("yaw_request") or {}
        closed = request.get("control") == "closed_loop"
        self._reset()
        degrees = 180.0 / math.pi
        self.psi_axis.plot(times, series(rows, "psi", degrees), linewidth=1.2, label="偏航角 ψ")
        twist = request.get("twist_deg")
        if twist:
            for sign in (1.0, -1.0):
                self.psi_axis.axhline(sign * float(twist), linestyle=":", linewidth=0.8, color="C3")
        self.psi_axis.set_ylabel("偏航角 ψ [°]")
        self.omega_axis.plot(times, series(rows, "omega"), linewidth=1.0, label="偏航角速度 ω")
        if closed:
            self.omega_axis.plot(times, series(rows, "r_ref"), linestyle="--", label="参考 r")
        self.omega_axis.set_ylabel("角速度 [rad/s]")
        self.delta_axis.plot(times, series(rows, "delta_t"), linewidth=1.0, label="差速 ΔT")
        self.delta_axis.set_ylabel("差速推力 ΔT [N]")
        self.moment_axis.plot(times, series(rows, "torque", 1000.0), linewidth=0.8, alpha=0.7,
                              color="C2", label="力矩指令 M")
        self.moment_axis.relim()
        self.moment_axis.autoscale_view()
        self.moment_axis.set_ylabel("M [mN·m]")
        self.erpm_axis.plot(times, series(rows, "erpm"), linewidth=0.9, label="上桨 eRPM")
        self.erpm_axis.plot(times, series(rows, "erpm_lower"), linewidth=0.9, label="下桨 eRPM")
        self.erpm_axis.set_ylabel("eRPM")
        self.erpm_axis.set_xlabel("时间 [s]")
        for axis in (self.psi_axis, self.omega_axis, self.erpm_axis):
            axis.legend(loc="upper right", fontsize=8)
        handles = self.delta_axis.get_lines() + self.moment_axis.get_lines()
        self.moment_axis.legend(handles, [line.get_label() for line in handles], loc="upper right",
                                fontsize=8)
        self.canvas.draw_idle()


__all__ = ["YawPlot"]
