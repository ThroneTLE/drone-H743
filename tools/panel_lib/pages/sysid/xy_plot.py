"""「XY 速度 / 位置环」页自己的图：三层叠放，共用时间轴。

* 上：沿 u 的位置 pos_u（实线）与位置参考 pos_sp_u（虚线，tilt 注入为 0 不画）[mm]；
* 中：沿 u 的速度 vel_u 与速度参考 vel_sp_u（虚线，tilt 注入为 0 不画）[mm/s]；
* 下：绕杆角 angle 与目标倾角 [°]，右轴是推力 thrust [N]。正角把推力偏向 −u（固件口径）。
  记录里的 angle_sp 只含偏置、angle 含开跑姿态，所以目标线画成「第一拍 angle + angle_sp」。

记录里的尾字段在 `tools/sysid/xy_analysis.rename_samples` 里改成 XY 含义的名字后才画，
所以这张图和它的图例里不出现 "height"。预览时画激励剖面（单位随注入类型）。
"""
from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

from ._core import xy_rename_samples
from .waveform import _HAVE_PLOT

if _HAVE_PLOT:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    from ...theme import UI_PALETTE, apply_matplotlib_theme

NAN = float("nan")


def series(rows, name: str, scale: float = 1.0) -> list[float]:
    """一列数据（缺字段画成断开）。"""
    out = []
    for row in rows:
        try:
            value = float(row.get(name, NAN)) * scale
        except (TypeError, ValueError):
            value = NAN
        out.append(value if math.isfinite(value) else NAN)
    return out


class XyPlot:
    def __init__(self, panel, parent: ttk.Frame, erpm_var: tk.Variable) -> None:
        box = ttk.LabelFrame(parent, text="波形（实线=实测，虚线=参考；右下轴=推力）", padding=6)
        box.pack(fill=tk.BOTH, expand=True, pady=(0, 8))
        self.figure = self.canvas = self.pos_axis = self.vel_axis = self.angle_axis = None
        self.thrust_axis = None
        if not _HAVE_PLOT:
            ttk.Label(box, text="matplotlib 不可用，本次不画波形。").pack(anchor=tk.W)
            return
        palette = getattr(panel, "ui_palette", None) or UI_PALETTE
        self.figure = Figure(figsize=(6, 5), dpi=100)
        self.pos_axis = self.figure.add_subplot(311)
        self.vel_axis = self.figure.add_subplot(312, sharex=self.pos_axis)
        self.angle_axis = self.figure.add_subplot(313, sharex=self.pos_axis)
        self.thrust_axis = self.angle_axis.twinx()
        apply_matplotlib_theme(self.figure, palette)
        self.thrust_axis.grid(False)
        self.canvas = FigureCanvasTkAgg(self.figure, master=box)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        ttk.Label(box, textvariable=erpm_var, style="Muted.TLabel", wraplength=660,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))

    def _reset(self) -> None:
        for axis in (self.pos_axis, self.vel_axis, self.angle_axis):
            axis.clear()
        for line in list(self.thrust_axis.lines):
            line.remove()
        legend = self.thrust_axis.get_legend()
        if legend is not None:
            legend.remove()

    def preview(self, spec, unit: str) -> None:
        """激励预览：只在最上面一层画剖面形状，其余层清空。"""
        if self.figure is None:
            return
        t_s, shape, _alpha = spec.preview(step_ms=4)
        self._reset()
        self.pos_axis.plot(t_s, shape, linestyle="--", label=f"水平槽激励（{unit}）")
        self.pos_axis.set_ylabel(f"激励 [{unit}]")
        self.pos_axis.legend(loc="upper right")
        self.angle_axis.set_xlabel("时间 [s]")
        self.thrust_axis.set_ylabel("")
        self.canvas.draw_idle()

    def measured(self, times, samples, snapshot) -> None:
        """实测三层；闭环才画参考（tilt 的参考字段为 0，表示不适用）。"""
        if self.figure is None:
            return
        rows = xy_rename_samples(samples)
        closed = ((snapshot or {}).get("xy_request") or {}).get("control") == "closed_loop"
        self._reset()
        self.pos_axis.plot(times, series(rows, "pos_u", 1000.0), linewidth=1.4, label="位置 pos_u")
        if closed:
            self.pos_axis.plot(times, series(rows, "pos_sp_u", 1000.0), linestyle="--",
                               label="位置参考 pos_sp_u")
        self.pos_axis.set_ylabel("沿 u 位置 [mm]")
        self.vel_axis.plot(times, series(rows, "vel_u", 1000.0), linewidth=1.0, label="速度 vel_u")
        if closed:
            self.vel_axis.plot(times, series(rows, "vel_sp_u", 1000.0), linestyle="--",
                               label="速度参考 vel_sp_u")
        self.vel_axis.set_ylabel("沿 u 速度 [mm/s]")
        degrees = 180.0 / math.pi
        self.angle_axis.plot(times, series(rows, "angle", degrees), linewidth=1.2, label="绕杆角 angle")
        start = next((v for v in series(rows, "angle") if math.isfinite(v)), 0.0)
        self.angle_axis.plot(times, [start * degrees + v for v in series(rows, "angle_sp", degrees)],
                             linestyle="--", label="目标倾角（开跑姿态 + angle_sp）")
        self.angle_axis.set_ylabel("角度 [°]（正 = 推向 −u）")
        self.angle_axis.set_xlabel("时间 [s]")
        self.thrust_axis.plot(times, series(rows, "thrust"), linewidth=0.8, alpha=0.7, color="C2",
                              label="推力 thrust")
        self.thrust_axis.relim()
        self.thrust_axis.autoscale_view()
        self.thrust_axis.set_ylabel("推力 [N]")
        for axis in (self.pos_axis, self.vel_axis):
            axis.legend(loc="upper right", fontsize=8)
        handles = self.angle_axis.get_lines() + self.thrust_axis.get_lines()
        self.thrust_axis.legend(handles, [line.get_label() for line in handles], loc="upper right",
                                fontsize=8)
        self.canvas.draw_idle()


__all__ = ["XyPlot", "series"]
