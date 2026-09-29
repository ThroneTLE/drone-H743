"""「看波形」页：激励预览与实测叠画。matplotlib 缺席时只显示一句说明。

实测角速度主线画 15 Hz 以下（中心滑动平均去掉 ~108 Hz 桨振动，否则整条曲线糊成色块），
原始值用细淡线垫在底下；纵轴按主线与期望定，不被振动尖峰撑开。记录里有真实电调转速
（v3 记录的 erpm / erpm_lower，> 0）时只在图下一行写中位数、电频率与按 7 对极换算的机械转频——
2026-09-27 起不再画转速副轴（量级差太多、阶梯虚线把图糊满，副轴也不跟深色主题）。
舵机单独轮没有期望角速度，虚线画的是舵机指令倾角。
高度轮（ALT）画高度 height 与高度设定 height_sp（主轴 [m]），推力 thrust 画在右侧副轴 [N]
（副轴跟主题、只在高度轮出现；别的模式切回单轴）。预览时纵轴单位随注入类型。
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

try:  # matplotlib 在无显示环境里可能缺席；缺了就只是没有波形图。
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from ...theme import UI_PALETTE, apply_matplotlib_theme
    _HAVE_PLOT = True
except Exception:  # pragma: no cover - 取决于运行环境
    _HAVE_PLOT = False


#: 中心滑动平均的点数：250 Hz 下 9 点 ≈ 36 ms，把 ~108 Hz 桨振动压下去、10 Hz 以下基本不动。
SMOOTH_SAMPLES = 9
#: 电机极对数（2026-09-27 实测：振动频率 = 电转速/60/7，两桨误差 < 1%）。
POLE_PAIRS = 7


def _moving_average(values, width):
    """中心滑动平均（两端按实际可用点数平均），不引 numpy。"""
    half = width // 2
    out = []
    total = 0.0
    prefix = [0.0]
    for v in values:
        total += v
        prefix.append(total)
    n = len(values)
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        out.append((prefix[hi] - prefix[lo]) / (hi - lo))
    return out


class WaveformView:
    def _build_plot(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="波形（虚线=期望，实线=实测）", padding=6)
        box.pack(fill=tk.BOTH, expand=True, pady=(0, 8))
        if not _HAVE_PLOT:
            self.figure = None
            self.canvas = None
            ttk.Label(box, text="matplotlib 不可用，本次不画波形。").pack(anchor=tk.W)
            return
        self.figure = Figure(figsize=(6, 3), dpi=100)
        self.plot_axis = self.figure.add_subplot(111)
        apply_matplotlib_theme(self.figure, getattr(self.panel, "ui_palette", None) or UI_PALETTE)
        self.canvas = FigureCanvasTkAgg(self.figure, master=box)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        ttk.Label(box, textvariable=self.erpm_var, style="Muted.TLabel", wraplength=660,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))

    def _draw(self, spec) -> None:
        if not _HAVE_PLOT or self.figure is None:
            return
        t_s, omega, _alpha = spec.preview(step_ms=4)
        self._alt_thrust_axis(False)
        self.plot_axis.clear()
        unit = self.alt_amp_unit() if getattr(self, "alt_mode_selected", lambda: False)() else None
        self.plot_axis.plot(t_s, omega, linestyle="--",
                            label=f"高度激励（{unit}）" if unit else "期望 ω_sp")
        self.plot_axis.set_xlabel("时间 [s]")
        self.plot_axis.set_ylabel(f"激励 [{unit}]" if unit else "角速度 [rad/s]")
        self.plot_axis.legend(loc="upper right")
        self._draw_erpm([], [])
        self.canvas.draw_idle()

    def _draw_measured(self) -> None:
        if not _HAVE_PLOT or self.figure is None or not self.samples:
            return
        times = self._timestamps()
        if not times:
            return
        if str((self.workflow.snapshot or {}).get("mode")) == "4":
            self._draw_altitude(times)
            return
        self._alt_thrust_axis(False)
        measured = [self._axis_rate(sample) for sample in self.samples]
        servo = str((self.workflow.snapshot or {}).get("mode")) == "3"
        if servo:
            expected = [sample.get("servo_tilt", 0.0) for sample in self.samples]
            label = "舵机指令倾角 [rad]"
        else:
            expected = [sample.get("omega_sp", 0.0) for sample in self.samples]
            label = "期望 ω_sp"
        smooth = _moving_average(measured, SMOOTH_SAMPLES)
        self.plot_axis.clear()
        self.plot_axis.plot(times, measured, linewidth=0.5, alpha=0.25, label="实测（原始，含桨振动）")
        self.plot_axis.plot(times, expected, linestyle="--", label=label)
        self.plot_axis.plot(times, smooth, linewidth=1.4, label="实测 ω·n（15 Hz 以下）")
        span = [v for v in list(smooth) + list(expected) if math.isfinite(v)]
        if span:
            low, high = min(span), max(span)
            pad = 0.15 * max(high - low, 1e-3)
            self.plot_axis.set_ylim(low - pad, high + pad)
        self.plot_axis.set_xlabel("时间 [s]")
        self.plot_axis.set_ylabel("角速度 [rad/s]")
        self.plot_axis.legend(loc="upper right")
        self._draw_erpm(times, self.samples)
        self.canvas.draw_idle()

    def _draw_altitude(self, times) -> None:
        """高度轮：高度与高度设定画在主轴 [m]，推力画在右侧副轴 [N]；缺字段的点画成断开。"""
        nan = float("nan")
        axis = self.plot_axis
        axis.clear()
        axis.plot(times, [s.get("height_sp", nan) for s in self.samples], linestyle="--",
                  label="高度设定 height_sp")
        axis.plot(times, [s.get("height", nan) for s in self.samples], linewidth=1.4,
                  label="高度 height")
        axis.set_xlabel("时间 [s]")
        axis.set_ylabel("高度 [m]")
        right = self._alt_thrust_axis(True)
        for line in list(right.lines):
            line.remove()
        right.plot(times, [s.get("thrust", nan) for s in self.samples], linewidth=0.8, alpha=0.7,
                   color="C2", label="推力 thrust [N]")
        right.relim()
        right.autoscale_view()
        right.set_ylabel("推力 [N]")
        handles = axis.get_lines() + right.get_lines()
        right.legend(handles, [line.get_label() for line in handles], loc="upper right")
        self._draw_erpm(times, self.samples)
        self.canvas.draw_idle()

    def _alt_thrust_axis(self, wanted: bool):
        """高度轮的推力副轴：要就建（并套主题），不要就拆掉，图回到单轴。"""
        axis = getattr(self, "alt_thrust_axis", None)
        if wanted and axis is None:
            axis = self.plot_axis.twinx()
            apply_matplotlib_theme(self.figure, getattr(self.panel, "ui_palette", None) or UI_PALETTE)
            axis.grid(False)
            self.alt_thrust_axis = axis
        elif not wanted and axis is not None:
            axis.remove()
            self.alt_thrust_axis = axis = None
        return axis

    def _draw_erpm(self, times, samples) -> None:
        """真实电调转速（> 0 的样本）只写成图下一行字；没有就清掉那行字。"""
        del times
        series = []
        for key, label in (("erpm", "上桨"), ("erpm_lower", "下桨")):
            spinning = sorted(float(sample.get(key, 0.0) or 0.0) for sample in samples
                              if float(sample.get(key, 0.0) or 0.0) > 0.0)
            if spinning:
                series.append((label, spinning[len(spinning) // 2]))
        if not series:
            self.erpm_var.set("")
            return
        self.erpm_var.set("电调转速（中位数）：" + "；".join(
            f"{label} {median:.0f} eRPM（电频率 {median / 60.0:.1f} Hz，"
            f"按 {POLE_PAIRS} 对极机械转频 {median / 60.0 / POLE_PAIRS:.1f} Hz）"
            for label, median in series))

    def _axis_rate(self, sample: dict[str, float]) -> float:
        snapshot = self.workflow.snapshot
        psi = (float(snapshot["psi_mrad"])*1e-3 if snapshot is not None
               else math.radians(float(self.psi_var.get() or 45.0)))
        return sample["gx"]*math.cos(psi) + sample["gy"]*math.sin(psi)
