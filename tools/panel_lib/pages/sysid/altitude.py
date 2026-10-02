"""「系统辨识 · Z 高度」页：槽式台架高度辨识（ALT）的开始/停止、设置、波形与结果。

高度**必须单独辨**，不能套用水平那一套。理由都是物理上的：

* 水平通道靠**倾转**产生力，量级受限于 `F·sin(tilt)`，而且是小角度线性化的；
  高度通道靠**总推力直接**产生力，是强非线性的（推力大致正比于转速平方）。
* 水平方向没有恒定偏置；垂直方向永远挂着一个 `m·g`，工作点就在悬停推力附近，
  而那一点的增益 dF/d油门 与低油门处差好几倍。
* 执行器完全不同：水平是舵机（延迟 16~41 ms 量级），高度是电调 + 桨的转速动态
  （一阶惯性，时间常数在几十到上百毫秒，而且上升/下降不对称）。
* 电池电压掉下来时推力跟着掉，高度环会直接感受到；水平环基本不受影响。

**本页只是视图。** 固件同一时间只跑一轮 SYSID，内环与高度共用同一个事务（回显核对、安全门、
收数据、存档）。所以状态和逻辑都在内环页对象里（`panel.sysid_page`，见 `alt_section.py`），
本页只把高度用的控件摆出来、绑到它的变量上；顶部状态文字两页是同一组。本页在前台时「本轮做」
就是 ALT（`__init__.py` 跟着标签切换）。杆轴方向、采样率与安全门沿用内环页的台架设置。
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from ._core import PROFILE_CODES
from .common import labelled_entry
from .live_status import STOP_HINT_AUTO, STOP_TEXT_AUTO
from .sections import PROFILE_LABELS
from .waveform import AltitudePlot


class SysIdAltitudePage:
    def __init__(self, panel, parent: ttk.Frame, engine) -> None:
        self.panel = panel
        self.parent = parent
        self.engine = engine
        self._build_run(parent)
        self.steps_notebook = ttk.Notebook(parent)
        self.steps_notebook.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        prepare, observe, results = [ttk.Frame(self.steps_notebook, padding=6) for _ in range(3)]
        for frame, title in zip((prepare, observe, results), ("1 · 准备", "2 · 看波形", "3 · 结果")):
            self.steps_notebook.add(frame, text=title)
        self.tabs = {"prepare": prepare, "observe": observe, "results": results}
        engine._build_alt(prepare)
        engine._build_alt_throttle(prepare)
        self._build_excitation(prepare)
        self._build_rig_note(prepare)
        engine.alt_plot = AltitudePlot(panel, observe, engine.erpm_var)
        ttk.Label(observe, textvariable=engine.sample_count_var,
                  style="Mono.TLabel").pack(anchor=tk.W)
        self._build_result(results)
        engine.alt_view = self
        engine.refresh_alt_hint()
        engine._refresh_preview()        # 已在 ALT（读设置文件）时把激励预览画到本页
        engine._banner_shown = None      # 新挂的状态文字要跟一次颜色
        engine.refresh_banner()

    # ------------------------------------------------------------ 界面

    def _build_run(self, parent: ttk.Frame) -> None:
        e = self.engine
        box = ttk.Frame(parent)
        box.pack(fill=tk.X, pady=(0, 4))
        self.start_button = ttk.Button(box, text="开始高度辨识", style="Primary.TButton",
                                       command=e.start_alt_run)
        self.start_button.pack(side=tk.LEFT)
        self.stop_button = ttk.Button(box, text=STOP_TEXT_AUTO, style="Danger.TButton",
                                      command=e.stop_run)
        self.stop_button.pack(side=tk.LEFT, padx=(8, 0))
        e.build_imuzero_button(box)
        ttk.Label(parent, style="Muted.TLabel", wraplength=660, justify=tk.LEFT, text=(
            "槽式台架：电机会转、机体会沿槽上下移动——先确认槽的上下限位与测距下方地面。"
            "break 跑完当场给离地/滑落阈值，vel/pos 只存档、离线分析；和「角速度 / 角度内环」页共用"
            "同一个辨识流程，同一时间只能跑一轮。")
        ).pack(anchor=tk.W)
        ttk.Label(parent, textvariable=e.banner_var, style="PageTitle.TLabel",
                  wraplength=660, justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))
        detail = ttk.Label(parent, textvariable=e.banner_detail_var, wraplength=660,
                           justify=tk.LEFT)
        detail.pack(anchor=tk.W, fill=tk.X)
        e.extra_banner_detail_labels = list(getattr(e, "extra_banner_detail_labels", [])) + [detail]
        ttk.Label(parent, textvariable=e.rc_var, style="Muted.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text=STOP_HINT_AUTO, style="Muted.TLabel", wraplength=660,
                  justify=tk.LEFT).pack(anchor=tk.W)
        ttk.Label(parent, textvariable=e.status_var, wraplength=560,
                  style="Mono.TLabel").pack(fill=tk.X, pady=(6, 0))

    def _build_excitation(self, parent: ttk.Frame) -> None:
        """与内环页「高级设置 → 激励编排」绑同一组变量：本页在前台时它们是高度激励
        （切页时两边各存一份，见 alt_section）。高度只用得到这几项。"""
        e = self.engine
        box = ttk.LabelFrame(parent, text="激励（break 只用速率；vel/pos 幅值单位随注入类型）",
                             padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        top = ttk.Frame(box)
        top.pack(fill=tk.X)
        ttk.Label(top, text="剖面").pack(side=tk.LEFT)
        self.profile_combo = ttk.Combobox(top, textvariable=e.profile_var, width=10,
                                          state="readonly", values=list(PROFILE_CODES))
        self.profile_combo.pack(side=tk.LEFT, padx=(6, 8))
        self.profile_combo.bind("<<ComboboxSelected>>", lambda _e: e._refresh_preview())
        self.profile_hint_var = tk.StringVar()
        ttk.Label(top, textvariable=self.profile_hint_var, style="Muted.TLabel").pack(side=tk.LEFT)
        e.profile_var.trace_add("write", lambda *_a: self.profile_hint_var.set(
            PROFILE_LABELS.get(e.profile_var.get(), "")))
        self.profile_hint_var.set(PROFILE_LABELS.get(e.profile_var.get(), ""))
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X, pady=(6, 0))
        labelled_entry(grid, 0, "", e.amp_var, hint_var=e.amp_hint_var, label_var=e.amp_label_var)
        labelled_entry(grid, 1, "平台/半周期 [ms]", e.hold_var)
        labelled_entry(grid, 2, "重复对数", e.repeat_var, hint="doublet 专用，上限 20")
        labelled_entry(grid, 3, "总时长 [ms]", e.dur_var,
                       hint="不能短于剖面本身，否则激励被截掉；break 不用")
        labelled_entry(grid, 4, "斜坡 [ms]", e.ramp_var, hint="每次电平过渡的时长；不能填 0")
        ttk.Label(box, textvariable=e.command_preview_var, wraplength=660,
                  style="Mono.TLabel").pack(anchor=tk.W, pady=(6, 0))

    def _build_rig_note(self, parent: ttk.Frame) -> None:
        e = self.engine
        self.rig_note_var = tk.StringVar()
        ttk.Label(parent, textvariable=self.rig_note_var, style="Muted.TLabel", wraplength=660,
                  justify=tk.LEFT).pack(anchor=tk.W)
        for variable in (e.psi_var, e.rate_var, e.angle_limit_var, e.resid_limit_var):
            variable.trace_add("write", lambda *_a: self._refresh_rig_note())
        self._refresh_rig_note()

    def _refresh_rig_note(self) -> None:
        e = self.engine
        self.rig_note_var.set(
            f"沿用「角速度 / 角度内环」页的台架设置：杆轴方位角 {e.psi_var.get()}°、"
            f"采样 {e.rate_var.get()} Hz、角度上限 {e.angle_limit_var.get()}°、"
            f"轴向残差上限 {e.resid_limit_var.get()}°/s（开跑时一并下发并核对）。")

    def _build_result(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="本轮结果", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(box, textvariable=self.engine.alt_result_var, wraplength=660,
                  justify=tk.LEFT).pack(anchor=tk.W)
        ttk.Label(box, style="Muted.TLabel", wraplength=660, justify=tk.LEFT, text=(
            "原始记录与本轮条件存进 data/identification/attitude/<日期>/alt_*。离地/滑落阈值给出"
            "推力表比例与槽摩擦（多轮用 python -m tools.sysid.breakaway 求均值与离散）；高度环参数"
            "按模型离线设计，只经 SYSID PARAM 写 RAM 用 vel/pos 验证，是否写 Flash 由你决定。")
        ).pack(anchor=tk.W, pady=(8, 0))

    # ------------------------------------------------------------ 由内环页对象调

    def show(self, which: str) -> None:
        tab = self.tabs.get(which)
        if tab is not None:
            try:
                self.steps_notebook.select(tab)
            except tk.TclError:
                pass


def mount_sysid_altitude(panel, parent: ttk.Frame) -> SysIdAltitudePage:
    page = SysIdAltitudePage(panel, parent, panel.sysid_page)
    panel.sysid_altitude_page = page
    return page
