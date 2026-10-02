"""「系统辨识 · 偏航（吊绳）」页：吊绳偏航辨识（YAW）的开始/停止、设置、波形与结果。

台架：机体用绳从上方吊住，推力始终小于机重（绳子绷紧、升不起来），舵机锁中位、只用上下桨差速产生
偏航力矩。契约见 `doc/sysid-yaw-contract.md`。

**本页只是视图。** 固件同一时间只跑一轮 SYSID，内环、Z 高度、XY 与偏航共用同一个事务（回显核对、安全门、
收数据、存档）。所以状态和逻辑都在内环页对象里（`panel.sysid_page`，见 `yaw_section.py`），
本页只把偏航用的控件摆出来、绑到它的变量上；顶部状态文字几页是同一组。本页在前台时「本轮做」
就是 YAW（`__init__.py` 跟着标签切换）。
"""
from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from ._core import PROFILE_CODES
from .common import labelled_entry
from .live_status import STOP_HINT_AUTO, STOP_TEXT_AUTO
from .sections import PROFILE_LABELS
from .yaw_plot import YawPlot


class SysIdYawPage:
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
        engine._build_yaw(prepare)
        self._build_excitation(prepare)
        engine.yaw_plot = YawPlot(panel, observe, engine.erpm_var)
        ttk.Label(observe, textvariable=engine.sample_count_var,
                  style="Mono.TLabel").pack(anchor=tk.W)
        self._build_result(results)
        engine.yaw_view = self
        engine.refresh_yaw_hint()
        engine._refresh_preview()        # 已在 YAW（读设置文件）时把激励预览画到本页
        engine._banner_shown = None      # 新挂的状态文字要跟一次颜色
        engine.refresh_banner()

    # ------------------------------------------------------------ 界面

    def _build_run(self, parent: ttk.Frame) -> None:
        e = self.engine
        box = ttk.Frame(parent)
        box.pack(fill=tk.X, pady=(0, 4))
        self.start_button = ttk.Button(box, text="开始吊绳偏航辨识", style="Primary.TButton",
                                       command=e.start_yaw_run)
        self.start_button.pack(side=tk.LEFT)
        self.stop_button = ttk.Button(box, text=STOP_TEXT_AUTO, style="Danger.TButton",
                                      command=e.stop_run)
        self.stop_button.pack(side=tk.LEFT, padx=(8, 0))
        e.build_imuzero_button(box)
        ttk.Label(parent, style="Muted.TLabel", wraplength=660, justify=tk.LEFT, text=(
            "吊绳台架：电机会转、机体会绕绳偏航——先确认绳子绷紧、周围没有会被桨扫到的东西。"
            "diff 跑完当场给偏航对象（b=1/Izz、k 实测/k 模型、阻尼、绳扭转刚度、延迟）与偏航环建议增益，"
            "rate 给阶跃指标；和其它辨识页共用同一个辨识流程，同一时间只能跑一轮。")
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
        """与内环页「高级设置 → 激励编排」绑同一组变量：本页在前台时它们是偏航激励
        （切页时各页各存一份，见 alt_section）。"""
        e = self.engine
        box = ttk.LabelFrame(
            parent, text="激励（幅值单位随注入类型：diff 差速推力 N、rate 偏航角速度 rad/s）", padding=8)
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
                       hint="不能短于剖面本身，否则激励被截掉")
        labelled_entry(grid, 4, "斜坡 [ms]", e.ramp_var, hint="每次电平过渡的时长；不能填 0")
        ttk.Label(box, textvariable=e.command_preview_var, wraplength=660,
                  style="Mono.TLabel").pack(anchor=tk.W, pady=(6, 0))

    def _build_result(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="本轮结果", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(box, textvariable=self.engine.yaw_result_var, wraplength=660,
                  justify=tk.LEFT).pack(anchor=tk.W)
        ttk.Label(box, style="Muted.TLabel", wraplength=660, justify=tk.LEFT, text=(
            "原始记录与本轮条件存进 data/identification/yaw/<日期>/yaw_*。分析读的是这一轮的陀螺 z 与差速力矩指令；"
            "命令行重算：python -m sysid.yaw_analysis <存档目录>（在 tools 目录下）。建议增益只是参考，"
            "本页不写飞控、更不写 Flash：确认后用 SYSID PARAM 试用，再用 rate 注入验证。")
        ).pack(anchor=tk.W, pady=(8, 0))

    # ------------------------------------------------------------ 由内环页对象调

    def show(self, which: str) -> None:
        tab = self.tabs.get(which)
        if tab is not None:
            try:
                self.steps_notebook.select(tab)
            except tk.TclError:
                pass


def mount_sysid_yaw(panel, parent: ttk.Frame) -> SysIdYawPage:
    page = SysIdYawPage(panel, parent, panel.sysid_page)
    panel.sysid_yaw_page = page
    return page
