"""「1 · 准备」页的「高度（本轮做选 ALT 时用）」分区：注入类型、附加质量、出窗余量、抬升高度。

只摆控件、绑变量、写提示；命令组装与回显核对在 `alt_config.AltWorkflow`。

* 选注入类型（且「本轮做」是 ALT）时，把该注入的默认激励写进「高级设置 → 激励编排」
  （照 `sections.apply_experiment` 的做法），作者仍可改。
* 「本轮做」切进 ALT 时写入当前注入的默认激励；从 ALT 切回别的模式时换回「实验类型」的
  角速度预设——两边幅值单位不同（N、m/s、m 对 rad/s），不能沿用。
* 合计质量 = 读回的机体质量 + 附加质量；读不到机体质量时开跑会被拒绝（`alt_config`）。
"""
from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from .alt_config import (ALT_MODE, AMP_UNITS, DEFAULT_EXTRA_MASS_G, DEFAULT_INJECT,
                         DEFAULT_LIFT_MM, DEFAULT_WIN_MM, EXTRA_MASS_RANGE_G, INJECTS,
                         INJECT_LABELS, INJECT_PRESETS, LIFT_RANGE_MM, WIN_RANGE_MM,
                         airframe_mass_kg)
from .common import labelled_entry
# _EXPERIMENT_VARS：预设键 -> 激励编排变量名，与实验类型预设共用一份。
from .sections import _EXPERIMENT_VARS, EXCITATION_TITLE

AMP_LABEL_RATE = "幅值 [rad/s]"
ALT_EXCITATION_TITLE = "激励编排（高度辨识 ALT：幅值是推力 / 速度 / 高度，按注入类型）"


class AltSection:
    def _init_alt_vars(self) -> None:
        self.alt_inject_var = tk.StringVar(value=DEFAULT_INJECT)
        self.alt_extra_mass_var = tk.StringVar(value=DEFAULT_EXTRA_MASS_G)
        self.alt_win_var = tk.StringVar(value=DEFAULT_WIN_MM)
        self.alt_lift_var = tk.StringVar(value=DEFAULT_LIFT_MM)
        self.alt_inject_hint_var = tk.StringVar(value=INJECT_LABELS[DEFAULT_INJECT])
        self.alt_mass_hint_var = tk.StringVar(value="")
        self.alt_amp_note_var = tk.StringVar(value="")
        self.amp_label_var = tk.StringVar(value=AMP_LABEL_RATE)
        self._alt_mode_prev = self.mode_var.get()
        self.mode_var.trace_add("write", lambda *_a: self._on_alt_mode())
        self.alt_inject_var.trace_add("write", lambda *_a: self._on_alt_inject())
        self.alt_extra_mass_var.trace_add("write", lambda *_a: self.refresh_alt_hint())

    def _build_alt(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="高度（「本轮做」选 ALT 时用）", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        top = ttk.Frame(box)
        top.pack(fill=tk.X)
        ttk.Label(top, text="注入类型").pack(side=tk.LEFT)
        self.alt_inject_combo = ttk.Combobox(top, textvariable=self.alt_inject_var, width=7,
                                             state="readonly", values=INJECTS)
        self.alt_inject_combo.pack(side=tk.LEFT, padx=(6, 8))
        self.alt_inject_combo.bind("<<ComboboxSelected>>", lambda _e: self.select_alt_inject())
        ttk.Label(top, textvariable=self.alt_inject_hint_var, style="Muted.TLabel",
                  wraplength=460, justify=tk.LEFT).pack(side=tk.LEFT)
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X, pady=(6, 0))
        labelled_entry(grid, 0, "台架随动附加质量 [g]", self.alt_extra_mass_var,
                       hint_var=self.alt_mass_hint_var)
        labelled_entry(grid, 1, "出窗余量 [mm]", self.alt_win_var,
                       hint=f"高度超过「开跑高度 + 抬升高度 + 余量」就中止（原因 height_window）；"
                            f"{WIN_RANGE_MM[0]}～{WIN_RANGE_MM[1]} mm，按槽的上限位留余量")
        labelled_entry(grid, 2, "抬升高度 [mm]", self.alt_lift_var,
                       hint=f"升推力后由高度环把机体抬高这么多、稳定后再激励；"
                            f"{LIFT_RANGE_MM[0]}～{LIFT_RANGE_MM[1]} mm")
        ttk.Label(box, textvariable=self.alt_amp_note_var, style="Muted.TLabel", wraplength=620,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(box, wraplength=620, justify=tk.LEFT, text=(
            "电机会转、机体会沿槽上下移动：开跑前确认槽的上下限位牢靠、测距（TOF）下方地面平整无遮挡。"
            "要程序油门（不要勾「遥控器手动给油门」）；解锁、油门杆在最低时点「开始辨识」：固件升推力、"
            "高度环抬升并稳定、激励、再下降回落。页面只存档，不拟合，分析离线进行。")
        ).pack(anchor=tk.W, pady=(4, 0))
        self.refresh_alt_hint()

    # ------------------------------------------------------------ 注入类型与默认激励

    def alt_mode_selected(self) -> bool:
        return self.mode_var.get() == ALT_MODE

    def select_alt_inject(self) -> None:
        """下拉选了注入类型：ALT 模式下把它的默认激励写进激励编排；别的模式只更新提示。"""
        if self.alt_mode_selected():
            self.apply_alt_inject()
        self.refresh_alt_hint()

    def apply_alt_inject(self) -> None:
        """把当前注入类型的默认激励写进「高级设置」的激励参数。"""
        preset = INJECT_PRESETS.get(self.alt_inject_var.get(), INJECT_PRESETS[DEFAULT_INJECT])
        for field, value in preset.items():
            getattr(self, _EXPERIMENT_VARS[field]).set(value)
        self._refresh_preview()

    def _on_alt_inject(self) -> None:
        self.refresh_alt_hint()
        if hasattr(self, "refresh_amp_hint"):
            self.refresh_amp_hint()        # 幅值旁的单位与上限跟着注入类型变

    def _on_alt_mode(self) -> None:
        mode, previous = self.mode_var.get(), self._alt_mode_prev
        self._alt_mode_prev = mode
        if hasattr(self, "profile_hint_var"):          # 界面搭好之后才动激励编排
            if mode == ALT_MODE and previous != ALT_MODE:
                self.apply_alt_inject()
            elif previous == ALT_MODE and mode != ALT_MODE:
                self.apply_experiment()
        self.refresh_alt_hint()

    # ------------------------------------------------------------ 提示

    def alt_amp_unit(self) -> str:
        return AMP_UNITS.get(self.alt_inject_var.get(), AMP_UNITS[DEFAULT_INJECT])[0]

    def alt_amp_hint(self) -> str:
        """ALT 下「幅值」旁的灰字：单位与上限随注入类型；不估算舵机摆幅、不自动改幅值。"""
        inject = self.alt_inject_var.get()
        unit, limit = AMP_UNITS.get(inject, AMP_UNITS[DEFAULT_INJECT])
        return (f"高度辨识（ALT · {inject}）：幅值单位 {unit}，上限 {limit:g} {unit}；"
                "不估算舵机摆幅、不自动改幅值")

    def refresh_alt_hint(self) -> None:
        if not hasattr(self, "alt_mass_hint_var") or not hasattr(self, "workflow"):
            return
        inject = self.alt_inject_var.get()
        self.alt_inject_hint_var.set(INJECT_LABELS.get(inject, ""))
        unit, limit = AMP_UNITS.get(inject, AMP_UNITS[DEFAULT_INJECT])
        preset = INJECT_PRESETS.get(inject, INJECT_PRESETS[DEFAULT_INJECT])
        self.alt_amp_note_var.set(
            f"激励幅值（「高级设置 → 激励编排」）的单位随注入类型：现在是 {unit}，上限 {limit:g} {unit}。"
            f"默认 {preset['profile']} 幅值 {preset['amp']} {unit}、每段 {preset['hold']} ms、"
            f"{preset['repeat']} 对；ALT 模式下选注入类型会写入这组默认值，可再改。")
        self.amp_label_var.set(f"幅值 [{unit}]" if self.alt_mode_selected() else AMP_LABEL_RATE)
        if hasattr(self, "excitation_box"):
            self.excitation_box.configure(text=ALT_EXCITATION_TITLE if self.alt_mode_selected()
                                          else EXCITATION_TITLE)
        self.alt_mass_hint_var.set(self._alt_mass_text())

    def _alt_mass_text(self) -> str:
        low, high = EXTRA_MASS_RANGE_G
        try:
            extra = float(self.alt_extra_mass_var.get())
            if not low <= extra <= high:
                raise ValueError
        except ValueError:
            return f"要填 {low:g}～{high:g} 的数字（单位 g，默认 {DEFAULT_EXTRA_MASS_G}：随机体上下动的碳杆）"
        w = self.workflow
        mass = airframe_mass_kg(w.params, w.ready)
        if mass is None:
            return ("合计 = 机体质量 + 附加质量；机体质量还没读到（连上飞控后自动读取，"
                    "开跑时仍读不到就不开跑）")
        airframe_g = mass * 1000.0
        total = airframe_g + extra
        return (f"合计 {total:.1f} g = 机体 {airframe_g:.1f} g（飞控机体参数）+ 附加 {extra:g} g；"
                f"下发 mass_g={int(round(total))}")


__all__ = ["AMP_LABEL_RATE", "AltSection"]
