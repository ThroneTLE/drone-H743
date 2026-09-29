"""「1 · 准备」里的「台架刚度（挂砝码，可选）」：输入、即时换算、交给拟合。

算法在 `sysid.rig_stiffness`（经 `_core`）；这里只摆控件、解析输入、显示结论。
填了就代替只算重力的 m·g·d 作单摆回中刚度（带桨与舵机单独两种拟合都用）；不填照旧。

角度是面板上的姿态读数，读哪个角、砝码挂哪两侧都随「台架」里的杆轴方位 ψ 变（标签跟着改）：
俯仰杆读俯仰、横滚杆读横滚，斜杆读俯仰再按 sinψ 换成绕杆转角。分析时按那一轮记录的 ψ 换算。
"""
from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

from ._core import (STIFFNESS_ASSUMPTIONS, extra_stiffness, hanging_sides, implied_pivot_m,
                    reading_axis, weight_test_stiffness)
from .common import labelled_entry
from .geometry import fc_above_cg_m


class StiffnessPanel:
    def _init_stiffness_vars(self) -> None:
        self.stiffness_weight_var = tk.StringVar(value="")
        self.stiffness_distance_var = tk.StringVar(value="")
        self.stiffness_front_var = tk.StringVar(value="")
        self.stiffness_back_var = tk.StringVar(value="")
        self.stiffness_level_var = tk.StringVar(value="")
        self.stiffness_hint_var = tk.StringVar(value="")
        # 三个角度的标签与"怎么挂、读哪个角"的说明随杆轴方位 ψ 变。
        self.stiffness_front_label_var = tk.StringVar(value="")
        self.stiffness_back_label_var = tk.StringVar(value="")
        self.stiffness_level_label_var = tk.StringVar(value="")
        self.stiffness_howto_var = tk.StringVar(value="")

    def _stiffness_vars(self) -> tuple[tk.StringVar, ...]:
        return (self.stiffness_weight_var, self.stiffness_distance_var, self.stiffness_front_var,
                self.stiffness_back_var, self.stiffness_level_var)

    def _build_stiffness(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="台架刚度（挂砝码，可选）", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(box, wraplength=620, justify=tk.LEFT, style="Muted.TLabel", text=(
            "机体静止在杆上，在杆的高度上、离杆轴水平 x 处挂一个砝码，杆的两侧各挂一次，"
            "读面板上的姿态角。算出的台架刚度代替只算重力的 m·g·d（线缆等额外刚度一起算进去）。"
            "不填就照旧。")).pack(anchor=tk.W)
        ttk.Label(box, textvariable=self.stiffness_howto_var, wraplength=620,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X, pady=(6, 0))
        labelled_entry(grid, 0, "砝码质量 [g]", self.stiffness_weight_var)
        labelled_entry(grid, 1, "砝码到杆轴的水平距离 [m]", self.stiffness_distance_var,
                       hint="砝码挂在杆的高度上，从杆轴量垂直于杆的水平距离")
        labelled_entry(grid, 2, "", self.stiffness_front_var,
                       label_var=self.stiffness_front_label_var)
        labelled_entry(grid, 3, "", self.stiffness_back_var,
                       label_var=self.stiffness_back_label_var)
        labelled_entry(grid, 4, "", self.stiffness_level_var,
                       label_var=self.stiffness_level_label_var,
                       hint="可选：用来核对两侧是否一致")
        ttk.Label(box, textvariable=self.stiffness_hint_var, justify=tk.LEFT,
                  wraplength=620).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(box, text="前提：" + STIFFNESS_ASSUMPTIONS, style="Muted.TLabel",
                  wraplength=620, justify=tk.LEFT).pack(anchor=tk.W)
        for variable in (*self._stiffness_vars(), self.psi_var):
            variable.trace_add("write", lambda *_a: self.refresh_stiffness_hint())

    def _stiffness_azimuth_deg(self) -> float | None:
        """「台架」里当前的杆轴方位 ψ [deg]；没填好返回 None。"""
        try:
            value = float(self.psi_var.get())
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    def _refresh_stiffness_labels(self, azimuth_deg: float | None) -> None:
        if azimuth_deg is None:
            reading, (first, second) = "姿态", ("一侧", "另一侧")
            howto = "先在「台架」里填好杆轴方位 ψ：读哪个角、砝码挂哪两侧都取决于它。"
        else:
            reading, projection = reading_axis(azimuth_deg)
            first, second = hanging_sides(azimuth_deg)
            howto = (f"当前杆轴方位 ψ = {azimuth_deg:g}°：砝码挂在与杆轴垂直的水平方向上——"
                     f"{first}、{second}各一次；读面板上的{reading}角。")
            if abs(abs(projection) - 1.0) > 1e-3:
                howto += (f"斜杆上{reading}只显示绕杆转角的 {abs(projection):.2f} 倍，"
                          "这里自动换算，照读数填即可。")
        self.stiffness_front_label_var.set(f"砝码挂{first}时的{reading}角 [deg]")
        self.stiffness_back_label_var.set(f"砝码挂{second}时的{reading}角 [deg]")
        self.stiffness_level_label_var.set(f"不挂砝码时的{reading}角 [deg]")
        self.stiffness_howto_var.set(howto)

    def rig_stiffness(self, azimuth_deg: float | None = None):
        """`StiffnessEstimate`；五项全空返回 None；填了一半或填错抛中文 ValueError。

        角度按杆轴方位换成绕杆转角：`azimuth_deg` 不给就用「台架」里当前的 ψ（分析时传那一轮
        记录的 ψ）。"""
        weight, distance, front, back, level = (v.get().strip() for v in self._stiffness_vars())
        if not any((weight, distance, front, back, level)):
            return None
        if not all((weight, distance, front, back)):
            raise ValueError("挂砝码试验没填全：砝码质量、水平距离、两侧各挂一次的角度都要填"
                             "（不挂时的角度可选）；不用这项就全部清空。")
        if azimuth_deg is None:
            azimuth_deg = self._stiffness_azimuth_deg()
            if azimuth_deg is None:
                raise ValueError("挂砝码试验要按杆轴方位换算角度：先在「台架」里填好 ψ。")
        return weight_test_stiffness(weight, distance, front, back, level or None,
                                     azimuth_deg=azimuth_deg)

    def rig_stiffness_n_m_rad(self, azimuth_deg: float | None = None) -> float | None:
        estimate = self.rig_stiffness(azimuth_deg)
        return None if estimate is None else estimate.stiffness_n_m_rad

    def _stiffness_context(self) -> tuple[float | None, float | None]:
        """(机体质量 kg, 几何杆高 d m)：有就用来给等效杆高，没有就不显示那一句。"""
        workflow = self.workflow
        mass = None
        for source in (lambda: float(workflow.params["airframe.mass_kg"]),
                       lambda: float(workflow.ready["mass_mg"]) * 1e-6):
            try:
                value = source()
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(value) and value > 0.0:
                mass = value
                break
        pivot = None
        try:
            rod, override = self.geometry_inputs()
            found = fc_above_cg_m(workflow.params, workflow.ready)
            pivot = override if override is not None else (
                rod + found[0] if (rod is not None and found is not None) else None)
        except (ValueError, TypeError):
            pivot = None
        return mass, pivot

    def refresh_stiffness_hint(self) -> None:
        if not hasattr(self, "stiffness_hint_var"):
            return
        self._refresh_stiffness_labels(self._stiffness_azimuth_deg())
        try:
            estimate = self.rig_stiffness()
        except ValueError as error:
            self.stiffness_hint_var.set(str(error))
            return
        if estimate is None:
            self.stiffness_hint_var.set("没填：拟合按 m·g·d（只算重力）作回中刚度。")
            return
        lines = [f"台架刚度 K = {estimate.stiffness_n_m_rad:.4f} N·m/rad（两侧两次一起算）"]
        if math.isfinite(estimate.front_n_m_rad):
            lines.append(f"单边核对：第一侧 {estimate.front_n_m_rad:.4f}、第二侧 "
                         f"{estimate.back_n_m_rad:.4f} N·m/rad，相差 "
                         f"{100.0 * estimate.side_disagreement:.0f}%")
        mass, pivot = self._stiffness_context()
        if mass is not None:
            text = (f"等效杆高 d_eff = K/(m·g) = "
                    f"{implied_pivot_m(estimate.stiffness_n_m_rad, mass):.4f} m")
            if pivot is not None:
                text += (f"（量的杆高 d = {pivot:.4f} m，多出 "
                         f"{extra_stiffness(estimate.stiffness_n_m_rad, mass, pivot):+.4f} N·m/rad）")
            lines.append(text)
        lines.extend(estimate.notes)
        self.stiffness_hint_var.set("\n".join(lines))

    def save_stiffness_settings(self) -> None:
        """分析用到挂砝码试验时单独记一次；写不进去只提示。"""
        from . import settings_store
        try:
            settings_store.save_stiffness(self, settings_store.settings_path())
        except (OSError, ValueError, TypeError) as error:
            self.status_var.set(f"挂砝码试验没能保存（{error}），这次照常分析。")


__all__ = ["StiffnessPanel"]
