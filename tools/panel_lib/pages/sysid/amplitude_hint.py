"""「幅值 [rad/s]」旁的预计舵机摆幅，以及按当前板子力臂自动换默认幅值。算法在 `excitation_scale.py`。

* 读到这块板子的机体参数（PARAM? 回显，缓存在 `workflow.params`）且力臂、推力、惯量都有时，
  幅值旁灰字写预计舵机摆幅；不在 6～15°（或扫频转速过快）就给建议幅值。
* 幅值还是预设值、或是上次自动设的值（作者没手填过）时，直接换成建议值并写明；手填的值永远不动。
* 下发配置或一轮在跑时不换：界面上的数必须是正在跑的那个数。
* 舵机单独（SERVO）直接给舵机摆幅 [deg]、ANGLE 验证按角度幅值走，这两种模式不估算、不换；
  高度（ALT）的幅值是推力/速度/高度，同样不估算、不换，灰字只写单位与上限。
* 开始前预计摆幅不到 5° 时，顶部大字状态加一句提醒，不挡开始（`amp_swing_warning`）。
"""

from __future__ import annotations

import math
import tkinter as tk

from ._core import firmware_tilt_levers
from .excitation_scale import (SLEW_SAFE_DPS, START_WARN_SWING_DEG, SwingEstimate, estimate,
                               format_amp, rod_lever_m, suggestion_text, swing_text)
from .sections import EXPERIMENTS

AMP_HINT_UNKNOWN = "上限 5 rad/s；连上飞控、读到机体参数（力臂、机重、惯量）后在这里估算舵机摆幅"
#: 这两种模式的舵机摆幅不由激励幅值决定（固件只拿它归一化波形）。
AMP_HINT_BY_MODE = {
    "SERVO": "上限 5 rad/s；舵机单独（SERVO）的摆幅直接按「准备」页的舵机摆幅 [deg]，这里只定波形",
    "ANGLE": "上限 5 rad/s；ANGLE 验证的摆动由「ANGLE 角度幅值」决定，这里只定波形",
    # 高度辨识：幅值是推力/速度/高度（单位随注入类型，页面上由 alt_section.alt_amp_hint 细化）。
    "ALT": "高度辨识（ALT）：幅值单位随注入类型（N / m/s / m），不估算舵机摆幅、不自动改幅值",
    "XY": "水平槽辨识（XY）：幅值单位随注入类型（rad / m/s / m），不估算舵机摆幅、不自动改幅值",
    # 吊绳偏航：幅值是差速推力 N 或偏航角速度 rad/s，舵机锁中位（页面上由 yaw_section.yaw_amp_hint 细化）。
    "YAW": "吊绳偏航辨识（YAW）：幅值单位随注入类型（N / rad/s），舵机锁中位，不估算舵机摆幅、不自动改幅值",
}
#: 实验类型预设里的幅值：输入框里还是它们就算作者没动过。
PRESET_AMPS = frozenset(preset["amp"] for _key, preset in EXPERIMENTS.values())
#: FF / 舵机单独的双脉冲单段保持到这个值就提醒（验证轮用 1000 ms，辨识要 250 ms）。
HOLD_WARN_MS = 500.0


def _positive(text) -> float | None:
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value > 0.0 else None


class AmplitudeHint:
    def _init_amp_hint_vars(self) -> None:
        self.amp_hint_var = tk.StringVar(value=AMP_HINT_UNKNOWN)
        self._amp_auto_text: str | None = None   # 上次自动写进幅值框的值
        self._amp_auto_note = ""
        self._amp_hint_running = False

    # ------------------------------------------------------------ 估算的输入

    def amp_thrust_n(self) -> float | None:
        """目标合推力：程序油门手填的目标，留空（或手动油门）用机重，与 `_throttle_command` 同源。"""
        text = self.target_thrust_var.get().strip()
        if text and not self.manual_throttle_var.get():
            return _positive(text)
        return self.workflow.weight_n()

    def amp_inertia_kg_m2(self) -> float | None:
        """固件前馈用的假定惯量：「假定惯量」填了正数用它，留空/0 固件取 airframe.ixx_kgm2。"""
        text = self.inertia_var.get().strip()
        if text:
            try:
                typed = float(text)
            except ValueError:
                return None
            if math.isfinite(typed) and typed > 0.0:
                return typed
        w = self.workflow
        found = _positive(w.params.get("airframe.ixx_kgm2"))
        if found is None:
            found = _positive(w.status.get("I_ugm2"))
            found = None if found is None else found * 1e-6
        return found

    def amp_swing_estimate(self) -> SwingEstimate | None:
        """当前激励设置在当前板子上的预计摆幅；缺参数、填错或模式不适用时返回 None。"""
        if self.mode_var.get() in AMP_HINT_BY_MODE:
            return None
        try:
            psi = float(self.psi_var.get())
            amplitude = float(self.amp_var.get())
            ramp = float(self.ramp_var.get())
            f0, f1 = float(self.f0_var.get()), float(self.f1_var.get())
        except ValueError:
            return None
        lever = rod_lever_m(firmware_tilt_levers(self.workflow.params), psi)
        thrust, inertia = self.amp_thrust_n(), self.amp_inertia_kg_m2()
        if None in (lever, thrust, inertia):
            return None
        return estimate(self.profile_var.get(), amplitude, ramp_ms=ramp, chirp_f0_hz=f0,
                        chirp_f1_hz=f1, lever_m=lever, thrust_n=thrust, inertia_kg_m2=inertia)

    # ------------------------------------------------------------ 刷新与自动换值

    def _amp_untouched(self) -> bool:
        text = self.amp_var.get().strip()
        return text in PRESET_AMPS or (self._amp_auto_text is not None and text == self._amp_auto_text)

    def _amp_locked(self) -> bool:
        """下发配置中或一轮在跑：幅值框此刻显示的就是本轮的数，不许自动改。"""
        w = self.workflow
        return bool(w.awaiting or w.pending) or (w.run_id is not None and w.end is None)

    def _autoscale_amp(self) -> None:
        if self._amp_locked() or not self._amp_untouched():
            return
        est = self.amp_swing_estimate()
        if est is None:
            return
        text = format_amp(est.suggested_amp_rad_s)
        how = (f"（扫到 {est.f1_hz:g} Hz 时舵机转速不超过 {SLEW_SAFE_DPS:.0f}°/s）"
               if est.slew_limited else "")
        self._amp_auto_text = text
        self._amp_auto_note = (f"按当前力臂 L={est.lever_m:.4f} m 自动设为 {text}，保持舵机约 "
                               f"{est.suggested_swing_deg:.0f}°{how}；手填别的值就不再自动改。")
        if self.amp_var.get().strip() != text:
            self.amp_var.set(text)

    def refresh_amp_hint(self) -> None:
        """重估舵机摆幅（需要时自动换默认幅值），再刷新顶部状态里的提醒。"""
        if not hasattr(self, "amp_hint_var") or self._amp_hint_running:
            return
        self._amp_hint_running = True
        try:
            self._autoscale_amp()
            text = self._amp_hint_text()
            if self.amp_hint_var.get() != text:
                self.amp_hint_var.set(text)
        finally:
            self._amp_hint_running = False
        self.refresh_banner()

    def _amp_hint_text(self) -> str:
        if self.mode_var.get() == "ALT" and hasattr(self, "alt_amp_hint"):
            return self.alt_amp_hint()
        if self.mode_var.get() == "XY" and hasattr(self, "xy_amp_hint"):
            return self.xy_amp_hint()
        if self.mode_var.get() == "YAW" and hasattr(self, "yaw_amp_hint"):
            return self.yaw_amp_hint()
        mode_text = AMP_HINT_BY_MODE.get(self.mode_var.get())
        if mode_text:
            return mode_text
        est = self.amp_swing_estimate()
        if est is None:
            return AMP_HINT_UNKNOWN
        lines = [swing_text(est)]
        if self._amp_auto_text is not None and self.amp_var.get().strip() == self._amp_auto_text:
            lines.append(self._amp_auto_note)
        elif est.needs_new_amplitude:
            lines.append(suggestion_text(est))
        return "\n".join(lines)

    def amp_swing_warning(self) -> str:
        """开始前的提醒（不挡开始）：预计摆幅不到 5°、或辨识轮单段保持太长时给一句，否则空串。"""
        parts = [self.hold_warning()]
        est = self.amp_swing_estimate()
        if est is not None and est.warn_before_start:
            parts.append(f"提醒：按当前幅值 {self.amp_var.get().strip()} rad/s，舵机只摆约 {est.swing_deg:.1f}°，"
                         f"小于 {START_WARN_SWING_DEG:.0f}° 多半落在舵机回差里，拟合会失真；建议把「高级设置」里的"
                         f"幅值改成 {format_amp(est.suggested_amp_rad_s)}。不改也能照常开始。")
        return " ".join(part for part in parts if part)

    def hold_warning(self) -> str:
        """FF / 舵机单独用双脉冲、单段保持 ≥ 500 ms 时的提醒（不挡开始）。

        2026-09-28 两次踩坑：沿用 RATE/ANGLE 验证的 1000 ms 做辨识，0.5 Hz 基频激不起舵机频段，
        纯延迟拟合成 0、舵机贴边界，结果被拦下；1 s 保持还累积转角，0.2 rad/s 就触发角度上限。
        """
        if self.mode_var.get() not in ("FF", "SERVO") or self.profile_var.get() != "doublet":
            return ""
        try:
            hold = float(self.hold_var.get())
        except ValueError:
            return ""
        if not hold >= HOLD_WARN_MS:
            return ""
        return (f"提醒：{self.mode_var.get()} 辨识的双脉冲单段保持 {hold:.0f} ms 太长——激不起舵机频段"
                "（纯延迟会拟合成 0、舵机参数贴边界，结果会被拦下），还容易累积转角超限。"
                "辨识请用 250 ms：在「实验类型」里重选一次「双脉冲（默认）」即可。不改也能照常开始。")


__all__ = ["AMP_HINT_BY_MODE", "AMP_HINT_UNKNOWN", "AmplitudeHint", "PRESET_AMPS"]
