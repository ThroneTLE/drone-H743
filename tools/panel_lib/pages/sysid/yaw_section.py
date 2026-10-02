"""吊绳偏航辨识（YAW）的设置状态与「偏航设置」分区：注入类型、总推力、差速上限、绞绳上限、实时读数。

控件摆在「系统辨识 · 偏航（吊绳）」页（`yaw_view.py`），状态和逻辑留在内环页对象里：固件同一时间只跑
一轮 SYSID，几页共用同一个事务（`workflow.py`），不能各开一套。命令组装与回显核对在
`yaw_config.YawWorkflow`。机制与 `xy_section.py` 一致：

* **哪一页在前台，「本轮做」就是哪一类**（`alt_section.set_front_view`）：切到偏航页即 YAW。
* 偏航页的幅值单位（diff 是差速推力 N、rate 是 rad/s）与别的页不同，激励参数各存一份、切换时还原。
* 总推力 [N] 留空 = 0.5×飞控悬停推力参数 coax.hover_thrust_n；页面同时显示机重（mass×g，与固件
  lift 判据同口径）与 0.8×机重上限（超过它绳子会松、机体被提起来，固件拒 `lift`）。
* 差速幅值的上限需要单桨最大推力 T单max：页面每隔几秒发 `THRMODE?` 读 `tmax_mn=`（此刻可用值），
  读不到时用保守值 6 N 并在提示里写明；开跑后以 `SYSID YAWSTART` 的 yaw_single_max_mn 为准。
* 实时偏航角速度取最近一个记录样本的 vz；偏航角由 vz 按时间积分得到（记录里的 height 在 ±187° 饱和）。
  只在一轮采集中/刚结束时有数。
"""
from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk

from ...proto import parse_kv
from .common import labelled_entry
from .sections import _EXPERIMENT_VARS
from ._core import yaw_integrate_psi
from .yaw_config import (AMP_UNITS, CONSERVATIVE_T_SINGLE_MAX_N, DEFAULT_INJECT, DEFAULT_TWIST_DEG,
                         INJECTS, INJECT_LABELS, INJECT_PRESETS, LIFT_FRACTION, TWIST_RANGE_DEG,
                         YAW_MAX_THROTTLE_PCT, YAW_MODE, YAW_MODE_CODE, amplitude_limit,
                         default_thrust_n, lift_weight_n)

YAW_EXCITATION_TITLE = "激励编排（吊绳偏航 YAW：diff 是差速推力 ΔT [N]，rate 是偏航角速度 [rad/s]）"
YAW_RESULT_IDLE = ("跑完一轮吊绳偏航辨识后，这里显示本轮设置、分析结果（diff：b=1/Izz、k 实测/k 模型、阻尼、"
                   "绳扭转刚度、延迟、偏航环建议增益；rate：阶跃指标与饱和占比）、数据是否可用与存档说明。")
#: 多久重读一次单桨最大推力（THRMODE?）[s]：它随电池电压变，开跑前取最新的。
TMAX_POLL_S = 5.0
#: 读到的值超过这个时间就当作旧的 [s]。
TMAX_FRESH_S = 30.0


class YawSection:
    def _init_yaw_vars(self) -> None:
        self.yaw_inject_var = tk.StringVar(value=DEFAULT_INJECT)
        self.yaw_thrust_var = tk.StringVar(value="")          # 总推力 [N]；留空 = 0.5×悬停推力
        self.yaw_twist_var = tk.StringVar(value=DEFAULT_TWIST_DEG)
        self.yaw_inject_hint_var = tk.StringVar(value=INJECT_LABELS[DEFAULT_INJECT])
        self.yaw_thrust_hint_var = tk.StringVar(value="")
        self.yaw_limit_var = tk.StringVar(value="")
        self.yaw_twist_hint_var = tk.StringVar(value="")
        self.yaw_amp_note_var = tk.StringVar(value="")
        self.yaw_live_var = tk.StringVar(value="")
        self.yaw_result_var = tk.StringVar(value=YAW_RESULT_IDLE)
        self.yaw_view = None                  # 「偏航（吊绳）」页挂上来后指向它（yaw_view.py）
        self._yaw_t_single_max_n: float | None = None
        self._yaw_t_single_max_time = 0.0
        self._yaw_tmax_asked = 0.0
        self._yaw_psi_cache: tuple[int, float] = (0, 0.0)     # (已积分的样本数, 此时的 ψ)
        self.yaw_inject_var.trace_add("write", lambda *_a: self._on_yaw_inject())
        for variable in (self.yaw_thrust_var, self.yaw_twist_var):
            variable.trace_add("write", lambda *_a: self.refresh_yaw_hint())

    # ------------------------------------------------------------ 界面

    def _build_yaw(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="偏航设置（吊绳）", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(box, wraplength=620, justify=tk.LEFT, text=(
            "台架：机体用绳从上方吊住（顶端最好装转环，偏航自由；绳越长扭转刚度越小）。推力始终小于机重，"
            "绳子一直绷紧、机体升不起来。本模式舵机锁中位、不跑横滚俯仰环，只用上下桨差速产生偏航力矩。"
            "推力越小，差速余量越大。")).pack(anchor=tk.W)
        top = ttk.Frame(box)
        top.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(top, text="注入类型").pack(side=tk.LEFT)
        self.yaw_inject_combo = ttk.Combobox(top, textvariable=self.yaw_inject_var, width=7,
                                             state="readonly", values=INJECTS)
        self.yaw_inject_combo.pack(side=tk.LEFT, padx=(6, 8))
        self.yaw_inject_combo.bind("<<ComboboxSelected>>", lambda _e: self.select_yaw_inject())
        ttk.Label(top, textvariable=self.yaw_inject_hint_var, style="Muted.TLabel",
                  wraplength=460, justify=tk.LEFT).pack(side=tk.LEFT)
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X, pady=(6, 0))
        labelled_entry(grid, 0, "总推力 [N]", self.yaw_thrust_var, hint_var=self.yaw_thrust_hint_var)
        labelled_entry(grid, 1, "绞绳上限 [°]", self.yaw_twist_var, hint_var=self.yaw_twist_hint_var)
        ttk.Label(box, textvariable=self.yaw_limit_var, style="Muted.TLabel", wraplength=620,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(6, 0))
        ttk.Label(box, textvariable=self.yaw_amp_note_var, style="Muted.TLabel", wraplength=620,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(box, textvariable=self.yaw_live_var, style="Mono.TLabel").pack(anchor=tk.W, pady=(6, 0))
        ttk.Label(box, wraplength=620, justify=tk.LEFT, text=(
            "开跑前确认：绳子绷紧且长度合适、机体周围没有会被桨扫到的东西、遥控器随时能上锁。"
            "解锁、油门杆在最低时点「开始吊绳偏航辨识」：程序把总推力升到设定值、稳定、按激励差速（或闭环偏航）、"
            "再降回；偏航角速度超过 8 rad/s 或偏航角超过绞绳上限会软停（推力 1 s 降回）。"
            "diff 测对象，算出 b、阻尼、绳扭转刚度与延迟并给偏航角速度环建议增益；rate 用生产偏航环验证。")
        ).pack(anchor=tk.W, pady=(4, 0))
        self.refresh_yaw_hint()
        self.refresh_yaw_live()

    # ------------------------------------------------------------ 开始与注入类型

    def start_yaw_run(self) -> None:
        """「偏航（吊绳）」页的开始按钮：不论之前在哪，都按 YAW 开跑。"""
        self._front_view = YAW_MODE
        self._apply_view_mode(YAW_MODE)
        self.start_run()

    def yaw_mode_selected(self) -> bool:
        return self.mode_var.get() == YAW_MODE

    def select_yaw_inject(self) -> None:
        """下拉选了注入类型：YAW 模式下把它的默认激励写进激励编排；别的模式只更新提示。"""
        if self.yaw_mode_selected():
            self.apply_yaw_inject()
        self.refresh_yaw_hint()

    def apply_yaw_inject(self) -> None:
        preset = INJECT_PRESETS.get(self.yaw_inject_var.get(), INJECT_PRESETS[DEFAULT_INJECT])
        for field, value in preset.items():
            getattr(self, _EXPERIMENT_VARS[field]).set(value)
        self._refresh_preview()

    def _on_yaw_inject(self) -> None:
        self.refresh_yaw_hint()
        if hasattr(self, "refresh_amp_hint"):
            self.refresh_amp_hint()        # 幅值旁的单位与上限跟着注入类型变

    def yaw_run_active(self) -> bool:
        """正在跑（或刚结束）的是吊绳偏航轮，或还没开始时「本轮做」选了 YAW。"""
        w = self.workflow
        if w.run_id is not None or w.awaiting == "SYSID START":
            return str((w.snapshot or {}).get("mode")) == str(YAW_MODE_CODE)
        return self.yaw_mode_selected()

    def yaw_throttle_settings(self) -> tuple[bool, float | None, float]:
        """`(手动?, 总推力 N 或 None=0.5×悬停, 最高油门%)`：偏航辨识总是程序油门，最高油门固定。"""
        text = self.yaw_thrust_var.get().strip()
        target = None
        if text:
            try:
                target = float(text)
            except ValueError:
                raise ValueError("总推力要填数字（单位 N），或留空用 0.5×悬停推力") from None
            if not (math.isfinite(target) and target > 0):
                raise ValueError("总推力要填正数（单位 N），或留空用 0.5×悬停推力")
        return False, target, YAW_MAX_THROTTLE_PCT

    def yaw_geometry_inputs(self) -> tuple[float | None, float | None]:
        """偏航不需要杆到质心的距离：内环页填了就用，没填就按 0（只为满足固件的台架几何下发）。"""
        try:
            return self.geometry_inputs()
        except ValueError:
            return 0.0, None

    # ------------------------------------------------------------ 总推力 / 单桨最大推力

    def yaw_effective_thrust_n(self) -> float | None:
        """总推力 [N]：填了用填的，留空 = 0.5×悬停推力参数；都没有返回 None。"""
        text = self.yaw_thrust_var.get().strip()
        if text:
            try:
                value = float(text)
            except ValueError:
                return None
            return value if math.isfinite(value) else None
        return default_thrust_n(self.workflow.params)

    @property
    def yaw_t_single_max_n(self) -> float | None:
        """最近读到的单桨最大推力 [N]（THRMODE? 的 tmax_mn）；没读到或过旧返回 None。"""
        if self._yaw_t_single_max_n is None or time.monotonic() - self._yaw_t_single_max_time > TMAX_FRESH_S:
            return None
        return self._yaw_t_single_max_n

    def yaw_handle_thrmode(self, text: str) -> None:
        """`THRMODE ... tmax_mn=<mN> hover_mn=<mN>`：记下单桨最大推力（拒绝行没有这个键，忽略）。"""
        values = parse_kv(text)
        try:
            tmax = float(values["tmax_mn"]) * 1e-3
        except (KeyError, ValueError):
            return
        if math.isfinite(tmax) and tmax > 0.0:
            self._yaw_t_single_max_n = tmax
            self._yaw_t_single_max_time = time.monotonic()
            self.refresh_yaw_hint()

    def yaw_poll_idle(self) -> None:
        """偏航页在前台且空闲时，每隔几秒问一次 `THRMODE?`（只回一行，不进配置事务）。"""
        view = self.yaw_view
        if view is None or not self.yaw_mode_selected():
            return
        w = self.workflow
        if w.closed or w.awaiting or (w.run_id is not None and w.end is None):
            return
        now = time.monotonic()
        if now - self._yaw_tmax_asked < TMAX_POLL_S:
            return
        try:
            if not view.parent.winfo_viewable():
                return
        except tk.TclError:
            return
        self._yaw_tmax_asked = now
        self.send("THRMODE?", quiet=True)

    # ------------------------------------------------------------ 提示

    def yaw_amp_unit(self) -> str:
        return AMP_UNITS.get(self.yaw_inject_var.get(), AMP_UNITS[DEFAULT_INJECT])

    def yaw_amp_hint(self) -> str:
        """YAW 下「幅值」旁的灰字：单位与上限随注入类型；不估算舵机摆幅、不自动改幅值。"""
        unit, limit = self._yaw_limit()
        text = f"{limit:.3g} {unit}" if limit is not None else "（先填好总推力才知道）"
        return (f"吊绳偏航（YAW · {self.yaw_inject_var.get()}）：幅值单位 {unit}，上限 {text}；"
                "不估算舵机摆幅、不自动改幅值")

    def _yaw_limit(self) -> tuple[str, float | None]:
        return amplitude_limit(self.yaw_inject_var.get(), self.yaw_effective_thrust_n(),
                               self.yaw_t_single_max_n)

    def refresh_yaw_hint(self) -> None:
        if not hasattr(self, "yaw_thrust_hint_var") or not hasattr(self, "workflow"):
            return
        inject = self.yaw_inject_var.get()
        self.yaw_inject_hint_var.set(INJECT_LABELS.get(inject, ""))
        w = self.workflow
        weight = lift_weight_n(w.params, w.ready)
        thrust = self.yaw_effective_thrust_n()
        text = self.yaw_thrust_var.get().strip()
        default = default_thrust_n(w.params)
        weight_text = (f"机重 {weight:.2f} N，{LIFT_FRACTION:g}×机重上限 {LIFT_FRACTION * weight:.2f} N"
                       if weight is not None else "机重还没读到（连上飞控后自动读取）")
        if text:
            head = f"将使用 {text} N"
        elif default is not None:
            head = f"留空 = 0.5×悬停推力 = {default:.2f} N（飞控参数 coax.hover_thrust_n）"
        else:
            head = "留空 = 0.5×悬停推力（连上飞控后读取；读不到或为 0 就必须手填）"
        warn = ""
        if thrust is not None and weight is not None and thrust > LIFT_FRACTION * weight:
            warn = "；超过上限，机体会被提起来，飞控会拒绝（lift）"
        self.yaw_thrust_hint_var.set(f"{head}；{weight_text}{warn}")
        self.yaw_twist_hint_var.set(
            f"开跑起陀螺积分的偏航角超过它就软停（{TWIST_RANGE_DEG[0]}～{TWIST_RANGE_DEG[1]}°，绳子会绞；默认 {DEFAULT_TWIST_DEG}°）")
        unit, limit = self._yaw_limit()
        t_max = self.yaw_t_single_max_n
        source = (f"单桨最大推力 {t_max:.2f} N（THRMODE? 读到）" if t_max is not None else
                  f"还没读到单桨最大推力，按保守值 {CONSERVATIVE_T_SINGLE_MAX_N:g} N 估")
        if inject == "diff":
            self.yaw_limit_var.set(
                ("差速幅值上限 " + (f"{limit:.3g} N" if limit is not None else "（先填好总推力）")
                 + f" = min(0.8×总推力, 2×(单桨最大推力 − 总推力/2))；{source}。"))
        else:
            self.yaw_limit_var.set(f"偏航角速度参考上限 {limit:g} rad/s。")
        preset = INJECT_PRESETS.get(inject, INJECT_PRESETS[DEFAULT_INJECT])
        self.yaw_amp_note_var.set(
            f"激励幅值（「激励」里）的单位随注入类型：现在是 {unit}。默认 {preset['profile']} 幅值 {preset['amp']} {unit}、"
            f"每段 {preset['hold']} ms、{preset['repeat']} 对；选注入类型会写入这组默认值，可再改。")
        if self.yaw_mode_selected():
            self.amp_label_var.set(f"幅值 [{unit}]")
            if hasattr(self, "excitation_box"):
                self.excitation_box.configure(text=YAW_EXCITATION_TITLE)

    # ------------------------------------------------------------ 实时读数

    def _live_yaw(self) -> str:
        if not getattr(self, "samples", None) or not self.yaw_run_active():
            return "实时偏航：没有数据（一轮采集开始后显示最近一个样本的偏航角速度与偏航角）"
        last = self.samples[-1]
        try:
            omega = float(last["vz"])
        except (KeyError, TypeError, ValueError):
            return "实时偏航：记录里没有偏航字段"
        psi = self._live_psi()
        twist = (self.workflow.snapshot or {}).get("yaw_request", {}).get("twist_deg")
        limit = f" / 上限 {twist}°" if twist else ""
        return f"实时偏航：ω = {omega:+.3f} rad/s · ψ = {math.degrees(psi):+.1f}°{limit}"

    def _live_psi(self) -> float:
        """开跑起陀螺 z 积分的偏航角 [rad]：样本数没变就用缓存，变了才重算（一轮最多 15000 条）。"""
        count = len(self.samples)
        if self._yaw_psi_cache[0] == count:
            return self._yaw_psi_cache[1]
        times = self._timestamps()
        psi = yaw_integrate_psi(times, [row.get("vz") for row in self.samples])[-1] if times else 0.0
        self._yaw_psi_cache = (count, psi)
        return psi

    def refresh_yaw_live(self) -> None:
        if hasattr(self, "yaw_live_var"):
            text = self._live_yaw()
            if self.yaw_live_var.get() != text:
                self.yaw_live_var.set(text)

    # ------------------------------------------------------------ 结果区

    def yaw_result(self, text: str) -> None:
        self.yaw_result_var.set(text)

    def yaw_show(self, which: str) -> None:
        """让偏航页切到它的「看波形」/「结果」子页（偏航页没挂上时什么也不做）。"""
        if self.yaw_view is not None:
            self.yaw_view.show(which)


__all__ = ["YAW_EXCITATION_TITLE", "YAW_RESULT_IDLE", "YawSection"]
