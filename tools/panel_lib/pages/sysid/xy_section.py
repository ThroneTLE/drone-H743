"""水平槽辨识（XY）的设置状态与「水平槽设置」分区：注入类型、出窗余量、附加质量、托住推力、实时读数。

控件摆在「系统辨识 · XY 速度 / 位置环」页（`horizontal.py`），状态和逻辑留在内环页对象里：
固件同一时间只跑一轮 SYSID，几页共用同一个事务（`workflow.py`），不能各开一套。命令组装与回显核对在
`xy_config.XyWorkflow`。机制与 `alt_section.py` 一致：

* **哪一页在前台，「本轮做」就是哪一类**（`alt_section.set_front_view`）：切到 XY 页即 XY，
  切回内环页恢复内环上次选的模式；有事务在等回显或正在跑时不切，跑完再对齐。
* 三边（内环 / Z 高度 / XY）幅值单位不同，激励参数各存一份，切换时还原；第一次进 XY 用所选
  注入类型的默认激励。
* XY 页有自己的托住推力与最高油门，与内环页、Z 高度页互不串值；水平槽辨识总是程序油门。
  托住推力留空 = 用飞控参数 coax.hover_thrust_n；「读取学到的悬停推力」发 `HOVER?`，把 est_n
  **只填进输入框、不写飞控**。
* 实时量 xy_pos_mm / xy_vel_mms / xy_ok 来自 `SYSID THR?` 行末尾（同 Z 页的测距读数）。
* 杆轴方位角 ψ 沿用内环页台架设置，平移方向 u = (−sin ψ, cos ψ)。
"""
from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

from ...proto import parse_kv
from .alt_config import (EXTRA_MASS_RANGE_G, airframe_mass_kg, airframe_max_force_n)
from .common import labelled_entry
from .sections import _EXPERIMENT_VARS
from .xy_config import (AMP_UNITS, DEFAULT_EXTRA_MASS_G, DEFAULT_INJECT, DEFAULT_WIN_MM, INJECTS,
                        INJECT_LABELS, INJECT_PRESETS, WIN_RANGE_MM, XY_MODE, amplitude_limit,
                        direction_text, hover_param_n)

AMP_LABEL_RATE = "幅值 [rad/s]"
XY_EXCITATION_TITLE = "激励编排（水平槽辨识 XY：tilt 是倾角 rad，vel 是速度 m/s，pos 是位置 m）"
XY_RESULT_IDLE = ("跑完一轮水平槽辨识后，这里显示本轮设置、分析结果（tilt：增益/静摩擦门槛角/光流滞后；"
                  "vel、pos：阶跃指标）、数据是否可用与存档说明。")
DEFAULT_XY_MAX_PCT = "90"


class XySection:
    def _init_xy_vars(self) -> None:
        self.xy_inject_var = tk.StringVar(value=DEFAULT_INJECT)
        self.xy_extra_mass_var = tk.StringVar(value=DEFAULT_EXTRA_MASS_G)
        self.xy_win_var = tk.StringVar(value=DEFAULT_WIN_MM)
        # 托住推力 [N]：留空 = 用飞控 coax.hover_thrust_n；「读取学到的悬停推力」只填这里。
        self.xy_target_var = tk.StringVar(value="")
        self.xy_max_pct_var = tk.StringVar(value=DEFAULT_XY_MAX_PCT)
        self.xy_inject_hint_var = tk.StringVar(value=INJECT_LABELS[DEFAULT_INJECT])
        self.xy_mass_hint_var = tk.StringVar(value="")
        self.xy_win_hint_var = tk.StringVar(value="")
        self.xy_amp_note_var = tk.StringVar(value="")
        self.xy_thrust_hint_var = tk.StringVar(value="")
        self.xy_live_var = tk.StringVar(value="")
        self.xy_direction_var = tk.StringVar(value="")
        self.xy_result_var = tk.StringVar(value=XY_RESULT_IDLE)
        self.xy_view = None                 # 「XY 速度 / 位置环」页挂上来后指向它（horizontal.py）
        self._hover_pending = False         # 已发 HOVER?，等它的回复
        self.xy_inject_var.trace_add("write", lambda *_a: self._on_xy_inject())
        for variable in (self.xy_extra_mass_var, self.xy_win_var, self.xy_target_var, self.psi_var):
            variable.trace_add("write", lambda *_a: self.refresh_xy_hint())

    def _build_xy(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="水平槽设置", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        top = ttk.Frame(box)
        top.pack(fill=tk.X)
        ttk.Label(top, text="注入类型").pack(side=tk.LEFT)
        self.xy_inject_combo = ttk.Combobox(top, textvariable=self.xy_inject_var, width=7,
                                            state="readonly", values=INJECTS)
        self.xy_inject_combo.pack(side=tk.LEFT, padx=(6, 8))
        self.xy_inject_combo.bind("<<ComboboxSelected>>", lambda _e: self.select_xy_inject())
        ttk.Label(top, textvariable=self.xy_inject_hint_var, style="Muted.TLabel",
                  wraplength=460, justify=tk.LEFT).pack(side=tk.LEFT)
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X, pady=(6, 0))
        labelled_entry(grid, 0, "出窗余量 [mm]", self.xy_win_var, hint_var=self.xy_win_hint_var)
        labelled_entry(grid, 1, "台架随动附加质量 [g]", self.xy_extra_mass_var,
                       hint_var=self.xy_mass_hint_var)
        ttk.Label(box, textvariable=self.xy_amp_note_var, style="Muted.TLabel", wraplength=620,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(6, 0))
        ttk.Label(box, textvariable=self.xy_direction_var, style="Muted.TLabel", wraplength=620,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))
        live = ttk.Frame(box)
        live.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(live, textvariable=self.xy_live_var, style="Mono.TLabel").pack(side=tk.LEFT)
        ttk.Label(box, wraplength=620, justify=tk.LEFT, text=(
            "电机会转、机体会沿水平槽平移：开跑前确认槽两端有挡、机体在槽中间（距两端至少出窗余量），"
            "光流与测距下方地面平整有纹理。杆装在 ±45° 或平行 Y 轴都行，本页沿用「角速度 / 角度内环」页"
            "的杆轴方位角。解锁、油门杆在最低时点「开始水平槽辨识」。"
            "tilt 直接给沿 u 的倾角（不跑位置/速度环）测对象；vel/pos 用生产位置/速度环（x 通道参数）"
            "沿 u 一维闭环，验证按 tilt 结果整定的参数。")
        ).pack(anchor=tk.W, pady=(4, 0))
        self.refresh_xy_hint()
        self.refresh_xy_live()

    def _build_xy_throttle(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="程序油门（水平槽辨识总是由程序控制）", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X)
        labelled_entry(grid, 0, "托住推力 [N]", self.xy_target_var, hint_var=self.xy_thrust_hint_var)
        labelled_entry(grid, 1, "最高油门 [%]", self.xy_max_pct_var,
                       hint="程序推油门不会超过这个百分比（10～95）；与其它页的设置互不影响")
        self.xy_hover_button = ttk.Button(box, text="读取学到的悬停推力",
                                          command=self.read_hover_thrust)
        self.xy_hover_button.pack(anchor=tk.W, pady=(6, 0))
        ttk.Label(box, wraplength=620, justify=tk.LEFT, style="Muted.TLabel", text=(
            "推力托住机体自重，槽只受很小的法向力。留空 = 用飞控参数 coax.hover_thrust_n；"
            "「读取学到的悬停推力」发 HOVER? 把在线估计值填进上面的框，只填不写飞控。"
            "程序只在你解锁后、点开始时接管油门；推油门杆或上锁立即交还。")
        ).pack(anchor=tk.W, pady=(4, 0))

    # ------------------------------------------------------------ 开始与注入类型

    def start_xy_run(self) -> None:
        """「XY 速度 / 位置环」页的开始按钮：不论之前在哪，都按 XY 开跑。"""
        self._front_view = XY_MODE
        self._apply_view_mode(XY_MODE)
        self.start_run()

    def select_xy_inject(self) -> None:
        """下拉选了注入类型：XY 模式下把它的默认激励写进激励编排；别的模式只更新提示。"""
        if self.xy_mode_selected():
            self.apply_xy_inject()
        self.refresh_xy_hint()

    def apply_xy_inject(self) -> None:
        preset = INJECT_PRESETS.get(self.xy_inject_var.get(), INJECT_PRESETS[DEFAULT_INJECT])
        for field, value in preset.items():
            getattr(self, _EXPERIMENT_VARS[field]).set(value)
        self._refresh_preview()

    def _on_xy_inject(self) -> None:
        self.refresh_xy_hint()
        if hasattr(self, "refresh_amp_hint"):
            self.refresh_amp_hint()        # 幅值旁的单位与上限跟着注入类型变

    def xy_run_active(self) -> bool:
        """正在跑（或刚结束）的是水平槽辨识轮，或还没开始时「本轮做」选了 XY。"""
        from .xy_config import XY_MODE_CODE

        w = self.workflow
        if w.run_id is not None or w.awaiting == "SYSID START":
            return str((w.snapshot or {}).get("mode")) == str(XY_MODE_CODE)
        return self.xy_mode_selected()

    # ------------------------------------------------------------ 提示

    def xy_amp_unit(self) -> str:
        return AMP_UNITS.get(self.xy_inject_var.get(), AMP_UNITS[DEFAULT_INJECT])[0]

    def _xy_win_mm(self) -> int | None:
        try:
            value = float(self.xy_win_var.get())
        except ValueError:
            return None
        return int(value) if WIN_RANGE_MM[0] <= value <= WIN_RANGE_MM[1] else None

    def xy_amp_hint(self) -> str:
        """XY 下「幅值」旁的灰字：单位与上限随注入类型；不估算舵机摆幅、不自动改幅值。"""
        inject = self.xy_inject_var.get()
        unit, limit = amplitude_limit(inject, self._xy_win_mm())
        extra = "（pos 还不能超过 0.7×出窗余量）" if inject == "pos" else ""
        return (f"水平槽辨识（XY · {inject}）：幅值单位 {unit}，上限 {limit:g} {unit}{extra}；"
                "不估算舵机摆幅、不自动改幅值")

    def refresh_xy_hint(self) -> None:
        if not hasattr(self, "xy_mass_hint_var") or not hasattr(self, "workflow"):
            return
        inject = self.xy_inject_var.get()
        unit, limit = AMP_UNITS.get(inject, AMP_UNITS[DEFAULT_INJECT])
        preset = INJECT_PRESETS.get(inject, INJECT_PRESETS[DEFAULT_INJECT])
        self.xy_inject_hint_var.set(INJECT_LABELS.get(inject, ""))
        win = self._xy_win_mm()
        limit_text = f"上限 {limit:g} {unit}"
        if inject == "tilt":
            limit_text += f"（≈ {math.degrees(limit):.1f}°）"
        elif inject == "pos" and win is not None:
            limit_text += f"，且不超过 0.7×出窗余量 {0.7 * win * 0.001:g} m"
        self.xy_amp_note_var.set(
            f"激励幅值（「激励」里）的单位随注入类型：现在是 {unit}，{limit_text}。"
            f"默认 {preset['profile']} 幅值 {preset['amp']} {unit}、每段 {preset['hold']} ms、"
            f"{preset['repeat']} 对；选注入类型会写入这组默认值，可再改。")
        self.xy_win_hint_var.set(
            f"|沿 u 位移| 超过它就先软停再中止（{WIN_RANGE_MM[0]}～{WIN_RANGE_MM[1]} mm）；"
            "要小于机体到槽端的余地")
        self.xy_mass_hint_var.set(self._xy_mass_text())
        self.xy_thrust_hint_var.set(self._xy_thrust_text())
        try:
            self.xy_direction_var.set(direction_text(float(self.psi_var.get())))
        except ValueError:
            self.xy_direction_var.set("杆轴方位角填的不是数字：到「角速度 / 角度内环」页「准备」检查。")
        if self.xy_mode_selected():
            self.amp_label_var.set(f"幅值 [{unit}]")
            if hasattr(self, "excitation_box"):
                self.excitation_box.configure(text=XY_EXCITATION_TITLE)

    def _xy_mass_text(self) -> str:
        low, high = EXTRA_MASS_RANGE_G
        try:
            extra = float(self.xy_extra_mass_var.get())
            if not low <= extra <= high:
                raise ValueError
        except ValueError:
            return f"要填 {low:g}～{high:g} 的数字（单位 g，默认 {DEFAULT_EXTRA_MASS_G}：随机体沿槽平移的碳杆）"
        w = self.workflow
        mass = airframe_mass_kg(w.params, w.ready)
        if mass is None:
            return ("合计 = 机体质量 + 附加质量；机体质量还没读到（连上飞控后自动读取，"
                    "开跑时仍读不到就不开跑）")
        total = mass * 1000.0 + extra
        return (f"合计 {total:.1f} g = 机体 {mass * 1000.0:.1f} g（飞控机体参数）+ 附加 {extra:g} g；"
                f"下发 mass_g={int(round(total))}")

    def _xy_thrust_text(self) -> str:
        limit = airframe_max_force_n(self.workflow.params)
        cap = f"（≤ 整机最大推力 {limit:.1f} N）" if limit is not None else ""
        text = self.xy_target_var.get().strip()
        if text:
            return f"将使用 {text} N{cap}"
        found = hover_param_n(self.workflow.params)
        return (f"留空 = 飞控 coax.hover_thrust_n = {found:.2f} N{cap}" if found is not None else
                "留空 = 飞控 coax.hover_thrust_n（连上飞控后读取；读不到或为 0 就必须手填）")

    # ------------------------------------------------------------ 实时读数

    def _live_xy(self) -> tuple[dict | None, str]:
        """`(最近一次新鲜的 THR 回报 或 None, 显示用的一句)`。"""
        thr = self._current_thr() if hasattr(self, "_current_thr") else None
        if thr is None:
            return None, "水平槽实时读数：没有新数据（连上飞控、本页在前台时每秒刷新）"
        if "xy_pos_mm" not in thr:
            return None, "水平槽实时读数：固件没报（需要回报 xy_pos_mm 的新固件）"
        try:
            pos, vel = int(round(float(thr["xy_pos_mm"]))), int(round(float(thr.get("xy_vel_mms", 0))))
        except (TypeError, ValueError):
            return None, "水平槽实时读数：格式异常"
        # 偏航（本轮或最近一轮的陀螺积分，开跑清零；超 15° 软停）：旧固件不报就不显示。
        try:
            yaw = f" · 偏航 {float(thr['xy_yaw_mrad']) * 0.0572958:+.1f}°" if "xy_yaw_mrad" in thr else ""
        except (TypeError, ValueError):
            yaw = ""
        if str(thr.get("xy_ok", "0")).strip() != "1":
            return thr, (f"水平槽实时读数：无效（光流/测距无效或不新鲜；上次 沿 u 位置 {pos} mm、"
                         f"速度 {vel} mm/s）{yaw}")
        return thr, f"水平槽实时读数：沿 u 位置 {pos} mm · 沿 u 速度 {vel} mm/s · 光流有效{yaw}"

    def refresh_xy_live(self) -> None:
        if hasattr(self, "xy_live_var"):
            text = self._live_xy()[1]
            if self.xy_live_var.get() != text:
                self.xy_live_var.set(text)

    # ------------------------------------------------------------ 读取学到的悬停推力

    def read_hover_thrust(self) -> None:
        """发 `HOVER?`，回复到了把 est_n 填进「托住推力」；不写飞控。"""
        if self.send("HOVER?"):
            self._hover_pending = True
            self.status_var.set("已请求悬停推力估计（HOVER?），等待飞控回复…")

    def xy_handle_hover(self, text: str) -> None:
        """`HOVER est_n=.. std_n=.. converged=..` → 填托住推力；`HOVER state=not_ready` 只提示。"""
        if not self._hover_pending or not text.startswith("HOVER "):
            return
        values = parse_kv(text)
        if "reset" in values:
            return
        self._hover_pending = False
        if values.get("state") == "not_ready":
            self.status_var.set("飞控的悬停推力估计器还没就绪（需要先飞/托起一段时间），没有填入。")
            return
        try:
            est = float(values["est_n"])
        except (KeyError, ValueError):
            est = float("nan")
        limit = airframe_max_force_n(self.workflow.params)
        converged = str(values.get("converged", "0")) == "1"
        if not converged:
            # 2026-09-30 晚：估计器在台架托着时学成 8.66 N（converged=0、拒收 549 次），页面照填，
            # 首轮 tilt 推力比浮起点少 5.6 N、横杆压死在下碳杆上推不动。未收敛时改用 Flash 里的参数。
            try:
                init = float(values["init_n"])
            except (KeyError, ValueError):
                init = float("nan")
            if math.isfinite(init) and init > 0.0 and (limit is None or init <= limit):
                self.xy_target_var.set(f"{init:.2f}")
                self.status_var.set(
                    f"悬停推力估计尚未收敛（est_n={values.get('est_n', '?')}，拒收 {values.get('rejected', '?')} 次），"
                    f"不可信；已改填飞控里存的悬停推力参数 {init:.2f} N。只在页面里，没有写飞控。")
            else:
                self.status_var.set(
                    f"悬停推力估计尚未收敛（est_n={values.get('est_n', '?')}），飞控里也没有悬停推力参数，"
                    "没有填入：请手填竖直槽实测的浮起推力。")
            return
        if not (math.isfinite(est) and est > 0.0) or (limit is not None and est > limit):
            self.status_var.set(f"悬停推力估计值无效（est_n={values.get('est_n', '缺')}），没有填入。")
            return
        self.xy_target_var.set(f"{est:.2f}")
        std = values.get("std_n", "?")
        self.status_var.set(
            f"已把学到的悬停推力 {est:.2f} N（标准差 {std} N，已收敛）填进托住推力；只在页面里，没有写飞控。")

    # ------------------------------------------------------------ 结果区

    def xy_result(self, text: str) -> None:
        self.xy_result_var.set(text)

    def xy_show(self, which: str) -> None:
        """让 XY 页切到它的「看波形」/「结果」子页（XY 页没挂上时什么也不做）。"""
        if self.xy_view is not None:
            self.xy_view.show(which)


__all__ = ["AMP_LABEL_RATE", "DEFAULT_XY_MAX_PCT", "XY_EXCITATION_TITLE", "XY_RESULT_IDLE", "XySection"]
