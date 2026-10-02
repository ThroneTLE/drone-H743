"""高度辨识（ALT）的设置状态与「高度设置」分区：注入类型、附加质量、槽底/槽顶测距读数、出窗余量、
抬升高度。

控件摆在「系统辨识 · Z 高度」页（`altitude.py`），状态和逻辑留在内环页对象里：固件同一时间
只跑一轮 SYSID，两页共用同一个事务（`workflow.py`），不能各开一套。命令组装与回显核对在
`alt_config.AltWorkflow`。

* **哪一页在前台，「本轮做」就是哪一类**：切到「Z 高度」页即 ALT，切回内环页恢复内环上次选的
  模式（`set_alt_view_active`）。有事务在等回显或正在跑时不切，跑完再对齐。
* 两边幅值单位不同（N、m/s、m 对 rad/s），激励参数不能沿用：切换时各自存一份、回来时还原；
  第一次进 ALT 用所选注入类型的默认激励，第一次回内环用「实验类型」的预设。
* 高度页有自己的「合推力（break 是离地搜索上限，必填）」与「最高油门」，与内环页的程序油门互不
  串值；高度辨识总是程序油门，内环页的「遥控器手动给油门」对它不起作用。
* 合计质量 = 读回的机体质量 + 附加质量；读不到机体质量时开跑会被拒绝（`alt_config`）。
* 槽底/槽顶是测距（TOF）的绝对读数：机体压到位时点「记为槽底」「记为槽顶」，取 `SYSID THR`
  回报里的 8 样本平均高度（alt_h_mm，要新鲜且 alt_h_ok=1）；也可手填。
"""
from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from .alt_config import (ALT_MODE, AMP_UNITS, DEFAULT_EXTRA_MASS_G,
                         airframe_max_force_n, break_target_limit,
                         DEFAULT_INJECT, DEFAULT_LIFT_MM, DEFAULT_WIN_MM, EXTRA_MASS_RANGE_G,
                         GRAVITY_M_S2, INJECTS, INJECT_LABELS, INJECT_PRESETS, LIFT_RANGE_MM,
                         TRAVEL_RANGE_MM, WIN_RANGE_MM, airframe_mass_kg, upper_window_mm)
from .common import labelled_entry
from .xy_config import XY_MODE
from .yaw_config import YAW_MODE
# _EXPERIMENT_VARS：预设键 -> 激励编排变量名，与实验类型预设共用一份。
from .sections import _EXPERIMENT_VARS, EXCITATION_TITLE

AMP_LABEL_RATE = "幅值 [rad/s]"
AMP_LABEL_BREAK = "速率 [N/s]"
BREAK_AMP_HINT = "慢升/慢降速率，默认 0.5 N/s，上限 1 N/s；剖面不用"
ALT_EXCITATION_TITLE = "激励编排（高度辨识 ALT：break 是慢升/慢降速率，vel/pos 是速度 / 高度）"
ALT_RESULT_IDLE = "跑完一轮高度辨识后，这里显示本轮设置、离地/滑落阈值（break）、数据是否可用与存档说明。"
DEFAULT_ALT_MAX_PCT = "90"


class AltSection:
    def _init_alt_vars(self) -> None:
        self.alt_inject_var = tk.StringVar(value=DEFAULT_INJECT)
        self.alt_extra_mass_var = tk.StringVar(value=DEFAULT_EXTRA_MASS_G)
        self.alt_win_var = tk.StringVar(value=DEFAULT_WIN_MM)
        self.alt_lift_var = tk.StringVar(value=DEFAULT_LIFT_MM)
        # 槽底/槽顶的测距绝对读数（空 = 还没量，开跑前必须填）。
        self.alt_bottom_var = tk.StringVar(value="")
        self.alt_top_var = tk.StringVar(value="")
        self.alt_window_hint_var = tk.StringVar(value="")
        self.alt_live_height_var = tk.StringVar(value="")
        self.alt_lift_label_var = tk.StringVar(value="抬升高度 [mm]")
        self.alt_lift_hint_var = tk.StringVar(value="")
        self.alt_inject_hint_var = tk.StringVar(value=INJECT_LABELS[DEFAULT_INJECT])
        self.alt_mass_hint_var = tk.StringVar(value="")
        self.alt_amp_note_var = tk.StringVar(value="")
        self.amp_label_var = tk.StringVar(value=AMP_LABEL_RATE)
        # 高度页自己的程序油门（break 时合推力是离地搜索上限，不是机重）。
        self.alt_target_var = tk.StringVar(value="")
        self.alt_max_pct_var = tk.StringVar(value=DEFAULT_ALT_MAX_PCT)
        self.alt_thrust_label_var = tk.StringVar(value="离地搜索上限 [N]")
        self.alt_thrust_hint_var = tk.StringVar(value="")
        self.alt_result_var = tk.StringVar(value=ALT_RESULT_IDLE)
        self.alt_view = None                # 「Z 高度」页挂上来后指向它（altitude.py）
        self._front_view = None             # 前台的是哪一页：None = 内环页，"ALT" / "XY" = 对应子页
        self._inner_mode = "FF"             # 内环页上次选的模式，从高度 / XY 页切回时还原
        # ALT / XY / 内环各一份激励参数（三边幅值单位不同，不能串）。
        self._saved_excitation = {ALT_MODE: None, XY_MODE: None, YAW_MODE: None, "INNER": None}
        self._alt_mode_prev = self.mode_var.get()
        self.mode_var.trace_add("write", lambda *_a: self._on_alt_mode())
        self.alt_inject_var.trace_add("write", lambda *_a: self._on_alt_inject())
        for variable in (self.alt_extra_mass_var, self.alt_target_var, self.alt_bottom_var,
                         self.alt_top_var, self.alt_win_var, self.alt_lift_var):
            variable.trace_add("write", lambda *_a: self.refresh_alt_hint())

    def _build_alt(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="高度设置", padding=8)
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
        labelled_entry(grid, 1, "槽底测距读数 [mm]", self.alt_bottom_var,
                       hint="机体压在槽底时点下面「记为槽底」（测距的绝对读数，不是离地高度）")
        labelled_entry(grid, 2, "槽顶测距读数 [mm]", self.alt_top_var,
                       hint=f"机体顶到槽顶止挡时点「记为槽顶」；行程要 {TRAVEL_RANGE_MM[0]}～"
                            f"{TRAVEL_RANGE_MM[1]} mm")
        labelled_entry(grid, 3, "出窗余量 [mm]", self.alt_win_var,
                       hint=f"vel/pos：高度超过 min(槽底 + 抬升 + 余量, 槽顶 − 40 mm) 就先慢降再中止；break 用槽顶 − 10 mm；"
                            f"{WIN_RANGE_MM[0]}～{WIN_RANGE_MM[1]} mm")
        labelled_entry(grid, 4, "抬升高度 [mm]", self.alt_lift_var,
                       label_var=self.alt_lift_label_var, hint_var=self.alt_lift_hint_var)
        live = ttk.Frame(box)
        live.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(live, textvariable=self.alt_live_height_var, style="Mono.TLabel").pack(side=tk.LEFT)
        self.alt_bottom_button = ttk.Button(live, text="记为槽底",
                                            command=lambda: self.record_alt_slot("bottom"))
        self.alt_bottom_button.pack(side=tk.LEFT, padx=(8, 0))
        self.alt_top_button = ttk.Button(live, text="记为槽顶",
                                         command=lambda: self.record_alt_slot("top"))
        self.alt_top_button.pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(box, textvariable=self.alt_window_hint_var, style="Muted.TLabel", wraplength=620,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(box, textvariable=self.alt_amp_note_var, style="Muted.TLabel", wraplength=620,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(box, wraplength=620, justify=tk.LEFT, text=(
            "电机会转、机体会沿槽上下移动：开跑前确认槽的上下限位牢靠、测距（TOF）下方地面平整无遮挡，"
            "机体压在槽底（离槽底读数 20 mm 以内）。解锁、油门杆在最低时点「开始高度辨识」。"
            "break 慢升推力找离地、制停，再慢降找滑落，最后滑回槽底；不经高度 PID，杆轴姿态环保持，"
            "高度只用于判离地/滑落和越界保护。跑完页面给离地/滑落推力、推力表比例与槽摩擦。"
            "vel/pos 仍使用生产高度 PID，用来验证按模型设计的候选高度环，分析离线进行。")
        ).pack(anchor=tk.W, pady=(4, 0))
        self.refresh_alt_hint()
        self.refresh_alt_height()

    def _build_alt_throttle(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="程序油门（高度辨识总是由程序控制）", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X)
        labelled_entry(grid, 0, "", self.alt_target_var, label_var=self.alt_thrust_label_var,
                       hint_var=self.alt_thrust_hint_var)
        labelled_entry(grid, 1, "最高油门 [%]", self.alt_max_pct_var,
                       hint="程序推油门不会超过这个百分比（10～95）；与内环页的设置互不影响")
        ttk.Label(box, wraplength=620, justify=tk.LEFT, style="Muted.TLabel", text=(
            "程序只在你解锁后、点开始时接管油门；推油门杆或上锁立即交还。"
            "内环页的「遥控器手动给油门」对高度辨识不起作用。")).pack(anchor=tk.W, pady=(4, 0))

    # ------------------------------------------------------------ 哪一页在前台

    def set_alt_view_active(self, active: bool) -> None:
        """「Z 高度」页到前台 = 本轮做 ALT；回到内环页还原内环上次选的模式。"""
        self.set_front_view(ALT_MODE if active else None)

    def set_front_view(self, view: str | None) -> None:
        """前台是哪一页：`"ALT"`（Z 高度）/ `"XY"`（XY 速度 / 位置环）/ `"YAW"`（偏航（吊绳））/ None（内环页）。
        切到哪一页，「本轮做」就是哪一类。"""
        self._front_view = view if view in (ALT_MODE, XY_MODE, YAW_MODE) else None
        self.sync_mode_to_view()

    def sync_mode_to_view(self) -> None:
        """按前台那一页对齐「本轮做」；没挂子页（单独测内环对象）时模式由调用方直接设。"""
        if (self.alt_view is not None or getattr(self, "xy_view", None) is not None
                or getattr(self, "yaw_view", None) is not None):
            self._apply_view_mode(self._front_view)

    def _apply_view_mode(self, view: str | None) -> None:
        """有事务在等回显或正在跑时不切（事务已按开跑那一刻的模式锁定），跑完再对齐。"""
        w = self.workflow
        if w.awaiting or (w.run_id is not None and w.end is None):
            return
        mode = self.mode_var.get()
        if view in (ALT_MODE, XY_MODE, YAW_MODE):
            if mode != view:
                self.mode_var.set(view)
        elif mode in (ALT_MODE, XY_MODE, YAW_MODE):
            self.mode_var.set(self._inner_mode)

    def start_alt_run(self) -> None:
        """「Z 高度」页的开始按钮：不论之前在哪，都按 ALT 开跑。"""
        self._front_view = ALT_MODE
        self._apply_view_mode(ALT_MODE)
        self.start_run()

    def start_inner_run(self) -> None:
        """内环页的开始按钮：本轮做绝不会是 ALT / XY（它们只从各自的子页开）。"""
        self._front_view = None
        self._apply_view_mode(None)
        self.start_run()

    # ------------------------------------------------------------ 注入类型与默认激励

    def alt_mode_selected(self) -> bool:
        return self.mode_var.get() == ALT_MODE

    def xy_mode_selected(self) -> bool:
        return self.mode_var.get() == XY_MODE

    def sub_view_mode_selected(self) -> bool:
        """「本轮做」是 ALT、XY 或 YAW：激励参数属于子页，不是内环页的。"""
        return self.mode_var.get() in (ALT_MODE, XY_MODE, YAW_MODE)

    def select_alt_inject(self) -> None:
        """下拉选了注入类型：ALT 模式下把它的默认激励写进激励编排；别的模式只更新提示。"""
        if self.alt_mode_selected():
            self.apply_alt_inject()
        self.refresh_alt_hint()

    def apply_alt_inject(self) -> None:
        """把当前注入类型的默认激励写进激励参数。"""
        preset = INJECT_PRESETS.get(self.alt_inject_var.get(), INJECT_PRESETS[DEFAULT_INJECT])
        for field, value in preset.items():
            getattr(self, _EXPERIMENT_VARS[field]).set(value)
        self._refresh_preview()

    def _excitation_values(self) -> dict:
        return {field: getattr(self, name).get() for field, name in _EXPERIMENT_VARS.items()}

    def _restore_excitation(self, values: dict) -> None:
        for field, value in values.items():
            getattr(self, _EXPERIMENT_VARS[field]).set(value)
        self._refresh_preview()

    def store_inner_excitation(self, preset: dict) -> None:
        """ALT 在前台时选「实验类型」（读设置文件时会这样）：只记给内环，不动当前的高度激励。"""
        base = self._saved_excitation["INNER"] or self._excitation_values()
        self._saved_excitation["INNER"] = {**base, **preset}

    def _on_alt_inject(self) -> None:
        self.refresh_alt_hint()
        if hasattr(self, "refresh_amp_hint"):
            self.refresh_amp_hint()        # 幅值旁的单位与上限跟着注入类型变

    def _on_alt_mode(self) -> None:
        mode, previous = self.mode_var.get(), self._alt_mode_prev
        self._alt_mode_prev = mode
        if mode not in (ALT_MODE, XY_MODE, YAW_MODE):
            self._inner_mode = mode
        if hasattr(self, "profile_hint_var"):          # 界面搭好之后才动激励编排
            leaving, entering = self._excitation_group(previous), self._excitation_group(mode)
            if leaving != entering:
                self._saved_excitation[leaving] = self._excitation_values()
                saved = self._saved_excitation[entering]
                if saved is not None:
                    self._restore_excitation(saved)
                elif entering == ALT_MODE:
                    self.apply_alt_inject()
                elif entering == XY_MODE:
                    self.apply_xy_inject()
                elif entering == YAW_MODE:
                    self.apply_yaw_inject()
                else:
                    self.apply_experiment()
        self.refresh_alt_hint()
        if hasattr(self, "refresh_xy_hint"):
            self.refresh_xy_hint()
        if hasattr(self, "refresh_yaw_hint"):
            self.refresh_yaw_hint()

    @staticmethod
    def _excitation_group(mode: str) -> str:
        return mode if mode in (ALT_MODE, XY_MODE, YAW_MODE) else "INNER"

    # ------------------------------------------------------------ 提示

    def alt_amp_unit(self) -> str:
        return AMP_UNITS.get(self.alt_inject_var.get(), AMP_UNITS[DEFAULT_INJECT])[0]

    def alt_amp_hint(self) -> str:
        """ALT 下「幅值」旁的灰字：单位与上限随注入类型；不估算舵机摆幅、不自动改幅值。"""
        inject = self.alt_inject_var.get()
        if inject == "break":
            return BREAK_AMP_HINT
        unit, limit = AMP_UNITS.get(inject, AMP_UNITS[DEFAULT_INJECT])
        return (f"高度辨识（ALT · {inject}）：幅值单位 {unit}，上限 {limit:g} {unit}；"
                "不估算舵机摆幅、不自动改幅值")

    def refresh_alt_hint(self) -> None:
        if not hasattr(self, "alt_mass_hint_var") or not hasattr(self, "workflow"):
            return
        inject = self.alt_inject_var.get()
        brk = inject == "break"
        self.alt_thrust_label_var.set("离地搜索上限 [N]" if brk else "目标合推力 [N]")
        text = self.alt_target_var.get().strip()
        span = self._alt_target_span()
        self.alt_thrust_hint_var.set(
            (f"break：慢升找离地，最多升到 {text} N" + (f"（要在 {span} 之间）" if span else "")
             if text else "break 必填：慢升找离地的推力上限" + (f"，填 {span}" if span else "")
             + "；不会自动用机重") if brk else
            (f"将使用 {text} N" if text else "留空则用机重（飞控机体参数）"))
        self.alt_lift_label_var.set("名义上升区间 [mm]" if brk else "抬升高度 [mm]")
        self.alt_lift_hint_var.set(
            f"break 不追踪这个高度，只与出窗余量一起定上端硬窗；"
            f"{LIFT_RANGE_MM[0]}～{LIFT_RANGE_MM[1]} mm" if brk else
            f"vel/pos 用现有高度环从槽底抬升到槽底 + 该高度再激励；"
            f"{LIFT_RANGE_MM[0]}～{LIFT_RANGE_MM[1]} mm")
        self.alt_inject_hint_var.set(INJECT_LABELS.get(inject, ""))
        unit, limit = AMP_UNITS.get(inject, AMP_UNITS[DEFAULT_INJECT])
        preset = INJECT_PRESETS.get(inject, INJECT_PRESETS[DEFAULT_INJECT])
        self.alt_amp_note_var.set(
            f"break 的「速率」是慢升/慢降推力的速度：默认 {preset['amp']} {unit}，上限 {limit:g} {unit}；"
            "剖面、平台、次数都不用。选注入类型会写入默认值，可再改。" if brk else
            f"激励幅值（下方「激励」）的单位随注入类型：现在是 {unit}，上限 {limit:g} {unit}。"
            f"默认 {preset['profile']} 幅值 {preset['amp']} {unit}、每段 {preset['hold']} ms、"
            f"{preset['repeat']} 对；选注入类型会写入这组默认值，可再改。")
        alt = self.alt_mode_selected()
        if not (self.xy_mode_selected() or self.yaw_mode_selected()):   # XY / YAW 在前台时标签与标题归各自的 section
            self.amp_label_var.set((AMP_LABEL_BREAK if brk else f"幅值 [{unit}]") if alt
                                   else AMP_LABEL_RATE)
            if hasattr(self, "excitation_box"):
                self.excitation_box.configure(text=ALT_EXCITATION_TITLE if alt else EXCITATION_TITLE)
        self.alt_mass_hint_var.set(self._alt_mass_text())
        self.alt_window_hint_var.set(self._alt_window_text())

    def _alt_run_mass_g(self) -> float | None:
        """机体质量 + 附加质量 [g]；读不到机体质量或附加质量填错时 None。"""
        try:
            extra = float(self.alt_extra_mass_var.get())
        except ValueError:
            return None
        mass = airframe_mass_kg(self.workflow.params, self.workflow.ready)
        if mass is None or not EXTRA_MASS_RANGE_G[0] <= extra <= EXTRA_MASS_RANGE_G[1]:
            return None
        return mass * 1000.0 + extra

    def _alt_target_span(self) -> str:
        """break 离地搜索上限的可填范围（本轮移动质量的重力 ～ min(重力 + 5 N, 整机最大推力)）；
        质量未知时空串。"""
        grams = self._alt_run_mass_g()
        if grams is None:
            return ""
        weight = round(grams) * 0.001 * GRAVITY_M_S2
        high = break_target_limit(round(grams), airframe_max_force_n(self.workflow.params))
        return f"{weight:.2f}～{high:.2f} N"

    def _alt_window_text(self) -> str:
        """槽行程与固件会用的上端/下端硬窗（按当前填写的数算）。"""
        try:
            bottom, top, lift, win = (int(float(v.get())) for v in (
                self.alt_bottom_var, self.alt_top_var, self.alt_lift_var, self.alt_win_var))
        except ValueError:
            return "填好槽底、槽顶读数后，这里显示槽行程和高度越界中止的上下界。"
        travel = top - bottom
        if not TRAVEL_RANGE_MM[0] <= travel <= TRAVEL_RANGE_MM[1]:
            return (f"槽行程 {travel} mm 不在 {TRAVEL_RANGE_MM[0]}～{TRAVEL_RANGE_MM[1]} mm："
                    "核对两个读数（槽顶读数要比槽底大）。")
        upper = upper_window_mm(bottom, top, lift, win, self.alt_inject_var.get())
        return (f"槽行程 {travel} mm；测距高于 {upper} mm（槽底以上 {upper - bottom} mm）或低于 "
                f"{bottom - 30} mm 就先慢降回槽底再中止；开跑时机体要在槽底读数 ±20 mm 内。")

    # ------------------------------------------------------------ 实时测距与记为槽底/槽顶

    def _live_height_mm(self) -> tuple[int | None, str]:
        """`(当前测距 mm 或 None, 显示用的一句)`：取最近一次新鲜的 `SYSID THR` 回报。"""
        thr = self._current_thr() if hasattr(self, "_current_thr") else None
        if thr is None:
            return None, "测距当前读数：没有新数据（连上飞控、本页在前台时每秒刷新）"
        if "alt_h_mm" not in thr:
            return None, "测距当前读数：固件没报（需要回报 alt_h_mm 的新固件）"
        if str(thr.get("alt_h_ok", "0")).strip() != "1":
            return None, "测距当前读数：无效（检查测距模块有没有被遮挡、下方地面是否在量程内）"
        try:
            value = int(round(float(thr["alt_h_mm"])))
        except (TypeError, ValueError):
            return None, "测距当前读数：格式异常"
        if value <= 0:
            return None, "测距当前读数：无效（读数不是正数）"
        return value, f"测距当前读数：{value} mm（8 样本平均）"

    def refresh_alt_height(self) -> None:
        if hasattr(self, "alt_live_height_var"):
            text = self._live_height_mm()[1]
            if self.alt_live_height_var.get() != text:
                self.alt_live_height_var.set(text)

    def record_alt_slot(self, which: str) -> None:
        """「记为槽底 / 记为槽顶」：把当前新鲜有效的测距读数填进对应输入框（只改页面）。"""
        name = "槽底" if which == "bottom" else "槽顶"
        value, text = self._live_height_mm()
        if value is None:
            self.status_var.set(f"没有记为{name}：{text.split('：', 1)[-1]}")
            return
        (self.alt_bottom_var if which == "bottom" else self.alt_top_var).set(str(value))
        self.status_var.set(f"已把当前测距 {value} mm 记为{name}（开跑时随 SYSID ALT 下发）")

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

    # ------------------------------------------------------------ 结果区与标签页

    def alt_result(self, text: str) -> None:
        self.alt_result_var.set(text)

    def alt_show(self, which: str) -> None:
        """让「Z 高度」页切到它的「看波形」/「结果」子页（高度页没挂上时什么也不做）。"""
        if self.alt_view is not None:
            self.alt_view.show(which)


__all__ = ["ALT_RESULT_IDLE", "AMP_LABEL_RATE", "AltSection", "DEFAULT_ALT_MAX_PCT"]
