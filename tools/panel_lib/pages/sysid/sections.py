"""内环辨识页各分区的布局：准备（油门、台架几何）、结果卡、高级设置。

只摆控件、绑变量；状态和命令都在 `inner_loop.py` / `workflow.py`。
拆出来是为了让页面主文件保持在能一眼读完的长度。
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

from ._core import PROFILE_CODES
from .common import labelled_entry

PROFILE_LABELS = {
    "step": "阶跃（看上升沿与延迟）",
    "doublet": "双脉冲（最常用，激励宽而不积累转角）",
    "chirp": "扫频（分开阻尼与偏心要靠它）",
    "prbs": "伪随机（频谱最平，时间最省）",
}

#: 实验类型 -> (保存用的键, 激励预设)。扫频"其余照旧"：只改列出来的几项。
EXPERIMENTS = {
    # 2026-09-27 固件力矩模型修正（原先高估倾转力矩约 2.3 倍）后，同样幅值舵机要多动约 2.3 倍；
    # 默认幅值 ×0.43（150→65、50→22 mrad/s），保持与 2026-09-27 那两轮相同的舵机实际摆动。
    # 这只是没读到板子参数时的起点：读到后按当前力臂自动换成舵机约 10° 的值（amplitude_hint.py）。
    "双脉冲（默认，8 s）": ("doublet", dict(profile="doublet", amp="0.065", dur="8000",
                                       hold="250", repeat="16", ramp="150")),
    "扫频（确认 1–3 Hz 响应）": ("chirp", dict(profile="chirp", amp="0.022", f0="0.5",
                                        f1="6", dur="12000")),
}
DEFAULT_EXPERIMENT = "双脉冲（默认，8 s）"
EXCITATION_TITLE = "激励编排（给的是期望角速度，不是舵机脉宽）"
_EXPERIMENT_VARS = dict(profile="profile_var", amp="amp_var", dur="dur_var", hold="hold_var",
                        repeat="repeat_var", ramp="ramp_var", f0="f0_var", f1="f1_var")

# Angle is the actual rotation axis in body FLU, not the insertion direction.
RIG_AXIS_PRESETS = {
    "绕 Y 轴 · Pitch（90°）": 90.0,
    "斜向 +45°（前—左）": 45.0,
    "斜向 −45°（前—右）": -45.0,
    "自定义": None,
}


class PageSections:
    def _build_experiment(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="实验类型", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        self.experiment_combo = ttk.Combobox(box, textvariable=self.experiment_var,
                                             values=list(EXPERIMENTS), state="readonly", width=24)
        self.experiment_combo.pack(anchor=tk.W)
        self.experiment_combo.bind("<<ComboboxSelected>>", lambda _e: self.apply_experiment())
        ttk.Label(box, style="Muted.TLabel", wraplength=620, justify=tk.LEFT, text=(
            "建议做一轮扫频，和双脉冲一起联合分析，结果更可信。激励细节在「高级设置」里。")
        ).pack(anchor=tk.W, pady=(4, 0))
        # 与「高级设置」幅值旁同一行灰字：自动按力臂换了幅值时，在这里也看得到。
        ttk.Label(box, textvariable=self.amp_hint_var, style="Muted.TLabel", wraplength=620,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(2, 0))

    def experiment_key(self) -> str:
        return EXPERIMENTS.get(self.experiment_var.get(), EXPERIMENTS[DEFAULT_EXPERIMENT])[0]

    def set_experiment(self, key: str) -> None:
        label = next((name for name, (code, _) in EXPERIMENTS.items() if code == key), None)
        if label is not None:
            self.experiment_var.set(label)
            self.apply_experiment()

    def apply_experiment(self) -> None:
        """把所选实验类型的激励预设写进「高级设置」的激励参数。"""
        _key, preset = EXPERIMENTS.get(self.experiment_var.get(), EXPERIMENTS[DEFAULT_EXPERIMENT])
        if getattr(self, "sub_view_mode_selected", lambda: False)():
            self.store_inner_excitation(preset)      # 高度 / XY 页在前台：记给内环，回内环时还原
            return
        for field, value in preset.items():
            getattr(self, _EXPERIMENT_VARS[field]).set(value)
        self._refresh_preview()

    def _build_servo(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="舵机单独（「本轮做」选 SERVO 时用）", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X)
        labelled_entry(grid, 0, "舵机摆幅 [deg]", self.servo_tilt_var,
                       hint="默认 5°（87 mrad），0.6°～15°；换几个幅值各跑一轮可以看出舵机回差")
        ttk.Label(box, wraplength=620, justify=tk.LEFT, style="Muted.TLabel", text=(
            "电机不转，只让舵机按激励摆：量舵机甩动机体的反作用、舵机响应和延迟。"
            "全程保持上锁；解锁会立即停止。")).pack(anchor=tk.W, pady=(4, 0))

    def _build_throttle(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="油门（由程序控制）", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(box, wraplength=620, justify=tk.LEFT, text=(
            "程序只在你解锁后、点开始时接管油门；推油门杆或上锁立即交还。")).pack(anchor=tk.W)
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X, pady=(6, 0))
        labelled_entry(grid, 0, "目标合推力 [N]", self.target_thrust_var,
                       hint_var=self.thrust_hint_var, label_var=self.thrust_label_var)
        labelled_entry(grid, 1, "最高油门 [%]", self.max_pct_var,
                       hint="程序推油门不会超过这个百分比（10～95）")
        ttk.Checkbutton(box, text="遥控器手动给油门（旧方式）",
                        variable=self.manual_throttle_var).pack(anchor=tk.W, pady=(4, 0))

    def _build_rig(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="台架几何", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        selector = ttk.Frame(box)
        selector.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(selector, text="杆轴方向").pack(side=tk.LEFT)
        self.axis_preset_combo = ttk.Combobox(
            selector, textvariable=self.axis_preset_var, state="readonly",
            values=list(RIG_AXIS_PRESETS), width=27)
        self.axis_preset_combo.pack(side=tk.LEFT, padx=6)
        self.axis_preset_combo.bind("<<ComboboxSelected>>", self._select_axis_preset)
        ttk.Label(box, wraplength=620, justify=tk.LEFT, style="Muted.TLabel", text=(
            "按机体实际绕哪根轴转来选：绕 Y 是 Pitch；杆沿机头前后方向时是 Roll（自定义 0°）。")
        ).pack(anchor=tk.W, pady=(0, 6))
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X)
        labelled_entry(grid, 0, "杆轴方位角 [deg]", self.psi_var,
                       hint="相对机头 +X，向左为正；手填会自动匹配预设")
        labelled_entry(grid, 1, "杆到飞控板的垂直距离 [m]", self.rod_to_fc_var,
                       hint="用尺子量杆中心到飞控板的垂直距离；杆在飞控上方为正（一般 0.1～0.2）")
        pivot_hint = ("不是台架的杆：是舵机带电机组摆动的那根转轴（推力线过它）。尺量；"
                      "在飞控板下方填负数（一般 −0.15～−0.25）。有机体参数时会预填，可改")
        labelled_entry(grid, 2, "俯仰倾转轴到飞控板的垂直距离 [m]", self.pitch_pivot_var,
                       hint=pivot_hint)
        labelled_entry(grid, 3, "横滚倾转轴到飞控板的垂直距离 [m]", self.roll_pivot_var,
                       hint=pivot_hint)
        # 按杆轴方向只露出需要的那项（见 LiveStatus.update_pivot_rows）。
        self.pivot_rows = {"pitch": grid.grid_slaves(row=2), "roll": grid.grid_slaves(row=3)}
        ttk.Label(box, textvariable=self.geometry_hint_var, justify=tk.LEFT,
                  wraplength=620).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(box, textvariable=self.pivot_warning_var, justify=tk.LEFT, wraplength=620,
                  style="Muted.TLabel").pack(anchor=tk.W)
        actions = ttk.Frame(box)
        actions.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(actions, text="下发台架几何", command=self.send_rig).pack(side=tk.LEFT)
        ttk.Button(actions, text="读回飞控状态", command=self.request_status).pack(
            side=tk.LEFT, padx=(6, 0))
        self.rig_status_label = self.mount_status(box)
        ttk.Label(box, text="单位 m：1 cm = 0.01 m。杆到质心距离是分析的输入，一定要量准。"
                  "不必单独下发，点「开始辨识」会自动下发并核对。", style="Muted.TLabel",
                  wraplength=560).pack(anchor=tk.W)

    def _select_axis_preset(self, _event=None) -> None:
        angle = RIG_AXIS_PRESETS[self.axis_preset_var.get()]
        if angle is not None:
            self.psi_var.set(f"{angle:g}")

    def _sync_axis_preset(self, *_args) -> None:
        try:
            angle = float(self.psi_var.get())
        except ValueError:
            angle = float("nan")
        label = next((name for name, value in RIG_AXIS_PRESETS.items()
                      if value is not None and math.isclose(angle, value, abs_tol=1e-6)),
                     "自定义")
        self.axis_preset_var.set(label)
        self.update_pivot_rows()

    def _build_manual_hold(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="手动油门方式（旧）", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(box, text="进入台架待机（舵机回中）",
                   command=lambda: self.send("SYSID HOLD")).pack(anchor=tk.W)
        ttk.Label(box, wraplength=620, justify=tk.LEFT, style="Muted.TLabel", text=(
            "只在勾选「遥控器手动给油门」时需要：低油门解锁 → 点这里 → 推稳油门 → 点「开始辨识」。")
        ).pack(anchor=tk.W, pady=(4, 0))

    def _build_excitation(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text=EXCITATION_TITLE, padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        self.excitation_box = box          # ALT 下标题换成高度激励（alt_section.py）
        top = ttk.Frame(box)
        top.pack(fill=tk.X)
        ttk.Label(top, text="剖面").pack(side=tk.LEFT)
        combo = ttk.Combobox(top, textvariable=self.profile_var, width=10,
                             state="readonly", values=list(PROFILE_CODES))
        combo.pack(side=tk.LEFT, padx=(6, 8))
        combo.bind("<<ComboboxSelected>>", lambda _e: self._refresh_preview())
        self.profile_hint_var = tk.StringVar(value=PROFILE_LABELS["doublet"])
        ttk.Label(top, textvariable=self.profile_hint_var,
                  style="Mono.TLabel").pack(side=tk.LEFT)

        grid = ttk.Frame(box)
        grid.pack(fill=tk.X, pady=(6, 0))
        # 标签随模式变：ALT 下是推力/速度/高度的单位（alt_section.py）。
        labelled_entry(grid, 0, "幅值 [rad/s]", self.amp_var, hint_var=self.amp_hint_var,
                       label_var=getattr(self, "amp_label_var", None))
        labelled_entry(grid, 1, "总时长 [ms]", self.dur_var)
        labelled_entry(grid, 2, "平台/半周期 [ms]", self.hold_var)
        labelled_entry(grid, 3, "重复对数", self.repeat_var, hint="doublet 专用，上限 20")
        labelled_entry(grid, 4, "斜坡 [ms]", self.ramp_var,
                       hint="每次电平过渡的时长；不能填 0")
        labelled_entry(grid, 5, "扫频起/止 [Hz]", self.f0_var)
        labelled_entry(grid, 6, "  终止 [Hz]", self.f1_var)
        labelled_entry(grid, 7, "PRBS 每位 [ms]", self.bit_var)
        labelled_entry(grid, 8, "PRBS 种子", self.seed_var)
        for variable in (self.amp_var, self.dur_var, self.hold_var,
                         self.repeat_var, self.ramp_var, self.f0_var,
                         self.f1_var, self.bit_var, self.seed_var):
            variable.trace_add("write", lambda *_a: self._refresh_preview())

        ttk.Label(box, textvariable=self.command_preview_var, wraplength=660,
                  style="Mono.TLabel").pack(anchor=tk.W, pady=(6, 0))
        ttk.Button(box, text="下发激励参数", command=self.send_excitation).pack(
            anchor=tk.W, pady=(6, 0))

    def _build_limits(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="采样与安全门", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        grid = ttk.Frame(box)
        grid.pack(fill=tk.X)
        labelled_entry(grid, 0, "线上采样率 [Hz]", self.rate_var,
                       hint="USB 默认 250 Hz；无线最多 100 Hz")
        labelled_entry(grid, 1, "假定惯量 [kg·m²]", self.inertia_var,
                       hint="留空取机体参数；过大可能触发执行器饱和并中止")
        labelled_entry(grid, 2, "角度上限 [deg]", self.angle_limit_var,
                       hint="超过就中止本轮（不削顶，削顶的数据会骗人）")
        labelled_entry(grid, 3, "轴向残差上限 [deg/s]", self.resid_limit_var,
                       hint="机体必须只绕杆转；超限说明杆没夹紧或方位角填错")
        labelled_entry(grid, 4, "ANGLE 角度幅值 [deg]", self.angle_amp_var,
                       hint="只用于 ANGLE 验证；0～15°，且小于角度上限")
        labelled_entry(grid, 5, "直接填杆到质心距离 [m]", self.axis_override_var,
                       hint="可选；留空 = 按「准备」页计算。填了就不再用杆到飞控的距离")
        ttk.Button(box, text="下发采样与限位", command=self.send_limits).pack(
            anchor=tk.W, pady=(6, 0))

    def _build_diagnostics(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="诊断", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        row = ttk.Frame(box)
        row.pack(fill=tk.X)
        ttk.Button(row, text="读取采集格式（诊断）", command=self.request_schema).pack(side=tk.LEFT)
        ttk.Button(row, text="清空采集", command=self.clear_samples).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(row, text="丢弃待传数据", command=self.workflow.discard).pack(
            side=tk.LEFT, padx=(6, 0))
        live = ttk.Frame(box)
        live.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(live, textvariable=self.run_state_var, style="Mono.TLabel").pack(side=tk.LEFT)
        ttk.Label(live, textvariable=self.sample_count_var,
                  style="Mono.TLabel").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(box, textvariable=self.schema_var, style="Mono.TLabel",
                  wraplength=660).pack(anchor=tk.W, pady=(6, 0))

    def _build_result(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="本轮结果", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        # 【给 PID 用的】+ 建议参数，然后【对象特性】，最后灰字【诊断】。
        ttk.Label(box, textvariable=self.fit_var, wraplength=660,
                  justify=tk.LEFT).pack(anchor=tk.W)
        ttk.Label(box, textvariable=self.gain_var, wraplength=660,
                  justify=tk.LEFT, style="Mono.TLabel").pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(box, textvariable=self.plant_var, wraplength=660,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(8, 0))
        ttk.Label(box, textvariable=self.fit_ref_var, wraplength=660, justify=tk.LEFT,
                  style="Muted.TLabel").pack(anchor=tk.W, pady=(8, 0))
        actions = ttk.Frame(box)
        actions.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(actions, text="试用比例").pack(side=tk.LEFT)
        ttk.Combobox(actions, textvariable=self.apply_scale_var, values=("50%", "75%", "100%"),
                     state="readonly", width=6).pack(side=tk.LEFT, padx=(4, 6))
        ttk.Button(actions, text="临时应用到飞控（RAM，不存 Flash）",
                   command=self.apply_to_ram).pack(side=tk.LEFT)
        ttk.Button(actions, text="恢复原参数", command=self.workflow.restore).pack(
            side=tk.LEFT, padx=(6, 0))
        ttk.Button(actions, text="重新分析", command=self.run_fit).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(box, wraplength=660, justify=tk.LEFT, style="Muted.TLabel", text=(
            "临时应用只改飞控内存，断电就恢复；首次验证建议按 50% 试用（速率环 kp/ki 与角度环 kp 一起按比例缩小）。"
            "应用后把「本轮做」换成 RATE 再点开始验证；"
            "满意后再到参数页自己决定是否永久保存。")).pack(anchor=tk.W, pady=(6, 0))
