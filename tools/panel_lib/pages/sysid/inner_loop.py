"""「系统辨识 · 角速度/角度内环」页。

作者要的用法和推力台一样：**解锁 → 点开始 → 等结果**。

* 飞手只负责用遥控器解锁（油门杆在最低）。点「开始辨识」后，页面自动读取采集格式、
  下发并逐项核对全部配置（含程序油门），然后开始；固件自己升油门、稳定、激励、降油门。
  推油门杆、上锁或点「停止」都会立即把油门交还遥控器。
* 顶部大字状态每秒刷新（后台 `SYSID THR?` 轮询，只回一行），告诉你现在该做什么。
* 杆到质心距离 d 是你量的输入：杆到飞控（尺量）+ 飞控到质心（机体参数），见 `geometry.py`。
* FF 轮正常结束 → 自动存档 → 自动拟合 → 质量够就自动算建议参数；写 RAM 仍要你点。
  RATE/ANGLE 轮结束给出跟踪误差。中止的轮次把原因翻成中文并给出下一步。
* 「舵机单独」（SERVO）：电机不转、全程保持上锁，量舵机甩动机体的反作用、舵机响应与延迟；
  只给对象特性，不给参数。挂砝码试验（台架刚度）填了就代替 m·g·d 参与两种拟合。
* 候选参数是录制那轮固件的力矩单位；「临时应用」前重读飞控机体参数，按当前倾转力臂换算
  （`lever_units.py`）。记录里有真实电调转速时，另报陀螺振动主频与 eRPM→振动比。
* 激励幅值旁灰字写按当前板子力臂估算的舵机摆幅；幅值还是预设值时自动换成让舵机摆约 10° 的值
  （`amplitude_hint.py`，手填的不动；舵机回差让 5～6° 以下的轮次失真）。
* 「高级设置」里的「陷波（桨叶振动）」看状态、开/关固件的转速陷波（`notch_panel.py`，只改 RAM）；
  每轮开跑时的陷波配置随 conditions.json 存档，并写在结果抬头。「舵机回差补偿」同样
  （`backlash_panel.py`，conditions 的 backlash）。
* 「高度」（ALT，槽式台架）：电机会转、机体沿槽上下移动；设置在「准备」页「高度」分区
  （`alt_section.py`），开跑前多下发并核对一行 `SYSID ALT`（`alt_config.py`）。只存档、画高度，
  页面不拟合，分析离线进行。
* 本页没有写 Flash 的路径；是否永久保存由作者验证后在参数页单独决定。

发的是**期望角速度**而不是舵机脉宽，由固件用在飞的那个分配器反解成倾角。
"""

from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk

from ...board_line_hooks import register_board_line_hook
from ...proto import parse_kv
from .alt_config import ALT_MODE_CODE, OFFLINE_NOTE, provenance_text as alt_provenance_text
from .alt_section import AltSection
from .amplitude_hint import AmplitudeHint
from .common import SysIdPageBase
from .stiffness_panel import StiffnessPanel
from .live_status import LiveStatus, STOP_HINT_AUTO, STOP_TEXT_AUTO
from .reasons import cannot_analyse, explain_end
from .joint import JointPanel
from .backlash_panel import BacklashPanel, backlash_provenance_text
from .notch_panel import NotchPanel, notch_provenance_text
from .results import (advice_for, card_blocks, fit_passes, gains_card, quality_problems,
                      tracking_summary, vibration_summary)
from . import settings_store
from .lever_units import describe as describe_units
from .sections import DEFAULT_EXPERIMENT, PROFILE_LABELS, PageSections
from .waveform import WaveformView
from .workflow import Workflow

from ._core import (  # noqa: E402  本地薄封装，见该文件头
    Excitation, PROFILE_CODES,
    SchemaMismatch, decode_batch, parse_schema_lines,
)

MODE_HINTS = {
    "FF": "测模型：算出惯量、延迟和建议参数（第一次先跑这个）",
    "RATE": "验证角速度跟踪（先把建议参数临时应用到飞控）",
    "ANGLE": "验证角度跟踪（幅值在「高级设置」，从 3° 开始）",
    "SERVO": "舵机单独（电机不转，需上锁）：量舵机甩动的反作用、舵机响应和延迟，摆幅在「准备」页",
    "ALT": "高度（槽式台架）：电机会转、机体会沿槽上下移动——先确认槽的上下限位与测距下方地面；"
           "设置在「准备」页「高度」分区，只存档、离线分析",
}

#: 批量帧 flags 里的模式位按本轮模式应有的取值（drv_sysid_record.h：0x20 RATE / 0x40 ANGLE /
#: 0x80 SERVO；FF 三位都不置）。
MODE_FLAG_BITS = 0xE0
MODE_FLAGS = {0: (0x00,), 1: (0x20,), 2: (0x40,), 3: (0x80,)}
#: ALT（mode=4）的模式位不在跨侧契约里（固件草稿用 0x0100，在上面的掩码之外）：
#: 这里只拒绝明确属于别的模式的单个位，不要求 ALT 位本身。
ALT_FOREIGN_FLAGS = (0x20, 0x40, 0x80)

#: 一次采集最多留多少条样本。500 Hz 跑满 30 s 是 15000 条，留两倍余量。
MAX_SAMPLES = 40000


class SysIdInnerLoopPage(PageSections, AltSection, WaveformView, LiveStatus, JointPanel,
                         StiffnessPanel, NotchPanel, BacklashPanel, AmplitudeHint, SysIdPageBase):
    def __init__(self, panel, parent: ttk.Frame) -> None:
        super().__init__(panel, parent)
        self.status_var.set("")   # 顶部大字状态负责"连没连上"，这一行只写最近一次动作的结果

        self.schema = None
        self.schema_lines: list[str] = []
        self.batches: list = []
        self.samples: list[dict[str, float]] = []
        self.gap_count = 0
        self.dropped_hint = 0
        self.run_state_var = tk.StringVar(value="")
        self.sample_count_var = tk.StringVar(value="样本 0")
        self.schema_var = tk.StringVar(value="字段表：未读取")
        self.fit_var = tk.StringVar(value="跑完一轮「FF 测模型」后，这里会自动显示惯量、延迟和建议参数。")
        self.gain_var = tk.StringVar(value="")
        self.fit_ref_var = tk.StringVar(value="")      # 【诊断】块（灰字）
        self.plant_var = tk.StringVar(value="")        # 【对象特性】块
        self.apply_scale_var = tk.StringVar(value="50%")   # 每次默认 50%，不持久化
        self.experiment_var = tk.StringVar(value=DEFAULT_EXPERIMENT)
        self._fit_context: dict = {}
        self._commands_joint = False
        #: 候选参数的力矩单位（录制那轮的倾转力臂，`lever_units.torque_record`）。
        self._commands_source = None
        self.vibration_text = ""
        self.erpm_var = tk.StringVar(value="")

        # 顶部状态
        self.banner_var = tk.StringVar(value="未连接")
        self.banner_detail_var = tk.StringVar(value="")
        self.rc_var = tk.StringVar(value="遥控器：状态未知")
        self.stop_hint_var = tk.StringVar(value=STOP_HINT_AUTO)
        self.mode_hint_var = tk.StringVar(value=MODE_HINTS["FF"])
        self.thr: dict | None = None
        self.thr_generation = None
        self.thr_time = 0.0
        self.fw_ver: int | None = None
        self.fw_ver_generation = None
        self._primed_generation = None
        self.phase: str | None = None
        self.stopping = False
        self.analysis_note = ""
        self._banner_shown = None
        self._poll_job = None

        # 程序油门
        self.target_thrust_var = tk.StringVar(value="")
        self.max_pct_var = tk.StringVar(value="75")
        self.manual_throttle_var = tk.BooleanVar(value=False)
        self.thrust_hint_var = tk.StringVar(value="")

        # 台架物理条件
        self.psi_var = tk.StringVar(value="45.0")
        self.axis_preset_var = tk.StringVar(value="斜向 +45°（前—左）")
        self.psi_var.trace_add("write", self._sync_axis_preset)
        # 杆到质心 d = 杆到飞控（尺量）+ 飞控到质心（机体参数）；d 是拟合的输入，必须量。
        self.rod_to_fc_var = tk.StringVar(value="")
        # 倾转轴到飞控板（尺量）：κ 与整定惯量的力臂是舵机倾转轴到质心，不是桨盘中点。
        self.roll_pivot_var = tk.StringVar(value="")
        self.pitch_pivot_var = tk.StringVar(value="")
        self.axis_override_var = tk.StringVar(value="")
        self.geometry_hint_var = tk.StringVar(value="")
        self.pivot_warning_var = tk.StringVar(value="")   # 与机体参数舵机转轴高度不一致时的灰字提醒

        # 激励
        self.profile_var = tk.StringVar(value="doublet")
        # 2026-09-27 固件力矩模型修正后按 ×0.43 缩小（见 sections.EXPERIMENTS）；
        # 读到板子参数后按当前力臂自动换（amplitude_hint.py）。
        self.amp_var = tk.StringVar(value="0.065")
        self.dur_var = tk.StringVar(value="8000")
        self.hold_var = tk.StringVar(value="250")
        self.repeat_var = tk.StringVar(value="16")  # 双脉冲 16 对，总 8 s（摆约 0.7 Hz，要多摆几个周期）
        self.ramp_var = tk.StringVar(value="150")
        self.f0_var = tk.StringVar(value="0.3")
        self.f1_var = tk.StringVar(value="6.0")
        self.bit_var = tk.StringVar(value="40")
        self.seed_var = tk.StringVar(value="1")

        self.rate_var = tk.StringVar(value="250")
        self.inertia_var = tk.StringVar(value="")
        self.angle_limit_var = tk.StringVar(value="20")
        self.resid_limit_var = tk.StringVar(value="30")

        self.mode_var = tk.StringVar(value="FF")
        self.angle_amp_var = tk.StringVar(value="3")
        self.servo_tilt_var = tk.StringVar(value="5")     # 舵机单独的摆幅 [deg]，5° = 87 mrad
        self._init_stiffness_vars()
        self._init_notch_vars()
        self._init_backlash_vars()
        self._init_amp_hint_vars()
        self._init_alt_vars()
        self.workflow = Workflow(self)
        self.command_preview_var = tk.StringVar(value="")
        self._build(parent)
        self.mode_var.trace_add("write", lambda *_a: self.mode_hint_var.set(
            MODE_HINTS.get(self.mode_var.get(), "")))
        self.manual_throttle_var.trace_add("write", lambda *_a: self._on_throttle_mode())
        self.target_thrust_var.trace_add("write", lambda *_a: self.refresh_thrust_hint())
        for variable in (self.rod_to_fc_var, self.axis_override_var,
                         self.roll_pivot_var, self.pitch_pivot_var):
            variable.trace_add("write", lambda *_a: self.refresh_geometry_hint())
            variable.trace_add("write", lambda *_a: self.refresh_stiffness_hint())
        self.mode_var.trace_add("write", lambda *_a: self.refresh_banner())
        # 预计舵机摆幅还取决于杆轴方向、目标推力、假定惯量和模式（激励参数走 _refresh_preview）。
        for variable in (self.psi_var, self.target_thrust_var, self.inertia_var,
                         self.manual_throttle_var, self.mode_var):
            variable.trace_add("write", lambda *_a: self.refresh_amp_hint())
        register_board_line_hook(panel, self.handle_line)
        parent.bind("<Destroy>", self._on_destroy, add="+")
        self.load_rig_settings()
        self.update_pivot_rows()
        self._refresh_preview()
        self.refresh_thrust_hint()
        self.refresh_geometry_hint()
        self.refresh_stiffness_hint()
        self.refresh_banner()
        self._schedule_poll()

    # ------------------------------------------------------------ 界面

    def _build(self, parent: ttk.Frame) -> None:
        self._build_run(parent)
        self.steps_notebook = ttk.Notebook(parent)
        self.steps_notebook.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        prepare, observe, results, advanced = [ttk.Frame(self.steps_notebook, padding=6)
                                             for _ in range(4)]
        for frame, title in zip((prepare, observe, results, advanced),
                ("1 · 准备", "2 · 看波形", "3 · 结果", "高级设置")):
            self.steps_notebook.add(frame, text=title)
        self.observe_tab, self.results_tab = observe, results
        self._build_experiment(prepare)
        self._build_throttle(prepare)
        self._build_rig(prepare)
        self._build_servo(prepare)
        self._build_alt(prepare)
        self._build_stiffness(prepare)
        self._build_plot(observe)
        self._build_result(results)
        self._build_joint(results)
        self._build_manual_hold(advanced)
        self._build_excitation(advanced)
        self._build_limits(advanced)
        self._build_notch(advanced)
        self._build_backlash(advanced)
        self._build_diagnostics(advanced)

    def _build_run(self, parent: ttk.Frame) -> None:
        box = ttk.Frame(parent)
        box.pack(fill=tk.X, pady=(0, 4))
        self.start_button = ttk.Button(box, text="开始辨识", style="Primary.TButton",
                                       command=self.start_run)
        self.start_button.pack(side=tk.LEFT)
        self.stop_button = ttk.Button(box, text=STOP_TEXT_AUTO, style="Danger.TButton",
                                      command=self.stop_run)
        self.stop_button.pack(side=tk.LEFT, padx=(8, 0))
        modes = ttk.Frame(parent)
        modes.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(modes, text="本轮做").pack(side=tk.LEFT)
        ttk.Combobox(modes, textvariable=self.mode_var, values=settings_store.MODES,
                     state="readonly", width=7).pack(side=tk.LEFT, padx=6)
        ttk.Label(modes, textvariable=self.mode_hint_var, style="Muted.TLabel",
                  wraplength=520).pack(side=tk.LEFT)
        ttk.Label(parent, textvariable=self.banner_var, style="PageTitle.TLabel",
                  wraplength=660, justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))
        self.banner_detail_label = ttk.Label(parent, textvariable=self.banner_detail_var,
                                             wraplength=660, justify=tk.LEFT)
        self.banner_detail_label.pack(anchor=tk.W, fill=tk.X)
        ttk.Label(parent, textvariable=self.rc_var, style="Muted.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, textvariable=self.stop_hint_var, style="Muted.TLabel",
                  wraplength=660, justify=tk.LEFT).pack(anchor=tk.W)
        self.primary_status_label = self.mount_status(parent)

    # ------------------------------------------------------------ 参数收集

    def excitation(self) -> Excitation | None:
        try:
            spec = Excitation(
                profile=PROFILE_CODES[self.profile_var.get()],
                amplitude_rad_s=float(self.amp_var.get()),
                duration_ms=int(float(self.dur_var.get())),
                hold_ms=int(float(self.hold_var.get())),
                repeat=int(float(self.repeat_var.get())),
                ramp_ms=int(float(self.ramp_var.get())),
                chirp_f0_hz=float(self.f0_var.get()),
                chirp_f1_hz=float(self.f1_var.get()),
                prbs_bit_ms=int(float(self.bit_var.get())),
                prbs_seed=int(float(self.seed_var.get())))
            spec.validate()
        except (ValueError, KeyError) as error:
            self.command_preview_var.set(f"参数无效：{error}")
            return None
        return spec

    def _refresh_preview(self) -> None:
        self.profile_hint_var.set(PROFILE_LABELS.get(self.profile_var.get(), ""))
        spec = self.excitation()
        if spec is not None:
            self.command_preview_var.set(f"{spec.command()}   （总时长 {spec.total_ms()} ms）")
            self._draw(spec)
        self.refresh_amp_hint()

    # ------------------------------------------------------------ 下发

    def send_rig(self) -> None:
        if self.workflow.awaiting or self.workflow.job:
            self.status_var.set("正在等待飞控回显或分析结束")
            return
        try:
            psi = float(self.psi_var.get())
            if not math.isfinite(psi):
                raise ValueError("杆轴方位角要填有限数字")
            rod_to_fc, override = self.geometry_inputs()
        except ValueError as error:
            self.status_var.set(str(error) or "台架几何填了非数字")
            return
        self.workflow.send_rig(psi, rod_to_fc, override)

    def send_excitation(self) -> None:
        spec = self.excitation()
        if spec is None:
            self.status_var.set("激励参数没通过体检，未下发")
            return
        self.send(spec.command())

    def send_limits(self) -> None:
        try:
            rate = int(float(self.rate_var.get()))
            angle = float(self.angle_limit_var.get())
            resid = float(self.resid_limit_var.get())
        except ValueError:
            self.status_var.set("采样/限位填了非数字")
            return
        self.send(f"SYSID RATE {rate}")
        inertia = self.inertia_var.get().strip()
        if inertia:
            self.send(f"SYSID INERTIA {inertia}")
        self.send(f"SYSID LIMIT angle_deg={angle:g} resid_dps={resid:g}")

    def request_status(self) -> None:
        self.send("SYSID?")

    def request_schema(self) -> None:
        self.schema_lines = []
        self.send("SYSID SCHEMA")

    def load_rig_settings(self) -> None:
        """上次开始辨识时的台架设置；读不到或有坏项就用默认值，只在状态行提示一次。"""
        values, note = settings_store.load(settings_store.settings_path())
        settings_store.apply_to_page(self, values)
        if note:
            self.status_var.set(note)

    def save_rig_settings(self) -> None:
        """开始辨识通过输入检查时记一次；写不进去只提示，不挡开跑。"""
        try:
            settings_store.save(settings_store.collect_from_page(self),
                                settings_store.settings_path())
        except (OSError, ValueError, TypeError) as error:
            text = f"台架设置没能保存（{error}），这次照常开始，下次打开可能要重填。"
            self.status_var.set(text)
            append = getattr(self.panel, "_append", None)
            if callable(append):
                append(f"[系统辨识] {text}")

    def start_run(self) -> None:
        self.workflow.start()
        self.refresh_banner()

    def stop_run(self) -> None:
        w = self.workflow
        was_starting = w.kind == "start" and bool(w.awaiting or w.pending)
        run_active = (w.run_id is not None and w.end is None) or w.awaiting == "SYSID START"
        w.fail("已取消排队动作，正在请求停止")
        w.notice = "你点了停止，这一轮没有开始。" if was_starting and not run_active else ""
        self.stopping = run_active
        self.send("SYSID STOP")
        self.refresh_banner()

    def clear_samples(self) -> None:
        self.batches = []
        self.samples = []
        self.gap_count = 0
        self._fit = None
        self._fit_context = {}
        self._commands = None
        self._commands_joint = False
        self._commands_source = None
        self.vibration_text = ""
        self.erpm_var.set("")
        self.workflow.job = None
        self.workflow.data_error = ""
        self.workflow.candidate_dir = None
        self.fit_ref_var.set("")
        self.plant_var.set("")
        self.sample_count_var.set("样本 0")
        self.fit_var.set("跑完一轮「FF 测模型」后，这里会自动显示惯量、延迟和建议参数。")
        self.gain_var.set("")

    # ------------------------------------------------------------ 收行

    def handle_line(self, line: str) -> None:
        text = line.strip()
        if text.startswith("SYSID SCHEMA "):
            self.schema_lines = [text]
            self.schema = None
        elif text.startswith("SYSID FIELD "):
            self.schema_lines.append(text)
        if text.startswith(("SYSID SCHEMA ", "SYSID FIELD ")):
            self._try_parse_schema()
        if text.startswith("SYSID LIMITS "):
            self._note_limits(text)
        if text.startswith("SYSID state="):
            self.run_state_var.set(text[6:])
        if text.startswith("SYSID READY "):
            try:
                self.fw_ver = int(parse_kv(text).get("ver", ""))
            except ValueError:
                self.fw_ver = None
            self.fw_ver_generation = self.workflow.generation()
        if text.startswith(("SYSID THR ", "SYSID PHASE ")):
            self._note_throttle(text)
        if self.notch_handle_line(text):
            return                       # 陷波的回复（含旧固件不认它的 ERR）不属于辨识事务
        if self.backlash_handle_line(text):
            return                       # 回差补偿的回复同上
        self.workflow.on_line(text)
        if text.startswith("SYSID READY "):
            self.refresh_thrust_hint()
        if text.startswith(("SYSID READY ", "PARAM name=airframe.mass_kg ")):
            self.refresh_alt_hint()      # 高度分区的合计质量 = 机体质量 + 附加质量
        if text.startswith(("SYSID", "ERR", "OK sysid", "PARAM ", "OK param ")):
            # 机体参数或事务状态变了：重估舵机摆幅（空闲时可能自动换幅值），顺带刷新顶部状态。
            self.refresh_amp_hint()

    def _note_throttle(self, text: str) -> None:
        values = parse_kv(text)
        w = self.workflow
        running = w.run_id is not None and w.end is None
        if text.startswith("SYSID THR "):
            self.thr = values
            self.thr_generation = w.generation()
            self.thr_time = time.monotonic()
            if running and values.get("phase") not in (None, "", "idle"):
                self.phase = values["phase"]
            return
        run = values.get("run")
        if w.run_id is None or run is None or str(w.run_id) == run:
            self.phase = values.get("phase") or self.phase

    def _note_limits(self, text: str) -> None:
        """记住飞控**实际**在用的假定惯量。

        拟合必须用这个数，而不是界面输入框里的那个：输入框留空时固件会自己去取
        `airframe.ixx_kgm2`，两边对不上就会让"惯量比值"这个收敛指标整体偏一个
        常数——而那个比值正是判断"还要不要再跑一轮"的唯一依据。
        """
        for token in text.split():
            if token.startswith("I_ugm2="):
                try:
                    self.firmware_inertia_kg_m2 = float(token.split("=", 1)[1]) * 1e-6
                except ValueError:
                    pass
                return

    def _try_parse_schema(self) -> None:
        try:
            schema = parse_schema_lines(self.schema_lines)
        except (ValueError, KeyError):
            return  # 还没收全，或这批行本来就不完整
        self.schema = schema
        self.schema_var.set(
            f"字段表：{len(schema.fields)} 项，hash=0x{schema.hash:08X}，"
            f"每条 {schema.record_bytes} 字节 —— "
            + ", ".join(f.name for f in schema.fields))

    def accept(self, payload: bytes, context=None) -> None:
        if context is not None and not context.is_current(self.panel.transport):
            return
        w = self.workflow
        if w.session and w.generation() != w.session:
            w.data_fail("连接已改变，本轮采集失效")
            return
        if self.schema is None:
            w.data_fail("收到数据但字段表尚未完整")
            return
        try:
            batch = decode_batch(payload, self.schema)
        except SchemaMismatch as error:
            self.schema = None
            w.data_fail(f"拒绝解码：{error}")
            return
        except ValueError as error:
            w.data_fail(f"拒绝无效数据：{error}")
            return
        if w.run_id is not None and batch.run_id != w.run_id:
            w.data_fail("收到其他轮次的数据，不能混合拟合")
            return
        if w.snapshot is not None:
            mode_code, bits = int(w.snapshot.get("mode", "0")), batch.flags & MODE_FLAG_BITS
            if (bits in ALT_FOREIGN_FLAGS if mode_code == ALT_MODE_CODE
                    else bits not in MODE_FLAGS.get(mode_code, ())):
                w.data_fail("采样模式与本轮快照不符")
                return
        if batch.first:
            if self.batches:
                w.data_fail("重复首包或轮次混入")
                return
            w.run_id = batch.run_id
        elif not self.batches:
            w.mark_data_error("缺少首包")
        if self.batches:
            previous = self.batches[-1].timestamps_us()[-1]
            step = (batch.timestamps_us()[0] - previous) & 0xffffffff
            # 采样跟着控制拍走，间隔本来就在 dt 上下抖（2～3 ms 的拍、250 Hz 采样最坏约 6 ms）。
            # 真丢样由固件打 GAP 标志；这里只把倒退/重复或离谱的大间隔当断点。
            if step == 0 or step > batch.dt_us * 4:
                self.gap_count += 1
        if batch.gap:
            self.gap_count += 1
        if batch.aborted:
            w.mark_data_error("固件中止本轮")
        if len(self.samples) + len(batch.samples) > MAX_SAMPLES:
            w.data_fail("采样长度超过上限")
            self.send("SYSID STOP")
            return
        self.batches.append(batch)
        self.samples.extend(batch.samples)
        if batch.last:
            self.status_var.set("本趟结束，等待固件结束报告" + ("（中止）" if batch.aborted else ""))
        self.sample_count_var.set(f"样本 {len(self.samples)}，断点 {self.gap_count}")
        # 10 Hz drawing; sample ingestion is independent.
        now = time.monotonic()
        if batch.last or now - getattr(self, "_last_draw", 0) >= 0.1:
            self._last_draw = now
            self._draw_measured()
            self.refresh_banner()

    # ------------------------------------------------------------ 轮次回调（由 workflow 调）

    def on_run_starting(self) -> None:
        self.phase = None
        self.stopping = False
        self.analysis_note = ""
        self.fit_var.set("正在采集；高度辨识轮跑完只存档，分析离线进行。" if self.alt_run_active()
                         else "正在采集；跑完后自动分析。")

    def alt_run_active(self) -> bool:
        """正在跑（或刚结束）的是高度辨识轮，或还没开始时「本轮做」选了 ALT。"""
        w = self.workflow
        if w.run_id is not None or w.awaiting == "SYSID START":
            return str((w.snapshot or {}).get("mode")) == str(ALT_MODE_CODE)
        return self.alt_mode_selected()

    def on_run_finished(self, end: dict, mode: str) -> None:
        self.phase = None
        self.stopping = False
        self.vibration_text = (vibration_summary(self._timestamps(), self.samples)
                               if end.get("state") == "done" else "")
        if end.get("state") != "done":
            self.analysis_note = ""
            self.fit_var.set(cannot_analyse(end.get("reason"), mode))
            self.gain_var.set("")
            self.steps_notebook.select(self.observe_tab)
            what, _step = explain_end(end.get("reason"), mode)
            self.status_var.set(f"本轮中止：{what}（原因代码 {end.get('reason')}）")
        elif mode in ("0", "3"):
            self.analysis_note = "正在保存数据并自动分析…"
        elif mode == str(ALT_MODE_CODE):
            # 高度辨识轮不在页面上拟合（姿态拟合对它没有意义）：只存档，结果区写明离线分析。
            self._commands = None
            self.fit_var.set("\n".join((alt_provenance_text(self.workflow.snapshot), OFFLINE_NOTE)))
            self.fit_ref_var.set(self.vibration_text)
            self.plant_var.set("")
            self.gain_var.set("")
            self.analysis_note = "高度辨识轮完成：数据正在存档，页面不拟合，分析离线进行。"
            self.steps_notebook.select(self.results_tab)
        else:
            snapshot = self.workflow.snapshot or {}
            psi = float(snapshot.get("psi_mrad", 0.0))*1e-3
            f1 = (float(snapshot.get("f1_mhz", 0)) / 1000.0
                  if str(snapshot.get("profile")) == "2" else None)
            try:
                summary, note = tracking_summary(
                    self._timestamps(), self.samples, psi, int(mode) if mode.isdigit() else 1,
                    f1_hz=f1, crossover_hz=self._group_crossover_hz(snapshot) if f1 else None)
            except (ImportError, ValueError) as error:
                summary, note = f"算不出跟踪误差：{error}", ""
            self.fit_var.set("\n".join(part for part in (notch_provenance_text(snapshot),
                                                         backlash_provenance_text(snapshot), summary)
                                       if part))
            self.fit_ref_var.set("\n".join(part for part in (note, self.vibration_text) if part))
            self.plant_var.set("")
            self.gain_var.set("")
            self.analysis_note = "验证轮完成。" + summary.splitlines()[0] + "。详情在「3 · 结果」。"
            self.steps_notebook.select(self.results_tab)
        self.refresh_banner()

    def _group_crossover_hz(self, snapshot: dict) -> float | None:
        """同一组（同一天、同杆、同推力、同力矩模型）最近一次拟合的速率环穿越频率。"""
        from .joint import ArchivedRun, latest_crossover_hz
        try:
            # 本轮存档目录还没建（结束报告先到、存档随后）：用它将要落的日期目录定"同一天"。
            folder = self.workflow.saved or self.workflow.archive_directory()
            return latest_crossover_hz(ArchivedRun(folder, snapshot).key)
        except (OSError, ImportError):
            return None

    def on_fit(self, result, note: str = "", context: dict | None = None) -> None:
        context = dict(context or {})
        self._fit = result
        self._fit_context = context
        self._commands = None
        self._commands_joint = False
        self._commands_source = None
        if context.get("kind") == "servo":
            self.on_servo_fit(result, note, context)
            return
        blocks = card_blocks(result, pivot_m=context.get("pivot_m"),
                             header=context.get("header", ""),
                             geometry=context.get("geometry", ""))
        self.fit_var.set(blocks["pid"] + note)
        self.plant_var.set("\n".join(part for part in (blocks["plant"], context.get("cross_check"))
                                      if part))
        vibration = "" if context.get("joint") else self.vibration_text
        self.fit_ref_var.set("\n".join(part for part in (blocks["diag"], vibration) if part))
        fit = getattr(result, "fit_percent", float("nan"))
        if fit_passes(result):
            self.synthesise()
            ready = self._commands is not None
            self.analysis_note = (
                f"分析完成：带内拟合度 {fit:.0f}%。" + ("建议参数已算好——到「3 · 结果」查看，"
                "满意再点「临时应用到飞控」。" if ready else "建议参数没算出来，见「3 · 结果」。"))
        else:
            problems = quality_problems(result)
            steps = "；".join(advice_for(problems))
            self.gain_var.set(f"不给建议参数：{'、'.join(problems)}。\n下一步：{steps}。")
            self.analysis_note = f"分析完成，但没给建议参数。下一步：{steps}。"
        self.steps_notebook.select(self.results_tab)
        self.refresh_banner()

    def on_servo_fit(self, outcome, note: str, context: dict) -> None:
        """舵机单独轮：只报对象特性，不给参数（写 RAM 的按钮对它无效）。"""
        blocks = outcome.blocks
        header = [line for line in (context.get("header", ""), context.get("geometry", "")) if line]
        self.fit_var.set("\n".join(header + [blocks["pid"]]) + note)
        self.plant_var.set(blocks["plant"])
        self.fit_ref_var.set(blocks["diag"])
        self.gain_var.set("舵机单独轮不给建议参数：它只测舵机与反作用，作为带桨拟合的交叉核对"
                          "（本版不把它钉进带桨联合拟合）。")
        fit = outcome.fit
        self.analysis_note = (f"舵机单独分析完成：反作用惯量 J = {fit.reaction_inertia_kg_m2:+.4f} kg·m²，"
                              f"舵机 {fit.servo_wn_rad_s / 6.283185307:.1f} Hz，纯延迟 "
                              f"{fit.dead_time_s * 1000:.0f} ms。详情在「3 · 结果」。")
        self.steps_notebook.select(self.results_tab)
        self.refresh_banner()

    def on_fit_error(self, text: str) -> None:
        self.fit_var.set(f"分析失败：{text}")
        self.analysis_note = f"自动分析失败：{text}"
        self.refresh_banner()

    def _timestamps(self) -> list[float]:
        stamps = [t for batch in self.batches for t in batch.timestamps_us()]
        if not stamps:
            return []
        elapsed = [0.0]
        for a, b in zip(stamps, stamps[1:]):
            elapsed.append(elapsed[-1] + ((b-a) & 0xffffffff)*1e-6)
        return elapsed

    # ------------------------------------------------------------ 分析与参数

    def run_fit(self) -> None:
        """「重新分析」：用「1 · 准备」页当前填写的几何重算本轮，不用重跑。"""
        self.workflow.fit(manual=True)

    def synthesise(self) -> None:
        from ._core import synthesise_gains

        result = getattr(self, "_fit", None)
        if result is None:
            self.gain_var.set("先分析，再算建议参数——带宽由辨出来的延迟决定。")
            return
        context = self._fit_context or {}
        if context.get("kind") == "servo":
            self.gain_var.set("舵机单独轮不给建议参数。")
            return
        joint = bool(context.get("joint"))
        if (not joint and self.workflow.data_error) or not fit_passes(result):
            self.gain_var.set("本轮数据或拟合质量不足，拒绝生成可写入参数")
            return
        snapshot = self.workflow.snapshot or {}
        try:
            psi = context.get("psi")
            if psi is None:
                psi = float(snapshot.get("psi_mrad", 0.0))*1e-3
            try:   # 新签名只给辨识到的轴出参数；拟合核心升级前的旧签名不认 azimuth_rad。
                rate, attitude, commands = synthesise_gains(result, azimuth_rad=psi)
            except TypeError as error:
                if "azimuth_rad" not in str(error):
                    raise
                rate, attitude, commands = synthesise_gains(result)
        except (ValueError, AttributeError, TypeError, ZeroDivisionError) as error:
            self.gain_var.set(f"建议参数算不出来：{error}")
            return
        self._commands = commands
        self._commands_joint = joint
        self._commands_source = context.get("source")
        self.workflow.candidate_dir = context.get("out_dir") or self.workflow.saved
        self.gain_var.set(gains_card(rate, attitude, commands) + "\n" + units_note(self._commands_source))

    def apply_to_ram(self) -> None:
        if self._fit_context.get("kind") == "servo":
            self.status_var.set("舵机单独轮没有候选参数可应用。")
            return
        alt_run = str((self.workflow.snapshot or {}).get("mode")) == str(ALT_MODE_CODE)
        if alt_run and not getattr(self, "_commands", None):
            self.status_var.set("高度辨识轮页面不给候选参数：高度环参数由离线分析脚本经 SYSID PARAM 试用。")
            return
        """按所选比例（默认 50%）临时应用：只缩放 kp/ki（含姿态 P），kd 本来就是 0。"""
        try:
            scale = float(self.apply_scale_var.get().rstrip("%")) / 100.0
        except ValueError:
            scale = 0.5
        self.workflow.apply(scale=scale if scale in (0.5, 0.75, 1.0) else 0.5)

    def _mass_kg(self) -> float:
        if self.workflow.snapshot is None:
            raise ValueError("未取得本轮机体质量")
        return float(self.workflow.snapshot["mass_mg"])*1e-6


def units_note(source) -> str:
    """候选参数卡片末尾那句：它是哪套力矩单位、应用时怎么换算。"""
    return (f"力矩单位：以上是录制那轮固件的单位（{describe_units(source)}）；点「临时应用」时会重读"
            "飞控机体参数，按当前力臂换算速率环 kp/ki/kd 再写（力臂读不出或方向相反就拒绝）。")


def mount_sysid_inner_loop(panel, parent: ttk.Frame) -> SysIdInnerLoopPage:
    page = SysIdInnerLoopPage(panel, parent)
    panel.sysid_page = page
    return page
