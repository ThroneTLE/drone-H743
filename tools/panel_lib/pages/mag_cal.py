"""「校准 · 磁力计校准」页：采样 → 覆盖度判定 → 椭球拟合 → 预览 → 提交 → 轴向验证。

固件唯一事实源是 `App/Src/app_cmd_magcal.c`（MAGCAL/MAGFRAME 命令族，格式细节见
`tools/panel_lib/mag_protocol.py` 顶部说明）；拟合算法唯一事实源是
`tools/mag_cal_fit.py`，本页只调用它，不重新定义覆盖度判据或拟合公式。

**本页最重要的一条信息**：磁力计只有在"已校准 ∧ 轴向已验证 ∧ 样本新鲜 ∧
场强合理"全部成立时才参与姿态解算（默认出厂状态是不参与）。判断当下到底参没参与，
权威信号是固件回报的 `MAGCAL fusion ... used=`，不是 `axis_verified`——顶部的
融合状态横幅直接显示这个结论，不要求用户自己拼凑几个字段。

**没有遥测通道可用。** 遥测流 v2（`telem_subscription.py`）目前没有磁力计原始读数
的通道，新增通道属于固件 App 层改动，不在本任务授权范围内（见任务交付里的
"上游发现但未改"说明）。采样和轴向验证的实时读数因此只能走 `MAG?` 问答轮询——
这是本页唯一的例外：其余状态（MAGCAL?/MAGFRAME?）按较慢节奏轮询，`MAG?`
按较快节奏轮询仅在本页可见时进行，不用于其他页面，也不在用户离开本页时继续占用链路。

**两段式写入，不允许一键写 Flash**：「应用到 RAM」只发 `MAGCAL SET ... / APPLY`，
断电即恢复；只有另外点「写入 Flash」并在确认对话框中确认后才会发 `MAGCAL COMMIT`
落盘。轴向验证同理：`MAGFRAME VERIFY CONFIRM` 需要先勾选物理确认复选框。

**没有"部分撤销"**：固件的 `MAGCAL CLEAR` 只会把零偏/软磁系数**和**轴向验证状态
一起恢复出厂（未校准、未验证），不存在"只退回上一次而不影响验证状态"的命令——
清除按钮的确认文案必须把这一点说清楚，不能让用户以为清除校准不影响轴向验证。
"""

from __future__ import annotations

import time
import tkinter as tk
from tkinter import messagebox, ttk

from .. import mag_protocol
from ..board_line_hooks import register_board_line_hook
from ..connection_state import receive_context

try:
    from ...mag_cal_fit import (
        DEFAULT_THRESHOLDS,
        NUMPY_AVAILABLE,
        MagCalFitError,
        NumpyRequiredError,
        evaluate_coverage,
        fit_mag_calibration,
        fit_problems,
    )
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools.mag_cal_fit import (
            DEFAULT_THRESHOLDS,
            NUMPY_AVAILABLE,
            MagCalFitError,
            NumpyRequiredError,
            evaluate_coverage,
            fit_mag_calibration,
            fit_problems,
        )
    except ImportError:
        from mag_cal_fit import (
            DEFAULT_THRESHOLDS,
            NUMPY_AVAILABLE,
            MagCalFitError,
            NumpyRequiredError,
            evaluate_coverage,
            fit_mag_calibration,
            fit_problems,
        )


MAG_CAL_TAB_TEXT = "磁力计校准"

#: 页面可见时的节拍；也是 `MAG?` 采样轮询的节奏（~5 Hz，足够跟手又不占满链路）。
MAG_CAL_TICK_MS = 150
MAG_RAW_POLL_S = 0.2
#: `MAGCAL?` 状态/融合诊断的轮询节奏——比采样慢得多，够用即可，见模块顶部说明。
MAG_STATUS_POLL_S = 1.0
#: 一次 SET.../APPLY 或 COMMIT 序列等待飞控确认的上限。
MAG_CAL_REPLY_TIMEOUT_S = 3.0

_RELEVANT_PREFIXES = (
    "MAGCAL", "MAGFRAME", "MAG ",
    "ERR usage MAGCAL", "ERR usage MAGFRAME",
    "ERR unknown cmd MAGCAL", "ERR unknown cmd MAGFRAME",
)


class MagCalPage(ttk.Frame):
    def __init__(self, parent, panel):
        super().__init__(parent, padding=12)
        self.panel = panel
        self.closed = False
        self.timer = None

        # --- 会话防护（见 `_connection()`）
        self.session = None
        self.unsupported = False
        self.protocol_errors = 0

        # --- 从飞控读到的状态（`_connection()`/换代次时整体清空）
        self.status: mag_protocol.MagCalStatus | None = None
        self.bias: mag_protocol.MagCalBias | None = None
        self.matrix: mag_protocol.MagCalMatrix | None = None
        self.draft: mag_protocol.MagCalDraft | None = None
        self.fusion: mag_protocol.MagFusionStatus | None = None
        self.frame: mag_protocol.MagFrameStatus | None = None
        self.status_received_at: float | None = None
        self.live_chip_mgauss: tuple[float, float, float] | None = None
        self.live_flu_mgauss: tuple[float, float, float] | None = None

        # --- 采样缓冲（FLU mgauss；来自 `MAG?`，见模块顶部关于坐标变换的说明）
        self.sampling = False
        self.samples: list[tuple[float, float, float]] = []
        self.coverage = None
        self.fit_result = None

        # --- 应用/提交跟踪
        self.applied = False
        self.apply_pending = False
        self.apply_deadline = 0.0
        self.draft_acks: set[str] = set()

        # --- 轮询节拍
        self.last_raw_poll = -1e9
        self.last_status_poll = -1e9
        self.opened_frame_once = False

        self.notice = tk.StringVar(self, value="")
        self.sample_notice = tk.StringVar(self, value="尚未开始采样。")

        self._build(self)
        register_board_line_hook(panel, self.handle_board_line)
        self.bind("<Destroy>", self._destroy, add="+")
        self._tick()

    # ------------------------------------------------------------ 构建

    def _build(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="MAGNETOMETER CALIBRATION", style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text="磁力计硬磁/软磁校准与轴向验证", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=(
                "流程：采样（缓慢转动机体覆盖各个朝向）→ 覆盖度足够后拟合 → 预览拟合质量 → "
                "应用到 RAM（可反复调整）→ 确认后写入 Flash；轴向验证是独立的一步，"
                "验证通过之前磁力计不会参与姿态融合。采样前尽量远离大块铁磁材料和已通电的电机。"
            ),
            style="Muted.TLabel", wraplength=1120, justify=tk.LEFT,
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))

        self._build_fusion_banner(parent)
        self._build_status_section(parent)
        self._build_sampling_section(parent)
        self._build_fit_section(parent)
        self._build_apply_section(parent)
        self._build_axis_section(parent)

        ttk.Label(parent, textvariable=self.notice, style="Guide.TLabel",
                  wraplength=1120, justify=tk.LEFT).pack(fill=tk.X, pady=(8, 0))

    def _build_fusion_banner(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="磁力计当前是否参与姿态融合", padding=10)
        box.pack(fill=tk.X, pady=(0, 8))
        self.fusion_banner_var = tk.StringVar(self, value="尚未读取。")
        self.fusion_banner_label = ttk.Label(
            box, textvariable=self.fusion_banner_var, style="PageTitle.TLabel",
            wraplength=1080, justify=tk.LEFT)
        self.fusion_banner_label.pack(anchor=tk.W)

    def _build_status_section(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="当前状态", padding=10)
        box.pack(fill=tk.X, pady=6)
        box.columnconfigure(1, weight=1)
        box.columnconfigure(3, weight=1)
        self.status_fields: dict[str, tk.StringVar] = {}
        entries = (
            ("calibrated", "是否已校准"), ("axis_verified", "轴向验证（存储值）"),
            ("axis_effective", "轴向验证（有效值）"), ("contract", "坐标契约版本"),
            ("dirty", "与 Flash 是否一致"), ("bias", "硬磁零偏"),
        )
        for index, (key, label) in enumerate(entries):
            row, column = (index, 0) if index < 5 else (index - 5, 2)
            ttk.Label(box, text=label).grid(row=row, column=column, sticky=tk.W, padx=(0, 12), pady=2)
            var = tk.StringVar(self, value="—")
            self.status_fields[key] = var
            ttk.Label(box, textvariable=var, style="Mono.TLabel").grid(
                row=row, column=column + 1, sticky=tk.W, pady=2)
        ttk.Label(box, text="软磁矩阵").grid(row=5, column=0, sticky=tk.NW, padx=(0, 12), pady=2)
        matrix_var = tk.StringVar(self, value="—")
        self.status_fields["matrix"] = matrix_var
        ttk.Label(box, textvariable=matrix_var, style="Mono.TLabel", wraplength=760).grid(
            row=5, column=1, columnspan=3, sticky=tk.W, pady=2)

        self.status_summary_var = tk.StringVar(self, value="尚未读取。")
        ttk.Label(box, textvariable=self.status_summary_var, style="Muted.TLabel",
                  wraplength=1080).grid(row=6, column=0, columnspan=4, sticky=tk.W, pady=(6, 0))

        actions = ttk.Frame(box)
        actions.grid(row=7, column=0, columnspan=4, sticky=tk.W, pady=(8, 0))
        ttk.Button(actions, text="从飞控读取", command=self.refresh_status,
                   style="Secondary.TButton").pack(side=tk.LEFT)
        self.clear_button = ttk.Button(
            actions, text="恢复出厂（清除校准）", command=self.clear_calibration,
            style="Danger.TButton")
        self.clear_button.pack(side=tk.LEFT, padx=(8, 0))

    def _build_sampling_section(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="采样与覆盖度", padding=10)
        box.pack(fill=tk.X, pady=6)
        ttk.Label(
            box,
            text="点「开始采样」后缓慢转动机体，尽量让机头朝向各个方向、正装/倒装都转到；"
                 "覆盖度足够前「拟合」按钮保持禁用。",
            style="Muted.TLabel", wraplength=1080, justify=tk.LEFT,
        ).pack(anchor=tk.W)

        actions = ttk.Frame(box)
        actions.pack(fill=tk.X, pady=(6, 0))
        self.sample_start_button = ttk.Button(
            actions, text="开始采样", command=self.start_sampling, style="Primary.TButton")
        self.sample_start_button.pack(side=tk.LEFT)
        self.sample_stop_button = ttk.Button(
            actions, text="暂停采样", command=self.stop_sampling,
            state=tk.DISABLED, style="Secondary.TButton")
        self.sample_stop_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(actions, text="清空采样缓冲", command=self.clear_samples_action,
                   style="Secondary.TButton").pack(side=tk.LEFT, padx=(8, 0))

        self.sample_count_var = tk.StringVar(self, value="已采集 0 个样本")
        ttk.Label(box, textvariable=self.sample_count_var, style="Mono.TLabel").pack(
            anchor=tk.W, pady=(8, 0))
        self.live_reading_var = tk.StringVar(self, value="—")
        ttk.Label(box, text="实时读数（默认轴向 (x,-y,-z)，未必已验证）：").pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(box, textvariable=self.live_reading_var, style="Mono.TLabel").pack(anchor=tk.W)

        self.coverage_var = tk.StringVar(self, value="尚未开始采样。")
        self.coverage_label = ttk.Label(box, textvariable=self.coverage_var,
                                        style="Muted.TLabel", wraplength=1080, justify=tk.LEFT)
        self.coverage_label.pack(anchor=tk.W, pady=(8, 0))
        ttk.Label(box, textvariable=self.sample_notice, style="Muted.TLabel",
                  wraplength=1080).pack(anchor=tk.W, pady=(2, 0))

    def _build_fit_section(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="拟合", padding=10)
        box.pack(fill=tk.X, pady=6)
        self.fit_button = ttk.Button(box, text="拟合", command=self.run_fit,
                                     state=tk.DISABLED, style="Primary.TButton")
        self.fit_button.pack(anchor=tk.W)
        self.fit_summary_var = tk.StringVar(self, value="尚未拟合。")
        ttk.Label(box, textvariable=self.fit_summary_var, style="Mono.TLabel",
                  wraplength=1080, justify=tk.LEFT).pack(anchor=tk.W, pady=(6, 0))
        self.fit_verdict_var = tk.StringVar(self, value="")
        self.fit_verdict_label = ttk.Label(box, textvariable=self.fit_verdict_var,
                                           style="Muted.TLabel", wraplength=1080,
                                           justify=tk.LEFT)
        self.fit_verdict_label.pack(anchor=tk.W, pady=(4, 0))

    def _build_apply_section(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="预览 → 应用到 RAM → 写入 Flash", padding=10)
        box.pack(fill=tk.X, pady=6)
        ttk.Label(
            box,
            text="拟合结果只是本地预览，点「应用到 RAM」才会发给飞控（断电即恢复出厂）；"
                 "确认无误后再点「写入 Flash」，写入前会弹出确认对话框，不会一键直接落盘。",
            style="Muted.TLabel", wraplength=1080, justify=tk.LEFT,
        ).pack(anchor=tk.W)
        actions = ttk.Frame(box)
        actions.pack(fill=tk.X, pady=(6, 0))
        self.apply_button = ttk.Button(
            actions, text="应用到 RAM", command=self.apply_to_ram,
            state=tk.DISABLED, style="Warning.TButton")
        self.apply_button.pack(side=tk.LEFT)
        self.commit_button = ttk.Button(
            actions, text="写入 Flash", command=self.commit_to_flash,
            state=tk.DISABLED, style="Danger.TButton")
        self.commit_button.pack(side=tk.LEFT, padx=(8, 0))
        self.apply_progress_var = tk.StringVar(self, value="")
        ttk.Label(box, textvariable=self.apply_progress_var, style="Muted.TLabel").pack(
            anchor=tk.W, pady=(4, 0))

    def _build_axis_section(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="轴向验证（融合门控的前提）", padding=10)
        box.pack(fill=tk.X, pady=6)
        ttk.Label(
            box,
            text=(
                "默认推导：芯片轴 → FLU 机体轴按 (x, -y, -z) 换算，这只是数学推导，"
                "从未经过实机验证——贴反、走线转向都可能让它错。物理检查（不需要指南针）："
                "保持机体水平，缓慢向左转动机头（正 yaw）；正确贴装时，上方「实时读数」里"
                "水平方向的磁场分量应当跟着朝右侧（-Y）平滑扫过，就像你转头看向左边时，"
                "正前方的固定参照物看起来往你右边划过一样。如果读数往错误方向扫、忽左忽右、"
                "或某一轴几乎不变而另一轴剧烈跳变，说明贴装与默认推导不符——请不要确认，改为报告。"
                "（若手头有指南针，可选做：机头指北时，水平分量应主要落在 +X 而不是 Y 轴上，作为补充参考。）"
            ),
            style="Muted.TLabel", wraplength=1080, justify=tk.LEFT,
        ).pack(anchor=tk.W)
        self.axis_summary_var = tk.StringVar(self, value="尚未读取。")
        ttk.Label(box, textvariable=self.axis_summary_var, style="Mono.TLabel",
                  wraplength=1080).pack(anchor=tk.W, pady=(8, 0))
        self.verify_confirmed_var = tk.BooleanVar(self, value=False)
        ttk.Checkbutton(
            box, text="我已完成上述物理检查，确认贴装轴向与默认推导一致",
            variable=self.verify_confirmed_var,
        ).pack(anchor=tk.W, pady=(6, 0))
        self.verify_button = ttk.Button(
            box, text="确认贴装轴向（写入 MAGFRAME 验证位）", command=self.verify_axis,
            state=tk.DISABLED, style="Danger.TButton")
        self.verify_button.pack(anchor=tk.W, pady=(6, 0))

    # ------------------------------------------------------------ 会话

    def _connection(self) -> bool:
        """连接代次守卫：换连接或断连一律清空显示状态，重置采样缓冲。

        采样缓冲也来自这条链路——换连接代次可能是换了一块板子，继续往同一个
        拟合缓冲里混合两条链路的读数会得到一个物理上不成立的椭球，不是简单的
        "数据旧了"，所以和电源页只清"显示"不同，这里连本地累积的采样也要清。
        """
        transport = self.panel.transport
        session = (transport, getattr(transport, "connection_generation", 0))
        connected = self.panel._transport_connected()
        if session != self.session or not connected:
            self.session = session
            self.status = None
            self.bias = None
            self.matrix = None
            self.draft = None
            self.fusion = None
            self.frame = None
            self.status_received_at = None
            self.live_chip_mgauss = None
            self.live_flu_mgauss = None
            self.applied = False
            self.apply_pending = False
            self.draft_acks = set()
            self.unsupported = False
            self.opened_frame_once = False
            if self.samples or self.sampling:
                self._reset_sampling(note="连接已重置，采样缓冲已清空，请重新采样。")
            self.notice.set("" if connected else "未连接。")
        return connected

    def _reset_sampling(self, *, note: str | None = None) -> None:
        self.sampling = False
        self.samples = []
        self.coverage = None
        self.fit_result = None
        if note:
            self.sample_notice.set(note)

    def visible(self) -> bool:
        panel = self.panel
        group = getattr(panel, "calibration_group_tab", None)
        tab = getattr(panel, "mag_cal_tab", None)
        notebook = getattr(panel, "notebook", None)
        calibration = getattr(panel, "calibration_notebook", None)
        if group is None or tab is None or notebook is None or calibration is None:
            return False
        try:
            return notebook.select() == str(group) and calibration.select() == str(tab)
        except tk.TclError:                       # pragma: no cover - 窗口正在销毁
            return False

    # ------------------------------------------------------------ 发送

    def _poll(self, text: str) -> bool:
        """静默轮询：不写控制台日志，不弹确认框。给 `_tick()` 的周期性问答用。

        和 `send()` 分开是因为 `send()` 每次都会往控制台追加一行——如果轮询也走
        它，`MAG?`/`MAGCAL?` 每秒几次的节奏会把控制台刷成除了这两条什么都看不到。
        """
        panel = self.panel
        transport = getattr(panel, "transport", None)
        if transport is None or not getattr(transport, "is_connected", False):
            return False
        allowed = getattr(panel, "_validation_command_allowed", None)
        if callable(allowed) and not allowed(text):
            return False
        sender = getattr(transport, "send_ascii_line", transport.send_line)
        return bool(sender(text))

    def send(self, text: str) -> bool:
        """用户触发的写命令：过校验门、留控制台痕迹，被拒时给出可见原因。"""
        panel = self.panel
        transport = getattr(panel, "transport", None)
        if transport is None or not getattr(transport, "is_connected", False):
            self.notice.set("未连接飞控，命令未发送。")
            return False
        guard = getattr(panel, "_validation_guard_command", None)
        if callable(guard) and not guard(text):
            return False
        append = getattr(panel, "_append", None)
        if callable(append):
            append(f"> {text}")
        sender = getattr(transport, "send_ascii_line", transport.send_line)
        if not sender(text):
            self.notice.set("发送被拒绝或链路占用。")
            return False
        return True

    def refresh_status(self) -> None:
        self.send("MAGCAL?")
        self.send("MAGFRAME?")

    # ------------------------------------------------------------ 节拍

    def _tick(self) -> None:
        if self.closed:
            return
        connected = self._connection()
        visible = self.visible()
        now = time.monotonic()
        if self.apply_pending and now > self.apply_deadline:
            self.apply_pending = False
            self.notice.set("MAGCAL 写入超时，未确认结果。")
        if connected and visible:
            if now - self.last_raw_poll >= MAG_RAW_POLL_S:
                self.last_raw_poll = now
                self._poll("MAG?")
            if now - self.last_status_poll >= MAG_STATUS_POLL_S:
                self.last_status_poll = now
                self._poll("MAGCAL?")
            if not self.opened_frame_once:
                self.opened_frame_once = True
                self._poll("MAGFRAME?")
        self._render()
        self.timer = self.after(MAG_CAL_TICK_MS, self._tick)

    def _destroy(self, event) -> None:
        if event.widget is self:
            self.closed = True
            if self.timer is not None:
                self.after_cancel(self.timer)
                self.timer = None

    # ------------------------------------------------------------ 接收

    def handle_board_line(self, line: str) -> None:
        if not line.startswith(_RELEVANT_PREFIXES):
            return
        receipt = getattr(self.panel, "_rx_context", None) or receive_context(self.panel.transport)
        if not receipt.is_current(self.panel.transport):
            return
        if line.startswith("ERR unknown cmd MAGCAL") or line.startswith("ERR unknown cmd MAGFRAME"):
            self.unsupported = True
            self.notice.set("当前固件未提供磁力计校准命令族。")
            return
        if line.startswith("ERR usage"):
            self.notice.set(f"命令格式错误：{line}")
            return
        try:
            decoded = mag_protocol.decode_line(line)
        except mag_protocol.MagProtocolError as exc:
            self.protocol_errors += 1
            self.notice.set(f"回包解析失败（第 {self.protocol_errors} 次，已忽略）：{exc}")
            return
        if decoded is None:
            return
        self._dispatch(decoded)

    def _dispatch(self, decoded) -> None:
        if isinstance(decoded, mag_protocol.MagCalStatus):
            self.status = decoded
            self.status_received_at = time.monotonic()
        elif isinstance(decoded, mag_protocol.MagCalBias):
            self.bias = decoded
        elif isinstance(decoded, mag_protocol.MagCalMatrix):
            self.matrix = decoded
        elif isinstance(decoded, mag_protocol.MagCalDraft):
            self.draft = decoded
        elif isinstance(decoded, mag_protocol.MagFusionStatus):
            self.fusion = decoded
        elif isinstance(decoded, mag_protocol.MagCalEvent):
            self._on_magcal_event(decoded)
        elif isinstance(decoded, mag_protocol.MagFrameStatus):
            self.frame = decoded
        elif isinstance(decoded, mag_protocol.MagFrameEvent):
            self._on_magframe_event(decoded)
        elif isinstance(decoded, mag_protocol.MagRawSample):
            self._on_raw_sample(decoded)

    def _on_raw_sample(self, sample: mag_protocol.MagRawSample) -> None:
        if not sample.ok:
            self.live_chip_mgauss = None
            self.live_flu_mgauss = None
            return
        self.live_chip_mgauss = sample.chip_mgauss
        flu = mag_protocol.chip_to_flu_default_mgauss(sample.chip_mgauss)
        self.live_flu_mgauss = flu
        if self.sampling:
            self.samples.append(flu)
            if self.fit_result is not None:
                # 缓冲变了，之前的拟合数字不再对应当前样本集——不能让预览悄悄过期。
                self.fit_result = None
                self.fit_summary_var.set("样本缓冲已变化，请重新拟合。")

    def _on_magcal_event(self, event: mag_protocol.MagCalEvent) -> None:
        state = event.state
        if state == "draft_bias_set":
            self.draft_acks.add("bias")
        elif state == "draft_matrix_row":
            if event.row is not None:
                self.draft_acks.add(f"row{event.row}")
        elif state == "applied_ram":
            self.apply_pending = False
            self.applied = True
            self.notice.set("已应用到 RAM：断电会恢复出厂状态，尚未写入 Flash。")
            self.refresh_status()
        elif state == "apply_rejected":
            self.apply_pending = False
            self.applied = False
            self.notice.set(f"飞控拒绝应用：{event.reason_text}")
        elif state == "armed_blocked":
            self.apply_pending = False
            self.notice.set("飞控已解锁，拒绝改磁力计校准。先上锁再改。")
        elif state == "cleared_ram":
            self.apply_pending = False
            self.applied = False
            self.draft_acks = set()
            self.fit_result = None
            self.notice.set("已恢复出厂未校准状态（RAM 生效，尚未保存到 Flash）。")
            self.refresh_status()
        elif state == "committed":
            self.notice.set("已保存到飞控 Flash，断电不丢。")
            self.refresh_status()
        elif state == "commit_failed":
            self.notice.set(f"写入 Flash 失败：{event.reason_text}")

    def _on_magframe_event(self, event: mag_protocol.MagFrameEvent) -> None:
        if event.state == "verified":
            self.notice.set("轴向验证已在 RAM 生效；和校准系数共用同一次「写入 Flash」落盘。")
            self.refresh_status()
        elif event.state == "armed_blocked":
            self.notice.set("飞控已解锁，拒绝写入轴向验证。先上锁再改。")

    # ------------------------------------------------------------ 采样

    def start_sampling(self) -> None:
        self.sampling = True
        self.sample_notice.set("采样中：请缓慢转动机体，尽量覆盖各个朝向（正装/倒装、四个偏航方向都要转到）。")

    def stop_sampling(self) -> None:
        self.sampling = False
        self.sample_notice.set(f"采样已暂停，缓冲区保留 {len(self.samples)} 个样本。")

    def clear_samples_action(self) -> None:
        self._reset_sampling(note="采样缓冲已清空。")

    # ------------------------------------------------------------ 拟合

    def run_fit(self) -> None:
        if self.coverage is None or not self.coverage.passed:
            return
        try:
            self.fit_result = fit_mag_calibration(self.samples)
        except (MagCalFitError, NumpyRequiredError) as exc:
            self.fit_result = None
            self.fit_summary_var.set(f"拟合被拒绝：{exc}")
            return
        self.fit_summary_var.set("拟合成功，见下方数值；确认无误后再「应用到 RAM」。")

    # ------------------------------------------------------------ 应用/提交/清除

    def apply_to_ram(self) -> None:
        if self.fit_result is None or self.apply_pending:
            return
        if not self._connection():
            self.notice.set("未连接，无法应用。")
            return
        bias = self.fit_result.hard_iron_offset_mgauss
        matrix = self.fit_result.soft_iron_matrix
        commands = [f"MAGCAL SET BIAS {bias[0]:.6f} {bias[1]:.6f} {bias[2]:.6f}"]
        commands += [
            f"MAGCAL SET MATRIX {index} {row[0]:.6f} {row[1]:.6f} {row[2]:.6f}"
            for index, row in enumerate(matrix)
        ]
        commands.append("MAGCAL APPLY")
        self.draft_acks = set()
        self.apply_pending = True
        self.apply_deadline = time.monotonic() + MAG_CAL_REPLY_TIMEOUT_S
        self.notice.set("正在写入草稿并应用到 RAM…")
        for command in commands:
            if not self.send(command):
                self.apply_pending = False
                self.notice.set("发送被拒绝或链路占用，应用已中止。")
                return

    def commit_to_flash(self) -> None:
        if not self.applied or self.apply_pending:
            return
        if not messagebox.askyesno(
            "写入磁力计校准到 Flash",
            "RAM 中的零偏/软磁系数将被写入飞控参数 Flash，断电后仍然生效。\n"
            "写入完成前请勿断电。确定要写入吗？",
        ):
            return
        self.send("MAGCAL COMMIT")
        self.notice.set("已请求写入 Flash，等待飞控确认…")

    def clear_calibration(self) -> None:
        if not messagebox.askyesno(
            "恢复出厂磁力计状态",
            "这会把硬磁/软磁系数和轴向验证状态一起恢复到出厂状态（未校准、未验证），"
            "不存在只退回系数、保留验证状态的选项。仍然只在 RAM 里生效，"
            "需要另外点「写入 Flash」才会持久化。确定要清除吗？",
        ):
            return
        self.send("MAGCAL CLEAR")

    # ------------------------------------------------------------ 轴向验证

    def verify_axis(self) -> None:
        if not self.verify_confirmed_var.get():
            return
        if not messagebox.askyesno(
            "确认贴装轴向",
            "这会把当前默认推导 (x,-y,-z) 标记为已通过物理确认，磁力计才可能参与姿态融合。\n"
            "如果刚才的检查看起来不对，请点「否」并报告，不要确认。",
        ):
            return
        self.send("MAGFRAME VERIFY CONFIRM")
        self.notice.set("已请求写入轴向验证…")

    # ------------------------------------------------------------ 渲染

    def _render(self) -> None:
        self._render_status()
        self._render_fusion_banner()
        self._render_sampling()
        self._render_fit()
        self._render_apply_buttons()
        self._render_axis()

    def _render_status(self) -> None:
        status = self.status
        if status is None:
            for var in self.status_fields.values():
                var.set("—")
            self.status_summary_var.set(
                "当前固件未提供磁力计校准命令族。" if self.unsupported else "尚未读取磁力计校准状态。")
            return
        self.status_fields["calibrated"].set("已校准" if status.calibrated else "未校准（出厂默认）")
        self.status_fields["axis_verified"].set("已验证" if status.axis_verified else "未验证")
        self.status_fields["axis_effective"].set("已验证" if status.axis_effective_verified else "未验证")
        self.status_fields["contract"].set(
            "不一致（存储 {} / 当前 {}，验证已失效）".format(status.contract_stored, status.contract_live)
            if status.contract_mismatch else "一致")
        self.status_fields["dirty"].set("有未保存改动" if status.dirty else "已与 Flash 一致")
        if self.bias is not None:
            bx, by, bz = self.bias.bias_mgauss
            self.status_fields["bias"].set(f"{bx:.3f}, {by:.3f}, {bz:.3f} mgauss")
        if self.matrix is not None:
            self.status_fields["matrix"].set(
                " / ".join("{:.4f},{:.4f},{:.4f}".format(*row) for row in self.matrix.rows))
        if self.status_received_at is None:
            age_text = "—"
            stale = False
        else:
            age = time.monotonic() - self.status_received_at
            age_text = f"{age:.1f} s 前"
            stale = age > (MAG_STATUS_POLL_S * 5)
        summary = f"最近一次状态回读：{age_text}"
        if stale:
            summary += "（已经一段时间没有刷新，可能不是最新状态）"
        if self.protocol_errors:
            summary += f" · 回包解析失败 {self.protocol_errors} 次"
        self.status_summary_var.set(summary)

    def _render_fusion_banner(self) -> None:
        fusion = self.fusion
        status = self.status
        if fusion is None or status is None:
            self.fusion_banner_var.set("磁力计融合状态未知 · 尚未读取。")
            self.fusion_banner_label.configure(style="Muted.TLabel")
            return
        if fusion.participating:
            self.fusion_banner_var.set(
                f"磁力计当前正在参与姿态融合 · 航向误差 {fusion.error_deg:.2f}°")
            self.fusion_banner_label.configure(style="Pass.TLabel")
            return
        reasons: list[str] = []
        if not status.calibrated:
            reasons.append("尚未校准")
        if not status.axis_effective_verified:
            if status.axis_verified == mag_protocol.AXIS_VERIFIED:
                reasons.append("轴向验证因坐标契约版本变化而失效，需要重新验证")
            else:
                reasons.append("轴向尚未验证")
        if fusion.subsystem_enabled and fusion.field_rejected:
            reasons.append("当前场强超出合理范围（可能是干扰或传感器异常）")
        if fusion.subsystem_enabled and fusion.ignored:
            reasons.append("融合库内部拒绝了这一拍磁力计观测")
        if not reasons:
            reasons.append("样本尚未足够新鲜或原因未知，见下方状态与融合诊断")
        self.fusion_banner_var.set("磁力计当前不参与姿态融合 · " + "；".join(reasons))
        self.fusion_banner_label.configure(style="Fail.TLabel")

    def _render_sampling(self) -> None:
        count = len(self.samples)
        self.sample_start_button.configure(state=tk.DISABLED if self.sampling else tk.NORMAL)
        self.sample_stop_button.configure(state=tk.NORMAL if self.sampling else tk.DISABLED)
        self.sample_count_var.set(
            f"已采集 {count} 个样本（拟合建议 ≥ {DEFAULT_THRESHOLDS.min_samples}）"
            + ("　· 采样中" if self.sampling else ""))
        if self.live_flu_mgauss is not None:
            fx, fy, fz = self.live_flu_mgauss
            self.live_reading_var.set(f"FLU {fx:.1f}, {fy:.1f}, {fz:.1f} mgauss")
        else:
            self.live_reading_var.set("—")

        if not NUMPY_AVAILABLE:
            self.coverage_var.set("未安装 numpy，无法计算覆盖度或拟合（pip install numpy）。")
            self.coverage_label.configure(style="Fail.TLabel")
            self.fit_button.configure(state=tk.DISABLED)
            return
        if count < DEFAULT_THRESHOLDS.min_samples:
            self.coverage = None
            self.coverage_var.set(
                f"样本不足，还差 {DEFAULT_THRESHOLDS.min_samples - count} 个。")
            self.coverage_label.configure(style="Muted.TLabel")
            self.fit_button.configure(state=tk.DISABLED)
            return
        try:
            self.coverage = evaluate_coverage(self.samples)
        except (MagCalFitError, NumpyRequiredError) as exc:
            self.coverage = None
            self.coverage_var.set(f"覆盖度暂时无法计算：{exc}")
            self.coverage_label.configure(style="Fail.TLabel")
            self.fit_button.configure(state=tk.DISABLED)
            return
        ok = self.coverage.passed
        self.coverage_var.set(
            f"象限覆盖 {self.coverage.occupied_octants}/8 · "
            f"方向均匀度 {self.coverage.min_eigen_fraction:.2f} · "
            + ("覆盖已足够，可以拟合" if ok else "；".join(self.coverage.findings)))
        self.coverage_label.configure(style="Pass.TLabel" if ok else "Fail.TLabel")
        self.fit_button.configure(state=tk.NORMAL if ok else tk.DISABLED)

    def _render_fit(self) -> None:
        result = self.fit_result
        if result is None:
            self.fit_verdict_var.set("")
            self.apply_button.configure(state=tk.DISABLED)
            return
        bx, by, bz = result.hard_iron_offset_mgauss
        self.fit_summary_var.set(
            f"拟合半径 {result.fitted_radius_mgauss:.1f} mgauss · "
            f"零偏 {bx:.2f},{by:.2f},{bz:.2f} mgauss · 行列式 {result.determinant:.3f} · "
            f"条件数 {result.condition_number:.2f} · 校正后 RMS {result.corrected_rms_mgauss:.2f} mgauss "
            f"（最大误差 {result.corrected_max_error_mgauss:.2f}）")
        problems = fit_problems(result)
        if problems:
            self.fit_verdict_var.set("不能用：" + "；".join(problems))
            self.fit_verdict_label.configure(style="Fail.TLabel")
        else:
            self.fit_verdict_var.set("可以用：半径在飞控接受范围内，残差正常。")
            self.fit_verdict_label.configure(style="Pass.TLabel")
        self.apply_button.configure(state=tk.DISABLED if self.apply_pending else tk.NORMAL)

    def _render_apply_buttons(self) -> None:
        self.commit_button.configure(
            state=tk.NORMAL if (self.applied and not self.apply_pending) else tk.DISABLED)
        self.clear_button.configure(state=tk.DISABLED if self.apply_pending else tk.NORMAL)
        if self.apply_pending:
            steps = (("bias", "零偏"), ("row0", "矩阵行0"), ("row1", "矩阵行1"), ("row2", "矩阵行2"))
            progress = " ".join(
                f"{label}{'✓' if key in self.draft_acks else '…'}" for key, label in steps)
            self.apply_progress_var.set(f"草稿写入进度：{progress}")
        else:
            self.apply_progress_var.set("")

    def _render_axis(self) -> None:
        frame = self.frame
        if frame is None:
            self.axis_summary_var.set("尚未读取轴向验证状态。")
        else:
            text = "已验证" if frame.axis_effective_verified else "未验证"
            if frame.axis_verified == mag_protocol.AXIS_VERIFIED and not frame.axis_effective_verified:
                text += (f"（存储值曾经验证，但坐标契约版本从 {frame.contract_stored} "
                        f"变为 {frame.contract_live}，已失效，需要重新验证）")
            self.axis_summary_var.set(text)
        self.verify_button.configure(
            state=tk.NORMAL if (self.verify_confirmed_var.get() and not self.apply_pending)
            else tk.DISABLED)


def mount_mag_cal(panel, parent):
    panel.mag_cal_page = MagCalPage(parent, panel)
    panel.mag_cal_page.pack(fill=tk.BOTH, expand=True)


__all__ = ["MAG_CAL_TAB_TEXT", "MagCalPage", "mount_mag_cal"]
