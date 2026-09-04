"""“示波器 / 调参”页：高帧率波形 + 滑块三态回显。

这一页要替掉的是"面板管命令、Synex 管波形"的两窗口工作方式。数据路径：

    固件掩码帧 -> transport 二进制分支 -> (收线程) TelemDecoder -> TelemRing
                                                                    |
                                        (Tk 线程, 33 ms 一次) 只读快照 -> ScopeCanvas

**收线程写、Tk 线程只读快照**是这一页的骨架：解码和入环都发生在收线程里，
Tk 只在重绘时抓一份拷贝。这样波形的时间轴由固件的 `t_us` 决定，不会被 Tk 的
事件循环节奏牵着走；反过来，界面再卡也不会丢数据。

可见性门控沿用 `flow_monitor.py` 的范式：本页选中才发 `TELEM STREAM on`，
切走立刻 `off`。流不常开是有意的——40 Hz 的帧占数传 57% 的带宽，不看波形的
时候把它留给命令回复。

关于 `drone_tcp_panel.py` 的挂载：那个文件只减不增，本页只允许在它里面留
Mixin 挂载与页签注册。所以页签的创建、注册、构建都收在 `_scope_mount()` 里，
可见性判断与轮询收在 `_scope_poll_tick()` 里，面板侧只剩调用。
"""

from __future__ import annotations

import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import ttk

from ..proto import PROTO_REQ_PARAM_SET, parse_kv
from ..scope import (
    SCOPE_RENDER_PERIOD_MS,
    SCOPE_WINDOW_CHOICES_S,
    ScopeCanvas,
)
from ..telem_stream import TelemDecoder, TelemRing, TelemSchema


try:  # 三段回退与面板其它页一致：包内 / tools 包 / 直接跑脚本
    from ...project_paths import TELEMETRY_DIR, dated_directory, ensure_directory
except ImportError:  # pragma: no cover - 取决于调用方的 sys.path
    try:
        from tools.project_paths import TELEMETRY_DIR, dated_directory, ensure_directory
    except ImportError:
        from project_paths import TELEMETRY_DIR, dated_directory, ensure_directory


# 环形缓冲：60 s × 40 Hz = 2400 个样本/通道，够看一分钟历史。
SCOPE_RING_CAPACITY = 2400

# 拖动中的写入节流。5 Hz 是"跟手"和"不把数传写满"之间的折中：每次
# PARAM SET 固件都会回 PARAM 记录 + PID legacy 长文本，拖动时不节流会把
# 命令回复和遥测帧一起挤爆 uartTxQueue（深度 32、满时丢最旧的一条）。
SCOPE_SLIDER_THROTTLE_S = 0.2

# 回显判定的相对容差。固件按 1e-3 的精度格式化，1e-4 的相对容差足以区分
# "固件收下了"和"固件钳位/拒绝了"。
SCOPE_ECHO_TOLERANCE = 1e-4

# 发送后多久之内的"不一致回显"不算 diverged。要盖住"发送前编码、发送后才到"
# 的在线帧：数传上一帧全量刷新 137 B ≈ 24 ms，再加一个 25 ms 的遥测拍。
SCOPE_ECHO_GRACE_S = 0.10

SCOPE_STATE_IDLE = "idle"
SCOPE_STATE_PENDING = "pending"
SCOPE_STATE_CONFIRMED = "confirmed"
SCOPE_STATE_DIVERGED = "diverged"

# 一次最多画几条曲线。再多颜色就分不开了，而且每条都要往 Tcl 里灌坐标。
SCOPE_MAX_CURVES = 8


class ScopePageMixin:
    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------

    def _init_scope_state(self) -> None:
        """控件引用 + 数据状态。只在挂载时调一次。"""
        self.scope_tab = None
        self.scope_channel_vars: dict[int, tk.BooleanVar] = {}
        self.scope_slider_vars: dict[int, tk.DoubleVar] = {}
        self.scope_slider_labels: dict[int, ttk.Label] = {}
        self.scope_stat_vars: dict[str, tk.StringVar] = {}
        self.scope_canvas: ScopeCanvas | None = None
        self.scope_window_var = None
        self.scope_paused_var = None
        self.scope_record_rows: deque = deque(maxlen=20000)
        self.scope_record_handle = None
        self.scope_record_path = None
        self.scope_record_var = None
        self.scope_rate_window: deque = deque(maxlen=64)
        self.scope_last_render_ns = 0
        self.scope_channel_tree = None
        self.scope_slider_host = None
        self.scope_hint_var = None
        self._scope_reset_session()

    def _scope_reset_session(self) -> None:
        """把这条链路的数据状态清干净，**保留已经建好的控件**。

        断开重连、或者换了一台通道表不同的飞控时必须整体清：留着上一台的
        schema 继续解，得到的是一堆长度合法但含义错位的曲线——正是掩码帧
        要消灭的那种失败。
        """
        self.scope_tab_visible = False
        self.scope_stream_requested = False
        self.scope_schema = TelemSchema()
        self.scope_decoder = TelemDecoder(schema_hash=None)
        self.scope_ring = TelemRing(capacity=SCOPE_RING_CAPACITY)
        self.scope_schema_pending_from: int | None = None
        self.scope_schema_reload_requested = False
        self.scope_selected: set[int] = set()
        self.scope_curve_keys: list[int] = []
        self.scope_slider_state: dict[int, str] = {}
        self.scope_slider_sent: dict[int, float] = {}
        self.scope_slider_last_send: dict[int, float] = {}
        self.scope_stream_status: dict[str, str] = {}
        self.scope_frames_seen = 0
        self.scope_measured_hz = 0.0
        self.scope_rate_window.clear()

    # ------------------------------------------------------------------
    # 挂载（drone_tcp_panel.py 里只留一行调用）
    # ------------------------------------------------------------------

    def _scope_mount(self, notebook: ttk.Notebook) -> None:
        # 状态初始化收在这里而不是让面板再加一行 `_init_scope_state()`：
        # `drone_tcp_panel.py` 只减不增，本页的挂载预算要花在刀刃上。
        self._init_scope_state()
        frame = ttk.Frame(notebook, padding=14, style="Page.TFrame")
        self.scope_tab = frame
        notebook.add(frame, text="示波器 / 调参")
        self._build_scope_page(frame)
        self.after(SCOPE_RENDER_PERIOD_MS, self._scope_render_tick)

    def _build_scope_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="TELEM  /  实时波形与调参", style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text="自描述掩码帧示波器", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=(
                "本页选中才开流（TELEM STREAM on），切走立刻关——40 Hz 的默认配置占数传 57% 的带宽，"
                "不看波形时把它留给命令回复。通道勾选直接改帧掩码，帧自己带掩码和通道表指纹，"
                "所以切换的瞬间不会错位；固件换了通道表，指纹一变本页会自动重拉表重建。"
                "增益滑块由通道表的 param 字段生成，平时不占带宽，只有值变了才回显一帧。"
            ),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))

        self._build_scope_stats(parent)

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=1)
        panes.add(right, weight=3)

        self._build_scope_channel_tree(left)
        self._build_scope_sliders(left)
        self._build_scope_plot(right)

    def _build_scope_stats(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="链路统计", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        rows = (
            ("sink", "当前出口"),
            ("rate", "实测帧率"),
            ("gap", "seq 缺口"),
            ("drop", "固件丢帧"),
            ("schema", "通道表指纹"),
            ("reject", "本机拒帧"),
        )
        for column, (key, label) in enumerate(rows):
            self.scope_stat_vars[key] = tk.StringVar(value="-")
            ttk.Label(box, text=label, style="Muted.TLabel").grid(
                row=0, column=column * 2, sticky=tk.W, padx=(0 if column == 0 else 12, 4)
            )
            ttk.Label(box, textvariable=self.scope_stat_vars[key], style="Mono.TLabel").grid(
                row=0, column=column * 2 + 1, sticky=tk.W
            )

    def _build_scope_channel_tree(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="通道（勾选即改帧掩码）", padding=8)
        box.pack(fill=tk.BOTH, expand=True)
        tree = ttk.Treeview(box, columns=("unit",), show="tree headings", height=14)
        tree.heading("#0", text="通道")
        tree.heading("unit", text="单位")
        tree.column("unit", width=70, anchor=tk.W)
        tree.pack(fill=tk.BOTH, expand=True)
        tree.bind("<Button-1>", self._scope_on_tree_click)
        self.scope_channel_tree = tree
        self.scope_hint_var = tk.StringVar(
            value="尚未取到通道表；本页选中并连上飞控后会自动拉取。"
        )
        # 点了没反应是最难查的界面问题之一，所以到上限时要说出来，
        # 而不是让那一下点击悄悄消失。
        ttk.Label(
            box, textvariable=self.scope_hint_var,
            style="Muted.TLabel", wraplength=280,
        ).pack(fill=tk.X, pady=(6, 0))

    def _build_scope_sliders(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="增益（三态回显）", padding=8)
        box.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        host = ttk.Frame(box)
        host.pack(fill=tk.BOTH, expand=True)
        host.columnconfigure(1, weight=1)
        self.scope_slider_host = host
        ttk.Label(
            box,
            text=(
                "pending=已发出等回显；confirmed=固件回显与发送值一致；"
                "diverged=固件钳位或拒绝，滑块已跳到固件实际值。"
            ),
            style="Muted.TLabel", wraplength=280, justify=tk.LEFT,
        ).pack(fill=tk.X, pady=(6, 0))

    def _build_scope_plot(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="波形", padding=8)
        box.pack(fill=tk.BOTH, expand=True)

        controls = ttk.Frame(box)
        controls.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(controls, text="时间窗口", style="Muted.TLabel").pack(side=tk.LEFT)
        self.scope_window_var = tk.StringVar(value="10")
        window = ttk.Combobox(
            controls, textvariable=self.scope_window_var, width=6, state="readonly",
            values=[f"{choice:g}" for choice in SCOPE_WINDOW_CHOICES_S],
        )
        window.pack(side=tk.LEFT, padx=(6, 12))
        window.bind("<<ComboboxSelected>>", self._scope_on_window_changed)

        self.scope_paused_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            controls, text="暂停", variable=self.scope_paused_var,
            command=self._scope_on_pause_toggled,
        ).pack(side=tk.LEFT)

        ttk.Button(
            controls, text="清空缓冲", command=self._scope_clear_buffer,
            style="Secondary.TButton",
        ).pack(side=tk.LEFT, padx=(12, 0))

        self.scope_record_var = tk.StringVar(value="未录制")
        ttk.Button(
            controls, text="录制 CSV", command=self._scope_toggle_record,
            style="Secondary.TButton",
        ).pack(side=tk.LEFT, padx=(12, 6))
        ttk.Label(controls, textvariable=self.scope_record_var, style="Mono.TLabel").pack(side=tk.LEFT)

        self.scope_canvas = ScopeCanvas(box)
        self.scope_canvas.canvas.pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------------------------
    # 轮询与可见性门控
    # ------------------------------------------------------------------

    def _scope_poll_tick(self, now: float) -> None:
        """本页自己的可见性门控（从 drone_tcp_panel._imu_poll_tick 搬出来的部分）。"""
        tab = getattr(self, "scope_tab", None)
        visible = tab is not None and self.notebook.select() == str(tab)
        connected = self._transport_connected()
        if visible != self.scope_tab_visible:
            self.scope_tab_visible = visible
            self._scope_sync_stream(visible)
        elif visible and connected and not self._scope_stream_attached():
            # 可见性没变但链路变了：先开页再连线，或者拔插后重连。固件在 USB
            # 出口下拔线会自己 stream=0，重连后不再发一次 STREAM on 就永远没波形；
            # 光盯着可见性的翻转看不见这两种情况（审核复现：test_scope_page_link_lifecycle）。
            self._scope_sync_stream(True)
        if not (visible and connected):
            if not connected:
                self.scope_stream_requested = False
            return
        if not self.scope_schema.complete and self.scope_schema_pending_from is None:
            self._scope_request_schema()
        elif self.scope_decoder.needs_schema_reload and not self.scope_schema_reload_requested:
            # 固件通道表变了。继续解只会得到一堆错位的曲线，所以整表重拉。
            self.scope_schema_reload_requested = True
            self._scope_request_schema()

    def _scope_stream_attached(self) -> bool:
        """本页是否已在**当前** transport 上开了流并挂上二进制 sink。"""
        if not self.scope_stream_requested:
            return False
        transport = getattr(self, "transport", None)
        return getattr(transport, "_binary_sink", None) is not None or (
            getattr(transport, "binary_sink", None) is not None
        )

    def _scope_sync_stream(self, active: bool) -> None:
        if not self._transport_connected():
            self.scope_stream_requested = False
            return
        command = "TELEM STREAM on" if active else "TELEM STREAM off"
        if not self._validation_command_allowed(command):
            return
        self.scope_stream_requested = active
        if active:
            self.transport.set_binary_sink(self._scope_on_binary_frame)
        self.transport.send_line(command)
        if not active:
            self.transport.set_binary_sink(None)

    def _scope_request_schema(self) -> None:
        self.scope_schema = TelemSchema()
        self.scope_schema_pending_from = 0
        for command in ("TELEM?", "TELEM CH from=0"):
            if self._validation_command_allowed(command):
                self.transport.send_line(command)

    # ------------------------------------------------------------------
    # 文本回包
    # ------------------------------------------------------------------

    def _scope_handle_line(self, line: str) -> None:
        if not hasattr(self, "scope_schema"):
            return
        if line.startswith("TELEM STREAM "):
            self.scope_stream_status = parse_kv(line)
            return
        if not self.scope_schema.feed_line(line):
            return
        if line.startswith("TELEM PAGE "):
            next_page = self.scope_schema.next_page
            self.scope_schema_pending_from = next_page
            if next_page is not None and self._transport_connected():
                command = f"TELEM CH from={next_page}"
                if self._validation_command_allowed(command):
                    self.transport.send_line(command)
                return
            if self.scope_schema.complete:
                self._scope_adopt_schema()

    def _scope_adopt_schema(self) -> None:
        """通道表齐了：绑定解码器、重建通道树和滑块。"""
        self.scope_schema_reload_requested = False
        self.scope_decoder.bind_schema(self.scope_schema)
        self.scope_ring = TelemRing(
            capacity=SCOPE_RING_CAPACITY,
            channel_count=max(self.scope_schema.channel_count, 1),
        )
        if not self.scope_selected:
            # 默认勾上非参数通道里的前几条，别一上来就是空白画布。
            candidates = [c.index for c in self.scope_schema.ordered() if not c.is_parameter]
            self.scope_selected = set(candidates[:SCOPE_MAX_CURVES])
        self._scope_rebuild_channel_tree()
        self._scope_rebuild_sliders()
        self._scope_apply_curves()

    def _scope_rebuild_channel_tree(self) -> None:
        tree = self.scope_channel_tree
        if tree is None:
            return
        for item in tree.get_children(""):
            tree.delete(item)
        self.scope_channel_vars = {}
        groups: dict[str, str] = {}
        for channel in self.scope_schema.ordered():
            if channel.is_parameter:
                continue        # 增益走滑块，不进曲线勾选树
            if channel.group not in groups:
                groups[channel.group] = tree.insert("", tk.END, text=channel.group, open=True)
            mark = "☑" if channel.index in self.scope_selected else "☐"
            tree.insert(
                groups[channel.group], tk.END, iid=str(channel.index),
                text=f"{mark} {channel.name}", values=(channel.unit,),
            )
        self._scope_set_hint(
            f"已选 {len(self.scope_selected)} / {SCOPE_MAX_CURVES} 条曲线。"
        )

    def _scope_set_hint(self, text: str) -> None:
        if self.scope_hint_var is not None:
            self.scope_hint_var.set(text)

    def _scope_on_tree_click(self, event: tk.Event) -> None:
        tree = self.scope_channel_tree
        if tree is None:
            return
        item = tree.identify_row(event.y)
        if not item or not item.isdigit():
            return
        self._scope_toggle_channel(int(item))

    def _scope_toggle_channel(self, index: int) -> None:
        if index in self.scope_selected:
            self.scope_selected.discard(index)
        elif len(self.scope_selected) < SCOPE_MAX_CURVES:
            self.scope_selected.add(index)
        else:
            self._scope_set_hint(
                f"最多同时显示 {SCOPE_MAX_CURVES} 条曲线，先取消一条再勾选。"
            )
            return
        self._scope_rebuild_channel_tree()
        self._scope_apply_curves()
        self._scope_send_mask()

    def _scope_mask(self) -> int:
        """曲线通道 + 全部参数通道。

        参数通道始终在掩码里但平时不置位（固件按变化回显），把它们留在掩码里
        才能让 1 Hz 的全量刷新帧把滑块喂回来；剔出去的话滑块会永远停在初值。
        """
        mask = 0
        for index in self.scope_selected:
            mask |= 1 << index
        for channel in self.scope_schema.ordered():
            if channel.is_parameter:
                mask |= 1 << channel.index
        return mask

    def _scope_send_mask(self) -> None:
        if not self._transport_connected():
            return
        command = f"TELEM MASK {self._scope_mask():X}"
        if self._validation_command_allowed(command):
            self.transport.send_line(command)

    def _scope_apply_curves(self) -> None:
        if self.scope_canvas is None:
            return
        names = {c.index: c.name for c in self.scope_schema.ordered()}
        self.scope_curve_keys = sorted(self.scope_selected)
        self.scope_canvas.set_curves(
            [(index, names.get(index, f"ch{index}")) for index in self.scope_curve_keys]
        )

    # ------------------------------------------------------------------
    # 滑块三态
    # ------------------------------------------------------------------

    def _scope_rebuild_sliders(self) -> None:
        host = self.scope_slider_host
        if host is None:
            return
        for child in host.winfo_children():
            child.destroy()
        self.scope_slider_vars = {}
        self.scope_slider_labels = {}
        self.scope_slider_state = {}
        self.scope_slider_sent = {}
        self.scope_slider_last_send = {}

        row = 0
        for channel in self.scope_schema.ordered():
            if not channel.is_parameter:
                continue
            variable = tk.DoubleVar(value=0.0)
            self.scope_slider_vars[channel.index] = variable
            self.scope_slider_state[channel.index] = SCOPE_STATE_IDLE
            ttk.Label(host, text=channel.name).grid(row=row, column=0, sticky=tk.W, padx=(0, 8))
            scale = ttk.Scale(
                host, from_=channel.minimum, to=channel.maximum,
                variable=variable, orient=tk.HORIZONTAL,
                command=lambda _value, key=channel.index: self._scope_slider_dragged(key),
            )
            scale.grid(row=row, column=1, sticky=tk.EW)
            scale.bind(
                "<ButtonRelease-1>",
                lambda _event, key=channel.index: self._scope_slider_released(key),
            )
            label = ttk.Label(host, text="—", style="Mono.TLabel", width=18)
            label.grid(row=row, column=2, sticky=tk.W, padx=(8, 0))
            self.scope_slider_labels[channel.index] = label
            row += 1

    def _scope_slider_dragged(self, index: int) -> None:
        """拖动中节流到 5 Hz。松手那一下由 `_scope_slider_released` 保证必发。"""
        now = time.monotonic()
        last = self.scope_slider_last_send.get(index, 0.0)
        if now - last < SCOPE_SLIDER_THROTTLE_S:
            return
        self._scope_send_param(index, now)

    def _scope_slider_released(self, index: int) -> None:
        # 松手必发：节流窗口内的最后一次拖动否则会被吃掉，滑块停在 A、
        # 固件停在 B，而且没有任何提示。
        self._scope_send_param(index, time.monotonic())

    def _scope_send_param(self, index: int, now: float) -> None:
        channel = self.scope_schema.channels.get(index)
        variable = self.scope_slider_vars.get(index)
        if channel is None or variable is None or not channel.is_parameter:
            return
        value = float(variable.get())
        payload = f"PARAM SET {channel.param} {value:.6g}"
        if not self._transport_connected():
            return
        if not self._validation_command_allowed(payload):
            return
        self.scope_slider_last_send[index] = now
        self.scope_slider_sent[index] = value
        self.scope_slider_state[index] = SCOPE_STATE_PENDING
        self._scope_refresh_slider_label(index, value)
        self.transport.send_frame(PROTO_REQ_PARAM_SET, payload.encode("utf-8"))

    def _scope_note_param_echo(self, index: int, echoed: float,
                               now: float | None = None) -> None:
        """遥测帧带回来的参数值：判定 confirmed / diverged。"""
        variable = self.scope_slider_vars.get(index)
        if variable is None:
            return
        if now is None:
            now = time.monotonic()
        sent = self.scope_slider_sent.get(index)
        state = self.scope_slider_state.get(index, SCOPE_STATE_IDLE)
        since_send = now - self.scope_slider_last_send.get(index, -1e9)
        if state != SCOPE_STATE_PENDING or sent is None:
            # 用户还在拖（节流窗口内刚发过）：这时把滑块拽回固件上一拍的值，
            # 手感就是"滑块往回蹦"。等节流窗口过去、松手那一发出去再跟随。
            if state != SCOPE_STATE_PENDING and since_send >= SCOPE_SLIDER_THROTTLE_S:
                variable.set(echoed)
            self._scope_refresh_slider_label(index, echoed)
            return

        scale = max(abs(sent), abs(echoed), 1.0)
        if abs(echoed - sent) <= SCOPE_ECHO_TOLERANCE * scale:
            self.scope_slider_state[index] = SCOPE_STATE_CONFIRMED
        elif since_send < SCOPE_ECHO_GRACE_S:
            # 刚发出去：这条旧值可能来自发送之前就已编码、还在线上的帧
            # （数传 57600 上一帧全量刷新 137 B 要跑 24 ms）。宽限期内不判，
            # 再等下一帧；宽限期过了还是旧值，才是固件真的拒了。
            self._scope_refresh_slider_label(index, sent)
            return
        else:
            # 固件钳位或拒绝了。滑块跳到固件的实际值——留在用户拖到的位置上
            # 等于让界面撒谎，而那个谎正好发生在参数没生效的时候。
            self.scope_slider_state[index] = SCOPE_STATE_DIVERGED
            variable.set(echoed)
        self._scope_refresh_slider_label(index, echoed)

    def _scope_refresh_slider_label(self, index: int, value: float) -> None:
        label = self.scope_slider_labels.get(index)
        if label is None:
            return
        state = self.scope_slider_state.get(index, SCOPE_STATE_IDLE)
        style = {
            SCOPE_STATE_CONFIRMED: "Pass.TLabel",
            SCOPE_STATE_DIVERGED: "Fail.TLabel",
        }.get(state, "Mono.TLabel")
        label.configure(text=f"{value:.4g}  {state}", style=style)

    # ------------------------------------------------------------------
    # 二进制帧（收线程）
    # ------------------------------------------------------------------

    def _scope_on_binary_frame(self, function: int, payload: bytes) -> None:
        """transport 收线程直接调用：解码、入环、记 CSV 行。

        故意不经过 `rx_queue`：那条队列由 Tk 主循环按批抽干，40 Hz~1 kHz 的帧
        走那里会让波形跟着界面卡顿走样。这里只碰自己的环形缓冲和两个 deque。
        """
        del function
        samples = self.scope_decoder.feed(payload)
        if not samples:
            return
        self.scope_ring.push_many(samples)
        self.scope_frames_seen += 1
        self.scope_rate_window.append(time.monotonic())
        if self.scope_record_handle is not None:
            for sample in samples:
                self.scope_record_rows.append(sample)

    # ------------------------------------------------------------------
    # 渲染
    # ------------------------------------------------------------------

    def _scope_render_tick(self) -> None:
        try:
            if self.scope_tab_visible and self.scope_canvas is not None:
                self._scope_draw()
            self._scope_flush_record()
        finally:
            self.after(SCOPE_RENDER_PERIOD_MS, self._scope_render_tick)

    def _scope_draw(self) -> None:
        canvas = self.scope_canvas
        if canvas is None:
            return
        canvas.set_window(float(self.scope_window_var.get() or 10.0))
        canvas.set_paused(bool(self.scope_paused_var.get()))
        series = {index: self.scope_ring.snapshot(index) for index in self.scope_curve_keys}
        canvas.render(series)
        self._scope_sync_param_echo()
        self._scope_refresh_stats()

    def _scope_sync_param_echo(self) -> None:
        for index in list(self.scope_slider_vars):
            times, values = self.scope_ring.snapshot(index)
            if values.size == 0:
                continue
            self._scope_note_param_echo(index, float(values[-1]))
            del times

    def _scope_refresh_stats(self) -> None:
        if not self.scope_stat_vars:
            return
        stamps = list(self.scope_rate_window)
        if len(stamps) >= 2 and stamps[-1] > stamps[0]:
            self.scope_measured_hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        status = self.scope_stream_status
        stats = self.scope_decoder.stats
        self.scope_stat_vars["sink"].set(status.get("active", "-"))
        self.scope_stat_vars["rate"].set(f"{self.scope_measured_hz:.1f} Hz")
        self.scope_stat_vars["gap"].set(f"{stats.seq_gaps} 次 / 丢 {stats.lost_frames} 帧")
        self.scope_stat_vars["drop"].set(status.get("drop", "-"))
        self.scope_stat_vars["schema"].set(
            f"{self.scope_schema.computed_hash():08X}" if self.scope_schema.complete else "-"
        )
        unclaimed = getattr(self.transport, "binary_unclaimed", 0)
        self.scope_stat_vars["reject"].set(
            f"{stats.rejected_total} 帧 / 未接手 {unclaimed}"
        )

    def _scope_on_window_changed(self, _event=None) -> None:
        if self.scope_canvas is not None:
            self.scope_canvas.set_window(float(self.scope_window_var.get() or 10.0))

    def _scope_on_pause_toggled(self) -> None:
        if self.scope_canvas is not None:
            self.scope_canvas.set_paused(bool(self.scope_paused_var.get()))

    def _scope_clear_buffer(self) -> None:
        """只清本地缓冲，一帧都不发——固件那边没有"缓冲"可清。"""
        self.scope_ring.clear()
        self.scope_rate_window.clear()
        self.scope_frames_seen = 0

    # ------------------------------------------------------------------
    # CSV 录制
    # ------------------------------------------------------------------

    def _scope_toggle_record(self) -> None:
        if self.scope_record_handle is not None:
            self._scope_stop_record()
            return
        if not self.scope_schema.complete:
            self.scope_record_var.set("还没取到通道表")
            return
        directory = ensure_directory(dated_directory(TELEMETRY_DIR))
        stamp = datetime.now().strftime("%H%M%S")
        path = directory / f"telem_{stamp}.csv"
        handle = path.open("w", encoding="utf-8", newline="")
        channels = self.scope_schema.ordered()
        # 表头带上通道名与指纹：没有指纹的话，事后没人能确定这份 CSV 是哪张
        # 通道表下录的，列名就成了无法验证的说法。
        handle.write(f"# schema_hash={self.scope_schema.computed_hash():08X}\n")
        handle.write("t_s," + ",".join(channel.name for channel in channels) + "\n")
        self.scope_record_handle = handle
        self.scope_record_path = path
        self.scope_record_rows.clear()
        self.scope_record_var.set(str(path.name))

    def _scope_stop_record(self) -> None:
        self._scope_flush_record()
        if self.scope_record_handle is not None:
            self.scope_record_handle.close()
        self.scope_record_handle = None
        self.scope_record_var.set("未录制")

    def _scope_flush_record(self) -> None:
        handle = self.scope_record_handle
        if handle is None:
            return
        count = self.scope_schema.channel_count
        while self.scope_record_rows:
            sample = self.scope_record_rows.popleft()
            cells = [f"{sample.t_us * 1e-6:.6f}"]
            # 本帧没带的通道留空，不补 0：补 0 会让"这一帧没发这条通道"和
            # "这条通道的值就是 0"变得不可区分。
            cells += [
                f"{sample.values[index]:.6g}" if index in sample.values else ""
                for index in range(count)
            ]
            handle.write(",".join(cells) + "\n")


__all__ = [
    "SCOPE_ECHO_GRACE_S",
    "SCOPE_ECHO_TOLERANCE",
    "SCOPE_MAX_CURVES",
    "SCOPE_RING_CAPACITY",
    "SCOPE_SLIDER_THROTTLE_S",
    "SCOPE_STATE_CONFIRMED",
    "SCOPE_STATE_DIVERGED",
    "SCOPE_STATE_IDLE",
    "SCOPE_STATE_PENDING",
    "ScopePageMixin",
]
