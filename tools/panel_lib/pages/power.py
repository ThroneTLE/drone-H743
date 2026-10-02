"""电源页：实时电压 / 电流走遥测流推送，配置与诊断走低频问答。

合并自原来的「电池电压」与「电流计」两页。两页都在每 2 s 各发一条命令
（`BATTERY? <nonce>` / `STATUS?`）取同一件事的两半，而且**两页都显示电流**——
同一个物理量在同一个分组里有两个可能不一致的读数。

页面因此分成两半，判据是"这个数据该多快"：

* **实时区**（总压 / 电流 / 平均单节 / 功率）—— 跟着遥测流走。通道 `batt_v`
  `batt_i` 由仲裁器（`telem_subscription.py`）登记进掩码，页面可见才订阅、
  不可见就退订，不占用不看的时候的带宽。**没有任何定时轮询。**
* **配置与诊断区**（串数 / 阈值 / ADC 计数 / ELRS 统计）—— 打开页面取一次，
  之后靠手动「刷新诊断」。这一半的数据是低频的，`BATTERY SET` 的 nonce 问答
  确认更是没法用推送替代：写入确认必须能对上是哪一次写入。

无效语义：固件在电压 / 电流无效或过期时发 **NaN**，不发 0。实时区遇到 NaN
显示 `—` 并把原因指到诊断区的 ADC 状态上——显示 0 会让"没读到"看起来像
"真的是 0 A"，那是这一页最危险的一种谎。
"""

from __future__ import annotations

import math
import secrets
import time
import tkinter as tk
from tkinter import ttk

from ..battery_monitor import decode_battery
from ..connection_state import receive_context
from ..current_monitor import ADC_STATUS, UINT32_MAX, parse_current
from ..proto import PROTO_REQ_STATUS
from ..telem_subscription import TELEM_OWNER_POWER


POWER_TAB_TEXT = "电源"

#: 实时区订阅的通道。名字是契约，编号不是——固件复用了 reserved_7 / reserved_8
#: 的槽位，按编号硬绑会在下一次通道表变动时静默绑错。
POWER_CHANNEL_VOLTAGE = "batt_v"
POWER_CHANNEL_CURRENT = "batt_i"
POWER_CHANNELS = (POWER_CHANNEL_VOLTAGE, POWER_CHANNEL_CURRENT)

#: 界面刷新周期。这**不是轮询**：它一条命令都不发，只把环形缓冲里的最新值画出来
#: 并检查可见性。原来那两个 2 秒轮询定时器已经删掉。
POWER_RENDER_MS = 250

#: 多久没有遥测帧就认为实时区的数据不能再显示。40 Hz 的流断 1.5 s 等于丢 60 帧，
#: 不是抖动是真的停了。
POWER_TELEM_STALE_S = 1.5

#: 问答回包的新鲜度上限（与原电池页一致）；超过就不许拿它做配置写入的前提。
POWER_REPLY_STALE_S = 3.0

#: nonce 问答的等待上限。
POWER_REPLY_TIMEOUT_S = 2.0

#: "打开页面取一次"失败之后的重试间隔。这**不是轮询**：它在第一次成功之后就
#: 彻底停下，只为覆盖"打开页面那一刻链路正好被日志导出占着"这种情况。取 5 s 是
#: 为了让被拒的链路上不出现按链路速度空转的重试。
POWER_FETCH_RETRY_S = 5.0


def average_cell_volts(total_volts: float, cells: int) -> float | None:
    """平均单节 = 总压 / 串数。**不是逐节测量**，串数错了这个数就错了。

    串数来自下半区的低频回读，不为它单开一条遥测通道：它一次运行里几乎不变，
    为一个常量占 40 Hz 的带宽不划算。
    """
    if cells is None or cells <= 0 or total_volts is None:
        return None
    if not math.isfinite(total_volts):
        return None
    return total_volts / cells


def electrical_power_watts(volts: float | None, amps: float | None) -> float | None:
    """P = V × I。两边都得是有限值——少一边就是未知，不是 0 W。"""
    if volts is None or amps is None:
        return None
    if not (math.isfinite(volts) and math.isfinite(amps)):
        return None
    return volts * amps


class _TelemStamp:
    """最近一帧遥测的到达证据。**在收线程里写**，Tk 线程只读。

    只记时刻和来源身份，不解码：解码已经由工作台那个消费者做了，值从同一个环形
    缓冲里取。这里要回答的是另一个问题——"这一帧是不是**当前这条链路**刚送来的"。
    """

    __slots__ = ("at", "transport", "generation")

    def __init__(self, at: float, transport, generation) -> None:
        self.at = at
        self.transport = transport
        self.generation = generation

    def is_current(self, transport) -> bool:
        return (self.transport is transport
                and self.generation == getattr(transport, "connection_generation", 0))


class PowerPage(ttk.Frame):
    def __init__(self, parent, panel):
        super().__init__(parent, padding=12)
        self.panel = panel
        self.closed = False
        self.timer = None

        # --- 问答侧状态（上一页原样保留的会话/代次防护）
        self.session = None
        self.battery = None
        self.battery_received = None
        self.current = None
        self.current_receipt = None
        self.current_error = ""
        self.pending = None
        self.deadline = 0.0
        self.nonce = secrets.randbelow(0xfffffffe) + 1
        self.protocol_errors = 0
        self.unsupported = False
        self.dirty = False
        self.updating = False
        self.fetched_for = None
        self.fetch_attempt_at = -1e9

        # --- 推送侧状态
        self.subscribed = False
        self.telem_stamp: _TelemStamp | None = None

        self.voltage_var = tk.StringVar(self, value="— V")
        self.current_var = tk.StringVar(self, value="— A")
        self.cell_var = tk.StringVar(self, value="— V")
        self.power_var = tk.StringVar(self, value="— W")
        self.live_state_var = tk.StringVar(self, value="等待遥测流")
        self.notice = tk.StringVar(self, value="")
        self.diag_state_var = tk.StringVar(self, value="尚未读取诊断")

        ttk.Label(self, text="电源 · 电池与电流", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(self, text="实时值来自遥测流推送；配置与诊断按需读取。",
                  style="Muted.TLabel").pack(anchor=tk.W)

        self._build_live(self)
        self._build_config(self)
        self._build_diagnostics(self)

        note = ttk.Label(
            self,
            text="总电压由飞控ADC分压换算；平均单节=总压/串数，未逐节测量，比例为标称值。\n"
                 "低压或电压数据无效时拒绝新解锁；已解锁后仅告警，不自动切断电机。\n"
                 "电流传感器位于电调，飞控读取 Curr 信号；标称换算尚未实测校准，"
                 "零读数不能证明 Curr 接线正常，也不能区分两台电机的电流。",
            style="Muted.TLabel", wraplength=820, justify=tk.LEFT)
        note.pack(anchor=tk.W, pady=8)
        self.bind("<Configure>", lambda e: note.configure(wraplength=max(220, e.width - 28)))
        self.bind("<Destroy>", self._destroy, add="+")
        self._tick()

    # ------------------------------------------------------------- 布局

    def _build_live(self, parent):
        box = ttk.LabelFrame(parent, text="实时（遥测流推送）", padding=10)
        box.pack(fill=tk.X, pady=8)
        row = ttk.Frame(box)
        row.pack(fill=tk.X)
        self.live_labels = {}
        for column, (key, caption, variable) in enumerate((
                ("voltage", "总电压", self.voltage_var),
                ("current", "电流", self.current_var),
                ("cell", "平均单节", self.cell_var),
                ("power", "功率", self.power_var))):
            cell = ttk.Frame(row)
            cell.grid(row=0, column=column, sticky=tk.W, padx=(0, 26))
            ttk.Label(cell, text=caption, style="Eyebrow.TLabel").pack(anchor=tk.W)
            label = ttk.Label(cell, textvariable=variable, style="PageTitle.TLabel")
            label.pack(anchor=tk.W)
            self.live_labels[key] = label
        self.live_state_label = ttk.Label(box, textvariable=self.live_state_var,
                                          style="Muted.TLabel", wraplength=760)
        self.live_state_label.pack(anchor=tk.W, pady=(8, 0))

    def _build_config(self, parent):
        config = ttk.LabelFrame(parent, text="本次运行配置（重启恢复默认）", padding=10)
        config.pack(fill=tk.X, pady=6)
        self.cells = tk.IntVar(self, value=3)
        # 与固件默认一致（DRV_BATTERY_DEFAULT_*，2026-10-01：3S 11.4 V 判低、11.45 V 恢复）。
        self.low = tk.IntVar(self, value=3800)
        self.recover = tk.IntVar(self, value=3817)
        for row, (label, var, lo, hi) in enumerate((
                ("串数", self.cells, 1, 12),
                ("每节告警 mV", self.low, 2500, 4100),
                ("每节恢复 mV", self.recover, 2501, 4400))):
            ttk.Label(config, text=label).grid(row=row, column=0, sticky=tk.W, pady=3)
            ttk.Spinbox(config, textvariable=var, from_=lo, to=hi, width=9,
                        style="Numeric.TSpinbox").grid(row=row, column=1, sticky=tk.W, padx=8)
            var.trace_add("write", self._draft_changed)
        actions = ttk.Frame(config)
        actions.grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=5)
        self.apply_button = ttk.Button(actions, text="应用并回读",
                                       command=self.apply_config, state=tk.DISABLED)
        self.apply_button.pack(side=tk.LEFT)
        self.refresh_button = ttk.Button(actions, text="刷新诊断",
                                         command=self.refresh_diagnostics)
        self.refresh_button.pack(side=tk.LEFT, padx=8)
        ttk.Label(config, textvariable=self.notice, style="Muted.TLabel").grid(
            row=4, column=0, columnspan=3, sticky=tk.W)

    def _build_diagnostics(self, parent):
        box = ttk.LabelFrame(parent, text="诊断（按需读取，不轮询）", padding=10)
        box.pack(fill=tk.X, pady=6)
        box.columnconfigure(1, weight=1)
        box.columnconfigure(3, weight=1)
        self.fields = {}
        self.value_labels = []
        entries = (
            ("cells", "配置串数"), ("limits", "告警 / 恢复线"),
            ("can_arm", "电池项允许新解锁"), ("arm_state", "飞控解锁状态"),
            ("v_raw", "电压ADC原始值"), ("v_valid", "电压采样有效"),
            ("v_age", "电压回包样本年龄"), ("v_samples", "电压采样次数"),
            ("v_errors", "电压采样错误次数"), ("v_adc_status", "电压ADC状态"),
            ("tx", "ELRS交给DMA / 拒绝次数"),
            ("i_raw", "电流ADC原始值"), ("i_adc_v", "电流ADC电压"),
            ("i_valid", "电流采样有效"), ("i_calibrated", "电流计校准"),
            ("i_saturated", "电流ADC高端饱和"), ("i_age", "电流回包样本年龄"),
            ("i_samples", "电流采样次数"), ("i_errors", "电流采样错误次数"),
            ("i_adc_status", "电流ADC状态"), ("i_source", "电流信号来源"),
            ("i_nominal", "标称灵敏度"), ("received_age", "距收到诊断回包"),
        )
        half = (len(entries) + 1) // 2
        for position, (key, label) in enumerate(entries):
            row, column = (position, 0) if position < half else (position - half, 2)
            ttk.Label(box, text=label).grid(row=row, column=column, sticky=tk.W,
                                            padx=(0, 16), pady=2)
            var = tk.StringVar(self, value="—")
            self.fields[key] = var
            value = ttk.Label(box, textvariable=var, style="Mono.TLabel")
            value.grid(row=row, column=column + 1, sticky=tk.W, pady=2)
            self.value_labels.append(value)
        ttk.Label(box, textvariable=self.diag_state_var, style="Muted.TLabel",
                  wraplength=760).grid(row=half, column=0, columnspan=4,
                                       sticky=tk.W, pady=(8, 0))

    # ------------------------------------------------------------- 会话

    def _connection(self):
        """连接代次守卫。换连接或断连一律清空快照——上一条链路的电压不能继续显示。"""
        transport = self.panel.transport
        session = (transport, getattr(transport, "connection_generation", 0))
        connected = self.panel._transport_connected()
        if session != self.session or not connected:
            self.session = session
            self.battery = None
            self.battery_received = None
            self.current = None
            self.current_receipt = None
            self.current_error = ""
            self.pending = None
            self.unsupported = False
            self.fetched_for = None
            self.fetch_attempt_at = -1e9
            self.telem_stamp = None
            for var in self.fields.values():
                var.set("—")
            self.diag_state_var.set("尚未读取诊断" if connected else "未连接")
        return connected

    def _draft_changed(self, *_args):
        if not self.updating:
            self.dirty = True
            self.notice.set("配置有未提交修改")

    # ------------------------------------------------------------- 订阅

    def visible(self) -> bool:
        panel = self.panel
        group = getattr(panel, "sensor_group_tab", None)
        tab = getattr(panel, "power_tab", None)
        notebook = getattr(panel, "notebook", None)
        sensors = getattr(panel, "sensor_notebook", None)
        if group is None or tab is None or notebook is None or sensors is None:
            return False
        try:
            return (notebook.select() == str(group)
                    and sensors.select() == str(tab))
        except tk.TclError:                       # pragma: no cover - 窗口正在销毁
            return False

    def _sync_subscription(self, visible: bool) -> None:
        """可见才订阅 `batt_v` / `batt_i`，不可见就退订。

        看不见的页面没有理由占数传带宽，而掩码是整条覆盖的——留在里面就等于
        永久涨两路。退订走仲裁器，不直接发 `TELEM MASK`：并集要和工作台一起算。
        """
        subscribe = getattr(self.panel, "_telem_subscribe", None)
        unsubscribe = getattr(self.panel, "_telem_unsubscribe", None)
        if subscribe is None or unsubscribe is None:  # pragma: no cover - 无工作台的替身面板
            return
        if visible:
            subscribe(TELEM_OWNER_POWER, POWER_CHANNELS, stream=True,
                      consumer=self._on_telem_frame)
            self.subscribed = True
        elif self.subscribed:
            unsubscribe(TELEM_OWNER_POWER)
            self.subscribed = False

    def _on_telem_frame(self, function: int, payload: bytes, *, transport,
                        generation=None) -> None:
        """**收线程**回调：只留一个到达证据，不碰任何 Tk 控件。

        `transport` / `generation` 是分发器在挂 sink 那一刻钉死的来源身份，不是
        `panel.transport` 和它此刻的代次：重连之后旧链路的最后一帧还可能在路上，
        现读的话会把它算成新会话的数据，实时区就会显示上一条链路的电压。
        """
        del function, payload
        if generation is None:
            generation = getattr(transport, "connection_generation", 0)
        self.telem_stamp = _TelemStamp(time.monotonic(), transport, generation)

    # ------------------------------------------------------------- 发送

    def _send(self, command, kind):
        if not self._connection():
            return False
        if not self.panel._validation_guard_command(command):
            return False
        sender = getattr(self.panel.transport, "send_ascii_line",
                         self.panel.transport.send_line)
        if not sender(command):
            self.notice.set("发送被拒绝或链路占用")
            return False
        self.pending = kind
        self.deadline = time.monotonic() + POWER_REPLY_TIMEOUT_S
        return True

    def refresh_diagnostics(self):
        """诊断区的唯一取数动作：打开页面时一次 + 手动按钮。没有定时器调它。"""
        if not self._connection():
            return False
        self.current_error = ""
        # 电流诊断只在 `STATUS?` 的大回包里；它没有 nonce，靠接收代次防护。
        status_sent = self.panel._send_proto_silent(PROTO_REQ_STATUS, "STATUS?")
        if self.pending is not None:
            return False
        self.unsupported = False
        self.nonce = (self.nonce + 1) & 0xffffffff or 1
        sent = self._send(f"BATTERY? {self.nonce}", "query")
        # **发成功了才记账**。链路被日志导出独占、或处在 V0 只读会话时命令会被
        # 拒，此时若把"已经取过一次"钉上，这一次诊断就永远丢了，而页面上只会
        # 显示一排 `—`，没有任何东西提示它其实一个字节都没发出去。
        if sent and status_sent:
            self.fetched_for = self.session
        elif not sent:
            self.notice.set("诊断读取未发出：链路被占用或当前会话拒绝")
        return sent

    def apply_config(self):
        if not self._can_configure():
            self.notice.set("需要当前连接的新鲜、未解锁回包")
            return False
        try:
            values = (int(self.cells.get()), int(self.low.get()), int(self.recover.get()))
        except (ValueError, tk.TclError):
            self.notice.set("配置必须为整数")
            return False
        cells, low, recover = values
        if not (1 <= cells <= 12 and 2500 <= low <= 4100 and low < recover <= 4400):
            self.notice.set("串数或阈值超界；恢复线必须高于告警线")
            return False
        self.nonce = (self.nonce + 1) & 0xffffffff or 1
        return self._send(f"BATTERY SET {self.nonce} {cells} {low} {recover}", values)

    def _can_configure(self):
        if not self._connection():
            return False
        return (self.battery is not None and self.battery_received is not None
                and time.monotonic() - self.battery_received <= POWER_REPLY_STALE_S
                and self.battery.arm_state == 1 and self.pending is None
                and self.panel._transport_connected())

    # ------------------------------------------------------------- 接收

    def accept(self, payload, context):
        """`BATTERY?` / `BATTERY SET` 的二进制回包。nonce 必须对上这一次问答。"""
        if not self._connection() or self.pending is None:
            return
        if context is not None and not context.is_current(self.panel.transport):
            return
        try:
            snapshot = decode_battery(payload)
        except ValueError as error:
            self.protocol_errors += 1
            self.pending = None
            self.battery = None
            self._clear_battery_fields()
            self.diag_state_var.set(f"{error} · 协议错误 {self.protocol_errors}")
            return
        if snapshot.nonce != self.nonce:
            return
        operation = self.pending
        self.pending = None
        self.battery = snapshot
        self.battery_received = time.monotonic() if context is None else context.received_at
        if operation != "query":
            actual = (snapshot.cells, snapshot.low_mv, snapshot.recover_mv)
            acknowledged = bool(snapshot.flags & 32) and actual == operation
            self.notice.set("飞控已应用 · RAM配置" if acknowledged
                            else "飞控未确认应用；当前值以回读为准")
            try:
                unchanged = (self.cells.get(), self.low.get(), self.recover.get()) == operation
            except tk.TclError:
                unchanged = False
            if acknowledged and unchanged:
                self.dirty = False
        if not self.dirty:
            self.updating = True
            for var, value in ((self.cells, snapshot.cells), (self.low, snapshot.low_mv),
                               (self.recover, snapshot.recover_mv)):
                var.set(value)
            self.updating = False
        self._render_diagnostics()

    def handle_line(self, line):
        if not isinstance(line, str):
            return
        if line.startswith("CURRENT "):
            self._accept_current(line)
            return
        if self.pending is not None and line.startswith("ERR unknown cmd BATTERY"):
            self.pending = None
            self.unsupported = True
            self.battery = None
            self._clear_battery_fields()
            self.diag_state_var.set("当前固件未提供电池电压回读")

    def _accept_current(self, line):
        self._connection()
        receipt = (getattr(self.panel, "_rx_context", None)
                   or receive_context(self.panel.transport))
        if not receipt.is_current(self.panel.transport):
            return
        try:
            snapshot = parse_current(line)
        except (KeyError, TypeError, ValueError, OverflowError):
            self.current = None
            self.current_receipt = receipt
            self.current_error = "电流回包缺字段或数据无效"
        else:
            self.current, self.current_receipt = snapshot, receipt
            self.current_error = ""
        self._render_diagnostics()

    # ------------------------------------------------------------- 渲染

    def _clear_battery_fields(self):
        for key in ("cells", "limits", "can_arm", "arm_state", "v_raw", "v_valid",
                    "v_age", "v_samples", "v_errors", "v_adc_status", "tx"):
            self.fields[key].set("—")

    def live_reason(self, *, newest_t: float | None = None) -> str:
        """实时值为什么是 `—`。把原因指到诊断区能对上的地方去。

        先分清"通道根本没来"和"通道来了但值是 NaN"：前者是掩码 / 固件复位的问题，
        后者才是 ADC 的问题。把前者说成 ADC 故障，会把人引到错误的方向上去查。
        """
        absent = [name for name in POWER_CHANNELS
                  if self._channel_stale(name, newest_t)]
        if absent:
            return (f"{' / '.join(absent)} 已经不在遥测帧里 · "
                    "可能是飞控刚复位、掩码被还原；正在自动重开流")
        battery = self.battery
        if battery is None:
            return "电压/电流无效或过期；点「刷新诊断」查看 ADC 状态"
        if not battery.valid:
            status = ADC_STATUS.get(battery.adc_status, f"状态 {battery.adc_status}")
            return f"飞控报电压采样无效 · 电压ADC状态：{status}"
        if battery.current_ma is None:
            return "飞控报电流不可用 · 见诊断区电流ADC状态"
        return "电压/电流无效或过期；点「刷新诊断」查看 ADC 状态"

    def _channel_stale(self, name, newest_t):
        """这一路是不是已经掉出掩码了（有帧在来，但它不在里面）。"""
        entry = self.panel._telem_latest_raw(name)
        if entry is None:
            return True
        return newest_t is not None and (newest_t - float(entry[0])) > POWER_TELEM_STALE_S

    def _render_live(self):
        panel = self.panel
        connected = panel._transport_connected()
        channel = getattr(panel, "_dashboard_channel", None)
        raw = getattr(panel, "_telem_latest_raw", None)

        def blank(state, style="Muted.TLabel"):
            for var in (self.voltage_var, self.current_var, self.cell_var, self.power_var):
                var.set("—")
            self.live_state_var.set(state)
            self.live_state_label.configure(style=style)

        if not connected:
            blank("未连接")
            return
        if channel is None or raw is None:      # pragma: no cover - 无工作台的替身面板
            blank("当前面板没有遥测流")
            return
        schema = getattr(panel, "dashboard_schema", None)
        if schema is None or not schema.complete:
            # 表还没拉齐时别急着喊"通道不存在"——那句话此刻还不成立。
            blank("等待飞控通道表")
            return
        missing = [name for name in POWER_CHANNELS if channel(name) is None]
        if missing:
            blank(f"当前固件通道表没有 {' / '.join(missing)}；实时区不可用", "Fail.TLabel")
            return
        stamp = self.telem_stamp
        if stamp is None or not stamp.is_current(panel.transport):
            blank("等待当前连接的遥测帧" + ("" if self.subscribed else "（页面不可见时不订阅）"))
            return
        age = time.monotonic() - stamp.at
        if age > POWER_TELEM_STALE_S:
            blank(f"遥测帧已过期 {age:.1f} s · 检查流是否还开着", "Fail.TLabel")
            return

        newest_t = self._newest_channel_time()
        volts = self._channel_value(POWER_CHANNEL_VOLTAGE, newest_t=newest_t)
        amps = self._channel_value(POWER_CHANNEL_CURRENT, newest_t=newest_t)
        cells = self.battery.cells if self.battery is not None else None
        cell = average_cell_volts(volts, cells) if volts is not None else None
        watts = electrical_power_watts(volts, amps)
        self.voltage_var.set("—" if volts is None else f"{volts:.3f} V")
        self.current_var.set("—" if amps is None else f"{amps:.3f} A")
        self.cell_var.set("—" if cell is None else f"{cell:.3f} V")
        self.power_var.set("—" if watts is None else f"{watts:.1f} W")
        if volts is None or amps is None:
            self.live_state_var.set(self.live_reason(newest_t=newest_t))
            self.live_state_label.configure(style="Fail.TLabel")
        elif cell is None:
            self.live_state_var.set("实时推送中 · 串数未知，点「刷新诊断」取回读后才有单节电压")
            self.live_state_label.configure(style="Muted.TLabel")
        else:
            self.live_state_var.set(f"实时推送中 · {cells} S · 数据 {age * 1000:.0f} ms 前到达")
            self.live_state_label.configure(style="Pass.TLabel")

    def _channel_value(self, name, *, newest_t: float | None = None):
        """通道最新值，**同时判年龄**。NaN = 无效/过期，不是 0，一律返回 None。

        只判 `isfinite` 是不够的：固件复位后掩码回到默认，`batt_v` / `batt_i` 不再
        出现在任何一帧里，可环形缓冲里还躺着复位前的那个值，而"有没有帧进来"
        这条检查仍然通过（别的通道在喂）。于是页面会把一个几分钟前的电压标成
        "实时推送中 · 数据 12 ms 前到达"——显示陈旧值当实时值，比显示 `—` 危险。

        年龄按**固件时间轴**算：拿这一路自己的时间戳跟本帧里最新的时间戳比，
        两者同源，不受主机时钟和调度抖动影响。
        """
        entry = self.panel._telem_latest_raw(name)
        if entry is None:
            return None
        stamp, value = float(entry[0]), float(entry[1])
        if not math.isfinite(value):
            return None
        if newest_t is not None and (newest_t - stamp) > POWER_TELEM_STALE_S:
            return None
        return value

    def _newest_channel_time(self) -> float | None:
        """最近一帧遥测的固件时间戳（秒），作为"现在几点了"的参照。

        由工作台的解码消费者在收线程里记一次，这里 O(1) 读。不去遍历环形缓冲
        找最大值：那是每次渲染 128 次加锁，换来的是同一个数。
        """
        t_us = getattr(self.panel, "telem_last_frame_t_us", None)
        return None if t_us is None else float(t_us) * 1e-6

    def _render_diagnostics(self):
        battery = self.battery
        if battery is not None:
            stale = (self.battery_received is None
                     or time.monotonic() - self.battery_received > POWER_REPLY_STALE_S)
            self.fields["cells"].set(f"{battery.cells} S")
            self.fields["limits"].set(
                f"{battery.low_mv / 1000:.2f} / {battery.recover_mv / 1000:.2f} V每节")
            self.fields["can_arm"].set("是" if battery.can_arm and not stale else "否")
            self.fields["arm_state"].set(
                {0: "未知", 1: "未解锁", 2: "已解锁"}.get(battery.arm_state, "未知"))
            self.fields["v_raw"].set(str(battery.raw))
            self.fields["v_valid"].set("是" if battery.valid and not stale else "否")
            self.fields["v_age"].set(f"{battery.age_ms} ms")
            self.fields["v_samples"].set(str(battery.samples))
            self.fields["v_errors"].set(str(battery.errors))
            self.fields["v_adc_status"].set(
                ADC_STATUS.get(battery.adc_status, f"状态 {battery.adc_status}"))
            self.fields["tx"].set(f"{battery.tx_accepted} / {battery.tx_rejected}")
        snapshot = self.current
        if snapshot is not None:
            self.fields["i_raw"].set(str(snapshot["raw"]))
            self.fields["i_adc_v"].set(
                f"{snapshot['adc_v']:.5f} V" if math.isfinite(snapshot["adc_v"]) else "不可用")
            self.fields["i_valid"].set("是" if snapshot["valid"] else "否")
            self.fields["i_calibrated"].set(
                "已标记校准" if snapshot["calibrated"] else "未校准 · 标称换算")
            self.fields["i_saturated"].set("是" if snapshot["saturated"] else "否")
            self.fields["i_age"].set(
                "无样本" if snapshot["age_ms"] == UINT32_MAX else f"{snapshot['age_ms']} ms")
            self.fields["i_samples"].set(str(snapshot["samples"]))
            self.fields["i_errors"].set(str(snapshot["errors"]))
            self.fields["i_adc_status"].set(ADC_STATUS[snapshot["adc_status"]])
            self.fields["i_source"].set(snapshot["source"])
            self.fields["i_nominal"].set(f"{snapshot['nominal_mv_per_a']:g} mV/A")
        elif self.current_error:
            for key in ("i_raw", "i_adc_v", "i_valid", "i_calibrated", "i_saturated",
                        "i_age", "i_samples", "i_errors", "i_adc_status", "i_source",
                        "i_nominal"):
                self.fields[key].set("—")
        stamps = [value for value in (self.battery_received,
                                      getattr(self.current_receipt, "received_at", None))
                  if value is not None]
        if stamps:
            self.fields["received_age"].set(f"{max(0.0, time.monotonic() - max(stamps)):.1f} s")
        if self.current_error:
            self.diag_state_var.set(self.current_error)
        elif self.unsupported:
            self.diag_state_var.set("当前固件未提供电池电压回读")
        elif battery is None and snapshot is None:
            self.diag_state_var.set("尚未读取诊断")
        else:
            self.diag_state_var.set("诊断为一次性回读，需要最新值请点「刷新诊断」")

    # ------------------------------------------------------------- 节拍

    def _tick(self):
        """界面节拍。**不发任何周期性命令**——这是本次合并的验收点之一。"""
        if self.closed:
            return
        if getattr(self.panel, "telem_registry", None) is None:
            # 工作台（仲裁器的宿主）还没挂载。shell 里页面构造顺序在它之前，
            # 直接往下走会在半初始化的面板上取数。等下一拍再来。
            self.timer = self.after(POWER_RENDER_MS, self._tick)
            return
        connected = self._connection()
        visible = self.visible()
        self._sync_subscription(visible and connected)
        now = time.monotonic()
        if self.pending is not None and now > self.deadline:
            self.pending = None
            self.notice.set("电池回包超时，未确认操作结果")
        self.apply_button.configure(
            state=tk.NORMAL if self._can_configure() else tk.DISABLED)
        self.refresh_button.configure(state=tk.NORMAL if connected else tk.DISABLED)
        if (visible and connected and self.fetched_for != self.session
                and (now - self.fetch_attempt_at) >= POWER_FETCH_RETRY_S):
            # "打开页面取一次"。挂在会话身份上而不是一个布尔量：重连之后的第一次
            # 可见同样要重新取，否则诊断区留着上一条链路的计数。成功之前按
            # POWER_FETCH_RETRY_S 退避重试，成功之后彻底停下。
            self.fetch_attempt_at = now
            self.refresh_diagnostics()
        self._render_live()
        self._render_diagnostics()
        self.timer = self.after(POWER_RENDER_MS, self._tick)

    def _destroy(self, event):
        if event.widget is self:
            self.closed = True
            self._sync_subscription(False)
            if self.timer is not None:
                self.after_cancel(self.timer)
                self.timer = None


def mount_power(panel, parent):
    panel.power_page = PowerPage(parent, panel)
    panel.power_page.pack(fill=tk.BOTH, expand=True)
