"""工作台的组件（tile）。

组件只认两样东西：自己的 `TileSpec`（类型、几何、绑定的通道名、选项）和一个
`TileContext`（取数、发参数、发命令）。它们不认识 `DronePanel`，所以可以脱离
面板单独构造和测试。

绑定用通道名：固件通道表变了之后按名重绑；名字在新表里找不到的组件明确显示
"通道不存在"。**不允许静默降级**——一个悄悄绑到别的通道上的曲线，比一个空着
的曲线危险得多。
"""

from __future__ import annotations

import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

from ..scope import SCOPE_PALETTE, SCOPE_WINDOW_CHOICES_S, ScopeCanvas
from .channel_picker import ChannelPicker
from .layout import (
    TILE_PARAM,
    TILE_VALUE,
    TILE_WAVE,
    TileSpec,
)


# 拖动中的写入节流。5 Hz 是"跟手"和"不把数传写满"之间的折中：每次 PARAM SET
# 固件都会回 PARAM 记录 + PID legacy 长文本，不节流会把命令回复和遥测帧一起
# 挤爆 uartTxQueue（深度 32、满时丢最旧的一条）。沿用 R-T1-3 的实测取值。
PARAM_THROTTLE_S = 0.2

# 回显判定的相对容差。固件按 1e-3 精度格式化，1e-4 足以区分"收下了"和"钳位了"。
PARAM_ECHO_TOLERANCE = 1e-4

# 发送后多久之内的"不一致回显"不算 diverged。要盖住"发送前已编码、发送后才到"
# 的在线帧：数传上一帧全量刷新 137 B ≈ 24 ms，再加一个 25 ms 的遥测拍。
# 这条是审核者在实机上抓到的竞态（滑块被旧值拽回），不是理论余量。
PARAM_ECHO_GRACE_S = 0.10

# 若整整一段遥测窗口都没有拿到参数回显，不能继续把 pending 当成“可能已经
# 应用”。这里取 0.75 s：远大于数传一帧+调度抖动，却足够让调参时的红灯及时提醒。
PARAM_ECHO_TIMEOUT_S = 0.75

PARAM_STATE_IDLE = "idle"
PARAM_STATE_PENDING = "pending"
PARAM_STATE_CONFIRMED = "confirmed"
PARAM_STATE_DIVERGED = "diverged"

# 不用只给文本染色：原生 ttk.Scale 在部分 Windows 主题下会忽略 trough 颜色，
# 所以旁边的 Canvas 色点是确定可见的反馈证据。绿只代表收到匹配的飞控遥测回显，
# 红代表不一致或超时无回显，黄是尚未判定。
PARAM_FEEDBACK_IDLE_COLOUR = "#7c8b99"
PARAM_FEEDBACK_PENDING_COLOUR = "#e0a84c"
PARAM_FEEDBACK_CONFIRMED_COLOUR = "#57c78b"
PARAM_FEEDBACK_DIVERGED_COLOUR = "#ef6b73"

PARAM_FEEDBACK_TEXT = {
    PARAM_STATE_IDLE: "等待飞控数据",
    PARAM_STATE_PENDING: "等待飞控回显",
    PARAM_STATE_CONFIRMED: "飞控已回显",
    PARAM_STATE_DIVERGED: "未收到飞控回显",
}

PARAM_SCALE_STYLES = {
    PARAM_STATE_IDLE: "DashParamIdle.Horizontal.TScale",
    PARAM_STATE_PENDING: "DashParamPending.Horizontal.TScale",
    PARAM_STATE_CONFIRMED: "DashParamConfirmed.Horizontal.TScale",
    PARAM_STATE_DIVERGED: "DashParamDiverged.Horizontal.TScale",
}

WAVE_MAX_BINDINGS = 4

MISSING_CHANNEL_TEXT = "通道不存在"

# 大字读数用的样式。**在这里注册而不是去改 drone_tcp_panel.py 的样式块**：
# 那个文件只减不增，本期只允许改三处挂载调用。颜色从面板已有的样式里 lookup
# 出来，不复制一份调色板——复制的那份迟早会和主题漂移。
DASH_METRIC_STYLE = "DashMetric.TLabel"
DASH_METRIC_PASS_STYLE = "DashMetricPass.TLabel"
DASH_METRIC_WARN_STYLE = "DashMetricWarn.TLabel"
DASH_METRIC_FAIL_STYLE = "DashMetricFail.TLabel"

_STYLES_READY = False
# **必须持有这个引用**：tkinter 的 Font 对象被回收时会 `font delete` 掉对应的
# 具名字体，而 ttk 样式里存的只是那个名字。丢了引用，样式会静默退回默认字号，
# "大字读数"就和普通标签一样大——本次截图复核时抓到的正是这个。
_METRIC_FONT = None


def ensure_dashboard_styles(widget: tk.Misc) -> None:
    global _STYLES_READY, _METRIC_FONT
    if _STYLES_READY:
        return
    style = ttk.Style(widget)
    background = style.lookup("TLabel", "background") or ""
    neutral = style.lookup("TLabel", "foreground") or ""
    # 用 tkfont 派生出一个**具名字体**：直接写 ("TkFixedFont", 20, "bold") 是错的
    # ——TkFixedFont 是具名字体不是字族名，Tk 会静默退回默认字号，于是"大字读数"
    # 和普通标签一样大（本次截图复核时抓到的）。
    base = tkfont.nametofont("TkFixedFont", root=widget)
    metric_font = tkfont.Font(
        root=widget, family=base.actual("family"), size=20, weight="bold"
    )
    _METRIC_FONT = metric_font
    for name, source in (
        (DASH_METRIC_STYLE, None),
        (DASH_METRIC_PASS_STYLE, "Pass.TLabel"),
        (DASH_METRIC_WARN_STYLE, "Warn.TLabel"),
        (DASH_METRIC_FAIL_STYLE, "Fail.TLabel"),
    ):
        foreground = neutral if source is None else (
            style.lookup(source, "foreground") or neutral
        )
        options = {"font": metric_font}
        if background:
            options["background"] = background
        if foreground:
            options["foreground"] = foreground
        style.configure(name, **options)
    for state, colour in (
        (PARAM_STATE_IDLE, PARAM_FEEDBACK_IDLE_COLOUR),
        (PARAM_STATE_PENDING, PARAM_FEEDBACK_PENDING_COLOUR),
        (PARAM_STATE_CONFIRMED, PARAM_FEEDBACK_CONFIRMED_COLOUR),
        (PARAM_STATE_DIVERGED, PARAM_FEEDBACK_DIVERGED_COLOUR),
    ):
        # 色点负责跨主题的确定性可见性；这里同时尽力给滑块槽/拇指上色。
        style.configure(PARAM_SCALE_STYLES[state], background=colour, troughcolor=colour)
    _STYLES_READY = True


class TileContext:
    """组件向页面要东西的唯一入口。页面实现它，组件只调它。"""

    def channel(self, name: str):                       # -> TelemChannel | None
        raise NotImplementedError

    def latest(self, name: str) -> float | None:
        raise NotImplementedError

    def series(self, name: str):                        # -> (np.ndarray, np.ndarray)
        raise NotImplementedError

    def send_param(self, name: str, value: float) -> bool:
        raise NotImplementedError

    def param_tracker(self, name: str) -> "ParamEchoTracker":
        raise NotImplementedError

    def send_command(self, text: str) -> bool:
        raise NotImplementedError

    def all_channels(self) -> list:
        """当前通道表的全部通道，按索引升序。"""
        raise NotImplementedError

    def bindings_changed(self, spec: TileSpec) -> None:
        """组件从自身下拉框改绑后，由页面重算 MASK 并持久化布局。"""
        raise NotImplementedError


class ParamEchoTracker:
    """滑块三态引擎：pending / confirmed / diverged。

    整套规则从 R-T1-3 原样搬过来，包括审核者在实机上修掉的两个竞态：

    * 发出后 `PARAM_ECHO_GRACE_S` 内到达的**不一致**回显不判 diverged——它可能
      是发送之前就已经编码、还在线上的那一帧；
    * 节流窗口内的非 pending 回显不覆盖控件位置——否则用户拖动时滑块会往回蹦。

    引擎不碰任何控件，只算状态；显示由组件自己做。这样同一份规则能被多张卡
    共用，也能脱离 Tk 测。
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self.state = PARAM_STATE_IDLE
        self.sent: float | None = None
        self.last_send = -1e9
        self.display: float | None = None
        #: 飞控明说的拒绝理由。为空时才回落到按状态推断的措辞。
        self.reason: str | None = None

    def note_rejected(self, reason: str) -> None:
        """飞控回了 `ERR param target`：立刻判红，并说出真正的原因。

        没有这条时，被拒绝的写入只能靠 PARAM_ECHO_TIMEOUT_S 超时间接发现，
        而那条路的措辞是"未收到飞控回显"——在这里是**假话**：飞控回了，回的是
        拒绝。操作者因此分不清"飞控不接受这个值"和"数传丢包"，只能干等 0.75 s
        再猜。2026-09-07 作者把偏航角度增益调到 100 却毫无反应，正是撞在这上面：
        `rate_kp[2]` 为 0 时固件按除零保护拒绝了 `coax.yaw_angle_kp`，界面却一直
        显示 100。
        """
        self.state = PARAM_STATE_DIVERGED
        self.reason = reason

    def may_send(self, now: float) -> bool:
        """拖动中是否到了下一次可发的时刻。松手那一发不走这里。"""
        return (now - self.last_send) >= PARAM_THROTTLE_S

    def note_sent(self, value: float, now: float) -> None:
        self.sent = float(value)
        self.last_send = now
        self.state = PARAM_STATE_PENDING
        # 新的一次写入：上一次的拒绝理由已经过期，留着会指向错误的值。
        self.reason = None
        # 不把“PC 已发出”伪装成“飞控已应用”。display 只由 note_echo 写入，
        # 因而大字始终是最后一条真实遥测回显（或尚未知的空值）。

    def expire_if_needed(self, now: float) -> bool:
        """回显超时后进入红色状态，返回本次是否刚发生状态翻转。"""
        if (
            self.state == PARAM_STATE_PENDING
            and (now - self.last_send) >= PARAM_ECHO_TIMEOUT_S
        ):
            self.state = PARAM_STATE_DIVERGED
            return True
        return False

    def note_echo(self, echoed: float, now: float) -> str:
        """喂一个固件回显值，返回处理动作。

        返回 `"follow"` 表示控件应该跟随 `echoed`；`"hold"` 表示保持不动。
        """
        since_send = now - self.last_send

        if self.state != PARAM_STATE_PENDING or self.sent is None:
            if since_send < PARAM_THROTTLE_S:
                # 用户还在拖：这时把控件拽回固件上一拍的值，手感就是往回蹦。
                return "hold"
            self.display = float(echoed)
            # 一个迟到但匹配的真实回显可以把“超时未回显”翻回绿色；若还不匹配，
            # 红色继续保留，显示的仍是飞控实际值而不是用户输入。
            if self.state == PARAM_STATE_DIVERGED and self.sent is not None:
                scale = max(abs(self.sent), abs(echoed), 1.0)
                if abs(echoed - self.sent) <= PARAM_ECHO_TOLERANCE * scale:
                    self.state = PARAM_STATE_CONFIRMED
            return "follow"

        scale = max(abs(self.sent), abs(echoed), 1.0)
        if abs(echoed - self.sent) <= PARAM_ECHO_TOLERANCE * scale:
            self.state = PARAM_STATE_CONFIRMED
            self.display = float(echoed)
            self.reason = None
            return "follow"

        if since_send < PARAM_ECHO_GRACE_S:
            # 刚发出去，这条旧值还在传输窗口里。再等下一帧。
            return "hold"

        # 宽限期过了还是旧值：固件真的钳位或拒绝了。控件必须跳到固件的实际
        # 值——留在用户拖到的位置上等于让界面撒谎，而且正好谎在参数没生效时。
        self.state = PARAM_STATE_DIVERGED
        self.display = float(echoed)
        return "follow"


class DashboardTile:
    """组件基类。子类实现 `_build()` 与 `refresh()`。"""

    TYPE = ""
    LABEL = ""
    MIN_BINDINGS = 1
    MAX_BINDINGS = 1
    #: 属性对话框里能选的通道范围：True = 只能选参数通道，False = 只能选非参数
    #: 通道，None = 都行。
    PARAM_ONLY: bool | None = False
    #: 新建时的默认格子尺寸 (colspan, rowspan)。
    DEFAULT_SPAN: tuple[int, int] = (3, 2)
    #: 属性对话框里额外可编辑的 options 字段：((键, 显示名), ...)。
    OPTION_FIELDS: tuple[tuple[str, str], ...] = ()

    def __init__(self, parent: tk.Misc, spec: TileSpec, context: TileContext) -> None:
        self.spec = spec
        self.context = context
        ensure_dashboard_styles(parent)
        self.frame = ttk.Frame(parent, style="Card.TFrame", padding=6)
        self.missing: list[str] = []
        self._build()
        self.rebind()

    # ------------------------------------------------------------- 生命周期

    def _build(self) -> None:
        raise NotImplementedError

    def rebind(self) -> None:
        """通道表变了之后重算绑定。绑不上的名字进 `missing`。"""
        self.missing = [
            name for name in self.spec.bindings if self.context.channel(name) is None
        ]

    def refresh(self) -> None:
        """33 ms 一次。默认什么都不做。"""

    def destroy(self) -> None:
        self.frame.destroy()

    # ------------------------------------------------------------- 工具

    def bound(self) -> list[str]:
        return [name for name in self.spec.bindings if self.context.channel(name) is not None]

    def title(self) -> str:
        custom = self.spec.options.get("title")
        if isinstance(custom, str) and custom.strip():
            return custom.strip()
        if self.spec.bindings:
            return " / ".join(self.spec.bindings)
        return self.LABEL

    def _set_bindings_from_picker(self, bindings: tuple[str, ...]) -> None:
        """把卡片内下拉的选择原子写回布局，再让页面收敛掩码和持久化。"""
        self.spec.bindings = list(bindings)
        self.rebind()
        self.context.bindings_changed(self.spec)


class WaveTile(DashboardTile):
    """波形：1~4 通道共用一张画布，自己的 Y 量程与时间窗口。

    R-T1-3 把 8 条曲线（含 uptime≈600 s）压在同一张画布的同一根 Y 轴上，
    结果除了 uptime 以外全是直线。这里每张波形卡自己一套量程，而且默认只绑
    量纲相同的一组通道。
    """

    TYPE = TILE_WAVE
    LABEL = "波形"
    MIN_BINDINGS = 1
    MAX_BINDINGS = WAVE_MAX_BINDINGS
    PARAM_ONLY = None
    DEFAULT_SPAN = (6, 5)

    def _build(self) -> None:
        header = ttk.Frame(self.frame)
        header.pack(fill=tk.X)
        self.title_var = tk.StringVar(value=self.title())
        ttk.Label(header, textvariable=self.title_var, style="Eyebrow.TLabel").pack(side=tk.LEFT)

        # 曲线选择留在卡片顶部：用户调试时无需进入编辑模式或翻属性对话框。菜单
        # 是多选，但最大四条，沿用 ScopeCanvas 的可读性上限。
        self.channel_picker = ChannelPicker(
            header, bindings=self.spec.bindings, multiple=True,
            min_selected=self.MIN_BINDINGS, max_selected=self.MAX_BINDINGS,
            empty_text="选择数据", on_change=self._set_bindings_from_picker,
            width=20,
        )
        self.channel_picker.pack(side=tk.RIGHT, padx=(0, 8))

        self.window_var = tk.StringVar(
            value=str(self.spec.options.get("window_s", 10))
        )
        window = ttk.Combobox(
            header, textvariable=self.window_var, width=4, state="readonly",
            values=[f"{choice:g}" for choice in SCOPE_WINDOW_CHOICES_S],
        )
        window.pack(side=tk.RIGHT)
        window.bind("<<ComboboxSelected>>", self._on_window_changed)

        self.lock_var = tk.BooleanVar(value=bool(self.spec.options.get("lock_y", False)))
        ttk.Checkbutton(
            header, text="锁定 Y", variable=self.lock_var, command=self._on_lock_toggled
        ).pack(side=tk.RIGHT, padx=(0, 8))

        self.legend = ttk.Frame(self.frame)
        self.legend.pack(fill=tk.X, pady=(2, 2))
        self.legend_vars: dict[str, tk.StringVar] = {}

        self.canvas = ScopeCanvas(self.frame, height=120)
        self.canvas.canvas.pack(fill=tk.BOTH, expand=True)

    def _on_window_changed(self, _event=None) -> None:
        try:
            seconds = float(self.window_var.get())
        except ValueError:
            return
        self.spec.options["window_s"] = seconds
        self.canvas.set_window(seconds)

    def _on_lock_toggled(self) -> None:
        locked = bool(self.lock_var.get())
        self.spec.options["lock_y"] = locked
        if locked:
            low, high = self.canvas.last_range
            self.canvas.set_manual_range(low, high)
        else:
            self.canvas.set_auto_range()

    def rebind(self) -> None:
        super().rebind()
        self.title_var.set(self.title())
        self.channel_picker.set_channels(
            self.context.all_channels(), selected=self.spec.bindings
        )
        for child in self.legend.winfo_children():
            child.destroy()
        self.legend_vars = {}

        # 只有真实存在的通道才进画布，图例配色必须与 set_curves 收到的**列表
        # 位置**一致——ScopeCanvas 是按列表位置取调色板的，中间少一条通道时
        # 按 bindings 下标配色会让图例和曲线的颜色对不上。
        entries: list[tuple[int, str]] = []
        for name in self.spec.bindings[:WAVE_MAX_BINDINGS]:
            if self.context.channel(name) is None:
                continue
            entries.append((len(entries), name))
        for position, name in entries:
            colour = SCOPE_PALETTE[position % len(SCOPE_PALETTE)]
            variable = tk.StringVar(value=f"{name} —")
            self.legend_vars[name] = variable
            ttk.Label(
                self.legend, textvariable=variable, foreground=colour,
                style="Mono.TLabel",
            ).pack(side=tk.LEFT, padx=(0, 10))
        self.canvas.set_curves(entries)
        self._curve_names = dict(entries)
        self.canvas.set_window(float(self.spec.options.get("window_s", 10)))
        if self.missing:
            ttk.Label(
                self.legend, text=f"{MISSING_CHANNEL_TEXT}：{', '.join(self.missing)}",
                style="Fail.TLabel",
            ).pack(side=tk.LEFT)

    def refresh(self) -> None:
        series = {
            position: self.context.series(name)
            for position, name in self._curve_names.items()
        }
        self.canvas.render(series)
        for position, name in self._curve_names.items():
            latest = self.context.latest(name)
            channel = self.context.channel(name)
            unit = channel.unit if channel is not None else ""
            text = "—" if latest is None else f"{latest:+.4g}"
            self.legend_vars[name].set(
                f"{name} {text}" + (f" {unit}" if unit and unit != "-" else "")
            )


class ValueTile(DashboardTile):
    """数值卡：大字当前值 + 单位 + 名称，可选上下限着色。

    光流高度、速度、姿态角这类"现在是多少"的量默认用它（作者裁决）：画成曲线
    要眯着眼睛读刻度，而这些量真正关心的是当下的数。
    """

    TYPE = TILE_VALUE
    LABEL = "数值卡"
    MIN_BINDINGS = 1
    MAX_BINDINGS = 1
    PARAM_ONLY = None

    def _build(self) -> None:
        self.name_var = tk.StringVar(value=self.title())
        self.value_var = tk.StringVar(value="—")
        self.unit_var = tk.StringVar(value="")
        self._last_value_text: str | None = None
        self._last_metric_style: str | None = None
        header = ttk.Frame(self.frame)
        header.pack(fill=tk.X)
        ttk.Label(header, textvariable=self.name_var, style="Eyebrow.TLabel").pack(side=tk.LEFT)
        # 数值卡保留一个标量的大字号可读性，因此是单选；每张固定卡都能从当前
        # schema 自由换成任意收到的通道。
        self.channel_picker = ChannelPicker(
            header, bindings=self.spec.bindings, multiple=False,
            min_selected=self.MIN_BINDINGS, max_selected=self.MAX_BINDINGS,
            empty_text="选择数据", on_change=self._set_bindings_from_picker,
            width=16,
        )
        self.channel_picker.pack(side=tk.RIGHT)
        row = ttk.Frame(self.frame)
        row.pack(fill=tk.BOTH, expand=True)
        self.value_label = ttk.Label(row, textvariable=self.value_var, style=DASH_METRIC_STYLE)
        self.value_label.pack(side=tk.LEFT, anchor=tk.W)
        ttk.Label(row, textvariable=self.unit_var, style="Muted.TLabel").pack(
            side=tk.LEFT, anchor=tk.S, padx=(6, 0)
        )

    def rebind(self) -> None:
        super().rebind()
        self.name_var.set(self.title())
        self.channel_picker.set_channels(
            self.context.all_channels(), selected=self.spec.bindings
        )
        if not self.spec.bindings:
            self._set_value_text("选择数据")
            self.unit_var.set("")
            self._set_metric_style(DASH_METRIC_WARN_STYLE)
            return
        if self.missing:
            self._set_value_text(MISSING_CHANNEL_TEXT)
            self.unit_var.set("")
            self._set_metric_style(DASH_METRIC_FAIL_STYLE)
            return
        channel = self.context.channel(self.spec.bindings[0]) if self.spec.bindings else None
        self.unit_var.set("" if channel is None or channel.unit == "-" else channel.unit)
        self._set_metric_style(DASH_METRIC_STYLE)
        if self.value_var.get() == MISSING_CHANNEL_TEXT:
            # 通道回来了（换固件 / 重拉表）。不清掉这行旧的报缺文字，卡片就会
            # 一直显示"通道不存在"直到下一个样本到达——而在等数据的这几百毫秒
            # 里，用户看到的是一条已经不成立的错误。
            self._set_value_text("—")

    def refresh(self) -> None:
        if self.missing or not self.spec.bindings:
            return
        latest = self.context.latest(self.spec.bindings[0])
        if latest is None:
            self._set_value_text("—")
            return
        digits = int(self.spec.options.get("digits", 3))
        self._set_value_text(f"{latest:+.{digits}f}")
        self._set_metric_style(self._style_for(latest))

    def _set_value_text(self, text: str) -> None:
        """跨 Tcl 边界的 StringVar 写入只在可见文本真的变化时发生。"""
        if text != self._last_value_text:
            self.value_var.set(text)
            self._last_value_text = text

    def _set_metric_style(self, style: str) -> None:
        """20 张数值卡每帧重复 configure 同一 style 会挤占 resize 的主线程。"""
        if style != self._last_metric_style:
            self.value_label.configure(style=style)
            self._last_metric_style = style

    def _style_for(self, value: float) -> str:
        """可选的上下限着色。没配阈值就一律中性色，不自作主张。"""
        warn = self.spec.options.get("warn")
        alarm = self.spec.options.get("alarm")
        try:
            if alarm is not None and abs(value) >= float(alarm):
                return DASH_METRIC_FAIL_STYLE
            if warn is not None and abs(value) >= float(warn):
                return DASH_METRIC_WARN_STYLE
        except (TypeError, ValueError):
            return DASH_METRIC_STYLE
        return DASH_METRIC_STYLE


class ParamTile(DashboardTile):
    """参数滑块卡：大字回显 + 滑块 + 数值输入框 + 三态着色。

    输入框是 R-T1-3 缺的那一半：滑块拖不到精确值，而 PID 参数经常要输入
    "0.0671" 这种数。回车/失焦发送，与滑块共用同一套节流与三态判定。
    """

    TYPE = TILE_PARAM
    LABEL = "参数滑块卡"
    MIN_BINDINGS = 1
    MAX_BINDINGS = 1
    PARAM_ONLY = True

    def _build(self) -> None:
        self.name_var = tk.StringVar(value=self.title())
        self.value_var = tk.StringVar(value="—")
        self.state_var = tk.StringVar(value=PARAM_STATE_IDLE)
        self.feedback_var = tk.StringVar(value=PARAM_FEEDBACK_TEXT[PARAM_STATE_IDLE])
        self.entry_var = tk.StringVar(value="")
        self.scale_var = tk.DoubleVar(value=0.0)

        ttk.Label(self.frame, textvariable=self.name_var, style="Eyebrow.TLabel").pack(anchor=tk.W)
        head = ttk.Frame(self.frame)
        head.pack(fill=tk.X)
        self.value_label = ttk.Label(head, textvariable=self.value_var, style=DASH_METRIC_STYLE)
        self.value_label.pack(side=tk.LEFT)
        feedback = ttk.Frame(head)
        feedback.pack(side=tk.LEFT, anchor=tk.S, padx=(8, 0))
        style = ttk.Style(self.frame)
        background = (
            style.lookup("Card.TFrame", "background")
            or style.lookup("TFrame", "background")
            or "#202733"
        )
        self.feedback_dot = tk.Canvas(
            feedback, width=10, height=10, highlightthickness=0, borderwidth=0,
            background=background,
        )
        self._feedback_dot = self.feedback_dot.create_oval(
            1, 1, 9, 9, fill=PARAM_FEEDBACK_IDLE_COLOUR, outline=""
        )
        self.feedback_dot.pack(side=tk.LEFT, padx=(0, 3))
        ttk.Label(feedback, textvariable=self.feedback_var, style="Muted.TLabel").pack(side=tk.LEFT)

        row = ttk.Frame(self.frame)
        row.pack(fill=tk.X, pady=(2, 0))
        self.scale = ttk.Scale(
            row, from_=0.0, to=1.0, variable=self.scale_var, orient=tk.HORIZONTAL,
            command=lambda _value: self._on_drag(),
            style=PARAM_SCALE_STYLES[PARAM_STATE_IDLE],
        )
        self.scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.scale.bind("<ButtonRelease-1>", lambda _event: self._on_release())
        self.entry = ttk.Entry(row, textvariable=self.entry_var, width=8)
        self.entry.pack(side=tk.LEFT, padx=(6, 0))
        self.entry.bind("<Return>", lambda _event: self._on_entry_commit())
        self.entry.bind("<FocusOut>", lambda _event: self._on_entry_commit())

    def rebind(self) -> None:
        super().rebind()
        self.name_var.set(self.title())
        if self.missing or not self.spec.bindings:
            self.value_var.set(MISSING_CHANNEL_TEXT)
            self.value_label.configure(style=DASH_METRIC_FAIL_STYLE)
            self.state_var.set(PARAM_STATE_DIVERGED)
            self._set_feedback_visual(PARAM_STATE_DIVERGED, "通道不存在")
            self.scale.state(["disabled"])
            self.entry.state(["disabled"])
            return
        channel = self.context.channel(self.spec.bindings[0])
        self.scale.configure(from_=channel.minimum, to=channel.maximum)
        self.scale.state(["!disabled"])
        self.entry.state(["!disabled"])
        if self.value_var.get() == MISSING_CHANNEL_TEXT:
            self.value_var.set("—")
        self._render()

    @property
    def tracker(self) -> ParamEchoTracker | None:
        if self.missing or not self.spec.bindings:
            return None
        return self.context.param_tracker(self.spec.bindings[0])

    def _on_drag(self) -> None:
        tracker = self.tracker
        if tracker is None:
            return
        now = time.monotonic()
        if not tracker.may_send(now):
            return
        self._send(float(self.scale_var.get()), now)

    def _on_release(self) -> None:
        # 松手必发：节流窗口内的最后一次拖动否则会被吃掉，滑块停在 A、固件
        # 停在 B，而且没有任何提示。
        if self.tracker is not None:
            self._send(float(self.scale_var.get()), time.monotonic())

    def _on_entry_commit(self) -> None:
        text = self.entry_var.get().strip()
        if not text or self.tracker is None:
            return
        try:
            value = float(text)
        except ValueError:
            # 输错了就退回当前回显值，不发。静默发一个 0 出去比什么都不做糟糕。
            self._sync_entry()
            return
        self.scale_var.set(value)
        self._send(value, time.monotonic())

    def _send(self, value: float, now: float) -> None:
        name = self.spec.bindings[0]
        tracker = self.tracker
        if tracker is None:
            return
        if not self.context.send_param(name, value):
            return
        tracker.note_sent(value, now)
        self._render()

    def _sync_entry(self) -> None:
        tracker = self.tracker
        if tracker is not None and tracker.display is not None:
            self.entry_var.set(f"{tracker.display:.6g}")

    def refresh(self) -> None:
        tracker = self.tracker
        if tracker is None:
            return
        now = time.monotonic()
        latest = self.context.latest(self.spec.bindings[0])
        if latest is not None:
            action = tracker.note_echo(latest, now)
            if action == "follow" and tracker.display is not None:
                self.scale_var.set(tracker.display)
        else:
            tracker.expire_if_needed(now)
        self._render()

    def _set_feedback_visual(self, state: str, text: str | None = None) -> None:
        colour = {
            PARAM_STATE_IDLE: PARAM_FEEDBACK_IDLE_COLOUR,
            PARAM_STATE_PENDING: PARAM_FEEDBACK_PENDING_COLOUR,
            PARAM_STATE_CONFIRMED: PARAM_FEEDBACK_CONFIRMED_COLOUR,
            PARAM_STATE_DIVERGED: PARAM_FEEDBACK_DIVERGED_COLOUR,
        }.get(state, PARAM_FEEDBACK_IDLE_COLOUR)
        self.feedback_dot.itemconfigure(self._feedback_dot, fill=colour)
        self.feedback_var.set(text or PARAM_FEEDBACK_TEXT.get(state, "等待飞控数据"))
        self.scale.configure(style=PARAM_SCALE_STYLES.get(state, PARAM_SCALE_STYLES[PARAM_STATE_IDLE]))

    def _render(self) -> None:
        tracker = self.tracker
        state = tracker.state if tracker is not None else PARAM_STATE_IDLE
        value = tracker.display if tracker is not None else None
        self.value_var.set("—" if value is None else f"{value:.4g}")
        self.state_var.set(state)
        self._set_feedback_visual(
            state, tracker.reason if tracker is not None else None)
        self.value_label.configure(style={
            PARAM_STATE_CONFIRMED: DASH_METRIC_PASS_STYLE,
            PARAM_STATE_DIVERGED: DASH_METRIC_FAIL_STYLE,
        }.get(state, DASH_METRIC_STYLE))
        # ttk 的下拉列表是 Tcl 内部窗口（路径含 ``popdown``），不在
        # Tkinter 的 children 表里；focus_get() 会尝试反解成 Python Widget
        # 并抛 KeyError。这里只需判断输入框自身是否持有焦点，比较原始 Tcl
        # 路径既保留“编辑中不覆盖”，也能安全容纳所有 Tcl-only 窗口。
        focused_path = str(self.frame.tk.call("focus"))
        if value is not None and focused_path != str(self.entry):
            self.entry_var.set(f"{value:.6g}")


TILE_CLASSES: dict[str, type[DashboardTile]] = {
    WaveTile.TYPE: WaveTile,
    ValueTile.TYPE: ValueTile,
    ParamTile.TYPE: ParamTile,
}


def register_tile(cls: type[DashboardTile]) -> type[DashboardTile]:
    """给第二批组件（R-T1-5b）用的注册入口，避免它们回头改这个表。"""
    TILE_CLASSES[cls.TYPE] = cls
    return cls


def build_tile(parent: tk.Misc, spec: TileSpec,
               context: TileContext) -> DashboardTile | None:
    factory = TILE_CLASSES.get(spec.type)
    if factory is None:
        return None
    return factory(parent, spec, context)


__all__ = [
    "DASH_METRIC_FAIL_STYLE",
    "DASH_METRIC_PASS_STYLE",
    "DASH_METRIC_STYLE",
    "DASH_METRIC_WARN_STYLE",
    "MISSING_CHANNEL_TEXT",
    "PARAM_ECHO_GRACE_S",
    "PARAM_ECHO_TIMEOUT_S",
    "PARAM_ECHO_TOLERANCE",
    "PARAM_FEEDBACK_CONFIRMED_COLOUR",
    "PARAM_FEEDBACK_DIVERGED_COLOUR",
    "PARAM_FEEDBACK_IDLE_COLOUR",
    "PARAM_FEEDBACK_PENDING_COLOUR",
    "PARAM_FEEDBACK_TEXT",
    "PARAM_STATE_CONFIRMED",
    "PARAM_STATE_DIVERGED",
    "PARAM_STATE_IDLE",
    "PARAM_STATE_PENDING",
    "PARAM_THROTTLE_S",
    "TILE_CLASSES",
    "WAVE_MAX_BINDINGS",
    "DashboardTile",
    "ParamEchoTracker",
    "ParamTile",
    "TileContext",
    "ValueTile",
    "WaveTile",
    "build_tile",
    "ensure_dashboard_styles",
    "register_tile",
]
