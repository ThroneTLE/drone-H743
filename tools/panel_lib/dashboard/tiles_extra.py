"""第二批组件（R-T1-5b）：仪表盘、命令按钮/开关、2D 姿态、全通道实时值列表。

放在 `tiles.py` 之外是为了守住每文件 ≤800 行；注册走 `tiles.register_tile`，
所以第一批那张表不用回头改。

**命令按钮这一类要特别说一句**：它是本工作台里唯一能主动往飞控发任意文本的
组件。它一律走 `TileContext.send_command()`，而那条路必过面板的
`_validation_command_allowed` 安全门（V0 只读会话、固件升级期间、`BOOT` 授权
等等）。组件自己不认识 transport，也就没有绕过去的路。
"""

from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk

from ..scope import SCOPE_PALETTE
from .channel_picker import ChannelPicker
from .layout import (
    TILE_ATTITUDE,
    TILE_BUTTON,
    TILE_CHANNELS,
    TILE_GAUGE,
)
from .tiles import (
    DASH_METRIC_STYLE,
    MISSING_CHANNEL_TEXT,
    DashboardTile,
    register_tile,
)


# 全通道实时值列表刷新到 10 Hz 就够读了：28 行文本每 33 ms 重写一遍纯属浪费，
# 而且数字跳得太快人反而读不出来。
CHANNEL_LIST_PERIOD_S = 0.1
CHANNEL_LIST_DOT_SIZE = 9

GAUGE_START_DEG = 210.0     # 弧形表起点（左下）
GAUGE_SWEEP_DEG = -240.0    # 顺时针扫过的角度

SELECT_CHANNEL_TEXT = "请选择通道"
SELECT_ATTITUDE_CHANNELS_TEXT = "请选择 roll 和 pitch 通道"

BUTTON_MODE_PUSH = "push"
BUTTON_MODE_TOGGLE = "toggle"


@register_tile
class GaugeTile(DashboardTile):
    """弧形仪表：量程直接取通道表的 min/max，不另设一套。

    量程来自通道表而不是选项，是为了让"表上刻度"和"固件认为的合理范围"永远
    是同一个数；两套量程迟早会漂移，而漂移之后指针指哪儿就没人说得清了。
    """

    TYPE = TILE_GAUGE
    LABEL = "仪表盘"
    MIN_BINDINGS = 1
    MAX_BINDINGS = 1
    PARAM_ONLY = None
    DEFAULT_SPAN = (3, 3)

    def _build(self) -> None:
        self.name_var = tk.StringVar(value=self.title())
        header = ttk.Frame(self.frame)
        header.pack(fill=tk.X)
        ttk.Label(header, textvariable=self.name_var,
                  style="Eyebrow.TLabel").pack(side=tk.LEFT)
        self.channel_picker = ChannelPicker(
            header, bindings=self.spec.bindings, multiple=False,
            min_selected=self.MIN_BINDINGS, max_selected=self.MAX_BINDINGS,
            empty_text="选择数据", on_change=self._set_bindings_from_picker,
            width=16,
        )
        self.channel_picker.pack(side=tk.RIGHT)
        self.canvas = tk.Canvas(self.frame, highlightthickness=0, background="#101418")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.value_var = tk.StringVar(value="—")
        ttk.Label(self.frame, textvariable=self.value_var,
                  style=DASH_METRIC_STYLE).pack(anchor=tk.CENTER)
        self._arc = self.canvas.create_arc(
            0, 0, 1, 1, start=GAUGE_START_DEG, extent=GAUGE_SWEEP_DEG,
            style=tk.ARC, outline="#2b3742", width=10,
        )
        self._value_arc = self.canvas.create_arc(
            0, 0, 1, 1, start=GAUGE_START_DEG, extent=0.0,
            style=tk.ARC, outline="#1f77b4", width=10,
        )
        self._needle = self.canvas.create_line(0, 0, 0, 0, fill="#d0d7de", width=2)
        self._low = self.canvas.create_text(0, 0, text="", fill="#7c8b99",
                                            font=("TkDefaultFont", 7))
        self._high = self.canvas.create_text(0, 0, text="", fill="#7c8b99",
                                             font=("TkDefaultFont", 7))

    def rebind(self) -> None:
        super().rebind()
        self.name_var.set(self.title())
        self.channel_picker.set_channels(
            self.context.all_channels(), selected=self.spec.bindings
        )
        if not self.spec.bindings:
            self._show_unavailable(SELECT_CHANNEL_TEXT)
            return
        if self.missing:
            self._show_unavailable(MISSING_CHANNEL_TEXT)
            return
        channel = self.context.channel(self.spec.bindings[0])
        if channel is None:
            # `rebind()` 之后通道表不会主动变，但上下文实现仍不该让 UI 在这条
            # 边界上崩掉。这里与 `missing` 分支同样明确报缺。
            self._show_unavailable(MISSING_CHANNEL_TEXT)
            return
        self.canvas.itemconfigure(self._low, text=f"{channel.minimum:g}")
        self.canvas.itemconfigure(self._high, text=f"{channel.maximum:g}")
        # 重新绑定后上一通道的指针/读数都不能留着，否则一张已经换通道的表会
        # 短暂地显示旧数据。等下一帧真实样本再填入。
        self._reset_indicator()
        self.value_var.set("—")

    def _reset_indicator(self) -> None:
        self.canvas.itemconfigure(self._value_arc, extent=0.0)
        self.canvas.itemconfigure(self._needle, state=tk.HIDDEN)

    def _show_unavailable(self, text: str) -> None:
        self.value_var.set(text)
        self.canvas.itemconfigure(self._low, text="")
        self.canvas.itemconfigure(self._high, text="")
        self._reset_indicator()

    def refresh(self) -> None:
        if not self.spec.bindings:
            self._show_unavailable(SELECT_CHANNEL_TEXT)
            return
        if self.missing:
            self._show_unavailable(MISSING_CHANNEL_TEXT)
            return
        name = self.spec.bindings[0]
        channel = self.context.channel(name)
        if channel is None:
            self._show_unavailable(MISSING_CHANNEL_TEXT)
            return
        latest = self.context.latest(name)
        width = max(int(self.canvas.winfo_width()), 40)
        height = max(int(self.canvas.winfo_height()), 40)
        size = min(width, height) - 12
        cx, cy = width / 2.0, height / 2.0 + size * 0.12
        box = (cx - size / 2, cy - size / 2, cx + size / 2, cy + size / 2)
        self.canvas.coords(self._arc, *box)
        self.canvas.coords(self._value_arc, *box)
        self.canvas.coords(self._low, cx - size * 0.46, cy + size * 0.30)
        self.canvas.coords(self._high, cx + size * 0.46, cy + size * 0.30)

        if latest is None:
            self.value_var.set("—")
            self._reset_indicator()
            return

        span = channel.maximum - channel.minimum
        fraction = 0.0 if span <= 0 else (latest - channel.minimum) / span
        fraction = min(max(fraction, 0.0), 1.0)
        angle = GAUGE_START_DEG + GAUGE_SWEEP_DEG * fraction
        self.canvas.itemconfigure(self._value_arc, extent=GAUGE_SWEEP_DEG * fraction)
        self.canvas.itemconfigure(self._needle, state=tk.NORMAL)
        radius = size * 0.42
        self.canvas.coords(
            self._needle, cx, cy,
            cx + radius * math.cos(math.radians(angle)),
            cy - radius * math.sin(math.radians(angle)),
        )
        unit = "" if channel.unit == "-" else f" {channel.unit}"
        self.value_var.set(f"{latest:+.3f}{unit}")


@register_tile
class ButtonTile(DashboardTile):
    """命令按钮 / 开关：把一条文本命令绑到一个按钮上。

    `push` 模式发一条 `command`；`toggle` 模式在 `command_on` / `command_off`
    之间切。命令一律经 `TileContext.send_command()` 出去，那条路必过面板的
    `_validation_command_allowed`——组件自己拿不到 transport，绕不过去。
    """

    TYPE = TILE_BUTTON
    LABEL = "命令按钮"
    MIN_BINDINGS = 0
    MAX_BINDINGS = 0
    PARAM_ONLY = None
    DEFAULT_SPAN = (3, 1)
    OPTION_FIELDS = (
        ("label", "按钮文字"),
        ("command", "命令（push）"),
        ("command_on", "命令（开）"),
        ("command_off", "命令（关）"),
        ("mode", "模式 push/toggle"),
    )

    def _build(self) -> None:
        self.state_var = tk.StringVar(value="")
        self.button = ttk.Button(self.frame, text=self._button_text(),
                                 command=self._on_click)
        self.button.pack(fill=tk.X)
        ttk.Label(self.frame, textvariable=self.state_var,
                  style="Muted.TLabel").pack(anchor=tk.W)
        self._toggled = False

    def _button_text(self) -> str:
        label = self.spec.options.get("label")
        if isinstance(label, str) and label.strip():
            return label.strip()
        return str(self.spec.options.get("command") or self.LABEL)

    @property
    def mode(self) -> str:
        return (BUTTON_MODE_TOGGLE
                if str(self.spec.options.get("mode", "")).strip() == BUTTON_MODE_TOGGLE
                else BUTTON_MODE_PUSH)

    def rebind(self) -> None:
        super().rebind()
        self.button.configure(text=self._button_text())
        self.state_var.set("")

    def command_for_click(self) -> str:
        if self.mode == BUTTON_MODE_TOGGLE:
            key = "command_off" if self._toggled else "command_on"
            return str(self.spec.options.get(key) or "")
        return str(self.spec.options.get("command") or "")

    def _on_click(self) -> None:
        command = self.command_for_click().strip()
        if not command:
            # 没配命令就明说，而不是点了没反应。
            self.state_var.set("未配置命令")
            return
        accepted = self.context.send_command(command)
        if not accepted:
            # 安全门挡下了（V0 只读会话、固件升级中、未连接……）。必须让人看见
            # 是"没发出去"，否则操作者会以为飞控收到了却没反应。
            self.state_var.set(f"已拦截：{command}")
            return
        if self.mode == BUTTON_MODE_TOGGLE:
            self._toggled = not self._toggled
            self.button.configure(text=self._button_text())
        self.state_var.set(f"已发送：{command}")


@register_tile
class AttitudeTile(DashboardTile):
    """2D 姿态：roll/pitch 地平仪 + yaw 罗盘。

    绑定顺序固定为 roll、pitch、yaw（yaw 可省）。刻意不做 3D：本页要的是
    "现在什么姿态"，2D 地平仪一眼就能读，3D 还要先在脑子里转一下。
    """

    TYPE = TILE_ATTITUDE
    LABEL = "姿态指示"
    MIN_BINDINGS = 2
    MAX_BINDINGS = 3
    PARAM_ONLY = False
    DEFAULT_SPAN = (4, 4)

    def _build(self) -> None:
        self.name_var = tk.StringVar(value=self.title())
        ttk.Label(self.frame, textvariable=self.name_var,
                  style="Eyebrow.TLabel").pack(anchor=tk.W)
        self.readout_var = tk.StringVar(value="—")
        self.canvas = tk.Canvas(self.frame, highlightthickness=0, background="#101418")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        ttk.Label(self.frame, textvariable=self.readout_var,
                  style="Mono.TLabel").pack(anchor=tk.W)
        self._ground = self.canvas.create_polygon(0, 0, 0, 0, 0, 0, fill="#6b4a2f")
        self._sky = self.canvas.create_polygon(0, 0, 0, 0, 0, 0, fill="#24506e")
        self._horizon = self.canvas.create_line(0, 0, 0, 0, fill="#e6edf3", width=2)
        self._wing = self.canvas.create_line(0, 0, 0, 0, fill="#f2c744", width=2)
        self._compass = self.canvas.create_line(0, 0, 0, 0, fill="#7ee081", width=2)
        self._compass_ring = self.canvas.create_oval(0, 0, 0, 0, outline="#2b3742")

    def rebind(self) -> None:
        super().rebind()
        self.name_var.set(self.title())
        issue = self._required_binding_issue()
        if issue is not None:
            self._show_unavailable(issue)
            return
        # 即使 roll/pitch 已重新可用，也不能把上一张表的姿态留在画面上，等真实
        # 样本抵达；可选 yaw 单独缺失时则仍可安全显示地平仪。
        self.readout_var.set("—")
        self._hide_indicators()

    def _required_binding_issue(self) -> str | None:
        if len(self.spec.bindings) < self.MIN_BINDINGS:
            return SELECT_ATTITUDE_CHANNELS_TEXT
        missing = [
            name for name in self.spec.bindings[:self.MIN_BINDINGS]
            if self.context.channel(name) is None
        ]
        if missing:
            return f"{MISSING_CHANNEL_TEXT}：{', '.join(missing)}"
        return None

    def _hide_indicators(self) -> None:
        for item in (
            self._ground, self._sky, self._horizon, self._wing,
            self._compass, self._compass_ring,
        ):
            self.canvas.itemconfigure(item, state=tk.HIDDEN)

    def _show_indicators(self) -> None:
        for item in (self._ground, self._sky, self._horizon, self._wing, self._compass_ring):
            self.canvas.itemconfigure(item, state=tk.NORMAL)

    def _show_unavailable(self, text: str) -> None:
        self.readout_var.set(text)
        self._hide_indicators()

    def refresh(self) -> None:
        issue = self._required_binding_issue()
        if issue is not None:
            self._show_unavailable(issue)
            return
        roll = self.context.latest(self.spec.bindings[0])
        pitch = self.context.latest(self.spec.bindings[1])
        yaw_name = self.spec.bindings[2] if len(self.spec.bindings) > 2 else None
        yaw_available = yaw_name is not None and self.context.channel(yaw_name) is not None
        yaw = self.context.latest(yaw_name) if yaw_available else None
        if roll is None or pitch is None:
            self.readout_var.set("等待数据")
            self._hide_indicators()
            return

        self._show_indicators()

        width = max(int(self.canvas.winfo_width()), 40)
        height = max(int(self.canvas.winfo_height()), 40)
        cx, cy = width / 2.0, height / 2.0
        radius = min(width, height) / 2.0 - 6

        # 地平线：roll 转角度，pitch 平移（每度 radius/45 像素）。
        angle = math.radians(-roll)
        offset = pitch * (radius / 45.0)
        dx, dy = math.cos(angle), math.sin(angle)
        px, py = -dy, dx
        mx, my = cx + px * offset, cy + py * offset
        far = radius * 2.5
        x1, y1 = mx - dx * far, my - dy * far
        x2, y2 = mx + dx * far, my + dy * far
        self.canvas.coords(self._horizon, x1, y1, x2, y2)
        self.canvas.coords(
            self._sky, x1, y1, x2, y2, x2 - px * far, y2 - py * far, x1 - px * far, y1 - py * far
        )
        self.canvas.coords(
            self._ground, x1, y1, x2, y2, x2 + px * far, y2 + py * far, x1 + px * far, y1 + py * far
        )
        self.canvas.tag_raise(self._horizon)
        # 固定的机翼标记：地平线动、机翼不动，这是地平仪的读法。
        self.canvas.coords(self._wing, cx - radius * 0.4, cy, cx + radius * 0.4, cy)
        self.canvas.tag_raise(self._wing)

        ring = radius * 0.28
        self.canvas.coords(self._compass_ring, cx - ring, cy - ring, cx + ring, cy + ring)
        self.canvas.tag_raise(self._compass_ring)
        if yaw is None:
            self.canvas.itemconfigure(self._compass, state=tk.HIDDEN)
            text = f"roll {roll:+.1f}°  pitch {pitch:+.1f}°"
            if yaw_name is not None and not yaw_available:
                text += "（yaw 通道不存在）"
            self.readout_var.set(text)
        else:
            self.canvas.itemconfigure(self._compass, state=tk.NORMAL)
            heading = math.radians(90.0 - yaw)
            self.canvas.coords(
                self._compass, cx, cy,
                cx + ring * math.cos(heading), cy - ring * math.sin(heading),
            )
            self.canvas.tag_raise(self._compass)
            self.readout_var.set(
                f"roll {roll:+.1f}°  pitch {pitch:+.1f}°  yaw {yaw:+.1f}°"
            )


@register_tile
class ChannelListTile(DashboardTile):
    """全通道实时值列表：颜色点 + 名称 + 值 + 单位。

    不绑通道——它显示的就是整张通道表。刷新压到 10 Hz：28 行文本每 33 ms
    重写一遍纯属浪费，而且数字跳太快人反而读不出来。
    """

    TYPE = TILE_CHANNELS
    LABEL = "全通道列表"
    MIN_BINDINGS = 0
    MAX_BINDINGS = 0
    PARAM_ONLY = None
    DEFAULT_SPAN = (4, 6)

    def _build(self) -> None:
        ttk.Label(self.frame, text="全通道实时值", style="Eyebrow.TLabel").pack(anchor=tk.W)
        self.tree = ttk.Treeview(
            self.frame, columns=("value", "unit"), show="tree headings", height=8
        )
        self.tree.heading("#0", text="通道")
        self.tree.heading("value", text="值")
        self.tree.heading("unit", text="单位")
        self.tree.column("value", width=90, anchor=tk.E)
        self.tree.column("unit", width=54, anchor=tk.W)
        self.tree.pack(fill=tk.BOTH, expand=True)
        self._last_refresh = 0.0
        self._rows: list[str] = []
        # Treeview 只保存图片名，不保留 Python 对象；不持有引用时点会被 GC 后
        # 静默消失，和大字样式的 Font 问题同一个 Tk 生命周期陷阱。
        self._colour_dots: dict[str, tk.PhotoImage] = {}

    def _make_colour_dot(self, colour: str) -> tk.PhotoImage:
        image = tk.PhotoImage(master=self.tree, width=CHANNEL_LIST_DOT_SIZE,
                              height=CHANNEL_LIST_DOT_SIZE)
        centre = (CHANNEL_LIST_DOT_SIZE - 1) / 2.0
        radius_squared = (CHANNEL_LIST_DOT_SIZE * 0.38) ** 2
        for y in range(CHANNEL_LIST_DOT_SIZE):
            for x in range(CHANNEL_LIST_DOT_SIZE):
                if (x - centre) ** 2 + (y - centre) ** 2 <= radius_squared:
                    image.put(colour, to=(x, y))
        return image

    def rebind(self) -> None:
        super().rebind()
        for item in self.tree.get_children(""):
            self.tree.delete(item)
        self._rows = []
        self._colour_dots = {}
        for position, channel in enumerate(self.context.all_channels()):
            unit = "" if channel.unit == "-" else channel.unit
            channel_index = getattr(channel, "index", position)
            colour = SCOPE_PALETTE[channel_index % len(SCOPE_PALETTE)]
            dot = self._make_colour_dot(colour)
            self._colour_dots[channel.name] = dot
            self.tree.insert("", tk.END, iid=channel.name, text=channel.name, image=dot,
                             values=("—", unit))
            self._rows.append(channel.name)

    def refresh(self) -> None:
        now = time.monotonic()
        if now - self._last_refresh < CHANNEL_LIST_PERIOD_S:
            return
        self._last_refresh = now
        for name in self._rows:
            latest = self.context.latest(name)
            self.tree.set(name, "value", "—" if latest is None else f"{latest:+.4g}")


__all__ = [
    "BUTTON_MODE_PUSH",
    "BUTTON_MODE_TOGGLE",
    "CHANNEL_LIST_PERIOD_S",
    "SELECT_CHANNEL_TEXT",
    "AttitudeTile",
    "ButtonTile",
    "ChannelListTile",
    "GaugeTile",
]
