"""纯 Tk Canvas 示波器控件（无 matplotlib 依赖）。

为什么不用 matplotlib / pyqtgraph（规划文档 §2.7 的裁决，摘要放在这里）：
matplotlib 的 `FigureCanvasTkAgg` 每帧都要重画整张位图，8 条曲线 × 10 k 点
根本跑不到 30 fps；pyqtgraph 快，但 Qt 事件循环进不了 Tk 进程，只能开独立进程，
而链路（尤其是独占的串口）在面板手里，于是还得加一层本地 UDP 镜像，滑块和波形
分在两个窗口——回到了"外部上位机"的体验，正是本期要摆脱的东西。

Tk Canvas 够用的关键在于**按像素列做 min/max 抽稀**：屏幕上一列像素最多只能
表达两个值（那一列的最小值和最大值），所以无论缓冲里有多少点，每条曲线送进
Tcl 的坐标数都只和画布宽度有关，与数据量无关。抽稀用 numpy 的 reshape+min/max
一次算完，然后 `canvas.coords(line_id, ...)` 一次性替换整条折线的坐标——不是
删了重建 line item，那会让 Tk 每帧重新分配。

本模块只管画，不碰协议、不碰链路。
"""

from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass, field

import numpy as np


# 33 ms ≈ 30 fps。再快人眼看不出来，只是把 Tk 主循环的时间烧掉。
SCOPE_RENDER_PERIOD_MS = 33

SCOPE_WINDOW_CHOICES_S = (1.0, 5.0, 10.0, 30.0, 60.0)

# 一条曲线最多送进 Tcl 的点数（= 画布像素列 × 2）。上限本身也钉死，避免有人
# 把窗口拉到 4 K 宽之后每帧往 Tcl 里灌几万个数。
SCOPE_MAX_COLUMNS = 900

SCOPE_PALETTE = (
    "#1f77b4", "#d62728", "#2ca02c", "#ff7f0e",
    "#9467bd", "#17becf", "#8c564b", "#e377c2",
)

SCOPE_PADDING = (52, 10, 12, 24)   # left, top, right, bottom


def decimate_min_max(
    times: np.ndarray, values: np.ndarray, columns: int
) -> tuple[np.ndarray, np.ndarray]:
    """按像素列做 min/max 抽稀。

    每一列出两个点（最小值、最大值），这样窄尖峰不会因为抽稀而消失——直接
    等间隔取样（`values[::step]`）会把毛刺整个漏掉，而毛刺往往正是要看的东西。
    """
    count = values.shape[0]
    if columns <= 0 or count == 0:
        return times[:0], values[:0]
    if count <= columns * 2:
        return times, values

    per_column = count // columns
    used = per_column * columns
    grid = values[:used].reshape(columns, per_column)
    lows = grid.min(axis=1)
    highs = grid.max(axis=1)
    column_times = times[:used:per_column]

    out_t = np.repeat(column_times, 2)
    out_v = np.empty(columns * 2, dtype=values.dtype)
    out_v[0::2] = lows
    out_v[1::2] = highs

    if used < count:
        # 尾巴上不足一列的点同样压成 min/max 再补一个真实的最新点。
        # 直接把整条尾巴接上是不行的：`count // columns` 的余数最大接近 columns，
        # 那样送进 Tcl 的点数就又跟数据量挂钩了，抽稀等于白做。补最新点是为了
        # 让曲线右端始终落在真实的最后一个样本上，否则波形看着像卡了一拍。
        tail_t = times[used:]
        tail_v = values[used:]
        out_t = np.concatenate((out_t, [tail_t[0], tail_t[0], tail_t[-1]]))
        out_v = np.concatenate((out_v, [tail_v.min(), tail_v.max(), tail_v[-1]]))
    return out_t, out_v


@dataclass
class ScopeCurve:
    key: int
    label: str
    colour: str
    item: int = 0
    visible: bool = True
    points: int = 0
    latest: float = float("nan")


@dataclass
class ScopeRange:
    auto: bool = True
    low: float = -1.0
    high: float = 1.0

    def apply(self, values_low: float, values_high: float) -> tuple[float, float]:
        if not self.auto:
            return self.low, self.high
        if not np.isfinite(values_low) or not np.isfinite(values_high):
            return -1.0, 1.0
        if values_high - values_low < 1e-9:
            centre = values_high
            return centre - 0.5, centre + 0.5
        margin = (values_high - values_low) * 0.08
        return values_low - margin, values_high + margin


class ScopeCanvas:
    """时间窗口 + 自动/手动量程 + 暂停 + 光标读数的 Canvas 折线示波器。"""

    def __init__(self, parent: tk.Misc, *, width: int = 900, height: int = 320) -> None:
        self.canvas = tk.Canvas(
            parent, width=width, height=height, background="#101418",
            highlightthickness=0,
        )
        self.window_s = 10.0
        self.paused = False
        self.range = ScopeRange()
        self.curves: dict[int, ScopeCurve] = {}
        self._grid_items: list[int] = []
        self._cursor_item = 0
        self._cursor_x: int | None = None
        self.cursor_readout: dict[int, float] = {}
        self.last_span: tuple[float, float] = (0.0, 1.0)
        self.last_range: tuple[float, float] = (-1.0, 1.0)
        self._frozen: dict[int, tuple[np.ndarray, np.ndarray]] = {}

        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<Leave>", self._on_leave)

    # ------------------------------------------------------------- 曲线

    def set_curves(self, entries: list[tuple[int, str]]) -> None:
        """重建曲线集合。entries = [(通道号, 显示名), ...]，顺序即配色顺序。"""
        for curve in self.curves.values():
            if curve.item:
                self.canvas.delete(curve.item)
        self.curves = {}
        self._frozen = {}
        for position, (key, label) in enumerate(entries):
            colour = SCOPE_PALETTE[position % len(SCOPE_PALETTE)]
            item = self.canvas.create_line(
                0, 0, 0, 0, fill=colour, width=1.4, state=tk.HIDDEN
            )
            self.curves[key] = ScopeCurve(key=key, label=label, colour=colour, item=item)

    def set_window(self, seconds: float) -> None:
        self.window_s = max(0.05, float(seconds))

    def set_paused(self, paused: bool) -> None:
        self.paused = bool(paused)

    def set_manual_range(self, low: float, high: float) -> None:
        if high <= low:
            return
        self.range = ScopeRange(auto=False, low=float(low), high=float(high))

    def set_auto_range(self) -> None:
        self.range = ScopeRange(auto=True)

    # ------------------------------------------------------------- 绘制

    def plot_area(self) -> tuple[int, int, int, int]:
        left, top, right, bottom = SCOPE_PADDING
        width = max(int(self.canvas.winfo_width()), 1)
        height = max(int(self.canvas.winfo_height()), 1)
        if width <= 1:
            width = int(self.canvas.cget("width"))
        if height <= 1:
            height = int(self.canvas.cget("height"))
        return left, top, max(width - right, left + 1), max(height - bottom, top + 1)

    def render(self, series: dict[int, tuple[np.ndarray, np.ndarray]]) -> None:
        """把每通道的 (t_seconds, values) 快照画出来。

        暂停时冻结的是**上一帧的数据**而不是停止取数：环形缓冲照常在收线程里
        写，松开暂停立刻能看到这段时间真实发生过的波形，而不是一段空白。
        """
        if self.paused:
            series = self._frozen
        else:
            self._frozen = series

        left, top, right, bottom = self.plot_area()
        columns = min(max(right - left, 1), SCOPE_MAX_COLUMNS)

        newest = 0.0
        for times, _values in series.values():
            if times.size:
                newest = max(newest, float(times[-1]))
        oldest = newest - self.window_s
        self.last_span = (oldest, newest)

        windowed: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        low = float("inf")
        high = float("-inf")
        for key, (times, values) in series.items():
            curve = self.curves.get(key)
            if curve is None or not curve.visible or times.size == 0:
                continue
            start = int(np.searchsorted(times, oldest, side="left"))
            slice_t = times[start:]
            slice_v = values[start:]
            if slice_v.size == 0:
                continue
            slice_t, slice_v = decimate_min_max(slice_t, slice_v, columns)
            windowed[key] = (slice_t, slice_v)
            low = min(low, float(slice_v.min()))
            high = max(high, float(slice_v.max()))

        value_low, value_high = self.range.apply(low, high)
        self.last_range = (value_low, value_high)
        self._draw_grid(left, top, right, bottom, value_low, value_high)

        span = max(self.window_s, 1e-6)
        scale = (right - left) / span
        value_span = max(value_high - value_low, 1e-9)
        vscale = (bottom - top) / value_span

        for key, curve in self.curves.items():
            entry = windowed.get(key)
            if entry is None:
                curve.points = 0
                self.canvas.itemconfigure(curve.item, state=tk.HIDDEN)
                continue
            slice_t, slice_v = entry
            # 一次性把 (x, y) 交错进一个 numpy 数组再 tolist()：分别 tolist()
            # 再切片赋值会多两次 list 转换，在 8 曲线 × 每帧的量级下是可测的。
            pairs = np.empty(slice_t.size * 2, dtype=np.float64)
            np.multiply(slice_t - oldest, scale, out=pairs[0::2])
            pairs[0::2] += left
            np.multiply(slice_v - value_low, -vscale, out=pairs[1::2])
            pairs[1::2] += bottom
            flat = pairs.tolist()
            curve.points = int(slice_v.size)
            curve.latest = float(slice_v[-1])
            if curve.points < 2:
                self.canvas.itemconfigure(curve.item, state=tk.HIDDEN)
                continue
            self.canvas.coords(curve.item, *flat)
            self.canvas.itemconfigure(curve.item, state=tk.NORMAL)

        self._update_cursor(left, top, right, bottom, oldest, scale, windowed)

    def _draw_grid(self, left: int, top: int, right: int, bottom: int,
                   value_low: float, value_high: float) -> None:
        """网格与刻度只建一次，之后每帧只改坐标和文本。

        每帧 delete + create 十来个 item 看着没什么，但那是 30 fps × 一直开着
        的窗口；Tk 每次都要重新分配 item id 并整块重排显示列表。
        """
        rows = 4
        if not self._grid_items:
            for _ in range(rows + 1):
                self._grid_items.append(self.canvas.create_line(0, 0, 0, 0, fill="#26303a"))
                self._grid_items.append(
                    self.canvas.create_text(
                        0, 0, text="", anchor=tk.E, fill="#7c8b99",
                        font=("TkDefaultFont", 7),
                    )
                )
            self._grid_items.append(
                self.canvas.create_text(
                    0, 0, text="", anchor=tk.E, fill="#7c8b99",
                    font=("TkDefaultFont", 7),
                )
            )

        for index in range(rows + 1):
            y = bottom - (bottom - top) * index / rows
            self.canvas.coords(self._grid_items[index * 2], left, y, right, y)
            label = value_low + (value_high - value_low) * index / rows
            self.canvas.coords(self._grid_items[index * 2 + 1], left - 6, y)
            self.canvas.itemconfigure(self._grid_items[index * 2 + 1], text=f"{label:.3g}")

        self.canvas.coords(self._grid_items[-1], right, bottom + 10)
        self.canvas.itemconfigure(self._grid_items[-1], text=f"{self.window_s:g} s")

    # ------------------------------------------------------------- 光标

    def _on_motion(self, event: tk.Event) -> None:
        self._cursor_x = int(event.x)

    def _on_leave(self, _event: tk.Event) -> None:
        self._cursor_x = None
        self.cursor_readout = {}
        if self._cursor_item:
            self.canvas.itemconfigure(self._cursor_item, state=tk.HIDDEN)

    def _update_cursor(self, left: int, top: int, right: int, bottom: int,
                       oldest: float, scale: float,
                       windowed: dict[int, tuple[np.ndarray, np.ndarray]]) -> None:
        if self._cursor_item == 0:
            self._cursor_item = self.canvas.create_line(
                0, 0, 0, 0, fill="#5a6b7a", dash=(2, 2), state=tk.HIDDEN
            )
        if self._cursor_x is None or not (left <= self._cursor_x <= right):
            self.canvas.itemconfigure(self._cursor_item, state=tk.HIDDEN)
            self.cursor_readout = {}
            return

        self.canvas.coords(self._cursor_item, self._cursor_x, top, self._cursor_x, bottom)
        self.canvas.itemconfigure(self._cursor_item, state=tk.NORMAL)
        at = oldest + (self._cursor_x - left) / max(scale, 1e-9)
        readout: dict[int, float] = {}
        for key, (slice_t, slice_v) in windowed.items():
            index = int(np.searchsorted(slice_t, at, side="left"))
            index = min(max(index, 0), slice_v.size - 1)
            readout[key] = float(slice_v[index])
        self.cursor_readout = readout


__all__ = [
    "SCOPE_MAX_COLUMNS",
    "SCOPE_PALETTE",
    "SCOPE_RENDER_PERIOD_MS",
    "SCOPE_WINDOW_CHOICES_S",
    "ScopeCanvas",
    "ScopeCurve",
    "ScopeRange",
    "decimate_min_max",
]
