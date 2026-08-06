#!/usr/bin/env python3
"""H743 飞行日志通道波形查看器。"""

from __future__ import annotations

import argparse
import math
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Iterable

try:
    import numpy as np
    import pandas as pd
except Exception:  # pragma: no cover - depends on host optional packages
    np = None
    pd = None

try:
    from matplotlib import rcParams
    from matplotlib.backends.backend_tkagg import (
        FigureCanvasTkAgg,
        NavigationToolbar2Tk,
    )
    from matplotlib.figure import Figure

    # 中文注释：优先使用 Windows 常见中文字体，避免标题和图例显示成方块。
    rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    rcParams["axes.unicode_minus"] = False
except Exception:  # pragma: no cover - depends on host optional packages
    Figure = None
    FigureCanvasTkAgg = None
    NavigationToolbar2Tk = None


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_LOG_DIR = ROOT_DIR / "log"
DEFAULT_MAX_POINTS = 6000
DEFAULT_GAP_MS = 200.0
WHEEL_ZOOM_BASE = 1.25
WHEEL_ZOOM_MODES = ("总缩放", "横轴", "纵轴")


@dataclass(frozen=True)
class ChannelStats:
    count: int
    minimum: float
    maximum: float
    mean: float
    std: float
    latest: float


CHANNEL_GROUPS: dict[str, tuple[str, ...]] = {
    "姿态": (
        "roll",
        "pitch",
        "yaw",
        "gyro",
        "attitude",
        "rate_error",
        "target_attitude",
        "desired_attitude",
    ),
    "舵机": (
        "servo",
        "tilt",
    ),
    "电机": (
        "motor",
        "throttle",
        "thrust",
        "force",
    ),
    "光流": (
        "flow",
        "height",
    ),
    "速度位置": (
        "vel",
        "pos",
        "acc",
    ),
    "控制量": (
        "ctrl",
        "pid",
        "moment",
        "protect",
        "utilization",
        "ident",
    ),
    "遥控": (
        "rc_",
        "ch",
    ),
}


def discover_csv_files(folder: Path) -> list[Path]:
    # 中文注释：按修改时间倒序列出 CSV，最近下载/导出的日志会排在最上面。
    if not folder.exists() or not folder.is_dir():
        return []
    return sorted(
        (path for path in folder.glob("*.csv") if path.is_file()),
        key=lambda path: (path.stat().st_mtime, path.name),
        reverse=True,
    )


def human_size(size_bytes: int) -> str:
    units = ("B", "KB", "MB", "GB")
    value = float(size_bytes)
    for unit in units:
        if value < 1024.0 or unit == units[-1]:
            return f"{value:.1f} {unit}" if unit != "B" else f"{size_bytes} B"
        value /= 1024.0
    return f"{size_bytes} B"


def choose_time_column(columns: Iterable[str]) -> str:
    names = list(columns)
    for candidate in ("timestamp_us", "tick_ms", "time_s", "time_ms", "t"):
        if candidate in names:
            return candidate
    return "记录序号"


def is_numeric_column(frame: "pd.DataFrame", column: str) -> bool:
    if pd is None:
        return False
    return bool(pd.api.types.is_numeric_dtype(frame[column]))


def numeric_columns(frame: "pd.DataFrame") -> list[str]:
    # 中文注释：只展示能画成曲线的数值列，字符串模式名保留给后续扩展。
    return [column for column in frame.columns if is_numeric_column(frame, column)]


def channel_matches_group(channel: str, group_name: str) -> bool:
    keywords = CHANNEL_GROUPS.get(group_name, ())
    lowered = channel.lower()
    return any(keyword in lowered for keyword in keywords)


def summarize_series(values: "pd.Series") -> ChannelStats | None:
    numeric = pd.to_numeric(values, errors="coerce").dropna()
    if numeric.empty:
        return None
    return ChannelStats(
        count=int(numeric.size),
        minimum=float(numeric.min()),
        maximum=float(numeric.max()),
        mean=float(numeric.mean()),
        std=float(numeric.std(ddof=0)),
        latest=float(numeric.iloc[-1]),
    )


def format_float(value: float) -> str:
    if not math.isfinite(value):
        return ""
    if abs(value) >= 1000.0 or (0.0 < abs(value) < 0.001):
        return f"{value:.4e}"
    return f"{value:.5g}"


def downsample_indices(length: int, max_points: int) -> slice:
    if max_points <= 0 or length <= max_points:
        return slice(None)
    stride = max(1, math.ceil(length / max_points))
    return slice(None, None, stride)


def zoom_limits(low: float, high: float, center: float | None, scale: float) -> tuple[float, float]:
    # 中文注释：以鼠标位置为中心缩放坐标轴；无效鼠标位置时退回当前轴中心。
    if not all(math.isfinite(value) for value in (low, high, scale)):
        return low, high
    if low == high or scale <= 0.0:
        return low, high

    reversed_axis = high < low
    left = min(low, high)
    right = max(low, high)
    if center is None or not math.isfinite(center):
        center = (left + right) * 0.5
    center = min(max(center, left), right)

    left_span = center - left
    right_span = right - center
    new_left = center - left_span * scale
    new_right = center + right_span * scale
    if new_left == new_right:
        return low, high
    if reversed_axis:
        return new_right, new_left
    return new_left, new_right


class FlightLogWaveformUI(tk.Tk):
    # 中文注释：纯离线查看器，不连串口、不发命令、不修改飞控参数。
    def __init__(self, initial_folder: Path | None = None, initial_csv: Path | None = None) -> None:
        super().__init__()
        self.title("H743 飞行日志波形查看器")
        self.geometry("1480x900")
        self.minsize(1180, 720)

        self.folder_path = initial_folder or DEFAULT_LOG_DIR
        self.csv_path: Path | None = None
        self.frame: pd.DataFrame | None = None if pd is not None else None
        self.numeric_channel_names: list[str] = []

        self.folder_var = tk.StringVar(value=str(self.folder_path))
        self.file_info_var = tk.StringVar(value="请选择一个 CSV 日志文件")
        self.status_var = tk.StringVar(value="就绪：选择文件夹，再点左侧 CSV 文件")
        self.search_var = tk.StringVar(value="")
        self.group_var = tk.StringVar(value="全部")
        self.time_var = tk.StringVar(value="自动")
        self.max_points_var = tk.StringVar(value=str(DEFAULT_MAX_POINTS))
        self.gap_ms_var = tk.StringVar(value=str(int(DEFAULT_GAP_MS)))
        self.wheel_zoom_var = tk.StringVar(value=WHEEL_ZOOM_MODES[0])
        self.subplot_var = tk.BooleanVar(value=False)
        self.normalize_var = tk.BooleanVar(value=False)
        self.show_gaps_var = tk.BooleanVar(value=True)

        self.figure = None
        self.canvas = None
        self.toolbar = None

        self._build_ui()
        self.refresh_files()
        if initial_csv is not None:
            self.load_csv(initial_csv)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(root)
        top.pack(fill=tk.X)
        ttk.Button(top, text="选择文件夹", command=self.choose_folder).pack(side=tk.LEFT)
        ttk.Button(top, text="刷新", command=self.refresh_files).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Entry(top, textvariable=self.folder_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        ttk.Button(top, text="打开CSV", command=self.choose_csv).pack(side=tk.LEFT)

        body = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        files_frame = ttk.Frame(body)
        body.add(files_frame, weight=1)
        ttk.Label(files_frame, text="日志文件").pack(anchor=tk.W)
        self.file_tree = ttk.Treeview(
            files_frame,
            columns=("name", "size", "mtime"),
            show="headings",
            selectmode="browse",
            height=18,
        )
        self.file_tree.heading("name", text="文件名")
        self.file_tree.heading("size", text="大小")
        self.file_tree.heading("mtime", text="修改时间")
        self.file_tree.column("name", width=260, anchor=tk.W)
        self.file_tree.column("size", width=76, anchor=tk.E)
        self.file_tree.column("mtime", width=140, anchor=tk.W)
        file_scroll = ttk.Scrollbar(files_frame, orient=tk.VERTICAL, command=self.file_tree.yview)
        self.file_tree.configure(yscrollcommand=file_scroll.set)
        self.file_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, pady=(4, 0))
        file_scroll.pack(side=tk.LEFT, fill=tk.Y, pady=(4, 0))
        self.file_tree.bind("<<TreeviewSelect>>", self._on_file_selected)

        channel_frame = ttk.Frame(body)
        body.add(channel_frame, weight=1)
        ttk.Label(channel_frame, text="通道选择").pack(anchor=tk.W)
        search = ttk.Frame(channel_frame)
        search.pack(fill=tk.X, pady=(4, 4))
        ttk.Entry(search, textvariable=self.search_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(search, text="清空", command=self.clear_search).pack(side=tk.LEFT, padx=(6, 0))
        self.search_var.trace_add("write", lambda *_: self.refresh_channel_list())

        group_row = ttk.Frame(channel_frame)
        group_row.pack(fill=tk.X, pady=(0, 4))
        groups = ["全部", *CHANNEL_GROUPS.keys()]
        self.group_box = ttk.Combobox(
            group_row,
            textvariable=self.group_var,
            values=groups,
            state="readonly",
            width=10,
        )
        self.group_box.pack(side=tk.LEFT)
        self.group_box.bind("<<ComboboxSelected>>", lambda _event: self.refresh_channel_list())
        ttk.Button(group_row, text="画选中", command=self.plot_selected).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(group_row, text="清图", command=self.clear_plot).pack(side=tk.LEFT, padx=(6, 0))

        self.channel_list = tk.Listbox(
            channel_frame,
            selectmode=tk.EXTENDED,
            exportselection=False,
            height=22,
        )
        channel_scroll = ttk.Scrollbar(channel_frame, orient=tk.VERTICAL, command=self.channel_list.yview)
        self.channel_list.configure(yscrollcommand=channel_scroll.set)
        self.channel_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        channel_scroll.pack(side=tk.LEFT, fill=tk.Y)
        self.channel_list.bind("<Double-Button-1>", lambda _event: self.plot_selected())

        plot_frame = ttk.Frame(body)
        body.add(plot_frame, weight=4)
        self._build_plot_area(plot_frame)

        bottom = ttk.Frame(root)
        bottom.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(bottom, textvariable=self.file_info_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(bottom, textvariable=self.status_var).pack(side=tk.RIGHT)

    def _build_plot_area(self, parent: ttk.Frame) -> None:
        controls = ttk.Frame(parent)
        controls.pack(fill=tk.X)
        ttk.Label(controls, text="横轴").pack(side=tk.LEFT)
        self.time_box = ttk.Combobox(
            controls,
            textvariable=self.time_var,
            values=("自动", "记录序号"),
            state="readonly",
            width=16,
        )
        self.time_box.pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(controls, text="最多点数").pack(side=tk.LEFT)
        ttk.Entry(controls, textvariable=self.max_points_var, width=8).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Checkbutton(controls, text="分图", variable=self.subplot_var).pack(side=tk.LEFT)
        ttk.Checkbutton(controls, text="归一化", variable=self.normalize_var).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Checkbutton(controls, text="按时间断点断开", variable=self.show_gaps_var).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(controls, text="断点ms").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Entry(controls, textvariable=self.gap_ms_var, width=7).pack(side=tk.LEFT)
        ttk.Label(controls, text="滚轮").pack(side=tk.LEFT, padx=(14, 4))
        for mode in WHEEL_ZOOM_MODES:
            ttk.Radiobutton(
                controls,
                text=mode,
                value=mode,
                variable=self.wheel_zoom_var,
            ).pack(side=tk.LEFT, padx=(2, 0))

        if Figure is None or FigureCanvasTkAgg is None:
            ttk.Label(
                parent,
                text="未安装 matplotlib，无法显示波形。请执行：python -m pip install matplotlib pandas numpy",
            ).pack(anchor=tk.W, pady=16)
            return

        self.figure = Figure(figsize=(9, 6), dpi=100, constrained_layout=True)
        self.canvas = FigureCanvasTkAgg(self.figure, master=parent)
        self.canvas.mpl_connect("scroll_event", self._on_scroll_zoom)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(8, 6))
        self.toolbar = NavigationToolbar2Tk(self.canvas, parent, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.pack(fill=tk.X)

        ttk.Label(parent, text="选中通道统计").pack(anchor=tk.W, pady=(8, 0))
        self.stats_tree = ttk.Treeview(
            parent,
            columns=("channel", "count", "min", "max", "mean", "std", "latest"),
            show="headings",
            height=7,
        )
        headings = {
            "channel": ("通道", 260, tk.W),
            "count": ("有效点", 80, tk.E),
            "min": ("最小", 95, tk.E),
            "max": ("最大", 95, tk.E),
            "mean": ("平均", 95, tk.E),
            "std": ("标准差", 95, tk.E),
            "latest": ("末值", 95, tk.E),
        }
        for column, (text, width, anchor) in headings.items():
            self.stats_tree.heading(column, text=text)
            self.stats_tree.column(column, width=width, anchor=anchor)
        self.stats_tree.pack(fill=tk.X, pady=(4, 0))

    def _on_scroll_zoom(self, event: object) -> None:
        if self.figure is None or self.canvas is None:
            return
        axis = getattr(event, "inaxes", None)
        if axis is None:
            return

        step = float(getattr(event, "step", 0.0) or 0.0)
        if step == 0.0:
            return
        scale = (1.0 / WHEEL_ZOOM_BASE) if step > 0.0 else WHEEL_ZOOM_BASE
        mode = self.wheel_zoom_var.get()
        x_center = getattr(event, "xdata", None)
        y_center = getattr(event, "ydata", None)

        if mode in ("总缩放", "横轴"):
            # 中文注释：多子图时横轴统一缩放，便于对齐时间点。
            for target_axis in self.figure.axes:
                x0, x1 = target_axis.get_xlim()
                target_axis.set_xlim(*zoom_limits(x0, x1, x_center, scale))
        if mode in ("总缩放", "纵轴"):
            y0, y1 = axis.get_ylim()
            axis.set_ylim(*zoom_limits(y0, y1, y_center, scale))

        self.canvas.draw_idle()
        self.status_var.set(f"滚轮{mode}：{'放大' if step > 0.0 else '缩小'}")

    def choose_folder(self) -> None:
        folder = filedialog.askdirectory(
            title="选择飞行日志文件夹",
            initialdir=str(self.folder_path if self.folder_path.exists() else ROOT_DIR),
        )
        if folder:
            self.folder_path = Path(folder)
            self.folder_var.set(str(self.folder_path))
            self.refresh_files()

    def choose_csv(self) -> None:
        csv_path = filedialog.askopenfilename(
            title="选择 flightlog CSV",
            initialdir=str(self.folder_path if self.folder_path.exists() else ROOT_DIR),
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if csv_path:
            path = Path(csv_path)
            self.folder_path = path.parent
            self.folder_var.set(str(self.folder_path))
            self.refresh_files(select_path=path)
            self.load_csv(path)

    def refresh_files(self, select_path: Path | None = None) -> None:
        self.folder_path = Path(self.folder_var.get()).expanduser()
        self.file_tree.delete(*self.file_tree.get_children())
        files = discover_csv_files(self.folder_path)
        for path in files:
            stat = path.stat()
            mtime = __import__("datetime").datetime.fromtimestamp(stat.st_mtime)
            item = self.file_tree.insert(
                "",
                tk.END,
                iid=str(path),
                values=(path.name, human_size(stat.st_size), mtime.strftime("%Y-%m-%d %H:%M:%S")),
            )
            if select_path is not None and path.resolve() == select_path.resolve():
                self.file_tree.selection_set(item)
                self.file_tree.see(item)
        self.status_var.set(f"发现 {len(files)} 个 CSV 文件")

    def _on_file_selected(self, _event: object) -> None:
        selection = self.file_tree.selection()
        if not selection:
            return
        path = Path(selection[0])
        if path != self.csv_path:
            self.load_csv(path)

    def load_csv(self, csv_path: Path) -> None:
        if pd is None:
            messagebox.showerror("缺少依赖", "需要 pandas/numpy：python -m pip install pandas numpy")
            return
        try:
            # 中文注释：low_memory=False 避免同一列前后类型不同导致数值列被误判为字符串。
            frame = pd.read_csv(csv_path, low_memory=False)
        except Exception as exc:
            messagebox.showerror("读取失败", f"无法读取 CSV：\n{csv_path}\n\n{exc}")
            return
        self.csv_path = csv_path
        self.frame = frame
        self.numeric_channel_names = numeric_columns(frame)
        self.file_info_var.set(
            f"{csv_path.name} | {len(frame)} 行 | {len(frame.columns)} 列 | "
            f"{len(self.numeric_channel_names)} 个数值通道"
        )
        time_candidates = ["自动", "记录序号", *self.numeric_channel_names]
        self.time_box.configure(values=time_candidates)
        self.time_var.set("自动")
        self.refresh_channel_list()
        self.clear_plot()
        self.status_var.set(f"已加载：{csv_path}")

    def clear_search(self) -> None:
        self.search_var.set("")
        self.group_var.set("全部")
        self.refresh_channel_list()

    def refresh_channel_list(self) -> None:
        query = self.search_var.get().strip().lower()
        group = self.group_var.get()
        self.channel_list.delete(0, tk.END)
        for column in self.numeric_channel_names:
            lowered = column.lower()
            if query and query not in lowered:
                continue
            if group != "全部" and not channel_matches_group(column, group):
                continue
            self.channel_list.insert(tk.END, column)

    def selected_channels(self) -> list[str]:
        return [self.channel_list.get(index) for index in self.channel_list.curselection()]

    def _read_int(self, var: tk.StringVar, default: int, minimum: int) -> int:
        try:
            value = int(float(var.get()))
        except ValueError:
            return default
        return max(minimum, value)

    def _read_float(self, var: tk.StringVar, default: float, minimum: float) -> float:
        try:
            value = float(var.get())
        except ValueError:
            return default
        return max(minimum, value)

    def _time_axis(self, frame: "pd.DataFrame") -> tuple[str, "pd.Series"]:
        requested = self.time_var.get()
        if requested == "自动":
            requested = choose_time_column(frame.columns)
        if requested == "记录序号" or requested not in frame.columns:
            return "记录序号", pd.Series(np.arange(len(frame), dtype=float), index=frame.index)

        values = pd.to_numeric(frame[requested], errors="coerce")
        first_valid = values.dropna().iloc[0] if not values.dropna().empty else 0.0
        if requested.endswith("_us"):
            axis = (values - first_valid) * 1.0e-6
            return f"{requested} 相对时间 / s", axis
        if requested.endswith("_ms") or requested == "tick_ms":
            axis = (values - first_valid) * 1.0e-3
            return f"{requested} 相对时间 / s", axis
        return requested, values

    def _plot_values(
        self,
        time_values: "pd.Series",
        channel_values: "pd.Series",
        max_points: int,
        break_gaps: bool,
        gap_ms: float,
    ) -> tuple["pd.Series", "pd.Series"]:
        series = pd.to_numeric(channel_values, errors="coerce")
        if self.normalize_var.get():
            mean = series.mean(skipna=True)
            std = series.std(skipna=True)
            if std and math.isfinite(float(std)):
                series = (series - mean) / std

        x = time_values.copy()
        if break_gaps and len(x) >= 2:
            gap_s = gap_ms * 0.001
            gaps = x.diff().abs() > gap_s
            series = series.mask(gaps)

        indexer = downsample_indices(len(series), max_points)
        return x.iloc[indexer], series.iloc[indexer]

    def clear_plot(self) -> None:
        if self.figure is not None:
            self.figure.clear()
            axis = self.figure.add_subplot(1, 1, 1)
            axis.set_title("选择通道后点击“画选中”")
            axis.set_xlabel("时间 / s 或记录序号")
            axis.grid(True, alpha=0.25)
            self.canvas.draw_idle()
        if hasattr(self, "stats_tree"):
            self.stats_tree.delete(*self.stats_tree.get_children())

    def plot_selected(self) -> None:
        if self.frame is None or self.figure is None:
            return
        channels = self.selected_channels()
        if not channels:
            self.status_var.set("请先在中间列表选择一个或多个通道")
            return

        max_points = self._read_int(self.max_points_var, DEFAULT_MAX_POINTS, 100)
        gap_ms = self._read_float(self.gap_ms_var, DEFAULT_GAP_MS, 1.0)
        time_label, time_values = self._time_axis(self.frame)

        self.figure.clear()
        axes = []
        if self.subplot_var.get() and len(channels) > 1:
            for index, _channel in enumerate(channels, start=1):
                axes.append(self.figure.add_subplot(len(channels), 1, index))
        else:
            axes.append(self.figure.add_subplot(1, 1, 1))

        self.stats_tree.delete(*self.stats_tree.get_children())
        for index, channel in enumerate(channels):
            axis = axes[index] if len(axes) > 1 else axes[0]
            x, y = self._plot_values(
                time_values,
                self.frame[channel],
                max_points,
                self.show_gaps_var.get(),
                gap_ms,
            )
            axis.plot(x, y, linewidth=1.15, label=channel)
            axis.grid(True, alpha=0.25)
            axis.set_ylabel(channel if len(axes) > 1 else "数值")
            stats = summarize_series(self.frame[channel])
            if stats is not None:
                self.stats_tree.insert(
                    "",
                    tk.END,
                    values=(
                        channel,
                        stats.count,
                        format_float(stats.minimum),
                        format_float(stats.maximum),
                        format_float(stats.mean),
                        format_float(stats.std),
                        format_float(stats.latest),
                    ),
                )

        for axis in axes:
            axis.set_xlabel(time_label)
            if len(axes) == 1:
                axis.legend(loc="best")
        title = self.csv_path.name if self.csv_path is not None else "飞行日志"
        if self.normalize_var.get():
            title += "（归一化）"
        axes[0].set_title(title)
        self.canvas.draw_idle()
        self.status_var.set(f"已绘制 {len(channels)} 个通道，最多 {max_points} 点/通道")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="H743 飞行日志 CSV 波形查看器")
    parser.add_argument(
        "path",
        nargs="?",
        type=Path,
        help="日志 CSV 文件或日志文件夹；省略时默认打开 ./log",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    initial_folder: Path | None = None
    initial_csv: Path | None = None
    if args.path is not None:
        if args.path.is_file():
            initial_csv = args.path
            initial_folder = args.path.parent
        else:
            initial_folder = args.path
    app = FlightLogWaveformUI(initial_folder=initial_folder, initial_csv=initial_csv)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
