#!/usr/bin/env python3
"""H743 飞行日志 All-in-One 查看与 Rerun 回放工作台。"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Iterable

try:
    from . import flight_log_rerun_replay as replay
    from .flight_log_waveform_ui import (
        CHANNEL_GROUPS,
        DEFAULT_GAP_MS,
        DEFAULT_LOG_DIR,
        DEFAULT_MAX_POINTS,
        WHEEL_ZOOM_BASE,
        WHEEL_ZOOM_MODES,
        Figure,
        FigureCanvasTkAgg,
        NavigationToolbar2Tk,
        channel_matches_group,
        choose_time_column,
        discover_csv_files,
        downsample_indices,
        format_float,
        human_size,
        numeric_columns,
        summarize_series,
        zoom_limits,
    )
except ImportError:  # Allows running as: python tools/flight_log_workbench.py
    import flight_log_rerun_replay as replay
    from flight_log_waveform_ui import (
        CHANNEL_GROUPS,
        DEFAULT_GAP_MS,
        DEFAULT_LOG_DIR,
        DEFAULT_MAX_POINTS,
        WHEEL_ZOOM_BASE,
        WHEEL_ZOOM_MODES,
        Figure,
        FigureCanvasTkAgg,
        NavigationToolbar2Tk,
        channel_matches_group,
        choose_time_column,
        discover_csv_files,
        downsample_indices,
        format_float,
        human_size,
        numeric_columns,
        summarize_series,
        zoom_limits,
    )


ROOT_DIR = Path(__file__).resolve().parents[1]
RERUN_WRAPPER = ROOT_DIR / "tools" / "run_flight_log_rerun_replay.ps1"
DEFAULT_RERUN_ROWS = 2600

PRESET_CHANNELS: dict[str, tuple[str, ...]] = {
    "姿态": (
        "roll_deg",
        "pitch_deg",
        "ctrl_target_attitude_rp_rad_0",
        "ctrl_target_attitude_rp_rad_1",
        "gyro_x_dps",
        "gyro_y_dps",
    ),
    "舵机": (
        "servo_alpha_us",
        "servo_alpha_feedback_us",
        "servo_beta_us",
        "servo_beta_feedback_us",
        "servo_move_sent_count",
        "servo_feedback_response_count",
    ),
    "电机": (
        "throttle_us",
        "motor_upper_us",
        "motor_lower_us",
        "ctrl_thrust_utilization",
    ),
    "光流": (
        "flow_height_m",
        "flow_quality",
        "flow_corrected_velocity_m_s_0",
        "flow_corrected_velocity_m_s_1",
    ),
    "控制量": (
        "ctrl_moment_cmd_n_m_0",
        "ctrl_moment_cmd_n_m_1",
        "ctrl_moment_utilization",
        "ctrl_protection_flags",
    ),
}


def segment_label(segment: replay.ReplaySegment) -> str:
    return (
        f"seg{segment.index:02d} | {segment.duration_s:.2f}s | "
        f"seq {segment.start_sequence}..{segment.end_sequence} | "
        f"{segment.reason_start}->{segment.reason_end}"
    )


def available_preset_channels(columns: Iterable[str], preset: str) -> list[str]:
    column_set = set(columns)
    return [channel for channel in PRESET_CHANNELS.get(preset, ()) if channel in column_set]


def make_rerun_wrapper_command(
    csv_path: Path,
    segment_index: int,
    action: str,
    max_rows: int,
) -> list[str]:
    if action not in ("--play", "--export"):
        raise ValueError(f"unsupported rerun action: {action}")
    return [
        "powershell",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(RERUN_WRAPPER),
        str(csv_path),
        action,
        "--segment",
        str(segment_index),
        "--max-rows",
        str(max_rows),
    ]


def format_optional(value: float | int | None, digits: int = 2) -> str:
    if value is None:
        return ""
    if isinstance(value, int):
        return str(value)
    return f"{value:.{digits}f}"


def time_axis_for_segment(frame: "replay.pd.DataFrame", time_choice: str) -> tuple[str, "replay.pd.Series"]:
    if replay.pd is None or replay.np is None:
        raise RuntimeError("缺少 pandas/numpy")
    if time_choice == "自动":
        time_choice = choose_time_column(frame.columns)
    if time_choice == "记录序号" or time_choice not in frame.columns:
        return "片段记录序号", replay.pd.Series(replay.np.arange(len(frame), dtype=float), index=frame.index)

    values = replay.pd.to_numeric(frame[time_choice], errors="coerce")
    valid = values.dropna()
    first = valid.iloc[0] if not valid.empty else 0.0
    if time_choice.endswith("_us"):
        return f"{time_choice} 片段相对时间 / s", (values - first) * 1.0e-6
    if time_choice.endswith("_ms") or time_choice == "tick_ms":
        return f"{time_choice} 片段相对时间 / s", (values - first) * 1.0e-3
    return time_choice, values


class FlightLogWorkbench(tk.Tk):
    # 中文注释：集成工作台只读本地日志；Rerun 回放通过隔离脚本弹出新窗口。
    def __init__(self, initial_path: Path | None = None) -> None:
        super().__init__()
        self.title("H743 飞行日志工作台")
        self.geometry("1540x940")
        self.minsize(1240, 760)

        self.folder_path = DEFAULT_LOG_DIR
        self.csv_path: Path | None = None
        self.frame: replay.pd.DataFrame | None = None if replay.pd is not None else None
        self.segments: list[replay.ReplaySegment] = []
        self.numeric_channel_names: list[str] = []

        self.folder_var = tk.StringVar(value=str(self.folder_path))
        self.path_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="打开 CSV 后选择片段")
        self.gap_ms_var = tk.StringVar(value=str(int(DEFAULT_GAP_MS)))
        self.max_points_var = tk.StringVar(value=str(DEFAULT_MAX_POINTS))
        self.rerun_rows_var = tk.StringVar(value=str(DEFAULT_RERUN_ROWS))
        self.group_var = tk.StringVar(value="全部")
        self.search_var = tk.StringVar(value="")
        self.time_var = tk.StringVar(value="自动")
        self.wheel_zoom_var = tk.StringVar(value=WHEEL_ZOOM_MODES[0])
        self.subplot_var = tk.BooleanVar(value=False)
        self.normalize_var = tk.BooleanVar(value=False)

        self.figure = None
        self.canvas = None
        self.toolbar = None

        self._build_ui()
        self.refresh_files()
        if initial_path is not None:
            self.open_initial(initial_path)
        else:
            latest = replay.latest_csv_in(DEFAULT_LOG_DIR)
            if latest is not None:
                self.load_csv(latest)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(root)
        top.pack(fill=tk.X)
        ttk.Button(top, text="选择文件夹", command=self.choose_folder).pack(side=tk.LEFT)
        ttk.Button(top, text="打开CSV", command=self.choose_csv).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(top, text="打开最新", command=self.open_latest).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(top, text="刷新分片", command=self.refresh_segments).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Entry(top, textvariable=self.path_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(10, 0))

        body = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        left = ttk.Frame(body)
        body.add(left, weight=2)
        self._build_file_and_segment_panel(left)

        right = ttk.Frame(body)
        body.add(right, weight=5)
        self._build_waveform_panel(right)

        ttk.Label(root, textvariable=self.status_var).pack(fill=tk.X, pady=(8, 0))

    def _build_file_and_segment_panel(self, parent: ttk.Frame) -> None:
        folder_row = ttk.Frame(parent)
        folder_row.pack(fill=tk.X)
        ttk.Entry(folder_row, textvariable=self.folder_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(folder_row, text="刷新文件", command=self.refresh_files).pack(side=tk.LEFT, padx=(6, 0))

        ttk.Label(parent, text="日志文件").pack(anchor=tk.W, pady=(8, 0))
        self.file_tree = ttk.Treeview(
            parent,
            columns=("name", "size", "mtime"),
            show="headings",
            selectmode="browse",
            height=8,
        )
        self.file_tree.heading("name", text="文件名")
        self.file_tree.heading("size", text="大小")
        self.file_tree.heading("mtime", text="修改时间")
        self.file_tree.column("name", width=260, anchor=tk.W)
        self.file_tree.column("size", width=80, anchor=tk.E)
        self.file_tree.column("mtime", width=145, anchor=tk.W)
        self.file_tree.pack(fill=tk.BOTH, expand=False, pady=(4, 0))
        self.file_tree.bind("<<TreeviewSelect>>", self._on_file_selected)

        split_row = ttk.Frame(parent)
        split_row.pack(fill=tk.X, pady=(10, 4))
        ttk.Label(split_row, text="分片断点ms").pack(side=tk.LEFT)
        ttk.Entry(split_row, textvariable=self.gap_ms_var, width=8).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(split_row, text="Rerun帧数").pack(side=tk.LEFT)
        ttk.Entry(split_row, textvariable=self.rerun_rows_var, width=8).pack(side=tk.LEFT, padx=(4, 0))

        ttk.Label(parent, text="文件内片段").pack(anchor=tk.W)
        self.segment_tree = ttk.Treeview(
            parent,
            columns=("idx", "rows", "duration", "seq", "reason", "throttle", "angle"),
            show="headings",
            selectmode="browse",
            height=14,
        )
        headings = {
            "idx": ("片段", 52, tk.E),
            "rows": ("行数", 70, tk.E),
            "duration": ("时长", 70, tk.E),
            "seq": ("Seq", 105, tk.W),
            "reason": ("模式", 170, tk.W),
            "throttle": ("油门", 96, tk.W),
            "angle": ("最大角", 115, tk.W),
        }
        for column, (text, width, anchor) in headings.items():
            self.segment_tree.heading(column, text=text)
            self.segment_tree.column(column, width=width, anchor=anchor)
        self.segment_tree.pack(fill=tk.BOTH, expand=True, pady=(4, 6))
        self.segment_tree.bind("<<TreeviewSelect>>", self._on_segment_selected)

        action_row = ttk.Frame(parent)
        action_row.pack(fill=tk.X)
        ttk.Button(action_row, text="画选中片段", command=self.plot_selected_channels).pack(side=tk.LEFT)
        ttk.Button(action_row, text="弹出Rerun回放", command=self.play_selected_segment).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(action_row, text="导出RRD", command=self.export_selected_segment).pack(side=tk.LEFT, padx=(6, 0))

        action_row2 = ttk.Frame(parent)
        action_row2.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(action_row2, text="打开RRD目录", command=self.open_rerun_dir).pack(side=tk.LEFT)
        ttk.Button(action_row2, text="复制片段摘要", command=self.copy_segment_summary).pack(side=tk.LEFT, padx=(6, 0))

    def _build_waveform_panel(self, parent: ttk.Frame) -> None:
        controls = ttk.Frame(parent)
        controls.pack(fill=tk.X)
        ttk.Label(controls, text="通道组").pack(side=tk.LEFT)
        self.group_box = ttk.Combobox(
            controls,
            textvariable=self.group_var,
            values=["全部", *CHANNEL_GROUPS.keys()],
            state="readonly",
            width=10,
        )
        self.group_box.pack(side=tk.LEFT, padx=(4, 8))
        self.group_box.bind("<<ComboboxSelected>>", lambda _event: self.refresh_channel_list())
        ttk.Entry(controls, textvariable=self.search_var, width=22).pack(side=tk.LEFT)
        ttk.Button(controls, text="搜索清空", command=self.clear_search).pack(side=tk.LEFT, padx=(6, 8))
        ttk.Label(controls, text="横轴").pack(side=tk.LEFT)
        self.time_box = ttk.Combobox(
            controls,
            textvariable=self.time_var,
            values=("自动", "记录序号"),
            state="readonly",
            width=16,
        )
        self.time_box.pack(side=tk.LEFT, padx=(4, 8))
        ttk.Label(controls, text="点数").pack(side=tk.LEFT)
        ttk.Entry(controls, textvariable=self.max_points_var, width=8).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Checkbutton(controls, text="分图", variable=self.subplot_var).pack(side=tk.LEFT)
        ttk.Checkbutton(controls, text="归一化", variable=self.normalize_var).pack(side=tk.LEFT, padx=(6, 0))

        zoom_row = ttk.Frame(parent)
        zoom_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(zoom_row, text="滚轮缩放").pack(side=tk.LEFT)
        for mode in WHEEL_ZOOM_MODES:
            ttk.Radiobutton(zoom_row, text=mode, value=mode, variable=self.wheel_zoom_var).pack(side=tk.LEFT, padx=(4, 0))
        for preset in PRESET_CHANNELS:
            ttk.Button(
                zoom_row,
                text=preset,
                command=lambda name=preset: self.apply_channel_preset(name),
            ).pack(side=tk.LEFT, padx=(10 if preset == "姿态" else 4, 0))

        middle = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        middle.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        channel_frame = ttk.Frame(middle)
        middle.add(channel_frame, weight=1)
        self.channel_list = tk.Listbox(channel_frame, selectmode=tk.EXTENDED, exportselection=False)
        channel_scroll = ttk.Scrollbar(channel_frame, orient=tk.VERTICAL, command=self.channel_list.yview)
        self.channel_list.configure(yscrollcommand=channel_scroll.set)
        self.channel_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        channel_scroll.pack(side=tk.LEFT, fill=tk.Y)
        self.channel_list.bind("<Double-Button-1>", lambda _event: self.plot_selected_channels())
        self.search_var.trace_add("write", lambda *_: self.refresh_channel_list())

        plot_frame = ttk.Frame(middle)
        middle.add(plot_frame, weight=5)
        if Figure is None or FigureCanvasTkAgg is None:
            ttk.Label(
                plot_frame,
                text="未安装 matplotlib，无法显示波形。请执行：python -m pip install matplotlib pandas numpy",
            ).pack(anchor=tk.W, pady=16)
            return

        self.figure = Figure(figsize=(9, 6), dpi=100, constrained_layout=True)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.mpl_connect("scroll_event", self._on_scroll_zoom)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.toolbar = NavigationToolbar2Tk(self.canvas, plot_frame, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.pack(fill=tk.X)

        ttk.Label(parent, text="当前片段通道统计").pack(anchor=tk.W, pady=(8, 0))
        self.stats_tree = ttk.Treeview(
            parent,
            columns=("channel", "count", "min", "max", "mean", "std", "latest"),
            show="headings",
            height=6,
        )
        stats_headings = {
            "channel": ("通道", 260, tk.W),
            "count": ("有效点", 80, tk.E),
            "min": ("最小", 95, tk.E),
            "max": ("最大", 95, tk.E),
            "mean": ("平均", 95, tk.E),
            "std": ("标准差", 95, tk.E),
            "latest": ("末值", 95, tk.E),
        }
        for column, (text, width, anchor) in stats_headings.items():
            self.stats_tree.heading(column, text=text)
            self.stats_tree.column(column, width=width, anchor=anchor)
        self.stats_tree.pack(fill=tk.X, pady=(4, 0))
        self.clear_plot()

    def _read_float(self, var: tk.StringVar, default: float) -> float:
        try:
            return float(var.get())
        except ValueError:
            return default

    def _read_int(self, var: tk.StringVar, default: int, minimum: int = 1) -> int:
        try:
            value = int(float(var.get()))
        except ValueError:
            return default
        return max(minimum, value)

    def open_initial(self, path: Path) -> None:
        if path.is_file():
            self.load_csv(path)
        else:
            self.folder_path = path
            self.folder_var.set(str(path))
            self.refresh_files()
            latest = replay.latest_csv_in(path)
            if latest is not None:
                self.load_csv(latest)

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
            initialdir=str(self.folder_path if self.folder_path.exists() else DEFAULT_LOG_DIR),
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if csv_path:
            self.load_csv(Path(csv_path))

    def open_latest(self) -> None:
        latest = replay.latest_csv_in(Path(self.folder_var.get()).expanduser())
        if latest is None:
            latest = replay.latest_csv_in(DEFAULT_LOG_DIR)
        if latest is None:
            messagebox.showwarning("没有日志", "没有找到 flightlog_*.csv")
            return
        self.load_csv(latest)

    def refresh_files(self) -> None:
        self.folder_path = Path(self.folder_var.get()).expanduser()
        self.file_tree.delete(*self.file_tree.get_children())
        for path in discover_csv_files(self.folder_path):
            stat = path.stat()
            mtime = __import__("datetime").datetime.fromtimestamp(stat.st_mtime)
            self.file_tree.insert(
                "",
                tk.END,
                iid=str(path),
                values=(path.name, human_size(stat.st_size), mtime.strftime("%Y-%m-%d %H:%M:%S")),
            )

    def _on_file_selected(self, _event: object) -> None:
        selection = self.file_tree.selection()
        if selection:
            self.load_csv(Path(selection[0]))

    def load_csv(self, csv_path: Path) -> None:
        try:
            frame = replay.load_csv(csv_path)
        except Exception as exc:
            messagebox.showerror("读取失败", str(exc))
            return
        self.csv_path = csv_path
        self.frame = frame
        self.folder_path = csv_path.parent
        self.folder_var.set(str(csv_path.parent))
        self.path_var.set(str(csv_path))
        self.numeric_channel_names = numeric_columns(frame)
        self.time_box.configure(values=["自动", "记录序号", *self.numeric_channel_names])
        self.refresh_files()
        self.refresh_segments()
        self.refresh_channel_list()
        self.apply_channel_preset("姿态")
        self.status_var.set(f"已加载 {csv_path.name}：{len(frame)} 行，{len(self.numeric_channel_names)} 个数值通道")

    def refresh_segments(self) -> None:
        if self.frame is None:
            return
        gap_ms = self._read_float(self.gap_ms_var, DEFAULT_GAP_MS)
        self.segments = replay.split_segments(self.frame, gap_ms=gap_ms)
        self.segment_tree.delete(*self.segment_tree.get_children())
        for segment in self.segments:
            self.segment_tree.insert(
                "",
                tk.END,
                iid=str(segment.index),
                values=(
                    segment.index,
                    segment.count,
                    f"{segment.duration_s:.2f}s",
                    f"{segment.start_sequence}..{segment.end_sequence}",
                    f"{segment.reason_start}->{segment.reason_end}",
                    f"{format_optional(segment.throttle_min_us, 0)}..{format_optional(segment.throttle_max_us, 0)}",
                    f"R{format_optional(segment.max_abs_roll_deg, 1)} P{format_optional(segment.max_abs_pitch_deg, 1)}",
                ),
            )
        if self.segments:
            last = str(self.segments[-1].index)
            self.segment_tree.selection_set(last)
            self.segment_tree.see(last)
        self.status_var.set(f"已分成 {len(self.segments)} 个片段")

    def selected_segment(self) -> replay.ReplaySegment | None:
        selection = self.segment_tree.selection()
        if not selection:
            return None
        index = int(selection[0])
        return self.segments[index] if 0 <= index < len(self.segments) else None

    def current_segment_frame(self) -> "replay.pd.DataFrame | None":
        if self.frame is None:
            return None
        segment = self.selected_segment()
        if segment is None:
            return None
        return self.frame.iloc[segment.start_row : segment.end_row + 1].copy()

    def _on_segment_selected(self, _event: object) -> None:
        segment = self.selected_segment()
        if segment is not None:
            self.status_var.set(segment_label(segment))
            self.plot_selected_channels()

    def refresh_channel_list(self) -> None:
        query = self.search_var.get().strip().lower()
        group = self.group_var.get()
        selected = set(self.selected_channels())
        self.channel_list.delete(0, tk.END)
        for column in self.numeric_channel_names:
            lowered = column.lower()
            if query and query not in lowered:
                continue
            if group != "全部" and not channel_matches_group(column, group):
                continue
            self.channel_list.insert(tk.END, column)
            if column in selected:
                self.channel_list.selection_set(tk.END)

    def clear_search(self) -> None:
        self.search_var.set("")
        self.group_var.set("全部")
        self.refresh_channel_list()

    def selected_channels(self) -> list[str]:
        return [self.channel_list.get(index) for index in self.channel_list.curselection()]

    def apply_channel_preset(self, name: str) -> None:
        channels = available_preset_channels(self.numeric_channel_names, name)
        if not channels:
            return
        self.group_var.set("全部")
        self.search_var.set("")
        self.refresh_channel_list()
        self.channel_list.selection_clear(0, tk.END)
        visible = [self.channel_list.get(index) for index in range(self.channel_list.size())]
        for channel in channels:
            if channel in visible:
                self.channel_list.selection_set(visible.index(channel))
        self.plot_selected_channels()

    def clear_plot(self) -> None:
        if self.figure is not None and self.canvas is not None:
            self.figure.clear()
            axis = self.figure.add_subplot(1, 1, 1)
            axis.set_title("选择文件、片段和通道后查看波形")
            axis.set_xlabel("片段时间 / s")
            axis.grid(True, alpha=0.25)
            self.canvas.draw_idle()
        if hasattr(self, "stats_tree"):
            self.stats_tree.delete(*self.stats_tree.get_children())

    def plot_selected_channels(self) -> None:
        segment_frame = self.current_segment_frame()
        if segment_frame is None or self.figure is None or self.canvas is None:
            return
        channels = self.selected_channels()
        if not channels:
            return
        max_points = self._read_int(self.max_points_var, DEFAULT_MAX_POINTS, 100)
        time_label, time_values = time_axis_for_segment(segment_frame, self.time_var.get())

        self.figure.clear()
        axes = []
        if self.subplot_var.get() and len(channels) > 1:
            for index in range(len(channels)):
                axes.append(self.figure.add_subplot(len(channels), 1, index + 1))
        else:
            axes.append(self.figure.add_subplot(1, 1, 1))

        self.stats_tree.delete(*self.stats_tree.get_children())
        for index, channel in enumerate(channels):
            if channel not in segment_frame.columns:
                continue
            axis = axes[index] if len(axes) > 1 else axes[0]
            series = replay.pd.to_numeric(segment_frame[channel], errors="coerce")
            if self.normalize_var.get():
                mean = series.mean(skipna=True)
                std = series.std(skipna=True)
                if std and math.isfinite(float(std)):
                    series = (series - mean) / std
            indexer = downsample_indices(len(series), max_points)
            axis.plot(time_values.iloc[indexer], series.iloc[indexer], linewidth=1.12, label=channel)
            axis.grid(True, alpha=0.25)
            axis.set_ylabel(channel if len(axes) > 1 else "数值")
            stats = summarize_series(segment_frame[channel])
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
        segment = self.selected_segment()
        title = self.csv_path.name if self.csv_path is not None else "flightlog"
        if segment is not None:
            title += f" | {segment_label(segment)}"
        axes[0].set_title(title)
        self.canvas.draw_idle()

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
            for target_axis in self.figure.axes:
                x0, x1 = target_axis.get_xlim()
                target_axis.set_xlim(*zoom_limits(x0, x1, x_center, scale))
        if mode in ("总缩放", "纵轴"):
            y0, y1 = axis.get_ylim()
            axis.set_ylim(*zoom_limits(y0, y1, y_center, scale))
        self.canvas.draw_idle()

    def _run_rerun_action(self, action: str) -> None:
        if self.csv_path is None:
            return
        segment = self.selected_segment()
        if segment is None:
            return
        max_rows = self._read_int(self.rerun_rows_var, DEFAULT_RERUN_ROWS, 100)
        command = make_rerun_wrapper_command(self.csv_path, segment.index, action, max_rows)
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        try:
            subprocess.Popen(command, cwd=str(ROOT_DIR), creationflags=creationflags)
        except Exception as exc:
            messagebox.showerror("Rerun 启动失败", str(exc))
            return
        verb = "播放" if action == "--play" else "导出"
        self.status_var.set(f"正在{verb} {segment_label(segment)}；首次运行会安装隔离 Rerun 环境")

    def play_selected_segment(self) -> None:
        self._run_rerun_action("--play")

    def export_selected_segment(self) -> None:
        self._run_rerun_action("--export")

    def open_rerun_dir(self) -> None:
        replay.DEFAULT_OUT_DIR.mkdir(parents=True, exist_ok=True)
        os.startfile(replay.DEFAULT_OUT_DIR)  # type: ignore[attr-defined]

    def copy_segment_summary(self) -> None:
        segment = self.selected_segment()
        if segment is None:
            return
        text = segment_label(segment)
        self.clipboard_clear()
        self.clipboard_append(text)
        self.status_var.set(f"已复制：{text}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="H743 飞行日志 All-in-One 工作台")
    parser.add_argument("path", nargs="?", type=Path, help="CSV 文件或日志文件夹；省略时默认打开 ./log 最新日志")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = FlightLogWorkbench(initial_path=args.path)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
