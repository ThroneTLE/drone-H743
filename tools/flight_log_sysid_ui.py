#!/usr/bin/env python3
"""H743 飞行日志系统辨识的 Tkinter 图形界面。"""

from __future__ import annotations

import argparse
import tkinter as tk
from dataclasses import asdict
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    from . import flight_log_sysid as sysid
except ImportError:  # Allows running as: python tools/flight_log_sysid_ui.py
    import flight_log_sysid as sysid

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    from matplotlib.figure import Figure

    # 中文注释：Windows 上优先使用微软雅黑/黑体，避免图表标题显示成方块。
    from matplotlib import rcParams

    rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    rcParams["axes.unicode_minus"] = False
except Exception:  # pragma: no cover - depends on host optional packages
    Figure = None
    FigureCanvasTkAgg = None
    NavigationToolbar2Tk = None


def format_cell(value: object, digits: int = 3) -> str:
    # 中文注释：表格统一格式化，空值留空，浮点数保留指定小数位。
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def default_report_dir(csv_path: Path) -> Path:
    # 中文注释：默认把报告放到日志同目录的 .tmp/sysid_xxx，方便每次复查。
    return csv_path.parent / ".tmp" / f"sysid_{csv_path.stem}"


def translate_flag(flag: str) -> str:
    # 中文注释：底层分析模块保留英文机器可读 flag，UI 负责翻成现场可读中文。
    if flag.startswith("log is discontinuous:"):
        return flag.replace("log is discontinuous:", "日志不连续：").replace("time segments", "个时间片段")
    if flag.startswith("flight recorder dropped"):
        return flag.replace("flight recorder dropped", "飞控记录阶段丢失").replace("records before export", "条记录，导出本身未必丢包")
    translations = {
        "tilt saturation is significant; avoid linear PID fits in saturated samples":
            "倾角饱和明显：饱和样本不要直接拿来做线性 PID 拟合",
        "motor high saturation is significant; thrust authority is a limiting factor":
            "电机高端饱和明显：推力余量已经限制系统响应",
        "velocity loop is disabled in this log":
            "这份日志里速度环处于关闭状态",
        "angle-P tilt contribution is zero; angle KP is acting through force terms or disabled":
            "角度 P 的倾角直通项为 0：角度 KP 可能已经迁移到力层，或被关闭",
        "no direct range/height channel found; Z position-loop identification is limited":
            "没有直接测距/高度通道：Z 位置环辨识能力受限",
        "export has missing byte ranges":
            "导出文件存在缺失字节区间",
    }
    return translations.get(flag, flag)


def translate_severity(severity: str) -> str:
    # 中文注释：底层 severity 保持英文，界面展示成现场更直观的中文。
    translations = {
        "bad": "严重",
        "warn": "警告",
        "info": "提示",
    }
    return translations.get(severity, severity)


class FlightLogSysidUI(tk.Tk):
    # 中文注释：这个类只做离线日志分析 UI，不连接串口、不改参数、不触碰飞控板。
    def __init__(self, initial_csv: Path | None = None) -> None:
        super().__init__()
        self.title("H743 飞行日志系统辨识")
        self.geometry("1320x840")
        self.minsize(1120, 700)

        self.csv_path: Path | None = None
        self.analysis: sysid.FlightLogAnalysis | None = None

        self.path_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="打开一个 flightlog_*.csv 文件开始分析")
        self.gap_ms_var = tk.StringVar(value=str(int(sysid.DEFAULT_GAP_S * 1000.0)))

        self._build_ui()
        if initial_csv is not None:
            self.load_csv(initial_csv)

    def _build_ui(self) -> None:
        # 中文注释：界面按“左侧结论、右侧明细”的方式组织，便于现场快速筛参数。
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(root)
        top.pack(fill=tk.X)
        ttk.Button(top, text="打开CSV", command=self.open_csv).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(top, text="重新分析", command=self.analyze_current).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="导出报告", command=self.export_reports).pack(side=tk.LEFT, padx=6)
        ttk.Label(top, text="分段间隔 ms").pack(side=tk.LEFT, padx=(18, 4))
        ttk.Entry(top, textvariable=self.gap_ms_var, width=8).pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.path_var).pack(side=tk.LEFT, padx=(18, 0), fill=tk.X, expand=True)

        body = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        left = ttk.Frame(body)
        body.add(left, weight=1)
        ttk.Label(left, text="概要").pack(anchor=tk.W)
        self.summary_text = tk.Text(left, height=18, wrap=tk.WORD)
        self.summary_text.pack(fill=tk.BOTH, expand=True, pady=(4, 8))
        self.summary_text.configure(state=tk.DISABLED)

        ttk.Label(left, text="风险提示").pack(anchor=tk.W)
        self.flag_list = tk.Listbox(left, height=9, exportselection=False)
        self.flag_list.pack(fill=tk.BOTH, expand=True, pady=(4, 0))

        right = ttk.Frame(body)
        body.add(right, weight=4)
        self.notebook = ttk.Notebook(right)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.gain_table = self._add_tree_tab(
            "参数组",
            (
                "params",
                "rows",
                "duration",
                "roll_rms",
                "pitch_rms",
                "gyro_xy",
                "yaw_rate",
                "tilt_sat",
                "motor_hi",
                "direct",
                "throttle",
            ),
            {
                "params": ("参数快照", 430, tk.W),
                "rows": ("记录数", 70, tk.E),
                "duration": ("时长 s", 70, tk.E),
                "roll_rms": ("横滚RMS", 80, tk.E),
                "pitch_rms": ("俯仰RMS", 85, tk.E),
                "gyro_xy": ("XY角速RMS", 90, tk.E),
                "yaw_rate": ("偏航角速RMS", 95, tk.E),
                "tilt_sat": ("倾角饱和%", 85, tk.E),
                "motor_hi": ("电机打满%", 85, tk.E),
                "direct": ("油门直通%", 85, tk.E),
                "throttle": ("油门us", 75, tk.E),
            },
        )
        self.advice_table = self._add_tree_tab(
            "调参建议",
            ("loop", "axis", "severity", "recommendation", "evidence", "channels"),
            {
                "loop": ("环路", 80, tk.W),
                "axis": ("轴", 70, tk.W),
                "severity": ("级别", 70, tk.W),
                "recommendation": ("建议", 430, tk.W),
                "evidence": ("证据", 420, tk.W),
                "channels": ("相关通道", 360, tk.W),
            },
        )
        self.segment_table = self._add_tree_tab(
            "时间片段",
            ("idx", "rows", "duration", "rate", "seq0", "seq1", "missing", "reason0", "reason1"),
            {
                "idx": ("序号", 50, tk.E),
                "rows": ("记录数", 80, tk.E),
                "duration": ("时长 s", 80, tk.E),
                "rate": ("采样Hz", 80, tk.E),
                "seq0": ("起始Seq", 90, tk.E),
                "seq1": ("结束Seq", 90, tk.E),
                "missing": ("丢记录", 80, tk.E),
                "reason0": ("起始输出模式", 140, tk.W),
                "reason1": ("结束输出模式", 140, tk.W),
            },
        )
        self.fit_table = self._add_tree_tab(
            "执行器拟合",
            ("label", "input", "output", "slope", "intercept", "r2", "corr", "count"),
            {
                "label": ("拟合项", 210, tk.W),
                "input": ("输入通道", 170, tk.W),
                "output": ("输出通道", 170, tk.W),
                "slope": ("斜率", 95, tk.E),
                "intercept": ("截距", 95, tk.E),
                "r2": ("R2", 80, tk.E),
                "corr": ("相关系数", 80, tk.E),
                "count": ("样本数", 80, tk.E),
            },
        )
        self.channel_table = self._add_tree_tab(
            "通道统计",
            ("channel", "count", "min", "p05", "mean", "rms", "p95", "max"),
            {
                "channel": ("通道", 220, tk.W),
                "count": ("样本数", 80, tk.E),
                "min": ("最小", 90, tk.E),
                "p05": ("P05", 90, tk.E),
                "mean": ("平均", 90, tk.E),
                "rms": ("RMS", 90, tk.E),
                "p95": ("P95", 90, tk.E),
                "max": ("最大", 90, tk.E),
            },
        )
        self.plot_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.plot_frame, text="图表")
        self._build_plot_tab()

        ttk.Label(root, textvariable=self.status_var).pack(fill=tk.X, pady=(8, 0))

    def _add_tree_tab(
        self,
        title: str,
        columns: tuple[str, ...],
        headings: dict[str, tuple[str, int, str]],
    ) -> ttk.Treeview:
        # 中文注释：所有明细页都用同一种表格构造，避免列宽/滚动条行为不一致。
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text=title)
        tree = ttk.Treeview(frame, columns=columns, show="headings")
        yscroll = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=tree.yview)
        xscroll = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=tree.xview)
        tree.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        for column in columns:
            text, width, anchor = headings[column]
            tree.heading(column, text=text)
            tree.column(column, width=width, anchor=anchor, stretch=True)
        tree.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        tree.tag_configure("warn", background="#fff2cc")
        tree.tag_configure("bad", background="#f8d7da")
        return tree

    def _build_plot_tab(self) -> None:
        if Figure is None or FigureCanvasTkAgg is None:
            ttk.Label(
                self.plot_frame,
                text="未安装 matplotlib；表格和报告仍可正常使用。",
            ).pack(anchor=tk.W, padx=10, pady=10)
            self.figure = None
            self.ax_rms = None
            self.ax_sat = None
            self.canvas = None
            return

        self.figure = Figure(figsize=(8, 6), dpi=100, constrained_layout=True)
        self.ax_rms = self.figure.add_subplot(2, 1, 1)
        self.ax_sat = self.figure.add_subplot(2, 1, 2)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        if NavigationToolbar2Tk is not None:
            toolbar = NavigationToolbar2Tk(self.canvas, self.plot_frame, pack_toolbar=False)
            toolbar.update()
            toolbar.pack(fill=tk.X)

    def open_csv(self) -> None:
        path = filedialog.askopenfilename(
            title="打开飞行日志 CSV",
            initialdir=Path.cwd(),
            filetypes=(("飞行日志 CSV", "flightlog_*.csv"), ("CSV 文件", "*.csv"), ("所有文件", "*.*")),
        )
        if path:
            self.load_csv(Path(path))

    def load_csv(self, path: Path) -> None:
        self.csv_path = path
        self.path_var.set(str(path))
        self.analyze_current()

    def analyze_current(self) -> None:
        if self.csv_path is None:
            messagebox.showinfo("没有CSV", "请先打开一个 flightlog_*.csv 文件。")
            return
        try:
            # 中文注释：分段间隔用于判断日志中断点，例如两段试车之间隔了几秒。
            gap_s = max(0.001, float(self.gap_ms_var.get()) / 1000.0)
            self.analysis = sysid.analyze_flight_log(self.csv_path, gap_s=gap_s)
        except Exception as exc:
            messagebox.showerror("分析失败", str(exc))
            return
        self.refresh_views()

    def export_reports(self) -> None:
        if self.analysis is None or self.csv_path is None:
            messagebox.showinfo("没有分析结果", "请先分析一份飞行日志再导出。")
            return
        initial = default_report_dir(self.csv_path)
        out_dir_text = filedialog.askdirectory(
            title="选择报告导出目录",
            initialdir=initial.parent,
            mustexist=False,
        )
        out_dir = Path(out_dir_text) if out_dir_text else initial
        try:
            paths = sysid.write_reports(self.analysis, out_dir, self.csv_path.stem)
        except Exception as exc:
            messagebox.showerror("导出失败", str(exc))
            return
        self.status_var.set("已导出：" + "，".join(f"{name}={path}" for name, path in paths.items()))

    def refresh_views(self) -> None:
        # 中文注释：分析完成后刷新所有页签，颜色标记饱和严重的参数组。
        analysis = self.analysis
        if analysis is None:
            return
        self._refresh_summary(analysis)
        self._refresh_gain_groups(analysis)
        self._refresh_advice(analysis)
        self._refresh_segments(analysis)
        self._refresh_fits(analysis)
        self._refresh_channels(analysis)
        self._refresh_plot(analysis)
        self.status_var.set(
            f"已分析 {analysis.row_count} 条记录，{len(analysis.segments)} 个时间片段，"
            f"{len(analysis.gain_groups)} 个参数组，{len(analysis.tuning_advice)} 条调参建议"
        )

    def _refresh_summary(self, analysis: sysid.FlightLogAnalysis) -> None:
        lines = [
            f"CSV文件：{analysis.csv_path}",
            f"元数据：{analysis.meta_path or ''}",
            f"记录数：{analysis.row_count}",
            f"通道数：{analysis.column_count}",
            f"总时间跨度：{analysis.duration_s:.3f} s",
            f"标称记录频率：{format_cell(analysis.nominal_log_rate_hz)} Hz",
            f"连续片段实测频率：{format_cell(analysis.observed_contiguous_rate_hz)} Hz",
            f"序号缺失：{analysis.sequence_missing_count}",
            f"dropped_records 增量：{format_cell(analysis.dropped_delta)}",
            f"导出完整：{format_cell(analysis.export_complete)}",
        ]
        self.summary_text.configure(state=tk.NORMAL)
        self.summary_text.delete("1.0", tk.END)
        self.summary_text.insert(tk.END, "\n".join(lines))
        self.summary_text.configure(state=tk.DISABLED)

        self.flag_list.delete(0, tk.END)
        for flag in analysis.flags:
            self.flag_list.insert(tk.END, translate_flag(flag))

    def _clear_tree(self, tree: ttk.Treeview) -> None:
        tree.delete(*tree.get_children())

    def _refresh_gain_groups(self, analysis: sysid.FlightLogAnalysis) -> None:
        self._clear_tree(self.gain_table)
        for group in analysis.gain_groups:
            tag = ""
            if group.tilt_saturation_pct >= 20.0 or group.motor_high_saturation_pct >= 40.0:
                tag = "bad"
            elif group.tilt_saturation_pct >= 5.0 or group.motor_high_saturation_pct >= 10.0:
                tag = "warn"
            self.gain_table.insert(
                "",
                tk.END,
                values=(
                    group.key,
                    group.count,
                    format_cell(group.duration_s),
                    format_cell(group.roll_rms_deg),
                    format_cell(group.pitch_rms_deg),
                    format_cell(group.gyro_xy_rms_dps),
                    format_cell(group.yaw_rate_rms_dps),
                    format_cell(group.tilt_saturation_pct, 1),
                    format_cell(group.motor_high_saturation_pct, 1),
                    format_cell(group.direct_throttle_pct, 1),
                    format_cell(group.throttle_mean_us, 1),
                ),
                tags=(tag,) if tag else (),
            )

    def _refresh_advice(self, analysis: sysid.FlightLogAnalysis) -> None:
        self._clear_tree(self.advice_table)
        for item in analysis.tuning_advice:
            tag = item.severity if item.severity in {"warn", "bad"} else ""
            self.advice_table.insert(
                "",
                tk.END,
                values=(
                    item.loop,
                    item.axis,
                    translate_severity(item.severity),
                    item.recommendation,
                    item.evidence,
                    item.channels,
                ),
                tags=(tag,) if tag else (),
            )

    def _refresh_segments(self, analysis: sysid.FlightLogAnalysis) -> None:
        self._clear_tree(self.segment_table)
        for segment in analysis.segments:
            tag = "warn" if segment.missing_sequence_count > 0 else ""
            self.segment_table.insert(
                "",
                tk.END,
                values=(
                    segment.index,
                    segment.count,
                    format_cell(segment.duration_s),
                    format_cell(segment.sample_rate_hz, 1),
                    format_cell(segment.start_sequence),
                    format_cell(segment.end_sequence),
                    segment.missing_sequence_count,
                    segment.motor_reason_start,
                    segment.motor_reason_end,
                ),
                tags=(tag,) if tag else (),
            )

    def _refresh_fits(self, analysis: sysid.FlightLogAnalysis) -> None:
        self._clear_tree(self.fit_table)
        for fit in analysis.actuator_fits:
            self.fit_table.insert(
                "",
                tk.END,
                values=(
                    fit.label,
                    fit.input_channel,
                    fit.output_channel,
                    format_cell(fit.slope, 6),
                    format_cell(fit.intercept, 3),
                    format_cell(fit.r2, 4),
                    format_cell(fit.correlation, 4),
                    fit.count,
                ),
            )

    def _refresh_channels(self, analysis: sysid.FlightLogAnalysis) -> None:
        self._clear_tree(self.channel_table)
        for channel, stats in analysis.channel_stats.items():
            self.channel_table.insert(
                "",
                tk.END,
                values=(
                    channel,
                    stats.count,
                    format_cell(stats.minimum),
                    format_cell(stats.p05),
                    format_cell(stats.mean),
                    format_cell(stats.rms),
                    format_cell(stats.p95),
                    format_cell(stats.maximum),
                ),
            )

    def _refresh_plot(self, analysis: sysid.FlightLogAnalysis) -> None:
        # 中文注释：图表只做参数组横向比较，具体数值以表格为准。
        if self.figure is None or self.ax_rms is None or self.ax_sat is None or self.canvas is None:
            return
        self.ax_rms.clear()
        self.ax_sat.clear()
        if not analysis.gain_groups:
            self.canvas.draw_idle()
            return

        labels = [short_gain_label(group) for group in analysis.gain_groups]
        x_values = list(range(len(labels)))
        roll = [group.roll_rms_deg or 0.0 for group in analysis.gain_groups]
        pitch = [group.pitch_rms_deg or 0.0 for group in analysis.gain_groups]
        tilt_sat = [group.tilt_saturation_pct for group in analysis.gain_groups]
        motor_sat = [group.motor_high_saturation_pct for group in analysis.gain_groups]

        self.ax_rms.bar([x - 0.2 for x in x_values], roll, width=0.4, label="横滚RMS")
        self.ax_rms.bar([x + 0.2 for x in x_values], pitch, width=0.4, label="俯仰RMS")
        self.ax_rms.set_ylabel("角度 deg")
        self.ax_rms.set_title("不同参数组的姿态均方根")
        self.ax_rms.legend(loc="upper right")

        self.ax_sat.plot(x_values, tilt_sat, marker="o", label="倾角饱和%")
        self.ax_sat.plot(x_values, motor_sat, marker="s", label="电机打满%")
        self.ax_sat.set_ylabel("%")
        self.ax_sat.set_title("不同参数组的饱和比例")
        self.ax_sat.legend(loc="upper right")
        self.ax_sat.set_xticks(x_values)
        self.ax_sat.set_xticklabels(labels, rotation=35, ha="right")
        self.canvas.draw_idle()


def short_gain_label(group: sysid.GainGroupSummary) -> str:
    params = group.params
    return (
        f"P{format_cell(params.get('roll_angle_kp'), 1)} "
        f"D{format_cell(params.get('roll_rate_kd'), 1)} "
        f"Z{format_cell(params.get('pos_z_kp'), 1)}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="打开 H743 飞行日志系统辨识中文 UI。")
    parser.add_argument("csv", type=Path, nargs="?", help="可选的 flightlog_*.csv 路径")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = FlightLogSysidUI(args.csv)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
