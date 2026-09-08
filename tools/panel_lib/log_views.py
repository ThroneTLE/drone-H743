"""Embedded adapters for the existing log tools; no analysis reimplementation."""
from pathlib import Path
from datetime import datetime
import tempfile
import tkinter as tk
from tkinter import filedialog

try:
    from .. import flight_log_workbench as waveform
    from .. import flight_log_sysid_ui as analysis
    from .. import flight_log_receive as receive
except ImportError:  # direct tools/drone_tcp_panel.py
    import flight_log_workbench as waveform
    import flight_log_sysid_ui as analysis
    import flight_log_receive as receive

from .log_jobs import LogJobs
from .log_layout import wrap_toolbars
from .theme import UI_PALETTE, apply_matplotlib_theme


class WaveformView(waveform.FlightLogWorkbenchView):
    def __init__(self, parent, selected, begin_selection):
        self.selected, self.begin_selection = selected, begin_selection
        self.jobs = None
        super().__init__(parent)
        wrap_toolbars(self)
        self.jobs = LogJobs(self, lambda error: self.status_var.set(f"读取失败：{error}"))
        self.refresh_files()

    def refresh_files(self):
        if self.jobs is None:
            return
        folder = Path(self.folder_var.get()).expanduser()

        def read():
            rows = []
            for path in waveform.discover_csv_files(folder):
                stat = path.stat()
                rows.append((path, waveform.human_size(stat.st_size),
                             datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M:%S")))
            return rows

        def show(rows):
            self.file_tree.delete(*self.file_tree.get_children())
            for path, size, stamp in rows:
                self.file_tree.insert("", tk.END, iid=str(path), values=(path.name, size, stamp))

        self.jobs.submit("files", read, show)

    def load_csv(self, path):
        path = Path(path)
        token = self.begin_selection()
        self.frame = None
        self.segments = []
        self.csv_path = None
        self.clear_plot()
        self.segment_tree.delete(*self.segment_tree.get_children())
        self.stats_tree.delete(*self.stats_tree.get_children())
        self.status_var.set(f"正在读取：{path}")

        def show(frame):
            self.display_csv(path, frame)
            self.selected(path, token)

        self.jobs.submit("load", lambda: waveform.replay.load_csv(path), show)

    def plot_selected_channels(self):
        super().plot_selected_channels()
        if self.figure is not None:
            apply_matplotlib_theme(self.figure)
            self.canvas.draw_idle()


class AnalysisView(analysis.FlightLogSysidUIView):
    def __init__(self, parent, selected, begin_selection):
        self.selected, self.begin_selection = selected, begin_selection
        super().__init__(parent)
        self.summary_text.configure(background=UI_PALETTE["surface"], foreground=UI_PALETTE["ink"])
        self.jobs = LogJobs(self, lambda error: self.status_var.set(f"分析失败：{error}"))

    def analyze_current(self):
        if self.csv_path is None:
            self.status_var.set("请先导入 CSV")
            return
        try:
            gap = max(0.001, float(self.gap_ms_var.get()) / 1000)
        except ValueError:
            self.status_var.set("分段间隔必须为数字")
            return
        path = self.csv_path
        token = self.begin_selection()
        self.analysis = None
        self.summary_text.configure(state="normal")
        self.summary_text.delete("1.0", "end")
        self.summary_text.configure(state="disabled")
        self.flag_list.delete(0, "end")
        for child in self.notebook.winfo_children():
            for widget in child.winfo_children():
                if widget.winfo_class() == "Treeview":
                    widget.delete(*widget.get_children())
        self.status_var.set(f"正在分析：{path}")

        def show(result):
            self.analysis = result
            self.refresh_views()
            self.selected(path, token)

        self.jobs.submit("analysis", lambda: analysis.sysid.analyze_flight_log(path, gap_s=gap), show)

    def refresh_views(self):
        super().refresh_views()
        if self.figure is not None:
            apply_matplotlib_theme(self.figure)
            self.canvas.draw_idle()

    def export_reports(self):
        if self.analysis is None or self.csv_path is None:
            self.status_var.set("请先等待当前日志分析成功")
            return
        chosen = filedialog.askdirectory(title="选择报告保存位置", parent=self)
        if not chosen:
            return
        result, stem = self.analysis, self.csv_path.stem

        def write():
            directory = Path(tempfile.mkdtemp(prefix=f"{stem}_", dir=chosen))
            return analysis.sysid.write_reports(result, directory, stem)

        self.jobs.submit("export", write, lambda paths: self.status_var.set(
            "已导出：" + "；".join(str(path) for path in paths.values())))
