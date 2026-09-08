"""Log workspace: receive/import, existing waveform view, existing analysis view.

Boundary: file analysis reuses tools; receiving has exclusive serial ownership.
Test seam: real Tk page + guarded transports, temporary CSV/BIN and job queues.
"""
from pathlib import Path
import json
import tempfile
import tkinter as tk
from tkinter import ttk, filedialog

from ..log_jobs import LogJobs
from ..viewport import VerticalScrolledFrame

try:
    from ...project_paths import FLIGHT_LOG_DIR, dated_directory
except ImportError:
    from project_paths import FLIGHT_LOG_DIR, dated_directory


class LogsPage(ttk.Frame):
    def __init__(self, parent, panel):
        super().__init__(parent)
        self.panel = panel
        self.path = tk.StringVar(master=self, value="尚未导入日志")
        self.status = tk.StringVar(master=self, value="导入本地 CSV / BIN，或从飞控接收日志")
        self.current_csv = None
        self.selection_epoch = 0
        self.waveform = self.analysis = self.receiver = None
        self.tabs = ttk.Notebook(self)
        self.tabs.pack(fill=tk.BOTH, expand=True)
        self.pages = []
        for title in ("接收与导入", "波形与回放", "离线分析"):
            viewport = VerticalScrolledFrame(self.tabs)
            self.tabs.add(viewport, text=title)
            self.pages.append(viewport)
        self.jobs = LogJobs(self, lambda error: self.status.set(f"导入失败：{error}"))
        source = self.pages[0].content
        ttk.Button(source, text="导入 CSV / BIN", command=self.import_file).pack(anchor="w")
        ttk.Label(source, textvariable=self.path, wraplength=700).pack(fill="x", pady=8)
        actions = ttk.Frame(source)
        actions.pack(fill="x")
        ttk.Button(actions, text="查看波形与回放", command=lambda: self.open_current(1)).pack(side="left")
        ttk.Button(actions, text="离线分析", command=lambda: self.open_current(2)).pack(side="left", padx=8)
        ttk.Label(source, textvariable=self.status, wraplength=700).pack(fill="x", pady=8)
        ttk.Separator(source).pack(fill="x", pady=8)
        ttk.Label(source, text="飞控日志接收 · 使用顶部当前连接", wraplength=700).pack(anchor="w")
        self.receiver_host = ttk.Frame(source)
        self.receiver_host.pack(fill="both", expand=True)
        self.show_receiver()
        for index in (1, 2):
            ttk.Button(self.pages[index].content, text="载入日志工具", command=lambda i=index: self.ensure_view(i)).pack(anchor="w")
        self.tabs.bind("<<NotebookTabChanged>>", self._tab_changed)

    def begin_selection(self):
        self.selection_epoch += 1
        return self.selection_epoch

    def set_current(self, path, token=None):
        if token is not None and token != self.selection_epoch:
            return
        self.current_csv = Path(path)
        self.path.set(str(path))

    def import_file(self):
        chosen = filedialog.askopenfilename(parent=self, title="导入飞行日志", filetypes=(
            ("飞行日志 CSV / BIN", "*.csv *.bin"), ("CSV", "*.csv"), ("BIN", "*.bin")))
        if not chosen:
            return
        self.import_path(Path(chosen))

    def import_path(self, path):
        token = self.begin_selection()
        self.current_csv = None
        self.path.set(str(path))
        self.status.set("正在导入…")

        def read():
            from ..log_views import receive, waveform
            if path.suffix.lower() == ".csv":
                frame = waveform.replay.load_csv(path)
                return path, f"已导入 CSV：{len(frame)} 条记录"
            if path.suffix.lower() != ".bin":
                raise ValueError("仅支持 CSV / BIN")
            sectors, records, errors = receive.parse_flash_image(path.read_bytes())
            if not records:
                raise ValueError("BIN 中没有可解析的飞行记录")
            root = dated_directory(FLIGHT_LOG_DIR)
            root.mkdir(parents=True, exist_ok=True)
            directory = Path(tempfile.mkdtemp(prefix="import_", dir=root))
            csv = directory / f"{path.stem}.csv"
            receive.write_csv(csv, records)
            (directory / "import.json").write_text(json.dumps({
                "source": str(path), "errors": errors, "sectors": sectors,
                "record_count": len(records)}, ensure_ascii=False, indent=2), encoding="utf-8")
            return csv, f"BIN 已转换：{len(records)} 条记录；解析问题 {len(errors)} 项（见 import.json）"

        def done(result):
            path, message = result
            self.set_current(path, token)
            if token == self.selection_epoch:
                self.status.set(message)

        self.jobs.submit("import", read, done)

    def show_receiver(self):
        if self.receiver is not None:
            return
        from ..log_receive_view import ReceiverView
        for child in self.receiver_host.winfo_children():
            child.destroy()
        self.receiver = ReceiverView(self.receiver_host, self.panel, self.received)
        self.receiver.dir_var.set(str(dated_directory(FLIGHT_LOG_DIR)))

    def received(self, result):
        if result.complete:
            self.set_current(result.csv_path)
            self.status.set(f"接收完成：{result.records} 条记录；解析问题 {len(result.errors)} 项")
        else:
            self.status.set(f"接收不完整：缺少 {result.missing_bytes} 字节。未自动载入；文件：{result.csv_path}")

    def allow_main_connect(self):
        if self.receiver is not None and self.receiver.busy():
            self.status.set("日志正在接收，请先取消接收并等待结束，再连接顶部通道。")
            return False
        return True

    def ensure_view(self, index):
        name = "waveform" if index == 1 else "analysis"
        view = getattr(self, name)
        if view is None:
            from ..log_views import WaveformView, AnalysisView
            host = self.pages[index].content
            for child in host.winfo_children():
                child.destroy()
            view = (WaveformView if index == 1 else AnalysisView)(host, self.set_current, self.begin_selection)
            view.pack(fill="both", expand=True)
            setattr(self, name, view)
        return view

    def open_current(self, index):
        if self.current_csv is None:
            self.status.set("请先成功导入或接收日志")
            self.tabs.select(0)
            return
        path = self.current_csv
        view = self.ensure_view(index)
        self.tabs.select(index)
        view.load_csv(path)

    def _tab_changed(self, _event):
        index = self.tabs.index(self.tabs.select())
        if index in (1, 2):
            self.ensure_view(index)


def mount_logs(panel):
    panel.logs_page = LogsPage(panel.notebook, panel)
    panel.logs_notebook = panel.logs_page.tabs
    panel.logs_group_tab = panel.logs_page
    panel.notebook.add(panel.logs_page, text="日志")
