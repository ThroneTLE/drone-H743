"""FlightLog reception using the main panel's current serial connection."""
from pathlib import Path
import json
import queue
import tempfile
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog

try:
    from .. import flight_log_receive as receive
    from ..project_paths import FLIGHT_LOG_DIR, dated_directory
except ImportError:
    import flight_log_receive as receive
    from project_paths import FLIGHT_LOG_DIR, dated_directory

from .theme import UI_PALETTE


class ReceiverView(ttk.Frame):
    def __init__(self, parent, panel, completed):
        super().__init__(parent)
        self.pack(fill="both", expand=True)
        self.panel, self.on_result = panel, completed
        self.worker = self.lease = None
        self.events = queue.Queue()
        self.cancel_requested = False
        self.closed = False
        self.latest_progress = (0, 0)
        self.started_at = 0.0
        self.link_var = tk.StringVar(master=self)
        self.dir_var = tk.StringVar(master=self, value=str(dated_directory(FLIGHT_LOG_DIR)))
        self.progress_var = tk.DoubleVar(master=self)
        self.progress_text = tk.StringVar(master=self, value="直接使用顶部当前串口连接，无需关闭或重新选择端口。")
        ttk.Label(self, textvariable=self.link_var, wraplength=700).pack(fill="x", pady=6)
        destination = ttk.Frame(self)
        destination.pack(fill="x", pady=6)
        ttk.Label(destination, text="保存位置").pack(side="left")
        ttk.Button(destination, text="选择目录", command=self._browse).pack(side="right")
        ttk.Entry(destination, textvariable=self.dir_var).pack(side="left", fill="x", expand=True, padx=8)
        actions = ttk.Frame(self)
        actions.pack(fill="x", pady=6)
        self.receive_btn = ttk.Button(actions, text="从当前连接接收日志", command=self._start_receive)
        self.receive_btn.pack(side="left")
        self.cancel_btn = ttk.Button(actions, text="取消接收", command=self._cancel, state="disabled")
        self.cancel_btn.pack(side="left", padx=8)
        ttk.Progressbar(self, variable=self.progress_var, maximum=100).pack(fill="x", pady=6)
        ttk.Label(self, textvariable=self.progress_text, wraplength=700).pack(fill="x", pady=6)
        log_box = ttk.Frame(self)
        log_box.pack(fill="both", expand=True)
        self.log_text = tk.Text(log_box, height=10, width=60, wrap="word", state="disabled",
                                background=UI_PALETTE["surface"], foreground=UI_PALETTE["ink"])
        scrollbar = ttk.Scrollbar(log_box, command=self.log_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(fill="both", expand=True)
        self.bind("<Destroy>", self._destroy, add="+")
        self._poll_events()

    def busy(self):
        return self.worker is not None and self.worker.is_alive()

    def _browse(self):
        directory = filedialog.askdirectory(parent=self, title="选择日志保存位置")
        if directory:
            self.dir_var.set(directory)

    def _append_log(self, message):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", str(message) + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _start_receive(self):
        if self.busy():
            return
        transport = self.panel.transport
        if transport is not self.panel.serial_transport or not transport.is_connected:
            self._append_log("请先使用顶部 serial 通道连接飞控，然后直接接收。")
            return
        reason = self.panel._link_keepalive_suppressed_reason()
        if reason:
            self._append_log(f"暂不能导出：{reason}")
            return
        try:
            lease = transport.claim_transfer(failure_type=receive.FlightLogError)
        except (OSError, receive.FlightLogError) as exc:
            self._append_log(exc)
            return
        self.lease = lease
        self.source_transport = transport
        self.cancel_requested = False
        self.latest_progress = (0, 0)
        self.started_at = time.monotonic()
        self.progress_var.set(0)
        self.progress_text.set("正在请求飞控日志；其他轮询暂时让位。")
        self.receive_btn.configure(state="disabled")
        self.cancel_btn.configure(state="normal")
        self._append_log(f"使用当前连接 {lease.port_name} @ {lease.baudrate}；连接保持打开。")
        self.worker = threading.Thread(target=self._worker,
                                       args=(lease, Path(self.dir_var.get())), daemon=True)
        self.worker.start()

    def _worker(self, lease, output_dir):
        result, error = None, None
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
            directory = Path(tempfile.mkdtemp(prefix="receive_", dir=output_dir))
            lease.write(b"TELEM STREAM off\r\n")
            result = receive.receive_dump(
                lease, directory, log=lambda text: self.events.put(("log", text)),
                progress=self._progress, should_cancel=lambda: self.cancel_requested)
            meta = json.loads(result.meta_path.read_text(encoding="utf-8"))
            meta["baud"] = lease.baudrate
            meta["host_connection"] = {"port": lease.port_name, "baud": lease.baudrate,
                                       "generation": lease.session.generation}
            result.meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception as exc:
            error = str(exc)
        finally:
            try:
                if result is None or not result.complete:
                    lease.write(b"FLOG CANCEL\r\n")
                lease.flush()
            except (OSError, receive.FlightLogError) as exc:
                self.events.put(("log", f"结束导出：{exc}"))
            lease.close()
        self.events.put(("finished", (lease, result, error)))

    def _progress(self, done, total):
        self.latest_progress = (done, total)

    def _cancel(self):
        self.cancel_requested = True
        if self.lease is not None:
            self.lease.abort("cancelled")
        self.progress_text.set("正在取消，保留已收到的数据…")

    def _poll_events(self):
        if self.closed:
            return
        transport = self.panel.transport
        if self.busy():
            self.link_var.set(f"当前接收：{self.lease.port_name} @ {self.lease.baudrate} · 同一串口连接")
        elif transport is self.panel.serial_transport and transport.is_connected:
            baud = getattr(getattr(transport, "port", None), "baudrate", "未知")
            self.link_var.set(f"当前连接：{transport.active_port} @ {baud}")
        else:
            self.link_var.set("当前未连接串口，请在顶部连接飞控。")
        if self.busy() and not self.cancel_requested:
            done, total = self.latest_progress
            elapsed = max(.001, time.monotonic() - self.started_at)
            self.progress_var.set(100 * done / total if total else 0)
            if total:
                self.progress_text.set(f"已接收 {done:,} / {total:,} 字节 · {done / elapsed / 1024:.1f} KiB/s · {elapsed:.0f} 秒")
        for _ in range(100):
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                self._append_log(value)
                continue
            lease, result, error = value
            if (self.panel.transport is self.source_transport
                    and self.source_transport.connection_generation == lease.session.generation):
                # Existing page logic re-enables streaming if the current page needs it.
                self.panel.dashboard_stream_requested = False
            self.receive_btn.configure(state="normal")
            self.cancel_btn.configure(state="disabled")
            if error:
                self.progress_text.set(f"接收已取消：{error}" if self.cancel_requested else f"接收失败：{error}")
                self._append_log(self.progress_text.get())
            if result is not None:
                self.progress_var.set(100 * result.good_bytes / result.total_bytes if result.total_bytes else 0)
                state = "接收完成" if result.complete else ("已取消，保留部分数据" if self.cancel_requested else "接收不完整")
                self.progress_text.set(f"{state}：{result.records} 条记录，缺少 {result.missing_bytes:,} 字节")
                self._append_log(self.progress_text.get())
                for detail in result.errors[:8]:
                    self._append_log(detail)
                self.on_result(result)
            self.lease = None
        self.timer = self.after(100, self._poll_events)

    def _destroy(self, event):
        if event.widget is self:
            self.closed = True
            self._cancel()
            self.after_cancel(self.timer)
