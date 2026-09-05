"""真实 `DronePanel` 的离线装置。

“真实”是重点。报告 §2 明确要求用真实页面方法而不是字符串断言，因为被测的东西是
布局和交互，源码搜出来的关键字证明不了按钮点不点得到。所以这里构造的是**没有任何
删减的 `DronePanel`**，只做三类无害改动：

* 窗口不上屏（`withdraw` + alpha=0），标题写死 `OFFLINE QA ... NO HARDWARE`，
  以免有人把 QA 窗口错当成连着板子的地面站；
* 启动副作用（自动重连、读用户历史产物、落盘）掐掉，串口枚举返回空；
* `after` 改成**记录**而不是丢弃 —— 报告的判据里有一句“不能通过关闭所有 after
  回调来冒充异步端到端测试通过”，所以定时回调必须还能被显式驱动。

transport 一律是 `MemoryTransport`。它带 `connection_generation`，因为“重连之后旧
会话的东西不许污染新会话”是连接修复那批的核心契约，后续包必须能在装置里模拟它。
"""

from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path

from .geometry import LeafPage, probe_geometry


OFFLINE_TITLE = "OFFLINE QA — SIMULATED TRANSPORT — NO HARDWARE"


class MemoryTransport:
    """只把命令记在内存里的 transport。一个字节都不出进程。"""

    def __init__(self, *, connected: bool = True, port: str = "QA") -> None:
        self.is_connected = connected
        self.active_port = port
        self.lines: list[str] = []
        self.frames: list[tuple[int, bytes]] = []
        self.binary_sink = None
        self.binary_unclaimed = 0
        self.stopped = False
        self._connection_generation = 1

    @property
    def connection_generation(self) -> int:
        return self._connection_generation

    def bump_generation(self) -> int:
        """模拟一次断开重连。代次是判断“这份数据属于哪次连接”的唯一依据。"""
        self._connection_generation += 1
        self.binary_sink = None
        return self._connection_generation

    # --- TransportBase 的那一小块被面板真正用到的表面 ---

    def send_line(self, line: str) -> bool:
        self.lines.append(line)
        return True

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        self.frames.append((int(function), bytes(payload)))
        return True

    def set_binary_sink(self, sink) -> None:
        self.binary_sink = sink

    def stop(self) -> None:
        self.stopped = True

    def cancel_pending_sends(self) -> None:
        pass

    def close(self) -> None:
        self.stopped = True


@dataclass
class PendingAfter:
    delay_ms: int
    callback: object
    args: tuple = ()


@dataclass
class OfflinePanel:
    """一次离线 QA 会话。用 `launch()` 构造，用完 `destroy()`（或 with 语句）。"""

    panel: object
    transport: MemoryTransport
    scale: float
    callback_errors: list[BaseException] = field(default_factory=list)
    pending_after: list[PendingAfter] = field(default_factory=list)
    _restore: list = field(default_factory=list)

    # ---------------------------------------------------------------- 构造

    @classmethod
    def launch(cls, *, scale: float = 1.0, size: tuple[int, int] = (1366, 768),
               connected: bool = True, record_after: bool = True) -> "OfflinePanel":
        from tools import drone_tcp_panel as panel_module

        restore: list = []

        def patch(target, name, value):
            restore.append((target, name, getattr(target, name)))
            setattr(target, name, value)

        # DPI 感知必须在根窗口创建前定死，否则拿到的是这台机器的真实缩放，
        # 三档矩阵就不可复现了。
        patch(panel_module, "enable_hidpi_awareness", lambda: scale)
        for method in ("_restore_last_connection", "_save_panel_state",
                       "_validation_load_latest_artifact", "_v1_load_latest_session"):
            if hasattr(panel_module.DronePanel, method):
                patch(panel_module.DronePanel, method, lambda self: None)
        patch(panel_module.DronePanel, "_load_panel_state", lambda self: {})
        patch(panel_module.DronePanel, "_refresh_serial_ports", lambda self: [])
        for name in ("showerror", "showwarning", "showinfo"):
            patch(panel_module.messagebox, name, lambda *a, **k: None)

        original_tk_init = tk.Tk.__init__

        def hidden_init(self, *args, **kwargs):
            original_tk_init(self, *args, **kwargs)
            self.withdraw()

        patch(tk.Tk, "__init__", hidden_init)

        try:
            instance = panel_module.DronePanel()
        except BaseException:
            for target, name, value in reversed(restore):
                setattr(target, name, value)
            raise

        transport = MemoryTransport(connected=connected)
        session = cls(panel=instance, transport=transport, scale=scale, _restore=restore)
        session._prepare(size, record_after)
        return session

    def _prepare(self, size: tuple[int, int], record_after: bool) -> None:
        panel = self.panel
        panel.update_idletasks()
        # 启动时排下去的定时器先全清掉；之后的调度按 record_after 决定是记录还是照常。
        for ident in panel.tk.call("after", "info"):
            try:
                panel.after_cancel(ident)
            except tk.TclError:                  # pragma: no cover - 已经跑掉了
                pass
        if record_after:
            def recording_after(delay, callback=None, *args):
                if callback is None:             # after(ms) 当 sleep 用，装置里没有意义
                    return ""
                self.pending_after.append(PendingAfter(int(delay), callback, args))
                return f"qa-after-{len(self.pending_after)}"

            panel.after = recording_after
            panel.after_cancel = lambda _ident: None

        # 回调异常不再只落盘：装置必须能断言“这一页没有静默抛异常”。
        errors = self.callback_errors

        def record_callback_exception(exc_type, exc_value, exc_tb):
            errors.append(exc_value)

        panel.report_callback_exception = record_callback_exception

        panel.serial_transport = self.transport
        panel.transport = self.transport
        panel.title(OFFLINE_TITLE)
        panel.attributes("-alpha", 0.0)
        panel.deiconify()
        self.resize(*size)

    # ---------------------------------------------------------------- 驱动

    def resize(self, width: int, height: int) -> None:
        self.panel.geometry(f"{width}x{height}")
        self.panel.update()

    def pump(self) -> None:
        self.panel.update()

    def run_pending_after(self) -> int:
        """执行当前排队的定时回调（只跑这一批，重排的留到下次）。"""
        batch, self.pending_after = self.pending_after, []
        for item in batch:
            item.callback(*item.args)
        return len(batch)

    def leaf_pages(self) -> list[LeafPage]:
        """展开“校准”“传感器”两个分组，得到真正有内容的叶页。"""
        panel = self.panel
        pages: list[LeafPage] = []
        groups = {
            id(getattr(panel, "calibration_group_tab", None)): getattr(
                panel, "calibration_notebook", None),
            id(getattr(panel, "sensor_group_tab", None)): getattr(
                panel, "sensor_notebook", None),
        }
        for tab in panel.notebook.tabs():
            top = panel.nametowidget(tab)
            label = panel.notebook.tab(tab, "text")
            inner = groups.get(id(top))
            if inner is not None:
                for sub in inner.tabs():
                    pages.append(LeafPage(
                        f"{label} / {inner.tab(sub, 'text')}", top, inner, sub))
            else:
                pages.append(LeafPage(label, top, None, None))
        return pages

    def select(self, page: LeafPage) -> None:
        self.panel.notebook.select(page.top)
        if page.sub_notebook is not None:
            page.sub_notebook.select(page.sub_tab)
        self.panel.update()

    def probe_geometry(self, **kwargs):
        return probe_geometry(self, **kwargs)

    def screenshot(self, path: Path) -> Path:
        """抓当前窗口。alpha=0 的窗口截不出内容，所以抓之前先恢复不透明。"""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.panel.attributes("-alpha", 1.0)
        self.panel.update()
        try:
            from PIL import ImageGrab
        except ImportError as exc:               # pragma: no cover - 可选依赖
            raise RuntimeError("screenshots need Pillow (pip install pillow)") from exc
        panel = self.panel
        box = (panel.winfo_rootx(), panel.winfo_rooty(),
               panel.winfo_rootx() + panel.winfo_width(),
               panel.winfo_rooty() + panel.winfo_height())
        ImageGrab.grab(bbox=box).save(path)
        self.panel.attributes("-alpha", 0.0)
        return path

    # ---------------------------------------------------------------- 收尾

    def destroy(self) -> None:
        try:
            self.panel.destroy()
        except tk.TclError:                      # pragma: no cover - 已经销毁
            pass
        finally:
            for target, name, value in reversed(self._restore):
                setattr(target, name, value)
            self._restore = []

    def __enter__(self) -> "OfflinePanel":
        return self

    def __exit__(self, *_exc) -> None:
        self.destroy()


__all__ = ["MemoryTransport", "OFFLINE_TITLE", "OfflinePanel", "PendingAfter"]
