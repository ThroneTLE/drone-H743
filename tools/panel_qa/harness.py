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

# 只有这几种才是"这台机器没有图形环境"。别的 TclError 是装置或页面自己坏了。
_NO_DISPLAY_MARKERS = (
    "no display name",
    "couldn't connect to display",
    "can't find a usable init.tcl",
    "application-specific initialization failed",
)


def is_display_unavailable(exc: BaseException) -> bool:
    """区分“没有显示环境”和“装置坏了”。

    `except TclError: pytest.skip(...)` 是个陷阱：构造面板时的任何 Tcl 错误都会被
    写成“无显示环境”跳过，于是一整批回归安静消失，而绿色的测试报告一个字都不说。
    这个项目已经吃过一次安静失败的亏（`work-modes.md` 的 `gz_dps`），所以只在确实
    连不上显示时才允许跳过，其余一律让它红。
    """
    text = str(exc).lower()
    return any(marker in text for marker in _NO_DISPLAY_MARKERS)


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

    def start(self, *_args, **_kwargs) -> None:
        """连接。真实 transport 在这里起线程/开套接字；替身只翻状态并进代次。

        必须如实翻 `is_connected` 和 `connection_generation`，否则"重连之后旧会话的
        东西不许污染新会话"这条契约在装置里根本演不出来（审核 Q2）。
        """
        self.stopped = False
        self.is_connected = True
        self._connection_generation += 1

    def stop(self) -> None:
        self.stopped = True
        self.is_connected = False
        self.binary_sink = None
        self._connection_generation += 1

    def cancel_pending_sends(self) -> None:
        pass

    def close(self) -> None:
        self.stop()


@dataclass
class PendingAfter:
    """一条被记录下来的定时回调。

    第一版只存了 (delay, callback)，`after_cancel` 是个空函数，而且启动时排下去的
    周期任务在替换 `after` 之前就被全部取消了——记录里只剩两个 resize 回调，接收/
    IMU/链路/录制的循环一个都不在（审核 Q4）。现在有真实的 id、到期时刻和取消语义，
    并且 `after` 在**构造面板之前**就被换掉，启动链因此完整保留。
    """

    ident: str
    due_ms: int
    callback: object
    args: tuple = ()
    cancelled: bool = False

    @property
    def delay_ms(self) -> int:
        return self.due_ms


@dataclass
class OfflinePanel:
    """一次离线 QA 会话。用 `launch()` 构造，用完 `destroy()`（或 with 语句）。"""

    panel: object
    transport: MemoryTransport
    scale: float
    callback_errors: list[BaseException] = field(default_factory=list)
    pending_after: list[PendingAfter] = field(default_factory=list)
    transports: dict = field(default_factory=dict)
    clock_ms: int = 0
    _after_seq: int = 0
    _real_after: object = None
    _real_after_cancel: object = None
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

        session = cls(panel=None, transport=MemoryTransport(connected=connected),
                      scale=scale, _restore=restore)
        if record_after:
            # **在构造之前**换掉：启动时排下去的接收/链路/录制周期任务因此会被记录，
            # 而不是先被排下去、再被我们一把取消（审核 Q4）。
            session._real_after = tk.Misc.after
            session._real_after_cancel = tk.Misc.after_cancel

            # 必须是**普通函数**才能拿到调用它的那个 widget：绑定方法放进类属性不会
            # 再次绑定，widget 就丢了，而 `after_idle` 需要它。
            def virtual_after(widget, delay, callback=None, *args):
                return session._virtual_after(widget, delay, callback, *args)

            def virtual_after_cancel(widget, ident):
                return session._virtual_after_cancel(widget, ident)

            patch(tk.Misc, "after", virtual_after)
            patch(tk.Misc, "after_cancel", virtual_after_cancel)

        try:
            instance = panel_module.DronePanel()
        except BaseException:
            for target, name, value in reversed(restore):
                setattr(target, name, value)
            raise

        session.panel = instance
        session._prepare(size, connected)
        return session

    def _prepare(self, size: tuple[int, int], connected: bool) -> None:
        panel = self.panel
        panel.update_idletasks()

        # 回调异常不再只落盘：装置必须能断言“这一页没有静默抛异常”。
        errors = self.callback_errors

        def record_callback_exception(exc_type, exc_value, exc_tb):
            errors.append(exc_value)

        panel.report_callback_exception = record_callback_exception

        # **三条**通道全部换成替身。第一版只换了 serial 和当前 transport，于是在面板里
        # 选到 TCP/UDP 再点连接，会走到真实适配器去开套接字（审核 Q2）。
        self.transports = {
            "serial": self.transport,
            "tcp": MemoryTransport(connected=connected, port="QA-TCP"),
            "udp": MemoryTransport(connected=connected, port="QA-UDP"),
        }
        panel.serial_transport = self.transports["serial"]
        panel.tcp_transport = self.transports["tcp"]
        panel.udp_transport = self.transports["udp"]
        panel.transport = panel._current_transport()
        self.transport = panel.transport
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

    # --- 虚拟调度器（审核 Q4） ---

    def _virtual_after(self, widget, delay, callback=None, *args):
        """`tk.Misc.after` 的替身：记录而不是丢弃，并且返回**真实可取消**的 id。"""
        if callback is None:                     # after(ms) 当 sleep 用，装置里没有意义
            return ""
        # `after_idle()` 转成 `after('idle', ...)`。它是"下一次空闲就跑"，不是定时器；
        # 交回真实实现，否则 ttk 内部的一堆布局收尾会永远不发生。
        if delay == "idle":
            return self._real_after(widget, delay, callback, *args)
        self._after_seq += 1
        ident = f"qa-after-{self._after_seq}"
        self.pending_after.append(
            PendingAfter(ident, self.clock_ms + int(delay), callback, tuple(args))
        )
        return ident

    def _virtual_after_cancel(self, widget, ident) -> None:
        if not isinstance(ident, str) or not ident.startswith("qa-after-"):
            return self._real_after_cancel(widget, ident)
        for item in self.pending_after:
            if item.ident == ident:
                item.cancelled = True
                return

    def run_pending_after(self) -> int:
        """执行当前排队且未取消的回调（只跑这一批，重排的留到下次）。"""
        batch, self.pending_after = self.pending_after, []
        ran = 0
        for item in sorted(batch, key=lambda entry: entry.due_ms):
            if item.cancelled:
                continue
            item.callback(*item.args)
            ran += 1
        return ran

    def advance_ms(self, milliseconds: int) -> int:
        """把虚拟时钟往前推，按到期顺序执行到期的回调。"""
        self.clock_ms += int(milliseconds)
        due = [item for item in self.pending_after
               if not item.cancelled and item.due_ms <= self.clock_ms]
        self.pending_after = [item for item in self.pending_after if item not in due]
        for item in sorted(due, key=lambda entry: entry.due_ms):
            item.callback(*item.args)
        return len(due)

    def scheduled_callbacks(self) -> list[str]:
        """当前排着的回调名字。用来断言启动链真的被保留下来了。"""
        return [getattr(item.callback, "__name__", repr(item.callback))
                for item in self.pending_after if not item.cancelled]

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


__all__ = [
    "MemoryTransport",
    "OFFLINE_TITLE",
    "OfflinePanel",
    "PendingAfter",
    "is_display_unavailable",
]
