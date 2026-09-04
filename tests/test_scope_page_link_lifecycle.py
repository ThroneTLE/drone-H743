"""示波器页在链路生命周期上的两处缺陷（审核者实机复核前的复现测试）。

`test_scope_page.py` 的 FakeTransport 恒为 `is_connected = True`，所以"先开页再连线"
与"USB 拔插后重连"这两条路径零覆盖。真实使用里这两条恰恰是最常走的：面板一启动
示波器页就可能被点开；固件在 USB 出口下拔线会自动 `stream=0`（usb_lost），
重新插上后必须有人再发一次 `TELEM STREAM on`。

缺陷 1：页面可见时链路未连 → `_scope_sync_stream` 直接返回；之后连上了，可见性
没有变化，`_scope_poll_tick` 永远不再发 `STREAM on`，也不挂二进制 sink。
缺陷 2：链路断开重连后 `scope_stream_requested` 仍为 True，同样不再发。

第三条是滑块回显的竞态：松手发出 `PARAM SET` 后，一帧编码于发送之前、携带旧值
的全量刷新帧（数传上一帧在线 24 ms）仍可能到达，被误判为 diverged 并把滑块拽回旧值。
"""

from __future__ import annotations

import time
import tkinter as tk

import pytest

from tools import drone_tcp_panel as panel
from tools.panel_lib.pages import scope as scope_page

from tests.test_scope_page import SCHEMA_LINES, FakeTransport


class SwitchableTransport(FakeTransport):
    def __init__(self, connected: bool) -> None:
        super().__init__()
        self.is_connected = connected


@pytest.fixture(scope="module")
def _panel():
    try:
        instance = panel.DronePanel()
    except tk.TclError as exc:  # pragma: no cover - 无显示环境
        pytest.skip(f"Tk display unavailable: {exc}")
    try:
        yield instance
    finally:
        instance.destroy()


@pytest.fixture
def app(_panel):
    _panel._scope_reset_session()
    _panel.transport = SwitchableTransport(connected=False)
    _panel.update_idletasks()
    return _panel


def select_scope_tab(app) -> None:
    app.notebook.select(app.scope_tab)
    app.update_idletasks()


def test_stream_starts_when_the_link_comes_up_after_the_page_is_open(app) -> None:
    select_scope_tab(app)
    app._scope_poll_tick(time.monotonic())
    assert "TELEM STREAM on" not in app.transport.lines  # 没连线不该发

    app.transport.is_connected = True
    app._scope_poll_tick(time.monotonic())
    app._scope_poll_tick(time.monotonic())
    assert app.transport.lines.count("TELEM STREAM on") == 1
    assert app.transport.binary_sink is not None


def test_stream_is_re_requested_after_a_reconnect(app) -> None:
    app.transport.is_connected = True
    select_scope_tab(app)
    app._scope_poll_tick(time.monotonic())
    assert app.transport.lines.count("TELEM STREAM on") == 1

    # 拔线：固件那头在 USB 出口下会自己 stream=0；面板换了一个新的 transport 对象。
    app.transport.is_connected = False
    app._scope_poll_tick(time.monotonic())
    app.transport = SwitchableTransport(connected=True)
    app._scope_poll_tick(time.monotonic())
    app._scope_poll_tick(time.monotonic())
    assert app.transport.lines.count("TELEM STREAM on") == 1
    assert app.transport.binary_sink is not None


def test_a_stale_echo_right_after_sending_does_not_diverge(app) -> None:
    app.transport.is_connected = True
    for line in SCHEMA_LINES:
        app._handle_board_line(line)
    app.update_idletasks()

    now = time.monotonic()
    app.scope_slider_vars[3].set(2.0)
    app._scope_send_param(3, now)
    assert app.scope_slider_state[3] == scope_page.SCOPE_STATE_PENDING

    # 发送后 10 ms 内到达的旧值：还在在线帧的传输窗口里，不能据此判 diverged。
    app._scope_note_param_echo(3, 0.5, now=now + 0.010)
    assert app.scope_slider_state[3] == scope_page.SCOPE_STATE_PENDING
    assert app.scope_slider_vars[3].get() == pytest.approx(2.0)

    # 新值到了：confirmed。
    app._scope_note_param_echo(3, 2.0, now=now + 0.040)
    assert app.scope_slider_state[3] == scope_page.SCOPE_STATE_CONFIRMED

    # 宽限期过后仍是旧值，才是真的被固件拒了。
    app.scope_slider_vars[4].set(9.5)
    app._scope_send_param(4, now)
    app._scope_note_param_echo(4, 5.0, now=now + 1.0)
    assert app.scope_slider_state[4] == scope_page.SCOPE_STATE_DIVERGED
    assert app.scope_slider_vars[4].get() == pytest.approx(5.0)
