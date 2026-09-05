"""R-T1-6 返修：关闭 2026-09-04 软件审核的 R1~R4。

审核对象是 `63223acd`。四条都不是"再多测一点"能发现的，它们各自对应第一版实现里
一个具体的结构错误：

  R1  Tk 线程仍在做 I/O 和等待。`start()` 自己建文件写表头；`stop()` 直接 drain
      5 秒；队列满时控制命令走的是无 timeout 的 `Queue.put()`。审核实测：表头每次
      write/flush 注入 120 ms → start 阻塞约 361 ms；满队列 → stop 约 358 ms；
      默认 drain 实测等满 5.012 s 后返回，状态还是 recording。
  R2  样本入口没有来源。代次只在 Tk 的 poll 里比，而帧走接收线程——新连接的第一帧
      在 poll 之前就写进了旧文件。审核实测：旧 CSV 里出现 roll=999，之后才写
      link_changed 尾行；换成另一个 transport 对象、两边整数代次都是 2 时，录制
      干脆不结束，继续把 555 写进旧文件。
  R3  submit 在锁内取 session、出锁后才入队，stop 能插进中间。审核实测
      `submit=True, rows_written=0, rows_dropped=0`——返回成功的样本无声消失。
  R4  终态无条件覆盖 `_status`。stop 超时后开新会话，旧 finish 落地时把界面状态
      指回旧文件，而 `_session` 指向新文件。

这些用例在 `63223acd` 上应当全红。写的时候是先在那个提交上跑出红灯的，红灯原文见
交付说明。
"""

from __future__ import annotations

import threading
import time
import tkinter as tk
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools import drone_tcp_panel as panel
from tools import panel_qa
from tools.panel_lib import record_service
from tools.panel_lib.pages import dashboard as dashboard_page
from tools.panel_lib.record_service import (
    LinkIdentity,
    RecordSchema,
    TelemetryRecorder,
    read_record_rows,
)


TWO_CHANNELS = RecordSchema(schema_hash=0x12345678, channel_names=("a", "b"))

# UI 线程上的调用要"立刻"返回。给到 50 ms 是为了容忍 Windows 上的调度抖动，仍然
# 远小于审核实测的 358~5012 ms，也远小于一帧（33 ms）的若干倍。
UI_CALL_BUDGET_S = 0.05


def sample(t_us: int, values: dict[int, float]):
    return SimpleNamespace(t_us=t_us, values=values)


class BlockingWriter:
    """表头写得下去，数据行一直堵着，直到测试放行。模拟"盘卡住了"。"""

    def __init__(self) -> None:
        self.release = threading.Event()
        self.rows = 0

    def write(self, text: str) -> None:
        if text.startswith("#") or text.startswith("t_s"):
            return
        self.rows += 1
        self.release.wait(5.0)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


class SlowOpenWriter:
    """连表头都写得很慢——审核 R1 注入的就是这一段。"""

    def __init__(self, delay_s: float = 0.12) -> None:
        self.delay_s = delay_s

    def write(self, _text: str) -> None:
        time.sleep(self.delay_s)

    def flush(self) -> None:
        time.sleep(self.delay_s)

    def close(self) -> None:
        pass


def install_writer(monkeypatch, writer, path: Path) -> None:
    monkeypatch.setattr(
        record_service, "unique_record_path", lambda *_a, **_k: (writer, path)
    )


@pytest.fixture
def recorder():
    instance = TelemetryRecorder()
    try:
        yield instance
    finally:
        instance.close(timeout=1.0)


# ---------------------------------------------------------------- R1 生命周期不阻塞


def test_start_does_no_disk_io_on_the_calling_thread(recorder, tmp_path, monkeypatch):
    """审核 R1：表头 write/flush 各注入 120 ms 时，第一版 start 阻塞约 361 ms。"""
    install_writer(monkeypatch, SlowOpenWriter(0.12), tmp_path / "slow.csv")

    started = time.perf_counter()
    status = recorder.start(tmp_path, TWO_CHANNELS, stem="telem_r1")
    elapsed = time.perf_counter() - started

    assert elapsed < UI_CALL_BUDGET_S, f"start() 在调用线程上阻塞了 {elapsed * 1000:.0f} ms"
    assert status.state == record_service.STATE_STARTING
    assert status.path is None, "文件还没建出来，就不该先编一个文件名"
    assert recorder.wait_idle(10.0)
    assert recorder.status().state == record_service.STATE_RECORDING


def test_stop_returns_immediately_while_the_writer_is_stuck(recorder, tmp_path,
                                                            monkeypatch):
    """审核 R1：第一版 stop 会一直等到 drain 超时（实测 5.012 s）才返回。"""
    writer = BlockingWriter()
    install_writer(monkeypatch, writer, tmp_path / "blocked.csv")
    recorder.start(tmp_path, TWO_CHANNELS, stem="telem_r1b")
    assert recorder.wait_idle(5.0)
    recorder.submit(sample(0, {0: 1.0}))
    time.sleep(0.05)                             # 让写线程真的进到那次阻塞的 write

    started = time.perf_counter()
    status = recorder.stop()
    elapsed = time.perf_counter() - started

    assert elapsed < UI_CALL_BUDGET_S, f"stop() 阻塞了 {elapsed * 1000:.0f} ms"
    assert status.state == record_service.STATE_STOPPING
    writer.release.set()
    assert recorder.wait_idle(5.0)
    assert recorder.status().state == record_service.STATE_FINISHED


def test_control_requests_are_accepted_when_the_row_queue_is_full(tmp_path, monkeypatch):
    """审核 R1：控制命令和数据行挤在同一个有界队列里，满了就把调用者挂死。"""
    writer = BlockingWriter()
    install_writer(monkeypatch, writer, tmp_path / "full.csv")
    instance = TelemetryRecorder(queue_limit=4)
    try:
        instance.start(tmp_path, TWO_CHANNELS, stem="telem_r1c")
        assert instance.wait_idle(5.0)
        for index in range(200):
            instance.submit(sample(index, {0: float(index)}))
        assert instance.status().rows_dropped > 0, "队列该满了"

        started = time.perf_counter()
        status = instance.stop()                 # 第一版这里是无 timeout 的 put()
        elapsed = time.perf_counter() - started

        assert elapsed < UI_CALL_BUDGET_S, f"满队列时 stop() 阻塞了 {elapsed * 1000:.0f} ms"
        assert status.state == record_service.STATE_STOPPING
        writer.release.set()
        assert instance.wait_idle(5.0)
    finally:
        writer.release.set()
        instance.close(timeout=2.0)


def test_close_waits_but_within_its_declared_bound(tmp_path, monkeypatch):
    """退出是**唯一**允许等的地方：不等就丢数据，而此时已经没有界面可卡。"""
    writer = BlockingWriter()
    install_writer(monkeypatch, writer, tmp_path / "exit.csv")
    instance = TelemetryRecorder()
    instance.start(tmp_path, TWO_CHANNELS, stem="telem_r1d")
    assert instance.wait_idle(5.0)
    instance.submit(sample(0, {0: 1.0}))
    time.sleep(0.05)

    started = time.perf_counter()
    status = instance.close(timeout=0.2)
    elapsed = time.perf_counter() - started
    writer.release.set()

    assert elapsed < 1.0, f"close() 超出了它自己声明的上限：{elapsed:.2f} s"
    # 没等完就要如实说：状态还停在 stopping，而不是谎报 finished。
    assert status.state == record_service.STATE_STOPPING


def test_the_page_lifecycle_buttons_never_touch_the_disk(app, monkeypatch):
    """审核 R1 点名：按钮、换表、代次轮询、关窗都调这些方法，只让 render_tick 变快不够。"""
    load_schema(app)
    writer = BlockingWriter()
    install_writer(monkeypatch, writer, tmp_path_for(app) / "ui.csv")
    try:
        for action in (app._dashboard_toggle_record, app._dashboard_stop_record):
            started = time.perf_counter()
            action()
            elapsed = time.perf_counter() - started
            assert elapsed < UI_CALL_BUDGET_S, (
                f"{action.__name__} 在 Tk 线程上阻塞了 {elapsed * 1000:.0f} ms"
            )
    finally:
        writer.release.set()
        assert app.dashboard_recorder.wait_idle(5.0)


# ---------------------------------------------------------------- R2 链路身份


def test_a_frame_from_a_new_transport_never_reaches_the_old_file(app):
    """审核 R2：新代次的帧先到、UI poll 后到时，第一版已经把它写进旧文件了。"""
    schema = load_schema(app)
    path = begin_record(app)
    old_transport = app.transport

    # 换连接。注意：**不** poll——审核的复现就是把帧放在 poll 之前。
    new_transport = panel_qa.MemoryTransport(connected=True)
    new_transport.bump_generation()
    app.transport = new_transport
    app._dashboard_on_binary_frame(
        0x2230,
        panel_qa.telemetry_frame({0: 999.0}, schema_hash=schema.computed_hash(), t_us=1),
        transport=new_transport,
    )
    assert app.dashboard_recorder.wait_idle(5.0)

    rows = read_record_rows(path)
    assert all("999" not in cell for row in rows for cell in row), rows
    status = app.dashboard_recorder.status()
    assert status.end_reason == record_service.REASON_LINK_CHANGED
    assert old_transport is not new_transport


def test_two_transports_with_the_same_generation_are_still_different_links(app):
    """审核 R2 的第二半：每个 transport 各有一个计数器，整数相同不代表同一条链路。"""
    schema = load_schema(app)
    first = app.transport
    path = begin_record(app)

    second = panel_qa.MemoryTransport(connected=True)
    assert second.connection_generation == first.connection_generation, (
        "这个用例的前提就是两边整数代次相同"
    )
    app.transport = second
    app._dashboard_on_binary_frame(
        0x2230,
        panel_qa.telemetry_frame({0: 555.0}, schema_hash=schema.computed_hash(), t_us=2),
        transport=second,
    )
    assert app.dashboard_recorder.wait_idle(5.0)

    rows = read_record_rows(path)
    assert all("555" not in cell for row in rows for cell in row), rows
    assert app.dashboard_recorder.status().end_reason == record_service.REASON_LINK_CHANGED


def test_the_binary_sink_pins_the_transport_that_registered_it(app):
    """sink 必须记住是谁挂上来的；只读 `self.transport` 会把重连前后的帧混为一谈。"""
    load_schema(app)
    origin = app.transport
    sink = app._dashboard_binary_sink(origin)
    seen: list = []
    app.dashboard_recorder.submit = lambda s, **kw: seen.append(kw) or True

    app.transport = panel_qa.MemoryTransport(connected=True)   # 换了当前 transport
    sink(0x2230, panel_qa.telemetry_frame(
        {0: 1.0}, schema_hash=app.dashboard_schema.computed_hash()))

    assert seen and seen[0]["transport"] is origin


# ---------------------------------------------------------------- R3 交错


def test_every_accepted_sample_survives_a_concurrent_stop(recorder, tmp_path):
    """审核 R3：submit 返回 True 的样本被排在 finish 之后，然后被无声丢弃。

    不靠运气：断言的是一条不变量——**返回 True 的行数必须等于文件里的行数**。
    """
    recorder.start(tmp_path, TWO_CHANNELS, stem="telem_r3")
    assert recorder.wait_idle(5.0)

    accepted = 0
    lock = threading.Lock()
    go = threading.Event()
    done = threading.Event()

    def producer() -> None:
        nonlocal accepted
        go.wait()
        while not done.is_set():
            if recorder.submit(sample(0, {0: 1.0})):
                with lock:
                    accepted += 1

    threads = [threading.Thread(target=producer) for _ in range(4)]
    for thread in threads:
        thread.start()
    go.set()
    time.sleep(0.05)
    status = recorder.stop()
    done.set()
    for thread in threads:
        thread.join(5.0)
    assert recorder.wait_idle(5.0)

    final = recorder.status()
    assert final.state == record_service.STATE_FINISHED
    written = len(read_record_rows(status.path or final.path))
    assert accepted > 0, "这个用例需要真的提交到一些样本"
    assert written == accepted, (
        f"接受了 {accepted} 行，文件里只有 {written} 行——返回 True 的样本消失了"
    )
    assert final.rows_written == accepted


def test_a_sample_submitted_after_stop_is_refused_not_silently_dropped(recorder, tmp_path):
    """结束栅栏之后不能再说"收下了"。"""
    recorder.start(tmp_path, TWO_CHANNELS, stem="telem_r3b")
    assert recorder.wait_idle(5.0)
    recorder.stop()
    assert recorder.submit(sample(0, {0: 1.0})) is False


# ---------------------------------------------------------------- R4 状态归属


def test_a_stale_finish_does_not_overwrite_the_new_session(tmp_path, monkeypatch):
    """审核 R4：旧会话收尾时把界面状态改回旧文件，而 `_session` 指向新文件。"""
    writer = BlockingWriter()
    paths = iter([tmp_path / "fake-1.csv", tmp_path / "fake-2.csv"])
    writers = iter([writer, BlockingWriter()])
    monkeypatch.setattr(
        record_service, "unique_record_path",
        lambda *_a, **_k: (next(writers), next(paths)),
    )
    instance = TelemetryRecorder()
    try:
        instance.start(tmp_path, TWO_CHANNELS, stem="one")
        assert instance.wait_idle(5.0)
        instance.submit(sample(0, {0: 1.0}))
        time.sleep(0.05)
        instance.stop()                          # 写线程还堵在第一份文件上

        second = instance.start(tmp_path, TWO_CHANNELS, stem="two")
        assert second.state == record_service.STATE_STARTING

        writer.release.set()                     # 旧会话现在才收尾
        time.sleep(0.2)

        status = instance.status()
        assert status.path != (tmp_path / "fake-1.csv"), (
            "旧会话的终态把新会话的状态盖掉了"
        )
        assert status.state in (record_service.STATE_STARTING,
                                record_service.STATE_RECORDING)
        # 旧文件的结果没有丢，只是不再冒充"当前状态"。
        assert any(entry.path == (tmp_path / "fake-1.csv")
                   for entry in instance.history())
    finally:
        writer.release.set()
        instance.close(timeout=1.0)


# ---------------------------------------------------------------- 真实面板夹具


@pytest.fixture(scope="module")
def _panel():
    try:
        instance = panel.DronePanel()
    except tk.TclError as exc:
        if not panel_qa.is_display_unavailable(exc):
            raise
        pytest.skip(f"Tk display unavailable: {exc}")        # pragma: no cover
    try:
        yield instance
    finally:
        instance.destroy()


@pytest.fixture
def app(_panel, monkeypatch, tmp_path):
    monkeypatch.setattr(type(_panel), "_save_panel_state", lambda self: None)
    monkeypatch.setattr(dashboard_page, "TELEMETRY_DIR", tmp_path)
    monkeypatch.setattr(dashboard_page, "dated_directory", lambda _root: tmp_path / "day")
    _panel._dashboard_reset_session()
    _panel.dashboard_recorder.close(timeout=1.0)
    _panel.dashboard_recorder = TelemetryRecorder()
    _panel.transport = panel_qa.MemoryTransport(connected=True)
    _panel.update_idletasks()
    try:
        yield _panel
    finally:
        _panel.dashboard_recorder.close(timeout=1.0)


def load_schema(app):
    for line in panel_qa.telemetry_schema_lines():
        app._handle_board_line(line)
    app.update_idletasks()
    return app.dashboard_schema


def begin_record(app):
    app._dashboard_toggle_record()
    assert app.dashboard_recorder.wait_idle(5.0)
    app._dashboard_refresh_record_status()
    return app.dashboard_record_path


def tmp_path_for(app) -> Path:
    return Path(dashboard_page.dated_directory(dashboard_page.TELEMETRY_DIR)).parent
