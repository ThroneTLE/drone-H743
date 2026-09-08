"""R-T1-6（TK-05）：录制的文件完整性、后台写入与失败恢复。

对应改版报告 N10–N13。每一条都先在 `tools/panel_qa/baseline_observations.py` 里作为
**基线观测**记录下修复前的事实（证据见
`data/analysis/tk_revamp/2026-09-04/baseline_2acfd82e/observations.json`），这里再写成
“正确行为”的断言。修复前跑这份文件应当是红的，那是它有意义的前提。

修复前的观测原文（HEAD `2acfd82e`）：

    n13_same_second_collision: same_path=true, data_rows_before=1, data_rows_after=0,
                               first_sample_survives=false       ← 已完成数据被销毁
    n12_schema_change:         row_widths=[3, 3, 4]              ← 行宽撑破表头
    n11_write_failure:         raised=OSError, handle_still_active=true, queued_rows_left=0
    n10_synchronous_flush:     200 行 × 2 ms 注入延迟 → 阻塞 0.504 s

慢盘那一项是**不利存储条件模拟**，0.5 s 不代表正常硬盘的写入速度；这里也只断言
“UI 不等它”，不断言任何绝对吞吐。
"""

from __future__ import annotations

import json
import subprocess
import sys
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
    RecordSchema,
    TelemetryRecorder,
    read_record_rows,
    unique_record_path,
)
from tools.panel_lib.telem_stream import TelemSchema


ROOT = Path(__file__).resolve().parents[1]

TWO_CHANNELS = RecordSchema(schema_hash=0x12345678, channel_names=("a", "b"))
THREE_CHANNELS = RecordSchema(schema_hash=0x0BADC0DE, channel_names=("a", "b", "c"))


def sample(t_us: int, values: dict[int, float]):
    return SimpleNamespace(t_us=t_us, values=values)


def settle(recorder) -> None:
    """等写线程把队列处理完。

    R-T1-6 返修后 start/stop 都是**异步**的（审核 R1：Tk 线程不许等写盘），所以
    测试要自己给一个显式栅栏。生产界面代码不许调 wait_idle。
    """
    assert recorder.wait_idle(5.0), "writer thread did not settle"


def stop_and_settle(recorder, reason=record_service.REASON_USER):
    recorder.stop(reason)
    settle(recorder)
    return recorder.status()


def start_and_settle(recorder, *args, **kwargs):
    recorder.start(*args, **kwargs)
    settle(recorder)
    return recorder.status()


@pytest.fixture
def recorder():
    instance = TelemetryRecorder()
    try:
        yield instance
    finally:
        instance.close()


def fixed_stem(_name: str = "telem_20260904_120000"):
    return _name


# ---------------------------------------------------------------- N13 文件碰撞


def test_a_second_recording_in_the_same_second_does_not_destroy_the_first(
    recorder, tmp_path,
) -> None:
    """P1。旧实现用 `telem_HHMMSS.csv` + `"w"`：同一秒再点一次，前一份被截断成 0 行。"""
    start_and_settle(recorder, tmp_path, TWO_CHANNELS, stem=fixed_stem())
    recorder.submit(sample(25000, {0: 123.0, 1: 456.0}))
    first = stop_and_settle(recorder).path

    start_and_settle(recorder, tmp_path, TWO_CHANNELS, stem=fixed_stem())
    second = stop_and_settle(recorder).path

    assert first != second, "同一秒的两次录制必须落到不同文件"
    assert read_record_rows(first) == [["0.025000", "123", "456"]]
    assert read_record_rows(second) == []


def test_recording_never_overwrites_a_file_that_is_already_there(tmp_path) -> None:
    """不只是"上一次录的"：目录里任何同名文件都不许被截断。"""
    victim = tmp_path / "telem_20260904_120000.csv"
    victim.write_text("已有的重要数据\n", encoding="utf-8")

    handle, path = unique_record_path(tmp_path, "telem_20260904_120000")
    handle.close()

    assert path != victim
    assert victim.read_text(encoding="utf-8") == "已有的重要数据\n"


def test_two_recorders_sharing_a_directory_get_separate_files(tmp_path) -> None:
    """两个面板实例对着同一个 data 目录：靠禁用按钮几百毫秒挡不住这个。"""
    first, second = TelemetryRecorder(), TelemetryRecorder()
    try:
        a = start_and_settle(first, tmp_path, TWO_CHANNELS, stem=fixed_stem()).path
        b = start_and_settle(second, tmp_path, TWO_CHANNELS, stem=fixed_stem()).path
        assert a != b
        assert a.exists() and b.exists()
    finally:
        first.close()
        second.close()


def test_concurrent_starts_never_collide(tmp_path) -> None:
    """排他创建必须在并发下也成立，否则"检查再打开"之间就有窗口。"""
    paths: list[Path] = []
    lock = threading.Lock()

    def claim() -> None:
        handle, path = unique_record_path(tmp_path, fixed_stem())
        handle.write("x\n")
        handle.close()
        with lock:
            paths.append(path)

    threads = [threading.Thread(target=claim) for _ in range(12)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert len(set(paths)) == 12
    assert all(path.read_text(encoding="utf-8") == "x\n" for path in paths)


# ---------------------------------------------------------------- N12 schema 契约


def test_a_schema_change_ends_the_session_instead_of_widening_the_rows(
    recorder, tmp_path,
) -> None:
    """旧实现：表头按开始时的表写死，行宽却读当前全局表，得到 [3, 3, 4]。"""
    start_and_settle(recorder, tmp_path, TWO_CHANNELS, stem=fixed_stem())
    recorder.submit(sample(0, {0: 1.0, 1: 2.0}))
    settle(recorder)

    recorder.note_schema(THREE_CHANNELS)
    settle(recorder)
    status = recorder.status()
    assert status.state == record_service.STATE_FINISHED
    assert status.end_reason == record_service.REASON_SCHEMA_CHANGED

    # 换表之后到达的样本不再进旧文件。
    recorder.submit(sample(25000, {0: 1.0, 1: 2.0, 2: 3.0}))
    settle(recorder)

    rows = read_record_rows(status.path)
    assert [len(row) for row in rows] == [3]
    assert "通道表变化" in status.describe()


def test_every_row_is_as_wide_as_its_own_header(recorder, tmp_path) -> None:
    start_and_settle(recorder, tmp_path, THREE_CHANNELS, stem=fixed_stem())
    for index in range(5):
        recorder.submit(sample(index * 25000, {0: float(index)}))
    status = stop_and_settle(recorder)

    lines = status.path.read_text(encoding="utf-8").splitlines()
    header = next(line for line in lines if not line.startswith("#"))
    width = len(header.split(","))
    assert width == 1 + THREE_CHANNELS.channel_count
    assert all(len(row) == width for row in read_record_rows(status.path))


def test_the_file_records_the_schema_hash_it_was_written_under(recorder, tmp_path) -> None:
    status = start_and_settle(recorder, tmp_path, TWO_CHANNELS, stem=fixed_stem())
    stop_and_settle(recorder)
    header = record_service.record_header(status.path)
    assert header[0] == f"# schema_hash={TWO_CHANNELS.schema_hash:08X}"


def test_queued_rows_are_written_with_their_own_sessions_width(
    recorder, tmp_path, monkeypatch,
) -> None:
    """迟到的行按**所属会话**解释，不按当前全局 schema。"""
    start_and_settle(recorder, tmp_path, TWO_CHANNELS, stem=fixed_stem())
    recorder.submit(sample(0, {0: 1.0, 1: 2.0}))
    recorder.note_schema(THREE_CHANNELS)
    settle(recorder)
    first = recorder.status()

    second = start_and_settle(
        recorder, tmp_path, THREE_CHANNELS, stem="telem_20260904_120001")
    recorder.submit(sample(25000, {0: 1.0, 1: 2.0, 2: 3.0}))
    stop_and_settle(recorder)

    assert [len(row) for row in read_record_rows(first.path)] == [3]
    assert [len(row) for row in read_record_rows(second.path)] == [4]


def test_a_new_connection_generation_ends_the_session(recorder, tmp_path) -> None:
    """重连之后是另一次连接，不能续到同一个文件里。"""
    start_and_settle(recorder, tmp_path, TWO_CHANNELS, generation=1, stem=fixed_stem())
    assert recorder.note_generation(1).active
    recorder.note_generation(2)
    settle(recorder)
    status = recorder.status()
    assert status.state == record_service.STATE_FINISHED
    assert status.end_reason == record_service.REASON_LINK_CHANGED
    assert "连接已更换" in status.describe()


def test_the_trailer_explains_how_the_file_ended(recorder, tmp_path) -> None:
    """“每个文件可解释”：光有数据行说明不了它是正常停的还是被动断的。"""
    start_and_settle(recorder, tmp_path, TWO_CHANNELS, stem=fixed_stem())
    recorder.submit(sample(0, {0: 1.0}))
    status = stop_and_settle(recorder)
    trailer = record_service.record_header(status.path)[-1]
    assert trailer.startswith("# end reason=user rows=1 dropped=0")


# ---------------------------------------------------------------- N11 失败恢复


class BrokenWriter:
    def __init__(self, error=OSError("simulated disk full")) -> None:
        self.error = error
        self.closed = False

    def write(self, _text: str) -> None:
        raise self.error

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


def install_writer(monkeypatch, writer, path: Path) -> None:
    monkeypatch.setattr(
        record_service, "unique_record_path", lambda *_a, **_k: (writer, path)
    )


def test_a_write_failure_ends_the_session_and_keeps_the_partial_file(
    recorder, tmp_path, monkeypatch,
) -> None:
    """旧实现：异常从渲染回调里冒出去，句柄还留着，之后每帧重复报错。"""
    class FailAfterHeader:
        def __init__(self) -> None:
            self.rows = 0
            self.closed = False
            self.handle = (tmp_path / "partial.csv").open("w", encoding="utf-8", newline="")

        def write(self, text: str) -> None:
            if text.startswith("#") or text.startswith("t_s"):
                self.handle.write(text)
                return
            self.rows += 1
            raise OSError("simulated disk full")

        def flush(self) -> None:
            self.handle.flush()

        def close(self) -> None:
            self.closed = True
            self.handle.close()

    writer = FailAfterHeader()
    install_writer(monkeypatch, writer, tmp_path / "partial.csv")

    start_and_settle(recorder, tmp_path, TWO_CHANNELS, stem=fixed_stem())
    for index in range(5):
        recorder.submit(sample(index, {0: 1.0}))
    settle(recorder)

    status = recorder.status()
    assert status.state == record_service.STATE_FAILED
    assert "simulated disk full" in (status.error or "")
    assert writer.closed, "失败必须释放句柄，不能留着让每帧继续撞"
    assert writer.rows == 1, "一次错误只报一次，不是每行都重试"
    assert status.path.exists(), "部分文件保留给用户，不静默删除"
    assert "录制失败" in status.describe()


def test_a_failed_recording_does_not_swallow_later_samples_into_the_same_file(
    recorder, tmp_path, monkeypatch,
) -> None:
    """失败之后 submit 直接不收；用户重试得到的是一个**新**会话。"""
    # 连表头都写不下去：也必须是一个明确的失败态，不是从按钮回调里冒出去的裸异常。
    install_writer(monkeypatch, BrokenWriter(), tmp_path / "broken.csv")
    status = start_and_settle(recorder, tmp_path, TWO_CHANNELS, stem=fixed_stem())
    assert status.state == record_service.STATE_FAILED
    assert "simulated disk full" in (status.error or "")
    assert recorder.submit(sample(1, {0: 2.0})) is False

    monkeypatch.undo()
    healthy = start_and_settle(
        recorder, tmp_path, TWO_CHANNELS, stem="telem_20260904_120002")
    recorder.submit(sample(2, {0: 3.0}))
    final = stop_and_settle(recorder)
    assert final.state == record_service.STATE_FINISHED
    assert healthy.path != (tmp_path / "broken.csv")
    assert read_record_rows(final.path) == [["0.000002", "3", ""]]


def test_a_directory_that_cannot_be_written_reports_instead_of_pretending(
    recorder, tmp_path, monkeypatch,
) -> None:
    def refuse(*_args, **_kwargs):
        raise PermissionError("read-only volume")

    monkeypatch.setattr(record_service, "unique_record_path", refuse)
    status = start_and_settle(recorder, tmp_path, TWO_CHANNELS, stem=fixed_stem())
    assert status.state == record_service.STATE_FAILED
    assert "read-only volume" in (status.error or "")
    assert not status.active, "失败的开始不能留下一个「录制中」的假象"


# ---------------------------------------------------------------- N10 背压


def test_a_slow_disk_does_not_block_the_submitting_thread(
    recorder, tmp_path, monkeypatch,
) -> None:
    """收线程提交 200 行不该等磁盘。基线上这一步是同步的，注入 2 ms/行 后阻塞 0.5 s。"""
    class SlowWriter:
        def __init__(self) -> None:
            self.writes = 0

        def write(self, _text: str) -> None:
            self.writes += 1
            time.sleep(0.002)

        def flush(self) -> None:
            pass

        def close(self) -> None:
            pass

    install_writer(monkeypatch, SlowWriter(), tmp_path / "slow.csv")
    start_and_settle(recorder, tmp_path, TWO_CHANNELS, stem=fixed_stem())

    started = time.perf_counter()
    for index in range(200):
        recorder.submit(sample(index * 25000, {0: float(index)}))
    elapsed = time.perf_counter() - started

    assert elapsed < 0.10, f"提交 200 行阻塞了 {elapsed * 1000:.0f} ms"
    recorder.stop()


def test_queue_overflow_is_counted_not_silently_dropped(tmp_path, monkeypatch) -> None:
    """有界队列是有意的；但"丢了多少"必须是一个数字，还要写进文件尾。"""
    class BlockedWriter:
        """开头写得下去（表头），之后一直堵着，直到测试放行。"""

        def __init__(self) -> None:
            self.release = threading.Event()
            self.written: list[str] = []

        def write(self, text: str) -> None:
            self.written.append(text)
            if not text.startswith("#") and not text.startswith("t_s"):
                self.release.wait(2.0)

        def flush(self) -> None:
            pass

        def close(self) -> None:
            pass

    writer = BlockedWriter()
    install_writer(monkeypatch, writer, tmp_path / "blocked.csv")
    instance = TelemetryRecorder(queue_limit=8)
    try:
        instance.start(tmp_path, TWO_CHANNELS, stem=fixed_stem())
        for index in range(200):
            instance.submit(sample(index, {0: float(index)}))
        assert instance.status().rows_dropped > 0
        assert "已丢" in instance.status().describe()
        writer.release.set()
        instance.stop()
        assert instance.wait_idle(5.0)
        status = instance.status()
    finally:
        instance.close()
    assert status.rows_dropped > 0
    # 丢弃必须落到文件里，事后拿到 CSV 的人才看得见。
    trailer = next(line for line in writer.written if line.startswith("# end reason="))
    assert f"dropped={status.rows_dropped}" in trailer


# ---------------------------------------------------------------- 真实面板端到端


@pytest.fixture(scope="module")
def _panel():
    try:
        instance = panel.DronePanel()
    except tk.TclError as exc:
        if not panel_qa.is_display_unavailable(exc):
            raise                                # 装置坏了就要红，不许伪装成无显示
        pytest.skip(f"Tk display unavailable: {exc}")       # pragma: no cover
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
    _panel.dashboard_recorder.close()
    _panel.dashboard_recorder = record_service.TelemetryRecorder()
    _panel.transport = panel_qa.MemoryTransport(connected=True)
    _panel.update_idletasks()
    try:
        yield _panel
    finally:
        _panel.dashboard_recorder.close()


def begin_record(app):
    """点"录制 CSV"并等文件真的建出来。

    返修后 `start()` 是异步的（审核 R1：按钮回调不许碰磁盘），所以页面级测试要自己
    等一个栅栏，再让页面把状态里的路径取回来——生产界面靠渲染拍做同一件事。
    """
    app._dashboard_toggle_record()
    settle(app.dashboard_recorder)
    app._dashboard_refresh_record_status()
    return app.dashboard_record_path


def load_schema(app) -> TelemSchema:
    for line in panel_qa.telemetry_schema_lines():
        app._handle_board_line(line)
    app.update_idletasks()
    return app.dashboard_schema


def test_the_real_page_records_through_the_service(app) -> None:
    schema = load_schema(app)
    app.transport.lines.clear()
    app.transport.frames.clear()

    path = begin_record(app)
    assert app.dashboard_recorder.status().active
    # 录制是纯本地动作，不许顺手改固件的流配置。
    assert app.transport.lines == []
    assert app.transport.frames == []

    app._dashboard_on_binary_frame(
        0x2230,
        panel_qa.telemetry_frame(
            {0: 1.0, 2: 3.0}, schema_hash=schema.computed_hash(), t_us=1000),
        transport=app.transport,
    )
    app._dashboard_stop_record()
    settle(app.dashboard_recorder)

    lines = path.read_text(encoding="utf-8").splitlines()
    assert lines[0] == f"# schema_hash={schema.computed_hash():08X}"
    assert lines[1].startswith("t_s,roll,pitch,yaw,")
    # 本帧没带的通道留空而不是补 0（旧语义原样保留）。
    assert lines[2] == "0.001000,1,,3,,,,,"
    assert len(lines[2].split(",")) == 1 + schema.channel_count


def tmp_path_for(app) -> Path:
    """`app` fixture 把录制目录钉在 tmp_path 上，这里把它取回来。"""
    return Path(dashboard_page.dated_directory(dashboard_page.TELEMETRY_DIR)).parent


def test_the_render_tick_never_waits_for_the_disk(app, monkeypatch) -> None:
    """N10 的现场就在这里：`_dashboard_render_tick` 里同步写完整条队列。"""
    load_schema(app)
    monkeypatch.setattr(app, "after", lambda *_a, **_k: None)

    class SlowWriter:
        def write(self, _text: str) -> None:
            time.sleep(0.002)

        def flush(self) -> None:
            pass

        def close(self) -> None:
            pass

    install_writer(monkeypatch, SlowWriter(), tmp_path_for(app) / "slow.csv")
    app._dashboard_toggle_record()
    for index in range(200):
        app.dashboard_recorder.submit(sample(index * 25000, {0: float(index)}))

    started = time.perf_counter()
    app._dashboard_render_tick()
    elapsed = time.perf_counter() - started
    assert elapsed < 0.05, f"渲染回调等了 {elapsed * 1000:.0f} ms"
    app._dashboard_stop_record()


def test_the_page_ends_the_recording_when_the_channel_table_changes(app) -> None:
    load_schema(app)
    path = begin_record(app)
    assert app.dashboard_recorder.status().active

    wider = list(panel_qa.DEFAULT_CHANNELS) + [
        ("fusion_acc_err", "-", -1.0, 1.0, "nav", "-")
    ]
    for line in panel_qa.telemetry_schema_lines(wider):
        app._handle_board_line(line)
    app.update_idletasks()
    settle(app.dashboard_recorder)
    app._dashboard_refresh_record_status()

    status = app.dashboard_recorder.status()
    assert status.state == record_service.STATE_FINISHED
    assert status.end_reason == record_service.REASON_SCHEMA_CHANGED
    assert all(len(row) == 1 + 8 for row in read_record_rows(path))
    assert "通道表变化" in app.dashboard_record_var.get()


def test_the_page_ends_the_recording_when_the_link_generation_changes(app) -> None:
    load_schema(app)
    begin_record(app)
    assert app.dashboard_recorder.status().active

    app.transport.bump_generation()
    app._dashboard_poll_tick(time.monotonic())
    settle(app.dashboard_recorder)

    status = app.dashboard_recorder.status()
    assert status.state == record_service.STATE_FINISHED
    assert status.end_reason == record_service.REASON_LINK_CHANGED


def test_two_recordings_in_the_same_second_through_the_real_page(app, monkeypatch) -> None:
    """P1 在真实页面上的现场：连点两次"录制 CSV"。"""
    schema = load_schema(app)
    monkeypatch.setattr(
        app.dashboard_recorder, "_now",
        lambda: __import__("datetime").datetime(2026, 9, 4, 12, 0, 0),
    )
    first = begin_record(app)
    app._dashboard_on_binary_frame(
        0x2230,
        panel_qa.telemetry_frame({0: 7.0}, schema_hash=schema.computed_hash(), t_us=1000),
        transport=app.transport,
    )
    app._dashboard_stop_record()
    settle(app.dashboard_recorder)

    second = begin_record(app)
    app._dashboard_stop_record()
    settle(app.dashboard_recorder)

    assert first != second
    assert read_record_rows(first) == [["0.001000", "7", "", "", "", "", "", "", ""]]


CLOSE_CHILD = """
import json, sys, tempfile
from pathlib import Path
from types import SimpleNamespace

from tools import panel_qa
from tools.panel_lib import record_service
from tools.panel_lib.pages import dashboard as dashboard_page

target = Path(sys.argv[1])
with panel_qa.isolated_environment(tempfile.mkdtemp()),         panel_qa.isolation.redirected_dated_directory(target):
    try:
        qa = panel_qa.OfflinePanel.launch(scale=1.0, size=(1080, 700))
    except BaseException as exc:
        if panel_qa.is_display_unavailable(exc):
            print("DISPLAY_UNAVAILABLE")
            raise SystemExit(0)
        raise
    panel = qa.panel
    for line in panel_qa.telemetry_schema_lines():
        panel._handle_board_line(line)
    panel.update_idletasks()

    panel._dashboard_toggle_record()
    assert panel.dashboard_recorder.wait_idle(5.0)
    panel._dashboard_refresh_record_status()
    path = panel.dashboard_record_path
    link = panel.dashboard_recorder._session.link
    accepted = panel.dashboard_recorder.submit(
        SimpleNamespace(t_us=1000, values={0: 5.0}),
        transport=link.transport, generation=link.generation,
    )
    # 刻意不在这里 settle：关窗必须自己把已接受的样本收干净。
    qa.destroy()
    print(json.dumps({"path": str(path), "accepted": accepted}))
"""


def test_closing_the_page_finishes_an_active_recording(tmp_path) -> None:
    """关窗时在录的会话必须收尾，最后一段数据不能烂在队列里。

    跑在**子进程**里：这个用例需要一个能被真正销毁的面板，而同一进程里再建一个 Tk
    root（本模块已经有一个 module 级面板）会间歇性地在 `Tk()` 处炸掉——2026-09-05
    的整合跑里就命中过一次。这不是产品缺陷，是装置层面的进程级限制；照
    `test_panel_qa_harness.py` 里高 DPI 矩阵那条的先例隔离，断言一条不少。
    """
    result = subprocess.run(
        [sys.executable, "-c", CLOSE_CHILD, str(tmp_path / "day")],
        cwd=ROOT, capture_output=True, text=True, timeout=180,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    if "DISPLAY_UNAVAILABLE" in result.stdout:
        pytest.skip("Tk display unavailable")    # pragma: no cover
    payload = json.loads(result.stdout.strip().splitlines()[-1])
    assert payload["accepted"] is True, "样本必须是在关窗之前被接受的"

    path = Path(payload["path"])
    assert read_record_rows(path) == [["0.001000", "5", "", "", "", "", "", "", ""]]
    assert any("end reason=" in line for line in record_service.record_header(path))


# ---------------------------------------------------------------- 边界保持


def test_recording_still_lands_under_the_project_data_root() -> None:
    """原契约保留：路径归页面（`data/` 根 + 按日目录），完整性归服务。"""
    source = (Path(__file__).resolve().parents[1] / "tools" / "panel_lib" / "pages"
              / "dashboard.py").read_text(encoding="utf-8")
    assert "project_paths import TELEMETRY_DIR" in source
    assert "dated_directory(TELEMETRY_DIR)" in source


def test_the_panel_entry_point_did_not_grow() -> None:
    """`drone_tcp_panel.py` 只减不增：录制服务不许在那里加第四处挂载。"""
    source = (Path(__file__).resolve().parents[1] / "tools"
              / "drone_tcp_panel.py").read_text(encoding="utf-8")
    assert source.count("_dashboard_") == 2
    assert "record_service" not in source
    assert "TelemetryRecorder" not in source
