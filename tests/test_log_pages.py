"""Existing log decoders/analysis through the new embedded Tk workspace."""
from pathlib import Path
from types import SimpleNamespace
import runpy
import threading
import time
import tkinter as tk

import pytest

from tools.panel_lib.pages.logs import LogsPage
from tools.panel_lib.log_receive_view import ReceiverView, receive


@pytest.fixture(scope="module")
def workspace():
    root = tk.Tk()
    root.withdraw()
    link = SimpleNamespace(is_connected=False, active_port=None, port=None)
    owner = SimpleNamespace(transport=link, serial_transport=link,
                            _transport_connected=lambda: link.is_connected,
                            _link_keepalive_suppressed_reason=lambda: None,
                            auto_connect_var=SimpleNamespace(get=lambda: True))
    page = LogsPage(root, owner)
    page.pack(fill="both", expand=True)
    yield root, page
    root.destroy()


def wait(root, predicate):
    until = time.monotonic() + 15
    while not predicate() and time.monotonic() < until:
        root.update()
        time.sleep(.01)
    assert predicate()


def test_import_waveform_and_analysis_reuse_existing_tools(workspace, tmp_path):
    root, page = workspace
    # Existing host log fixture; this checks UI plumbing, not algorithm acceptance.
    fixture = runpy.run_path(str(Path(__file__).with_name("test_flight_log_sysid.py")))
    path = fixture["write_sample_log"](tmp_path)
    original = path.read_bytes()
    assert [page.tabs.tab(tab, "text") for tab in page.tabs.tabs()] == [
        "接收与导入", "波形与回放", "离线分析"]
    page.import_path(path)
    wait(root, lambda: page.current_csv is not None)
    page.open_current(1)
    wait(root, lambda: page.waveform.frame is not None)
    assert len(page.waveform.segments) > 0
    page.open_current(2)
    wait(root, lambda: page.analysis.analysis is not None)
    assert page.analysis.analysis.row_count == len(page.waveform.frame)
    assert path.read_bytes() == original


def test_failed_import_cannot_reuse_previous_file(workspace, tmp_path):
    root, page = workspace
    page.import_path(tmp_path / "missing.csv")
    wait(root, lambda: "导入失败" in page.status.get())
    assert page.current_csv is None
    page.open_current(1)
    assert page.tabs.index(page.tabs.select()) == 0


def test_bin_import_reuses_decoder_and_never_overwrites(workspace, tmp_path, monkeypatch):
    root, page = workspace
    from tools.panel_lib.pages import logs
    fixture = runpy.run_path(str(Path(__file__).with_name("test_flight_log_receive.py")))
    image = bytearray(b"\xff" * receive.SECTOR_SIZE)
    image[:receive.SECTOR_HEADER_SIZE] = fixture["make_sector_header"](
        version=9, record_size=receive.V9_RECORD_SIZE,
        params_struct=receive.V9_PARAMS_STRUCT, param_names=receive.V9_PARAM_NAMES)
    record = fixture["make_record"]()
    image[receive.SECTOR_HEADER_SIZE:receive.SECTOR_HEADER_SIZE + len(record)] = record
    path = tmp_path / "flightlog.bin"
    path.write_bytes(image)
    monkeypatch.setattr(logs, "FLIGHT_LOG_DIR", tmp_path / "converted")
    page.import_path(path)
    wait(root, lambda: page.current_csv is not None)
    first = page.current_csv
    assert first.exists() and (first.parent / "import.json").exists()
    page.import_path(path)
    wait(root, lambda: page.current_csv is not None)
    assert first != page.current_csv and first.exists()
    assert path.read_bytes() == image


def test_closed_jobs_never_invoke_callback(workspace):
    root, _ = workspace
    from tools.panel_lib.log_jobs import LogJobs
    frame = tk.Frame(root)
    results = []
    jobs = LogJobs(frame, results.append)
    jobs.events.put(("closed", 1, results.append, (True, "bad")))
    jobs.versions["closed"] = 1
    frame.destroy()
    jobs.poll()
    assert jobs.closed and results == []


def test_dialog_cancel_does_not_write_or_change_selection(workspace, monkeypatch):
    _, page = workspace
    from tools.panel_lib.pages import logs
    before = page.current_csv
    monkeypatch.setattr(logs.filedialog, "askopenfilename", lambda **_: "")
    page.import_file()
    assert page.current_csv == before


def test_receiving_reuses_current_serial_and_preserves_auto_reconnect(workspace, tmp_path, monkeypatch):
    root, page = workspace
    fixture = runpy.run_path(str(Path(__file__).with_name("test_shared_log_transfer.py")))
    _, stream = fixture["make_stream"]()
    port = fixture["Port"](stream)
    port.baudrate = 115200
    link, opens = fixture["connect"](monkeypatch, port)
    previous = page.panel.transport
    try:
        page.panel.transport = page.panel.serial_transport = link
        page.receiver.dir_var.set(str(tmp_path))
        generation = link.connection_generation
        page.receiver._start_receive()
        assert page.receiver.busy()
        assert not page.allow_main_connect()
        wait(root, lambda: not page.receiver.busy())
        page.receiver._poll_events()
        assert "接收完成" in page.receiver.progress_text.get()
        assert page.current_csv.exists()
        assert link.connection_generation == generation and len(opens) == 1
        assert page.panel.auto_connect_var.get() is True
        assert not hasattr(page.receiver, "port_combo")
        assert not hasattr(page.receiver, "baud_var")
        assert page.allow_main_connect() and port.is_open
    finally:
        link.stop()
        page.panel.transport = page.panel.serial_transport = previous


def test_partial_receive_is_not_silently_loaded(workspace, tmp_path):
    _, page = workspace
    page.current_csv = None
    page.received(SimpleNamespace(complete=False, missing_bytes=32, csv_path=tmp_path / "partial.csv"))
    assert page.current_csv is None
    assert "不完整" in page.status.get()


def test_cancel_keeps_current_port_open_and_reports_partial_data(workspace, tmp_path, monkeypatch):
    root, page = workspace
    fixture = runpy.run_path(str(Path(__file__).with_name("test_shared_log_transfer.py")))
    _, stream = fixture["make_stream"]()
    port = fixture["Port"](stream[:len(stream) // 2])
    link, opens = fixture["connect"](monkeypatch, port)
    previous = page.panel.transport
    try:
        page.panel.transport = page.panel.serial_transport = link
        page.receiver.dir_var.set(str(tmp_path))
        page.current_csv = None
        page.receiver._start_receive()
        wait(root, lambda: page.receiver.latest_progress[0] > 0)
        page.receiver._cancel()
        wait(root, lambda: not page.receiver.busy())
        page.receiver._poll_events()
        assert "已取消" in page.receiver.progress_text.get()
        assert page.current_csv is None
        assert port.is_open and len(opens) == 1 and not link.transfer_active
        assert b"FLOG CANCEL\r\n" in port.writes
    finally:
        link.stop()
        page.panel.transport = page.panel.serial_transport = previous


def test_superseded_jobs_and_closed_views_drop_results(workspace):
    root, page = workspace
    release = threading.Event()
    finished = threading.Event()
    results = []
    page.jobs.submit("race", lambda: (release.wait(5), finished.set(), "old")[2], results.append)
    page.jobs.submit("race", lambda: "new", results.append)
    wait(root, lambda: results == ["new"])
    release.set()
    assert finished.wait(5)
    page.jobs.poll()
    assert results == ["new"]
