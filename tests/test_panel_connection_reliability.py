"""H1/H2/H4: real transport and Tk boundaries with no physical device access."""

import queue
import re
import threading
import time
import tkinter as tk
from pathlib import Path

import pytest

from tools import drone_tcp_panel as panel
from tools.panel_lib import transport as tr
from tools.panel_lib.connection_state import ReceivedMessage
from tools.panel_lib.proto import PROTO_DIR_FROM_FC, PROTO_MSG_TEXT_LINE


class Port:
    def __init__(self, *, slow_write=False, pending=False, read_error=False):
        self.is_open = True
        self.entered = threading.Event()
        self.release = threading.Event()
        self.closed = threading.Event()
        self.slow_write = slow_write
        self.pending = pending
        self.read_error = read_error
        self.writes = []
        self.close_count = 0
        self.flush_count = 0

    @property
    def in_waiting(self):
        if self.read_error:
            raise OSError(5, "simulated unplug")
        return 0

    @property
    def out_waiting(self):
        return int(self.pending and not self.release.is_set())

    def read(self, size):
        self.closed.wait(.01)
        return b""

    def write(self, data):
        self.entered.set()
        if self.slow_write:
            self.release.wait(2)
        self.writes.append(data)
        return len(data)

    def flush(self):
        self.flush_count += 1
        if self.pending:
            self.release.wait(2)

    def cancel_write(self):
        self.release.set()

    def cancel_read(self):
        self.closed.set()

    def close(self):
        self.close_count += 1
        self.is_open = False
        self.closed.set()


def connect(monkeypatch, port):
    monkeypatch.setattr(tr.serial, "Serial", lambda *a, **k: port)
    transport = tr.SerialTransport(queue.Queue())
    transport.start("FAKE", 115200)
    return transport


@pytest.mark.parametrize("phase", ["write", "flush"])
def test_slow_io_does_not_block_ui_queries_cancel_or_stop(monkeypatch, phase):
    port = Port(slow_write=phase == "write", pending=phase == "flush")
    transport = connect(monkeypatch, port)
    timer = None
    try:
        assert transport.send_line("PING")
        assert port.entered.wait(1)
        # Bound baseline failure too: old code holds the UI lock until release.
        timer = threading.Timer(.35, port.release.set)
        timer.start()
        start = time.perf_counter()
        assert transport.is_connected
        assert transport.active_port == "FAKE"
        transport.cancel_pending_sends()
        transport.stop()
        elapsed = time.perf_counter() - start
        assert elapsed < .1, f"UI blocked for {elapsed:.3f}s during {phase}"
        assert port.closed.wait(1)
    finally:
        port.release.set()
        transport.stop()
        if timer is not None:
            timer.join()


def test_read_failure_closes_port_and_preserves_reason(monkeypatch):
    port = Port(read_error=True)
    transport = connect(monkeypatch, port)
    try:
        transport.thread.join(1)
        assert port.closed.wait(.1), "reader exited without closing its port"
        assert port.close_count == 1
        assert not transport.is_connected
        messages = "\n".join(map(str, list(transport.rx_queue.queue)))
        assert "simulated unplug" in messages
        assert "errno=5" in messages and "generation=" in messages
        assert "last_rx=" in messages and "last_tx=" in messages
        transport._sender_thread.join(1)
        assert not transport._sender_thread.is_alive()
    finally:
        transport.stop()


@pytest.fixture(scope="module")
def _window():
    try:
        instance = panel.DronePanel()
    except tk.TclError as exc:
        pytest.skip(str(exc))
    instance.withdraw()
    for ident in instance.tk.call("after", "info"):
        instance.tk.call("after", "cancel", ident)
    instance.after = lambda *args: None
    yield instance
    instance.destroy()


@pytest.fixture
def app(_window):
    instance = _window
    instance.serial_transport = tr.SerialTransport(instance.rx_queue)
    instance.transport = instance.serial_transport
    instance.serial_transport.port = Port()
    instance.serial_transport._connection_generation = 1
    instance.validation_latest_values = {}
    instance.validation_latest_host_time = 0.0
    instance.validation_latest_transport_generation = None
    instance.validation_latest_sequence = None
    instance.validation_latest_timestamp_ms = None
    instance._snapshot_receipt = None
    instance._snapshot_fragment_sequence = None
    instance._snapshot_live = False
    return instance


def imu_lines(seq=50000):
    source = (Path(__file__).resolve().parents[1] / "App/Src/app_control.c").read_text(encoding="utf-8")
    formats = re.findall(r'"(IMU sample (?:valid=1|seq=%lu)[^"]*)"', source)
    assert len(formats) == 2
    values = [("legacy_intermediate", 1, 0, 255, seq, seq, 1, 0, 1000, 1000, 2500),
              (seq, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2)]
    return tuple(re.sub(r"%([0-9]*)l?u", r"%\1d", fmt).replace("%ld", "%d")
                 .replace("%02lX", "%02X").replace(r"\r\n", "\r\n") % args
                 for fmt, args in zip(formats, values))


def test_backlog_keeps_receive_time_and_fails_existing_safety_gate(app, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(tr.time, "monotonic", lambda: clock[0])
    app.serial_transport._consume_buffer(bytearray("".join(imu_lines()).encode()))
    clock[0] += 2.2
    app._drain_rx()
    assert app.validation_latest_host_time == 100.0
    assert app._firmware_safety_advisory()[0] != "ok"
    assert not app._validation_latest_is_safe()[0]


def test_old_queue_cannot_be_relabelled_as_new_connection(app):
    app.serial_transport._consume_buffer(bytearray("".join(imu_lines()).encode()))
    app.serial_transport._connection_generation = 2
    app._drain_rx()
    assert app.validation_latest_transport_generation != 2
    assert app._firmware_safety_advisory()[0] != "ok"


def test_split_snapshot_retains_earliest_fragment_time(app, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(tr.time, "monotonic", lambda: clock[0])
    head, body = imu_lines()
    app.serial_transport._consume_buffer(bytearray(head.encode()))
    app._drain_rx()
    clock[0] += 2.2
    app.serial_transport._consume_buffer(bytearray(body.encode()))
    app._drain_rx()
    assert app.validation_latest_host_time == 100.0
    assert app._firmware_safety_advisory()[0] != "ok"


def test_same_sequence_fragments_from_different_sessions_do_not_merge(app):
    head, body = imu_lines()
    app.serial_transport._consume_buffer(bytearray(head.encode()))
    app._drain_rx()
    app.serial_transport._connection_generation = 2
    app.serial_transport._consume_buffer(bytearray(body.encode()))
    app._drain_rx()
    assert app.validation_latest_host_time == 0.0
    assert app._firmware_safety_advisory()[0] != "ok"


@pytest.mark.parametrize("phase", ["write", "drain"])
def test_send_deadline_detaches_and_cancels_the_session(monkeypatch, phase):
    port = Port(slow_write=phase == "write", pending=phase == "drain")
    transport = connect(monkeypatch, port)
    try:
        assert transport.send_line("PING")
        assert port.entered.wait(1)
        assert port.closed.wait(1.2)
        assert not transport.is_connected
        assert "deadline" in transport.last_disconnect.phase
        assert port.flush_count == 0
        transport._sender_thread.join(1)
        assert not transport._sender_thread.is_alive()
    finally:
        port.release.set()
        transport.stop()


def test_late_old_sender_cannot_consume_new_queue_or_disconnect_new_port(monkeypatch):
    old = Port(slow_write=True)
    old.cancel_write = lambda: None  # Simulate a driver completing after reconnect.
    transport = connect(monkeypatch, old)
    new = Port()
    try:
        assert transport.send_line("PING")
        assert old.entered.wait(1)
        assert transport.send_line("SERVOCAL COMMIT")
        old_sender = transport._sender_thread
        old_generation = transport.connection_generation
        monkeypatch.setattr(tr.serial, "Serial", lambda *a, **k: new)
        transport.start("FAKE-NEW", 115200)
        new_generation = transport.connection_generation
        assert new_generation > old_generation
        assert transport.send_line("IMU?")
        assert new.entered.wait(1)
        old.release.set()
        old_sender.join(1)
        assert not old_sender.is_alive()
        assert transport.is_connected and transport.active_port == "FAKE-NEW"
        assert transport.connection_generation == new_generation
        assert old.writes == [b"PING\r\n"]
        assert new.writes == [b"IMU?\r\n"]
    finally:
        old.release.set()
        transport.stop()


def test_slow_close_never_blocks_stop_or_state_queries(monkeypatch):
    port = Port()
    close_entered, close_release = threading.Event(), threading.Event()
    original_close = port.close

    def close():
        close_entered.set()
        close_release.wait(2)
        original_close()

    port.close = close
    transport = connect(monkeypatch, port)
    try:
        start = time.perf_counter()
        transport.stop()
        assert not transport.is_connected
        assert transport.active_port is None
        assert time.perf_counter() - start < .1
        assert close_entered.wait(1)
    finally:
        close_release.set()


def test_reopen_waits_off_ui_for_previous_handle_to_close(monkeypatch):
    old, new = Port(), Port()
    close_release, opened = threading.Event(), threading.Event()
    original_close = old.close

    def close():
        close_release.wait(2)
        original_close()

    old.close = close
    transport = connect(monkeypatch, old)

    def reopen(*args, **kwargs):
        assert not old.is_open, "reopen raced the still-owned serial handle"
        opened.set()
        return new

    monkeypatch.setattr(tr.serial, "Serial", reopen)
    try:
        start = time.perf_counter()
        transport.start("FAKE", 115200)
        assert time.perf_counter() - start < .1
        assert not opened.is_set()
        close_release.set()
        assert opened.wait(1), "reconnect failed instead of waiting for close"
    finally:
        close_release.set()
        transport.stop()


def test_stop_cancels_a_reopen_waiting_for_old_handle(monkeypatch):
    old = Port()
    close_release = threading.Event()
    original_close = old.close

    def close():
        close_release.wait(2)
        original_close()

    old.close = close
    transport = connect(monkeypatch, old)
    opens = []
    monkeypatch.setattr(tr.serial, "Serial", lambda *a, **k: opens.append(True) or Port())
    try:
        transport.start("FAKE", 115200)
        worker = transport._opener_thread
        transport.stop()
        close_release.set()
        worker.join(1)
        assert not worker.is_alive()
        assert not opens and not transport.is_connected
    finally:
        close_release.set()
        transport.stop()


def test_late_old_reader_cannot_publish_into_reconnected_session(monkeypatch):
    entered, release = threading.Event(), threading.Event()

    class LatePort(Port):
        def read(self, size):
            entered.set()
            release.wait(2)
            return "".join(imu_lines()).encode()

    old = LatePort()
    transport = connect(monkeypatch, old)
    assert entered.wait(1)
    reader = transport.thread
    new = Port()
    monkeypatch.setattr(tr.serial, "Serial", lambda *a, **k: new)
    try:
        transport.start("FAKE-NEW", 115200)
        if transport._opener_thread is not None:
            transport._opener_thread.join(1)
        generation = transport.connection_generation
        release.set()
        reader.join(1)
        assert not reader.is_alive()
        assert transport.is_connected and transport.connection_generation == generation
        assert not any("IMU sample" in str(message) for message in transport.rx_queue.queue)
        assert old.close_count == 1
    finally:
        release.set()
        transport.stop()


def test_old_connection_notifications_do_not_probe_or_reset_new_session(app, monkeypatch):
    old = Port()
    transport = connect(monkeypatch, old)
    app.serial_transport = app.transport = transport
    app.rx_queue = transport.rx_queue
    try:
        transport.stop()
        transport._connection_generation += 1
        transport.port = Port()  # New connection already probed before old events drain.
        app.structured_protocol_supported = True
        probes = []
        monkeypatch.setattr(app, "_begin_protocol_probe", lambda: probes.append(True))
        app._drain_rx()
        assert not probes
        assert app.structured_protocol_supported is True
    finally:
        transport.stop()


@pytest.mark.parametrize("phase", ["write", "drain", "short"])
def test_failed_send_retains_reason_and_closes_exactly_once(monkeypatch, phase):
    class FaultPort(Port):
        def write(self, data):
            if phase == "write":
                raise OSError(5, "write failed")
            return len(data) - int(phase == "short")

        @property
        def out_waiting(self):
            raise OSError(6, "drain failed")

    port = FaultPort()
    transport = connect(monkeypatch, port)
    try:
        assert transport.send_line("PING")
        assert port.closed.wait(1)
        transport.thread.join(1)
        transport._sender_thread.join(1)
        assert port.close_count == 1
        assert not transport.is_connected
        assert phase in transport.last_disconnect.reason
    finally:
        transport.stop()


def test_binary_text_envelope_keeps_its_receive_time(app, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(tr.time, "monotonic", lambda: clock[0])
    for line in imu_lines():
        frame = tr.build_proto_frame(PROTO_DIR_FROM_FC, PROTO_MSG_TEXT_LINE, line.encode())
        app.serial_transport._consume_buffer(bytearray(frame))
    queued = list(app.rx_queue.queue)
    assert all(isinstance(item, ReceivedMessage) for item in queued)
    assert all(item.context.received_at == 100.0 for item in queued)
    clock[0] += 2.2
    app._drain_rx()
    assert app.validation_latest_host_time == 100.0
    assert app._firmware_safety_advisory()[0] != "ok"


def test_completed_snapshot_immediately_invalidates_on_disconnect(app):
    app.serial_transport._consume_buffer(bytearray("".join(imu_lines()).encode()))
    app._drain_rx()
    assert app._firmware_safety_advisory()[0] == "ok"
    app.serial_transport.port = None
    assert app._firmware_safety_advisory()[0] != "ok"
    assert not app._validation_latest_is_safe()[0]
    app._validation_refresh_readiness()
    assert str(app.validation_begin_button["state"]) == "disabled"


def test_new_partial_snapshot_cannot_borrow_previous_sample_freshness(app):
    app.serial_transport._consume_buffer(bytearray("".join(imu_lines()).encode()))
    app._drain_rx()
    assert app._firmware_safety_advisory()[0] == "ok"
    app.serial_transport._consume_buffer(bytearray(imu_lines(50001)[0].encode()))
    app._drain_rx()
    assert app.validation_latest_host_time == 0
    assert app._firmware_safety_advisory()[0] != "ok"


def test_duplicate_snapshot_never_refreshes_receive_age(app, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(tr.time, "monotonic", lambda: clock[0])
    for _ in range(2):
        app.serial_transport._consume_buffer(bytearray("".join(imu_lines()).encode()))
        app._drain_rx()
        clock[0] += 2.2
    assert app.validation_latest_host_time == 100.0
    assert app._firmware_safety_advisory()[0] != "ok"


@pytest.mark.parametrize("transport_class", [tr.TcpTransport, tr.UdpTransport])
def test_foreign_transport_queue_cannot_become_serial_snapshot(app, transport_class):
    other = transport_class(app.rx_queue)
    other._consume_buffer(bytearray("".join(imu_lines()).encode()))
    app._drain_rx()
    assert app.validation_latest_host_time == 0
    assert app._firmware_safety_advisory()[0] != "ok"
