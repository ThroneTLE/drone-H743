"""Single-reader export over the already-open serial session, using FLOG fixtures."""
import queue
import runpy
import threading
import time
from pathlib import Path

import pytest

from tools.panel_lib import transport as tr
from tools import flight_log_receive as flog


class Port:
    def __init__(self, stream=b"", chunk=4096):
        self.incoming = queue.Queue()
        self.buffer = bytearray()
        self.stream, self.chunk = stream, chunk
        self.is_open = True
        self.writes = []
        self.baudrate = 57600

    @property
    def in_waiting(self):
        return len(self.buffer)

    out_waiting = 0

    def read(self, size):
        if not self.buffer:
            try:
                self.buffer.extend(self.incoming.get(timeout=.02))
            except queue.Empty:
                return b""
        count = min(size, self.chunk, len(self.buffer))
        result = bytes(self.buffer[:count])
        del self.buffer[:count]
        return result

    def write(self, data):
        self.writes.append(data)
        if data in (b"FLOG DUMP\r\n", b"FLOGDUMP LAST\r\n"):
            self.incoming.put(self.stream)
        return len(data)

    def close(self):
        self.is_open = False


def connect(monkeypatch, port):
    opens = []
    def factory(*args, **kwargs):
        opens.append((args, kwargs))
        return port
    monkeypatch.setattr(tr.serial, "Serial", factory)
    link = tr.SerialTransport(queue.Queue())
    link.start("COM10", 57600)
    return link, opens


def test_current_connection_can_be_borrowed_without_reopen(monkeypatch):
    port = Port()
    link, opens = connect(monkeypatch, port)
    try:
        generation = link.connection_generation
        lease = link.claim_transfer()
        assert lease.port_name == "COM10" and lease.baudrate == 57600
        assert not link.send_line("PING")
        lease.close()
        assert link.send_line("PING")
        assert len(opens) == 1 and link.connection_generation == generation
        assert port.is_open
    finally:
        link.stop()


def make_stream():
    fixture = runpy.run_path(str(Path(__file__).with_name("test_flight_log_receive.py")))
    image = bytearray(b"\xff" * flog.SECTOR_SIZE)
    image[:flog.SECTOR_HEADER_SIZE] = fixture["make_sector_header"](
        version=9, record_size=flog.V9_RECORD_SIZE,
        params_struct=flog.V9_PARAMS_STRUCT, param_names=flog.V9_PARAM_NAMES)
    record = fixture["make_record"]()
    image[flog.SECTOR_HEADER_SIZE:flog.SECTOR_HEADER_SIZE + len(record)] = record
    image[-16:-8] = b"$X>\x00\nXYZ"
    begin = (f"FLOG BEGIN version=1 block_magic=0x31424C46 transport=usbcdc encoding=binary "
             f"total=4096 sectors=1 sector_size=4096 header_size=256 record_size={flog.V9_RECORD_SIZE} "
             "log_rate=125 baud=57600 session=123 payload=1024\r\n").encode()
    return bytes(image), begin + fixture["make_export_blocks"](bytes(image)) + b"FLOG END reason=done sent=4096 total=4096\r\n"


@pytest.mark.parametrize("chunk", [1, 7, 37, 4096])
def test_existing_decoder_receives_every_binary_byte(monkeypatch, tmp_path, chunk):
    image, stream = make_stream()
    port = Port(stream, chunk)
    link, opens = connect(monkeypatch, port)
    try:
        lease = link.claim_transfer()
        result = flog.receive_dump(lease, tmp_path)
        lease.close()
        assert result.complete and result.bin_path.read_bytes() == image
        assert len(opens) == 1 and port.is_open
    finally:
        link.stop()


def test_disconnect_wakes_reader_and_old_lease_cannot_send_to_new_session(monkeypatch):
    port = Port()
    link, _ = connect(monkeypatch, port)
    lease = link.claim_transfer()
    failed = threading.Event()
    def read():
        try:
            lease.read()
        except OSError:
            failed.set()
    worker = threading.Thread(target=read)
    worker.start()
    link.stop()
    assert failed.wait(.5)
    worker.join(1)
    replacement = Port()
    monkeypatch.setattr(tr.serial, "Serial", lambda *a, **k: replacement)
    link.start("COM31", 57600)
    try:
        with pytest.raises(OSError):
            lease.write(b"FLOG DUMP\r\n")
        lease.close()
        assert replacement.writes == []
    finally:
        link.stop()


def test_stop_command_preempts_transfer_without_closing_port(monkeypatch):
    port = Port()
    link, _ = connect(monkeypatch, port)
    try:
        lease = link.claim_transfer()
        assert link.send_line("SERVO JOG STOP")
        with pytest.raises(OSError, match="停止"):
            lease.read()
        with pytest.raises(OSError):
            lease.write(b"FLOG DUMP\r\n")
        deadline = time.monotonic() + 1
        while b"SERVO JOG STOP\r\n" not in port.writes and time.monotonic() < deadline:
            time.sleep(.01)
        lease.close()
        assert port.writes == [b"FLOG CANCEL\r\n", b"SERVO JOG STOP\r\n"]
        assert port.is_open
    finally:
        link.stop()


def test_buffer_overflow_and_duplicate_claim_are_explicit(monkeypatch):
    port = Port()
    link, _ = connect(monkeypatch, port)
    try:
        lease = link.claim_transfer()
        with pytest.raises(OSError):
            link.claim_transfer()
        lease.MAX_BUFFER_BYTES = 16
        assert lease.feed(b"x" * 17)
        with pytest.raises(OSError, match="超限"):
            lease.read()
        lease.close()
    finally:
        link.stop()


def test_normal_rx_and_tx_resume_without_replaying_blocked_commands(monkeypatch):
    from tools.panel_lib.connection_state import ReceivedMessage
    port = Port()
    link, _ = connect(monkeypatch, port)
    try:
        lease = link.claim_transfer()
        assert not link.send_line("PARAM SET forbidden 1")
        lease.close()
        port.incoming.put(b"OK normal-after-export\r\n")
        found = False
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            item = link.rx_queue.get(timeout=1)
            text = item.payload if isinstance(item, ReceivedMessage) else item
            if text == "OK normal-after-export":
                found = True
                break
        assert found and not port.writes
    finally:
        link.stop()


def test_uart_xorhex_and_corruption_use_existing_checks(monkeypatch, tmp_path):
    image, _ = make_stream()
    lines = []
    for seq, offset in enumerate(range(0, len(image), 48)):
        payload = image[offset:offset + 48]
        lines.append((f"FLOG BLK seq={seq} offset={offset} len={len(payload)} flags=0 "
                      f"crc={flog.crc32(payload):08X} data={flog.xor_whiten(payload, offset).hex()}\r\n").encode())
    # Corrupt one line just like the user's captured malformed UART block.
    lines[3] = b"FLOG BLK seq=3 offset=144 len=48 flags=0BAD\r\n"
    stream = (f"FLOG BEGIN version=1 block_magic=0x31424C46 transport=uart encoding=xorhex "
              f"total=4096 sectors=1 sector_size=4096 header_size=256 record_size={flog.V9_RECORD_SIZE} "
              "log_rate=125 baud=57600 session=123 payload=48\r\n").encode()
    port = Port(stream + b"".join(lines) + b"FLOG END reason=done sent=4096 total=4096\r\n", 13)
    link, _ = connect(monkeypatch, port)
    try:
        lease = link.claim_transfer()
        result = flog.receive_dump(lease, tmp_path)
        lease.close()
        assert not result.complete and result.missing_bytes == 48
        assert any("bad FLOG BLK" in error for error in result.errors)
        assert port.is_open
    finally:
        link.stop()
