"""Temporary raw RX/TX ownership of a live serial session; never opens a port.

The session's existing reader feeds this stream. Its existing sender performs
all writes. A lease is bound to that session, not a COM name that can reconnect.
"""
import threading
import time


STOP_COMMANDS = frozenset({b"DISARM", b"IDENT DISARM", b"IDENT STOP",
                           b"SERVO JOG STOP", b"ACCEPT STOP", b"ACCEPT ABORT"})


def is_stop_command(data):
    return data.strip().upper() in STOP_COMMANDS


class SerialTransfer:
    MAX_BUFFER_BYTES = 8 * 1024 * 1024

    def __init__(self, transport, session, failure_type=OSError):
        self.transport, self.session = transport, session
        self.failure_type = failure_type
        self.port_name = session.name
        self.baudrate = getattr(session.port, "baudrate", None)
        self.timeout = 2.0
        self.condition = threading.Condition()
        self.buffer = bytearray()
        self.offset = 0
        self.error = None
        self.closed = False
        self.pending = []

    def abort(self, reason):
        with self.condition:
            if self.error is None:
                self.error = str(reason)
            self.condition.notify_all()

    def feed(self, chunk):
        with self.condition:
            if self.closed:
                return False
            if self.error is None:
                if len(self.buffer) - self.offset + len(chunk) > self.MAX_BUFFER_BYTES:
                    self.error = "日志接收缓冲超限；本次导出中止，不丢弃后伪装完整"
                else:
                    if self.offset >= 65536:
                        del self.buffer[:self.offset]
                        self.offset = 0
                    self.buffer.extend(chunk)
            self.condition.notify_all()
            return True

    def read(self, size=1):
        if size <= 0:
            return b""
        deadline = time.monotonic() + self.timeout
        with self.condition:
            while True:
                if self.error:
                    raise self.failure_type(self.error)
                if self.closed or self.session.finished or self.session.transfer is not self:
                    raise self.failure_type("日志接收所用连接已断开或改变")
                available = len(self.buffer) - self.offset
                if available:
                    count = min(size, available)
                    result = bytes(self.buffer[self.offset:self.offset + count])
                    self.offset += count
                    return result
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return b""
                self.condition.wait(remaining)

    def readline(self):
        line = bytearray()
        while len(line) < 8192:
            byte = self.read(1)
            if not byte:
                return bytes(line)
            line.extend(byte)
            if byte == b"\n":
                return bytes(line)
        raise self.failure_type("日志文本行超限")

    def reset_input_buffer(self):
        # Reset only this consumer's memory, never the shared hardware RX buffer.
        with self.condition:
            self.buffer.clear()
            self.offset = 0

    def write(self, data):
        completed = threading.Event()
        if not self.transport._enqueue(bytes(data), owner=self, completed=completed):
            raise self.failure_type("日志发送所用连接已断开或改变")
        self.pending.append(completed)
        return len(data)

    def flush(self):
        for event in self.pending:
            if not event.wait(.75):
                raise self.failure_type("日志命令发送未在串口期限内完成")
        self.pending.clear()
        if self.session.finished:
            raise self.failure_type("日志发送期间连接断开")

    def close(self):
        with self.transport.lock:
            if self.session.transfer is self:
                self.session.transfer = None
            with self.condition:
                self.closed = True
                self.buffer.clear()
                self.offset = 0
                self.condition.notify_all()


def claim_transfer(transport, failure_type=OSError):
    with transport.lock:
        session = transport._session
        if session is None or session.finished:
            raise failure_type("请先在顶部连接飞控串口")
        if session.transfer is not None:
            raise failure_type("当前串口已有数据传输任务")
        lease = SerialTransfer(transport, session, failure_type)
        session.transfer = lease
        return lease
