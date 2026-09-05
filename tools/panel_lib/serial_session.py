"""Cancelable serial sessions; slow driver calls never hold the UI state lock.

Each open owns its port, stop event, TX queue, reader and sender until cleanup.
Deadline/cancel detaches state immediately; driver cancellation/close run off Tk.
No action is retried or replayed after disconnect. Tests use fake port objects.
"""

from dataclasses import dataclass, field
import queue
import threading
import time

from .connection_state import ReceivedMessage, receive_context


WRITE_DEADLINE_S = 0.5


@dataclass
class SerialSession:
    port: object
    name: str
    generation: int
    stop: threading.Event = field(default_factory=threading.Event)
    sends: queue.Queue = field(default_factory=queue.Queue)
    last_rx: float | None = None
    last_tx: float | None = None
    writing: bool = False
    finished: bool = False
    close_done: threading.Event = field(default_factory=threading.Event)
    close_error: str | None = None


@dataclass(frozen=True)
class DisconnectInfo:
    port: str
    generation: int
    phase: str
    reason: str
    last_rx: float | None
    last_tx: float | None
    occurred_at: float

    def __str__(self):
        return (f"[上位机] 串口已断开: port={self.port} generation={self.generation} "
                f"phase={self.phase} reason={self.reason} "
                f"last_rx={self.last_rx} last_tx={self.last_tx} at={self.occurred_at:.6f}")


def exception_reason(exc):
    return (f"{type(exc).__name__}: {exc} errno={getattr(exc, 'errno', None)} "
            f"winerror={getattr(exc, 'winerror', None)}")


class SerialSessionMixin:
    _stamp_received = True

    def __init__(self, rx_queue):
        super().__init__(rx_queue)
        self.lock = threading.Lock()
        self.port = None
        self.thread = self._sender_thread = None
        self._session = None
        self._send_queue = queue.Queue()
        self._send_generation = 0
        self._connection_generation = 0
        self._active_port = None
        self.stop_event = threading.Event()
        self.last_disconnect = None
        self._closing_session = None
        self._open_generation = 0
        self._opener_thread = None

    @property
    def is_connected(self):
        with self.lock:
            return self.port is not None

    @property
    def active_port(self):
        with self.lock:
            return self._active_port

    @property
    def connection_generation(self):
        with self.lock:
            return self._connection_generation

    def start(self, port_name, baudrate):
        from . import transport as wire
        if not wire.HAS_PYSERIAL or wire.serial is None:
            self.rx_queue.put(f"[上位机] 串口模式不可用: {wire.PYSERIAL_ERROR or '未安装 pyserial'}")
            return
        self.stop()
        with self.lock:
            token = self._open_generation
            closing = self._closing_session
        if closing is not None and not closing.close_done.is_set():
            self.rx_queue.put(f"[上位机] 等待旧串口释放后连接: {port_name}")
            self._opener_thread = threading.Thread(
                target=self._open_after_close, args=(closing, token, port_name, baudrate), daemon=True)
            self._opener_thread.start()
        else:
            self._open_port(token, port_name, baudrate)

    def _open_after_close(self, closing, token, port_name, baudrate):
        deadline = time.monotonic() + 2.0
        while not closing.close_done.wait(.05):
            with self.lock:
                if token != self._open_generation:
                    return
            if time.monotonic() >= deadline:
                self.rx_queue.put(f"[上位机] 串口重连失败: port={port_name} 旧端口释放超时")
                return
        if closing.close_error is not None:
            self.rx_queue.put(f"[上位机] 串口重连失败: port={port_name} {closing.close_error}")
            return
        self._open_port(token, port_name, baudrate)

    def _open_port(self, token, port_name, baudrate):
        from . import transport as wire
        with self.lock:
            if token != self._open_generation:
                return
        try:
            opened = wire.serial.Serial(port_name, baudrate=baudrate, timeout=.2,
                                        write_timeout=WRITE_DEADLINE_S)
        except Exception as exc:
            self.rx_queue.put(f"[上位机] 打开串口失败: port={port_name} {exception_reason(exc)}")
            return
        with self.lock:
            cancelled = token != self._open_generation
            if not cancelled:
                self._connection_generation += 1
                self._send_generation += 1
            session = SerialSession(opened, str(port_name), self._connection_generation)
            if not cancelled:
                self._session = session
                self.port = opened
                self._active_port = session.name
                self._send_queue = session.sends
                self.stop_event = session.stop
        if cancelled:
            self._finish_session(session, "open cancelled", "connection request cancelled")
            return
        self.rx_queue.put(ReceivedMessage(
            f"[上位机] 串口已连接: {port_name} @ {baudrate} generation={session.generation}",
            receive_context(self, generation=session.generation), diagnostic=True))
        self._sender_thread = threading.Thread(target=self._sender_loop, args=(session,), daemon=True)
        self.thread = threading.Thread(target=self._read_loop, args=(session,), daemon=True)
        self._sender_thread.start()
        self.thread.start()

    def stop(self):
        with self.lock:
            self._open_generation += 1
            session = self._session
        if session is not None:
            self._finish_session(session, "stop", "user requested stop")
        else:
            self.stop_event.set()
            self._send_queue.put(None)

    def _finish_session(self, session, phase, reason):
        with self.lock:
            if session.finished:
                return
            session.finished = True
            session.stop.set()
            session.sends.put(None)
            info = DisconnectInfo(session.name, session.generation, phase, reason,
                                  session.last_rx, session.last_tx, time.monotonic())
            if self._session is session:
                self.port = self._active_port = self._session = None
                self._connection_generation += 1
                self._send_generation += 1
                self.last_disconnect = info
                self._closing_session = session
        self.rx_queue.put(ReceivedMessage(str(info), receive_context(
            self, generation=session.generation), diagnostic=True))
        threading.Thread(target=self._close_session, args=(session,), daemon=True).start()

    def _close_session(self, session):
        for method in ("cancel_read", "cancel_write", "close"):
            try:
                action = getattr(session.port, method, None)
                if action is not None:
                    action()
            except Exception as exc:
                if method == "close":
                    session.close_error = exception_reason(exc)
                self.rx_queue.put(
                    f"[上位机] 串口收尾失败: port={session.name} generation={session.generation} "
                    f"phase={method} {exception_reason(exc)}")
        session.close_done.set()

    def send_frame(self, function, payload=b""):
        from . import transport as wire
        if wire.SERIAL_ASCII_COMPAT_MODE:
            try:
                text = payload.decode("utf-8") if payload else ""
            except UnicodeDecodeError:
                text = ""
            if text:
                return self.send_line(text)
        return self._enqueue(wire.build_proto_frame(wire.PROTO_DIR_TO_FC, function, payload))

    def send_line(self, line):
        return self._enqueue((line.rstrip("\r\n") + "\r\n").encode("utf-8"))

    def _enqueue(self, data):
        with self.lock:
            if self._session is None or self._session.finished:
                return False
            self._session.sends.put_nowait((self._send_generation, data))
        return True

    def cancel_pending_sends(self):
        with self.lock:
            self._send_generation += 1
            session = self._session
            pending = self._send_queue
            writing = session is not None and session.writing
            try:
                while True:
                    pending.get_nowait()
            except queue.Empty:
                pass
        if writing:
            # A partial write cannot be retracted. End this session rather than
            # let its pending bytes cross into a newly declared read-only phase.
            self._finish_session(session, "cancel", "in-flight send cancelled")

    def _sender_loop(self, session):
        from . import transport as wire
        while not session.stop.is_set():
            try:
                item = session.sends.get(timeout=.3)
            except queue.Empty:
                continue
            if item is None:
                break
            generation, frame = item
            with self.lock:
                if generation != self._send_generation or self._session is not session or session.finished:
                    continue
                session.writing = True
            phase = "write"
            timer = threading.Timer(WRITE_DEADLINE_S, self._finish_session, args=(
                session, "write/drain deadline", "TimeoutError: serial send exceeded 0.5s"))
            timer.daemon = True
            timer.start()
            try:
                if session.stop.is_set():
                    break
                written = session.port.write(frame)
                if written != len(frame):
                    raise OSError(f"short write: {written}/{len(frame)} bytes")
                phase = "drain"
                # pyserial.flush() has an unbounded Windows out_waiting loop.
                # Poll it with a cancellable wait and the same write deadline.
                while not session.stop.is_set() and session.port.out_waiting:
                    session.stop.wait(.01)
                if not session.stop.is_set():
                    session.last_tx = time.monotonic()
                    if wire.SERIAL_TX_DEBUG_ENABLED:
                        shown = frame.decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                        self.rx_queue.put(f"[host] serial tx bytes={written} data={shown}")
            except Exception as exc:
                self._finish_session(session, phase, exception_reason(exc))
            finally:
                timer.cancel()
                with self.lock:
                    session.writing = False

    def _read_loop(self, session):
        buffer = bytearray()
        phase, reason = "read", "port closed"
        try:
            while not session.stop.is_set():
                if not session.port.is_open:
                    break
                chunk = session.port.read(session.port.in_waiting or 1)
                received_at = time.monotonic()
                if not chunk:
                    continue
                session.last_rx = received_at
                context = receive_context(self, received_at=received_at, generation=session.generation)
                if session.stop.is_set():
                    break
                buffer += chunk
                phase = "receive dispatch"
                self._consume_buffer(buffer, context=context)
                phase = "read"
        except Exception as exc:
            reason = exception_reason(exc)
        finally:
            self._finish_session(session, phase, reason)
