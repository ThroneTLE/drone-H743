"""TCP client device process. The existing ground station remains the server."""

from __future__ import annotations

import select
import socket
import threading
import time

from tools.panel_lib.proto import PROTO_DIR_TO_FC

from .experiments import ExperimentKind, SimulationEngine
from .clocking import SimulationClock
from .protocol import FrameDecoder, SimulatorProtocol


class SimulatorDevice:
    def __init__(self, host: str = "127.0.0.1", port: int = 6666,
                 engine: SimulationEngine | None = None) -> None:
        self.host, self.port = host, port
        self.engine = engine or SimulationEngine()
        self.protocol = SimulatorProtocol(self.engine.bridge, self.engine.snapshot)
        self.decoder = FrameDecoder()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self._lock = threading.RLock()
        self.time_scale = 1.0

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._socket is not None

    def start(self) -> None:
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, name="sim-xz-device", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        with self._lock:
            sock = self._socket
            self._socket = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        if self.thread:
            self.thread.join(timeout=1.0)

    def set_kind(self, kind: ExperimentKind) -> None:
        self.engine.set_kind(kind)

    def reset(self) -> None:
        self.engine.reset()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=0.5)
                sock.setblocking(False)
            except OSError:
                self.stop_event.wait(0.5)
                continue
            with self._lock:
                self._socket = sock
            self.decoder = FrameDecoder(PROTO_DIR_TO_FC)
            last_telem = time.monotonic()
            clock = SimulationClock(self.engine, last_telem)
            try:
                while not self.stop_event.is_set():
                    timeout = min(0.05, self.engine.dt_s / max(self.time_scale, 0.05))
                    readable, _, _ = select.select([sock], [], [], timeout)
                    if readable:
                        try:
                            data = sock.recv(4096)
                        except BlockingIOError:
                            data = b""
                        if not data:
                            break
                        for frame in self.decoder.feed(data):
                            for response in self.protocol.handle_frame(frame.function, frame.payload):
                                sock.sendall(response)
                    now = time.monotonic()
                    clock.advance(now, self.time_scale)
                    if (self.protocol.stream_enabled and
                            now - last_telem >= 1.0 / self.protocol.stream_rate_hz):
                        sock.sendall(self.protocol.telemetry_frame())
                        last_telem = now
            except OSError:
                pass
            finally:
                self.engine.running = False
                with self._lock:
                    if self._socket is sock:
                        self._socket = None
                try:
                    sock.close()
                except OSError:
                    pass
