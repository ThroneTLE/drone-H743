"""Serial adapters. The FC side composes the panel's canonical transport/parser."""
from __future__ import annotations

import queue
import math
import threading
import time
from dataclasses import dataclass
from typing import Any

from tools.panel_lib.connection_state import ReceivedMessage
from tools.panel_lib.proto import PROTO_MSG_CMD_LINE
from tools.panel_lib.transport import (
    HAS_PYSERIAL,
    SerialTransport,
    serial,
    serial_device_identity_policy,
    serial_port_identity,
)

THRUST_BENCH_FUNCTION = 0x2235


def estimated_wire_bytes_per_second(snapshot_hz: float = 15.0,
                                    heartbeat_hz: float = 10.0) -> int:
    """Worst-case steady bidirectional bytes using the actual line/$X envelopes."""
    from .fc_protocol import PAYLOAD_BYTES, set_command, snapshot_command
    query = len((snapshot_command(0xFFFFFFFF) + "\r\n").encode("utf-8"))
    snapshot_frame = 9 + PAYLOAD_BYTES
    set_line = len((set_command(
        100, 100, request_id=0xFFFFFFFF,
        window_token=0xFFFFFFFF) + "\r\n").encode("utf-8"))
    ack_payload = len((
        "TBENCH state=set active=1 upper_pct=100.00 lower_pct=100.00 "
        "age_ms=0 request_id=4294967295 token=4294967295").encode("utf-8"))
    ack_frame = 9 + ack_payload
    return math.ceil(snapshot_hz * (query + snapshot_frame)
                     + heartbeat_hz * (set_line + ack_frame))


@dataclass(frozen=True)
class LinkEvent:
    kind: str
    generation: int
    host_time_s: float
    data: Any = None


class FlightControllerConnection:
    """Own one SerialTransport and expose decoded frames as immutable events."""
    def __init__(self, event_queue: "queue.Queue[LinkEvent]", transport=None,
                 identity_resolver=None) -> None:
        self.events = event_queue
        self.rx: queue.Queue = queue.Queue()
        self.transport = transport or SerialTransport(self.rx)
        self._stop: threading.Event | None = None
        self._reader: threading.Thread | None = None
        self._identity_resolver = identity_resolver or self._resolve_identity
        self.identity: dict[str, object] | None = None
        self.identity_status = "unknown"
        self.identity_reason = "尚未选择串口"
        self.last_rate_decision: dict[str, object] | None = None
        self.transport.set_binary_sink(self._on_binary, with_context=True)
        self.transport.set_frame_error_sink(self._on_frame_error)

    @property
    def generation(self) -> int: return self.transport.connection_generation
    @property
    def is_connected(self) -> bool: return self.transport.is_connected

    @staticmethod
    def _resolve_identity(port_name: str) -> dict[str, object] | None:
        if not HAS_PYSERIAL or serial is None:
            return None
        for item in serial.tools.list_ports.comports():
            identity = serial_port_identity(item)
            if str(identity.get("device", "")).casefold() == port_name.casefold():
                return identity
        return None

    def validate_snapshot_rate(self, snapshot_hz: float) -> dict[str, object]:
        budget = estimated_wire_bytes_per_second(snapshot_hz, 10.0)
        confirmed_usb = bool(
            self.identity
            and self.identity.get("vid") == 0x0483
            and self.identity.get("pid") == 0x5740
            and self.identity_status == "allowed")
        decision = {
            "snapshot_hz": float(snapshot_hz),
            "estimated_wire_bytes_per_second": budget,
            "serial_budget_bytes_per_second": 3456,
            "identity_status": self.identity_status,
            "identity_reason": self.identity_reason,
            "confirmed_stm32_usb_cdc": confirmed_usb,
            "device": str((self.identity or {}).get("device", "")),
            "vid": (self.identity or {}).get("vid"),
            "pid": (self.identity or {}).get("pid"),
            "rate_policy": "confirmed_usb_high_rate" if budget > 3456 else "serial_budget",
        }
        self.last_rate_decision = decision
        if budget > 3456 and not confirmed_usb:
            raise ValueError(
                f"快照 {snapshot_hz:g} Hz 预计 {budget} B/s，超过数传预算 3456 B/s；"
                f"当前链路：{self.identity_reason}。请改用已确认的 STM32 USB CDC "
                "(0483:5740) 或降低快照频率。")
        return decision

    def connect(self, port: str, baudrate: int = 115200,
                snapshot_hz: float = 15.0) -> None:
        if not port.strip():
            raise ValueError("必须明确选择飞控串口")
        self.disconnect()
        self.identity = self._identity_resolver(port)
        self.identity_status, self.identity_reason = serial_device_identity_policy(
            self.identity)
        self.validate_snapshot_rate(snapshot_hz)
        stop = threading.Event()
        self._stop = stop
        self.transport.start(port, baudrate)
        self._reader = threading.Thread(
            target=self._read_loop, args=(stop,),
            name="tbench-fc-events", daemon=True)
        self._reader.start()

    def disconnect(self) -> None:
        stop = self._stop
        reader = self._reader
        self._stop = None
        self._reader = None
        if stop is not None:
            stop.set()
        try:
            self.transport.stop()
        finally:
            if reader is not None and reader is not threading.current_thread():
                reader.join(timeout=0.4)
            self.events.put(LinkEvent(
                "fc_disconnected", self.generation, time.monotonic()))

    def send_command(self, text: str) -> bool:
        return bool(self.transport.send_frame(PROTO_MSG_CMD_LINE, text.encode("utf-8")))

    def _on_binary(self, function: int, payload: bytes, context) -> None:
        if function == THRUST_BENCH_FUNCTION:
            generation = context.generation if context is not None else self.generation
            received_at = context.received_at if context is not None else time.monotonic()
            self.events.put(LinkEvent("fc_payload", generation, received_at, bytes(payload)))

    def _on_frame_error(self, kind: str, context, details: dict) -> None:
        generation = context.generation if context is not None else self.generation
        received_at = context.received_at if context is not None else time.monotonic()
        self.events.put(LinkEvent("fc_frame_error", generation, received_at,
                                  {"error": kind, **details}))

    def _read_loop(self, stop: threading.Event) -> None:
        while not stop.is_set():
            try:
                item = self.rx.get(timeout=0.1)
            except queue.Empty:
                continue
            context = item.context if isinstance(item, ReceivedMessage) else None
            payload = item.payload if isinstance(item, ReceivedMessage) else item
            generation = context.generation if context is not None else self.generation
            if stop.is_set() or generation != self.generation:
                continue
            if isinstance(payload, tuple) and len(payload) == 3 and payload[0] == "proto":
                self.events.put(LinkEvent("fc_text", generation,
                                          context.received_at if context else time.monotonic(), payload[2]))
            elif isinstance(payload, str):
                self.events.put(LinkEvent("fc_text", generation, time.monotonic(), payload))


class ScaleSerialTransport:
    """Small synchronous Modbus adapter for LoadCell, with explicit port choice."""
    def __init__(self, serial_factory=None) -> None:
        self.serial_factory = serial_factory
        self.port = None

    def connect(self, port: str, baudrate: int = 9600, timeout_s: float = 0.25) -> None:
        if not port.strip(): raise ValueError("必须明确选择称重串口")
        if self.serial_factory is None:
            import serial
            self.serial_factory = serial.Serial
        self.close(); self.port = self.serial_factory(port=port, baudrate=baudrate, timeout=timeout_s)

    def request(self, payload: bytes, expected: int) -> bytes:
        if self.port is None: raise RuntimeError("称重串口未连接")
        self.port.reset_input_buffer(); self.port.write(payload); self.port.flush()
        response = self.port.read(expected)
        if len(response) != expected: raise TimeoutError(f"称重回复超时：{len(response)}/{expected} 字节")
        return response

    def close(self) -> None:
        port, self.port = self.port, None
        if port is not None: port.close()
