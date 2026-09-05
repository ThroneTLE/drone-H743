"""Transport primitives and serial-device reconnect helpers for the panel."""

from __future__ import annotations

import queue
import socket
import threading
import time
from abc import ABC, abstractmethod
from typing import Callable, Sequence

from .connection_state import ReceivedMessage, receive_context
from .serial_session import SerialSessionMixin

from .proto import (
    PROTO_BINARY_FUNCTIONS,
    PROTO_DIR_FROM_FC,
    PROTO_DIR_TO_FC,
    PROTO_HEADER,
    PROTO_MAX_FRAME_PAYLOAD,
    PROTO_MSG_CMD_LINE,
)


SERIAL_ASCII_COMPAT_MODE = True
SERIAL_TX_DEBUG_ENABLED = True

try:
    import serial  # type: ignore
    import serial.tools.list_ports  # type: ignore

    HAS_PYSERIAL = True
    PYSERIAL_ERROR = ""
except Exception as exc:  # pragma: no cover - depends on local optional package
    serial = None  # type: ignore[assignment]
    HAS_PYSERIAL = False
    PYSERIAL_ERROR = str(exc)


def udp_payload_is_probably_text(data: bytes) -> bool:
    if not data:
        return False
    if data.startswith(PROTO_HEADER):
        return True
    if b"\x00" in data:
        return False

    allowed = 0
    for byte in data:
        if byte in (9, 10, 13) or 32 <= byte <= 126:
            allowed += 1
    return (allowed / len(data)) >= 0.95


def serial_device_identity_policy(identity: dict[str, object] | None) -> tuple[str, str]:
    """Classify a COM device for the application-CDC to ROM-DFU handoff."""

    if not identity:
        return "unknown", "没有该 COM 口的 USB 身份信息"
    combined = " ".join(
        str(identity.get(key) or "")
        for key in ("description", "hwid", "location", "serial_number")
    ).upper()
    rejected_tokens = (
        "STLINK", "ST-LINK", "CH340", "CH341", "CP210", "FTDI", "FT232",
        "FT4232", "BLUETOOTH", " BLE ",
    )
    matched = next((token for token in rejected_tokens if token in f" {combined} "), None)
    if matched is not None:
        return "rejected", f"检测到不允许的串口适配器：{matched.strip()}"
    vid = identity.get("vid")
    pid = identity.get("pid")
    if vid == 0x0483 and pid == 0x5740:
        return "allowed", "STM32 application USB CDC VID:PID=0483:5740"
    return "unknown", (
        f"未知 USB 串口身份 VID:PID="
        f"{int(vid):04X}:{int(pid):04X}" if isinstance(vid, int) and isinstance(pid, int)
        else "未知 USB 串口身份（无 VID/PID）"
    )


def serial_port_identity(port: object) -> dict[str, object]:
    """Copy the stable USB identity fields from a pyserial ListPortInfo."""

    return {
        "device": str(getattr(port, "device", "") or ""),
        "vid": getattr(port, "vid", None),
        "pid": getattr(port, "pid", None),
        "description": str(getattr(port, "description", "") or ""),
        "hwid": str(getattr(port, "hwid", "") or ""),
        "location": str(getattr(port, "location", "") or ""),
        "serial_number": str(getattr(port, "serial_number", "") or ""),
    }


def serial_port_fingerprint(identity: dict[str, object] | None) -> str:
    """A key that survives Windows renumbering a COM port.

    USB 串口的 COM 号是系统分配的，同一块飞控今天是 COM31、明天可能是 COM30
    （本机就出现过 COM30/31/32 三个 0483:5740 的历史记录）。真正稳定的是 USB
    序列号；没有序列号时退到 VID:PID + 物理位置。
    """
    if not identity:
        return ""
    serial_number = str(identity.get("serial_number") or "").strip()
    vid = identity.get("vid")
    pid = identity.get("pid")
    if isinstance(vid, int) and isinstance(pid, int):
        prefix = f"{vid:04X}:{pid:04X}"
    else:
        prefix = "----:----"
    if serial_number:
        return f"{prefix}/{serial_number}"
    location = str(identity.get("location") or "").strip()
    if location:
        return f"{prefix}@{location}"
    return ""


def match_remembered_serial_port(
    remembered_device: str,
    remembered_fingerprint: str,
    identities: dict[str, dict[str, object]],
) -> tuple[str | None, str]:
    """Find last session's flight controller among the currently present ports.

    优先按 USB 指纹认板子，其次才认 COM 号。反过来做会在系统重新分配 COM 号之后
    连到另一台设备上——那可能是 ST-Link 或者别人的串口。
    """
    if remembered_fingerprint:
        for device, identity in identities.items():
            if serial_port_fingerprint(identity) == remembered_fingerprint:
                if device.casefold() == remembered_device.casefold():
                    return device, f"按 USB 序列号匹配到 {device}"
                return device, f"按 USB 序列号匹配到 {device}（上次是 {remembered_device}）"
    if not remembered_device:
        return None, "没有上次连接记录"
    for device in identities:
        if device.casefold() == remembered_device.casefold():
            if remembered_fingerprint:
                # 指纹对不上却占着同一个 COM 号：多半换了设备，交给用户自己选。
                return None, f"{remembered_device} 存在，但 USB 身份和上次不一致"
            return device, f"按串口号匹配到 {device}"
    return None, f"上次用的 {remembered_device} 现在不在"


def select_reenumerated_application_port(
    previous_port: str,
    previous_identity: dict[str, object] | None,
    candidates: Sequence[dict[str, object]],
) -> str | None:
    """Select only the same application CDC after ROM-DFU re-enumeration."""

    if not previous_identity:
        return None
    previous_vid = previous_identity.get("vid")
    previous_pid = previous_identity.get("pid")
    previous_serial = str(previous_identity.get("serial_number") or "").casefold()
    previous_location = str(previous_identity.get("location") or "").casefold()
    eligible = [
        row for row in candidates
        if serial_device_identity_policy(row)[0] == "allowed"
        and row.get("vid") == previous_vid
        and row.get("pid") == previous_pid
    ]
    if previous_serial:
        matches = [row for row in eligible if str(
            row.get("serial_number") or "").casefold() == previous_serial]
        if len(matches) == 1:
            return str(matches[0].get("device") or "") or None
    if previous_location:
        matches = [row for row in eligible if str(
            row.get("location") or "").casefold() == previous_location]
        if len(matches) == 1:
            return str(matches[0].get("device") or "") or None
    matches = [row for row in eligible if str(
        row.get("device") or "").casefold() == previous_port.casefold()]
    if len(matches) == 1:
        return str(matches[0].get("device") or "") or None
    return None


def wait_for_application_serial(
    previous_port: str,
    previous_identity: dict[str, object] | None,
    *,
    timeout_s: float = 15.0,
    poll_interval_s: float = 0.25,
    cancel_event: threading.Event | None = None,
    enumerate_ports: Callable[[], Sequence[object]] | None = None,
    on_log: Callable[[str], None] | None = None,
) -> str | None:
    """Wait for the same STM32 application CDC and return its COM device."""

    if timeout_s <= 0.0:
        return None
    cancel = cancel_event or threading.Event()
    if enumerate_ports is None:
        if not HAS_PYSERIAL or serial is None:
            return None
        enumerate_ports = lambda: tuple(  # type: ignore[union-attr]
            serial.tools.list_ports.comports())
    deadline = time.monotonic() + timeout_s
    if on_log is not None:
        on_log("等待飞控 application CDC 重新枚举…")
    while time.monotonic() < deadline:
        if cancel.is_set():
            return None
        try:
            identities = [
                row if isinstance(row, dict) else serial_port_identity(row)
                for row in enumerate_ports()
            ]
        except Exception as exc:
            if on_log is not None:
                on_log(f"枚举 application CDC 暂时失败：{exc}")
            identities = []
        matched = select_reenumerated_application_port(
            previous_port, previous_identity, identities)
        if matched is not None:
            if on_log is not None:
                on_log(f"已匹配原飞控 application CDC：{matched}")
            return matched
        cancel.wait(max(0.05, poll_interval_s))
    if on_log is not None:
        on_log("application CDC 自动重连等待超时")
    return None


def proto_crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_proto_frame(direction: int, function: int, payload: bytes) -> bytes:
    body = bytearray()
    body.append(0)
    body.append(function & 0xFF)
    body.append((function >> 8) & 0xFF)
    body.append(len(payload) & 0xFF)
    body.append((len(payload) >> 8) & 0xFF)
    body.extend(payload)
    crc = proto_crc8_dvb_s2(bytes(body))
    return PROTO_HEADER + bytes([direction]) + bytes(body) + bytes([crc])


class TransportBase(ABC):
    def __init__(self, rx_queue: "queue.Queue[str]") -> None:
        self.rx_queue = rx_queue
        self._connection_generation = 0

    @property
    def connection_generation(self) -> int:
        return self._connection_generation

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def start(self, *args, **kwargs) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        raise NotImplementedError

    def send_line(self, line: str) -> bool:
        return self.send_frame(
            PROTO_MSG_CMD_LINE,
            line.rstrip("\r\n").encode("utf-8"),
        )

    def cancel_pending_sends(self) -> None:
        """Invalidate queued writes before entering a read-only V0 session."""

        return

    def set_binary_sink(self, sink: Callable[[int, bytes], None] | None) -> None:
        """Attach the consumer for binary-payload frames (telemetry stream v2).

        走回调而不是走 `rx_queue`（与规划文档 §2.6 的"投递 ("proto_bin", fn,
        bytes)"有意偏离，理由写在这里）：`rx_queue` 是 Tk 主循环按批次抽干的，
        把 40 Hz~1 kHz 的遥测帧塞进去，波形就被 Tk 的事件循环节奏牵着走了——
        正好与同一节要求的"环形缓冲在收线程里写、Tk 线程只读快照"相反。而且
        `_drain_rx` 不认识的元组会掉进 `str(item)` 分支刷屏原始命令日志，要修
        它就得改 `drone_tcp_panel.py`，而 R-T1-2 的判据是不改那个文件。

        没挂 sink 时不是丢掉就算了：每一帧都计进 `binary_unclaimed`，
        示波器页的统计条会把它显示出来。
        """

        self._binary_sink = sink

    @property
    def binary_unclaimed(self) -> int:
        """收到但没人接手的二进制帧数（示波器页没打开时就是它在涨）。"""

        return getattr(self, "_binary_unclaimed", 0)

    def _deliver_binary(self, function: int, payload: bytes) -> None:
        sink = getattr(self, "_binary_sink", None)
        if sink is None:
            self._binary_unclaimed = self.binary_unclaimed + 1
            return
        sink(function, payload)

    def _consume_buffer(self, buffer: bytearray, *, context=None) -> None:
        if context is None and getattr(self, "_stamp_received", False):
            context = receive_context(self)

        def emit(payload):
            self.rx_queue.put(ReceivedMessage(payload, context) if context is not None else payload)

        while buffer:
            if len(buffer) >= 9 and buffer[0:2] == PROTO_HEADER:
                if buffer[2] not in (PROTO_DIR_TO_FC, PROTO_DIR_FROM_FC):
                    # `$X` 后面跟着非法方向字节：这两个字节只是碰巧长得像帧头。
                    # 必须在这里丢掉一个字节重新找——否则下面的文本分支会算出
                    # frame_index == 0，`if frame_index > 0` 不成立就 break，
                    # 缓冲区永远以这个假帧头开头，整条链路就此静止。
                    del buffer[0]
                    continue

                payload_length = buffer[6] | (buffer[7] << 8)
                if payload_length > PROTO_MAX_FRAME_PAYLOAD:
                    # len 被打坏成一个不可能的长度。当成"还没收全"去等的话，
                    # 要等到 65 KB 之后才会发现不对——57600 baud 上就是十几秒
                    # 的黑屏，而且期间到达的每一帧好数据都被吞进这个假帧里。
                    del buffer[0]
                    continue

                frame_length = 9 + payload_length
                if len(buffer) < frame_length:
                    break

                frame = bytes(buffer[:frame_length])
                body = frame[3:-1]
                if proto_crc8_dvb_s2(body) == frame[-1]:
                    function = frame[4] | (frame[5] << 8)
                    payload = frame[8:-1]
                    del buffer[:frame_length]

                    if frame[2] == PROTO_DIR_FROM_FC:
                        if function in PROTO_BINARY_FUNCTIONS:
                            if context is None or context.is_current(self):
                                self._deliver_binary(function, payload)
                            continue
                        text = payload.decode("utf-8", errors="replace").rstrip("\r\n")
                        emit(("proto", function, text))
                    else:
                        shown = payload.decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                        emit(f"RXRAW fn=0x{function:04X} len={len(payload)} data={shown}")
                    continue

                del buffer[0]
                continue

            newline_index = buffer.find(b"\n")
            frame_index = buffer.find(PROTO_HEADER)
            if newline_index != -1 and (frame_index == -1 or newline_index < frame_index):
                line = bytes(buffer[:newline_index]).rstrip(b"\r")
                del buffer[: newline_index + 1]
                emit(line.decode("utf-8", errors="replace"))
                continue

            if frame_index > 0:
                raw = bytes(buffer[:frame_index])
                del buffer[:frame_index]
                shown = raw.decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                emit(f"RXRAW len={len(raw)} data={shown}")
                continue

            break


class TcpTransport(TransportBase):
    _stamp_received = True

    def __init__(self, rx_queue: "queue.Queue[str]") -> None:
        super().__init__(rx_queue)
        self.sock: socket.socket | None = None
        self.client: socket.socket | None = None
        self.thread: threading.Thread | None = None
        self._sender_thread: threading.Thread | None = None
        self._send_queue: "queue.Queue[tuple[int, bytes] | None]" = queue.Queue()
        self._send_generation = 0
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        with self.lock:
            return self.client is not None

    def start(self, host: str, port: int) -> None:
        self.stop()
        self._send_queue = queue.Queue()
        with self.lock:
            self._send_generation += 1
        self.stop_event.clear()
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()
        self.thread = threading.Thread(target=self._run, args=(host, port), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        try:
            self._send_queue.put_nowait(None)
        except queue.Full:
            pass
        with self.lock:
            sockets = [self.client, self.sock]
            self._connection_generation += 1
            self.client = None
            self.sock = None
        for item in sockets:
            if item is not None:
                try:
                    item.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    item.close()
                except OSError:
                    pass

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        frame = build_proto_frame(PROTO_DIR_TO_FC, function, payload)
        with self.lock:
            if self.client is None:
                return False
            generation = self._send_generation
        try:
            self._send_queue.put_nowait((generation, frame))
            return True
        except queue.Full:
            return False

    def cancel_pending_sends(self) -> None:
        with self.lock:
            self._send_generation += 1
        try:
            while True:
                self._send_queue.get_nowait()
        except queue.Empty:
            return

    def _sender_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                item = self._send_queue.get(timeout=0.3)
            except queue.Empty:
                continue
            if item is None:
                break
            generation, frame = item
            with self.lock:
                if generation != self._send_generation:
                    continue
                client = self.client
                if client is None:
                    continue
                try:
                    client.sendall(frame)
                except OSError as exc:
                    self.rx_queue.put(f"[上位机] 发送失败: {exc}")

    def _run(self, host: str, port: int) -> None:
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, port))
            server.listen(1)
            server.settimeout(0.5)
            with self.lock:
                self.sock = server
            self.rx_queue.put(f"[上位机] 正在监听 {host}:{port}")
        except OSError as exc:
            self.rx_queue.put(f"[上位机] 监听失败: {exc}")
            return

        while not self.stop_event.is_set():
            try:
                client, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            with self.lock:
                if self.client is not None:
                    try:
                        self.client.close()
                    except OSError:
                        pass
                self.client = client
                self._connection_generation += 1
                generation = self._connection_generation
            self.rx_queue.put(f"[上位机] 板子已连接: {addr[0]}:{addr[1]}")
            self._read_client(client, generation)

        self.rx_queue.put("[上位机] TCP 服务已停止")

    def _read_client(self, client: socket.socket, generation: int) -> None:
        client.settimeout(0.5)
        buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                data = client.recv(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            buffer += data
            self._consume_buffer(buffer, context=receive_context(self, generation=generation))
        with self.lock:
            if self.client is client:
                self.client = None
                self._connection_generation += 1
        try:
            client.close()
        except OSError:
            pass
        self.rx_queue.put("[上位机] 板子已断开")


class UdpTransport(TransportBase):
    _stamp_received = True

    def __init__(self, rx_queue: "queue.Queue[str]") -> None:
        super().__init__(rx_queue)
        self.sock: socket.socket | None = None
        self.thread: threading.Thread | None = None
        self.remote: tuple[str, int] | None = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        with self.lock:
            return self.sock is not None and self.remote is not None

    def start(self, bind_ip: str, local_port: int, module_ip: str, module_port: int) -> None:
        self.stop()
        self.stop_event.clear()
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((bind_ip, local_port))
            sock.settimeout(0.5)
        except OSError as exc:
            self.rx_queue.put(f"[host] UDP open failed: {exc}")
            return

        with self.lock:
            self.sock = sock
            self.remote = (module_ip, module_port)
            self._connection_generation += 1
            generation = self._connection_generation
        self.rx_queue.put(f"[host] UDP ready local={bind_ip}:{local_port} module={module_ip}:{module_port}")
        self.thread = threading.Thread(target=self._read_loop, args=(sock, generation), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        with self.lock:
            sock = self.sock
            self.sock = None
            self.remote = None
            self._connection_generation += 1
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        del function
        try:
            text = payload.decode("utf-8") if payload else ""
        except UnicodeDecodeError:
            return False
        return self.send_line(text)

    def send_line(self, line: str) -> bool:
        data = (line.rstrip("\r\n") + "\r\n").encode("utf-8")
        with self.lock:
            sock = self.sock
            remote = self.remote
        if sock is None or remote is None:
            return False
        try:
            sock.sendto(data, remote)
            return True
        except OSError as exc:
            self.rx_queue.put(f"[host] UDP send failed: {exc}")
            return False

    def _read_loop(self, sock: socket.socket, generation: int) -> None:
        while not self.stop_event.is_set():
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not udp_payload_is_probably_text(data):
                self.rx_queue.put(("udp_raw", addr[0], addr[1], len(data)))
                continue
            buffer = bytearray(data)
            self._consume_buffer(buffer, context=receive_context(self, generation=generation))
            if buffer:
                shown = bytes(buffer).decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                self.rx_queue.put(f"RXRAW len={len(buffer)} data={shown}")
        with self.lock:
            if self.sock is sock:
                self.sock = None
                self.remote = None
                self._connection_generation += 1
        self.rx_queue.put("[host] UDP stopped")


class SerialTransport(SerialSessionMixin, TransportBase):
    """Public transport API; serial session lifecycle lives in serial_session.py."""


__all__ = [
    "HAS_PYSERIAL",
    "PYSERIAL_ERROR",
    "PROTO_DIR_FROM_FC",
    "PROTO_DIR_TO_FC",
    "PROTO_HEADER",
    "PROTO_MSG_CMD_LINE",
    "SERIAL_ASCII_COMPAT_MODE",
    "SERIAL_TX_DEBUG_ENABLED",
    "SerialTransport",
    "TcpTransport",
    "TransportBase",
    "UdpTransport",
    "build_proto_frame",
    "match_remembered_serial_port",
    "proto_crc8_dvb_s2",
    "select_reenumerated_application_port",
    "serial",
    "serial_device_identity_policy",
    "serial_port_fingerprint",
    "serial_port_identity",
    "udp_payload_is_probably_text",
    "wait_for_application_serial",
]
