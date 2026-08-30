"""Transport primitives and serial-device reconnect helpers for the panel."""

from __future__ import annotations

import queue
import socket
import threading
import time
from abc import ABC, abstractmethod
from typing import Callable, Sequence


SERIAL_ASCII_COMPAT_MODE = True
SERIAL_TX_DEBUG_ENABLED = True
PROTO_HEADER = b"$X"
PROTO_DIR_TO_FC = ord("<")
PROTO_DIR_FROM_FC = ord(">")
PROTO_MSG_CMD_LINE = 0x2000

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

    def _consume_buffer(self, buffer: bytearray) -> None:
        while buffer:
            if len(buffer) >= 9 and buffer[0:2] == PROTO_HEADER and buffer[2] in (PROTO_DIR_TO_FC, PROTO_DIR_FROM_FC):
                payload_length = buffer[6] | (buffer[7] << 8)
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
                        text = payload.decode("utf-8", errors="replace").rstrip("\r\n")
                        self.rx_queue.put(("proto", function, text))
                    else:
                        shown = payload.decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                        self.rx_queue.put(f"RXRAW fn=0x{function:04X} len={len(payload)} data={shown}")
                    continue

                del buffer[0]
                continue

            newline_index = buffer.find(b"\n")
            frame_index = buffer.find(PROTO_HEADER)
            if newline_index != -1 and (frame_index == -1 or newline_index < frame_index):
                line = bytes(buffer[:newline_index]).rstrip(b"\r")
                del buffer[: newline_index + 1]
                self.rx_queue.put(line.decode("utf-8", errors="replace"))
                continue

            if frame_index > 0:
                raw = bytes(buffer[:frame_index])
                del buffer[:frame_index]
                shown = raw.decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                self.rx_queue.put(f"RXRAW len={len(raw)} data={shown}")
                continue

            break


class TcpTransport(TransportBase):
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
            self.rx_queue.put(f"[上位机] 板子已连接: {addr[0]}:{addr[1]}")
            self._read_client(client)

        self.rx_queue.put("[上位机] TCP 服务已停止")

    def _read_client(self, client: socket.socket) -> None:
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
            self._consume_buffer(buffer)
        with self.lock:
            if self.client is client:
                self.client = None
        try:
            client.close()
        except OSError:
            pass
        self.rx_queue.put("[上位机] 板子已断开")


class UdpTransport(TransportBase):
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
        self.rx_queue.put(f"[host] UDP ready local={bind_ip}:{local_port} module={module_ip}:{module_port}")
        self.thread = threading.Thread(target=self._read_loop, args=(sock,), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        with self.lock:
            sock = self.sock
            self.sock = None
            self.remote = None
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

    def _read_loop(self, sock: socket.socket) -> None:
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
            self._consume_buffer(buffer)
            if buffer:
                shown = bytes(buffer).decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                self.rx_queue.put(f"RXRAW len={len(buffer)} data={shown}")
        with self.lock:
            if self.sock is sock:
                self.sock = None
                self.remote = None
        self.rx_queue.put("[host] UDP stopped")


class SerialTransport(TransportBase):
    def __init__(self, rx_queue: "queue.Queue[str]") -> None:
        super().__init__(rx_queue)
        self.port: "serial.Serial | None" = None
        self.thread: threading.Thread | None = None
        self._sender_thread: threading.Thread | None = None
        self._send_queue: "queue.Queue[tuple[int, bytes] | None]" = queue.Queue()
        self._send_generation = 0
        self._connection_generation = 0
        self._active_port: str | None = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        with self.lock:
            return self.port is not None and bool(self.port.is_open)

    @property
    def active_port(self) -> str | None:
        with self.lock:
            return self._active_port

    @property
    def connection_generation(self) -> int:
        with self.lock:
            return self._connection_generation

    def start(self, port_name: str, baudrate: int) -> None:
        if not HAS_PYSERIAL or serial is None:
            self.rx_queue.put(f"[上位机] 串口模式不可用: {PYSERIAL_ERROR or '未安装 pyserial'}")
            return
        self.stop()
        self._send_queue = queue.Queue()
        with self.lock:
            self._send_generation += 1
        self.stop_event.clear()
        try:
            opened = serial.Serial(port_name, baudrate=baudrate, timeout=0.2, write_timeout=0.5)
        except Exception as exc:
            self.rx_queue.put(f"[上位机] 打开串口失败: {exc}")
            return
        with self.lock:
            self.port = opened
            self._active_port = str(port_name)
            self._connection_generation += 1
        self.rx_queue.put(f"[上位机] 串口已连接: {port_name} @ {baudrate}")
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()
        self.thread = threading.Thread(target=self._read_loop, args=(opened,), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        try:
            self._send_queue.put_nowait(None)
        except queue.Full:
            pass
        with self.lock:
            port = self.port
            self.port = None
            self._active_port = None
            self._connection_generation += 1
        if port is not None:
            self._safe_close(port)

    @staticmethod
    def _safe_close(port) -> None:
        try:
            if hasattr(port, 'cancel_read'):
                port.cancel_read()
        except Exception:
            pass
        try:
            if hasattr(port, 'cancel_write'):
                port.cancel_write()
        except Exception:
            pass
        try:
            port.close()
        except Exception:
            pass

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        if SERIAL_ASCII_COMPAT_MODE:
            try:
                text = payload.decode("utf-8") if payload else ""
            except UnicodeDecodeError:
                text = ""
            if text:
                return self.send_line(text)
        frame = build_proto_frame(PROTO_DIR_TO_FC, function, payload)
        with self.lock:
            if self.port is None or not self.port.is_open:
                return False
            generation = self._send_generation
        try:
            self._send_queue.put_nowait((generation, frame))
            return True
        except queue.Full:
            return False

    def send_line(self, line: str) -> bool:
        data = (line.rstrip("\r\n") + "\r\n").encode("utf-8")
        with self.lock:
            if self.port is None or not self.port.is_open:
                return False
            generation = self._send_generation
        try:
            self._send_queue.put_nowait((generation, data))
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
                port = self.port
                if port is None or not port.is_open:
                    continue
                try:
                    written = port.write(frame)
                    port.flush()
                    if SERIAL_TX_DEBUG_ENABLED:
                        shown = frame.decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                        self.rx_queue.put(f"[host] serial tx bytes={written} data={shown}")
                except Exception as exc:
                    self.rx_queue.put(f"[上位机] 串口发送失败: {exc}")

    def _read_loop(self, port: "serial.Serial") -> None:
        buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                if not port.is_open:
                    break
                waiting = port.in_waiting
                chunk = port.read(waiting or 1)
            except Exception:
                break
            if not chunk:
                continue
            buffer += chunk
            self._consume_buffer(buffer)
        with self.lock:
            was_current = self.port is port
            if self.port is port:
                self.port = None
                self._active_port = None
                self._connection_generation += 1
        if was_current:
            self.rx_queue.put("[上位机] 串口已断开")


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
