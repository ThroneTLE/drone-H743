#!/usr/bin/env python3
"""Ground-station panel for the drone-H743 Ai-WB2 transparent link."""

from __future__ import annotations

import csv
import json
import math
import os
import queue
import re
import socket
import threading
import time
import tkinter as tk
from abc import ABC, abstractmethod
from pathlib import Path
from tkinter import filedialog, messagebox, ttk


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 6666
DEFAULT_MODULE_IP = "192.168.223.181"
DEFAULT_UDP_LOCAL_PORT = 6668
DEFAULT_UDP_MODULE_PORT = 7777
MAX_BARO_SAMPLES = 2000
MAX_GPS_TRACK_POINTS = 5000
MAX_IDENT_SAMPLES = 5000
BARO_STREAM_PERIOD_MS = 50
IMU_POLL_PERIOD_MS = 100
CMD_REPLY_TIMEOUT_MS = 2500
PROTO_COMPAT_FALLBACK_DELAY_MS = 400
PROTO_PROBE_SETTLE_MS = 900
SERIAL_ASCII_COMPAT_MODE = True
SERIAL_TX_DEBUG_ENABLED = True
PROTO_HEADER = b"$X"
PROTO_DIR_TO_FC = ord("<")
PROTO_DIR_FROM_FC = ord(">")
PROTO_REQ_PING = 0x1000
PROTO_REQ_STATUS = 0x1001
PROTO_REQ_CONFIG = 0x1002
PROTO_REQ_PARAMS = 0x1003
PROTO_REQ_PID = 0x1004
PROTO_REQ_BARO = 0x1005
PROTO_REQ_BARO_STREAM = 0x1006
PROTO_REQ_FLASH = 0x1007
PROTO_REQ_IMU = 0x1008
PROTO_REQ_MODULES = 0x1009
PROTO_REQ_CAPS = 0x100A
PROTO_REQ_SAVE = 0x100B
PROTO_REQ_LOAD = 0x100C
PROTO_REQ_DEFAULTS = 0x100D
PROTO_REQ_PARAM_SET = 0x100E
PROTO_REQ_PID_SET = 0x100F
PROTO_REQ_SERVO_MOVE = 0x1010
PROTO_REQ_SERVO_MOVE_ALL = 0x1011
PROTO_REQ_SERVO_ID = 0x1012
PROTO_REQ_SERVO_SETID = 0x1013
PROTO_REQ_SERVO_MODE = 0x1014
PROTO_REQ_SERVO_ENABLE = 0x1015
PROTO_REQ_SERVO_ACTION = 0x1016
PROTO_REQ_SERVO_RAW = 0x1017
PROTO_REQ_WIFI = 0x1018
PROTO_REQ_GPS = 0x1019
PROTO_REQ_MAG = 0x101A
PROTO_REQ_RTOS = 0x101B
PROTO_REQ_AIRFRAME = 0x101C
PROTO_REQ_IDENT = 0x101D
PROTO_MSG_CMD_LINE = 0x2000
PROTO_MSG_TEXT_LINE = 0x2001
PROTO_MSG_CMD_RX = 0x2100
PROTO_MSG_CMD_ACK = 0x2101
PROTO_MSG_CMD_ERR = 0x2102
PROTO_MSG_CMD_OK = 0x2103
PROTO_MSG_PONG = 0x2200
PROTO_MSG_HW_FLASH = 0x2201
PROTO_MSG_HW_BARO = 0x2202
PROTO_MSG_HW_IMU = 0x2203
PROTO_MSG_STATUS_FLASH = 0x2204
PROTO_MSG_STATUS_BARO = 0x2205
PROTO_MSG_STATUS_IMU = 0x2206
PROTO_MSG_UART_STATS = 0x2207
PROTO_MSG_CONFIG_SUMMARY = 0x2208
PROTO_MSG_CONFIG_SERVO = 0x2209
PROTO_MSG_PARAM_RECORD = 0x220A
PROTO_MSG_PID_RECORD = 0x220B
PROTO_MSG_FLASH_RECORD = 0x220C
PROTO_MSG_BARO_STATE = 0x220D
PROTO_MSG_BARO_DIAG = 0x220E
PROTO_MSG_BARO_RAW = 0x220F
PROTO_MSG_BARO_STREAM = 0x2210
PROTO_MSG_IMU_STATE = 0x2211
PROTO_MSG_IMU_SCALED = 0x2212
PROTO_MSG_MODULES_SUMMARY = 0x2213
PROTO_MSG_CAPS_RECORD = 0x2214
PROTO_MSG_READY = 0x2215
PROTO_MSG_SAVE_RESULT = 0x2216
PROTO_MSG_LOAD_RESULT = 0x2217
PROTO_MSG_DEFAULTS_RESULT = 0x2218
PROTO_MSG_SERVO_RESULT = 0x2219
PROTO_MSG_WIFI_RECORD = 0x221A
PROTO_MSG_GPS_RECORD = 0x221B
PROTO_MSG_MAG_RECORD = 0x221C
PROTO_MSG_RTOS_RECORD = 0x221D
PROTO_MSG_FLASH_BENCH = 0x221E
PROTO_MSG_AIRFRAME_RECORD = 0x221F

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure

    HAS_MATPLOTLIB = True
    MATPLOTLIB_ERROR = ""
except Exception as exc:  # pragma: no cover - depends on local optional package
    FigureCanvasTkAgg = None  # type: ignore[assignment]
    Figure = None  # type: ignore[assignment]
    HAS_MATPLOTLIB = False
    MATPLOTLIB_ERROR = str(exc)

try:
    import serial  # type: ignore
    import serial.tools.list_ports  # type: ignore

    HAS_PYSERIAL = True
    PYSERIAL_ERROR = ""
except Exception as exc:  # pragma: no cover - depends on local optional package
    serial = None  # type: ignore[assignment]
    HAS_PYSERIAL = False
    PYSERIAL_ERROR = str(exc)


MODULES = [
    ("FLASH", "GD25Q32 Flash", "STATUS?"),
    ("SPL06", "SPL06 气压计", "STATUS?"),
    ("ICM42688", "ICM42688 IMU", "STATUS?"),
    ("GPS", "M9N GPS", "GPS?"),
    ("MAG", "I2C1 Magnetometer", "MAG?"),
    ("UART1", "USART1 链路", "STATUS?"),
    ("WIFI", "Ai-WB2 WiFi", "WIFI?"),
]

MODULE_ALIASES = {
    "FLASH": "FLASH",
    "GD25Q32": "FLASH",
    "SPL06": "SPL06",
    "BARO": "SPL06",
    "ICM42688": "ICM42688",
    "IMU": "ICM42688",
    "GPS": "GPS",
    "GPS_USART2": "GPS",
    "M9N": "GPS",
    "UBX": "GPS",
    "MAG": "MAG",
    "MAG_I2C1": "MAG",
    "IST8310": "MAG",
    "HMC5883": "MAG",
    "QMC5883L": "MAG",
    "UART1": "UART1",
    "USART1": "UART1",
    "WIFI": "WIFI",
    "AIWB2": "WIFI",
}


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


def parse_kv(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for match in re.finditer(r"([A-Za-z0-9_.-]+)=([^ \r\n]+)", line):
        result[match.group(1)] = match.group(2).rstrip(",")
    return result


def safe_int(value: str | None, default: int = 0) -> int:
    if value is None:
        return default
    try:
        return int(value, 0)
    except ValueError:
        return default


def first_value(values: dict[str, str], *names: str) -> str:
    for name in names:
        if name in values:
            return values[name]
    return "-"


def first_float(values: dict[str, str], *names: str) -> float | None:
    for name in names:
        if name not in values:
            continue
        try:
            return float(values[name])
        except ValueError:
            return None
    return None


def airframe_record_from_line(line: str) -> dict[str, str | float] | None:
    if not line.startswith("AIRFRAME "):
        return None
    values = parse_kv(line)
    record: dict[str, str | float] = {"line": line}
    for key, value in values.items():
        parsed = first_float(values, key)
        record[key] = parsed if parsed is not None else value
    return record


def ident_record_from_line(line: str) -> dict[str, str | float | int] | None:
    if not line.startswith("IDENT sample "):
        return None
    values = parse_kv(line)
    record: dict[str, str | float | int] = {"line": line}
    for key, value in values.items():
        record[key] = value
    for key in ("id", "seq", "t_ms", "alpha_us", "beta_us", "rc_arm", "throttle_us"):
        if key in values:
            record[key] = safe_int(values[key])
    if "roll_mdeg" in values:
        record["roll"] = safe_int(values["roll_mdeg"]) / 1000.0
    elif "roll" in values:
        parsed = first_float(values, "roll")
        if parsed is not None:
            record["roll"] = parsed
    if "pitch_mdeg" in values:
        record["pitch"] = safe_int(values["pitch_mdeg"]) / 1000.0
    elif "pitch" in values:
        parsed = first_float(values, "pitch")
        if parsed is not None:
            record["pitch"] = parsed
    if "gx_cdps" in values:
        record["gx"] = safe_int(values["gx_cdps"]) / 100.0
    elif "gx" in values:
        parsed = first_float(values, "gx")
        if parsed is not None:
            record["gx"] = parsed
    if "gy_cdps" in values:
        record["gy"] = safe_int(values["gy_cdps"]) / 100.0
    elif "gy" in values:
        parsed = first_float(values, "gy")
        if parsed is not None:
            record["gy"] = parsed
    return record


def fit_ident_step(samples: list[dict[str, str | float | int]], axis: str) -> dict[str, float] | None:
    if len(samples) < 8:
        return None
    value_key = "roll" if axis == "roll" else "pitch"
    input_key = "alpha_us" if axis == "roll" else "beta_us"
    rows = [
        row for row in samples
        if isinstance(row.get("t_ms"), int)
        and isinstance(row.get(value_key), float)
        and isinstance(row.get(input_key), int)
    ]
    if len(rows) < 8:
        return None
    t0 = float(rows[0]["t_ms"]) / 1000.0
    times = [(float(row["t_ms"]) / 1000.0) - t0 for row in rows]
    outputs = [float(row[value_key]) for row in rows]
    inputs = [float(row[input_key]) for row in rows]
    baseline_n = max(1, min(5, len(outputs) // 5))
    baseline_y = sum(outputs[:baseline_n]) / baseline_n
    baseline_u = sum(inputs[:baseline_n]) / baseline_n
    final_n = max(1, min(8, len(outputs) // 4))
    final_y = sum(outputs[-final_n:]) / final_n
    final_u = sum(inputs[-final_n:]) / final_n
    du = final_u - baseline_u
    dy = final_y - baseline_y
    if abs(du) < 1.0 or abs(dy) < 0.01:
        return None
    target_10 = baseline_y + (0.10 * dy)
    target_63 = baseline_y + (0.632 * dy)

    def crossing(target: float) -> float | None:
        for t, y in zip(times, outputs):
            if (dy >= 0.0 and y >= target) or (dy < 0.0 and y <= target):
                return t
        return None

    t10 = crossing(target_10)
    t63 = crossing(target_63)
    delay = t10 if t10 is not None else 0.0
    tau = max(0.02, (t63 - delay) if t63 is not None else (times[-1] / 3.0))
    gain = dy / du
    kp = max(0.0, min(10.0, 0.35 / max(abs(gain), 0.001)))
    kd = max(0.0, min(5.0, kp * tau * 0.25))
    return {"K": gain, "tau": tau, "L": delay, "kp": kp, "kd": kd}


def normalize_module_key(key: str) -> str | None:
    return MODULE_ALIASES.get(key.upper())


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
        self._send_queue: "queue.Queue[bytes | None]" = queue.Queue()
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        with self.lock:
            return self.client is not None

    def start(self, host: str, port: int) -> None:
        self.stop()
        self._send_queue = queue.Queue()
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
        try:
            self._send_queue.put_nowait(frame)
            return True
        except queue.Full:
            return False

    def _sender_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                frame = self._send_queue.get(timeout=0.3)
            except queue.Empty:
                continue
            if frame is None:
                break
            with self.lock:
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
        self._send_queue: "queue.Queue[bytes | None]" = queue.Queue()
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        with self.lock:
            return self.port is not None and bool(self.port.is_open)

    def start(self, port_name: str, baudrate: int) -> None:
        if not HAS_PYSERIAL or serial is None:
            self.rx_queue.put(f"[上位机] 串口模式不可用: {PYSERIAL_ERROR or '未安装 pyserial'}")
            return
        self.stop()
        self._send_queue = queue.Queue()
        self.stop_event.clear()
        try:
            opened = serial.Serial(port_name, baudrate=baudrate, timeout=0.2, write_timeout=0.5)
        except Exception as exc:
            self.rx_queue.put(f"[上位机] 打开串口失败: {exc}")
            return
        with self.lock:
            self.port = opened
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
        try:
            self._send_queue.put_nowait(frame)
            return True
        except queue.Full:
            return False

    def send_line(self, line: str) -> bool:
        data = (line.rstrip("\r\n") + "\r\n").encode("utf-8")
        with self.lock:
            if self.port is None or not self.port.is_open:
                return False
        try:
            self._send_queue.put_nowait(data)
            return True
        except queue.Full:
            return False

    def _sender_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                frame = self._send_queue.get(timeout=0.3)
            except queue.Empty:
                continue
            if frame is None:
                break
            with self.lock:
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
        if was_current:
            self.rx_queue.put("[上位机] 串口已断开")


class DronePanel(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("drone-H743 地面站")
        self.geometry("1280x820")
        self.minsize(1080, 720)

        self.rx_queue: "queue.Queue[str]" = queue.Queue()
        self.tcp_transport = TcpTransport(self.rx_queue)
        self.udp_transport = UdpTransport(self.rx_queue)
        self.serial_transport = SerialTransport(self.rx_queue)
        self.transport: TransportBase = self.tcp_transport
        self.last_board_rx = 0.0
        self.last_reply_rx = 0.0
        self.selected_module = "FLASH"
        self.baro_capture_enabled = tk.BooleanVar(value=True)
        self.baro_buffer: list[dict[str, float | str]] = []
        self._baro_dirty = False
        self._last_baro_plot_ns = 0
        self.gps_track: list[dict[str, float | str | int]] = []
        self.gps_origin_lat: float | None = None
        self.gps_origin_lon: float | None = None
        self.gps_last_plot_ns = 0
        self.imu_poll_enabled = tk.BooleanVar(value=False)
        self.imu_last_poll = 0.0
        self.imu_last_sample_time = 0.0
        self.imu_last_count = -1
        self.imu_roll_deg = 0.0
        self.imu_pitch_deg = 0.0
        self.imu_yaw_deg = 0.0
        self.params: dict[str, dict[str, str | bool]] = {}
        self.param_iids: dict[str, str] = {}
        self.param_names_by_iid: dict[str, str] = {}
        self.servo_widgets: list[dict[str, tk.Variable]] = []
        self.structured_protocol_supported: bool | None = None
        self.ident_samples: list[dict[str, str | float | int]] = []
        self.ident_csv_file = None
        self.ident_csv_writer: csv.DictWriter | None = None
        self.ident_current_path: Path | None = None
        self.ident_current_meta_path: Path | None = None
        self.ident_last_fit: dict[str, float] | None = None
        self.ident_current_command = ""
        self.airframe_info: dict[str, str | float] = {}
        self.udp_raw_hidden_count = 0
        self.udp_raw_last_note = ""

        self.link_var = tk.StringVar(value="未连接")
        self.last_cmd_var = tk.StringVar(value="-")
        self.config_summary_var = tk.StringVar(value="尚未读取配置")
        self.baro_count_var = tk.StringVar(value="暂存样本: 0")
        self.plot_var = tk.StringVar(value="pressure")
        self.gps_count_var = tk.StringVar(value="轨迹点: 0")
        self.gps_status_var = tk.StringVar(value="等待 GPS")
        self.transport_var = tk.StringVar(value="tcp")
        self.udp_bind_var = tk.StringVar(value=DEFAULT_HOST)
        self.udp_local_port_var = tk.IntVar(value=DEFAULT_UDP_LOCAL_PORT)
        self.udp_module_ip_var = tk.StringVar(value=DEFAULT_MODULE_IP)
        self.udp_module_port_var = tk.IntVar(value=DEFAULT_UDP_MODULE_PORT)
        self._serial_port_map: dict[str, str] = {}
        self.serial_port_var = tk.StringVar(value=self._default_serial_port())
        self.serial_baud_var = tk.IntVar(value=115200)
        self.ident_axis_var = tk.StringVar(value="roll")
        self.ident_mode_var = tk.StringVar(value="STEP")
        self.ident_pulse_var = tk.IntVar(value=20)
        self.ident_duration_var = tk.IntVar(value=3000)
        self.ident_hold_var = tk.IntVar(value=800)
        self.ident_repeat_var = tk.IntVar(value=2)
        self.ident_bit_var = tk.IntVar(value=250)
        self.ident_seed_var = tk.IntVar(value=1)
        self.ident_alpha_center_var = tk.IntVar(value=1500)
        self.ident_beta_center_var = tk.IntVar(value=1500)
        self.ident_status_var = tk.StringVar(value="idle")
        self.ident_sample_count_var = tk.StringVar(value="samples=0")
        self.ident_reason_var = tk.StringVar(value="-")
        self.ident_current_var = tk.StringVar(value="-")
        self.ident_fit_var = tk.StringVar(value="no fit")
        self.ident_airframe_var = tk.StringVar(value="AIRFRAME: not loaded")
        self.ident_save_dir = Path(__file__).resolve().parent / "ident_runs"
        self.ident_output_dir_var = tk.StringVar(value=f"save dir: {self.ident_save_dir}")
        self.ident_last_file_var = tk.StringVar(value="last file: none")
        self.ident_link_var = tk.StringVar(value="UDP text: waiting")
        self.ident_command_preview_var = tk.StringVar(value="")
        self.ident_mode_hint_var = tk.StringVar(value="")
        self.ident_safety_hint_var = tk.StringVar(
            value="建议第一次：STEP，小幅 20us，3s；确认方向后再加到 30-40us。"
        )
        self.ident_field_rows: dict[str, tuple[ttk.Label, ttk.Widget, ttk.Label]] = {}

        self.module_state: dict[str, dict[str, tk.StringVar]] = {}
        self.baro_vars: dict[str, tk.StringVar] = {}
        self.imu_vars: dict[str, tk.StringVar] = {}
        self.gps_vars: dict[str, tk.StringVar] = {}
        self.mag_vars: dict[str, tk.StringVar] = {}

        self._build_ui()
        self.ident_axis_var.trace_add("write", self._on_ident_config_change)
        self.ident_mode_var.trace_add("write", self._on_ident_config_change)
        for var in (
            self.ident_pulse_var,
            self.ident_duration_var,
            self.ident_hold_var,
            self.ident_repeat_var,
            self.ident_bit_var,
            self.ident_seed_var,
            self.ident_alpha_center_var,
            self.ident_beta_center_var,
        ):
            var.trace_add("write", self._on_ident_config_change)
        self._on_ident_config_change()
        self.after(1000, self._check_link_health)
        self.after(100, self._drain_rx)
        self.after(250, self._baro_tick)
        self.after(100, self._imu_poll_tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_port_label(self, p) -> str:
        desc = (p.description or "").upper()
        hwid = (p.hwid or "").upper()
        combined = f"{desc} {hwid}"
        if "CH340" in combined:
            return f"CH340 ({p.device})"
        if "CP210" in combined:
            return f"CP210x ({p.device})"
        if "FTDI" in combined or "FT232" in combined or "FT4232" in combined:
            return f"FTDI ({p.device})"
        if "BLUETOOTH" in combined or "BLE" in combined:
            return f"BLE ({p.device})"
        if "STLINK" in combined or "ST-LINK" in combined:
            return f"STLink ({p.device})"
        if desc and desc not in ("USB SERIAL DEVICE", "USB SERIAL", "SERIAL"):
            return f"{p.device} - {p.description}"
        return p.device

    def _refresh_serial_ports(self) -> list[str]:
        if not HAS_PYSERIAL or serial is None:
            return []
        try:
            ports = list(serial.tools.list_ports.comports())  # type: ignore[attr-defined]
        except Exception:
            return []
        self._serial_port_map.clear()
        names: list[str] = []
        for p in ports:
            label = self._build_port_label(p)
            self._serial_port_map[label] = p.device
            names.append(label)
        return names

    def _default_serial_port(self) -> str:
        names = self._refresh_serial_ports()
        if names:
            return names[0]
        return "COM18"

    def _on_refresh_ports(self) -> None:
        names = self._refresh_serial_ports()
        if hasattr(self, '_serial_port_combo'):
            self._serial_port_combo['values'] = names
        if names:
            self.serial_port_var.set(names[0])

    def _on_transport_mode_change(self, *args) -> None:
        self._tcp_frame.pack_forget()
        self._udp_frame.pack_forget()
        self._serial_frame.pack_forget()

        mode = self.transport_var.get()
        if mode == "serial":
            self._serial_frame.pack(side=tk.LEFT, padx=(0, 12), before=self._action_frame)
            self._on_refresh_ports()
        elif mode == "udp":
            self._udp_frame.pack(side=tk.LEFT, padx=(0, 12), before=self._action_frame)
        else:
            self._tcp_frame.pack(side=tk.LEFT, padx=(0, 12), before=self._action_frame)

    def _current_transport(self) -> TransportBase:
        mode = self.transport_var.get()
        if mode == "serial":
            return self.serial_transport
        if mode == "udp":
            return self.udp_transport
        return self.tcp_transport

    def _transport_connected(self) -> bool:
        return self.transport.is_connected

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        self._build_connection_bar(root)

        body = ttk.PanedWindow(root, orient=tk.VERTICAL)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        self.notebook = ttk.Notebook(body)
        body.add(self.notebook, weight=6)

        overview = ttk.Frame(self.notebook, padding=10)
        baro = ttk.Frame(self.notebook, padding=10)
        imu = ttk.Frame(self.notebook, padding=10)
        gps = ttk.Frame(self.notebook, padding=10)
        ident = ttk.Frame(self.notebook, padding=10)
        params = ttk.Frame(self.notebook, padding=10)
        servos = ttk.Frame(self.notebook, padding=10)
        commands = ttk.Frame(self.notebook, padding=10)
        self.baro_tab = baro
        self.imu_tab = imu
        self.gps_tab = gps
        self.ident_tab = ident

        self.notebook.add(overview, text="模块总览")
        self.notebook.add(baro, text="SPL06 气压计")
        self.notebook.add(imu, text="IMU 姿态")
        self.notebook.add(params, text="参数 / PID")
        self.notebook.add(servos, text="舵机控制")
        self.notebook.add(commands, text="日志 / 原始命令")

        self.notebook.add(gps, text="GPS 轨迹")

        self.notebook.add(ident, text="系统辨识")

        self._build_overview_page(overview)
        self._build_baro_page(baro)
        self._build_imu_page(imu)
        self._build_gps_page(gps)
        self._build_ident_page(ident)
        self._build_params_page(params)
        self._build_servo_page(servos)
        self._build_command_page(commands)

        log_box = ttk.LabelFrame(body, text="原始命令日志", padding=8)
        body.add(log_box, weight=2)
        self._build_log_area(log_box)

    def _build_connection_bar(self, parent: ttk.Frame) -> None:
        conn = ttk.Frame(parent)
        conn.pack(fill=tk.X)

        ttk.Label(conn, text="通道").pack(side=tk.LEFT)
        ttk.Combobox(
            conn,
            textvariable=self.transport_var,
            values=("tcp", "udp", "serial"),
            width=8,
            state="readonly",
        ).pack(side=tk.LEFT, padx=(4, 12))

        # --- TCP controls ---
        self._tcp_frame = ttk.Frame(conn)
        ttk.Label(self._tcp_frame, text="监听地址").pack(side=tk.LEFT)
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(self._tcp_frame, textvariable=self.host_var, width=16).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(self._tcp_frame, text="端口").pack(side=tk.LEFT)
        self.port_var = tk.IntVar(value=DEFAULT_PORT)
        ttk.Entry(self._tcp_frame, textvariable=self.port_var, width=8).pack(side=tk.LEFT)

        # --- UDP controls ---
        self._udp_frame = ttk.Frame(conn)
        ttk.Label(self._udp_frame, text="本地").pack(side=tk.LEFT)
        ttk.Entry(self._udp_frame, textvariable=self.udp_bind_var, width=13).pack(side=tk.LEFT, padx=(4, 4))
        ttk.Entry(self._udp_frame, textvariable=self.udp_local_port_var, width=7).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(self._udp_frame, text="模块").pack(side=tk.LEFT)
        ttk.Entry(self._udp_frame, textvariable=self.udp_module_ip_var, width=15).pack(side=tk.LEFT, padx=(4, 4))
        ttk.Entry(self._udp_frame, textvariable=self.udp_module_port_var, width=7).pack(side=tk.LEFT)

        # --- Serial controls (hidden by default) ---
        self._serial_frame = ttk.Frame(conn)
        ttk.Label(self._serial_frame, text="串口").pack(side=tk.LEFT)
        self._serial_port_combo = ttk.Combobox(
            self._serial_frame,
            textvariable=self.serial_port_var,
            values=self._refresh_serial_ports(),
            width=20,
        )
        self._serial_port_combo.pack(side=tk.LEFT, padx=(4, 2))
        ttk.Button(self._serial_frame, text="刷新", command=self._on_refresh_ports).pack(side=tk.LEFT)
        ttk.Label(self._serial_frame, text="波特率").pack(side=tk.LEFT)
        ttk.Entry(self._serial_frame, textvariable=self.serial_baud_var, width=8).pack(side=tk.LEFT, padx=(4, 0))

        # Show TCP frame initially
        self._tcp_frame.pack(side=tk.LEFT, padx=(0, 12))

        # --- Action buttons (always visible) ---
        self._action_frame = ttk.Frame(conn)
        self._action_frame.pack(side=tk.LEFT)
        ttk.Button(self._action_frame, text="启动连接", command=self._start).pack(side=tk.LEFT)
        ttk.Button(self._action_frame, text="停止", command=self._stop).pack(side=tk.LEFT, padx=6)
        ttk.Button(self._action_frame, text="PING", command=lambda: self._send_proto(PROTO_REQ_PING, "PING")).pack(side=tk.LEFT, padx=(18, 4))
        ttk.Button(self._action_frame, text="硬件状态", command=self._request_overview_status).pack(side=tk.LEFT, padx=4)
        ttk.Button(self._action_frame, text="读取配置", command=self._read_all_params).pack(side=tk.LEFT, padx=4)
        ttk.Button(self._action_frame, text="保存到 Flash", command=lambda: self._send_proto_once(PROTO_REQ_SAVE, "SAVE")).pack(side=tk.LEFT, padx=4)
        ttk.Button(self._action_frame, text="从 Flash 读取", command=lambda: self._send_proto_once(PROTO_REQ_LOAD, "LOAD")).pack(side=tk.LEFT, padx=4)

        ttk.Label(conn, text="链路").pack(side=tk.LEFT, padx=(18, 4))
        ttk.Label(conn, textvariable=self.link_var).pack(side=tk.LEFT)

        self.transport_var.trace_add("write", self._on_transport_mode_change)

    def _build_overview_page(self, parent: ttk.Frame) -> None:
        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=3)
        panes.add(right, weight=2)

        toolbar = ttk.Frame(left)
        toolbar.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(toolbar, text="刷新全部", command=self._request_overview_status).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="读取 UART 统计", command=lambda: self._send_proto(PROTO_REQ_STATUS, "STATUS?")).pack(side=tk.LEFT, padx=6)
        ttk.Label(toolbar, textvariable=self.last_cmd_var).pack(side=tk.LEFT, padx=(16, 0))

        columns = ("state", "stage", "value", "code", "hint")
        self.module_tree = ttk.Treeview(left, columns=columns, show="tree headings", height=10)
        self.module_tree.heading("#0", text="模块")
        self.module_tree.column("#0", width=160, anchor=tk.W)
        headings = {
            "state": "状态",
            "stage": "失败阶段",
            "value": "关键读数",
            "code": "返回码",
            "hint": "提示",
        }
        widths = {"state": 80, "stage": 120, "value": 230, "code": 170, "hint": 360}
        for col, label in headings.items():
            self.module_tree.heading(col, text=label)
            self.module_tree.column(col, width=widths[col], anchor=tk.W)

        self.module_tree.pack(fill=tk.BOTH, expand=True)
        self.module_tree.bind("<<TreeviewSelect>>", self._on_module_select)

        for key, title, _command in MODULES:
            values = {
                "title": tk.StringVar(value=title),
                "state": tk.StringVar(value="等待数据"),
                "stage": tk.StringVar(value="-"),
                "value": tk.StringVar(value="-"),
                "code": tk.StringVar(value="-"),
                "hint": tk.StringVar(value="点击“硬件状态”或等待心跳"),
                "last": tk.StringVar(value="-"),
            }
            self.module_state[key] = values
            self.module_tree.insert(
                "",
                tk.END,
                iid=key,
                values=(values["state"].get(), values["stage"].get(), values["value"].get(), values["code"].get(), values["hint"].get()),
                text=title,
            )
        self.module_tree.selection_set("FLASH")

        detail = ttk.LabelFrame(right, text="模块详情", padding=10)
        detail.pack(fill=tk.BOTH, expand=True)
        self.detail_vars = {
            "title": tk.StringVar(value="GD25Q32 Flash"),
            "state": tk.StringVar(value="等待数据"),
            "stage": tk.StringVar(value="-"),
            "value": tk.StringVar(value="-"),
            "code": tk.StringVar(value="-"),
            "hint": tk.StringVar(value="-"),
            "last": tk.StringVar(value="-"),
        }
        self._detail_row(detail, 0, "模块", self.detail_vars["title"])
        self._detail_row(detail, 1, "状态", self.detail_vars["state"])
        self._detail_row(detail, 2, "失败阶段", self.detail_vars["stage"])
        self._detail_row(detail, 3, "关键读数", self.detail_vars["value"])
        self._detail_row(detail, 4, "返回码", self.detail_vars["code"])
        self._detail_row(detail, 5, "提示", self.detail_vars["hint"], wrap=420)
        self._detail_row(detail, 6, "最近原始行", self.detail_vars["last"], wrap=420)

        actions = ttk.Frame(detail)
        actions.grid(row=7, column=0, columnspan=2, sticky=tk.EW, pady=(12, 0))
        ttk.Button(actions, text="请求该模块详情", command=self._request_selected_module).pack(side=tk.LEFT)
        ttk.Button(actions, text="打开气压计页", command=self._open_baro_tab).pack(side=tk.LEFT, padx=6)
        ttk.Button(actions, text="打开姿态页", command=self._open_imu_tab).pack(side=tk.LEFT)
        ttk.Button(actions, text="打开 GPS 页", command=self._open_gps_tab).pack(side=tk.LEFT, padx=6)
        detail.columnconfigure(1, weight=1)

    def _detail_row(self, parent: ttk.Frame, row: int, label: str, variable: tk.StringVar, wrap: int = 0) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.NW, padx=(0, 8), pady=4)
        ttk.Label(parent, textvariable=variable, wraplength=wrap).grid(row=row, column=1, sticky=tk.EW, pady=4)

    def _build_baro_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="请求状态", command=lambda: self._send_proto(PROTO_REQ_STATUS, "STATUS?")).pack(side=tk.LEFT)
        ttk.Button(top, text="请求气压计", command=lambda: self._send_proto(PROTO_REQ_BARO, "BARO?")).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="开始流", command=lambda: self._send_proto(PROTO_REQ_BARO_STREAM, f"BARO STREAM 1 {BARO_STREAM_PERIOD_MS}", f"BARO STREAM 1 {BARO_STREAM_PERIOD_MS}")).pack(side=tk.LEFT)
        ttk.Button(top, text="停止流", command=lambda: self._send_proto(PROTO_REQ_BARO_STREAM, "BARO STREAM 0", "BARO STREAM 0")).pack(side=tk.LEFT, padx=6)
        ttk.Checkbutton(top, text="暂存新数据", variable=self.baro_capture_enabled).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Button(top, text="清空暂存", command=self._clear_baro_buffer).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="导出 CSV", command=self._export_baro_csv).pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.baro_count_var).pack(side=tk.LEFT, padx=(12, 0))

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=2)
        panes.add(right, weight=3)

        fields = [
            ("state", "状态"),
            ("pressure", "压力"),
            ("temperature", "温度"),
            ("altitude", "高度"),
            ("raw_pressure", "压力原始值"),
            ("raw_temperature", "温度原始值"),
            ("sample_count", "样本计数"),
            ("product_id", "Product ID"),
            ("split_id", "Split ID"),
            ("txrx_id", "TxRx ID"),
            ("bmp_id", "BMP280 ID"),
            ("init_status", "init 返回码"),
            ("split_status", "split 返回码"),
            ("txrx_status", "txrx 返回码"),
            ("cs_level", "CS 电平"),
            ("miso_level", "MISO 电平"),
            ("stage", "失败阶段"),
            ("last_update", "最近更新"),
        ]
        diagnostic = ttk.LabelFrame(left, text="实时 / 诊断字段", padding=10)
        diagnostic.pack(fill=tk.X)
        for row, (key, label) in enumerate(fields):
            self.baro_vars[key] = tk.StringVar(value="-")
            ttk.Label(diagnostic, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 8), pady=2)
            ttk.Label(diagnostic, textvariable=self.baro_vars[key]).grid(row=row, column=1, sticky=tk.W, pady=2)
        diagnostic.columnconfigure(1, weight=1)

        sample_box = ttk.LabelFrame(left, text="暂存数据预览", padding=8)
        sample_box.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self.baro_sample_tree = ttk.Treeview(sample_box, columns=("t", "p", "temp", "alt"), show="headings", height=8)
        for col, label, width in [
            ("t", "t(s)", 70),
            ("p", "pressure", 100),
            ("temp", "temp", 80),
            ("alt", "alt", 80),
        ]:
            self.baro_sample_tree.heading(col, text=label)
            self.baro_sample_tree.column(col, width=width, anchor=tk.W)
        self.baro_sample_tree.pack(fill=tk.BOTH, expand=True)

        plot_box = ttk.LabelFrame(right, text="曲线", padding=8)
        plot_box.pack(fill=tk.BOTH, expand=True)
        controls = ttk.Frame(plot_box)
        controls.pack(fill=tk.X)
        ttk.Label(controls, text="字段").pack(side=tk.LEFT)
        ttk.Combobox(
            controls,
            textvariable=self.plot_var,
            values=("pressure", "temperature", "altitude"),
            width=14,
            state="readonly",
        ).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="刷新曲线", command=self._update_baro_plot).pack(side=tk.LEFT)

        if HAS_MATPLOTLIB and Figure is not None and FigureCanvasTkAgg is not None:
            self.baro_figure = Figure(figsize=(5, 3), dpi=100)
            self.baro_axis = self.baro_figure.add_subplot(111)
            self.baro_canvas = FigureCanvasTkAgg(self.baro_figure, master=plot_box)
            self.baro_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(8, 0))
            self._update_baro_plot()
        else:
            ttk.Label(
                plot_box,
                text=f"未安装 matplotlib，曲线区停用。安装后重启面板即可绘图。\npython -m pip install matplotlib\n{MATPLOTLIB_ERROR}",
                wraplength=520,
            ).pack(fill=tk.X, pady=(12, 0))
            self.baro_figure = None
            self.baro_axis = None
            self.baro_canvas = None

    def _build_imu_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="请求 IMU", command=lambda: self._send_proto(PROTO_REQ_IMU, "IMU?")).pack(side=tk.LEFT)
        ttk.Checkbutton(top, text=f"备用轮询 {IMU_POLL_PERIOD_MS} ms", variable=self.imu_poll_enabled).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(top, text="姿态归零", command=self._reset_imu_attitude).pack(side=tk.LEFT, padx=8)

        body = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        attitude = ttk.LabelFrame(body, text="人工地平仪", padding=8)
        values = ttk.LabelFrame(body, text="实时数值", padding=10)
        body.add(attitude, weight=3)
        body.add(values, weight=2)

        self.imu_canvas = tk.Canvas(attitude, width=520, height=380, bg="#101820", highlightthickness=0)
        self.imu_canvas.pack(fill=tk.BOTH, expand=True)
        self.imu_canvas.bind("<Configure>", lambda _event: self._draw_imu_attitude())

        fields = [
            ("roll", "Roll deg"),
            ("pitch", "Pitch deg"),
            ("yaw", "Yaw deg"),
            ("ax", "Accel X mg"),
            ("ay", "Accel Y mg"),
            ("az", "Accel Z mg"),
            ("gx", "Gyro X mdps"),
            ("gy", "Gyro Y mdps"),
            ("gz", "Gyro Z mdps"),
            ("temp", "Temp cdeg"),
            ("count", "Sample"),
            ("who", "WHO_AM_I"),
            ("age", "Age"),
        ]
        for row, (key, label) in enumerate(fields):
            self.imu_vars[key] = tk.StringVar(value="-")
            ttk.Label(values, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 10), pady=3)
            ttk.Label(values, textvariable=self.imu_vars[key]).grid(row=row, column=1, sticky=tk.W, pady=3)
        values.columnconfigure(1, weight=1)
        self._draw_imu_attitude()

    def _build_gps_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="请求 GPS", command=lambda: self._send_proto(PROTO_REQ_GPS, "GPS?")).pack(side=tk.LEFT)
        ttk.Button(top, text="请求磁力计", command=lambda: self._send_proto(PROTO_REQ_MAG, "MAG?")).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="清空轨迹", command=self._clear_gps_track).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="导出轨迹 CSV", command=self._export_gps_csv).pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.gps_count_var).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(top, textvariable=self.gps_status_var).pack(side=tk.LEFT, padx=(12, 0))

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=3)
        panes.add(right, weight=2)

        plot_box = ttk.LabelFrame(left, text="XY 轨迹图 (m)", padding=8)
        plot_box.pack(fill=tk.BOTH, expand=True)

        if HAS_MATPLOTLIB and Figure is not None and FigureCanvasTkAgg is not None:
            self.gps_figure = Figure(figsize=(6, 4), dpi=100)
            self.gps_axis = self.gps_figure.add_subplot(111)
            self.gps_canvas = FigureCanvasTkAgg(self.gps_figure, master=plot_box)
            self.gps_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            self._update_gps_plot()
        else:
            ttk.Label(
                plot_box,
                text=f"未安装 matplotlib，轨迹图区停用。\npython -m pip install matplotlib\n{MATPLOTLIB_ERROR}",
                wraplength=560,
            ).pack(fill=tk.X, pady=(12, 0))
            self.gps_figure = None
            self.gps_axis = None
            self.gps_canvas = None

        info = ttk.LabelFrame(right, text="GPS / MAG 状态", padding=10)
        info.pack(fill=tk.BOTH, expand=True)

        gps_fields = [
            ("state", "GPS 状态"),
            ("fix", "Fix"),
            ("sv", "卫星数"),
            ("lon", "经度"),
            ("lat", "纬度"),
            ("hmsl", "海拔 mm"),
            ("x", "X (m)"),
            ("y", "Y (m)"),
            ("spd", "地速 mm/s"),
            ("hdg", "航向 deg"),
            ("age", "数据时延"),
        ]
        for row, (key, label) in enumerate(gps_fields):
            self.gps_vars[key] = tk.StringVar(value="-")
            ttk.Label(info, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 8), pady=2)
            ttk.Label(info, textvariable=self.gps_vars[key]).grid(row=row, column=1, sticky=tk.W, pady=2)

        mag_title = ttk.Label(info, text="MAG", font=("Segoe UI", 10, "bold"))
        mag_title.grid(row=len(gps_fields), column=0, columnspan=2, sticky=tk.W, pady=(10, 2))

        mag_fields = [
            ("state", "磁力计状态"),
            ("type", "型号"),
            ("raw", "Raw xyz"),
            ("scaled", "mGauss xyz"),
        ]
        base = len(gps_fields) + 1
        for row, (key, label) in enumerate(mag_fields):
            self.mag_vars[key] = tk.StringVar(value="-")
            ttk.Label(info, text=label).grid(row=base + row, column=0, sticky=tk.W, padx=(0, 8), pady=2)
            ttk.Label(info, textvariable=self.mag_vars[key]).grid(row=base + row, column=1, sticky=tk.W, pady=2)

        info.columnconfigure(1, weight=1)

    def _build_ident_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="ARM 辨识", command=lambda: self._send_proto(PROTO_REQ_IDENT, "IDENT ARM")).pack(side=tk.LEFT)
        ttk.Button(top, text="DISARM", command=lambda: self._send_proto(PROTO_REQ_IDENT, "IDENT DISARM")).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="STOP", command=lambda: self._send_proto(PROTO_REQ_IDENT, "IDENT STOP")).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="STATUS", command=lambda: self._send_proto(PROTO_REQ_IDENT, "IDENT?")).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="AIRFRAME", command=self._request_airframe).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, text="状态").pack(side=tk.LEFT, padx=(18, 4))
        ttk.Label(top, textvariable=self.ident_status_var).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, textvariable=self.ident_sample_count_var).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, textvariable=self.ident_reason_var).pack(side=tk.LEFT, padx=4)

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=2)
        panes.add(right, weight=3)

        cfg = ttk.LabelFrame(left, text="Experiment", padding=10)
        cfg.pack(fill=tk.X)
        cfg.columnconfigure(1, weight=1)
        ttk.Label(cfg, text="辨识轴").grid(row=0, column=0, sticky=tk.W, pady=3)
        ttk.Combobox(cfg, textvariable=self.ident_axis_var, values=("roll", "pitch"), width=12, state="readonly").grid(row=0, column=1, sticky=tk.W, pady=3)
        ttk.Label(cfg, text="波形").grid(row=1, column=0, sticky=tk.W, pady=3)
        ttk.Combobox(cfg, textvariable=self.ident_mode_var, values=("STEP", "DOUBLET", "PRBS"), width=12, state="readonly").grid(row=1, column=1, sticky=tk.W, pady=3)

        presets = ttk.Frame(cfg)
        presets.grid(row=2, column=0, columnspan=3, sticky=tk.EW, pady=(6, 8))
        ttk.Button(presets, text="小幅首测", command=lambda: self._ident_apply_preset("small")).pack(side=tk.LEFT)
        ttk.Button(presets, text="标准阶跃", command=lambda: self._ident_apply_preset("step")).pack(side=tk.LEFT, padx=4)
        ttk.Button(presets, text="标准双脉冲", command=lambda: self._ident_apply_preset("doublet")).pack(side=tk.LEFT)

        field_specs = [
            ("pulse", "舵机偏置", self.ident_pulse_var, "us，正负都可；越大响应越明显"),
            ("duration", "阶跃/PRBS 时长", self.ident_duration_var, "ms，最大 10000"),
            ("hold", "双脉冲保持", self.ident_hold_var, "ms，每段保持时间"),
            ("repeat", "双脉冲次数", self.ident_repeat_var, "次，默认 2"),
            ("bit", "PRBS 位宽", self.ident_bit_var, "ms，随机输入切换间隔"),
            ("seed", "PRBS seed", self.ident_seed_var, "相同 seed 可复现实验"),
            ("alpha_center", "alpha 中位", self.ident_alpha_center_var, "us，roll/1号左右舵机中心"),
            ("beta_center", "beta 中位", self.ident_beta_center_var, "us，pitch/2号前后舵机中心"),
        ]
        self.ident_field_rows.clear()
        for index, (key, label, var, hint) in enumerate(field_specs, start=3):
            label_widget = ttk.Label(cfg, text=label)
            entry = ttk.Entry(cfg, textvariable=var, width=12)
            hint_widget = ttk.Label(cfg, text=hint, foreground="#555555")
            label_widget.grid(row=index, column=0, sticky=tk.W, pady=3)
            entry.grid(row=index, column=1, sticky=tk.W, pady=3)
            hint_widget.grid(row=index, column=2, sticky=tk.W, padx=(8, 0), pady=3)
            self.ident_field_rows[key] = (label_widget, entry, hint_widget)

        ttk.Label(cfg, textvariable=self.ident_mode_hint_var, wraplength=520, foreground="#333333").grid(
            row=11, column=0, columnspan=3, sticky=tk.EW, pady=(8, 2)
        )
        ttk.Label(cfg, textvariable=self.ident_command_preview_var, wraplength=520, font=("Consolas", 9)).grid(
            row=12, column=0, columnspan=3, sticky=tk.EW, pady=(2, 6)
        )
        ttk.Button(cfg, text="Set Center", command=self._ident_send_center).grid(row=13, column=0, sticky=tk.EW, pady=(4, 0))
        ttk.Button(cfg, text="Run", command=self._ident_run).grid(row=13, column=1, sticky=tk.EW, pady=(4, 0))
        ttk.Label(cfg, textvariable=self.ident_safety_hint_var, wraplength=520, foreground="#7A3E00").grid(
            row=14, column=0, columnspan=3, sticky=tk.EW, pady=(8, 0)
        )

        fit = ttk.LabelFrame(left, text="Fit / PID", padding=10)
        fit.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(fit, textvariable=self.ident_fit_var, wraplength=320).pack(fill=tk.X)
        actions = ttk.Frame(fit)
        actions.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(actions, text="Fit", command=self._ident_fit).pack(side=tk.LEFT)
        ttk.Button(actions, text="Apply", command=self._ident_apply_fit).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="Save", command=lambda: self._send_proto_once(PROTO_REQ_SAVE, "SAVE")).pack(side=tk.LEFT)

        live = ttk.LabelFrame(left, text="Live", padding=10)
        live.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(live, textvariable=self.ident_link_var, wraplength=520).pack(fill=tk.X, pady=(0, 4))
        ttk.Label(live, textvariable=self.ident_airframe_var, wraplength=320).pack(fill=tk.X, pady=(0, 8))
        ttk.Label(live, textvariable=self.ident_current_var, wraplength=320).pack(fill=tk.X)
        ttk.Label(live, textvariable=self.ident_output_dir_var, wraplength=520).pack(fill=tk.X, pady=(8, 0))
        ttk.Label(live, textvariable=self.ident_last_file_var, wraplength=520).pack(fill=tk.X, pady=(2, 0))
        actions_live = ttk.Frame(live)
        actions_live.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(actions_live, text="Open CSV Folder", command=self._ident_open_folder).pack(side=tk.LEFT)
        ttk.Button(actions_live, text="Pause VOFA", command=lambda: self._send("Sensor_Data:0")).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions_live, text="Resume VOFA", command=lambda: self._send("Sensor_Data:1")).pack(side=tk.LEFT)

        plot_box = ttk.LabelFrame(right, text="Input / Response", padding=8)
        plot_box.pack(fill=tk.BOTH, expand=True)
        if HAS_MATPLOTLIB and Figure is not None and FigureCanvasTkAgg is not None:
            self.ident_figure = Figure(figsize=(6, 4), dpi=100)
            self.ident_axis_plot = self.ident_figure.add_subplot(111)
            self.ident_canvas = FigureCanvasTkAgg(self.ident_figure, master=plot_box)
            self.ident_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        else:
            ttk.Label(
                plot_box,
                text=f"matplotlib unavailable; CSV logging still works.\npython -m pip install matplotlib\n{MATPLOTLIB_ERROR}",
                wraplength=560,
            ).pack(fill=tk.X, pady=(12, 0))
            self.ident_figure = None
            self.ident_axis_plot = None
            self.ident_canvas = None

    def _build_params_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="读取参数/PID", command=self._read_all_params).pack(side=tk.LEFT)
        ttk.Button(top, text="保存到 Flash", command=lambda: self._send_proto_once(PROTO_REQ_SAVE, "SAVE")).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="从 Flash 读取", command=lambda: self._send_proto_once(PROTO_REQ_LOAD, "LOAD")).pack(side=tk.LEFT)
        ttk.Button(top, text="恢复默认", command=lambda: self._send_proto(PROTO_REQ_DEFAULTS, "DEFAULTS")).pack(side=tk.LEFT, padx=6)
        ttk.Label(top, textvariable=self.config_summary_var).pack(side=tk.LEFT, padx=(14, 0))

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=3)
        panes.add(right, weight=2)

        self.param_tree = ttk.Treeview(left, columns=("value", "source", "dirty"), show="tree headings")
        self.param_tree.heading("#0", text="参数")
        self.param_tree.column("#0", width=220, anchor=tk.W)
        for col, label, width in [("value", "值", 150), ("source", "来源", 100), ("dirty", "待发送", 80)]:
            self.param_tree.heading(col, text=label)
            self.param_tree.column(col, width=width, anchor=tk.W)
        self.param_tree.pack(fill=tk.BOTH, expand=True)
        self.param_tree.bind("<<TreeviewSelect>>", self._on_param_select)

        edit = ttk.LabelFrame(right, text="参数编辑", padding=10)
        edit.pack(fill=tk.X)
        self.param_name_var = tk.StringVar()
        self.param_value_var = tk.StringVar()
        ttk.Label(edit, text="名称").grid(row=0, column=0, sticky=tk.W, pady=4)
        ttk.Entry(edit, textvariable=self.param_name_var).grid(row=0, column=1, sticky=tk.EW, pady=4)
        ttk.Label(edit, text="值").grid(row=1, column=0, sticky=tk.W, pady=4)
        ttk.Entry(edit, textvariable=self.param_value_var).grid(row=1, column=1, sticky=tk.EW, pady=4)
        actions = ttk.Frame(edit)
        actions.grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))
        ttk.Button(actions, text="暂存修改", command=self._stage_param_edit).pack(side=tk.LEFT)
        ttk.Button(actions, text="发送修改", command=self._send_param_edit).pack(side=tk.LEFT, padx=6)
        edit.columnconfigure(1, weight=1)

        pid = ttk.LabelFrame(right, text="PID 快速编辑", padding=10)
        pid.pack(fill=tk.X, pady=(10, 0))
        self.pid_vars: dict[str, dict[str, tk.StringVar]] = {}
        ttk.Label(pid, text="轴").grid(row=0, column=0, sticky=tk.W)
        for col, term in enumerate(("kp", "ki", "kd"), start=1):
            ttk.Label(pid, text=term.upper()).grid(row=0, column=col, sticky=tk.W)
        for row, axis in enumerate(("roll", "pitch", "yaw"), start=1):
            ttk.Label(pid, text=axis).grid(row=row, column=0, sticky=tk.W, pady=4)
            self.pid_vars[axis] = {}
            for col, term in enumerate(("kp", "ki", "kd"), start=1):
                var = tk.StringVar(value="")
                self.pid_vars[axis][term] = var
                ttk.Entry(pid, textvariable=var, width=10).grid(row=row, column=col, sticky=tk.EW, padx=(4, 0), pady=4)
        ttk.Button(pid, text="发送 PID", command=self._send_pid_values).grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))
        ttk.Button(pid, text="读取 PID", command=lambda: self._send_proto(PROTO_REQ_PID, "PID?")).grid(row=4, column=2, columnspan=2, sticky=tk.EW, padx=(6, 0), pady=(8, 0))
        for col in range(1, 4):
            pid.columnconfigure(col, weight=1)

    def _build_servo_page(self, parent: ttk.Frame) -> None:
        servo_notebook = ttk.Notebook(parent)
        servo_notebook.pack(fill=tk.BOTH, expand=True)
        for index in range(2):
            frame = ttk.Frame(servo_notebook, padding=10)
            servo_notebook.add(frame, text=f"舵机 {index}")
            self._build_servo_tab(frame, index)

        raw = ttk.LabelFrame(parent, text="手动原始舵机指令", padding=10)
        raw.pack(fill=tk.X, pady=(10, 0))
        self.raw_var = tk.StringVar(value="{#001P1500T0500!#002P1500T0500!}")
        ttk.Entry(raw, textvariable=self.raw_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(raw, text="发送原始指令", command=self._send_raw).pack(side=tk.LEFT, padx=(6, 0))

    def _build_command_page(self, parent: ttk.Frame) -> None:
        quick = ttk.LabelFrame(parent, text="兼容 / 调试命令", padding=10)
        quick.pack(fill=tk.X)
        for label, command in [
            ("PING", "PING"),
            ("STATUS?", "STATUS?"),
            ("CONFIG?", "CONFIG?"),
            ("PARAM?", "PARAM?"),
            ("AIRFRAME?", "AIRFRAME?"),
            ("PID?", "PID?"),
            ("BARO?", "BARO?"),
            ("GPS?", "GPS?"),
            ("MAG?", "MAG?"),
            ("SAVE", "SAVE"),
            ("LOAD", "LOAD"),
        ]:
            ttk.Button(quick, text=label, command=lambda c=command: self._send(c)).pack(side=tk.LEFT, padx=3)

        help_box = ttk.LabelFrame(parent, text="兼容格式", padding=10)
        help_box.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        text = tk.Text(help_box, height=10, wrap=tk.WORD)
        text.pack(fill=tk.BOTH, expand=True)
        text.insert(
            tk.END,
            "面板会解析现有固件行: READY, HW FLASH/SPL06/ICM42688, STATUS flash/baro/imu, UART1, CFG, OK/ERR。\n"
            "也预留解析: BARO pressure=... temp=... alt=..., PARAM name=... value=..., PID axis=roll kp=...。\n"
            "未知行不会报错，会保留在原始命令日志里，方便固件侧逐步补命令。\n",
        )
        text.configure(state=tk.DISABLED)

    def _build_log_area(self, parent: ttk.Frame) -> None:
        text_frame = ttk.Frame(parent)
        text_frame.pack(fill=tk.BOTH, expand=True)
        self.status_text = tk.Text(text_frame, height=9, wrap=tk.NONE)
        self.status_text.configure(font=("Consolas", 10))
        yscroll = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        xscroll = ttk.Scrollbar(text_frame, orient=tk.HORIZONTAL, command=self.status_text.xview)
        self.status_text.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        self.status_text.grid(row=0, column=0, sticky=tk.NSEW)
        yscroll.grid(row=0, column=1, sticky=tk.NS)
        xscroll.grid(row=1, column=0, sticky=tk.EW)
        text_frame.rowconfigure(0, weight=1)
        text_frame.columnconfigure(0, weight=1)

        cmd_frame = ttk.Frame(parent)
        cmd_frame.pack(fill=tk.X, pady=(8, 0))
        self.cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(cmd_frame, textvariable=self.cmd_var)
        cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        cmd_entry.bind("<Return>", lambda _event: self._send_custom())
        ttk.Button(cmd_frame, text="发送", command=self._send_custom).pack(side=tk.LEFT, padx=(6, 0))

    def _build_servo_tab(self, parent: ttk.Frame, index: int) -> None:
        values: dict[str, tk.Variable] = {
            "id": tk.IntVar(value=index + 1),
            "pulse": tk.IntVar(value=1500),
            "time": tk.IntVar(value=500),
            "mode": tk.IntVar(value=1),
            "enabled": tk.IntVar(value=1),
            "new_id": tk.IntVar(value=index + 1),
            "baud": tk.IntVar(value=4),
        }
        self.servo_widgets.append(values)

        row = 0
        ttk.Checkbutton(
            parent,
            text="启用此舵机槽位",
            variable=values["enabled"],
            command=lambda i=index: self._servo_enable(i),
        ).grid(row=row, column=0, sticky=tk.W)
        row += 1
        self._spin(parent, row, "当前舵机 ID", values["id"], 0, 255, lambda i=index: self._servo_set_id(i))
        row += 1
        self._scale(parent, row, "目标位置 us", values["pulse"], 500, 2500)
        row += 1
        self._spin(parent, row, "运行时间 ms", values["time"], 0, 9999, None)
        row += 1
        self._spin(parent, row, "模式 1-8", values["mode"], 1, 8, lambda i=index: self._servo_mode(i))
        row += 1
        ttk.Button(parent, text="移动此舵机", command=lambda i=index: self._servo_move(i)).grid(row=row, column=0, pady=6, sticky=tk.EW)
        ttk.Button(
            parent,
            text="按配置同时移动两路",
            command=lambda: self._send_proto(PROTO_REQ_SERVO_MOVE_ALL, "SERVO MOVEALL"),
        ).grid(row=row, column=1, pady=6, sticky=tk.EW)
        row += 1

        id_box = ttk.LabelFrame(parent, text="修改实体舵机 ID", padding=8)
        id_box.grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=(8, 4))
        ttk.Spinbox(id_box, from_=0, to=255, textvariable=values["new_id"], width=8).pack(side=tk.LEFT)
        ttk.Button(id_box, text="写入新 ID", command=lambda i=index: self._servo_set_physical_id(i)).pack(side=tk.LEFT, padx=6)
        row += 1

        actions = ttk.LabelFrame(parent, text="众灵手册动作指令", padding=8)
        actions.grid(row=row, column=0, columnspan=2, sticky=tk.EW)
        action_names = [
            ("读取版本", "VER"),
            ("检测 ID", "PID"),
            ("读取位置", "RAD"),
            ("读取模式", "MOD?"),
            ("释放扭力", "ULK"),
            ("恢复扭力", "ULR"),
            ("暂停", "DPT"),
            ("继续", "DCT"),
            ("停止", "DST"),
            ("当前位置设中位", "SCK"),
            ("设置启动位置", "CSD"),
            ("清除启动位置", "CSM"),
            ("恢复启动位置", "CSR"),
            ("设置最小值", "SMI"),
            ("设置最大值", "SMX"),
            ("半恢复出厂", "CLEO"),
            ("全恢复出厂", "CLE"),
        ]
        for n, (label, command) in enumerate(action_names):
            ttk.Button(actions, text=label, command=lambda i=index, c=command: self._servo_cmd(i, c)).grid(
                row=n // 2,
                column=n % 2,
                padx=3,
                pady=3,
                sticky=tk.EW,
            )
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)

        baud_box = ttk.Frame(parent)
        baud_box.grid(row=row + 1, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))
        ttk.Label(baud_box, text="波特率代码").pack(side=tk.LEFT)
        ttk.Spinbox(baud_box, from_=0, to=7, textvariable=values["baud"], width=5).pack(side=tk.LEFT, padx=6)
        ttk.Button(baud_box, text="设置波特率", command=lambda i=index: self._servo_baud(i)).pack(side=tk.LEFT)

        parent.columnconfigure(1, weight=1)

    def _spin(self, parent: ttk.Frame, row: int, label: str, variable: tk.Variable, minimum: int, maximum: int, command) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        box = ttk.Spinbox(parent, from_=minimum, to=maximum, textvariable=variable, width=10)
        box.grid(row=row, column=1, sticky=tk.EW, pady=4)
        if command is not None:
            ttk.Button(parent, text="应用", command=command).grid(row=row, column=2, padx=(6, 0))

    def _scale(self, parent: ttk.Frame, row: int, label: str, variable: tk.Variable, minimum: int, maximum: int) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        scale = ttk.Scale(parent, from_=minimum, to=maximum, variable=variable, orient=tk.HORIZONTAL)
        scale.grid(row=row, column=1, sticky=tk.EW, pady=4)
        ttk.Spinbox(parent, from_=minimum, to=maximum, textvariable=variable, width=8).grid(row=row, column=2, padx=(6, 0))

    def _start(self) -> None:
        self._stop()
        self.transport = self._current_transport()
        if self.transport is self.tcp_transport:
            try:
                port = int(self.port_var.get())
            except tk.TclError:
                messagebox.showerror("输入错误", "端口号无效")
                return
            self.transport.start(self.host_var.get(), port)
            return

        if self.transport is self.udp_transport:
            try:
                local_port = int(self.udp_local_port_var.get())
                module_port = int(self.udp_module_port_var.get())
            except tk.TclError:
                messagebox.showerror("输入错误", "UDP 端口号无效")
                return
            module_ip = self.udp_module_ip_var.get().strip()
            if not module_ip:
                messagebox.showerror("输入错误", "请输入 Ai-WB2 模块 IP")
                return
            self.transport.start(self.udp_bind_var.get(), local_port, module_ip, module_port)
            self.structured_protocol_supported = False
            self.after(250, lambda: self.transport.send_line("PING") if self.transport is self.udp_transport else None)
            self.after(450, lambda: self.transport.send_line("AIRFRAME?") if self.transport is self.udp_transport else None)
            return

        try:
            baud = int(self.serial_baud_var.get())
        except tk.TclError:
            messagebox.showerror("输入错误", "波特率无效")
            return
        port_display = self.serial_port_var.get().strip()
        if not port_display:
            messagebox.showerror("输入错误", "请输入串口号")
            return
        port_name = self._serial_port_map.get(port_display, port_display)
        self.transport.start(port_name, baud)

    def _stop(self) -> None:
        self.tcp_transport.stop()
        self.udp_transport.stop()
        self.serial_transport.stop()
        self.transport = self._current_transport()

    def _send(self, line: str) -> None:
        self.last_cmd_var.set(f"最近命令: {line}")
        self._append(f"> {line}")
        if not self.transport.send_line(line):
            self._append("[上位机] 发送失败")
        elif line in {"PING", "STATUS?", "CONFIG?", "PARAM?", "PID?", "BARO?", "GPS?", "MAG?", "AIRFRAME?"}:
            sent_at = time.monotonic()
            self.after(CMD_REPLY_TIMEOUT_MS, lambda sent=line, start=sent_at: self._warn_if_no_reply(sent, start))

    def _send_proto(self, function: int, label: str, payload: str = "", expect_reply: bool = True) -> None:
        payload_text = payload if payload else label
        self.last_cmd_var.set(f"最近命令: {label}")
        self._append(f"> {label}")
        if not self.transport.send_frame(function, payload_text.encode("utf-8")):
            self._append("[上位机] 发送失败")
            return
        sent_at = time.monotonic()
        if self.transport is self.udp_transport and label in {"PING", "AIRFRAME?", "IDENT?", "STATUS?"}:
            self.ident_link_var.set(f"sent {label}; waiting for text reply")
        if self.transport is self.udp_transport:
            pass
        elif self.structured_protocol_supported is False and not (self.transport is self.serial_transport and SERIAL_ASCII_COMPAT_MODE):
            self.transport.send_line(payload_text)
        elif self.structured_protocol_supported is None and function != PROTO_REQ_CAPS and self.transport is not self.serial_transport:
            self.after(
                PROTO_COMPAT_FALLBACK_DELAY_MS,
                lambda legacy=payload_text, start=sent_at: self._fallback_proto_request(legacy, start),
            )
        if expect_reply:
            self.after(CMD_REPLY_TIMEOUT_MS, lambda sent=label, start=sent_at: self._warn_if_no_reply(sent, start))

    def _send_proto_once(self, function: int, label: str, payload: str = "", expect_reply: bool = True) -> None:
        payload_text = payload if payload else label
        self.last_cmd_var.set(f"最近命令: {label}")
        self._append(f"> {label}")
        if not self.transport.send_frame(function, payload_text.encode("utf-8")):
            self._append("[上位机] 发送失败")
            return
        if expect_reply:
            sent_at = time.monotonic()
            self.after(CMD_REPLY_TIMEOUT_MS, lambda sent=label, start=sent_at: self._warn_if_no_reply(sent, start))

    def _send_proto_silent(self, function: int, payload: str) -> bool:
        if not self._transport_connected():
            return False
        return self.transport.send_frame(function, payload.encode("utf-8"))

    def _fallback_proto_request(self, legacy_line: str, started_at: float) -> None:
        if not self._transport_connected():
            return
        if self.structured_protocol_supported is True:
            return
        if self.last_reply_rx >= started_at:
            return
        self.transport.send_line(legacy_line)

    def _begin_protocol_probe(self) -> None:
        if self.transport is self.udp_transport:
            self.structured_protocol_supported = False
            return
        if self.transport is self.serial_transport and SERIAL_ASCII_COMPAT_MODE:
            self.structured_protocol_supported = False
            return
        self.structured_protocol_supported = None
        if not self._transport_connected():
            return

        started_at = time.monotonic()
        self.transport.send_frame(PROTO_REQ_CAPS, b"CAPS?")
        self.after(
            PROTO_COMPAT_FALLBACK_DELAY_MS,
            lambda start=started_at: self._fallback_proto_request("CAPS?", start),
        )
        self.after(PROTO_PROBE_SETTLE_MS, self._finalize_protocol_probe)

    def _finalize_protocol_probe(self) -> None:
        if not self._transport_connected():
            self.structured_protocol_supported = None
            return
        if self.structured_protocol_supported is None:
            self.structured_protocol_supported = False

    def _request_overview_status(self) -> None:
        self._send_proto(PROTO_REQ_MODULES, "MODULES?")
        self._send_proto(PROTO_REQ_STATUS, "STATUS?")

    def _send_custom(self) -> None:
        line = self.cmd_var.get().strip()
        if line:
            self._send(line)
            if self.transport is self.udp_transport and line in {"PING", "AIRFRAME?", "IDENT?", "STATUS?"}:
                self.ident_link_var.set(f"sent {line}; waiting for text reply")
            self.cmd_var.set("")

    def _read_all_params(self) -> None:
        self._send_proto(PROTO_REQ_CONFIG, "CONFIG?")
        self._send_proto(PROTO_REQ_PARAMS, "PARAM?")
        self._send_proto(PROTO_REQ_PID, "PID?")
        self._request_airframe()

    def _request_airframe(self) -> None:
        self._send_proto(PROTO_REQ_AIRFRAME, "AIRFRAME?")

    def _send_raw(self) -> None:
        payload = f"SERVO RAW {self.raw_var.get().strip()}"
        self._send_proto(PROTO_REQ_SERVO_RAW, payload, payload)

    def _servo_values(self, index: int) -> dict[str, int]:
        widgets = self.servo_widgets[index]
        return {key: int(var.get()) for key, var in widgets.items()}

    def _servo_move(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO MOVE {index} {values['pulse']} {values['time']}"
        self._send_proto(PROTO_REQ_SERVO_MOVE, payload, payload)

    def _servo_mode(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO MODE {index} {values['mode']}"
        self._send_proto(PROTO_REQ_SERVO_MODE, payload, payload)

    def _servo_enable(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO ENABLE {index} {values['enabled']}"
        self._send_proto(PROTO_REQ_SERVO_ENABLE, payload, payload)

    def _servo_set_id(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO ID {index} {values['id']}"
        self._send_proto(PROTO_REQ_SERVO_ID, payload, payload)

    def _servo_set_physical_id(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO SETID {index} {values['new_id']}"
        self._send_proto(PROTO_REQ_SERVO_SETID, payload, payload)

    def _servo_cmd(self, index: int, command: str) -> None:
        payload = f"SERVO CMD {index} {command}"
        self._send_proto(PROTO_REQ_SERVO_ACTION, payload, payload)

    def _servo_baud(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO CMD {index} BD {values['baud']}"
        self._send_proto(PROTO_REQ_SERVO_ACTION, payload, payload)

    MAX_LOG_LINES = 2000

    def _should_show_raw_log(self, line: str) -> bool:
        if line.startswith(("READY", "UART1 ")):
            return False
        if line.startswith(("GPS_USART2 ", "MAG_I2C1 ")):
            return False
        if line.startswith("BARO ok="):
            return False
        if line.startswith("[host] UDP RX "):
            return False
        if line.startswith("RXRAW "):
            return False
        if re.search(r"(^|[ ,])ax=", line) and re.search(r"(^|[ ,])az=", line):
            return False
        return True

    def _append(self, line: str) -> None:
        if not self._should_show_raw_log(line):
            return
        stamp = time.strftime("%H:%M:%S")
        self.status_text.insert(tk.END, f"[{stamp}] {line}\n")
        if int(self.status_text.index("end-1c").split(".")[0]) > self.MAX_LOG_LINES:
            self.status_text.delete("1.0", "2.0")
        self.status_text.see(tk.END)

    def _transport_label(self) -> str:
        if self.transport is self.tcp_transport:
            return "TCP"
        if self.transport is self.udp_transport:
            return "UDP"
        return "串口"

    def _handle_proto_frame(self, function: int, text: str) -> None:
        self.last_board_rx = time.monotonic()
        self.link_var.set("已收到 STM32 数据，TCP 与串口透明链路正常")
        if function >= PROTO_MSG_PONG:
            self.structured_protocol_supported = True

        display = self._normalize_proto_line(function, text)

        self._append(display)

        if function in {
            PROTO_MSG_CMD_RX,
            PROTO_MSG_CMD_ACK,
            PROTO_MSG_CMD_ERR,
            PROTO_MSG_CMD_OK,
            PROTO_MSG_PONG,
        }:
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近回包: {display}")
            return

        if function == PROTO_MSG_READY:
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近回包: {display}")
            self._handle_ready_line(text)
            return

        typed_handlers = {
            PROTO_MSG_HW_FLASH: self._update_hardware_line,
            PROTO_MSG_HW_BARO: self._update_hardware_line,
            PROTO_MSG_HW_IMU: self._update_hardware_line,
            PROTO_MSG_STATUS_FLASH: self._update_status_line,
            PROTO_MSG_STATUS_BARO: self._update_status_line,
            PROTO_MSG_STATUS_IMU: self._update_status_line,
            PROTO_MSG_UART_STATS: self._update_uart_line,
            PROTO_MSG_CONFIG_SUMMARY: self._update_config_line,
            PROTO_MSG_CONFIG_SERVO: self._update_config_line,
            PROTO_MSG_PARAM_RECORD: self._update_param_line,
            PROTO_MSG_PID_RECORD: self._update_pid_line,
            PROTO_MSG_FLASH_RECORD: self._update_flash_line,
            PROTO_MSG_BARO_STATE: self._update_baro_line,
            PROTO_MSG_BARO_DIAG: self._handle_board_line,
            PROTO_MSG_BARO_RAW: self._handle_board_line,
            PROTO_MSG_BARO_STREAM: self._update_baro_line,
            PROTO_MSG_IMU_STATE: self._update_imu_line,
            PROTO_MSG_IMU_SCALED: self._update_imu_line,
            PROTO_MSG_MODULES_SUMMARY: self._handle_board_line,
            PROTO_MSG_CAPS_RECORD: self._handle_board_line,
            PROTO_MSG_SAVE_RESULT: self._handle_board_line,
            PROTO_MSG_LOAD_RESULT: self._handle_board_line,
            PROTO_MSG_DEFAULTS_RESULT: self._handle_board_line,
            PROTO_MSG_SERVO_RESULT: self._handle_board_line,
            PROTO_MSG_WIFI_RECORD: self._update_wifi_line,
            PROTO_MSG_GPS_RECORD: self._handle_board_line,
            PROTO_MSG_MAG_RECORD: self._handle_board_line,
            PROTO_MSG_AIRFRAME_RECORD: self._handle_board_line,
            PROTO_MSG_TEXT_LINE: self._handle_board_line,
        }

        handler = typed_handlers.get(function)
        if handler is not None:
            handler(display)
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近回包: {display}")

    def _normalize_proto_line(self, function: int, text: str) -> str:
        stripped = text.strip()
        if function == PROTO_MSG_TEXT_LINE:
            return stripped
        if function == PROTO_MSG_CMD_RX:
            return self._ensure_line_prefix(stripped, "RX")
        if function == PROTO_MSG_CMD_ACK:
            return self._ensure_line_prefix(stripped, "ACK")
        if function == PROTO_MSG_CMD_ERR:
            return self._ensure_line_prefix(stripped, "ERR")
        if function == PROTO_MSG_CMD_OK:
            return self._ensure_line_prefix(stripped, "OK")
        if function == PROTO_MSG_PONG:
            return self._ensure_line_prefix(stripped, "PONG")
        if function == PROTO_MSG_READY:
            return self._ensure_line_prefix(stripped, "READY")
        if function == PROTO_MSG_WIFI_RECORD:
            if stripped.startswith(("WIFI ", "RSP ")):
                return stripped
            return self._ensure_line_prefix(stripped, "WIFI")
        if function == PROTO_MSG_GPS_RECORD:
            if stripped.startswith(("GPS ", "GPS_USART2 ", "M9N ", "HW ", "STATUS ", "RSP ")):
                return stripped
            return self._ensure_line_prefix(stripped, "GPS")
        if function == PROTO_MSG_MAG_RECORD:
            if stripped.startswith(("MAG ", "MAG_I2C1 ", "HW ", "STATUS ", "RSP ")):
                return stripped
            return self._ensure_line_prefix(stripped, "MAG")

        prefix_map = {
            PROTO_MSG_HW_FLASH: "HW FLASH",
            PROTO_MSG_HW_BARO: "HW SPL06",
            PROTO_MSG_HW_IMU: "HW ICM42688",
            PROTO_MSG_STATUS_FLASH: "STATUS flash",
            PROTO_MSG_STATUS_BARO: "STATUS baro",
            PROTO_MSG_STATUS_IMU: "STATUS imu",
            PROTO_MSG_UART_STATS: "UART1",
            PROTO_MSG_CONFIG_SUMMARY: "CFG",
            PROTO_MSG_CONFIG_SERVO: "CFG",
            PROTO_MSG_PARAM_RECORD: "PARAM",
            PROTO_MSG_PID_RECORD: "PID",
            PROTO_MSG_FLASH_RECORD: "FLASH",
            PROTO_MSG_BARO_STATE: "BARO",
            PROTO_MSG_BARO_DIAG: "BARO",
            PROTO_MSG_BARO_RAW: "BARO",
            PROTO_MSG_BARO_STREAM: "BARO",
            PROTO_MSG_RTOS_RECORD: "RTOS",
            PROTO_MSG_FLASH_BENCH: "FLASH",
            PROTO_MSG_AIRFRAME_RECORD: "AIRFRAME",
        }
        prefix = prefix_map.get(function)
        if prefix is None:
            return stripped if stripped else f"fn=0x{function:04X}"
        return self._ensure_line_prefix(stripped, prefix)

    def _ensure_line_prefix(self, text: str, prefix: str) -> str:
        stripped = text.strip()
        if not stripped:
            return prefix
        if stripped.startswith(prefix):
            return stripped
        return f"{prefix} {stripped}"

    def _update_servo_ok_line(self, line: str) -> None:
        values = parse_kv(line)
        parts = line.split()
        if len(parts) < 2 or not parts[1].startswith("servo"):
            return
        index = safe_int(parts[1].replace("servo", ""), -1)
        if index < 0:
            return

        field_map = {
            "id": "id",
            "enabled": "enabled",
            "mode": "mode",
            "pulse": "pulse",
            "time": "time",
        }
        for src, dst in field_map.items():
            if src in values:
                self._set_param(f"servo{index}.{dst}", values[src], "OK", dirty=False)
        if 0 <= index < len(self.servo_widgets):
            widgets = self.servo_widgets[index]
            for src, dst in field_map.items():
                if src in values and dst in widgets:
                    widgets[dst].set(safe_int(values[src]))

    def _update_config_result_line(self, line: str) -> None:
        values = parse_kv(line)
        status = values.get("st", "-")
        if line.startswith("OK save"):
            if status == "0":
                self._set_param("config.valid", "1", "SAVE", dirty=False)
            self._set_param("config.last_flash_status", status, "SAVE", dirty=False)
        elif line.startswith("OK load"):
            if status == "0":
                self._set_param("config.loaded", "1", "LOAD", dirty=False)
            self._set_param("config.last_flash_status", status, "LOAD", dirty=False)
        elif line.startswith("OK defaults"):
            self._set_param("config.loaded", "0", "DEFAULTS", dirty=False)
            self._set_param("config.valid", "0", "DEFAULTS", dirty=False)
        self._send_proto_once(PROTO_REQ_CONFIG, "CONFIG?")

    def _on_module_select(self, _event: tk.Event) -> None:
        selection = self.module_tree.selection()
        if not selection:
            return
        self.selected_module = selection[0]
        self._refresh_detail()

    def _request_selected_module(self) -> None:
        if self.selected_module == "FLASH":
            self._send_proto(PROTO_REQ_FLASH, "FLASH?")
            return
        if self.selected_module == "SPL06":
            self._send_proto(PROTO_REQ_BARO, "BARO?")
            return
        if self.selected_module == "ICM42688":
            self._send_proto(PROTO_REQ_IMU, "IMU?")
            return
        if self.selected_module == "GPS":
            self._send_proto(PROTO_REQ_GPS, "GPS?")
            return
        if self.selected_module == "MAG":
            self._send_proto(PROTO_REQ_MAG, "MAG?")
            return
        if self.selected_module == "WIFI":
            self._send_proto(PROTO_REQ_WIFI, "WIFI?")
            return
        self._send_proto(PROTO_REQ_STATUS, "STATUS?")

    def _open_baro_tab(self) -> None:
        self.notebook.select(self.baro_tab)
        self._send_proto(PROTO_REQ_BARO, "BARO?")

    def _open_imu_tab(self) -> None:
        self.notebook.select(self.imu_tab)
        self._send_proto(PROTO_REQ_IMU, "IMU?")

    def _open_gps_tab(self) -> None:
        self.notebook.select(self.gps_tab)
        self._send_proto(PROTO_REQ_GPS, "GPS?")

    def _refresh_detail(self) -> None:
        state = self.module_state[self.selected_module]
        for name in self.detail_vars:
            self.detail_vars[name].set(state[name].get())

    def _update_module(self, key: str, *, state: str | None = None, stage: str | None = None, value: str | None = None, code: str | None = None, hint: str | None = None, line: str | None = None) -> None:
        module = self.module_state.get(key)
        if module is None:
            return
        updates = {
            "state": state,
            "stage": stage,
            "value": value,
            "code": code,
            "hint": hint,
            "last": line,
        }
        for name, new_value in updates.items():
            if new_value is not None:
                module[name].set(new_value)
        self.module_tree.item(
            key,
            values=(
                module["state"].get(),
                module["stage"].get(),
                module["value"].get(),
                module["code"].get(),
                module["hint"].get(),
            ),
        )
        if key == self.selected_module:
            self._refresh_detail()

    def _handle_board_line(self, line: str) -> None:
        if line.startswith("[上位机] 板子已连接") or line.startswith("[上位机] 串口已连接"):
            self._begin_protocol_probe()
            return
        if line.startswith("[上位机] 板子已断开") or line.startswith("[上位机] 串口已断开"):
            self.structured_protocol_supported = None
            return

        if not (line.startswith("[上位机]") or line.startswith("[host]")):
            self.last_board_rx = time.monotonic()
            mode = self._transport_label()
            self.link_var.set(f"已收到 STM32 数据，{mode} 链路正常")
            if line.startswith(("AIRFRAME ", "IDENT ", "OK ", "ERR ", "PONG", "READY")):
                self.ident_link_var.set(f"{mode} text OK: {line[:96]}")

        if line.startswith("READY"):
            self.last_board_rx = time.monotonic()
            self.link_var.set("STM32 已就绪")
            self._handle_ready_line(line)
        elif line.startswith("HW "):
            self._update_hardware_line(line)
        elif line.startswith("STATUS "):
            self._update_status_line(line)
        elif line.startswith("UART1 "):
            self._update_uart_line(line)
        elif line.startswith("CFG "):
            self._update_config_line(line)
        elif line.startswith("PARAM "):
            self._update_param_line(line)
        elif line.startswith("AIRFRAME "):
            self._update_airframe_line(line)
        elif line.startswith("PID "):
            self._update_pid_line(line)
        elif line.startswith("FLASH "):
            self._update_flash_line(line)
        elif line.startswith("BARO ") or line.startswith("SPL06 "):
            self._update_baro_line(line)
        elif line.startswith("IMU "):
            self._update_imu_line(line)
        elif line.startswith(("GPS ", "GPS_USART2 ", "M9N ")):
            self._update_gps_line(line)
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近回包: {line}")
        elif line.startswith(("MAG ", "MAG_I2C1 ")):
            self._update_mag_line(line)
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近回包: {line}")
        elif re.search(r"(^|[ ,])ax=", line) and re.search(r"(^|[ ,])az=", line):
            self._update_imu_line("IMU " + line)
        elif line.startswith("WIFI "):
            self._update_wifi_line(line)
        elif line.startswith("IDENT "):
            self._ident_handle_line(line)
        elif line.startswith("RSP "):
            self._handle_rsp_line(line)
        elif line.startswith("OK servo"):
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近命令: {line}")
            self._update_servo_ok_line(line)
        elif line.startswith(("OK save", "OK load", "OK defaults")):
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近命令: {line}")
            self._update_config_result_line(line)
        elif line.startswith("OK ") or line.startswith("ERR ") or line.startswith("ACK ") or line.startswith("RX "):
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近命令: {line}")
        elif line.startswith(("PONG ", "READY ", "FLASH ", "BARO ", "IMU ", "GPS ", "GPS_USART2 ", "M9N ", "MAG ", "MAG_I2C1 ")):
            self.last_reply_rx = time.monotonic()

    def _handle_ready_line(self, line: str) -> None:
        values = parse_kv(line)
        if "cfg_valid" in values:
            self._set_param("system.cfg_valid", values["cfg_valid"], "READY", dirty=False)
        if "servo0_id" in values:
            self._set_param("servo0.id", values["servo0_id"], "READY", dirty=False)
        if "servo1_id" in values:
            self._set_param("servo1.id", values["servo1_id"], "READY", dirty=False)

    def _update_flash_line(self, line: str) -> None:
        values = parse_kv(line)
        if {"ok", "probe", "status", "sr_st", "read"} & values.keys():
            ok = values.get("ok") == "1"
            if "ok" not in values:
                ok = all(safe_int(values.get(name), 1) == 0 for name in ("probe", "status", "sr_st", "read"))
            self._update_module(
                "FLASH",
                state="正常" if ok else "异常",
                stage=self._stage_text(values.get("stage", "ready" if ok else "-")),
                value=f"ID={values.get('id', '-')} 期望={values.get('exp', '-')} SR1={values.get('sr1', '-')}",
                code=f"probe={values.get('probe', '-')} status={first_value(values, 'status', 'sr', 'sr_st')} read={values.get('read', '-')}",
                hint=self._hardware_hint("FLASH", ok, values),
                line=line,
            )

        if "cfg_addr" in values or "cfg_valid" in values or "cfg_last" in values:
            for key, value in values.items():
                self._set_param(f"flash.{key}", value, "FLASH", dirty=False)
            if "cfg_valid" in values:
                self._set_param("config.valid", values["cfg_valid"], "FLASH", dirty=False)

    def _update_imu_line(self, line: str) -> None:
        values = parse_kv(line)
        ok = values.get("ok") == "1" if "ok" in values else safe_int(values.get("init"), 0) != 0
        self._update_imu_attitude(values, line)
        self._update_module(
            "ICM42688",
            state="正常" if ok else "异常",
            stage=self._stage_text(values.get("stage", "-")),
            value=(
                f"WHO={first_value(values, 'who', 'id')} n={first_value(values, 'n', 'count')} "
                f"ax={first_value(values, 'ax', 'ax_mg')} ay={first_value(values, 'ay', 'ay_mg')} "
                f"az={first_value(values, 'az', 'az_mg')}"
            ),
            code=f"st={first_value(values, 'st', 'code')} err={values.get('err', '-')}",
            hint=self._hardware_hint("ICM42688", ok, values),
            line=line,
        )

    def _update_imu_attitude(self, values: dict[str, str], line: str) -> None:
        ax = first_float(values, "ax", "ax_mg")
        ay = first_float(values, "ay", "ay_mg")
        az = first_float(values, "az", "az_mg")
        gx = first_float(values, "gx", "gx_mdps")
        gy = first_float(values, "gy", "gy_mdps")
        gz = first_float(values, "gz", "gz_mdps")
        now = time.monotonic()

        if ax is None or ay is None or az is None:
            return

        if gx is None:
            gx = 0.0
        if gy is None:
            gy = 0.0
        if gz is None:
            gz = 0.0

        roll_cdeg = first_float(values, "roll", "roll_cdeg")
        pitch_cdeg = first_float(values, "pitch", "pitch_cdeg")
        yaw_cdeg = first_float(values, "yaw", "yaw_cdeg")
        if roll_cdeg is not None and pitch_cdeg is not None and yaw_cdeg is not None:
            self.imu_roll_deg = roll_cdeg / 100.0
            self.imu_pitch_deg = pitch_cdeg / 100.0
            self.imu_yaw_deg = yaw_cdeg / 100.0
        else:
            roll_acc = math.degrees(math.atan2(ay, az))
            pitch_acc = math.degrees(math.atan2(-ax, math.sqrt((ay * ay) + (az * az))))
            dt = 0.0
            if self.imu_last_sample_time > 0.0:
                dt = max(0.0, min(now - self.imu_last_sample_time, 0.25))

            if dt <= 0.0:
                self.imu_roll_deg = roll_acc
                self.imu_pitch_deg = pitch_acc
            else:
                alpha = 0.96
                self.imu_roll_deg = alpha * (self.imu_roll_deg + (gx / 1000.0) * dt) + (1.0 - alpha) * roll_acc
                self.imu_pitch_deg = alpha * (self.imu_pitch_deg + (gy / 1000.0) * dt) + (1.0 - alpha) * pitch_acc
                self.imu_yaw_deg += (gz / 1000.0) * dt
                if self.imu_yaw_deg > 180.0 or self.imu_yaw_deg < -180.0:
                    self.imu_yaw_deg = ((self.imu_yaw_deg + 180.0) % 360.0) - 180.0

        self.imu_last_sample_time = now
        self.imu_last_count = safe_int(first_value(values, "n", "count"), self.imu_last_count)

        updates = {
            "roll": f"{self.imu_roll_deg:.1f}",
            "pitch": f"{self.imu_pitch_deg:.1f}",
            "yaw": f"{self.imu_yaw_deg:.1f}",
            "ax": f"{ax:.0f}",
            "ay": f"{ay:.0f}",
            "az": f"{az:.0f}",
            "gx": f"{gx:.0f}",
            "gy": f"{gy:.0f}",
            "gz": f"{gz:.0f}",
            "temp": first_value(values, "t", "temp_cdeg"),
            "count": first_value(values, "n", "count"),
            "who": first_value(values, "who", "id"),
            "age": "0 ms",
        }
        for key, value in updates.items():
            if key in self.imu_vars and value != "-":
                self.imu_vars[key].set(value)
        self._draw_imu_attitude()

    def _reset_imu_attitude(self) -> None:
        self.imu_roll_deg = 0.0
        self.imu_pitch_deg = 0.0
        self.imu_yaw_deg = 0.0
        self.imu_last_sample_time = 0.0
        for key, value in {"roll": "0.0", "pitch": "0.0", "yaw": "0.0"}.items():
            if key in self.imu_vars:
                self.imu_vars[key].set(value)
        self._draw_imu_attitude()

    def _draw_imu_attitude(self) -> None:
        canvas = getattr(self, "imu_canvas", None)
        if canvas is None:
            return

        width = max(int(canvas.winfo_width()), 320)
        height = max(int(canvas.winfo_height()), 240)
        canvas.delete("all")

        cx = width / 2.0
        cy = height / 2.0
        radius = min(width, height) * 0.42
        roll = math.radians(self.imu_roll_deg)
        pitch_offset = max(-radius * 0.75, min(radius * 0.75, self.imu_pitch_deg * radius / 45.0))

        def rotate(point: tuple[float, float]) -> tuple[float, float]:
            x, y = point
            return (
                cx + (x * math.cos(roll)) - (y * math.sin(roll)),
                cy + (x * math.sin(roll)) + (y * math.cos(roll)),
            )

        span = radius * 2.8
        sky_poly = [
            rotate((-span, -span + pitch_offset)),
            rotate((span, -span + pitch_offset)),
            rotate((span, pitch_offset)),
            rotate((-span, pitch_offset)),
        ]
        ground_poly = [
            rotate((-span, pitch_offset)),
            rotate((span, pitch_offset)),
            rotate((span, span + pitch_offset)),
            rotate((-span, span + pitch_offset)),
        ]
        canvas.create_polygon(sky_poly, fill="#2878b8", outline="")
        canvas.create_polygon(ground_poly, fill="#7a4b2a", outline="")
        canvas.create_line(*rotate((-span, pitch_offset)), *rotate((span, pitch_offset)), fill="#f7f3dc", width=3)

        for deg in range(-60, 75, 15):
            if deg == 0:
                continue
            y = pitch_offset - (deg * radius / 45.0)
            half = radius * (0.38 if deg % 30 == 0 else 0.22)
            x1, y1 = rotate((-half, y))
            x2, y2 = rotate((half, y))
            canvas.create_line(x1, y1, x2, y2, fill="#f7f3dc", width=2)
            if deg % 30 == 0:
                tx, ty = rotate((half + 14, y + 4))
                canvas.create_text(tx, ty, text=str(abs(deg)), fill="#f7f3dc", font=("Segoe UI", 9))

        canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#e5e7eb", width=3)
        canvas.create_line(cx - radius * 0.55, cy, cx - radius * 0.15, cy, fill="#ffd166", width=5)
        canvas.create_line(cx + radius * 0.15, cy, cx + radius * 0.55, cy, fill="#ffd166", width=5)
        canvas.create_polygon(cx - 10, cy, cx, cy + 12, cx + 10, cy, fill="#ffd166", outline="")
        canvas.create_text(
            cx,
            height - 26,
            text=f"roll {self.imu_roll_deg:+.1f}   pitch {self.imu_pitch_deg:+.1f}   yaw {self.imu_yaw_deg:+.1f}",
            fill="#e5e7eb",
            font=("Segoe UI", 12, "bold"),
        )

    def _handle_rsp_line(self, line: str) -> None:
        values = parse_kv(line)
        mod = values.get("mod", "").upper()
        payload = {key: value for key, value in values.items() if key not in {"id", "mod", "op"}}

        if mod == "MODULES":
            self._update_modules_summary(payload, line)
            return

        if mod == "CAPS":
            for key, value in payload.items():
                self._set_param(f"caps.{key}", value, "CAPS", dirty=False)
            return

        if mod == "SPL06":
            rename_map = {
                "who": "product_id",
                "sid": "split_id",
                "tid": "txrx_id",
                "press_raw": "raw_pressure",
                "temp_raw": "raw_temperature",
            }
            for old, new in rename_map.items():
                if old in payload and new not in payload:
                    payload[new] = payload.pop(old)
            self._update_baro_line("BARO " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

        if mod == "ICM42688":
            if "ok" in payload and "init" not in payload:
                payload["init"] = payload["ok"]
            if "code" in payload and "err" not in payload:
                payload["err"] = payload["code"]
            self._update_imu_line("IMU " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

        if mod in {"GPS", "M9N"}:
            self._update_gps_line("GPS " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

        if mod == "MAG":
            self._update_mag_line("MAG " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

        if mod == "FLASH":
            self._update_flash_line("FLASH " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

        if mod == "WIFI":
            self._update_wifi_line("WIFI " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

    def _update_modules_summary(self, values: dict[str, str], line: str) -> None:
        if "flash" in values or "flash_stage" in values:
            ok = safe_int(values.get("flash"), 0) != 0
            self._update_module(
                "FLASH",
                state="正常" if ok else "异常",
                stage=self._stage_text(values.get("flash_stage", "-")),
                hint=self._hardware_hint("FLASH", ok, values),
                line=line,
            )

        if "baro" in values or "baro_stage" in values:
            ok = safe_int(values.get("baro"), 0) != 0
            self._update_module(
                "SPL06",
                state="正常" if ok else "异常",
                stage=self._stage_text(values.get("baro_stage", "-")),
                hint=self._hardware_hint("SPL06", ok, values),
                line=line,
            )

        if "imu" in values or "imu_stage" in values:
            ok = safe_int(values.get("imu"), 0) != 0
            self._update_module(
                "ICM42688",
                state="正常" if ok else "异常",
                stage=self._stage_text(values.get("imu_stage", "-")),
                hint=self._hardware_hint("ICM42688", ok, values),
                line=line,
            )

        if "gps" in values or "gps_fix" in values or "gps_sv" in values:
            ok = safe_int(first_value(values, "gps", "gps_ok"), 0) != 0
            self._update_module(
                "GPS",
                state="已初始化" if ok else "等待",
                stage=f"fix={first_value(values, 'gps_fix', 'fix')}",
                value=f"sv={first_value(values, 'gps_sv', 'sv')}",
                code=f"init={first_value(values, 'gps', 'gps_ok')}",
                hint=self._hardware_hint("GPS", ok, values),
                line=line,
            )

        if "mag" in values or "mag_type" in values:
            ok = safe_int(first_value(values, "mag", "mag_ok"), 0) != 0
            self._update_module(
                "MAG",
                state="正常" if ok else "异常",
                stage=first_value(values, "mag_type", "type"),
                value=f"type={first_value(values, 'mag_type', 'type')}",
                code=f"init={first_value(values, 'mag', 'mag_ok')}",
                hint=self._hardware_hint("MAG", ok, values),
                line=line,
            )

        if "cfg_valid" in values:
            self._set_param("config.valid", values["cfg_valid"], "MODULES", dirty=False)
        if "cfg_loaded" in values:
            self._set_param("config.loaded", values["cfg_loaded"], "MODULES", dirty=False)
        if "servo_slots" in values:
            self._set_param("system.servo_slots", values["servo_slots"], "MODULES", dirty=False)
        if "wifi_en" in values:
            ok = safe_int(values.get("wifi_en"), 0) != 0
            self._update_module(
                "WIFI",
                state="已使能" if ok else "已关闭",
                stage="-",
                value=f"en={values.get('wifi_en', '-')}",
                code="-",
                hint="等待 Ai-WB2 透明 TCP 连接。" if ok else "PC6 当前关闭 WiFi EN。",
                line=line,
            )
        if {"cfg_valid", "cfg_loaded"} & values.keys():
            self.config_summary_var.set(
                f"配置 loaded={values.get('cfg_loaded', '-')} valid={values.get('cfg_valid', '-')} flash_st=-"
            )

    def _update_hardware_line(self, line: str) -> None:
        parts = line.split(maxsplit=2)
        if len(parts) < 2:
            return
        key = normalize_module_key(parts[1])
        if key is None:
            return
        values = parse_kv(line)
        ok = values.get("ok") == "1"
        state = "正常" if ok else "异常"
        stage = self._stage_text(values.get("stage", "-"))

        if key == "FLASH":
            value = f"ID={values.get('id', '-')} 期望={values.get('exp', '-')} SR1={values.get('sr1', '-')}"
            code = f"probe={values.get('probe', '-')} sr={values.get('sr', '-')} read={values.get('read', '-')}"
        elif key == "SPL06":
            value = f"ID={values.get('id', '-')} 期望={values.get('exp', '-')} split={values.get('split_id', '-')} txrx={values.get('txrx_id', '-')}"
            code = f"init={values.get('init', '-')} split={values.get('split', '-')} txrx={values.get('txrx', '-')}"
            self._update_baro_from_values(values, line)
        elif key == "ICM42688":
            value = f"WHO={values.get('who', '-')} 期望={values.get('exp', '-')} n={values.get('n', '-')}"
            code = f"st={values.get('st', '-')}"
        elif key == "GPS":
            value = f"fix={first_value(values, 'fix', 'fix_type')} sv={first_value(values, 'sv', 'num_sv')} lat={values.get('lat', '-')} lon={values.get('lon', '-')}"
            code = f"init={values.get('init', '-')} nav={values.get('nav', '-')}"
            self._update_gps_line("GPS " + " ".join(f"{name}={value}" for name, value in values.items()))
        elif key == "MAG":
            value = f"type={values.get('type', '-')} addr={values.get('addr', '-')} n={values.get('n', '-')}"
            code = f"init={values.get('init', '-')} st={values.get('st', '-')}"
            self._update_mag_line("MAG " + " ".join(f"{name}={value}" for name, value in values.items()))
        else:
            value = "-"
            code = "-"

        self._update_module(key, state=state, stage=stage, value=value, code=code, hint=self._hardware_hint(key, ok, values), line=line)

    def _update_status_line(self, line: str) -> None:
        parts = line.split(maxsplit=2)
        if len(parts) < 2:
            return
        subject = parts[1].lower()
        values = parse_kv(line)
        if subject == "flash":
            ok = all(safe_int(values.get(name), 1) == 0 for name in ("probe", "sr_st", "read"))
            self._update_module(
                "FLASH",
                state="正常" if ok else "异常",
                stage="初始化完成" if ok else "状态读取",
                value=f"ID={values.get('id', '-')} SR1={values.get('sr1', '-')}",
                code=f"probe={values.get('probe', '-')} sr_st={values.get('sr_st', '-')} read={values.get('read', '-')}",
                hint=self._hardware_hint("FLASH", ok, values),
                line=line,
            )
        elif subject == "baro":
            ok = safe_int(values.get("init"), 1) == 0
            self._update_module(
                "SPL06",
                state="正常" if ok else "异常",
                stage="初始化完成" if ok else "初始化/识别",
                value=f"ID={values.get('id', '-')} split={values.get('split_id', '-')} txrx={values.get('txrx_id', '-')} bmp={values.get('bmp', '-')}",
                code=f"init={values.get('init', '-')} split={values.get('split', '-')} txrx={values.get('txrx', '-')}",
                hint=self._hardware_hint("SPL06", ok, values),
                line=line,
            )
            self._update_baro_from_values(values, line)
        elif subject == "imu":
            ok = safe_int(values.get("init"), 0) != 0
            self._update_module(
                "ICM42688",
                state="正常" if ok else "异常",
                stage="初始化完成" if ok else "初始化/识别",
                value=f"WHO={values.get('who', '-')} n={values.get('n', '-')} ax={values.get('ax', '-')} ay={values.get('ay', '-')} az={values.get('az', '-')}",
                code=f"st={values.get('st', '-')}",
                hint=self._hardware_hint("ICM42688", ok, values),
                line=line,
            )
        elif subject == "gps":
            self._update_gps_line("GPS " + " ".join(f"{key}={value}" for key, value in values.items()))
        elif subject == "mag":
            self._update_mag_line("MAG " + " ".join(f"{key}={value}" for key, value in values.items()))

    def _update_wifi_line(self, line: str) -> None:
        values = parse_kv(line)
        enabled = safe_int(values.get("en"), 0) != 0
        transparent = safe_int(values.get("transparent"), 0) != 0
        cycling = safe_int(values.get("cycling"), 0) != 0
        state_text = values.get("state", "-")

        if transparent:
            state = "已连接"
            hint = "Ai-WB2 已进入 TCP 透明模式，可以通过 TCP 发送控制命令。"
        elif cycling:
            state = "重启中"
            hint = "PC6 正在重启 Ai-WB2，等待模块重新连接上位机 TCP 服务。"
        elif enabled:
            state = "等待连接"
            hint = "先保持上位机 TCP 监听；若连接失败，固件会通过 PC6 自动重启模块重试。"
        else:
            state = "已关闭"
            hint = "WiFi EN 为低；发送 WIFI EN 1 或 WIFI RESET 可重新拉起。"

        self._update_module(
            "WIFI",
            state=state,
            stage=state_text,
            value=f"en={values.get('en', '-')} trans={values.get('transparent', '-')} wait={values.get('wait_ms', '-')}",
            code=f"retry={values.get('retry', '-')} socket={values.get('socket', '-')} writes={values.get('writes', '-')}",
            hint=hint,
            line=line,
        )

    def _update_uart_line(self, line: str) -> None:
        values = parse_kv(line)
        rx_bytes = safe_int(values.get("rx_bytes"))
        rx_lines = safe_int(values.get("rx_lines"))
        rx_errors = safe_int(values.get("rx_errors"))
        rx_overflows = safe_int(values.get("rx_overflows"))
        ok = rx_errors == 0 and rx_overflows == 0

        if rx_bytes == 0:
            hint = "STM32 正在发心跳，但还没收到上位机命令；检查 Ai-WB2 到 USART1 的 TX/RX。"
        elif rx_lines == 0:
            hint = "已经收到字节但没有收到换行；检查上位机发送是否带 CR/LF。"
        elif ok:
            hint = "USART1 收发链路已打通。"
        else:
            hint = "串口出现溢出或错误；先降低发送频率，再检查波特率和接线。"

        self._update_module(
            "UART1",
            state="正常" if ok else "有错误",
            stage="接收统计",
            value=f"bytes={rx_bytes} lines={rx_lines}",
            code=f"overflow={rx_overflows} err={rx_errors}",
            hint=hint,
            line=line,
        )

    def _update_baro_line(self, line: str) -> None:
        values = parse_kv(line)
        ok_text = values.get("ok")
        if ok_text is not None:
            ok = ok_text == "1"
            self._update_module(
                "SPL06",
                state="正常" if ok else "异常",
                stage=self._stage_text(values.get("stage", "-")),
                value=f"P={first_value(values, 'pressure', 'pressure_pa', 'p', 'pa')} T={first_value(values, 'temperature', 'temp', 't')} Alt={first_value(values, 'altitude', 'alt')}",
                code=f"init={values.get('init', '-')} st={values.get('st', '-')}",
                hint=self._hardware_hint("SPL06", ok, values),
                line=line,
            )
        self._update_baro_from_values(values, line)

    def _update_baro_from_values(self, values: dict[str, str], line: str) -> None:
        is_raw_line = line.startswith("BARO raw")
        mapping = {
            "product_id": first_value(values, "product_id", "id", "who"),
            "split_id": first_value(values, "split_id", "sid"),
            "txrx_id": first_value(values, "txrx_id", "tid"),
            "bmp_id": first_value(values, "bmp", "bmp_id"),
            "init_status": first_value(values, "init"),
            "split_status": first_value(values, "split"),
            "txrx_status": first_value(values, "txrx"),
            "cs_level": first_value(values, "cs"),
            "miso_level": first_value(values, "miso"),
            "stage": self._stage_text(values.get("stage", "-")),
            "raw_pressure": first_value(values, "raw_pressure", "raw_p", "prs_raw", "press_raw"),
            "raw_temperature": first_value(values, "raw_temperature", "raw_t", "tmp_raw", "temp_raw"),
            "sample_count": first_value(values, "n", "count", "samples"),
            "last_update": time.strftime("%H:%M:%S"),
        }
        pressure = None if is_raw_line else first_float(values, "pressure", "pressure_pa", "p", "pa")
        temperature = None if is_raw_line else first_float(values, "temperature", "temp", "temp_c", "t")
        altitude = first_float(values, "altitude", "alt", "alt_m")
        if temperature is None and "temp_cdeg" in values:
            temperature = safe_int(values.get("temp_cdeg")) / 100.0
        if is_raw_line:
            if mapping["raw_pressure"] == "-":
                mapping["raw_pressure"] = first_value(values, "pressure")
            if mapping["raw_temperature"] == "-":
                mapping["raw_temperature"] = first_value(values, "temp")

        if pressure is not None:
            mapping["pressure"] = f"{pressure:g}"
        if temperature is not None:
            mapping["temperature"] = f"{temperature:g}"
        if altitude is not None:
            mapping["altitude"] = f"{altitude:g}"
        if "init" in values:
            mapping["state"] = "正常" if safe_int(values.get("init"), 1) == 0 else "异常"

        for key, value in mapping.items():
            if value != "-" and key in self.baro_vars:
                self.baro_vars[key].set(value)

        if pressure is None and temperature is None and altitude is None:
            return
        if not self.baro_capture_enabled.get():
            return

        sample: dict[str, float | str] = {"time": time.time(), "line": line}
        if pressure is not None:
            sample["pressure"] = pressure
        if temperature is not None:
            sample["temperature"] = temperature
        if altitude is not None:
            sample["altitude"] = altitude
        self.baro_buffer.append(sample)
        if len(self.baro_buffer) > MAX_BARO_SAMPLES:
            del self.baro_buffer[: len(self.baro_buffer) - MAX_BARO_SAMPLES]
        self._baro_dirty = True

    def _baro_tick(self) -> None:
        if self._baro_dirty:
            self._baro_dirty = False
            now_ns = time.monotonic_ns()
            self._refresh_baro_samples()
            if now_ns - self._last_baro_plot_ns > 300_000_000:
                self._last_baro_plot_ns = now_ns
                self._update_baro_plot()
        self.after(250, self._baro_tick)

    def _imu_poll_tick(self) -> None:
        now = time.monotonic()
        if self.imu_last_sample_time > 0.0 and "age" in self.imu_vars:
            age_ms = int((now - self.imu_last_sample_time) * 1000.0)
            self.imu_vars["age"].set(f"{age_ms} ms")
        if self.imu_poll_enabled.get() and self._transport_connected():
            if now - self.imu_last_poll >= IMU_POLL_PERIOD_MS / 1000.0:
                self.imu_last_poll = now
                self._send_proto_silent(PROTO_REQ_IMU, "IMU?")
        self.after(50, self._imu_poll_tick)

    def _refresh_baro_samples(self) -> None:
        self.baro_count_var.set(f"暂存样本: {len(self.baro_buffer)}")
        latest = self.baro_buffer[-80:]
        children = self.baro_sample_tree.get_children()
        n_existing = len(children)
        n_needed = len(latest)
        base = float(latest[0]["time"]) if latest else 0.0
        for i, sample in enumerate(latest):
            t = float(sample["time"]) - base
            vals = (
                f"{t:.2f}",
                self._fmt_sample(sample, "pressure"),
                self._fmt_sample(sample, "temperature"),
                self._fmt_sample(sample, "altitude"),
            )
            if i < n_existing:
                self.baro_sample_tree.item(children[i], values=vals)
            else:
                self.baro_sample_tree.insert("", tk.END, values=vals)
        for i in range(n_needed, n_existing):
            self.baro_sample_tree.delete(children[i])

    def _fmt_sample(self, sample: dict[str, float | str], key: str) -> str:
        value = sample.get(key)
        if isinstance(value, float):
            return f"{value:g}"
        return "-"

    def _update_baro_plot(self) -> None:
        if not HAS_MATPLOTLIB or self.baro_axis is None or self.baro_canvas is None:
            return
        key = self.plot_var.get()
        points = [(float(s["time"]), float(s[key])) for s in self.baro_buffer if key in s]
        self.baro_axis.clear()
        self.baro_axis.set_title(f"SPL06 {key}")
        self.baro_axis.set_xlabel("time (s)")
        self.baro_axis.grid(True, alpha=0.3)
        if points:
            base = points[0][0]
            self.baro_axis.plot([t - base for t, _v in points], [v for _t, v in points], linewidth=1.4)
        else:
            self.baro_axis.text(0.5, 0.5, "waiting for samples", ha="center", va="center", transform=self.baro_axis.transAxes)
        self.baro_figure.tight_layout()
        self.baro_canvas.draw_idle()

    def _clear_baro_buffer(self) -> None:
        self.baro_buffer.clear()
        self._baro_dirty = False
        self._refresh_baro_samples()
        self._last_baro_plot_ns = time.monotonic_ns()
        self._update_baro_plot()

    def _export_baro_csv(self) -> None:
        if not self.baro_buffer:
            messagebox.showinfo("没有数据", "气压计暂存区为空")
            return
        initial = Path.cwd() / f"baro_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        filename = filedialog.asksaveasfilename(
            title="导出气压计暂存数据",
            defaultextension=".csv",
            initialfile=initial.name,
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not filename:
            return
        with open(filename, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["time", "pressure", "temperature", "altitude", "line"])
            writer.writeheader()
            for sample in self.baro_buffer:
                writer.writerow(sample)
        self._append(f"[上位机] 已导出气压计暂存数据: {filename}")

    def _gps_lat_lon_from_values(self, values: dict[str, str]) -> tuple[float | None, float | None]:
        lat = first_float(values, "lat_deg", "latitude_deg", "latitude")
        lon = first_float(values, "lon_deg", "longitude_deg", "longitude")
        if lat is None:
            lat = first_float(values, "lat", "lat_deg_e7")
        if lon is None:
            lon = first_float(values, "lon", "lon_deg_e7")
        if lat is not None and abs(lat) > 90.0:
            lat /= 10000000.0
        if lon is not None and abs(lon) > 180.0:
            lon /= 10000000.0
        return lat, lon

    def _gps_xy_from_lat_lon(self, lat: float, lon: float) -> tuple[float, float]:
        if self.gps_origin_lat is None or self.gps_origin_lon is None:
            self.gps_origin_lat = lat
            self.gps_origin_lon = lon
        radius_m = 6378137.0
        lat0 = self.gps_origin_lat
        lon0 = self.gps_origin_lon
        x = math.radians(lon - lon0) * radius_m * math.cos(math.radians(lat0))
        y = math.radians(lat - lat0) * radius_m
        return x, y

    def _update_gps_line(self, line: str) -> None:
        values = parse_kv(line)
        lat, lon = self._gps_lat_lon_from_values(values)
        fix_text = first_value(values, "fix", "fix_type")
        previous_fix = self.gps_vars.get("fix").get() if "fix" in self.gps_vars else "0"
        fix = safe_int(fix_text, safe_int(previous_fix, 0))
        valid = safe_int(first_value(values, "valid", "valid_fix"), 0) != 0
        if "ok" in values:
            initialized = safe_int(values.get("ok"), 0) != 0
        elif line.startswith("STATUS gps"):
            initialized = safe_int(values.get("init"), 0) != 0
        elif "init" in values:
            initialized = safe_int(values.get("init"), 1) == 0
        else:
            initialized = False
        sv_text = first_value(values, "sv", "num_sv")
        previous_sv = self.gps_vars.get("sv").get() if "sv" in self.gps_vars else "0"
        sv = safe_int(sv_text, safe_int(previous_sv, 0))
        now = time.time()
        x: float | None = None
        y: float | None = None

        if lat is not None and lon is not None and abs(lat) <= 90.0 and abs(lon) <= 180.0:
            if abs(lat) > 0.000001 or abs(lon) > 0.000001:
                x, y = self._gps_xy_from_lat_lon(lat, lon)
                self.gps_track.append(
                    {
                        "time": now,
                        "lat": lat,
                        "lon": lon,
                        "x_m": x,
                        "y_m": y,
                        "fix": fix,
                        "sv": sv,
                        "hmsl_mm": safe_int(first_value(values, "hmsl_mm", "hmsl"), 0),
                        "hacc_mm": safe_int(first_value(values, "hacc_mm", "hacc"), 0),
                        "line": line,
                    }
                )
                if len(self.gps_track) > MAX_GPS_TRACK_POINTS:
                    del self.gps_track[: len(self.gps_track) - MAX_GPS_TRACK_POINTS]

        previous_state = self.gps_vars.get("state").get() if "state" in self.gps_vars else "-"
        if valid or fix >= 2:
            state = "已定位"
        elif initialized:
            state = "已初始化"
        elif previous_state not in {"-", "等待 GPS"} and not ({"ok", "init", "valid", "valid_fix", "fix", "fix_type"} & values.keys()):
            state = previous_state
        else:
            state = "等待 GPS"
        age = first_value(values, "age_ms", "age")
        if age != "-" and age.isdigit():
            age = f"{age} ms"
        speed = first_value(values, "gspd", "ground_speed_mm_s", "speed_mm_s")
        if speed == "-":
            vn = first_float(values, "vn", "vel_n_mm_s")
            ve = first_float(values, "ve", "vel_e_mm_s")
            if vn is not None and ve is not None:
                speed = f"{math.hypot(vn, ve):.0f}"

        updates = {
            "state": state,
            "fix": str(fix) if fix_text != "-" else "-",
            "sv": str(sv) if sv_text != "-" else "-",
            "lon": f"{lon:.7f}" if lon is not None else "-",
            "lat": f"{lat:.7f}" if lat is not None else "-",
            "hmsl": first_value(values, "hmsl_mm", "hmsl"),
            "x": f"{x:.2f}" if x is not None else "-",
            "y": f"{y:.2f}" if y is not None else "-",
            "spd": speed,
            "hdg": "-",
            "age": age,
        }
        heading = first_float(values, "head_e5", "heading_motion_deg_e5", "heading_deg_e5")
        if heading is not None:
            updates["hdg"] = f"{heading / 100000.0:.2f}"
        for key, value in updates.items():
            if key in self.gps_vars and value != "-":
                self.gps_vars[key].set(value)

        value_text = f"fix={fix} sv={sv}"
        if lat is not None and lon is not None:
            value_text += f" lat={lat:.7f} lon={lon:.7f}"
        code_parts = [
            f"nav={values.get('nav', '-')}",
            f"pkts={first_value(values, 'pkts', 'packets')}",
        ]
        if "nmea" in values or "gga" in values:
            code_parts.append(f"nmea={values.get('nmea', '-')}")
            code_parts.append(f"gga={values.get('gga', '-')}")

        self._update_module(
            "GPS",
            state=state,
            stage=f"fix={fix}",
            value=value_text,
            code=" ".join(code_parts),
            hint=self._hardware_hint("GPS", valid or initialized, values),
            line=line,
        )
        self.gps_count_var.set(f"轨迹点: {len(self.gps_track)}")
        self.gps_status_var.set(f"{state}  fix={fix}  sv={sv}")

        now_ns = time.monotonic_ns()
        if now_ns - self.gps_last_plot_ns > 200_000_000:
            self.gps_last_plot_ns = now_ns
            self._update_gps_plot()

    def _update_mag_line(self, line: str) -> None:
        values = parse_kv(line)
        has_state_fields = bool({"ok", "init", "st", "raw", "mgauss", "x", "y", "z"} & values.keys())
        if "ok" in values:
            ok = safe_int(values.get("ok"), 0) != 0
        elif line.startswith("STATUS mag"):
            ok = safe_int(values.get("init"), 0) != 0
        elif "init" not in values:
            ok = self.mag_vars.get("state").get() == "正常" if "state" in self.mag_vars else False
        else:
            ok = safe_int(values.get("init"), 1) == 0
        raw = first_value(values, "raw")
        scaled = first_value(values, "mgauss")
        if raw == "-":
            raw = f"{first_value(values, 'raw_x', 'x_raw')},{first_value(values, 'raw_y', 'y_raw')},{first_value(values, 'raw_z', 'z_raw')}"
        if scaled == "-":
            scaled = f"{first_value(values, 'x_mgauss', 'x')},{first_value(values, 'y_mgauss', 'y')},{first_value(values, 'z_mgauss', 'z')}"
        if raw == "-,-,-":
            raw = "-"
        if scaled == "-,-,-":
            scaled = "-"

        updates = {
            "state": ("正常" if ok else "异常") if has_state_fields else "-",
            "type": first_value(values, "type"),
            "raw": raw,
            "scaled": scaled,
        }
        for key, value in updates.items():
            if key in self.mag_vars and value != "-":
                self.mag_vars[key].set(value)

        module_state = ("正常" if ok else "异常") if has_state_fields else None
        if {"ist", "hmc", "qmc"} & values.keys():
            value_text = f"probe ist={values.get('ist', '-')} hmc={values.get('hmc', '-')} qmc={values.get('qmc', '-')}"
            code_text = f"hmc_id={values.get('hmc_id', '-')}"
        else:
            value_text = f"type={first_value(values, 'type')} addr={values.get('addr', '-')} n={values.get('n', '-')}"
            code_text = f"init={values.get('init', '-')} st={values.get('st', '-')}"
        self._update_module(
            "MAG",
            state=module_state,
            stage=first_value(values, "type"),
            value=value_text,
            code=code_text,
            hint=self._hardware_hint("MAG", ok, values) if has_state_fields else None,
            line=line,
        )

    def _update_gps_plot(self) -> None:
        if not HAS_MATPLOTLIB or self.gps_axis is None or self.gps_canvas is None:
            return
        self.gps_axis.clear()
        self.gps_axis.set_title("M9N XY track")
        self.gps_axis.set_xlabel("X east (m)")
        self.gps_axis.set_ylabel("Y north (m)")
        self.gps_axis.grid(True, alpha=0.3)
        self.gps_axis.set_aspect("equal", adjustable="datalim")

        if self.gps_track:
            xs = [float(point["x_m"]) for point in self.gps_track]
            ys = [float(point["y_m"]) for point in self.gps_track]
            self.gps_axis.plot(xs, ys, linewidth=1.2, marker=".", markersize=3)
            self.gps_axis.scatter([xs[-1]], [ys[-1]], s=45, color="#d62728", zorder=3)
            pad = max(max(xs) - min(xs), max(ys) - min(ys), 2.0) * 0.08
            self.gps_axis.set_xlim(min(xs) - pad, max(xs) + pad)
            self.gps_axis.set_ylim(min(ys) - pad, max(ys) + pad)
        else:
            self.gps_axis.text(0.5, 0.5, "waiting for GPS points", ha="center", va="center", transform=self.gps_axis.transAxes)

        self.gps_figure.tight_layout()
        self.gps_canvas.draw_idle()

    def _clear_gps_track(self) -> None:
        self.gps_track.clear()
        self.gps_origin_lat = None
        self.gps_origin_lon = None
        self.gps_count_var.set("轨迹点: 0")
        self.gps_status_var.set("等待 GPS")
        self.gps_last_plot_ns = time.monotonic_ns()
        self._update_gps_plot()

    def _export_gps_csv(self) -> None:
        if not self.gps_track:
            messagebox.showinfo("没有数据", "GPS 轨迹为空")
            return
        initial = Path.cwd() / f"gps_track_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        filename = filedialog.asksaveasfilename(
            title="导出 GPS 轨迹",
            defaultextension=".csv",
            initialfile=initial.name,
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not filename:
            return
        with open(filename, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["time", "lat", "lon", "x_m", "y_m", "fix", "sv", "hmsl_mm", "hacc_mm", "line"])
            writer.writeheader()
            for point in self.gps_track:
                writer.writerow(point)
        self._append(f"[上位机] 已导出 GPS 轨迹: {filename}")

    def _ident_csv_fields(self) -> list[str]:
        return [
            "host_time", "id", "seq", "t_ms", "axis", "mode",
            "alpha_us", "beta_us", "roll", "pitch", "gx", "gy",
            "rc_arm", "throttle_us", "line",
        ]

    def _ident_meta_payload(self) -> dict[str, object]:
        return {
            "host_time": time.time(),
            "command": self.ident_current_command,
            "axis": self.ident_axis_var.get(),
            "mode": self.ident_mode_var.get(),
            "pulse_us": int(self.ident_pulse_var.get()),
            "duration_ms": int(self.ident_duration_var.get()),
            "hold_ms": int(self.ident_hold_var.get()),
            "repeat": int(self.ident_repeat_var.get()),
            "bit_ms": int(self.ident_bit_var.get()),
            "seed": int(self.ident_seed_var.get()),
            "center": {
                "alpha_us": int(self.ident_alpha_center_var.get()),
                "beta_us": int(self.ident_beta_center_var.get()),
            },
            "airframe": self.airframe_info,
        }

    def _ident_write_meta(self) -> None:
        if self.ident_current_path is None:
            return
        meta_path = self.ident_current_path.with_name(f"{self.ident_current_path.stem}_meta.json")
        self.ident_current_meta_path = meta_path
        meta_path.write_text(
            json.dumps(self._ident_meta_payload(), ensure_ascii=False, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        if hasattr(self, "_ident_update_file_label"):
            self._ident_update_file_label()

    def _ident_update_file_label(self) -> None:
        csv_text = str(self.ident_current_path) if self.ident_current_path is not None else "none"
        meta_text = str(self.ident_current_meta_path) if self.ident_current_meta_path is not None else "none"
        self.ident_last_file_var.set(f"last csv: {csv_text}\nlast meta: {meta_text}")

    def _ident_begin_recording(self) -> None:
        self._ident_close_csv()
        path = self.ident_save_dir
        path.mkdir(parents=True, exist_ok=True)
        self.ident_current_path = path / f"ident_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        self.ident_current_meta_path = self.ident_current_path.with_name(f"{self.ident_current_path.stem}_meta.json")
        self.ident_csv_file = self.ident_current_path.open("w", newline="", encoding="utf-8")
        self.ident_csv_writer = csv.DictWriter(self.ident_csv_file, fieldnames=self._ident_csv_fields(), extrasaction="ignore")
        self.ident_csv_writer.writeheader()
        self.ident_samples.clear()
        self.ident_last_fit = None
        self.ident_sample_count_var.set("samples=0")
        self.ident_fit_var.set("no fit")
        self.ident_output_dir_var.set(f"save dir: {path}")
        self._ident_update_file_label()
        self._ident_write_meta()

    def _ident_close_csv(self) -> None:
        if self.ident_csv_file is not None:
            self.ident_csv_file.close()
        self.ident_csv_file = None
        self.ident_csv_writer = None

    def _ident_payload(self) -> str:
        axis = self.ident_axis_var.get()
        mode = self.ident_mode_var.get().upper()
        pulse = int(self.ident_pulse_var.get())
        if mode == "STEP":
            return f"IDENT STEP {axis} pulse_us={pulse} duration_ms={int(self.ident_duration_var.get())}"
        if mode == "DOUBLET":
            return f"IDENT DOUBLET {axis} pulse_us={pulse} hold_ms={int(self.ident_hold_var.get())} repeat={int(self.ident_repeat_var.get())}"
        return f"IDENT PRBS {axis} pulse_us={pulse} bit_ms={int(self.ident_bit_var.get())} duration_ms={int(self.ident_duration_var.get())} seed={int(self.ident_seed_var.get())}"

    def _on_ident_config_change(self, *_args) -> None:
        try:
            payload = self._ident_payload()
        except Exception:
            payload = "参数未完整"
        self.ident_command_preview_var.set(f"将发送: {payload}")
        self._ident_update_mode_fields()

    def _ident_update_mode_fields(self) -> None:
        if not self.ident_field_rows:
            return
        mode = self.ident_mode_var.get().upper()
        visible = {
            "STEP": {"pulse", "duration", "alpha_center", "beta_center"},
            "DOUBLET": {"pulse", "hold", "repeat", "alpha_center", "beta_center"},
            "PRBS": {"pulse", "duration", "bit", "seed", "alpha_center", "beta_center"},
        }.get(mode, set(self.ident_field_rows.keys()))
        hints = {
            "STEP": "STEP 用来看最基础的阶跃响应，适合第一次确认方向和估计 K/tau/L。",
            "DOUBLET": "DOUBLET 正负各打一段，能减少持续偏置，适合绑绳台架。",
            "PRBS": "PRBS 信息量大，建议等 STEP/DOUBLET 都正常后再用。",
        }
        self.ident_mode_hint_var.set(hints.get(mode, ""))
        for key, widgets in self.ident_field_rows.items():
            for widget in widgets:
                if key in visible:
                    widget.grid()
                else:
                    widget.grid_remove()

    def _ident_apply_preset(self, name: str) -> None:
        if name == "small":
            self.ident_mode_var.set("STEP")
            self.ident_pulse_var.set(20)
            self.ident_duration_var.set(3000)
        elif name == "step":
            self.ident_mode_var.set("STEP")
            self.ident_pulse_var.set(30)
            self.ident_duration_var.set(4000)
        elif name == "doublet":
            self.ident_mode_var.set("DOUBLET")
            self.ident_pulse_var.set(30)
            self.ident_hold_var.set(700)
            self.ident_repeat_var.set(2)
        self._on_ident_config_change()

    def _ident_run(self) -> None:
        payload = self._ident_payload()
        self.ident_current_command = payload
        self._ident_begin_recording()
        self._send_proto(PROTO_REQ_IDENT, payload)

    def _ident_send_center(self) -> None:
        payload = f"IDENT CENTER alpha_us={int(self.ident_alpha_center_var.get())} beta_us={int(self.ident_beta_center_var.get())}"
        self._send_proto(PROTO_REQ_IDENT, payload)

    def _update_airframe_line(self, line: str) -> None:
        record = airframe_record_from_line(line)
        if record is None:
            return
        self.airframe_info = record
        self.ident_link_var.set("AIRFRAME received")
        self.ident_airframe_var.set(
            f"AIRFRAME m={record.get('mass_kg', '-')}kg cg_z={record.get('cg_z_m', '-')}m "
            f"attach_cg={record.get('tether_attach_to_cg_m', '-')}m rope={record.get('rope_m', '-')}m "
            f"maxF={record.get('max_total_force_n', '-')}N hover={record.get('hover_thrust_pct', '-')}%"
        )

    def _ident_handle_line(self, line: str) -> None:
        values = parse_kv(line)
        if line.startswith("IDENT start "):
            self.ident_status_var.set("running")
            self.ident_reason_var.set("running")
            self.ident_link_var.set(f"IDENT running: {line}")
            self._ident_write_meta()
            return
        if line.startswith("IDENT done "):
            self.ident_status_var.set("done")
            self.ident_reason_var.set(values.get("reason", "complete"))
            self.ident_link_var.set(f"IDENT done: {line}")
            self._ident_close_csv()
            return
        if line.startswith("IDENT abort "):
            self.ident_status_var.set("aborted")
            self.ident_reason_var.set(values.get("reason", "abort"))
            self.ident_link_var.set(f"IDENT aborted: {line}")
            self._ident_close_csv()
            return
        if line.startswith("IDENT state="):
            self.ident_status_var.set(values.get("state", "-"))
            self.ident_reason_var.set(values.get("reason", "-"))
            self.ident_link_var.set(f"IDENT status: {line}")
            return
        record = ident_record_from_line(line)
        if record is None:
            return
        record["host_time"] = f"{time.time():.6f}"
        self.ident_samples.append(record)
        if len(self.ident_samples) > MAX_IDENT_SAMPLES:
            del self.ident_samples[: len(self.ident_samples) - MAX_IDENT_SAMPLES]
        if self.ident_csv_writer is None:
            self._ident_begin_recording()
        if self.ident_csv_writer is not None:
            self.ident_csv_writer.writerow(record)
            if self.ident_csv_file is not None:
                self.ident_csv_file.flush()
            self._ident_update_file_label()
        self.ident_sample_count_var.set(f"samples={len(self.ident_samples)}")
        self.ident_current_var.set(
            f"seq={record.get('seq', '-')} alpha={record.get('alpha_us', '-')} beta={record.get('beta_us', '-')} "
            f"roll={record.get('roll', '-')} pitch={record.get('pitch', '-')} gx={record.get('gx', '-')} gy={record.get('gy', '-')}"
        )
        self._ident_update_plot()

    def _ident_update_plot(self) -> None:
        if not HAS_MATPLOTLIB or getattr(self, "ident_axis_plot", None) is None or getattr(self, "ident_canvas", None) is None:
            return
        if not self.ident_samples:
            return
        axis_name = self.ident_axis_var.get()
        value_key = "roll" if axis_name == "roll" else "pitch"
        input_key = "alpha_us" if axis_name == "roll" else "beta_us"
        rows = [row for row in self.ident_samples if isinstance(row.get("t_ms"), int)]
        if not rows:
            return
        t0 = float(rows[0]["t_ms"]) / 1000.0
        xs = [(float(row["t_ms"]) / 1000.0) - t0 for row in rows]
        ys = [float(row.get(value_key, 0.0)) for row in rows]
        us = [float(row.get(input_key, 0.0)) for row in rows]
        axis = self.ident_axis_plot
        axis.clear()
        axis.plot(xs, ys, label=value_key)
        if us:
            base = us[0]
            axis.plot(xs, [(u - base) / 10.0 for u in us], label=f"{input_key} delta/10")
        axis.set_xlabel("s")
        axis.legend(loc="best")
        axis.grid(True, alpha=0.3)
        self.ident_canvas.draw_idle()

    def _ident_fit(self) -> None:
        fit = fit_ident_step(self.ident_samples, self.ident_axis_var.get())
        self.ident_last_fit = fit
        if fit is None:
            self.ident_fit_var.set("fit failed: need a clean step/doublet response")
            return
        self.ident_fit_var.set(
            f"K={fit['K']:.5f} tau={fit['tau']:.3f}s L={fit['L']:.3f}s "
            f"kp={fit['kp']:.5f} kd={fit['kd']:.5f}"
        )

    def _ident_apply_fit(self) -> None:
        if self.ident_last_fit is None:
            self._ident_fit()
        if self.ident_last_fit is None:
            return
        axis = self.ident_axis_var.get()
        fit = self.ident_last_fit
        payload = f"IDENT APPLY {axis} kp={fit['kp']:.6f} kd={fit['kd']:.6f}"
        self._send_proto(PROTO_REQ_IDENT, payload)

    def _ident_open_folder(self) -> None:
        path = self.ident_save_dir
        path.mkdir(parents=True, exist_ok=True)
        try:
            os.startfile(path)  # type: ignore[attr-defined]
        except Exception:
            messagebox.showinfo("IDENT", f"CSV folder:\n{path}")

    def _update_config_line(self, line: str) -> None:
        values = parse_kv(line)
        if "loaded" in values or "valid" in values:
            self.config_summary_var.set(
                f"配置 loaded={values.get('loaded', '-')} valid={values.get('valid', '-')} flash_st={values.get('flash_st', '-')}"
            )
            for key, value in values.items():
                self._set_param(f"config.{key}", value, "CFG", dirty=False)
            return

        parts = line.split()
        prefix: str | None = None
        index = -1
        if len(parts) >= 2 and parts[1].startswith("servo"):
            prefix = parts[1]
            index = safe_int(prefix.replace("servo", ""), -1)
        elif "slot" in values or "index" in values:
            index = safe_int(values.get("slot") or values.get("index"), -1)
            if index >= 0:
                prefix = f"servo{index}"

        if prefix is not None:
            for key, value in values.items():
                self._set_param(f"{prefix}.{key}", value, "CFG", dirty=False)
            if 0 <= index < len(self.servo_widgets):
                widgets = self.servo_widgets[index]
                servo_map = {"id": "id", "pulse": "pulse", "time": "time", "mode": "mode", "en": "enabled"}
                for src, dst in servo_map.items():
                    if src in values:
                        widgets[dst].set(safe_int(values[src]))

    def _update_param_line(self, line: str) -> None:
        values = parse_kv(line)
        tokens = line.split()
        record = tokens[1] if len(tokens) >= 2 and "=" not in tokens[1] else None
        name = values.get("name") or values.get("key")
        value = values.get("value") or values.get("val")
        if name is not None and value is not None:
            self._set_param(name, value, "PARAM", dirty=False)
            return

        if record is not None:
            for key, item_value in values.items():
                if key not in {"ok", "st", "count"}:
                    self._set_param(f"{record}.{key}", item_value, "PARAM", dirty=False)
            return

        for key, item_value in values.items():
            if key not in {"ok", "st", "count"}:
                self._set_param(key, item_value, "PARAM", dirty=False)

    def _update_pid_line(self, line: str) -> None:
        values = parse_kv(line)
        tokens = line.split()
        axis = values.get("axis")
        if axis is None and len(tokens) >= 2 and "=" not in tokens[1]:
            axis = tokens[1].lower()
        if axis is None:
            group = (values.get("group") or values.get("target") or "").lower()
            index = safe_int(values.get("index"), -1)
            if group in {"rate", "angle"} and 0 <= index < 3:
                axis = ("roll", "pitch", "yaw")[index]

        if axis in self.pid_vars:
            for term in ("kp", "ki", "kd"):
                if term in values:
                    self.pid_vars[axis][term].set(values[term])
                    self._set_param(f"pid.{axis}.{term}", values[term], "PID", dirty=False)
            return

        for key, value in values.items():
            if key in {"axis", "ok", "st"}:
                continue
            lowered = key.lower().replace("_", ".")
            if lowered.startswith("pid."):
                name = lowered
            elif "." in lowered:
                name = f"pid.{lowered}"
            else:
                name = f"pid.{key}"
            self._set_param(name, value, "PID", dirty=False)
            self._sync_pid_quick_var(name, value)

    def _sync_pid_quick_var(self, name: str, value: str) -> None:
        parts = name.lower().split(".")
        if len(parts) >= 3 and parts[-2] in self.pid_vars and parts[-1] in self.pid_vars[parts[-2]]:
            self.pid_vars[parts[-2]][parts[-1]].set(value)

    def _set_param(self, name: str, value: str, source: str, dirty: bool) -> None:
        self.params[name] = {"value": value, "source": source, "dirty": dirty}
        dirty_text = "yes" if dirty else ""
        iid = self.param_iids.get(name)
        if iid is None:
            iid = f"p{len(self.param_iids)}"
            self.param_iids[name] = iid
            self.param_names_by_iid[iid] = name
        if self.param_tree.exists(iid):
            self.param_tree.item(iid, text=name, values=(value, source, dirty_text))
        else:
            self.param_tree.insert("", tk.END, iid=iid, text=name, values=(value, source, dirty_text))
        self._sync_pid_quick_var(name, value)

    def _on_param_select(self, _event: tk.Event) -> None:
        selection = self.param_tree.selection()
        if not selection:
            return
        name = self.param_names_by_iid.get(selection[0], selection[0])
        self.param_name_var.set(name)
        self.param_value_var.set(str(self.params.get(name, {}).get("value", "")))

    def _stage_param_edit(self) -> None:
        name = self.param_name_var.get().strip()
        value = self.param_value_var.get().strip()
        if not name:
            messagebox.showerror("参数错误", "参数名不能为空")
            return
        self._set_param(name, value, "local", dirty=True)

    def _param_value_for_servo(self, index: int, param_key: str, widget_key: str, fallback: str) -> str:
        param_value = str(self.params.get(f"servo{index}.{param_key}", {}).get("value", "")).strip()
        if param_value:
            return param_value
        if 0 <= index < len(self.servo_widgets):
            return str(self.servo_widgets[index][widget_key].get()).strip()
        return fallback

    def _payload_for_param_edit(self, name: str, value: str) -> tuple[int, str]:
        lowered = name.strip().lower()
        parts = lowered.split(".")
        pid_aliases = {
            "pid.roll.kp": "coax.roll_angle_kp",
            "pid.pitch.kp": "coax.pitch_angle_kp",
            "pid.roll.kd": "coax.roll_rate_kd",
            "pid.pitch.kd": "coax.pitch_rate_kd",
            "pid.yaw.kp": "coax.yaw_angle_kp",
            "pid.yaw.kd": "coax.yaw_rate_kd",
            "pid.vel.x.kd": "coax.vel_x_kd",
            "pid.vel.y.kd": "coax.vel_y_kd",
            "pid.vel.z.kd": "coax.vel_z_kd",
            "pid.vel_loop.enable": "coax.vel_loop_enable",
            "pid.vel_loop.x.kp": "coax.vel_loop_x_kp",
            "pid.vel_loop.x.ki": "coax.vel_loop_x_ki",
            "pid.vel_loop.x.kd": "coax.vel_loop_x_kd",
            "pid.vel_loop.y.kp": "coax.vel_loop_y_kp",
            "pid.vel_loop.y.ki": "coax.vel_loop_y_ki",
            "pid.vel_loop.y.kd": "coax.vel_loop_y_kd",
        }
        if lowered in pid_aliases:
            payload = f"PARAM SET {pid_aliases[lowered]} {value}"
            return PROTO_REQ_PARAM_SET, payload
        if len(parts) == 2 and parts[0].startswith("servo") and parts[0][5:].isdigit():
            index = int(parts[0][5:])
            field = parts[1]
            if field == "id":
                payload = f"SERVO ID {index} {value}"
                return PROTO_REQ_SERVO_ID, payload
            if field in {"enabled", "en"}:
                payload = f"SERVO ENABLE {index} {value}"
                return PROTO_REQ_SERVO_ENABLE, payload
            if field == "mode":
                payload = f"SERVO MODE {index} {value}"
                return PROTO_REQ_SERVO_MODE, payload
            if field in {"pulse", "pulse_us"}:
                time_ms = self._param_value_for_servo(index, "time", "time", "500")
                payload = f"SERVO MOVE {index} {value} {time_ms}"
                return PROTO_REQ_SERVO_MOVE, payload
            if field in {"time", "time_ms"}:
                pulse = self._param_value_for_servo(index, "pulse", "pulse", "1500")
                payload = f"SERVO MOVE {index} {pulse} {value}"
                return PROTO_REQ_SERVO_MOVE, payload

        payload = f"PARAM SET {name} {value}"
        return PROTO_REQ_PARAM_SET, payload

    def _send_param_edit(self) -> None:
        self._stage_param_edit()
        name = self.param_name_var.get().strip()
        value = self.param_value_var.get().strip()
        if name:
            function, payload = self._payload_for_param_edit(name, value)
            self._send_proto_once(function, payload, payload)

    def _send_pid_values(self) -> None:
        for axis, terms in self.pid_vars.items():
            parts = []
            for term in ("kp", "ki", "kd"):
                value = terms[term].get().strip()
                if value:
                    parts.append(f"{term}={value}")
                    self._set_param(f"pid.{axis}.{term}", value, "local", dirty=True)
            if parts:
                payload = f"PID SET {axis} {' '.join(parts)}"
                self._send_proto(PROTO_REQ_PID_SET, payload, payload)

    def _stage_text(self, stage: str) -> str:
        mapping = {
            "ready": "初始化完成",
            "probe": "读取芯片 ID",
            "status": "读取状态寄存器",
            "read": "读取数据",
            "init": "初始化/识别",
            "txrx": "发送接收",
            "split": "分段读写",
        }
        return mapping.get(stage, stage)

    def _hardware_hint(self, key: str, ok: bool, _values: dict[str, str]) -> str:
        if ok:
            return "初始化成功，当前读数符合预期。"
        if key == "FLASH":
            return "优先看 SPI1/CS/MISO/MOSI 和 GD25Q32 供电；ID 应为 C84016。"
        if key == "SPL06":
            return "重点看 SPI4、CS、MISO、器件方向或焊接型号；SPL06 ID 通常应为 0x10。"
        if key == "ICM42688":
            return "重点看 SPI2/IMU 硬件链路；CHIP_ID ?? 0xA1。"
        if key == "GPS":
            return "确认 M9N 接 USART2: PD5 TX 到 GPS RX，PD6 RX 接 GPS TX，115200 8N1，且 USART2 IRQ 已开。"
        if key == "MAG":
            return "确认磁力计在 I2C1: PB6 SCL / PB7 SDA，外部上拉、电源和地址匹配。"
        if key == "UART1":
            return "确认 USART1: PB14 TX 接 Ai-WB2 RX，PB15 RX 接 Ai-WB2 TX，115200 8N1。"
        if key == "WIFI":
            return "确认上位机 TCP 服务先监听 6666；固件会用 PC6 自动重启 Ai-WB2 触发重连。"
        return "查看返回码和初始化阶段。"

    def _warn_if_no_reply(self, sent: str, started_at: float) -> None:
        if self.last_reply_rx >= started_at:
            return
        if self.last_board_rx >= started_at:
            return
        if (time.monotonic() - started_at) < (CMD_REPLY_TIMEOUT_MS / 1000.0):
            return
        if self._transport_connected():
            mode = "TCP" if self.transport is self.tcp_transport else "串口"
            self.link_var.set(f"{mode} 已连接但超时未收到 STM32 回复，请检查固件")
            self._append(f"[上位机] {sent} 超时: STM32 未通过{mode}回复，请检查固件是否运行")

    def _check_link_health(self) -> None:
        if not self._transport_connected():
            if self.transport is self.tcp_transport:
                self.link_var.set("等待 Ai-WB2 接入 TCP")
            else:
                self.link_var.set("串口未打开")
        elif self.last_board_rx == 0.0:
            if self.transport is self.tcp_transport:
                self.link_var.set("TCP 已连接，等待 STM32 数据")
            elif self.transport is self.udp_transport:
                self.link_var.set("UDP 已打开，等待 STM32 文本回复")
            else:
                self.link_var.set("串口已打开，等待 STM32 数据")
        elif (time.monotonic() - self.last_board_rx) > 5.0:
            self.link_var.set("超过 5 秒未收到 STM32 数据")
        self.after(1000, self._check_link_health)

    def _drain_rx(self) -> None:
        try:
            while True:
                item = self.rx_queue.get_nowait()
                if isinstance(item, tuple) and len(item) == 3 and item[0] == "proto":
                    _tag, function, text = item
                    self._handle_proto_frame(int(function), str(text))
                    continue
                if isinstance(item, tuple) and len(item) == 4 and item[0] == "udp_raw":
                    _tag, host, port, size = item
                    self.udp_raw_hidden_count += 1
                    self.udp_raw_last_note = f"hidden UDP binary frames={self.udp_raw_hidden_count} last={host}:{port} {size}B"
                    self.ident_link_var.set(self.udp_raw_last_note)
                    continue

                line = str(item)
                self._append(line)
                self._handle_board_line(line)
        except queue.Empty:
            pass
        self.after(100, self._drain_rx)

    def _on_close(self) -> None:
        self._stop()
        self._ident_close_csv()
        self.destroy()


def main() -> None:
    app = DronePanel()
    app.mainloop()


if __name__ == "__main__":
    main()
