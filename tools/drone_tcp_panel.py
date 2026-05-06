#!/usr/bin/env python3
"""TCP ground-station panel for the drone-H743 Ai-WB2 transparent link."""

from __future__ import annotations

import csv
import queue
import re
import socket
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 6666
MAX_BARO_SAMPLES = 2000
CMD_REPLY_TIMEOUT_MS = 2500
PROTO_COMPAT_FALLBACK_DELAY_MS = 400
PROTO_PROBE_SETTLE_MS = 900
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


MODULES = [
    ("FLASH", "GD25Q32 Flash", "STATUS?"),
    ("SPL06", "SPL06 气压计", "STATUS?"),
    ("ICM42688", "ICM42688 IMU", "STATUS?"),
    ("UART1", "USART1 链路", "STATUS?"),
]

MODULE_ALIASES = {
    "FLASH": "FLASH",
    "GD25Q32": "FLASH",
    "SPL06": "SPL06",
    "BARO": "SPL06",
    "ICM42688": "ICM42688",
    "IMU": "ICM42688",
    "UART1": "UART1",
    "USART1": "UART1",
}


def parse_kv(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for match in re.finditer(r"([A-Za-z0-9_.-]+)=([^ \r\n]+)", line):
        result[match.group(1)] = match.group(2)
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


class TcpServer:
    def __init__(self, rx_queue: "queue.Queue[str]") -> None:
        self.rx_queue = rx_queue
        self.sock: socket.socket | None = None
        self.client: socket.socket | None = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    def start(self, host: str, port: int) -> None:
        self.stop()
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, args=(host, port), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
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
        payload = build_proto_frame(
            PROTO_DIR_TO_FC,
            function,
            payload,
        )
        with self.lock:
            client = self.client
        if client is None:
            return False
        try:
            client.sendall(payload)
            return True
        except OSError as exc:
            self.rx_queue.put(f"[上位机] 发送失败: {exc}")
            return False

    def send_line(self, line: str) -> bool:
        return self.send_frame(
            PROTO_MSG_CMD_LINE,
            line.rstrip("\r\n").encode("utf-8"),
        )

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
        with self.lock:
            if self.client is client:
                self.client = None
        try:
            client.close()
        except OSError:
            pass
        self.rx_queue.put("[上位机] 板子已断开")


class DronePanel(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("drone-H743 TCP 地面站")
        self.geometry("1280x820")
        self.minsize(1080, 720)

        self.rx_queue: "queue.Queue[str]" = queue.Queue()
        self.server = TcpServer(self.rx_queue)
        self.last_board_rx = 0.0
        self.last_reply_rx = 0.0
        self.selected_module = "FLASH"
        self.baro_capture_enabled = tk.BooleanVar(value=True)
        self.baro_buffer: list[dict[str, float | str]] = []
        self.params: dict[str, dict[str, str | bool]] = {}
        self.param_iids: dict[str, str] = {}
        self.param_names_by_iid: dict[str, str] = {}
        self.servo_widgets: list[dict[str, tk.Variable]] = []
        self.structured_protocol_supported: bool | None = None

        self.link_var = tk.StringVar(value="未连接")
        self.last_cmd_var = tk.StringVar(value="-")
        self.config_summary_var = tk.StringVar(value="尚未读取配置")
        self.baro_count_var = tk.StringVar(value="暂存样本: 0")
        self.plot_var = tk.StringVar(value="pressure")

        self.module_state: dict[str, dict[str, tk.StringVar]] = {}
        self.baro_vars: dict[str, tk.StringVar] = {}

        self._build_ui()
        self.after(1000, self._check_link_health)
        self.after(100, self._drain_rx)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

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
        params = ttk.Frame(self.notebook, padding=10)
        servos = ttk.Frame(self.notebook, padding=10)
        commands = ttk.Frame(self.notebook, padding=10)
        self.baro_tab = baro

        self.notebook.add(overview, text="模块总览")
        self.notebook.add(baro, text="SPL06 气压计")
        self.notebook.add(params, text="参数 / PID")
        self.notebook.add(servos, text="舵机控制")
        self.notebook.add(commands, text="日志 / 原始命令")

        self._build_overview_page(overview)
        self._build_baro_page(baro)
        self._build_params_page(params)
        self._build_servo_page(servos)
        self._build_command_page(commands)

        log_box = ttk.LabelFrame(body, text="原始命令日志", padding=8)
        body.add(log_box, weight=2)
        self._build_log_area(log_box)

    def _build_connection_bar(self, parent: ttk.Frame) -> None:
        conn = ttk.Frame(parent)
        conn.pack(fill=tk.X)

        ttk.Label(conn, text="监听地址").pack(side=tk.LEFT)
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(conn, textvariable=self.host_var, width=16).pack(side=tk.LEFT, padx=(4, 12))

        ttk.Label(conn, text="端口").pack(side=tk.LEFT)
        self.port_var = tk.IntVar(value=DEFAULT_PORT)
        ttk.Entry(conn, textvariable=self.port_var, width=8).pack(side=tk.LEFT, padx=(4, 12))

        ttk.Button(conn, text="启动 TCP 服务", command=self._start).pack(side=tk.LEFT)
        ttk.Button(conn, text="停止", command=self._stop).pack(side=tk.LEFT, padx=6)
        ttk.Button(conn, text="PING", command=lambda: self._send_proto(PROTO_REQ_PING, "PING")).pack(side=tk.LEFT, padx=(18, 4))
        ttk.Button(conn, text="硬件状态", command=self._request_overview_status).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="读取配置", command=self._read_all_params).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="保存到 Flash", command=lambda: self._send_proto(PROTO_REQ_SAVE, "SAVE")).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="从 Flash 读取", command=lambda: self._send_proto(PROTO_REQ_LOAD, "LOAD")).pack(side=tk.LEFT, padx=4)

        ttk.Label(conn, text="链路").pack(side=tk.LEFT, padx=(18, 4))
        ttk.Label(conn, textvariable=self.link_var).pack(side=tk.LEFT)

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
        detail.columnconfigure(1, weight=1)

    def _detail_row(self, parent: ttk.Frame, row: int, label: str, variable: tk.StringVar, wrap: int = 0) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.NW, padx=(0, 8), pady=4)
        ttk.Label(parent, textvariable=variable, wraplength=wrap).grid(row=row, column=1, sticky=tk.EW, pady=4)

    def _build_baro_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="请求状态", command=lambda: self._send_proto(PROTO_REQ_STATUS, "STATUS?")).pack(side=tk.LEFT)
        ttk.Button(top, text="请求气压计", command=lambda: self._send_proto(PROTO_REQ_BARO, "BARO?")).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="开始流", command=lambda: self._send_proto(PROTO_REQ_BARO_STREAM, "BARO STREAM 1", "BARO STREAM 1")).pack(side=tk.LEFT)
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

    def _build_params_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="读取参数/PID", command=self._read_all_params).pack(side=tk.LEFT)
        ttk.Button(top, text="保存到 Flash", command=lambda: self._send_proto(PROTO_REQ_SAVE, "SAVE")).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="从 Flash 读取", command=lambda: self._send_proto(PROTO_REQ_LOAD, "LOAD")).pack(side=tk.LEFT)
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
            ("PID?", "PID?"),
            ("BARO?", "BARO?"),
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
        try:
            port = int(self.port_var.get())
        except tk.TclError:
            messagebox.showerror("端口错误", "端口号无效")
            return
        self.server.start(self.host_var.get(), port)

    def _stop(self) -> None:
        self.server.stop()

    def _send(self, line: str) -> None:
        self.last_cmd_var.set(f"最近发送: {line}")
        self._append(f"> {line}")
        if not self.server.send_line(line):
            self._append("[上位机] 板子尚未连接")
        elif line in {"PING", "STATUS?", "CONFIG?", "PARAM?", "PID?", "BARO?"}:
            sent_at = time.monotonic()
            self.after(CMD_REPLY_TIMEOUT_MS, lambda sent=line, start=sent_at: self._warn_if_no_reply(sent, start))

    def _send_proto(self, function: int, label: str, payload: str = "", expect_reply: bool = True) -> None:
        payload_text = payload if payload else label
        self.last_cmd_var.set(f"最近发送: {label}")
        self._append(f"> {label}")
        if not self.server.send_frame(function, payload_text.encode("utf-8")):
            self._append("[上位机] 板子尚未连接")
            return
        sent_at = time.monotonic()
        if self.structured_protocol_supported is False:
            self.server.send_line(payload_text)
        elif self.structured_protocol_supported is None and function != PROTO_REQ_CAPS:
            self.after(
                PROTO_COMPAT_FALLBACK_DELAY_MS,
                lambda legacy=payload_text, start=sent_at: self._fallback_proto_request(legacy, start),
            )
        if expect_reply:
            self.after(CMD_REPLY_TIMEOUT_MS, lambda sent=label, start=sent_at: self._warn_if_no_reply(sent, start))

    def _fallback_proto_request(self, legacy_line: str, started_at: float) -> None:
        if self.server.client is None:
            return
        if self.structured_protocol_supported is True:
            return
        if self.last_reply_rx >= started_at:
            return
        self.server.send_line(legacy_line)

    def _begin_protocol_probe(self) -> None:
        self.structured_protocol_supported = None
        if self.server.client is None:
            return

        started_at = time.monotonic()
        self.server.send_frame(PROTO_REQ_CAPS, b"CAPS?")
        self.after(
            PROTO_COMPAT_FALLBACK_DELAY_MS,
            lambda start=started_at: self._fallback_proto_request("CAPS?", start),
        )
        self.after(PROTO_PROBE_SETTLE_MS, self._finalize_protocol_probe)

    def _finalize_protocol_probe(self) -> None:
        if self.server.client is None:
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
            self.cmd_var.set("")

    def _read_all_params(self) -> None:
        self._send_proto(PROTO_REQ_CONFIG, "CONFIG?")
        self._send_proto(PROTO_REQ_PARAMS, "PARAM?")
        self._send_proto(PROTO_REQ_PID, "PID?")

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

    def _append(self, line: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.status_text.insert(tk.END, f"[{stamp}] {line}\n")
        self.status_text.see(tk.END)

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
        self._send_proto(PROTO_REQ_STATUS, "STATUS?")

    def _open_baro_tab(self) -> None:
        self.notebook.select(self.baro_tab)
        self._send_proto(PROTO_REQ_BARO, "BARO?")

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
        if line.startswith("[上位机] 板子已连接"):
            self._begin_protocol_probe()
            return
        if line.startswith("[上位机] 板子已断开"):
            self.structured_protocol_supported = None
            return

        if not line.startswith("[上位机]"):
            self.last_board_rx = time.monotonic()
            self.link_var.set("已收到 STM32 数据，TCP 与串口透明链路正常")

        if line.startswith("READY"):
            self.last_board_rx = time.monotonic()
            self.link_var.set("STM32 心跳正常")
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
        elif line.startswith("PID "):
            self._update_pid_line(line)
        elif line.startswith("FLASH "):
            self._update_flash_line(line)
        elif line.startswith("BARO ") or line.startswith("SPL06 "):
            self._update_baro_line(line)
        elif line.startswith("IMU "):
            self._update_imu_line(line)
        elif line.startswith("RSP "):
            self._handle_rsp_line(line)
        elif line.startswith("OK ") or line.startswith("ERR ") or line.startswith("ACK ") or line.startswith("RX "):
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近回包: {line}")
        elif line.startswith("PONG ") or line.startswith("READY ") or line.startswith("FLASH ") or line.startswith("BARO ") or line.startswith("IMU "):
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

        if mod == "FLASH":
            self._update_flash_line("FLASH " + " ".join(f"{key}={value}" for key, value in payload.items()))

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

        if "cfg_valid" in values:
            self._set_param("config.valid", values["cfg_valid"], "MODULES", dirty=False)
        if "cfg_loaded" in values:
            self._set_param("config.loaded", values["cfg_loaded"], "MODULES", dirty=False)
        if "servo_slots" in values:
            self._set_param("system.servo_slots", values["servo_slots"], "MODULES", dirty=False)
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
        self._refresh_baro_samples()
        self._update_baro_plot()

    def _refresh_baro_samples(self) -> None:
        self.baro_count_var.set(f"暂存样本: {len(self.baro_buffer)}")
        for iid in self.baro_sample_tree.get_children():
            self.baro_sample_tree.delete(iid)
        latest = self.baro_buffer[-80:]
        base = float(latest[0]["time"]) if latest else 0.0
        for sample in latest:
            t = float(sample["time"]) - base
            self.baro_sample_tree.insert(
                "",
                tk.END,
                values=(
                    f"{t:.2f}",
                    self._fmt_sample(sample, "pressure"),
                    self._fmt_sample(sample, "temperature"),
                    self._fmt_sample(sample, "altitude"),
                ),
            )

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
        self._refresh_baro_samples()
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

    def _send_param_edit(self) -> None:
        self._stage_param_edit()
        name = self.param_name_var.get().strip()
        value = self.param_value_var.get().strip()
        if name:
            payload = f"PARAM SET {name} {value}"
            self._send_proto(PROTO_REQ_PARAM_SET, payload, payload)

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
            return "重点看 SPI2/IMU 硬件链路；WHO_AM_I 应为 0x47。"
        if key == "UART1":
            return "确认 USART1: PB14 TX 接 Ai-WB2 RX，PB15 RX 接 Ai-WB2 TX，115200 8N1。"
        return "查看返回码和初始化阶段。"

    def _warn_if_no_reply(self, sent: str, started_at: float) -> None:
        if self.last_reply_rx >= started_at:
            return
        if self.last_board_rx >= started_at:
            return
        if (time.monotonic() - started_at) < (CMD_REPLY_TIMEOUT_MS / 1000.0):
            return
        if self.server.client is not None:
            self.link_var.set("Ai-WB2 已连上 TCP，但没有收到 STM32 回复；请检查 USART1 RX/TX 透明串口链路")
            self._append(f"[上位机] {sent} 没收到 STM32 回复：TCP 客户端可能只是 Wi-Fi 模块，串口到 STM32 未通")

    def _check_link_health(self) -> None:
        if self.server.client is None:
            self.link_var.set("等待 Ai-WB2 连接 TCP")
        elif self.last_board_rx == 0.0:
            self.link_var.set("Ai-WB2 已连上 TCP，尚未收到 STM32 数据")
        elif (time.monotonic() - self.last_board_rx) > 5.0:
            self.link_var.set("超过 5 秒未收到 STM32 心跳")
        self.after(1000, self._check_link_health)

    def _drain_rx(self) -> None:
        try:
            while True:
                item = self.rx_queue.get_nowait()
                if isinstance(item, tuple) and len(item) == 3 and item[0] == "proto":
                    _tag, function, text = item
                    self._handle_proto_frame(int(function), str(text))
                    continue

                line = str(item)
                self._append(line)
                self._handle_board_line(line)
        except queue.Empty:
            pass
        self.after(100, self._drain_rx)

    def _on_close(self) -> None:
        self.server.stop()
        self.destroy()


def main() -> None:
    app = DronePanel()
    app.mainloop()


if __name__ == "__main__":
    main()
