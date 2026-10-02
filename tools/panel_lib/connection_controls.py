"""Connection controls and device selection, independent of the panel's feature pages."""
import tkinter as tk
from tkinter import ttk, messagebox
from . import transport as _panel_transport
from .transport import serial_port_identity, match_remembered_serial_port, TransportBase
from .proto import PROTO_REQ_PING
from . import link_failover as _link_failover
from .bluetooth_channel import BluetoothChannelMixin, channel_mode

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 6666


class ConnectionControlsMixin(BluetoothChannelMixin):
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
        if "BLUETOOTH" in combined or "BTHENUM" in combined or "蓝牙" in combined:
            return f"蓝牙串口 ({p.device})"
        if "STLINK" in combined or "ST-LINK" in combined:
            return f"STLink ({p.device})"
        if desc and desc not in ("USB SERIAL DEVICE", "USB SERIAL", "SERIAL"):
            return f"{p.device} - {p.description}"
        return p.device

    def _restore_last_connection(self) -> None:
        """启动时按上次的记录自动连回去。"""
        state = self._panel_state
        if not state:
            self.autoconnect_var.set("上次连接：无记录")
            return
        transport = str(state.get("transport") or "")
        if transport in {"bluetooth", "蓝牙"}:
            if isinstance(state.get("serial_baud"), int):
                self.serial_baud_var.set(state["serial_baud"])
            self.transport_var.set("蓝牙")
            self._request_bluetooth_scan(connect=bool(self.auto_connect_var.get()),
                                         preferred=str(state.get("bluetooth_address") or ""))
            return
        if transport in {"serial", "tcp", "udp"}:
            self.transport_var.set(transport)
        baud = state.get("serial_baud")
        if isinstance(baud, int):
            self.serial_baud_var.set(baud)
        if transport != "serial":
            self.autoconnect_var.set(f"上次连接：{transport or '未知'} 通道")
            if self.auto_connect_var.get() and transport == "tcp":
                self.autoconnect_var.set("上次连接：tcp，正在监听")
                self.after(300, self._start)
            return

        names = self._refresh_serial_ports()
        if hasattr(self, "_serial_port_combo"):
            self._serial_port_combo["values"] = names
        device, reason = match_remembered_serial_port(
            str(state.get("serial_port") or ""),
            str(state.get("serial_fingerprint") or ""),
            self._serial_port_identity,
        )
        if device is None:
            self.autoconnect_var.set(f"上次连接：{reason}")
            return
        label = next(
            (name for name, mapped in self._serial_port_map.items()
             if str(mapped).casefold() == device.casefold()),
            None,
        )
        if label is None:
            self.autoconnect_var.set(f"上次连接：{reason}，但下拉列表里没有它")
            return
        self.serial_port_var.set(label)
        if not self.auto_connect_var.get():
            self.autoconnect_var.set(f"上次连接：{reason}（自动连接已关闭）")
            return
        self.autoconnect_var.set(f"上次连接：{reason}，正在连接…")
        # 延后一拍再连：让主窗口先画出来，否则连接失败的弹窗会挡在空白窗口上。
        self.after(300, self._auto_connect_now)

    def _auto_connect_now(self) -> None:
        if self._transport_connected():
            return
        self._start()
        if self._transport_connected():
            self.autoconnect_var.set(
                f"上次连接：已自动重连 {self.serial_transport.active_port or ''}"
            )
        else:
            self.autoconnect_var.set("上次连接：自动连接失败，请手动选择串口")

    def _refresh_serial_ports(self) -> list[str]:
        if not _panel_transport.HAS_PYSERIAL or _panel_transport.serial is None:
            return []
        try:
            ports = list(  # type: ignore[union-attr]
                _panel_transport.serial.tools.list_ports.comports()
            )
        except Exception:
            return []
        self._serial_port_map.clear()
        self._serial_port_identity.clear()
        names: list[str] = []
        for p in ports:
            label = self._build_port_label(p)
            self._serial_port_map[label] = p.device
            self._serial_port_identity[str(p.device)] = serial_port_identity(p)
            names.append(label)
        return names

    def _default_serial_port(self) -> str:
        names = self._refresh_serial_ports()
        if names:
            return names[0]
        return "COM18"

    def _on_refresh_ports(self) -> None:
        if self.serial_transport.is_connected:
            return
        names = self._refresh_serial_ports()
        if hasattr(self, '_serial_port_combo'):
            self._serial_port_combo['values'] = names
        if names:
            self.serial_port_var.set(names[0])

    def _refresh_serial_selection_lock(self) -> None:
        if not hasattr(self, "_serial_port_combo"):
            return
        connected = self.serial_transport.is_connected
        self._serial_port_combo.configure(state=tk.DISABLED if connected else "readonly")
        self._serial_refresh_button.configure(state=tk.DISABLED if connected else tk.NORMAL)
        if hasattr(self, "_transport_combo"):
            self._transport_combo.configure(state=tk.DISABLED if self._transport_connected() else "readonly")

    def _on_serial_port_selection_change(self, *_args: object) -> None:
        self.firmware_unknown_usb_override_var.set(False)
        self._firmware_refresh_safety()

    def _on_transport_mode_change(self, *args) -> None:
        self._tcp_frame.pack_forget()
        self._udp_frame.pack_forget()
        self._serial_frame.pack_forget()
        self._bluetooth_frame.pack_forget()

        mode = channel_mode(self)
        if mode != "bluetooth":
            self._cancel_bluetooth()
            if hasattr(self, "_bt_prior_baud"):
                self.serial_baud_var.set(self._bt_prior_baud)
                del self._bt_prior_baud
        if mode == "serial":
            self._serial_frame.pack(side=tk.LEFT, padx=(0, 12), before=self._action_frame)
            self._on_refresh_ports()
        elif mode == "bluetooth":
            if not hasattr(self, "_bt_prior_baud"):
                self._bt_prior_baud=self.serial_baud_var.get()
            self._bluetooth_frame.pack(side=tk.LEFT, padx=(0, 12), before=self._action_frame)
            self._request_bluetooth_scan()
        elif mode == "udp":
            self._udp_frame.pack(side=tk.LEFT, padx=(0, 12), before=self._action_frame)
        else:
            self._tcp_frame.pack(side=tk.LEFT, padx=(0, 12), before=self._action_frame)

    def _current_transport(self) -> TransportBase:
        mode = channel_mode(self)
        if mode in {"serial", "bluetooth"}:
            return self.serial_transport
        if mode == "udp":
            return self.udp_transport
        return self.tcp_transport

    def _transport_connected(self) -> bool:
        return self.transport.is_connected

    def _build_connection_bar(self, parent: ttk.Frame) -> None:
        conn = ttk.Frame(parent)
        conn.pack(fill=tk.X)

        ttk.Label(conn, text="通道").pack(side=tk.LEFT)
        self._transport_combo = ttk.Combobox(
            conn,
            textvariable=self.transport_var,
            values=("tcp", "udp", "serial", "蓝牙"),
            width=8,
            state="readonly",
        )
        self._transport_combo.pack(side=tk.LEFT, padx=(4, 12))

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
            state="readonly",
        )
        self._serial_port_combo.pack(side=tk.LEFT, padx=(4, 2))
        self._serial_refresh_button = ttk.Button(
            self._serial_frame, text="刷新", command=self._on_refresh_ports
        )
        self._serial_refresh_button.pack(side=tk.LEFT)
        ttk.Label(self._serial_frame, text="波特率").pack(side=tk.LEFT)
        ttk.Entry(self._serial_frame, textvariable=self.serial_baud_var, width=8).pack(side=tk.LEFT, padx=(4, 0))
        self._build_bluetooth_controls(conn)

        # Show TCP frame initially
        self._tcp_frame.pack(side=tk.LEFT, padx=(0, 12))

        # --- Action buttons (always visible) ---
        self._action_frame = ttk.Frame(conn)
        self._action_frame.pack(side=tk.LEFT)
        ttk.Button(self._action_frame, text="启动连接", command=self._start,
                   style="Primary.TButton").pack(side=tk.LEFT)
        ttk.Button(self._action_frame, text="停止", command=self._stop,
                   style="Danger.TButton").pack(side=tk.LEFT, padx=6)

        utility = ttk.Frame(parent)
        utility.pack(fill=tk.X, pady=(7, 0))
        ttk.Label(utility, text="快捷诊断", style="Eyebrow.TLabel").pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(utility, text="PING", command=lambda: self._send_proto(PROTO_REQ_PING, "PING"),
                   style="Secondary.TButton").pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(utility, text="硬件状态", command=self._request_overview_status,
                   style="Secondary.TButton").pack(side=tk.LEFT, padx=4)
        ttk.Button(utility, text="读取配置", command=self._read_all_params,
                   style="Secondary.TButton").pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(
            utility,
            text="原始日志",
            variable=self.show_log_var,
            command=self._toggle_log_area,
        ).pack(side=tk.LEFT, padx=(10, 2))
        ttk.Checkbutton(
            utility,
            text="启动自动重连",
            variable=self.auto_connect_var,
            command=self._save_panel_state,
        ).pack(side=tk.LEFT, padx=(10, 2))
        _link_failover.install(self)
        ttk.Checkbutton(
            utility,
            text="USB断开自动切蓝牙",
            variable=self.usb_failover_var,
            command=self._save_panel_state,
        ).pack(side=tk.LEFT, padx=(10, 2))
        ttk.Label(utility, text="链路").pack(side=tk.LEFT, padx=(18, 4))
        self.link_status_label = ttk.Label(
            utility, textvariable=self.link_var, style="Fail.TLabel")
        self.link_status_label.pack(side=tk.LEFT)

        autoconnect = ttk.Frame(parent)
        autoconnect.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(
            autoconnect, textvariable=self.autoconnect_var, style="Muted.TLabel",
        ).pack(side=tk.LEFT)

        self.transport_var.trace_add("write", self._on_transport_mode_change)

    def _start(self) -> None:
        if getattr(self, "logs_page", None) is not None and not self.logs_page.allow_main_connect():
            return
        if self.v1_worker is not None and self.v1_worker.is_alive():
            messagebox.showwarning("IMU 校准正在占用 USB CDC", "请先完成当前 IMU 采集，再启动面板连接。")
            return
        if channel_mode(self) == "bluetooth":
            self._request_bluetooth_scan(connect=True)
            return
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
        self._refresh_serial_selection_lock()
        if self.serial_transport.is_connected:
            # 只记成功连上的那一次；连失败还记下来会让下次启动一直去撞同一个坏口。
            self._save_panel_state()

