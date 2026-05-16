#!/usr/bin/env python3
"""Tkinter GUI for the RS485 Modbus pressure/weight transmitter."""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial is required: pip install pyserial") from exc

from pressure_rs485_test import (
    DEFAULT_BAUD,
    DEFAULT_DIP,
    DEFAULT_TIMEOUT,
    dip_to_addr,
    read_channel_weight,
    read_holding_registers,
    scan_addresses,
    write_single_register,
)


class PressureGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("RS485 Pressure Sensor Test")
        self.geometry("860x560")
        self.minsize(760, 480)

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: threading.Thread | None = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.dip_var = tk.StringVar(value=DEFAULT_DIP)
        self.addr_var = tk.StringVar(value="")
        self.channel_var = tk.StringVar(value="1")
        self.interval_var = tk.StringVar(value="0.2")
        self.timeout_var = tk.StringVar(value=str(DEFAULT_TIMEOUT))
        self.raw_var = tk.BooleanVar(value=True)
        self.value_var = tk.StringVar(value="--")
        self.status_var = tk.StringVar(value="Idle")

        self._build_ui()
        self.refresh_ports()
        self.after(60, self._poll_events)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=12)
        root.pack(fill=tk.BOTH, expand=True)

        cfg = ttk.LabelFrame(root, text="Connection")
        cfg.pack(fill=tk.X)

        ttk.Label(cfg, text="Port").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        self.port_combo = ttk.Combobox(cfg, textvariable=self.port_var, width=14)
        self.port_combo.grid(row=0, column=1, sticky="w", padx=6, pady=6)
        ttk.Button(cfg, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=6, pady=6)

        ttk.Label(cfg, text="Baud").grid(row=0, column=3, sticky="w", padx=6, pady=6)
        ttk.Entry(cfg, textvariable=self.baud_var, width=10).grid(row=0, column=4, sticky="w", padx=6, pady=6)

        ttk.Label(cfg, text="DIP").grid(row=0, column=5, sticky="w", padx=6, pady=6)
        ttk.Entry(cfg, textvariable=self.dip_var, width=8).grid(row=0, column=6, sticky="w", padx=6, pady=6)

        ttk.Label(cfg, text="Addr").grid(row=1, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(cfg, textvariable=self.addr_var, width=8).grid(row=1, column=1, sticky="w", padx=6, pady=6)

        ttk.Label(cfg, text="Channel").grid(row=1, column=3, sticky="w", padx=6, pady=6)
        ttk.Spinbox(cfg, from_=1, to=4, textvariable=self.channel_var, width=8).grid(row=1, column=4, sticky="w", padx=6, pady=6)

        ttk.Label(cfg, text="Interval s").grid(row=1, column=5, sticky="w", padx=6, pady=6)
        ttk.Entry(cfg, textvariable=self.interval_var, width=8).grid(row=1, column=6, sticky="w", padx=6, pady=6)

        ttk.Checkbutton(cfg, text="Raw TX/RX", variable=self.raw_var).grid(row=1, column=7, sticky="w", padx=6, pady=6)

        readout = ttk.Frame(root)
        readout.pack(fill=tk.X, pady=(12, 8))

        value_box = ttk.LabelFrame(readout, text="Realtime Value")
        value_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ttk.Label(value_box, textvariable=self.value_var, font=("Consolas", 34, "bold")).pack(padx=18, pady=18)

        buttons = ttk.LabelFrame(readout, text="Actions")
        buttons.pack(side=tk.LEFT, fill=tk.Y, padx=(12, 0))
        self.start_btn = ttk.Button(buttons, text="Start", command=self.start_reading)
        self.start_btn.pack(fill=tk.X, padx=10, pady=(10, 5))
        self.stop_btn = ttk.Button(buttons, text="Stop", command=self.stop_reading, state=tk.DISABLED)
        self.stop_btn.pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(buttons, text="Read Once", command=self.read_once).pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(buttons, text="Scan Addr", command=self.scan_addr).pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(buttons, text="Tare", command=lambda: self.write_command(2)).pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(buttons, text="Zero", command=lambda: self.write_command(1)).pack(fill=tk.X, padx=10, pady=(5, 10))

        log_frame = ttk.LabelFrame(root, text="Log")
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(log_frame, height=14, wrap=tk.NONE, font=("Consolas", 10))
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=scroll.set)

        status = ttk.Label(root, textvariable=self.status_var, anchor="w")
        status.pack(fill=tk.X, pady=(6, 0))

    def refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        self._log(f"Ports: {', '.join(ports) if ports else '(none)'}")

    def _settings(self) -> tuple[str, int, int, int, float, float]:
        port = self.port_var.get().strip()
        if not port:
            raise ValueError("serial port is empty")
        baud = int(self.baud_var.get(), 0)
        channel = int(self.channel_var.get(), 0)
        interval = float(self.interval_var.get())
        timeout = float(self.timeout_var.get())
        addr_text = self.addr_var.get().strip()
        addr = int(addr_text, 0) if addr_text else dip_to_addr(self.dip_var.get())
        if not 1 <= addr <= 254:
            raise ValueError("address must be 1..254")
        if not 1 <= channel <= 4:
            raise ValueError("channel must be 1..4")
        return port, baud, addr, channel, interval, timeout

    def _open_serial(self) -> serial.Serial:
        port, baud, _addr, _channel, _interval, timeout = self._settings()
        return serial.Serial(
            port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=timeout,
        )

    def _raw_log(self, request: bytes, response: bytes) -> None:
        if not self.raw_var.get():
            return
        self.events.put(("log", f"TX {request.hex(' ').upper()}"))
        self.events.put(("log", f"RX {response.hex(' ').upper() if response else '(timeout)'}"))

    def start_reading(self) -> None:
        if self.worker and self.worker.is_alive():
            return
        try:
            port, baud, addr, channel, interval, _timeout = self._settings()
        except Exception as exc:
            messagebox.showerror("Bad settings", str(exc))
            return

        self.stop_event.clear()
        self.start_btn.configure(state=tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL)
        self.status_var.set(f"Reading {port} {baud} addr={addr} ch={channel}")

        self.worker = threading.Thread(
            target=self._read_loop,
            args=(addr, channel, interval),
            daemon=True,
        )
        self.worker.start()

    def stop_reading(self) -> None:
        self.stop_event.set()
        self.status_var.set("Stopping...")

    def read_once(self) -> None:
        self._run_once_worker("read")

    def scan_addr(self) -> None:
        self._run_once_worker("scan")

    def write_command(self, value: int) -> None:
        self._run_once_worker("write", value)

    def _run_once_worker(self, action: str, value: int = 0) -> None:
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("Busy", "Stop continuous reading first.")
            return
        try:
            _ = self._settings()
        except Exception as exc:
            messagebox.showerror("Bad settings", str(exc))
            return
        self.worker = threading.Thread(target=self._single_action, args=(action, value), daemon=True)
        self.worker.start()

    def _read_loop(self, addr: int, channel: int, interval: float) -> None:
        try:
            with self._open_serial() as ser:
                while not self.stop_event.is_set():
                    value = read_channel_weight(ser, addr, channel, log=self._raw_log)
                    self.events.put(("value", value))
                    self.events.put(("log", f"{time.strftime('%H:%M:%S')} ch{channel}={value}"))
                    time.sleep(interval)
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            self.events.put(("stopped", None))

    def _single_action(self, action: str, command_value: int) -> None:
        try:
            _port, _baud, addr, channel, _interval, _timeout = self._settings()
            command_register = 0x0028 + (channel - 1) * 10
            with self._open_serial() as ser:
                if action == "read":
                    value = read_channel_weight(ser, addr, channel, log=self._raw_log)
                    self.events.put(("value", value))
                    self.events.put(("log", f"{time.strftime('%H:%M:%S')} ch{channel}={value}"))
                elif action == "scan":
                    found = scan_addresses(ser, 1, 15, verbose=False)
                    self.events.put(("log", f"scan result: {found}"))
                    if found:
                        self.events.put(("addr", found[0]))
                elif action == "write":
                    write_single_register(ser, addr, command_register, command_value, log=self._raw_log)
                    name = "zero" if command_value == 1 else "tare"
                    self.events.put(("log", f"OK {name} ch{channel} reg=0x{command_register:04X}"))
                else:
                    raise ValueError(action)
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            self.events.put(("single_done", None))

    def _poll_events(self) -> None:
        while True:
            try:
                event, payload = self.events.get_nowait()
            except queue.Empty:
                break

            if event == "value":
                self.value_var.set(str(payload))
                self.status_var.set("OK")
            elif event == "log":
                self._log(str(payload))
            elif event == "addr":
                self.addr_var.set(str(payload))
                self._log(f"using addr={payload}")
            elif event == "error":
                self.status_var.set(f"ERR {payload}")
                self._log(f"ERR {payload}")
            elif event == "stopped":
                self.start_btn.configure(state=tk.NORMAL)
                self.stop_btn.configure(state=tk.DISABLED)
                if self.status_var.get() == "Stopping...":
                    self.status_var.set("Stopped")
            elif event == "single_done":
                pass

        self.after(60, self._poll_events)

    def _log(self, text: str) -> None:
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)


def main() -> None:
    app = PressureGui()
    app.mainloop()


if __name__ == "__main__":
    main()
