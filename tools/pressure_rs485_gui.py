#!/usr/bin/env python3
"""Tkinter GUI for the RS485 Modbus pressure/weight transmitter."""

from __future__ import annotations

import json
import csv
import queue
import re
import statistics
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
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

CALIBRATION_FILE = Path(__file__).with_name("pressure_calibration.json")
DEFAULT_REFERENCE_WEIGHTS = (231.8, 346.5, 504.9)
IDENT_SAMPLE_RE = re.compile(
    r"IDENT sample seq=(?P<seq>\d+) motor=(?P<motor>\d+) pct=(?P<pct>\d+) "
    r"pulse=(?P<pulse>\d+) dwell_ms=(?P<dwell>\d+) ms=(?P<ms>\d+)"
)


class PressureGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("RS485 Pressure Sensor Test")
        self.geometry("980x640")
        self.minsize(860, 540)

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.ident_stop_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.ident_worker: threading.Thread | None = None
        self.calibration_points: list[tuple[float, float]] = []
        self.last_raw_value: int | None = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.dip_var = tk.StringVar(value=DEFAULT_DIP)
        self.addr_var = tk.StringVar(value="")
        self.channel_var = tk.StringVar(value="1")
        self.interval_var = tk.StringVar(value="0.2")
        self.timeout_var = tk.StringVar(value=str(DEFAULT_TIMEOUT))
        self.raw_var = tk.BooleanVar(value=True)
        self.value_var = tk.StringVar(value="raw --")
        self.cal_value_var = tk.StringVar(value="cal -- g")
        self.status_var = tk.StringVar(value="Idle")
        self.ref_weight_var = tk.StringVar(value=str(DEFAULT_REFERENCE_WEIGHTS[0]))
        self.fc_port_var = tk.StringVar()
        self.fc_baud_var = tk.StringVar(value="115200")
        self.ident_motor_var = tk.StringVar(value="1")
        self.ident_min_var = tk.StringVar(value="0")
        self.ident_max_var = tk.StringVar(value="100")
        self.ident_step_var = tk.StringVar(value="5")
        self.ident_dwell_var = tk.StringVar(value="2000")
        self.ident_samples_var = tk.StringVar(value="20")
        self.ident_file_var = tk.StringVar(value="")

        self._build_ui()
        self.load_calibration()
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
        ttk.Label(value_box, textvariable=self.value_var, font=("Consolas", 28, "bold")).pack(padx=18, pady=(14, 4))
        ttk.Label(value_box, textvariable=self.cal_value_var, font=("Consolas", 26, "bold")).pack(padx=18, pady=(4, 14))

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

        cal = ttk.LabelFrame(root, text="Software Calibration")
        cal.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(cal, text="Reference g").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        self.ref_combo = ttk.Combobox(
            cal,
            textvariable=self.ref_weight_var,
            values=[str(v) for v in DEFAULT_REFERENCE_WEIGHTS],
            width=12,
        )
        self.ref_combo.grid(row=0, column=1, sticky="w", padx=6, pady=6)
        ttk.Button(cal, text="Capture Current Raw", command=self.capture_calibration_point).grid(row=0, column=2, padx=6, pady=6)
        ttk.Button(cal, text="Write Module Cal", command=self.write_module_calibration).grid(row=0, column=3, padx=6, pady=6)
        ttk.Button(cal, text="Clear Software Cal", command=self.clear_calibration).grid(row=0, column=4, padx=6, pady=6)
        ttk.Label(cal, text="Points").grid(row=1, column=0, sticky="e", padx=6, pady=6)
        self.cal_points_var = tk.StringVar(value="none")
        ttk.Label(cal, textvariable=self.cal_points_var).grid(row=1, column=1, columnspan=5, sticky="w", padx=6, pady=6)

        ident = ttk.LabelFrame(root, text="Thrust Identification")
        ident.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(ident, text="FC UART8").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        self.fc_port_combo = ttk.Combobox(ident, textvariable=self.fc_port_var, width=14)
        self.fc_port_combo.grid(row=0, column=1, sticky="w", padx=6, pady=6)
        ttk.Label(ident, text="Baud").grid(row=0, column=2, sticky="w", padx=6, pady=6)
        ttk.Entry(ident, textvariable=self.fc_baud_var, width=10).grid(row=0, column=3, sticky="w", padx=6, pady=6)
        ttk.Label(ident, text="Motor").grid(row=0, column=4, sticky="w", padx=6, pady=6)
        ttk.Combobox(ident, textvariable=self.ident_motor_var, values=("0", "1", "2"), width=5, state="readonly").grid(row=0, column=5, sticky="w", padx=6, pady=6)

        ttk.Label(ident, text="Min").grid(row=1, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(ident, textvariable=self.ident_min_var, width=7).grid(row=1, column=1, sticky="w", padx=6, pady=6)
        ttk.Label(ident, text="Max").grid(row=1, column=2, sticky="w", padx=6, pady=6)
        ttk.Entry(ident, textvariable=self.ident_max_var, width=7).grid(row=1, column=3, sticky="w", padx=6, pady=6)
        ttk.Label(ident, text="Step").grid(row=1, column=4, sticky="w", padx=6, pady=6)
        ttk.Entry(ident, textvariable=self.ident_step_var, width=7).grid(row=1, column=5, sticky="w", padx=6, pady=6)
        ttk.Label(ident, text="Dwell ms").grid(row=1, column=6, sticky="w", padx=6, pady=6)
        ttk.Entry(ident, textvariable=self.ident_dwell_var, width=9).grid(row=1, column=7, sticky="w", padx=6, pady=6)
        ttk.Label(ident, text="Samples").grid(row=1, column=8, sticky="w", padx=6, pady=6)
        ttk.Entry(ident, textvariable=self.ident_samples_var, width=7).grid(row=1, column=9, sticky="w", padx=6, pady=6)

        ttk.Label(ident, text="CSV").grid(row=2, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(ident, textvariable=self.ident_file_var, width=58).grid(row=2, column=1, columnspan=6, sticky="we", padx=6, pady=6)
        ttk.Button(ident, text="New File", command=self.new_ident_file).grid(row=2, column=7, padx=6, pady=6)
        self.ident_start_btn = ttk.Button(ident, text="Start Identification", command=self.start_identification)
        self.ident_start_btn.grid(row=2, column=8, padx=6, pady=6)
        self.ident_stop_btn = ttk.Button(ident, text="Stop", command=self.stop_identification, state=tk.DISABLED)
        self.ident_stop_btn.grid(row=2, column=9, padx=6, pady=6)

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
        self.fc_port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        if ports and not self.fc_port_var.get():
            self.fc_port_var.set(ports[1] if len(ports) > 1 else ports[0])
        self._log(f"Ports: {', '.join(ports) if ports else '(none)'}")

    def load_calibration(self) -> None:
        if not CALIBRATION_FILE.exists():
            self._update_calibration_label()
            return
        try:
            data = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
            points = data.get("points", [])
            self.calibration_points = [
                (float(item["raw"]), float(item["grams"]))
                for item in points
            ]
            self.calibration_points.sort(key=lambda item: item[0])
            self._log(f"Loaded calibration: {CALIBRATION_FILE}")
        except Exception as exc:
            self._log(f"ERR load calibration: {exc}")
        self._update_calibration_label()

    def save_calibration(self) -> None:
        data = {
            "points": [
                {"raw": raw, "grams": grams}
                for raw, grams in sorted(self.calibration_points, key=lambda item: item[0])
            ]
        }
        CALIBRATION_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")
        self._update_calibration_label()

    def calibrated_grams(self, raw_value: float) -> float | None:
        points = sorted(self.calibration_points, key=lambda item: item[0])
        if not points:
            return None
        if len(points) == 1:
            raw, grams = points[0]
            return grams if raw_value == raw else raw_value * (grams / raw) if raw != 0 else None

        if raw_value <= points[0][0]:
            p0, p1 = points[0], points[1]
        elif raw_value >= points[-1][0]:
            p0, p1 = points[-2], points[-1]
        else:
            p0, p1 = points[0], points[1]
            for left, right in zip(points, points[1:]):
                if left[0] <= raw_value <= right[0]:
                    p0, p1 = left, right
                    break

        raw0, grams0 = p0
        raw1, grams1 = p1
        if raw1 == raw0:
            return grams0
        ratio = (raw_value - raw0) / (raw1 - raw0)
        return grams0 + ratio * (grams1 - grams0)

    def _update_value_display(self, raw_value: int) -> None:
        self.last_raw_value = raw_value
        self.value_var.set(f"raw {raw_value}")
        calibrated = self.calibrated_grams(float(raw_value))
        if calibrated is None:
            self.cal_value_var.set("cal -- g")
        else:
            self.cal_value_var.set(f"cal {calibrated:.1f} g")

    def _update_calibration_label(self) -> None:
        if not self.calibration_points:
            self.cal_points_var.set("none")
            return
        items = [
            f"{raw:g}->{grams:g}g"
            for raw, grams in sorted(self.calibration_points, key=lambda item: item[0])
        ]
        self.cal_points_var.set("; ".join(items))

    def capture_calibration_point(self) -> None:
        if self.last_raw_value is None:
            messagebox.showinfo("No raw value", "Read the sensor once before capturing a calibration point.")
            return
        try:
            grams = float(self.ref_weight_var.get())
        except ValueError:
            messagebox.showerror("Bad reference", "Reference weight must be a number, e.g. 346.5")
            return

        raw = float(self.last_raw_value)
        self.calibration_points = [
            point for point in self.calibration_points
            if abs(point[0] - raw) > 1e-6 and abs(point[1] - grams) > 1e-6
        ]
        self.calibration_points.append((raw, grams))
        self.calibration_points.sort(key=lambda item: item[0])
        self.save_calibration()
        self._update_value_display(self.last_raw_value)
        self._log(f"CAL raw={raw:g} -> {grams:g}g")

    def clear_calibration(self) -> None:
        self.calibration_points = []
        if CALIBRATION_FILE.exists():
            CALIBRATION_FILE.unlink()
        self._update_calibration_label()
        if self.last_raw_value is not None:
            self._update_value_display(self.last_raw_value)
        self._log("Calibration cleared")

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

    def _open_fc_serial(self) -> serial.Serial:
        port = self.fc_port_var.get().strip()
        if not port:
            raise ValueError("flight controller serial port is empty")
        baud = int(self.fc_baud_var.get(), 0)
        return serial.Serial(
            port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=0.5,
        )

    def _ident_settings(self) -> tuple[int, int, int, int, int, int, Path]:
        motor = int(self.ident_motor_var.get(), 0)
        min_percent = int(self.ident_min_var.get(), 0)
        max_percent = int(self.ident_max_var.get(), 0)
        step_percent = int(self.ident_step_var.get(), 0)
        dwell_ms = int(self.ident_dwell_var.get(), 0)
        samples = int(self.ident_samples_var.get(), 0)

        if motor not in (0, 1, 2):
            raise ValueError("motor must be 0, 1 or 2")
        if not 0 <= min_percent <= 100:
            raise ValueError("min must be 0..100")
        if not 0 <= max_percent <= 100:
            raise ValueError("max must be 0..100")
        if min_percent > max_percent:
            raise ValueError("min must be <= max")
        if step_percent <= 0:
            raise ValueError("step must be > 0")
        if dwell_ms <= 0:
            raise ValueError("dwell_ms must be > 0")
        if samples <= 0:
            raise ValueError("samples must be > 0")

        csv_text = self.ident_file_var.get().strip()
        csv_path = Path(csv_text) if csv_text else self.default_ident_file()
        return motor, min_percent, max_percent, step_percent, dwell_ms, samples, csv_path

    def default_ident_file(self) -> Path:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return Path(__file__).with_name(f"thrust_ident_{stamp}.csv")

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

    def write_module_calibration(self) -> None:
        try:
            grams = float(self.ref_weight_var.get())
        except ValueError:
            messagebox.showerror("Bad reference", "Reference weight must be a number, e.g. 346.5")
            return
        if grams <= 0.0 or grams > 65535.0:
            messagebox.showerror("Bad reference", "Module calibration weight must be in 1..65535.")
            return
        weight_value = int(round(grams))
        if abs(weight_value - grams) > 0.001:
            ok = messagebox.askyesno(
                "Round weight",
                f"The module stores integer weights only.\nWrite {weight_value} for {grams:g} g?",
            )
            if not ok:
                return
        self._run_once_worker("module_cal", weight_value)

    def new_ident_file(self) -> None:
        path = self.default_ident_file()
        self.ident_file_var.set(str(path))
        self._log(f"IDENT csv={path}")

    def start_identification(self) -> None:
        if self.ident_worker and self.ident_worker.is_alive():
            return
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("Busy", "Stop continuous pressure reading first.")
            return
        try:
            _ = self._settings()
            motor, min_percent, max_percent, step_percent, dwell_ms, samples, csv_path = self._ident_settings()
        except Exception as exc:
            messagebox.showerror("Bad identification settings", str(exc))
            return

        self.ident_file_var.set(str(csv_path))
        self.ident_stop_event.clear()
        self.ident_start_btn.configure(state=tk.DISABLED)
        self.ident_stop_btn.configure(state=tk.NORMAL)
        self.status_var.set(f"IDENT motor={motor} {min_percent}..{max_percent}%")
        self.ident_worker = threading.Thread(
            target=self._ident_loop,
            args=(motor, min_percent, max_percent, step_percent, dwell_ms, samples, csv_path),
            daemon=True,
        )
        self.ident_worker.start()

    def stop_identification(self) -> None:
        self.ident_stop_event.set()
        self.status_var.set("Stopping identification...")

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

    def _ident_loop(
        self,
        motor: int,
        min_percent: int,
        max_percent: int,
        step_percent: int,
        dwell_ms: int,
        samples: int,
        csv_path: Path,
    ) -> None:
        try:
            _port, _baud, addr, channel, _interval, _timeout = self._settings()
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            with self._open_serial() as pressure_ser, self._open_fc_serial() as fc_ser, csv_path.open(
                "w", newline="", encoding="utf-8"
            ) as csv_file:
                writer = csv.DictWriter(
                    csv_file,
                    fieldnames=[
                        "kind",
                        "host_time",
                        "seq",
                        "motor",
                        "pct",
                        "pulse_us",
                        "fc_ms",
                        "dwell_ms",
                        "sample_index",
                        "raw",
                        "grams",
                        "count",
                        "mean_g",
                        "min_g",
                        "max_g",
                        "std_g",
                        "error",
                    ],
                )
                writer.writeheader()

                fc_ser.reset_input_buffer()
                command = f"IDENT START {motor} {min_percent} {max_percent} {step_percent} {dwell_ms}\r\n"
                fc_ser.write(command.encode("ascii"))
                fc_ser.flush()
                self.events.put(("log", f"FC TX {command.strip()}"))
                self.events.put(("log", f"IDENT csv={csv_path}"))

                while not self.ident_stop_event.is_set():
                    raw_line = fc_ser.readline()
                    if not raw_line:
                        continue
                    line = raw_line.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue

                    self.events.put(("log", f"FC RX {line}"))
                    match = IDENT_SAMPLE_RE.search(line)
                    if match is not None:
                        record = {key: int(value) for key, value in match.groupdict().items()}
                        self._capture_ident_step(writer, pressure_ser, addr, channel, samples, record)
                        csv_file.flush()
                    elif line.startswith("IDENT stop") or line.startswith("IDENT done"):
                        break
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            try:
                with self._open_fc_serial() as fc_ser:
                    fc_ser.write(b"IDENT STOP\r\n")
                    fc_ser.flush()
            except Exception:
                pass
            self.events.put(("ident_stopped", None))

    def _capture_ident_step(
        self,
        writer: csv.DictWriter,
        pressure_ser: serial.Serial,
        addr: int,
        channel: int,
        samples: int,
        record: dict[str, int],
    ) -> None:
        grams_values: list[float] = []

        for sample_index in range(samples):
            if self.ident_stop_event.is_set():
                break
            error_text = ""
            raw_value: int | str = ""
            grams_value: float | str = ""
            try:
                raw_reading = read_channel_weight(pressure_ser, addr, channel, log=self._raw_log)
                grams = self.calibrated_grams(float(raw_reading))
                grams_value = float(raw_reading) if grams is None else float(grams)
                raw_value = raw_reading
                grams_values.append(float(grams_value))
                self.events.put(("value", raw_reading))
            except Exception as exc:
                error_text = str(exc)
                self.events.put(
                    (
                        "log",
                        f"WARN pressure seq={record['seq']} sample={sample_index} {error_text}",
                    )
                )

            writer.writerow(
                {
                    "kind": "sample",
                    "host_time": time.time(),
                    "seq": record["seq"],
                    "motor": record["motor"],
                    "pct": record["pct"],
                    "pulse_us": record["pulse"],
                    "fc_ms": record["ms"],
                    "dwell_ms": record["dwell"],
                    "sample_index": sample_index,
                    "raw": raw_value,
                    "grams": grams_value,
                    "count": "",
                    "mean_g": "",
                    "min_g": "",
                    "max_g": "",
                    "std_g": "",
                    "error": error_text,
                }
            )

        if grams_values:
            mean_g = statistics.fmean(grams_values)
            std_g = statistics.pstdev(grams_values) if len(grams_values) > 1 else 0.0
            writer.writerow(
                {
                    "kind": "summary",
                    "host_time": time.time(),
                    "seq": record["seq"],
                    "motor": record["motor"],
                    "pct": record["pct"],
                    "pulse_us": record["pulse"],
                    "fc_ms": record["ms"],
                    "dwell_ms": record["dwell"],
                    "sample_index": "",
                    "raw": "",
                    "grams": "",
                    "count": len(grams_values),
                    "mean_g": f"{mean_g:.3f}",
                    "min_g": f"{min(grams_values):.3f}",
                    "max_g": f"{max(grams_values):.3f}",
                    "std_g": f"{std_g:.3f}",
                    "error": "",
                }
            )
            self.events.put(
                (
                    "log",
                    f"IDENT seq={record['seq']} motor={record['motor']} pct={record['pct']} "
                    f"n={len(grams_values)} mean={mean_g:.2f}g std={std_g:.2f}g",
                )
            )
        else:
            writer.writerow(
                {
                    "kind": "summary",
                    "host_time": time.time(),
                    "seq": record["seq"],
                    "motor": record["motor"],
                    "pct": record["pct"],
                    "pulse_us": record["pulse"],
                    "fc_ms": record["ms"],
                    "dwell_ms": record["dwell"],
                    "sample_index": "",
                    "raw": "",
                    "grams": "",
                    "count": 0,
                    "mean_g": "",
                    "min_g": "",
                    "max_g": "",
                    "std_g": "",
                    "error": "no_valid_pressure_samples",
                }
            )
            self.events.put(("log", f"WARN IDENT seq={record['seq']} no valid pressure samples"))

    def _single_action(self, action: str, command_value: int) -> None:
        try:
            _port, _baud, addr, channel, _interval, _timeout = self._settings()
            command_register = 0x0028 + (channel - 1) * 10
            cal_weight_register = 0x0029 + (channel - 1) * 10
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
                elif action == "module_cal":
                    write_single_register(ser, addr, cal_weight_register, command_value, log=self._raw_log)
                    self.events.put(("log", f"OK module_cal ch{channel} reg=0x{cal_weight_register:04X} weight={command_value}g"))
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
                self._update_value_display(int(payload))
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
            elif event == "ident_stopped":
                self.ident_start_btn.configure(state=tk.NORMAL)
                self.ident_stop_btn.configure(state=tk.DISABLED)
                if self.status_var.get() == "Stopping identification...":
                    self.status_var.set("Identification stopped")
                else:
                    self.status_var.set("Identification done")
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
