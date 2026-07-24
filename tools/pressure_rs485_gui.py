#!/usr/bin/env python3
"""Tkinter GUI for the RS485 Modbus pressure/weight transmitter."""

from __future__ import annotations

import json
import contextlib
import csv
import math
import queue
import re
import statistics
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Callable

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
DEFAULT_MOTOR_KV = 1300.0
DEFAULT_BATTERY_VOLTAGE = 12.6
DEFAULT_PROP = "9050"
DEFAULT_LOAD_FACTOR = 0.80
HISTORY_TAIL_SAMPLES = 5
ESP_CONTROLLER_NAMES = {"ESP12E", "ESP8266"}
AUTO_IDENT_TEXT = "AUTO"
AUTO_TARE_COMMAND = 2
AUTO_PRE_TARE_SETTLE_S = 2.0
AUTO_TARE_SETTLE_S = 1.2
AUTO_TARE_VERIFY_SAMPLES = 8
AUTO_TARE_VERIFY_INTERVAL_S = 0.08
AUTO_TARE_WARN_ABS_G = 200.0
ESP_STATUS_RE = re.compile(
    r"STATUS mode=(?P<mode>\S+) armed=(?P<armed>\d+) "
    r"m1_pulse_us=(?P<m1_pulse>\d+) m2_pulse_us=(?P<m2_pulse>\d+) "
    r"arm_settled=(?P<arm_settled>\d+) ident_motor=(?P<ident_motor>\d+) "
    r"ident_seq=(?P<ident_seq>\d+) ident_pct=(?P<ident_pct>\d+)"
)
IDENT_SAMPLE_RE = re.compile(
    r"IDENT sample seq=(?P<seq>\d+) motor=(?P<motor>\d+) pct=(?P<pct>\d+) "
    r"pulse=(?P<pulse>\d+) dwell_ms=(?P<dwell>\d+) ms=(?P<ms>\d+)"
)


@dataclass(frozen=True)
class IdentPoint:
    pct: int
    pulse_us: int
    thrust_g: float


@dataclass(frozen=True)
class IdentRun:
    path: Path
    modified_s: float
    motor: int
    points: dict[int, IdentPoint]


@dataclass(frozen=True)
class LossRow:
    pct: int
    pulse_us: int
    single1_g: float
    single2_g: float
    single_sum_g: float
    dual_g: float
    loss_coeff: float
    loss_pct: float
    rpm_no_load_est: float
    rpm_loaded_est: float
    tip_speed_m_s_est: float
    pitch_speed_m_s_est: float


def motor_name(motor: int) -> str:
    if motor == 0:
        return "Dual"
    return f"M{motor}"


def is_esp_controller(controller: str) -> bool:
    return controller in ESP_CONTROLLER_NAMES


def percent_to_pulse(percent: int) -> int:
    return 1100 + round(max(0, min(100, percent)) * (1940 - 1100) / 100.0)


def parse_prop(prop: str) -> tuple[float, float]:
    text = prop.strip().lower().replace("x", "")
    if len(text) != 4 or not text.isdigit():
        raise ValueError("prop must look like 9050, 9047, or 1045")
    diameter_code = int(text[:2])
    pitch_code = int(text[2:])
    diameter_in = diameter_code / 10.0 if diameter_code >= 50 else float(diameter_code)
    pitch_in = pitch_code / 10.0
    return diameter_in, pitch_in


def row_motor(row: dict[str, str]) -> int | None:
    value = row.get("motor", "").strip()
    if not value:
        return None
    try:
        return int(value)
    except ValueError:
        return None


def load_ident_points(path: Path, motor: int | None, tail_samples: int = HISTORY_TAIL_SAMPLES) -> dict[int, IdentPoint]:
    summaries: dict[int, IdentPoint] = {}
    sample_blocks: dict[tuple[int, int, int], list[float]] = {}

    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if motor is not None and row_motor(row) != motor:
                continue

            kind = row.get("kind", "").strip()
            if kind == "summary" and row.get("mean_g"):
                pct = int(float(row["pct"]))
                summaries[pct] = IdentPoint(
                    pct=pct,
                    pulse_us=int(float(row["pulse_us"])),
                    thrust_g=float(row["mean_g"]),
                )
            elif kind == "sample" and not row.get("error") and row.get("grams"):
                key = (
                    int(row["seq"]),
                    int(float(row["pct"])),
                    int(float(row["pulse_us"])),
                )
                sample_blocks.setdefault(key, []).append(float(row["grams"]))

    if summaries:
        return dict(sorted(summaries.items()))

    points: dict[int, IdentPoint] = {}
    for (_seq, pct, pulse_us), values in sample_blocks.items():
        if not values:
            continue
        tail = values[-tail_samples:] if tail_samples > 0 else values
        points[pct] = IdentPoint(pct=pct, pulse_us=pulse_us, thrust_g=statistics.fmean(tail))
    return dict(sorted(points.items()))


def ident_baseline(points: dict[int, IdentPoint], preferred_pct: int = 0) -> IdentPoint | None:
    if not points:
        return None
    if preferred_pct in points:
        return points[preferred_pct]
    return points[min(points)]


def zero_baseline_points(
    points: dict[int, IdentPoint],
    preferred_pct: int = 0,
) -> dict[int, IdentPoint]:
    baseline = ident_baseline(points, preferred_pct)
    if baseline is None:
        return {}
    offset_g = baseline.thrust_g
    return {
        pct: IdentPoint(
            pct=point.pct,
            pulse_us=point.pulse_us,
            thrust_g=point.thrust_g - offset_g,
        )
        for pct, point in sorted(points.items())
    }


def load_ident_runs(paths: list[Path]) -> list[IdentRun]:
    runs: list[IdentRun] = []
    for path in paths:
        if not path.exists() or not path.is_file():
            continue
        for motor in (0, 1, 2):
            try:
                points = load_ident_points(path, motor)
            except Exception:
                continue
            if points:
                runs.append(IdentRun(path=path, modified_s=path.stat().st_mtime, motor=motor, points=points))
    return runs


def compute_loss_rows(
    single1: dict[int, IdentPoint],
    single2: dict[int, IdentPoint],
    dual: dict[int, IdentPoint],
    kv: float,
    voltage: float,
    load_factor: float,
    prop: str,
) -> list[LossRow]:
    diameter_in, pitch_in = parse_prop(prop)
    diameter_m = diameter_in * 0.0254
    pitch_m = pitch_in * 0.0254
    rows: list[LossRow] = []

    for pct in sorted(set(single1) & set(single2) & set(dual)):
        p1 = single1[pct]
        p2 = single2[pct]
        pd = dual[pct]
        single_sum = p1.thrust_g + p2.thrust_g
        loss_coeff = pd.thrust_g / single_sum if abs(single_sum) > 1e-9 else 0.0
        rpm_no_load = kv * voltage * (pct / 100.0)
        rpm_loaded = rpm_no_load * load_factor
        rows.append(
            LossRow(
                pct=pct,
                pulse_us=pd.pulse_us,
                single1_g=p1.thrust_g,
                single2_g=p2.thrust_g,
                single_sum_g=single_sum,
                dual_g=pd.thrust_g,
                loss_coeff=loss_coeff,
                loss_pct=(1.0 - loss_coeff) * 100.0,
                rpm_no_load_est=rpm_no_load,
                rpm_loaded_est=rpm_loaded,
                tip_speed_m_s_est=math.pi * diameter_m * rpm_loaded / 60.0,
                pitch_speed_m_s_est=pitch_m * rpm_loaded / 60.0,
            )
        )
    return rows


def write_loss_report(path: Path, rows: list[LossRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "pct",
                "pulse_us",
                "single1_g",
                "single2_g",
                "single_sum_g",
                "dual_g",
                "loss_coeff",
                "loss_pct",
                "rpm_no_load_est",
                "rpm_loaded_est",
                "tip_speed_m_s_est",
                "pitch_speed_m_s_est",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "pct": row.pct,
                    "pulse_us": row.pulse_us,
                    "single1_g": f"{row.single1_g:.3f}",
                    "single2_g": f"{row.single2_g:.3f}",
                    "single_sum_g": f"{row.single_sum_g:.3f}",
                    "dual_g": f"{row.dual_g:.3f}",
                    "loss_coeff": f"{row.loss_coeff:.6f}",
                    "loss_pct": f"{row.loss_pct:.3f}",
                    "rpm_no_load_est": f"{row.rpm_no_load_est:.1f}",
                    "rpm_loaded_est": f"{row.rpm_loaded_est:.1f}",
                    "tip_speed_m_s_est": f"{row.tip_speed_m_s_est:.3f}",
                    "pitch_speed_m_s_est": f"{row.pitch_speed_m_s_est:.3f}",
                }
            )


class PressureGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("RS485 Pressure Sensor Test")
        self.geometry("1120x780")
        self.minsize(980, 660)

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.ident_stop_event = threading.Event()
        self.pwm_stop_event = threading.Event()
        self.manual_pwm_lock = threading.Lock()
        self.fc_serial_lock = threading.RLock()
        self.worker: threading.Thread | None = None
        self.ident_worker: threading.Thread | None = None
        self.pwm_worker: threading.Thread | None = None
        self.fc_worker: threading.Thread | None = None
        self.fc_serial: serial.Serial | None = None
        self.calibration_points: list[tuple[float, float]] = []
        self.last_raw_value: int | None = None
        self.latest_loss_rows: list[LossRow] = []
        self.manual_pwm_target = {"m1_pct": 0, "m2_pct": 0, "hold_ms": 3000}
        self.raw_log_enabled = True

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
        self.pwm_controller_var = tk.StringVar(value="H743")
        self.ident_motor_var = tk.StringVar(value=AUTO_IDENT_TEXT)
        self.ident_min_var = tk.StringVar(value="0")
        self.ident_max_var = tk.StringVar(value="100")
        self.ident_step_var = tk.StringVar(value="5")
        self.ident_dwell_var = tk.StringVar(value="2000")
        self.ident_samples_var = tk.StringVar(value="20")
        self.ident_file_var = tk.StringVar(value="")
        self.manual_m1_pct_var = tk.IntVar(value=0)
        self.manual_m2_pct_var = tk.IntVar(value=0)
        self.manual_hold_ms_var = tk.StringVar(value="3000")
        self.manual_target_var = tk.StringVar(value="Target M1 0% 1100us | M2 0% 1100us")
        self.esp_status_var = tk.StringVar(value="ESP STATUS: unknown")
        self.esp_serial_var = tk.StringVar(value="ESP SERIAL: closed")
        self.kv_var = tk.StringVar(value=f"{DEFAULT_MOTOR_KV:g}")
        self.voltage_var = tk.StringVar(value=f"{DEFAULT_BATTERY_VOLTAGE:g}")
        self.prop_var = tk.StringVar(value=DEFAULT_PROP)
        self.load_factor_var = tk.StringVar(value=f"{DEFAULT_LOAD_FACTOR:g}")
        self.history_var = tk.StringVar(value="History: not loaded")

        self._build_ui()
        self.on_pwm_slider_change()
        self.load_calibration()
        self.refresh_ports()
        self.refresh_history()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
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
        ttk.Label(ident, text="PWM Controller").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Combobox(
            ident,
            textvariable=self.pwm_controller_var,
            values=("H743", "ESP12E"),
            width=10,
            state="readonly",
        ).grid(row=0, column=1, sticky="w", padx=6, pady=6)
        ttk.Label(ident, text="Serial").grid(row=0, column=2, sticky="w", padx=6, pady=6)
        self.fc_port_combo = ttk.Combobox(ident, textvariable=self.fc_port_var, width=14)
        self.fc_port_combo.grid(row=0, column=3, sticky="w", padx=6, pady=6)
        ttk.Label(ident, text="Baud").grid(row=0, column=4, sticky="w", padx=6, pady=6)
        ttk.Entry(ident, textvariable=self.fc_baud_var, width=10).grid(row=0, column=5, sticky="w", padx=6, pady=6)
        ttk.Label(ident, text="Run").grid(row=0, column=6, sticky="w", padx=6, pady=6)
        ttk.Combobox(
            ident,
            textvariable=self.ident_motor_var,
            values=(AUTO_IDENT_TEXT, "1", "2", "0"),
            width=7,
            state="readonly",
        ).grid(row=0, column=7, sticky="w", padx=6, pady=6)

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

        manual = ttk.LabelFrame(root, text="ESP12E Manual PWM")
        manual.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(manual, text="M1 %").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Scale(
            manual,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            variable=self.manual_m1_pct_var,
            command=self.on_pwm_slider_change,
        ).grid(row=0, column=1, columnspan=4, sticky="we", padx=6, pady=6)
        ttk.Label(manual, text="M2 %").grid(row=1, column=0, sticky="w", padx=6, pady=6)
        ttk.Scale(
            manual,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            variable=self.manual_m2_pct_var,
            command=self.on_pwm_slider_change,
        ).grid(row=1, column=1, columnspan=4, sticky="we", padx=6, pady=6)
        ttk.Label(manual, text="Hold ms").grid(row=0, column=5, sticky="w", padx=6, pady=6)
        ttk.Entry(manual, textvariable=self.manual_hold_ms_var, width=9).grid(row=0, column=6, sticky="w", padx=6, pady=6)
        self.esp_open_btn = ttk.Button(manual, text="Open ESP", command=self.open_esp_serial)
        self.esp_open_btn.grid(row=0, column=7, padx=6, pady=6)
        self.esp_close_btn = ttk.Button(manual, text="Close ESP", command=self.close_esp_serial, state=tk.DISABLED)
        self.esp_close_btn.grid(row=0, column=8, padx=6, pady=6)
        self.pwm_status_btn = ttk.Button(manual, text="Status", command=self.query_pwm_status, state=tk.DISABLED)
        self.pwm_status_btn.grid(row=0, column=9, padx=6, pady=6)
        self.pwm_arm_btn = ttk.Button(manual, text="Arm", command=self.arm_pwm, state=tk.DISABLED)
        self.pwm_arm_btn.grid(row=0, column=10, padx=6, pady=6)
        self.pwm_stop_btn = ttk.Button(manual, text="Stop/Disarm", command=self.stop_pwm_manual, state=tk.DISABLED)
        self.pwm_stop_btn.grid(row=0, column=11, padx=6, pady=6)
        self.pwm_apply_btn = ttk.Button(manual, text="Apply Once", command=self.apply_pwm_once)
        self.pwm_apply_btn.grid(row=1, column=10, padx=6, pady=6)
        self.pwm_start_btn = ttk.Button(manual, text="Start Manual", command=self.start_pwm_manual)
        self.pwm_start_btn.grid(row=1, column=11, padx=6, pady=6)
        self.pwm_apply_btn.configure(state=tk.DISABLED)
        self.pwm_start_btn.configure(state=tk.DISABLED)
        ttk.Label(manual, textvariable=self.manual_target_var).grid(row=1, column=5, columnspan=3, sticky="w", padx=6, pady=6)
        ttk.Label(manual, textvariable=self.esp_status_var, font=("Consolas", 10)).grid(
            row=1, column=8, columnspan=2, sticky="w", padx=6, pady=6
        )
        ttk.Label(manual, textvariable=self.esp_serial_var, font=("Consolas", 10)).grid(
            row=2, column=5, columnspan=7, sticky="w", padx=6, pady=(0, 6)
        )
        manual.columnconfigure(1, weight=1)
        manual.columnconfigure(2, weight=1)
        manual.columnconfigure(3, weight=1)
        manual.columnconfigure(4, weight=1)

        history = ttk.LabelFrame(root, text="History And Loss")
        history.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(history, text="KV").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(history, textvariable=self.kv_var, width=8).grid(row=0, column=1, sticky="w", padx=6, pady=6)
        ttk.Label(history, text="Voltage").grid(row=0, column=2, sticky="w", padx=6, pady=6)
        ttk.Entry(history, textvariable=self.voltage_var, width=8).grid(row=0, column=3, sticky="w", padx=6, pady=6)
        ttk.Label(history, text="Prop").grid(row=0, column=4, sticky="w", padx=6, pady=6)
        ttk.Entry(history, textvariable=self.prop_var, width=8).grid(row=0, column=5, sticky="w", padx=6, pady=6)
        ttk.Label(history, text="Load").grid(row=0, column=6, sticky="w", padx=6, pady=6)
        ttk.Entry(history, textvariable=self.load_factor_var, width=8).grid(row=0, column=7, sticky="w", padx=6, pady=6)
        ttk.Button(history, text="Refresh History", command=self.refresh_history).grid(row=0, column=8, padx=6, pady=6)
        ttk.Button(history, text="Export Loss CSV", command=self.export_loss_csv).grid(row=0, column=9, padx=6, pady=6)
        ttk.Label(history, textvariable=self.history_var, justify=tk.LEFT, wraplength=1060).grid(
            row=1, column=0, columnspan=10, sticky="w", padx=6, pady=(0, 6)
        )

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
        return self._make_pressure_serial(port, baud, timeout)

    def _make_pressure_serial(self, port: str, baud: int, timeout: float) -> serial.Serial:
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
        return self._make_fc_serial(port, baud)

    def _make_fc_serial(self, port: str, baud: int) -> serial.Serial:
        return serial.Serial(
            port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=0.5,
        )

    def _esp_serial_is_open(self) -> bool:
        with self.fc_serial_lock:
            return self.fc_serial is not None and self.fc_serial.is_open

    def _require_open_esp_serial(self) -> serial.Serial:
        if self.fc_serial is None or not self.fc_serial.is_open:
            raise ValueError("Open ESP serial first")
        return self.fc_serial

    def _set_esp_controls_state(self, opened: bool) -> None:
        opened_state = tk.NORMAL if opened else tk.DISABLED
        closed_state = tk.DISABLED if opened else tk.NORMAL
        self.esp_open_btn.configure(state=closed_state)
        self.esp_close_btn.configure(state=opened_state)
        self.pwm_status_btn.configure(state=opened_state)
        self.pwm_arm_btn.configure(state=opened_state)
        self.pwm_stop_btn.configure(state=opened_state)
        self.pwm_apply_btn.configure(state=opened_state)
        self.pwm_start_btn.configure(state=opened_state)

    def open_esp_serial(self) -> None:
        if self.fc_worker and self.fc_worker.is_alive():
            return
        if self._pwm_busy():
            return
        try:
            self._require_esp_pwm()
            if self._esp_serial_is_open():
                messagebox.showinfo("ESP serial", "ESP serial is already open.")
                return
            port = self.fc_port_var.get().strip()
            if not port:
                raise ValueError("ESP serial port is empty")
            baud = int(self.fc_baud_var.get(), 0)
        except Exception as exc:
            messagebox.showerror("Bad ESP settings", str(exc))
            return

        self.esp_open_btn.configure(state=tk.DISABLED)
        self.esp_serial_var.set("ESP SERIAL: opening...")
        self.fc_worker = threading.Thread(target=self._open_esp_serial_worker, args=(port, baud), daemon=True)
        self.fc_worker.start()

    def _open_esp_serial_worker(self, port: str, baud: int) -> None:
        ser: serial.Serial | None = None
        try:
            ser = self._make_fc_serial(port, baud)
            try:
                ser.setDTR(False)
                ser.setRTS(False)
            except Exception:
                pass
            with self.fc_serial_lock:
                if self.fc_serial is not None and self.fc_serial.is_open:
                    self.fc_serial.close()
                self.fc_serial = ser
            self._sync_esp_after_open(ser)
            self.events.put(
                (
                    "esp_serial_state",
                    {
                        "open": True,
                        "text": f"ESP SERIAL: open {ser.port} @ {ser.baudrate}",
                    },
                )
            )
        except Exception as exc:
            if ser is not None:
                with contextlib.suppress(Exception):
                    ser.close()
            with self.fc_serial_lock:
                if self.fc_serial is ser:
                    self.fc_serial = None
            self.events.put(("error", str(exc)))
            self.events.put(("esp_serial_state", {"open": False, "text": "ESP SERIAL: closed"}))

    def close_esp_serial(self) -> None:
        if self.ident_worker and self.ident_worker.is_alive():
            self.stop_identification()
            messagebox.showinfo("Busy", "Identification is stopping. Close ESP after it stops.")
            return
        if self.pwm_worker and self.pwm_worker.is_alive():
            self.pwm_stop_event.set()
            messagebox.showinfo("Busy", "PWM is stopping. Close ESP after it stops.")
            return
        self._close_esp_serial("closed")

    def _close_esp_serial(self, reason: str) -> None:
        with self.fc_serial_lock:
            if self.fc_serial is not None:
                with contextlib.suppress(Exception):
                    if self.fc_serial.is_open:
                        self.fc_serial.write(b"DISARM\r\n")
                        self.fc_serial.flush()
                        time.sleep(0.05)
                with contextlib.suppress(Exception):
                    self.fc_serial.close()
                self.fc_serial = None
        self.events.put(("esp_serial_state", {"open": False, "text": f"ESP SERIAL: {reason}"}))

    def _require_esp_pwm(self) -> None:
        if not is_esp_controller(self.pwm_controller_var.get()):
            raise ValueError("manual PWM controls require PWM Controller = ESP12E")

    def _update_manual_pwm_target(self) -> tuple[int, int, int]:
        m1_pct = int(round(float(self.manual_m1_pct_var.get())))
        m2_pct = int(round(float(self.manual_m2_pct_var.get())))
        hold_ms = int(self.manual_hold_ms_var.get(), 0)
        if not 0 <= m1_pct <= 100:
            raise ValueError("M1 percent must be 0..100")
        if not 0 <= m2_pct <= 100:
            raise ValueError("M2 percent must be 0..100")
        if not 1 <= hold_ms <= 5000:
            raise ValueError("hold_ms must be 1..5000")
        with self.manual_pwm_lock:
            self.manual_pwm_target = {
                "m1_pct": m1_pct,
                "m2_pct": m2_pct,
                "hold_ms": hold_ms,
            }
        self.manual_target_var.set(
            f"Target M1 {m1_pct}% {percent_to_pulse(m1_pct)}us | "
            f"M2 {m2_pct}% {percent_to_pulse(m2_pct)}us"
        )
        return m1_pct, m2_pct, hold_ms

    def _current_manual_pwm_target(self) -> tuple[int, int, int]:
        with self.manual_pwm_lock:
            target = dict(self.manual_pwm_target)
        return int(target["m1_pct"]), int(target["m2_pct"]), int(target["hold_ms"])

    def on_pwm_slider_change(self, _value: str | None = None) -> None:
        try:
            self._update_manual_pwm_target()
        except Exception as exc:
            self.manual_target_var.set(f"Target error: {exc}")

    def _manual_pwm_settings(self) -> tuple[int, int, int]:
        self._require_esp_pwm()
        return self._update_manual_pwm_target()

    def _pwm_busy(self) -> bool:
        if self.fc_worker and self.fc_worker.is_alive():
            messagebox.showinfo("Busy", "ESP serial is opening.")
            return True
        if self.pwm_worker and self.pwm_worker.is_alive():
            messagebox.showinfo("Busy", "PWM command is already running.")
            return True
        if self.ident_worker and self.ident_worker.is_alive():
            messagebox.showinfo("Busy", "Identification is using the PWM serial port.")
            return True
        return False

    def _read_fc_lines(self, fc_ser: serial.Serial, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            raw_line = fc_ser.readline()
            if not raw_line:
                continue
            line = raw_line.decode("utf-8", errors="replace").strip()
            if line:
                self._handle_esp_line(line)

    def _handle_esp_line(self, line: str) -> None:
        self.events.put(("log", f"ESP RX {line}"))
        match = ESP_STATUS_RE.search(line)
        if match is not None:
            payload: dict[str, int | str] = {"mode": match.group("mode")}
            for key in (
                "armed",
                "m1_pulse",
                "m2_pulse",
                "arm_settled",
                "ident_motor",
                "ident_seq",
                "ident_pct",
            ):
                payload[key] = int(match.group(key))
            self.events.put(("esp_status", payload))

    def _write_fc_line(self, fc_ser: serial.Serial, command: str) -> None:
        fc_ser.write((command + "\r\n").encode("ascii"))
        fc_ser.flush()
        self.events.put(("log", f"ESP TX {command}"))

    def _request_esp_status(self, fc_ser: serial.Serial) -> None:
        self._write_fc_line(fc_ser, "STATUS?")

    def _sync_esp_after_open(self, fc_ser: serial.Serial) -> None:
        try:
            fc_ser.setDTR(False)
            fc_ser.setRTS(False)
        except Exception:
            pass

        self.events.put(("log", "ESP sync after serial open"))
        ready = False
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline:
            raw_line = fc_ser.readline()
            if not raw_line:
                continue
            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if "PWM calibrator ready" in line:
                ready = True
                self._handle_esp_line(line)
                break

        if not ready:
            self.events.put(("log", "WARN ESP ready banner not seen; querying STATUS anyway"))
        fc_ser.reset_input_buffer()
        self._request_esp_status(fc_ser)
        self._read_fc_lines(fc_ser, 0.5)

    def _send_manual_pair(self, fc_ser: serial.Serial, m1_pct: int, m2_pct: int, hold_ms: int) -> None:
        if m1_pct == m2_pct:
            self._write_fc_line(fc_ser, f"PCT 0 {m1_pct} {hold_ms}")
            return
        self._write_fc_line(fc_ser, f"PCT 1 {m1_pct} {hold_ms}")
        self._write_fc_line(fc_ser, f"PCT 2 {m2_pct} {hold_ms}")

    def _wait_pwm_stop_or_timeout(self, fc_ser: serial.Serial, duration_s: float) -> bool:
        deadline = time.monotonic() + duration_s
        while not self.pwm_stop_event.is_set() and time.monotonic() < deadline:
            self._read_fc_lines(fc_ser, 0.05)
        return self.pwm_stop_event.is_set()

    def _start_pwm_thread(self, target: Callable[..., None], *args: object) -> None:
        if self._pwm_busy():
            return
        try:
            self._require_esp_pwm()
            if not self._esp_serial_is_open():
                raise ValueError("Open ESP serial first")
        except Exception as exc:
            messagebox.showerror("Bad PWM settings", str(exc))
            return
        self.pwm_stop_event.clear()
        self.pwm_apply_btn.configure(state=tk.DISABLED)
        self.pwm_start_btn.configure(state=tk.DISABLED)
        self.pwm_worker = threading.Thread(target=target, args=args, daemon=True)
        self.pwm_worker.start()

    def arm_pwm(self) -> None:
        self._start_pwm_thread(self._pwm_single_command, "ARM", 0.5)

    def query_pwm_status(self) -> None:
        self._start_pwm_thread(self._pwm_single_command, "STATUS?", 0.7)

    def apply_pwm_once(self) -> None:
        try:
            m1_pct, m2_pct, hold_ms = self._manual_pwm_settings()
        except Exception as exc:
            messagebox.showerror("Bad PWM settings", str(exc))
            return
        self._start_pwm_thread(self._pwm_apply_once, m1_pct, m2_pct, hold_ms)

    def start_pwm_manual(self) -> None:
        try:
            m1_pct, m2_pct, hold_ms = self._manual_pwm_settings()
        except Exception as exc:
            messagebox.showerror("Bad PWM settings", str(exc))
            return
        self._start_pwm_thread(self._pwm_manual_loop, m1_pct, m2_pct, hold_ms)

    def stop_pwm_manual(self) -> None:
        if self.ident_worker and self.ident_worker.is_alive():
            self.stop_identification()
            return
        if self.pwm_worker and self.pwm_worker.is_alive():
            self.pwm_stop_event.set()
            self.status_var.set("Stopping PWM...")
            return
        self._start_pwm_thread(self._pwm_single_command, "DISARM", 0.5)

    def _pwm_single_command(self, command: str, read_s: float) -> None:
        try:
            with self.fc_serial_lock:
                fc_ser = self._require_open_esp_serial()
                self._write_fc_line(fc_ser, command)
                if command == "ARM":
                    self._read_fc_lines(fc_ser, 0.3)
                    for _ in range(7):
                        if self.pwm_stop_event.is_set():
                            break
                        self._request_esp_status(fc_ser)
                        self._read_fc_lines(fc_ser, 0.45)
                elif command in ("DISARM", "STOP", "IDENT STOP"):
                    self._read_fc_lines(fc_ser, 0.2)
                    self._request_esp_status(fc_ser)
                    self._read_fc_lines(fc_ser, 0.4)
                else:
                    self._read_fc_lines(fc_ser, read_s)
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            self.events.put(("pwm_stopped", None))

    def _pwm_apply_once(self, m1_pct: int, m2_pct: int, hold_ms: int) -> None:
        try:
            with self.fc_serial_lock:
                fc_ser = self._require_open_esp_serial()
                self._write_fc_line(fc_ser, "ARM")
                self.events.put(("log", "ESP arm settle 3.2 s"))
                if self._wait_pwm_stop_or_timeout(fc_ser, 3.2):
                    self._write_fc_line(fc_ser, "DISARM")
                    return
                self._send_manual_pair(fc_ser, m1_pct, m2_pct, hold_ms)
                hold_deadline = time.monotonic() + (hold_ms / 1000.0) + 0.4
                while not self.pwm_stop_event.is_set() and time.monotonic() < hold_deadline:
                    self._request_esp_status(fc_ser)
                    self._read_fc_lines(fc_ser, 0.35)
                self.events.put(
                    (
                        "log",
                        f"PWM once m1={m1_pct}% m2={m2_pct}% hold={hold_ms}ms; ESP firmware will timeout",
                    )
                )
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            self.events.put(("pwm_stopped", None))

    def _pwm_manual_loop(self, m1_pct: int, m2_pct: int, hold_ms: int) -> None:
        try:
            with self.fc_serial_lock:
                fc_ser = self._require_open_esp_serial()
                self._write_fc_line(fc_ser, "ARM")
                self.events.put(("log", "ESP arm settle 3.2 s"))
                if self._wait_pwm_stop_or_timeout(fc_ser, 3.2):
                    return
                self.events.put(("status", f"Manual PWM m1={m1_pct}% m2={m2_pct}%"))
                while not self.pwm_stop_event.is_set():
                    m1_pct, m2_pct, hold_ms = self._current_manual_pwm_target()
                    self._send_manual_pair(fc_ser, m1_pct, m2_pct, hold_ms)
                    self._request_esp_status(fc_ser)
                    self.events.put(("status", f"Manual PWM m1={m1_pct}% m2={m2_pct}%"))
                    if self._wait_pwm_stop_or_timeout(fc_ser, 0.4):
                        break
                self._write_fc_line(fc_ser, "DISARM")
                self._read_fc_lines(fc_ser, 0.5)
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            self.events.put(("pwm_stopped", None))

    def _ident_settings(self) -> tuple[list[int], int, int, int, int, int, Path]:
        motor_text = self.ident_motor_var.get().strip().upper()
        min_percent = int(self.ident_min_var.get(), 0)
        max_percent = int(self.ident_max_var.get(), 0)
        step_percent = int(self.ident_step_var.get(), 0)
        dwell_ms = int(self.ident_dwell_var.get(), 0)
        samples = int(self.ident_samples_var.get(), 0)

        if motor_text in (AUTO_IDENT_TEXT, "ALL", "AUTO 1+2+DUAL"):
            motors = [1, 2, 0]
            file_motor: int | None = None
        else:
            motor = int(motor_text, 0)
            if motor not in (0, 1, 2):
                raise ValueError("run must be AUTO, 0, 1 or 2")
            motors = [motor]
            file_motor = motor
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
        csv_path = Path(csv_text) if csv_text else self.default_ident_file(file_motor)
        if csv_path.exists():
            csv_path = self.default_ident_file(file_motor)
        return motors, min_percent, max_percent, step_percent, dwell_ms, samples, csv_path

    def default_ident_file(self, motor: int | None = None) -> Path:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        suffix = "auto" if motor is None else ("dual" if motor == 0 else f"m{motor}")
        return Path(__file__).with_name(f"thrust_ident_{suffix}_{stamp}.csv")

    def _raw_log(self, request: bytes, response: bytes) -> None:
        if not self.raw_log_enabled:
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

        self.raw_log_enabled = bool(self.raw_var.get())
        self.stop_event.clear()
        self.start_btn.configure(state=tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL)
        self.status_var.set(f"Reading {port} {baud} addr={addr} ch={channel}")

        self.worker = threading.Thread(
            target=self._read_loop,
            args=(port, baud, _timeout, addr, channel, interval),
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
        motor_text = self.ident_motor_var.get().strip().upper()
        if motor_text in (AUTO_IDENT_TEXT, "ALL", "AUTO 1+2+DUAL"):
            motor = None
        else:
            try:
                motor = int(motor_text, 0)
            except ValueError:
                motor = None
        path = self.default_ident_file(motor)
        self.ident_file_var.set(str(path))
        self._log(f"IDENT csv={path}")

    def _history_settings(self) -> tuple[float, float, float, str]:
        kv = float(self.kv_var.get())
        voltage = float(self.voltage_var.get())
        load_factor = float(self.load_factor_var.get())
        prop = self.prop_var.get().strip()
        if kv <= 0.0:
            raise ValueError("KV must be > 0")
        if voltage <= 0.0:
            raise ValueError("voltage must be > 0")
        if not 0.0 < load_factor <= 1.0:
            raise ValueError("load factor must be in (0, 1]")
        parse_prop(prop)
        return kv, voltage, load_factor, prop

    def _history_csv_paths(self) -> list[Path]:
        paths = set(Path(__file__).resolve().parent.glob("thrust_ident_*.csv"))
        current_text = self.ident_file_var.get().strip()
        if current_text:
            current_path = Path(current_text)
            if current_path.exists():
                paths.add(current_path)
        return sorted(paths, key=lambda item: item.stat().st_mtime if item.exists() else 0.0)

    def refresh_history(self) -> None:
        self.latest_loss_rows = []
        try:
            kv, voltage, load_factor, prop = self._history_settings()
        except Exception as exc:
            self.history_var.set(f"History settings error: {exc}")
            return

        paths = self._history_csv_paths()
        runs = load_ident_runs(paths)
        if not runs:
            self.history_var.set("History: no thrust_ident_*.csv data yet")
            return

        best_by_motor: dict[int, tuple[IdentRun, IdentPoint]] = {}
        latest_by_motor: dict[int, IdentRun] = {}
        for run in runs:
            net_points = zero_baseline_points(run.points)
            if not net_points:
                continue
            best_point = max(net_points.values(), key=lambda item: item.thrust_g)
            old_best = best_by_motor.get(run.motor)
            if old_best is None or best_point.thrust_g > old_best[1].thrust_g:
                best_by_motor[run.motor] = (run, best_point)

            old_latest = latest_by_motor.get(run.motor)
            if old_latest is None or run.modified_s > old_latest.modified_s:
                latest_by_motor[run.motor] = IdentRun(
                    path=run.path,
                    modified_s=run.modified_s,
                    motor=run.motor,
                    points=net_points,
                )

        lines = [
            f"History: {len(set(run.path for run in runs))} csv file(s), {len(runs)} motor run(s)",
            "History uses net thrust after subtracting each run's 0% baseline.",
        ]
        for motor in (1, 2, 0):
            best = best_by_motor.get(motor)
            if best is None:
                lines.append(f"{motor_name(motor)} max: no data")
                continue
            run, point = best
            stamp = datetime.fromtimestamp(run.modified_s).strftime("%m-%d %H:%M")
            rpm_est = kv * voltage * (point.pct / 100.0) * load_factor
            lines.append(
                f"{motor_name(motor)} max: {point.thrust_g:.1f} g @ {point.pct}%/"
                f"{point.pulse_us} us, rpm_est {rpm_est:.0f}, {stamp}, {run.path.name}"
            )

        missing = [motor_name(motor) for motor in (1, 2, 0) if motor not in latest_by_motor]
        if missing:
            lines.append(f"Loss latest set: waiting for {', '.join(missing)}")
            self.history_var.set("\n".join(lines))
            return

        try:
            rows = compute_loss_rows(
                latest_by_motor[1].points,
                latest_by_motor[2].points,
                latest_by_motor[0].points,
                kv,
                voltage,
                load_factor,
                prop,
            )
        except Exception as exc:
            lines.append(f"Loss latest set: {exc}")
            self.history_var.set("\n".join(lines))
            return

        self.latest_loss_rows = rows
        if not rows:
            lines.append("Loss latest set: no common pct points")
            self.history_var.set("\n".join(lines))
            return

        usable_rows = [row for row in rows if row.pct > 0 and row.single_sum_g > 0.0]
        selected_rows = usable_rows if usable_rows else rows
        mean_coeff = statistics.fmean(row.loss_coeff for row in selected_rows)
        best_dual = max(rows, key=lambda item: item.dual_g)
        lines.append(
            f"Loss latest set: coeff {mean_coeff:.3f}, loss {(1.0 - mean_coeff) * 100.0:.1f}%, "
            f"points {len(rows)}, dual max {best_dual.dual_g:.1f} g @ {best_dual.pct}%"
        )
        lines.append("RPM/tip/pitch values are estimates from KV, voltage, command pct, prop, and load factor.")
        self.history_var.set("\n".join(lines))

    def export_loss_csv(self) -> None:
        self.refresh_history()
        if not self.latest_loss_rows:
            messagebox.showinfo("No loss data", "Need M1, M2, and Dual runs with common pct points first.")
            return
        path = Path(__file__).with_name("dual_prop_loss_report.csv")
        try:
            write_loss_report(path, self.latest_loss_rows)
        except Exception as exc:
            messagebox.showerror("Export failed", str(exc))
            return
        self._log(f"LOSS csv={path}")

    def start_identification(self) -> None:
        if self.ident_worker and self.ident_worker.is_alive():
            return
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("Busy", "Stop continuous pressure reading first.")
            return
        try:
            pressure_settings = self._settings()
            motors, min_percent, max_percent, step_percent, dwell_ms, samples, csv_path = self._ident_settings()
            if is_esp_controller(self.pwm_controller_var.get()) and not self._esp_serial_is_open():
                raise ValueError("Open ESP serial before starting ESP12E identification")
        except Exception as exc:
            messagebox.showerror("Bad identification settings", str(exc))
            return

        self.raw_log_enabled = bool(self.raw_var.get())
        run_text = "->".join(motor_name(motor) for motor in motors)
        self.ident_file_var.set(str(csv_path))
        self.ident_stop_event.clear()
        self.ident_start_btn.configure(state=tk.DISABLED)
        self.ident_stop_btn.configure(state=tk.NORMAL)
        self.status_var.set(f"IDENT {run_text} {min_percent}..{max_percent}%")
        self.ident_worker = threading.Thread(
            target=self._ident_loop,
            args=(
                self.pwm_controller_var.get(),
                motors,
                min_percent,
                max_percent,
                step_percent,
                dwell_ms,
                samples,
                csv_path,
                pressure_settings,
            ),
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
            settings = self._settings()
        except Exception as exc:
            messagebox.showerror("Bad settings", str(exc))
            return
        self.raw_log_enabled = bool(self.raw_var.get())
        self.worker = threading.Thread(target=self._single_action, args=(action, value, settings), daemon=True)
        self.worker.start()

    def _read_loop(self, port: str, baud: int, timeout: float, addr: int, channel: int, interval: float) -> None:
        try:
            with self._make_pressure_serial(port, baud, timeout) as ser:
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
        controller: str,
        motors: list[int],
        min_percent: int,
        max_percent: int,
        step_percent: int,
        dwell_ms: int,
        samples: int,
        csv_path: Path,
        pressure_settings: tuple[str, int, int, int, float, float],
    ) -> None:
        try:
            port, baud, addr, channel, _interval, timeout = pressure_settings
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            with contextlib.ExitStack() as stack:
                pressure_ser = stack.enter_context(self._make_pressure_serial(port, baud, timeout))
                csv_file = stack.enter_context(csv_path.open("w", newline="", encoding="utf-8"))
                if is_esp_controller(controller):
                    self.fc_serial_lock.acquire()
                    stack.callback(self.fc_serial_lock.release)
                    fc_ser = self._require_open_esp_serial()
                else:
                    fc_ser = stack.enter_context(self._open_fc_serial())
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

                self.events.put(("log", f"IDENT csv={csv_path}"))

                for stage_index, motor in enumerate(motors, start=1):
                    if self.ident_stop_event.is_set():
                        break
                    self.events.put(
                        (
                            "status",
                            f"IDENT {stage_index}/{len(motors)} {motor_name(motor)} "
                            f"{min_percent}..{max_percent}%",
                        )
                    )

                    if is_esp_controller(controller):
                        fc_ser.write(b"ARM\r\n")
                        fc_ser.flush()
                        self.events.put(("log", f"ESP TX ARM for {motor_name(motor)}; waiting 3.2 s at 1100 us"))
                        settle_deadline = time.monotonic() + 3.2
                        while not self.ident_stop_event.is_set() and time.monotonic() < settle_deadline:
                            self._read_fc_lines(fc_ser, 0.1)
                    else:
                        fc_ser.reset_input_buffer()

                    if self.ident_stop_event.is_set():
                        break
                    command = f"IDENT START {motor} {min_percent} {max_percent} {step_percent} {dwell_ms}\r\n"
                    fc_ser.write(command.encode("ascii"))
                    fc_ser.flush()
                    self.events.put(("log", f"{controller} TX {command.strip()}"))

                    next_keepalive = time.monotonic() + 0.4
                    next_status = time.monotonic() + 0.5

                    def service_ident_serial() -> None:
                        nonlocal next_keepalive, next_status
                        if not is_esp_controller(controller):
                            return
                        now = time.monotonic()
                        if now >= next_keepalive:
                            fc_ser.write(b"IDENT KEEPALIVE\r\n")
                            fc_ser.flush()
                            next_keepalive = now + 0.4
                        if now >= next_status:
                            self._request_esp_status(fc_ser)
                            next_status = now + 0.5

                    stage_finished = False
                    while not self.ident_stop_event.is_set():
                        service_ident_serial()
                        raw_line = fc_ser.readline()
                        if not raw_line:
                            continue
                        line = raw_line.decode("utf-8", errors="replace").strip()
                        if not line:
                            continue

                        if is_esp_controller(controller):
                            self._handle_esp_line(line)
                        else:
                            self.events.put(("log", f"FC RX {line}"))
                        match = IDENT_SAMPLE_RE.search(line)
                        if match is not None:
                            record = {key: int(value) for key, value in match.groupdict().items()}
                            self._capture_ident_step(
                                writer,
                                pressure_ser,
                                addr,
                                channel,
                                samples,
                                record,
                                service_ident_serial,
                            )
                            csv_file.flush()
                        elif line.startswith("IDENT stop") or line.startswith("IDENT done"):
                            stage_finished = True
                            break

                    if stage_finished and not self.ident_stop_event.is_set():
                        if is_esp_controller(controller):
                            fc_ser.write(b"DISARM\r\n")
                            fc_ser.flush()
                            self._read_fc_lines(fc_ser, 0.3)
                        self._tare_pressure_after_stage(pressure_ser, addr, channel, motor_name(motor))
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            try:
                if is_esp_controller(controller):
                    with self.fc_serial_lock:
                        fc_ser = self._require_open_esp_serial()
                        fc_ser.write(b"IDENT STOP\r\n")
                        fc_ser.write(b"DISARM\r\n")
                        fc_ser.flush()
                else:
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
        keepalive: Callable[[], None] | None = None,
    ) -> None:
        grams_values: list[float] = []

        for sample_index in range(samples):
            if self.ident_stop_event.is_set():
                break
            if keepalive is not None:
                keepalive()
            error_text = ""
            raw_value: int | str = ""
            grams_value: float | str = ""
            try:
                raw_reading = read_channel_weight(pressure_ser, addr, channel, log=self._raw_log)
                if keepalive is not None:
                    keepalive()
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

    def _read_pressure_sample_grams(
        self,
        pressure_ser: serial.Serial,
        addr: int,
        channel: int,
    ) -> tuple[int, float]:
        raw_value = read_channel_weight(pressure_ser, addr, channel, log=self._raw_log)
        grams = self.calibrated_grams(float(raw_value))
        return raw_value, float(raw_value) if grams is None else float(grams)

    def _read_pressure_sample_window(
        self,
        pressure_ser: serial.Serial,
        addr: int,
        channel: int,
    ) -> tuple[float, float, float]:
        raw_values: list[int] = []
        gram_values: list[float] = []
        for _ in range(AUTO_TARE_VERIFY_SAMPLES):
            raw_value, grams = self._read_pressure_sample_grams(pressure_ser, addr, channel)
            raw_values.append(raw_value)
            gram_values.append(grams)
            time.sleep(AUTO_TARE_VERIFY_INTERVAL_S)
        mean_raw = statistics.fmean(raw_values)
        mean_g = statistics.fmean(gram_values)
        std_g = statistics.pstdev(gram_values) if len(gram_values) > 1 else 0.0
        return mean_raw, mean_g, std_g

    def _tare_pressure_after_stage(
        self,
        pressure_ser: serial.Serial,
        addr: int,
        channel: int,
        stage_name: str,
    ) -> None:
        command_register = 0x0028 + (channel - 1) * 10
        time.sleep(AUTO_PRE_TARE_SETTLE_S)
        try:
            before_raw, before_g, before_std = self._read_pressure_sample_window(pressure_ser, addr, channel)
        except Exception as exc:
            before_raw = before_g = before_std = float("nan")
            self.events.put(("log", f"WARN tare pre-read after {stage_name}: {exc}"))

        write_single_register(pressure_ser, addr, command_register, AUTO_TARE_COMMAND, log=self._raw_log)
        time.sleep(AUTO_TARE_SETTLE_S)

        try:
            after_raw, after_g, after_std = self._read_pressure_sample_window(pressure_ser, addr, channel)
            if abs(after_g) > AUTO_TARE_WARN_ABS_G:
                self.events.put(
                    (
                        "log",
                        f"WARN tare after {stage_name} still offset {after_g:.1f}g; "
                        "check fixture rest state before next run",
                    )
                )
        except Exception as exc:
            after_raw = after_g = after_std = float("nan")
            self.events.put(("log", f"WARN tare verify after {stage_name}: {exc}"))

        self.events.put(
            (
                "log",
                f"TARE after {stage_name} ch{channel} reg=0x{command_register:04X} value={AUTO_TARE_COMMAND} "
                f"before raw={before_raw:.1f} {before_g:.1f}g std={before_std:.1f}g; "
                f"after raw={after_raw:.1f} {after_g:.1f}g std={after_std:.1f}g",
            )
        )

    def _single_action(
        self,
        action: str,
        command_value: int,
        settings: tuple[str, int, int, int, float, float],
    ) -> None:
        try:
            port, baud, addr, channel, _interval, timeout = settings
            command_register = 0x0028 + (channel - 1) * 10
            cal_weight_register = 0x0029 + (channel - 1) * 10
            with self._make_pressure_serial(port, baud, timeout) as ser:
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
            elif event == "status":
                self.status_var.set(str(payload))
            elif event == "esp_status":
                status = payload
                if isinstance(status, dict):
                    self.esp_status_var.set(
                        "ESP STATUS: "
                        f"{status.get('mode')} "
                        f"armed={status.get('armed')} "
                        f"settled={status.get('arm_settled')} "
                        f"M1={status.get('m1_pulse')}us "
                        f"M2={status.get('m2_pulse')}us "
                        f"ident={status.get('ident_motor')}:{status.get('ident_pct')}%"
                    )
            elif event == "esp_serial_state":
                state = payload
                if isinstance(state, dict):
                    opened = bool(state.get("open"))
                    self.esp_serial_var.set(str(state.get("text", "ESP SERIAL: unknown")))
                    self._set_esp_controls_state(opened)
                    if not opened:
                        self.esp_status_var.set("ESP STATUS: unknown")
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
                self.refresh_history()
            elif event == "pwm_stopped":
                self._set_esp_controls_state(self._esp_serial_is_open())
                if self.status_var.get() == "Stopping PWM...":
                    self.status_var.set("PWM stopped")
                elif self.status_var.get().startswith("Manual PWM"):
                    self.status_var.set("PWM done")
            elif event == "single_done":
                pass

        self.after(60, self._poll_events)

    def _log(self, text: str) -> None:
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)

    def on_close(self) -> None:
        self.stop_event.set()
        self.ident_stop_event.set()
        self.pwm_stop_event.set()
        with self.fc_serial_lock:
            if self.fc_serial is not None:
                with contextlib.suppress(Exception):
                    if self.fc_serial.is_open:
                        self.fc_serial.write(b"DISARM\r\n")
                        self.fc_serial.flush()
                with contextlib.suppress(Exception):
                    self.fc_serial.close()
                self.fc_serial = None
        self.destroy()


def main() -> None:
    app = PressureGui()
    app.mainloop()


if __name__ == "__main__":
    main()
