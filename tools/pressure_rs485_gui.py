#!/usr/bin/env python3
"""Pressure calibration and H743 measured thrust control in one workflow.

The original scale/calibration workflow is retained. Motor controls use the H743
TBENCH state machine. Historical CSV helpers remain for the offline viewer only.
"""

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
    from .project_paths import (
        PRESSURE_CALIBRATION_DIR,
        THRUST_IDENT_DIR,
        dated_directory,
    )
except ImportError:  # Allows running as: python tools/pressure_rs485_gui.py
    try:
        from tools.project_paths import (
            PRESSURE_CALIBRATION_DIR,
            THRUST_IDENT_DIR,
            dated_directory,
        )
    except ImportError:
        from project_paths import (
            PRESSURE_CALIBRATION_DIR,
            THRUST_IDENT_DIR,
            dated_directory,
        )

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial is required: pip install pyserial") from exc

try:
    from tools.pressure_rs485_test import (
        DEFAULT_BAUD, DEFAULT_DIP, DEFAULT_TIMEOUT, dip_to_addr,
        read_channel_weight, read_holding_registers, scan_addresses,
        write_single_register,
    )
    from tools.thrust_bench.legacy_calibration import legacy_grams
    from tools.thrust_bench.pressure_host import PressureGuiBenchBridge
    from tools.thrust_bench.ui import ThrustBenchFrame
except ImportError:  # Direct script: python tools/pressure_rs485_gui.py
    import importlib.util
    import sys
    tools_dir = Path(__file__).resolve().parent
    spec = importlib.util.spec_from_file_location(
        "tools", tools_dir / "__init__.py",
        submodule_search_locations=[str(tools_dir)])
    if spec is None or spec.loader is None:
        raise ImportError("cannot initialize tools package for direct script")
    tools_package = importlib.util.module_from_spec(spec)
    sys.modules.setdefault("tools", tools_package)
    spec.loader.exec_module(tools_package)
    from tools.pressure_rs485_test import (
        DEFAULT_BAUD, DEFAULT_DIP, DEFAULT_TIMEOUT, dip_to_addr,
        read_channel_weight, read_holding_registers, scan_addresses,
        write_single_register,
    )
    from tools.thrust_bench.legacy_calibration import legacy_grams
    from tools.thrust_bench.pressure_host import PressureGuiBenchBridge
    from tools.thrust_bench.ui import ThrustBenchFrame

CALIBRATION_FILE = PRESSURE_CALIBRATION_DIR / "pressure_calibration.json"
DEFAULT_REFERENCE_WEIGHTS = (231.8, 346.5, 504.9)
DEFAULT_MOTOR_KV = 1300.0
DEFAULT_BATTERY_VOLTAGE = 12.6
DEFAULT_PROP = "9050"
DEFAULT_LOAD_FACTOR = 0.80
HISTORY_TAIL_SAMPLES = 5


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
        return "双桨"
    return f"M{motor}"




def percent_to_pulse(percent: int) -> int:
    return 1100 + round(max(0, min(100, percent)) * (1940 - 1100) / 100.0)


def parse_prop(prop: str) -> tuple[float, float]:
    text = prop.strip().lower().replace("x", "")
    if len(text) != 4 or not text.isdigit():
        raise ValueError("桨叶规格请填写 9050、9047 或 1045 这样的四位数字")
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
        self.title("H743 共轴推力台 · 称重与实测辨识")
        self.geometry("1120x860"); self.minsize(900, 660)
        self.events = queue.Queue(); self.stop_event = threading.Event()
        self.worker = None; self.calibration_points = []; self.last_raw_value = None
        self.raw_log_enabled = True; self._closing = False; self._poll_id = None
        defaults = {"port":"", "baud":str(DEFAULT_BAUD), "dip":DEFAULT_DIP,
                    "addr":"", "channel":"1", "interval":"0.2", "timeout":str(DEFAULT_TIMEOUT),
                    "value":"原始值 --", "cal_value":"重量 -- 克", "status":"待命",
                    "ref_weight":str(DEFAULT_REFERENCE_WEIGHTS[0]), "fc_port":"", "fc_baud":"115200"}
        for name,value in defaults.items(): setattr(self,name+"_var",tk.StringVar(self,value=value))
        self.raw_var = tk.BooleanVar(self,value=True)
        self.bench_bridge = PressureGuiBenchBridge(self)
        self._build_ui(); self.load_calibration(); self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW",self.on_close)
        self._poll_id = self.after(60,self._poll_events)

    def _build_ui(self) -> None:
        footer=ttk.Frame(self,padding=(10,6)); footer.pack(side=tk.BOTTOM,fill=tk.X)
        body=ttk.Frame(self); body.pack(fill=tk.BOTH,expand=True)
        self.main_canvas=tk.Canvas(body,highlightthickness=0)
        scrollbar=ttk.Scrollbar(body,orient=tk.VERTICAL,command=self.main_canvas.yview)
        scrollbar.pack(side=tk.RIGHT,fill=tk.Y); self.main_canvas.pack(fill=tk.BOTH,expand=True)
        self.main_canvas.configure(yscrollcommand=scrollbar.set)
        root=ttk.Frame(self.main_canvas,padding=10)
        window=self.main_canvas.create_window((0,0),window=root,anchor="nw")
        root.bind("<Configure>",lambda _e:self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all")))
        self.main_canvas.bind("<Configure>",lambda e:self.main_canvas.itemconfigure(window,width=e.width))
        cfg=ttk.LabelFrame(root,text="1  称重连接与读数",padding=6); cfg.pack(fill=tk.X)
        for col,(text,key,width) in enumerate((("称重串口","port",12),("波特率","baud",8),("拨码","dip",6),("地址","addr",5),("通道","channel",3))):
            ttk.Label(cfg,text=text).grid(row=0,column=col*2,padx=3)
            w=ttk.Combobox(cfg,textvariable=getattr(self,key+"_var"),width=width) if key=="port" else ttk.Entry(cfg,textvariable=getattr(self,key+"_var"),width=width)
            w.grid(row=0,column=col*2+1,padx=3)
            if key=="port": self.port_combo=w
        ttk.Button(cfg,text="刷新串口",command=self.refresh_ports).grid(row=0,column=10,padx=3)
        controls=ttk.Frame(cfg);controls.grid(row=1,column=0,columnspan=11,sticky="ew",pady=(6,0))
        self.start_btn=ttk.Button(controls,text="开始称重",command=self.start_reading);self.start_btn.pack(side=tk.LEFT)
        self.stop_btn=ttk.Button(controls,text="停止读取",command=self.stop_reading,state=tk.DISABLED);self.stop_btn.pack(side=tk.LEFT,padx=3)
        for text,callback in (("读取一次",self.read_once),("扫描地址",self.scan_addr),("去皮",lambda:self.write_command(2)),("置零",lambda:self.write_command(1))):
            ttk.Button(controls,text=text,command=callback).pack(side=tk.LEFT,padx=3)
        ttk.Checkbutton(controls,text="原始收发",variable=self.raw_var).pack(side=tk.LEFT,padx=4)
        readings=ttk.Frame(cfg);readings.grid(row=2,column=0,columnspan=11,sticky="ew",pady=4)
        ttk.Label(readings,textvariable=self.cal_value_var,font=("Microsoft YaHei UI",18,"bold")).pack(side=tk.LEFT,padx=8)
        ttk.Label(readings,textvariable=self.value_var).pack(side=tk.LEFT,padx=15)
        ttk.Label(readings,textvariable=self.status_var).pack(side=tk.RIGHT,padx=5)
        cal=ttk.LabelFrame(root,text="2  砝码标定（沿用已有记录）",padding=6);cal.pack(fill=tk.X,pady=6)
        ttk.Label(cal,text="砝码重量（克）").grid(row=0,column=0,padx=3)
        self.ref_combo=ttk.Combobox(cal,textvariable=self.ref_weight_var,values=DEFAULT_REFERENCE_WEIGHTS,width=10)
        self.ref_combo.grid(row=0,column=1,padx=3)
        for col,(text,callback) in enumerate((("采集标定点",self.capture_calibration_point),("写入传感器标定",self.write_module_calibration),("清除软件标定",self.clear_calibration)),2):
            ttk.Button(cal,text=text,command=callback).grid(row=0,column=col,padx=4)
        self.cal_points_var=tk.StringVar(self,value="尚无标定点")
        ttk.Label(cal,textvariable=self.cal_points_var,wraplength=820).grid(row=1,column=0,columnspan=5,sticky="w",padx=4,pady=(4,0))
        self.thrust_bench_frame=ThrustBenchFrame(root,host_bridge=self.bench_bridge)
        self.thrust_bench_frame.pack(fill=tk.X)
        self.log_text=self.thrust_bench_frame.live
        self.global_stop=ttk.Button(footer,text="立即停止上下桨",command=self.thrust_bench_frame.stop)
        self.global_stop.pack(side=tk.RIGHT,padx=6)
        self.thrust_bench_frame.status_label=ttk.Label(footer,textvariable=self.thrust_bench_frame.status,wraplength=680)
        self.thrust_bench_frame.status_label.pack(side=tk.LEFT,fill=tk.X,expand=True)

    def refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        self.fc_port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        if ports and not self.fc_port_var.get():
            self.fc_port_var.set(ports[1] if len(ports) > 1 else ports[0])
        self._log(f"可用串口：{', '.join(ports) if ports else '无'}")

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
            self._log(f"已加载标定：{CALIBRATION_FILE}")
        except Exception as exc:
            self._log(f"标定加载失败：{exc}")
        self._update_calibration_label()

    def save_calibration(self) -> None:
        data = {
            "points": [
                {"raw": raw, "grams": grams}
                for raw, grams in sorted(self.calibration_points, key=lambda item: item[0])
            ]
        }
        CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
        CALIBRATION_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")
        self._update_calibration_label()

    def calibrated_grams(self, raw_value: float) -> float | None:
        return legacy_grams(raw_value, self.calibration_points)

    def _bench_blocks_legacy(self, action: str) -> bool:
        return self.bench_bridge.scale_blocked(action)

    def _update_value_display(self, raw_value: int) -> None:
        self.last_raw_value = raw_value
        self.value_var.set(f"原始值 {raw_value}")
        calibrated = self.calibrated_grams(float(raw_value))
        if calibrated is None:
            self.cal_value_var.set("重量 -- 克")
        else:
            self.cal_value_var.set(f"重量 {calibrated:.1f} 克")

    def _update_calibration_label(self) -> None:
        if not self.calibration_points:
            self.cal_points_var.set("尚无标定点")
            return
        items = [
            f"{raw:g}->{grams:g}g"
            for raw, grams in sorted(self.calibration_points, key=lambda item: item[0])
        ]
        self.cal_points_var.set("; ".join(items))

    def capture_calibration_point(self) -> None:
        if self._bench_blocks_legacy("采集砝码点"): return
        if self.last_raw_value is None:
            messagebox.showinfo("尚无读数", "请先读取一次传感器，再采集标定点。")
            return
        try:
            grams = float(self.ref_weight_var.get())
        except ValueError:
            messagebox.showerror("砝码重量有误", "砝码重量须为数字，例如 346.5 克。")
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
        self._log(f"标定：原始值={raw:g} -> {grams:g}g")

    def clear_calibration(self) -> None:
        if self._bench_blocks_legacy("清除软件标定"): return
        self.calibration_points = []
        if CALIBRATION_FILE.exists():
            CALIBRATION_FILE.unlink()
        self._update_calibration_label()
        if self.last_raw_value is not None:
            self._update_value_display(self.last_raw_value)
        self._log("软件标定已清除")

    def _settings(self) -> tuple[str, int, int, int, float, float]:
        port = self.port_var.get().strip()
        if not port:
            raise ValueError("请选择称重串口")
        baud = int(self.baud_var.get(), 0)
        channel = int(self.channel_var.get(), 0)
        interval = float(self.interval_var.get())
        timeout = float(self.timeout_var.get())
        addr_text = self.addr_var.get().strip()
        addr = int(addr_text, 0) if addr_text else dip_to_addr(self.dip_var.get())
        if not 1 <= addr <= 254:
            raise ValueError("地址必须为 1～254")
        if not 1 <= channel <= 4:
            raise ValueError("通道必须为 1～4")
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

    def _raw_log(self, request: bytes, response: bytes) -> None:
        if not self.raw_log_enabled:
            return
        self.events.put(("log", f"发送：{request.hex(' ').upper()}"))
        self.events.put(("log", f"接收：{response.hex(' ').upper() if response else '超时'}"))

    def start_reading(self) -> None:
        if self._bench_blocks_legacy("连续称重"):
            return
        if self.worker and self.worker.is_alive():
            return
        try:
            port, baud, addr, channel, interval, _timeout = self._settings()
        except Exception as exc:
            messagebox.showerror("连接设置有误", str(exc))
            return

        self.raw_log_enabled = bool(self.raw_var.get())
        self.stop_event.clear()
        self.start_btn.configure(state=tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL)
        self.status_var.set(f"正在读取 {port}，波特率 {baud}，地址 {addr}，通道 {channel}")

        self.worker = threading.Thread(
            target=self._read_loop,
            args=(port, baud, _timeout, addr, channel, interval),
            daemon=True,
        )
        self.worker.start()

    def stop_reading(self) -> None:
        self.stop_event.set()
        self.status_var.set("正在停止读取…")

    def read_once(self) -> None:
        self._run_once_worker("read")

    def scan_addr(self) -> None:
        self._run_once_worker("scan")

    def write_command(self, value: int) -> None:
        if self.bench_bridge.bench_active and value == 2:
            self.thrust_bench_frame.tare()
            return
        self._run_once_worker("write", value)

    def write_module_calibration(self) -> None:
        try:
            grams = float(self.ref_weight_var.get())
        except ValueError:
            messagebox.showerror("砝码重量有误", "砝码重量须为数字，例如 346.5 克。")
            return
        if grams <= 0.0 or grams > 65535.0:
            messagebox.showerror("砝码重量有误", "传感器标定重量必须为 1～65535 克。")
            return
        weight_value = int(round(grams))
        if abs(weight_value - grams) > 0.001:
            ok = messagebox.askyesno(
                "确认重量取整",
                f"传感器仅支持整数克数。\n是否将 {grams:g} 克取整为 {weight_value} 克写入？",
            )
            if not ok:
                return
        self._run_once_worker("module_cal", weight_value)

    def _run_once_worker(self, action: str, value: int = 0) -> None:
        if self._bench_blocks_legacy("称重短操作"):
            return
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("正在使用", "请先停止连续读取。")
            return
        try:
            settings = self._settings()
        except Exception as exc:
            messagebox.showerror("连接设置有误", str(exc))
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
                    self.events.put(("log", f"地址扫描结果：{found}"))
                    if found:
                        self.events.put(("addr", found[0]))
                elif action == "write":
                    write_single_register(ser, addr, command_register, command_value, log=self._raw_log)
                    name = "置零" if command_value == 1 else "去皮"
                    self.events.put(("log", f"{name}完成，通道{channel}，寄存器=0x{command_register:04X}"))
                elif action == "module_cal":
                    write_single_register(ser, addr, cal_weight_register, command_value, log=self._raw_log)
                    self.events.put(("log", f"传感器标定完成，通道{channel}，寄存器0x{cal_weight_register:04X}，重量{command_value}克"))
                else:
                    raise ValueError(action)
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            self.events.put(("single_done", None))

    def _poll_events(self) -> None:
        if self._closing: return
        for _ in range(100):
            try: event,payload=self.events.get_nowait()
            except queue.Empty: break
            if event=="value": self._update_value_display(int(payload)); self.status_var.set("读取正常")
            elif event=="log": self._log(str(payload))
            elif event=="addr": self.addr_var.set(str(payload))
            elif event=="error": self.status_var.set(f"错误：{payload}"); self._log(str(payload))
            elif event=="stopped":
                self.start_btn.configure(state=tk.NORMAL);self.stop_btn.configure(state=tk.DISABLED)
                self.status_var.set("读取已停止")
        self._poll_id=self.after(60,self._poll_events)

    def _log(self,text: str) -> None:
        self.thrust_bench_frame._append(text)

    def on_close(self) -> None:
        self._closing=True;self.stop_event.set()
        if self._poll_id is not None:
            with contextlib.suppress(tk.TclError): self.after_cancel(self._poll_id)
        try: self.thrust_bench_frame.shutdown()
        finally: self.destroy()


def main() -> None:
    PressureGui().mainloop()

if __name__ == "__main__": main()
