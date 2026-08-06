#!/usr/bin/env python3
"""Point-and-click UI for full-rate raw IMU vibration captures.

Pick a COM port, pick a test step, press the button. Each step is a preset that
records one capture and tags the file, so the throttle sweep produces a
consistently named set that the analysis can compare.

Capture and analysis logic lives in imu_vibration_capture.py; this module only
drives it. Firmware side: App/Src/app_imu_capture.c.
"""

from __future__ import annotations

import argparse
import json
import queue
import threading
import time
import traceback
from pathlib import Path

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    # Works when imported as part of the `tools` package (e.g. from tests).
    from . import imu_vibration_capture as ivc
except ImportError:  # pragma: no cover - direct script execution
    import imu_vibration_capture as ivc


# Test steps for the vibration sweep. The fixed-frame throttle steps are the
# important ones: the blade tone moves with rotor speed, while an aliased peak
# moves the opposite way, and that is the only reliable way to tell them apart.
TEST_STEPS: list[tuple[str, str, str]] = [
    ("静止（噪声底）", "static",
     "电机停转，飞机放在桌面。测传感器噪声底和混叠基线。"),
    ("定速 30%", "thr30",
     "机架必须固定牢靠。电机定速，不解锁。"),
    ("定速 50%", "thr50",
     "机架必须固定牢靠。接近悬停转速，谱线最有代表性。"),
    ("定速 70%", "thr70",
     "机架必须固定牢靠。用于确认基频随转速上移。"),
    ("定速 90%", "thr90",
     "机架必须固定牢靠。检查二次谐波是否越过 Nyquist。"),
    ("悬停", "hover",
     "仅在姿态已能稳住时进行。完整工况，含机架共振。"),
]

FRAME_WARNING_TAGS = {"thr30", "thr50", "thr70", "thr90"}


class VibrationCaptureUI(tk.Tk):
    def __init__(self, out_dir: Path | None = None):
        super().__init__()
        self.title("drone-H743 IMU 振动采集")
        self.geometry("880x620")

        self.out_dir = Path(out_dir) if out_dir else ivc.DEFAULT_OUT_DIR
        self.events: queue.Queue = queue.Queue()
        self.worker: threading.Thread | None = None
        self.last_csv: Path | None = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.step_var = tk.StringVar(value=TEST_STEPS[0][0])
        self.samples_var = tk.StringVar(value="0")
        self.out_var = tk.StringVar(value=str(self.out_dir))
        self.status_var = tk.StringVar(value="就绪")
        self.analyse_var = tk.BooleanVar(value=True)

        self._build_ui()
        self.refresh_ports()
        self._on_step_change()
        self.after(80, self._poll_events)

    # ---------------- UI ----------------

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        conn = ttk.LabelFrame(root, text="连接", padding=8)
        conn.pack(fill=tk.X)

        ttk.Label(conn, text="COM 口").pack(side=tk.LEFT)
        self.port_box = ttk.Combobox(conn, textvariable=self.port_var,
                                     width=14, state="readonly")
        self.port_box.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Button(conn, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT)

        ttk.Label(conn, text="波特率").pack(side=tk.LEFT, padx=(14, 4))
        ttk.Combobox(conn, textvariable=self.baud_var, width=10,
                     values=("115200", "921600"), state="readonly").pack(side=tk.LEFT)

        ttk.Button(conn, text="读取固件状态",
                   command=self.query_status).pack(side=tk.LEFT, padx=(14, 0))

        step = ttk.LabelFrame(root, text="测试档位", padding=8)
        step.pack(fill=tk.X, pady=(10, 0))

        row = ttk.Frame(step)
        row.pack(fill=tk.X)
        ttk.Label(row, text="档位").pack(side=tk.LEFT)
        self.step_box = ttk.Combobox(
            row, textvariable=self.step_var, width=20, state="readonly",
            values=[name for name, _, _ in TEST_STEPS])
        self.step_box.pack(side=tk.LEFT, padx=(6, 10))
        self.step_box.bind("<<ComboboxSelected>>", lambda _e: self._on_step_change())

        ttk.Label(row, text="样本数 (0=满缓冲)").pack(side=tk.LEFT, padx=(10, 4))
        ttk.Entry(row, textvariable=self.samples_var, width=8).pack(side=tk.LEFT)

        ttk.Checkbutton(row, text="采集后立即分析",
                        variable=self.analyse_var).pack(side=tk.LEFT, padx=(14, 0))

        self.hint_label = ttk.Label(step, text="", foreground="#555",
                                    wraplength=820, justify=tk.LEFT)
        self.hint_label.pack(fill=tk.X, pady=(8, 0))

        out = ttk.Frame(root)
        out.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(out, text="输出目录").pack(side=tk.LEFT)
        ttk.Entry(out, textvariable=self.out_var).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        ttk.Button(out, text="选择", command=self.choose_out_dir).pack(side=tk.LEFT)

        actions = ttk.Frame(root)
        actions.pack(fill=tk.X, pady=(12, 0))
        self.capture_btn = ttk.Button(actions, text="开始采集",
                                      command=self.start_capture)
        self.capture_btn.pack(side=tk.LEFT)
        ttk.Button(actions, text="分析已有文件",
                   command=self.analyse_existing).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(actions, text="清空日志",
                   command=lambda: self.log_text.delete("1.0", tk.END)).pack(
                       side=tk.LEFT, padx=(8, 0))

        self.progress = ttk.Progressbar(root, mode="determinate", maximum=1.0)
        self.progress.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(root, textvariable=self.status_var).pack(fill=tk.X, pady=(4, 0))

        log_frame = ttk.LabelFrame(root, text="日志 / 分析报告", padding=6)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self.log_text = tk.Text(log_frame, wrap=tk.NONE, height=18,
                                font=("Consolas", 9))
        scroll_y = ttk.Scrollbar(log_frame, orient=tk.VERTICAL,
                                 command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll_y.set)
        scroll_y.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    # ---------------- helpers ----------------

    def _current_step(self) -> tuple[str, str, str]:
        for entry in TEST_STEPS:
            if entry[0] == self.step_var.get():
                return entry
        return TEST_STEPS[0]

    def _on_step_change(self) -> None:
        _, tag, hint = self._current_step()
        prefix = "⚠ " if tag in FRAME_WARNING_TAGS else ""
        self.hint_label.configure(
            text=f"{prefix}{hint}",
            foreground="#a33" if tag in FRAME_WARNING_TAGS else "#555")

    def refresh_ports(self) -> None:
        ports = ivc.list_serial_ports()
        self.port_box.configure(values=ports)
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if not ports:
            self.log("未发现串口。检查 USB 连接，或确认已安装 pyserial。")

    def choose_out_dir(self) -> None:
        chosen = filedialog.askdirectory(initialdir=self.out_var.get())
        if chosen:
            self.out_var.set(chosen)

    def log(self, message: str) -> None:
        self.log_text.insert(tk.END, message.rstrip() + "\n")
        self.log_text.see(tk.END)

    def _busy(self, busy: bool) -> None:
        self.capture_btn.configure(state=tk.DISABLED if busy else tk.NORMAL)

    def _poll_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "log":
                    self.log(payload)
                elif kind == "status":
                    message, fraction = payload
                    self.status_var.set(message)
                    if fraction is not None:
                        self.progress.configure(value=max(0.0, min(1.0, fraction)))
                elif kind == "done":
                    self._busy(False)
                    self.status_var.set(payload)
                elif kind == "error":
                    self._busy(False)
                    self.status_var.set("失败")
                    self.log(payload)
                    messagebox.showerror("采集失败", payload.strip().splitlines()[-1])
        except queue.Empty:
            pass
        self.after(80, self._poll_events)

    # ---------------- actions ----------------

    def query_status(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("缺少串口", "请先选择 COM 口。")
            return

        def work() -> None:
            try:
                link = ivc.CaptureLink(port, int(self.baud_var.get()))
                try:
                    link.send("IMUCAP?")
                    line = link.read_text_line(time.monotonic() + 3.0)
                    self.events.put(("log", line or "（无响应）"))
                finally:
                    link.close()
                self.events.put(("done", "就绪"))
            except Exception:
                self.events.put(("error", traceback.format_exc()))

        self._start_worker(work)

    def start_capture(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("缺少串口", "请先选择 COM 口。")
            return

        try:
            samples = int(self.samples_var.get() or "0")
        except ValueError:
            messagebox.showwarning("样本数无效", "样本数必须是整数，0 表示满缓冲。")
            return

        name, tag, _ = self._current_step()
        if tag in FRAME_WARNING_TAGS and not messagebox.askokcancel(
                "确认机架已固定",
                f"即将进行「{name}」，电机会转起来。\n\n"
                "请确认机架已牢靠固定，桨叶周围无人无障碍物。\n\n继续？"):
            return

        out_dir = Path(self.out_var.get())
        baud = int(self.baud_var.get())
        analyse = self.analyse_var.get()

        self.progress.configure(value=0.0)
        self.log(f"\n===== {name}  (tag={tag}) =====")

        def work() -> None:
            try:
                def progress(message: str, fraction: float | None) -> None:
                    self.events.put(("status", (message, fraction)))

                csv_path = ivc.run_capture(
                    port, baud=baud, samples=samples, tag=tag,
                    out_dir=out_dir, progress=progress)
                self.last_csv = csv_path
                self.events.put(("log", f"已保存 {csv_path}"))

                if analyse:
                    self.events.put(("status", ("分析中 ...", None)))
                    rows, meta = ivc.read_csv(csv_path)
                    report = ivc.analyse(rows, meta, label=csv_path.name)
                    self.events.put(("log", _format_report(report)))
                    out_json = csv_path.with_name(csv_path.stem + "_analysis.json")
                    out_json.write_text(json.dumps(report, indent=2),
                                        encoding="utf-8")
                    self.events.put(("log", f"报告已写入 {out_json.name}"))

                self.events.put(("done", "完成"))
            except Exception:
                self.events.put(("error", traceback.format_exc()))

        self._start_worker(work)

    def analyse_existing(self) -> None:
        path = filedialog.askopenfilename(
            initialdir=self.out_var.get(),
            filetypes=[("CSV", "*.csv"), ("所有文件", "*.*")])
        if not path:
            return
        csv_path = Path(path)

        def work() -> None:
            try:
                rows, meta = ivc.read_csv(csv_path)
                report = ivc.analyse(rows, meta, label=csv_path.name)
                self.events.put(("log", _format_report(report)))
                self.events.put(("done", "分析完成"))
            except Exception:
                self.events.put(("error", traceback.format_exc()))

        self._start_worker(work)

    def _start_worker(self, target) -> None:
        if self.worker is not None and self.worker.is_alive():
            messagebox.showinfo("忙", "上一个操作还在进行中。")
            return
        self._busy(True)
        self.worker = threading.Thread(target=target, daemon=True)
        self.worker.start()


def _format_report(report: dict) -> str:
    """Render the analysis report as text for the log pane."""
    lines: list[str] = []
    lines.append(f"--- 分析: {report.get('label')}")
    lines.append(f"  样本 {report['samples']}  速率 {report['effective_rate_hz']:.1f} Hz"
                 f"  时长 {report['duration_s']:.2f} s"
                 f"  分辨率 {report['freq_resolution_hz']:.2f} Hz")
    fw = report["firmware"]
    lines.append(f"  固件: 加速度 ±{fw['accel_range_g']}g  陀螺 ±{fw['gyro_range_dps']}dps"
                 f"  AAF {fw['accel_aaf_hz']}/{fw['gyro_aaf_hz']} Hz")
    rotor = report["rotor"]
    lines.append(f"  电机: 中位 {rotor['motor_upper_us_median']:.0f} us"
                 f"  转动={rotor['spinning']}")

    for kind, label in (("accel", "加速度"), ("gyro", "陀螺")):
        c = report["clipping"][kind]
        lines.append(f"  {label}原始: |max| {c['abs_max_lsb']:.0f} LSB"
                     f"  p99.9 {c['p99_9_lsb']:.0f}  削顶 {c['clipped_pct']:.2f}%")

    lines.append("  主频 (原始，滤波前):")
    for key in sorted(report["raw_spectra"]):
        block = report["raw_spectra"][key]
        peaks = ", ".join(f"{p['freq_hz']:.1f}Hz({p['amplitude']:.4f})"
                          for p in block["peaks_hz"][:3])
        lines.append(f"    {key:<10} rms {block['rms']:.4f} {block['unit']:<4} {peaks}")

    if report["lpf_measured"]:
        lines.append("  IIR 实测衰减:")
        for key in sorted(report["lpf_measured"]):
            m = report["lpf_measured"][key]
            att = f"{m['attenuation_db']:.1f} dB" if m["attenuation_db"] is not None else "n/a"
            lines.append(f"    {key:<10} {m['pre_rms']:.4f} -> {m['post_rms']:.4f}  {att}")

    fh = report["fusion_health"]
    lines.append(f"  Fusion: 忽略加速度 {fh['accel_ignored']}%"
                 f"  模长拒绝 {fh['norm_rejected']}%"
                 f"  恢复 {fh['accel_recovery']}%"
                 f"  创新中位 {fh['accel_error_deg_median']:.2f}°")

    rec = report["recommendation"]
    lines.append(f"  建议: 主频 {rec['dominant_gyro_tone_hz']} Hz"
                 f"  陀螺 LPF {rec['suggested_gyro_lpf_hz']} Hz"
                 f"  加速度 LPF {rec['suggested_accel_lpf_hz']} Hz")
    for note in rec["notes"]:
        lines.append(f"  ! {note}")
    if not rec["notes"]:
        lines.append("  (未发现异常)")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args(argv)

    app = VibrationCaptureUI(out_dir=args.out_dir)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
