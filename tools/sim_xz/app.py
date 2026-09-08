"""Tk front end for the host-only simulator."""

from __future__ import annotations

import math
import queue
import threading
import tkinter as tk
import argparse
from collections import deque
from tkinter import ttk

from tools.project_paths import SIMULATION_DIR

from .device import SimulatorDevice
from .experiments import ExperimentKind, ExperimentTargets, run_ab, write_ab_artifact


class SimulationApp(tk.Tk):
    def __init__(self, device: SimulatorDevice | None = None) -> None:
        super().__init__()
        self.title("R-SIM-1 X/Z 教学仿真")
        self.geometry("1180x900")
        self.device = device or SimulatorDevice()
        self._ui_queue: queue.Queue[object] = queue.Queue()
        self._ab_result = None
        self._baseline_config = None
        self._trajectory = deque(maxlen=1200)
        self._last_trajectory_time = -1.0
        self._closing = False
        self._ab_busy = False
        self._ab_thread: threading.Thread | None = None
        self.device.start()
        self._build_widgets()
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(33, self._render)

    def _build_widgets(self) -> None:
        self.canvas = tk.Canvas(self, width=650, height=540, background="#101820", highlightthickness=0)
        self.canvas.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=12, pady=12)
        side = ttk.Frame(self, padding=12)
        side.grid(row=0, column=1, sticky="nsew")
        self.status = tk.StringVar(value="等待地面站")
        ttk.Label(side, textvariable=self.status).pack(anchor="w")
        ttk.Button(side, text="开始 / 暂停", command=self._toggle).pack(fill="x", pady=4)
        ttk.Button(side, text="复位", command=self._reset).pack(fill="x", pady=4)
        ttk.Label(side, text="实验").pack(anchor="w", pady=(14, 2))
        self.kind = tk.StringVar(value=ExperimentKind.POSITION_STEP.value)
        combo = ttk.Combobox(side, textvariable=self.kind, state="readonly",
                             values=[kind.value for kind in ExperimentKind])
        combo.pack(fill="x")
        combo.bind("<<ComboboxSelected>>", lambda _event: self.device.set_kind(ExperimentKind(self.kind.get())))
        targets = ttk.Frame(side)
        targets.pack(fill="x", pady=(10, 0))
        self.target_x = tk.StringVar(value="0.5")
        self.target_vx = tk.StringVar(value="0.2")
        self.target_pitch = tk.StringVar(value="3.0")
        for row, (label, variable) in enumerate((("位置阶跃 x (m)", self.target_x),
                                                  ("速度阶跃 vx (m/s)", self.target_vx),
                                                  ("俯仰阶跃 (deg)", self.target_pitch))):
            ttk.Label(targets, text=label).grid(row=row, column=0, sticky="w")
            ttk.Entry(targets, textvariable=variable, width=8).grid(row=row, column=1, sticky="e")
        ttk.Button(side, text="应用实验目标", command=self._apply_targets).pack(fill="x", pady=4)
        self.motor_tau = tk.StringVar(value=f"{self.device.engine.plant.thrust_tau_s:.3f}")
        ttk.Label(side, text="电机时间常数 tau (s)").pack(anchor="w", pady=(8, 2))
        ttk.Entry(side, textvariable=self.motor_tau).pack(fill="x")
        ttk.Button(side, text="应用模型假设", command=self._apply_model).pack(fill="x", pady=4)
        ttk.Label(side, text="慢放").pack(anchor="w", pady=(14, 2))
        self.scale = tk.DoubleVar(value=1.0)
        ttk.Scale(side, from_=0.1, to=2.0, variable=self.scale,
                  command=lambda value: setattr(self.device, "time_scale", float(value))).pack(fill="x")
        self._baseline_params = None
        ttk.Button(side, text="保存 A 参数快照", command=self._save_a).pack(fill="x", pady=(18, 4))
        ttk.Button(side, text="运行 B 并保存", command=self._run_ab).pack(fill="x", pady=4)
        self.ab_status = tk.StringVar(value="A/B 空闲")
        ttk.Label(side, textvariable=self.ab_status, wraplength=250).pack(anchor="w")
        self.ab_canvas = tk.Canvas(side, width=340, height=300, background="#0b1117",
                                   highlightthickness=0)
        self.ab_canvas.pack(fill="both", expand=True, pady=6)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

    def _toggle(self) -> None:
        self.device.engine.running = not self.device.engine.running

    def _reset(self) -> None:
        self._trajectory.clear()
        self._last_trajectory_time = -1.0
        self.device.reset()

    def _apply_targets(self) -> None:
        try:
            self.device.engine.set_targets(float(self.target_x.get()), float(self.target_vx.get()),
                                           float(self.target_pitch.get()))
        except ValueError:
            self.status.set("实验目标无效")

    def _apply_model(self) -> None:
        try:
            self.device.engine.set_model_assumptions(float(self.motor_tau.get()))
        except ValueError:
            self.status.set("模型参数无效")

    def _save_a(self) -> None:
        if self._ab_busy:
            self.ab_status.set("请等待本次 A/B 结束再更新 A")
            return
        self._baseline_params = self.device.engine.bridge.parameter_snapshot()
        targets = self.device.engine.targets
        self._baseline_config = {
            "kind": self.device.engine.kind,
            "targets": targets,
            "model": self.device.engine.bridge.physical_model(),
        }
        self._baseline_config["model"]["thrust_tau_s"] = self.device.engine.plant.thrust_tau_s
        self.ab_status.set("A 已保存；请从地面站调参后运行 B")

    def _run_ab(self) -> None:
        if self._ab_thread is not None and self._ab_thread.is_alive():
            self.ab_status.set("A/B 正在运行")
            return
        if self._baseline_params is None:
            self.ab_status.set("请先保存 A 快照")
            return
        kind = ExperimentKind(self.kind.get())
        targets = self.device.engine.targets
        current_model = self.device.engine.bridge.physical_model()
        current_model["thrust_tau_s"] = self.device.engine.plant.thrust_tau_s
        if (self._baseline_config["kind"] is not kind or
                self._baseline_config["targets"] != targets or
                self._baseline_config["model"] != current_model):
            self.ab_status.set("实验类型/目标/模型已变更，请重新保存 A")
            return
        # Freeze the complete job before handing it to another thread.
        baseline_params = dict(self._baseline_params)
        tuned_params = self.device.engine.bridge.parameter_snapshot()
        thrust_tau_s = self.device.engine.plant.thrust_tau_s
        self._ab_busy = True
        self.ab_status.set("A/B 运行中...")
        def work() -> None:
            try:
                result = run_ab(self.device.engine.bridge, kind,
                                baseline_params=baseline_params,
                                tuned_params=tuned_params, targets=targets,
                                thrust_tau_s=thrust_tau_s,
                                duration_s=3.0)
                if self._closing:
                    return
                paths = write_ab_artifact(result, SIMULATION_DIR)
                self._ui_queue.put((result, f"A/B 已保存: {paths[0].name}"))
            except Exception as exc:
                if not self._closing:
                    self._ui_queue.put(("error", f"A/B 失败: {exc}"))
        self._ab_thread = threading.Thread(target=work, name="sim-xz-ab", daemon=True)
        self._ab_thread.start()

    def _render(self) -> None:
        try:
            item = self._ui_queue.get_nowait()
            self._ab_busy = False
            if isinstance(item, tuple):
                if item and item[0] == "error":
                    self.ab_status.set(str(item[1]))
                else:
                    self._ab_result, status = item
                    self.ab_status.set(status)
            else:
                self.ab_status.set(str(item))
        except queue.Empty:
            pass
        state = self.device.engine.snapshot()
        self.status.set(f"{'已连接' if self.device.connected else '等待连接'}  t={state.time_s:.2f}s  x={state.x_m:.2f}m  z={state.z_m:.2f}m")
        self.canvas.delete("all")
        width, height = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        ground, scale = height - 70, min(width / 4.0, (height - 100) / 2.5)
        x, z = width / 2 + state.x_m * scale, ground - state.z_m * scale
        if state.time_s != self._last_trajectory_time:
            self._trajectory.append((state.x_m, state.z_m))
            self._last_trajectory_time = state.time_s
        self.canvas.create_line(20, ground, width - 20, ground, fill="#7f8c8d")
        self.canvas.create_line(width / 2, 20, width / 2, ground, fill="#273746")
        if len(self._trajectory) > 1:
            trail = []
            for tx, tz in self._trajectory:
                trail.extend((width / 2 + tx * scale, ground - tz * scale))
            self.canvas.create_line(*trail, fill="#f4d03f", width=2)
        target_x = self.device.engine.targets.position_step_m if self.device.engine.kind is ExperimentKind.POSITION_STEP else state.x_m
        target_z = 1.0
        target_px = width / 2 + target_x * scale
        target_pz = ground - target_z * scale
        self.canvas.create_oval(target_px - 7, target_pz - 7, target_px + 7, target_pz + 7,
                                outline="#58d68d", dash=(4, 3))
        self.canvas.create_line(x, z, target_px, target_pz, fill="#58d68d", dash=(3, 3))
        nose = (x + 55 * math.cos(state.pitch_rad), z + 55 * math.sin(state.pitch_rad))
        tail = (x - 55 * math.cos(state.pitch_rad), z - 55 * math.sin(state.pitch_rad))
        self.canvas.create_line(*nose, *tail, fill="#f5b041", width=8)
        thrust_scale = min(90.0, max(10.0, state.thrust_n * 4.0))
        thrust_angle = state.pitch_rad - state.pitch_tilt_rad
        force_x = math.sin(thrust_angle)
        force_z = math.cos(thrust_angle)
        self.canvas.create_line(x, z, x + thrust_scale * force_x,
                                z - thrust_scale * force_z,
                                fill="#5dade2", width=3, arrow=tk.LAST)
        servo_dx = 22 * math.sin(state.pitch_tilt_rad)
        servo_dy = -22 * math.cos(state.pitch_tilt_rad)
        body_c, body_s = math.cos(state.pitch_rad), math.sin(state.pitch_rad)
        for local_x in (-25, 25):
            px = x + local_x * body_c
            py = z + local_x * body_s
            dx = servo_dx * body_c - servo_dy * body_s
            dy = servo_dx * body_s + servo_dy * body_c
            self.canvas.create_line(px, py, px + dx, py + dy, fill="#af7ac5", width=3)
        self.canvas.create_oval(x - 8, z - 8, x + 8, z + 8, fill="#ecf0f1", outline="")
        self._draw_ab_plot()
        self.after(33, self._render)

    def _draw_ab_plot(self) -> None:
        canvas = self.ab_canvas
        canvas.delete("all")
        canvas.create_text(8, 6, anchor="nw", text="A/B 五量对比",
                           fill="#ecf0f1")
        canvas.create_line(120, 10, 145, 10, fill="#ecf0f1", width=2)
        canvas.create_text(150, 6, anchor="nw", text="A 实线", fill="#ecf0f1")
        canvas.create_line(205, 10, 230, 10, fill="#ecf0f1", width=2, dash=(3, 2))
        canvas.create_text(235, 6, anchor="nw", text="B 虚线", fill="#ecf0f1")
        result = self._ab_result
        if result is None:
            return
        names = (("pitch_rad", "俯仰", "rad"),
                 ("pitch_rate_rad_s", "俯仰速率", "rad/s"),
                 ("vx_m_s", "X速度", "m/s"),
                 ("x_m", "X位置", "m"),
                 ("z_m", "Z高度", "m"))
        colors = ("#f5b041", "#ec7063", "#5dade2", "#58d68d", "#af7ac5")
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        plot_top, row_height = 32, max(32.0, (height - 38) / 5.0)
        for row, ((name, label, unit), color) in enumerate(zip(names, colors)):
            values = [getattr(sample, name) for sample in result.baseline + result.tuned]
            lo, hi = min(values), max(values)
            span = max(hi - lo, 1.0e-6)
            y0 = plot_top + row * row_height + 12
            y1 = plot_top + (row + 1) * row_height - 4
            canvas.create_text(6, y0, anchor="w", text=f"{label} [{unit}]", fill=color)
            for samples, dash in ((result.baseline, ()), (result.tuned, (3, 2))):
                points = []
                for index, sample in enumerate(samples):
                    px = 110 + index * (width - 116) / max(1, len(samples) - 1)
                    py = y1 - (getattr(sample, name) - lo) / span * (y1 - y0)
                    points.extend((px, py))
                if len(points) >= 4:
                    canvas.create_line(*points, fill=color, dash=dash)

    def _close(self) -> None:
        self._closing = True
        self.device.stop()
        self.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="R-SIM-1 X/Z teaching simulator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6666)
    args = parser.parse_args()
    SimulationApp(SimulatorDevice(host=args.host, port=args.port)).mainloop()


if __name__ == "__main__":
    main()
