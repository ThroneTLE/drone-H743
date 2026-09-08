"""Tk front end for the host-only simulator."""

from __future__ import annotations

import math
import queue
import threading
import tkinter as tk
from tkinter import ttk

from tools.project_paths import SIMULATION_DIR

from .device import SimulatorDevice
from .experiments import ExperimentKind, ExperimentTargets, run_ab, write_ab_artifact


class SimulationApp(tk.Tk):
    def __init__(self, device: SimulatorDevice | None = None) -> None:
        super().__init__()
        self.title("R-SIM-1 X/Z Teaching Simulator")
        self.geometry("980x640")
        self.device = device or SimulatorDevice()
        self._ui_queue: queue.Queue[object] = queue.Queue()
        self._ab_result = None
        self._baseline_config = None
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
        self.status = tk.StringVar(value="waiting for ground station")
        ttk.Label(side, textvariable=self.status).pack(anchor="w")
        ttk.Button(side, text="Start / pause", command=self._toggle).pack(fill="x", pady=4)
        ttk.Button(side, text="Reset", command=self.device.reset).pack(fill="x", pady=4)
        ttk.Label(side, text="Experiment").pack(anchor="w", pady=(14, 2))
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
        for row, (label, variable) in enumerate((("step x (m)", self.target_x),
                                                  ("step vx (m/s)", self.target_vx),
                                                  ("step pitch (deg)", self.target_pitch))):
            ttk.Label(targets, text=label).grid(row=row, column=0, sticky="w")
            ttk.Entry(targets, textvariable=variable, width=8).grid(row=row, column=1, sticky="e")
        ttk.Button(side, text="Apply experiment targets", command=self._apply_targets).pack(fill="x", pady=4)
        ttk.Label(side, text="slow motion").pack(anchor="w", pady=(14, 2))
        self.scale = tk.DoubleVar(value=1.0)
        ttk.Scale(side, from_=0.1, to=2.0, variable=self.scale,
                  command=lambda value: setattr(self.device, "time_scale", float(value))).pack(fill="x")
        self._baseline_params = None
        ttk.Button(side, text="Save A parameter snapshot", command=self._save_a).pack(fill="x", pady=(18, 4))
        ttk.Button(side, text="Run B vs saved A", command=self._run_ab).pack(fill="x", pady=4)
        self.ab_status = tk.StringVar(value="A/B idle")
        ttk.Label(side, textvariable=self.ab_status, wraplength=250).pack(anchor="w")
        self.ab_canvas = tk.Canvas(side, width=280, height=180, background="#0b1117",
                                   highlightthickness=0)
        self.ab_canvas.pack(fill="x", pady=6)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

    def _toggle(self) -> None:
        self.device.engine.running = not self.device.engine.running

    def _apply_targets(self) -> None:
        try:
            self.device.engine.set_targets(float(self.target_x.get()), float(self.target_vx.get()),
                                           float(self.target_pitch.get()))
        except ValueError:
            self.status.set("invalid experiment target")

    def _save_a(self) -> None:
        self._baseline_params = self.device.engine.bridge.parameter_snapshot()
        targets = self.device.engine.targets
        self._baseline_config = {
            "kind": self.device.engine.kind,
            "targets": targets,
            "model": self.device.engine.bridge.physical_model(),
        }
        self.ab_status.set("A snapshot saved; tune parameters from the ground station, then run B")

    def _run_ab(self) -> None:
        if self._ab_thread is not None and self._ab_thread.is_alive():
            self.ab_status.set("A/B already running")
            return
        if self._baseline_params is None:
            self.ab_status.set("save A snapshot first")
            return
        kind = ExperimentKind(self.kind.get())
        targets = self.device.engine.targets
        if (self._baseline_config["kind"] is not kind or
                self._baseline_config["targets"] != targets or
                self._baseline_config["model"] != self.device.engine.bridge.physical_model()):
            self.ab_status.set("experiment kind/targets changed; save A again")
            return
        self.ab_status.set("A/B running...")
        def work() -> None:
            tuned_params = self.device.engine.bridge.parameter_snapshot()
            result = run_ab(self.device.engine.bridge, kind,
                            baseline_params=self._baseline_params,
                            tuned_params=tuned_params, targets=targets, duration_s=3.0)
            paths = write_ab_artifact(result, SIMULATION_DIR)
            self._ui_queue.put((result, f"A/B saved: {paths[0].name}; {result.peak_abs_delta}"))
        self._ab_thread = threading.Thread(target=work, name="sim-xz-ab", daemon=True)
        self._ab_thread.start()

    def _render(self) -> None:
        try:
            item = self._ui_queue.get_nowait()
            if isinstance(item, tuple):
                self._ab_result, status = item
                self.ab_status.set(status)
            else:
                self.ab_status.set(str(item))
        except queue.Empty:
            pass
        state = self.device.engine.snapshot()
        self.status.set(f"{'connected' if self.device.connected else 'waiting'}  t={state.time_s:.2f}s  x={state.x_m:.2f}m  z={state.z_m:.2f}m")
        self.canvas.delete("all")
        width, height = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        ground, scale = height - 70, min(width / 4.0, (height - 100) / 2.5)
        x, z = width / 2 + state.x_m * scale, ground - state.z_m * scale
        self.canvas.create_line(20, ground, width - 20, ground, fill="#7f8c8d")
        self.canvas.create_line(width / 2, 20, width / 2, ground, fill="#273746")
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
        thrust_angle = state.pitch_rad + state.pitch_tilt_rad
        self.canvas.create_line(x, z, x - thrust_scale * math.sin(thrust_angle),
                                z + thrust_scale * math.cos(thrust_angle),
                                fill="#5dade2", width=3, arrow=tk.LAST)
        self.canvas.create_line(x - 25, z, x - 25 + 22 * math.sin(state.pitch_tilt_rad),
                                z - 22 * math.cos(state.pitch_tilt_rad), fill="#af7ac5", width=3)
        self.canvas.create_line(x + 25, z, x + 25 + 22 * math.sin(state.pitch_tilt_rad),
                                z - 22 * math.cos(state.pitch_tilt_rad), fill="#af7ac5", width=3)
        self.canvas.create_oval(x - 8, z - 8, x + 8, z + 8, fill="#ecf0f1", outline="")
        self._draw_ab_plot()
        self.after(33, self._render)

    def _draw_ab_plot(self) -> None:
        canvas = self.ab_canvas
        canvas.delete("all")
        canvas.create_text(6, 6, anchor="nw", text="A/B: pitch, pitch_rate, vx, x, z",
                           fill="#ecf0f1")
        result = self._ab_result
        if result is None:
            return
        names = ("pitch_rad", "pitch_rate_rad_s", "vx_m_s", "x_m", "z_m")
        colors = ("#f5b041", "#ec7063", "#5dade2", "#58d68d", "#af7ac5")
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        for row, (name, color) in enumerate(zip(names, colors)):
            values = [getattr(sample, name) for sample in result.baseline + result.tuned]
            lo, hi = min(values), max(values)
            span = max(hi - lo, 1.0e-6)
            y0, y1 = 24 + row * 30, 46 + row * 30
            for samples, dash in ((result.baseline, ()), (result.tuned, (3, 2))):
                points = []
                for index, sample in enumerate(samples):
                    px = 6 + index * (width - 12) / max(1, len(samples) - 1)
                    py = y1 - (getattr(sample, name) - lo) / span * (y1 - y0)
                    points.extend((px, py))
                if len(points) >= 4:
                    canvas.create_line(*points, fill=color, dash=dash)

    def _close(self) -> None:
        self.device.stop()
        self.destroy()


def main() -> None:
    SimulationApp().mainloop()


if __name__ == "__main__":
    main()
