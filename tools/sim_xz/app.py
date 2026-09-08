"""Tk front end for the host-only simulator."""

from __future__ import annotations

import math
import queue
import threading
import tkinter as tk
import argparse
from collections import deque

from tools.project_paths import SIMULATION_DIR

from .presentation import PresentationMixin
from .device import SimulatorDevice
from .experiments import ExperimentKind, run_ab, write_ab_artifact


class SimulationApp(PresentationMixin, tk.Tk):
    def __init__(self, device: SimulatorDevice | None = None) -> None:
        super().__init__()
        self.title("R-SIM-1 X/Z 教学仿真")
        self.geometry("1320x820")
        self.device = device or SimulatorDevice()
        self._ui_queue: queue.Queue[object] = queue.Queue()
        self._ab_result = None
        self._baseline_config = None
        self._trajectory = deque(maxlen=1200)
        self._history = deque(maxlen=1200)
        self._last_trajectory_time = -1.0
        self._closing = False
        self._ab_busy = False
        self._link_state = None
        self._ab_thread: threading.Thread | None = None
        self.device.start()
        self._build_widgets()
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(33, self._render)

    def _toggle(self) -> None:
        if not self.device.connected:
            self.status.set("尚未连接：请先在上位机开启本机 TCP 监听")
            return
        if self.device.engine.pause_reason:
            self.status.set(self.device.engine.pause_reason)
            return
        self.device.engine.running = not self.device.engine.running

    def _reset(self) -> None:
        self._trajectory.clear()
        self._history.clear()
        self._last_trajectory_time = -1.0
        self.device.reset()

    def _apply_targets(self) -> None:
        try:
            self.device.engine.set_targets(float(self.target_x.get()), float(self.target_vx.get()),
                                           float(self.target_pitch.get()))
        except ValueError:
            self.status.set("目标必须是有限正数，例如 0.5、0.2、3")
        else:
            self.status.set("实验目标已应用；更改后请重新保存 A")

    def _apply_model(self) -> None:
        try:
            self.device.engine.set_model_assumptions(float(self.motor_tau.get()))
        except ValueError:
            self.status.set("模型时间常数必须是有限正数")
        else:
            self.status.set("模型假设已应用；更改后请重新保存 A")

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
        connected = self.device.connected
        if self._link_state is not connected:
            self.status.set(f"已连接 {getattr(self.device, "host", "127.0.0.1")}:{getattr(self.device, "port", 6666)} · 参数由上位机调节" if connected
                            else "等待上位机 TCP 监听；仿真保持暂停")
            self._link_state = connected
        self.metrics.set(f"X {state.x_m:+.2f} m    Z {state.z_m:.2f} m    俯仰 {math.degrees(state.pitch_rad):+.1f}°")
        mode = "运行中" if self.device.engine.running else "暂停"
        link = "TCP 已连接" if self.device.connected else "等待 TCP"
        self.run_status.set(f"{link}  ·  {mode}  ·  {state.time_s:.2f} s")
        reason = self.device.engine.pause_reason
        if reason:
            self.status.set(reason)
        self.save_a_button.state(["disabled"] if self._ab_busy else ["!disabled"])
        self.run_b_button.state(["disabled"] if self._ab_busy else ["!disabled"])
        if all(math.isfinite(v) for v in (state.x_m, state.z_m, state.pitch_rad, state.pitch_rate_rad_s, state.vx_m_s)):
            if state.time_s != self._last_trajectory_time:
                self._trajectory.append((state.x_m, state.z_m))
                self._history.append(state)
                self._last_trajectory_time = state.time_s
            self._draw_scene(state)
        self._draw_ab_plot()
        self.after(33, self._render)

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
