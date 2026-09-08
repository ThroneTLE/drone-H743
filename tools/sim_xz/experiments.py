"""Approved R-SIM-1 experiments and A/B result production."""

from __future__ import annotations

import csv
import json
import math
import threading
import uuid
from dataclasses import dataclass, astuple, asdict
from datetime import date, datetime
from enum import Enum
from pathlib import Path

from .controller_bridge import ControllerBridge
from .physics import SimulationState, XZPlant


class ExperimentKind(str, Enum):
    POSITION_STEP = "position_step"
    VELOCITY_STEP = "velocity_step"
    PITCH_STEP = "pitch_step"


@dataclass(frozen=True)
class SimulationSample:
    time_s: float
    x_m: float
    z_m: float
    vx_m_s: float
    vz_m_s: float
    pitch_rad: float
    pitch_rate_rad_s: float

    @classmethod
    def from_state(cls, state: SimulationState) -> "SimulationSample":
        return cls(state.time_s, state.x_m, state.z_m, state.vx_m_s, state.vz_m_s,
                   state.pitch_rad, state.pitch_rate_rad_s)


@dataclass(frozen=True)
class ExperimentTargets:
    position_step_m: float = 0.5
    velocity_step_m_s: float = 0.2
    pitch_step_rad: float = math.radians(3.0)


def _reference(kind: ExperimentKind, state: SimulationState,
               targets: ExperimentTargets | None = None) -> dict[str, float | int]:
    targets = targets or ExperimentTargets()
    active = state.time_s >= 1.0
    if kind is ExperimentKind.POSITION_STEP:
        return {"target_x_m": targets.position_step_m if active else 0.0, "target_z_m": 1.0}
    if kind is ExperimentKind.VELOCITY_STEP:
        return {"target_x_m": state.x_m, "target_z_m": 1.0,
                "direct_velocity_x_m_s": targets.velocity_step_m_s if active else 0.0}
    return {"target_x_m": state.x_m, "target_z_m": 1.0,
            "target_pitch_rad": targets.pitch_step_rad if active else 0.0,
            "direct_attitude_target_valid": 1}


def run_experiment(bridge: ControllerBridge, kind: ExperimentKind,
                   duration_s: float = 4.0, dt_s: float = 0.001,
                   plant: XZPlant | None = None,
                   reset_controller: bool = True,
                   targets: ExperimentTargets | None = None) -> list[SimulationSample]:
    if not all(math.isfinite(v) and v > 0.0 for v in (duration_s, dt_s)):
        raise ValueError("duration_s and dt_s must be positive")
    plant = plant or XZPlant.from_bridge(bridge)
    if reset_controller:
        bridge.reset()
    plant.reset(z_m=1.0, thrust_n=plant.hover_thrust_n)
    samples: list[SimulationSample] = []
    for index in range(int(round(duration_s / dt_s))):
        state = plant.state
        step_kwargs: dict[str, float | int] = {
            "x_m": state.x_m, "z_m": state.z_m, "vx_m_s": state.vx_m_s,
            "vz_m_s": state.vz_m_s, "pitch_rad": state.pitch_rad,
            "pitch_rate_rad_s": state.pitch_rate_rad_s, "dt_s": dt_s,
            "accel_x_m_s2": state.accel_x_m_s2, "accel_z_m_s2": state.accel_z_m_s2,
            "acceleration_valid": 1, "integrator_reset": 1 if index == 0 else 0,
        }
        if kind is ExperimentKind.PITCH_STEP:
            step_kwargs.update({"manual_total_force_valid": 1,
                                "manual_total_force_n": plant.hover_thrust_n})
        command = bridge.step(
            **step_kwargs, **_reference(kind, state, targets))
        samples.append(SimulationSample.from_state(plant.step(command, dt_s)))
    return samples


@dataclass(frozen=True)
class ABResult:
    kind: ExperimentKind
    parameter: str
    baseline_value: float
    tuned_value: float
    baseline_params: dict[str, float]
    tuned_params: dict[str, float]
    baseline: tuple[SimulationSample, ...]
    tuned: tuple[SimulationSample, ...]
    targets: ExperimentTargets
    model: dict[str, float]
    dt_s: float

    @property
    def peak_abs_delta(self) -> dict[str, float]:
        names = ("pitch_rad", "pitch_rate_rad_s", "vx_m_s", "x_m", "z_m")
        return {name: max(abs(getattr(a, name) - getattr(b, name))
                          for a, b in zip(self.baseline, self.tuned)) for name in names}


def run_ab(bridge: ControllerBridge, kind: ExperimentKind,
           parameter: str | None = None,
           tuned_params: dict[str, float] | None = None,
           baseline_params: dict[str, float] | None = None,
           targets: ExperimentTargets | None = None,
           thrust_tau_s: float = 0.08,
           duration_s: float = 4.0, dt_s: float = 0.001) -> ABResult:
    parameter = parameter or {
        ExperimentKind.POSITION_STEP: "coax.pos_x_kp",
        ExperimentKind.VELOCITY_STEP: "coax.vel_x_kp",
        ExperimentKind.PITCH_STEP: "coax.att_pitch_kp",
    }[kind]
    baseline_params = dict(baseline_params or bridge.parameter_snapshot())
    tuned_params = dict(tuned_params or baseline_params)
    run_id = uuid.uuid4().hex
    baseline_bridge = bridge.clone(f"ab_baseline_{run_id}")
    tuned_bridge = bridge.clone(f"ab_tuned_{run_id}")
    for name, value in baseline_params.items():
        baseline_bridge.set_param(name, value)
    for name, value in tuned_params.items():
        tuned_bridge.set_param(name, value)
    baseline_value = baseline_params.get(parameter)
    if baseline_value is None:
        raise KeyError(parameter)
    baseline_plant = XZPlant.from_bridge(baseline_bridge)
    tuned_plant = XZPlant.from_bridge(tuned_bridge)
    baseline_plant.thrust_tau_s = thrust_tau_s
    tuned_plant.thrust_tau_s = thrust_tau_s
    baseline = tuple(run_experiment(baseline_bridge, kind, duration_s, dt_s,
                                    plant=baseline_plant, targets=targets))
    tuned_value = tuned_params.get(parameter, baseline_value)
    tuned = tuple(run_experiment(tuned_bridge, kind, duration_s, dt_s,
                                 plant=tuned_plant, targets=targets))
    return ABResult(kind, parameter, baseline_value, tuned_value,
                    baseline_params, tuned_params, baseline, tuned,
                    targets or ExperimentTargets(),
                    {**baseline_bridge.physical_model(), "thrust_tau_s": thrust_tau_s,
                     "linear_drag": baseline_plant.linear_drag, "pitch_damping": baseline_plant.pitch_damping}, dt_s)


class SimulationEngine:
    """Fixed-step engine used by the TCP device and Tk snapshot view."""

    def __init__(self, bridge: ControllerBridge | None = None, dt_s: float = 0.001) -> None:
        self.bridge = bridge or ControllerBridge()
        self.plant = XZPlant.from_bridge(self.bridge)
        self.plant.reset(z_m=1.0, thrust_n=self.plant.hover_thrust_n)
        self.dt_s = dt_s
        self.kind = ExperimentKind.POSITION_STEP
        self.targets = ExperimentTargets()
        self.running = False
        self.pause_reason = ""
        self._lock = threading.RLock()

    def set_kind(self, kind: ExperimentKind) -> None:
        with self._lock:
            self.kind = kind

    def set_targets(self, position_step_m: float, velocity_step_m_s: float,
                    pitch_step_deg: float) -> None:
        if not all(math.isfinite(v) and v > 0.0
                   for v in (position_step_m, velocity_step_m_s, pitch_step_deg)):
            raise ValueError("experiment targets must be positive")
        with self._lock:
            self.targets = ExperimentTargets(position_step_m, velocity_step_m_s,
                                             math.radians(pitch_step_deg))

    def set_model_assumptions(self, thrust_tau_s: float) -> None:
        if thrust_tau_s <= 0.0 or not math.isfinite(thrust_tau_s):
            raise ValueError("motor time constant must be positive")
        with self._lock:
            self.plant.thrust_tau_s = thrust_tau_s

    def reset(self) -> None:
        with self._lock:
            self.pause_reason = ""
            self.running = False
            self.bridge.reset()
            self.plant.reset(z_m=1.0, thrust_n=self.plant.hover_thrust_n)

    def step(self) -> SimulationState:
        with self._lock:
            if not self.running:
                return self.plant.state
            state = self.plant.state
            if not all(math.isfinite(v) for v in astuple(state)):
                self.running = False
                self.pause_reason = "状态含非有限值，请复位"
                return state
            step_kwargs: dict[str, float | int] = {
                "x_m": state.x_m, "z_m": state.z_m, "vx_m_s": state.vx_m_s,
                "vz_m_s": state.vz_m_s, "pitch_rad": state.pitch_rad,
                "pitch_rate_rad_s": state.pitch_rate_rad_s, "dt_s": self.dt_s,
                "accel_x_m_s2": state.accel_x_m_s2, "accel_z_m_s2": state.accel_z_m_s2,
                "acceleration_valid": 1, "integrator_reset": 1 if state.time_s == 0.0 else 0,
            }
            if self.kind is ExperimentKind.PITCH_STEP:
                step_kwargs.update({"manual_total_force_valid": 1,
                                    "manual_total_force_n": self.plant.hover_thrust_n})
            command = self.bridge.step(**step_kwargs,
                                       **_reference(self.kind, state, self.targets))
            next_state = self.plant.step(command, self.dt_s)
            if not all(math.isfinite(v) for v in astuple(next_state)):
                self.running = False
                self.pause_reason = "计算产生非有限值，请复位"
                self.plant.state = state
                return state
            if (next_state.z_m <= 0.0 or next_state.z_m >= 3.0 or
                    abs(next_state.x_m) >= 10.0 or abs(next_state.pitch_rad) >= math.pi / 2.0):
                self.running = False
                self.pause_reason = "已到教学场景边界，请复位或减小目标"
            return next_state

    def snapshot(self) -> SimulationState:
        with self._lock:
            return self.plant.state


def write_ab_artifact(result: ABResult, root: Path) -> tuple[Path, Path]:
    run_dir = root / date.today().isoformat()
    run_dir.mkdir(parents=True, exist_ok=True)
    stem = result.kind.value
    run_id = f"{datetime.now().strftime('%H%M%S_%f')}_{uuid.uuid4().hex[:8]}"
    json_path = run_dir / f"{stem}_ab_{run_id}.json"
    csv_path = run_dir / f"{stem}_ab_{run_id}.csv"
    json_path.write_text(json.dumps({"simulation": True, "kind": stem,
        "parameter": result.parameter, "baseline_value": result.baseline_value,
        "tuned_value": result.tuned_value, "baseline_params": result.baseline_params,
        "tuned_params": result.tuned_params, "peak_abs_delta": result.peak_abs_delta,
        "targets": asdict(result.targets), "model": result.model, "dt_s": result.dt_s},
        indent=2) + "\n", encoding="utf-8")
    with csv_path.open("w", newline="", encoding="ascii") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "baseline_x_m", "tuned_x_m", "baseline_z_m", "tuned_z_m",
                         "baseline_vx_m_s", "tuned_vx_m_s", "baseline_pitch_rad", "tuned_pitch_rad",
                         "baseline_pitch_rate_rad_s", "tuned_pitch_rate_rad_s"])
        for base, tuned in zip(result.baseline, result.tuned):
            writer.writerow([base.time_s, base.x_m, tuned.x_m, base.z_m, tuned.z_m,
                             base.vx_m_s, tuned.vx_m_s, base.pitch_rad, tuned.pitch_rad,
                             base.pitch_rate_rad_s, tuned.pitch_rate_rad_s])
    return json_path, csv_path
