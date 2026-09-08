"""ctypes bridge to the real pure-C cascade and allocator."""

from __future__ import annotations

import ctypes
import hashlib
import os
import shutil
import subprocess
import tempfile
import threading
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = Path(__file__).resolve().parent


class _CInput(ctypes.Structure):
    _fields_ = [
        ("x_m", ctypes.c_float), ("z_m", ctypes.c_float),
        ("vx_m_s", ctypes.c_float), ("vz_m_s", ctypes.c_float),
        ("pitch_rad", ctypes.c_float), ("pitch_rate_rad_s", ctypes.c_float),
        ("target_x_m", ctypes.c_float), ("target_z_m", ctypes.c_float),
        ("direct_velocity_x_m_s", ctypes.c_float),
        ("target_pitch_rad", ctypes.c_float),
        ("accel_m_s2", ctypes.c_float * 3),
        ("manual_total_force_n", ctypes.c_float), ("dt_s", ctypes.c_float),
        ("position_control_bypass", ctypes.c_uint8),
        ("direct_attitude_target_valid", ctypes.c_uint8),
        ("integrator_reset", ctypes.c_uint8), ("acceleration_valid", ctypes.c_uint8),
        ("manual_total_force_valid", ctypes.c_uint8),
    ]


class _COutput(ctypes.Structure):
    _fields_ = [
        ("thrust_upper_n", ctypes.c_float), ("thrust_lower_n", ctypes.c_float),
        ("pitch_tilt_rad", ctypes.c_float), ("roll_tilt_rad", ctypes.c_float),
        ("moment_achieved_n_m", ctypes.c_float * 3),
        ("force_cmd_n", ctypes.c_float * 3),
        ("velocity_sp_m_s", ctypes.c_float * 3),
        ("omega_sp_rad_s", ctypes.c_float * 3),
    ]


@dataclass(frozen=True)
class ControllerOutput:
    thrust_upper_n: float
    thrust_lower_n: float
    pitch_tilt_rad: float
    roll_tilt_rad: float
    moment_achieved_n_m: tuple[float, float, float]
    force_cmd_n: tuple[float, float, float]
    velocity_sp_m_s: tuple[float, float, float]
    omega_sp_rad_s: tuple[float, float, float]

    @property
    def total_thrust_n(self) -> float:
        return self.thrust_upper_n + self.thrust_lower_n


def _source_digest() -> str:
    paths = [SIM_ROOT / "sim_controller_bridge.c", SIM_ROOT / "sim_controller_bridge.h",
             ROOT / "Driver" / "Inc" / "drv_coax_ctrl.h",
             ROOT / "Driver" / "Src" / "drv_coax_ctrl.c",
             ROOT / "Driver" / "Src" / "drv_position_control.c",
             ROOT / "Driver" / "Src" / "drv_attitude_control.c",
             ROOT / "Driver" / "Src" / "drv_rate_control.c"]
    digest = hashlib.sha256()
    for path in paths:
        digest.update(path.read_bytes())
    return digest.hexdigest()[:16]


def build_controller_library(instance_tag: str = "default") -> Path:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        raise RuntimeError("R-SIM-1 requires host gcc or clang")
    output_dir = Path(tempfile.gettempdir()) / "drone_h743_sim_xz"
    output_dir.mkdir(parents=True, exist_ok=True)
    suffix = ".dll" if os.name == "nt" else ".so"
    safe_tag = "".join(char if char.isalnum() else "_" for char in instance_tag)
    output = output_dir / f"controller_{_source_digest()}_{safe_tag}{suffix}"
    if output.exists():
        return output
    command = [compiler, "-shared", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
               "-I", str(ROOT / "Driver" / "Inc"), "-I", str(ROOT / "BSP" / "Inc"),
               "-I", str(SIM_ROOT), "-I", str(ROOT / "App" / "Inc"),
               str(SIM_ROOT / "sim_controller_bridge.c"),
               str(ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"),
               str(ROOT / "Driver" / "Src" / "drv_position_control.c"),
               str(ROOT / "Driver" / "Src" / "drv_attitude_control.c"),
               str(ROOT / "Driver" / "Src" / "drv_rate_control.c"), "-lm", "-o", str(output)]
    command.insert(-2, str(ROOT / "App" / "Src" / "app_control_scheduler.c"))
    subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    return output


class ControllerBridge:
    """Thread-safe, single-instance view of the real C controller."""

    def __init__(self, library: Path | None = None, instance_tag: str = "default") -> None:
        self._lock = threading.RLock()
        self._lib = ctypes.CDLL(str(library or build_controller_library(instance_tag)))
        self._lib.sim_controller_reset.argtypes = []
        self._lib.sim_controller_reset_params.argtypes = []
        for name in ("sim_controller_mass_kg", "sim_controller_gravity_m_s2",
                     "sim_controller_pitch_inertia_kgm2", "sim_controller_pitch_lever_arm_m",
                     "sim_controller_tilt_tau_s"):
            getattr(self._lib, name).restype = ctypes.c_float
        self._lib.sim_controller_param_count.restype = ctypes.c_uint32
        self._lib.sim_controller_param_name.argtypes = [ctypes.c_uint32]
        self._lib.sim_controller_param_name.restype = ctypes.c_char_p
        self._lib.sim_controller_get_param.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_float)]
        self._lib.sim_controller_get_param.restype = ctypes.c_uint8
        self._lib.sim_controller_set_param.argtypes = [ctypes.c_char_p, ctypes.c_float]
        self._lib.sim_controller_set_param.restype = ctypes.c_uint8
        self._lib.sim_controller_step.argtypes = [ctypes.POINTER(_CInput), ctypes.POINTER(_COutput)]
        self._lib.sim_controller_reset()

    def reset_params(self) -> None:
        with self._lock:
            self._lib.sim_controller_reset_params()

    def physical_model(self) -> dict[str, float]:
        with self._lock:
            return {"mass_kg": float(self._lib.sim_controller_mass_kg()),
                    "gravity_m_s2": float(self._lib.sim_controller_gravity_m_s2()),
                    "pitch_inertia_kgm2": float(self._lib.sim_controller_pitch_inertia_kgm2()),
                    "pitch_lever_arm_m": float(self._lib.sim_controller_pitch_lever_arm_m()),
                    "tilt_tau_s": float(self._lib.sim_controller_tilt_tau_s())}

    def parameter_snapshot(self) -> dict[str, float]:
        with self._lock:
            count = int(self._lib.sim_controller_param_count())
            snapshot: dict[str, float] = {}
            for index in range(count):
                name = self._lib.sim_controller_param_name(index).decode("ascii")
                value = ctypes.c_float()
                if self._lib.sim_controller_get_param(name.encode("ascii"), ctypes.byref(value)):
                    snapshot[name] = float(value.value)
            return snapshot

    def clone(self, instance_tag: str) -> "ControllerBridge":
        clone = ControllerBridge(instance_tag=instance_tag)
        for name, value in self.parameter_snapshot().items():
            clone.set_param(name, value)
        return clone

    def reset(self) -> None:
        with self._lock:
            self._lib.sim_controller_reset()

    def parameter_names(self) -> tuple[str, ...]:
        with self._lock:
            count = int(self._lib.sim_controller_param_count())
            return tuple(self._lib.sim_controller_param_name(i).decode("ascii") for i in range(count))

    def get_param(self, name: str) -> float | None:
        value = ctypes.c_float()
        with self._lock:
            ok = self._lib.sim_controller_get_param(name.encode("ascii"), ctypes.byref(value))
        return float(value.value) if ok else None

    def set_param(self, name: str, value: float) -> bool:
        with self._lock:
            return bool(self._lib.sim_controller_set_param(name.encode("ascii"), ctypes.c_float(value)))

    def step(self, **kwargs: float | int) -> ControllerOutput:
        value = _CInput(
            x_m=float(kwargs.get("x_m", 0.0)), z_m=float(kwargs.get("z_m", 1.0)),
            vx_m_s=float(kwargs.get("vx_m_s", 0.0)), vz_m_s=float(kwargs.get("vz_m_s", 0.0)),
            pitch_rad=float(kwargs.get("pitch_rad", 0.0)),
            pitch_rate_rad_s=float(kwargs.get("pitch_rate_rad_s", 0.0)),
            target_x_m=float(kwargs.get("target_x_m", 0.0)),
            target_z_m=float(kwargs.get("target_z_m", 1.0)),
            direct_velocity_x_m_s=float(kwargs.get("direct_velocity_x_m_s", 0.0)),
            target_pitch_rad=float(kwargs.get("target_pitch_rad", 0.0)),
            accel_m_s2=(float(kwargs.get("accel_x_m_s2", 0.0)),
                        float(kwargs.get("accel_y_m_s2", 0.0)),
                        float(kwargs.get("accel_z_m_s2", 0.0))),
            manual_total_force_n=float(kwargs.get("manual_total_force_n", 0.0)),
            dt_s=float(kwargs.get("dt_s", 0.002)),
            position_control_bypass=int(kwargs.get("position_control_bypass", 0)),
            direct_attitude_target_valid=int(kwargs.get("direct_attitude_target_valid", 0)),
            integrator_reset=int(kwargs.get("integrator_reset", 0)),
            acceleration_valid=int(kwargs.get("acceleration_valid", 0)),
            manual_total_force_valid=int(kwargs.get("manual_total_force_valid", 0)),
        )
        output = _COutput()
        with self._lock:
            self._lib.sim_controller_step(ctypes.byref(value), ctypes.byref(output))
        return ControllerOutput(
            float(output.thrust_upper_n), float(output.thrust_lower_n),
            float(output.pitch_tilt_rad), float(output.roll_tilt_rad),
            tuple(float(v) for v in output.moment_achieved_n_m),
            tuple(float(v) for v in output.force_cmd_n),
            tuple(float(v) for v in output.velocity_sp_m_s),
            tuple(float(v) for v in output.omega_sp_rad_s),
        )
