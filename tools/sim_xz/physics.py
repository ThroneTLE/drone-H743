"""Deterministic X/Z plant and actuator model; controller equations stay in C."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .controller_bridge import ControllerOutput
from .actuators import TiltActuator, lag


@dataclass(frozen=True)
class SimulationState:
    time_s: float = 0.0
    x_m: float = 0.0
    z_m: float = 1.0
    vx_m_s: float = 0.0
    vz_m_s: float = 0.0
    pitch_rad: float = 0.0
    pitch_rate_rad_s: float = 0.0
    accel_x_m_s2: float = 0.0
    accel_z_m_s2: float = 0.0
    thrust_n: float = 0.0
    pitch_tilt_rad: float = 0.0
    thrust_upper_n: float = 0.0
    thrust_lower_n: float = 0.0


class XZPlant:
    def __init__(self, mass_kg: float = 1.367, inertia_pitch: float = 0.051,
                 gravity_m_s2: float = 9.81, pitch_lever_arm_m: float = 0.145,
                 tilt_tau_s: float = 0.071236) -> None:
        self.mass_kg = mass_kg
        self.inertia_pitch = inertia_pitch
        self.gravity_m_s2 = gravity_m_s2
        self.pitch_lever_arm_m = pitch_lever_arm_m
        self.pitch_effectiveness = 1.0
        self.thrust_tau_s = 0.08  # teaching assumption; no thrust actuator fit is in the repo
        self.tilt_tau_s = tilt_tau_s  # sourced from the real airframe model via the C bridge
        self.pitch_damping = 0.0  # deliberately no unmeasured damping hidden in the result
        self.linear_drag = 0.0  # deliberately no unmeasured drag hidden in the result
        self.max_total_thrust_n = float("inf")
        self.tilt_actuator = TiltActuator(tau_increase=tilt_tau_s, tau_decrease=tilt_tau_s)
        self.state = SimulationState()

    @classmethod
    def from_bridge(cls, bridge) -> "XZPlant":
        model = bridge.physical_model()
        plant = cls(model["mass_kg"], model["pitch_inertia_kgm2"], model["gravity_m_s2"],
                    model["pitch_lever_arm_m"], model["tilt_tau_s"])
        plant.max_total_thrust_n = model["max_total_thrust_n"]
        plant.pitch_effectiveness = model["pitch_effectiveness"]
        plant.tilt_actuator = TiltActuator(
            gain=model["tilt_gain"], delay=model["tilt_delay_s"],
            tau_increase=model["tilt_tau_s"], tau_decrease=model["tilt_tau_decrease_s"],
            pulse_direction=model["pitch_pulse_direction"], limit=model["servo_limit_rad"])
        return plant

    @property
    def hover_thrust_n(self) -> float:
        return self.mass_kg * self.gravity_m_s2

    def reset(self, **values: float) -> SimulationState:
        total=values.get("thrust_n",0.)
        values.setdefault("thrust_upper_n",total/2)
        values.setdefault("thrust_lower_n",total/2)
        self.state = SimulationState(**values)
        self.tilt_actuator.reset(self.state.pitch_tilt_rad)
        return self.state

    @staticmethod
    def _first_order(actual: float, target: float, dt_s: float, tau_s: float) -> float:
        alpha = min(1.0, max(0.0, dt_s / max(tau_s, 1.0e-6)))
        return actual + alpha * (target - actual)

    def step(self, command: ControllerOutput, dt_s: float) -> SimulationState:
        if dt_s <= 0.0 or not math.isfinite(dt_s):
            raise ValueError("dt_s must be finite and positive")
        state = self.state
        # Preserve the allocator's separate rotor commands and physical thrust ceiling.
        upper_cmd=max(0.,min(self.max_total_thrust_n/2,command.thrust_upper_n))
        lower_cmd=max(0.,min(self.max_total_thrust_n/2,command.thrust_lower_n))
        upper_mid=lag(state.thrust_upper_n,upper_cmd,dt_s/2,self.thrust_tau_s)
        lower_mid=lag(state.thrust_lower_n,lower_cmd,dt_s/2,self.thrust_tau_s)
        thrust_mid=upper_mid+lower_mid
        tilt_mid,tilt=self.tilt_actuator.step(state.pitch_tilt_rad,command.pitch_tilt_rad,state.time_s,dt_s)
        pitch_mid = state.pitch_rad + state.pitch_rate_rad_s * dt_s / 2.0
        body_x = -thrust_mid * math.sin(tilt_mid)
        body_z = thrust_mid * math.cos(tilt_mid)
        world_x = math.cos(pitch_mid) * body_x + math.sin(pitch_mid) * body_z
        world_z = -math.sin(pitch_mid) * body_x + math.cos(pitch_mid) * body_z
        ax = world_x / self.mass_kg - self.linear_drag * state.vx_m_s
        az = (world_z / self.mass_kg) - self.gravity_m_s2 - self.linear_drag * state.vz_m_s
        # r_z is negative in FLU (the thrust point is below CG), so
        # tau_y = r_z * F_x is positive when the C allocator's alpha is positive.
        pitch_moment = -self.pitch_effectiveness * self.pitch_lever_arm_m * body_x
        pitch_accel = pitch_moment / self.inertia_pitch - self.pitch_damping * state.pitch_rate_rad_s
        vx = state.vx_m_s + ax * dt_s
        vz = state.vz_m_s + az * dt_s
        pitch_rate = state.pitch_rate_rad_s + pitch_accel * dt_s
        upper=lag(state.thrust_upper_n,upper_cmd,dt_s,self.thrust_tau_s)
        lower=lag(state.thrust_lower_n,lower_cmd,dt_s,self.thrust_tau_s)
        thrust=upper+lower
        self.state = replace(
            state, time_s=state.time_s + dt_s,
            x_m=state.x_m + (state.vx_m_s + vx) * dt_s / 2.0,
            z_m=state.z_m + (state.vz_m_s + vz) * dt_s / 2.0,
            vx_m_s=vx, vz_m_s=vz,
            pitch_rad=state.pitch_rad + (state.pitch_rate_rad_s + pitch_rate) * dt_s / 2.0,
            pitch_rate_rad_s=pitch_rate, thrust_n=thrust, pitch_tilt_rad=tilt,
            accel_x_m_s2=ax, accel_z_m_s2=az,
            thrust_upper_n=upper, thrust_lower_n=lower,
        )
        return self.state
