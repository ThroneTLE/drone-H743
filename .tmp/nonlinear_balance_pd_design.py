#!/usr/bin/env python3
"""Design conservative nonlinear-balance attitude PD gains from bench data."""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import asdict, dataclass
from itertools import product
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / ".tmp" / "nonlinear_balance_pd_design_20260725"

INERTIA_NOMINAL = 0.051
INERTIA_CORNERS = (0.0459, 0.0561)
FORCE_NOMINAL = 13.410270
FORCE_CORNERS = (5.5, 15.644959)
EFFECTIVENESS_RATIO_CORNERS = (0.75, 1.25)
GYRO_FILTER_HZ = 80.0
TILT_LIMIT_RAD = math.radians(18.0)

FREQUENCY_RAD_S = np.logspace(-3.0, 2.5, 1400)
S = 1j * FREQUENCY_RAD_S
LOG_W = np.log(FREQUENCY_RAD_S)
GYRO_FILTER = 1.0 / (1.0 + S / (2.0 * math.pi * GYRO_FILTER_HZ))


AXES = {
    "roll": {
        "lever_m": 0.105,
        "effectiveness": 0.581,
        "servo_name": "servo_alpha_index0_id1_drives_beta",
        "servo_gain_corners": (0.773, 0.793),
        "servo_tau_corners_s": (0.070, 0.310),
        "servo_delay_corners_s": (0.016, 0.036),
        "servo_gain_nominal": 0.781794,
        "servo_tau_nominal_s": 0.188826,
        "servo_delay_nominal_s": 0.026,
        "current": (0.0671, 0.1104),
        "grid_kp": np.arange(0.010, 0.151, 0.005),
        "grid_kd": np.arange(0.030, 0.201, 0.005),
    },
    "pitch": {
        "lever_m": 0.145,
        "effectiveness": 0.569,
        "servo_name": "servo_beta_index1_id2_drives_alpha",
        "servo_gain_corners": (0.756, 0.779),
        "servo_tau_corners_s": (0.054, 0.087),
        "servo_delay_corners_s": (0.036, 0.063),
        "servo_gain_nominal": 0.769332,
        "servo_tau_nominal_s": 0.070966,
        "servo_delay_nominal_s": 0.0495,
        "current": (0.0660, 0.1138),
        "grid_kp": np.arange(0.010, 0.151, 0.005),
        "grid_kd": np.arange(0.030, 0.201, 0.005),
    },
}


@dataclass
class Margins:
    crossover_rad_s: float
    phase_margin_deg: float
    gain_margin_db: float


def interpolate(x0: float, x1: float, y0: float, y1: float, target: float) -> float:
    if abs(y1 - y0) < 1e-12:
        return x0
    return x0 + (x1 - x0) * (target - y0) / (y1 - y0)


def margins(open_loop: np.ndarray) -> Margins:
    log_magnitude = np.log(np.maximum(np.abs(open_loop), 1e-30))
    phase = np.unwrap(np.angle(open_loop))
    if phase[0] > -math.pi / 2.0:
        phase -= 2.0 * math.pi

    gain_indices = np.flatnonzero(
        log_magnitude[:-1] * log_magnitude[1:] <= 0.0
    )
    if gain_indices.size == 0:
        return Margins(0.0, -math.inf, -math.inf)

    gain_crossings: list[float] = []
    phase_margins: list[float] = []
    for index in gain_indices:
        log_wc = interpolate(
            LOG_W[index], LOG_W[index + 1],
            log_magnitude[index], log_magnitude[index + 1], 0.0,
        )
        phase_wc = np.interp(
            log_wc, LOG_W[index:index + 2], phase[index:index + 2]
        )
        gain_crossings.append(math.exp(log_wc))
        phase_margins.append(math.degrees(math.pi + phase_wc))

    gain_margins: list[float] = []
    for target in (-math.pi, -3.0 * math.pi, -5.0 * math.pi):
        phase_indices = np.flatnonzero(
            (phase[:-1] - target) * (phase[1:] - target) <= 0.0
        )
        for index in phase_indices:
            log_wg = interpolate(
                LOG_W[index], LOG_W[index + 1],
                phase[index], phase[index + 1], target,
            )
            log_mag_wg = np.interp(
                log_wg,
                LOG_W[index:index + 2],
                log_magnitude[index:index + 2],
            )
            gain_margins.append(-20.0 * log_mag_wg / math.log(10.0))

    selected = int(
        np.argmin(np.abs(np.asarray(gain_crossings, dtype=float) - 1.5))
    )
    return Margins(
        crossover_rad_s=gain_crossings[selected],
        phase_margin_deg=min(phase_margins),
        gain_margin_db=min(gain_margins) if gain_margins else 300.0,
    )


def allocation_capacity(axis: dict[str, object], force_n: float) -> float:
    return float(axis["effectiveness"]) * float(axis["lever_m"]) * force_n


def loop_transfer(
    axis: dict[str, object],
    kp: float,
    kd: float,
    force_n: float,
    inertia: float,
    effectiveness_ratio: float,
    servo_gain: float,
    servo_tau_s: float,
    servo_delay_s: float,
) -> np.ndarray:
    capacity = allocation_capacity(axis, force_n)
    if capacity <= kp:
        return np.full(S.shape, complex(np.inf, 0.0))
    implicit_gain = capacity / (capacity - kp)
    actuator = (
        servo_gain * effectiveness_ratio * np.exp(-S * servo_delay_s) /
        (1.0 + S * servo_tau_s)
    )
    controller = kp + kd * S * GYRO_FILTER
    return actuator * implicit_gain * controller / (inertia * S * S)


def evaluate(axis: dict[str, object], kp: float, kd: float) -> dict[str, object]:
    corner_margins = []
    for values in product(
        FORCE_CORNERS,
        INERTIA_CORNERS,
        EFFECTIVENESS_RATIO_CORNERS,
        axis["servo_gain_corners"],
        axis["servo_tau_corners_s"],
        axis["servo_delay_corners_s"],
    ):
        corner_margins.append(asdict(margins(loop_transfer(axis, kp, kd, *values))))

    nominal = asdict(margins(loop_transfer(
        axis,
        kp,
        kd,
        FORCE_NOMINAL,
        INERTIA_NOMINAL,
        1.0,
        float(axis["servo_gain_nominal"]),
        float(axis["servo_tau_nominal_s"]),
        float(axis["servo_delay_nominal_s"]),
    )))
    capacity = allocation_capacity(axis, FORCE_NOMINAL)
    implicit_gain = capacity / (capacity - kp)
    effective_gain = float(axis["servo_gain_nominal"]) * implicit_gain
    wn = math.sqrt(effective_gain * kp / INERTIA_NOMINAL)
    zeta = effective_gain * kd / (2.0 * INERTIA_NOMINAL * wn)
    return {
        "kp_n_m_per_rad": kp,
        "kd_n_m_s_per_rad": kd,
        "nominal": nominal,
        "worst_phase_margin_deg": min(x["phase_margin_deg"] for x in corner_margins),
        "worst_gain_margin_db": min(x["gain_margin_db"] for x in corner_margins),
        "nominal_wn_rad_s": wn,
        "nominal_zeta": zeta,
        "corners": len(corner_margins),
    }


def select_gains(axis: dict[str, object]) -> dict[str, object]:
    passed = []
    for kp in axis["grid_kp"]:
        for kd in axis["grid_kd"]:
            result = evaluate(axis, float(kp), float(kd))
            nominal_wc = result["nominal"]["crossover_rad_s"]
            if (
                result["worst_phase_margin_deg"] >= 50.0
                and result["worst_gain_margin_db"] >= 10.0
                and 1.0 <= nominal_wc <= 2.2
            ):
                score = abs(nominal_wc - 1.5) + abs(result["nominal_zeta"] - 0.9)
                passed.append((score, result))
    if not passed:
        raise RuntimeError("no gain pair passed the robust design gates")
    return min(passed, key=lambda item: item[0])[1]


def authority(axis: dict[str, object], kp: float, kd: float) -> dict[str, float]:
    capacity = allocation_capacity(axis, FORCE_NOMINAL)
    actual_tilt = float(axis["servo_gain_nominal"]) * TILT_LIMIT_RAD
    moment = capacity * math.sin(actual_tilt)
    rate_capacity = (moment - kp * math.sin(TILT_LIMIT_RAD)) / kd
    return {
        "allocation_capacity_n_m_per_sin_rad": capacity,
        "ideal_18deg_moment_n_m": capacity * math.sin(TILT_LIMIT_RAD),
        "servo_gain_limited_moment_n_m": moment,
        "servo_gain_limited_accel_rad_s2": moment / INERTIA_NOMINAL,
        "rate_capacity_at_18deg_error_rad_s": rate_capacity,
    }


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    report: dict[str, object] = {
        "method": "exact-delay frequency margins around the nonlinear allocation linearization",
        "linearization": (
            "L(s)=Ka*A(s)*rho*[C/(C-Kp)]*(Kp+Kd*s*Hgyro)/(I*s^2), "
            "C=eta*lever*thrust"
        ),
        "assumptions": {
            "inertia_kg_m2": INERTIA_NOMINAL,
            "inertia_corners_kg_m2": INERTIA_CORNERS,
            "force_nominal_n": FORCE_NOMINAL,
            "force_corners_n": FORCE_CORNERS,
            "effectiveness_ratio_corners": EFFECTIVENESS_RATIO_CORNERS,
            "gyro_filter_hz": GYRO_FILTER_HZ,
            "tilt_limit_deg": 18.0,
            "phase_margin_gate_deg": 50.0,
            "gain_margin_gate_db": 10.0,
        },
    }

    for name, axis in AXES.items():
        selected = select_gains(axis)
        current = evaluate(axis, *axis["current"])
        report[name] = {
            "physics": {
                key: value
                for key, value in axis.items()
                if key not in {"grid_kp", "grid_kd", "current"}
            },
            "recommended": selected,
            "current_source_default": current,
            "recommended_authority_at_hover": authority(
                axis,
                selected["kp_n_m_per_rad"],
                selected["kd_n_m_s_per_rad"],
            ),
        }

    output = OUT_DIR / "results.json"
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"saved={output}")


if __name__ == "__main__":
    main()
