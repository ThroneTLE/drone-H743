#!/usr/bin/env python3
"""Robust PD search for the corrected force-frame attitude controller."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from itertools import product
from pathlib import Path
from functools import lru_cache

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / ".tmp" / "force_frame_pd_robust_search_20260725"

INERTIA = 0.051
GYRO_TAU = 1.0 / (2.0 * math.pi * 80.0)
FORCES_N = np.array([5.5, 9.25, 13.40927, 16.85, 17.5])
DELAYS_S = (0.04, 0.06, 0.08, 0.10)
SERVO_TAUS_S = (0.01, 0.03)
FREQUENCY_RAD_S = np.logspace(-2.0, 2.7, 1000)
S = 1j * FREQUENCY_RAD_S
LOG_W = np.log(FREQUENCY_RAD_S)
DESIGN_MIN_PHASE_MARGIN_DEG = 56.0
DESIGN_MIN_GAIN_MARGIN_DB = 12.0
DESIGN_MIN_ZETA = 0.90
DESIGN_MAX_ZETA = 1.05


AXES = {
    "pitch": {"lever_m": 0.145, "effectiveness": 0.5691933250, "delay_s": 0.04},
    "roll": {"lever_m": 0.105, "effectiveness": 0.5809669373, "delay_s": 0.06},
}


@dataclass
class Metrics:
    stable: bool
    min_phase_margin_deg: float
    min_gain_margin_db: float
    nominal_crossover_rad_s: float
    nominal_phase_margin_deg: float
    nominal_gain_margin_db: float
    corners: int


def _interpolate(x0: float, x1: float, y0: float, y1: float, target: float) -> float:
    if abs(y1 - y0) < 1e-12:
        return x0
    return x0 + (x1 - x0) * (target - y0) / (y1 - y0)


def margins(open_loop: np.ndarray) -> tuple[float, float, float]:
    log_mag = np.log(np.maximum(np.abs(open_loop), 1e-30))
    phase = np.unwrap(np.angle(open_loop))
    if phase[0] > -math.pi / 2.0:
        phase -= 2.0 * math.pi

    gain_crossings = np.flatnonzero(log_mag[:-1] * log_mag[1:] <= 0.0)
    if gain_crossings.size == 0:
        return 0.0, -math.inf, -math.inf

    crossovers: list[float] = []
    phase_margins: list[float] = []
    for index in gain_crossings:
        log_wc = _interpolate(
            LOG_W[index], LOG_W[index + 1],
            log_mag[index], log_mag[index + 1], 0.0,
        )
        phase_wc = np.interp(log_wc, LOG_W[index:index + 2], phase[index:index + 2])
        crossovers.append(math.exp(log_wc))
        phase_margins.append(math.degrees(math.pi + phase_wc))

    gain_margins_db: list[float] = []
    phase_min = float(np.min(phase))
    phase_max = float(np.max(phase))
    harmonic = 0
    while True:
        target = -(2 * harmonic + 1) * math.pi
        if target < phase_min - 1e-9:
            break
        if target <= phase_max + 1e-9:
            indices = np.flatnonzero((phase[:-1] - target) * (phase[1:] - target) <= 0.0)
            for index in indices:
                log_wg = _interpolate(
                    LOG_W[index], LOG_W[index + 1],
                    phase[index], phase[index + 1], target,
                )
                log_mag_wg = np.interp(
                    log_wg, LOG_W[index:index + 2], log_mag[index:index + 2]
                )
                gain_margins_db.append(-20.0 * log_mag_wg / math.log(10.0))
        harmonic += 1

    gain_margin_db = min(gain_margins_db) if gain_margins_db else 300.0
    best_index = int(np.argmin(np.abs(np.asarray(crossovers) - 1.5)))
    return crossovers[best_index], min(phase_margins), gain_margin_db


@lru_cache(maxsize=None)
def plant_base(effectiveness: float, inertia: float, tau: float, delay: float) -> np.ndarray:
    return (
        effectiveness / inertia
        * np.exp(-S * delay)
        / ((1.0 + S * tau) * S * S)
    )


def controller_term(lever: float, stiffness_n_rad: float, damping_nm_s_rad: float) -> np.ndarray:
    return lever * stiffness_n_rad + damping_nm_s_rad * S / (1.0 + S * GYRO_TAU)


def evaluate(
    axis: dict[str, float],
    stiffness_by_force: np.ndarray,
    damping: float,
) -> Metrics:
    phase_margins: list[float] = []
    gain_margins: list[float] = []
    stable = True
    corner_count = 0

    # Force-scheduled candidates have identical stiffness at every force point.
    # Deduplicating here avoids evaluating the same uncertainty corner five times.
    stiffness_corners = np.unique(np.asarray(stiffness_by_force, dtype=float))
    for eta_scale, inertia_scale, tau, delay, stiffness in product(
        (0.8, 1.2), (0.9, 1.1), SERVO_TAUS_S, DELAYS_S, stiffness_corners
    ):
        if stiffness <= 0.0:
            stable = False
            phase_margins.append(-math.inf)
            gain_margins.append(-math.inf)
            corner_count += 1
            continue
        loop = plant_base(
            axis["effectiveness"] * eta_scale,
            INERTIA * inertia_scale,
            tau,
            delay,
        ) * controller_term(axis["lever_m"], float(stiffness), damping)
        _, phase_margin, gain_margin = margins(loop)
        corner_stable = bool(phase_margin > 0.0 and gain_margin > 0.0)
        stable = bool(stable and corner_stable)
        phase_margins.append(phase_margin)
        gain_margins.append(gain_margin)
        corner_count += 1

    nominal_stiffness = float(np.interp(13.40927, FORCES_N, stiffness_by_force))
    nominal_loop = plant_base(
        axis["effectiveness"], INERTIA, 0.02, axis["delay_s"]
    ) * controller_term(axis["lever_m"], nominal_stiffness, damping)
    nominal_wc, nominal_pm, nominal_gm = margins(nominal_loop)
    return Metrics(
        stable=stable,
        min_phase_margin_deg=float(min(phase_margins)),
        min_gain_margin_db=float(min(gain_margins)),
        nominal_crossover_rad_s=float(nominal_wc),
        nominal_phase_margin_deg=float(nominal_pm),
        nominal_gain_margin_db=float(nominal_gm),
        corners=corner_count,
    )


def candidate_score(metrics: Metrics) -> float:
    return (
        abs(metrics.nominal_crossover_rad_s - 1.5)
        + max(0.0, 60.0 - metrics.min_phase_margin_deg) / 20.0
    )


def search_fixed(axis: dict[str, float]) -> dict[str, object]:
    passed: list[tuple[float, float, Metrics]] = []
    stable: list[tuple[float, float, Metrics]] = []
    for kp in np.arange(-5.25, 5.001, 0.25):
        stiffness = FORCES_N + kp
        for damping in np.arange(0.05, 0.801, 0.025):
            result = evaluate(axis, stiffness, float(damping))
            item = (float(kp), float(damping), result)
            if result.stable:
                stable.append(item)
                if (result.min_phase_margin_deg >= DESIGN_MIN_PHASE_MARGIN_DEG and
                        result.min_gain_margin_db >= DESIGN_MIN_GAIN_MARGIN_DB):
                    passed.append(item)

    if passed:
        kp, damping, result = min(passed, key=lambda item: candidate_score(item[2]))
        status = "passed"
    elif stable:
        kp, damping, result = max(stable, key=lambda item: item[2].min_phase_margin_deg)
        status = "margin_failed"
    else:
        return {"status": "no_stable_candidate"}
    return {
        "status": status,
        "internal_kp_n_rad": kp,
        "internal_kd_nm_s_rad": -damping,
        "synex_kp": -kp,
        "synex_kd": damping,
        "metrics": asdict(result),
        "stiffness_range_n_rad": [float(FORCES_N[0] + kp), float(FORCES_N[-1] + kp)],
    }


def search_scheduled(axis: dict[str, float]) -> dict[str, object]:
    passed: list[tuple[float, float, float, float, Metrics]] = []
    stable: list[tuple[float, float, Metrics]] = []
    for stiffness in np.arange(0.25, 4.001, 0.05):
        stiffness_values = np.full(FORCES_N.shape, stiffness)
        for damping in np.arange(0.05, 0.501, 0.01):
            result = evaluate(axis, stiffness_values, float(damping))
            item = (float(stiffness), float(damping), result)
            if result.stable:
                stable.append(item)
                wn = math.sqrt(
                    axis["effectiveness"] * axis["lever_m"] * stiffness / INERTIA
                )
                zeta = axis["effectiveness"] * damping / (2.0 * INERTIA * wn)
                if (result.min_phase_margin_deg >= DESIGN_MIN_PHASE_MARGIN_DEG and
                        result.min_gain_margin_db >= DESIGN_MIN_GAIN_MARGIN_DB and
                        DESIGN_MIN_ZETA <= zeta <= DESIGN_MAX_ZETA):
                    passed.append((float(stiffness), float(damping), wn, zeta, result))

    if passed:
        stiffness, damping, wn, zeta, result = max(
            passed, key=lambda item: (item[2], item[4].min_phase_margin_deg)
        )
        status = "passed"
    elif stable:
        stiffness, damping, result = max(
            stable, key=lambda item: item[2].min_phase_margin_deg
        )
        status = "margin_failed"
    else:
        return {"status": "no_stable_candidate"}

    eta = axis["effectiveness"]
    wn = math.sqrt(eta * axis["lever_m"] * stiffness / INERTIA)
    zeta = eta * damping / (2.0 * INERTIA * wn)
    return {
        "status": status,
        "effective_stiffness_n_rad": stiffness,
        "internal_kd_nm_s_rad": -damping,
        "synex_kd": damping,
        "internal_kp_schedule": "effective_stiffness_n_rad - ctrl_total_force_n",
        "internal_kp_range_n_rad": [
            float(stiffness - FORCES_N[-1]),
            float(stiffness - FORCES_N[0]),
        ],
        "synex_kp_range": [
            float(FORCES_N[0] - stiffness),
            float(FORCES_N[-1] - stiffness),
        ],
        "hover_internal_kp_n_rad": float(stiffness - 13.40927),
        "hover_synex_kp": float(13.40927 - stiffness),
        "rigid_body_wn_rad_s": wn,
        "rigid_body_zeta": zeta,
        "metrics": asdict(result),
    }


def write_report(results: dict[str, object]) -> None:
    lines = [
        "# Corrected Force-Frame Robust PD Search",
        "",
        f"Force range: `{FORCES_N[0]:.2f}..{FORCES_N[-1]:.2f} N`; inertia: `{INERTIA:.3f} kg*m^2`.",
        "Uncertainty: effectiveness +/-20%, inertia +/-10%, servo tau 10/30 ms, delay 40/60/80/100 ms.",
        f"Design gates: PM >= {DESIGN_MIN_PHASE_MARGIN_DEG:.0f} deg, GM >= "
        f"{DESIGN_MIN_GAIN_MARGIN_DB:.0f} dB, nominal zeta "
        f"{DESIGN_MIN_ZETA:.2f}..{DESIGN_MAX_ZETA:.2f}.",
        "",
    ]
    for axis_name in ("pitch", "roll"):
        axis_result = results[axis_name]
        lines.extend([f"## {axis_name.upper()}", ""])
        for mode in ("fixed", "scheduled"):
            item = axis_result[mode]
            lines.append(f"### {mode}")
            lines.append("")
            lines.append(f"- Status: `{item['status']}`")
            if "metrics" in item:
                metric = item["metrics"]
                lines.append(
                    f"- Worst PM/GM: `{metric['min_phase_margin_deg']:.2f} deg / "
                    f"{metric['min_gain_margin_db']:.2f} dB`; nominal crossover: "
                    f"`{metric['nominal_crossover_rad_s']:.3f} rad/s`."
                )
            if mode == "fixed" and "internal_kp_n_rad" in item:
                lines.append(
                    f"- Internal Kp/Kd: `{item['internal_kp_n_rad']:+.3f} / "
                    f"{item['internal_kd_nm_s_rad']:+.3f}`; Synex: "
                    f"`{item['synex_kp']:+.3f} / {item['synex_kd']:+.3f}`."
                )
            if mode == "scheduled" and "effective_stiffness_n_rad" in item:
                lines.append(
                    f"- Effective stiffness/Kd: `{item['effective_stiffness_n_rad']:.3f} / "
                    f"{item['internal_kd_nm_s_rad']:+.3f}`; Kp schedule: "
                    "`Kp_internal = stiffness - ctrl_total_force_n`."
                )
                lines.append(
                    f"- Hover internal/Synex Kp: `{item['hover_internal_kp_n_rad']:+.3f} / "
                    f"{item['hover_synex_kp']:+.3f}`; Synex Kp range: "
                    f"`{item['synex_kp_range'][0]:.3f}..{item['synex_kp_range'][1]:.3f}`."
                )
                lines.append(
                    f"- Rigid-body wn/zeta: `{item['rigid_body_wn_rad_s']:.3f} / "
                    f"{item['rigid_body_zeta']:.3f}`."
                )
            lines.append("")
    (OUT_DIR / "report.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    benchmark_axis = AXES["pitch"]
    benchmark_loop = plant_base(
        benchmark_axis["effectiveness"], INERTIA, 0.02, 0.04
    ) * controller_term(
        benchmark_axis["lever_m"], 13.40927 - 12.0, 0.30
    )
    benchmark_wc, benchmark_pm, _ = margins(benchmark_loop)
    if not (abs(benchmark_wc - 3.411) < 0.01 and abs(benchmark_pm - 66.6) < 0.2):
        raise RuntimeError(
            f"margin self-check failed: wc={benchmark_wc:.3f}, pm={benchmark_pm:.2f}"
        )

    results: dict[str, object] = {
        "method": "Exact-delay frequency-domain robust grid search",
        "force_range_n": FORCES_N.tolist(),
        "inertia_kg_m2": INERTIA,
        "delay_s": list(DELAYS_S),
        "servo_tau_s": list(SERVO_TAUS_S),
    }
    for axis_name, axis in AXES.items():
        print(f"Searching {axis_name} fixed gains...", flush=True)
        fixed = search_fixed(axis)
        print(f"Searching {axis_name} force-scheduled gains...", flush=True)
        scheduled = search_scheduled(axis)
        results[axis_name] = {"physics": axis, "fixed": fixed, "scheduled": scheduled}

    (OUT_DIR / "results.json").write_text(
        json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    write_report(results)
    print(OUT_DIR / "report.md")


if __name__ == "__main__":
    main()
