from __future__ import annotations

import json
import struct
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


SAMPLE = struct.Struct("<IHHHBB")
TRANSITIONS_S = (0.3, 0.9, 1.2, 1.8)


def load_run(path: Path, delta_us: float) -> dict[str, object]:
    raw = path.read_bytes()
    if len(raw) % SAMPLE.size != 0:
        raise ValueError(f"bad sample file size: {path} {len(raw)}")
    rows = [SAMPLE.unpack_from(raw, offset) for offset in range(0, len(raw), SAMPLE.size)]
    return {
        "path": str(path),
        "delta_us": delta_us,
        "t_s": np.asarray([row[0] for row in rows], dtype=float) * 0.001,
        "target_us": np.asarray([row[1] for row in rows], dtype=float),
        "actual_us": np.asarray([row[2] for row in rows], dtype=float),
        "rtt_ms": np.asarray([row[3] for row in rows], dtype=float),
        "id": int(rows[0][4]),
    }


def command_offset(t_s: float, delta_us: float, delay_s: float) -> float:
    delayed_t = t_s - delay_s
    if TRANSITIONS_S[0] <= delayed_t < TRANSITIONS_S[1]:
        return delta_us
    if TRANSITIONS_S[2] <= delayed_t < TRANSITIONS_S[3]:
        return -delta_us
    return 0.0


def simulate(run: dict[str, object], params: np.ndarray, center_us: float) -> np.ndarray:
    gain, tau_increase_s, tau_decrease_s, delay_s = params[:4]
    t_s = run["t_s"]
    delta_us = float(run["delta_us"])
    measured = run["actual_us"]
    prediction = np.empty_like(measured)
    prediction[0] = measured[0]

    delayed_transitions = [value + delay_s for value in TRANSITIONS_S]
    for index in range(1, len(t_s)):
        start = float(t_s[index - 1])
        end = float(t_s[index])
        state = float(prediction[index - 1])
        boundaries = [start]
        boundaries.extend(value for value in delayed_transitions if start < value < end)
        boundaries.append(end)
        for left, right in zip(boundaries, boundaries[1:]):
            midpoint = 0.5 * (left + right)
            equilibrium = center_us + gain * command_offset(midpoint, delta_us, delay_s)
            tau_s = tau_increase_s if equilibrium >= state else tau_decrease_s
            state = equilibrium + (state - equilibrium) * np.exp(-(right - left) / tau_s)
        prediction[index] = state
    return prediction


def fit_runs(runs: list[dict[str, object]]) -> dict[str, object]:
    centers0 = [float(np.median(run["actual_us"][run["t_s"] < 0.25])) for run in runs]
    x0 = np.asarray([0.6, 0.16, 0.10, 0.06, *centers0], dtype=float)
    lower = np.asarray([0.05, 0.005, 0.005, 0.0, *([1300.0] * len(runs))])
    upper = np.asarray([1.5, 1.0, 1.0, 0.30, *([1700.0] * len(runs))])

    def residual(params: np.ndarray) -> np.ndarray:
        return np.concatenate(
            [simulate(run, params, params[4 + index]) - run["actual_us"]
             for index, run in enumerate(runs)]
        )

    result = least_squares(
        residual,
        x0,
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=3.0,
        max_nfev=5000,
    )
    residuals = residual(result.x)
    measured = np.concatenate([run["actual_us"] for run in runs])
    sse = float(np.sum(residuals ** 2))
    sst = float(np.sum((measured - np.mean(measured)) ** 2))

    return {
        "gain": float(result.x[0]),
        "tau_increase_s": float(result.x[1]),
        "tau_decrease_s": float(result.x[2]),
        "delay_s": float(result.x[3]),
        "center_us": [float(value) for value in result.x[4:]],
        "rmse_us": float(np.sqrt(np.mean(residuals ** 2))),
        "mae_us": float(np.mean(np.abs(residuals))),
        "r2": float(1.0 - (sse / sst)),
        "sample_count": int(len(measured)),
        "optimizer_cost": float(result.cost),
        "optimizer_success": bool(result.success),
    }


def plateau_stats(run: dict[str, object]) -> dict[str, float]:
    t_s = run["t_s"]
    actual = run["actual_us"]

    def median(start: float, end: float) -> float:
        return float(np.median(actual[(t_s >= start) & (t_s <= end)]))

    baseline = median(0.05, 0.25)
    positive = median(0.80, 0.89)
    center_mid = median(1.05, 1.19)
    negative = median(1.70, 1.79)
    center_final = median(1.95, 2.09)
    delta = float(run["delta_us"])
    return {
        "baseline_us": baseline,
        "positive_plateau_us": positive,
        "center_mid_us": center_mid,
        "negative_plateau_us": negative,
        "center_final_us": center_final,
        "positive_gain": (positive - baseline) / delta,
        "negative_gain": (center_mid - negative) / delta,
        "center_drift_us": center_final - baseline,
        "rtt_mean_ms": float(np.mean(run["rtt_ms"])),
        "rtt_max_ms": float(np.max(run["rtt_ms"])),
        "samples": int(len(actual)),
    }


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    axes = {
        "servo0_alpha_roll": [
            load_run(root / ".tmp/servo0_full_power_step_20260725.bin", 100.0),
            load_run(root / ".tmp/servo0_full_power_step200_20260725.bin", 200.0),
            load_run(root / ".tmp/servo0_full_supply_step200_repeat1_20260725.bin", 200.0),
            load_run(root / ".tmp/servo0_full_supply_step200_repeat2_20260725.bin", 200.0),
            load_run(root / ".tmp/servo0_full_supply_step200_repeat3_20260725.bin", 200.0),
        ],
        "servo1_beta_pitch": [
            load_run(root / ".tmp/servo1_full_power_step_20260725.bin", 100.0),
            load_run(root / ".tmp/servo1_full_power_step200_20260725.bin", 200.0),
            load_run(root / ".tmp/servo1_full_power_step200_repeat1_20260725.bin", 200.0),
            load_run(root / ".tmp/servo1_full_power_step200_repeat2_20260725.bin", 200.0),
            load_run(root / ".tmp/servo1_full_supply_step200_repeat4_20260725.bin", 200.0),
            load_run(root / ".tmp/servo1_full_supply_step200_repeat5_20260725.bin", 200.0),
            load_run(root / ".tmp/servo1_full_supply_step200_repeat6_20260725.bin", 200.0),
        ],
    }
    report: dict[str, object] = {}
    for axis, runs in axes.items():
        report[axis] = {
            "joint_model": fit_runs(runs),
            "large_signal_model": fit_runs(
                [run for run in runs if run["delta_us"] == 200.0]
            ),
            "per_run_models": {
                f"run_{index}_delta_{int(run['delta_us'])}_us": fit_runs([run])
                for index, run in enumerate(runs)
            },
            "plateaus": {
                f"run_{index}_delta_{int(run['delta_us'])}_us": plateau_stats(run)
                for index, run in enumerate(runs)
            },
        }

    output = root / ".tmp/servo_full_power_models_20260725.json"
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"saved={output}")


if __name__ == "__main__":
    main()
