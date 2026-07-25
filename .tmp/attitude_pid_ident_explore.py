#!/usr/bin/env python3
"""Read-only attitude system-identification experiment for one H743 flight log."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import signal


PWM_US = np.array([
    1100, 1142, 1184, 1226, 1268, 1310, 1352, 1394, 1436, 1478,
    1520, 1562, 1604, 1646, 1688, 1730, 1772, 1814, 1856, 1898, 1940,
], dtype=float)
DUAL_THRUST_G = np.array([
    0.000, 5.069, 25.589, 60.655, 106.361, 165.084, 216.696,
    287.758, 386.724, 501.680, 624.697, 725.173, 828.680, 923.574,
    981.674, 1114.845, 1256.137, 1366.352, 1466.668, 1541.404, 1595.342,
], dtype=float)
G_PER_N = 101.971621


@dataclass
class AxisSpec:
    name: str
    angle_col: str
    gyro_col: str
    servo_col: str
    servo_to_body_sign: float
    lever_m: float
    kp_col: str
    kd_col: str


AXES = (
    AxisSpec("pitch", "pitch_deg", "gyro_y_dps", "servo_beta_us", -1.0,
             0.145, "pitch_angle_kp", "pitch_rate_kd"),
    AxisSpec("roll", "roll_deg", "gyro_x_dps", "servo_alpha_us", -1.0,
             0.105, "roll_angle_kp", "roll_rate_kd"),
)


def robust_fit(x: np.ndarray, y: np.ndarray, iterations: int = 12) -> tuple[np.ndarray, np.ndarray]:
    beta = np.linalg.lstsq(x, y, rcond=None)[0]
    weights = np.ones(len(y))
    for _ in range(iterations):
        residual = y - x @ beta
        scale = 1.4826 * np.median(np.abs(residual - np.median(residual))) + 1.0e-9
        cutoff = 1.5 * scale
        weights = np.minimum(1.0, cutoff / (np.abs(residual) + 1.0e-12))
        xw = x * np.sqrt(weights)[:, None]
        yw = y * np.sqrt(weights)
        beta = np.linalg.lstsq(xw, yw, rcond=None)[0]
    return beta, weights


def lowpass(values: np.ndarray, cutoff_hz: float, sample_hz: float) -> np.ndarray:
    sos = signal.butter(4, cutoff_hz, fs=sample_hz, output="sos")
    return signal.sosfiltfilt(sos, values)


def first_order(values: np.ndarray, dt: float, tau_s: float, delay_s: float) -> np.ndarray:
    delayed = np.interp(
        np.arange(len(values)) * dt - delay_s,
        np.arange(len(values)) * dt,
        values,
        left=values[0],
        right=values[-1],
    )
    if tau_s <= 0.0:
        return delayed
    alpha = 1.0 - math.exp(-dt / tau_s)
    result = np.empty_like(delayed)
    result[0] = delayed[0]
    for index in range(1, len(delayed)):
        result[index] = result[index - 1] + alpha * (delayed[index] - result[index - 1])
    return result


def pwm_total_force_n(upper_us: np.ndarray, lower_us: np.ndarray) -> np.ndarray:
    upper_dual_g = np.interp(np.clip(upper_us, PWM_US[0], PWM_US[-1]), PWM_US, DUAL_THRUST_G)
    lower_dual_g = np.interp(np.clip(lower_us, PWM_US[0], PWM_US[-1]), PWM_US, DUAL_THRUST_G)
    return (upper_dual_g + lower_dual_g) / (2.0 * G_PER_N)


def attach_params(frame: pd.DataFrame, meta: dict) -> pd.DataFrame:
    lookup = {
        (int(sector["sector_seq"]), int(sector["sector_index"])): sector.get("params", {})
        for sector in meta.get("sectors", [])
    }
    params = [lookup.get((int(seq), int(index)), {})
              for seq, index in zip(frame["sector_seq"], frame["sector_index"])]
    names = (
        "roll_angle_kp", "roll_rate_kd", "pitch_angle_kp", "pitch_rate_kd",
        "pos_z_kp", "mass_kg", "tilt_lever_arm_m",
    )
    for name in names:
        frame[name] = [item.get(name, np.nan) for item in params]
    return frame


def build_segments(frame: pd.DataFrame, sample_hz: float) -> list[pd.DataFrame]:
    frame = frame.copy()
    frame["segment"] = (frame["timestamp_us"].diff().abs().fillna(0) > 100_000).cumsum()
    segments: list[pd.DataFrame] = []
    dt = 1.0 / sample_hz
    needed = [
        "roll_deg", "pitch_deg", "gyro_x_dps", "gyro_y_dps",
        "servo_alpha_us", "servo_beta_us", "motor_upper_us",
        "motor_lower_us", "roll_angle_kp", "roll_rate_kd",
        "pitch_angle_kp", "pitch_rate_kd", "pos_z_kp", "mass_kg",
        "tilt_lever_arm_m",
    ]
    for segment_id, raw in frame.groupby("segment", sort=True):
        raw = raw.loc[raw["motor_output_reason_name"].eq("stabilized_mix")].copy()
        if len(raw) < 100:
            continue
        t = raw["timestamp_us"].to_numpy(dtype=float) / 1.0e6
        t_uniform = np.arange(t[0], t[-1] + 0.25 * dt, dt)
        data = {"time_s": t_uniform, "source_segment": np.full(len(t_uniform), int(segment_id))}
        for column in needed:
            data[column] = np.interp(t_uniform, t, raw[column].to_numpy(dtype=float))
        data["force_n"] = pwm_total_force_n(data["motor_upper_us"], data["motor_lower_us"])
        segments.append(pd.DataFrame(data))
    return segments


def rate_sign(segment: pd.DataFrame, spec: AxisSpec, sample_hz: float) -> tuple[float, float]:
    angle_rate = np.gradient(lowpass(np.deg2rad(segment[spec.angle_col].to_numpy()), 8.0, sample_hz),
                             1.0 / sample_hz)
    gyro = lowpass(np.deg2rad(segment[spec.gyro_col].to_numpy()), 8.0, sample_hz)
    correlation = float(np.corrcoef(angle_rate, gyro)[0, 1])
    return (1.0 if correlation >= 0.0 else -1.0), correlation


def prepared_axis_segments(segments: list[pd.DataFrame], spec: AxisSpec,
                           sample_hz: float, inertia: float,
                           tau_s: float, delay_s: float) -> list[dict[str, np.ndarray | float | int]]:
    result = []
    dt = 1.0 / sample_hz
    for segment in segments:
        sign_rate, correlation = rate_sign(segment, spec, sample_hz)
        angle = lowpass(np.deg2rad(segment[spec.angle_col].to_numpy(dtype=float)), 8.0, sample_hz)
        rate = lowpass(sign_rate * np.deg2rad(segment[spec.gyro_col].to_numpy(dtype=float)), 8.0, sample_hz)
        rate_dot = np.gradient(rate, dt)
        # 500..2500 us maps to 0..180 deg; 1500 us is the neutral 90 deg.
        command = (segment[spec.servo_col].to_numpy(dtype=float) - 1500.0) * math.pi / 2000.0
        command *= spec.servo_to_body_sign
        actuator = first_order(command, dt, tau_s, delay_s)
        force = first_order(segment["force_n"].to_numpy(dtype=float), dt, 0.176624, 0.0)
        drive = force * spec.lever_m / inertia * np.sin(actuator)
        margin = max(0.25, 3.0 * tau_s + delay_s)
        trim = int(round(margin * sample_hz))
        valid = np.ones(len(segment), dtype=bool)
        valid[:trim] = False
        valid[-max(1, int(0.10 * sample_hz)):] = False
        valid &= np.abs(command) < 0.29
        result.append({
            "segment": int(segment["source_segment"].iloc[0]),
            "angle": angle,
            "rate": rate,
            "rate_dot": rate_dot,
            "drive": drive,
            "valid": valid,
            "rate_sign": sign_rate,
            "angle_rate_corr": correlation,
            "kp": float(np.median(segment[spec.kp_col])),
            "kd": float(np.median(segment[spec.kd_col])),
            "z_kp": float(np.median(segment["pos_z_kp"])),
        })
    return result


def plant_fit(prepared: list[dict[str, np.ndarray | float | int]], holdout: int | None = None):
    xs = []
    ys = []
    for item in prepared:
        if holdout is not None and int(item["segment"]) == holdout:
            continue
        valid = item["valid"]
        drive = item["drive"]
        rate = item["rate"]
        angle = item["angle"]
        xs.append(np.column_stack([
            drive[valid], rate[valid], angle[valid], np.ones(np.sum(valid)),
        ]))
        ys.append(item["rate_dot"][valid])
    x = np.vstack(xs)
    y = np.concatenate(ys)
    beta, weights = robust_fit(x, y)
    residual = y - x @ beta
    return beta, float(np.sqrt(np.mean(residual ** 2))), float(np.mean(weights)), len(y)


def validation_score(prepared: list[dict[str, np.ndarray | float | int]]) -> tuple[float, list[float]]:
    scores = []
    for item in prepared:
        beta, _, _, _ = plant_fit(prepared, holdout=int(item["segment"]))
        valid = item["valid"]
        x = np.column_stack([
            item["drive"][valid], item["rate"][valid], item["angle"][valid],
            np.ones(np.sum(valid)),
        ])
        y = item["rate_dot"][valid]
        residual = y - x @ beta
        scale = np.sqrt(np.mean((y - np.mean(y)) ** 2)) + 1.0e-9
        scores.append(float(np.sqrt(np.mean(residual ** 2)) / scale))
    return float(np.median(scores)), scores


def closed_loop_fits(prepared: list[dict[str, np.ndarray | float | int]]) -> list[dict]:
    fits = []
    for item in prepared:
        valid = item["valid"]
        x = np.column_stack([
            item["angle"][valid], item["rate"][valid], np.ones(np.sum(valid)),
        ])
        y = item["rate_dot"][valid]
        beta, weights = robust_fit(x, y)
        residual = y - x @ beta
        a_angle, a_rate, bias = beta
        wn = math.sqrt(-a_angle) if a_angle < 0.0 else math.nan
        zeta = (-a_rate / (2.0 * wn)) if wn > 0.0 else math.nan
        fits.append({
            "segment": item["segment"], "kp": item["kp"], "kd": item["kd"],
            "z_kp": item["z_kp"], "a_angle_s2": float(a_angle),
            "a_rate_s1": float(a_rate), "bias_rad_s2": float(bias),
            "wn_rad_s": wn, "zeta": zeta,
            "rmse_rad_s2": float(np.sqrt(np.mean(residual ** 2))),
            "robust_weight_mean": float(np.mean(weights)),
            "rate_sign": item["rate_sign"], "angle_rate_corr": item["angle_rate_corr"],
            "samples": int(np.sum(valid)),
        })
    return fits


def identify(frame: pd.DataFrame, meta: dict, inertia: float) -> dict:
    sample_hz = float(meta.get("begin", {}).get("log_rate", 250.0))
    segments = build_segments(frame, sample_hz)
    report = {
        "sample_hz": sample_hz,
        "inertia_kg_m2": inertia,
        "segments": len(segments),
        "axes": {},
    }
    for spec in AXES:
        candidates = []
        for tau_s in np.arange(0.0, 0.401, 0.02):
            for delay_s in np.arange(0.0, 0.201, 0.01):
                prepared = prepared_axis_segments(
                    segments, spec, sample_hz, inertia, float(tau_s), float(delay_s)
                )
                score, per_segment = validation_score(prepared)
                candidates.append((score, float(tau_s), float(delay_s), per_segment, prepared))
        candidates.sort(key=lambda item: item[0])
        score, tau_s, delay_s, per_segment, prepared = candidates[0]
        beta, rmse, robust_weight, count = plant_fit(prepared)
        report["axes"][spec.name] = {
            "physical_lever_m": spec.lever_m,
            "servo_pwm_column": spec.servo_col,
            "actuator_tau_s": tau_s,
            "actuator_delay_s": delay_s,
            "validation_nrmse_median": score,
            "validation_nrmse_by_segment": per_segment,
            "plant_drive_gain": float(beta[0]),
            "plant_rate_coefficient_s1": float(beta[1]),
            "plant_angle_coefficient_s2": float(beta[2]),
            "plant_bias_rad_s2": float(beta[3]),
            "plant_fit_rmse_rad_s2": rmse,
            "robust_weight_mean": robust_weight,
            "samples": count,
            "closed_loop": closed_loop_fits(prepared),
            "candidate_top10": [
                {"nrmse": item[0], "tau_s": item[1], "delay_s": item[2]}
                for item in candidates[:10]
            ],
        }
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--meta", type=Path, required=True)
    parser.add_argument("--inertia", type=float, default=0.051)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    frame = pd.read_csv(args.csv)
    meta = json.loads(args.meta.read_text(encoding="utf-8"))
    attach_params(frame, meta)
    result = identify(frame, meta, args.inertia)
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
