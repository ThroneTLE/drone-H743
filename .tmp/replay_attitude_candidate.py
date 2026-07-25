#!/usr/bin/env python3
"""Replay the current force-layer attitude controller on one recorded gain group."""

from __future__ import annotations

import json
import math

import numpy as np
import pandas as pd


CSV = "flightlog_20260724_200832.csv"
META = "flightlog_20260724_200832_meta.json"
TILT_LIMIT = 0.314159


def replay(frame: pd.DataFrame, mass: float, lever: float, kp: float, kd: float) -> pd.DataFrame:
    old_mass = frame["mass_kg"].to_numpy()
    pitch = np.deg2rad(frame["pitch_deg"].to_numpy())
    roll = np.deg2rad(frame["roll_deg"].to_numpy())
    gyro_y = np.deg2rad(frame["gyro_y_dps"].to_numpy())
    gyro_x = np.deg2rad(frame["gyro_x_dps"].to_numpy())

    # Remove the recorded force-layer P term, then rescale the remaining model force.
    base_x_old = frame["ctrl_force_cmd_n_0"].to_numpy() + frame["pitch_angle_kp"].to_numpy() * pitch
    base_y_old = frame["ctrl_force_cmd_n_1"].to_numpy() + frame["roll_angle_kp"].to_numpy() * roll
    scale = mass / old_mass
    force_x = base_x_old * scale - kp * pitch
    force_y = base_y_old * scale - kp * roll
    force_z = frame["ctrl_force_cmd_n_2"].to_numpy() * scale

    alpha_ff = np.arctan2(force_x, force_z)
    alpha_d = kd * gyro_y / (force_z * lever)
    alpha = np.clip(alpha_ff + alpha_d, -TILT_LIMIT, TILT_LIMIT)
    beta_ff = -np.arctan2(force_y * np.cos(alpha), force_z)
    beta_d = kd * gyro_x / (force_z * lever)
    beta = np.clip(beta_ff + beta_d, -TILT_LIMIT, TILT_LIMIT)
    total_force = force_z / (np.cos(alpha) * np.cos(beta))
    return pd.DataFrame({
        "alpha_ff": alpha_ff,
        "alpha_d": alpha_d,
        "alpha": alpha,
        "beta_ff": beta_ff,
        "beta_d": beta_d,
        "beta": beta,
        "total_force": total_force,
    })


def metrics(label: str, replayed: pd.DataFrame) -> dict:
    alpha = replayed["alpha"].to_numpy()
    beta = replayed["beta"].to_numpy()
    force = replayed["total_force"].to_numpy()
    return {
        "label": label,
        "tilt_vector_rms_rad": float(np.sqrt(np.mean(alpha ** 2 + beta ** 2))),
        "tilt_saturation_pct": float(100.0 * np.mean((np.abs(alpha) >= TILT_LIMIT) | (np.abs(beta) >= TILT_LIMIT))),
        "force_mean_n": float(np.mean(force)),
        "force_p95_n": float(np.quantile(force, 0.95)),
        "force_over_measured_max_pct": float(100.0 * np.mean(force > 15.644959)),
    }


def main() -> None:
    frame = pd.read_csv(CSV)
    meta = json.load(open(META, encoding="utf-8"))
    lookup = {
        (int(sector["sector_seq"]), int(sector["sector_index"])): sector["params"]
        for sector in meta["sectors"]
    }
    params = [lookup[(int(seq), int(index))]
              for seq, index in zip(frame["sector_seq"], frame["sector_index"])]
    for name in ("mass_kg", "roll_angle_kp", "pitch_angle_kp", "roll_rate_kd", "pitch_rate_kd"):
        frame[name] = [item[name] for item in params]
    selected = frame.loc[
        frame["motor_output_reason_name"].eq("stabilized_mix")
        & frame["pitch_angle_kp"].eq(-1.0)
        & frame["pitch_rate_kd"].eq(-1.5)
    ].copy()

    recorded = replay(selected, mass=1.2, lever=0.201, kp=-1.0, kd=-1.5)
    alpha_error = recorded["alpha"].to_numpy() - selected["ctrl_tilt_out_rad_0"].to_numpy()
    beta_error = recorded["beta"].to_numpy() - selected["ctrl_tilt_out_rad_1"].to_numpy()

    candidates = [
        ("unchanged", 1.367, 0.250, -1.0, -1.5),
        ("lever_scaled", 1.367, 0.250, -1.0, -1.5 * 0.250 / 0.201),
        ("mass_and_lever_scaled", 1.367, 0.250, -1.0, -1.5 * (1.367 / 1.2) * 0.250 / 0.201),
        ("rounded_candidate", 1.367, 0.250, -1.0, -1.9),
    ]
    report = {
        "selected_rows": len(selected),
        "recorded_replay_max_abs_error_rad": {
            "pitch": float(np.max(np.abs(alpha_error))),
            "roll": float(np.max(np.abs(beta_error))),
        },
        "recorded_metrics": metrics("recorded", recorded),
        "candidates": [metrics(label, replay(selected, mass, lever, kp, kd))
                       for label, mass, lever, kp, kd in candidates],
        "exact_lever_scaled_kd": -1.5 * 0.250 / 0.201,
        "exact_mass_and_lever_scaled_kd": -1.5 * (1.367 / 1.2) * 0.250 / 0.201,
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
