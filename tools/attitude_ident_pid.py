#!/usr/bin/env python3
"""Analyze closed-loop attitude-identification FLOG CSVs and suggest PD gains.

This tool is intentionally conservative: it uses IDENT ATT logs as a known
closed-loop reference excitation, fits a second-order closed-loop response, and
then converts the chosen target poles to the controller's moment-layer gains.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import signal
from scipy.optimize import least_squares


AXIS_NAMES = {
    1: "roll",
    2: "pitch",
}

AXIS_OUTPUT_DEG = {
    "roll": "roll_deg",
    "pitch": "pitch_deg",
}


@dataclass
class SegmentFit:
    axis: str
    start_row: int
    end_row: int
    rows: int
    duration_s: float
    fit_pct: float
    gain: float
    wn_rad_s: float
    zeta: float
    delay_s: float
    dominant_hz: float
    rms_error_deg: float
    suggested_kr: float
    suggested_kw: float
    param_kp: float
    param_kd: float


def split_segments(df: pd.DataFrame) -> list[tuple[int, int]]:
    active = df["ident_att_active"].to_numpy(dtype=float) >= 0.5
    axis = df["ident_att_axis"].to_numpy(dtype=int)
    timestamp = df["timestamp_us"].to_numpy(dtype=float) * 1.0e-6
    sequence = df["sequence"].to_numpy(dtype=int)
    segments: list[tuple[int, int]] = []
    start: int | None = None

    for i in range(len(df)):
        valid = active[i] and axis[i] in AXIS_NAMES
        if not valid:
            if start is not None:
                segments.append((start, i))
                start = None
            continue
        if start is None:
            start = i
            continue
        gap = timestamp[i] - timestamp[i - 1]
        seq_gap = sequence[i] - sequence[i - 1]
        if gap <= 0.0 or gap > 0.5 or seq_gap < 0 or seq_gap > 100 or axis[i] != axis[i - 1]:
            segments.append((start, i))
            start = i
    if start is not None:
        segments.append((start, len(df)))
    return segments


def clean_segment(df: pd.DataFrame) -> pd.DataFrame:
    mask = np.ones(len(df), dtype=bool)
    if "motor_output_reason_name" in df.columns:
        mask &= df["motor_output_reason_name"].astype(str).eq("stabilized_mix").to_numpy()
    if "throttle_over_20" in df.columns:
        mask &= df["throttle_over_20"].to_numpy(dtype=float) >= 0.5
    if "ctrl_protection_flags" in df.columns:
        mask &= df["ctrl_protection_flags"].to_numpy(dtype=int) == 0
    if {"ctrl_tilt_out_rad_0", "ctrl_tilt_out_rad_1"}.issubset(df.columns):
        mask &= df["ctrl_tilt_out_rad_0"].abs().to_numpy() < 0.300
        mask &= df["ctrl_tilt_out_rad_1"].abs().to_numpy() < 0.300
    return df.loc[mask].copy()


def uniform_signal(t: np.ndarray, x: np.ndarray, fs: float = 100.0) -> tuple[np.ndarray, np.ndarray]:
    t0 = float(np.nanmin(t))
    t1 = float(np.nanmax(t))
    tu = np.arange(math.ceil(t0 * fs) / fs, math.floor(t1 * fs) / fs, 1.0 / fs)
    xu = np.interp(tu, t, x)
    tu = tu - tu[0]
    return tu, xu


def simulate_second_order(params: np.ndarray, t: np.ndarray, u: np.ndarray) -> np.ndarray:
    gain, log_wn, log_zeta, delay_s = params
    wn = math.exp(log_wn)
    zeta = math.exp(log_zeta)
    delayed_u = np.interp(t - delay_s, t, u, left=u[0], right=u[-1])
    num = [gain * wn * wn]
    den = [1.0, 2.0 * zeta * wn, wn * wn]
    _, y, _ = signal.lsim((num, den), U=delayed_u, T=t)
    return y


def fit_closed_loop(t: np.ndarray, u: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    u0 = u - np.mean(u)
    y0 = y - np.mean(y)
    if np.std(u0) < 1.0e-5 or np.std(y0) < 1.0e-5:
        raise ValueError("input or output excitation is too small")
    corr = float(np.corrcoef(u0, y0)[0, 1])
    gain0 = 1.0 if corr >= 0.0 else -1.0
    x0 = np.array([gain0, math.log(2.0 * math.pi * 0.6), math.log(1.0), 0.08])
    lower = np.array([-3.0, math.log(0.4), math.log(0.25), 0.0])
    upper = np.array([3.0, math.log(12.0), math.log(3.0), 0.40])

    def residual(x: np.ndarray) -> np.ndarray:
        return simulate_second_order(x, t, u0) - y0

    result = least_squares(residual, x0, bounds=(lower, upper), max_nfev=400)
    yhat = simulate_second_order(result.x, t, u0)
    err = yhat - y0
    fit_pct = 100.0 * (1.0 - np.linalg.norm(err) / max(np.linalg.norm(y0), 1.0e-9))
    return result.x, yhat, fit_pct


def dominant_hz(t: np.ndarray, y: np.ndarray) -> float:
    if len(t) < 64:
        return float("nan")
    fs = 1.0 / np.median(np.diff(t))
    yy = y - np.mean(y)
    freq = np.fft.rfftfreq(len(yy), d=1.0 / fs)
    amp = np.abs(np.fft.rfft(yy))
    keep = (freq >= 0.1) & (freq <= 5.0)
    if not np.any(keep):
        return float("nan")
    return float(freq[keep][np.argmax(amp[keep])])


def analyze(csv_path: Path, inertia_kg_m2: float, target_zeta: float) -> list[SegmentFit]:
    df = pd.read_csv(csv_path)
    required = {"ident_att_active", "ident_att_axis", "ident_att_signal_rad", "timestamp_us"}
    missing = sorted(required - set(df.columns))
    if missing:
        raise ValueError(f"CSV lacks IDENT ATT columns: {', '.join(missing)}")

    fits: list[SegmentFit] = []
    for start, end in split_segments(df):
        raw = df.iloc[start:end]
        axis_code = int(raw["ident_att_axis"].mode().iloc[0])
        axis = AXIS_NAMES.get(axis_code, "unknown")
        if axis not in AXIS_OUTPUT_DEG:
            continue
        seg = clean_segment(raw)
        if len(seg) < 200:
            continue
        t = seg["timestamp_us"].to_numpy(dtype=float) * 1.0e-6
        u = seg["ident_att_signal_rad"].to_numpy(dtype=float)
        y = np.deg2rad(seg[AXIS_OUTPUT_DEG[axis]].to_numpy(dtype=float))
        order = np.argsort(t)
        t, u, y = t[order], u[order], y[order]
        tu, uu = uniform_signal(t, u)
        _, yy = uniform_signal(t, y)
        params, yhat, fit_pct = fit_closed_loop(tu, uu, yy)
        gain, log_wn, log_zeta, delay_s = params
        wn = math.exp(log_wn)
        zeta = math.exp(log_zeta)
        # Use the identified closed-loop speed as the center suggestion and
        # request explicit damping. This maps to the firmware's moment PD layer.
        suggested_kr = inertia_kg_m2 * wn * wn
        suggested_kw = 2.0 * target_zeta * inertia_kg_m2 * wn
        rms_error_deg = math.degrees(float(np.sqrt(np.mean((yhat - (yy - np.mean(yy))) ** 2))))
        fits.append(
            SegmentFit(
                axis=axis,
                start_row=start,
                end_row=end - 1,
                rows=len(seg),
                duration_s=float(t[-1] - t[0]),
                fit_pct=float(fit_pct),
                gain=float(gain),
                wn_rad_s=float(wn),
                zeta=float(zeta),
                delay_s=float(delay_s),
                dominant_hz=dominant_hz(tu, yy),
                rms_error_deg=rms_error_deg,
                suggested_kr=float(suggested_kr),
                suggested_kw=float(suggested_kw),
                # Firmware stores the negative of UI-positive moment gains.
                param_kp=float(-suggested_kr),
                param_kd=float(-suggested_kw),
            )
        )
    return fits


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fit closed-loop IDENT ATT logs and suggest moment-layer PD gains."
    )
    parser.add_argument("csv", type=Path)
    parser.add_argument("--inertia", type=float, default=0.051, help="axis inertia in kg*m^2")
    parser.add_argument("--target-zeta", type=float, default=1.10)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    fits = analyze(args.csv, args.inertia, args.target_zeta)
    if not fits:
        raise SystemExit(
            "No usable IDENT ATT segments found. Run IDENT ARM then "
            "IDENT ATT PRBS roll|pitch amp_mdeg=800 bit_ms=250 duration_ms=60000."
        )

    payload = [asdict(fit) for fit in fits]
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    print(text)
    print("\nSuggested PARAM commands:")
    best_by_axis: dict[str, SegmentFit] = {}
    for fit in sorted(fits, key=lambda item: item.fit_pct, reverse=True):
        best_by_axis.setdefault(fit.axis, fit)
    for axis, fit in best_by_axis.items():
        print(
            f"PARAM SET coax.{axis}_angle_kp {fit.param_kp:.4f}  "
            f"# kr={fit.suggested_kr:.4f} N*m/rad, fit={fit.fit_pct:.1f}%"
        )
        print(
            f"PARAM SET coax.{axis}_rate_kd {fit.param_kd:.4f}   "
            f"# kw={fit.suggested_kw:.4f} N*m*s/rad"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
