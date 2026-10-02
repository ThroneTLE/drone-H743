"""Offline trial of a latent two-level disturbance on magnetometer Z.

This is a diagnostic replay, not a flight estimator or calibration exporter.
It uses only the magnetometer's own samples; consequently its decoded state is
not independently observable.  A passing replay would still require a source
reference and physical validation before any use in firmware.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

try:
    from tools.mag_cal_fit import (
        FIRMWARE_FIELD_MAX_MGAUSS,
        FIRMWARE_FIELD_MIN_MGAUSS,
        MAX_USABLE_RMS_MGAUSS,
        fit_mag_calibration,
        fit_problems,
    )
except ModuleNotFoundError:  # Direct invocation: python tools/mag_dcdc_replay.py
    from mag_cal_fit import (
        FIRMWARE_FIELD_MAX_MGAUSS,
        FIRMWARE_FIELD_MIN_MGAUSS,
        MAX_USABLE_RMS_MGAUSS,
        fit_mag_calibration,
        fit_problems,
    )


AXIS_COLUMNS = (
    ("x_flu_mgauss", "y_flu_mgauss", "z_flu_mgauss"),
    ("x_mgauss", "y_mgauss", "z_mgauss"),
)


def load_capture(path: Path) -> np.ndarray:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        columns = next(
            (names for names in AXIS_COLUMNS if set(names) <= set(reader.fieldnames or ())),
            None,
        )
        if columns is None:
            raise ValueError(f"{path}: expected FLU X/Y/Z mG columns")
        samples = np.asarray(
            [[float(row[name]) for name in columns] for row in reader], dtype=float
        )
    if samples.ndim != 2 or samples.shape[1] != 3 or len(samples) == 0:
        raise ValueError(f"{path}: empty or malformed 3-axis capture")
    if not np.all(np.isfinite(samples)):
        raise ValueError(f"{path}: non-finite sample")
    return samples


def two_centres(values: np.ndarray) -> tuple[float, float, np.ndarray]:
    """Deterministic one-dimensional 2-means on the putative static Z series."""
    if len(values) < 20:
        raise ValueError("static capture needs at least 20 samples")
    centres = np.quantile(values, [0.25, 0.75]).astype(float)
    for _ in range(100):
        state = np.argmin(abs(values[:, None] - centres[None, :]), axis=1)
        counts = np.bincount(state, minlength=2)
        if np.min(counts) < 5:
            raise ValueError("static Z does not contain two supported levels")
        updated = np.array([np.mean(values[state == i]) for i in range(2)])
        if np.allclose(updated, centres, atol=1e-10, rtol=0.0):
            break
        centres = updated
    if centres[0] > centres[1]:
        centres = centres[::-1]
        state = 1 - state
    return float(centres[0]), float(centres[1]), state.astype(np.uint8)


def decode_offline(z: np.ndarray, delta: float) -> np.ndarray:
    """Best two-state sequence for Z smoothness, using the *whole* capture.

    This deliberately optimistic Viterbi replay can use future samples.  It
    therefore cannot be cited as proof of an implementable online correction.
    No switching penalty is used: the actual static trace switches frequently.
    """
    n = len(z)
    costs = np.zeros((n, 2), dtype=float)
    parent = np.zeros((n, 2), dtype=np.uint8)
    offsets = delta * np.array([0.0, 1.0])
    for i in range(1, n):
        previous = z[i - 1] - offsets
        current = z[i] - offsets
        edges = costs[i - 1, :, None] + (current[None, :] - previous[:, None]) ** 2
        parent[i] = np.argmin(edges, axis=0)
        costs[i] = np.min(edges, axis=0)
    state = np.empty(n, dtype=np.uint8)
    state[-1] = np.argmin(costs[-1])
    for i in range(n - 2, -1, -1):
        state[i] = parent[i + 1, state[i + 1]]
    return state


def decode_causal(z: np.ndarray, delta: float, initial_state: int) -> np.ndarray:
    """Choose each state from the previous corrected Z, without future data."""
    state = np.empty(len(z), dtype=np.uint8)
    state[0] = initial_state
    for i in range(1, len(z)):
        previous = z[i - 1] - delta * state[i - 1]
        candidates = abs(z[i] - delta * np.array([0.0, 1.0]) - previous)
        if math.isclose(float(candidates[0]), float(candidates[1]), abs_tol=1e-9):
            state[i] = state[i - 1]
        else:
            state[i] = int(np.argmin(candidates))
    return state


def correct_z(samples: np.ndarray, delta: float, state: np.ndarray) -> np.ndarray:
    corrected = samples.copy()
    corrected[:, 2] -= delta * state
    return corrected


def median_z_offline(samples: np.ndarray, width: int) -> np.ndarray:
    """Centered median control: also optimistic because it sees future samples."""
    radius = width // 2
    corrected = samples.copy()
    for i in range(len(samples)):
        corrected[i, 2] = np.median(
            samples[max(0, i - radius) : min(len(samples), i + radius + 1), 2]
        )
    return corrected


def fit_summary(train: np.ndarray, holdout: np.ndarray | None) -> dict[str, object]:
    try:
        fit = fit_mag_calibration(train)
    except ValueError as exc:
        return {"fit_error": str(exc), "acceptable": False}

    result: dict[str, object] = {
        "radius_mgauss": fit.fitted_radius_mgauss,
        "train_rms_mgauss": fit.corrected_rms_mgauss,
        "train_over_16_count": int(
            np.count_nonzero(
                abs(
                    np.linalg.norm(
                        (train - np.asarray(fit.hard_iron_offset_mgauss))
                        @ np.asarray(fit.soft_iron_matrix),
                        axis=1,
                    )
                    - fit.fitted_radius_mgauss
                ) > MAX_USABLE_RMS_MGAUSS
            )
        ),
        "fit_problems": list(fit_problems(fit)),
    }
    result["acceptable"] = not result["fit_problems"]
    if holdout is not None:
        norms = np.linalg.norm(
            (holdout - np.asarray(fit.hard_iron_offset_mgauss))
            @ np.asarray(fit.soft_iron_matrix),
            axis=1,
        )
        error = norms - fit.fitted_radius_mgauss
        result["holdout_rms_mgauss"] = float(np.sqrt(np.mean(error * error)))
        result["holdout_p95_abs_mgauss"] = float(np.percentile(abs(error), 95))
        result["holdout_over_16_count"] = int(
            np.count_nonzero(abs(error) > MAX_USABLE_RMS_MGAUSS)
        )
        result["holdout_acceptable"] = (
            result["acceptable"]
            and result["holdout_rms_mgauss"] <= MAX_USABLE_RMS_MGAUSS
        )
    return result


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def slow_yaw_proxy(static: np.ndarray, yaw_circle: np.ndarray, delta: float) -> dict[str, object]:
    """Geometry-only noise estimate; not an AHRS replay or heading calibration.

    The horizontal radius comes from an algebraic XY circle on the separate
    yaw-only capture.  A body-Z error projects into horizontal heading by
    sin(tilt); this reports the perpendicular worst-orientation angle scale.
    The static capture has no timestamps, so windows are in samples, not time.
    """
    xy = yaw_circle[:, :2]
    design = np.column_stack((2.0 * xy[:, 0], 2.0 * xy[:, 1], np.ones(len(xy))))
    cx, cy, constant = np.linalg.lstsq(design, np.sum(xy * xy, axis=1), rcond=None)[0]
    horizontal_radius = math.sqrt(float(constant + cx * cx + cy * cy))
    radial_error = np.linalg.norm(xy - np.array([cx, cy]), axis=1) - horizontal_radius
    def heading_stats(samples: np.ndarray) -> dict[str, float]:
        angle = np.degrees(
            np.unwrap(np.arctan2(samples[:, 1] - cy, samples[:, 0] - cx))
        )
        return {
            "mean_deg": float(np.mean(angle)),
            "std_deg": float(np.std(angle, ddof=1)),
            "span_deg": float(np.ptp(angle)),
        }

    held_pose_tail = heading_stats(yaw_circle[-20:])
    static_heading = heading_stats(static)
    repeat_mean_difference = (
        (static_heading["mean_deg"] - held_pose_tail["mean_deg"] + 180.0) % 360.0
    ) - 180.0
    tilts = (0, 10, 30, 45)
    windows: dict[str, object] = {}
    for count in (1, 3, 5, 10, 20):
        averaged = np.convolve(static[:, 2], np.ones(count) / count, mode="valid")
        sigma = float(np.std(averaged, ddof=1))
        windows[str(count)] = {
            "z_std_mgauss": sigma,
            "heading_one_sigma_deg": {
                str(tilt): math.degrees(
                    math.atan2(sigma * math.sin(math.radians(tilt)), horizontal_radius)
                )
                for tilt in tilts
            },
        }
    return {
        "xy_circle_centre_mgauss": [float(cx), float(cy)],
        "xy_circle_radius_mgauss": horizontal_radius,
        "xy_circle_radial_rms_mgauss": float(np.sqrt(np.mean(radial_error**2))),
        "yaw_capture_tail_20_heading": held_pose_tail,
        "later_96_heading": static_heading,
        "held_pose_mean_difference_deg": repeat_mean_difference,
        "windows_in_samples": windows,
        "missed_step_heading_deg": {
            str(tilt): math.degrees(
                math.atan2(delta * math.sin(math.radians(tilt)), horizontal_radius)
            )
            for tilt in tilts
        },
        "interpretation": "measurement angle scale only; no timestamped IMU or AHRS yaw replay",
    }


def run(static_path: Path, train_path: Path, holdout_path: Path) -> dict[str, object]:
    static = load_capture(static_path)
    train = load_capture(train_path)
    holdout = load_capture(holdout_path)
    low, high, static_state = two_centres(static[:, 2])
    delta = high - low
    static_corrected = static[:, 2] - delta * static_state

    result: dict[str, object] = {
        "inputs": {
            name: {"path": str(path.resolve()), "sha256": digest(path), "count": len(data)}
            for name, path, data in (
                ("static", static_path, static),
                ("train", train_path, train),
                ("holdout", holdout_path, holdout),
            )
        },
        "existing_limits": {
            "field_mgauss": [FIRMWARE_FIELD_MIN_MGAUSS, FIRMWARE_FIELD_MAX_MGAUSS],
            "rms_mgauss": MAX_USABLE_RMS_MGAUSS,
        },
        "static": {
            "z_level_low_mgauss": low,
            "z_level_high_mgauss": high,
            "estimated_delta_mgauss": delta,
            "counts": np.bincount(static_state, minlength=2).tolist(),
            "raw_z_std_mgauss": float(np.std(static[:, 2], ddof=1)),
            "corrected_z_std_mgauss": float(np.std(static_corrected, ddof=1)),
        },
        "slow_yaw_proxy": slow_yaw_proxy(static, holdout, delta),
        "methods": {"raw": fit_summary(train, holdout)},
    }
    for name, decoder in (
        ("offline_full_capture", decode_offline),
        ("causal_initial_low", lambda z, d: decode_causal(z, d, 0)),
        ("causal_initial_high", lambda z, d: decode_causal(z, d, 1)),
    ):
        train_state = decoder(train[:, 2], delta)
        holdout_state = decoder(holdout[:, 2], delta)
        corrected_train = correct_z(train, delta, train_state)
        corrected_holdout = correct_z(holdout, delta, holdout_state)
        result["methods"][name] = {
            "train_state_counts": np.bincount(train_state, minlength=2).tolist(),
            "holdout_state_counts": np.bincount(holdout_state, minlength=2).tolist(),
            **fit_summary(corrected_train, corrected_holdout),
        }
    for width in (3, 5):
        result["methods"][f"centered_median_{width}"] = fit_summary(
            median_z_offline(train, width), median_z_offline(holdout, width)
        )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--static", type=Path, required=True)
    parser.add_argument("--train", type=Path, required=True)
    parser.add_argument("--holdout", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run(args.static, args.train, args.holdout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.output}")
    print(f"estimated Z step: {result['static']['estimated_delta_mgauss']:.2f} mG")
    for name, outcome in result["methods"].items():
        print(
            name,
            "fit_error=" + str(outcome.get("fit_error", "-")),
            "train_rms=" + str(outcome.get("train_rms_mgauss", "-")),
            "holdout_rms=" + str(outcome.get("holdout_rms_mgauss", "-")),
            "acceptable=" + str(outcome["acceptable"]),
        )


if __name__ == "__main__":
    main()
