"""Fit an offline near-level XY magnetometer heading candidate.

The result is a two-dimensional *relative-heading* candidate.  It is not a
three-axis hard/soft-iron calibration and must not be loaded as MAGCAL data.
It does not verify the physical chip-to-FLU mounting direction or accept the
magnetometer for flight.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


COLUMNS = ("x_flu_mgauss", "y_flu_mgauss", "z_flu_mgauss")


def load_flu(path: Path) -> np.ndarray:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if not set(COLUMNS) <= set(reader.fieldnames or ()):  # includes provenance frame
            raise ValueError(f"{path}: expected FLU mG X/Y/Z columns")
        data = np.asarray(
            [[float(row[key]) for key in COLUMNS] for row in reader], dtype=float
        )
    if data.ndim != 2 or data.shape[1] != 3 or len(data) < 20:
        raise ValueError(f"{path}: need at least 20 three-axis samples")
    if not np.all(np.isfinite(data)):
        raise ValueError(f"{path}: non-finite sample")
    return data


def fit_circle(data: np.ndarray) -> tuple[np.ndarray, float]:
    """Linear least-squares circle in the FLU body XY plane, mG units."""
    xy = data[:, :2]
    design = np.column_stack((2.0 * xy[:, 0], 2.0 * xy[:, 1], np.ones(len(xy))))
    rhs = np.sum(xy * xy, axis=1)
    parameters, _, rank, _ = np.linalg.lstsq(design, rhs, rcond=None)
    if rank != 3:
        raise ValueError("XY points do not determine a circle")
    cx, cy, constant = parameters
    radius_squared = constant + cx * cx + cy * cy
    if radius_squared <= 0.0:
        raise ValueError("fitted circle radius is invalid")
    return np.array([cx, cy]), math.sqrt(float(radius_squared))


def circle_metrics(data: np.ndarray, centre: np.ndarray, radius: float) -> dict[str, object]:
    xy = data[:, :2]
    radial_error = np.linalg.norm(xy - centre, axis=1) - radius
    heading = np.degrees(np.unwrap(np.arctan2(xy[:, 1] - centre[1], xy[:, 0] - centre[0])))
    sectors = np.floor((np.arctan2(xy[:, 1] - centre[1], xy[:, 0] - centre[0]) + math.pi) / (math.pi / 4.0)).astype(int) % 8
    return {
        "count": len(data),
        "radial_rms_mgauss": float(np.sqrt(np.mean(radial_error * radial_error))),
        "radial_p95_abs_mgauss": float(np.percentile(abs(radial_error), 95)),
        "heading_mean_deg": float(np.mean(heading)),
        "heading_std_deg": float(np.std(heading, ddof=1)),
        "heading_span_deg": float(np.ptp(heading)),
        "sectors_8": np.bincount(sectors, minlength=8).tolist(),
    }


def wrapped_heading_difference(xy: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    angle_a = np.arctan2(xy[:, 1] - a[1], xy[:, 0] - a[0])
    angle_b = np.arctan2(xy[:, 1] - b[1], xy[:, 0] - b[0])
    return np.degrees(np.angle(np.exp(1j * (angle_a - angle_b))))


def analyse(sweep: np.ndarray, later: np.ndarray, split: int = 200) -> dict[str, object]:
    if len(sweep) <= split + 20:
        raise ValueError("sweep needs enough samples before and after split")
    early_centre, early_radius = fit_circle(sweep[:split])
    centre, radius = fit_circle(sweep)
    delta = wrapped_heading_difference(sweep[:, :2], early_centre, centre)
    tail = circle_metrics(sweep[-20:], centre, radius)
    later_metrics = circle_metrics(later, centre, radius)
    mean_difference = (
        (later_metrics["heading_mean_deg"] - tail["heading_mean_deg"] + 180.0) % 360.0
    ) - 180.0
    return {
        "schema": "mag_heading_xy_candidate_v1",
        "status": "offline_candidate_only",
        "scope": "near_level_relative_heading_only",
        "frame": "FLU after default chip-to-body transform; physical axis unverified",
        "hard_iron_bias_xy_mgauss": centre.tolist(),
        "soft_iron_matrix_xy": [[1.0, 0.0], [0.0, 1.0]],
        "radius_xy_mgauss": radius,
        "full_sweep": circle_metrics(sweep, centre, radius),
        "first_200_fit": {
            "bias_xy_mgauss": early_centre.tolist(),
            "radius_xy_mgauss": early_radius,
            "training": circle_metrics(sweep[:split], early_centre, early_radius),
            "later_110": circle_metrics(sweep[split:], early_centre, early_radius),
        },
        "first_200_vs_full_310_heading_difference": {
            "median_abs_deg": float(np.median(abs(delta))),
            "p95_abs_deg": float(np.percentile(abs(delta), 95)),
            "max_abs_deg": float(np.max(abs(delta))),
        },
        "same_pose_tail_20": tail,
        "later_capture": later_metrics,
        "same_pose_mean_difference_deg": mean_difference,
        "compatible_with_existing_magcal_record": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep", type=Path, required=True)
    parser.add_argument("--later", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = analyse(load_flu(args.sweep), load_flu(args.later))
    result["inputs"] = {
        name: {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        for name, path in (("sweep", args.sweep), ("later", args.later))
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.output}")
    print("XY bias mG:", *(round(v, 3) for v in result["hard_iron_bias_xy_mgauss"]))
    print("XY radius mG:", round(result["radius_xy_mgauss"], 3))
    print("sweep radial RMS mG:", round(result["full_sweep"]["radial_rms_mgauss"], 3))
    print("later heading std deg:", round(result["later_capture"]["heading_std_deg"], 3))


if __name__ == "__main__":
    main()
