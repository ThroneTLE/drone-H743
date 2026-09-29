"""Thrust-vs-throttle report view.

The trained model grouped by battery charge.

The model maps measured eRPM/V to thrust; battery voltage mostly acts through
the throttle->eRPM step, so this view shows it on the throttle axis. Only
points with both propellers at the same throttle form a single curve.
"""
from __future__ import annotations

import math
from collections import defaultdict
from collections.abc import Mapping
from typing import Any

from .dataset_model import predict_dataset_thrust
from .sweep_schedule import CHARGE_BAND_V, SAG_K_DEFAULT, load_index

BALANCED_TOLERANCE_PCT = 0.6
MAX_BANDS = 5  # The ordinal blue ramp keeps five visibly distinct steps.
GRAMS_PER_NEWTON = 1000.0 / 9.80665


def _finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def charge_voltage(point: Mapping[str, Any]) -> float:
    """Loaded voltage plus estimated sag: the same basis as auto collection."""
    return float(point["voltage_v"]) + SAG_K_DEFAULT * load_index(
        float(point["upper_erpm"]), float(point["lower_erpm"]))


def _band_floor(charge: float, width: float) -> float:
    return round(math.floor(charge / width + 1e-9) * width, 3)


def throttle_curves(analysis: Mapping[str, Any]) -> dict[str, Any]:
    fields = ("upper_command_pct", "lower_command_pct", "upper_erpm", "lower_erpm",
              "voltage_v", "thrust_n")
    points = []
    total = 0
    for role, key in (("train", "train_operating_points"),
                      ("validation", "validation_operating_points")):
        for point in analysis.get("data", {}).get(key, []):
            total += 1
            if not all(_finite(point.get(name)) for name in fields):
                continue
            upper, lower = float(point["upper_command_pct"]), float(point["lower_command_pct"])
            if abs(upper - lower) > BALANCED_TOLERANCE_PCT or min(upper, lower) <= 0:
                continue
            try:
                predicted = predict_dataset_thrust(
                    analysis, upper_erpm=float(point["upper_erpm"]),
                    lower_erpm=float(point["lower_erpm"]), voltage_v=float(point["voltage_v"]))
            except ValueError:
                predicted = None  # Outside the training domain or no model: never extrapolate.
            points.append({"role": role, "throttle_pct": round((upper + lower) / 2, 1),
                           "charge_v": charge_voltage(point), "loaded_v": float(point["voltage_v"]),
                           "measured_gf": float(point["thrust_n"]) * GRAMS_PER_NEWTON,
                           "predicted_gf": None if predicted is None else predicted * GRAMS_PER_NEWTON})
    width = CHARGE_BAND_V
    while points and len({_band_floor(p["charge_v"], width) for p in points}) > MAX_BANDS:
        width *= 2
    grouped: dict[float, list[dict[str, Any]]] = defaultdict(list)
    for point in points:
        grouped[_band_floor(point["charge_v"], width)].append(point)
    bands = []
    for low in sorted(grouped):
        members = sorted(grouped[low], key=lambda p: p["throttle_pct"])
        levels: dict[float, list[dict[str, Any]]] = defaultdict(list)
        for point in members:
            levels[round(point["throttle_pct"] * 2) / 2].append(point)
        rows = []
        for throttle in sorted(levels):
            items = levels[throttle]
            covered = [p for p in items if p["predicted_gf"] is not None]
            mean = lambda values: sum(values) / len(values) if values else None
            rows.append({"throttle_pct": throttle, "count": len(items),
                         "measured_gf": mean([p["measured_gf"] for p in items]),
                         "predicted_gf": mean([p["predicted_gf"] for p in covered]),
                         "error_gf": mean([p["predicted_gf"] - p["measured_gf"] for p in covered])})
        bands.append({"low_v": low, "high_v": round(low + width, 3),
                      "label": f"{low:.1f}–{low + width:.1f} V", "points": members, "levels": rows})
    return {"band_width_v": round(width, 3), "sag_k": SAG_K_DEFAULT,
            "balanced_points": len(points), "total_points": total,
            "predicted_points": sum(p["predicted_gf"] is not None for p in points), "bands": bands}
