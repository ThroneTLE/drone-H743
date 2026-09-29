"""Small bounded smart-scan plans; no voltage labels or model fitting here."""
from __future__ import annotations

import math
from collections.abc import Iterable, Mapping

from .plans import ExperimentPlan, PlanPoint

COMMAND_TOLERANCE_PCT = 2.0
VOLTAGE_TOLERANCE_V = 0.15


def build_smart_plan(kind: str, max_percent: float, covered_points=(), *,
                     current_voltage_v: float | None = None) -> ExperimentPlan:
    kind = str(kind).strip().lower()
    maximum = float(max_percent)
    if kind not in {"quick", "standard", "supplement"}:
        raise ValueError("kind must be quick, standard, or supplement")
    if not math.isfinite(maximum) or not 1.0 <= maximum <= 100.0:
        raise ValueError("max_percent must be finite and in 1..100")
    candidates = _quick_targets(maximum) if kind == "quick" else _standard_targets(maximum)
    fallback = False
    if kind == "supplement":
        covered = tuple(_valid_covered(covered_points))
        if not covered or not _finite(current_voltage_v):
            fallback = True
        else:
            candidates = tuple(target for target in candidates
                               if not _covered(target, covered,
                                               float(current_voltage_v)))
            if not candidates:
                # Recheck one central common point instead of claiming that a
                # previous data set makes a new run unnecessary.
                candidates = ((round(maximum * 0.5, 3),
                               round(maximum * 0.5, 3)),)
    points = tuple(PlanPoint(
        segment_id=f"smart-{kind}-{index:02d}",
        upper_percent=upper, lower_percent=lower,
        phase="steady", direction="steady",
        settle_s=0.25, duration_s=0.8,
        adaptive=True, max_wait_s=4.0, stable_window_s=0.4,
    ) for index, (upper, lower) in enumerate(candidates))
    suffix = "-fallback-standard" if fallback else ""
    reference = (float(current_voltage_v)
                 if _finite(current_voltage_v) else None)
    return ExperimentPlan(
        f"smart-{kind}{suffix}", "dual", points,
        reference_voltage_v=reference,
        voltage_tolerance_v=VOLTAGE_TOLERANCE_V if reference is not None else 0.0)


def _quick_targets(maximum: float) -> tuple[tuple[float, float], ...]:
    """Five link/response checks; deliberately insufficient for model training."""
    low = round(maximum * 0.35, 3)
    middle = round(maximum * 0.55, 3)
    high = round(maximum * 0.75, 3)
    return ((low, low), (high, high), (middle, low),
            (low, middle), (maximum, maximum))


def _standard_targets(maximum: float) -> tuple[tuple[float, float], ...]:
    a, b, c, d = (round(maximum * fraction, 3)
                  for fraction in (0.25, 0.5, 0.75, 1.0))
    return ((a, a), (b, b), (c, c), (d, d),
            (a, d), (d, a), (a, c), (c, a),
            (b, d), (d, b), (a, b), (b, a),
            (b, c), (c, b), (c, d), (d, c))


def _valid_covered(points: Iterable[Mapping]) -> Iterable[tuple[float, float, float]]:
    for point in points:
        try:
            upper = float(point["upper_command_pct"])
            lower = float(point["lower_command_pct"])
            voltage = float(point["voltage_v"])
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in (upper, lower, voltage)):
            yield upper, lower, voltage


def _covered(target: tuple[float, float], covered, voltage: float) -> bool:
    upper, lower = target
    return any(abs(upper - old_upper) <= COMMAND_TOLERANCE_PCT
               and abs(lower - old_lower) <= COMMAND_TOLERANCE_PCT
               and abs(voltage - old_voltage) <= VOLTAGE_TOLERANCE_V
               for old_upper, old_lower, old_voltage in covered)


def _finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False
