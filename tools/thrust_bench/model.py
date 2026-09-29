"""Offline models for the installed coaxial thrust-bench assembly.

The response is total measured thrust.  Both propellers remain installed, so
the coefficients must not be interpreted as isolated upper/lower propeller
thrust or torque.  This module performs no I/O and never emits controller
tables or commands.
"""
from __future__ import annotations

from collections import Counter, defaultdict
from collections.abc import Mapping, Sequence
import math
from typing import Any

import numpy as np

from .records import BenchSample
from .current_sources import dshot_input_power, valid_current

ANALYSIS_SCHEMA_VERSION = 2
_MIN_STEADY_SAMPLES = 3
_MAX_CONDITION = 1.0e8


def _finite(value: object) -> bool:
    try:
        return value is not None and math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _mean(values: Sequence[float]) -> float:
    ordered = sorted(float(value) for value in values)
    cut = int(len(ordered) * 0.1)
    core = ordered[cut:len(ordered) - cut] if cut else ordered
    return float(sum(core) / len(core))


def _metadata_number(metadata: Mapping[str, Any], key: str, default: float) -> float:
    value = metadata.get(key, default)
    return float(value) if _finite(value) else default


def _measured_erpm(sample: BenchSample) -> bool:
    return sample.speed_source.strip().lower() == "dshot_erpm"


def _sample_reasons(sample: BenchSample, metadata: Mapping[str, Any]) -> list[str]:
    # Current diagnostics do not invalidate thrust/eRPM/scale observations.
    diagnostic_prefixes = ("upper_esc_current_", "lower_esc_current_",
                           "esc_current_", "board_current_")
    # A single-drive installed-assembly model only requires active eRPM. The
    # passive propeller may windmill without a speed observation; never fake 0.
    if sample.mode == "upper":
        diagnostic_prefixes += ("lower_erpm_",)
    elif sample.mode == "lower":
        diagnostic_prefixes += ("upper_erpm_",)
    reasons = [reason for reason in sample.quality
               if not reason.startswith(diagnostic_prefixes)]
    if sample.mode not in {"dual", "upper", "lower"}:
        reasons.append("unsupported_mode")
    max_erpm_age = _metadata_number(metadata, "max_erpm_age_ms", 150.0)
    max_electrical_age = _metadata_number(metadata, "max_electrical_age_ms", 150.0)
    max_scale_skew = _metadata_number(metadata, "max_scale_skew_s", 0.20)
    required = [sample.host_time_s, sample.scale_time_s, sample.thrust_n]
    if sample.mode in {"dual", "upper"}:
        required.append(sample.upper_erpm)
    if sample.mode in {"dual", "lower"}:
        required.append(sample.lower_erpm)
    if not all(_finite(value) for value in required):
        reasons.append("missing_static_measurement")
    if (_finite(sample.upper_erpm) and float(sample.upper_erpm) < 0.0) or (
            _finite(sample.lower_erpm) and float(sample.lower_erpm) < 0.0):
        reasons.append("negative_erpm")
    if not _measured_erpm(sample):
        reasons.append("erpm_not_measured")
    ages = []
    if sample.mode in {"dual", "upper"}:
        ages.append(sample.upper_erpm_age_ms)
    if sample.mode in {"dual", "lower"}:
        ages.append(sample.lower_erpm_age_ms)
    for age in ages:
        if age is None or age < 0 or age > max_erpm_age:
            reasons.append("erpm_stale_or_age_missing")
            break
    if (not _finite(sample.host_time_s) or not _finite(sample.scale_time_s) or
            abs(float(sample.host_time_s) - float(sample.scale_time_s)) > max_scale_skew):
        reasons.append("scale_not_synchronised")
    if sample.voltage_v is not None and (sample.voltage_age_ms is None or sample.voltage_age_ms < 0 or
                                         sample.voltage_age_ms > max_electrical_age):
        reasons.append("voltage_stale_or_age_missing")
    return sorted(set(reasons))


def _aligned_response_point(group: Sequence[BenchSample], mode: str,
                            response: str, metadata: Mapping[str, Any]) -> dict[str, Any] | None:
    """Aggregate response and regressors from the exact same aligned samples."""
    max_current_age = _metadata_number(metadata, "max_esc_current_age_ms", 1000.0)
    max_voltage_age = _metadata_number(metadata, "max_voltage_age_ms", 250.0)
    max_skew = _metadata_number(metadata, "max_power_source_skew_ms", 50.0)
    selected: list[tuple[BenchSample, float]] = []
    for item in group:
        if (item.current_source != "dshot" or not _finite(item.voltage_v) or
                item.voltage_age_ms is None or item.voltage_age_ms < 0 or
                item.voltage_age_ms > max_voltage_age):
            continue
        required_roles = ("upper", "lower") if mode == "dual" else (mode,)
        ages = [float(item.voltage_age_ms)]
        valid_regressors = True
        for role in required_roles:
            erpm = getattr(item, f"{role}_erpm")
            age = getattr(item, f"{role}_erpm_age_ms")
            if not _finite(erpm) or age is None or age < 0:
                valid_regressors = False
                break
            ages.append(float(age))
        if not valid_regressors:
            continue
        if response in {"upper_esc_current_a", "lower_esc_current_a"}:
            role = response.split("_", 1)[0]
            value = getattr(item, response)
            age = getattr(item, f"{role}_esc_current_age_ms")
            if not valid_current(value, age, max_age_ms=max_current_age):
                continue
            ages.append(float(age))
            response_value = float(value)
        else:
            if not (valid_current(item.upper_esc_current_a, item.upper_esc_current_age_ms,
                                  max_age_ms=max_current_age) and
                    valid_current(item.lower_esc_current_a, item.lower_esc_current_age_ms,
                                  max_age_ms=max_current_age)):
                continue
            ages.extend((float(item.upper_esc_current_age_ms),
                         float(item.lower_esc_current_age_ms)))
            response_value = dshot_input_power(
                item.voltage_v, item.voltage_age_ms,
                item.upper_esc_current_a, item.lower_esc_current_a,
                item.upper_esc_current_age_ms, item.lower_esc_current_age_ms,
                max_age_ms=max_current_age, max_skew_ms=max_skew,
                max_voltage_age_ms=max_voltage_age)
            if response_value is None:
                continue
        if max(ages) - min(ages) > max_skew:
            continue
        selected.append((item, float(response_value)))
    if not selected:
        return None
    samples = [item for item, _ in selected]
    voltages = [float(item.voltage_v) for item in samples]
    result = {
        "run_id": samples[0].run_id, "segment_id": samples[0].segment_id,
        "mode": mode, "direction": samples[0].direction,
        "sample_count": len(samples), "voltage_v": _mean(voltages),
        "voltage_range_v": [min(voltages), max(voltages)],
        "voltage_layer_v": next((float(item.voltage_layer_v) for item in samples
                                  if _finite(item.voltage_layer_v)), None),
        response: _mean([value for _, value in selected]),
    }
    for role in ("upper", "lower"):
        values = [float(getattr(item, f"{role}_erpm")) for item in samples
                  if _finite(getattr(item, f"{role}_erpm"))]
        result[f"{role}_erpm"] = _mean(values) if values else None
    return result


def _steady_points(samples: Sequence[BenchSample], metadata: Mapping[str, Any]) -> tuple[list[dict[str, Any]], dict[str, int]]:
    groups: dict[tuple[str, str], list[BenchSample]] = defaultdict(list)
    excluded: Counter[str] = Counter()
    for sample in samples:
        if sample.phase != "steady":
            continue
        reasons = _sample_reasons(sample, metadata)
        if reasons:
            excluded.update(reasons)
            continue
        groups[(sample.run_id, sample.segment_id)].append(sample)

    points: list[dict[str, Any]] = []
    max_thrust_cv = _metadata_number(metadata, "max_steady_thrust_cv", 0.05)
    max_erpm_cv = _metadata_number(metadata, "max_steady_erpm_cv", 0.02)
    max_command_span = _metadata_number(metadata, "max_steady_command_span_pct", 0.5)
    for (run_id, segment_id), group in sorted(groups.items()):
        if len(group) < _MIN_STEADY_SAMPLES:
            excluded["too_few_steady_samples"] += len(group)
            continue
        if len({item.mode for item in group}) != 1:
            excluded["mixed_mode_within_segment"] += len(group)
            continue
        thrusts = [float(item.thrust_n) for item in group]
        thrust = _mean(thrusts)
        spread = float(np.std(thrusts, ddof=1)) if len(thrusts) > 1 else 0.0
        if abs(thrust) > 0.05 and spread / abs(thrust) > max_thrust_cv:
            excluded["unsteady_thrust"] += len(group)
            continue
        mode = group[0].mode
        inactive_field = "lower_command_pct" if mode == "upper" else (
            "upper_command_pct" if mode == "lower" else None)
        if inactive_field is not None and any(
                _finite(getattr(item, inactive_field)) and
                float(getattr(item, inactive_field)) > 0.05 for item in group):
            excluded["single_drive_inactive_command_nonzero"] += len(group)
            continue
        if mode == "dual":
            fake_boundaries = [
                any(_finite(getattr(item, command)) and float(getattr(item, command)) <= 0.05 and
                    _finite(getattr(item, erpm)) and float(getattr(item, erpm)) == 0.0
                    for item in group)
                for command, erpm in (("upper_command_pct", "upper_erpm"),
                                     ("lower_command_pct", "lower_erpm"))]
            if any(fake_boundaries):
                excluded["dual_undriven_zero_erpm_boundary"] += len(group)
                continue
        erpm_fields = (["upper_erpm", "lower_erpm"] if mode == "dual" else
                      ["upper_erpm"] if mode == "upper" else ["lower_erpm"])
        erpm_values = [[float(getattr(item, field)) for item in group] for field in erpm_fields]
        if any(np.std(values, ddof=1) / max(abs(np.mean(values)), 1.0) > max_erpm_cv
               for values in erpm_values):
            excluded["unsteady_erpm"] += len(group)
            continue
        commands = [[float(getattr(item, name)) for item in group if _finite(getattr(item, name))]
                    for name in ("upper_command_pct", "lower_command_pct")]
        if any(values and np.ptp(values) > max_command_span for values in commands):
            excluded["unsteady_command"] += len(group)
            continue
        upper_updates = {int(item.fc_time_ms) - int(item.upper_erpm_age_ms)
                         for item in group if item.fc_time_ms is not None and
                         item.upper_erpm_age_ms is not None}
        lower_updates = {int(item.fc_time_ms) - int(item.lower_erpm_age_ms)
                         for item in group if item.fc_time_ms is not None and
                         item.lower_erpm_age_ms is not None}
        scale_updates = {float(item.scale_time_s) for item in group}
        if (((mode in {"dual", "upper"}) and len(upper_updates) < _MIN_STEADY_SAMPLES) or
                ((mode in {"dual", "lower"}) and len(lower_updates) < _MIN_STEADY_SAMPLES) or
                len(scale_updates) < _MIN_STEADY_SAMPLES):
            excluded["too_few_independent_source_updates"] += len(group)
            continue
        voltage_values = [float(item.voltage_v) for item in group if _finite(item.voltage_v)]
        response_points = {
            response: _aligned_response_point(group, mode, response, metadata)
            for response in ("upper_esc_current_a", "lower_esc_current_a", "input_power_w")
        }
        board_values = [float(item.board_current_a) for item in group
                        if _finite(item.board_current_a)]
        points.append({
            "run_id": run_id,
            "segment_id": segment_id,
            "direction": group[0].direction,
            "mode": group[0].mode,
            "sample_count": len(group),
            "upper_erpm": _mean([float(item.upper_erpm) for item in group
                                  if _finite(item.upper_erpm)])
                                  if any(_finite(item.upper_erpm) for item in group) else None,
            "lower_erpm": _mean([float(item.lower_erpm) for item in group
                                  if _finite(item.lower_erpm)])
                                  if any(_finite(item.lower_erpm) for item in group) else None,
            "thrust_n": thrust,
            "thrust_std_n": spread,
            "voltage_v": _mean(voltage_values) if voltage_values else None,
            "voltage_range_v": [min(voltage_values), max(voltage_values)] if voltage_values else None,
            "response_points": response_points,
            "esc_current_calibrated": bool(group) and all(item.esc_current_calibrated for item in group),
            "board_current_a_diagnostic": _mean(board_values) if board_values else None,
            "board_current_calibrated": bool(board_values) and all(
                item.board_current_calibrated for item in group if _finite(item.board_current_a)),
            "voltage_layer_v": next((float(item.voltage_layer_v) for item in group
                                     if _finite(item.voltage_layer_v)), None),
            "upper_command_pct": _mean([float(item.upper_command_pct) for item in group
                                         if _finite(item.upper_command_pct)])
                                         if any(_finite(item.upper_command_pct) for item in group) else None,
            "lower_command_pct": _mean([float(item.lower_command_pct) for item in group
                                         if _finite(item.lower_command_pct)])
                                         if any(_finite(item.lower_command_pct) for item in group) else None,
        })
    return points, dict(sorted(excluded.items()))


def _design(points: Sequence[Mapping[str, Any]], power: bool = False,
            erpm_scale: float | None = None) -> tuple[np.ndarray, float]:
    scale = erpm_scale or max(max(float(p["upper_erpm"]), float(p["lower_erpm"])) for p in points)
    scale = max(scale, 1.0)
    u = np.asarray([float(point["upper_erpm"]) / scale for point in points])
    l = np.asarray([float(point["lower_erpm"]) / scale for point in points])
    if power:
        matrix = np.column_stack((np.ones(len(points)), u ** 3, l ** 3,
                                  u * l * (u + l)))
    else:
        matrix = np.column_stack((np.ones(len(points)), u ** 2, l ** 2, u * l))
    return matrix, scale


def _coverage(points: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    upper = np.asarray([float(point["upper_erpm"]) for point in points])
    lower = np.asarray([float(point["lower_erpm"]) for point in points])
    if not len(points):
        return {"points": 0, "runs": [], "voltage_layers_v": [],
                "upper_erpm": None, "lower_erpm": None,
                "erpm_correlation": None, "independently_varied": False}
    ratio = upper / np.maximum(lower, 1.0)
    corr = float(np.corrcoef(upper, lower)[0, 1]) if len(points) > 2 and np.std(upper) and np.std(lower) else None
    independently_varied = (np.ptp(upper) > 100.0 and np.ptp(lower) > 100.0 and
                            np.ptp(ratio) > 0.10 and
                            (corr is None or abs(corr) < 0.995))
    return {
        "points": len(points),
        "runs": sorted({str(point["run_id"]) for point in points}),
        "voltage_layers_v": sorted({float(point["voltage_layer_v"]) for point in points
                                     if point.get("voltage_layer_v") is not None}),
        "upper_erpm": [float(np.min(upper)), float(np.max(upper))] if len(upper) else None,
        "lower_erpm": [float(np.min(lower)), float(np.max(lower))] if len(lower) else None,
        "erpm_correlation": corr,
        "independently_varied": bool(independently_varied),
    }


def _metrics(actual: np.ndarray, predicted: np.ndarray) -> dict[str, float | None]:
    error = predicted - actual
    rmse = float(np.sqrt(np.mean(error ** 2)))
    mae = float(np.mean(np.abs(error)))
    reference = float(np.mean(np.abs(actual)))
    return {
        "rmse_n": rmse,
        "mae_n": mae,
        "relative_mae": mae / reference if reference > 1.0e-9 else None,
        "reference_thrust_n": reference,
        "max_abs_error_n": float(np.max(np.abs(error))),
        "p95_abs_error_n": float(np.percentile(np.abs(error), 95)),
    }


def _fit_static(points: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    coverage = _coverage(points)
    if len(points) < 6:
        return {"status": "unavailable", "reason": "fewer_than_6_steady_operating_points",
                "coverage": coverage}
    design, scale = _design(points)
    rank = int(np.linalg.matrix_rank(design))
    condition = float(np.linalg.cond(design))
    condition_json = condition if math.isfinite(condition) else None
    if not coverage["independently_varied"]:
        return {"status": "unavailable", "reason": "inputs_not_independently_varied",
                "coverage": coverage, "rank": rank, "condition_number": condition_json}
    if rank < design.shape[1] or not math.isfinite(condition) or condition > _MAX_CONDITION:
        return {"status": "unavailable", "reason": "rank_deficient_or_ill_conditioned",
                "coverage": coverage, "rank": rank, "condition_number": condition_json}
    actual = np.asarray([float(point["thrust_n"]) for point in points])
    coefficients, _, _, _ = np.linalg.lstsq(design, actual, rcond=None)
    predicted = design @ coefficients
    residual_by_layer: dict[str, dict[str, float | int | None]] = {}
    grouped_indices: dict[str, list[int]] = defaultdict(list)
    for index, point in enumerate(points):
        key = "unlayered" if point.get("voltage_layer_v") is None else f"{float(point['voltage_layer_v']):g}"
        grouped_indices[key].append(index)
    for layer, indices in sorted(grouped_indices.items()):
        residual_by_layer[layer] = {"points": len(indices),
                                    **_metrics(actual[indices], predicted[indices])}
    return {
        "status": "available",
        "model": {
            "kind": "installed_coax_total_thrust_erpm2",
            "equation": "F_N=c0+c_upper*(upper_erpm/erpm_scale)^2+c_lower*(lower_erpm/erpm_scale)^2+c_cross*(upper_erpm*lower_erpm/erpm_scale^2)",
            "erpm_scale": float(scale),
            "coefficients_n": {key: float(value) for key, value in zip(
                ("intercept", "upper_squared", "lower_squared", "cross"), coefficients)},
        },
        "coverage": coverage,
        "rank": rank,
        "condition_number": condition_json,
        "training_metrics": _metrics(actual, predicted),
        "residuals_by_voltage_layer": residual_by_layer,
    }


def _voltage_groups(points: Sequence[Mapping[str, Any]]) -> list[tuple[str, list[Mapping[str, Any]]]]:
    groups: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for point in points:
        if not _finite(point.get("voltage_v")):
            continue
        label = point.get("voltage_layer_v")
        key = f"layer:{float(label):g}" if _finite(label) else f"run:{point['run_id']}"
        groups[key].append(point)
    return sorted(groups.items(), key=lambda item: np.mean([float(p["voltage_v"]) for p in item[1]]))


def _fit_single_erpm(points: Sequence[Mapping[str, Any]], mode: str) -> dict[str, Any]:
    field = f"{mode}_erpm"
    usable = [point for point in points if _finite(point.get(field))]
    if len(usable) < 4:
        return {"status": "unavailable", "reason": "fewer_than_4_single_drive_points"}
    erpm = np.asarray([float(point[field]) for point in usable])
    if np.ptp(erpm) <= 100.0:
        return {"status": "unavailable", "reason": "insufficient_active_erpm_span"}
    scale = max(float(np.max(erpm)), 1.0)
    design = np.column_stack((np.ones(len(erpm)), (erpm / scale) ** 2))
    rank = int(np.linalg.matrix_rank(design))
    condition = float(np.linalg.cond(design))
    if rank < 2 or not math.isfinite(condition) or condition > _MAX_CONDITION:
        return {"status": "unavailable", "reason": "rank_deficient_or_ill_conditioned",
                "rank": rank, "condition_number": condition if math.isfinite(condition) else None}
    actual = np.asarray([float(point["thrust_n"]) for point in usable])
    coefficients, _, _, _ = np.linalg.lstsq(design, actual, rcond=None)
    return {"status": "available", "model": {"kind": "installed_single_drive_total_thrust_erpm2",
            "mode": mode, "erpm_scale": scale,
            "coefficients_n": {"intercept": float(coefficients[0]),
                               "active_squared": float(coefficients[1])}},
            "coverage": {"active_erpm": [float(np.min(erpm)), float(np.max(erpm))],
                         "points": len(usable)},
            "training_metrics": _metrics(actual, design @ coefficients)}


def _layer_model(points: Sequence[Mapping[str, Any]], mode: str) -> dict[str, Any]:
    fitted = _fit_static(points) if mode == "dual" else _fit_single_erpm(points, mode)
    voltages = [float(point["voltage_v"]) for point in points if _finite(point.get("voltage_v"))]
    voltage_lows = [float(point["voltage_range_v"][0]) for point in points
                    if point.get("voltage_range_v") is not None]
    voltage_highs = [float(point["voltage_range_v"][1]) for point in points
                     if point.get("voltage_range_v") is not None]
    erpm_hull = (_convex_hull([(float(point["upper_erpm"]), float(point["lower_erpm"]))
                              for point in points]) if mode == "dual" else None)
    return {"status": fitted.get("status", "unavailable"),
            "reason": fitted.get("reason"),
            "voltage_center_v": float(np.mean(voltages)) if voltages else None,
            "voltage_range_v": [min(voltage_lows), max(voltage_highs)] if voltage_lows else None,
            "erpm_hull": [[float(x), float(y)] for x, y in erpm_hull] if erpm_hull is not None else None,
            "fit": fitted}


def _cross(origin: tuple[float, float], a: tuple[float, float],
           b: tuple[float, float]) -> float:
    return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])


def _convex_hull(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    unique = sorted(set(points))
    if len(unique) < 3:
        return unique
    lower: list[tuple[float, float]] = []
    upper: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    for point in reversed(unique):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _polygon_area(polygon: Sequence[tuple[float, float]]) -> float:
    if len(polygon) < 3:
        return 0.0
    return abs(sum(polygon[i][0] * polygon[(i + 1) % len(polygon)][1] -
                   polygon[(i + 1) % len(polygon)][0] * polygon[i][1]
                   for i in range(len(polygon))) * 0.5)


def _line_intersection(start: tuple[float, float], end: tuple[float, float],
                       clip_a: tuple[float, float], clip_b: tuple[float, float]) -> tuple[float, float]:
    dx, dy = end[0] - start[0], end[1] - start[1]
    ex, ey = clip_b[0] - clip_a[0], clip_b[1] - clip_a[1]
    denominator = dx * ey - dy * ex
    if abs(denominator) < 1.0e-12:
        return end
    t = ((clip_a[0] - start[0]) * ey - (clip_a[1] - start[1]) * ex) / denominator
    return start[0] + t * dx, start[1] + t * dy


def _convex_intersection(subject: Sequence[tuple[float, float]],
                         clip: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    output = list(subject)
    for index, clip_a in enumerate(clip):
        clip_b = clip[(index + 1) % len(clip)]
        input_polygon, output = output, []
        if not input_polygon:
            break
        start = input_polygon[-1]
        for end in input_polygon:
            end_inside = _cross(clip_a, clip_b, end) >= -1.0e-9
            start_inside = _cross(clip_a, clip_b, start) >= -1.0e-9
            if end_inside:
                if not start_inside:
                    output.append(_line_intersection(start, end, clip_a, clip_b))
                output.append(end)
            elif start_inside:
                output.append(_line_intersection(start, end, clip_a, clip_b))
            start = end
    return output


def _point_in_convex(polygon: Sequence[Sequence[float]], point: tuple[float, float]) -> bool:
    if len(polygon) < 3:
        return False
    converted = [(float(item[0]), float(item[1])) for item in polygon]
    return all(_cross(converted[index], converted[(index + 1) % len(converted)], point) >= -1.0e-7
               for index in range(len(converted)))


def _common_coverage(layers: Sequence[Mapping[str, Any]], mode: str) -> dict[str, Any] | None:
    if mode == "dual":
        pairwise = []
        for left, right in zip(layers, layers[1:]):
            intersection = _convex_intersection(left["erpm_hull"], right["erpm_hull"])
            area = _polygon_area(intersection)
            if area <= 1.0e-6:
                return None
            pairwise.append({"voltage_centers_v": [left["voltage_center_v"],
                                                    right["voltage_center_v"]],
                             "intersection_hull": [[float(x), float(y)] for x, y in intersection],
                             "intersection_area_erpm2": float(area)})
        return {"adjacent_layer_intersections": pairwise}
    active_min = max(float(layer["fit"]["coverage"]["active_erpm"][0]) for layer in layers)
    active_max = min(float(layer["fit"]["coverage"]["active_erpm"][1]) for layer in layers)
    if active_max - active_min <= 100.0:
        return None
    return {"active_erpm": [active_min, active_max]}


def _fit_voltage_model(points: Sequence[Mapping[str, Any]], mode: str,
                       max_layer_span_v: float = 0.50,
                       min_layer_gap_v: float = 0.10) -> dict[str, Any]:
    mode_points = [point for point in points if point.get("mode") == mode]
    groups = _voltage_groups(mode_points)
    if len(groups) < 2:
        return {"status": "unavailable", "reason": "fewer_than_2_measured_voltage_layers",
                "mode": mode, "layers": []}
    layers = []
    for label, group in groups:
        layer = _layer_model(group, mode)
        layer["label"] = label
        layers.append(layer)
    failed = [layer for layer in layers if layer["status"] != "available"]
    if failed:
        return {"status": "unavailable", "reason": "voltage_layer_fit_unavailable",
                "mode": mode, "layers": layers}
    if any(float(layer["voltage_range_v"][1]) - float(layer["voltage_range_v"][0]) >
           max_layer_span_v for layer in layers):
        return {"status": "unavailable", "reason": "voltage_layer_span_too_wide",
                "mode": mode, "layers": layers,
                "voltage_layer_limits": {"max_span_v": max_layer_span_v,
                                         "min_adjacent_gap_v": min_layer_gap_v}}
    centers = [float(layer["voltage_center_v"]) for layer in layers]
    if any(b - a < 0.10 for a, b in zip(centers, centers[1:])):
        return {"status": "unavailable", "reason": "measured_voltage_layers_not_distinct",
                "mode": mode, "layers": layers}
    if any(float(right["voltage_range_v"][0]) - float(left["voltage_range_v"][1]) <
           min_layer_gap_v for left, right in zip(layers, layers[1:])):
        return {"status": "unavailable", "reason": "measured_voltage_layer_ranges_overlap",
                "mode": mode, "layers": layers,
                "voltage_layer_limits": {"max_span_v": max_layer_span_v,
                                         "min_adjacent_gap_v": min_layer_gap_v}}
    common = _common_coverage(layers, mode)
    if common is None:
        return {"status": "unavailable", "reason": "no_overlapping_erpm_coverage_across_voltage_layers",
                "mode": mode, "layers": layers}
    return {"status": "available", "kind": "measured_voltage_layer_interpolation",
            "mode": mode, "layers": layers,
            "voltage_range_v": [centers[0], centers[-1]],
            "common_coverage": common,
            "voltage_layer_limits": {"max_span_v": max_layer_span_v,
                                     "min_adjacent_gap_v": min_layer_gap_v},
            "interpolation": "linear_between_adjacent_measured_voltage_layer_models",
            "extrapolation": "rejected"}


def _predict_layer(layer: Mapping[str, Any], *, upper_erpm: float | None,
                   lower_erpm: float | None, mode: str) -> float:
    fitted = layer["fit"]
    model = fitted["model"]
    if mode == "dual":
        scale = float(model["erpm_scale"])
        u, l = float(upper_erpm) / scale, float(lower_erpm) / scale
        c = model["coefficients_n"]
        return float(c["intercept"] + c["upper_squared"] * u * u +
                     c["lower_squared"] * l * l + c["cross"] * u * l)
    erpm = float(upper_erpm if mode == "upper" else lower_erpm)
    scale = float(model["erpm_scale"])
    c = model["coefficients_n"]
    return float(c["intercept"] + c["active_squared"] * (erpm / scale) ** 2)


def _predict_voltage_model(model: Mapping[str, Any], *, upper_erpm: float | None,
                           lower_erpm: float | None, voltage_v: float) -> float:
    if model.get("status") != "available":
        raise ValueError("voltage_model_unavailable")
    mode = str(model["mode"])
    if not _finite(voltage_v):
        raise ValueError("invalid_voltage")
    if mode == "dual" and (not _finite(upper_erpm) or not _finite(lower_erpm)):
        raise ValueError("measured_dual_erpm_required")
    if mode == "upper" and not _finite(upper_erpm):
        raise ValueError("measured_upper_erpm_required")
    if mode == "lower" and not _finite(lower_erpm):
        raise ValueError("measured_lower_erpm_required")
    coverage = model["common_coverage"]
    if mode != "dual":
        low, high = coverage["active_erpm"]
        active_erpm = upper_erpm if mode == "upper" else lower_erpm
        if float(active_erpm) < low or float(active_erpm) > high:
            raise ValueError("erpm_outside_common_coverage")
    low_v, high_v = model["voltage_range_v"]
    if float(voltage_v) < low_v or float(voltage_v) > high_v:
        raise ValueError("voltage_outside_measured_interpolation_range")
    layers = model["layers"]
    for left, right in zip(layers, layers[1:]):
        lv, rv = float(left["voltage_center_v"]), float(right["voltage_center_v"])
        if lv <= float(voltage_v) <= rv:
            if mode == "dual":
                point = (float(upper_erpm), float(lower_erpm))
                if (not _point_in_convex(left["erpm_hull"], point) or
                        not _point_in_convex(right["erpm_hull"], point)):
                    raise ValueError("erpm_outside_adjacent_layer_convex_hulls")
            left_value = _predict_layer(left, upper_erpm=upper_erpm, lower_erpm=lower_erpm, mode=mode)
            right_value = _predict_layer(right, upper_erpm=upper_erpm, lower_erpm=lower_erpm, mode=mode)
            weight = (float(voltage_v) - lv) / (rv - lv)
            return left_value + weight * (right_value - left_value)
    raise ValueError("voltage_layer_bracket_not_found")


def predict_thrust(analysis: Mapping[str, Any], *, upper_erpm: float | None,
                   lower_erpm: float | None, voltage_v: float,
                   mode: str = "dual") -> float:
    """Predict installed-assembly total thrust only inside measured coverage."""
    if analysis.get("schema_version") != 2:
        raise ValueError("unsupported_analysis_schema: v2 electrical eRPM analysis required")
    if mode not in {"dual", "upper", "lower"}:
        raise ValueError("invalid_mode")
    models = analysis.get("static", {}).get("voltage_models", {})
    return _predict_voltage_model(models.get(mode, {}), upper_erpm=upper_erpm,
                                  lower_erpm=lower_erpm, voltage_v=voltage_v)


def _voltage_validation(points: Sequence[Mapping[str, Any]], mode: str,
                        max_layer_span_v: float, min_layer_gap_v: float) -> dict[str, Any]:
    mode_points = [point for point in points if point.get("mode") == mode]
    holdouts: list[dict[str, Any]] = []
    groups: list[tuple[str, str, list[Mapping[str, Any]]]] = []
    runs = sorted({str(point["run_id"]) for point in mode_points})
    if len(runs) >= 2:
        groups.extend(("run", run, [p for p in mode_points if str(p["run_id"]) == run])
                      for run in runs)
    layer_keys = sorted({(f"layer:{float(p['voltage_layer_v']):g}" if
                         _finite(p.get("voltage_layer_v")) else f"run:{p['run_id']}")
                         for p in mode_points})
    if len(layer_keys) >= 3:
        for key in layer_keys:
            selected = [p for p in mode_points if
                        (f"layer:{float(p['voltage_layer_v']):g}" if
                         _finite(p.get("voltage_layer_v")) else f"run:{p['run_id']}") == key]
            groups.append(("voltage_layer", key, selected))
    for kind, value, test in groups:
        ids = {(str(point["run_id"]), str(point["segment_id"])) for point in test}
        train = [point for point in mode_points
                 if (str(point["run_id"]), str(point["segment_id"])) not in ids]
        fitted = _fit_voltage_model(train, mode, max_layer_span_v, min_layer_gap_v)
        if fitted.get("status") != "available":
            holdouts.append({"group": kind, "value": value, "status": "unavailable",
                             "reason": fitted.get("reason"), "test_points": len(test)})
            continue
        actual, predicted = [], []
        prediction_rows: list[dict[str, Any]] = []
        rejected = Counter()
        for point in test:
            try:
                prediction = _predict_voltage_model(
                    fitted, upper_erpm=point.get("upper_erpm"), lower_erpm=point.get("lower_erpm"),
                    voltage_v=float(point["voltage_v"]))
            except ValueError as exc:
                rejected[str(exc)] += 1
                continue
            actual.append(float(point["thrust_n"]))
            predicted.append(prediction)
            prediction_rows.append({
                "run_id": str(point["run_id"]),
                "segment_id": str(point["segment_id"]),
                "mode": mode,
                "upper_erpm": point.get("upper_erpm"),
                "lower_erpm": point.get("lower_erpm"),
                "voltage_v": float(point["voltage_v"]),
                "actual_thrust_n": float(point["thrust_n"]),
                "predicted_thrust_n": float(prediction),
                "residual_n": float(prediction - float(point["thrust_n"])),
            })
        if not actual:
            holdouts.append({"group": kind, "value": value, "status": "unavailable",
                             "reason": "holdout_outside_training_interpolation_coverage",
                             "test_points": len(test), "rejected": dict(rejected)})
        else:
            holdouts.append({"group": kind, "value": value, "status": "available",
                             "test_points": len(test), "predicted_points": len(actual),
                             "rejected": dict(rejected),
                             "metrics": _metrics(np.asarray(actual), np.asarray(predicted)),
                             "predictions": prediction_rows})
    valid = [item for item in holdouts if item["status"] == "available"]
    if not valid:
        return {"status": "training_only", "strategy": "whole_run_and_voltage_layer_holdout",
                "reason": "insufficient_replicated_runs_or_interpolating_voltage_layers",
                "holdouts": holdouts}
    return {"status": "available", "strategy": "whole_run_and_voltage_layer_holdout",
            "holdouts": holdouts}


def _fit_response_surface(points: Sequence[Mapping[str, Any]], response: str,
                          kind: str, unit: str) -> dict[str, Any]:
    usable = [point for point in points if _finite(point.get(response))]
    if len(usable) < 6:
        return {"status": "unavailable", "reason": "fewer_than_6_response_points"}
    coverage = _coverage(usable)
    if not coverage["independently_varied"]:
        return {"status": "unavailable", "reason": "inputs_not_independently_varied",
                "coverage": coverage}
    design, scale = _design(usable)
    rank = int(np.linalg.matrix_rank(design))
    condition = float(np.linalg.cond(design))
    if rank < design.shape[1] or not math.isfinite(condition) or condition > _MAX_CONDITION:
        return {"status": "unavailable", "reason": "rank_deficient_or_ill_conditioned"}
    actual = np.asarray([float(point[response]) for point in usable])
    coefficients, _, _, _ = np.linalg.lstsq(design, actual, rcond=None)
    error = design @ coefficients - actual
    return {"status": "available", "model": {"kind": kind, "unit": unit,
            "equation": "y=c0+c_upper*u^2+c_lower*l^2+c_cross*u*l",
            "erpm_scale": scale,
            "coefficients": {key: float(value) for key, value in zip(
                ("intercept", "upper_squared", "lower_squared", "cross"), coefficients)}},
            "coverage": coverage, "rank": rank, "condition_number": condition,
            "training_metrics": {"rmse": float(np.sqrt(np.mean(error ** 2))),
                                 "mae": float(np.mean(np.abs(error))), "unit": unit}}


def _fit_single_response_surface(points: Sequence[Mapping[str, Any]], response: str,
                                 mode: str, kind: str, unit: str) -> dict[str, Any]:
    active = f"{mode}_erpm"
    usable = [point for point in points if _finite(point.get(response)) and
              _finite(point.get(active))]
    if len(usable) < 4:
        return {"status": "unavailable", "reason": "fewer_than_4_response_points"}
    erpm = np.asarray([float(point[active]) for point in usable])
    if np.ptp(erpm) <= 100.0:
        return {"status": "unavailable", "reason": "insufficient_active_erpm_span"}
    scale = max(float(np.max(erpm)), 1.0)
    design = np.column_stack((np.ones(len(erpm)), (erpm / scale) ** 2))
    actual = np.asarray([float(point[response]) for point in usable])
    coefficients, _, rank, singular = np.linalg.lstsq(design, actual, rcond=None)
    condition = float(singular[0] / singular[-1]) if len(singular) > 1 and singular[-1] else math.inf
    if rank < 2 or not math.isfinite(condition) or condition > _MAX_CONDITION:
        return {"status": "unavailable", "reason": "rank_deficient_or_ill_conditioned"}
    error = design @ coefficients - actual
    return {"status": "available", "model": {"kind": kind, "unit": unit,
            "mode": mode, "equation": "y=c0+c_active*(active_erpm/erpm_scale)^2",
            "erpm_scale": scale, "coefficients": {
                "intercept": float(coefficients[0]), "active_squared": float(coefficients[1])}},
            "coverage": {"active_erpm": [float(np.min(erpm)), float(np.max(erpm))]},
            "training_metrics": {"rmse": float(np.sqrt(np.mean(error ** 2))),
                                 "mae": float(np.mean(np.abs(error))), "unit": unit}}


def _fit_layered_response(points: Sequence[Mapping[str, Any]], response: str,
                          kind: str, unit: str, max_layer_span_v: float,
                          min_layer_gap_v: float, mode: str = "dual") -> dict[str, Any]:
    usable = [point for point in points if _finite(point.get(response))]
    groups = _voltage_groups(usable)
    if len(groups) < 2:
        return {"status": "unavailable", "reason": "fewer_than_2_measured_voltage_layers",
                "layers": []}
    layers = []
    for label, group in groups:
        fit = (_fit_response_surface(group, response, kind, unit) if mode == "dual" else
               _fit_single_response_surface(group, response, mode, kind, unit))
        voltages = [float(point["voltage_v"]) for point in group]
        lows = [float(point["voltage_range_v"][0]) for point in group]
        highs = [float(point["voltage_range_v"][1]) for point in group]
        hull = (_convex_hull([(float(point["upper_erpm"]), float(point["lower_erpm"]))
                              for point in group]) if mode == "dual" else None)
        layers.append({"label": label, "status": fit["status"], "reason": fit.get("reason"),
                       "voltage_center_v": float(np.mean(voltages)),
                       "voltage_range_v": [min(lows), max(highs)],
                       "erpm_hull": [[x, y] for x, y in hull] if hull is not None else None,
                       "fit": fit})
    if any(layer["status"] != "available" for layer in layers):
        return {"status": "unavailable", "reason": "voltage_layer_fit_unavailable",
                "layers": layers}
    if any(layer["voltage_range_v"][1] - layer["voltage_range_v"][0] > max_layer_span_v
           for layer in layers):
        return {"status": "unavailable", "reason": "voltage_layer_span_too_wide",
                "layers": layers}
    if any(right["voltage_range_v"][0] - left["voltage_range_v"][1] < min_layer_gap_v
           for left, right in zip(layers, layers[1:])):
        return {"status": "unavailable", "reason": "measured_voltage_layer_ranges_overlap",
                "layers": layers}
    common = _common_coverage(layers, mode)
    if common is None:
        return {"status": "unavailable", "reason": "no_overlapping_erpm_coverage_across_voltage_layers",
                "layers": layers}
    return {"status": "available", "kind": kind, "unit": unit, "mode": mode,
            "layers": layers,
            "voltage_range_v": [layers[0]["voltage_center_v"], layers[-1]["voltage_center_v"]],
            "common_coverage": common, "extrapolation": "rejected",
            "validation": {"status": "training_only", "reason": "insufficient_independent_groups"}}


def _predict_response(model: Mapping[str, Any], point: Mapping[str, Any]) -> float:
    voltage = float(point["voltage_v"])
    mode = str(model.get("mode", "dual"))
    upper = point.get("upper_erpm")
    lower = point.get("lower_erpm")
    if mode == "dual" and (not _finite(upper) or not _finite(lower)):
        raise ValueError("measured_dual_erpm_required")
    if mode == "upper" and not _finite(upper):
        raise ValueError("measured_upper_erpm_required")
    if mode == "lower" and not _finite(lower):
        raise ValueError("measured_lower_erpm_required")
    erpm_point = (float(upper) if _finite(upper) else math.nan,
                  float(lower) if _finite(lower) else math.nan)
    for left, right in zip(model["layers"], model["layers"][1:]):
        lv, rv = float(left["voltage_center_v"]), float(right["voltage_center_v"])
        if lv <= voltage <= rv:
            if mode == "dual":
                if (not _point_in_convex(left["erpm_hull"], erpm_point) or
                        not _point_in_convex(right["erpm_hull"], erpm_point)):
                    raise ValueError("erpm_outside_adjacent_layer_convex_hulls")
            else:
                active = erpm_point[0] if mode == "upper" else erpm_point[1]
                low, high = model["common_coverage"]["active_erpm"]
                if active < low or active > high:
                    raise ValueError("erpm_outside_common_coverage")
            values = []
            for layer in (left, right):
                fit = layer["fit"]["model"]
                scale = float(fit["erpm_scale"])
                c = fit["coefficients"]
                if mode == "dual":
                    u, l = erpm_point[0] / scale, erpm_point[1] / scale
                    values.append(c["intercept"] + c["upper_squared"] * u * u +
                                  c["lower_squared"] * l * l + c["cross"] * u * l)
                else:
                    active = erpm_point[0] if mode == "upper" else erpm_point[1]
                    values.append(c["intercept"] + c["active_squared"] * (active / scale) ** 2)
            weight = (voltage - lv) / (rv - lv)
            return float(values[0] + weight * (values[1] - values[0]))
    raise ValueError("voltage_outside_measured_interpolation_range")


def predict_current(analysis: Mapping[str, Any], *, upper_erpm: float | None,
                    lower_erpm: float | None, voltage_v: float,
                    role: str = "upper", mode: str = "dual") -> float:
    """Predict one DShot ESC-current channel inside measured eRPM/V coverage."""
    if analysis.get("schema_version") != 2:
        raise ValueError("unsupported_analysis_schema: v2 electrical eRPM analysis required")
    if mode not in {"dual", "upper", "lower"}:
        raise ValueError("invalid_current_mode")
    if role not in {"upper", "lower"}:
        raise ValueError("invalid_current_role")
    if not _finite(voltage_v):
        raise ValueError("finite_voltage_required")
    model = analysis.get("power", {}).get("current_models", {}).get(mode, {}).get(role, {})
    if model.get("status") != "available":
        raise ValueError("current_model_unavailable")
    return _predict_response(model, {"upper_erpm": upper_erpm,
                                     "lower_erpm": lower_erpm,
                                     "voltage_v": voltage_v})


def _validate_layered_response(points: Sequence[Mapping[str, Any]], response: str,
                               kind: str, unit: str, max_span: float,
                               min_gap: float, mode: str = "dual") -> dict[str, Any]:
    usable = [point for point in points if _finite(point.get(response))]
    runs = sorted({str(point["run_id"]) for point in usable})
    holdouts = []
    groups = [("run", run, [point for point in usable if str(point["run_id"]) == run])
              for run in runs] if len(runs) >= 2 else []
    layer_keys = sorted({(f"layer:{float(point['voltage_layer_v']):g}" if
                         _finite(point.get("voltage_layer_v")) else f"run:{point['run_id']}")
                         for point in usable})
    if len(layer_keys) >= 3:
        for key in layer_keys:
            groups.append(("voltage_layer", key, [point for point in usable if
                           (f"layer:{float(point['voltage_layer_v']):g}" if
                            _finite(point.get("voltage_layer_v")) else
                            f"run:{point['run_id']}") == key]))
    for group_kind, group_value, test in groups:
        test_ids = {(str(point["run_id"]), str(point["segment_id"])) for point in test}
        train = [point for point in usable if
                 (str(point["run_id"]), str(point["segment_id"])) not in test_ids]
        fitted = _fit_layered_response(train, response, kind, unit, max_span, min_gap, mode)
        if fitted.get("status") != "available":
            holdouts.append({"group": group_kind, "value": group_value, "status": "unavailable",
                             "reason": fitted.get("reason")})
            continue
        actual, predicted = [], []
        for point in test:
            try:
                predicted.append(_predict_response(fitted, point))
                actual.append(float(point[response]))
            except ValueError:
                pass
        if not actual:
            holdouts.append({"group": group_kind, "value": group_value, "status": "unavailable",
                             "reason": "holdout_outside_training_coverage"})
        else:
            error = np.asarray(predicted) - np.asarray(actual)
            holdouts.append({"group": group_kind, "value": group_value, "status": "available",
                             "points": len(actual), "rmse": float(np.sqrt(np.mean(error ** 2))),
                             "mae": float(np.mean(np.abs(error))), "unit": unit})
    return {"status": "available" if any(item["status"] == "available" for item in holdouts)
            else "training_only", "strategy": "whole_run_and_voltage_layer_holdout",
            "holdouts": holdouts}


def _fit_power(points: Sequence[Mapping[str, Any]], metadata: Mapping[str, Any]) -> dict[str, Any]:
    max_span = _metadata_number(metadata, "max_voltage_layer_span_v", 0.50)
    min_gap = _metadata_number(metadata, "min_voltage_layer_gap_v", 0.10)
    models = {}
    for mode in ("dual", "upper", "lower"):
        mode_points = [point for point in points if point.get("mode") == mode]
        response_sets = {
            response: [point["response_points"][response] for point in mode_points
                       if point.get("response_points", {}).get(response) is not None]
            for response in ("upper_esc_current_a", "lower_esc_current_a", "input_power_w")
        }
        role_models = {}
        for role in ("upper", "lower"):
            response = f"{role}_esc_current_a"
            fitted = _fit_layered_response(
                response_sets[response], response, f"dshot_{mode}_{role}_esc_current",
                "A", max_span, min_gap, mode)
            if fitted.get("status") == "available":
                fitted["validation"] = _validate_layered_response(
                    response_sets[response], response, f"dshot_{mode}_{role}_esc_current",
                    "A", max_span, min_gap, mode)
            role_models[role] = fitted
        combined = _fit_layered_response(
            response_sets["input_power_w"], "input_power_w", f"dshot_{mode}_combined_input_power",
            "W", max_span, min_gap, mode)
        if combined.get("status") == "available":
            combined["validation"] = _validate_layered_response(
                response_sets["input_power_w"], "input_power_w", f"dshot_{mode}_combined_input_power",
                "W", max_span, min_gap, mode)
        models[mode] = {**role_models, "combined_input_power": combined}
    upper = models["dual"]["upper"]
    lower = models["dual"]["lower"]
    combined = models["dual"]["combined_input_power"]
    observations = [{"run_id": point["run_id"], "segment_id": point["segment_id"],
                     "upper_esc_current_a": (point.get("response_points", {}).get(
                         "upper_esc_current_a") or {}).get("upper_esc_current_a"),
                     "lower_esc_current_a": (point.get("response_points", {}).get(
                         "lower_esc_current_a") or {}).get("lower_esc_current_a"),
                     "input_power_w": (point.get("response_points", {}).get(
                         "input_power_w") or {}).get("input_power_w"),
                     "board_current_a_diagnostic": point.get("board_current_a_diagnostic")}
                    for point in points]
    statuses = [entry["status"] for mode_models in models.values()
                for entry in mode_models.values()]
    dual_statuses = [upper["status"], lower["status"], combined["status"]]
    return {"status": "available" if all(value == "available" for value in dual_statuses) else
            "partial" if any(value == "available" for value in statuses) else "unavailable",
            "source": "dshot_esc_current", "external_calibration_verified": False,
            "current_resolution_a": _metadata_number(metadata, "current_resolution_a", 1.0),
            "current_resolution_source": str(metadata.get(
                "current_resolution_source", "TBENCH v1 whole-amp ESC current fields")),
            "current_models": models,
            "upper_current_model": upper, "lower_current_model": lower,
            "combined_input_power_model": combined, "observations": observations,
            "board_current_diagnostic_only": True,
            "note": "Board ADC current is diagnostic only and is never a fitting fallback."}


def _validation(points: Sequence[Mapping[str, Any]], static: Mapping[str, Any]) -> dict[str, Any]:
    if static.get("status") != "available":
        return {"status": "unavailable", "strategy": "grouped_holdout",
                "reason": "static_model_unavailable", "holdouts": []}
    holdout_groups: list[tuple[str, str, list[Mapping[str, Any]]]] = []
    runs = sorted({str(point["run_id"]) for point in points})
    if len(runs) >= 2:
        holdout_groups.extend(("run", run, [p for p in points if str(p["run_id"]) == run]) for run in runs)
    layers = sorted({float(point["voltage_layer_v"]) for point in points
                     if point.get("voltage_layer_v") is not None})
    if len(layers) >= 2:
        holdout_groups.extend(("voltage_layer_v", f"{layer:g}",
                               [p for p in points if p.get("voltage_layer_v") == layer])
                              for layer in layers)
    holdouts: list[dict[str, Any]] = []
    for kind, value, test in holdout_groups:
        test_ids = {(str(p["run_id"]), str(p["segment_id"])) for p in test}
        train = [p for p in points if (str(p["run_id"]), str(p["segment_id"])) not in test_ids]
        fitted = _fit_static(train)
        if fitted.get("status") != "available":
            holdouts.append({"group": kind, "value": value, "status": "unavailable",
                             "reason": fitted.get("reason"), "test_points": len(test)})
            continue
        model = fitted["model"]
        design, _ = _design(test, erpm_scale=float(model["erpm_scale"]))
        coefficients = np.asarray(list(model["coefficients_n"].values()))
        actual = np.asarray([float(point["thrust_n"]) for point in test])
        holdouts.append({"group": kind, "value": value, "status": "available",
                         "test_points": len(test), "metrics": _metrics(actual, design @ coefficients)})
    valid = [item for item in holdouts if item["status"] == "available"]
    if not valid:
        return {"status": "training_only", "strategy": "grouped_holdout",
                "reason": "insufficient_independent_runs_or_voltage_layers",
                "holdouts": holdouts,
                "note": "Adjacent samples from one segment were not randomly split."}
    return {"status": "available", "strategy": "leave_one_run_and_voltage_layer_out",
            "holdouts": holdouts,
            "note": "Every steady segment stays wholly in train or holdout."}


def _dynamic_group(samples: Sequence[BenchSample], channel: str,
                   metadata: Mapping[str, Any]) -> tuple[dict[str, Any] | None, str | None]:
    erpm_name = f"{channel}_erpm"
    command_name = f"{channel}_command_pct"
    age_name = f"{channel}_erpm_age_ms"
    max_age = _metadata_number(metadata, "max_erpm_age_ms", 150.0)
    valid = [sample for sample in samples if (_finite(getattr(sample, erpm_name)) and
             _finite(getattr(sample, command_name)) and _measured_erpm(sample) and
             getattr(sample, age_name) is not None and 0 <= getattr(sample, age_name) <= max_age and
             sample.fc_time_ms is not None)]
    valid.sort(key=lambda sample: int(sample.fc_time_ms))
    if len(valid) < 3:
        return None, "fewer_than_3_fresh_dynamic_samples"
    # Each eRPM belongs to its source update, not to the later poll that carried
    # it.  Repeated source timestamps are stale duplicates, not a higher rate.
    source_ms = [int(sample.fc_time_ms) - int(getattr(sample, age_name)) for sample in valid]
    deduplicated: list[BenchSample] = []
    deduplicated_ms: list[int] = []
    for item, timestamp in zip(valid, source_ms):
        if deduplicated_ms and timestamp == deduplicated_ms[-1]:
            continue
        deduplicated.append(item)
        deduplicated_ms.append(timestamp)
    valid = deduplicated
    if len(valid) < 3:
        return None, "fewer_than_3_independent_erpm_updates"
    times = np.asarray(deduplicated_ms, dtype=float) / 1000.0
    times -= times[0]
    gaps = np.diff(times)
    if np.any(gaps <= 0):
        return None, "non_monotonic_firmware_time"
    median_dt = float(np.median(gaps))
    if median_dt > 0.10:
        return None, "effective_sample_rate_below_10_hz"
    if len(valid) < 12:
        return None, "fewer_than_12_independent_erpm_updates"
    if float(np.max(gaps)) > max(0.25, 3.0 * median_dt):
        return None, "dropped_frames_or_large_time_gap"
    erpm = np.asarray([float(getattr(sample, erpm_name)) for sample in valid])
    command = np.asarray([float(getattr(sample, command_name)) for sample in valid])
    changed = np.flatnonzero(np.abs(command - command[0]) >= 0.5)
    if not len(changed):
        return None, "insufficient_step_excitation"
    change = int(changed[0])
    if change < 3:
        return None, "fewer_than_3_pre_step_updates"
    if len(valid) - change < 8:
        return None, "fewer_than_8_post_step_updates"
    pre = erpm[:max(change, 2)]
    post_count = max(3, len(erpm) // 5)
    y0 = float(np.median(pre))
    yinf = float(np.median(erpm[-post_count:]))
    amplitude = yinf - y0
    if abs(amplitude) < max(100.0, 0.05 * max(abs(yinf), 1.0)):
        return None, "insufficient_erpm_response"
    step_time = float(times[change])
    relative_time = times - step_time
    best: tuple[float, float, float, np.ndarray] | None = None
    max_delay = min(0.5, max(0.0, float(relative_time[-1]) * 0.4))
    duration = float(relative_time[-1] - max(relative_time[change], 0.0))
    if duration < 0.35:
        return None, "dynamic_window_too_short"
    delay_candidates = np.arange(0.0, max_delay + median_dt * 0.5, median_dt)
    for delay in delay_candidates:
        for tau in np.geomspace(max(median_dt / 2.0, 0.005), max(duration * 2.0, 0.02), 120):
            elapsed = np.maximum(relative_time - delay, 0.0)
            predicted = np.where(relative_time < 0.0, y0,
                                 y0 + amplitude * (1.0 - np.exp(-elapsed / tau)))
            mse = float(np.mean((erpm - predicted) ** 2))
            if best is None or mse < best[0]:
                best = (mse, float(delay), float(tau), predicted)
    assert best is not None
    tau_intervals = best[2] / median_dt
    observed_time_constants = duration / best[2]
    if tau_intervals < 2.0:
        return None, "time_constant_below_2_update_intervals"
    if observed_time_constants < 4.0:
        return None, "dynamic_window_below_4_time_constants"
    return {
        "status": "available", "channel": channel,
        "direction": "rise" if amplitude > 0 else "fall",
        "samples": len(valid), "effective_rate_hz": 1.0 / median_dt,
        "delay_s": best[1], "delay_resolution_s": median_dt,
        "time_constant_s": best[2], "time_constant_update_intervals": tau_intervals,
        "observed_time_constants": observed_time_constants,
        "initial_erpm": y0, "final_erpm": yinf,
        "rmse_erpm": float(math.sqrt(best[0])),
        "source_time": "unwrapped_firmware_fc_time_ms",
        "warning": "The fit uses eRPM only. It includes eRPM telemetry/acquisition delay and does not fit force-cell filtering as motor delay.",
    }, None


def _fit_dynamics(samples: Sequence[BenchSample], metadata: Mapping[str, Any]) -> dict[str, Any]:
    groups: dict[tuple[str, str], list[BenchSample]] = defaultdict(list)
    for sample in samples:
        if sample.phase == "dynamic":
            groups[(sample.run_id, sample.segment_id)].append(sample)
    fits: list[dict[str, Any]] = []
    rejected: list[dict[str, str]] = []
    for (run_id, segment_id), group in sorted(groups.items()):
        for channel in ("upper", "lower"):
            result, reason = _dynamic_group(group, channel, metadata)
            if result is None:
                rejected.append({"run_id": run_id, "segment_id": segment_id,
                                 "channel": channel, "reason": str(reason)})
            else:
                result.update({"run_id": run_id, "segment_id": segment_id})
                fits.append(result)
    if not fits:
        reason = "no_dynamic_segments" if not groups else "all_dynamic_segments_rejected"
        return {"status": "unavailable", "reason": reason, "fits": [], "rejected": rejected}
    by_kind: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for fit in fits:
        by_kind[f"{fit['channel']}_{fit['direction']}"].append(fit)
    summaries = {key: {"count": len(values),
                       "median_delay_s": float(np.median([v["delay_s"] for v in values])),
                       "median_time_constant_s": float(np.median([v["time_constant_s"] for v in values]))}
                 for key, values in sorted(by_kind.items())}
    return {"status": "available", "model": "first_order_plus_dead_time",
            "fits": fits, "summaries": summaries, "rejected": rejected,
            "limitation": "Rise/fall and upper/lower fits remain separate; force-cell filtering is not claimed as motor delay."}


def _diagnostic_summary(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {"status": "unavailable", "points": 0, "metrics": None,
                "error_by_loaded_voltage": []}
    actual = np.asarray([float(row["actual_thrust_n"]) for row in rows])
    predicted = np.asarray([float(row["predicted_thrust_n"]) for row in rows])
    groups: dict[float, list[Mapping[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[round(float(row["voltage_v"]), 1)].append(row)
    by_voltage = []
    for voltage, items in sorted(groups.items()):
        item_actual = np.asarray([float(item["actual_thrust_n"]) for item in items])
        item_predicted = np.asarray([float(item["predicted_thrust_n"]) for item in items])
        by_voltage.append({"loaded_voltage_v": voltage, "points": len(items),
                           "metrics": _metrics(item_actual, item_predicted)})
    return {"status": "available", "points": len(rows),
            "metrics": _metrics(actual, predicted),
            "error_by_loaded_voltage": by_voltage}


def _training_diagnostics(model: Mapping[str, Any],
                          points: Sequence[Mapping[str, Any]], mode: str) -> dict[str, Any]:
    if model.get("status") != "available":
        return {"status": "unavailable", "reason": "voltage_model_unavailable",
                "predictions": [], **_diagnostic_summary([])}
    rows, rejected = [], Counter()
    for point in points:
        if point.get("mode") != mode:
            continue
        try:
            prediction = _predict_voltage_model(
                model, upper_erpm=point.get("upper_erpm"),
                lower_erpm=point.get("lower_erpm"), voltage_v=float(point["voltage_v"]))
        except (TypeError, ValueError) as exc:
            rejected[str(exc)] += 1
            continue
        rows.append({"run_id": str(point["run_id"]),
                     "segment_id": str(point["segment_id"]), "mode": mode,
                     "upper_erpm": point.get("upper_erpm"),
                     "lower_erpm": point.get("lower_erpm"),
                     "voltage_v": float(point["voltage_v"]),
                     "actual_thrust_n": float(point["thrust_n"]),
                     "predicted_thrust_n": float(prediction),
                     "residual_n": float(prediction - float(point["thrust_n"]))})
    result = _diagnostic_summary(rows)
    result.update({"kind": "training_fit", "predictions": rows,
                   "rejected": dict(rejected),
                   "input_points": sum(1 for point in points if point.get("mode") == mode),
                   "coverage_fraction": (len(rows) / max(1, sum(
                       1 for point in points if point.get("mode") == mode))),
                   "warning": "Training fit is not independent validation."})
    return result


def _validation_diagnostics(validation: Mapping[str, Any]) -> dict[str, Any]:
    rows = []
    unavailable_groups = []
    group_summary = {
        "run": {"attempted": 0, "available": 0, "rejected": 0,
                "test_points": 0, "predicted_points": 0},
        "voltage_layer": {"attempted": 0, "available": 0, "rejected": 0,
                          "test_points": 0, "predicted_points": 0},
    }
    for holdout in validation.get("holdouts", []):
        group_kind = str(holdout.get("group"))
        summary = group_summary.get(group_kind)
        if summary is not None:
            summary["attempted"] += 1
            summary["test_points"] += int(holdout.get("test_points", 0))
            summary["predicted_points"] += int(holdout.get("predicted_points", 0))
        if holdout.get("status") != "available":
            if summary is not None:
                summary["rejected"] += 1
            unavailable_groups.append({"group": holdout.get("group"),
                                       "value": holdout.get("value"),
                                       "reason": holdout.get("reason"),
                                       "test_points": holdout.get("test_points", 0),
                                       "explanation_zh": (
                                           "该电压层位于训练插值范围边界或之外；禁止外推，"
                                           "应使用同电压层的独立重复轮验证。"
                                           if group_kind == "voltage_layer" and
                                           holdout.get("reason") ==
                                           "holdout_outside_training_interpolation_coverage"
                                           else "该留出组无法由剩余训练组建立并预测。")})
            continue
        if summary is not None:
            summary["available"] += 1
        for item in holdout.get("predictions", []):
            rows.append({**item, "holdout_group": holdout.get("group"),
                         "holdout_value": holdout.get("value")})
    result = _diagnostic_summary(rows)
    for summary in group_summary.values():
        summary["prediction_coverage_fraction"] = (
            summary["predicted_points"] / summary["test_points"]
            if summary["test_points"] else 0.0)
    total_test = sum(item["test_points"] for item in group_summary.values())
    total_predicted = sum(item["predicted_points"] for item in group_summary.values())
    result.update({"kind": "independent_group_holdout", "predictions": rows,
                   "available_groups": sum(1 for item in validation.get("holdouts", [])
                                           if item.get("status") == "available"),
                   "unavailable_groups": unavailable_groups,
                   "groups": group_summary,
                   "attempted_prediction_points": total_test,
                   "prediction_coverage_fraction": (
                       total_predicted / total_test if total_test else 0.0)})
    if not rows:
        result["status"] = "unvalidated"
        result["reason"] = "no_available_group_holdout_predictions"
    return result


def _command_grid_diagnostics(points: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    groups: dict[tuple[str, str], dict[str, Any]] = {}
    for point in points:
        mode = str(point.get("mode"))
        voltage_group = (f"layer:{float(point['voltage_layer_v']):g}" if
                         _finite(point.get("voltage_layer_v")) else
                         f"run:{point['run_id']}")
        entry = groups.setdefault((mode, voltage_group), {
            "mode": mode, "voltage_group": voltage_group, "steady_points": 0,
            "missing_command_points": 0, "combinations": set(),
            "nonzero_combinations": set(),
        })
        entry["steady_points"] += 1
        required = (("upper_command_pct", "lower_command_pct") if mode == "dual" else
                    (f"{mode}_command_pct",))
        values = [point.get(field) for field in required]
        if not all(_finite(value) for value in values):
            entry["missing_command_points"] += 1
            continue
        combination = tuple(float(value) for value in values)
        entry["combinations"].add(combination)
        if all(value > 0.0 for value in combination):
            entry["nonzero_combinations"].add(combination)
    result = []
    for entry in groups.values():
        result.append({"mode": entry["mode"], "voltage_group": entry["voltage_group"],
                       "steady_points": entry["steady_points"],
                       "missing_command_points": entry["missing_command_points"],
                       "unique_command_combinations": len(entry["combinations"]),
                       "unique_nonzero_command_combinations": len(entry["nonzero_combinations"]),
                       "command_combinations": [list(values) for values in
                                                sorted(entry["combinations"])]})
    result.sort(key=lambda item: (item["mode"], item["voltage_group"]))
    return {"status": "available" if result else "unavailable",
            "meaning": "Exact recorded command grid counts; not unique eRPM operating points.",
            "groups": result,
            "missing_command_points": sum(item["missing_command_points"] for item in result)}


def _sufficiency(points: Sequence[Mapping[str, Any]], model: Mapping[str, Any],
                 validation_diagnostics: Mapping[str, Any],
                 metadata: Mapping[str, Any]) -> dict[str, Any]:
    dual = [point for point in points if point.get("mode") == "dual"]
    nonzero_steady = [point for point in dual if float(point.get("upper_erpm") or 0) > 0 and
                      float(point.get("lower_erpm") or 0) > 0]
    command_grid = _command_grid_diagnostics(points)
    dual_command_groups = [item for item in command_grid["groups"]
                           if item["mode"] == "dual"]
    minimum_nonzero_commands = min(
        (item["unique_nonzero_command_combinations"] for item in dual_command_groups),
        default=0)
    full_scale = _metadata_number(metadata, "full_scale_thrust_n",
                                  max((abs(float(point["thrust_n"])) for point in dual),
                                      default=0.0))
    rmse_fraction = _metadata_number(metadata, "reference_rmse_full_scale_fraction", 0.05)
    p95_fraction = _metadata_number(metadata, "reference_p95_full_scale_fraction", 0.10)
    reference = {"kind": "starting_reference_only", "advisory_only": True,
                 "recommended_unique_nonzero_command_combinations_per_voltage_group": 25,
                 "recommended_actual_voltage_coverage_count": 3,
                 "recommended_independent_validation_rounds": 1,
                 "rmse_full_scale_fraction": rmse_fraction,
                 "p95_full_scale_fraction": p95_fraction,
                 "does_not_authorize_flight": True}
    layers = model.get("layers", [])
    gaps = []
    if minimum_nonzero_commands < 25:
        gaps.append("fewer_than_25_unique_nonzero_command_combinations_per_voltage_group_reference")
    if command_grid["missing_command_points"]:
        gaps.append("recorded_command_fields_missing_for_some_steady_points")
    if len(layers) < 3:
        gaps.append("fewer_than_3_actual_voltage_coverage_groups_reference")
    if model.get("status") != "available":
        gaps.append(str(model.get("reason", "voltage_model_unavailable")))
    validation_metrics = validation_diagnostics.get("metrics")
    validation_groups = validation_diagnostics.get("groups", {})
    run_groups = validation_groups.get("run", {})
    voltage_groups = validation_groups.get("voltage_layer", {})
    if validation_diagnostics.get("status") != "available" or not validation_metrics:
        gaps.append("no_independent_holdout_validation")
        status = "unvalidated" if model.get("status") == "available" else "insufficient"
    else:
        if run_groups.get("attempted", 0) == 0 or run_groups.get("rejected", 0) > 0 or \
                run_groups.get("prediction_coverage_fraction", 0.0) < 1.0:
            gaps.append("independent_run_holdout_not_complete_across_recorded_domain")
        if len(layers) >= 3 and voltage_groups.get("available", 0) == 0:
            gaps.append("no_available_internal_voltage_layer_holdout")
        if full_scale <= 0:
            gaps.append("full_scale_thrust_unavailable")
        else:
            if float(validation_metrics["rmse_n"]) > full_scale * rmse_fraction:
                gaps.append("holdout_rmse_above_starting_reference")
            if float(validation_metrics["p95_abs_error_n"]) > full_scale * p95_fraction:
                gaps.append("holdout_p95_above_starting_reference")
        status = "reference_ready" if not gaps else "insufficient"
    voltage_count = len(layers)
    if status == "reference_ready":
        summary = (f"每个双驱动电压组至少 {minimum_nonzero_commands} 个唯一非零指令组合，"
                   f"{voltage_count} 个实际电压覆盖组；整轮留出 {run_groups.get('available', 0)} 组、"
                   f"中间电压层留出 {voltage_groups.get('available', 0)} 组可评估；"
                   "达到起步参考，但不代表飞行放行。")
    elif status == "unvalidated":
        summary = (f"每个双驱动电压组最少 {minimum_nonzero_commands} 个唯一非零指令组合，"
                   f"共 {voltage_count} 个实际电压覆盖组，"
                   "但没有可用的独立整轮/电压层留出验证，当前未验证。")
    else:
        summary = (f"双驱动聚合稳态点 {len(dual)}，每个电压组最少 {minimum_nonzero_commands} 个"
                   f"唯一非零指令组合，实际电压覆盖组 {voltage_count}；"
                   f"整轮留出可用/拒绝 {run_groups.get('available', 0)}/{run_groups.get('rejected', 0)}，"
                   f"中间电压层留出可用 {voltage_groups.get('available', 0)}；仍有 {len(gaps)} 项缺口。")
    mode_counts = Counter(str(point.get("mode")) for point in points)
    return {"status": status, "summary": summary, "reference": reference,
            "aggregated_steady_points": len(dual),
            "nonzero_steady_points": len(nonzero_steady),
            "command_grid": command_grid,
            "minimum_unique_nonzero_command_combinations_per_dual_voltage_group":
                minimum_nonzero_commands,
            "independent_runs": len({str(point["run_id"]) for point in dual}),
            "actual_voltage_coverage": [{"label": layer.get("label"),
                                         "center_v": layer.get("voltage_center_v"),
                                         "range_v": layer.get("voltage_range_v")}
                                        for layer in layers],
            "input_independently_varied": bool(model.get("layers")) and all(
                layer.get("fit", {}).get("coverage", {}).get("independently_varied", False)
                for layer in layers),
            "adjacent_domain_intersections": model.get("common_coverage"),
            "holdout_available_groups": validation_diagnostics.get("available_groups", 0),
            "full_scale_thrust_n": full_scale if full_scale > 0 else None,
            "holdout_metrics": validation_metrics,
            "gaps": gaps,
            "zero_or_not_spinning_points_counted_separately": len(dual) - len(nonzero_steady),
            "mode_point_counts": dict(sorted(mode_counts.items())),
            "modes_observed": sorted(mode_counts)}


def analyze_samples(samples: Sequence[BenchSample],
                    metadata: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Analyze one or more independent bench runs into JSON-safe evidence."""
    meta: Mapping[str, Any] = metadata or {}
    sample_list = list(samples)
    if meta.get("schema_version", 2) != 2:
        raise ValueError("unsupported_sample_schema: v2 electrical eRPM required")
    if meta.get("speed_domain", "electrical_erpm") != "electrical_erpm":
        raise ValueError("unsupported_speed_domain: electrical_erpm required")
    for sample in sample_list:
        if (not hasattr(sample, "upper_erpm") or not hasattr(sample, "lower_erpm") or
                hasattr(sample, "upper_rpm") or hasattr(sample, "lower_rpm")):
            raise ValueError("unsupported_sample_schema: v2 electrical eRPM required")
        if sample.speed_source != "dshot_erpm":
            raise ValueError("unsupported_speed_source: dshot_erpm required")
        if sample.current_source != "dshot":
            raise ValueError("unsupported_current_source: dshot required")
    points, exclusions = _steady_points(sample_list, meta)
    dual_points = [point for point in points if point["mode"] == "dual"]
    single_points = [point for point in points if point["mode"] in {"upper", "lower"}]
    erpm_only = _fit_static(dual_points)
    max_layer_span_v = _metadata_number(meta, "max_voltage_layer_span_v", 0.50)
    min_layer_gap_v = _metadata_number(meta, "min_voltage_layer_gap_v", 0.10)
    voltage_models = {mode: _fit_voltage_model(points, mode, max_layer_span_v,
                                               min_layer_gap_v)
                      for mode in ("dual", "upper", "lower")}
    for mode, voltage_model in voltage_models.items():
        voltage_model["validation"] = _voltage_validation(
            points, mode, max_layer_span_v, min_layer_gap_v)
    static = {
        "status": ("available" if voltage_models["dual"]["status"] == "available" else
                   "partial" if any(voltage_models[mode]["status"] == "available"
                                    for mode in ("upper", "lower")) else "unavailable"),
        "reason": voltage_models["dual"].get("reason"),
        "primary_model": "dual_measured_voltage_interpolation",
        "voltage_layer_limits": {"max_span_v": max_layer_span_v,
                                 "min_adjacent_gap_v": min_layer_gap_v},
        "voltage_grouping_policy": {
            "labeled_samples": "group_by_voltage_layer_v_but_measure_center_and_range_from_loaded_voltage",
            "unlabeled_samples": "group_by_run_id",
            "continuous_discharge_auto_binning": False,
        },
        "voltage_models": voltage_models,
        "erpm_only_baseline": erpm_only,
        "operating_points": dual_points,
    }
    for key in ("model", "coverage", "rank", "condition_number", "training_metrics",
                "residuals_by_voltage_layer"):
        if key in erpm_only:
            static[key] = erpm_only[key]
    static["single_drive_installed_baselines"] = {
        "status": "available" if single_points else "unavailable",
        "points": single_points,
        "interpretation": "Installed-assembly baseline only; the other propeller may windmill and is not an isolated-propeller result.",
    }
    static["data_quality"] = {"input_samples": len(sample_list),
                              "accepted_steady_points": len(points),
                              "dual_model_points": len(dual_points),
                              "excluded_samples_by_reason": exclusions}
    power = _fit_power(points, meta)
    dynamics = _fit_dynamics(sample_list, meta)
    validation = voltage_models["dual"]["validation"]
    validation["component"] = "dual_voltage_dependent_total_thrust"
    training_diagnostics = _training_diagnostics(voltage_models["dual"], dual_points, "dual")
    validation_diagnostics = _validation_diagnostics(validation)
    static["training_diagnostics"] = training_diagnostics
    validation["diagnostics"] = validation_diagnostics
    static["sufficiency"] = _sufficiency(
        points, voltage_models["dual"], validation_diagnostics, meta)
    warnings = [
        "This is a total-thrust model of the actual two-propeller installed assembly.",
        "Speed inputs are raw electrical eRPM; no pole-pair or mechanical-RPM conversion is applied.",
        "DShot ESC current is not externally calibrated and remains a telemetry-scale observation.",
        "Board ADC current is diagnostic only and is never used as a fitting fallback.",
        "It cannot uniquely decompose upper/lower thrust or torque; a nominally undriven propeller may windmill.",
        "Results are valid only inside the reported eRPM and voltage coverage.",
        "No model is automatically sent to the flight controller, written to Flash, or substituted for a control thrust table.",
    ]
    if power.get("status") != "available":
        warnings.append("Quantitative input-power calibration is unavailable: " + str(power.get("reason")))
    if validation.get("status") != "available":
        warnings.append("Independent generalisation evidence is unavailable; training fit alone is not validation.")
    if static.get("status") != "available":
        warnings.append("The primary voltage-dependent thrust calibration is unavailable: " +
                        str(static.get("reason")))
    return {"schema_version": ANALYSIS_SCHEMA_VERSION, "static": static,
            "power": power, "dynamics": dynamics, "validation": validation,
            "warnings": warnings}
