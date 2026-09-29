"""Whole-run dataset training for installed coaxial total thrust.

Report/experiment-library model (models/<date>/training-*), not the flight-controller
thrust mapping; that is the lookup table in thrust_lut.py (models/lut/current.json).

Training owns preprocessing, scaling, coefficients and support domains.
Validation runs are never folded back into fitting; when used to select a
candidate they are explicitly a selection-validation set, not a final test.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from typing import Any

import numpy as np
from scipy.spatial import Delaunay, QhullError

from .model import _convex_hull, _point_in_convex, _steady_points
from .records import BenchSample

DATASET_ANALYSIS_SCHEMA_VERSION = 1
GRAM_FORCE_PER_NEWTON = 1000.0 / 9.80665

REASON_ZH = {
    "insufficient_training_operating_points": "训练稳态工作点不足",
    "rank_deficient_or_ill_conditioned": "输入变化不足，拟合矩阵秩不足或条件数过大",
    "loaded_voltage_span_too_small": "训练中的实际带载电压跨度太小",
    "loaded_voltage_confounding_with_erpm": "实际带载电压与 eRPM 变化混淆，无法单独辨识电压影响",
    "joint_erpm_voltage_domain_rank_deficient": "训练 eRPM/V 联合覆盖退化，无法建立三维支持域",
    "joint_erpm_voltage_hull_unavailable": "训练 eRPM/V 点无法形成稳定的三维支持域",
    "erpm_hull_area_unavailable": "训练 eRPM 点无法形成有面积的二维支持域",
}


def reason_zh(reason: object) -> str:
    return REASON_ZH.get(str(reason), f"模型不可用：{reason}")


def _finite(value: object) -> bool:
    try:
        return value is not None and math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _check_samples(samples: Sequence[BenchSample], name: str) -> set[str]:
    runs = set()
    for sample in samples:
        if not hasattr(sample, "upper_erpm") or hasattr(sample, "upper_rpm"):
            raise ValueError(f"{name}: schema v2 electrical eRPM samples required")
        if sample.speed_source != "dshot_erpm":
            raise ValueError(f"{name}: dshot_erpm source required")
        runs.add(str(sample.run_id))
    return runs


def _dual_points(samples: Sequence[BenchSample], metadata: Mapping[str, Any]) -> tuple[
        list[dict[str, Any]], dict[str, int], dict[str, int]]:
    points, excluded = _steady_points(samples, metadata)
    modes: dict[str, int] = {}
    for point in points:
        mode = str(point.get("mode"))
        modes[mode] = modes.get(mode, 0) + 1
    unsupported = sum(count for mode, count in modes.items() if mode != "dual")
    if unsupported:
        excluded = {**excluded, "unsupported_nondual_mode_points": unsupported}
    return [point for point in points if point.get("mode") == "dual"], excluded, modes


def _metrics(actual: np.ndarray, predicted: np.ndarray) -> dict[str, Any]:
    error = predicted - actual
    absolute = np.abs(error)
    values_n = {
        "mae": float(np.mean(absolute)),
        "rmse": float(np.sqrt(np.mean(error ** 2))),
        "p95_abs_error": float(np.percentile(absolute, 95)),
        "max_abs_error": float(np.max(absolute)),
    }
    return {
        "points": int(len(actual)),
        "newton": values_n,
        "gram_force": {key: value * GRAM_FORCE_PER_NEWTON for key, value in values_n.items()},
    }


def _scales(points: Sequence[Mapping[str, Any]], include_voltage: bool) -> dict[str, float]:
    upper = np.asarray([float(point["upper_erpm"]) for point in points])
    lower = np.asarray([float(point["lower_erpm"]) for point in points])
    result = {"upper_erpm_scale": max(float(np.max(np.abs(upper))), 1.0),
              "lower_erpm_scale": max(float(np.max(np.abs(lower))), 1.0)}
    if include_voltage:
        voltage = np.asarray([float(point["voltage_v"]) for point in points])
        result["voltage_center_v"] = float(np.mean(voltage))
        result["voltage_scale_v"] = max(float(np.ptp(voltage)) / 2.0, 0.05)
    return result


def _coordinates(points: Sequence[Mapping[str, Any]], scale: Mapping[str, float],
                 include_voltage: bool) -> np.ndarray:
    upper = np.asarray([float(point["upper_erpm"]) / scale["upper_erpm_scale"]
                        for point in points])
    lower = np.asarray([float(point["lower_erpm"]) / scale["lower_erpm_scale"]
                        for point in points])
    columns = [upper, lower]
    if include_voltage:
        columns.append(np.asarray([(float(point["voltage_v"]) - scale["voltage_center_v"]) /
                                   scale["voltage_scale_v"] for point in points]))
    return np.column_stack(columns)


def _design(coordinates: np.ndarray, include_voltage: bool) -> np.ndarray:
    upper, lower = coordinates[:, 0], coordinates[:, 1]
    columns = [np.ones(len(coordinates)), upper ** 2, lower ** 2, upper * lower]
    if include_voltage:
        columns.append(coordinates[:, 2])
    return np.column_stack(columns)


def _support_domain(coordinates: np.ndarray, include_voltage: bool) -> dict[str, Any]:
    if include_voltage:
        rank = int(np.linalg.matrix_rank(coordinates - np.mean(coordinates, axis=0)))
        if rank < 3 or len(coordinates) < 5:
            return {"status": "unavailable", "reason": "joint_erpm_voltage_domain_rank_deficient"}
        try:
            Delaunay(coordinates)
        except QhullError:
            return {"status": "unavailable", "reason": "joint_erpm_voltage_hull_unavailable"}
        return {"status": "available", "kind": "training_delaunay_3d",
                "coordinates": coordinates.tolist(), "dimension": 3}
    hull = _convex_hull([(float(row[0]), float(row[1])) for row in coordinates])
    if len(hull) < 3:
        return {"status": "unavailable", "reason": "erpm_hull_area_unavailable"}
    return {"status": "available", "kind": "training_convex_hull_2d",
            "coordinates": [[x, y] for x, y in hull], "dimension": 2}


def _inside_domain(domain: Mapping[str, Any], coordinate: np.ndarray) -> bool:
    if domain.get("status") != "available":
        return False
    if domain["kind"] == "training_convex_hull_2d":
        return _point_in_convex(domain["coordinates"], (float(coordinate[0]), float(coordinate[1])))
    try:
        triangulation = Delaunay(np.asarray(domain["coordinates"], dtype=float))
        return bool(triangulation.find_simplex(coordinate) >= 0)
    except QhullError:
        return False


def _fit_candidate(points: Sequence[Mapping[str, Any]], *, include_voltage: bool,
                   metadata: Mapping[str, Any]) -> dict[str, Any]:
    kind = "continuous_loaded_voltage" if include_voltage else "erpm_only"
    usable = [point for point in points if all(_finite(point.get(field)) for field in
              ("upper_erpm", "lower_erpm", "thrust_n", "voltage_v"))]
    parameter_count = 5 if include_voltage else 4
    if len(usable) < max(parameter_count + 2, 8):
        return {"status": "unavailable", "kind": kind,
                "reason": "insufficient_training_operating_points", "training_points": len(usable)}
    scale = _scales(usable, True)
    coordinates = _coordinates(usable, scale, include_voltage)
    design = _design(coordinates, include_voltage)
    voltage_identifiability = None
    if include_voltage:
        voltage_span = float(np.ptp([float(point["voltage_v"]) for point in usable]))
        if voltage_span < float(metadata.get("min_loaded_voltage_span_v", 0.30)):
            return {"status": "unavailable", "kind": kind,
                    "reason": "loaded_voltage_span_too_small", "voltage_span_v": voltage_span}
        voltage_column = coordinates[:, 2]
        base = design[:, :4]
        fitted_voltage = base @ np.linalg.lstsq(base, voltage_column, rcond=None)[0]
        denominator = float(np.linalg.norm(voltage_column - np.mean(voltage_column)))
        residual_fraction = (float(np.linalg.norm(voltage_column - fitted_voltage)) / denominator
                             if denominator > 1.0e-12 else 0.0)
        voltage_identifiability = {"voltage_span_v": voltage_span,
                                   "residual_fraction_after_erpm_features": residual_fraction,
                                   "minimum_fraction": float(metadata.get(
                                       "min_voltage_identifiability_fraction", 0.10))}
        if residual_fraction < voltage_identifiability["minimum_fraction"]:
            return {"status": "unavailable", "kind": kind,
                    "reason": "loaded_voltage_confounding_with_erpm",
                    "voltage_identifiability": voltage_identifiability}
    rank = int(np.linalg.matrix_rank(design))
    condition = float(np.linalg.cond(design))
    if rank < parameter_count or not math.isfinite(condition) or condition > float(
            metadata.get("max_design_condition", 1.0e8)):
        return {"status": "unavailable", "kind": kind,
                "reason": "rank_deficient_or_ill_conditioned", "rank": rank,
                "condition_number": condition if math.isfinite(condition) else None}
    applicability = {
        "upper_erpm": [min(float(point["upper_erpm"]) for point in usable),
                       max(float(point["upper_erpm"]) for point in usable)],
        "lower_erpm": [min(float(point["lower_erpm"]) for point in usable),
                       max(float(point["lower_erpm"]) for point in usable)],
        "loaded_voltage_v": [min(float(point["voltage_v"]) for point in usable),
                             max(float(point["voltage_v"]) for point in usable)],
    }
    if include_voltage:
        domain = _support_domain(coordinates, True)
        if domain["status"] != "available":
            return {"status": "unavailable", "kind": kind, "reason": domain["reason"],
                    "domain": domain, "applicability": applicability}
        coverage_claim = "joint_erpm_voltage_training_domain"
        joint_domain = {"status": "available", "reason": None}
    else:
        joint_coordinates = _coordinates(usable, scale, True)
        joint_domain = _support_domain(joint_coordinates, True)
        if joint_domain.get("status") == "available":
            domain = joint_domain
            coverage_claim = "joint_erpm_voltage_training_domain_for_support_only"
        else:
            domain = _support_domain(coordinates, False)
            coverage_claim = "two_dimensional_erpm_hull_plus_loaded_voltage_range_only"
            if domain["status"] != "available":
                return {"status": "unavailable", "kind": kind, "reason": domain["reason"],
                        "domain": domain, "applicability": applicability,
                        "joint_domain": joint_domain}
    ridge = float(metadata.get("dataset_ridge", 1.0e-8))
    penalty = np.eye(design.shape[1]) * ridge
    penalty[0, 0] = 0.0
    coefficients = np.linalg.solve(design.T @ design + penalty, design.T @ np.asarray(
        [float(point["thrust_n"]) for point in usable]))
    predicted = design @ coefficients
    rows = [{"run_id": str(point["run_id"]), "segment_id": str(point["segment_id"]),
             "upper_erpm": float(point["upper_erpm"]), "lower_erpm": float(point["lower_erpm"]),
             "voltage_v": float(point["voltage_v"]) if _finite(point.get("voltage_v")) else None,
             "actual_thrust_n": float(point["thrust_n"]),
             "predicted_thrust_n": float(prediction),
             "residual_n": float(prediction - float(point["thrust_n"]))}
            for point, prediction in zip(usable, predicted)]
    return {"status": "available", "kind": kind,
            "features": ["1", "upper_erpm_squared", "lower_erpm_squared", "erpm_cross"] +
                        (["loaded_voltage_linear"] if include_voltage else []),
            "scale": scale, "coefficients": coefficients.tolist(), "ridge": ridge,
            "rank": rank, "condition_number": condition, "domain": domain,
            "joint_domain": joint_domain, "coverage_claim": coverage_claim,
            "applicability": applicability,
            "training_points": len(usable), "training_metrics": _metrics(
                np.asarray([float(point["thrust_n"]) for point in usable]), predicted),
            "training_predictions": rows,
            "voltage_identifiability": voltage_identifiability}


def _predict(candidate: Mapping[str, Any], point: Mapping[str, Any]) -> tuple[float | None, str | None]:
    include_voltage = candidate.get("kind") == "continuous_loaded_voltage"
    fields = ("upper_erpm", "lower_erpm", "voltage_v")
    if not all(_finite(point.get(field)) for field in fields):
        return None, "missing_required_input"
    voltage_low, voltage_high = candidate["applicability"]["loaded_voltage_v"]
    if not voltage_low <= float(point["voltage_v"]) <= voltage_high:
        return None, "outside_training_loaded_voltage_range"
    coordinates = _coordinates([point], candidate["scale"], include_voltage)[0]
    support_coordinates = (_coordinates([point], candidate["scale"], True)[0]
                           if candidate["domain"].get("dimension") == 3 else coordinates)
    if not _inside_domain(candidate["domain"], support_coordinates):
        return None, "outside_training_support_domain"
    prediction = float(_design(coordinates.reshape(1, -1), include_voltage)[0] @ np.asarray(
        candidate["coefficients"]))
    return prediction, None


def predict_dataset_thrust(analysis: Mapping[str, Any], *, upper_erpm: float,
                           lower_erpm: float, voltage_v: float) -> float:
    """Predict with the selected candidate, enforcing its training support domain."""
    if analysis.get("schema_version") != DATASET_ANALYSIS_SCHEMA_VERSION:
        raise ValueError("unsupported_dataset_analysis_schema")
    selected = analysis.get("selection", {}).get("selected_candidate")
    if not selected:
        raise ValueError("no_selected_candidate")
    candidate = analysis.get("candidates", {}).get(selected, {})
    if candidate.get("status") != "available":
        raise ValueError("selected_candidate_unavailable")
    prediction, reason = _predict(candidate, {
        "upper_erpm": upper_erpm, "lower_erpm": lower_erpm,
        "voltage_v": voltage_v})
    if prediction is None:
        raise ValueError(str(reason))
    return prediction


def _validate_candidate(candidate: Mapping[str, Any],
                        points: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    if candidate.get("status") != "available":
        return {"status": "unavailable", "reason": "candidate_unavailable",
                "predictions": [], "coverage_fraction": 0.0}
    rows, rejected = [], {}
    for point in points:
        prediction, reason = _predict(candidate, point)
        if prediction is None:
            rejected[reason] = rejected.get(reason, 0) + 1
            continue
        rows.append({"run_id": str(point["run_id"]), "segment_id": str(point["segment_id"]),
                     "upper_erpm": point.get("upper_erpm"), "lower_erpm": point.get("lower_erpm"),
                     "voltage_v": point.get("voltage_v"), "actual_thrust_n": float(point["thrust_n"]),
                     "predicted_thrust_n": prediction,
                     "residual_n": prediction - float(point["thrust_n"])})
    actual = np.asarray([row["actual_thrust_n"] for row in rows])
    predicted = np.asarray([row["predicted_thrust_n"] for row in rows])
    return {"status": "available" if rows else "unavailable",
            "role": "candidate_selection_validation_not_final_test",
            "points": len(rows), "input_points": len(points),
            "coverage_fraction": len(rows) / len(points) if points else 0.0,
            "rejected": rejected, "metrics": _metrics(actual, predicted) if rows else None,
            "predictions": rows}


def _next_steps(candidates: Mapping[str, Any], selected: str | None,
                validation: Mapping[str, Any]) -> list[str]:
    steps = []
    voltage = candidates["continuous_loaded_voltage"]
    if voltage.get("status") != "available":
        reason = voltage.get("reason")
        if reason == "loaded_voltage_confounding_with_erpm":
            steps.append("跨不同电量重复相同指令组合，并交错或正反扫描，降低电压与 eRPM 共线。")
        elif reason == "loaded_voltage_span_too_small":
            steps.append("补充不同实际带载电压的完整重复轮次；不要只依靠一次连续放电。")
        else:
            steps.append(f"连续电压候选不可用：{reason_zh(reason)}。补齐其训练覆盖后再比较。")
    if validation.get("status") != "available":
        steps.append("当前只有训练诊断；增加与训练 run_id 不重叠的完整验证轮。")
    if (validation.get("input_points", 0) and
            validation.get("coverage_fraction", 0.0) < 1.0):
        steps.append("验证点部分落在训练联合支持域外；在被拒绝的 eRPM/V 区域补训练工况。")
    if selected is None:
        steps.append("尚无可选候选；不要生成或下发推力预测表。")
    return steps


def train_dataset(train_samples: Sequence[BenchSample],
                  validation_samples: Sequence[BenchSample],
                  metadata: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Fit candidates on train runs and compare them on disjoint held-out runs."""
    meta = dict(metadata or {})
    train_list, validation_list = list(train_samples), list(validation_samples)
    train_runs = _check_samples(train_list, "train")
    validation_runs = _check_samples(validation_list, "validation")
    overlap = sorted(train_runs & validation_runs)
    if overlap:
        raise ValueError(f"train/validation run_id overlap: {overlap}")
    declared_train = {str(value) for value in meta.get("train_run_ids", [])}
    declared_validation = {str(value) for value in meta.get("validation_run_ids", [])}
    warnings = []
    if declared_train and declared_train != train_runs:
        warnings.append("metadata_train_run_ids_do_not_match_samples")
    if declared_validation and declared_validation != validation_runs:
        warnings.append("metadata_validation_run_ids_do_not_match_samples")
    train_points, train_excluded, train_modes = _dual_points(train_list, meta)
    validation_points, validation_excluded, validation_modes = _dual_points(
        validation_list, meta)
    if any(mode != "dual" for mode in train_modes):
        warnings.append("train_nondual_modes_are_not_supported_by_this_total_thrust_dataset_model")
    if any(mode != "dual" for mode in validation_modes):
        warnings.append("validation_nondual_modes_are_not_supported_by_this_total_thrust_dataset_model")
    candidates = {
        "erpm_only": _fit_candidate(train_points, include_voltage=False, metadata=meta),
        "continuous_loaded_voltage": _fit_candidate(
            train_points, include_voltage=True, metadata=meta),
    }
    validation_results = {name: _validate_candidate(candidate, validation_points)
                          for name, candidate in candidates.items()}
    minimum_coverage = float(meta.get("minimum_selection_validation_coverage", 1.0))
    error_limit_gf = float(meta.get("prediction_error_limit_gf", 50.0))
    if not math.isfinite(error_limit_gf) or error_limit_gf <= 0:
        raise ValueError("预测误差目标须为正的克力数")
    eligible = [(name, result) for name, result in validation_results.items()
                if result.get("status") == "available" and
                result.get("coverage_fraction", 0.0) >= minimum_coverage]
    selected = min(eligible, key=lambda item: (
        item[1]["metrics"]["gram_force"]["max_abs_error"] > error_limit_gf,
        item[1]["metrics"]["gram_force"]["max_abs_error"]
        if item[1]["metrics"]["gram_force"]["max_abs_error"] > error_limit_gf else 0.0,
        item[1]["metrics"]["gram_force"]["rmse"]))[0] \
        if eligible else ("erpm_only" if candidates["erpm_only"].get("status") == "available" else None)
    selection = {
        "selected_candidate": selected,
        "basis": ("selection_validation_50gf_worst_case_then_rmse_with_coverage_gate" if eligible else
                  "training_only_default_simpler_candidate" if selected else "no_available_candidate"),
        "minimum_validation_coverage": minimum_coverage,
        "validation_is_final_test": False,
        "coefficients_refit_after_selection": False,
    }
    selected_validation = validation_results.get(selected, {}) if selected else {}
    selected_training = candidates.get(selected, {}) if selected else {}
    next_steps = _next_steps(candidates, selected, selected_validation)
    selection_coverage=selected_validation.get("coverage_fraction",0.0)
    selection_metrics=(selected_validation.get("metrics") or {}).get("gram_force") or {}
    if not validation_points:
        target_status="unvalidated"
    elif not eligible or selection_coverage < minimum_coverage:
        target_status="insufficient_coverage"
    elif selection_metrics.get("max_abs_error",math.inf) <= error_limit_gf:
        target_status="meets_selection_target"
    else:
        target_status="above_tolerance"
        next_steps.insert(0,f"留出轮最大误差 {selection_metrics['max_abs_error']:.1f} 克力；"
                             f"在对应 eRPM/V 工况补数据，目标为每点不超过 {error_limit_gf:g} 克力。")
    acceptance={"status":target_status,"max_allowed_abs_error_gf":error_limit_gf,
                "observed_max_abs_error_gf":selection_metrics.get("max_abs_error"),
                "validation_coverage_fraction":selection_coverage,
                "selection_validation_is_final_test":False}
    if selection["basis"] == "selection_validation_50gf_worst_case_then_rmse_with_coverage_gate":
        grams = selected_validation["metrics"]["gram_force"]
        summary = (f"训练 {len(train_runs)} 轮、选择验证 {len(validation_runs)} 轮；"
                   f"平均相差 {grams['mae']:.1f} 克力，最大相差 {grams['max_abs_error']:.1f} 克力，"
                   f"{'达到' if target_status == 'meets_selection_target' else '未达到'}每点不超过 {error_limit_gf:g} 克力的目标；预测覆盖 "
                   f"{selected_validation.get('coverage_fraction', 0.0):.0%}；"
                   f"下一步：{next_steps[0] if next_steps else '补一组未参与选择的最终测试轮。'}")
    elif selected_validation.get("status") == "available":
        grams = selected_validation["metrics"]["gram_force"]
        summary = (f"训练 {len(train_runs)} 轮；验证仅局部可预测，覆盖 "
                   f"{selected_validation.get('coverage_fraction', 0.0):.0%}，可预测点平均相差 "
                   f"{grams['mae']:.1f} 克力，未达到选模覆盖门；"
                   f"下一步：{next_steps[0] if next_steps else '补齐训练支持域。'}")
    else:
        summary = (f"训练 {len(train_runs)} 轮，当前只有训练诊断、没有可用选择验证；"
                   f"下一步：{next_steps[0] if next_steps else '增加独立验证轮。'}")
    return {
        "schema_version": DATASET_ANALYSIS_SCHEMA_VERSION,
        "analysis_kind": "whole_run_dataset_candidate_selection",
        "summary": summary,
        "prediction_acceptance": acceptance,
        "compatibility_identity": meta.get("compatibility_identity"),
        "source_manifest": meta.get("source_manifest"),
        "runs": {"train": sorted(train_runs), "validation": sorted(validation_runs),
                 "overlap": overlap},
        "data": {"train_raw_samples": len(train_list), "validation_raw_samples": len(validation_list),
                 "train_steady_points": len(train_points),
                 "validation_steady_points": len(validation_points),
                 "train_mode_points": train_modes,
                 "validation_mode_points": validation_modes,
                 "train_unsupported_nondual_points": sum(
                     count for mode, count in train_modes.items() if mode != "dual"),
                 "validation_unsupported_nondual_points": sum(
                     count for mode, count in validation_modes.items() if mode != "dual"),
                 "train_excluded_by_reason": train_excluded,
                 "validation_excluded_by_reason": validation_excluded,
                 "train_operating_points": train_points,
                 "validation_operating_points": validation_points},
        "candidates": candidates,
        "selection_validation": validation_results,
        "selection": selection,
        "selected_training_diagnostics": selected_training.get("training_metrics"),
        "selected_validation_diagnostics": selected_validation,
        "selected_applicability": selected_training.get("applicability"),
        "selected_support_domain": selected_training.get("domain"),
        "next_steps": next_steps,
        "warnings": warnings + [
            "Selection validation was used to choose a candidate and is not an untouched final test.",
            "Predictions outside the training joint support domain are rejected.",
            "No model is automatically sent to firmware or written to Flash.",
        ],
    }
