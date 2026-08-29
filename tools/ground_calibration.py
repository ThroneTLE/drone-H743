"""Host-only analysis for ground calibration evidence.

The functions in this module never write target parameters.  They turn samples
captured by :mod:`drone_tcp_panel` into auditable diagnostics for optical-flow,
range and servo-mechanical setup.  Keeping the maths outside Tkinter makes the
acceptance rules executable and testable.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import Iterable, Sequence


class GroundCalibrationError(ValueError):
    """Raised when evidence is insufficient or physically inconsistent."""


@dataclass(frozen=True)
class FlowRangeSample:
    host_time_s: float
    vx_m_s: float
    vy_m_s: float
    height_raw_m: float | None = None
    height_m: float | None = None
    gyro_z_dps: float | None = None
    vx_compensated_m_s: float | None = None
    vy_compensated_m_s: float | None = None
    quality: int | None = None
    frame_contract: int | None = None
    orientation_code: int | None = None


def _finite(values: Iterable[float]) -> list[float]:
    result = [float(value) for value in values]
    if not result or not all(math.isfinite(value) for value in result):
        raise GroundCalibrationError("样本为空或包含非有限数值")
    return result


def _require_samples(samples: Sequence[FlowRangeSample], minimum: int = 5) -> None:
    if len(samples) < minimum:
        raise GroundCalibrationError(f"至少需要 {minimum} 个有效样本，当前只有 {len(samples)} 个")
    times = _finite(sample.host_time_s for sample in samples)
    if any(right <= left for left, right in zip(times, times[1:])):
        raise GroundCalibrationError("样本时间戳必须严格递增")


def _mean_std(values: Iterable[float]) -> tuple[float, float]:
    finite = _finite(values)
    return statistics.fmean(finite), statistics.pstdev(finite)


def analyze_flow_zero(samples: Sequence[FlowRangeSample]) -> dict[str, float | int]:
    """Summarise stationary flow zero-offset/noise without inventing limits."""

    _require_samples(samples)
    vx_mean, vx_std = _mean_std(sample.vx_m_s for sample in samples)
    vy_mean, vy_std = _mean_std(sample.vy_m_s for sample in samples)
    speed_rms = math.sqrt(statistics.fmean(
        sample.vx_m_s * sample.vx_m_s + sample.vy_m_s * sample.vy_m_s
        for sample in samples
    ))
    return {
        "sample_count": len(samples),
        "duration_s": samples[-1].host_time_s - samples[0].host_time_s,
        "vx_mean_m_s": vx_mean,
        "vy_mean_m_s": vy_mean,
        "vx_std_m_s": vx_std,
        "vy_std_m_s": vy_std,
        "speed_rms_m_s": speed_rms,
    }


def _trapezoid(samples: Sequence[FlowRangeSample], field: str) -> float:
    total = 0.0
    for left, right in zip(samples, samples[1:]):
        dt = right.host_time_s - left.host_time_s
        total += 0.5 * (float(getattr(left, field)) + float(getattr(right, field))) * dt
    return total


def analyze_flow_axis(
    samples: Sequence[FlowRangeSample],
    *,
    expected_axis: str,
    reference_distance_m: float,
) -> dict[str, float | int | bool | str | None]:
    """Check FLU sign/dominance and estimate one-axis velocity scale.

    The operator moves the airframe by a measured positive displacement.  The
    returned scale is diagnostic only: ``true_distance / integrated_distance``.
    """

    _require_samples(samples)
    if expected_axis not in {"x", "y"}:
        raise GroundCalibrationError("expected_axis 只能是 x 或 y")
    if not math.isfinite(reference_distance_m) or reference_distance_m <= 0.0:
        raise GroundCalibrationError("参考位移必须为正有限值")

    primary_field = "vx_m_s" if expected_axis == "x" else "vy_m_s"
    cross_field = "vy_m_s" if expected_axis == "x" else "vx_m_s"
    observed = _trapezoid(samples, primary_field)
    cross = _trapezoid(samples, cross_field)
    sign_ok = observed > 0.0
    dominance = abs(observed) / max(abs(cross), 1.0e-6)
    scale = reference_distance_m / observed if sign_ok and abs(observed) > 1.0e-6 else None
    return {
        "sample_count": len(samples),
        "expected_axis": expected_axis,
        "reference_distance_m": reference_distance_m,
        "observed_distance_m": observed,
        "cross_axis_distance_m": cross,
        "positive_sign_ok": sign_ok,
        "axis_dominance_ratio": dominance,
        "diagnostic_scale": scale,
    }


def fit_range_two_point(
    near_samples: Sequence[FlowRangeSample],
    far_samples: Sequence[FlowRangeSample],
    *,
    near_reference_m: float,
    far_reference_m: float,
) -> dict[str, float | int]:
    """Fit ``true_height = scale * measured_height + offset`` from two stands."""

    _require_samples(near_samples)
    _require_samples(far_samples)
    if not (0.0 < near_reference_m < far_reference_m):
        raise GroundCalibrationError("远距离参考值必须大于近距离参考值，且二者都为正")
    near_values = _finite(
        sample.height_raw_m for sample in near_samples if sample.height_raw_m is not None
    )
    far_values = _finite(
        sample.height_raw_m for sample in far_samples if sample.height_raw_m is not None
    )
    near_mean = statistics.fmean(near_values)
    far_mean = statistics.fmean(far_values)
    measured_span = far_mean - near_mean
    if measured_span <= 1.0e-4:
        raise GroundCalibrationError("测距远近两组均值没有形成有效正跨度")
    scale = (far_reference_m - near_reference_m) / measured_span
    offset = near_reference_m - scale * near_mean
    return {
        "near_sample_count": len(near_values),
        "far_sample_count": len(far_values),
        "near_measured_mean_m": near_mean,
        "far_measured_mean_m": far_mean,
        "near_measured_std_m": statistics.pstdev(near_values),
        "far_measured_std_m": statistics.pstdev(far_values),
        "range_scale": scale,
        "range_offset_m": offset,
    }


def analyze_rotation_compensation(
    samples: Sequence[FlowRangeSample],
) -> dict[str, float | int | bool | str]:
    """Evaluate rotation residual only when post-compensation velocity exists."""

    _require_samples(samples)
    gyro = _finite(
        sample.gyro_z_dps for sample in samples if sample.gyro_z_dps is not None
    )
    raw_rms = math.sqrt(statistics.fmean(
        sample.vx_m_s * sample.vx_m_s + sample.vy_m_s * sample.vy_m_s
        for sample in samples
    ))
    compensated = [
        sample for sample in samples
        if sample.vx_compensated_m_s is not None and sample.vy_compensated_m_s is not None
    ]
    result: dict[str, float | int | bool | str] = {
        "sample_count": len(samples),
        "gyro_abs_mean_dps": statistics.fmean(abs(value) for value in gyro),
        "raw_flow_speed_rms_m_s": raw_rms,
        "supported": len(compensated) == len(samples),
    }
    if len(compensated) != len(samples):
        result["reason"] = "目标未上报旋转补偿后的速度，当前证据不能判定通过"
        return result
    compensated_rms = math.sqrt(statistics.fmean(
        float(sample.vx_compensated_m_s) ** 2 + float(sample.vy_compensated_m_s) ** 2
        for sample in compensated
    ))
    gyro_abs_mean = float(result["gyro_abs_mean_dps"])
    motion_ok = gyro_abs_mean >= 15.0
    residual_ok = compensated_rms <= 0.08
    result["compensated_speed_rms_m_s"] = compensated_rms
    result["motion_ok"] = motion_ok
    result["residual_ok"] = residual_ok
    result["passed"] = motion_ok and residual_ok
    result["reason"] = (
        "PASS：偏航动作充分且补偿后残余速度不超过 0.08 m/s"
        if result["passed"] else
        "FAIL：偏航均值需至少 15 dps，补偿后残余速度需不超过 0.08 m/s"
    )
    return result


def validate_servo_geometry(center_us: int, minimum_us: int, maximum_us: int) -> None:
    """Reject unsafe or degenerate host-side servo evidence values."""

    if not (500 <= minimum_us < center_us < maximum_us <= 2500):
        raise GroundCalibrationError(
            "舵机行程必须满足 500 <= 最小值 < 中心值 < 最大值 <= 2500 µs"
        )
    if center_us - minimum_us < 50 or maximum_us - center_us < 50:
        raise GroundCalibrationError("中心两侧至少各保留 50 µs 的可验证行程")
