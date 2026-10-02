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


# ── 光流安装方向（R-FLOWMOUNT-1）───────────────────────────────────────────
#
# 固件在 app_optical_flow.c 的 app_flow_fill_sample() 里先按最初的安装把芯片读数
# 换成 FLU（记作 v0），再用 airframe.flow_mount_yaw_deg / flow_mount_mirror 纠正：
#     v = R(yaw) · F^mirror · v0，F = diag(1, -1)（翻 Y），R 为绕 +Z 逆时针旋转。
# 8 种 (yaw, mirror) 恰好是 2×2 带号置换矩阵的全部 8 个。
#
# 推荐的思路：+X、+Y 两步是在**当前**安装参数 M 下采的，读数 = M·S·真实位移，S 是
# 未知的物理安装（也是带号置换）。找一个纠正矩阵 C 让 C·读数 ≈ 真实方向，则
# C·M·S = I，新的安装参数 N = C·M = S⁻¹，再把 N 化回 (yaw, mirror)。

FLOW_MOUNT_YAW_CHOICES = (0, 90, 180, 270)
FLOW_MOUNT_MIN_STEP_M = 0.10
FLOW_MOUNT_MIN_COSINE = 0.9

Matrix2 = tuple[tuple[int, int], tuple[int, int]]

_ROTATIONS: dict[int, Matrix2] = {
    0: ((1, 0), (0, 1)),
    90: ((0, -1), (1, 0)),
    180: ((-1, 0), (0, -1)),
    270: ((0, 1), (-1, 0)),
}
_MIRROR_Y: Matrix2 = ((1, 0), (0, -1))


def _matmul(left: Matrix2, right: Matrix2) -> Matrix2:
    return (
        (left[0][0] * right[0][0] + left[0][1] * right[1][0],
         left[0][0] * right[0][1] + left[0][1] * right[1][1]),
        (left[1][0] * right[0][0] + left[1][1] * right[1][0],
         left[1][0] * right[0][1] + left[1][1] * right[1][1]),
    )


def _apply(matrix: Matrix2, vector: tuple[float, float]) -> tuple[float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1],
    )


def _check_flow_mount(yaw_deg: int, mirror: int) -> None:
    if yaw_deg not in FLOW_MOUNT_YAW_CHOICES:
        raise GroundCalibrationError(f"安装转角只能是 0/90/180/270，收到 {yaw_deg!r}")
    if mirror not in (0, 1):
        raise GroundCalibrationError(f"镜像只能是 0 或 1，收到 {mirror!r}")


def flow_mount_matrix(yaw_deg: int, mirror: int) -> Matrix2:
    """固件安装变换的矩阵：先按 mirror 翻 Y，再绕 +Z 逆时针转 yaw。"""

    _check_flow_mount(yaw_deg, mirror)
    rotation = _ROTATIONS[yaw_deg]
    return _matmul(rotation, _MIRROR_Y) if mirror else rotation


def flow_mount_from_matrix(matrix: Matrix2) -> tuple[int, int]:
    """把带号置换矩阵化回 (yaw_deg, mirror)。行列式 -1 的是带镜像的那 4 个。"""

    det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
    mirror = 1 if det < 0 else 0
    rotation = _matmul(matrix, _MIRROR_Y) if mirror else matrix
    for yaw_deg, candidate in _ROTATIONS.items():
        if candidate == rotation:
            return yaw_deg, mirror
    raise GroundCalibrationError(f"不是带号置换矩阵：{matrix!r}")


def describe_flow_mount(yaw_deg: int, mirror: int) -> str:
    """给人看的安装参数，如"旋转 90°、不镜像"。"""

    return f"旋转 {yaw_deg}°、{'镜像' if mirror else '不镜像'}"


_ALL_FLOW_MOUNT_MATRICES: tuple[Matrix2, ...] = tuple(
    flow_mount_matrix(yaw_deg, mirror)
    for mirror in (0, 1)
    for yaw_deg in FLOW_MOUNT_YAW_CHOICES
)


def _observation(value: Sequence[float], label: str) -> tuple[float, float]:
    if len(value) != 2:
        raise GroundCalibrationError(f"{label} 位移必须是 (x, y) 两个数")
    x, y = (float(item) for item in value)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise GroundCalibrationError(f"{label} 位移包含非有限数值")
    return x, y


def _axis_ratio(primary: float, cross: float) -> float:
    return abs(primary) / max(abs(cross), 1.0e-6)


def recommend_flow_mount(
    forward_obs: Sequence[float],
    left_obs: Sequence[float],
    current_yaw_deg: int,
    current_mirror: int,
) -> dict[str, object]:
    """由 +X、+Y 两步的积分位移推荐光流安装参数。

    ``forward_obs`` / ``left_obs`` 是机体分别沿机头（+X）、向左（+Y）推一段之后，
    在**当前**安装参数 (current_yaw_deg, current_mirror) 下积分得到的 (x, y) 位移 [m]。
    返回推荐的 ``yaw_deg`` / ``mirror`` 与依据（两步纠正后的余弦、主/串轴比等），
    全部是可写进证据 JSON 的普通数值。

    判据：两步的位移模长都要 ≥ 0.10 m（否则多半没推动或光流没跟上）；8 个候选里
    取两步余弦和最大者，且两步的余弦都要 ≥ 0.9（约 26° 以内），否则说明两步的方向
    彼此矛盾（例如两步都落在同一根轴上），不给推荐。
    """

    _check_flow_mount(current_yaw_deg, current_mirror)
    forward = _observation(forward_obs, "+X 步")
    left = _observation(left_obs, "+Y 步")
    forward_norm = math.hypot(*forward)
    left_norm = math.hypot(*left)
    for label, norm in (("+X", forward_norm), ("+Y", left_norm)):
        if norm < FLOW_MOUNT_MIN_STEP_M:
            raise GroundCalibrationError(
                f"{label} 步位移只有 {norm:.3f} m（至少 {FLOW_MOUNT_MIN_STEP_M:.2f} m）："
                "这一步没推动或光流没跟上，请重采"
            )

    best: tuple[float, Matrix2, float, float] | None = None
    for correction in _ALL_FLOW_MOUNT_MATRICES:
        forward_cos = _apply(correction, forward)[0] / forward_norm
        left_cos = _apply(correction, left)[1] / left_norm
        score = forward_cos + left_cos
        if best is None or score > best[0]:
            best = (score, correction, forward_cos, left_cos)
    assert best is not None
    _score, correction, forward_cos, left_cos = best
    if forward_cos < FLOW_MOUNT_MIN_COSINE or left_cos < FLOW_MOUNT_MIN_COSINE:
        raise GroundCalibrationError(
            f"两步方向不一致（纠正后余弦 +X={forward_cos:.2f}、+Y={left_cos:.2f}，"
            f"都要 ≥ {FLOW_MOUNT_MIN_COSINE:.1f}）：请沿机头、机体左侧各直推一次后重采"
        )

    current = flow_mount_matrix(current_yaw_deg, current_mirror)
    mount = _matmul(correction, current)
    yaw_deg, mirror = flow_mount_from_matrix(mount)
    corrected_forward = _apply(correction, forward)
    corrected_left = _apply(correction, left)
    return {
        "yaw_deg": yaw_deg,
        "mirror": mirror,
        "description": describe_flow_mount(yaw_deg, mirror),
        "current_yaw_deg": current_yaw_deg,
        "current_mirror": current_mirror,
        "current_description": describe_flow_mount(current_yaw_deg, current_mirror),
        "changed": (yaw_deg, mirror) != (current_yaw_deg, current_mirror),
        "forward_observed_m": [forward[0], forward[1]],
        "left_observed_m": [left[0], left[1]],
        "forward_distance_m": forward_norm,
        "left_distance_m": left_norm,
        "forward_corrected_m": [corrected_forward[0], corrected_forward[1]],
        "left_corrected_m": [corrected_left[0], corrected_left[1]],
        "forward_cosine": forward_cos,
        "left_cosine": left_cos,
        # 纠正之后的主轴 / 串轴位移比；越大说明这一步推得越正。
        "forward_axis_ratio": _axis_ratio(corrected_forward[0], corrected_forward[1]),
        "left_axis_ratio": _axis_ratio(corrected_left[1], corrected_left[0]),
        "correction_matrix": [list(row) for row in correction],
        "mount_matrix": [list(row) for row in mount],
    }


def validate_servo_geometry(center_us: int, minimum_us: int, maximum_us: int) -> None:
    """Reject unsafe or degenerate host-side servo evidence values."""

    if not (500 <= minimum_us < center_us < maximum_us <= 2500):
        raise GroundCalibrationError(
            "舵机行程必须满足 500 <= 最小值 < 中心值 < 最大值 <= 2500 µs"
        )
    if center_us - minimum_us < 50 or maximum_us - center_us < 50:
        raise GroundCalibrationError("中心两侧至少各保留 50 µs 的可验证行程")
