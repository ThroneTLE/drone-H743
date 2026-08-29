#!/usr/bin/env python3
"""Host-only V1 IMU metrology and immutable calibration evidence.

V0 answers the discrete frame question.  This module starts *after* that
boundary and estimates continuous accelerometer/gyroscope corrections in the
canonical FLU body frame.  It has no transport, parameter, Flash, or firmware
API.  Even a fully passing candidate is evidence only until a separate guarded
review/apply workflow accepts it.

The numerical fit uses NumPy, which is already used by the project's analysis
tools.  Importing this module remains safe without NumPy; only numerical
analysis calls then raise :class:`NumpyRequiredError` with an actionable error.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, fields, is_dataclass
from datetime import datetime
from enum import Enum
from types import MappingProxyType
from typing import Any, Mapping, Sequence

try:  # Keep schemas/JSON usable on a minimal host installation.
    import numpy as _np
except ImportError:  # pragma: no cover - exercised by monkeypatch in tests.
    _np = None


Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]

CANONICAL_FRAME_ID = "FLU"
SUPPORTED_FRAME_CONTRACT_VERSION = 1
METROLOGY_FORMAT = "drone-h743-imu-metrology-candidate"
METROLOGY_SCHEMA = 2
NUMPY_AVAILABLE = _np is not None


class NumpyRequiredError(RuntimeError):
    """Raised when a numerical V1 analysis is requested without NumPy."""


class MetrologyStatus(str, Enum):
    PASS = "PASS"
    WARN = "WARN"
    FAIL = "FAIL"
    INCOMPLETE = "INCOMPLETE"
    NOT_RUN = "NOT_RUN"


class CaptureMethod(str, Enum):
    """How the physical reference for a sample was established."""

    BENCH = "bench"
    MANUAL = "manual"
    REFERENCE_FIXTURE = "reference_fixture"
    V0_IMPORTED = "v0_imported"


class MetrologyStage(str, Enum):
    ACCEL_POS_X = "accel_pos_x"
    ACCEL_NEG_X = "accel_neg_x"
    ACCEL_POS_Y = "accel_pos_y"
    ACCEL_NEG_Y = "accel_neg_y"
    ACCEL_POS_Z = "accel_pos_z"
    ACCEL_NEG_Z = "accel_neg_z"
    GYRO_STATIC = "gyro_static"
    GYRO_POS_360_X = "gyro_pos_360_x"
    GYRO_POS_360_Y = "gyro_pos_360_y"
    GYRO_POS_360_Z = "gyro_pos_360_z"
    TEMPERATURE_STATIC = "temperature_static"


@dataclass(frozen=True)
class StageDefinition:
    stage: MetrologyStage
    label_zh: str
    kind: str
    expected_specific_force_g: Vec3 | None = None
    expected_rotation_deg: Vec3 | None = None


_STAGE_DEFINITIONS = {
    MetrologyStage.ACCEL_POS_X: StageDefinition(
        MetrologyStage.ACCEL_POS_X, "加速度 +X（机头朝上）", "accel_face", (1.0, 0.0, 0.0)
    ),
    MetrologyStage.ACCEL_NEG_X: StageDefinition(
        MetrologyStage.ACCEL_NEG_X, "加速度 -X（机头朝下）", "accel_face", (-1.0, 0.0, 0.0)
    ),
    MetrologyStage.ACCEL_POS_Y: StageDefinition(
        MetrologyStage.ACCEL_POS_Y, "加速度 +Y（左侧朝上）", "accel_face", (0.0, 1.0, 0.0)
    ),
    MetrologyStage.ACCEL_NEG_Y: StageDefinition(
        MetrologyStage.ACCEL_NEG_Y, "加速度 -Y（右侧朝上）", "accel_face", (0.0, -1.0, 0.0)
    ),
    MetrologyStage.ACCEL_POS_Z: StageDefinition(
        MetrologyStage.ACCEL_POS_Z, "加速度 +Z（水平）", "accel_face", (0.0, 0.0, 1.0)
    ),
    MetrologyStage.ACCEL_NEG_Z: StageDefinition(
        MetrologyStage.ACCEL_NEG_Z, "加速度 -Z（倒置）", "accel_face", (0.0, 0.0, -1.0)
    ),
    MetrologyStage.GYRO_STATIC: StageDefinition(
        MetrologyStage.GYRO_STATIC, "陀螺仪静止", "gyro_static"
    ),
    MetrologyStage.GYRO_POS_360_X: StageDefinition(
        MetrologyStage.GYRO_POS_360_X,
        "+X 精确 360°",
        "gyro_rotation",
        expected_rotation_deg=(360.0, 0.0, 0.0),
    ),
    MetrologyStage.GYRO_POS_360_Y: StageDefinition(
        MetrologyStage.GYRO_POS_360_Y,
        "+Y 精确 360°",
        "gyro_rotation",
        expected_rotation_deg=(0.0, 360.0, 0.0),
    ),
    MetrologyStage.GYRO_POS_360_Z: StageDefinition(
        MetrologyStage.GYRO_POS_360_Z,
        "+Z 精确 360°",
        "gyro_rotation",
        expected_rotation_deg=(0.0, 0.0, 360.0),
    ),
    MetrologyStage.TEMPERATURE_STATIC: StageDefinition(
        MetrologyStage.TEMPERATURE_STATIC,
        "温度平台静止水平",
        "temperature_static",
        expected_specific_force_g=(0.0, 0.0, 1.0),
    ),
}

STAGE_DEFINITIONS: Mapping[MetrologyStage, StageDefinition] = MappingProxyType(
    _STAGE_DEFINITIONS
)
ACCEL_FACE_STAGES = (
    MetrologyStage.ACCEL_POS_X,
    MetrologyStage.ACCEL_NEG_X,
    MetrologyStage.ACCEL_POS_Y,
    MetrologyStage.ACCEL_NEG_Y,
    MetrologyStage.ACCEL_POS_Z,
    MetrologyStage.ACCEL_NEG_Z,
)
GYRO_ROTATION_STAGES = (
    MetrologyStage.GYRO_POS_360_X,
    MetrologyStage.GYRO_POS_360_Y,
    MetrologyStage.GYRO_POS_360_Z,
)
# 手转 360° 的采集窗口里，动手之前和收手之后必然有静止段。把它们一起积分，只会把
# bias 残差乘上静止时长塞进角度里；判定该只看真正在转的那一段。
GYRO_ROTATION_MOTION_FLOOR_DPS = 5.0
GYRO_ROTATION_MOTION_FRACTION = 0.10


def gyro_rotation_motion_window(rates: Any, axis: int) -> tuple[int, int]:
    """返回主轴真正在转的连续区间 ``[start, stop)``；无法判定时返回整段。

    阈值取"主轴峰值的 10%"与 5 dps 中的较大者：慢转也能框住，静止噪声框不住。
    只裁首尾，不剔中间停顿 —— 转到一半停一下仍是同一次转动，剔掉会把角度算少。
    """

    np = _require_numpy()
    values = np.abs(np.asarray(rates, dtype=float)[:, axis])
    if values.size == 0:
        return 0, 0
    threshold = max(GYRO_ROTATION_MOTION_FLOOR_DPS,
                    float(np.max(values)) * GYRO_ROTATION_MOTION_FRACTION)
    moving = np.nonzero(values >= threshold)[0]
    if moving.size < 2:
        return 0, int(values.size)
    return int(moving[0]), int(moving[-1]) + 1


@dataclass(frozen=True)
class MetrologyThresholds:
    """Auditable V1 acceptance limits.

    Accelerometer limits are deliberately hard gates.  A candidate outside any
    one of them is rejected instead of being hidden by a good aggregate score.
    """

    accel_min_samples_per_face: int = 1500
    accel_min_duration_s: float = 1.0
    accel_static_std_max_g: float = 0.030
    accel_condition_max: float = 1.25
    accel_singular_min: float = 0.85
    accel_singular_max: float = 1.15
    accel_identity_deviation_max: float = 0.25
    accel_diagonal_min: float = 0.75
    accel_off_diagonal_max: float = 0.20
    accel_bias_norm_max_g: float = 0.20
    # 残差衡量的是"六面之间摆得一致不一致"，不是数据坏没坏 —— 坏数据由 std、
    # 奇异值、对角/非对角那几道门先拦掉。
    #
    # 原来 0.025 g 单档硬拒，等价于要求同一对正负面的倾角差 ≤6°，而这个不一致
    # 换算成飞行代价只有约 1° 的水平误差 —— PX4 干脆没有这道检查（它的 3x3 只用
    # 三个正面求逆，负面只贡献零偏，天然没有残差可判）。徒手摆六面达不到 ≤6°，
    # 拿 1° 的代价去拦人不合理。
    #
    # 现在分两档：PASS 线保持 0.025 g（≈1.4°，摆得好就该给 PASS），超过 0.075 g
    # （≈4.3°，实测折合约 3° 水平误差）才 FAIL，中间给 WARN 并把折合角度报出来，
    # 让人自己决定收不收。
    accel_corrected_rms_max_g: float = 0.025
    accel_corrected_max_error_g: float = 0.05
    accel_corrected_rms_fail_g: float = 0.075
    accel_corrected_max_error_fail_g: float = 0.11
    gyro_static_min_samples: int = 4000
    gyro_static_min_duration_s: float = 3.0
    gyro_static_std_pass_dps: float = 0.50
    gyro_static_std_fail_dps: float = 1.50
    gyro_rotation_min_samples_per_axis: int = 50
    gyro_rotation_min_duration_s: float = 0.50
    gyro_condition_max: float = 1.25
    gyro_singular_min: float = 0.85
    gyro_singular_max: float = 1.15
    gyro_cross_axis_pass_fraction: float = 0.05
    gyro_cross_axis_fail_fraction: float = 0.10
    temperature_min_platforms: int = 3
    temperature_min_span_c: float = 15.0
    temperature_min_samples_per_platform: int = 4000
    temperature_min_duration_s: float = 3.0
    temperature_platform_span_max_c: float = 1.0
    temperature_accel_std_max_g: float = 0.030
    temperature_gyro_std_max_dps: float = 1.0
    temperature_level_error_max_g: float = 0.15
    temperature_gyro_mean_norm_max_dps: float = 5.0
    temperature_accel_slope_abs_max_g_per_c: float = 0.003
    temperature_gyro_slope_abs_max_dps_per_c: float = 0.20
    temperature_accel_fit_rms_max_g: float = 0.010
    temperature_gyro_fit_rms_max_dps: float = 0.10
    temperature_fit_r2_min: float = 0.80

    def __post_init__(self) -> None:
        integer_fields = (
            "accel_min_samples_per_face",
            "gyro_static_min_samples",
            "gyro_rotation_min_samples_per_axis",
            "temperature_min_platforms",
            "temperature_min_samples_per_platform",
        )
        for name in integer_fields:
            _strict_int(getattr(self, name), name=name, minimum=1)
        nonnegative_fields = (
            "accel_min_duration_s",
            "gyro_static_min_duration_s",
            "gyro_rotation_min_duration_s",
            "temperature_min_duration_s",
        )
        for name in nonnegative_fields:
            if _finite(getattr(self, name), name=name) < 0.0:
                raise ValueError(f"{name} must be >= 0")
        positive_fields = tuple(
            item.name
            for item in fields(self)
            if item.name not in integer_fields
            and item.name not in nonnegative_fields
            and item.name != "temperature_fit_r2_min"
        )
        for name in positive_fields:
            if _finite(getattr(self, name), name=name) <= 0.0:
                raise ValueError(f"{name} must be > 0")
        r2 = _finite(self.temperature_fit_r2_min, name="temperature_fit_r2_min")
        if not 0.0 <= r2 <= 1.0:
            raise ValueError("temperature_fit_r2_min must be in [0, 1]")

def _finite(value: Any, *, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _strict_int(value: Any, *, name: str, minimum: int = 0) -> int:
    if type(value) is not int:
        raise TypeError(f"{name} must be an integer")
    if value < minimum:
        raise ValueError(f"{name} must be >= {minimum}")
    return value


def _nonempty(value: Any, *, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise TypeError(f"{name} must be a non-empty string")
    return value.strip()


def _binding(value: Any, *, name: str) -> str:
    result = _nonempty(value, name=name)
    if result.casefold() in {"unknown", "none", "unavailable", "legacy"}:
        raise ValueError(f"{name} must identify a concrete capture binding")
    return result


def _enum_value(enum_type: type[Enum], value: Any, *, name: str) -> Any:
    if isinstance(value, enum_type):
        return value
    if not isinstance(value, str):
        raise TypeError(f"{name} must be {enum_type.__name__} or string")
    try:
        return enum_type(value)
    except ValueError as exc:
        raise ValueError(f"unknown {name}: {value!r}") from exc


DEFAULT_THRESHOLDS = MetrologyThresholds()


@dataclass(frozen=True)
class MetrologySample:
    """One fully-provenanced V1 sample in canonical FLU units.

    ``orientation_code`` is the already-applied proper sensor rotation (0..23),
    while ``calibration_generation`` identifies the continuous calibration that
    was active while capturing the evidence.  This prevents reports from
    silently mixing old and new target state.
    """

    timestamp_s: float
    accel_x_g: float
    accel_y_g: float
    accel_z_g: float
    gyro_x_dps: float
    gyro_y_dps: float
    gyro_z_dps: float
    temperature_c: float
    stage: MetrologyStage | str
    frame_id: str
    frame_contract_version: int
    orientation_code: int
    calibration_generation: int
    firmware_hash: str
    session_id: str
    capture_source: str
    provenance: str
    capture_method: CaptureMethod | str
    temperature_platform: str | None = None

    def __post_init__(self) -> None:
        for name in (
            "timestamp_s",
            "accel_x_g",
            "accel_y_g",
            "accel_z_g",
            "gyro_x_dps",
            "gyro_y_dps",
            "gyro_z_dps",
            "temperature_c",
        ):
            object.__setattr__(self, name, _finite(getattr(self, name), name=name))
        stage = _enum_value(MetrologyStage, self.stage, name="stage")
        method = _enum_value(CaptureMethod, self.capture_method, name="capture_method")
        object.__setattr__(self, "stage", stage)
        object.__setattr__(self, "capture_method", method)
        object.__setattr__(self, "frame_id", _nonempty(self.frame_id, name="frame_id"))
        object.__setattr__(
            self,
            "frame_contract_version",
            _strict_int(
                self.frame_contract_version,
                name="frame_contract_version",
                minimum=1,
            ),
        )
        if self.frame_contract_version != SUPPORTED_FRAME_CONTRACT_VERSION:
            raise ValueError(
                "frame_contract_version must equal the supported canonical contract "
                f"version {SUPPORTED_FRAME_CONTRACT_VERSION}"
            )
        orientation = _strict_int(self.orientation_code, name="orientation_code")
        if orientation > 23:
            raise ValueError("orientation_code must identify a proper rotation (0..23)")
        object.__setattr__(self, "orientation_code", orientation)
        object.__setattr__(
            self,
            "calibration_generation",
            _strict_int(
                self.calibration_generation,
                name="calibration_generation",
            ),
        )
        object.__setattr__(
            self, "provenance", _nonempty(self.provenance, name="provenance")
        )
        object.__setattr__(
            self, "firmware_hash", _binding(self.firmware_hash, name="firmware_hash")
        )
        object.__setattr__(
            self, "session_id", _binding(self.session_id, name="session_id")
        )
        object.__setattr__(
            self,
            "capture_source",
            _binding(self.capture_source, name="capture_source"),
        )
        platform = self.temperature_platform
        if stage is MetrologyStage.TEMPERATURE_STATIC:
            platform = _nonempty(platform, name="temperature_platform")
        elif platform is not None:
            raise ValueError(
                "temperature_platform is only valid for temperature_static samples"
            )
        object.__setattr__(self, "temperature_platform", platform)

    @property
    def accel_g(self) -> Vec3:
        return (self.accel_x_g, self.accel_y_g, self.accel_z_g)

    @property
    def gyro_dps(self) -> Vec3:
        return (self.gyro_x_dps, self.gyro_y_dps, self.gyro_z_dps)


@dataclass(frozen=True)
class StageCount:
    stage: str
    sample_count: int

    def __post_init__(self) -> None:
        _nonempty(self.stage, name="stage")
        _strict_int(self.sample_count, name="sample_count")


@dataclass(frozen=True)
class AccelerometerCalibrationResult:
    status: MetrologyStatus
    sample_count: int
    stage_counts: tuple[StageCount, ...]
    bias_g: Vec3 | None
    bias_norm_g: float | None
    correction_matrix: Mat3 | None
    determinant: float | None
    singular_values: Vec3 | None
    condition_number: float | None
    corrected_rms_g: float | None
    corrected_max_error_g: float | None
    findings: tuple[str, ...]


@dataclass(frozen=True)
class GyroStaticResult:
    status: MetrologyStatus
    sample_count: int
    bias_dps: Vec3 | None
    std_dps: Vec3 | None
    noise_vector_rms_dps: float | None
    findings: tuple[str, ...]


@dataclass(frozen=True)
class GyroRotationStageResult:
    stage: str
    sample_count: int
    duration_s: float
    integrated_angle_deg: Vec3 | None
    main_axis_scale: float | None
    cross_axis_fraction: float | None
    timestamps_monotonic: bool
    capture_methods: tuple[str, ...]


@dataclass(frozen=True)
class GyroRotationResult:
    status: MetrologyStatus
    sample_count: int
    stages: tuple[GyroRotationStageResult, ...]
    correction_matrix: Mat3 | None
    determinant: float | None
    singular_values: Vec3 | None
    condition_number: float | None
    corrected_integrals_deg: Mat3 | None
    findings: tuple[str, ...]


@dataclass(frozen=True)
class TemperaturePlatformResult:
    platform: str
    sample_count: int
    mean_temperature_c: float
    temperature_span_c: float
    accel_mean_g: Vec3
    gyro_mean_dps: Vec3
    accel_std_g: Vec3
    gyro_std_dps: Vec3
    corrected_accel_mean_g: Vec3
    corrected_gyro_mean_dps: Vec3


@dataclass(frozen=True)
class TemperatureDriftResult:
    status: MetrologyStatus
    sample_count: int
    platforms: tuple[TemperaturePlatformResult, ...]
    temperature_span_c: float | None
    reference_temperature_c: float | None
    accel_slope_g_per_c: Vec3 | None
    gyro_slope_dps_per_c: Vec3 | None
    accel_fit_rms_g: Vec3 | None
    gyro_fit_rms_dps: Vec3 | None
    accel_fit_r2: Vec3 | None
    gyro_fit_r2: Vec3 | None
    findings: tuple[str, ...]


@dataclass(frozen=True)
class MetrologyCandidate:
    """Immutable V1 calibration candidate; never an apply command."""

    created_at: str
    data_source: str
    status: MetrologyStatus
    frame_id: str
    frame_contract_version: int
    orientation_code: int
    base_calibration_generation: int
    candidate_calibration_generation: int
    firmware_hash: str
    session_id: str
    capture_source: str
    sample_count: int
    source_provenance: tuple[str, ...]
    capture_methods: tuple[str, ...]
    accelerometer: AccelerometerCalibrationResult
    gyro_static: GyroStaticResult
    gyro_rotation: GyroRotationResult
    temperature: TemperatureDriftResult
    thresholds: MetrologyThresholds
    findings: tuple[str, ...]
    format: str = field(default=METROLOGY_FORMAT, init=False)
    schema: int = field(default=METROLOGY_SCHEMA, init=False)
    evidence_only: bool = field(default=True, init=False)
    applied: bool = field(default=False, init=False)
    parameter_changes_applied: bool = field(default=False, init=False)
    parameters_written: bool = field(default=False, init=False)
    flash_writes: int = field(default=0, init=False)
    flight_release: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        try:
            datetime.fromisoformat(self.created_at.replace("Z", "+00:00"))
        except (AttributeError, ValueError) as exc:
            raise ValueError("created_at must be an ISO-8601 timestamp") from exc
        _nonempty(self.data_source, name="data_source")
        if self.frame_id != CANONICAL_FRAME_ID:
            raise ValueError("V1 candidates must be expressed in canonical FLU")
        _strict_int(
            self.frame_contract_version,
            name="frame_contract_version",
            minimum=1,
        )
        if self.frame_contract_version != SUPPORTED_FRAME_CONTRACT_VERSION:
            raise ValueError(
                "candidate frame_contract_version is not supported"
            )
        orientation = _strict_int(self.orientation_code, name="orientation_code")
        if orientation > 23:
            raise ValueError("orientation_code must be in 0..23")
        base = _strict_int(
            self.base_calibration_generation,
            name="base_calibration_generation",
        )
        candidate_generation = _strict_int(
            self.candidate_calibration_generation,
            name="candidate_calibration_generation",
        )
        if candidate_generation != base + 1:
            raise ValueError("candidate_calibration_generation must equal base + 1")
        _binding(self.firmware_hash, name="firmware_hash")
        _binding(self.session_id, name="session_id")
        _binding(self.capture_source, name="capture_source")
        _strict_int(self.sample_count, name="sample_count")
        for value in self.source_provenance:
            _nonempty(value, name="source_provenance item")
        for value in self.capture_methods:
            _enum_value(CaptureMethod, value, name="capture_methods item")
        if not isinstance(self.thresholds, MetrologyThresholds):
            raise TypeError("thresholds must be MetrologyThresholds")
        _validate_candidate_semantics(self)


def _require_numpy() -> Any:
    if _np is None:
        raise NumpyRequiredError(
            "V1 IMU metrology requires NumPy; install it with "
            "'python -m pip install numpy'."
        )
    return _np


def _sample_tuple(samples: Sequence[MetrologySample]) -> tuple[MetrologySample, ...]:
    if isinstance(samples, (str, bytes, bytearray)) or not isinstance(samples, Sequence):
        raise TypeError("samples must be a sequence of MetrologySample")
    result = tuple(samples)
    for index, sample in enumerate(result):
        if not isinstance(sample, MetrologySample):
            raise TypeError(f"samples[{index}] must be a MetrologySample")
    return result


def _uniform_context(
    samples: Sequence[MetrologySample],
) -> tuple[str, int, int, int, str, str, str]:
    if not samples:
        raise ValueError("at least one metrology sample is required")
    contexts = {
        (
            sample.frame_id,
            sample.frame_contract_version,
            sample.orientation_code,
            sample.calibration_generation,
            sample.firmware_hash,
            sample.session_id,
            sample.capture_source,
        )
        for sample in samples
    }
    if len(contexts) != 1:
        # 只说"混了上下文"等于什么都没说：把真正分叉的那一项和它的取值点出来，
        # 用户才知道该重采哪几步。
        names = ("frame_id", "frame_contract_version", "orientation_code",
                 "calibration_generation", "firmware_hash", "session_id", "capture_source")
        divergent = []
        for index, name in enumerate(names):
            values = sorted({str(context[index]) for context in contexts})
            if len(values) > 1:
                divergent.append(f"{name}={'/'.join(values)}")
        raise ValueError(
            "samples mix capture context: " + "; ".join(divergent)
            + "。同一份候选必须来自同一台飞机的同一次固件与同一代标定，请重采分叉的步骤。"
        )
    context = next(iter(contexts))
    if context[0] != CANONICAL_FRAME_ID:
        raise ValueError("V1 metrology only accepts canonical frame_id='FLU'")
    if context[1] != SUPPORTED_FRAME_CONTRACT_VERSION:
        raise ValueError("V1 metrology received an unsupported frame contract version")
    return context


def _as_vec3(value: Any) -> Vec3:
    return (float(value[0]), float(value[1]), float(value[2]))


def _as_mat3(value: Any) -> Mat3:
    return (_as_vec3(value[0]), _as_vec3(value[1]), _as_vec3(value[2]))


def _stage_counts(
    grouped: Mapping[MetrologyStage, Sequence[MetrologySample]],
    stages: Sequence[MetrologyStage],
) -> tuple[StageCount, ...]:
    return tuple(StageCount(stage.value, len(grouped.get(stage, ()))) for stage in stages)


def _contains_v0(samples: Sequence[MetrologySample]) -> bool:
    return any(sample.capture_method is CaptureMethod.V0_IMPORTED for sample in samples)


def _cap_v0_pass(
    status: MetrologyStatus,
    samples: Sequence[MetrologySample],
    findings: list[str],
) -> MetrologyStatus:
    if _contains_v0(samples):
        findings.append(
            "包含 V0 导入样本；它们可用于预估，但不能自动取得 V1 PASS。"
        )
        if status is MetrologyStatus.PASS:
            return MetrologyStatus.WARN
    return status


def fit_accelerometer(
    samples: Sequence[MetrologySample],
    thresholds: MetrologyThresholds = DEFAULT_THRESHOLDS,
) -> AccelerometerCalibrationResult:
    """Fit ``corrected = C @ (observed - bias)`` from all six FLU faces."""

    np = _require_numpy()
    sample_list = _sample_tuple(samples)
    relevant = tuple(sample for sample in sample_list if sample.stage in ACCEL_FACE_STAGES)
    grouped = {
        stage: tuple(sample for sample in relevant if sample.stage is stage)
        for stage in ACCEL_FACE_STAGES
    }
    counts = _stage_counts(grouped, ACCEL_FACE_STAGES)
    findings: list[str] = []
    if not relevant:
        return AccelerometerCalibrationResult(
            MetrologyStatus.NOT_RUN,
            0,
            counts,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            ("尚未采集加速度计 ±X/±Y/±Z 六面数据。",),
        )
    _uniform_context(relevant)
    missing = [stage.value for stage in ACCEL_FACE_STAGES if not grouped[stage]]
    if missing:
        return AccelerometerCalibrationResult(
            MetrologyStatus.INCOMPLETE,
            len(relevant),
            counts,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            ("缺少六面阶段: " + ", ".join(missing),),
        )

    status = MetrologyStatus.PASS
    short = [
        stage.value
        for stage in ACCEL_FACE_STAGES
        if len(grouped[stage]) < thresholds.accel_min_samples_per_face
    ]
    if short:
        status = MetrologyStatus.INCOMPLETE
        findings.append(
            "以下六面样本数不足: " + ", ".join(short)
        )

    face_quality_failures: list[str] = []
    short_duration: list[str] = []
    for stage in ACCEL_FACE_STAGES:
        rows = grouped[stage]
        times = np.asarray([sample.timestamp_s for sample in rows], dtype=float)
        deltas = np.diff(times)
        monotonic = bool(np.all(deltas > 0.0))
        duration = float(times[-1] - times[0]) if len(times) >= 2 and monotonic else 0.0
        values = np.asarray([sample.accel_g for sample in rows], dtype=float)
        maximum_std = float(np.max(np.std(values, axis=0)))
        if not monotonic:
            face_quality_failures.append(f"{stage.value} 时间戳不是严格递增。")
        if duration < thresholds.accel_min_duration_s:
            short_duration.append(stage.value)
        if maximum_std > thresholds.accel_static_std_max_g:
            face_quality_failures.append(
                f"{stage.value} 静止标准差 {maximum_std:.6f} g，超过 "
                f"{thresholds.accel_static_std_max_g:.3f} g。"
            )
    if short_duration and status is not MetrologyStatus.FAIL:
        status = MetrologyStatus.INCOMPLETE
        findings.append(
            "以下六面持续时间不足: " + ", ".join(short_duration)
        )

    means = {
        stage: np.mean(np.asarray([sample.accel_g for sample in grouped[stage]], dtype=float), axis=0)
        for stage in ACCEL_FACE_STAGES
    }
    pairs = (
        (MetrologyStage.ACCEL_POS_X, MetrologyStage.ACCEL_NEG_X),
        (MetrologyStage.ACCEL_POS_Y, MetrologyStage.ACCEL_NEG_Y),
        (MetrologyStage.ACCEL_POS_Z, MetrologyStage.ACCEL_NEG_Z),
    )
    bias = np.mean(np.asarray([(means[pos] + means[neg]) * 0.5 for pos, neg in pairs]), axis=0)
    sensitivity = np.column_stack([(means[pos] - means[neg]) * 0.5 for pos, neg in pairs])
    try:
        correction = np.linalg.inv(sensitivity)
    except np.linalg.LinAlgError:
        return AccelerometerCalibrationResult(
            MetrologyStatus.FAIL,
            len(relevant),
            counts,
            _as_vec3(bias),
            float(np.linalg.norm(bias)),
            None,
            None,
            None,
            None,
            None,
            None,
            tuple(findings + ["六面响应矩阵奇异，无法求出 3x3 校正矩阵。"]),
        )

    determinant = float(np.linalg.det(correction))
    singular = np.linalg.svd(correction, compute_uv=False)
    condition = float(singular[0] / singular[-1])
    identity_deviation = float(np.linalg.norm(correction - np.eye(3), ord="fro"))
    minimum_diagonal = float(np.min(np.diag(correction)))
    off_diagonal = correction - np.diag(np.diag(correction))
    maximum_off_diagonal = float(np.max(np.abs(off_diagonal)))
    bias_norm = float(np.linalg.norm(bias))
    errors: list[float] = []
    for stage in ACCEL_FACE_STAGES:
        expected = np.asarray(STAGE_DEFINITIONS[stage].expected_specific_force_g, dtype=float)
        observed = np.asarray([sample.accel_g for sample in grouped[stage]], dtype=float)
        corrected = (correction @ (observed - bias).T).T
        errors.extend(np.linalg.norm(corrected - expected, axis=1).tolist())
    rms = float(math.sqrt(sum(error * error for error in errors) / len(errors)))
    maximum = float(max(errors))

    failures: list[str] = list(face_quality_failures)
    if determinant <= 0.0:
        failures.append(f"det(C)={determinant:.6f}，必须 > 0")
    if condition > thresholds.accel_condition_max:
        failures.append(
            f"condition={condition:.6f}，超过 {thresholds.accel_condition_max:.2f}"
        )
    if float(singular[-1]) < thresholds.accel_singular_min or float(singular[0]) > thresholds.accel_singular_max:
        failures.append(
            "校正矩阵奇异值不在 "
            f"[{thresholds.accel_singular_min:.2f}, {thresholds.accel_singular_max:.2f}]"
        )
    if identity_deviation > thresholds.accel_identity_deviation_max:
        failures.append(
            f"校正矩阵距单位阵 {identity_deviation:.6f}，超过 "
            f"{thresholds.accel_identity_deviation_max:.2f}；应先完成 V0 离散轴映射。"
        )
    if minimum_diagonal < thresholds.accel_diagonal_min:
        failures.append(
            f"校正矩阵最小对角 {minimum_diagonal:.6f}，低于 "
            f"{thresholds.accel_diagonal_min:.2f}；V1 不允许吸收 V0 极性/错轴。"
        )
    if maximum_off_diagonal > thresholds.accel_off_diagonal_max:
        failures.append(
            f"校正矩阵最大非对角 {maximum_off_diagonal:.6f}，超过 "
            f"{thresholds.accel_off_diagonal_max:.2f}；V1 只接受小非正交修正。"
        )
    if bias_norm > thresholds.accel_bias_norm_max_g:
        failures.append(
            f"|bias|={bias_norm:.6f} g，超过 {thresholds.accel_bias_norm_max_g:.2f} g"
        )
    # 残差单独一档：只说明六面摆得一致不一致，折合角度报出来让人判断。
    inconsistency: list[str] = []
    if rms > thresholds.accel_corrected_rms_fail_g:
        failures.append(
            f"校正后 RMS={rms:.4f} g（折合约 {math.degrees(math.asin(min(1.0, rms))):.1f}°），"
            f"超过 {thresholds.accel_corrected_rms_fail_g:.3f} g；六面摆放差异过大"
        )
    elif rms > thresholds.accel_corrected_rms_max_g:
        inconsistency.append(
            f"六面一致性折合约 {math.degrees(math.asin(min(1.0, rms))):.1f}°"
            f"（RMS={rms:.4f} g，理想 ≤{thresholds.accel_corrected_rms_max_g:.3f} g）；"
            "可用，但同一对正负面用同一个支撑重采会更准"
        )
    if maximum > thresholds.accel_corrected_max_error_fail_g:
        failures.append(
            f"校正后最大误差={maximum:.4f} g，超过 {thresholds.accel_corrected_max_error_fail_g:.2f} g"
        )
    if failures:
        status = MetrologyStatus.FAIL
        findings.extend(failures)
    elif inconsistency:
        if status is MetrologyStatus.PASS:
            status = MetrologyStatus.WARN
        findings.extend(inconsistency)
    status = _cap_v0_pass(status, relevant, findings)
    return AccelerometerCalibrationResult(
        status=status,
        sample_count=len(relevant),
        stage_counts=counts,
        bias_g=_as_vec3(bias),
        bias_norm_g=bias_norm,
        correction_matrix=_as_mat3(correction),
        determinant=determinant,
        singular_values=_as_vec3(singular),
        condition_number=condition,
        corrected_rms_g=rms,
        corrected_max_error_g=maximum,
        findings=tuple(findings),
    )


def accelerometer_face_residuals(
    samples: Sequence[MetrologySample],
    result: AccelerometerCalibrationResult,
) -> dict[str, tuple[float, float]]:
    """逐面返回 (校正后 RMS 残差 g, 相对理想轴的倾角 °)。

    六面是一起拟合的，整体 FAIL 时不指出是哪一面摆歪了，用户只能六面全重来。

    3x3 拟合能吸收一对正负面的"平均倾角"（当成传感器装歪），但吸收不了同一对
    内部的不对称：+Y 歪 2°、-Y 歪 10°，任何矩阵都没法同时满足两边，差值的一半
    会变成两面都甩不掉的残差。所以残差高说明的是"这一对摆得不一致"。
    """

    if result.correction_matrix is None or result.bias_g is None:
        return {}
    try:
        np = _require_numpy()
    except Exception:
        return {}
    correction = np.asarray(result.correction_matrix, dtype=float)
    bias = np.asarray(result.bias_g, dtype=float)
    residuals: dict[str, tuple[float, float]] = {}
    for stage in ACCEL_FACE_STAGES:
        rows = [sample for sample in samples if sample.stage is stage]
        if not rows:
            continue
        expected = np.asarray(STAGE_DEFINITIONS[stage].expected_specific_force_g, dtype=float)
        observed = np.asarray([sample.accel_g for sample in rows], dtype=float)
        corrected = (correction @ (observed - bias).T).T
        errors = np.linalg.norm(corrected - expected, axis=1)
        rms = float(math.sqrt(float(np.mean(errors ** 2))))
        # 倾角只看原始读数：这是用户真正控制得了的量。
        mean = np.mean(observed - bias, axis=0)
        norm = float(np.linalg.norm(mean))
        cosine = float(np.dot(mean, expected)) / norm if norm > 0.0 else 0.0
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
        residuals[stage.value] = (rms, tilt)
    return residuals


def analyze_stationary_gyro(
    samples: Sequence[MetrologySample],
    thresholds: MetrologyThresholds = DEFAULT_THRESHOLDS,
) -> GyroStaticResult:
    """Estimate stationary gyro bias and zero-rate noise."""

    np = _require_numpy()
    sample_list = _sample_tuple(samples)
    relevant = tuple(
        sample for sample in sample_list if sample.stage is MetrologyStage.GYRO_STATIC
    )
    if not relevant:
        return GyroStaticResult(
            MetrologyStatus.NOT_RUN,
            0,
            None,
            None,
            None,
            ("尚未采集静止陀螺仪数据。",),
        )
    _uniform_context(relevant)
    values = np.asarray([sample.gyro_dps for sample in relevant], dtype=float)
    times = np.asarray([sample.timestamp_s for sample in relevant], dtype=float)
    deltas = np.diff(times)
    monotonic = bool(np.all(deltas > 0.0))
    duration = float(times[-1] - times[0]) if len(times) >= 2 and monotonic else 0.0
    bias = np.mean(values, axis=0)
    std = np.std(values, axis=0)
    rms = float(math.sqrt(float(np.mean(np.sum((values - bias) ** 2, axis=1)))))
    maximum_std = float(np.max(std))
    findings: list[str] = []
    incomplete = False
    if len(relevant) < thresholds.gyro_static_min_samples:
        incomplete = True
        findings.append(
            f"静止陀螺样本 {len(relevant)}，需要至少 {thresholds.gyro_static_min_samples}。"
        )
    if duration < thresholds.gyro_static_min_duration_s:
        incomplete = True
        findings.append(
            f"静止陀螺持续 {duration:.3f} s，需要至少 "
            f"{thresholds.gyro_static_min_duration_s:.3f} s。"
        )
    if not monotonic:
        status = MetrologyStatus.FAIL
        findings.append("静止陀螺时间戳不是严格递增。")
    elif maximum_std > thresholds.gyro_static_std_fail_dps:
        status = MetrologyStatus.FAIL
        findings.append("静止陀螺噪声超过允许上限。")
    elif incomplete:
        status = MetrologyStatus.INCOMPLETE
    elif maximum_std <= thresholds.gyro_static_std_pass_dps:
        status = MetrologyStatus.PASS
    else:
        status = MetrologyStatus.WARN
        findings.append("静止陀螺噪声高于 PASS 阈值。")
    status = _cap_v0_pass(status, relevant, findings)
    return GyroStaticResult(
        status=status,
        sample_count=len(relevant),
        bias_dps=_as_vec3(bias),
        std_dps=_as_vec3(std),
        noise_vector_rms_dps=rms,
        findings=tuple(findings),
    )


def _rotation_axis(stage: MetrologyStage) -> int:
    return GYRO_ROTATION_STAGES.index(stage)


def analyze_gyro_rotations(
    samples: Sequence[MetrologySample],
    *,
    gyro_bias_dps: Vec3 | None,
    thresholds: MetrologyThresholds = DEFAULT_THRESHOLDS,
) -> GyroRotationResult:
    """Fit a full gyro correction from referenced +360° rotations.

    Hand rotations are useful direction/rough-scale evidence, but an operator
    cannot establish exactly 360° at metrology accuracy; consequently manual
    captures are capped at WARN.
    """

    np = _require_numpy()
    sample_list = _sample_tuple(samples)
    relevant = tuple(sample for sample in sample_list if sample.stage in GYRO_ROTATION_STAGES)
    grouped = {
        stage: tuple(sample for sample in relevant if sample.stage is stage)
        for stage in GYRO_ROTATION_STAGES
    }
    findings: list[str] = []
    if not relevant:
        return GyroRotationResult(
            MetrologyStatus.NOT_RUN,
            0,
            (),
            None,
            None,
            None,
            None,
            None,
            ("尚未采集 +360° X/Y/Z 陀螺仪数据。",),
        )
    _uniform_context(relevant)
    if gyro_bias_dps is None:
        return GyroRotationResult(
            MetrologyStatus.INCOMPLETE,
            len(relevant),
            (),
            None,
            None,
            None,
            None,
            None,
            ("缺少同代静止陀螺 bias，不能进行 360° 积分拟合。",),
        )
    bias = np.asarray([_finite(value, name="gyro_bias_dps") for value in gyro_bias_dps], dtype=float)
    if bias.shape != (3,):
        raise ValueError("gyro_bias_dps must contain exactly three values")

    stage_results: list[GyroRotationStageResult] = []
    integrated_columns: list[Any] = []
    status = MetrologyStatus.PASS
    for stage in GYRO_ROTATION_STAGES:
        rows = grouped[stage]
        methods = tuple(sorted({sample.capture_method.value for sample in rows}))
        if len(rows) < 2:
            status = MetrologyStatus.INCOMPLETE
            stage_results.append(
                GyroRotationStageResult(
                    stage.value, len(rows), 0.0, None, None, None, False, methods
                )
            )
            continue
        times = np.asarray([sample.timestamp_s for sample in rows], dtype=float)
        deltas = np.diff(times)
        monotonic = bool(np.all(deltas > 0.0))
        duration = float(times[-1] - times[0]) if monotonic else 0.0
        if monotonic:
            rates = np.asarray([sample.gyro_dps for sample in rows], dtype=float) - bias
            axis = _rotation_axis(stage)
            start, stop = gyro_rotation_motion_window(rates, axis)
            if stop - start < 2:
                start, stop = 0, len(rows)
            if stop - start < len(rows):
                findings.append(
                    f"{stage.value} 已截取运动段 {times[stop - 1] - times[start]:.2f}s / "
                    f"{stop - start} 样本（采集共 {duration:.2f}s / {len(rows)} 样本）。"
                )
            integrated = np.trapz(rates[start:stop], times[start:stop], axis=0)
            main = float(integrated[axis])
            off = max(abs(float(integrated[index])) for index in range(3) if index != axis)
            # Never place Infinity in evidence: a zero main-axis integral is a
            # failed measurement, represented explicitly by null plus FAIL.
            scale = 360.0 / main if main != 0.0 else None
            cross = off / abs(main) if main != 0.0 else None
            integrated_columns.append(integrated)
            integrated_tuple: Vec3 | None = _as_vec3(integrated)
        else:
            scale = None
            cross = None
            integrated_tuple = None
            status = MetrologyStatus.FAIL
            findings.append(f"{stage.value} 时间戳不是严格递增。")
        if len(rows) < thresholds.gyro_rotation_min_samples_per_axis:
            if status is not MetrologyStatus.FAIL:
                status = MetrologyStatus.INCOMPLETE
            findings.append(f"{stage.value} 样本数不足。")
        if duration < thresholds.gyro_rotation_min_duration_s:
            if status is not MetrologyStatus.FAIL:
                status = MetrologyStatus.INCOMPLETE
            findings.append(f"{stage.value} 持续时间不足。")
        stage_results.append(
            GyroRotationStageResult(
                stage=stage.value,
                sample_count=len(rows),
                duration_s=duration,
                integrated_angle_deg=integrated_tuple,
                main_axis_scale=scale,
                cross_axis_fraction=cross,
                timestamps_monotonic=monotonic,
                capture_methods=methods,
            )
        )

    if len(integrated_columns) != 3:
        return GyroRotationResult(
            status=status if status is MetrologyStatus.FAIL else MetrologyStatus.INCOMPLETE,
            sample_count=len(relevant),
            stages=tuple(stage_results),
            correction_matrix=None,
            determinant=None,
            singular_values=None,
            condition_number=None,
            corrected_integrals_deg=None,
            findings=tuple(findings),
        )

    response = np.column_stack(integrated_columns) / 360.0
    try:
        correction = np.linalg.inv(response)
    except np.linalg.LinAlgError:
        return GyroRotationResult(
            MetrologyStatus.FAIL,
            len(relevant),
            tuple(stage_results),
            None,
            None,
            None,
            None,
            None,
            tuple(findings + ["三轴 360° 响应矩阵奇异。"]),
        )
    determinant = float(np.linalg.det(correction))
    singular = np.linalg.svd(correction, compute_uv=False)
    condition = float(singular[0] / singular[-1])
    corrected_integrals = correction @ np.column_stack(integrated_columns)
    numerical_failures: list[str] = []
    if determinant <= 0.0:
        numerical_failures.append("陀螺 3x3 校正矩阵 determinant 必须 > 0。")
    if condition > thresholds.gyro_condition_max:
        numerical_failures.append("陀螺 3x3 校正矩阵 condition 超限。")
    if float(singular[-1]) < thresholds.gyro_singular_min or float(singular[0]) > thresholds.gyro_singular_max:
        numerical_failures.append("陀螺主轴 scale 超出允许范围。")
    for stage_result in stage_results:
        if stage_result.main_axis_scale is None or stage_result.main_axis_scale <= 0.0:
            numerical_failures.append(f"{stage_result.stage} 主轴方向/scale 无效。")
        if (
            stage_result.cross_axis_fraction is not None
            and stage_result.cross_axis_fraction > thresholds.gyro_cross_axis_fail_fraction
        ):
            numerical_failures.append(f"{stage_result.stage} cross-axis 超过 FAIL 阈值。")
        elif (
            stage_result.cross_axis_fraction is not None
            and stage_result.cross_axis_fraction > thresholds.gyro_cross_axis_pass_fraction
            and status is MetrologyStatus.PASS
        ):
            status = MetrologyStatus.WARN
            findings.append(f"{stage_result.stage} cross-axis 高于 PASS 阈值。")
    if numerical_failures:
        status = MetrologyStatus.FAIL
        findings.extend(numerical_failures)

    methods = {sample.capture_method for sample in relevant}
    if methods != {CaptureMethod.REFERENCE_FIXTURE}:
        findings.append("360° 非精密转台数据只能作为 WARN 级证据。")
        if status is MetrologyStatus.PASS:
            status = MetrologyStatus.WARN
    status = _cap_v0_pass(status, relevant, findings)
    return GyroRotationResult(
        status=status,
        sample_count=len(relevant),
        stages=tuple(stage_results),
        correction_matrix=_as_mat3(correction),
        determinant=determinant,
        singular_values=_as_vec3(singular),
        condition_number=condition,
        corrected_integrals_deg=_as_mat3(corrected_integrals),
        findings=tuple(findings),
    )


def _linear_fit_metrics(x: Any, y: Any) -> tuple[Any, Any, Any] | None:
    np = _require_numpy()
    centered = x - float(np.mean(x))
    denominator = float(np.dot(centered, centered))
    if denominator <= 0.0:
        return None
    slope = (centered[:, None] * y).sum(axis=0) / denominator
    prediction = np.mean(y, axis=0) + centered[:, None] * slope
    residual = y - prediction
    rms = np.sqrt(np.mean(residual**2, axis=0))
    residual_energy = np.sum(residual**2, axis=0)
    centered_y = y - np.mean(y, axis=0)
    total_energy = np.sum(centered_y**2, axis=0)
    r2 = np.empty(3, dtype=float)
    for axis in range(3):
        if float(total_energy[axis]) <= 1.0e-18:
            r2[axis] = 1.0 if float(residual_energy[axis]) <= 1.0e-18 else 0.0
        else:
            r2[axis] = 1.0 - float(residual_energy[axis] / total_energy[axis])
    return slope, rms, r2


def analyze_temperature_drift(
    samples: Sequence[MetrologySample],
    *,
    accel_bias_g: Vec3 | None = None,
    accel_correction_matrix: Mat3 | None = None,
    gyro_bias_dps: Vec3 | None = None,
    gyro_correction_matrix: Mat3 | None = None,
    thresholds: MetrologyThresholds = DEFAULT_THRESHOLDS,
) -> TemperatureDriftResult:
    """Fit linear bias drift from stationary level temperature plateaus."""

    np = _require_numpy()
    sample_list = _sample_tuple(samples)
    relevant = tuple(
        sample
        for sample in sample_list
        if sample.stage is MetrologyStage.TEMPERATURE_STATIC
    )
    if not relevant:
        return TemperatureDriftResult(
            status=MetrologyStatus.NOT_RUN,
            sample_count=0,
            platforms=(),
            temperature_span_c=None,
            reference_temperature_c=None,
            accel_slope_g_per_c=None,
            gyro_slope_dps_per_c=None,
            accel_fit_rms_g=None,
            gyro_fit_rms_dps=None,
            accel_fit_r2=None,
            gyro_fit_r2=None,
            findings=("尚未采集温度平台；温漂为 NOT_RUN。",),
        )
    _uniform_context(relevant)
    grouped: dict[str, list[MetrologySample]] = {}
    for sample in relevant:
        assert sample.temperature_platform is not None
        grouped.setdefault(sample.temperature_platform, []).append(sample)

    accel_bias = np.zeros(3) if accel_bias_g is None else np.asarray(accel_bias_g, dtype=float)
    accel_correction = np.eye(3) if accel_correction_matrix is None else np.asarray(accel_correction_matrix, dtype=float)
    gyro_bias = np.zeros(3) if gyro_bias_dps is None else np.asarray(gyro_bias_dps, dtype=float)
    gyro_correction = np.eye(3) if gyro_correction_matrix is None else np.asarray(gyro_correction_matrix, dtype=float)
    for name, value, shape in (
        ("accel_bias_g", accel_bias, (3,)),
        ("accel_correction_matrix", accel_correction, (3, 3)),
        ("gyro_bias_dps", gyro_bias, (3,)),
        ("gyro_correction_matrix", gyro_correction, (3, 3)),
    ):
        if value.shape != shape or not bool(np.all(np.isfinite(value))):
            raise ValueError(f"{name} must be finite with shape {shape}")

    platform_results: list[TemperaturePlatformResult] = []
    corrected_accel_means: list[Any] = []
    corrected_gyro_means: list[Any] = []
    incomplete_findings: list[str] = []
    quality_failures: list[str] = []
    for platform in sorted(grouped):
        rows = grouped[platform]
        temperatures = np.asarray([sample.temperature_c for sample in rows], dtype=float)
        times = np.asarray([sample.timestamp_s for sample in rows], dtype=float)
        deltas = np.diff(times)
        monotonic = bool(np.all(deltas > 0.0))
        duration = float(times[-1] - times[0]) if len(times) >= 2 and monotonic else 0.0
        accel_values = np.asarray([sample.accel_g for sample in rows], dtype=float)
        gyro_values = np.asarray([sample.gyro_dps for sample in rows], dtype=float)
        accel_mean = np.mean(accel_values, axis=0)
        gyro_mean = np.mean(gyro_values, axis=0)
        accel_std = np.std(accel_values, axis=0)
        gyro_std = np.std(gyro_values, axis=0)
        corrected_accel = accel_correction @ (accel_mean - accel_bias)
        corrected_gyro = gyro_correction @ (gyro_mean - gyro_bias)
        within_temperature_span = float(np.max(temperatures) - np.min(temperatures))
        platform_results.append(
            TemperaturePlatformResult(
                platform=platform,
                sample_count=len(rows),
                mean_temperature_c=float(np.mean(temperatures)),
                temperature_span_c=within_temperature_span,
                accel_mean_g=_as_vec3(accel_mean),
                gyro_mean_dps=_as_vec3(gyro_mean),
                accel_std_g=_as_vec3(accel_std),
                gyro_std_dps=_as_vec3(gyro_std),
                corrected_accel_mean_g=_as_vec3(corrected_accel),
                corrected_gyro_mean_dps=_as_vec3(corrected_gyro),
            )
        )
        corrected_accel_means.append(corrected_accel)
        corrected_gyro_means.append(corrected_gyro)
        if len(rows) < thresholds.temperature_min_samples_per_platform:
            incomplete_findings.append(
                f"{platform} 样本 {len(rows)}，需要至少 "
                f"{thresholds.temperature_min_samples_per_platform}。"
            )
        if duration < thresholds.temperature_min_duration_s:
            incomplete_findings.append(
                f"{platform} 持续 {duration:.3f} s，需要至少 "
                f"{thresholds.temperature_min_duration_s:.3f} s。"
            )
        if not monotonic:
            quality_failures.append(f"{platform} 时间戳不是严格递增。")
        if within_temperature_span > thresholds.temperature_platform_span_max_c:
            quality_failures.append(
                f"{platform} 平台内温度跨度 {within_temperature_span:.3f} °C，超过 "
                f"{thresholds.temperature_platform_span_max_c:.3f} °C。"
            )
        if float(np.max(accel_std)) > thresholds.temperature_accel_std_max_g:
            quality_failures.append(f"{platform} 加速度静止噪声超限。")
        if float(np.max(gyro_std)) > thresholds.temperature_gyro_std_max_dps:
            quality_failures.append(f"{platform} 陀螺静止噪声超限。")
        level_error = float(
            np.linalg.norm(corrected_accel - np.asarray((0.0, 0.0, 1.0)))
        )
        if level_error > thresholds.temperature_level_error_max_g:
            quality_failures.append(
                f"{platform} 校正后水平比力误差 {level_error:.6f} g 超限。"
            )
        gyro_mean_norm = float(np.linalg.norm(corrected_gyro))
        if gyro_mean_norm > thresholds.temperature_gyro_mean_norm_max_dps:
            quality_failures.append(
                f"{platform} 校正后静止陀螺均值 {gyro_mean_norm:.6f} dps 超限。"
            )

    temperatures = np.asarray(
        [platform.mean_temperature_c for platform in platform_results], dtype=float
    )
    span = float(np.max(temperatures) - np.min(temperatures))
    reference = float(np.mean(temperatures))
    findings: list[str] = []
    capture_complete = True
    if len(platform_results) < thresholds.temperature_min_platforms:
        status = MetrologyStatus.INCOMPLETE
        capture_complete = False
        findings.append(
            f"温度平台 {len(platform_results)}，需要至少 {thresholds.temperature_min_platforms}。"
        )
    elif span < thresholds.temperature_min_span_c:
        status = MetrologyStatus.INCOMPLETE
        capture_complete = False
        findings.append(
            f"温度跨度 {span:.2f} °C，需要至少 {thresholds.temperature_min_span_c:.1f} °C。"
        )
    elif incomplete_findings:
        status = MetrologyStatus.INCOMPLETE
        capture_complete = False
    else:
        status = MetrologyStatus.PASS
    findings.extend(incomplete_findings)

    accel_values = np.asarray(corrected_accel_means)
    gyro_values = np.asarray(corrected_gyro_means)
    accel_fit = _linear_fit_metrics(temperatures, accel_values)
    gyro_fit = _linear_fit_metrics(temperatures, gyro_values)
    if accel_fit is None or gyro_fit is None:
        accel_slope_value = gyro_slope_value = None
        accel_rms_value = gyro_rms_value = None
        accel_r2_value = gyro_r2_value = None
    else:
        accel_slope, accel_rms, accel_r2 = accel_fit
        gyro_slope, gyro_rms, gyro_r2 = gyro_fit
        accel_slope_value = _as_vec3(accel_slope)
        gyro_slope_value = _as_vec3(gyro_slope)
        accel_rms_value = _as_vec3(accel_rms)
        gyro_rms_value = _as_vec3(gyro_rms)
        accel_r2_value = _as_vec3(accel_r2)
        gyro_r2_value = _as_vec3(gyro_r2)
        if capture_complete:
            if float(np.max(np.abs(accel_slope))) > thresholds.temperature_accel_slope_abs_max_g_per_c:
                quality_failures.append("加速度温漂斜率超过物理上限。")
            if float(np.max(np.abs(gyro_slope))) > thresholds.temperature_gyro_slope_abs_max_dps_per_c:
                quality_failures.append("陀螺温漂斜率超过物理上限。")
            if float(np.max(accel_rms)) > thresholds.temperature_accel_fit_rms_max_g:
                quality_failures.append("加速度温漂线性拟合残差超限。")
            if float(np.max(gyro_rms)) > thresholds.temperature_gyro_fit_rms_max_dps:
                quality_failures.append("陀螺温漂线性拟合残差超限。")
            accel_meaningful = np.ptp(accel_values, axis=0) > (
                2.0 * thresholds.temperature_accel_fit_rms_max_g
            )
            gyro_meaningful = np.ptp(gyro_values, axis=0) > (
                2.0 * thresholds.temperature_gyro_fit_rms_max_dps
            )
            if bool(np.any(accel_meaningful & (accel_r2 < thresholds.temperature_fit_r2_min))):
                quality_failures.append("加速度温漂线性拟合 R² 不足。")
            if bool(np.any(gyro_meaningful & (gyro_r2 < thresholds.temperature_fit_r2_min))):
                quality_failures.append("陀螺温漂线性拟合 R² 不足。")
    if quality_failures:
        status = MetrologyStatus.FAIL
        findings.extend(quality_failures)
    status = _cap_v0_pass(status, relevant, findings)
    return TemperatureDriftResult(
        status=status,
        sample_count=len(relevant),
        platforms=tuple(platform_results),
        temperature_span_c=span,
        reference_temperature_c=reference,
        accel_slope_g_per_c=accel_slope_value,
        gyro_slope_dps_per_c=gyro_slope_value,
        accel_fit_rms_g=accel_rms_value,
        gyro_fit_rms_dps=gyro_rms_value,
        accel_fit_r2=accel_r2_value,
        gyro_fit_r2=gyro_r2_value,
        findings=tuple(findings),
    )


def _overall_status(statuses: Sequence[MetrologyStatus]) -> MetrologyStatus:
    if any(status is MetrologyStatus.FAIL for status in statuses):
        return MetrologyStatus.FAIL
    if any(status in (MetrologyStatus.INCOMPLETE, MetrologyStatus.NOT_RUN) for status in statuses):
        return MetrologyStatus.INCOMPLETE
    if any(status is MetrologyStatus.WARN for status in statuses):
        return MetrologyStatus.WARN
    return MetrologyStatus.PASS


def build_metrology_candidate(
    samples: Sequence[MetrologySample],
    *,
    data_source: str = "caller_supplied_v1_samples",
    created_at: str | None = None,
    thresholds: MetrologyThresholds = DEFAULT_THRESHOLDS,
) -> MetrologyCandidate:
    """Analyze V1 evidence and return a non-applying immutable candidate."""

    sample_list = _sample_tuple(samples)
    (
        frame_id,
        contract,
        orientation,
        generation,
        firmware_hash,
        session_id,
        capture_source,
    ) = _uniform_context(sample_list)
    accelerometer = fit_accelerometer(sample_list, thresholds)
    gyro_static = analyze_stationary_gyro(sample_list, thresholds)
    gyro_rotation = analyze_gyro_rotations(
        sample_list,
        gyro_bias_dps=gyro_static.bias_dps,
        thresholds=thresholds,
    )
    temperature = analyze_temperature_drift(
        sample_list,
        accel_bias_g=accelerometer.bias_g,
        accel_correction_matrix=accelerometer.correction_matrix,
        gyro_bias_dps=gyro_static.bias_dps,
        gyro_correction_matrix=gyro_rotation.correction_matrix,
        thresholds=thresholds,
    )
    status = _overall_status(
        (
            accelerometer.status,
            gyro_static.status,
            gyro_rotation.status,
            temperature.status,
        )
    )
    findings = (
        "该对象仅为 V1 evidence candidate，不会自动修改 RAM 或 Flash。",
        "V1 PASS/WARN 也不代表 V2 完成或允许飞行。",
    )
    timestamp = created_at or datetime.now().astimezone().isoformat()
    return MetrologyCandidate(
        created_at=timestamp,
        data_source=_nonempty(data_source, name="data_source"),
        status=status,
        frame_id=frame_id,
        frame_contract_version=contract,
        orientation_code=orientation,
        base_calibration_generation=generation,
        candidate_calibration_generation=generation + 1,
        firmware_hash=firmware_hash,
        session_id=session_id,
        capture_source=capture_source,
        sample_count=len(sample_list),
        source_provenance=tuple(sorted({sample.provenance for sample in sample_list})),
        capture_methods=tuple(
            sorted({sample.capture_method.value for sample in sample_list})
        ),
        accelerometer=accelerometer,
        gyro_static=gyro_static,
        gyro_rotation=gyro_rotation,
        temperature=temperature,
        thresholds=thresholds,
        findings=findings,
    )


def metrology_sample_from_v0(
    sample: Any,
    *,
    stage: MetrologyStage | str,
    temperature_c: float,
    frame_id: str,
    frame_contract_version: int,
    orientation_code: int,
    calibration_generation: int,
    firmware_hash: str,
    session_id: str,
    capture_source: str,
    provenance: str,
    temperature_platform: str | None = None,
) -> MetrologySample:
    """Explicitly adapt a V0 ``ImuSample`` into V1 evidence.

    There are intentionally no metadata defaults: callers must supply the
    target state and measured temperature.  Converted rows are tagged
    ``V0_IMPORTED`` and therefore can estimate a candidate but cannot auto-PASS
    a V1 section.
    """

    required = (
        "timestamp_s",
        "accel_x_g",
        "accel_y_g",
        "accel_z_g",
        "gyro_x_dps",
        "gyro_y_dps",
        "gyro_z_dps",
    )
    missing = [name for name in required if not hasattr(sample, name)]
    if missing:
        raise TypeError("V0 sample is missing fields: " + ", ".join(missing))
    return MetrologySample(
        timestamp_s=sample.timestamp_s,
        accel_x_g=sample.accel_x_g,
        accel_y_g=sample.accel_y_g,
        accel_z_g=sample.accel_z_g,
        gyro_x_dps=sample.gyro_x_dps,
        gyro_y_dps=sample.gyro_y_dps,
        gyro_z_dps=sample.gyro_z_dps,
        temperature_c=temperature_c,
        stage=stage,
        frame_id=frame_id,
        frame_contract_version=frame_contract_version,
        orientation_code=orientation_code,
        calibration_generation=calibration_generation,
        firmware_hash=firmware_hash,
        session_id=session_id,
        capture_source=capture_source,
        provenance=provenance,
        capture_method=CaptureMethod.V0_IMPORTED,
        temperature_platform=temperature_platform,
    )


def _primitive(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value) and not isinstance(value, type):
        return {item.name: _primitive(getattr(value, item.name)) for item in fields(value)}
    if isinstance(value, Mapping):
        return {str(key): _primitive(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_primitive(item) for item in value]
    return value


def candidate_to_dict(candidate: MetrologyCandidate) -> dict[str, Any]:
    if not isinstance(candidate, MetrologyCandidate):
        raise TypeError("candidate must be a MetrologyCandidate")
    result = _primitive(candidate)
    assert isinstance(result, dict)
    return result


def candidate_to_json(candidate: MetrologyCandidate, *, indent: int | None = 2) -> str:
    return json.dumps(
        candidate_to_dict(candidate),
        ensure_ascii=False,
        indent=indent,
        allow_nan=False,
    ) + "\n"


def _expect_fields(value: Any, cls: type[Any], *, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{path} must be an object")
    expected = {item.name for item in fields(cls)}
    actual = set(value)
    if actual != expected:
        raise ValueError(
            f"{path} fields mismatch; unknown={sorted(actual - expected)}, "
            f"missing={sorted(expected - actual)}"
        )
    return value


def _optional_number(value: Any, *, path: str) -> float | None:
    return None if value is None else _finite(value, name=path)


def _vec_from_json(value: Any, *, path: str) -> Vec3 | None:
    if value is None:
        return None
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence) or len(value) != 3:
        raise ValueError(f"{path} must be null or a three-number array")
    return tuple(_finite(item, name=f"{path}[{index}]") for index, item in enumerate(value))  # type: ignore[return-value]


def _mat_from_json(value: Any, *, path: str) -> Mat3 | None:
    if value is None:
        return None
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence) or len(value) != 3:
        raise ValueError(f"{path} must be null or a 3x3 number array")
    rows = tuple(_vec_from_json(row, path=f"{path}[{index}]") for index, row in enumerate(value))
    if any(row is None for row in rows):
        raise ValueError(f"{path} rows cannot be null")
    return rows  # type: ignore[return-value]


def _strings(value: Any, *, path: str) -> tuple[str, ...]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        raise TypeError(f"{path} must be an array of strings")
    return tuple(_nonempty(item, name=f"{path} item") for item in value)


def _status(value: Any, *, path: str) -> MetrologyStatus:
    return _enum_value(MetrologyStatus, value, name=path)


def _thresholds_from_dict(value: Any) -> MetrologyThresholds:
    row = _expect_fields(value, MetrologyThresholds, path="thresholds")
    integer_names = {
        "accel_min_samples_per_face",
        "gyro_static_min_samples",
        "gyro_rotation_min_samples_per_axis",
        "temperature_min_platforms",
        "temperature_min_samples_per_platform",
    }
    parsed: dict[str, Any] = {}
    for item in fields(MetrologyThresholds):
        if item.name in integer_names:
            parsed[item.name] = _strict_int(
                row[item.name], name=f"thresholds.{item.name}", minimum=1
            )
        else:
            parsed[item.name] = _finite(
                row[item.name], name=f"thresholds.{item.name}"
            )
    return MetrologyThresholds(**parsed)


def _stage_count_from_dict(value: Any, *, path: str) -> StageCount:
    row = _expect_fields(value, StageCount, path=path)
    return StageCount(
        stage=_nonempty(row["stage"], name=f"{path}.stage"),
        sample_count=_strict_int(row["sample_count"], name=f"{path}.sample_count"),
    )


def _accel_from_dict(value: Any) -> AccelerometerCalibrationResult:
    row = _expect_fields(value, AccelerometerCalibrationResult, path="accelerometer")
    raw_counts = row["stage_counts"]
    if not isinstance(raw_counts, Sequence) or isinstance(raw_counts, (str, bytes, bytearray)):
        raise TypeError("accelerometer.stage_counts must be an array")
    return AccelerometerCalibrationResult(
        status=_status(row["status"], path="accelerometer.status"),
        sample_count=_strict_int(row["sample_count"], name="accelerometer.sample_count"),
        stage_counts=tuple(
            _stage_count_from_dict(item, path=f"accelerometer.stage_counts[{index}]")
            for index, item in enumerate(raw_counts)
        ),
        bias_g=_vec_from_json(row["bias_g"], path="accelerometer.bias_g"),
        bias_norm_g=_optional_number(row["bias_norm_g"], path="accelerometer.bias_norm_g"),
        correction_matrix=_mat_from_json(row["correction_matrix"], path="accelerometer.correction_matrix"),
        determinant=_optional_number(row["determinant"], path="accelerometer.determinant"),
        singular_values=_vec_from_json(row["singular_values"], path="accelerometer.singular_values"),
        condition_number=_optional_number(row["condition_number"], path="accelerometer.condition_number"),
        corrected_rms_g=_optional_number(row["corrected_rms_g"], path="accelerometer.corrected_rms_g"),
        corrected_max_error_g=_optional_number(row["corrected_max_error_g"], path="accelerometer.corrected_max_error_g"),
        findings=_strings(row["findings"], path="accelerometer.findings"),
    )


def _gyro_static_from_dict(value: Any) -> GyroStaticResult:
    row = _expect_fields(value, GyroStaticResult, path="gyro_static")
    return GyroStaticResult(
        status=_status(row["status"], path="gyro_static.status"),
        sample_count=_strict_int(row["sample_count"], name="gyro_static.sample_count"),
        bias_dps=_vec_from_json(row["bias_dps"], path="gyro_static.bias_dps"),
        std_dps=_vec_from_json(row["std_dps"], path="gyro_static.std_dps"),
        noise_vector_rms_dps=_optional_number(row["noise_vector_rms_dps"], path="gyro_static.noise_vector_rms_dps"),
        findings=_strings(row["findings"], path="gyro_static.findings"),
    )


def _gyro_rotation_stage_from_dict(value: Any, *, path: str) -> GyroRotationStageResult:
    row = _expect_fields(value, GyroRotationStageResult, path=path)
    monotonic = row["timestamps_monotonic"]
    if type(monotonic) is not bool:
        raise TypeError(f"{path}.timestamps_monotonic must be boolean")
    return GyroRotationStageResult(
        stage=_nonempty(row["stage"], name=f"{path}.stage"),
        sample_count=_strict_int(row["sample_count"], name=f"{path}.sample_count"),
        duration_s=_finite(row["duration_s"], name=f"{path}.duration_s"),
        integrated_angle_deg=_vec_from_json(row["integrated_angle_deg"], path=f"{path}.integrated_angle_deg"),
        main_axis_scale=_optional_number(row["main_axis_scale"], path=f"{path}.main_axis_scale"),
        cross_axis_fraction=_optional_number(row["cross_axis_fraction"], path=f"{path}.cross_axis_fraction"),
        timestamps_monotonic=monotonic,
        capture_methods=_strings(row["capture_methods"], path=f"{path}.capture_methods"),
    )


def _gyro_rotation_from_dict(value: Any) -> GyroRotationResult:
    row = _expect_fields(value, GyroRotationResult, path="gyro_rotation")
    raw_stages = row["stages"]
    if not isinstance(raw_stages, Sequence) or isinstance(raw_stages, (str, bytes, bytearray)):
        raise TypeError("gyro_rotation.stages must be an array")
    return GyroRotationResult(
        status=_status(row["status"], path="gyro_rotation.status"),
        sample_count=_strict_int(row["sample_count"], name="gyro_rotation.sample_count"),
        stages=tuple(
            _gyro_rotation_stage_from_dict(item, path=f"gyro_rotation.stages[{index}]")
            for index, item in enumerate(raw_stages)
        ),
        correction_matrix=_mat_from_json(row["correction_matrix"], path="gyro_rotation.correction_matrix"),
        determinant=_optional_number(row["determinant"], path="gyro_rotation.determinant"),
        singular_values=_vec_from_json(row["singular_values"], path="gyro_rotation.singular_values"),
        condition_number=_optional_number(row["condition_number"], path="gyro_rotation.condition_number"),
        corrected_integrals_deg=_mat_from_json(row["corrected_integrals_deg"], path="gyro_rotation.corrected_integrals_deg"),
        findings=_strings(row["findings"], path="gyro_rotation.findings"),
    )


def _temperature_platform_from_dict(value: Any, *, path: str) -> TemperaturePlatformResult:
    row = _expect_fields(value, TemperaturePlatformResult, path=path)
    accel = _vec_from_json(row["accel_mean_g"], path=f"{path}.accel_mean_g")
    gyro = _vec_from_json(row["gyro_mean_dps"], path=f"{path}.gyro_mean_dps")
    accel_std = _vec_from_json(row["accel_std_g"], path=f"{path}.accel_std_g")
    gyro_std = _vec_from_json(row["gyro_std_dps"], path=f"{path}.gyro_std_dps")
    corrected_accel = _vec_from_json(
        row["corrected_accel_mean_g"], path=f"{path}.corrected_accel_mean_g"
    )
    corrected_gyro = _vec_from_json(
        row["corrected_gyro_mean_dps"], path=f"{path}.corrected_gyro_mean_dps"
    )
    if any(
        item is None
        for item in (accel, gyro, accel_std, gyro_std, corrected_accel, corrected_gyro)
    ):
        raise ValueError(f"{path} vectors cannot be null")
    return TemperaturePlatformResult(
        platform=_nonempty(row["platform"], name=f"{path}.platform"),
        sample_count=_strict_int(row["sample_count"], name=f"{path}.sample_count"),
        mean_temperature_c=_finite(row["mean_temperature_c"], name=f"{path}.mean_temperature_c"),
        temperature_span_c=_finite(
            row["temperature_span_c"], name=f"{path}.temperature_span_c"
        ),
        accel_mean_g=accel,
        gyro_mean_dps=gyro,
        accel_std_g=accel_std,
        gyro_std_dps=gyro_std,
        corrected_accel_mean_g=corrected_accel,
        corrected_gyro_mean_dps=corrected_gyro,
    )


def _temperature_from_dict(value: Any) -> TemperatureDriftResult:
    row = _expect_fields(value, TemperatureDriftResult, path="temperature")
    raw_platforms = row["platforms"]
    if not isinstance(raw_platforms, Sequence) or isinstance(raw_platforms, (str, bytes, bytearray)):
        raise TypeError("temperature.platforms must be an array")
    return TemperatureDriftResult(
        status=_status(row["status"], path="temperature.status"),
        sample_count=_strict_int(row["sample_count"], name="temperature.sample_count"),
        platforms=tuple(
            _temperature_platform_from_dict(item, path=f"temperature.platforms[{index}]")
            for index, item in enumerate(raw_platforms)
        ),
        temperature_span_c=_optional_number(row["temperature_span_c"], path="temperature.temperature_span_c"),
        reference_temperature_c=_optional_number(row["reference_temperature_c"], path="temperature.reference_temperature_c"),
        accel_slope_g_per_c=_vec_from_json(row["accel_slope_g_per_c"], path="temperature.accel_slope_g_per_c"),
        gyro_slope_dps_per_c=_vec_from_json(row["gyro_slope_dps_per_c"], path="temperature.gyro_slope_dps_per_c"),
        accel_fit_rms_g=_vec_from_json(row["accel_fit_rms_g"], path="temperature.accel_fit_rms_g"),
        gyro_fit_rms_dps=_vec_from_json(row["gyro_fit_rms_dps"], path="temperature.gyro_fit_rms_dps"),
        accel_fit_r2=_vec_from_json(row["accel_fit_r2"], path="temperature.accel_fit_r2"),
        gyro_fit_r2=_vec_from_json(row["gyro_fit_r2"], path="temperature.gyro_fit_r2"),
        findings=_strings(row["findings"], path="temperature.findings"),
    )


_SAFETY_VALUES: Mapping[str, Any] = MappingProxyType(
    {
        "format": METROLOGY_FORMAT,
        "schema": METROLOGY_SCHEMA,
        "evidence_only": True,
        "applied": False,
        "parameter_changes_applied": False,
        "parameters_written": False,
        "flash_writes": 0,
        "flight_release": False,
    }
)


def _determinant3(matrix: Mat3) -> float:
    a, b, c = matrix
    return (
        a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
    )


def _vector_norm(value: Vec3) -> float:
    return math.sqrt(sum(component * component for component in value))


def _matrix_identity_deviation(matrix: Mat3) -> float:
    return math.sqrt(
        sum(
            (matrix[row][column] - (1.0 if row == column else 0.0)) ** 2
            for row in range(3)
            for column in range(3)
        )
    )


def _require_close(actual: float, expected: float, *, name: str) -> None:
    if not math.isclose(actual, expected, rel_tol=1.0e-9, abs_tol=1.0e-12):
        raise ValueError(f"candidate semantic mismatch for {name}")


def _validate_candidate_semantics(candidate: MetrologyCandidate) -> None:
    """Reject internally contradictory evidence, even when JSON is well formed.

    This is deliberately independent from the raw-evidence replay required by
    :func:`validate_candidate_for_application`; it catches obvious status,
    count, frame and metric forgeries at the deserialisation boundary.
    """

    for name, expected_type in (
        ("accelerometer", AccelerometerCalibrationResult),
        ("gyro_static", GyroStaticResult),
        ("gyro_rotation", GyroRotationResult),
        ("temperature", TemperatureDriftResult),
    ):
        if not isinstance(getattr(candidate, name), expected_type):
            raise TypeError(f"{name} has the wrong result type")
    if not isinstance(candidate.status, MetrologyStatus):
        raise TypeError("candidate status must be MetrologyStatus")
    section_statuses = (
        candidate.accelerometer.status,
        candidate.gyro_static.status,
        candidate.gyro_rotation.status,
        candidate.temperature.status,
    )
    if any(not isinstance(status, MetrologyStatus) for status in section_statuses):
        raise TypeError("candidate section status must be MetrologyStatus")
    if candidate.status is not _overall_status(section_statuses):
        raise ValueError("candidate status does not match section statuses")

    if tuple(sorted(set(candidate.source_provenance))) != candidate.source_provenance:
        raise ValueError("source_provenance must be sorted and unique")
    if tuple(sorted(set(candidate.capture_methods))) != candidate.capture_methods:
        raise ValueError("capture_methods must be sorted and unique")
    if not candidate.source_provenance or not candidate.capture_methods:
        raise ValueError("candidate provenance and capture methods cannot be empty")
    if (
        CaptureMethod.V0_IMPORTED.value in candidate.capture_methods
        and candidate.status is MetrologyStatus.PASS
    ):
        raise ValueError("V0-imported evidence cannot be a V1 PASS candidate")

    accel = candidate.accelerometer
    expected_accel_stages = tuple(stage.value for stage in ACCEL_FACE_STAGES)
    actual_accel_stages = tuple(item.stage for item in accel.stage_counts)
    if actual_accel_stages != expected_accel_stages:
        raise ValueError("accelerometer stage_counts are not the canonical six faces")
    if sum(item.sample_count for item in accel.stage_counts) != accel.sample_count:
        raise ValueError("accelerometer sample_count mismatch")
    if accel.bias_g is not None and accel.bias_norm_g is not None:
        _require_close(
            accel.bias_norm_g, _vector_norm(accel.bias_g), name="accelerometer.bias_norm_g"
        )
    if accel.correction_matrix is not None:
        calculated_det = _determinant3(accel.correction_matrix)
        if accel.determinant is None:
            raise ValueError("accelerometer determinant missing")
        _require_close(accel.determinant, calculated_det, name="accelerometer.determinant")
    if accel.status is MetrologyStatus.PASS:
        required = (
            accel.bias_g,
            accel.bias_norm_g,
            accel.correction_matrix,
            accel.determinant,
            accel.singular_values,
            accel.condition_number,
            accel.corrected_rms_g,
            accel.corrected_max_error_g,
        )
        if any(value is None for value in required):
            raise ValueError("PASS accelerometer result is incomplete")
        assert accel.correction_matrix is not None
        assert accel.determinant is not None
        assert accel.singular_values is not None
        assert accel.bias_norm_g is not None
        assert accel.condition_number is not None
        assert accel.corrected_rms_g is not None
        assert accel.corrected_max_error_g is not None
        limits = candidate.thresholds
        if any(
            item.sample_count < limits.accel_min_samples_per_face
            for item in accel.stage_counts
        ):
            raise ValueError("PASS accelerometer result has too few samples")
        if accel.determinant <= 0.0:
            raise ValueError("PASS accelerometer determinant must be positive")
        if _matrix_identity_deviation(accel.correction_matrix) > limits.accel_identity_deviation_max:
            raise ValueError("PASS accelerometer correction is not near identity")
        if min(accel.correction_matrix[index][index] for index in range(3)) < limits.accel_diagonal_min:
            raise ValueError("PASS accelerometer diagonal violates the V0/V1 boundary")
        if max(
            abs(accel.correction_matrix[row][column])
            for row in range(3)
            for column in range(3)
            if row != column
        ) > limits.accel_off_diagonal_max:
            raise ValueError("PASS accelerometer off-diagonal correction is too large")
        if (
            min(accel.singular_values) < limits.accel_singular_min
            or max(accel.singular_values) > limits.accel_singular_max
            or accel.condition_number > limits.accel_condition_max
            or accel.bias_norm_g > limits.accel_bias_norm_max_g
            or accel.corrected_rms_g > limits.accel_corrected_rms_max_g
            or accel.corrected_max_error_g > limits.accel_corrected_max_error_g
        ):
            raise ValueError("PASS accelerometer metrics violate recorded thresholds")

    gyro_static = candidate.gyro_static
    if gyro_static.status is MetrologyStatus.PASS:
        if gyro_static.std_dps is None or gyro_static.bias_dps is None:
            raise ValueError("PASS gyro_static result is incomplete")
        if gyro_static.sample_count < candidate.thresholds.gyro_static_min_samples:
            raise ValueError("PASS gyro_static result has too few samples")
        if max(gyro_static.std_dps) > candidate.thresholds.gyro_static_std_pass_dps:
            raise ValueError("PASS gyro_static noise violates recorded thresholds")

    rotation = candidate.gyro_rotation
    expected_rotation_stages = tuple(stage.value for stage in GYRO_ROTATION_STAGES)
    if rotation.stages and tuple(item.stage for item in rotation.stages) != expected_rotation_stages:
        raise ValueError("gyro rotation stages are not canonical +X/+Y/+Z")
    if sum(item.sample_count for item in rotation.stages) != rotation.sample_count:
        raise ValueError("gyro rotation sample_count mismatch")
    if rotation.correction_matrix is not None:
        calculated_det = _determinant3(rotation.correction_matrix)
        if rotation.determinant is None:
            raise ValueError("gyro rotation determinant missing")
        _require_close(rotation.determinant, calculated_det, name="gyro_rotation.determinant")
    if rotation.status is MetrologyStatus.PASS:
        if rotation.correction_matrix is None or rotation.determinant is None:
            raise ValueError("PASS gyro rotation result is incomplete")
        if rotation.determinant <= 0.0:
            raise ValueError("PASS gyro rotation determinant must be positive")
        for item in rotation.stages:
            if (
                item.sample_count < candidate.thresholds.gyro_rotation_min_samples_per_axis
                or item.duration_s < candidate.thresholds.gyro_rotation_min_duration_s
                or not item.timestamps_monotonic
                or item.main_axis_scale is None
                or item.main_axis_scale <= 0.0
                or item.cross_axis_fraction is None
                or item.cross_axis_fraction > candidate.thresholds.gyro_cross_axis_pass_fraction
                or item.capture_methods != (CaptureMethod.REFERENCE_FIXTURE.value,)
            ):
                raise ValueError("PASS gyro rotation stage violates recorded thresholds")

    temperature = candidate.temperature
    if sum(item.sample_count for item in temperature.platforms) != temperature.sample_count:
        raise ValueError("temperature sample_count mismatch")
    platform_names = tuple(item.platform for item in temperature.platforms)
    if len(set(platform_names)) != len(platform_names):
        raise ValueError("temperature platform names must be unique")
    if platform_names != tuple(sorted(platform_names)):
        raise ValueError("temperature platform names must be sorted")
    if temperature.status is MetrologyStatus.PASS:
        required = (
            temperature.temperature_span_c,
            temperature.reference_temperature_c,
            temperature.accel_slope_g_per_c,
            temperature.gyro_slope_dps_per_c,
            temperature.accel_fit_rms_g,
            temperature.gyro_fit_rms_dps,
            temperature.accel_fit_r2,
            temperature.gyro_fit_r2,
        )
        if any(value is None for value in required):
            raise ValueError("PASS temperature result is incomplete")
        limits = candidate.thresholds
        assert temperature.temperature_span_c is not None
        assert temperature.reference_temperature_c is not None
        assert temperature.accel_slope_g_per_c is not None
        assert temperature.gyro_slope_dps_per_c is not None
        assert temperature.accel_fit_rms_g is not None
        assert temperature.gyro_fit_rms_dps is not None
        assert temperature.accel_fit_r2 is not None
        assert temperature.gyro_fit_r2 is not None
        if len(temperature.platforms) < limits.temperature_min_platforms:
            raise ValueError("PASS temperature result has too few platforms")
        platform_temperatures = tuple(item.mean_temperature_c for item in temperature.platforms)
        _require_close(
            temperature.temperature_span_c,
            max(platform_temperatures) - min(platform_temperatures),
            name="temperature.temperature_span_c",
        )
        _require_close(
            temperature.reference_temperature_c,
            sum(platform_temperatures) / len(platform_temperatures),
            name="temperature.reference_temperature_c",
        )
        if temperature.temperature_span_c < limits.temperature_min_span_c:
            raise ValueError("PASS temperature span violates recorded thresholds")
        for item in temperature.platforms:
            if item.sample_count < limits.temperature_min_samples_per_platform:
                raise ValueError("PASS temperature platform has too few samples")
            if item.temperature_span_c < 0.0:
                raise ValueError("PASS temperature platform span cannot be negative")
            if item.temperature_span_c > limits.temperature_platform_span_max_c:
                raise ValueError("PASS temperature platform is not temperature-stable")
            if min(item.accel_std_g) < 0.0 or min(item.gyro_std_dps) < 0.0:
                raise ValueError("PASS temperature platform standard deviation cannot be negative")
            if max(item.accel_std_g) > limits.temperature_accel_std_max_g:
                raise ValueError("PASS temperature platform is not accelerometer-static")
            if max(item.gyro_std_dps) > limits.temperature_gyro_std_max_dps:
                raise ValueError("PASS temperature platform is not gyro-static")
            level_error = _vector_norm(
                (
                    item.corrected_accel_mean_g[0],
                    item.corrected_accel_mean_g[1],
                    item.corrected_accel_mean_g[2] - 1.0,
                )
            )
            if level_error > limits.temperature_level_error_max_g:
                raise ValueError("PASS temperature platform is not horizontal")
            if _vector_norm(item.corrected_gyro_mean_dps) > limits.temperature_gyro_mean_norm_max_dps:
                raise ValueError("PASS temperature platform gyro mean is not physical")
        if (
            max(abs(value) for value in temperature.accel_slope_g_per_c)
            > limits.temperature_accel_slope_abs_max_g_per_c
            or max(abs(value) for value in temperature.gyro_slope_dps_per_c)
            > limits.temperature_gyro_slope_abs_max_dps_per_c
            or max(temperature.accel_fit_rms_g) > limits.temperature_accel_fit_rms_max_g
            or max(temperature.gyro_fit_rms_dps) > limits.temperature_gyro_fit_rms_max_dps
        ):
            raise ValueError("PASS temperature fit violates recorded thresholds")
        if (
            min(temperature.accel_fit_rms_g) < 0.0
            or min(temperature.gyro_fit_rms_dps) < 0.0
            or any(not 0.0 <= value <= 1.0 for value in temperature.accel_fit_r2)
            or any(not 0.0 <= value <= 1.0 for value in temperature.gyro_fit_r2)
        ):
            raise ValueError("PASS temperature fit metrics are not physical")
        for axis in range(3):
            accel_values = tuple(
                item.corrected_accel_mean_g[axis] for item in temperature.platforms
            )
            gyro_values = tuple(
                item.corrected_gyro_mean_dps[axis] for item in temperature.platforms
            )
            if (
                max(accel_values) - min(accel_values)
                > 2.0 * limits.temperature_accel_fit_rms_max_g
                and temperature.accel_fit_r2[axis] < limits.temperature_fit_r2_min
            ):
                raise ValueError("PASS accelerometer temperature fit R2 is too low")
            if (
                max(gyro_values) - min(gyro_values)
                > 2.0 * limits.temperature_gyro_fit_rms_max_dps
                and temperature.gyro_fit_r2[axis] < limits.temperature_fit_r2_min
            ):
                raise ValueError("PASS gyro temperature fit R2 is too low")

    section_sample_count = (
        accel.sample_count
        + gyro_static.sample_count
        + rotation.sample_count
        + temperature.sample_count
    )
    if candidate.sample_count != section_sample_count:
        raise ValueError("candidate sample_count does not match section sample counts")


def candidate_from_dict(payload: Mapping[str, Any]) -> MetrologyCandidate:
    row = _expect_fields(payload, MetrologyCandidate, path="candidate")
    for name, expected in _SAFETY_VALUES.items():
        if type(row[name]) is not type(expected) or row[name] != expected:
            raise ValueError(f"unsafe or unsupported candidate field {name}: {row[name]!r}")
    return MetrologyCandidate(
        created_at=_nonempty(row["created_at"], name="created_at"),
        data_source=_nonempty(row["data_source"], name="data_source"),
        status=_status(row["status"], path="status"),
        frame_id=_nonempty(row["frame_id"], name="frame_id"),
        frame_contract_version=_strict_int(row["frame_contract_version"], name="frame_contract_version", minimum=1),
        orientation_code=_strict_int(row["orientation_code"], name="orientation_code"),
        base_calibration_generation=_strict_int(row["base_calibration_generation"], name="base_calibration_generation"),
        candidate_calibration_generation=_strict_int(row["candidate_calibration_generation"], name="candidate_calibration_generation"),
        firmware_hash=_binding(row["firmware_hash"], name="firmware_hash"),
        session_id=_binding(row["session_id"], name="session_id"),
        capture_source=_binding(row["capture_source"], name="capture_source"),
        sample_count=_strict_int(row["sample_count"], name="sample_count"),
        source_provenance=_strings(row["source_provenance"], path="source_provenance"),
        capture_methods=_strings(row["capture_methods"], path="capture_methods"),
        accelerometer=_accel_from_dict(row["accelerometer"]),
        gyro_static=_gyro_static_from_dict(row["gyro_static"]),
        gyro_rotation=_gyro_rotation_from_dict(row["gyro_rotation"]),
        temperature=_temperature_from_dict(row["temperature"]),
        thresholds=_thresholds_from_dict(row["thresholds"]),
        findings=_strings(row["findings"], path="findings"),
    )


def _strict_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number is forbidden: {value}")


def candidate_from_json(text: str) -> MetrologyCandidate:
    if not isinstance(text, str):
        raise TypeError("candidate JSON must be text")
    payload = json.loads(
        text,
        object_pairs_hook=_strict_json_object,
        parse_constant=_reject_json_constant,
    )
    return candidate_from_dict(payload)


def validate_candidate_for_application(
    candidate: MetrologyCandidate,
    samples: Sequence[MetrologySample],
) -> MetrologyCandidate:
    """Replay raw evidence and require exact candidate equivalence before use.

    The returned object is still evidence-only.  This function deliberately
    does not expose transport, RAM or Flash operations; it is the numerical
    gate a separately guarded apply workflow must call immediately before
    accepting any continuous calibration parameters.
    """

    if not isinstance(candidate, MetrologyCandidate):
        raise TypeError("candidate must be a MetrologyCandidate")
    if candidate.thresholds != DEFAULT_THRESHOLDS:
        raise ValueError("candidate thresholds do not match application policy")
    sample_list = _sample_tuple(samples)
    rebuilt = build_metrology_candidate(
        sample_list,
        data_source=candidate.data_source,
        created_at=candidate.created_at,
        thresholds=DEFAULT_THRESHOLDS,
    )
    if candidate_to_dict(candidate) != candidate_to_dict(rebuilt):
        raise ValueError("candidate does not exactly match replayed raw evidence")
    if rebuilt.status is not MetrologyStatus.PASS:
        raise ValueError("only a fully PASS V1 candidate is application-eligible")
    if any(
        sample.capture_method is CaptureMethod.V0_IMPORTED for sample in sample_list
    ):
        raise ValueError("V0-imported evidence is never application-eligible for V1")
    return candidate


def validate_room_temperature_candidate_for_application(
    candidate: MetrologyCandidate,
    samples: Sequence[MetrologySample],
) -> MetrologyCandidate:
    """Replay evidence for the useful V1 room-temperature subset.

    This tier applies only the six-face accelerometer fit and stationary gyro
    residual bias. It does not authorize the gyro correction matrix or a
    temperature slope, so manual 360-degree motion is never promoted to
    metrology-grade evidence.
    """

    if not isinstance(candidate, MetrologyCandidate):
        raise TypeError("candidate must be a MetrologyCandidate")
    if candidate.thresholds != DEFAULT_THRESHOLDS:
        raise ValueError("candidate thresholds do not match application policy")
    sample_list = _sample_tuple(samples)
    rebuilt = build_metrology_candidate(
        sample_list,
        data_source=candidate.data_source,
        created_at=candidate.created_at,
        thresholds=DEFAULT_THRESHOLDS,
    )
    if candidate_to_dict(candidate) != candidate_to_dict(rebuilt):
        raise ValueError("candidate does not exactly match replayed raw evidence")
    # V0 导入的样本会把 PASS 压成 WARN，必须先单独拒掉，否则下面放行 WARN 时会漏过去。
    if any(
        sample.capture_method is CaptureMethod.V0_IMPORTED for sample in sample_list
    ):
        raise ValueError("V0-imported evidence is never application-eligible for V1")
    # WARN 放行：加速度计的 WARN 只表示六面摆放一致性在 0.025~0.075 g（折合 1.4~4.3°），
    # 陀螺的 WARN 只表示静止段噪声偏大（4500 样本下零偏标准误 <0.03 dps）。两者都远好于
    # 不标定，PX4 在这两处根本没有对应的拒收条件。真正的问题（矩阵非正交、刻度错、单点
    # 野值、样本不足）在上面已经判成 FAIL/INCOMPLETE。
    applicable = {MetrologyStatus.PASS, MetrologyStatus.WARN}
    for label, section in (("六面加速度计", rebuilt.accelerometer),
                           ("陀螺静止", rebuilt.gyro_static)):
        if section.status not in applicable:
            detail = "；".join(section.findings) or "无附加说明"
            raise ValueError(
                f"{label}证据为 {section.status.value}，不可应用到飞机。原因：{detail}"
            )
    return candidate


__all__ = [
    "ACCEL_FACE_STAGES",
    "CANONICAL_FRAME_ID",
    "CaptureMethod",
    "DEFAULT_THRESHOLDS",
    "GYRO_ROTATION_STAGES",
    "METROLOGY_FORMAT",
    "METROLOGY_SCHEMA",
    "NUMPY_AVAILABLE",
    "SUPPORTED_FRAME_CONTRACT_VERSION",
    "STAGE_DEFINITIONS",
    "AccelerometerCalibrationResult",
    "GyroRotationResult",
    "GyroRotationStageResult",
    "GyroStaticResult",
    "Mat3",
    "MetrologyCandidate",
    "MetrologySample",
    "MetrologyStage",
    "MetrologyStatus",
    "MetrologyThresholds",
    "NumpyRequiredError",
    "StageCount",
    "StageDefinition",
    "TemperatureDriftResult",
    "TemperaturePlatformResult",
    "Vec3",
    "analyze_gyro_rotations",
    "analyze_stationary_gyro",
    "analyze_temperature_drift",
    "build_metrology_candidate",
    "candidate_from_dict",
    "candidate_from_json",
    "candidate_to_dict",
    "candidate_to_json",
    "fit_accelerometer",
    "metrology_sample_from_v0",
    "validate_candidate_for_application",
    "validate_room_temperature_candidate_for_application",
]
