#!/usr/bin/env python3
"""Read-only V0 flight-sensor validation algorithms.

This module deliberately has no transport, parameter, Flash, or firmware-update
API.  It consumes already captured samples and produces *validation evidence*.
The running firmware is still legacy/unverified until the compile-time FLU
migration contract says otherwise; a successful report must not be interpreted
as an automatic calibration or as runtime migration completion.
"""

from __future__ import annotations

import csv
import io
import json
import math
import statistics
from dataclasses import dataclass, field, fields, is_dataclass, replace
from datetime import datetime
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Sequence


Vec3 = tuple[float, float, float]
SignedPermutation = tuple[tuple[int, int, int], tuple[int, int, int], tuple[int, int, int]]


class ValidationStatus(str, Enum):
    """Severity used by every V0 validation result."""

    PASS = "PASS"
    WARN = "WARN"
    SKIPPED = "SKIPPED"
    NOT_RUN = "NOT_RUN"
    UNSUPPORTED = "UNSUPPORTED"
    FAIL = "FAIL"


class ValidationStage(str, Enum):
    """Stable identifiers for the nine guided V0 capture stages."""

    LEVEL = "level"
    NOSE_UP = "nose_up"
    NOSE_DOWN = "nose_down"
    LEFT_SIDE_UP = "left_side_up"
    RIGHT_SIDE_UP = "right_side_up"
    INVERTED = "inverted"
    POSITIVE_ROLL = "positive_roll"
    POSITIVE_PITCH = "positive_pitch"
    POSITIVE_YAW = "positive_yaw"


@dataclass(frozen=True)
class StageDefinition:
    stage: ValidationStage
    label_zh: str
    prompt_zh: str
    kind: str
    expected_specific_force_g: Vec3 | None = None
    expected_gyro_axis: str | None = None
    required: bool = True


_STAGE_DEFINITIONS = {
    ValidationStage.LEVEL: StageDefinition(
        ValidationStage.LEVEL,
        "静止水平",
        "拆除桨叶，将飞机水平放稳并保持静止。",
        "static",
        (0.0, 0.0, 1.0),
    ),
    ValidationStage.NOSE_UP: StageDefinition(
        ValidationStage.NOSE_UP,
        "机头朝上",
        "使机头竖直朝上，放稳并保持静止。",
        "static",
        (1.0, 0.0, 0.0),
    ),
    ValidationStage.NOSE_DOWN: StageDefinition(
        ValidationStage.NOSE_DOWN,
        "机头朝下",
        "使机头竖直朝下，放稳并保持静止。",
        "static",
        (-1.0, 0.0, 0.0),
        required=False,
    ),
    ValidationStage.LEFT_SIDE_UP: StageDefinition(
        ValidationStage.LEFT_SIDE_UP,
        "左侧朝上",
        "使飞机左侧竖直朝上，放稳并保持静止。",
        "static",
        (0.0, 1.0, 0.0),
    ),
    ValidationStage.RIGHT_SIDE_UP: StageDefinition(
        ValidationStage.RIGHT_SIDE_UP,
        "右侧朝上",
        "使飞机右侧竖直朝上，放稳并保持静止。",
        "static",
        (0.0, -1.0, 0.0),
        required=False,
    ),
    ValidationStage.INVERTED: StageDefinition(
        ValidationStage.INVERTED,
        "倒置",
        "使飞机完全倒置，放稳并保持静止。",
        "static",
        (0.0, 0.0, -1.0),
        required=False,
    ),
    ValidationStage.POSITIVE_ROLL: StageDefinition(
        ValidationStage.POSITIVE_ROLL,
        "+roll 手动旋转",
        "手持飞机缓慢做正 roll：让右翼向下运动。",
        "rotation",
        expected_gyro_axis="x",
    ),
    ValidationStage.POSITIVE_PITCH: StageDefinition(
        ValidationStage.POSITIVE_PITCH,
        "+pitch 手动旋转",
        "手持飞机缓慢做正 pitch：让机头向下运动。",
        "rotation",
        expected_gyro_axis="y",
    ),
    ValidationStage.POSITIVE_YAW: StageDefinition(
        ValidationStage.POSITIVE_YAW,
        "+yaw 手动旋转",
        "手持飞机缓慢做正 yaw：让机头向左转动。",
        "rotation",
        expected_gyro_axis="z",
    ),
}

# Exposed as a read-only mapping so UI code cannot silently redefine a stage.
STAGE_DEFINITIONS: Mapping[ValidationStage, StageDefinition] = MappingProxyType(
    _STAGE_DEFINITIONS
)
STATIC_STAGES = (
    ValidationStage.LEVEL,
    ValidationStage.NOSE_UP,
    ValidationStage.NOSE_DOWN,
    ValidationStage.LEFT_SIDE_UP,
    ValidationStage.RIGHT_SIDE_UP,
    ValidationStage.INVERTED,
)
ROTATION_STAGES = (
    ValidationStage.POSITIVE_ROLL,
    ValidationStage.POSITIVE_PITCH,
    ValidationStage.POSITIVE_YAW,
)
REQUIRED_STAGES = tuple(
    stage for stage, definition in STAGE_DEFINITIONS.items() if definition.required
)
OPTIONAL_STAGES = tuple(
    stage for stage, definition in STAGE_DEFINITIONS.items() if not definition.required
)


@dataclass(frozen=True)
class ValidationThresholds:
    """All V0 decision thresholds, kept in one auditable definition."""

    static_min_samples: int = 20
    static_recommended_samples: int = 50
    invalid_fraction_warn: float = 0.01
    invalid_fraction_fail: float = 0.10
    accel_norm_error_pass_g: float = 0.08
    accel_norm_error_warn_g: float = 0.15
    accel_std_pass_g: float = 0.03
    accel_std_warn_g: float = 0.08
    gyro_bias_pass_dps: float = 1.0
    gyro_bias_warn_dps: float = 3.0
    gyro_std_pass_dps: float = 0.5
    gyro_std_warn_dps: float = 1.5
    canonical_axis_error_pass_g: float = 0.12
    canonical_axis_error_warn_g: float = 0.25
    canonical_cross_axis_pass_g: float = 0.12
    canonical_cross_axis_warn_g: float = 0.25
    candidate_confidence_pass: float = 0.90
    candidate_confidence_warn: float = 0.75
    rotation_min_samples: int = 5
    rotation_recommended_samples: int = 20
    rotation_min_duration_s: float = 0.20
    rotation_angle_pass_deg: float = 20.0
    rotation_angle_warn_deg: float = 8.0
    rotation_peak_pass_dps: float = 10.0
    rotation_peak_warn_dps: float = 5.0
    rotation_dominance_pass: float = 1.5
    rotation_dominance_warn: float = 1.0
    attitude_direction_min_deg: float = 5.0


DEFAULT_THRESHOLDS = ValidationThresholds()
THRESHOLD_VERSION = 1
INPUT_UNITS: Mapping[str, str] = MappingProxyType(
    {
        "timestamp": "s",
        "accelerometer": "g",
        "gyroscope": "deg/s",
        "attitude_optional": "deg",
    }
)
SESSION_FORMAT = "drone-h743-flight-validation-session"
SESSION_SCHEMA = 1


def _now_iso_timestamp() -> str:
    return datetime.now().astimezone().isoformat()


def _session_stage(value: ValidationStage | str, *, field_name: str) -> ValidationStage:
    if isinstance(value, ValidationStage):
        return value
    if not isinstance(value, str):
        raise TypeError(f"{field_name} stage must be a string or ValidationStage")
    try:
        return ValidationStage(value)
    except ValueError as exc:
        raise ValueError(f"unknown stage in {field_name}: {value!r}") from exc


def _finite_session_number(value: Any, *, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{field_name} must be a finite JSON number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{field_name} must be finite")
    return number


def _normalise_session_sample(sample: ImuSample, *, field_name: str) -> ImuSample:
    if not isinstance(sample, ImuSample):
        raise TypeError(f"{field_name} must contain ImuSample values")
    required_values = {
        "timestamp_s": sample.timestamp_s,
        "accel_x_g": sample.accel_x_g,
        "accel_y_g": sample.accel_y_g,
        "accel_z_g": sample.accel_z_g,
        "gyro_x_dps": sample.gyro_x_dps,
        "gyro_y_dps": sample.gyro_y_dps,
        "gyro_z_dps": sample.gyro_z_dps,
    }
    optional_values = {
        "roll_deg": sample.roll_deg,
        "pitch_deg": sample.pitch_deg,
        "yaw_deg": sample.yaw_deg,
    }
    normalised: dict[str, float | None] = {
        name: _finite_session_number(value, field_name=f"{field_name}.{name}")
        for name, value in required_values.items()
    }
    for name, value in optional_values.items():
        normalised[name] = (
            None
            if value is None
            else _finite_session_number(value, field_name=f"{field_name}.{name}")
        )
    return ImuSample(**normalised)  # type: ignore[arg-type]


def _normalise_session_samples(
    samples_by_stage: Mapping[
        ValidationStage | str, Sequence[ImuSample]
    ],
) -> Mapping[ValidationStage, tuple[ImuSample, ...]]:
    if not isinstance(samples_by_stage, Mapping):
        raise TypeError("samples_by_stage must be a mapping")
    normalised: dict[ValidationStage, tuple[ImuSample, ...]] = {
        stage: () for stage in ValidationStage
    }
    seen: set[ValidationStage] = set()
    for raw_stage, raw_samples in samples_by_stage.items():
        stage = _session_stage(raw_stage, field_name="samples_by_stage")
        if stage in seen:
            raise ValueError(f"duplicate stage in samples_by_stage: {stage.value}")
        seen.add(stage)
        if isinstance(raw_samples, (str, bytes, bytearray)) or not isinstance(
            raw_samples, Sequence
        ):
            raise TypeError(f"samples_by_stage.{stage.value} must be a sequence")
        normalised[stage] = tuple(
            _normalise_session_sample(
                sample,
                field_name=f"samples_by_stage.{stage.value}[{index}]",
            )
            for index, sample in enumerate(raw_samples)
        )
    return MappingProxyType(normalised)


def _normalise_session_stage_set(
    values: Sequence[ValidationStage | str] | set[ValidationStage | str] | frozenset[ValidationStage | str],
    *,
    field_name: str,
) -> frozenset[ValidationStage]:
    if isinstance(values, (str, bytes, bytearray)) or not isinstance(
        values, (Sequence, set, frozenset)
    ):
        raise TypeError(f"{field_name} must be a stage sequence")
    normalised: list[ValidationStage] = []
    for value in values:
        stage = _session_stage(value, field_name=field_name)
        if stage in normalised:
            raise ValueError(f"duplicate stage in {field_name}: {stage.value}")
        normalised.append(stage)
    return frozenset(normalised)


def _validate_session_timestamp(value: str, *, field_name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise TypeError(f"{field_name} must be a non-empty ISO-8601 string")
    try:
        datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError(f"{field_name} must be an ISO-8601 timestamp") from exc
    return value


@dataclass(frozen=True)
class ImuSample:
    """One host-side validation sample.

    Accelerometer values are in g and gyroscope values are in degrees/second.
    Euler angles are optional diagnostic evidence and must never be inferred to
    be canonical FLU merely because their field names are roll/pitch/yaw.
    """

    timestamp_s: float
    accel_x_g: float
    accel_y_g: float
    accel_z_g: float
    gyro_x_dps: float
    gyro_y_dps: float
    gyro_z_dps: float
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None

    @property
    def accel_g(self) -> Vec3:
        return (self.accel_x_g, self.accel_y_g, self.accel_z_g)

    @property
    def gyro_dps(self) -> Vec3:
        return (self.gyro_x_dps, self.gyro_y_dps, self.gyro_z_dps)

    @property
    def attitude_deg(self) -> tuple[float | None, float | None, float | None]:
        return (self.roll_deg, self.pitch_deg, self.yaw_deg)


@dataclass(frozen=True)
class ValidationSession:
    """Restartable PC-side V0 capture state; never a target configuration."""

    samples_by_stage: Mapping[
        ValidationStage | str, Sequence[ImuSample]
    ] = field(default_factory=dict)
    unsupported_stages: frozenset[ValidationStage] | set[ValidationStage] | Sequence[ValidationStage | str] = field(
        default_factory=frozenset
    )
    skipped_stages: frozenset[ValidationStage] | set[ValidationStage] | Sequence[ValidationStage | str] = field(
        default_factory=frozenset
    )
    finished_stages: frozenset[ValidationStage] | set[ValidationStage] | Sequence[ValidationStage | str] = field(
        default_factory=frozenset
    )
    firmware_hash: str | None = None
    created_at: str = field(default_factory=_now_iso_timestamp)
    updated_at: str | None = None
    format: str = field(default=SESSION_FORMAT, init=False)
    schema: int = field(default=SESSION_SCHEMA, init=False)
    evidence_type: str = field(default="validation_session", init=False)
    evidence_only: bool = field(default=True, init=False)
    runtime_migration_complete: bool = field(default=False, init=False)
    automatic_calibration_performed: bool = field(default=False, init=False)
    parameter_changes_applied: bool = field(default=False, init=False)
    parameters_written: bool = field(default=False, init=False)
    flash_writes: int = field(default=0, init=False)
    firmware_written: bool = field(default=False, init=False)
    flight_release: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        created_at = _validate_session_timestamp(
            self.created_at, field_name="created_at"
        )
        updated_at = _validate_session_timestamp(
            self.updated_at or created_at, field_name="updated_at"
        )
        if self.firmware_hash is not None and not isinstance(self.firmware_hash, str):
            raise TypeError("firmware_hash must be a string or null")
        samples = _normalise_session_samples(self.samples_by_stage)
        unsupported = _normalise_session_stage_set(
            self.unsupported_stages, field_name="unsupported_stages"
        )
        skipped = _normalise_session_stage_set(
            self.skipped_stages, field_name="skipped_stages"
        )
        finished = _normalise_session_stage_set(
            self.finished_stages, field_name="finished_stages"
        )
        conflict = unsupported & skipped
        if conflict:
            names = ", ".join(sorted(stage.value for stage in conflict))
            raise ValueError(
                "stages cannot be both skipped and unsupported: " + names
            )
        invalid_skips = skipped - frozenset(OPTIONAL_STAGES)
        if invalid_skips:
            names = ", ".join(sorted(stage.value for stage in invalid_skips))
            raise ValueError("required stages cannot be skipped: " + names)
        object.__setattr__(self, "created_at", created_at)
        object.__setattr__(self, "updated_at", updated_at)
        object.__setattr__(self, "samples_by_stage", samples)
        object.__setattr__(self, "unsupported_stages", unsupported)
        object.__setattr__(self, "skipped_stages", skipped)
        object.__setattr__(self, "finished_stages", finished)


@dataclass(frozen=True)
class StaticStageResult:
    stage: str
    label_zh: str
    prompt_zh: str
    status: ValidationStatus
    capture_quality_status: ValidationStatus
    canonical_match_status: ValidationStatus
    sample_count: int
    valid_sample_count: int
    invalid_sample_count: int
    accel_mean_g: Vec3 | None
    accel_std_g: Vec3 | None
    gyro_mean_dps: Vec3 | None
    gyro_std_dps: Vec3 | None
    attitude_mean_deg: tuple[float | None, float | None, float | None]
    attitude_std_deg: tuple[float | None, float | None, float | None]
    accel_norm_mean_g: float | None
    accel_norm_std_g: float | None
    dominant_axis: str | None
    dominant_sign: int | None
    dominant_value_g: float | None
    expected_axis: str
    expected_sign: int
    canonical_alignment_g: float | None
    max_cross_axis_g: float | None
    findings: tuple[str, ...]


@dataclass(frozen=True)
class SignedPermutationCandidate:
    """Suggested observed-vector -> canonical-FLU mapping; never auto-applied."""

    status: ValidationStatus
    complete: bool
    recommendation_only: bool
    applied: bool
    matrix_flu_from_observed: SignedPermutation | None
    axis_mapping: Mapping[str, str]
    determinant: int | None
    proper_rotation: bool
    confidence: float
    findings: tuple[str, ...]


@dataclass(frozen=True)
class SixFaceResult:
    status: ValidationStatus
    capture_quality_status: ValidationStatus
    canonical_match_status: ValidationStatus
    sample_count: int
    stages: Mapping[str, StaticStageResult]
    candidate: SignedPermutationCandidate
    findings: tuple[str, ...]


@dataclass(frozen=True)
class RotationStageResult:
    stage: str
    label_zh: str
    prompt_zh: str
    status: ValidationStatus
    sample_count: int
    valid_sample_count: int
    invalid_sample_count: int
    duration_s: float
    target_axis: str
    expected_direction: str
    integrated_angle_deg: Vec3 | None
    peak_abs_dps: Vec3 | None
    target_positive_peak_dps: float | None
    target_negative_peak_dps: float | None
    dominant_axis: str | None
    dominant_sign: int | None
    axis_dominance_ratio: float | None
    gyro_direction_matches_flu: bool | None
    attitude_delta_deg: tuple[float | None, float | None, float | None]
    attitude_direction_matches_flu: bool | None
    timestamps_monotonic: bool
    findings: tuple[str, ...]


@dataclass(frozen=True)
class RotationResult:
    status: ValidationStatus
    sample_count: int
    stages: Mapping[str, RotationStageResult]
    findings: tuple[str, ...]


@dataclass(frozen=True)
class ValidationReport:
    """A read-only validation-evidence report, not a calibration certificate."""

    created_at: str
    firmware_hash: str | None
    data_source: str
    status: ValidationStatus
    six_face: SixFaceResult
    rotations: RotationResult
    thresholds: ValidationThresholds
    format: str = field(
        default="drone-h743-flight-validation-evidence-v1", init=False
    )
    evidence_type: str = field(default="validation_evidence", init=False)
    evidence_only: bool = field(default=True, init=False)
    frame_contract_version: int = field(default=1, init=False)
    frame_id: str = field(default="FLU", init=False)
    input_frame: str = field(default="legacy_or_unverified", init=False)
    input_units: Mapping[str, str] = field(
        default_factory=lambda: INPUT_UNITS, init=False
    )
    threshold_version: int = field(default=THRESHOLD_VERSION, init=False)
    runtime_migration_complete: bool = field(default=False, init=False)
    automatic_calibration_performed: bool = field(default=False, init=False)
    parameter_changes_applied: bool = field(default=False, init=False)
    parameters_written: bool = field(default=False, init=False)
    flash_writes: int = field(default=0, init=False)
    firmware_written: bool = field(default=False, init=False)
    flight_release: bool = field(default=False, init=False)
    notes: tuple[str, ...] = field(
        default=(
            "结果仅是 validation evidence，不代表自动校准完成。",
            "当前运行数据按 legacy_or_unverified 处理，不宣称 FLU 运行迁移完成。",
            "分析结果不会自动应用；须经上位机二次确认、RAM复验后才能提交。",
        ),
        init=False,
    )


_STATUS_RANK = {
    ValidationStatus.SKIPPED: -1,
    ValidationStatus.PASS: 0,
    ValidationStatus.WARN: 1,
    ValidationStatus.NOT_RUN: 2,
    ValidationStatus.UNSUPPORTED: 3,
    ValidationStatus.FAIL: 4,
}
_AXES = ("x", "y", "z")


def _worst_status(*statuses: ValidationStatus) -> ValidationStatus:
    return max(statuses, key=_STATUS_RANK.__getitem__)


def _upper_bound_status(
    value: float, pass_limit: float, warn_limit: float
) -> ValidationStatus:
    if value <= pass_limit:
        return ValidationStatus.PASS
    if value <= warn_limit:
        return ValidationStatus.WARN
    return ValidationStatus.FAIL


def _lower_bound_status(
    value: float, pass_limit: float, warn_limit: float
) -> ValidationStatus:
    if value >= pass_limit:
        return ValidationStatus.PASS
    if value >= warn_limit:
        return ValidationStatus.WARN
    return ValidationStatus.FAIL


def _stage(value: ValidationStage | str) -> ValidationStage:
    if isinstance(value, ValidationStage):
        return value
    return ValidationStage(value)


def _sample_is_valid(sample: ImuSample) -> bool:
    return all(
        math.isfinite(value)
        for value in (sample.timestamp_s, *sample.accel_g, *sample.gyro_dps)
    )


def _vector_mean(vectors: Sequence[Vec3]) -> Vec3:
    return tuple(statistics.fmean(vector[index] for vector in vectors) for index in range(3))  # type: ignore[return-value]


def _vector_pstdev(vectors: Sequence[Vec3]) -> Vec3:
    return tuple(statistics.pstdev(vector[index] for vector in vectors) for index in range(3))  # type: ignore[return-value]


def _optional_attitude_stats(
    samples: Sequence[ImuSample],
) -> tuple[
    tuple[float | None, float | None, float | None],
    tuple[float | None, float | None, float | None],
]:
    means: list[float | None] = []
    deviations: list[float | None] = []
    for index in range(3):
        values = [
            sample.attitude_deg[index]
            for sample in samples
            if sample.attitude_deg[index] is not None
            and math.isfinite(float(sample.attitude_deg[index]))
        ]
        means.append(statistics.fmean(values) if values else None)  # type: ignore[arg-type]
        deviations.append(statistics.pstdev(values) if values else None)  # type: ignore[arg-type]
    return (
        (means[0], means[1], means[2]),
        (deviations[0], deviations[1], deviations[2]),
    )


def _expected_axis_and_sign(expected: Vec3) -> tuple[str, int]:
    index = max(range(3), key=lambda axis: abs(expected[axis]))
    return _AXES[index], 1 if expected[index] > 0.0 else -1


def analyze_static_stage(
    stage: ValidationStage | str,
    samples: Sequence[ImuSample],
    thresholds: ValidationThresholds = DEFAULT_THRESHOLDS,
) -> StaticStageResult:
    """Analyze one stationary pose against the canonical FLU expectation."""

    stage_id = _stage(stage)
    definition = STAGE_DEFINITIONS[stage_id]
    if definition.kind != "static" or definition.expected_specific_force_g is None:
        raise ValueError(f"{stage_id.value} is not a static stage")

    expected = definition.expected_specific_force_g
    expected_axis, expected_sign = _expected_axis_and_sign(expected)
    sample_list = list(samples)
    valid = [sample for sample in sample_list if _sample_is_valid(sample)]
    invalid_count = len(sample_list) - len(valid)
    findings: list[str] = []

    if not valid:
        return StaticStageResult(
            stage=stage_id.value,
            label_zh=definition.label_zh,
            prompt_zh=definition.prompt_zh,
            status=ValidationStatus.FAIL,
            capture_quality_status=ValidationStatus.FAIL,
            canonical_match_status=ValidationStatus.FAIL,
            sample_count=len(sample_list),
            valid_sample_count=0,
            invalid_sample_count=invalid_count,
            accel_mean_g=None,
            accel_std_g=None,
            gyro_mean_dps=None,
            gyro_std_dps=None,
            attitude_mean_deg=(None, None, None),
            attitude_std_deg=(None, None, None),
            accel_norm_mean_g=None,
            accel_norm_std_g=None,
            dominant_axis=None,
            dominant_sign=None,
            dominant_value_g=None,
            expected_axis=expected_axis,
            expected_sign=expected_sign,
            canonical_alignment_g=None,
            max_cross_axis_g=None,
            findings=("没有有效样本，无法形成验证证据。",),
        )

    accel_vectors = [sample.accel_g for sample in valid]
    gyro_vectors = [sample.gyro_dps for sample in valid]
    accel_mean = _vector_mean(accel_vectors)
    accel_std = _vector_pstdev(accel_vectors)
    gyro_mean = _vector_mean(gyro_vectors)
    gyro_std = _vector_pstdev(gyro_vectors)
    accel_norms = [math.sqrt(sum(value * value for value in vector)) for vector in accel_vectors]
    accel_norm_mean = statistics.fmean(accel_norms)
    accel_norm_std = statistics.pstdev(accel_norms)
    attitude_mean, attitude_std = _optional_attitude_stats(valid)
    dominant_index = max(range(3), key=lambda index: abs(accel_mean[index]))
    dominant_axis = _AXES[dominant_index]
    dominant_sign = 1 if accel_mean[dominant_index] >= 0.0 else -1
    expected_index = _AXES.index(expected_axis)
    canonical_alignment = expected_sign * accel_mean[expected_index]
    max_cross_axis = max(
        abs(accel_mean[index]) for index in range(3) if index != expected_index
    )

    count_status = (
        ValidationStatus.FAIL
        if len(valid) < thresholds.static_min_samples
        else ValidationStatus.WARN
        if len(valid) < thresholds.static_recommended_samples
        else ValidationStatus.PASS
    )
    if count_status is not ValidationStatus.PASS:
        findings.append(
            f"有效样本 {len(valid)}，建议至少 {thresholds.static_recommended_samples} 个。"
        )
    invalid_fraction = invalid_count / len(sample_list) if sample_list else 1.0
    invalid_status = _upper_bound_status(
        invalid_fraction,
        thresholds.invalid_fraction_warn,
        thresholds.invalid_fraction_fail,
    )
    if invalid_count:
        findings.append(f"发现 {invalid_count} 个非有限值样本。")
    norm_status = _upper_bound_status(
        abs(accel_norm_mean - 1.0),
        thresholds.accel_norm_error_pass_g,
        thresholds.accel_norm_error_warn_g,
    )
    accel_noise_status = _upper_bound_status(
        max(accel_std), thresholds.accel_std_pass_g, thresholds.accel_std_warn_g
    )
    gyro_bias_status = _upper_bound_status(
        max(abs(value) for value in gyro_mean),
        thresholds.gyro_bias_pass_dps,
        thresholds.gyro_bias_warn_dps,
    )
    gyro_noise_status = _upper_bound_status(
        max(gyro_std), thresholds.gyro_std_pass_dps, thresholds.gyro_std_warn_dps
    )
    capture_quality = _worst_status(
        count_status,
        invalid_status,
        norm_status,
        accel_noise_status,
        gyro_bias_status,
        gyro_noise_status,
    )
    if norm_status is not ValidationStatus.PASS:
        findings.append(f"加速度模长均值为 {accel_norm_mean:.4f} g。")
    if gyro_bias_status is not ValidationStatus.PASS:
        findings.append("静止陀螺均值偏大。")
    if accel_noise_status is not ValidationStatus.PASS or gyro_noise_status is not ValidationStatus.PASS:
        findings.append("静止噪声高于 PASS 阈值。")

    axis_error = abs(canonical_alignment - 1.0)
    axis_status = _worst_status(
        _upper_bound_status(
            axis_error,
            thresholds.canonical_axis_error_pass_g,
            thresholds.canonical_axis_error_warn_g,
        ),
        _upper_bound_status(
            max_cross_axis,
            thresholds.canonical_cross_axis_pass_g,
            thresholds.canonical_cross_axis_warn_g,
        ),
    )
    if dominant_axis != expected_axis or dominant_sign != expected_sign:
        axis_status = ValidationStatus.FAIL
        findings.append(
            f"观测主轴为 {dominant_sign:+d}{dominant_axis}，FLU 期望为 {expected_sign:+d}{expected_axis}。"
        )
    elif axis_status is not ValidationStatus.PASS:
        findings.append("主轴方向正确，但轴向或交叉轴误差超出 PASS 阈值。")

    return StaticStageResult(
        stage=stage_id.value,
        label_zh=definition.label_zh,
        prompt_zh=definition.prompt_zh,
        status=_worst_status(capture_quality, axis_status),
        capture_quality_status=capture_quality,
        canonical_match_status=axis_status,
        sample_count=len(sample_list),
        valid_sample_count=len(valid),
        invalid_sample_count=invalid_count,
        accel_mean_g=accel_mean,
        accel_std_g=accel_std,
        gyro_mean_dps=gyro_mean,
        gyro_std_dps=gyro_std,
        attitude_mean_deg=attitude_mean,
        attitude_std_deg=attitude_std,
        accel_norm_mean_g=accel_norm_mean,
        accel_norm_std_g=accel_norm_std,
        dominant_axis=dominant_axis,
        dominant_sign=dominant_sign,
        dominant_value_g=accel_mean[dominant_index],
        expected_axis=expected_axis,
        expected_sign=expected_sign,
        canonical_alignment_g=canonical_alignment,
        max_cross_axis_g=max_cross_axis,
        findings=tuple(findings),
    )


def _determinant(matrix: SignedPermutation) -> int:
    a, b, c = matrix
    return (
        a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
    )


def _infer_signed_permutation(
    stages: Mapping[str, StaticStageResult],
    thresholds: ValidationThresholds,
) -> SignedPermutationCandidate:
    positive_bases = (
        ("x", ValidationStage.NOSE_UP),
        ("y", ValidationStage.LEFT_SIDE_UP),
        ("z", ValidationStage.LEVEL),
    )
    optional_opposites = (
        (ValidationStage.NOSE_UP, ValidationStage.NOSE_DOWN),
        (ValidationStage.LEFT_SIDE_UP, ValidationStage.RIGHT_SIDE_UP),
        (ValidationStage.LEVEL, ValidationStage.INVERTED),
    )
    rows: list[tuple[int, int, int]] = []
    mapping: dict[str, str] = {}
    source_indices: list[int] = []
    confidence_values: list[float] = []
    findings: list[str] = []
    optional_conflict = False

    # Three positive, mutually orthogonal static poses directly observe the
    # canonical +X, +Y, and +Z basis vectors.  Opposite poses are not needed to
    # solve a signed permutation and therefore never gate candidate completeness.
    for flu_axis, positive_stage in positive_bases:
        positive = stages[positive_stage.value]
        if (
            positive.accel_mean_g is None
            or positive.dominant_axis is None
            or positive.dominant_sign is None
            or positive.accel_norm_mean_g is None
            or positive.dominant_value_g is None
        ):
            findings.append(f"FLU {flu_axis} 轴缺少有效的正向静置证据。")
            continue

        source_index = _AXES.index(positive.dominant_axis)
        sign = positive.dominant_sign
        row = [0, 0, 0]
        row[source_index] = sign
        rows.append((row[0], row[1], row[2]))
        source_indices.append(source_index)
        sign_text = "+" if sign > 0 else "-"
        mapping[f"flu_{flu_axis}"] = f"{sign_text}observed_{positive.dominant_axis}"
        confidence_values.append(
            abs(positive.dominant_value_g)
            / max(positive.accel_norm_mean_g, 1e-12)
        )

    # Optional opposite poses may expose a clear contradiction, but their
    # absence, unsupported state, or hand-held WARN quality must not reduce the
    # status of the required three-pose workflow.
    for positive_stage, opposite_stage in optional_opposites:
        positive = stages[positive_stage.value]
        opposite = stages[opposite_stage.value]
        if (
            opposite.sample_count == 0
            or opposite.valid_sample_count == 0
            or opposite.capture_quality_status is ValidationStatus.FAIL
            or positive.dominant_axis is None
            or positive.dominant_sign is None
            or opposite.dominant_axis is None
            or opposite.dominant_sign is None
        ):
            continue
        if (
            opposite.dominant_axis != positive.dominant_axis
            or opposite.dominant_sign != -positive.dominant_sign
        ):
            optional_conflict = True
            findings.append(
                f"可选姿态 {opposite_stage.value} 与 {positive_stage.value} "
                "的主轴/符号不互反。"
            )

    complete = len(rows) == 3 and len(set(source_indices)) == 3
    if len(rows) == 3 and len(set(source_indices)) != 3:
        findings.append("候选映射重复使用同一观测轴，不是 signed permutation。")
    confidence = min(confidence_values, default=0.0)
    matrix: SignedPermutation | None = None
    determinant: int | None = None
    proper_rotation = False
    if complete:
        matrix = (rows[0], rows[1], rows[2])
        determinant = _determinant(matrix)
        proper_rotation = determinant == 1
        if not proper_rotation:
            findings.append(
                "候选 signed permutation 的 determinant 不是 +1，不能作为物理安装旋转应用。"
            )
    else:
        findings.append("三个必需正向静置方向证据不足，无法形成完整候选 signed permutation。")

    unavailable_statuses = tuple(
        stages[stage.value].status
        for _, stage in positive_bases
        if stages[stage.value].status
        in (ValidationStatus.NOT_RUN, ValidationStatus.UNSUPPORTED)
    )
    if unavailable_statuses:
        status = _worst_status(*unavailable_statuses)
    elif not complete:
        status = ValidationStatus.FAIL
    elif optional_conflict:
        status = ValidationStatus.FAIL
    elif not proper_rotation:
        status = ValidationStatus.FAIL
    else:
        status = _lower_bound_status(
            confidence,
            thresholds.candidate_confidence_pass,
            thresholds.candidate_confidence_warn,
        )
    findings.append("候选矩阵不会自动应用；须经上位机二次确认和RAM复验。")
    return SignedPermutationCandidate(
        status=status,
        complete=complete,
        recommendation_only=True,
        applied=False,
        matrix_flu_from_observed=matrix,
        axis_mapping=MappingProxyType(mapping),
        determinant=determinant,
        proper_rotation=proper_rotation,
        confidence=confidence,
        findings=tuple(findings),
    )


def _normalise_stage_samples(
    samples_by_stage: Mapping[
        ValidationStage | str, Sequence[ImuSample] | None
    ],
) -> dict[ValidationStage, Sequence[ImuSample] | None]:
    normalised: dict[ValidationStage, Sequence[ImuSample] | None] = {}
    for key, samples in samples_by_stage.items():
        stage_id = _stage(key)
        if stage_id in normalised:
            raise ValueError(f"duplicate stage: {stage_id.value}")
        normalised[stage_id] = samples
    return normalised


def _unavailable_static_stage(
    stage: ValidationStage, status: ValidationStatus
) -> StaticStageResult:
    if status not in (
        ValidationStatus.SKIPPED,
        ValidationStatus.NOT_RUN,
        ValidationStatus.UNSUPPORTED,
    ):
        raise ValueError("unavailable stage requires SKIPPED, NOT_RUN or UNSUPPORTED")
    messages = {
        ValidationStatus.SKIPPED: "该可选阶段未执行，不阻止整体验收。",
        ValidationStatus.NOT_RUN: "该必需阶段尚未执行。",
        ValidationStatus.UNSUPPORTED: "当前数据源不支持该阶段所需信号。",
    }
    return replace(
        analyze_static_stage(stage, ()),
        status=status,
        capture_quality_status=status,
        canonical_match_status=status,
        findings=(messages[status],),
    )


def analyze_six_face(
    samples_by_stage: Mapping[
        ValidationStage | str, Sequence[ImuSample] | None
    ],
    thresholds: ValidationThresholds = DEFAULT_THRESHOLDS,
) -> SixFaceResult:
    """Analyze compatible static stages and infer from the three required bases."""

    normalised = _normalise_stage_samples(samples_by_stage)
    results: dict[str, StaticStageResult] = {}
    for stage in STATIC_STAGES:
        if stage not in normalised:
            missing_status = (
                ValidationStatus.NOT_RUN
                if STAGE_DEFINITIONS[stage].required
                else ValidationStatus.SKIPPED
            )
            result = _unavailable_static_stage(stage, missing_status)
        elif normalised[stage] is None:
            result = _unavailable_static_stage(stage, ValidationStatus.UNSUPPORTED)
        else:
            result = analyze_static_stage(
                stage, normalised[stage], thresholds=thresholds
            )
        results[stage.value] = result
    candidate = _infer_signed_permutation(results, thresholds)
    required_results = tuple(
        results[stage.value]
        for stage in STATIC_STAGES
        if STAGE_DEFINITIONS[stage].required
    )
    capture_quality = _worst_status(
        *(result.capture_quality_status for result in required_results)
    )
    canonical_match = _worst_status(
        *(result.canonical_match_status for result in required_results)
    )
    status = _worst_status(capture_quality, canonical_match, candidate.status)
    findings: list[str] = []
    if canonical_match is ValidationStatus.FAIL:
        findings.append("至少一个静置方向与 canonical FLU 不匹配。")
    if candidate.complete:
        findings.append("已生成 signed permutation 候选，等待上位机二次确认。")
    return SixFaceResult(
        status=status,
        capture_quality_status=capture_quality,
        canonical_match_status=canonical_match,
        sample_count=sum(result.sample_count for result in results.values()),
        stages=MappingProxyType(results),
        candidate=candidate,
        findings=tuple(findings),
    )


def _integrate_vectors(samples: Sequence[ImuSample]) -> Vec3:
    integrals = [0.0, 0.0, 0.0]
    for previous, current in zip(samples, samples[1:]):
        dt = current.timestamp_s - previous.timestamp_s
        for index in range(3):
            integrals[index] += (
                previous.gyro_dps[index] + current.gyro_dps[index]
            ) * 0.5 * dt
    return (integrals[0], integrals[1], integrals[2])


def _wrapped_delta_degrees(values: Sequence[float]) -> float:
    total = 0.0
    for previous, current in zip(values, values[1:]):
        delta = (current - previous + 180.0) % 360.0 - 180.0
        total += delta
    return total


def _attitude_deltas(
    samples: Sequence[ImuSample],
) -> tuple[float | None, float | None, float | None]:
    deltas: list[float | None] = []
    for index in range(3):
        values = [sample.attitude_deg[index] for sample in samples]
        if any(value is None or not math.isfinite(float(value)) for value in values):
            deltas.append(None)
        else:
            deltas.append(_wrapped_delta_degrees(values))  # type: ignore[arg-type]
    return (deltas[0], deltas[1], deltas[2])


def analyze_rotation_stage(
    stage: ValidationStage | str,
    samples: Sequence[ImuSample],
    thresholds: ValidationThresholds = DEFAULT_THRESHOLDS,
) -> RotationStageResult:
    """Check one explicitly positive FLU hand rotation using gyro integration."""

    stage_id = _stage(stage)
    definition = STAGE_DEFINITIONS[stage_id]
    if definition.kind != "rotation" or definition.expected_gyro_axis is None:
        raise ValueError(f"{stage_id.value} is not a rotation stage")
    target_axis = definition.expected_gyro_axis
    target_index = _AXES.index(target_axis)
    sample_list = list(samples)
    valid = [sample for sample in sample_list if _sample_is_valid(sample)]
    invalid_count = len(sample_list) - len(valid)
    monotonic = all(
        current.timestamp_s > previous.timestamp_s
        for previous, current in zip(valid, valid[1:])
    )
    findings: list[str] = []

    count_status = (
        ValidationStatus.FAIL
        if len(valid) < thresholds.rotation_min_samples
        else ValidationStatus.WARN
        if len(valid) < thresholds.rotation_recommended_samples
        else ValidationStatus.PASS
    )
    if count_status is not ValidationStatus.PASS:
        findings.append(
            f"有效样本 {len(valid)}，建议至少 {thresholds.rotation_recommended_samples} 个。"
        )
    if invalid_count:
        findings.append(f"发现 {invalid_count} 个非有限值样本。")
    if not monotonic:
        findings.append("时间戳不严格递增，无法可信积分。")

    if len(valid) < 2 or not monotonic:
        return RotationStageResult(
            stage=stage_id.value,
            label_zh=definition.label_zh,
            prompt_zh=definition.prompt_zh,
            status=ValidationStatus.FAIL,
            sample_count=len(sample_list),
            valid_sample_count=len(valid),
            invalid_sample_count=invalid_count,
            duration_s=0.0,
            target_axis=target_axis,
            expected_direction="positive",
            integrated_angle_deg=None,
            peak_abs_dps=None,
            target_positive_peak_dps=None,
            target_negative_peak_dps=None,
            dominant_axis=None,
            dominant_sign=None,
            axis_dominance_ratio=None,
            gyro_direction_matches_flu=None,
            attitude_delta_deg=(None, None, None),
            attitude_direction_matches_flu=None,
            timestamps_monotonic=monotonic,
            findings=tuple(findings),
        )

    duration = valid[-1].timestamp_s - valid[0].timestamp_s
    integrated = _integrate_vectors(valid)
    peaks = tuple(
        max(abs(sample.gyro_dps[index]) for sample in valid) for index in range(3)
    )
    positive_peak = max(sample.gyro_dps[target_index] for sample in valid)
    negative_peak = min(sample.gyro_dps[target_index] for sample in valid)
    dominant_index = max(range(3), key=lambda index: abs(integrated[index]))
    dominant_sign = 1 if integrated[dominant_index] >= 0.0 else -1
    off_axis_angle = max(
        abs(integrated[index]) for index in range(3) if index != target_index
    )
    dominance = abs(integrated[target_index]) / max(off_axis_angle, 1e-12)
    target_angle = integrated[target_index]
    if target_angle >= thresholds.rotation_angle_warn_deg:
        gyro_matches: bool | None = True
    elif target_angle <= -thresholds.rotation_angle_warn_deg:
        gyro_matches = False
    else:
        gyro_matches = None

    attitude_delta = _attitude_deltas(valid)
    target_attitude_delta = attitude_delta[target_index]
    if target_attitude_delta is None or abs(target_attitude_delta) < thresholds.attitude_direction_min_deg:
        attitude_matches: bool | None = None
    else:
        attitude_matches = target_attitude_delta > 0.0

    invalid_fraction = invalid_count / len(sample_list) if sample_list else 1.0
    invalid_status = _upper_bound_status(
        invalid_fraction,
        thresholds.invalid_fraction_warn,
        thresholds.invalid_fraction_fail,
    )
    duration_status = (
        ValidationStatus.PASS
        if duration >= thresholds.rotation_min_duration_s
        else ValidationStatus.FAIL
    )
    angle_status = _lower_bound_status(
        target_angle,
        thresholds.rotation_angle_pass_deg,
        thresholds.rotation_angle_warn_deg,
    )
    peak_status = _lower_bound_status(
        positive_peak,
        thresholds.rotation_peak_pass_dps,
        thresholds.rotation_peak_warn_dps,
    )
    dominance_status = _lower_bound_status(
        dominance,
        thresholds.rotation_dominance_pass,
        thresholds.rotation_dominance_warn,
    )
    status = _worst_status(
        count_status,
        invalid_status,
        duration_status,
        angle_status,
        peak_status,
        dominance_status,
    )
    if gyro_matches is False:
        status = ValidationStatus.FAIL
        findings.append(
            f"正 {target_axis} 动作得到负向陀螺积分，方向与 canonical FLU 相反。"
        )
    elif gyro_matches is None:
        findings.append("目标轴转角不足，方向证据不充分。")
    if dominant_index != target_index:
        status = ValidationStatus.FAIL
        findings.append(f"积分主轴为 {_AXES[dominant_index]}，不是目标 {target_axis} 轴。")
    elif dominance_status is not ValidationStatus.PASS:
        findings.append("目标轴相对交叉轴的主导程度不足。")
    if attitude_matches is False:
        status = ValidationStatus.FAIL
        findings.append("可选姿态输出的变化方向与同轴陀螺/FLU 期望相反。")
    elif attitude_matches is None:
        findings.append("未获得足够的可选姿态变化证据；不影响纯陀螺方向判定。")

    return RotationStageResult(
        stage=stage_id.value,
        label_zh=definition.label_zh,
        prompt_zh=definition.prompt_zh,
        status=status,
        sample_count=len(sample_list),
        valid_sample_count=len(valid),
        invalid_sample_count=invalid_count,
        duration_s=duration,
        target_axis=target_axis,
        expected_direction="positive",
        integrated_angle_deg=integrated,
        peak_abs_dps=peaks,  # type: ignore[arg-type]
        target_positive_peak_dps=positive_peak,
        target_negative_peak_dps=negative_peak,
        dominant_axis=_AXES[dominant_index],
        dominant_sign=dominant_sign,
        axis_dominance_ratio=dominance,
        gyro_direction_matches_flu=gyro_matches,
        attitude_delta_deg=attitude_delta,
        attitude_direction_matches_flu=attitude_matches,
        timestamps_monotonic=monotonic,
        findings=tuple(findings),
    )


def _unavailable_rotation_stage(
    stage: ValidationStage, status: ValidationStatus
) -> RotationStageResult:
    if status not in (
        ValidationStatus.SKIPPED,
        ValidationStatus.NOT_RUN,
        ValidationStatus.UNSUPPORTED,
    ):
        raise ValueError("unavailable stage requires SKIPPED, NOT_RUN or UNSUPPORTED")
    messages = {
        ValidationStatus.SKIPPED: "该可选阶段未执行，不阻止整体验收。",
        ValidationStatus.NOT_RUN: "该必需阶段尚未执行。",
        ValidationStatus.UNSUPPORTED: "当前数据源不支持该阶段所需信号。",
    }
    return replace(
        analyze_rotation_stage(stage, ()),
        status=status,
        findings=(messages[status],),
    )


def analyze_rotations(
    samples_by_stage: Mapping[
        ValidationStage | str, Sequence[ImuSample] | None
    ],
    thresholds: ValidationThresholds = DEFAULT_THRESHOLDS,
) -> RotationResult:
    """Analyze the +roll, +pitch, and +yaw guided hand rotations."""

    normalised = _normalise_stage_samples(samples_by_stage)
    results: dict[str, RotationStageResult] = {}
    for stage in ROTATION_STAGES:
        if stage not in normalised:
            missing_status = (
                ValidationStatus.NOT_RUN
                if STAGE_DEFINITIONS[stage].required
                else ValidationStatus.SKIPPED
            )
            result = _unavailable_rotation_stage(stage, missing_status)
        elif normalised[stage] is None:
            result = _unavailable_rotation_stage(
                stage, ValidationStatus.UNSUPPORTED
            )
        else:
            result = analyze_rotation_stage(
                stage, normalised[stage], thresholds=thresholds
            )
        results[stage.value] = result
    status = _worst_status(*(result.status for result in results.values()))
    findings = (
        "按 canonical FLU 判定：+roll→gyro_x>0，+pitch→gyro_y>0，+yaw→gyro_z>0。",
    )
    return RotationResult(
        status=status,
        sample_count=sum(result.sample_count for result in results.values()),
        stages=MappingProxyType(results),
        findings=findings,
    )


def build_validation_report(
    samples_by_stage: Mapping[
        ValidationStage | str, Sequence[ImuSample] | None
    ],
    *,
    firmware_hash: str | None = None,
    data_source: str = "caller_supplied_legacy_or_unverified_samples",
    created_at: str | None = None,
    thresholds: ValidationThresholds = DEFAULT_THRESHOLDS,
) -> ValidationReport:
    """Build immutable evidence from captured data without touching the target."""

    six_face = analyze_six_face(samples_by_stage, thresholds=thresholds)
    rotations = analyze_rotations(samples_by_stage, thresholds=thresholds)
    timestamp = created_at or datetime.now().astimezone().isoformat()
    return ValidationReport(
        created_at=timestamp,
        firmware_hash=firmware_hash,
        data_source=data_source,
        status=_worst_status(six_face.status, rotations.status),
        six_face=six_face,
        rotations=rotations,
        thresholds=thresholds,
    )


def _primitive(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value) and not isinstance(value, type):
        return {
            item.name: _primitive(getattr(value, item.name)) for item in fields(value)
        }
    if isinstance(value, Mapping):
        return {str(key): _primitive(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_primitive(item) for item in value]
    return value


_SESSION_SAMPLE_FIELDS = (
    "timestamp_s",
    "accel_x_g",
    "accel_y_g",
    "accel_z_g",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
)
_SESSION_SAFETY_FIELDS: Mapping[str, Any] = MappingProxyType(
    {
        "evidence_type": "validation_session",
        "evidence_only": True,
        "runtime_migration_complete": False,
        "automatic_calibration_performed": False,
        "parameter_changes_applied": False,
        "parameters_written": False,
        "flash_writes": 0,
        "firmware_written": False,
        "flight_release": False,
    }
)
_SESSION_FIELDS = frozenset(
    {
        "format",
        "schema",
        "created_at",
        "updated_at",
        "firmware_hash",
        "samples_by_stage",
        "unsupported_stages",
        "skipped_stages",
        "finished_stages",
        *_SESSION_SAFETY_FIELDS,
    }
)


def _sample_to_session_dict(sample: ImuSample) -> dict[str, float | None]:
    sample = _normalise_session_sample(sample, field_name="sample")
    return {name: getattr(sample, name) for name in _SESSION_SAMPLE_FIELDS}


def session_to_dict(session: ValidationSession) -> dict[str, Any]:
    """Return the complete, JSON-compatible V0 capture session."""

    if not isinstance(session, ValidationSession):
        raise TypeError("session must be a ValidationSession")
    result: dict[str, Any] = {
        "format": session.format,
        "schema": session.schema,
        "created_at": session.created_at,
        "updated_at": session.updated_at,
        "firmware_hash": session.firmware_hash,
        "samples_by_stage": {
            stage.value: [
                _sample_to_session_dict(sample)
                for sample in session.samples_by_stage[stage]
            ]
            for stage in ValidationStage
        },
        "unsupported_stages": [
            stage.value for stage in ValidationStage if stage in session.unsupported_stages
        ],
        "skipped_stages": [
            stage.value for stage in ValidationStage if stage in session.skipped_stages
        ],
        "finished_stages": [
            stage.value for stage in ValidationStage if stage in session.finished_stages
        ],
    }
    result.update(_SESSION_SAFETY_FIELDS)
    return result


def session_to_json(session: ValidationSession, *, indent: int | None = 2) -> str:
    """Serialize a restartable evidence session as strict UTF-8 JSON text."""

    return json.dumps(
        session_to_dict(session),
        ensure_ascii=False,
        indent=indent,
        allow_nan=False,
    ) + "\n"


def _sample_from_session_dict(value: Any, *, field_name: str) -> ImuSample:
    if not isinstance(value, Mapping):
        raise TypeError(f"{field_name} must be an object")
    keys = set(value)
    expected = set(_SESSION_SAMPLE_FIELDS)
    if keys != expected:
        unknown = sorted(str(key) for key in keys - expected)
        missing = sorted(expected - keys)
        raise ValueError(
            f"{field_name} fields mismatch; unknown={unknown}, missing={missing}"
        )
    required = {
        name: _finite_session_number(value[name], field_name=f"{field_name}.{name}")
        for name in _SESSION_SAMPLE_FIELDS[:7]
    }
    optional = {
        name: (
            None
            if value[name] is None
            else _finite_session_number(value[name], field_name=f"{field_name}.{name}")
        )
        for name in _SESSION_SAMPLE_FIELDS[7:]
    }
    return ImuSample(**required, **optional)


def session_from_dict(payload: Mapping[str, Any]) -> ValidationSession:
    """Validate and restore a session; unsafe or ambiguous input is rejected."""

    if not isinstance(payload, Mapping):
        raise TypeError("session payload must be an object")
    keys = set(payload)
    if keys != _SESSION_FIELDS:
        unknown = sorted(str(key) for key in keys - _SESSION_FIELDS)
        missing = sorted(_SESSION_FIELDS - keys)
        raise ValueError(
            f"session fields mismatch; unknown={unknown}, missing={missing}"
        )
    if payload["format"] != SESSION_FORMAT:
        raise ValueError(f"unsupported session format: {payload['format']!r}")
    if type(payload["schema"]) is not int or payload["schema"] != SESSION_SCHEMA:
        raise ValueError(f"unsupported session schema: {payload['schema']!r}")
    for name, expected in _SESSION_SAFETY_FIELDS.items():
        value = payload[name]
        if type(value) is not type(expected) or value != expected:
            raise ValueError(f"unsafe or invalid session field {name}: {value!r}")

    raw_samples = payload["samples_by_stage"]
    if not isinstance(raw_samples, Mapping):
        raise TypeError("samples_by_stage must be an object")
    stage_keys = set(raw_samples)
    expected_stage_keys = {stage.value for stage in ValidationStage}
    unknown_stages = sorted(str(key) for key in stage_keys - expected_stage_keys)
    missing_stages = sorted(expected_stage_keys - stage_keys)
    if unknown_stages or missing_stages:
        raise ValueError(
            "samples_by_stage stage mismatch; "
            f"unknown={unknown_stages}, missing={missing_stages}"
        )
    samples: dict[ValidationStage, tuple[ImuSample, ...]] = {}
    for stage in ValidationStage:
        rows = raw_samples[stage.value]
        if isinstance(rows, (str, bytes, bytearray)) or not isinstance(rows, Sequence):
            raise TypeError(f"samples_by_stage.{stage.value} must be an array")
        samples[stage] = tuple(
            _sample_from_session_dict(
                row, field_name=f"samples_by_stage.{stage.value}[{index}]"
            )
            for index, row in enumerate(rows)
        )

    return ValidationSession(
        samples_by_stage=samples,
        unsupported_stages=payload["unsupported_stages"],
        skipped_stages=payload["skipped_stages"],
        finished_stages=payload["finished_stages"],
        firmware_hash=payload["firmware_hash"],
        created_at=payload["created_at"],
        updated_at=payload["updated_at"],
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


def session_from_json(text: str) -> ValidationSession:
    """Parse strict JSON and restore a validated V0 session."""

    if not isinstance(text, str):
        raise TypeError("session JSON must be text")
    payload = json.loads(
        text,
        object_pairs_hook=_strict_json_object,
        parse_constant=_reject_json_constant,
    )
    return session_from_dict(payload)


def write_session(session: ValidationSession, path: Path | str) -> Path:
    """Persist PC-side capture state; this never communicates with the aircraft."""

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".tmp")
    temporary.write_text(session_to_json(session), encoding="utf-8")
    temporary.replace(output)
    return output


def load_session(path: Path | str) -> ValidationSession:
    """Load and strictly validate a previously persisted V0 session."""

    return session_from_json(Path(path).read_text(encoding="utf-8"))


def report_to_dict(report: ValidationReport) -> dict[str, Any]:
    """Return a JSON-compatible report dictionary."""

    result = _primitive(report)
    assert isinstance(result, dict)
    return result


def report_to_json(report: ValidationReport, *, indent: int | None = 2) -> str:
    """Serialize evidence as UTF-8-safe JSON text."""

    return json.dumps(report_to_dict(report), ensure_ascii=False, indent=indent) + "\n"


_CSV_FIELDS = (
    "format",
    "evidence_type",
    "evidence_only",
    "created_at",
    "firmware_hash",
    "data_source",
    "frame_contract_version",
    "frame_id",
    "input_frame",
    "input_units",
    "threshold_version",
    "runtime_migration_complete",
    "parameter_changes_applied",
    "flash_writes",
    "flight_release",
    "overall_status",
    "section",
    "stage",
    "label_zh",
    "status",
    "sample_count",
    "accel_mean_g",
    "accel_std_g",
    "gyro_mean_dps",
    "gyro_std_dps",
    "accel_norm_mean_g",
    "accel_norm_std_g",
    "dominant_axis",
    "dominant_sign",
    "integrated_angle_deg",
    "peak_abs_dps",
    "direction_matches_flu",
    "candidate_matrix_flu_from_observed",
    "candidate_axis_mapping",
    "candidate_recommendation_only",
    "findings",
)


def _csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, (tuple, list, Mapping)):
        return json.dumps(_primitive(value), ensure_ascii=False, separators=(",", ":"))
    if isinstance(value, bool):
        return str(value).lower()
    return value


def report_to_csv(report: ValidationReport) -> str:
    """Serialize a flattened, one-row-per-stage validation evidence report."""

    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=_CSV_FIELDS, lineterminator="\n")
    writer.writeheader()
    common = {
        "format": report.format,
        "evidence_type": report.evidence_type,
        "evidence_only": report.evidence_only,
        "created_at": report.created_at,
        "firmware_hash": report.firmware_hash,
        "data_source": report.data_source,
        "frame_contract_version": report.frame_contract_version,
        "frame_id": report.frame_id,
        "input_frame": report.input_frame,
        "input_units": report.input_units,
        "threshold_version": report.threshold_version,
        "runtime_migration_complete": report.runtime_migration_complete,
        "parameter_changes_applied": report.parameter_changes_applied,
        "flash_writes": report.flash_writes,
        "flight_release": report.flight_release,
        "overall_status": report.status,
    }
    candidate = report.six_face.candidate
    for result in report.six_face.stages.values():
        row = {
            **common,
            "section": "six_face",
            "stage": result.stage,
            "label_zh": result.label_zh,
            "status": result.status,
            "sample_count": result.sample_count,
            "accel_mean_g": result.accel_mean_g,
            "accel_std_g": result.accel_std_g,
            "gyro_mean_dps": result.gyro_mean_dps,
            "gyro_std_dps": result.gyro_std_dps,
            "accel_norm_mean_g": result.accel_norm_mean_g,
            "accel_norm_std_g": result.accel_norm_std_g,
            "dominant_axis": result.dominant_axis,
            "dominant_sign": result.dominant_sign,
            "candidate_matrix_flu_from_observed": candidate.matrix_flu_from_observed,
            "candidate_axis_mapping": candidate.axis_mapping,
            "candidate_recommendation_only": candidate.recommendation_only,
            "findings": result.findings,
        }
        writer.writerow({key: _csv_value(row.get(key)) for key in _CSV_FIELDS})
    for result in report.rotations.stages.values():
        row = {
            **common,
            "section": "rotation",
            "stage": result.stage,
            "label_zh": result.label_zh,
            "status": result.status,
            "sample_count": result.sample_count,
            "dominant_axis": result.dominant_axis,
            "dominant_sign": result.dominant_sign,
            "integrated_angle_deg": result.integrated_angle_deg,
            "peak_abs_dps": result.peak_abs_dps,
            "direction_matches_flu": result.gyro_direction_matches_flu,
            "findings": result.findings,
        }
        writer.writerow({key: _csv_value(row.get(key)) for key in _CSV_FIELDS})
    return stream.getvalue()


def write_json_report(report: ValidationReport, path: Path) -> Path:
    """Write PC-side evidence JSON; this never communicates with the aircraft."""

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(report_to_json(report), encoding="utf-8")
    return path


def write_csv_report(report: ValidationReport, path: Path) -> Path:
    """Write PC-side evidence CSV; this never communicates with the aircraft."""

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(report_to_csv(report), encoding="utf-8", newline="")
    return path


__all__ = [
    "DEFAULT_THRESHOLDS",
    "INPUT_UNITS",
    "OPTIONAL_STAGES",
    "REQUIRED_STAGES",
    "ROTATION_STAGES",
    "SESSION_FORMAT",
    "SESSION_SCHEMA",
    "STAGE_DEFINITIONS",
    "STATIC_STAGES",
    "THRESHOLD_VERSION",
    "ImuSample",
    "RotationResult",
    "RotationStageResult",
    "SignedPermutationCandidate",
    "SixFaceResult",
    "StageDefinition",
    "StaticStageResult",
    "ValidationReport",
    "ValidationSession",
    "ValidationStage",
    "ValidationStatus",
    "ValidationThresholds",
    "analyze_rotation_stage",
    "analyze_rotations",
    "analyze_six_face",
    "analyze_static_stage",
    "build_validation_report",
    "load_session",
    "report_to_csv",
    "report_to_dict",
    "report_to_json",
    "session_from_dict",
    "session_from_json",
    "session_to_dict",
    "session_to_json",
    "write_session",
    "write_csv_report",
    "write_json_report",
]
