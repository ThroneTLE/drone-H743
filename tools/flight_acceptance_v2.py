#!/usr/bin/env python3
"""Strict, evidence-only V2A flight-control acceptance engine."""

from __future__ import annotations

import hashlib
import json
import math
import statistics
from dataclasses import dataclass, field, fields, is_dataclass
from datetime import datetime
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Sequence


V2_REPORT_FORMAT = "drone-h743-flight-acceptance-v2a-report"
V2_REPORT_SCHEMA = 2
CANONICAL_FRAME_ID = "FLU"
SAFE_FAILSAFE_MODE = "safe"
ACCEPTANCE_PROVENANCE = "flight_acceptance_v2"
ACCEPTANCE_CAPTURE_SOURCE = "target_acceptance_snapshot"


class AcceptanceStatus(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    NOT_RUN = "NOT_RUN"
    SKIPPED = "SKIPPED"
    UNSUPPORTED = "UNSUPPORTED"


class ServoType(str, Enum):
    """The actuator feedback contract selected for this acceptance report."""

    BUS = "bus"
    PWM = "pwm"


class V2Stage(str, Enum):
    RC_CENTER = "rc_center"
    RC_POSITIVE_ROLL = "rc_positive_roll"
    RC_POSITIVE_PITCH = "rc_positive_pitch"
    RC_POSITIVE_YAW = "rc_positive_yaw"
    NAV_STATIC = "nav_static"
    NAV_FORWARD = "nav_forward"
    NAV_LEFT = "nav_left"
    RESTORE_POSITIVE_ROLL = "restore_positive_roll"
    RESTORE_POSITIVE_PITCH = "restore_positive_pitch"
    SERVO_ALPHA_POSITIVE = "servo_alpha_positive_50us"
    SERVO_ALPHA_NEGATIVE = "servo_alpha_negative_50us"
    SERVO_BETA_POSITIVE = "servo_beta_positive_50us"
    SERVO_BETA_NEGATIVE = "servo_beta_negative_50us"
    FAILSAFE = "failsafe"
    MOTOR_ROTATION = "motor_rotation"
    YAW_TORQUE = "yaw_torque"


@dataclass(frozen=True)
class StageDefinition:
    stage: V2Stage
    label: str
    kind: str
    required: bool = True
    expected_mechanical_direction: str | None = None


_STAGE_DEFINITIONS = {
    V2Stage.RC_CENTER: StageDefinition(V2Stage.RC_CENTER, "RC center", "rc_center"),
    V2Stage.RC_POSITIVE_ROLL: StageDefinition(V2Stage.RC_POSITIVE_ROLL, "RC +roll (right wing down)", "rc_direction"),
    V2Stage.RC_POSITIVE_PITCH: StageDefinition(V2Stage.RC_POSITIVE_PITCH, "RC +pitch (nose down)", "rc_direction"),
    V2Stage.RC_POSITIVE_YAW: StageDefinition(V2Stage.RC_POSITIVE_YAW, "RC +yaw (nose left)", "rc_direction"),
    V2Stage.NAV_STATIC: StageDefinition(V2Stage.NAV_STATIC, "Navigation static", "nav_static"),
    V2Stage.NAV_FORWARD: StageDefinition(V2Stage.NAV_FORWARD, "Navigation +X forward", "nav_direction"),
    V2Stage.NAV_LEFT: StageDefinition(V2Stage.NAV_LEFT, "Navigation +Y left", "nav_direction"),
    V2Stage.RESTORE_POSITIVE_ROLL: StageDefinition(V2Stage.RESTORE_POSITIVE_ROLL, "Controller restoring response from +roll", "restoring"),
    V2Stage.RESTORE_POSITIVE_PITCH: StageDefinition(V2Stage.RESTORE_POSITIVE_PITCH, "Controller restoring response from +pitch", "restoring"),
    V2Stage.SERVO_ALPHA_POSITIVE: StageDefinition(V2Stage.SERVO_ALPHA_POSITIVE, "Servo alpha +50 us", "servo", expected_mechanical_direction="tilt_vector_toward_airframe_left_mark"),
    V2Stage.SERVO_ALPHA_NEGATIVE: StageDefinition(V2Stage.SERVO_ALPHA_NEGATIVE, "Servo alpha -50 us", "servo", expected_mechanical_direction="tilt_vector_toward_airframe_right_mark"),
    V2Stage.SERVO_BETA_POSITIVE: StageDefinition(V2Stage.SERVO_BETA_POSITIVE, "Servo beta +50 us", "servo", expected_mechanical_direction="tilt_vector_toward_airframe_front_mark"),
    V2Stage.SERVO_BETA_NEGATIVE: StageDefinition(V2Stage.SERVO_BETA_NEGATIVE, "Servo beta -50 us", "servo", expected_mechanical_direction="tilt_vector_toward_airframe_rear_mark"),
    V2Stage.FAILSAFE: StageDefinition(V2Stage.FAILSAFE, "Link failsafe", "failsafe"),
    V2Stage.MOTOR_ROTATION: StageDefinition(V2Stage.MOTOR_ROTATION, "Powered motor rotation", "unsupported", required=False),
    V2Stage.YAW_TORQUE: StageDefinition(V2Stage.YAW_TORQUE, "Powered yaw torque", "unsupported", required=False),
}
STAGE_DEFINITIONS: Mapping[V2Stage, StageDefinition] = MappingProxyType(_STAGE_DEFINITIONS)
REQUIRED_STAGES = tuple(stage for stage, item in STAGE_DEFINITIONS.items() if item.required)
UNSUPPORTED_STAGES = (V2Stage.MOTOR_ROTATION, V2Stage.YAW_TORQUE)
SERVO_STAGES = tuple(stage for stage, item in STAGE_DEFINITIONS.items() if item.kind == "servo")


def _finite(value: Any, *, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _optional_finite(value: Any, *, name: str) -> float | None:
    return None if value is None else _finite(value, name=name)


def _strict_int(value: Any, *, name: str, minimum: int = 0) -> int:
    if type(value) is not int:
        raise TypeError(f"{name} must be an integer")
    if value < minimum:
        raise ValueError(f"{name} must be >= {minimum}")
    return value


def _strict_bool(value: Any, *, name: str) -> bool:
    if type(value) is not bool:
        raise TypeError(f"{name} must be a boolean")
    return value


def _nonempty(value: Any, *, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise TypeError(f"{name} must be a non-empty string")
    return value.strip()


def _enum_value(enum_type: type[Enum], value: Any, *, name: str) -> Any:
    if isinstance(value, enum_type):
        return value
    if not isinstance(value, str):
        raise TypeError(f"{name} must be {enum_type.__name__} or string")
    try:
        return enum_type(value)
    except ValueError as exc:
        raise ValueError(f"unknown {name}: {value!r}") from exc


def _primitive(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return {item.name: _primitive(getattr(value, item.name)) for item in fields(value)}
    if isinstance(value, Mapping):
        return {str(key): _primitive(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_primitive(item) for item in value]
    return value


def _hash(value: Any) -> str:
    encoded = json.dumps(_primitive(value), sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class V2Thresholds:
    rc_center_min_us: float = 1450.0
    rc_center_max_us: float = 1550.0
    rc_center_fraction_min: float = 0.90
    rc_target_delta_min_us: float = 200.0
    rc_sign_fraction_min: float = 0.90
    rc_crosstalk_max_us: float = 100.0
    nav_static_max_mps: float = 0.05
    nav_main_min_mps: float = 0.05
    nav_positive_fraction_min: float = 0.90
    nav_cross_median_ratio_max: float = 0.50
    nav_cross_rms_ratio_max: float = 0.75
    nav_cross_p95_ratio_max: float = 0.75
    restoring_angle_min_deg: float = 8.0
    restoring_angle_max_deg: float = 20.0
    restoring_rate_min_dps: float = 1.0
    restoring_moment_min: float = 0.05
    damping_moment_min: float = 0.02
    restoring_sign_fraction_min: float = 0.90
    servo_target_delta_us: float = 50.0
    servo_target_tolerance_us: float = 5.0
    servo_command_fraction_min: float = 0.90
    servo_feedback_valid_fraction_min: float = 0.95
    servo_feedback_error_max_us: float = 20.0
    failsafe_latency_max_ms: float = 600.0
    failsafe_post_transition_min_samples: int = 3
    required_stage_min_samples: int = 10
    required_stage_min_duration_s: float = 0.10

    def __post_init__(self) -> None:
        for item in fields(self):
            value = getattr(self, item.name)
            if item.name in {"failsafe_post_transition_min_samples", "required_stage_min_samples"}:
                _strict_int(value, name=item.name, minimum=1)
            else:
                object.__setattr__(self, item.name, _finite(value, name=item.name))
        if not self.rc_center_min_us < self.rc_center_max_us:
            raise ValueError("rc center min must be less than max")
        if not 0.0 < self.restoring_angle_min_deg < self.restoring_angle_max_deg:
            raise ValueError("restoring angle bounds are invalid")
        fractions = (
            "rc_center_fraction_min", "rc_sign_fraction_min", "nav_positive_fraction_min",
            "restoring_sign_fraction_min", "servo_command_fraction_min",
            "servo_feedback_valid_fraction_min",
        )
        if any(not 0.0 <= getattr(self, name) <= 1.0 for name in fractions):
            raise ValueError("fraction thresholds must be in 0..1")
        safe_fraction_floors = {
            "rc_center_fraction_min": 0.90,
            "rc_sign_fraction_min": 0.90,
            "nav_positive_fraction_min": 0.90,
            "restoring_sign_fraction_min": 0.90,
            "servo_command_fraction_min": 0.90,
            "servo_feedback_valid_fraction_min": 0.95,
        }
        if any(getattr(self, name) < floor for name, floor in safe_fraction_floors.items()):
            raise ValueError("acceptance consistency fractions cannot be relaxed below safety floors")
        positive = (
            "rc_target_delta_min_us", "rc_crosstalk_max_us", "nav_static_max_mps",
            "nav_main_min_mps", "nav_cross_median_ratio_max", "nav_cross_rms_ratio_max",
            "nav_cross_p95_ratio_max", "restoring_rate_min_dps", "restoring_moment_min",
            "damping_moment_min", "servo_target_delta_us", "servo_target_tolerance_us",
            "servo_feedback_error_max_us", "failsafe_latency_max_ms", "required_stage_min_duration_s",
        )
        if any(getattr(self, name) <= 0.0 for name in positive):
            raise ValueError("positive thresholds must be > 0")
        if not (
            self.nav_cross_median_ratio_max
            <= self.nav_cross_rms_ratio_max
            <= self.nav_cross_p95_ratio_max
            <= 1.0
        ):
            raise ValueError("navigation cross-axis ratio thresholds are inconsistent")
        if self.failsafe_latency_max_ms > 600.0:
            raise ValueError("failsafe latency cannot be relaxed beyond 600 ms")
        if self.required_stage_min_samples < 10 or self.required_stage_min_duration_s < 0.10:
            raise ValueError("required-stage evidence floor cannot be relaxed")
        if self.failsafe_post_transition_min_samples < 3:
            raise ValueError("failsafe persistence evidence floor cannot be relaxed")


DEFAULT_THRESHOLDS = V2Thresholds()


@dataclass(frozen=True)
class V2Sample:
    timestamp_s: float
    sequence: int
    stage: V2Stage | str
    frame_id: str
    frame_contract_version: int
    orientation_code: int
    calibration_generation: int
    firmware_id: str
    session_id: str
    capture_source: str
    provenance: str
    v0_record_id: str
    v1_record_id: str
    lease_id: str
    lease_issued_at_s: float
    lease_expires_at_s: float
    v0_persisted: bool
    v1_room_temp_valid: bool
    props_removed: bool
    acceptance_mode: bool
    lease_valid: bool
    esc_enabled: bool
    esc_ccr_1: int
    esc_ccr_2: int
    rc_roll_us: float | None = None
    rc_pitch_us: float | None = None
    rc_yaw_us: float | None = None
    nav_velocity_x_mps: float | None = None
    nav_velocity_y_mps: float | None = None
    roll_angle_deg: float | None = None
    pitch_angle_deg: float | None = None
    roll_rate_dps: float | None = None
    pitch_rate_dps: float | None = None
    restoring_roll_moment: float | None = None
    restoring_pitch_moment: float | None = None
    damping_roll_moment: float | None = None
    damping_pitch_moment: float | None = None
    servo_alpha_center_us: float | None = None
    servo_alpha_command_us: float | None = None
    servo_alpha_feedback_us: float | None = None
    servo_alpha_feedback_valid: bool | None = None
    servo_beta_center_us: float | None = None
    servo_beta_command_us: float | None = None
    servo_beta_feedback_us: float | None = None
    servo_beta_feedback_valid: bool | None = None
    link_present: bool | None = None
    failsafe_elapsed_ms: float | None = None
    failsafe_active: bool | None = None
    control_mode: str | None = None

    def __post_init__(self) -> None:
        timestamp = _finite(self.timestamp_s, name="timestamp_s")
        if timestamp < 0.0:
            raise ValueError("timestamp_s must be nonnegative")
        object.__setattr__(self, "timestamp_s", timestamp)
        object.__setattr__(self, "sequence", _strict_int(self.sequence, name="sequence"))
        stage = _enum_value(V2Stage, self.stage, name="stage")
        object.__setattr__(self, "stage", stage)
        if _nonempty(self.frame_id, name="frame_id") != CANONICAL_FRAME_ID:
            raise ValueError("V2A samples must use canonical FLU")
        object.__setattr__(self, "frame_contract_version", _strict_int(self.frame_contract_version, name="frame_contract_version", minimum=1))
        orientation = _strict_int(self.orientation_code, name="orientation_code")
        if orientation > 23:
            raise ValueError("orientation_code must identify a proper rotation (0..23)")
        object.__setattr__(self, "orientation_code", orientation)
        object.__setattr__(self, "calibration_generation", _strict_int(self.calibration_generation, name="calibration_generation"))
        for name in ("firmware_id", "session_id", "v0_record_id", "v1_record_id", "lease_id"):
            object.__setattr__(self, name, _nonempty(getattr(self, name), name=name))
        if _nonempty(self.capture_source, name="capture_source") != ACCEPTANCE_CAPTURE_SOURCE:
            raise ValueError("capture_source is not the V2 acceptance snapshot source")
        if _nonempty(self.provenance, name="provenance") != ACCEPTANCE_PROVENANCE:
            raise ValueError("provenance is not the V2 acceptance provenance")
        issued = _finite(self.lease_issued_at_s, name="lease_issued_at_s")
        expires = _finite(self.lease_expires_at_s, name="lease_expires_at_s")
        if issued < 0.0 or expires <= issued or not issued <= timestamp <= expires:
            raise ValueError("sample timestamp must lie inside a valid lease interval")
        object.__setattr__(self, "lease_issued_at_s", issued)
        object.__setattr__(self, "lease_expires_at_s", expires)
        for name in ("v0_persisted", "v1_room_temp_valid", "props_removed", "acceptance_mode", "lease_valid", "esc_enabled"):
            _strict_bool(getattr(self, name), name=name)
        object.__setattr__(self, "esc_ccr_1", _strict_int(self.esc_ccr_1, name="esc_ccr_1"))
        object.__setattr__(self, "esc_ccr_2", _strict_int(self.esc_ccr_2, name="esc_ccr_2"))
        numeric = (
            "rc_roll_us", "rc_pitch_us", "rc_yaw_us", "nav_velocity_x_mps", "nav_velocity_y_mps",
            "roll_angle_deg", "pitch_angle_deg", "roll_rate_dps", "pitch_rate_dps",
            "restoring_roll_moment", "restoring_pitch_moment", "damping_roll_moment", "damping_pitch_moment",
            "servo_alpha_center_us", "servo_alpha_command_us", "servo_alpha_feedback_us",
            "servo_beta_center_us", "servo_beta_command_us", "servo_beta_feedback_us", "failsafe_elapsed_ms",
        )
        for name in numeric:
            object.__setattr__(self, name, _optional_finite(getattr(self, name), name=name))
        for name in ("servo_alpha_feedback_valid", "servo_beta_feedback_valid", "link_present", "failsafe_active"):
            if getattr(self, name) is not None:
                _strict_bool(getattr(self, name), name=name)
        if self.control_mode is not None:
            object.__setattr__(self, "control_mode", _nonempty(self.control_mode, name="control_mode"))
        self._validate_payload(stage)

    def _require(self, *names: str) -> None:
        missing = [name for name in names if getattr(self, name) is None]
        if missing:
            raise ValueError(f"{self.stage.value} sample missing fields: {', '.join(missing)}")

    def _validate_payload(self, stage: V2Stage) -> None:
        kind = STAGE_DEFINITIONS[stage].kind
        if kind.startswith("rc_"):
            self._require("rc_roll_us", "rc_pitch_us", "rc_yaw_us")
        elif kind.startswith("nav_"):
            self._require("nav_velocity_x_mps", "nav_velocity_y_mps")
        elif kind == "restoring":
            names = ("roll_angle_deg", "roll_rate_dps", "restoring_roll_moment", "damping_roll_moment") if stage is V2Stage.RESTORE_POSITIVE_ROLL else ("pitch_angle_deg", "pitch_rate_dps", "restoring_pitch_moment", "damping_pitch_moment")
            self._require(*names)
        elif kind == "servo":
            prefix = "servo_alpha" if "alpha" in stage.value else "servo_beta"
            self._require(f"{prefix}_center_us", f"{prefix}_command_us", f"{prefix}_feedback_valid")
            if getattr(self, f"{prefix}_feedback_valid"):
                self._require(f"{prefix}_feedback_us")
        elif kind == "failsafe":
            self._require("link_present", "failsafe_elapsed_ms", "failsafe_active", "control_mode")
            if float(self.failsafe_elapsed_ms) < 0.0:
                raise ValueError("failsafe_elapsed_ms must be nonnegative")


@dataclass(frozen=True)
class PhysicalConfirmation:
    stage: V2Stage | str
    confirmed: bool
    observed_direction: str
    note: str
    operator_id: str
    confirmed_at_s: float
    session_id: str
    firmware_id: str
    calibration_generation: int
    lease_id: str
    props_removed: bool

    def __post_init__(self) -> None:
        stage = _enum_value(V2Stage, self.stage, name="stage")
        if stage not in SERVO_STAGES:
            raise ValueError("physical confirmation is valid only for servo stages")
        object.__setattr__(self, "stage", stage)
        _strict_bool(self.confirmed, name="confirmed")
        object.__setattr__(self, "observed_direction", _nonempty(self.observed_direction, name="observed_direction"))
        object.__setattr__(self, "note", _nonempty(self.note, name="note"))
        object.__setattr__(self, "operator_id", _nonempty(self.operator_id, name="operator_id"))
        confirmed_at = _finite(self.confirmed_at_s, name="confirmed_at_s")
        if confirmed_at < 0.0:
            raise ValueError("confirmed_at_s must be nonnegative")
        object.__setattr__(self, "confirmed_at_s", confirmed_at)
        for name in ("session_id", "firmware_id", "lease_id"):
            object.__setattr__(self, name, _nonempty(getattr(self, name), name=name))
        object.__setattr__(self, "calibration_generation", _strict_int(self.calibration_generation, name="calibration_generation"))
        _strict_bool(self.props_removed, name="props_removed")


MechanicalConfirmation = PhysicalConfirmation


@dataclass(frozen=True)
class Metric:
    name: str
    value: float
    unit: str
    def __post_init__(self) -> None:
        object.__setattr__(self, "name", _nonempty(self.name, name="metric.name"))
        object.__setattr__(self, "value", _finite(self.value, name=f"metric.{self.name}"))
        object.__setattr__(self, "unit", _nonempty(self.unit, name="metric.unit"))


@dataclass(frozen=True)
class GlobalGateResult:
    status: AcceptanceStatus | str
    sample_count: int
    reasons: tuple[str, ...]
    def __post_init__(self) -> None:
        object.__setattr__(self, "status", _enum_value(AcceptanceStatus, self.status, name="status"))
        object.__setattr__(self, "sample_count", _strict_int(self.sample_count, name="sample_count"))
        object.__setattr__(self, "reasons", tuple(_nonempty(v, name="reason") for v in self.reasons))


@dataclass(frozen=True)
class EvidenceContext:
    frame_id: str
    frame_contract_version: int
    orientation_code: int
    calibration_generation: int
    firmware_id: str
    session_id: str
    capture_source: str
    provenance: str
    v0_record_id: str
    v1_record_id: str
    lease_id: str
    lease_issued_at_s: float
    lease_expires_at_s: float
    servo_type: ServoType | str = ServoType.BUS

    def __post_init__(self) -> None:
        if self.frame_id != CANONICAL_FRAME_ID:
            raise ValueError("V2A context must be canonical FLU")
        _strict_int(self.frame_contract_version, name="frame_contract_version", minimum=1)
        orientation = _strict_int(self.orientation_code, name="orientation_code")
        if orientation > 23:
            raise ValueError("orientation_code must be in 0..23")
        _strict_int(self.calibration_generation, name="calibration_generation")
        for name in ("firmware_id", "session_id", "v0_record_id", "v1_record_id", "lease_id"):
            _nonempty(getattr(self, name), name=name)
        if self.capture_source != ACCEPTANCE_CAPTURE_SOURCE or self.provenance != ACCEPTANCE_PROVENANCE:
            raise ValueError("context provenance is not exact V2 acceptance provenance")
        object.__setattr__(self, "servo_type", _enum_value(ServoType, self.servo_type, name="servo_type"))
        issued = _finite(self.lease_issued_at_s, name="lease_issued_at_s")
        expires = _finite(self.lease_expires_at_s, name="lease_expires_at_s")
        if issued < 0.0 or expires <= issued:
            raise ValueError("context lease interval is invalid")


@dataclass(frozen=True)
class StageResult:
    stage: V2Stage | str
    status: AcceptanceStatus | str
    sample_count: int
    metrics: tuple[Metric, ...]
    findings: tuple[str, ...]
    def __post_init__(self) -> None:
        object.__setattr__(self, "stage", _enum_value(V2Stage, self.stage, name="stage"))
        object.__setattr__(self, "status", _enum_value(AcceptanceStatus, self.status, name="status"))
        object.__setattr__(self, "sample_count", _strict_int(self.sample_count, name="sample_count"))
        object.__setattr__(self, "metrics", tuple(self.metrics))
        object.__setattr__(self, "findings", tuple(_nonempty(v, name="finding") for v in self.findings))
        names = [metric.name for metric in self.metrics]
        if len(names) != len(set(names)):
            raise ValueError("stage metric names must be unique")


@dataclass(frozen=True)
class V2Report:
    created_at: str
    data_source: str
    status: AcceptanceStatus | str
    thresholds: V2Thresholds
    context: EvidenceContext
    global_gate: GlobalGateResult
    sample_count: int
    stages: tuple[StageResult, ...]
    physical_confirmations: tuple[PhysicalConfirmation, ...]
    findings: tuple[str, ...]
    evidence_sha256: str
    integrity_sha256: str = "AUTO"
    format: str = field(default=V2_REPORT_FORMAT, init=False)
    schema: int = field(default=V2_REPORT_SCHEMA, init=False)
    evidence_only: bool = field(default=True, init=False)
    applied: bool = field(default=False, init=False)
    auto_apply: bool = field(default=False, init=False)
    parameter_changes_applied: bool = field(default=False, init=False)
    parameters_written: bool = field(default=False, init=False)
    flash_writes: int = field(default=0, init=False)
    flight_release: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        try:
            datetime.fromisoformat(self.created_at.replace("Z", "+00:00"))
        except (AttributeError, ValueError) as exc:
            raise ValueError("created_at must be an ISO-8601 timestamp") from exc
        object.__setattr__(self, "data_source", _nonempty(self.data_source, name="data_source"))
        status = _enum_value(AcceptanceStatus, self.status, name="status")
        object.__setattr__(self, "status", status)
        if not isinstance(self.thresholds, V2Thresholds):
            raise TypeError("thresholds must be V2Thresholds")
        object.__setattr__(self, "sample_count", _strict_int(self.sample_count, name="sample_count"))
        object.__setattr__(self, "stages", tuple(self.stages))
        object.__setattr__(self, "physical_confirmations", tuple(self.physical_confirmations))
        object.__setattr__(self, "findings", tuple(_nonempty(v, name="finding") for v in self.findings))
        evidence_hash = _nonempty(self.evidence_sha256, name="evidence_sha256")
        if len(evidence_hash) != 64 or any(char not in "0123456789abcdef" for char in evidence_hash):
            raise ValueError("evidence_sha256 must be lowercase SHA-256 hex")
        if tuple(item.stage for item in self.stages) != tuple(STAGE_DEFINITIONS):
            raise ValueError("report must contain every V2A stage exactly once in canonical order")
        if self.global_gate.sample_count != self.sample_count or sum(item.sample_count for item in self.stages) != self.sample_count:
            raise ValueError("report sample counts are inconsistent")
        if self.global_gate.status is AcceptanceStatus.PASS and (self.sample_count == 0 or self.global_gate.reasons):
            raise ValueError("passing global gate requires nonzero evidence and no reasons")
        for stage in UNSUPPORTED_STAGES:
            if self.stage_result(stage).status is not AcceptanceStatus.UNSUPPORTED:
                raise ValueError(f"{stage.value} must remain explicitly UNSUPPORTED")
        if self.context.servo_type is ServoType.PWM:
            for stage in SERVO_STAGES:
                result = self.stage_result(stage)
                if result.status is not AcceptanceStatus.SKIPPED:
                    raise ValueError(f"PWM report must explicitly SKIP {stage.value}")
                if not any("SKIPPED" in finding.upper() and "PWM" in finding.upper() for finding in result.findings):
                    raise ValueError(f"PWM {stage.value} skip reason must name PWM")
        else:
            for stage in SERVO_STAGES:
                if self.stage_result(stage).status is AcceptanceStatus.SKIPPED:
                    raise ValueError(f"bus report cannot skip {stage.value}")
        confirmation_by_stage = {item.stage: item for item in self.physical_confirmations}
        if len(confirmation_by_stage) != len(self.physical_confirmations):
            raise ValueError("physical confirmations must have unique stages")
        for stage in SERVO_STAGES:
            confirmation = confirmation_by_stage.get(stage)
            if self.stage_result(stage).status is AcceptanceStatus.PASS and not _confirmation_matches(confirmation, self.context, stage):
                raise ValueError(f"passing {stage.value} requires bound physical confirmation")
        for stage in REQUIRED_STAGES:
            result = self.stage_result(stage)
            if result.status is AcceptanceStatus.PASS:
                if result.sample_count < self.thresholds.required_stage_min_samples:
                    raise ValueError(f"passing {stage.value} has too few samples")
                duration = next((metric.value for metric in result.metrics if metric.name == "duration_s"), None)
                if duration is None or duration < self.thresholds.required_stage_min_duration_s:
                    raise ValueError(f"passing {stage.value} has insufficient duration")
        expected = AcceptanceStatus.PASS
        required_ok = all(
            self.stage_result(stage).status is AcceptanceStatus.PASS
            or (self.context.servo_type is ServoType.PWM and stage in SERVO_STAGES and self.stage_result(stage).status is AcceptanceStatus.SKIPPED)
            for stage in REQUIRED_STAGES
        )
        if self.global_gate.status is not AcceptanceStatus.PASS or not required_ok:
            expected = AcceptanceStatus.FAIL
        if status is not expected:
            raise ValueError(f"report status must be {expected.value}")
        integrity_data = {item.name: _primitive(getattr(self, item.name)) for item in fields(self) if item.name != "integrity_sha256"}
        expected_hash = _hash(integrity_data)
        if self.integrity_sha256 == "AUTO":
            object.__setattr__(self, "integrity_sha256", expected_hash)
        elif self.integrity_sha256 != expected_hash:
            raise ValueError("report integrity_sha256 mismatch")

    @property
    def mechanical_confirmations(self) -> tuple[PhysicalConfirmation, ...]:
        return self.physical_confirmations

    def stage_result(self, stage: V2Stage | str) -> StageResult:
        selected = _enum_value(V2Stage, stage, name="stage")
        return next(item for item in self.stages if item.stage is selected)


def _median(values: Sequence[float]) -> float:
    return float(statistics.median(values))


def _fraction(values: Sequence[bool]) -> float:
    return sum(bool(value) for value in values) / len(values) if values else 0.0


def _p95(values: Sequence[float]) -> float:
    ordered = sorted(values)
    return ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]


def _duration(samples: Sequence[V2Sample]) -> float:
    return float(samples[-1].timestamp_s - samples[0].timestamp_s) if len(samples) > 1 else 0.0


def _sample_gate_reasons(sample: V2Sample) -> tuple[str, ...]:
    reasons = [name for name in ("v0_persisted", "v1_room_temp_valid", "props_removed", "acceptance_mode", "lease_valid") if not getattr(sample, name)]
    if sample.esc_enabled:
        reasons.append("esc_enabled")
    if sample.esc_ccr_1 != 0 or sample.esc_ccr_2 != 0:
        reasons.append("esc_ccr_nonzero")
    return tuple(reasons)


def _confirmation_matches(
    item: PhysicalConfirmation | None,
    context: EvidenceContext,
    stage: V2Stage,
    stage_samples: Sequence[V2Sample] | None = None,
) -> bool:
    matched = bool(
        item is not None and item.confirmed and item.props_removed
        and item.observed_direction == STAGE_DEFINITIONS[stage].expected_mechanical_direction
        and item.session_id == context.session_id and item.firmware_id == context.firmware_id
        and item.calibration_generation == context.calibration_generation and item.lease_id == context.lease_id
        and context.lease_issued_at_s <= item.confirmed_at_s <= context.lease_expires_at_s
    )
    if matched and stage_samples:
        matched = stage_samples[0].timestamp_s <= item.confirmed_at_s <= stage_samples[-1].timestamp_s
    return matched


def evaluate_global_gate(
    samples: Sequence[V2Sample],
    physical_confirmations: Sequence[PhysicalConfirmation] = (),
    *,
    servo_type: ServoType | str = ServoType.BUS,
) -> GlobalGateResult:
    evidence = tuple(samples)
    if not evidence:
        return GlobalGateResult(AcceptanceStatus.FAIL, 0, ("no samples",))
    servo_mode = _enum_value(ServoType, servo_type, name="servo_type")
    reasons = {reason for sample in evidence for reason in _sample_gate_reasons(sample)}
    context = _uniform_context(evidence)
    by_stage = {item.stage: item for item in physical_confirmations}
    grouped = {stage: tuple(sample for sample in evidence if sample.stage is stage) for stage in SERVO_STAGES}
    if servo_mode is ServoType.BUS and any(not _confirmation_matches(by_stage.get(stage), context, stage, grouped[stage]) for stage in SERVO_STAGES):
        reasons.add("physical_props_or_servo_confirmation")
    return GlobalGateResult(AcceptanceStatus.FAIL if reasons else AcceptanceStatus.PASS, len(evidence), tuple(sorted(reasons)))


def _stage_precheck(stage: V2Stage, samples: Sequence[V2Sample], thresholds: V2Thresholds) -> StageResult | None:
    if not samples:
        return StageResult(stage, AcceptanceStatus.NOT_RUN, 0, (), ("no evidence samples",))
    reasons = sorted({reason for sample in samples for reason in _sample_gate_reasons(sample)})
    if reasons:
        return StageResult(stage, AcceptanceStatus.FAIL, len(samples), (), ("hard global gate failed: " + ", ".join(reasons),))
    duration = _duration(samples)
    if len(samples) < thresholds.required_stage_min_samples or duration < thresholds.required_stage_min_duration_s:
        return StageResult(stage, AcceptanceStatus.FAIL, len(samples), (Metric("duration_s", duration, "s"),), ("insufficient sample count or duration",))
    return None


def _with_duration(samples: Sequence[V2Sample], metrics: Sequence[Metric]) -> tuple[Metric, ...]:
    return (Metric("duration_s", _duration(samples), "s"), *metrics)


_RC_MAIN_FIELD = {V2Stage.RC_POSITIVE_ROLL: "rc_roll_us", V2Stage.RC_POSITIVE_PITCH: "rc_pitch_us", V2Stage.RC_POSITIVE_YAW: "rc_yaw_us"}


def _analyze_rc_center(samples: Sequence[V2Sample], t: V2Thresholds) -> StageResult:
    stage = V2Stage.RC_CENTER
    blocked = _stage_precheck(stage, samples, t)
    if blocked:
        return blocked
    fields_ = ("rc_roll_us", "rc_pitch_us", "rc_yaw_us")
    medians = [_median([float(getattr(s, name)) for s in samples]) for name in fields_]
    fractions = [_fraction([t.rc_center_min_us <= float(getattr(s, name)) <= t.rc_center_max_us for s in samples]) for name in fields_]
    passed = all(t.rc_center_min_us <= value <= t.rc_center_max_us for value in medians) and min(fractions) >= t.rc_center_fraction_min
    metrics = [Metric(f"{name.split('_')[1]}_center", value, "us") for name, value in zip(fields_, medians)] + [Metric("center_in_range_fraction_min", min(fractions), "fraction")]
    return StageResult(stage, AcceptanceStatus.PASS if passed else AcceptanceStatus.FAIL, len(samples), _with_duration(samples, metrics), ("RC center distribution passes" if passed else "RC center distribution failed",))


def _analyze_rc_direction(stage: V2Stage, samples: Sequence[V2Sample], centers: Sequence[V2Sample], t: V2Thresholds) -> StageResult:
    blocked = _stage_precheck(stage, samples, t)
    if blocked:
        return blocked
    if not centers:
        return StageResult(stage, AcceptanceStatus.FAIL, len(samples), (), ("RC center reference is missing",))
    names = ("rc_roll_us", "rc_pitch_us", "rc_yaw_us")
    center = {name: _median([float(getattr(s, name)) for s in centers]) for name in names}
    main_name = _RC_MAIN_FIELD[stage]
    deltas = [float(getattr(s, main_name)) - center[main_name] for s in samples]
    main_delta = _median(deltas)
    sign_fraction = _fraction([value > 0.0 for value in deltas])
    crosstalk = [max(abs(float(getattr(s, name)) - center[name]) for name in names if name != main_name) for s in samples]
    cross_p95 = _p95(crosstalk)
    passed = main_delta >= t.rc_target_delta_min_us and sign_fraction >= t.rc_sign_fraction_min and cross_p95 <= t.rc_crosstalk_max_us
    metrics = (Metric("main_delta", main_delta, "us"), Metric("positive_sign_fraction", sign_fraction, "fraction"), Metric("crosstalk_p95", cross_p95, "us"))
    return StageResult(stage, AcceptanceStatus.PASS if passed else AcceptanceStatus.FAIL, len(samples), _with_duration(samples, metrics), ("positive FLU RC intent distribution passes" if passed else "RC intent magnitude, sign consistency, or crosstalk failed",))


def _analyze_nav_static(samples: Sequence[V2Sample], t: V2Thresholds) -> StageResult:
    stage = V2Stage.NAV_STATIC
    blocked = _stage_precheck(stage, samples, t)
    if blocked:
        return blocked
    x = _median([abs(float(s.nav_velocity_x_mps)) for s in samples]); y = _median([abs(float(s.nav_velocity_y_mps)) for s in samples])
    passed = x <= t.nav_static_max_mps and y <= t.nav_static_max_mps
    return StageResult(stage, AcceptanceStatus.PASS if passed else AcceptanceStatus.FAIL, len(samples), _with_duration(samples, (Metric("median_abs_x", x, "m/s"), Metric("median_abs_y", y, "m/s"))), ("static navigation residual passes" if passed else "static navigation residual failed",))


def _analyze_nav_direction(stage: V2Stage, samples: Sequence[V2Sample], t: V2Thresholds) -> StageResult:
    blocked = _stage_precheck(stage, samples, t)
    if blocked:
        return blocked
    main_name, cross_name = (("nav_velocity_x_mps", "nav_velocity_y_mps") if stage is V2Stage.NAV_FORWARD else ("nav_velocity_y_mps", "nav_velocity_x_mps"))
    main_values = [float(getattr(s, main_name)) for s in samples]; cross_abs = [abs(float(getattr(s, cross_name))) for s in samples]
    main = _median(main_values); sign_fraction = _fraction([value > 0.0 for value in main_values]); denominator = abs(main)
    median_ratio = _median(cross_abs) / denominator if denominator else math.inf
    rms_ratio = math.sqrt(sum(value * value for value in cross_abs) / len(cross_abs)) / denominator if denominator else math.inf
    p95_ratio = _p95(cross_abs) / denominator if denominator else math.inf
    passed = main >= t.nav_main_min_mps and sign_fraction >= t.nav_positive_fraction_min and median_ratio <= t.nav_cross_median_ratio_max and rms_ratio <= t.nav_cross_rms_ratio_max and p95_ratio <= t.nav_cross_p95_ratio_max
    values = [main, sign_fraction, median_ratio, rms_ratio, p95_ratio]
    values = [value if math.isfinite(value) else 1e30 for value in values]
    metrics = tuple(Metric(name, value, unit) for name, value, unit in zip(("main_median", "positive_sign_fraction", "cross_median_abs_ratio", "cross_rms_ratio", "cross_p95_ratio"), values, ("m/s", "fraction", "ratio", "ratio", "ratio")))
    return StageResult(stage, AcceptanceStatus.PASS if passed else AcceptanceStatus.FAIL, len(samples), _with_duration(samples, metrics), ("canonical navigation distribution passes" if passed else "navigation magnitude, sign, or cross-axis distribution failed",))


def _analyze_restoring(stage: V2Stage, samples: Sequence[V2Sample], t: V2Thresholds) -> StageResult:
    blocked = _stage_precheck(stage, samples, t)
    if blocked:
        return blocked
    if stage is V2Stage.RESTORE_POSITIVE_ROLL:
        angle_name, rate_name, restoring_name, damping_name = "roll_angle_deg", "roll_rate_dps", "restoring_roll_moment", "damping_roll_moment"
    else:
        angle_name, rate_name, restoring_name, damping_name = "pitch_angle_deg", "pitch_rate_dps", "restoring_pitch_moment", "damping_pitch_moment"
    angles = [float(getattr(s, angle_name)) for s in samples]
    angle_fraction = _fraction([t.restoring_angle_min_deg <= value <= t.restoring_angle_max_deg for value in angles])
    rate_fraction = _fraction([abs(float(getattr(s, rate_name))) >= t.restoring_rate_min_dps for s in samples])
    restoring_sign = _fraction([float(getattr(s, restoring_name)) * float(getattr(s, angle_name)) < 0.0 for s in samples])
    damping_sign = _fraction([float(getattr(s, damping_name)) * float(getattr(s, rate_name)) < 0.0 for s in samples])
    restoring_mag = _fraction([abs(float(getattr(s, restoring_name))) >= t.restoring_moment_min for s in samples])
    damping_mag = _fraction([abs(float(getattr(s, damping_name))) >= t.damping_moment_min for s in samples])
    fractions = (angle_fraction, rate_fraction, restoring_sign, damping_sign, restoring_mag, damping_mag)
    passed = min(fractions) >= t.restoring_sign_fraction_min
    metrics = tuple(Metric(name, value, "fraction") for name, value in zip(("angle_in_range_fraction", "rate_magnitude_fraction", "restoring_sign_fraction", "damping_sign_fraction", "restoring_magnitude_fraction", "damping_magnitude_fraction"), fractions))
    return StageResult(stage, AcceptanceStatus.PASS if passed else AcceptanceStatus.FAIL, len(samples), _with_duration(samples, metrics), ("restoring sign and magnitude distributions pass" if passed else "restoring angle/rate/sign/magnitude failed",))


def _analyze_servo(
    stage: V2Stage,
    samples: Sequence[V2Sample],
    confirmation: PhysicalConfirmation | None,
    context: EvidenceContext,
    t: V2Thresholds,
) -> StageResult:
    if context.servo_type is ServoType.PWM:
        return StageResult(
            stage,
            AcceptanceStatus.SKIPPED,
            len(samples),
            (),
            ("SKIPPED: PWM servo type has no bus feedback; servo mechanical acceptance is not applicable",),
        )
    blocked = _stage_precheck(stage, samples, t)
    if blocked:
        return blocked
    prefix = "servo_alpha" if "alpha" in stage.value else "servo_beta"; expected_sign = 1.0 if "positive" in stage.value else -1.0
    deltas = [float(getattr(s, f"{prefix}_command_us")) - float(getattr(s, f"{prefix}_center_us")) for s in samples]
    delta = _median(deltas)
    sign_fraction = _fraction([expected_sign * value > 0.0 for value in deltas])
    target_fraction = _fraction([abs(value - expected_sign * t.servo_target_delta_us) <= t.servo_target_tolerance_us for value in deltas])
    valid = [s for s in samples if getattr(s, f"{prefix}_feedback_valid") is True]; valid_fraction = len(valid) / len(samples)
    errors = [abs(float(getattr(s, f"{prefix}_feedback_us")) - float(getattr(s, f"{prefix}_command_us"))) for s in valid]
    max_error = max(errors) if errors else 1e30
    mechanical_ok = _confirmation_matches(confirmation, context, stage, samples)
    passed = sign_fraction >= t.servo_command_fraction_min and target_fraction >= t.servo_command_fraction_min and valid_fraction >= t.servo_feedback_valid_fraction_min and max_error <= t.servo_feedback_error_max_us and mechanical_ok
    metrics = (Metric("command_delta", delta, "us"), Metric("command_sign_fraction", sign_fraction, "fraction"), Metric("command_target_fraction", target_fraction, "fraction"), Metric("feedback_valid_fraction", valid_fraction, "fraction"), Metric("feedback_max_error", max_error, "us"), Metric("physical_confirmed", 1.0 if mechanical_ok else 0.0, "boolean"))
    findings = ["servo command/feedback/physical direction pass" if passed else "servo acceptance failed"]
    if not mechanical_ok:
        findings.append("bound props-removed physical confirmation with concrete airframe direction is required")
    return StageResult(stage, AcceptanceStatus.PASS if passed else AcceptanceStatus.FAIL, len(samples), _with_duration(samples, metrics), tuple(findings))


def _analyze_failsafe(samples: Sequence[V2Sample], t: V2Thresholds) -> StageResult:
    stage = V2Stage.FAILSAFE
    blocked = _stage_precheck(stage, samples, t)
    if blocked:
        return blocked
    loss_indices = [index for index, sample in enumerate(samples) if sample.link_present is False]
    baseline_ok = bool(loss_indices) and loss_indices[0] > 0 and all(sample.link_present is True and sample.failsafe_active is False for sample in samples[:loss_indices[0]])
    no_recovery = bool(loss_indices) and all(sample.link_present is False for sample in samples[loss_indices[0]:])
    post_loss = list(samples[loss_indices[0]:]) if loss_indices else []
    elapsed = [float(sample.failsafe_elapsed_ms) for sample in post_loss]
    elapsed_monotonic = all(a <= b for a, b in zip(elapsed, elapsed[1:]))
    transition = next((index for index, sample in enumerate(post_loss) if sample.failsafe_active is True and str(sample.control_mode).lower() == SAFE_FAILSAFE_MODE), None)
    latency = float(post_loss[transition].failsafe_elapsed_ms) if transition is not None else 1e30
    after = post_loss[transition:] if transition is not None else []
    persistent = len(after) >= t.failsafe_post_transition_min_samples and all(sample.failsafe_active is True and str(sample.control_mode).lower() == SAFE_FAILSAFE_MODE and not sample.esc_enabled and sample.esc_ccr_1 == 0 and sample.esc_ccr_2 == 0 for sample in after)
    passed = baseline_ok and no_recovery and elapsed_monotonic and transition is not None and latency <= t.failsafe_latency_max_ms and persistent
    metrics = (Metric("safe_mode_latency", latency, "ms"), Metric("baseline_present", 1.0 if baseline_ok else 0.0, "boolean"), Metric("elapsed_monotonic", 1.0 if elapsed_monotonic else 0.0, "boolean"), Metric("post_transition_safe_samples", float(len(after)), "count"))
    return StageResult(stage, AcceptanceStatus.PASS if passed else AcceptanceStatus.FAIL, len(samples), _with_duration(samples, metrics), ("failsafe chronology and persistent safe state pass" if passed else "failsafe baseline/chronology/latency/persistence failed",))


def _validate_evidence_sequence(samples: Sequence[V2Sample]) -> None:
    for previous, current in zip(samples, samples[1:]):
        if current.sequence <= previous.sequence or current.timestamp_s <= previous.timestamp_s:
            raise ValueError("V2A sequence and timestamp must be strictly increasing")


def _uniform_context(samples: Sequence[V2Sample]) -> EvidenceContext:
    if not samples:
        raise ValueError("at least one V2A sample is required")
    keys = {(s.frame_id, s.frame_contract_version, s.orientation_code, s.calibration_generation, s.firmware_id, s.session_id, s.capture_source, s.provenance, s.v0_record_id, s.v1_record_id, s.lease_id, s.lease_issued_at_s, s.lease_expires_at_s) for s in samples}
    if len(keys) != 1:
        raise ValueError("V2A evidence mixes frame, firmware, session, capture, calibration, provenance, or lease context")
    return EvidenceContext(*next(iter(keys)))


def build_v2_report(
    samples: Sequence[V2Sample],
    *,
    physical_confirmations: Sequence[PhysicalConfirmation] = (),
    mechanical_confirmations: Sequence[PhysicalConfirmation] | None = None,
    thresholds: V2Thresholds = DEFAULT_THRESHOLDS,
    created_at: str | None = None,
    data_source: str = "v2a_host_evidence",
    servo_type: ServoType | str = ServoType.BUS,
) -> V2Report:
    evidence = tuple(samples)
    if any(not isinstance(sample, V2Sample) for sample in evidence):
        raise TypeError("samples must contain only V2Sample")
    if not isinstance(thresholds, V2Thresholds):
        raise TypeError("thresholds must be V2Thresholds")
    servo_mode = _enum_value(ServoType, servo_type, name="servo_type")
    _validate_evidence_sequence(evidence)
    context = _uniform_context(evidence)
    context = EvidenceContext(
        context.frame_id, context.frame_contract_version, context.orientation_code,
        context.calibration_generation, context.firmware_id, context.session_id,
        context.capture_source, context.provenance, context.v0_record_id,
        context.v1_record_id, context.lease_id, context.lease_issued_at_s,
        context.lease_expires_at_s, servo_mode,
    )
    if mechanical_confirmations is not None:
        if physical_confirmations:
            raise ValueError("use only physical_confirmations")
        physical_confirmations = mechanical_confirmations
    confirmations = tuple(physical_confirmations)
    if any(not isinstance(item, PhysicalConfirmation) for item in confirmations):
        raise TypeError("physical_confirmations must contain PhysicalConfirmation")
    by_stage: dict[V2Stage, PhysicalConfirmation] = {}
    for item in confirmations:
        if item.stage in by_stage:
            raise ValueError(f"duplicate physical confirmation: {item.stage.value}")
        by_stage[item.stage] = item
    grouped = {stage: tuple(sample for sample in evidence if sample.stage is stage) for stage in STAGE_DEFINITIONS}
    results: list[StageResult] = []
    for stage, definition in STAGE_DEFINITIONS.items():
        stage_samples = grouped[stage]
        if definition.kind == "rc_center": result = _analyze_rc_center(stage_samples, thresholds)
        elif definition.kind == "rc_direction": result = _analyze_rc_direction(stage, stage_samples, grouped[V2Stage.RC_CENTER], thresholds)
        elif definition.kind == "nav_static": result = _analyze_nav_static(stage_samples, thresholds)
        elif definition.kind == "nav_direction": result = _analyze_nav_direction(stage, stage_samples, thresholds)
        elif definition.kind == "restoring": result = _analyze_restoring(stage, stage_samples, thresholds)
        elif definition.kind == "servo": result = _analyze_servo(stage, stage_samples, by_stage.get(stage), context, thresholds)
        elif definition.kind == "failsafe": result = _analyze_failsafe(stage_samples, thresholds)
        else: result = StageResult(stage, AcceptanceStatus.UNSUPPORTED, len(stage_samples), (), ("requires powered physical equipment; V2A cannot validate it",))
        results.append(result)
    gate = evaluate_global_gate(evidence, confirmations, servo_type=servo_mode)
    status = AcceptanceStatus.PASS if gate.status is AcceptanceStatus.PASS and all(
        next(item for item in results if item.stage is stage).status is AcceptanceStatus.PASS
        or (servo_mode is ServoType.PWM and stage in SERVO_STAGES and next(item for item in results if item.stage is stage).status is AcceptanceStatus.SKIPPED)
        for stage in REQUIRED_STAGES
    ) else AcceptanceStatus.FAIL
    evidence_hash = _hash({"samples": evidence, "physical_confirmations": confirmations})
    return V2Report(
        created_at=created_at or datetime.now().astimezone().isoformat(), data_source=data_source,
        status=status, thresholds=thresholds, context=context, global_gate=gate, sample_count=len(evidence),
        stages=tuple(results), physical_confirmations=confirmations,
        findings=("V2A is immutable host evidence only", "powered motor/yaw remain unsupported", "flight_release=false"),
        evidence_sha256=evidence_hash,
    )


def report_to_dict(report: V2Report) -> dict[str, Any]:
    if not isinstance(report, V2Report):
        raise TypeError("report must be a V2Report")
    result = _primitive(report)
    assert isinstance(result, dict)
    return result


def report_to_json(report: V2Report, *, indent: int | None = 2) -> str:
    return json.dumps(report_to_dict(report), ensure_ascii=False, indent=indent, allow_nan=False) + "\n"


def _expect_fields(value: Any, cls: type[Any], *, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{path} must be an object")
    expected = {item.name for item in fields(cls)}; actual = set(value)
    if actual != expected:
        raise ValueError(f"{path} fields mismatch; unknown={sorted(actual - expected)}, missing={sorted(expected - actual)}")
    return value


def _strings(value: Any, *, path: str) -> tuple[str, ...]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        raise TypeError(f"{path} must be an array")
    return tuple(_nonempty(item, name=f"{path} item") for item in value)


def _metric_from_dict(value: Any, path: str) -> Metric:
    row = _expect_fields(value, Metric, path=path)
    return Metric(row["name"], row["value"], row["unit"])


def _stage_from_dict(value: Any, path: str) -> StageResult:
    row = _expect_fields(value, StageResult, path=path)
    if isinstance(row["metrics"], (str, bytes)) or not isinstance(row["metrics"], Sequence):
        raise TypeError(f"{path}.metrics must be an array")
    return StageResult(row["stage"], row["status"], row["sample_count"], tuple(_metric_from_dict(item, f"{path}.metrics[{i}]") for i, item in enumerate(row["metrics"])), _strings(row["findings"], path=f"{path}.findings"))


def _confirmation_from_dict(value: Any, path: str) -> PhysicalConfirmation:
    row = _expect_fields(value, PhysicalConfirmation, path=path)
    return PhysicalConfirmation(**row)


_SAFETY_VALUES: Mapping[str, Any] = MappingProxyType({"format": V2_REPORT_FORMAT, "schema": V2_REPORT_SCHEMA, "evidence_only": True, "applied": False, "auto_apply": False, "parameter_changes_applied": False, "parameters_written": False, "flash_writes": 0, "flight_release": False})


def report_from_dict(payload: Mapping[str, Any]) -> V2Report:
    row = _expect_fields(payload, V2Report, path="report")
    for name, expected in _SAFETY_VALUES.items():
        if type(row[name]) is not type(expected) or row[name] != expected:
            raise ValueError(f"unsafe or unsupported report field {name}: {row[name]!r}")
    threshold_row = _expect_fields(row["thresholds"], V2Thresholds, path="thresholds")
    thresholds = V2Thresholds(**threshold_row)
    context_value = row["context"]
    if not isinstance(context_value, Mapping):
        raise TypeError("context must be an object")
    context_fields = {item.name for item in fields(EvidenceContext)}
    legacy_context_fields = context_fields - {"servo_type"}
    actual_context_fields = set(context_value)
    legacy_context = "servo_type" not in actual_context_fields
    if legacy_context:
        if actual_context_fields != legacy_context_fields:
            raise ValueError(
                f"context fields mismatch; unknown={sorted(actual_context_fields - legacy_context_fields)}, "
                f"missing={sorted(legacy_context_fields - actual_context_fields)}"
            )
        context_data = dict(context_value)
        context_data["servo_type"] = ServoType.BUS.value
        # A legacy payload's integrity covered the context without servo_type.
        # Verify it before normalizing the in-memory context to explicit bus.
        legacy_integrity = row["integrity_sha256"]
        legacy_data = {name: row[name] for name in row if name != "integrity_sha256"}
        if legacy_integrity != _hash(legacy_data):
            raise ValueError("report integrity_sha256 mismatch")
    else:
        context_data = dict(_expect_fields(context_value, EvidenceContext, path="context"))
    context = EvidenceContext(**context_data)
    gate_row = _expect_fields(row["global_gate"], GlobalGateResult, path="global_gate")
    gate = GlobalGateResult(gate_row["status"], gate_row["sample_count"], _strings(gate_row["reasons"], path="global_gate.reasons"))
    stage_rows = row["stages"]; confirmation_rows = row["physical_confirmations"]
    if isinstance(stage_rows, (str, bytes)) or not isinstance(stage_rows, Sequence) or isinstance(confirmation_rows, (str, bytes)) or not isinstance(confirmation_rows, Sequence):
        raise TypeError("stages and physical_confirmations must be arrays")
    integrity = "AUTO" if legacy_context else _nonempty(row["integrity_sha256"], name="integrity_sha256")
    if integrity == "AUTO" and not legacy_context:
        raise ValueError("serialized report cannot request automatic integrity")
    return V2Report(
        created_at=row["created_at"], data_source=row["data_source"], status=row["status"], thresholds=thresholds,
        context=context, global_gate=gate, sample_count=row["sample_count"],
        stages=tuple(_stage_from_dict(item, f"stages[{i}]") for i, item in enumerate(stage_rows)),
        physical_confirmations=tuple(_confirmation_from_dict(item, f"physical_confirmations[{i}]") for i, item in enumerate(confirmation_rows)),
        findings=_strings(row["findings"], path="findings"), evidence_sha256=row["evidence_sha256"], integrity_sha256=integrity,
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


def report_from_json(text: str) -> V2Report:
    if not isinstance(text, str):
        raise TypeError("report JSON must be text")
    return report_from_dict(json.loads(text, object_pairs_hook=_strict_json_object, parse_constant=_reject_json_constant))


def validate_report_for_application(report: V2Report, raw_samples: Sequence[V2Sample], physical_confirmations: Sequence[PhysicalConfirmation]) -> bool:
    if not isinstance(report, V2Report):
        raise TypeError("report must be V2Report")
    rebuilt = build_v2_report(
        raw_samples,
        physical_confirmations=physical_confirmations,
        thresholds=report.thresholds,
        created_at=report.created_at,
        data_source=report.data_source,
        servo_type=report.context.servo_type,
    )
    if rebuilt != report:
        raise ValueError("report does not exactly match recomputed raw evidence")
    return True


def write_report(report: V2Report, path: Path | str) -> Path:
    target = Path(path); target.parent.mkdir(parents=True, exist_ok=True); target.write_text(report_to_json(report), encoding="utf-8"); return target


def load_report(path: Path | str) -> V2Report:
    return report_from_json(Path(path).read_text(encoding="utf-8"))


__all__ = [
    "ACCEPTANCE_CAPTURE_SOURCE", "ACCEPTANCE_PROVENANCE", "AcceptanceStatus", "CANONICAL_FRAME_ID", "ServoType",
    "DEFAULT_THRESHOLDS", "EvidenceContext", "GlobalGateResult", "MechanicalConfirmation", "Metric",
    "PhysicalConfirmation", "REQUIRED_STAGES", "SAFE_FAILSAFE_MODE", "SERVO_STAGES", "STAGE_DEFINITIONS",
    "StageDefinition", "StageResult", "UNSUPPORTED_STAGES", "V2Report", "V2Sample", "V2Stage", "V2Thresholds",
    "V2_REPORT_FORMAT", "V2_REPORT_SCHEMA", "build_v2_report", "evaluate_global_gate", "load_report",
    "report_from_dict", "report_from_json", "report_to_dict", "report_to_json", "validate_report_for_application", "write_report",
]
