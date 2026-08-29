from __future__ import annotations

import csv
import io
import json
import math
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

from tools.flight_validation import (
    OPTIONAL_STAGES,
    REQUIRED_STAGES,
    ROTATION_STAGES,
    SESSION_FORMAT,
    SESSION_SCHEMA,
    STAGE_DEFINITIONS,
    STATIC_STAGES,
    ImuSample,
    ValidationSession,
    ValidationStage,
    ValidationStatus,
    analyze_rotation_stage,
    analyze_six_face,
    analyze_static_stage,
    build_validation_report,
    load_session,
    report_to_csv,
    report_to_json,
    session_from_dict,
    session_from_json,
    session_to_dict,
    session_to_json,
    write_session,
    write_csv_report,
    write_json_report,
)


EXPECTED_STATIC = {
    ValidationStage.LEVEL: (0.0, 0.0, 1.0),
    ValidationStage.NOSE_UP: (1.0, 0.0, 0.0),
    ValidationStage.NOSE_DOWN: (-1.0, 0.0, 0.0),
    ValidationStage.LEFT_SIDE_UP: (0.0, 1.0, 0.0),
    ValidationStage.RIGHT_SIDE_UP: (0.0, -1.0, 0.0),
    ValidationStage.INVERTED: (0.0, 0.0, -1.0),
}


def static_samples(vector: tuple[float, float, float], count: int = 80) -> list[ImuSample]:
    rows = []
    for index in range(count):
        # Deterministic, zero-mean small perturbation keeps the tests reproducible.
        ripple = ((index % 5) - 2) * 0.0005
        rows.append(
            ImuSample(
                timestamp_s=index * 0.01,
                accel_x_g=vector[0] + ripple,
                accel_y_g=vector[1] - ripple * 0.5,
                accel_z_g=vector[2] + ripple * 0.25,
                gyro_x_dps=ripple * 10.0,
                gyro_y_dps=-ripple * 8.0,
                gyro_z_dps=ripple * 6.0,
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_deg=0.0,
            )
        )
    return rows


def rotation_samples(
    axis: int,
    rate_dps: float = 40.0,
    count: int = 101,
) -> list[ImuSample]:
    rows = []
    for index in range(count):
        gyro = [0.2, -0.1, 0.15]
        gyro[axis] = rate_dps
        angles = [0.0, 0.0, 0.0]
        angles[axis] = rate_dps * index * 0.01
        rows.append(
            ImuSample(
                timestamp_s=index * 0.01,
                accel_x_g=0.0,
                accel_y_g=0.0,
                accel_z_g=1.0,
                gyro_x_dps=gyro[0],
                gyro_y_dps=gyro[1],
                gyro_z_dps=gyro[2],
                roll_deg=angles[0],
                pitch_deg=angles[1],
                yaw_deg=angles[2],
            )
        )
    return rows


def complete_flu_samples() -> dict[ValidationStage, list[ImuSample]]:
    samples = {stage: static_samples(EXPECTED_STATIC[stage]) for stage in STATIC_STAGES}
    for axis, stage in enumerate(ROTATION_STAGES):
        samples[stage] = rotation_samples(axis)
    return samples


def transpose_multiply(
    matrix: tuple[tuple[int, int, int], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    # observed = R^T * canonical, where canonical = R * observed.
    return tuple(
        sum(matrix[row][column] * vector[row] for row in range(3))
        for column in range(3)
    )


def test_fixed_stages_document_the_requested_flu_actions() -> None:
    assert tuple(EXPECTED_STATIC) == STATIC_STAGES
    assert tuple(stage.value for stage in ROTATION_STAGES) == (
        "positive_roll",
        "positive_pitch",
        "positive_yaw",
    )
    assert STAGE_DEFINITIONS[ValidationStage.LEVEL].label_zh == "静止水平"
    assert "右翼向下" in STAGE_DEFINITIONS[ValidationStage.POSITIVE_ROLL].prompt_zh
    assert "机头向下" in STAGE_DEFINITIONS[ValidationStage.POSITIVE_PITCH].prompt_zh
    assert "机头向左" in STAGE_DEFINITIONS[ValidationStage.POSITIVE_YAW].prompt_zh
    assert STAGE_DEFINITIONS[ValidationStage.LEVEL].expected_specific_force_g == (
        0.0,
        0.0,
        1.0,
    )
    assert REQUIRED_STAGES == (
        ValidationStage.LEVEL,
        ValidationStage.NOSE_UP,
        ValidationStage.LEFT_SIDE_UP,
        ValidationStage.POSITIVE_ROLL,
        ValidationStage.POSITIVE_PITCH,
        ValidationStage.POSITIVE_YAW,
    )
    assert OPTIONAL_STAGES == (
        ValidationStage.NOSE_DOWN,
        ValidationStage.RIGHT_SIDE_UP,
        ValidationStage.INVERTED,
    )
    assert all(STAGE_DEFINITIONS[stage].required for stage in REQUIRED_STAGES)
    assert all(not STAGE_DEFINITIONS[stage].required for stage in OPTIONAL_STAGES)


def test_static_stage_reports_statistics_axis_sign_and_pass() -> None:
    result = analyze_static_stage(
        ValidationStage.LEFT_SIDE_UP,
        static_samples((0.0, 1.0, 0.0)),
    )

    assert result.status is ValidationStatus.PASS
    assert result.sample_count == 80
    assert result.valid_sample_count == 80
    assert result.accel_mean_g == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)
    assert result.accel_std_g is not None
    assert max(result.accel_std_g) > 0.0
    assert result.gyro_mean_dps == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    assert result.gyro_std_dps is not None
    assert result.accel_norm_mean_g == pytest.approx(1.0, abs=2e-6)
    assert result.accel_norm_std_g is not None
    assert result.dominant_axis == "y"
    assert result.dominant_sign == 1


def test_six_face_passes_flu_and_candidate_is_identity_but_not_applied() -> None:
    result = analyze_six_face(
        {
            stage: static_samples(EXPECTED_STATIC[stage])
            for stage in STATIC_STAGES
            if STAGE_DEFINITIONS[stage].required
        }
    )

    assert result.status is ValidationStatus.PASS
    assert result.capture_quality_status is ValidationStatus.PASS
    assert result.canonical_match_status is ValidationStatus.PASS
    assert result.candidate.matrix_flu_from_observed == (
        (1, 0, 0),
        (0, 1, 0),
        (0, 0, 1),
    )
    assert result.candidate.determinant == 1
    assert result.candidate.proper_rotation is True
    assert result.candidate.recommendation_only is True
    assert result.candidate.applied is False
    assert all(
        result.stages[stage.value].status is ValidationStatus.SKIPPED
        for stage in OPTIONAL_STAGES
    )


def test_six_face_infers_nonidentity_signed_permutation_without_claiming_match() -> None:
    # body_FLU.x=-observed.z, body_FLU.y=+observed.x,
    # body_FLU.z=-observed.y; this is a proper right-handed rotation.
    expected_matrix = (
        (0, 0, -1),
        (1, 0, 0),
        (0, -1, 0),
    )
    observed_samples = {
        stage: static_samples(transpose_multiply(expected_matrix, canonical))
        for stage, canonical in EXPECTED_STATIC.items()
    }

    result = analyze_six_face(observed_samples)

    assert result.status is ValidationStatus.FAIL
    assert result.capture_quality_status is ValidationStatus.PASS
    assert result.canonical_match_status is ValidationStatus.FAIL
    assert result.candidate.complete is True
    assert result.candidate.matrix_flu_from_observed == expected_matrix
    assert result.candidate.axis_mapping == {
        "flu_x": "-observed_z",
        "flu_y": "+observed_x",
        "flu_z": "-observed_y",
    }
    assert result.candidate.determinant == 1
    assert result.candidate.status is ValidationStatus.PASS
    assert result.candidate.applied is False


def test_three_positive_bases_reject_an_improper_signed_permutation() -> None:
    result = analyze_six_face(
        {
            ValidationStage.LEVEL: static_samples((0.0, 0.0, -1.0)),
            ValidationStage.NOSE_UP: static_samples((1.0, 0.0, 0.0)),
            ValidationStage.LEFT_SIDE_UP: static_samples((0.0, 1.0, 0.0)),
        }
    )

    assert result.candidate.complete is True
    assert result.candidate.matrix_flu_from_observed == (
        (1, 0, 0),
        (0, 1, 0),
        (0, 0, -1),
    )
    assert result.candidate.determinant == -1
    assert result.candidate.proper_rotation is False
    assert result.candidate.status is ValidationStatus.FAIL
    assert any("determinant" in finding for finding in result.candidate.findings)


@pytest.mark.parametrize(
    ("stage", "axis"),
    tuple(zip(ROTATION_STAGES, range(3))),
)
def test_positive_rotation_uses_canonical_flu_gyro_direction(
    stage: ValidationStage, axis: int
) -> None:
    result = analyze_rotation_stage(stage, rotation_samples(axis))

    assert result.status is ValidationStatus.PASS
    assert result.target_axis == ("x", "y", "z")[axis]
    assert result.integrated_angle_deg is not None
    assert result.integrated_angle_deg[axis] == pytest.approx(40.0)
    assert result.dominant_axis == result.target_axis
    assert result.dominant_sign == 1
    assert result.gyro_direction_matches_flu is True
    assert result.attitude_direction_matches_flu is True


def test_reversed_positive_rotation_fails_direction_evidence() -> None:
    result = analyze_rotation_stage(
        ValidationStage.POSITIVE_PITCH,
        rotation_samples(axis=1, rate_dps=-40.0),
    )

    assert result.status is ValidationStatus.FAIL
    assert result.integrated_angle_deg is not None
    assert result.integrated_angle_deg[1] == pytest.approx(-40.0)
    assert result.gyro_direction_matches_flu is False
    assert result.dominant_axis == "y"
    assert result.dominant_sign == -1
    assert any("相反" in finding for finding in result.findings)


def test_insufficient_samples_fail_without_fabricating_statistics() -> None:
    static_result = analyze_static_stage(
        ValidationStage.LEVEL,
        static_samples((0.0, 0.0, 1.0), count=3),
    )
    empty_result = analyze_static_stage(ValidationStage.INVERTED, [])
    rotation_result = analyze_rotation_stage(
        ValidationStage.POSITIVE_YAW,
        rotation_samples(axis=2, count=3),
    )

    assert static_result.status is ValidationStatus.FAIL
    assert static_result.sample_count == 3
    assert static_result.accel_mean_g is not None
    assert empty_result.status is ValidationStatus.FAIL
    assert empty_result.sample_count == 0
    assert empty_result.accel_mean_g is None
    assert rotation_result.status is ValidationStatus.FAIL
    assert rotation_result.sample_count == 3


def test_report_json_and_csv_are_explicitly_legacy_validation_evidence(tmp_path) -> None:
    created_at = "2026-08-28T12:34:56+08:00"
    report = build_validation_report(
        complete_flu_samples(),
        firmware_hash="sha256:test-firmware",
        created_at=created_at,
    )

    assert report.status is ValidationStatus.PASS
    assert report.frame_contract_version == 1
    assert report.frame_id == "FLU"
    assert report.input_frame == "legacy_or_unverified"
    assert report.data_source == "caller_supplied_legacy_or_unverified_samples"
    assert report.input_units == {
        "timestamp": "s",
        "accelerometer": "g",
        "gyroscope": "deg/s",
        "attitude_optional": "deg",
    }
    assert report.threshold_version == 1
    assert report.runtime_migration_complete is False
    assert report.evidence_type == "validation_evidence"
    assert report.evidence_only is True
    assert report.automatic_calibration_performed is False
    assert report.parameter_changes_applied is False
    assert report.parameters_written is False
    assert report.flash_writes == 0
    assert report.firmware_written is False
    assert report.flight_release is False
    with pytest.raises(FrozenInstanceError):
        report.runtime_migration_complete = True  # type: ignore[misc]

    json_text = report_to_json(report)
    payload = json.loads(json_text)
    assert payload["created_at"] == created_at
    assert payload["firmware_hash"] == "sha256:test-firmware"
    assert payload["runtime_migration_complete"] is False
    assert payload["parameter_changes_applied"] is False
    assert payload["flash_writes"] == 0
    assert payload["flight_release"] is False
    assert payload["data_source"] == "caller_supplied_legacy_or_unverified_samples"
    assert payload["input_units"]["gyroscope"] == "deg/s"
    assert payload["threshold_version"] == 1
    assert payload["six_face"]["candidate"]["recommendation_only"] is True
    assert payload["six_face"]["candidate"]["applied"] is False

    csv_text = report_to_csv(report)
    rows = list(csv.DictReader(io.StringIO(csv_text)))
    assert len(rows) == 9
    assert {row["section"] for row in rows} == {"six_face", "rotation"}
    assert all(row["evidence_type"] == "validation_evidence" for row in rows)
    assert all(row["runtime_migration_complete"] == "false" for row in rows)
    assert all(row["parameter_changes_applied"] == "false" for row in rows)
    assert all(row["flash_writes"] == "0" for row in rows)
    assert all(row["flight_release"] == "false" for row in rows)
    assert all(row["threshold_version"] == "1" for row in rows)
    assert all(row["data_source"] for row in rows)
    assert all(row["frame_id"] == "FLU" for row in rows)

    json_path = write_json_report(report, tmp_path / "report.json")
    csv_path = write_csv_report(report, tmp_path / "report.csv")
    assert json.loads(json_path.read_text(encoding="utf-8"))["firmware_hash"] == (
        "sha256:test-firmware"
    )
    assert len(list(csv.DictReader(csv_path.open(encoding="utf-8")))) == 9


def test_imu_sample_and_module_are_pure_standard_library_contracts() -> None:
    sample = ImuSample(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    assert sample.accel_g == (0.0, 0.0, 1.0)
    assert sample.gyro_dps == (0.0, 0.0, 0.0)
    assert sample.attitude_deg == (None, None, None)
    assert math.isclose(sample.accel_z_g, 1.0)


def test_missing_stage_is_not_run_and_blocks_overall_pass() -> None:
    samples = complete_flu_samples()
    del samples[ValidationStage.POSITIVE_YAW]

    report = build_validation_report(samples)

    yaw = report.rotations.stages[ValidationStage.POSITIVE_YAW.value]
    assert yaw.status is ValidationStatus.NOT_RUN
    assert report.rotations.status is ValidationStatus.NOT_RUN
    assert report.status is ValidationStatus.NOT_RUN
    assert any("尚未执行" in finding for finding in yaw.findings)


def test_explicitly_unsupported_optional_stage_does_not_block_required_flow() -> None:
    samples: dict[ValidationStage, list[ImuSample] | None] = complete_flu_samples()
    samples[ValidationStage.INVERTED] = None

    report = build_validation_report(samples)

    inverted = report.six_face.stages[ValidationStage.INVERTED.value]
    assert inverted.status is ValidationStatus.UNSUPPORTED
    assert report.six_face.candidate.status is ValidationStatus.PASS
    assert report.six_face.status is ValidationStatus.PASS
    assert report.status is ValidationStatus.PASS
    assert any("不支持" in finding for finding in inverted.findings)


def test_explicitly_unsupported_required_static_stage_still_blocks() -> None:
    samples: dict[ValidationStage, list[ImuSample] | None] = complete_flu_samples()
    samples[ValidationStage.LEVEL] = None

    report = build_validation_report(samples)

    level = report.six_face.stages[ValidationStage.LEVEL.value]
    assert level.status is ValidationStatus.UNSUPPORTED
    assert report.six_face.candidate.status is ValidationStatus.UNSUPPORTED
    assert report.six_face.status is ValidationStatus.UNSUPPORTED
    assert report.status is ValidationStatus.UNSUPPORTED


@pytest.mark.parametrize(
    ("static_count", "expected_status"),
    ((80, ValidationStatus.PASS), (30, ValidationStatus.WARN)),
)
def test_three_required_static_bases_and_rotations_complete_the_standard_flow(
    static_count: int,
    expected_status: ValidationStatus,
) -> None:
    samples = {
        stage: static_samples(EXPECTED_STATIC[stage], count=static_count)
        for stage in STATIC_STAGES
        if STAGE_DEFINITIONS[stage].required
    }
    for axis, stage in enumerate(ROTATION_STAGES):
        samples[stage] = rotation_samples(axis)

    report = build_validation_report(samples)

    assert all(
        report.six_face.stages[stage.value].status is ValidationStatus.SKIPPED
        for stage in OPTIONAL_STAGES
    )
    assert report.six_face.candidate.complete is True
    assert report.six_face.candidate.matrix_flu_from_observed == (
        (1, 0, 0),
        (0, 1, 0),
        (0, 0, 1),
    )
    assert report.six_face.status is expected_status
    assert report.status is expected_status


def test_optional_handheld_warn_is_analyzed_without_lowering_required_status() -> None:
    samples = {
        stage: static_samples(EXPECTED_STATIC[stage])
        for stage in STATIC_STAGES
        if STAGE_DEFINITIONS[stage].required
    }
    samples[ValidationStage.INVERTED] = static_samples(
        EXPECTED_STATIC[ValidationStage.INVERTED], count=30
    )
    for axis, stage in enumerate(ROTATION_STAGES):
        samples[stage] = rotation_samples(axis)

    report = build_validation_report(samples)

    inverted = report.six_face.stages[ValidationStage.INVERTED.value]
    assert inverted.status is ValidationStatus.WARN
    assert inverted.sample_count == 30
    assert report.six_face.candidate.complete is True
    assert report.six_face.status is ValidationStatus.PASS
    assert report.status is ValidationStatus.PASS
    assert all("冗余" not in finding for finding in report.six_face.candidate.findings)


def test_optional_opposite_pose_can_detect_a_clear_direction_conflict() -> None:
    samples = {
        ValidationStage.LEVEL: static_samples((0.0, 0.0, 1.0)),
        ValidationStage.NOSE_UP: static_samples((1.0, 0.0, 0.0)),
        ValidationStage.LEFT_SIDE_UP: static_samples((0.0, 1.0, 0.0)),
        # A real nose-down pose must observe the inverse of nose-up.  Repeating
        # +X is usable, high-quality evidence and therefore a genuine conflict.
        ValidationStage.NOSE_DOWN: static_samples((1.0, 0.0, 0.0)),
    }

    result = analyze_six_face(samples)

    assert result.capture_quality_status is ValidationStatus.PASS
    assert result.canonical_match_status is ValidationStatus.PASS
    assert result.candidate.complete is True
    assert result.candidate.status is ValidationStatus.FAIL
    assert result.status is ValidationStatus.FAIL
    assert any("不互反" in finding for finding in result.candidate.findings)
    assert all("冗余" not in finding for finding in result.candidate.findings)


def _session_fixture() -> ValidationSession:
    samples = complete_flu_samples()
    del samples[ValidationStage.INVERTED]
    del samples[ValidationStage.POSITIVE_YAW]
    return ValidationSession(
        samples_by_stage=samples,
        unsupported_stages={ValidationStage.POSITIVE_YAW},
        skipped_stages={ValidationStage.INVERTED},
        finished_stages=set(ValidationStage),
        firmware_hash="sha256:test-session-firmware",
        created_at="2026-08-28T10:00:00+08:00",
        updated_at="2026-08-28T10:05:00+08:00",
    )


def test_validation_session_round_trip_preserves_every_sample_and_safety_contract() -> None:
    session = _session_fixture()

    payload = session_to_dict(session)
    assert payload["format"] == SESSION_FORMAT
    assert payload["schema"] == SESSION_SCHEMA
    assert payload["created_at"] == "2026-08-28T10:00:00+08:00"
    assert payload["updated_at"] == "2026-08-28T10:05:00+08:00"
    assert payload["firmware_hash"] == "sha256:test-session-firmware"
    assert payload["evidence_only"] is True
    assert payload["parameter_changes_applied"] is False
    assert payload["parameters_written"] is False
    assert payload["flash_writes"] == 0
    assert payload["firmware_written"] is False
    assert payload["flight_release"] is False
    assert set(payload["samples_by_stage"]) == {stage.value for stage in ValidationStage}
    assert len(payload["samples_by_stage"][ValidationStage.LEVEL.value]) == 80
    assert payload["samples_by_stage"][ValidationStage.INVERTED.value] == []
    assert payload["unsupported_stages"] == [ValidationStage.POSITIVE_YAW.value]
    assert payload["skipped_stages"] == [ValidationStage.INVERTED.value]
    assert set(payload["finished_stages"]) == {stage.value for stage in ValidationStage}

    from_dict = session_from_dict(payload)
    from_json = session_from_json(session_to_json(session))
    for restored in (from_dict, from_json):
        assert restored.created_at == session.created_at
        assert restored.updated_at == session.updated_at
        assert restored.firmware_hash == session.firmware_hash
        assert restored.unsupported_stages == session.unsupported_stages
        assert restored.skipped_stages == session.skipped_stages
        assert restored.finished_stages == session.finished_stages
        assert restored.samples_by_stage[ValidationStage.LEVEL] == tuple(
            session.samples_by_stage[ValidationStage.LEVEL]
        )
        assert restored.evidence_only is True
        assert restored.flash_writes == 0
        assert restored.flight_release is False


def test_validation_session_file_round_trip(tmp_path: Path) -> None:
    session = _session_fixture()
    path = write_session(session, tmp_path / "nested" / "session.json")

    assert path == tmp_path / "nested" / "session.json"
    assert path.exists()
    assert not path.with_name(path.name + ".tmp").exists()
    restored = load_session(path)
    assert restored.samples_by_stage[ValidationStage.NOSE_UP] == tuple(
        session.samples_by_stage[ValidationStage.NOSE_UP]
    )
    assert restored.skipped_stages == frozenset({ValidationStage.INVERTED})


@pytest.mark.parametrize(
    "mutate",
    (
        lambda payload: payload["samples_by_stage"].__setitem__("future_stage", []),
        lambda payload: payload["unsupported_stages"].append("future_stage"),
        lambda payload: payload["samples_by_stage"]["level"][0].__setitem__(
            "gyro_x_dps", "not-a-number"
        ),
        lambda payload: payload["samples_by_stage"]["level"][0].__setitem__(
            "accel_z_g", math.nan
        ),
        lambda payload: payload["samples_by_stage"]["level"][0].__setitem__(
            "timestamp_s", True
        ),
    ),
)
def test_session_loader_rejects_unknown_stages_and_bad_numeric_values(mutate) -> None:
    payload = session_to_dict(_session_fixture())
    mutate(payload)

    with pytest.raises((TypeError, ValueError)):
        session_from_dict(payload)


def test_session_loader_rejects_conflicting_and_unsafe_metadata() -> None:
    payload = session_to_dict(_session_fixture())
    payload["unsupported_stages"].append(ValidationStage.INVERTED.value)
    with pytest.raises(ValueError, match="skipped.*unsupported|unsupported.*skipped"):
        session_from_dict(payload)

    payload = session_to_dict(_session_fixture())
    payload["flash_writes"] = 1
    with pytest.raises(ValueError, match="flash_writes"):
        session_from_dict(payload)

    with pytest.raises(ValueError):
        session_from_json('{"format":"x","schema":1,"samples_by_stage":{"level":[{"timestamp_s":NaN}]}}')
