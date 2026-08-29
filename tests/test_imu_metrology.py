from __future__ import annotations

import json
import math
from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest

from tools import imu_metrology as metrology
from tools.flight_validation import ImuSample
from tools.imu_metrology import (
    ACCEL_FACE_STAGES,
    CaptureMethod,
    MetrologySample,
    MetrologyStage,
    MetrologyStatus,
    analyze_gyro_rotations,
    analyze_stationary_gyro,
    analyze_temperature_drift,
    build_metrology_candidate,
    candidate_from_dict,
    candidate_from_json,
    candidate_to_dict,
    candidate_to_json,
    fit_accelerometer,
    metrology_sample_from_v0,
    validate_candidate_for_application,
    validate_room_temperature_candidate_for_application,
)


BIAS_ACCEL = np.asarray((0.030, -0.020, 0.010))
ACCEL_RESPONSE = np.asarray(
    (
        (1.020, 0.010, -0.005),
        (0.004, 0.985, 0.007),
        (-0.006, 0.003, 1.010),
    )
)
BIAS_GYRO = np.asarray((0.40, -0.25, 0.10))
GYRO_RESPONSE = np.asarray(
    (
        (1.020, 0.010, -0.005),
        (0.003, 0.990, 0.006),
        (-0.004, 0.005, 1.010),
    )
)
EXPECTED_FACE = {
    MetrologyStage.ACCEL_POS_X: np.asarray((1.0, 0.0, 0.0)),
    MetrologyStage.ACCEL_NEG_X: np.asarray((-1.0, 0.0, 0.0)),
    MetrologyStage.ACCEL_POS_Y: np.asarray((0.0, 1.0, 0.0)),
    MetrologyStage.ACCEL_NEG_Y: np.asarray((0.0, -1.0, 0.0)),
    MetrologyStage.ACCEL_POS_Z: np.asarray((0.0, 0.0, 1.0)),
    MetrologyStage.ACCEL_NEG_Z: np.asarray((0.0, 0.0, -1.0)),
}


def sample(
    stage: MetrologyStage,
    *,
    timestamp_s: float,
    accel=(0.0, 0.0, 1.0),
    gyro=(0.0, 0.0, 0.0),
    temperature_c: float = 25.0,
    method: CaptureMethod = CaptureMethod.BENCH,
    platform: str | None = None,
    provenance: str = "synthetic-v1",
    orientation_code: int = 3,
    generation: int = 7,
    firmware_hash: str = "sha256:synthetic-firmware-20260828",
    session_id: str = "v1-session-synthetic-001",
    capture_source: str = "usb_cdc:synthetic",
) -> MetrologySample:
    return MetrologySample(
        timestamp_s=timestamp_s,
        accel_x_g=float(accel[0]),
        accel_y_g=float(accel[1]),
        accel_z_g=float(accel[2]),
        gyro_x_dps=float(gyro[0]),
        gyro_y_dps=float(gyro[1]),
        gyro_z_dps=float(gyro[2]),
        temperature_c=temperature_c,
        stage=stage,
        frame_id="FLU",
        frame_contract_version=1,
        orientation_code=orientation_code,
        calibration_generation=generation,
        firmware_hash=firmware_hash,
        session_id=session_id,
        capture_source=capture_source,
        provenance=provenance,
        capture_method=method,
        temperature_platform=platform,
    )


def accel_samples(
    response: np.ndarray = ACCEL_RESPONSE,
    bias: np.ndarray = BIAS_ACCEL,
    *,
    count: int = 1500,
    method: CaptureMethod = CaptureMethod.BENCH,
) -> list[MetrologySample]:
    result: list[MetrologySample] = []
    timestamp = 0.0
    for stage in ACCEL_FACE_STAGES:
        observed = bias + response @ EXPECTED_FACE[stage]
        for _ in range(count):
            result.append(
                sample(
                    stage,
                    timestamp_s=timestamp,
                    accel=observed,
                    method=method,
                )
            )
            timestamp += 0.001
    return result


def gyro_static_samples(count: int = 4000) -> list[MetrologySample]:
    result = []
    for index in range(count):
        # Deterministic, zero-mean sub-threshold noise on every axis.
        noise = np.asarray(
            (
                0.04 if index % 2 else -0.04,
                0.03 if index % 4 < 2 else -0.03,
                0.02 if index % 8 < 4 else -0.02,
            )
        )
        result.append(
            sample(
                MetrologyStage.GYRO_STATIC,
                timestamp_s=2.0 + index * 0.001,
                gyro=BIAS_GYRO + noise,
            )
        )
    return result


def gyro_rotation_samples(
    method: CaptureMethod = CaptureMethod.REFERENCE_FIXTURE,
) -> list[MetrologySample]:
    result: list[MetrologySample] = []
    stages = (
        MetrologyStage.GYRO_POS_360_X,
        MetrologyStage.GYRO_POS_360_Y,
        MetrologyStage.GYRO_POS_360_Z,
    )
    for axis, stage in enumerate(stages):
        rate = BIAS_GYRO + 360.0 * GYRO_RESPONSE[:, axis]
        for index in range(101):
            result.append(
                sample(
                    stage,
                    timestamp_s=index / 100.0,
                    gyro=rate,
                    method=method,
                )
            )
    return result


def temperature_samples(count: int = 4000) -> list[MetrologySample]:
    accel_slope = np.asarray((0.0004, -0.0002, 0.0006))
    gyro_slope = np.asarray((0.010, -0.020, 0.005))
    result: list[MetrologySample] = []
    for platform, temperature in (("cold", 10.0), ("room", 25.0), ("warm", 40.0)):
        delta = temperature - 25.0
        corrected_accel = np.asarray((0.0, 0.0, 1.0)) + accel_slope * delta
        observed_accel = BIAS_ACCEL + ACCEL_RESPONSE @ corrected_accel
        corrected_gyro = gyro_slope * delta
        observed_gyro = BIAS_GYRO + GYRO_RESPONSE @ corrected_gyro
        for index in range(count):
            result.append(
                sample(
                    MetrologyStage.TEMPERATURE_STATIC,
                    timestamp_s=index * 0.001,
                    accel=observed_accel,
                    gyro=observed_gyro,
                    temperature_c=temperature,
                    platform=platform,
                )
            )
    return result


@pytest.fixture(scope="module")
def complete_samples() -> tuple[MetrologySample, ...]:
    return tuple(
        accel_samples()
        + gyro_static_samples()
        + gyro_rotation_samples()
        + temperature_samples()
    )


def test_sample_requires_finite_and_complete_provenance() -> None:
    valid = sample(MetrologyStage.ACCEL_POS_Z, timestamp_s=0.0)
    assert valid.frame_id == "FLU"
    assert valid.frame_contract_version == 1
    assert valid.orientation_code == 3
    assert valid.calibration_generation == 7
    assert valid.firmware_hash.startswith("sha256:")
    assert valid.session_id == "v1-session-synthetic-001"
    assert valid.capture_source == "usb_cdc:synthetic"
    assert valid.temperature_c == 25.0

    with pytest.raises(ValueError, match="finite"):
        sample(MetrologyStage.ACCEL_POS_Z, timestamp_s=math.nan)
    with pytest.raises(ValueError, match="0..23"):
        sample(MetrologyStage.ACCEL_POS_Z, timestamp_s=0.0, orientation_code=255)
    with pytest.raises(TypeError, match="temperature_platform"):
        sample(MetrologyStage.TEMPERATURE_STATIC, timestamp_s=0.0)
    with pytest.raises(ValueError, match="only valid"):
        sample(
            MetrologyStage.ACCEL_POS_Z,
            timestamp_s=0.0,
            platform="room",
        )


def test_accel_six_face_fits_bias_and_full_3x3_correction() -> None:
    result = fit_accelerometer(accel_samples())
    assert result.status is MetrologyStatus.PASS
    assert result.bias_g == pytest.approx(tuple(BIAS_ACCEL), abs=1e-10)
    assert np.asarray(result.correction_matrix) == pytest.approx(
        np.linalg.inv(ACCEL_RESPONSE), abs=1e-10
    )
    assert result.determinant is not None and result.determinant > 0.0
    assert result.condition_number is not None and result.condition_number <= 1.25
    assert result.singular_values is not None
    assert min(result.singular_values) >= 0.85
    assert max(result.singular_values) <= 1.15
    assert result.bias_norm_g is not None and result.bias_norm_g <= 0.20
    assert result.corrected_rms_g == pytest.approx(0.0, abs=1e-10)
    assert result.corrected_max_error_g == pytest.approx(0.0, abs=1e-10)


@pytest.mark.parametrize(
    "response",
    (
        np.diag((-1.0, -1.0, 1.0)),
        np.asarray(
            (
                (0.0, -1.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
            )
        ),
    ),
)
def test_accel_rejects_proper_rotations_that_belong_to_v0(
    response: np.ndarray,
) -> None:
    assert np.linalg.det(response) == pytest.approx(1.0)
    result = fit_accelerometer(accel_samples(response=response, bias=np.zeros(3)))
    assert result.status is MetrologyStatus.FAIL
    assert any("V0" in finding or "单位阵" in finding for finding in result.findings)


def test_accel_requires_1500_samples_duration_monotonicity_and_static_data() -> None:
    assert metrology.DEFAULT_THRESHOLDS.accel_min_samples_per_face == 1500
    assert fit_accelerometer(accel_samples(count=1499)).status is MetrologyStatus.INCOMPLETE
    assert fit_accelerometer(accel_samples(count=1500)).status is MetrologyStatus.PASS

    compressed = [
        replace(row, timestamp_s=row.timestamp_s * 0.1)
        for row in accel_samples()
    ]
    assert fit_accelerometer(compressed).status is MetrologyStatus.INCOMPLETE

    nonmonotonic = accel_samples()
    nonmonotonic[100] = replace(
        nonmonotonic[100], timestamp_s=nonmonotonic[99].timestamp_s
    )
    broken_time = fit_accelerometer(nonmonotonic)
    assert broken_time.status is MetrologyStatus.FAIL
    assert any("时间戳" in finding for finding in broken_time.findings)

    moving = accel_samples()
    for index in range(1500):
        row = moving[index]
        moving[index] = replace(
            row,
            accel_y_g=row.accel_y_g + (0.05 if index % 2 else -0.05),
        )
    unstable = fit_accelerometer(moving)
    assert unstable.status is MetrologyStatus.FAIL
    assert any("静止标准差" in finding for finding in unstable.findings)


@pytest.mark.parametrize(
    ("response", "bias", "reason"),
    (
        (np.diag((-1.0, 1.0, 1.0)), np.zeros(3), "det"),
        (np.diag((0.75, 1.0, 1.0)), np.zeros(3), "奇异值"),
        (np.eye(3), np.asarray((0.21, 0.0, 0.0)), "bias"),
    ),
)
def test_accel_hard_gates_reject_bad_geometry_or_bias(
    response: np.ndarray,
    bias: np.ndarray,
    reason: str,
) -> None:
    result = fit_accelerometer(accel_samples(response=response, bias=bias))
    assert result.status is MetrologyStatus.FAIL
    assert any(reason in finding for finding in result.findings)


def tilt_one_face(samples, stage, degrees_: float, axis: str = "x"):
    """把整面样本绕一个轴倾斜 —— 模拟"这一面摆得跟对面不一样"。"""
    radians_ = math.radians(degrees_)
    result = []
    for item in samples:
        if item.stage is not stage:
            result.append(item)
            continue
        vector = np.asarray(item.accel_g, dtype=float)
        cos, sin = math.cos(radians_), math.sin(radians_)
        if axis == "x":
            turned = np.array([vector[0] * cos - vector[1] * sin,
                               vector[0] * sin + vector[1] * cos, vector[2]])
        else:
            turned = np.array([vector[0], vector[1] * cos - vector[2] * sin,
                               vector[1] * sin + vector[2] * cos])
        result.append(replace(item, accel_x_g=turned[0], accel_y_g=turned[1], accel_z_g=turned[2]))
    return result


def test_a_hand_placed_six_face_set_is_usable_instead_of_rejected() -> None:
    """徒手摆六面达不到 0.025 g；差一点只该 WARN，不该整份拒收。

    残差衡量的是"这一对正负面摆得一致不一致"，不是数据坏没坏 —— 坏数据由 std、
    奇异值、对角/非对角那几道门先拦。10° 的对内差折合约 1.7° 水平误差，远小于
    "不标定"的代价；PX4 连这道检查都没有。
    """
    tilted = tilt_one_face(accel_samples(), MetrologyStage.ACCEL_NEG_Y, 10.0)

    result = fit_accelerometer(tilted)

    assert result.status is MetrologyStatus.WARN
    assert 0.025 < result.corrected_rms_g <= 0.075
    assert any("一致性折合约" in finding for finding in result.findings)


def test_accel_hard_gate_still_rejects_a_wildly_inconsistent_face() -> None:
    tilted = tilt_one_face(accel_samples(), MetrologyStage.ACCEL_NEG_Y, 25.0)

    result = fit_accelerometer(tilted)

    assert result.status is MetrologyStatus.FAIL
    assert result.corrected_rms_g > 0.075
    assert any("摆放差异过大" in finding for finding in result.findings)


def test_a_single_outlier_sample_is_still_caught_by_the_max_error_gate() -> None:
    """RMS 会被 9000 个样本摊平，单点野值只有最大误差这道门拦得住。"""
    samples = accel_samples()
    bad = samples[-1]
    samples[-1] = replace(bad, accel_x_g=bad.accel_x_g + 0.40)

    result = fit_accelerometer(samples)

    assert result.status is MetrologyStatus.FAIL
    assert result.corrected_rms_g < 0.025
    assert any("最大误差" in finding for finding in result.findings)


def test_stationary_gyro_reports_bias_noise_and_sample_gate() -> None:
    result = analyze_stationary_gyro(gyro_static_samples())
    assert result.status is MetrologyStatus.PASS
    assert result.bias_dps == pytest.approx(tuple(BIAS_GYRO), abs=1e-10)
    assert result.std_dps == pytest.approx((0.04, 0.03, 0.02), abs=1e-10)
    assert result.noise_vector_rms_dps == pytest.approx(
        math.sqrt(0.04**2 + 0.03**2 + 0.02**2), abs=1e-10
    )

    short = analyze_stationary_gyro(gyro_static_samples(100))
    assert short.status is MetrologyStatus.INCOMPLETE

    compressed = [
        replace(row, timestamp_s=2.0 + (row.timestamp_s - 2.0) * 0.1)
        for row in gyro_static_samples()
    ]
    assert analyze_stationary_gyro(compressed).status is MetrologyStatus.INCOMPLETE

    nonmonotonic = gyro_static_samples()
    nonmonotonic[100] = replace(
        nonmonotonic[100], timestamp_s=nonmonotonic[99].timestamp_s
    )
    result = analyze_stationary_gyro(nonmonotonic)
    assert result.status is MetrologyStatus.FAIL
    assert any("时间戳" in finding for finding in result.findings)


def test_gyro_360_fits_scale_cross_axis_and_caps_manual_data() -> None:
    fixture = analyze_gyro_rotations(
        gyro_rotation_samples(), gyro_bias_dps=tuple(BIAS_GYRO)
    )
    assert fixture.status is MetrologyStatus.PASS
    assert fixture.correction_matrix is not None
    assert np.asarray(fixture.correction_matrix) == pytest.approx(
        np.linalg.inv(GYRO_RESPONSE), abs=1e-10
    )
    assert all(
        stage.main_axis_scale is not None
        and 0.85 <= stage.main_axis_scale <= 1.15
        and stage.cross_axis_fraction is not None
        and stage.cross_axis_fraction < 0.05
        for stage in fixture.stages
    )

    manual = analyze_gyro_rotations(
        gyro_rotation_samples(CaptureMethod.MANUAL),
        gyro_bias_dps=tuple(BIAS_GYRO),
    )
    assert manual.status is MetrologyStatus.WARN
    assert any("只能作为 WARN" in finding for finding in manual.findings)


def test_temperature_requires_three_platforms_span_and_4000_samples() -> None:
    one_platform = temperature_samples(count=10)[:10]
    single = analyze_temperature_drift(one_platform)
    assert single.status is MetrologyStatus.INCOMPLETE
    assert len(single.platforms) == 1
    assert single.accel_slope_g_per_c is None

    result = analyze_temperature_drift(
        temperature_samples(),
        accel_bias_g=tuple(BIAS_ACCEL),
        accel_correction_matrix=tuple(tuple(row) for row in np.linalg.inv(ACCEL_RESPONSE)),
        gyro_bias_dps=tuple(BIAS_GYRO),
        gyro_correction_matrix=tuple(tuple(row) for row in np.linalg.inv(GYRO_RESPONSE)),
    )
    assert result.status is MetrologyStatus.PASS
    assert len(result.platforms) == 3
    assert result.temperature_span_c == pytest.approx(30.0)
    assert all(platform.sample_count == 4000 for platform in result.platforms)
    assert result.accel_slope_g_per_c == pytest.approx(
        (0.0004, -0.0002, 0.0006), abs=1e-10
    )
    assert result.gyro_slope_dps_per_c == pytest.approx(
        (0.010, -0.020, 0.005), abs=1e-10
    )
    assert result.accel_fit_rms_g == pytest.approx((0.0, 0.0, 0.0), abs=1e-10)
    assert result.gyro_fit_rms_dps == pytest.approx((0.0, 0.0, 0.0), abs=1e-10)
    assert result.accel_fit_r2 == pytest.approx((1.0, 1.0, 1.0), abs=1e-10)
    assert result.gyro_fit_r2 == pytest.approx((1.0, 1.0, 1.0), abs=1e-10)
    assert all(platform.temperature_span_c == 0.0 for platform in result.platforms)

    too_short = analyze_temperature_drift(temperature_samples(count=3999))
    assert too_short.status is MetrologyStatus.INCOMPLETE
    narrow = [
        replace(row, temperature_c=20.0 + (0.5 if row.temperature_platform == "room" else 1.0 if row.temperature_platform == "warm" else 0.0))
        for row in temperature_samples()
    ]
    assert analyze_temperature_drift(narrow).status is MetrologyStatus.INCOMPLETE


def test_temperature_rejects_nonphysical_unstable_and_nonlinear_platforms() -> None:
    impossible: list[MetrologySample] = []
    for platform, temperature, accel, gyro in (
        ("cold", 10.0, (8.0, 0.0, 0.0), (100.0, 0.0, 0.0)),
        ("room", 25.0, (-8.0, 3.0, 9.0), (-100.0, 50.0, 0.0)),
        ("warm", 40.0, (40.0, -20.0, 5.0), (500.0, -300.0, 200.0)),
    ):
        impossible.extend(
            sample(
                MetrologyStage.TEMPERATURE_STATIC,
                timestamp_s=index * 0.001,
                accel=accel,
                gyro=gyro,
                temperature_c=temperature,
                platform=platform,
            )
            for index in range(4000)
        )
    impossible_result = analyze_temperature_drift(impossible)
    assert impossible_result.status is MetrologyStatus.FAIL
    assert any("水平" in finding or "物理" in finding for finding in impossible_result.findings)

    unstable_temperature = [
        replace(
            row,
            temperature_c=row.temperature_c + (0.75 if index % 2 else -0.75),
        )
        for index, row in enumerate(temperature_samples())
    ]
    unstable_result = analyze_temperature_drift(unstable_temperature)
    assert unstable_result.status is MetrologyStatus.FAIL
    assert any("平台内温度跨度" in finding for finding in unstable_result.findings)

    nonlinear: list[MetrologySample] = []
    for platform, temperature, offset_x in (
        ("cold", 10.0, -0.02),
        ("room", 25.0, 0.04),
        ("warm", 40.0, -0.02),
    ):
        corrected_accel = np.asarray((offset_x, 0.0, 1.0))
        observed_accel = BIAS_ACCEL + ACCEL_RESPONSE @ corrected_accel
        for index in range(4000):
            nonlinear.append(
                sample(
                    MetrologyStage.TEMPERATURE_STATIC,
                    timestamp_s=index * 0.001,
                    accel=observed_accel,
                    gyro=BIAS_GYRO,
                    temperature_c=temperature,
                    platform=platform,
                )
            )
    nonlinear_result = analyze_temperature_drift(
        nonlinear,
        accel_bias_g=tuple(BIAS_ACCEL),
        accel_correction_matrix=tuple(tuple(row) for row in np.linalg.inv(ACCEL_RESPONSE)),
        gyro_bias_dps=tuple(BIAS_GYRO),
        gyro_correction_matrix=tuple(tuple(row) for row in np.linalg.inv(GYRO_RESPONSE)),
    )
    assert nonlinear_result.status is MetrologyStatus.FAIL
    assert any("残差" in finding or "R²" in finding for finding in nonlinear_result.findings)


def test_candidate_is_immutable_safe_and_strict_json_roundtrips(
    complete_samples: tuple[MetrologySample, ...],
) -> None:
    candidate = build_metrology_candidate(
        complete_samples,
        data_source="synthetic-complete-v1",
        created_at="2026-08-28T12:00:00+08:00",
    )
    assert candidate.status is MetrologyStatus.PASS
    assert candidate.candidate_calibration_generation == 8
    assert candidate.evidence_only is True
    assert candidate.applied is False
    assert candidate.flash_writes == 0
    assert candidate.flight_release is False
    with pytest.raises(FrozenInstanceError):
        candidate.applied = True  # type: ignore[misc]

    encoded = candidate_to_json(candidate)
    restored = candidate_from_json(encoded)
    assert restored == candidate

    unsafe = candidate_to_dict(candidate)
    unsafe["flash_writes"] = 1
    with pytest.raises(ValueError, match="unsafe"):
        candidate_from_dict(unsafe)

    duplicate = encoded.replace(
        '"evidence_only": true,',
        '"evidence_only": true, "evidence_only": true,',
        1,
    )
    with pytest.raises(ValueError, match="duplicate JSON key"):
        candidate_from_json(duplicate)
    payload = candidate_to_dict(candidate)
    payload["accelerometer"]["bias_g"][0] = math.nan
    with pytest.raises(ValueError, match="finite"):
        candidate_from_json(json.dumps(payload))


def test_candidate_json_rejects_semantic_status_count_and_v0_forgery(
    complete_samples: tuple[MetrologySample, ...],
) -> None:
    candidate = build_metrology_candidate(
        complete_samples,
        data_source="semantic-json-v1",
        created_at="2026-08-28T12:00:00+08:00",
    )

    v0_forgery = candidate_to_dict(candidate)
    v0_forgery["capture_methods"] = [
        "bench",
        "reference_fixture",
        "v0_imported",
    ]
    with pytest.raises(ValueError, match="V0-imported"):
        candidate_from_dict(v0_forgery)

    status_forgery = candidate_to_dict(candidate)
    status_forgery["status"] = "WARN"
    with pytest.raises(ValueError, match="status"):
        candidate_from_dict(status_forgery)

    count_forgery = candidate_to_dict(candidate)
    count_forgery["sample_count"] += 1
    with pytest.raises(ValueError, match="sample_count"):
        candidate_from_dict(count_forgery)

    determinant_forgery = candidate_to_dict(candidate)
    determinant_forgery["accelerometer"]["determinant"] = -1.0
    with pytest.raises(ValueError, match="determinant"):
        candidate_from_dict(determinant_forgery)


def test_application_validation_replays_raw_evidence_and_policy(
    complete_samples: tuple[MetrologySample, ...],
) -> None:
    candidate = build_metrology_candidate(
        complete_samples,
        data_source="application-replay-v1",
        created_at="2026-08-28T12:00:00+08:00",
    )
    assert validate_candidate_for_application(candidate, complete_samples) is candidate

    tampered = replace(candidate, findings=("tampered evidence",))
    with pytest.raises(ValueError, match="replayed raw evidence"):
        validate_candidate_for_application(tampered, complete_samples)

    changed_samples = list(complete_samples)
    first = changed_samples[0]
    changed_samples[0] = replace(first, accel_x_g=first.accel_x_g + 0.001)
    with pytest.raises(ValueError, match="replayed raw evidence"):
        validate_candidate_for_application(candidate, changed_samples)

    different_policy = replace(
        metrology.DEFAULT_THRESHOLDS,
        accel_condition_max=1.24,
    )
    different_policy_candidate = build_metrology_candidate(
        complete_samples,
        data_source="application-replay-v1",
        created_at="2026-08-28T12:00:00+08:00",
        thresholds=different_policy,
    )
    with pytest.raises(ValueError, match="application policy"):
        validate_candidate_for_application(
            different_policy_candidate,
            complete_samples,
        )


def test_room_temperature_application_replays_only_base_evidence() -> None:
    samples = tuple(accel_samples() + gyro_static_samples())
    candidate = build_metrology_candidate(
        samples,
        data_source="room-temperature-v1",
        created_at="2026-08-28T12:00:00+08:00",
    )
    assert candidate.status is not MetrologyStatus.PASS
    assert candidate.accelerometer.status is MetrologyStatus.PASS
    assert candidate.gyro_static.status is MetrologyStatus.PASS
    assert validate_room_temperature_candidate_for_application(
        candidate, samples) is candidate
    with pytest.raises(ValueError, match="fully PASS"):
        validate_candidate_for_application(candidate, samples)

    missing_gyro = tuple(accel_samples())
    incomplete = build_metrology_candidate(
        missing_gyro,
        data_source="room-temperature-v1-incomplete",
        created_at="2026-08-28T12:00:00+08:00",
    )
    with pytest.raises(ValueError, match="陀螺静止证据为 NOT_RUN"):
        validate_room_temperature_candidate_for_application(
            incomplete, missing_gyro)


def test_v0_conversion_requires_explicit_metadata_and_never_auto_passes() -> None:
    v0 = ImuSample(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    with pytest.raises(TypeError):
        metrology_sample_from_v0(  # type: ignore[call-arg]
            v0,
            stage=MetrologyStage.ACCEL_POS_X,
            temperature_c=25.0,
        )

    converted: list[MetrologySample] = []
    for stage in ACCEL_FACE_STAGES:
        expected = EXPECTED_FACE[stage]
        old = ImuSample(
            0.0,
            float(expected[0]),
            float(expected[1]),
            float(expected[2]),
            0.0,
            0.0,
            0.0,
        )
        converted.append(
            metrology_sample_from_v0(
                old,
                stage=stage,
                temperature_c=25.0,
                frame_id="FLU",
                frame_contract_version=1,
                orientation_code=3,
                calibration_generation=7,
                firmware_hash="sha256:v0-firmware-173317",
                session_id="v0-session-173317",
                capture_source="v0-report:173317",
                provenance="v0-session-173317",
            )
        )
    thresholds = replace(
        metrology.DEFAULT_THRESHOLDS,
        accel_min_samples_per_face=1,
        accel_min_duration_s=0.0,
    )
    result = fit_accelerometer(converted, thresholds)
    assert result.status is MetrologyStatus.WARN
    assert all(row.capture_method is CaptureMethod.V0_IMPORTED for row in converted)
    assert any("不能自动取得 V1 PASS" in finding for finding in result.findings)


def test_contract_and_capture_bindings_are_exact_and_uniform() -> None:
    with pytest.raises(ValueError, match="supported canonical contract"):
        replace(
            sample(MetrologyStage.ACCEL_POS_Z, timestamp_s=0.0),
            frame_contract_version=2,
        )
    with pytest.raises(ValueError, match="concrete capture binding"):
        replace(
            sample(MetrologyStage.ACCEL_POS_Z, timestamp_s=0.0),
            firmware_hash="unknown",
        )

    rows = accel_samples(count=1)
    rows[-1] = replace(rows[-1], session_id="v1-session-other")
    # 报错必须点名分叉的那一项，否则用户不知道该重采哪一步。
    with pytest.raises(ValueError, match=r"mix capture context: session_id=.*v1-session-other"):
        fit_accelerometer(rows)


def test_mixed_target_state_is_rejected_instead_of_silently_merged() -> None:
    rows = accel_samples(count=1)
    rows[-1] = replace(rows[-1], calibration_generation=8)
    with pytest.raises(ValueError, match=r"mix capture context: calibration_generation=7/8"):
        fit_accelerometer(rows)


def test_module_import_survives_without_numpy_but_analysis_is_explicit(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(metrology, "_np", None)
    with pytest.raises(metrology.NumpyRequiredError, match="pip install numpy"):
        fit_accelerometer([])


def room_temperature_evidence(*, tilt_deg: float = 0.0):
    """六面 + 陀螺静止的可应用证据；tilt_deg 制造一对正负面的摆放不一致。"""
    samples = accel_samples()
    if tilt_deg:
        samples = tilt_one_face(samples, MetrologyStage.ACCEL_NEG_Y, tilt_deg)
    samples = samples + gyro_static_samples()
    candidate = metrology.build_metrology_candidate(samples, data_source="test")
    return candidate, samples


def test_a_hand_placed_candidate_can_still_be_applied_to_the_aircraft() -> None:
    """WARN 不是"坏数据"，是"摆得不够一致"。拒掉它等于逼人做不到的事。

    2026-08-29 实况：六面 WARN（折合 1.5°），上位机按钮亮了，最后一道写机门却
    仍然抛 "six-face accelerometer evidence must PASS"。
    """
    candidate, samples = room_temperature_evidence(tilt_deg=10.0)
    assert candidate.accelerometer.status is MetrologyStatus.WARN

    accepted = metrology.validate_room_temperature_candidate_for_application(
        candidate, samples)

    assert accepted is candidate


def test_a_clean_candidate_is_still_accepted() -> None:
    candidate, samples = room_temperature_evidence()
    assert candidate.accelerometer.status is MetrologyStatus.PASS

    assert metrology.validate_room_temperature_candidate_for_application(
        candidate, samples) is candidate


def test_a_failed_six_face_fit_is_still_refused_and_says_why() -> None:
    candidate, samples = room_temperature_evidence(tilt_deg=25.0)
    assert candidate.accelerometer.status is MetrologyStatus.FAIL

    with pytest.raises(ValueError) as failure:
        metrology.validate_room_temperature_candidate_for_application(candidate, samples)

    message = str(failure.value)
    assert "六面加速度计" in message and "FAIL" in message
    assert "摆放差异过大" in message, "只说 must PASS，用户不知道该改什么"


def test_v0_imported_evidence_is_still_refused_even_though_warn_is_allowed() -> None:
    """V0 导入会把 PASS 压成 WARN —— 放行 WARN 之后这条必须仍然单独拦住。"""
    samples = accel_samples(method=CaptureMethod.V0_IMPORTED) + gyro_static_samples()
    candidate = metrology.build_metrology_candidate(samples, data_source="test")
    assert candidate.accelerometer.status is MetrologyStatus.WARN

    with pytest.raises(ValueError, match="V0-imported"):
        metrology.validate_room_temperature_candidate_for_application(candidate, samples)
