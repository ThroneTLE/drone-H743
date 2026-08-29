from __future__ import annotations

import json
import struct
from datetime import datetime, timezone
from pathlib import Path

import pytest

from tools import imu_vibration_capture as capture
from tools import imu_metrology
from tools.imu_metrology import MetrologyStage
from tools.v1_metrology_session import (
    CAPTURE_PLANS,
    DISCARDED_DIRNAME,
    analyze_session,
    discard_capture,
    load_session,
    load_session_samples,
    new_session,
    persist_capture,
    probe_capture,
    summarise_rotation,
    session_from_dict,
    session_to_dict,
)


def v4_header(**overrides: int) -> dict:
    values = {
        "magic": capture.CAPTURE_MAGIC, "version": 4, "header_size": 56,
        "session_id": 11, "total_samples": 1, "offset_samples": 0,
        "block_samples": 1, "sample_size": 48, "accel_aaf_hz": 213,
        "gyro_aaf_hz": 213, "accel_range_g": 16, "gyro_range_dps": 1000,
        "frame_contract": 1, "orientation_code": 3, "calibration_valid_mask": 0,
        "calibration_generation": 7, "base_frame": 1,
        "firmware_image_crc32": 0x1234ABCD, "flags": capture.FLAG_LAST,
        "payload_crc32": 0,
    }
    values.update(overrides)
    return values


def v4_sample(timestamp: int = 1000) -> tuple:
    return (
        timestamp, 1325,
        10, 20, 30, 40, 50, 60,
        1000, 2000, 3000, 100, 200, 300,
        0, 0, 0, 1000, 1000, 1500, 1500, 0, 2, 0,
    )


def test_v4_exact_abi_header_sample_and_temperature_conversion() -> None:
    header = v4_header()
    packed_header = struct.pack(
        capture.HEADER_FMT_V4,
        header["magic"], header["version"], header["header_size"], header["session_id"],
        header["total_samples"], header["offset_samples"], header["block_samples"],
        header["sample_size"], header["accel_aaf_hz"], header["gyro_aaf_hz"],
        header["accel_range_g"], header["gyro_range_dps"], header["frame_contract"],
        header["orientation_code"], header["calibration_valid_mask"],
        header["calibration_generation"], header["base_frame"], header["firmware_image_crc32"],
        header["flags"], header["payload_crc32"],
    )
    parsed = capture.parse_header(packed_header)
    assert struct.calcsize(capture.HEADER_FMT_V4) == 56
    assert struct.calcsize(capture.SAMPLE_FMT_V4) == 48
    assert parsed["firmware_image_crc32"] == 0x1234ABCD
    payload = struct.pack(capture.SAMPLE_FMT_V4, *v4_sample())
    samples = capture.decode_samples(payload, 1, version=4)
    rows = capture.samples_to_rows(samples, parsed)
    assert rows[0]["temperature_c"] == pytest.approx(1325 / 132.48 + 25.0)
    assert (rows[0]["accel_filt_x_g"], rows[0]["accel_filt_y_g"], rows[0]["accel_filt_z_g"]) == (-1.0, -2.0, 3.0)
    assert (rows[0]["gyro_filt_x_dps"], rows[0]["gyro_filt_y_dps"], rows[0]["gyro_filt_z_dps"]) == (-1.0, -2.0, 3.0)


def test_v3_40_46_decoder_remains_read_compatible_but_not_v1() -> None:
    header_values = (capture.CAPTURE_MAGIC, 3, 40, 1, 1, 0, 1, 46, 213, 213, 16, 1000, 1, 0)
    parsed = capture.parse_header(struct.pack(capture.HEADER_FMT_V3, *header_values))
    assert parsed["version"] == 3 and parsed["orientation_code"] is None
    sample = (1000, 10, 20, 30, 40, 50, 60, 1000, 0, 0, 100, 0, 0, 0, 0, 0, 1000, 1000, 1500, 1500, 0, 2, 0)
    decoded = capture.decode_samples(struct.pack(capture.SAMPLE_FMT_V3, *sample), 1, version=3)
    assert capture.samples_to_rows(decoded, parsed)[0]["temperature_c"] == ""
    with pytest.raises(ValueError, match="v4"):
        capture.validate_v1_provenance(parsed)


@pytest.mark.parametrize("change", [
    {"flags": capture.FLAG_INVALID_PROVENANCE}, {"frame_contract": 2},
    {"orientation_code": 24}, {"calibration_generation": 0},
    {"firmware_image_crc32": 0}, {"base_frame": 2},
])
def test_v1_rejects_invalid_v4_provenance(change: dict[str, int]) -> None:
    with pytest.raises(ValueError):
        capture.validate_v1_provenance(v4_header(**change))


def test_session_manifest_capture_reload_and_incomplete_candidate(tmp_path: Path) -> None:
    session, manifest = new_session(now=datetime(2026, 8, 28, 12, 0, tzinfo=timezone.utc), root=tmp_path)
    assert manifest.parent.parent.name == "2026-08-28"
    updated, record = persist_capture(
        session, manifest, stage=MetrologyStage.ACCEL_POS_Z,
        samples=[v4_sample(1000), v4_sample(2000)], header=v4_header(total_samples=2, block_samples=2),
    )
    restored = load_session(manifest)
    assert restored == updated and record.sample_count == 2
    samples = load_session_samples(restored, manifest)
    assert samples[0].firmware_hash == "crc32:1234abcd"
    assert samples[0].session_id == session.session_id
    assert samples[0].capture_source == f"imucap-v4-session:{session.session_id}"
    summary = analyze_session(restored, manifest)
    assert "V1 室温基础候选未通过" in summary.status_line
    # 只报流程里真有的两组；转动/温度已移除，不再当作"待完成"挂在结论里。
    assert "六面=" in summary.status_line and "陀螺静止=" in summary.status_line
    assert "转台" not in summary.status_line and "温点" not in summary.status_line
    assert summary.candidate_path.is_file() and summary.references_path.is_file()


def capture_stage(session, manifest, stage, *, count, step_us=1000, platform=None):
    samples = [v4_sample(1000 + index * step_us) for index in range(count)]
    return persist_capture(
        session, manifest, stage=stage, samples=samples,
        header=v4_header(total_samples=count, block_samples=count),
        temperature_platform=platform,
    )


def test_recapturing_a_stage_replaces_the_previous_evidence(tmp_path: Path) -> None:
    session, manifest = new_session(now=datetime(2026, 8, 29, tzinfo=timezone.utc), root=tmp_path)
    session, first = capture_stage(session, manifest, MetrologyStage.ACCEL_POS_X, count=3)
    session, second = capture_stage(session, manifest, MetrologyStage.ACCEL_POS_X, count=5)

    assert [record.csv_path for record in session.captures] == [second.csv_path]
    assert session.captures[0].sample_count == 5
    assert not (manifest.parent / first.csv_path).exists()
    # 旧证据只是挪走，误替换一次仍可找回。
    assert (manifest.parent / DISCARDED_DIRNAME / first.csv_path).is_file()
    assert (manifest.parent / DISCARDED_DIRNAME / first.meta_path).is_file()
    assert load_session(manifest) == session


@pytest.mark.parametrize("stage", [
    MetrologyStage.GYRO_POS_360_X, MetrologyStage.GYRO_POS_360_Y,
    MetrologyStage.GYRO_POS_360_Z, MetrologyStage.TEMPERATURE_STATIC,
])
def test_retired_stages_cannot_take_new_evidence(tmp_path: Path, stage) -> None:
    """手转 360°/温度平台已从流程移除：老数据读得出来，但不能再采新的。"""
    session, manifest = new_session(now=datetime(2026, 8, 29, tzinfo=timezone.utc), root=tmp_path)

    assert stage not in {plan.stage for plan in CAPTURE_PLANS}
    with pytest.raises(ValueError, match="已从 V1 采集流程移除"):
        capture_stage(session, manifest, stage, count=3)


def test_the_flow_is_exactly_the_six_faces_plus_the_static_gyro() -> None:
    assert [plan.stage for plan in CAPTURE_PLANS] == [
        MetrologyStage.ACCEL_POS_X, MetrologyStage.ACCEL_NEG_X,
        MetrologyStage.ACCEL_POS_Y, MetrologyStage.ACCEL_NEG_Y,
        MetrologyStage.ACCEL_POS_Z, MetrologyStage.ACCEL_NEG_Z,
        MetrologyStage.GYRO_STATIC,
    ]


def test_discarding_a_capture_keeps_the_raw_csv_recoverable(tmp_path: Path) -> None:
    session, manifest = new_session(now=datetime(2026, 8, 29, tzinfo=timezone.utc), root=tmp_path)
    session, record = capture_stage(session, manifest, MetrologyStage.GYRO_STATIC, count=4)

    session, removed = discard_capture(session, manifest, csv_path=record.csv_path)

    assert removed.csv_path == record.csv_path and session.captures == ()
    assert (manifest.parent / DISCARDED_DIRNAME / record.csv_path).is_file()
    assert load_session(manifest).captures == ()
    with pytest.raises(ValueError, match="has no capture"):
        discard_capture(session, manifest, csv_path=record.csv_path)


def test_probe_flags_a_degraded_capture_before_analysis_is_attempted(tmp_path: Path) -> None:
    session, manifest = new_session(now=datetime(2026, 8, 29, tzinfo=timezone.utc), root=tmp_path)
    # 20 ms 步进 = DRDY 失效后退到轮询兜底的 50 Hz。
    session, degraded = capture_stage(
        session, manifest, MetrologyStage.ACCEL_POS_Z, count=150, step_us=20_000)

    health = probe_capture(manifest, degraded)

    assert health.level == "rate"
    assert round(health.rate_hz) == 50
    assert "删除重采" in health.detail


def test_probe_separates_a_short_capture_from_a_degraded_one(tmp_path: Path) -> None:
    session, manifest = new_session(now=datetime(2026, 8, 29, tzinfo=timezone.utc), root=tmp_path)
    session, short = capture_stage(session, manifest, MetrologyStage.ACCEL_POS_Z, count=200)
    session, enough = capture_stage(session, manifest, MetrologyStage.GYRO_STATIC, count=4200)

    assert probe_capture(manifest, short).level == "short"
    assert probe_capture(manifest, short).required_samples == 1500
    assert probe_capture(manifest, enough).level == "ok"


def test_analysis_failure_names_the_capture_that_caused_it(tmp_path: Path) -> None:
    session, manifest = new_session(now=datetime(2026, 8, 29, tzinfo=timezone.utc), root=tmp_path)
    session, degraded = capture_stage(
        session, manifest, MetrologyStage.ACCEL_POS_Z, count=150, step_us=20_000)

    with pytest.raises(ValueError) as failure:
        load_session_samples(session, manifest)

    assert degraded.csv_path in str(failure.value)
    assert "accel_pos_z" in str(failure.value)


def rotation_rows(profile, *, rate_hz: float = 1000.0, axis: int = 0):
    """把 (每样本角速度) 序列铺成 IMUCAP CSV 行；非主轴恒为 0。"""
    rows = []
    for index, value in enumerate(profile):
        rates = [0.0, 0.0, 0.0]
        rates[axis] = value
        rows.append({
            "timestamp_us": str(int(index * 1e6 / rate_hz)),
            "gyro_filt_x_dps": repr(rates[0]),
            "gyro_filt_y_dps": repr(rates[1]),
            "gyro_filt_z_dps": repr(rates[2]),
        })
    return rows


def test_the_motion_window_trims_the_still_head_and_tail() -> None:
    numpy = pytest.importorskip("numpy")
    # 前后各 1 s 静止，中间 4 s 以 90°/s 转动。
    rates = numpy.zeros((6000, 3))
    rates[1000:5000, 0] = 90.0

    start, stop = imu_metrology.gyro_rotation_motion_window(rates, 0)

    assert (start, stop) == (1000, 5000)


def test_a_mid_turn_pause_stays_inside_the_window() -> None:
    numpy = pytest.importorskip("numpy")
    rates = numpy.zeros((6000, 3))
    rates[1000:2500, 0] = 90.0
    rates[3500:5000, 0] = 90.0  # 中途停了 1 s 又继续转

    start, stop = imu_metrology.gyro_rotation_motion_window(rates, 0)

    # 剔掉停顿会把总角度算少：这仍然是同一次转动。
    assert (start, stop) == (1000, 5000)


def test_still_samples_do_not_add_bias_drift_to_the_angle() -> None:
    pytest.importorskip("numpy")
    turn = [90.0] * 4000                       # 4 s × 90°/s = 360°
    rows = rotation_rows([0.0] * 1000 + turn + [0.0] * 1000)

    level, detail = summarise_rotation(rows, 0)

    assert level == "ok"
    assert "转过 +360°" in detail
    assert "运动段 4.00s" in detail


def test_an_incomplete_turn_reports_how_much_is_missing() -> None:
    pytest.importorskip("numpy")
    # 复现 2026-08-29 那次：整段都在转，只转到 300° 就被窗口截断。
    rows = rotation_rows([50.0] * 6000)

    level, detail = summarise_rotation(rows, 0)

    assert level == "angle"
    assert "转过 +300°" in detail
    assert "缺 +60°" in detail and "建议重采" in detail


def test_a_wobbly_turn_is_flagged_on_cross_axis(tmp_path: Path) -> None:
    pytest.importorskip("numpy")
    rows = rotation_rows([90.0] * 4000)          # 4 s × 90°/s = 360°
    for row in rows:
        row["gyro_filt_y_dps"] = repr(4.0)       # 4 s × 4°/s = 16°，占 4.4%
    assert summarise_rotation(rows, 0)[0] == "ok"

    for row in rows:
        row["gyro_filt_y_dps"] = repr(12.0)      # 4 s × 12°/s = 48°，占 13.3%
    level, detail = summarise_rotation(rows, 0)

    assert level == "angle" and "离轴 13.3% 超 10%" in detail


def test_an_archived_rotation_capture_still_reports_its_angle() -> None:
    """流程里没有了，但老会话的转动证据仍要读得出结论。"""
    pytest.importorskip("numpy")
    rows = rotation_rows([50.0] * 6000)   # 复现 2026-08-29：整段都在转，只到 300°

    level, detail = summarise_rotation(rows, 0)

    assert level == "angle" and "转过 +300°" in detail


def test_session_json_rejects_enabling_target_writes(tmp_path: Path) -> None:
    session, _path = new_session(now=datetime(2026, 8, 28, tzinfo=timezone.utc), root=tmp_path)
    payload = session_to_dict(session)
    payload["target_apply_enabled"] = True
    with pytest.raises(ValueError, match="unsafe"):
        session_from_dict(payload)
