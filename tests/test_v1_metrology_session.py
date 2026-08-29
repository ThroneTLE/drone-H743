from __future__ import annotations

import json
import struct
from datetime import datetime, timezone
from pathlib import Path

import pytest

from tools import imu_vibration_capture as capture
from tools.imu_metrology import MetrologyStage
from tools.v1_metrology_session import (
    analyze_session,
    load_session,
    load_session_samples,
    new_session,
    persist_capture,
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
    assert "V1 室温基础候选" in summary.status_line
    assert "尚未完成" in summary.status_line
    assert summary.candidate_path.is_file() and summary.references_path.is_file()


def test_session_json_rejects_enabling_target_writes(tmp_path: Path) -> None:
    session, _path = new_session(now=datetime(2026, 8, 28, tzinfo=timezone.utc), root=tmp_path)
    payload = session_to_dict(session)
    payload["target_apply_enabled"] = True
    with pytest.raises(ValueError, match="unsafe"):
        session_from_dict(payload)
