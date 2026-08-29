from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


HARNESS = r"""
#include "app_flight_calibration.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { if (!(condition)) return (code); } while (0)

static void identity(float matrix[3][3])
{
    memset(matrix, 0, 9U * sizeof(float));
    matrix[0][0] = 1.0f;
    matrix[1][1] = 1.0f;
    matrix[2][2] = 1.0f;
}

static void candidate_defaults(APP_FlightCalibrationV1Candidate *candidate)
{
    memset(candidate, 0, sizeof(*candidate));
    candidate->magic = APP_FLIGHT_CAL_V1_CANDIDATE_MAGIC;
    candidate->schema = APP_FLIGHT_CAL_V1_CANDIDATE_SCHEMA;
    candidate->size = (uint16_t)sizeof(*candidate);
    candidate->base_generation = 7U;
    candidate->frame_contract = 1U;
    candidate->orientation_code = 3U;
    candidate->valid_mask = APP_FLIGHT_CAL_VALID_ORIENTATION |
                            APP_FLIGHT_CAL_VALID_ACCEL |
                            APP_FLIGHT_CAL_VALID_GYRO |
                            APP_FLIGHT_CAL_VALID_GYRO_TEMP;
    identity(candidate->accel_correction);
    identity(candidate->gyro_correction);
    candidate->accel_bias[0] = 0.01f;
    candidate->gyro_bias_ref[1] = -0.02f;
    candidate->gyro_temp_slope[2] = 0.001f;
    candidate->reference_temp_c = 25.0f;
}

static void to_hex(const uint8_t *bytes, uint32_t size, char *hex)
{
    static const char digits[] = "0123456789abcdef";
    uint32_t index;
    for (index = 0U; index < size; ++index) {
        hex[index * 2U] = digits[bytes[index] >> 4U];
        hex[index * 2U + 1U] = digits[bytes[index] & 0x0FU];
    }
    hex[size * 2U] = '\0';
}

static int upload_all(APP_FlightCalibrationUpload *upload,
                      const APP_FlightCalibrationV1Candidate *candidate,
                      uint32_t crc,
                      uint32_t now)
{
    const uint8_t *bytes = (const uint8_t *)candidate;
    char hex[APP_FLIGHT_CAL_V1_CHUNK_MAX_BYTES * 2U + 1U];
    uint32_t offset;

    if (APP_FlightCalibration_UploadBegin(upload, sizeof(*candidate),
                                          crc, now) !=
        APP_FLIGHT_CAL_TRANSFER_OK) return 0;
    for (offset = 0U; offset < sizeof(*candidate);
         offset += APP_FLIGHT_CAL_V1_CHUNK_MAX_BYTES) {
        to_hex(&bytes[offset], APP_FLIGHT_CAL_V1_CHUNK_MAX_BYTES, hex);
        if (APP_FlightCalibration_UploadDataHex(upload, offset, hex, ++now) !=
            APP_FLIGHT_CAL_TRANSFER_OK) return 0;
    }
    return APP_FlightCalibration_UploadEnd(upload, ++now) ==
           APP_FLIGHT_CAL_TRANSFER_OK;
}

int main(void)
{
    APP_FlightCalibrationV1Candidate candidate;
    APP_FlightCalibrationV1Candidate invalid;
    APP_FlightCalibrationUpload upload;
    APP_FlightCalibration base;
    APP_FlightCalibration merged;
    APP_FlightCalibrationSnapshot active;
    char chunk[67];
    uint32_t crc;

    CHECK(sizeof(candidate) == 128U, 1);
    candidate_defaults(&candidate);
    CHECK(APP_FlightCalibration_ValidateV1Candidate(&candidate) ==
          APP_FLIGHT_CAL_TRANSFER_OK, 2);
    crc = APP_FlightCalibration_Crc32((const uint8_t *)&candidate,
                                      sizeof(candidate));

    APP_FlightCalibration_UploadReset(&upload);
    CHECK(APP_FlightCalibration_UploadBegin(&upload, 129U, crc, 0U) ==
          APP_FLIGHT_CAL_TRANSFER_OVERSIZE, 3);
    CHECK(APP_FlightCalibration_UploadBegin(&upload, 128U, crc, 0U) ==
          APP_FLIGHT_CAL_TRANSFER_OK, 4);
    CHECK(APP_FlightCalibration_UploadDataHex(&upload, 1U, "00", 1U) ==
          APP_FLIGHT_CAL_TRANSFER_GAP, 5);

    CHECK(APP_FlightCalibration_UploadBegin(&upload, 128U, crc, 0U) ==
          APP_FLIGHT_CAL_TRANSFER_OK, 6);
    CHECK(APP_FlightCalibration_UploadDataHex(&upload, 0U, "0g", 1U) ==
          APP_FLIGHT_CAL_TRANSFER_BAD_HEX, 7);

    memset(chunk, '0', 66U); chunk[66] = '\0';
    CHECK(APP_FlightCalibration_UploadBegin(&upload, 128U, crc, 0U) ==
          APP_FLIGHT_CAL_TRANSFER_OK, 8);
    CHECK(APP_FlightCalibration_UploadDataHex(&upload, 0U, chunk, 1U) ==
          APP_FLIGHT_CAL_TRANSFER_OVERSIZE, 9);

    to_hex((const uint8_t *)&candidate, 32U, chunk);
    CHECK(APP_FlightCalibration_UploadBegin(&upload, 128U, crc, 0U) ==
          APP_FLIGHT_CAL_TRANSFER_OK, 10);
    CHECK(APP_FlightCalibration_UploadDataHex(&upload, 0U, chunk, 1U) ==
          APP_FLIGHT_CAL_TRANSFER_OK, 11);
    CHECK(APP_FlightCalibration_UploadDataHex(&upload, 0U, chunk, 2U) ==
          APP_FLIGHT_CAL_TRANSFER_OVERLAP, 12);

    CHECK(APP_FlightCalibration_UploadBegin(&upload, 128U, crc, 0U) ==
          APP_FLIGHT_CAL_TRANSFER_OK, 13);
    CHECK(APP_FlightCalibration_UploadDataHex(
              &upload, 0U, chunk, APP_FLIGHT_CAL_V1_UPLOAD_TIMEOUT_MS) ==
          APP_FLIGHT_CAL_TRANSFER_TIMEOUT, 14);

    CHECK(upload_all(&upload, &candidate, crc ^ 1U, 10U) == 0, 15);
    CHECK(upload.state == APP_FLIGHT_CAL_UPLOAD_EMPTY, 16);
    CHECK(upload_all(&upload, &candidate, crc, 100U) != 0, 17);
    CHECK(upload.state == APP_FLIGHT_CAL_UPLOAD_READY, 18);
    CHECK(upload.candidate.base_generation == 7U, 19);
    CHECK(APP_FlightCalibration_UploadExpire(&upload, 5104U) == 0U, 20);
    CHECK(APP_FlightCalibration_UploadExpire(&upload, 5105U) == 1U, 21);

    invalid = candidate;
    invalid.accel_bias[0] = NAN;
    CHECK(APP_FlightCalibration_ValidateV1Candidate(&invalid) ==
          APP_FLIGHT_CAL_TRANSFER_NONFINITE, 22);
    invalid = candidate;
    invalid.accel_correction[0][0] = 1.20f;
    CHECK(APP_FlightCalibration_ValidateV1Candidate(&invalid) ==
          APP_FLIGHT_CAL_TRANSFER_BAD_MATRIX, 23);
    invalid = candidate;
    invalid.orientation_code = 24U;
    CHECK(APP_FlightCalibration_ValidateV1Candidate(&invalid) ==
          APP_FLIGHT_CAL_TRANSFER_BAD_ORIENTATION, 24);
    invalid = candidate;
    invalid.valid_mask = APP_FLIGHT_CAL_VALID_ORIENTATION |
                         APP_FLIGHT_CAL_VALID_GYRO_TEMP;
    CHECK(APP_FlightCalibration_ValidateV1Candidate(&invalid) ==
          APP_FLIGHT_CAL_TRANSFER_BAD_VALID_MASK, 25);
    invalid = candidate;
    invalid.accel_bias[0] = 0.51f;
    CHECK(APP_FlightCalibration_ValidateV1Candidate(&invalid) ==
          APP_FLIGHT_CAL_TRANSFER_BAD_RANGE, 38);
    invalid = candidate;
    invalid.gyro_bias_ref[0] = 20.01f;
    CHECK(APP_FlightCalibration_ValidateV1Candidate(&invalid) ==
          APP_FLIGHT_CAL_TRANSFER_BAD_RANGE, 39);
    invalid = candidate;
    invalid.gyro_temp_slope[0] = 2.01f;
    CHECK(APP_FlightCalibration_ValidateV1Candidate(&invalid) ==
          APP_FLIGHT_CAL_TRANSFER_BAD_RANGE, 40);
    invalid = candidate;
    invalid.reference_temp_c = 85.01f;
    CHECK(APP_FlightCalibration_ValidateV1Candidate(&invalid) ==
          APP_FLIGHT_CAL_TRANSFER_BAD_RANGE, 41);
    invalid = candidate;
    invalid.valid_mask |= APP_FLIGHT_CAL_VALID_SERVO_MECHANICAL;
    CHECK(APP_FlightCalibration_ValidateV1Candidate(&invalid) ==
          APP_FLIGHT_CAL_TRANSFER_BAD_VALID_MASK, 49);

    APP_FlightCalibration_Defaults(&base);
    CHECK(APP_FlightCalibration_UpdateOrientation(&base, 3U) != 0U, 26);
    base.v2_navigation_mapping = 11U;
    base.v2_controller_mapping = 12U;
    base.v2_rc_mapping = 13U;
    base.v2_actuator_mapping = 14U;
    base.v2_reserved[2] = 0x12345678UL;
    CHECK(APP_FlightCalibration_MergeV1Candidate(&base, &candidate,
                                                  &merged) != 0U, 27);
    CHECK(merged.orientation_code == base.orientation_code, 28);
    CHECK(merged.v2_navigation_mapping == 11U &&
          merged.v2_controller_mapping == 12U &&
          merged.v2_rc_mapping == 13U &&
          merged.v2_actuator_mapping == 14U &&
          merged.v2_reserved[2] == 0x12345678UL, 29);
    CHECK(merged.calibration_generation == 8U, 30);
    CHECK(merged.accel_bias[0] == candidate.accel_bias[0], 31);
    invalid = candidate;
    invalid.orientation_code = 4U;
    CHECK(APP_FlightCalibration_MergeV1Candidate(&base, &invalid,
                                                  &merged) == 0U, 32);

    APP_FlightCalibration_ResetActive();
    CHECK(APP_FlightCalibration_PublishConfirmed(&base) != 0U, 33);
    CHECK(APP_FlightCalibration_ReadActive(&active) != 0U, 34);
    {
        uint32_t confirmed_generation = active.generation;
        CHECK(APP_FlightCalibration_PublishPreview(&base) != 0U, 35);
        CHECK(APP_FlightCalibration_ReadActive(&active) != 0U, 36);
        CHECK(active.generation != confirmed_generation, 37);
    }

    puts("ok");
    return 0;
}
"""


def test_candidate_upload_state_machine_runs_on_host(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host C compiler is unavailable")
    (tmp_path / "app_sensor.h").write_text(
        "#define APP_SENSOR_FLU_ORIENTATION_COUNT 24U\n"
        "#define APP_SENSOR_FLU_ORIENTATION_LEGACY 255U\n",
        encoding="ascii",
    )
    (tmp_path / "drv_frame_contract.h").write_text(
        "#define DRV_FRAME_CONTRACT_VERSION 1U\n", encoding="ascii"
    )
    harness = tmp_path / "imucal_candidate.c"
    executable = tmp_path / "imucal_candidate.exe"
    harness.write_text(HARNESS, encoding="ascii")
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{tmp_path}",
            f"-I{ROOT / 'App' / 'Inc'}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(ROOT / "App" / "Src" / "app_flight_calibration.c"),
            str(harness),
            "-lm",
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    result = subprocess.run(
        [str(executable)], check=True, capture_output=True, text=True
    )
    assert result.stdout.strip() == "ok"


def test_ascii_protocol_is_bounded_guarded_and_machine_parseable() -> None:
    source = read("App/Src/app_control.c")
    proto = read("App/Inc/app_proto.h")

    assert "#define APP_PROTO_REQ_IMU_CAL        0x1020U" in proto
    assert "#define APP_PROTO_MSG_IMU_CAL           0x2221U" in proto
    for command in ("BEGIN", "DATA", "END", "APPLY", "REVERT", "COMMIT"):
        assert f'strcmp(tokens[1], "{command}")' in source
    for usage in (
        "IMUCAL BEGIN size=<n> crc=<8hex>",
        "IMUCAL DATA offset=<n> hex=<max64hex>",
        "IMUCAL END",
        "IMUCAL APPLY",
        "IMUCAL REVERT",
        "IMUCAL COMMIT",
        "IMUCAL?",
    ):
        assert usage in source
    assert "APP_FLIGHT_CAL_V1_CHUNK_MAX_BYTES  32U" in read(
        "App/Inc/app_flight_calibration.h"
    )
    assert "APP_FLIGHT_CAL_V1_UPLOAD_TIMEOUT_MS 5000U" in read(
        "App/Inc/app_flight_calibration.h"
    )
    assert len(
        "IMUCAL DATA offset=4294967295 hex=" + "f" * 64 + "\r\n"
    ) < 128
    for field in (
        "event=%s",
        "reason=%s",
        "transfer=%s",
        "received=%lu",
        "expected=%lu",
        "candidate=%u",
        "applied=%u",
        "commit_pending=%u",
        "dirty=%u",
        "arm_lock=%u",
        "active=%lu",
        "persisted=%lu",
        "base=%lu",
    ):
        assert field in source


def test_apply_revert_commit_obey_runtime_and_flash_boundaries() -> None:
    source = read("App/Src/app_control.c")
    handler_start = source.index(
        "static void app_control_handle_imucal(char **tokens, uint32_t count)\n{"
    )
    handler = source[handler_start:source.index("static void app_control_service_imucal", handler_start)]
    apply = handler[handler.index('strcmp(tokens[1], "APPLY")'):
                    handler.index('strcmp(tokens[1], "REVERT")')]
    revert = handler[handler.index('strcmp(tokens[1], "REVERT")'):
                     handler.index('strcmp(tokens[1], "COMMIT")')]
    commit = handler[handler.index('strcmp(tokens[1], "COMMIT")'):]

    assert "APP_FlightCalibration_PublishPreview" in apply
    assert "SVC_Param_SetBlob" not in apply
    assert "APP_FlightCalibration_PublishPreview" in revert
    assert "app_control_imucal_clear_candidate" in revert
    assert "APP_Boot_HasSequenceAdvanced" in source
    assert "APP_CONTROL_IMUCAL_SNAPSHOT_MAX_AGE_US 100000ULL" in source
    assert "snapshot->armed != 0U" in source
    assert "snapshot->esc_pulse_us[0]" in source
    assert "snapshot->esc_pulse_us[1]" in source
    assert "safety_snapshot.calibration_generation" in commit
    assert "APP_FlightCalibration_MergeV1Candidate" in commit
    assert commit.index("SVC_Param_GetBlob") < commit.index(
        "APP_FlightCalibration_MergeV1Candidate"
    )
    assert commit.index("SVC_Param_SetBlob") < commit.index(
        "SVC_Param_RequestSaveBlob"
    )
    assert "control_imucal_pending_record" in commit
    assert "control_imucal_commit_pending = 1U" in commit


def test_candidate_hard_locks_arm_and_general_save_cannot_persist_preview() -> None:
    stabilizer_h = read("App/Inc/app_stabilizer.h")
    stabilizer_c = read("App/Src/app_stabilizer.c")
    control = read("App/Src/app_control.c")

    assert "APP_Stabilizer_SetImuCalibrationCandidateArmLock" in stabilizer_h
    arm = stabilizer_c[stabilizer_c.index("uint8_t APP_Stabilizer_IsImuFrameArmLocked"):
                       stabilizer_c.index("void APP_Stabilizer_SetImuCalibrationCandidateArmLock")]
    assert "stabilizer_imu_calibration_candidate_arm_lock" in arm
    assert "APP_Stabilizer_SetImuCalibrationCandidateArmLock(1U)" in control
    assert "APP_Stabilizer_SetImuCalibrationCandidateArmLock(0U)" in control
    assert 'app_control_report_imuframe("imucal_busy", 0U)' in control

    save_start = control.index('} else if (strcmp(tokens[0], "SAVE") == 0)')
    save_end = control.index('} else if (strcmp(tokens[0], "LOAD") == 0)', save_start)
    save = control[save_start:save_end]
    assert "APP_FlightCalibration" not in save
    assert "SVC_Param" not in save
    assert "control_imucal_preview" not in save


def test_protocol_is_transport_not_a_claim_of_v1_verification() -> None:
    source = read("App/Src/app_control.c")

    assert "must already have accepted validate_candidate_for_application" in source
    assert "flight_release" not in source[source.index("static void app_control_handle_imucal"):]
    assert "verified=1" not in source[source.index("static void app_control_handle_imucal"):]
