from __future__ import annotations

import re
import shutil
import struct
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


DRIVER_HARNESS = r"""
#include "drv_imu_calibration.h"

#include <math.h>
#include <stdint.h>
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

static int closef(float a, float b)
{
    return fabsf(a - b) < 0.0001f;
}

int main(void)
{
    DRV_IMU_Calibration cal;
    float accel[3] = {2.0f, 4.0f, 6.0f};
    float gyro[3] = {1.2f, 2.0f, 3.2f};
    float original_accel[3];
    float original_gyro[3];
    uint8_t applied;

    memset(&cal, 0, sizeof(cal));
    memcpy(original_accel, accel, sizeof(accel));
    memcpy(original_gyro, gyro, sizeof(gyro));
    applied = DRV_IMU_Calibration_Apply(&cal, NAN, accel, gyro);
    CHECK(applied == 0U, 1);
    CHECK(memcmp(accel, original_accel, sizeof(accel)) == 0, 2);
    CHECK(memcmp(gyro, original_gyro, sizeof(gyro)) == 0, 3);

    cal.valid_mask = DRV_IMU_CAL_VALID_ACCEL |
                     DRV_IMU_CAL_VALID_GYRO |
                     DRV_IMU_CAL_VALID_GYRO_TEMP;
    cal.accel_bias_g[0] = 1.0f;
    cal.accel_bias_g[1] = 2.0f;
    cal.accel_bias_g[2] = 3.0f;
    identity(cal.accel_correction);
    cal.accel_correction[0][0] = 2.0f;
    cal.accel_correction[0][1] = 0.1f;
    cal.accel_correction[1][1] = 3.0f;
    cal.accel_correction[2][2] = 4.0f;
    cal.gyro_bias_ref_dps[0] = 0.1f;
    cal.gyro_bias_ref_dps[1] = -0.2f;
    cal.gyro_bias_ref_dps[2] = 0.3f;
    identity(cal.gyro_correction);
    cal.gyro_correction[0][0] = 1.1f;
    cal.gyro_correction[1][1] = 0.9f;
    cal.gyro_temp_slope_dps_per_c[0] = 0.01f;
    cal.gyro_temp_slope_dps_per_c[1] = 0.02f;
    cal.gyro_temp_slope_dps_per_c[2] = -0.01f;
    cal.reference_temp_c = 25.0f;
    applied = DRV_IMU_Calibration_Apply(&cal, 35.0f, accel, gyro);
    CHECK(applied == DRV_IMU_CAL_VALID_MASK, 4);
    CHECK(closef(accel[0], 2.2f) && closef(accel[1], 6.0f) &&
          closef(accel[2], 12.0f), 5);
    CHECK(closef(gyro[0], 1.1f) && closef(gyro[1], 1.8f) &&
          closef(gyro[2], 3.0f), 6);

    /* A malformed enabled accel branch is ignored; valid gyro still applies. */
    accel[0] = 7.0f; accel[1] = 8.0f; accel[2] = 9.0f;
    gyro[0] = 1.2f; gyro[1] = 2.0f; gyro[2] = 3.2f;
    memcpy(original_accel, accel, sizeof(accel));
    cal.accel_correction[0][0] = NAN;
    applied = DRV_IMU_Calibration_Apply(&cal, 35.0f, accel, gyro);
    CHECK(applied == (DRV_IMU_CAL_VALID_GYRO |
                      DRV_IMU_CAL_VALID_GYRO_TEMP), 7);
    CHECK(memcmp(accel, original_accel, sizeof(accel)) == 0, 8);

    /* Temperature-only validity has no independent physical operation. */
    memset(&cal, 0, sizeof(cal));
    cal.valid_mask = DRV_IMU_CAL_VALID_GYRO_TEMP;
    memcpy(original_accel, accel, sizeof(accel));
    memcpy(original_gyro, gyro, sizeof(gyro));
    CHECK(DRV_IMU_Calibration_Apply(&cal, 30.0f, accel, gyro) == 0U, 9);
    CHECK(memcmp(accel, original_accel, sizeof(accel)) == 0, 10);
    CHECK(memcmp(gyro, original_gyro, sizeof(gyro)) == 0, 11);

    puts("ok");
    return 0;
}
"""


ACTIVE_SNAPSHOT_HARNESS = r"""
#include "app_flight_calibration.h"

#include <math.h>
#include <stdio.h>

#define CHECK(condition, code) do { if (!(condition)) return (code); } while (0)

int main(void)
{
    APP_FlightCalibration cal;
    APP_FlightCalibration bad;
    APP_FlightCalibrationSnapshot snapshot;
    DRV_IMU_Calibration imu;

    APP_FlightCalibration_ResetActive();
    CHECK(APP_FlightCalibration_ReadActive(&snapshot) == 1U, 1);
    CHECK(snapshot.generation == 0U, 2);
    CHECK(snapshot.calibration.valid_mask == 0U, 3);

    APP_FlightCalibration_Defaults(&cal);
    CHECK(APP_FlightCalibration_UpdateOrientation(&cal, 3U) == 1U, 4);
    cal.valid_mask |= APP_FLIGHT_CAL_VALID_ACCEL;
    cal.accel_bias[0] = 0.01f;
    cal.calibration_generation = 7U;
    CHECK(APP_FlightCalibration_PublishConfirmed(&cal) == 1U, 5);
    CHECK(APP_FlightCalibration_ReadActive(&snapshot) == 1U, 6);
    CHECK(snapshot.generation == 7U, 7);
    CHECK(snapshot.calibration.accel_bias[0] == 0.01f, 8);
    CHECK(APP_FlightCalibration_BuildImuCalibration(
              &snapshot.calibration, 3U, &imu) ==
          (APP_FLIGHT_CAL_VALID_ORIENTATION | APP_FLIGHT_CAL_VALID_ACCEL), 9);
    CHECK(imu.valid_mask == DRV_IMU_CAL_VALID_ACCEL, 10);
    CHECK(APP_FlightCalibration_BuildImuCalibration(
              &snapshot.calibration, 4U, &imu) == 0U, 11);
    CHECK(imu.valid_mask == 0U, 12);

    /* Re-publishing identical bytes is not a new runtime epoch. */
    CHECK(APP_FlightCalibration_PublishConfirmed(&cal) == 1U, 13);
    CHECK(APP_FlightCalibration_GetActiveGeneration() == 7U, 14);

    bad = cal;
    bad.accel_bias[0] = NAN;
    CHECK(APP_FlightCalibration_PublishConfirmed(&bad) == 0U, 15);
    CHECK(APP_FlightCalibration_GetActiveGeneration() == 7U, 16);

    /* Changed content with a stale record generation still advances runtime. */
    cal.accel_bias[0] = 0.02f;
    CHECK(APP_FlightCalibration_PublishConfirmed(&cal) == 1U, 17);
    CHECK(APP_FlightCalibration_GetActiveGeneration() == 8U, 18);

    puts("ok");
    return 0;
}
"""


FIRMWARE_IDENTITY_HARNESS = r"""
#include "app_firmware_identity.h"

#include <stdint.h>
#include <stdio.h>

/* Satisfy the firmware-only linker symbol; Get() is not called on the host. */
const uint8_t __data_source_end = 0U;

int main(void)
{
    static const uint8_t canonical[] = "123456789";
    static const uint8_t changed[] = "123456788";
    uint32_t first = APP_FirmwareIdentity_ComputeCrc32(canonical, 9U);
    uint32_t second = APP_FirmwareIdentity_ComputeCrc32(changed, 9U);

    if (first != 0xCBF43926UL) return 1;
    if (second == first) return 2;
    if (APP_FirmwareIdentity_IsRangeValid(0x08000000UL,
                                           0x08050000UL) == 0U) return 3;
    if (APP_FirmwareIdentity_IsRangeValid(0x08000004UL,
                                           0x08050000UL) != 0U) return 4;
    if (APP_FirmwareIdentity_IsRangeValid(0x08000000UL,
                                           0x08200001UL) != 0U) return 5;
    puts("ok");
    return 0;
}
"""


def _compile_and_run(tmp_path: Path, sources: list[Path], harness_text: str,
                     include_dirs: list[Path]) -> str:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host C compiler is unavailable")
    harness = tmp_path / "harness.c"
    executable = tmp_path / "harness.exe"
    harness.write_text(harness_text, encoding="ascii")
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            *(f"-I{path}" for path in include_dirs),
            *(str(source) for source in sources),
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
    return result.stdout.strip()


def test_portable_driver_math_noop_and_invalid_branch(tmp_path: Path) -> None:
    assert _compile_and_run(
        tmp_path,
        [ROOT / "Driver" / "Src" / "drv_imu_calibration.c"],
        DRIVER_HARNESS,
        [ROOT / "Driver" / "Inc"],
    ) == "ok"


def test_active_snapshot_generation_and_invalid_publish(tmp_path: Path) -> None:
    (tmp_path / "app_sensor.h").write_text(
        "#define APP_SENSOR_FLU_ORIENTATION_COUNT 24U\n"
        "#define APP_SENSOR_FLU_ORIENTATION_LEGACY 255U\n",
        encoding="ascii",
    )
    (tmp_path / "drv_frame_contract.h").write_text(
        "#define DRV_FRAME_CONTRACT_VERSION 1U\n", encoding="ascii"
    )
    assert _compile_and_run(
        tmp_path,
        [ROOT / "App" / "Src" / "app_flight_calibration.c"],
        ACTIVE_SNAPSHOT_HARNESS,
        [tmp_path, ROOT / "App" / "Inc", ROOT / "Driver" / "Inc"],
    ) == "ok"


def test_runtime_pipeline_applies_v1_once_and_resets_on_generation() -> None:
    source = read("App/Src/app_stabilizer.c")
    step = source[source.index("static void stabilizer_imu_step(") :]
    apply_v0 = step.index("APP_Sensor_ApplyFrameCorrection(&msg->imu)")
    apply_v1 = step.index("DRV_IMU_Calibration_Apply(")
    fusion = step.index("DRV_AttitudeFusion_Update(")
    navigation = step.index("DRV_IMU_NAV_Update(")

    assert apply_v0 < apply_v1 < fusion < navigation
    assert "calibration_snapshot.generation !=" in step
    reset = source[source.index("static void stabilizer_reset_for_imu_frame(") :
                   source.index("static void stabilizer_imu_step(")]
    assert "calibration_generation" in reset
    assert "DRV_AttitudeFusion_InitForConvention(" in reset
    assert "DRV_IMU_NAV_Reset(&ctx->nav_state);" in reset
    assert "ctx->attitude_zero_ready = 0U;" in reset


def test_confirmed_snapshot_and_guarded_imucal_protocol_contract() -> None:
    header = read("App/Inc/app_flight_calibration.h")
    source = read("App/Src/app_flight_calibration.c")
    control = read("App/Src/app_control.c")
    imucal = read("App/Src/app_cmd_imucal.c")

    assert "APP_FlightCalibrationSnapshot" in header
    assert "APP_FlightCalibration_PublishConfirmed" in header
    assert "app_flight_cal_active_seqlock" in source
    sync_start = control.index("static void app_control_imuframe_sync_param(void)\n{")
    sync = control[sync_start:
                   control.index("static uint8_t app_control_send_boot_scheduled(",
                                 sync_start)]
    assert sync.index("if (dirty != 0U)") < sync.index(
        "APP_FlightCalibration_PublishConfirmed(&calibration)"
    )
    assert 'strcmp(tokens[0], "IMUCAL?") == 0' in imucal
    assert 'strcmp(tokens[0], "IMUCAL") != 0' in imucal
    assert "app_control_handle_imucal(tokens, count);" in control
    assert "cal_generation=" in imucal
    assert "valid_mask=" in imucal
    assert "firmware_crc32=" in imucal


def test_imucap_v4_layout_provenance_and_v3_constants() -> None:
    header = read("App/Inc/app_imu_capture.h")
    source = read("App/Src/app_imu_capture.c")

    assert struct.calcsize("<Ih3h3h3h3h3h4HBBh") == 48
    assert struct.calcsize("<IHHIIIHHHHHHHBBIB3xIII") == 56
    assert "APP_IMU_CAPTURE_VERSION_V3       3U" in header
    assert "APP_IMU_CAPTURE_V3_SAMPLE_SIZE   46U" in header
    assert "APP_IMU_CAPTURE_V3_HEADER_SIZE   40U" in header
    assert "APP_IMU_CAPTURE_VERSION          4U" in header
    assert "APP_IMU_CAPTURE_V4_SAMPLE_SIZE   48U" in header
    assert "APP_IMU_CAPTURE_V4_HEADER_SIZE   56U" in header
    assert "int16_t  temperature_raw" in header
    for field in (
        "frame_contract",
        "orientation_code",
        "calibration_valid_mask",
        "calibration_generation",
        "base_frame",
        "firmware_image_crc32",
    ):
        assert field in header
    assert "APP_IMU_CAPTURE_BLOCK_SAMPLES 30U" in source
    assert "sizeof(imu_capture_tx_frame) <= APP_USB_CDC_TX_SIZE" in source
    assert "slot->temperature_raw = raw->temperature;" in source


def test_capture_never_mixes_provenance_epochs() -> None:
    header = read("App/Inc/app_imu_capture.h")
    source = read("App/Src/app_imu_capture.c")
    push = source[source.index("void APP_IMU_Capture_Push") :
                  source.index("static int16_t imu_capture_saturate")]
    start = source[source.index("APP_IMU_CaptureCommandStatus APP_IMU_Capture_Start") :
                   source.index("APP_IMU_CaptureCommandStatus APP_IMU_Capture_Stop")]

    assert "APP_IMU_CAPTURE_FLAG_INVALID_PROVENANCE" in header
    assert "imu_capture_orientation_code = orientation_after;" in start
    assert "imu_capture_calibration_generation = calibration_snapshot.generation;" in start
    assert "APP_FlightCalibration_ReadActiveGeneration(" in push
    assert "imu_capture_provenance_invalid = 1U;" in push
    assert "imu_capture_state = APP_IMU_CAPTURE_FULL;" in push
    assert "flags |= APP_IMU_CAPTURE_FLAG_INVALID_PROVENANCE;" in source


def test_firmware_identity_is_real_linked_image_crc_not_placeholder() -> None:
    header = read("App/Inc/app_firmware_identity.h")
    source = read("App/Src/app_firmware_identity.c")
    linker = read("STM32H743XX_FLASH.ld")

    assert "APP_FIRMWARE_FLASH_START 0x08000000UL" in header
    assert "APP_FIRMWARE_FLASH_LIMIT 0x08200000UL" in header
    assert "__data_source_end" in source
    assert "APP_FirmwareIdentity_ComputeCrc32" in source
    assert "app_firmware_identity_ready" in source
    assert "unknown" not in source.lower()
    assert "__data_source_end = __tdata_source_end" in linker


def test_firmware_identity_crc_and_range_runtime(tmp_path: Path) -> None:
    assert _compile_and_run(
        tmp_path,
        [ROOT / "App" / "Src" / "app_firmware_identity.c"],
        FIRMWARE_IDENTITY_HARNESS,
        [ROOT / "App" / "Inc"],
    ) == "ok"


def test_icm42688_temperature_scale_is_unified() -> None:
    app_sensor = read("App/Src/app_sensor.c")
    driver = read("Driver/Src/drv_imu.c")

    assert re.search(r"APP_IMU_TEMP_LSB_PER_C\s+132\.48f", app_sensor)
    assert "/ 132.48f" in driver
    assert "+ APP_IMU_TEMP_OFFSET_C" in app_sensor


def test_new_runtime_sources_are_built() -> None:
    cmake = read("CMakeLists.txt")

    assert "Driver/Src/drv_imu_calibration.c" in cmake
    assert "App/Src/app_firmware_identity.c" in cmake
