"""Versioned aggregate flight-calibration parameter contract."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "app_flight_calibration.h"
SOURCE = ROOT / "App" / "Src" / "app_flight_calibration.c"
CONTROL = ROOT / "App" / "Src" / "app_control.c"
CMAKE = ROOT / "CMakeLists.txt"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_fcal_schema_owns_v0_v1_and_reserved_v2_fields() -> None:
    header = read(HEADER)
    source = read(SOURCE)

    for field in (
        "magic",
        "schema",
        "size",
        "frame_contract",
        "orientation_code",
        "valid_mask",
        "calibration_generation",
        "accel_bias[3]",
        "accel_correction[3][3]",
        "gyro_bias_ref[3]",
        "gyro_correction[3][3]",
        "gyro_temp_slope[3]",
        "reference_temp_c",
        "v2_navigation_mapping",
        "v2_controller_mapping",
        "v2_rc_mapping",
        "v2_actuator_mapping",
    ):
        assert field in header
    assert "sizeof(APP_FlightCalibration) == 160U" in source
    assert "APP_FLIGHT_CAL_MATRIX_DIAG_MIN    0.85f" in source
    assert "APP_FLIGHT_CAL_MATRIX_DIAG_MAX    1.15f" in source
    assert "APP_FLIGHT_CAL_MATRIX_OFFDIAG_MAX 0.10f" in source
    assert "APP_FLIGHT_CAL_MATRIX_DET_MIN" in source
    assert "APP_FLIGHT_CAL_MATRIX_COND_MAX" in source
    assert "isfinite" in source
    assert "App/Src/app_flight_calibration.c" in read(CMAKE)


def test_imuframe_commit_updates_the_aggregate_instead_of_replacing_it() -> None:
    control = read(CONTROL)

    assert '#include "app_flight_calibration.h"' in control
    assert "APP_ControlImuFrameParam" not in control
    assert "APP_CONTROL_IMU_FRAME_PARAM_MAGIC" not in control
    assert "APP_FlightCalibration_Decode" in control
    assert "APP_FlightCalibration_UpdateOrientation" in control
    assert "APP_FlightCalibration_Encode" in control
    decode_at = control.index("APP_FlightCalibration_Decode(", control.index("COMMIT"))
    update_at = control.index("APP_FlightCalibration_UpdateOrientation", decode_at)
    encode_at = control.index("APP_FlightCalibration_Encode", update_at)
    set_at = control.index("SVC_Param_SetBlob(encoded, encoded_size)", encode_at)
    assert decode_at < update_at < encode_at < set_at


HARNESS = r"""
#include "app_flight_calibration.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { if (!(condition)) return (code); } while (0)

typedef struct {
    uint32_t magic;
    uint16_t schema;
    uint16_t base_contract;
    uint8_t orientation_code;
    uint8_t reserved[3];
} LegacyImuf;

int main(void)
{
    APP_FlightCalibration cal;
    APP_FlightCalibration decoded;
    uint8_t encoded[sizeof(APP_FlightCalibration)];
    LegacyImuf legacy = {0x494D5546U, 1U, 1U, 3U, {0U, 0U, 0U}};
    uint32_t size;

    APP_FlightCalibration_Defaults(&cal);
    CHECK(cal.magic == APP_FLIGHT_CAL_MAGIC, 1);
    CHECK(cal.schema == APP_FLIGHT_CAL_SCHEMA, 2);
    CHECK(cal.size == sizeof(cal), 3);
    CHECK(cal.orientation_code == 255U, 4);
    CHECK(cal.valid_mask == 0U, 5);
    CHECK(cal.accel_correction[0][0] == 1.0f, 6);
    CHECK(cal.gyro_correction[2][2] == 1.0f, 7);
    CHECK(APP_FlightCalibration_Validate(&cal) == 1U, 8);

    cal.accel_bias[0] = 0.125f;
    cal.gyro_temp_slope[2] = -0.02f;
    cal.valid_mask = APP_FLIGHT_CAL_VALID_ACCEL | APP_FLIGHT_CAL_VALID_GYRO_TEMP;
    CHECK(APP_FlightCalibration_UpdateOrientation(&cal, 3U) == 1U, 9);
    CHECK(cal.accel_bias[0] == 0.125f, 10);
    CHECK(cal.gyro_temp_slope[2] == -0.02f, 11);
    CHECK((cal.valid_mask & APP_FLIGHT_CAL_VALID_ORIENTATION) != 0U, 12);
    CHECK((cal.valid_mask & (APP_FLIGHT_CAL_VALID_ACCEL |
                            APP_FLIGHT_CAL_VALID_GYRO_TEMP)) == 0U, 26);
    size = APP_FlightCalibration_Encode(&cal, encoded, sizeof(encoded));
    CHECK(size == sizeof(cal), 13);
    CHECK(APP_FlightCalibration_Decode(encoded, size, &decoded) ==
          APP_FLIGHT_CAL_DECODE_CURRENT, 14);
    CHECK(decoded.orientation_code == 3U, 15);
    CHECK(decoded.accel_bias[0] == 0.125f, 16);

    memset(&decoded, 0, sizeof(decoded));
    CHECK(APP_FlightCalibration_Decode((const uint8_t *)&legacy,
                                      sizeof(legacy), &decoded) ==
          APP_FLIGHT_CAL_DECODE_MIGRATED_IMUF_V1, 17);
    CHECK(decoded.orientation_code == 3U, 18);
    CHECK((decoded.valid_mask & APP_FLIGHT_CAL_VALID_ORIENTATION) != 0U, 19);
    CHECK(decoded.accel_correction[1][1] == 1.0f, 20);
    legacy.orientation_code = 255U;
    CHECK(APP_FlightCalibration_Decode((const uint8_t *)&legacy,
                                      sizeof(legacy), &decoded) ==
          APP_FLIGHT_CAL_DECODE_MIGRATED_IMUF_V1, 27);
    CHECK(decoded.valid_mask == 0U, 28);

    APP_FlightCalibration_Defaults(&cal);
    CHECK(APP_FlightCalibration_UpdateOrientation(&cal, 3U) == 1U, 29);
    cal.valid_mask |= APP_FLIGHT_CAL_VALID_ACCEL | APP_FLIGHT_CAL_VALID_GYRO |
                      APP_FLIGHT_CAL_VALID_GYRO_TEMP;
    CHECK(APP_FlightCalibration_UpdateOrientation(&cal, 3U) == 1U, 30);
    CHECK((cal.valid_mask & APP_FLIGHT_CAL_VALID_ACCEL) != 0U, 31);
    CHECK(APP_FlightCalibration_UpdateOrientation(&cal, 4U) == 1U, 32);
    CHECK((cal.valid_mask & (APP_FLIGHT_CAL_VALID_ACCEL |
                            APP_FLIGHT_CAL_VALID_GYRO |
                            APP_FLIGHT_CAL_VALID_GYRO_TEMP)) == 0U, 33);
    CHECK(APP_FlightCalibration_UpdateOrientation(&cal, 255U) == 1U, 34);
    CHECK((cal.valid_mask & APP_FLIGHT_CAL_VALID_ORIENTATION) == 0U, 35);

    APP_FlightCalibration_Defaults(&cal);
    cal.accel_bias[0] = NAN;
    CHECK(APP_FlightCalibration_Validate(&cal) == 0U, 21);
    APP_FlightCalibration_Defaults(&cal);
    cal.accel_correction[0][0] = -1.0f;
    CHECK(APP_FlightCalibration_Validate(&cal) == 0U, 22);
    APP_FlightCalibration_Defaults(&cal);
    cal.gyro_correction[0][1] = 0.11f;
    CHECK(APP_FlightCalibration_Validate(&cal) == 0U, 23);
    APP_FlightCalibration_Defaults(&cal);
    cal.gyro_correction[0][0] = 0.84f;
    CHECK(APP_FlightCalibration_Validate(&cal) == 0U, 36);
    APP_FlightCalibration_Defaults(&cal);
    cal.valid_mask = 0x80U;
    CHECK(APP_FlightCalibration_Validate(&cal) == 0U, 24);
    legacy.orientation_code = 24U;
    CHECK(APP_FlightCalibration_Decode((const uint8_t *)&legacy,
                                      sizeof(legacy), &decoded) ==
          APP_FLIGHT_CAL_DECODE_INVALID, 25);

    puts("ok");
    return 0;
}
"""


def test_fcal_codec_and_migration_compile_and_run_on_host(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.fail("FCAL contract requires host gcc or clang")

    (tmp_path / "app_sensor.h").write_text(
        "#define APP_SENSOR_FLU_ORIENTATION_COUNT 24U\n"
        "#define APP_SENSOR_FLU_ORIENTATION_LEGACY 255U\n",
        encoding="ascii",
    )
    (tmp_path / "drv_frame_contract.h").write_text(
        "#define DRV_FRAME_CONTRACT_VERSION 1U\n", encoding="ascii"
    )
    harness = tmp_path / "fcal_harness.c"
    executable = tmp_path / "fcal_harness.exe"
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
            str(SOURCE),
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
