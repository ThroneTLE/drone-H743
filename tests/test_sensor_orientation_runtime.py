"""Runtime IMU orientation selection at the raw-to-airframe seam."""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "app_sensor.h"
SOURCE = ROOT / "App" / "Src" / "app_sensor.c"


EXPECTED_DESCRIPTORS = (
    "+x,+y,+z",
    "+x,-y,-z",
    "-x,+y,-z",
    "-x,-y,+z",
    "+x,+z,-y",
    "+x,-z,+y",
    "-x,+z,+y",
    "-x,-z,-y",
    "+y,+x,-z",
    "+y,-x,+z",
    "-y,+x,+z",
    "-y,-x,-z",
    "+y,+z,+x",
    "+y,-z,-x",
    "-y,+z,-x",
    "-y,-z,+x",
    "+z,+x,+y",
    "+z,-x,-y",
    "-z,+x,-y",
    "-z,-x,+y",
    "+z,+y,-x",
    "+z,-y,+x",
    "-z,+y,+x",
    "-z,-y,-x",
)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def descriptor_matrix(descriptor: str) -> tuple[tuple[int, int, int], ...]:
    rows = []
    for token in descriptor.split(","):
        sign = 1 if token[0] == "+" else -1
        axis = "xyz".index(token[1])
        row = [0, 0, 0]
        row[axis] = sign
        rows.append(tuple(row))
    return tuple(rows)


def determinant(matrix: tuple[tuple[int, int, int], ...]) -> int:
    return (
        matrix[0][0]
        * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1]
        * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2]
        * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def test_public_api_uses_one_byte_legacy_sentinel_and_stable_descriptors() -> None:
    header = read(HEADER)
    source = read(SOURCE)

    assert "#define APP_SENSOR_FLU_ORIENTATION_COUNT 24U" in header
    assert "#define APP_SENSOR_FLU_ORIENTATION_LEGACY 255U" in header
    assert "APP_Sensor_SetFluOrientation(const char *descriptor)" in header
    assert "APP_Sensor_SetFluOrientationCode(uint8_t code)" in header
    assert "APP_Sensor_GetFluOrientation(void)" in header
    assert "APP_Sensor_IsFluOrientationActive(void)" in header
    assert "APP_Sensor_GetFluOrientationDescriptorForCode(uint8_t code)" in header
    assert "APP_Sensor_GetFluOrientationDescriptor(void)" in header
    assert "APP_Sensor_ApplyFrameCorrection(DRV_IMU_ScaledData *imu)" in header
    assert "temperature field is intentionally untouched" in header
    assert "R_FLU<-legacy_intermediate_v1" in header
    assert re.search(
        r"static\s+volatile\s+uint8_t\s+app_sensor_flu_orientation\s*=\s*"
        r"APP_SENSOR_FLU_ORIENTATION_LEGACY\s*;",
        source,
    )

    descriptors = tuple(
        re.findall(r'\{\s*"([+-][xyz],[+-][xyz],[+-][xyz])"\s*,', source)
    )
    assert descriptors == EXPECTED_DESCRIPTORS
    assert len(set(descriptors)) == 24
    assert all(determinant(descriptor_matrix(item)) == 1 for item in descriptors)
    assert "-x,+y,+z" not in descriptors  # determinant -1 mirror


RUNTIME_HARNESS = r"""
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { if (!(condition)) return (code); } while (0)

static int same3(const float value[3], float x, float y, float z)
{
    return value[0] == x && value[1] == y && value[2] == z;
}

int main(void)
{
    static const char *const descriptors[24] = {
        "+x,+y,+z", "+x,-y,-z", "-x,+y,-z", "-x,-y,+z",
        "+x,+z,-y", "+x,-z,+y", "-x,+z,+y", "-x,-z,-y",
        "+y,+x,-z", "+y,-x,+z", "-y,+x,+z", "-y,-x,-z",
        "+y,+z,+x", "+y,-z,-x", "-y,+z,-x", "-y,-z,+x",
        "+z,+x,+y", "+z,-x,-y", "-z,+x,-y", "-z,-x,+y",
        "+z,+y,-x", "+z,-y,+x", "-z,+y,+x", "-z,-y,-x",
    };
    const float accel_raw[3] = {1.0f, 2.0f, 3.0f};
    float accel_out[3];
    DRV_IMU_ScaledData imu = {
        .temperature_c = 27.5f,
        .accel_x_g = -3.0f, .accel_y_g = -1.0f, .accel_z_g = 2.0f,
        .gyro_x_dps = 6.0f, .gyro_y_dps = 4.0f, .gyro_z_dps = 5.0f,
    };
    uint8_t before;
    int i;

    CHECK(APP_Sensor_GetFluOrientation() == 255U, 1);
    CHECK(APP_Sensor_IsFluOrientationActive() == 0U, 2);
    CHECK(strcmp(APP_Sensor_GetFluOrientationDescriptor(), "legacy") == 0, 3);
    APP_Sensor_AlignToAirframe(accel_raw, accel_out);
    CHECK(same3(accel_out, -3.0f, -1.0f, 2.0f), 4);

    for (i = 0; i < 24; ++i) {
        CHECK(APP_Sensor_SetFluOrientation(descriptors[i]) == 1U, 10 + i);
        CHECK(APP_Sensor_GetFluOrientation() == (uint8_t)i, 40 + i);
        CHECK(strcmp(APP_Sensor_GetFluOrientationDescriptor(), descriptors[i]) == 0,
              70 + i);
        CHECK(strcmp(APP_Sensor_GetFluOrientationDescriptorForCode((uint8_t)i),
                     descriptors[i]) == 0, 220 + i);
    }

    for (i = 0; i < 24; ++i) {
        CHECK(APP_Sensor_SetFluOrientationCode((uint8_t)i) == 1U, 120 + i);
        CHECK(APP_Sensor_GetFluOrientation() == (uint8_t)i, 150 + i);
        CHECK(strcmp(APP_Sensor_GetFluOrientationDescriptor(), descriptors[i]) == 0,
              180 + i);
    }
    before = APP_Sensor_GetFluOrientation();
    CHECK(APP_Sensor_SetFluOrientationCode(24U) == 0U, 210);
    CHECK(APP_Sensor_SetFluOrientationCode(254U) == 0U, 211);
    CHECK(APP_Sensor_GetFluOrientation() == before, 212);
    CHECK(APP_Sensor_SetFluOrientationCode(255U) == 1U, 213);
    CHECK(APP_Sensor_GetFluOrientation() == 255U, 214);
    CHECK(strcmp(APP_Sensor_GetFluOrientationDescriptorForCode(255U),
                 "legacy") == 0, 215);
    CHECK(strcmp(APP_Sensor_GetFluOrientationDescriptorForCode(24U),
                 "invalid") == 0, 216);
    CHECK(strcmp(APP_Sensor_GetFluOrientationDescriptorForCode(254U),
                 "invalid") == 0, 217);

    CHECK(APP_Sensor_SetFluOrientation("-x,-y,+z") == 1U, 100);
    /* The chip-to-legacy function never observes the runtime candidate. */
    APP_Sensor_AlignToAirframe(accel_raw, accel_out);
    CHECK(same3(accel_out, -3.0f, -1.0f, 2.0f), 101);
    CHECK(APP_Sensor_ApplyFrameCorrection(&imu) == 3U, 102);
    CHECK(same3(&imu.accel_x_g, 3.0f, 1.0f, 2.0f), 103);
    CHECK(same3(&imu.gyro_x_dps, -6.0f, -4.0f, 5.0f), 104);
    CHECK(imu.temperature_c == 27.5f, 105);

    before = APP_Sensor_GetFluOrientation();
    CHECK(APP_Sensor_SetFluOrientation("-x,+y,+z") == 0U, 106);
    CHECK(APP_Sensor_SetFluOrientation("+x,+x,+z") == 0U, 107);
    CHECK(APP_Sensor_SetFluOrientation(NULL) == 0U, 108);
    CHECK(APP_Sensor_GetFluOrientation() == before, 109);

    CHECK(APP_Sensor_SetFluOrientation("legacy") == 1U, 110);
    CHECK(APP_Sensor_GetFluOrientation() == 255U, 111);
    CHECK(APP_Sensor_IsFluOrientationActive() == 0U, 112);

    puts("ok");
    return 0;
}
"""


def test_runtime_applies_one_proper_rotation_after_legacy_mapping(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.fail("sensor orientation contract requires host gcc or clang")

    source = read(SOURCE)
    start = source.index("typedef struct {\n    const char *descriptor;")
    end = source.index("uint8_t APP_IMU_ReadDataReadyTimestamp")
    orientation_source = source[start:end]

    apply_start = orientation_source.index(
        "uint8_t APP_Sensor_ApplyFrameCorrection(DRV_IMU_ScaledData *imu)"
    )
    apply_body = orientation_source[apply_start:]
    assert apply_body.count("orientation = app_sensor_flu_orientation") == 1
    assert "APP_Sensor_ApplyOrientation(accel_in, accel_out, orientation)" in apply_body
    assert "APP_Sensor_ApplyOrientation(gyro_in, gyro_out, orientation)" in apply_body

    isolated = tmp_path / "sensor_orientation.c"
    isolated.write_text(
        "#include <stddef.h>\n#include <stdint.h>\n#include <string.h>\n"
        "#define APP_SENSOR_FLU_ORIENTATION_COUNT 24U\n"
        "#define APP_SENSOR_FLU_ORIENTATION_LEGACY 255U\n"
        "typedef struct { float temperature_c, accel_x_g, accel_y_g, accel_z_g; "
        "float gyro_x_dps, gyro_y_dps, gyro_z_dps; } DRV_IMU_ScaledData;\n"
        + orientation_source,
        encoding="utf-8",
    )
    harness = tmp_path / "sensor_orientation_harness.c"
    harness.write_text(
        "#include <stddef.h>\n#include <stdint.h>\n"
        "uint8_t APP_Sensor_SetFluOrientation(const char *descriptor);\n"
        "uint8_t APP_Sensor_SetFluOrientationCode(uint8_t code);\n"
        "uint8_t APP_Sensor_GetFluOrientation(void);\n"
        "uint8_t APP_Sensor_IsFluOrientationActive(void);\n"
        "const char *APP_Sensor_GetFluOrientationDescriptorForCode(uint8_t code);\n"
        "const char *APP_Sensor_GetFluOrientationDescriptor(void);\n"
        "typedef struct { float temperature_c, accel_x_g, accel_y_g, accel_z_g; "
        "float gyro_x_dps, gyro_y_dps, gyro_z_dps; } DRV_IMU_ScaledData;\n"
        "uint8_t APP_Sensor_ApplyFrameCorrection(DRV_IMU_ScaledData *imu);\n"
        "void APP_Sensor_AlignToAirframe(const float in[3], float out[3]);\n"
        + RUNTIME_HARNESS,
        encoding="utf-8",
    )
    executable = tmp_path / "sensor_orientation.exe"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(isolated),
            str(harness),
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
