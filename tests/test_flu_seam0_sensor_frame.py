"""R-F0 seam 0 (SENSOR) FLU evidence contract.

`tests/test_sensor_orientation_runtime.py` already pins the 24-entry Flash ABI
table, the fixed chip -> legacy_intermediate_v1 mapping and the per-code
`APP_Sensor_ApplyFrameCorrection` behaviour, but it checks each stage in
isolation.  Nothing pinned the property the migration mask actually cares
about: that the *composition* of both stages, under the orientation code that
is persisted on the airframe, is a proper rotation delivering the canonical FLU
body frame of `Driver/Inc/drv_frame_contract.h`.

This module links the real `app_sensor.c` orientation code against the real
normative header in one host executable and pins:

  * the composed chip -> published mapping under persisted code 3,
  * that the composition is a proper rotation (det = +1, no reflection),
  * that accelerometer and gyroscope receive the identical rotation,
  * that a level, stationary board publishes the contract's specific force.

Seam 0 carries no source behaviour change (author ruling 2026-08-30, option C):
the runtime already publishes FLU whenever a V0 candidate is active, so this
contract is the executable evidence the reviewer needs before flipping
`DRV_FRAME_MIGRATION_SENSOR_TO_FLU_BIT`.  It is green on arrival by design.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "App" / "Src" / "app_sensor.c"
CONTRACT_DIR = ROOT / "Driver" / "Inc"
CONTRACT = CONTRACT_DIR / "drv_frame_contract.h"

# The orientation code written to Flash on this airframe (PIPELINE M2/M4:
# persisted=-x,-y,+z, dirty=0).  Seam 0 evidence is only meaningful for the
# code the board actually boots with.
PERSISTED_ORIENTATION_CODE = 3
PERSISTED_ORIENTATION_DESCRIPTOR = "-x,-y,+z"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def extract_orientation_source() -> str:
    """Slice the orientation block out of app_sensor.c.

    Mirrors the slicing used by test_sensor_orientation_runtime.py so both
    contracts compile the same real code rather than a transcription.
    """
    source = read(SOURCE)
    start = source.index("typedef struct {\n    const char *descriptor;")
    end = source.index("uint8_t APP_IMU_ReadDataReadyTimestamp")
    return source[start:end]


def test_persisted_orientation_code_still_maps_to_its_descriptor() -> None:
    """Guard the Flash ABI slot this seam's evidence is anchored to."""
    source = read(SOURCE)
    table_start = source.index("app_sensor_flu_orientations[APP_SENSOR_FLU_ORIENTATION_COUNT]")
    table_end = source.index("};", table_start)
    entries = [
        line.strip()
        for line in source[table_start:table_end].splitlines()
        if line.strip().startswith("{ \"")
    ]
    assert len(entries) == 24
    assert entries[PERSISTED_ORIENTATION_CODE].startswith(
        f'{{ "{PERSISTED_ORIENTATION_DESCRIPTOR}"'
    )
    assert "{ -1, -2, +3 }" in entries[PERSISTED_ORIENTATION_CODE]


@pytest.mark.xfail(
    strict=True,
    reason="R-F0 red test: app_sensor.c still documents the pre-V0 mounting; "
    "cleared by the seam 0 implementation commit",
)
def test_mounting_comment_matches_the_measured_v0_result() -> None:
    """The physical-mounting comment must not contradict the persisted fit.

    The composed mapping is published = [+chip_z, +chip_x, +chip_y], so IMU +X
    points left, IMU +Y points up and IMU +Z points forward.  A comment
    claiming +Y is down and +Z is aft describes a frame rotated 180 deg about
    Y from the one M2 measured and committed, and is exactly the sort of stale
    text that invites a "fix" by sign flipping.
    """
    source = read(SOURCE)
    assert "IMU +Y 朝飞机下方，IMU +Z 朝飞机后方" not in source
    assert "IMU +X 朝飞机左方，IMU +Y 朝飞机上方，IMU +Z 朝飞机前方" in source


SEAM0_HARNESS = r"""
#include "drv_frame_contract.h"

#include <stdint.h>
#include <stdio.h>

#define CHECK(condition, code) do { if (!(condition)) return (code); } while (0)

/* Exact float comparison: every value here is +-1 or 0 through a signed
 * permutation, so any drift is a real mapping change, not rounding. */
static int same3(const float value[3], float x, float y, float z)
{
    return value[0] == x && value[1] == y && value[2] == z;
}

/* Run one chip-frame vector through the complete sensor seam. */
static void publish(const float chip[3], float out[3], int is_gyro)
{
    float aligned[3];
    DRV_IMU_ScaledData imu = {0};

    APP_Sensor_AlignToAirframe(chip, aligned);
    if (is_gyro) {
        imu.gyro_x_dps = aligned[0];
        imu.gyro_y_dps = aligned[1];
        imu.gyro_z_dps = aligned[2];
    } else {
        imu.accel_x_g = aligned[0];
        imu.accel_y_g = aligned[1];
        imu.accel_z_g = aligned[2];
    }
    (void)APP_Sensor_ApplyFrameCorrection(&imu);
    if (is_gyro) {
        out[0] = imu.gyro_x_dps;
        out[1] = imu.gyro_y_dps;
        out[2] = imu.gyro_z_dps;
    } else {
        out[0] = imu.accel_x_g;
        out[1] = imu.accel_y_g;
        out[2] = imu.accel_z_g;
    }
}

int main(void)
{
    const float chip_x[3] = {1.0f, 0.0f, 0.0f};
    const float chip_y[3] = {0.0f, 1.0f, 0.0f};
    const float chip_z[3] = {0.0f, 0.0f, 1.0f};
    float col_x[3], col_y[3], col_z[3];
    float gyro_col[3];
    float level[3];
    float det;

    /* The contract header must still describe canonical FLU. */
    CHECK(DRV_FRAME_CANONICAL_BODY_IS_FLU == 1U, 1);
    CHECK(DRV_FRAME_BODY_IS_RIGHT_HANDED == 1U, 2);

    /* Seam 0 evidence is anchored to the orientation persisted on the board. */
    CHECK(APP_Sensor_SetFluOrientationCode(3U) == 1U, 3);
    CHECK(APP_Sensor_IsFluOrientationActive() == 1U, 4);

    publish(chip_x, col_x, 0);
    publish(chip_y, col_y, 0);
    publish(chip_z, col_z, 0);

    /*
     * Composed chip -> published mapping:
     *   published X (forward) = chip Z
     *   published Y (left)    = chip X
     *   published Z (up)      = chip Y
     */
    CHECK(same3(col_x, 0.0f, 1.0f, 0.0f), 10);
    CHECK(same3(col_y, 0.0f, 0.0f, 1.0f), 11);
    CHECK(same3(col_z, 1.0f, 0.0f, 0.0f), 12);

    /*
     * Proper rotation: det = +1.  A reflection would still line up a static
     * gravity check while silently inverting every rate sign.
     */
    det = col_x[0] * (col_y[1] * col_z[2] - col_y[2] * col_z[1])
        - col_y[0] * (col_x[1] * col_z[2] - col_x[2] * col_z[1])
        + col_z[0] * (col_x[1] * col_y[2] - col_x[2] * col_y[1]);
    CHECK(det == 1.0f, 20);

    /* Axial (gyro) and polar (accel) vectors take the identical rotation. */
    publish(chip_x, gyro_col, 1);
    CHECK(same3(gyro_col, col_x[0], col_x[1], col_x[2]), 30);
    publish(chip_y, gyro_col, 1);
    CHECK(same3(gyro_col, col_y[0], col_y[1], col_y[2]), 31);
    publish(chip_z, gyro_col, 1);
    CHECK(same3(gyro_col, col_z[0], col_z[1], col_z[2]), 32);

    /*
     * Level and stationary.  Because published Z = chip Y, the chip reads
     * +1 g on its own +Y axis when the airframe is level; the seam must
     * publish the contract's canonical level specific force.
     */
    {
        const float chip_level[3] = {0.0f, 1.0f, 0.0f};
        publish(chip_level, level, 0);
    }
    CHECK(same3(level,
                DRV_FRAME_LEVEL_SPECIFIC_FORCE_X_G,
                DRV_FRAME_LEVEL_SPECIFIC_FORCE_Y_G,
                DRV_FRAME_LEVEL_SPECIFIC_FORCE_Z_G), 40);

    /*
     * Legacy sentinel must stay a pass-through: seam 0 has no authority to
     * invent a mounting rotation when no V0 candidate is persisted.
     */
    CHECK(APP_Sensor_SetFluOrientationCode(255U) == 1U, 50);
    CHECK(APP_Sensor_IsFluOrientationActive() == 0U, 51);
    {
        DRV_IMU_ScaledData imu = {0};
        imu.accel_x_g = 7.0f;
        imu.accel_y_g = 8.0f;
        imu.accel_z_g = 9.0f;
        CHECK(APP_Sensor_ApplyFrameCorrection(&imu) == 255U, 52);
        CHECK(same3(&imu.accel_x_g, 7.0f, 8.0f, 9.0f), 53);
    }

    /* Runtime migration must not be claimable from this seam alone. */
    CHECK(DRV_FRAME_RUNTIME_MIGRATION_COMPLETE == 0U, 60);

    puts("ok");
    return 0;
}
"""


def test_sensor_seam_publishes_canonical_flu_under_persisted_orientation(
    tmp_path: Path,
) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.fail("seam 0 FLU contract requires host gcc or clang")

    isolated = tmp_path / "sensor_seam0.c"
    isolated.write_text(
        "#include <stddef.h>\n#include <stdint.h>\n#include <string.h>\n"
        "#define APP_SENSOR_FLU_ORIENTATION_COUNT 24U\n"
        "#define APP_SENSOR_FLU_ORIENTATION_LEGACY 255U\n"
        "typedef struct { float temperature_c, accel_x_g, accel_y_g, accel_z_g; "
        "float gyro_x_dps, gyro_y_dps, gyro_z_dps; } DRV_IMU_ScaledData;\n"
        + extract_orientation_source(),
        encoding="utf-8",
    )
    harness = tmp_path / "sensor_seam0_harness.c"
    harness.write_text(
        "#include <stddef.h>\n#include <stdint.h>\n"
        "typedef struct { float temperature_c, accel_x_g, accel_y_g, accel_z_g; "
        "float gyro_x_dps, gyro_y_dps, gyro_z_dps; } DRV_IMU_ScaledData;\n"
        "uint8_t APP_Sensor_SetFluOrientationCode(uint8_t code);\n"
        "uint8_t APP_Sensor_IsFluOrientationActive(void);\n"
        "uint8_t APP_Sensor_ApplyFrameCorrection(DRV_IMU_ScaledData *imu);\n"
        "void APP_Sensor_AlignToAirframe(const float in[3], float out[3]);\n"
        + SEAM0_HARNESS,
        encoding="utf-8",
    )
    executable = tmp_path / "sensor_seam0.exe"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{CONTRACT_DIR}",
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
