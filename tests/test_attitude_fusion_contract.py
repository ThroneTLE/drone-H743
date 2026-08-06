from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_xio_fusion_is_vendored_and_replaces_height_gated_attitude() -> None:
    cmake = read("CMakeLists.txt")
    header = read("Driver/Inc/drv_attitude_fusion.h")
    source = read("Driver/Src/drv_attitude_fusion.c")
    license_text = read("ThirdParty/Fusion/LICENSE.md")

    assert "Driver/Src/drv_attitude_fusion.c" in cmake
    assert "ThirdParty/Fusion/FusionAhrs.c" in cmake
    assert "ThirdParty/Fusion" in cmake
    assert "x-io Technologies" in license_text
    assert "015d68494274b479b5996bff2530ecbcfdc266f2" in source
    assert "FusionAhrsUpdateNoMagnetometer" in source
    assert "FusionConventionNed" in source
    # Gates are deliberately loose: coaxial-rotor vibration made the previous
    # 10 deg / [0.85, 1.15] g pair ignore the accelerometer for 75-85 % of
    # powered flight, leaving attitude on free-running gyro integration.
    assert "DRV_ATTITUDE_FUSION_ACCEL_REJECTION_DEG 45.0f" in source
    assert "DRV_ATTITUDE_FUSION_REJECTION_TIMEOUT_S 0.5f" in source
    assert "DRV_ATTITUDE_FUSION_ACCEL_NORM_MIN_G 0.40f" in source
    assert "DRV_ATTITUDE_FUSION_ACCEL_NORM_MAX_G 1.80f" in source
    assert "in_air" not in header
    assert "height" not in header.lower()
    assert "drv_px4_attitude" not in cmake
    assert "app_attitude_flight_state" not in cmake


FUSION_HARNESS = r"""
#include "drv_attitude_fusion.h"

#include <math.h>
#include <string.h>

#define CHECK(condition, code) do { if (!(condition)) { return (code); } } while (0)

static DRV_AttitudeFusionOutput update(float gx, float gy, float gz,
                                       float ax, float ay, float az,
                                       unsigned long long *time_us)
{
    DRV_AttitudeFusionInput input;
    DRV_AttitudeFusionOutput output;
    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));
    *time_us += 1000ULL;
    input.time_us = *time_us;
    input.dt_s = 0.001f;
    input.gyroscope_dps[0] = gx;
    input.gyroscope_dps[1] = gy;
    input.gyroscope_dps[2] = gz;
    input.accelerometer_g[0] = ax;
    input.accelerometer_g[1] = ay;
    input.accelerometer_g[2] = az;
    (void)DRV_AttitudeFusion_Update(&input, &output);
    return output;
}

static void static_accel(float roll_deg, float pitch_deg,
                         float *ax, float *ay, float *az)
{
    const float roll = roll_deg * 0.017453292519943295f;
    const float pitch = pitch_deg * 0.017453292519943295f;
    *ax = sinf(pitch);
    *ay = -sinf(roll) * cosf(pitch);
    *az = -cosf(roll) * cosf(pitch);
}

int main(void)
{
    DRV_AttitudeFusionOutput output;
    unsigned long long time_us = 0ULL;
    float ax;
    float ay;
    float az;

    /* Static tilt must converge with the established NED/sign contract. */
    DRV_AttitudeFusion_Init();
    static_accel(20.0f, -15.0f, &ax, &ay, &az);
    for (int index = 0; index < 4000; ++index) {
        output = update(0.0f, 0.0f, 0.0f, ax, ay, az, &time_us);
    }
    CHECK(fabsf(output.roll_deg - 20.0f) < 0.25f, 1);
    CHECK(fabsf(output.pitch_deg + 15.0f) < 0.25f, 2);

    /* A large apparent tilt is still rejected, well beyond the widened gate. */
    DRV_AttitudeFusion_Init();
    time_us = 0ULL;
    for (int index = 0; index < 4000; ++index) {
        output = update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, &time_us);
    }
    for (int index = 0; index < 200; ++index) {
        output = update(0.0f, 0.0f, 0.0f, 0.9848f, 0.0f, -0.1736f, &time_us);
    }
    CHECK(output.accelerometer_ignored != 0U, 3);
    CHECK(fabsf(output.roll_deg) < 0.5f, 4);

    /* Vibration-scale innovation must be trusted, not rejected: this is the
     * regression that left attitude on gyro integration in powered flight. */
    DRV_AttitudeFusion_Init();
    time_us = 0ULL;
    for (int index = 0; index < 4000; ++index) {
        output = update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, &time_us);
    }
    static_accel(15.0f, 0.0f, &ax, &ay, &az);
    output = update(0.0f, 0.0f, 0.0f, ax, ay, az, &time_us);
    CHECK(output.accelerometer_ignored == 0U, 5);
    CHECK(output.accel_norm_rejected == 0U, 6);

    /* Only physically implausible specific force is magnitude-rejected. */
    output = update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -2.5f, &time_us);
    CHECK(output.accel_norm_rejected != 0U, 7);
    CHECK(output.accelerometer_ignored != 0U, 20);

    /* Norm-gated samples must still drive the recovery counter. Feeding a zero
     * vector makes the library skip its rejection block entirely, so without
     * the driver advancing the trigger the built-in recovery could never fire
     * and attitude stayed on gyro integration indefinitely. */
    DRV_AttitudeFusion_Init();
    time_us = 0ULL;
    for (int index = 0; index < 4000; ++index) {
        output = update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, &time_us);
    }
    for (int index = 0; index < 2000; ++index) {
        output = update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -3.0f, &time_us);
    }
    CHECK(output.accel_norm_rejected != 0U, 21);
    CHECK(output.acceleration_recovery_trigger > 0.0f, 22);

    /* Reproduce a 43 degree gyro-only error, then prove autonomous recovery. */
    DRV_AttitudeFusion_Init();
    time_us = 0ULL;
    for (int index = 0; index < 4000; ++index) {
        output = update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, &time_us);
    }
    for (int index = 0; index < 1000; ++index) {
        output = update(43.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &time_us);
    }
    CHECK(fabsf(output.roll_deg - 43.0f) < 0.5f, 8);
    for (int index = 0; index < 20000; ++index) {
        output = update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, &time_us);
    }
    CHECK(fabsf(output.roll_deg) < 1.0f, 9);
    CHECK(fabsf(output.pitch_deg) < 1.0f, 10);
    CHECK(output.accel_correction_count > 0U, 11);

    /* A real roll rotation has consistent gyro and gravity cues.  Holding the
     * angle must not trigger rejection or delayed recovery. */
    DRV_AttitudeFusion_Init();
    time_us = 0ULL;
    for (int index = 0; index < 4000; ++index) {
        output = update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, &time_us);
    }
    for (int index = 0; index < 1000; ++index) {
        static_accel(20.0f * (float)(index + 1) / 1000.0f, 0.0f,
                     &ax, &ay, &az);
        output = update(20.0f, 0.0f, 0.0f, ax, ay, az, &time_us);
    }
    static_accel(20.0f, 0.0f, &ax, &ay, &az);
    for (int index = 0; index < 10000; ++index) {
        output = update(0.0f, 0.0f, 0.0f, ax, ay, az, &time_us);
    }
    CHECK(fabsf(output.roll_deg - 20.0f) < 0.5f, 12);
    CHECK(fabsf(output.pitch_deg) < 0.5f, 13);
    CHECK(output.accelerometer_ignored == 0U, 14);
    CHECK(output.acceleration_recovery == 0U, 15);

    /* Exercise pitch independently so both corrected accelerometer signs are
     * protected by a physical transition, not only a static initial tilt. */
    DRV_AttitudeFusion_Init();
    time_us = 0ULL;
    for (int index = 0; index < 4000; ++index) {
        output = update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, &time_us);
    }
    for (int index = 0; index < 1000; ++index) {
        static_accel(0.0f, 20.0f * (float)(index + 1) / 1000.0f,
                     &ax, &ay, &az);
        output = update(0.0f, 20.0f, 0.0f, ax, ay, az, &time_us);
    }
    static_accel(0.0f, 20.0f, &ax, &ay, &az);
    for (int index = 0; index < 10000; ++index) {
        output = update(0.0f, 0.0f, 0.0f, ax, ay, az, &time_us);
    }
    CHECK(fabsf(output.roll_deg) < 0.5f, 16);
    CHECK(fabsf(output.pitch_deg - 20.0f) < 0.5f, 17);
    CHECK(output.accelerometer_ignored == 0U, 18);
    CHECK(output.acceleration_recovery == 0U, 19);
    return 0;
}
"""


def test_attitude_fusion_runtime(tmp_path: Path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "attitude_fusion_harness.c"
    executable = tmp_path / "attitude_fusion_harness.exe"
    harness.write_text(FUSION_HARNESS, encoding="ascii")
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            f"-I{ROOT / 'ThirdParty' / 'Fusion'}",
            str(ROOT / "ThirdParty" / "Fusion" / "FusionAhrs.c"),
            str(ROOT / "Driver" / "Src" / "drv_attitude_fusion.c"),
            str(harness),
            "-lm",
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)
