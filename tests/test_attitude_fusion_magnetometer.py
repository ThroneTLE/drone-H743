"""Host-side tests for the C4 magnetometer fusion gate in
Driver/Src/drv_attitude_fusion.c.

Compiles the real drv_attitude_fusion.c (+ the vendored FusionAhrs.c and the
real drv_mag_calibration.c field-magnitude gate) with host gcc and runs a
harness exercising each of the four fusion-gate conditions independently:

  1/2/3. calibrated / axis-verified / fresh -- folded by the caller into
         magnetometer_valid (this driver cannot see Services types, see
         drv_attitude_fusion.h); tested here as "subsystem disabled".
  4.     field-magnitude plausibility -- decided inside this driver via
         DRV_MAG_FieldMagnitude_InRange().

Also proves: default/disabled path is bit-identical to the
magnetometer-free FusionAhrsUpdateNoMagnetometer() call (the C4 safety
floor), the magnetometer measurably influences yaw once actually used, and
the field-magnitude gate's recovery trigger advances during a sustained
gated stretch exactly like the pre-existing accelerometer norm gate fix in
tests/test_attitude_fusion_contract.py.

Derivation of the `in_range = {500, 0, 0}` mgauss vector used throughout:
its only required property is "the magnetic reading a level, yaw == 0
FLU/NWU body would report" (purely horizontal, magnitude inside
[DRV_MAG_FIELD_MIN_MGAUSS, DRV_MAG_FIELD_MAX_MGAUSS] -- no inclination
component, since this is a synthetic self-consistency check, not a claim
about Earth's real field). That was verified empirically rather than
hand-derived from the vendored library's internal HalfMagnetic() sign
convention (a NED/NWU-style AHRS's magnetic reference axis is an internal
implementation detail, not part of this repository's FLU contract): feeding
it from t=0 with no rotation holds yaw at exactly 0 throughout, which is
exactly the self-consistency the recovery scenario below depends on.
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r"""
#include "drv_attitude_fusion.h"
#include "drv_mag_calibration.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { if (!(condition)) { \
    fprintf(stderr, "line %d, code %d: %s\n", __LINE__, (code), #condition); \
    return (code); } } while (0)

static DRV_AttitudeFusionOutput step(DRV_AttitudeFusionConvention convention,
                                     uint8_t restart,
                                     float gx, float gy, float gz,
                                     float ax, float ay, float az,
                                     const float *mag_mgauss,
                                     uint8_t mag_valid,
                                     unsigned long long *time_us)
{
    DRV_AttitudeFusionInput input;
    DRV_AttitudeFusionOutput output;

    if (restart) {
        DRV_AttitudeFusion_InitForConvention(convention);
    }
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
    if (mag_mgauss != NULL) {
        input.magnetometer_mgauss[0] = mag_mgauss[0];
        input.magnetometer_mgauss[1] = mag_mgauss[1];
        input.magnetometer_mgauss[2] = mag_mgauss[2];
    }
    input.magnetometer_valid = mag_valid;
    (void)DRV_AttitudeFusion_Update(&input, &output);
    return output;
}

/* Warm up past STARTUP_PERIOD (3s) with a level FLU/NWU attitude and no
 * magnetometer, mirroring the existing accelerometer-only test harness. */
static void warmup_nwu_level(unsigned long long *time_us)
{
    int index;
    DRV_AttitudeFusion_InitForConvention(DRV_ATTITUDE_FUSION_CONVENTION_NWU);
    *time_us = 0ULL;
    for (index = 0; index < 4000; ++index) {
        (void)step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, NULL, 0U, time_us);
    }
}

int main(void)
{
    unsigned long long time_us;
    DRV_AttitudeFusionOutput output;
    DRV_AttitudeFusionOutput reference;
    int index;
    float in_range[3] = { 500.0f, 0.0f, 0.0f };
    float too_weak[3] = { 10.0f, 0.0f, 0.0f };
    float too_strong[3] = { 2000.0f, 0.0f, 0.0f };
    float yaw_no_mag;
    float yaw_with_mag;

    /* --- Gate 1-3 folded: magnetometer_valid == 0 is "no magnetometer". */
    warmup_nwu_level(&time_us);
    output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 1.0f, in_range, 0U, &time_us);
    CHECK(output.magnetometer_subsystem_enabled == 0U, 1);
    CHECK(output.magnetometer_used == 0U, 2);
    CHECK(output.magnetometer_field_rejected == 0U, 3);

    /*
     * Bit-identical default path: run the exact same gyro/accel sequence
     * twice from the same warmed-up state, once with magnetometer_valid==0
     * and once with magnetometer_valid==1 but a field magnitude outside
     * [DRV_MAG_FIELD_MIN_MGAUSS, DRV_MAG_FIELD_MAX_MGAUSS]. Both must reach
     * FusionAhrsUpdateNoMagnetometer() through the identical call, so every
     * output field -- not just the flags -- must match exactly.
     */
    warmup_nwu_level(&time_us);
    for (index = 0; index < 50; ++index) {
        reference = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                         1.0f, 2.0f, 3.0f, 0.05f, -0.02f, 0.99f,
                         NULL, 0U, &time_us);
    }
    warmup_nwu_level(&time_us);
    for (index = 0; index < 50; ++index) {
        output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                      1.0f, 2.0f, 3.0f, 0.05f, -0.02f, 0.99f,
                      too_weak, 1U, &time_us);
    }
    CHECK(output.magnetometer_subsystem_enabled == 1U, 4);
    CHECK(output.magnetometer_field_rejected == 1U, 5);
    CHECK(output.magnetometer_used == 0U, 6);
    CHECK(output.roll_deg == reference.roll_deg, 7);
    CHECK(output.pitch_deg == reference.pitch_deg, 8);
    CHECK(output.yaw_deg == reference.yaw_deg, 9);
    CHECK(output.quaternion[0] == reference.quaternion[0], 10);
    CHECK(output.quaternion[1] == reference.quaternion[1], 11);
    CHECK(output.quaternion[2] == reference.quaternion[2], 12);
    CHECK(output.quaternion[3] == reference.quaternion[3], 13);

    /* Field magnitude gate: too weak and too strong are both rejected. */
    warmup_nwu_level(&time_us);
    output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, too_weak, 1U, &time_us);
    CHECK(output.magnetometer_subsystem_enabled == 1U, 14);
    CHECK(output.magnetometer_field_rejected == 1U, 15);
    CHECK(output.magnetometer_used == 0U, 16);
    CHECK(DRV_MAG_FieldMagnitude_InRange(too_weak) == 0U, 17);

    warmup_nwu_level(&time_us);
    output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, too_strong, 1U, &time_us);
    CHECK(output.magnetometer_field_rejected == 1U, 18);
    CHECK(output.magnetometer_used == 0U, 19);
    CHECK(DRV_MAG_FieldMagnitude_InRange(too_strong) == 0U, 20);

    /* A field magnitude inside the gate is actually used. */
    warmup_nwu_level(&time_us);
    output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, in_range, 1U, &time_us);
    CHECK(output.magnetometer_subsystem_enabled == 1U, 21);
    CHECK(output.magnetometer_field_rejected == 0U, 22);
    CHECK(output.magnetometer_used == 1U, 23);
    CHECK(DRV_MAG_FieldMagnitude_InRange(in_range) == 1U, 24);

    /*
     * The magnetometer measurably influences yaw once used, mirroring
     * tests/test_attitude_fusion_contract.py's "reproduce a gyro-only
     * error, then prove autonomous recovery" structure for the
     * accelerometer: a gyro-only burst (magnetometer disabled) creates a
     * ~40 degree yaw error with nothing to correct it (exactly the
     * "GPS-less, mag-less yaw has no absolute reference" failure mode this
     * task exists to fix), then a steady reference is fed for the recovery
     * phase. `in_range` was verified (see the harness derivation notes in
     * the .py docstring) to be self-consistent with yaw == 0 at identity
     * attitude in the NWU/FLU convention, so feeding it during recovery
     * asserts "the sensor says you never actually turned".
     */
    warmup_nwu_level(&time_us);
    for (index = 0; index < 1000; ++index) {
        output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                      0.0f, 0.0f, 40.0f, 0.0f, 0.0f, 1.0f,
                      NULL, 0U, &time_us);
    }
    yaw_no_mag = output.yaw_deg;
    CHECK(fabsf(yaw_no_mag) > 30.0f, 25); /* sanity: the burst really drifted */

    /* Without a magnetometer, that error is permanent: nothing observes yaw. */
    for (index = 0; index < 20000; ++index) {
        output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                      NULL, 0U, &time_us);
    }
    CHECK(fabsf(output.yaw_deg - yaw_no_mag) < 0.5f, 26);

    /* With a magnetometer, the same burst recovers to (near) the truth. */
    warmup_nwu_level(&time_us);
    for (index = 0; index < 1000; ++index) {
        output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                      0.0f, 0.0f, 40.0f, 0.0f, 0.0f, 1.0f,
                      NULL, 0U, &time_us);
    }
    for (index = 0; index < 20000; ++index) {
        output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                      in_range, 1U, &time_us);
    }
    yaw_with_mag = output.yaw_deg;
    CHECK(output.magnetometer_used == 1U, 27);
    CHECK(fabsf(yaw_with_mag) < 1.0f, 28);
    CHECK(fabsf(yaw_with_mag) < fabsf(yaw_no_mag) * 0.1f, 29);

    /*
     * Sustained field-magnitude rejection must still drive the recovery
     * trigger (mirrors the accelerometer norm-gate fix, tests 21/22 in
     * tests/test_attitude_fusion_contract.py): feeding a zero vector makes
     * FusionAhrsUpdate() skip its whole magnetic rejection/recovery block,
     * so without the driver manually advancing magneticRecoveryTrigger, a
     * long gated stretch would leave it frozen at 0 and the first good
     * sample afterwards would not benefit from recovery admission.
     */
    warmup_nwu_level(&time_us);
    for (index = 0; index < 600; ++index) {
        output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                      too_strong, 1U, &time_us);
    }
    CHECK(output.magnetometer_field_rejected != 0U, 30);
    CHECK(output.magnetic_recovery_trigger > 0.0f, 31);

    /* magnetometer_valid == 0 for the whole run never touches the trigger
     * (a vehicle with no magnetometer at all has nothing to recover). */
    warmup_nwu_level(&time_us);
    for (index = 0; index < 600; ++index) {
        output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                      NULL, 0U, &time_us);
    }
    CHECK(output.magnetic_recovery_trigger == 0.0f, 32);

    /*
     * The no-magnetometer branch must stay FusionAhrsUpdateNoMagnetometer(),
     * not FusionAhrsUpdate(..., FUSION_VECTOR_ZERO).
     *
     * Those two are NOT interchangeable, and the difference is invisible to
     * every check above. ThirdParty/Fusion/FusionAhrs.c:347-354:
     *
     *     void FusionAhrsUpdateNoMagnetometer(ahrs, gyroscope, accelerometer) {
     *         FusionAhrsUpdate(ahrs, gyroscope, accelerometer, FUSION_VECTOR_ZERO);
     *         if (ahrs->startup) {
     *             FusionAhrsSetHeading(ahrs, 0.0f);
     *         }
     *     }
     *
     * i.e. the wrapper additionally pins heading to zero for the whole
     * STARTUP_PERIOD. The bit-identical checks 7-13 cannot see this: they
     * compare two runs that BOTH take the no-magnetometer branch, so
     * collapsing the branch changes both sides equally and they still match.
     *
     * Observe it directly instead: inside startup, with no magnetometer, a
     * sustained yaw rate must NOT accumulate into yaw, because every tick
     * re-zeroes the heading. Under the collapsed form the gyro integrates
     * freely and yaw walks away (~25 deg for the rate/duration below).
     */
    DRV_AttitudeFusion_InitForConvention(DRV_ATTITUDE_FUSION_CONVENTION_NWU);
    time_us = 0ULL;
    for (index = 0; index < 500; ++index) {   /* 0.5 s, well inside 3 s startup */
        output = step(DRV_ATTITUDE_FUSION_CONVENTION_NWU, 0U,
                      0.0f, 0.0f, 50.0f, 0.0f, 0.0f, 1.0f,
                      NULL, 0U, &time_us);
    }
    CHECK(output.yaw_deg > -1.0f && output.yaw_deg < 1.0f, 33);

    return 0;
}
"""


def test_attitude_fusion_magnetometer_runtime(tmp_path: Path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "attitude_fusion_mag_harness.c"
    executable = tmp_path / "attitude_fusion_mag_harness.exe"
    harness.write_text(HARNESS, encoding="ascii")
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
            str(ROOT / "Driver" / "Src" / "drv_mag_calibration.c"),
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
