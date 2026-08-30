"""R-F1 seam 1 (ESTIMATOR) FLU evidence contract.

`tests/test_attitude_fusion_contract.py` exercises the estimator only through
`DRV_AttitudeFusion_Init()`, i.e. the legacy NED convention fed with FRD
specific force.  The NWU convention -- the one the runtime actually selects
whenever a V0 candidate is active (`app_stabilizer.c`) -- had no executable
coverage at all, so nothing pinned that the estimator reports canonical FLU
signs, and nothing pinned the exported quaternion's direction or component
order.  The FLU reference requires both to be declared before attitude is
exported.

Seam 1 carries no runtime behaviour change (author ruling 2026-08-30, option
C).  This contract is the executable evidence for
`DRV_FRAME_MIGRATION_ESTIMATOR_ADAPTER_BIT`.

It also pins the legacy fallback's input transform exactly as it is today,
including the fact that its gyro transform is improper.  That is a finding, not
something this batch may repair: changing it would move an observable output
(legacy yaw sign) and AGENTS.md rule 4 reserves coordinate-sign changes for a
separately approved item.
"""

from __future__ import annotations

from pathlib import Path
import re
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
FUSION_HEADER = ROOT / "Driver" / "Inc" / "drv_attitude_fusion.h"
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_stabilizer_selects_nwu_for_flu_and_ned_for_legacy() -> None:
    """The convention must track the active body frame, not a build flag."""
    source = read(STABILIZER)
    assert re.search(
        r"DRV_AttitudeFusion_InitForConvention\(\s*\n?\s*"
        r"\(flu_active\s*!=\s*0U\)\s*\?\s*DRV_ATTITUDE_FUSION_CONVENTION_NWU\s*:\s*\n?"
        r"\s*DRV_ATTITUDE_FUSION_CONVENTION_NED\)",
        source,
    ), "estimator convention is no longer selected by the active FLU orientation"


def _fusion_input_block(source: str) -> str:
    start = source.index("if (msg->gyro_bias_ready != 0U) {")
    end = source.index("if (DRV_AttitudeFusion_Update(", start)
    return source[start:end]


def test_flu_branch_feeds_the_estimator_without_sign_compensation() -> None:
    """Seam 1's whole point: no scattered negations once the body frame is FLU."""
    block = _fusion_input_block(read(STABILIZER))
    flu_branch = block[block.index("if (flu_active != 0U) {"):block.index("} else {")]
    for axis, field in enumerate(("gyro_x_dps", "gyro_y_dps", "gyro_z_dps")):
        assert f"fusion_input.gyroscope_dps[{axis}] = msg->imu.{field};" in flu_branch
    for axis, field in enumerate(("accel_x_g", "accel_y_g", "accel_z_g")):
        assert f"fusion_input.accelerometer_g[{axis}] = msg->imu.{field};" in flu_branch
    assert "-msg->imu." not in flu_branch


def test_legacy_branch_transform_is_pinned_including_its_improper_gyro() -> None:
    """Pin the un-migrated fallback exactly as it is.

    Expressed in chip axes the legacy branch feeds the estimator
    accel diag(-1,+1,-1) (det = +1, a proper 180 deg rotation about Y that
    yields FRD) but gyro diag(-1,+1,+1) (det = -1, a reflection).  The two
    disagree in Z, so the legacy yaw rate carries the opposite sign to the
    frame its own accelerometer defines.  Yaw is unobservable without a
    magnetometer, which is why this survived; roll/pitch are unaffected.

    This test exists so the discrepancy cannot be "tidied" silently in either
    direction.  Repairing it changes an observable output and needs its own
    author-approved item.
    """
    block = _fusion_input_block(read(STABILIZER))
    legacy_branch = block[block.index("} else {"):]
    assert "fusion_input.gyroscope_dps[0] = -msg->imu.gyro_x_dps;" in legacy_branch
    assert "fusion_input.gyroscope_dps[1] =  msg->imu.gyro_y_dps;" in legacy_branch
    assert "fusion_input.gyroscope_dps[2] =  msg->imu.gyro_z_dps;" in legacy_branch
    assert "fusion_input.accelerometer_g[0] = -msg->imu.accel_x_g;" in legacy_branch
    assert "fusion_input.accelerometer_g[1] =  msg->imu.accel_y_g;" in legacy_branch
    assert "fusion_input.accelerometer_g[2] = -msg->imu.accel_z_g;" in legacy_branch


@pytest.mark.xfail(
    strict=True,
    reason="R-F1 red test: drv_attitude_fusion.h exports quaternion[4] without "
    "declaring direction, component order or the two named frames; cleared by "
    "the seam 1 implementation commit",
)
def test_header_declares_quaternion_direction_order_and_named_frames() -> None:
    """The FLU reference forbids exporting attitude without this declaration."""
    header = read(FUSION_HEADER)
    assert "body-to-navigation" in header
    assert "(w, x, y, z)" in header
    # Both conventions must name their body frame and their navigation frame.
    assert "FLU" in header and "NWU" in header
    assert "FRD" in header and "NED" in header


SEAM1_HARNESS = r"""
#include "drv_attitude_fusion.h"
#include "drv_frame_contract.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define CHECK(condition, code) do { if (!(condition)) return (code); } while (0)

static uint64_t harness_time_us;

static DRV_AttitudeFusionOutput settle(float ax, float ay, float az,
                                       float gx, float gy, float gz, int n)
{
    DRV_AttitudeFusionOutput out = {0};
    int i;

    for (i = 0; i < n; ++i) {
        DRV_AttitudeFusionInput in = {0};
        harness_time_us += 1000ULL;
        in.time_us = harness_time_us;
        in.dt_s = 0.001f;
        in.gyroscope_dps[0] = gx;
        in.gyroscope_dps[1] = gy;
        in.gyroscope_dps[2] = gz;
        in.accelerometer_g[0] = ax;
        in.accelerometer_g[1] = ay;
        in.accelerometer_g[2] = az;
        (void)DRV_AttitudeFusion_Update(&in, &out);
    }
    return out;
}

int main(void)
{
    const float d2r = 3.14159265358979f / 180.0f;
    DRV_AttitudeFusionOutput o;
    float phi, th;

    /*
     * Level and stationary in canonical FLU: the contract's level specific
     * force must settle to zero attitude and an identity quaternion.
     */
    DRV_AttitudeFusion_InitForConvention(DRV_ATTITUDE_FUSION_CONVENTION_NWU);
    harness_time_us = 0ULL;
    o = settle(DRV_FRAME_LEVEL_SPECIFIC_FORCE_X_G,
               DRV_FRAME_LEVEL_SPECIFIC_FORCE_Y_G,
               DRV_FRAME_LEVEL_SPECIFIC_FORCE_Z_G,
               0.0f, 0.0f, 0.0f, 20000);
    CHECK(fabsf(o.roll_deg) < 0.05f, 1);
    CHECK(fabsf(o.pitch_deg) < 0.05f, 2);
    CHECK(fabsf(o.quaternion[0]) > 0.999f, 3);
    CHECK(fabsf(o.quaternion[1]) < 0.001f, 4);
    CHECK(fabsf(o.quaternion[2]) < 0.001f, 5);
    CHECK(fabsf(o.quaternion[3]) < 0.001f, 6);

    /*
     * DRV_FRAME_POSITIVE_ROLL_IS_RIGHT_WING_DOWN.  Rolling the body +phi about
     * +X puts the nav up-vector at [0, sin phi, cos phi] in body coordinates.
     * The estimator must report +phi, and the quaternion must be a pure +X
     * rotation of phi/2 -- i.e. body-to-navigation in (w, x, y, z) order.
     */
    phi = 30.0f * d2r;
    DRV_AttitudeFusion_InitForConvention(DRV_ATTITUDE_FUSION_CONVENTION_NWU);
    harness_time_us = 0ULL;
    o = settle(0.0f, sinf(phi), cosf(phi), 0.0f, 0.0f, 0.0f, 20000);
    CHECK(fabsf(o.roll_deg - 30.0f) < 0.1f, 10);
    CHECK(fabsf(o.pitch_deg) < 0.1f, 11);
    CHECK(fabsf(o.quaternion[0] - cosf(phi * 0.5f)) < 0.002f, 12);
    CHECK(fabsf(o.quaternion[1] - sinf(phi * 0.5f)) < 0.002f, 13);
    CHECK(fabsf(o.quaternion[2]) < 0.002f, 14);
    CHECK(fabsf(o.quaternion[3]) < 0.002f, 15);

    /*
     * DRV_FRAME_POSITIVE_PITCH_IS_NOSE_DOWN.  Pitching +theta about +Y puts
     * the nav up-vector at [-sin theta, 0, cos theta] in body coordinates.
     */
    th = 20.0f * d2r;
    DRV_AttitudeFusion_InitForConvention(DRV_ATTITUDE_FUSION_CONVENTION_NWU);
    harness_time_us = 0ULL;
    o = settle(-sinf(th), 0.0f, cosf(th), 0.0f, 0.0f, 0.0f, 20000);
    CHECK(fabsf(o.pitch_deg - 20.0f) < 0.1f, 20);
    CHECK(fabsf(o.roll_deg) < 0.1f, 21);
    CHECK(fabsf(o.quaternion[0] - cosf(th * 0.5f)) < 0.002f, 22);
    CHECK(fabsf(o.quaternion[1]) < 0.002f, 23);
    CHECK(fabsf(o.quaternion[2] - sinf(th * 0.5f)) < 0.002f, 24);
    CHECK(fabsf(o.quaternion[3]) < 0.002f, 25);

    /*
     * DRV_FRAME_POSITIVE_YAW_IS_NOSE_LEFT.  A positive body yaw rate must
     * integrate into a positive heading, so the sign survives the estimator.
     */
    DRV_AttitudeFusion_InitForConvention(DRV_ATTITUDE_FUSION_CONVENTION_NWU);
    harness_time_us = 0ULL;
    (void)settle(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 20000);
    o = settle(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 10.0f, 1000);
    CHECK(fabsf(o.yaw_deg - 10.0f) < 0.2f, 30);
    CHECK(fabsf(o.roll_deg) < 0.2f, 31);
    CHECK(fabsf(o.pitch_deg) < 0.2f, 32);

    /*
     * The legacy convention must stay reachable and keep its own polarity, so
     * an unmigrated board is not silently re-signed by this seam.  In NED the
     * level specific force is FRD [0, 0, -1].
     */
    DRV_AttitudeFusion_InitForConvention(DRV_ATTITUDE_FUSION_CONVENTION_NED);
    harness_time_us = 0ULL;
    o = settle(0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 20000);
    CHECK(fabsf(o.roll_deg) < 0.05f, 40);
    CHECK(fabsf(o.pitch_deg) < 0.05f, 41);

    puts("ok");
    return 0;
}
"""


def test_estimator_reports_canonical_flu_signs_under_nwu(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.fail("seam 1 FLU contract requires host gcc or clang")

    harness = tmp_path / "estimator_seam1.c"
    harness.write_text(SEAM1_HARNESS, encoding="ascii")
    executable = tmp_path / "estimator_seam1.exe"
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
    result = subprocess.run(
        [str(executable)], check=True, capture_output=True, text=True
    )
    assert result.stdout.strip() == "ok"
