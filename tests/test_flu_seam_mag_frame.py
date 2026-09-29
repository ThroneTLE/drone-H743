"""Magnetometer mounting-adapter contract (Services/Inc/svc_mag.h).

This is deliberately named `test_flu_seam_mag_frame.py`, not
`test_flu_seam<N>_*.py`: it is not one of the six
`DRV_FRAME_RUNTIME_MIGRATION_*` seams (see the reviewer's C5 ruling recorded
in flu-coordinate-contract.md) and must not be picked up by
`test_done_mask_bits_have_seam_test_files`'s glob.

Pins, independently of the implementation under test:
  * svc_mag.h has no HAL/RTOS dependency (D1-2, D5-1): it must compile and be
    usable on a host with only drv_frame_contract.h, no "main.h"/HAL/CMSIS.
  * The C2 mounting derivation for MicoAir743v2's on-board QMC5883L
    (hwdef ROTATION_NONE -> FRD -> DRV_FRAME_FrdToFlu) composes to
    flu = (chip.x, -chip.y, -chip.z). The expected values below are written
    by hand from that derivation, not produced by calling the function under
    test -- calling SVC_MAG_RotateToFlu()/SVC_MAG_DefaultRotation() to
    compute their own expectation would be self-certifying.
  * Every one of the 8 supported mounting rotations is a proper rotation
    (determinant +1, no mirror) -- an accidental reflection would still pass
    a single-axis gravity/field-strength sanity check while quietly
    inverting every other axis.
  * The default axis-verification state is UNVERIFIED (numeric 0), so a
    zero-initialized/never-touched state is safe by construction.
"""

from __future__ import annotations

import re
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_DIR = ROOT / "Driver" / "Inc"
SERVICES_DIR = ROOT / "Services" / "Inc"
HEADER = SERVICES_DIR / "svc_mag.h"
SOURCE = ROOT / "Services" / "Src" / "svc_mag.c"


def test_header_has_no_hal_or_rtos_dependency() -> None:
    """decoupling-spec D1-2: Services/ must not #include HAL/RTOS headers.

    svc_mag.h deliberately avoids Driver/Inc/drv_mag.h (which drags in
    "main.h"/HAL just to declare an I2C handle) precisely so this module
    stays host-compilable. A regression here would silently break every
    downstream PC-side unit test that includes this header.
    """
    text = HEADER.read_text(encoding="utf-8")
    # Only actual #include directives matter here; the header's own doc
    # comments discuss "main.h"/"drv_mag.h" in prose to explain *why* they
    # are avoided, so a plain substring search would false-positive on its
    # own rationale.
    includes = re.findall(r'^\s*#include\s*[<"]([^">]+)[">]', text, re.MULTILINE)
    forbidden = {"main.h", "stm32h7xx_hal.h", "cmsis_os.h", "cmsis_os2.h", "drv_mag.h"}
    hit = forbidden.intersection(includes)
    assert not hit, f"svc_mag.h must not #include {hit}"


MAG_SEAM_HARNESS = r"""
#include "svc_mag.h"

#include <stdio.h>

#define CHECK(condition, code) do { if (!(condition)) { \
    printf("FAIL %d\n", (code)); return (code); } } while (0)

static int near(float lhs, float rhs)
{
    float d = lhs - rhs;
    if (d < 0.0f) { d = -d; }
    return d < 1.0e-6f;
}

/* Determinant of the 3x3 matrix whose columns are the images of the FLU
 * basis vectors under `rotation`. +1 means a proper rotation (no mirror). */
static float rotation_determinant(SVC_MAG_Rotation rotation)
{
    const DRV_FRAME_Vector3f ex = {1.0f, 0.0f, 0.0f};
    const DRV_FRAME_Vector3f ey = {0.0f, 1.0f, 0.0f};
    const DRV_FRAME_Vector3f ez = {0.0f, 0.0f, 1.0f};
    DRV_FRAME_Vector3f cx = SVC_MAG_RotateToFlu(rotation, ex);
    DRV_FRAME_Vector3f cy = SVC_MAG_RotateToFlu(rotation, ey);
    DRV_FRAME_Vector3f cz = SVC_MAG_RotateToFlu(rotation, ez);

    return cx.x * (cy.y * cz.z - cy.z * cz.y)
         - cy.x * (cx.y * cz.z - cx.z * cz.y)
         + cz.x * (cx.y * cy.z - cx.z * cy.y);
}

int main(void)
{
    const SVC_MAG_Rotation all_rotations[8] = {
        SVC_MAG_ROTATION_NONE,
        SVC_MAG_ROTATION_YAW_90,
        SVC_MAG_ROTATION_YAW_180,
        SVC_MAG_ROTATION_YAW_270,
        SVC_MAG_ROTATION_ROLL_180,
        SVC_MAG_ROTATION_ROLL_180_YAW_90,
        SVC_MAG_ROTATION_PITCH_180,
        SVC_MAG_ROTATION_ROLL_180_YAW_270,
    };
    int i;

    /* --- C2: default rotation is ROTATION_NONE for MicoAir743v2. --- */
    CHECK(SVC_MAG_DefaultRotation() == SVC_MAG_ROTATION_NONE, 1);

    /*
     * --- C2 pinned result, derived by hand (NOT via the function under
     * test): ROTATION_NONE means chip axis == FRD axis (identity), then
     * FRD -> FLU is the contract's diag(+1,-1,-1). So for any chip vector
     * (cx, cy, cz):
     *     flu = ( cx, -cy, -cz )
     * Three independent sample vectors, including one with all-distinct
     * nonzero components so a component swap (not just a sign error) would
     * also be caught.
     */
    {
        const DRV_FRAME_Vector3f chip_x = {1.0f, 0.0f, 0.0f};
        const DRV_FRAME_Vector3f chip_y = {0.0f, 1.0f, 0.0f};
        const DRV_FRAME_Vector3f chip_z = {0.0f, 0.0f, 1.0f};
        const DRV_FRAME_Vector3f chip_arbitrary = {2.0f, -3.0f, 5.0f};
        DRV_FRAME_Vector3f flu;

        flu = SVC_MAG_RotateToFlu(SVC_MAG_ROTATION_NONE, chip_x);
        CHECK(near(flu.x, 1.0f) && near(flu.y, 0.0f) && near(flu.z, 0.0f), 10);

        flu = SVC_MAG_RotateToFlu(SVC_MAG_ROTATION_NONE, chip_y);
        CHECK(near(flu.x, 0.0f) && near(flu.y, -1.0f) && near(flu.z, 0.0f), 11);

        flu = SVC_MAG_RotateToFlu(SVC_MAG_ROTATION_NONE, chip_z);
        CHECK(near(flu.x, 0.0f) && near(flu.y, 0.0f) && near(flu.z, -1.0f), 12);

        /* Independently-rewritten expectation: flu = (x, -y, -z). */
        flu = SVC_MAG_RotateToFlu(SVC_MAG_ROTATION_NONE, chip_arbitrary);
        CHECK(near(flu.x, 2.0f), 13);
        CHECK(near(flu.y, 3.0f), 14);
        CHECK(near(flu.z, -5.0f), 15);
    }

    /* --- Every supported mounting rotation is a proper rotation. --- */
    for (i = 0; i < 8; i++) {
        float det = rotation_determinant(all_rotations[i]);
        CHECK(near(det, 1.0f), 20 + i);
    }

    /* --- Axis verification: default is UNVERIFIED, numerically 0. --- */
    CHECK((int)SVC_MAG_AXIS_UNVERIFIED == 0, 40);
    CHECK((int)SVC_MAG_AXIS_VERIFIED == 1, 41);
    CHECK(SVC_MAG_AxisUsableForFusion(SVC_MAG_AXIS_UNVERIFIED) == 0U, 42);
    CHECK(SVC_MAG_AxisUsableForFusion(SVC_MAG_AXIS_VERIFIED) == 1U, 43);
    {
        /* A zero-initialized struct field must land on UNVERIFIED without
         * any extra "have I been touched" flag. */
        SVC_MAG_AxisVerification zeroed = (SVC_MAG_AxisVerification)0;
        CHECK(zeroed == SVC_MAG_AXIS_UNVERIFIED, 44);
        CHECK(SVC_MAG_AxisUsableForFusion(zeroed) == 0U, 45);
    }

    puts("ok");
    return 0;
}
"""


def test_mag_mounting_matches_independent_expectation(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.fail("magnetometer FLU seam requires host gcc or clang")

    harness = tmp_path / "mag_seam_harness.c"
    harness.write_text(MAG_SEAM_HARNESS, encoding="utf-8")
    executable = tmp_path / "mag_seam.exe"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{CONTRACT_DIR}",
            f"-I{SERVICES_DIR}",
            str(harness),
            str(SOURCE),
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
