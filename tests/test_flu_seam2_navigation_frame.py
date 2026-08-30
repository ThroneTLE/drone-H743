"""R-F2 seam 2 (NAVIGATION) FLU migration contract."""

# drv_imu_nav.c resolves specific force into a local-level frame built from the
# gravity reference.  Before this seam that frame was (forward, right, down):
# level_z_body_unit held the *down* axis (z = -accel), y = cross(z, x) resolved
# to right, and the vertical channel added +gravity.  Seam 2 migrates it to
# local-level FLU (forward, left, up).
#
# Downstream is not migrated yet.  nav_state.vel_m_s feeds the velocity
# estimator and reaches the wire as ekf_vx_mm_s / ekf_vy_mm_s from
# App/Src/app_control.c, so app_stabilizer.c converts back at the boundary.
# Every one of those observable numbers must stay bit-identical.
#
# The transform is a signed axis flip and IEEE arithmetic is sign-symmetric for
# every operation involved (division, dot, cross, EMA, integration), so the
# equivalence is exact, not approximate.  This module proves it by running the
# real migrated driver against a reference implementation of the pre-migration
# algorithm and requiring exact equality through the adapter.

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
NAV_SOURCE = ROOT / "Driver" / "Src" / "drv_imu_nav.c"
NAV_HEADER = ROOT / "Driver" / "Inc" / "drv_imu_nav.h"
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"

ADAPTER = "stabilizer_nav_flu_to_legacy_fwd_right_down"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_nav_builds_local_level_flu() -> None:
    """The gravity reference must be the up axis, and gravity must subtract."""
    source = read(NAV_SOURCE)
    assert "level_z_body_unit[0] = -input->accel_x_g" not in source
    assert "z_body[0] = input->accel_x_g;" in source
    assert "imu_nav_dot3(z_body, acc_body_m_s2) - gravity" in source


def test_nav_header_names_frame() -> None:
    header = read(NAV_HEADER)
    assert "local-level FLU" in header
    assert "level_z_body_unit" in header


def test_boundary_adapter_declares_removal() -> None:
    """The adapter must say, in words, when it is to be deleted."""
    source = read(STABILIZER)
    assert f"static void {ADAPTER}(" in source
    start = source.index(f"static void {ADAPTER}(")
    context = source[max(0, start - 900):start]
    assert "临时边界适配" in context
    assert "seam 3" in context or "seam 4" in context


NAV_HARNESS = r"""
#include "drv_imu_nav.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { if (!(condition)) return (code); } while (0)

/* ---- Reference implementation of the pre-seam-2 (forward, right, down)
 * algorithm, transcribed from drv_imu_nav.c at commit 77f658f4. ---- */

#define LEG_ALPHA 0.995f
#define LEG_MIN_ACCEL_NORM_G 0.10f
#define LEG_MIN_AXIS_NORM 1.0e-4f

typedef struct {
    float acc_bias_nav_m_s2[3];
    float acc_nav_m_s2[3];
    float vel_m_s[3];
    float level_z_body_unit[3];
    uint8_t bias_ready;
    uint8_t level_ready;
} LegacyState;

static float leg_clamp(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static float leg_dot(const float a[3], const float b[3])
{
    return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

static void leg_cross(const float a[3], const float b[3], float out[3])
{
    out[0] = (a[1] * b[2]) - (a[2] * b[1]);
    out[1] = (a[2] * b[0]) - (a[0] * b[2]);
    out[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

static uint8_t leg_normalize(float v[3])
{
    const float norm = sqrtf(leg_dot(v, v));
    if (norm <= LEG_MIN_AXIS_NORM) { return 0U; }
    v[0] /= norm; v[1] /= norm; v[2] /= norm;
    return 1U;
}

static void leg_level(LegacyState *s, const DRV_IMU_NAV_Input *in)
{
    float z[3];
    float n;

    z[0] = -in->accel_x_g;
    z[1] = -in->accel_y_g;
    z[2] = -in->accel_z_g;
    n = sqrtf(leg_dot(z, z));
    if (n < LEG_MIN_ACCEL_NORM_G) { return; }
    z[0] /= n; z[1] /= n; z[2] /= n;

    if (s->level_ready == 0U) {
        s->level_z_body_unit[0] = z[0];
        s->level_z_body_unit[1] = z[1];
        s->level_z_body_unit[2] = z[2];
        s->level_ready = 1U;
        return;
    }
    s->level_z_body_unit[0] =
        (LEG_ALPHA * s->level_z_body_unit[0]) + ((1.0f - LEG_ALPHA) * z[0]);
    s->level_z_body_unit[1] =
        (LEG_ALPHA * s->level_z_body_unit[1]) + ((1.0f - LEG_ALPHA) * z[1]);
    s->level_z_body_unit[2] =
        (LEG_ALPHA * s->level_z_body_unit[2]) + ((1.0f - LEG_ALPHA) * z[2]);
    if (leg_normalize(s->level_z_body_unit) == 0U) { s->level_ready = 0U; }
}

static void leg_acc_body_to_nav(const DRV_IMU_NAV_Input *in,
                                const float level_z[3], float out[3])
{
    const float g = (in->gravity_m_s2 > 0.0f) ? in->gravity_m_s2 : 9.80665f;
    const float acc[3] = {
        in->accel_x_g * g, in->accel_y_g * g, in->accel_z_g * g,
    };
    float z[3] = { level_z[0], level_z[1], level_z[2] };
    float x[3] = {1.0f, 0.0f, 0.0f};
    float y[3];
    float xz;

    if (leg_normalize(z) == 0U) { z[0] = 0.0f; z[1] = 0.0f; z[2] = 1.0f; }

    xz = leg_dot(x, z);
    x[0] -= xz * z[0]; x[1] -= xz * z[1]; x[2] -= xz * z[2];
    if (leg_normalize(x) == 0U) {
        x[0] = 0.0f; x[1] = 1.0f; x[2] = 0.0f;
        xz = leg_dot(x, z);
        x[0] -= xz * z[0]; x[1] -= xz * z[1]; x[2] -= xz * z[2];
        (void)leg_normalize(x);
    }
    leg_cross(z, x, y);

    out[0] = leg_dot(x, acc);
    out[1] = leg_dot(y, acc);
    out[2] = leg_dot(z, acc) + g;
}

static void leg_capture_bias(LegacyState *s, const DRV_IMU_NAV_Input *in)
{
    leg_level(s, in);
    leg_acc_body_to_nav(in, s->level_z_body_unit, s->acc_bias_nav_m_s2);
    s->bias_ready = 1U;
}

static void leg_update(LegacyState *s, const DRV_IMU_NAV_Input *in)
{
    float raw[3];
    float dt = ((in->dt_sec > 0.0f) && (in->dt_sec <= 0.02f)) ? in->dt_sec : 0.001f;
    const float alpha = leg_clamp(in->accel_lpf_alpha, 0.0f, 1.0f);
    const float leak = leg_clamp(in->velocity_leak_rate_hz, 0.0f, 5.0f);
    uint32_t axis;

    leg_level(s, in);
    leg_acc_body_to_nav(in, s->level_z_body_unit, raw);
    for (axis = 0U; axis < 3U; ++axis) {
        float a = raw[axis];
        if (s->bias_ready != 0U) { a -= s->acc_bias_nav_m_s2[axis]; }
        s->acc_nav_m_s2[axis] = alpha * s->acc_nav_m_s2[axis] + (1.0f - alpha) * a;
        s->vel_m_s[axis] += s->acc_nav_m_s2[axis] * dt;
        s->vel_m_s[axis] -= s->vel_m_s[axis] * leak * dt;
    }
}

/* ---- The seam 2/3 boundary adapter, mirrored from app_stabilizer.c. ---- */
static void to_legacy(const float flu[3], float legacy[3])
{
    legacy[0] =  flu[0];
    legacy[1] = -flu[1];
    legacy[2] = -flu[2];
}

static int exact3(const float flu[3], const float legacy[3])
{
    float converted[3];
    to_legacy(flu, converted);
    return converted[0] == legacy[0]
        && converted[1] == legacy[1]
        && converted[2] == legacy[2];
}

/*
 * level_z_body_unit is not a navigation-frame vector, so it does not take the
 * boundary adapter.  It is the navigation Z axis expressed in *body*
 * coordinates, and seam 2 turns it from down into up, which negates all three
 * components.
 */
static int exact_negated3(const float up_axis[3], const float down_axis[3])
{
    return up_axis[0] == -down_axis[0]
        && up_axis[1] == -down_axis[1]
        && up_axis[2] == -down_axis[2];
}

int main(void)
{
    DRV_IMU_NAV_State migrated;
    LegacyState legacy;
    DRV_IMU_NAV_Input in;
    int step;

    DRV_IMU_NAV_Reset(&migrated);
    memset(&legacy, 0, sizeof(legacy));
    memset(&in, 0, sizeof(in));
    in.dt_sec = 0.002f;
    in.gravity_m_s2 = 9.80665f;
    in.accel_lpf_alpha = 0.90f;
    in.velocity_leak_rate_hz = 0.20f;

    /* Level and stationary in canonical FLU. */
    in.accel_x_g = 0.0f; in.accel_y_g = 0.0f; in.accel_z_g = 1.0f;
    DRV_IMU_NAV_CaptureBias(&migrated, &in);
    leg_capture_bias(&legacy, &in);
    CHECK(exact3(migrated.acc_bias_nav_m_s2, legacy.acc_bias_nav_m_s2), 1);

    /*
     * The migrated gravity reference is the UP axis: level FLU specific force
     * is +Z, so level_z_body_unit must be +Z, and the legacy reference must
     * hold exactly its negation.
     */
    CHECK(migrated.level_z_body_unit[2] > 0.99f, 2);
    CHECK(legacy.level_z_body_unit[2] < -0.99f, 3);

    /* At rest the vertical channel must cancel to zero in both frames. */
    CHECK(fabsf(migrated.acc_bias_nav_m_s2[2]) < 1.0e-4f, 4);

    /*
     * Drive a trajectory that exercises tilt, lateral and vertical dynamics.
     * Every sample stays above the 0.10 g gate so the degenerate fallback is
     * not involved in the equivalence proof.
     */
    for (step = 0; step < 3000; ++step) {
        const float t = (float)step * 0.002f;
        in.accel_x_g = 0.25f * sinf(1.7f * t);
        in.accel_y_g = 0.30f * sinf(2.3f * t + 0.4f);
        in.accel_z_g = 1.0f + 0.20f * sinf(3.1f * t + 1.1f);
        DRV_IMU_NAV_Update(&migrated, &in);
        leg_update(&legacy, &in);

        CHECK(exact3(migrated.acc_nav_m_s2, legacy.acc_nav_m_s2), 10);
        CHECK(exact3(migrated.vel_m_s, legacy.vel_m_s), 11);
        CHECK(exact_negated3(migrated.level_z_body_unit,
                             legacy.level_z_body_unit), 12);
    }

    /*
     * Frame orientation checks on the migrated driver, held level so the nav
     * axes coincide with the body axes.
     */
    DRV_IMU_NAV_Reset(&migrated);
    in.accel_lpf_alpha = 0.0f;
    in.velocity_leak_rate_hz = 0.0f;
    in.accel_x_g = 0.0f; in.accel_y_g = 0.0f; in.accel_z_g = 1.0f;
    DRV_IMU_NAV_CaptureBias(&migrated, &in);

    /* Specific force greater than gravity = accelerating upward = nav +Z. */
    in.accel_z_g = 1.5f;
    DRV_IMU_NAV_Update(&migrated, &in);
    CHECK(migrated.acc_nav_m_s2[2] > 0.0f, 20);

    /* Body +Y is left; the local-level Y axis must agree in sign. */
    DRV_IMU_NAV_Reset(&migrated);
    in.accel_x_g = 0.0f; in.accel_y_g = 0.0f; in.accel_z_g = 1.0f;
    DRV_IMU_NAV_CaptureBias(&migrated, &in);
    in.accel_y_g = 0.20f;
    DRV_IMU_NAV_Update(&migrated, &in);
    CHECK(migrated.acc_nav_m_s2[1] > 0.0f, 21);

    /* Body +X is forward; the local-level X axis must agree in sign. */
    DRV_IMU_NAV_Reset(&migrated);
    in.accel_y_g = 0.0f; in.accel_z_g = 1.0f;
    DRV_IMU_NAV_CaptureBias(&migrated, &in);
    in.accel_x_g = 0.20f;
    DRV_IMU_NAV_Update(&migrated, &in);
    CHECK(migrated.acc_nav_m_s2[0] > 0.0f, 22);

    puts("ok");
    return 0;
}
"""


def test_nav_bit_exact_through_adapter(
    tmp_path: Path,
) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.fail("seam 2 FLU contract requires host gcc or clang")

    harness = tmp_path / "nav_seam2.c"
    harness.write_text(NAV_HARNESS, encoding="ascii")
    executable = tmp_path / "nav_seam2.exe"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(ROOT / "Driver" / "Src" / "drv_imu_nav.c"),
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
