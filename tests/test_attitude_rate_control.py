"""Host-gcc contract tests for the pure SO(3) attitude/rate cascade."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


HARNESS = r"""
#include "drv_attitude_control.h"
#include "drv_rate_control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(x, n) do { if (!(x)) { fprintf(stderr, "check %d failed\n", (n)); return (n); } } while (0)
#define NEAR(x, y, e) (fabsf((x) - (y)) <= (e))
#define PI 3.14159265358979323846f

static void identity(float r[3][3])
{
    memset(r, 0, sizeof(float) * 9U);
    r[0][0] = r[1][1] = r[2][2] = 1.0f;
}

static void rz(float angle, float r[3][3])
{
    identity(r);
    r[0][0] = r[1][1] = cosf(angle);
    r[0][1] = -sinf(angle);
    r[1][0] = sinf(angle);
}

static void rx(float angle, float r[3][3])
{
    identity(r);
    r[1][1] = r[2][2] = cosf(angle);
    r[1][2] = -sinf(angle);
    r[2][1] = sinf(angle);
}

static DRV_RateControl_Params rate_params(void)
{
    DRV_RateControl_Params p = {0};
    for (int i = 0; i < 3; ++i) {
        p.kp[i] = 2.0f;
        p.ki[i] = 0.0f;
        p.kd[i] = 0.0f;
        p.integrator_limit[i] = 10.0f;
        p.large_error_threshold[i] = 100.0f;
        p.large_error_scale[i] = 1.0f;
        p.ff_gain[i] = 1.0f;
    }
    p.alpha_lpf_cutoff_rad_s = 0.0f;
    return p;
}

static DRV_RateControl_Input rate_input(void)
{
    DRV_RateControl_Input in = {0};
    for (int i = 0; i < 3; ++i) {
        in.inertia[i] = (float)(i + 1);
        in.saturation_positive[i] = 0.0f;
        in.saturation_negative[i] = 0.0f;
    }
    in.dt_s = 0.01f;
    in.measurement_valid = 1U;
    in.integrator_enable = 1U;
    return in;
}

int main(void)
{
    DRV_AttitudeControl_Params ap = {{2.0f, 2.0f, 2.0f}, {1.0f, 1.0f, 1.0f}};
    DRV_AttitudeControl_Input ai = {0};
    DRV_AttitudeControl_Output ao;
    float actual[3][3];
    float desired[3][3];

    /* SO(3) error is matrix based, finite at a large angle, and rate output is
     * a per-axis negative-feedback command with explicit clipping. */
    identity(actual);
    rz(0.2f, desired);
    memcpy(ai.actual_rotation, actual, sizeof(actual));
    memcpy(ai.desired_rotation, desired, sizeof(desired));
    ai.desired_rate_in_desired_frame[2] = 0.3f;
    CHECK(DRV_AttitudeControl_Step(&ap, &ai, &ao) != 0U, 1);
    CHECK(NEAR(ao.attitude_error[2], -sinf(0.2f), 1.0e-6f), 2);
    CHECK(NEAR(ao.omega_ff[2], 0.3f, 1.0e-6f), 3);
    CHECK(NEAR(ao.omega_sp[2], 0.3f + 2.0f * sinf(0.2f), 1.0e-6f), 4);

    ap.rate_limit_rad_s[2] = 0.1f;
    CHECK(DRV_AttitudeControl_Step(&ap, &ai, &ao) != 0U, 5);
    CHECK(NEAR(ao.omega_sp[2], 0.1f, 1.0e-6f) && ao.rate_saturated[2], 6);

    ai.actual_rotation[0][0] = 2.0f;
    CHECK(DRV_AttitudeControl_Step(&ap, &ai, &ao) == 0U, 9);

    /* R^T Rd maps desired-body rate into actual body rate under tilt. */
    rx(0.5f, actual);
    identity(desired);
    memcpy(ai.actual_rotation, actual, sizeof(actual));
    memcpy(ai.desired_rotation, desired, sizeof(desired));
    ai.desired_rate_in_desired_frame[0] = 0.0f;
    ai.desired_rate_in_desired_frame[1] = 0.0f;
    ai.desired_rate_in_desired_frame[2] = 1.0f;
    ap.rate_limit_rad_s[0] = ap.rate_limit_rad_s[1] = ap.rate_limit_rad_s[2] = 100.0f;
    CHECK(DRV_AttitudeControl_Step(&ap, &ai, &ao) != 0U, 7);
    CHECK(NEAR(ao.omega_ff[1], sinf(0.5f), 1.0e-6f) &&
          NEAR(ao.omega_ff[2], cosf(0.5f), 1.0e-6f), 8);

    /* Rate P/I/D/FF and rigid-body term.  The first sample seeds the measured
     * rate history; the next sample gives a known measured acceleration. */
    DRV_RateControl_Params rp = rate_params();
    DRV_RateControl_State rs;
    DRV_RateControl_Input ri = rate_input();
    DRV_RateControl_Output ro;
    DRV_RateControl_InitState(&rs);
    rp.kp[0] = 2.0f; rp.ki[0] = 1.0f; rp.kd[0] = 0.5f;
    rp.ff_gain[0] = 2.0f; rp.alpha_lpf_cutoff_rad_s = 0.0f;
    ri.omega_sp[0] = 1.0f; ri.alpha_ff[0] = 0.25f;
    ri.integrator_enable = 0U;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 10);
    ri.integrator_enable = 1U; ri.omega[0] = 0.1f; ri.dt_s = 0.1f;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 11);
    CHECK(NEAR(ro.error[0], 0.9f, 1.0e-6f), 12);
    CHECK(NEAR(ro.filtered_alpha[0], 1.0f, 1.0e-6f), 13);
    CHECK(NEAR(ro.p_term[0], 1.8f, 1.0e-6f), 14);
    CHECK(NEAR(ro.i_term[0], 0.09f, 1.0e-6f), 15);
    CHECK(NEAR(ro.d_term[0], 0.5f, 1.0e-6f), 16);
    CHECK(NEAR(ro.ff_term[0], 0.5f, 1.0e-6f), 17);
    /* omega x J omega, x-axis = (Jz-Jy)*wy*wz = 0 here. */
    CHECK(NEAR(ro.moment_cmd[0], 1.89f, 1.0e-5f), 18);

    /* LPF uses measured omega finite differences, not error differences. */
    DRV_RateControl_InitState(&rs);
    rp = rate_params(); rp.kd[0] = 1.0f; rp.alpha_lpf_cutoff_rad_s = 10.0f;
    ri = rate_input(); ri.dt_s = 0.1f;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 20);
    ri.omega[0] = 1.0f;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 21);
    CHECK(NEAR(ro.filtered_alpha[0], 10.0f * (1.0f - expf(-1.0f)), 1.0e-5f), 22);
    ri.dt_s = 1.0f; ri.omega[0] = 50.0f;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 23);
    CHECK(NEAR(ro.filtered_alpha[0], 0.0f, 1.0e-6f) && isfinite(ro.moment_cmd[0]), 24);

    /* Large-error weighting, I limits, freeze/reset, and both saturation
     * directions are independent per-axis. */
    DRV_RateControl_InitState(&rs);
    rp = rate_params(); rp.kp[0] = 0.0f; rp.ki[0] = 1.0f;
    rp.integrator_limit[0] = 0.25f; rp.large_error_threshold[0] = 0.5f;
    rp.large_error_scale[0] = 0.25f;
    ri = rate_input(); ri.dt_s = 0.1f; ri.omega_sp[0] = 1.0f;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 30);
    CHECK(NEAR(ro.i_term[0], 0.025f, 1.0e-6f), 31);
    ri.integrator_freeze = 1U;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 32);
    CHECK(NEAR(ro.i_term[0], 0.025f, 1.0e-6f), 33);
    ri.integrator_reset = 1U; ri.integrator_freeze = 0U;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 34);
    CHECK(NEAR(ro.i_term[0], 0.0f, 1.0e-6f), 35);

    DRV_RateControl_InitState(&rs);
    rp = rate_params(); rp.kp[0] = 0.0f; rp.ki[0] = 1.0f;
    ri = rate_input(); ri.dt_s = 0.1f; ri.omega_sp[0] = 1.0f;
    ri.saturation_positive[0] = 0.05f; ri.saturation_negative[0] = -0.05f;
    int positive_seen = 0;
    for (int i = 0; i < 10; ++i) {
        CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 40 + i);
        positive_seen |= ro.saturated_pos[0];
    }
    CHECK(NEAR(rs.integrator[0], 0.05f, 1.0e-6f) && positive_seen, 51);
    ri.omega_sp[0] = -1.0f;
    int negative_seen = 0;
    for (int i = 0; i < 10; ++i) {
        CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 60 + i);
        negative_seen |= ro.saturated_neg[0];
    }
    CHECK(NEAR(rs.integrator[0], -0.05f, 1.0e-6f) && negative_seen, 71);

    DRV_RateControl_InitState(&rs);
    ri = rate_input(); ri.omega_sp[0] = 1.0f;
    ri.saturation_positive_active[0] = 1U;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 72);
    CHECK(NEAR(rs.integrator[0], 0.0f, 1.0e-6f), 73);

    /* Invalid measurement/dt must not integrate or create a derivative spike;
     * invalid numeric input is rejected without producing a non-finite result. */
    ri.measurement_valid = 0U; ri.dt_s = 0.1f; ri.omega_sp[0] = 1.0f;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 80);
    CHECK(isfinite(ro.moment_cmd[0]), 81);
    ri.measurement_valid = 1U; ri.dt_s = 0.0f;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 82);
    CHECK(isfinite(ro.moment_cmd[0]), 83);
    ri.dt_s = 0.1f; ri.inertia[0] = NAN;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) == 0U, 84);

    rp = rate_params(); rp.kp[0] = -1.0f; ri = rate_input();
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) == 0U, 85);

    /* With I/D disabled, the cascade exactly gives the old SO(3) direct PD
     * signs, including reference-rate transform and rigid-body feedforward. */
    rz(0.03f, actual); rz(0.08f, desired);
    memset(&ai, 0, sizeof(ai)); memcpy(ai.actual_rotation, actual, sizeof(actual));
    memcpy(ai.desired_rotation, desired, sizeof(desired));
    ai.desired_rate_in_desired_frame[2] = 0.05f;
    ap.att_kp[2] = 2.0f; ap.rate_limit_rad_s[2] = 100.0f;
    CHECK(DRV_AttitudeControl_Step(&ap, &ai, &ao) != 0U, 90);
    DRV_RateControl_InitState(&rs);
    rp = rate_params(); rp.kp[2] = 3.0f; rp.ki[2] = rp.kd[2] = 0.0f;
    ri = rate_input(); ri.omega[2] = 0.02f; ri.omega_sp[2] = ao.omega_sp[2];
    ri.alpha_ff[2] = 0.0f; ri.inertia[0] = 2.0f; ri.inertia[1] = 3.0f; ri.inertia[2] = 5.0f;
    ri.omega[0] = 0.7f; ri.omega[1] = -0.4f;
    CHECK(DRV_RateControl_Step(&rp, &rs, &ri, &ro) != 0U, 91);
    CHECK(NEAR(ro.moment_cmd[2],
               3.0f * (0.05f + 2.0f * sinf(0.05f) - 0.02f) +
               (3.0f - 2.0f) * 0.7f * -0.4f, 1.0e-5f), 92);
    return 0;
}
"""


def test_attitude_rate_host_harness(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("gcc is required for pure-C controller harness")
    harness = tmp_path / "attitude_rate_harness.c"
    executable = tmp_path / "attitude_rate_harness.exe"
    harness.write_text(HARNESS, encoding="ascii")
    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(ROOT / "Driver" / "Src" / "drv_attitude_control.c"),
            str(ROOT / "Driver" / "Src" / "drv_rate_control.c"),
            str(harness),
            "-lm",
            "-o",
            str(executable),
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    result = subprocess.run([str(executable)], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_controller_modules_are_pure_and_so3_based() -> None:
    attitude = (ROOT / "Driver" / "Src" / "drv_attitude_control.c").read_text(
        encoding="utf-8"
    )
    rate = (ROOT / "Driver" / "Src" / "drv_rate_control.c").read_text(encoding="utf-8")
    headers = "\n".join(
        (ROOT / path).read_text(encoding="utf-8")
        for path in ("Driver/Inc/drv_attitude_control.h", "Driver/Inc/drv_rate_control.h")
    )
    forbidden = ("FreeRTOS", "cmsis_os", "stm32h7xx_hal", "HAL_", "printf(", "osDelay")
    assert not any(token in attitude + rate for token in forbidden)
    assert "R^T R_d" in attitude and "actual-minus-desired" in headers
    assert "omega x (J omega)" in rate
    assert "desired_rate_in_desired_frame" in headers
