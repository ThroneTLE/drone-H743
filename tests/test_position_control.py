"""R-S5-1 pure translational controller contract.

The executable part compiles the real Driver C module with host gcc.  The
inputs are deliberately synthetic unit-test vectors; they are not flight
evidence and make no performance claim about the controller.
"""

from __future__ import annotations

import pathlib
import shutil
import subprocess
import textwrap

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
HEADER = ROOT / "Driver" / "Inc" / "drv_position_control.h"
SOURCE = ROOT / "Driver" / "Src" / "drv_position_control.c"


HARNESS = r"""
#include "drv_position_control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { if (!(condition)) return (code); } while (0)
#define NEAR(actual, expected) (fabsf((actual) - (expected)) < 0.0005f)

static DRV_POSITION_CONTROL_Params params(void)
{
    DRV_POSITION_CONTROL_Params p = {0};
    p.pos_kp[0] = 2.0f; p.pos_kp[1] = 3.0f; p.pos_kp[2] = 4.0f;
    p.vel_kp[0] = 2.0f; p.vel_kp[1] = 3.0f; p.vel_kp[2] = 4.0f;
    p.vel_ki[0] = 1.0f; p.vel_ki[1] = 1.0f; p.vel_ki[2] = 1.0f;
    p.vel_kd[0] = 0.5f; p.vel_kd[1] = 0.5f; p.vel_kd[2] = 0.5f;
    p.xy_speed_limit_m_s = 10.0f;
    p.z_speed_limit_up_m_s = 1.5f;
    p.z_speed_limit_down_m_s = 1.25f;
    p.vel_integrator_limit[0] = 0.2f;
    p.vel_integrator_limit[1] = 0.3f;
    p.vel_integrator_limit[2] = 0.4f;
    p.xy_accel_limit_m_s2 = 100.0f;
    p.z_accel_limit_up_m_s2 = 100.0f;
    p.z_accel_limit_down_m_s2 = 100.0f;
    return p;
}

static int position_contract(void)
{
    DRV_POSITION_CONTROL_Params p = params();
    DRV_POSITION_CONTROL_PositionInput in = {0};
    DRV_POSITION_CONTROL_PositionOutput out = {0};

    in.position_sp_m[0] = 3.0f; in.position_sp_m[1] = -2.0f; in.position_sp_m[2] = 1.0f;
    in.position_meas_m[0] = 1.0f; in.position_meas_m[1] = 0.0f; in.position_meas_m[2] = 1.5f;
    in.dt_sec = 0.01f; in.measurement_valid = 1U;
    DRV_POSITION_CONTROL_PositionStep(&p, &in, &out);
    /* Old outer-loop equivalence: Kp_pos * (position_sp - position). */
    CHECK(NEAR(out.velocity_sp_m_s[0], 4.0f), 1);
    CHECK(NEAR(out.velocity_sp_m_s[1], -6.0f), 2);
    /* R-F6-2: +Z is up, so negative excess is now clamped by down_limit (1.25),
     * not up_limit (1.5). */
    CHECK(NEAR(out.velocity_sp_m_s[2], -1.25f), 3);
    CHECK(out.sat.neg_limit[2] != 0U && out.sat.thrust_saturated != 0U, 4);

    p.xy_speed_limit_m_s = 4.0f;
    in.position_sp_m[0] = 1.5f; in.position_meas_m[0] = 0.0f;
    in.position_sp_m[1] = 1.3333333f; in.position_meas_m[1] = 0.0f;
    in.position_sp_m[2] = 0.0f; in.position_meas_m[2] = 0.0f;
    DRV_POSITION_CONTROL_PositionStep(&p, &in, &out);
    CHECK(NEAR(out.velocity_sp_m_s[0], 2.4f), 5);
    CHECK(NEAR(out.velocity_sp_m_s[1], 3.2f), 6);
    CHECK(out.sat.tilt_saturated != 0U && NEAR(out.sat.horizontal_scale, 0.8f), 7);
    CHECK(out.sat.pos_limit[0] != 0U && out.sat.pos_limit[1] != 0U, 8);

    p.xy_speed_limit_m_s = 10.0f;
    in.position_bypass = 1U;
    in.direct_velocity_m_s[0] = -2.0f;
    in.direct_velocity_m_s[1] = 1.0f;
    in.direct_velocity_m_s[2] = 2.0f;
    DRV_POSITION_CONTROL_PositionStep(&p, &in, &out);
    CHECK(NEAR(out.velocity_sp_m_s[0], -2.0f) && NEAR(out.velocity_sp_m_s[1], 1.0f), 9);
    /* R-F6-2: +Z is up, so positive excess is now clamped by up_limit (1.5),
     * not down_limit (1.25). */
    CHECK(NEAR(out.velocity_sp_m_s[2], 1.5f) && out.sat.pos_limit[2] != 0U, 10);

    in.position_bypass = 0U; in.measurement_valid = 0U;
    DRV_POSITION_CONTROL_PositionStep(&p, &in, &out);
    CHECK(out.velocity_sp_m_s[0] == 0.0f &&
          (out.flags & DRV_POSITION_CONTROL_FLAG_MEAS_INVALID) != 0U, 11);
    return 0;
}

static void make_velocity_input(DRV_POSITION_CONTROL_VelocityInput *in)
{
    memset(in, 0, sizeof(*in));
    in->velocity_sp_m_s[0] = 1.0f;
    in->velocity_sp_m_s[1] = -1.0f;
    in->velocity_sp_m_s[2] = 0.5f;
    in->accel_ff_m_s2[0] = 0.5f;
    in->accel_ff_m_s2[1] = -0.25f;
    in->measured_accel_m_s2[0] = 2.0f;
    in->measured_accel_m_s2[1] = -4.0f;
    in->dt_sec = 0.01f;
    in->measurement_valid = 1U;
    in->integrator_enable = 1U;
}

static int velocity_terms_and_lifecycle(void)
{
    DRV_POSITION_CONTROL_Params p = params();
    DRV_POSITION_CONTROL_State state = {0};
    DRV_POSITION_CONTROL_VelocityInput in;
    DRV_POSITION_CONTROL_VelocityOutput out;

    make_velocity_input(&in);
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK(NEAR(out.error_m_s[0], 1.0f) && NEAR(out.error_m_s[1], -1.0f), 20);
    CHECK(NEAR(out.p_term_m_s2[0], 2.0f) && NEAR(out.p_term_m_s2[1], -3.0f), 21);
    CHECK(NEAR(out.ff_term_m_s2[0], 0.5f) && NEAR(out.ff_term_m_s2[1], -0.25f), 22);
    CHECK(NEAR(out.d_term_m_s2[0], -1.0f) && NEAR(out.d_term_m_s2[1], 2.0f), 23);
    CHECK(NEAR(out.i_term_m_s2[0], 0.01f) && NEAR(out.i_term_m_s2[1], -0.01f), 24);
    CHECK(NEAR(out.accel_unsat_m_s2[0], 1.51f) && NEAR(out.accel_unsat_m_s2[1], -1.26f), 25);

    /* A setpoint step alone cannot create a D kick: D uses measured accel. */
    DRV_POSITION_CONTROL_ResetState(&state);
    memset(&in, 0, sizeof(in));
    in.measurement_valid = 1U; in.integrator_enable = 1U; in.dt_sec = 0.01f;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    in.velocity_sp_m_s[0] = 10.0f;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK(NEAR(out.d_term_m_s2[0], 0.0f), 26);

    /* Integrator limit and explicit reset. */
    DRV_POSITION_CONTROL_ResetState(&state);
    memset(&in, 0, sizeof(in));
    in.velocity_sp_m_s[0] = 1.0f; in.dt_sec = 0.01f;
    in.measurement_valid = 1U; in.integrator_enable = 1U;
    p.vel_ki[0] = 100.0f;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK(NEAR(state.velocity_integrator_m_s2[0], 0.2f), 27);
    in.integrator_reset = 1U;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    if (!(NEAR(state.velocity_integrator_m_s2[0], 0.0f) &&
          (out.flags & DRV_POSITION_CONTROL_FLAG_I_RESET) != 0U)) {
        printf("debug reset i=%f flags=%u\\n", state.velocity_integrator_m_s2[0], out.flags);
        return 28;
    }
    return 0;
}

static int saturation_and_invalid(void)
{
    DRV_POSITION_CONTROL_Params p = params();
    DRV_POSITION_CONTROL_State state = {0};
    DRV_POSITION_CONTROL_VelocityInput in = {0};
    DRV_POSITION_CONTROL_VelocityOutput out;

    p.vel_kp[0] = 10.0f; p.vel_ki[0] = 1.0f;
    p.xy_accel_limit_m_s2 = 1.0f;
    in.velocity_sp_m_s[0] = 1.0f; in.dt_sec = 0.01f;
    in.measurement_valid = 1U; in.integrator_enable = 1U;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK(out.sat.pos_limit[0] != 0U && out.sat.neg_limit[0] == 0U, 30);
    CHECK(state.velocity_integrator_m_s2[0] == 0.0f &&
          (out.flags & DRV_POSITION_CONTROL_FLAG_I_FREEZE) != 0U, 31);

    in.velocity_sp_m_s[0] = -1.0f;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK(out.sat.neg_limit[0] != 0U && out.sat.pos_limit[0] == 0U, 32);
    CHECK(state.velocity_integrator_m_s2[0] == 0.0f, 33);

    p.xy_accel_limit_m_s2 = 100.0f;
    p.z_accel_limit_up_m_s2 = 0.5f;
    p.z_accel_limit_down_m_s2 = 0.8f;
    /* R-F6-2: +Z is up.  +velocity_sp[2] (upward) clamps to up_limit. */
    in.velocity_sp_m_s[0] = 0.0f; in.velocity_sp_m_s[2] = 2.0f;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK(NEAR(out.accel_sat_m_s2[2], 0.5f) && out.sat.pos_limit[2] != 0U, 34);
    in.velocity_sp_m_s[2] = -2.0f;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK(NEAR(out.accel_sat_m_s2[2], -0.8f) && out.sat.neg_limit[2] != 0U, 35);

    state.velocity_integrator_m_s2[0] = 0.15f;
    state.accel_lpf_m_s2[0] = 3.0f;
    state.accel_lpf_initialized = 1U;
    in.velocity_sp_m_s[0] = 0.0f; in.velocity_sp_m_s[2] = 0.0f;
    in.measurement_valid = 0U; in.integrator_freeze = 0U;
    in.measured_accel_m_s2[0] = NAN;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK(NEAR(state.velocity_integrator_m_s2[0], 0.15f) &&
          NEAR(state.accel_lpf_m_s2[0], 3.0f), 36);
    CHECK((out.flags & DRV_POSITION_CONTROL_FLAG_MEAS_INVALID) != 0U &&
          (out.flags & DRV_POSITION_CONTROL_FLAG_I_FREEZE) != 0U, 37);

    in.measurement_valid = 1U; in.measured_accel_m_s2[0] = 0.0f;
    in.dt_sec = 0.0f;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK((out.flags & DRV_POSITION_CONTROL_FLAG_DT_INVALID) != 0U &&
          NEAR(state.velocity_integrator_m_s2[0], 0.15f), 38);
    in.dt_sec = 0.25f;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK((out.flags & DRV_POSITION_CONTROL_FLAG_DT_CLIPPED) != 0U, 39);

    /* Real downstream allocator feedback freezes same-direction growth. */
    DRV_POSITION_CONTROL_ResetState(&state);
    in.dt_sec = 0.01f; in.velocity_sp_m_s[0] = 1.0f;
    in.downstream_saturation.pos_limit[0] = 1U;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK(NEAR(state.velocity_integrator_m_s2[0], 0.0f), 40);

    p.vel_kp[0] = -1.0f;
    DRV_POSITION_CONTROL_VelocityStep(&p, &state, &in, &out);
    CHECK((out.flags & DRV_POSITION_CONTROL_FLAG_INPUT_INVALID) != 0U, 41);
    return 0;
}

int main(void)
{
    int rc = position_contract();
    if (rc != 0) return rc;
    rc = velocity_terms_and_lifecycle();
    if (rc != 0) return rc;
    rc = saturation_and_invalid();
    if (rc != 0) return rc;
    puts("position-control-contract: PASS");
    return 0;
}
"""


def test_position_control_source_is_pure_and_documented() -> None:
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    assert "+X forward" in header
    assert "+Y left" in header
    assert "+Z up" in header
    assert "metres per second" in header and "metres per second squared" in header
    assert "measurement_valid" in header and "integrator_freeze" in header
    assert "HAL" not in source
    assert "FreeRTOS" not in source
    assert "static " in source  # helpers only; no file-scope mutable state
    assert "velocity_sp_m_s[axis] -" in source
    assert "-params->vel_kd[axis]" in source


def test_position_control_host_gcc(tmp_path: pathlib.Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("gcc is required for the pure-C position controller harness")

    harness = tmp_path / "position_control_harness.c"
    executable = tmp_path / "position_control_harness"
    harness.write_text(textwrap.dedent(HARNESS), encoding="utf-8")
    compile_result = subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(ROOT / "Driver" / "Inc"),
            str(SOURCE),
            str(harness),
            "-lm",
            "-o",
            str(executable),
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert compile_result.returncode == 0, compile_result.stderr
    run_result = subprocess.run(
        [str(executable)], cwd=ROOT, capture_output=True, text=True, check=False
    )
    assert run_result.returncode == 0, run_result.stdout + run_result.stderr
    assert "position-control-contract: PASS" in run_result.stdout
