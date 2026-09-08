"""R-S5-1 integration contract for the real coax-controller entry path."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


HARNESS = r"""
#include "drv_coax_ctrl.h"
#include <math.h>
#include <string.h>

#define CHECK(x,n) do { if (!(x)) return (n); } while (0)

int main(void) {
    DRV_COAX_CTRL_AttitudeInput a = {0};
    DRV_COAX_CTRL_Reference r = {0};
    DRV_COAX_CTRL_Schedule s = {0};
    DRV_COAX_CTRL_Output o;
    DRV_COAX_CTRL_Debug d;
    DRV_COAX_CTRL_Params p;

    DRV_COAX_CTRL_ResetParams();
    DRV_COAX_CTRL_GetParams(&p);
    p.position.vel_ki[0] = 0.5f;
    p.position.vel_integrator_limit[0] = 1.0f;
    p.rate.kp[0] = p.rate.kp[1] = p.rate.kp[2] = 0.0f;
    p.attitude.att_kp[0] = p.attitude.att_kp[1] = p.attitude.att_kp[2] = 0.0f;
    DRV_COAX_CTRL_SetParams(&p);
    r.dt_sec = 0.01f;
    r.horizontal_velocity_valid = 1U;
    r.navigation_position_valid = 1U;
    r.navigation_velocity_valid = 1U;
    r.vx_m_s = 0.2f;
    a.acceleration_valid = 1U;
    s.position_update = s.velocity_update = s.attitude_update = s.rate_update = 1U;
    s.integrator_enable = 1U;
    s.position_dt_s = 0.02f; s.velocity_dt_s = 0.01f;
    s.attitude_dt_s = 0.004f; s.rate_dt_s = 0.002f;
    DRV_COAX_CTRL_RunScheduled(&a, &r, &s, &o);
    DRV_COAX_CTRL_GetLastDebug(&d);
    const float first_i = d.velocity_i_m_s2[0];
    CHECK(first_i > 0.0f, 1);

    s.position_update = s.velocity_update = 0U;
    for (int i = 0; i < 10; ++i) {
        DRV_COAX_CTRL_RunScheduled(&a, &r, &s, &o);
    }
    DRV_COAX_CTRL_GetLastDebug(&d);
    CHECK(fabsf(d.velocity_i_m_s2[0] - first_i) < 1.0e-7f, 2);

    s.velocity_update = 1U;
    DRV_COAX_CTRL_RunScheduled(&a, &r, &s, &o);
    DRV_COAX_CTRL_GetLastDebug(&d);
    CHECK(d.velocity_i_m_s2[0] >= first_i, 3);

    s.integrator_reset = 1U;
    DRV_COAX_CTRL_RunScheduled(&a, &r, &s, &o);
    DRV_COAX_CTRL_GetLastDebug(&d);
    CHECK(fabsf(d.velocity_i_m_s2[0]) < 1.0e-7f, 4);

    DRV_COAX_CTRL_ResetParams();
    DRV_COAX_CTRL_GetParams(&p);
    /* Deliberately drive yaw past what the allocator can deliver.  The value
     * tracks the reaction-torque coefficient k: it grew 50x on 2026-09-07, so
     * the moment limit grew with it and the old 0.01 no longer saturates. */
    p.rate.kp[2] = 0.5f;
    p.attitude.att_kp[2] = 40.0f;
    DRV_COAX_CTRL_SetParams(&p);
    memset(&s, 0, sizeof(s));
    s.position_update = s.velocity_update = s.attitude_update = s.rate_update = 1U;
    s.position_dt_s = s.velocity_dt_s = s.attitude_dt_s = s.rate_dt_s = 0.01f;
    r.yaw_rad = 1.2f;
    DRV_COAX_CTRL_RunScheduled(&a, &r, &s, &o);
    DRV_COAX_CTRL_GetLastDebug(&d);
    /* When the yaw demand exceeds authority, TOTAL THRUST MUST SURVIVE INTACT.
     *
     * This used to assert o.yaw_differential_saturated != 0, i.e. "let the
     * allocator run into the motor bounds".  That is exactly the behaviour
     * fixed on 2026-09-07: the two rotors are clamped independently, so when
     * one hits its limit the other does not make up the difference and
     * upper+lower silently falls below F -- felt as "raise the yaw gain and the
     * lift collapses / a rotor stalls".  The yaw moment is now clamped to the
     * achievable range before allocation, so the allocator no longer leaves the
     * range and the lift is preserved; "the demand was too big" is reported
     * honestly by the rate-loop saturation flag on the next line. */
    /* Bound the actual loss rather than the allocator's flag: at the limit one
     * rotor sits exactly on T_max, so a 1-ULP clip can still raise
     * yaw_differential_saturated.  That is harmless -- what must not happen is
     * losing thrust, and this bounds it to 1e-5 N. */
    CHECK(fabsf((o.thrust_upper_n + o.thrust_lower_n) - d.total_force_n) < 1.0e-5f, 5);
    CHECK(o.saturation_positive[2] != 0U, 6);
    CHECK(fabsf(o.moment_achieved_n_m[2]) < fabsf(d.moment_cmd_n_m[2]), 7);
    CHECK(fabsf(o.moment_achieved_n_m[2] - d.moment_achieved_n_m[2]) < 1.0e-8f, 8);
    return 0;
}
"""


def test_scheduled_cascade_and_allocator_feedback_on_host(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("gcc is required")
    stub = tmp_path / "stub"
    stub.mkdir()
    (stub / "bsp_pwm.h").write_text(
        "#define BSP_PWM_ESC_MIN_US 1000U\n#define BSP_PWM_ESC_MAX_US 2000U\n",
        encoding="ascii",
    )
    harness = tmp_path / "cascade.c"
    exe = tmp_path / "cascade.exe"
    harness.write_text(HARNESS, encoding="ascii")
    sources = [
        "drv_coax_ctrl.c",
        "drv_position_control.c",
        "drv_attitude_control.c",
        "drv_rate_control.c",
    ]
    result = subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{stub}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            *(str(ROOT / "Driver" / "Src" / name) for name in sources),
            str(harness),
            "-lm",
            "-o",
            str(exe),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, check=False)
    assert run.returncode == 0, run.stdout + run.stderr


def test_real_app_path_uses_scheduler_and_no_extra_acceleration_pid() -> None:
    stabilizer = (ROOT / "App" / "Src" / "app_stabilizer.c").read_text(
        encoding="utf-8"
    )
    coax = (ROOT / "Driver" / "Src" / "drv_coax_ctrl.c").read_text(
        encoding="utf-8"
    )
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    for source in (
        "drv_position_control.c",
        "drv_attitude_control.c",
        "drv_rate_control.c",
        "app_control_scheduler.c",
    ):
        assert source in cmake
    assert "APP_ControlScheduler_Step" in stabilizer
    assert "SVC_Timestamp_Us()" in stabilizer
    assert "DRV_COAX_CTRL_RunScheduled" in stabilizer
    assert "schedule.integrator_reset" in stabilizer
    assert "DRV_POSITION_CONTROL_PositionStep" in coax
    assert "DRV_POSITION_CONTROL_VelocityStep" in coax
    assert "DRV_AttitudeControl_Step" in coax
    assert "DRV_RateControl_Step" in coax
    assert "acceleration_pid" not in coax.lower()
    assert "DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK" not in coax
