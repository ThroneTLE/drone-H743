"""Regression tests for the 919fcfd9 software-review findings."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


CONTROLLER_HARNESS = r"""
#include "drv_coax_ctrl.h"
#include <math.h>
#include <string.h>

#define CHECK(x,n) do { if (!(x)) return (n); } while (0)
#define NEAR(a,b,e) (fabsf((a)-(b)) <= (e))

static DRV_COAX_CTRL_Schedule all_due(void) {
    DRV_COAX_CTRL_Schedule s = {0};
    s.position_update = s.velocity_update = s.attitude_update = s.rate_update = 1U;
    s.position_dt_s = .02f; s.velocity_dt_s = .01f;
    s.attitude_dt_s = .004f; s.rate_dt_s = .002f;
    s.integrator_enable = 1U;
    return s;
}

int main(void) {
    DRV_COAX_CTRL_Params p;
    DRV_POSITION_CONTROL_State ps = {0};
    DRV_POSITION_CONTROL_VelocityInput vi = {0};
    DRV_POSITION_CONTROL_VelocityOutput vo;
    DRV_COAX_CTRL_AttitudeInput a = {0};
    DRV_COAX_CTRL_Reference r = {0};
    DRV_COAX_CTRL_Output o;
    DRV_COAX_CTRL_Debug d;
    DRV_COAX_CTRL_Schedule s = all_due();

    DRV_COAX_CTRL_GetDefaultParams(&p);
    ps.velocity_integrator_m_s2[2] = .5f;
    p.position.vel_ki[2] = 1.0f;
    p.position.vel_integrator_limit[2] = 1.0f;
    vi.velocity_sp_m_s[2] = -1.0f;
    vi.dt_sec = .01f; vi.measurement_valid = 1U; vi.integrator_enable = 1U;
    vi.downstream_saturation.pos_limit[2] = 1U;
    vi.downstream_saturation.thrust_saturated = 1U;
    DRV_POSITION_CONTROL_VelocityStep(&p.position, &ps, &vi, &vo);
    CHECK(NEAR(ps.velocity_integrator_m_s2[2], .49f, 1e-6f), 1);

    DRV_COAX_CTRL_ResetParams();
    DRV_COAX_CTRL_GetParams(&p);
    p.position.vel_ki[0] = 1.0f;
    for (unsigned i=0; i<3; ++i) { p.rate.kp[i]=0; p.attitude.att_kp[i]=0; }
    DRV_COAX_CTRL_SetParams(&p);
    r.horizontal_velocity_valid = r.navigation_position_valid =
        r.navigation_velocity_valid = a.acceleration_valid = 1U;
    r.vx_m_s = .2f;
    for (int i=0; i<10; ++i) DRV_COAX_CTRL_RunScheduled(&a,&r,&s,&o);
    r.direct_attitude_target_valid = r.manual_total_force_valid = 1U;
    r.manual_total_force_n = 13.0f;
    s.integrator_reset = 1U; s.integrator_enable = 0U; s.integrator_freeze = 1U;
    DRV_COAX_CTRL_RunScheduled(&a,&r,&s,&o);
    r.direct_attitude_target_valid = r.manual_total_force_valid = 0U;
    r.vx_m_s = 0.0f;
    s.integrator_reset = 0U; s.integrator_enable = 1U; s.integrator_freeze = 0U;
    DRV_COAX_CTRL_RunScheduled(&a,&r,&s,&o);
    DRV_COAX_CTRL_GetLastDebug(&d);
    CHECK(NEAR(d.velocity_i_m_s2[0], 0.0f, 1e-7f), 2);

    DRV_COAX_CTRL_ResetParams();
    memset(&a,0,sizeof a); memset(&r,0,sizeof r); s=all_due();
    r.horizontal_velocity_valid = r.navigation_position_valid =
        r.navigation_velocity_valid = a.acceleration_valid = 1U;
    DRV_COAX_CTRL_RunScheduled(&a,&r,&s,&o);
    CHECK(o.tilt_saturated == 0U, 3);
    CHECK(o.saturation_positive[0] == 0U && o.saturation_negative[0] == 0U, 4);
    CHECK(o.saturation_positive[1] == 0U && o.saturation_negative[1] == 0U, 5);

    DRV_COAX_CTRL_ServoCalibration cal;
    DRV_COAX_CTRL_GetDefaultServoCalibration(&cal);
    for (unsigned i=0; i<2; ++i) {
        cal.center_us[i]=1500; cal.min_us[i]=1450; cal.max_us[i]=1550;
        cal.pulse_sign[i]=1;
    }
    CHECK(DRV_COAX_CTRL_SetServoCalibration(&cal) == 1U, 6);
    memset(&a,0,sizeof a); memset(&r,0,sizeof r); s=all_due();
    a.gyro_x_rad_s=2.0f;
    r.direct_attitude_target_valid=r.manual_total_force_valid=1U;
    r.manual_total_force_n=13.0f;
    DRV_COAX_CTRL_RunScheduled(&a,&r,&s,&o);
    CHECK(o.servo_alpha_us == 1450U, 7);
    CHECK(fabsf(o.beta_rad) < 0.08f, 8);
    CHECK(o.saturation_negative[0] != 0U, 9);

    DRV_COAX_CTRL_ResetServoCalibration();
    DRV_COAX_CTRL_ResetParams();
    DRV_COAX_CTRL_SetParam("coax.att_roll_kp", 1.2f);
    DRV_COAX_CTRL_GetParams(&p);
    const float before_rate = p.rate.kp[0];
    const float before_att = p.attitude.att_kp[0];
    CHECK(DRV_COAX_CTRL_SetParam("coax.roll_rate_kd", 0.0f) == 0U, 10);
    DRV_COAX_CTRL_GetParams(&p);
    CHECK(NEAR(p.rate.kp[0], before_rate, 1e-8f) &&
          NEAR(p.attitude.att_kp[0], before_att, 1e-8f), 11);
    CHECK(DRV_COAX_CTRL_SetParam("coax.yaw_rate_limit_rad_s", 0.0f) == 0U, 12);
    return 0;
}
"""


def _compile_and_run(tmp_path: Path, source: str) -> subprocess.CompletedProcess[str]:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("gcc is required")
    stub = tmp_path / "stub"
    stub.mkdir()
    (stub / "bsp_pwm.h").write_text(
        "#define BSP_PWM_ESC_MIN_US 1000U\n#define BSP_PWM_ESC_MAX_US 2000U\n",
        encoding="ascii",
    )
    harness = tmp_path / "review.c"
    exe = tmp_path / "review.exe"
    harness.write_text(source, encoding="ascii")
    compiled = subprocess.run(
        [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", f"-I{stub}",
         f"-I{ROOT / 'Driver' / 'Inc'}",
         *(str(ROOT / "Driver" / "Src" / name) for name in (
             "drv_coax_ctrl.c", "drv_position_control.c",
             "drv_attitude_control.c", "drv_rate_control.c")),
         str(harness), "-lm", "-o", str(exe)],
        capture_output=True, text=True, check=False,
    )
    assert compiled.returncode == 0, compiled.stderr
    return subprocess.run([str(exe)], capture_output=True, text=True, check=False)


def test_allocator_lifecycle_antiwindup_and_param_regressions(tmp_path: Path) -> None:
    result = _compile_and_run(tmp_path, CONTROLLER_HARNESS)
    assert result.returncode == 0, result.stdout + result.stderr


def test_time_domains_telemetry_and_z_measurement_are_explicit() -> None:
    stabilizer = (ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8")
    scheduler_h = (ROOT / "App/Inc/app_control_scheduler.h").read_text(encoding="utf-8")
    scheduler_c = (ROOT / "App/Src/app_control_scheduler.c").read_text(encoding="utf-8")
    port = (ROOT / "App/Src/app_telem_port.c").read_text(encoding="utf-8")

    assert "frame.now_ms = HAL_GetTick();" in stabilizer
    assert "navigation_sample_token" in scheduler_h
    assert "navigation_sample_us > now_us" not in scheduler_c
    assert "APP_ControlScheduler_Commit" in scheduler_h
    assert "frame->attitude.accel_m_s2[2] = 0.0f;" not in stabilizer
    assert "vertical_accel" in stabilizer
    angular_section = port[port.index("APP_TELEM_CH_CTRL_ATT_ERR_X"):]
    assert "linear_sign * ctrl_debug.omega_sp_rad_s" not in angular_section
    assert "linear_sign * ctrl_debug.rate_error_rad_s" not in angular_section
    assert "linear_sign * ctrl_debug.moment_cmd_n_m" not in angular_section


SCHEDULER_HARNESS = r"""
#include "app_control_scheduler.h"
#include <stdint.h>

int main(void) {
    APP_ControlSchedulerState state = {0};
    APP_ControlSchedule out;
    unsigned rate=0,att=0,vel=0,pos=0;
    for (uint64_t t=1000; t<=1000000; t+=1000) {
        const uint64_t nav_token=(t/10000)*10000;
        APP_ControlScheduler_Step(&state,t,nav_token,nav_token!=0,&out);
        if (out.rate_due) {
            rate += out.rate_due; att += out.attitude_due;
            vel += out.velocity_due; pos += out.position_due;
            APP_ControlScheduler_Commit(&state,t,nav_token,&out);
        }
    }
    return (rate==500 && att==250 && vel==100 && pos==50) ? 0 : 1;
}
"""


def test_scheduler_retains_slow_due_until_control_executes(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("gcc is required")
    harness = tmp_path / "scheduler.c"
    exe = tmp_path / "scheduler.exe"
    harness.write_text(SCHEDULER_HARNESS, encoding="ascii")
    compiled = subprocess.run(
        [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
         f"-I{ROOT / 'App' / 'Inc'}", str(ROOT / "App/Src/app_control_scheduler.c"),
         str(harness), "-o", str(exe)], capture_output=True, text=True, check=False,
    )
    assert compiled.returncode == 0, compiled.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, check=False)
    assert run.returncode == 0
