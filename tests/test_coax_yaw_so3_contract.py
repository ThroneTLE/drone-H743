"""偏航并入 SO(3) 控制律的契约。

偏航 PD（2026-07-24，`85a5cacb`）比 SO(3) 控制器（2026-07-25，`2d1d2cd2`）早一天，
当天的设计记录 §1 明写「Z 高度环、偏航控制和双电机推力分配保持原路径」，此后该函数
零修改。也就是说偏航走 PD 是一条从未关闭的迁移边界，不是设计判断。

倾转机构确实无法产生偏航力矩（辨识记录：`τ_yaw = 0，同轴双桨差速独立提供`），但那
只要求**分配器**分叉，不要求**控制律**分叉。本模块钉住合并后的三件事：

1. 悬停小角度下新旧力矩逐值等价（增益按 K_R = Izz·kp 换算，对外参数名不变）；
2. 有倾角时 e_w 带 R^T Rd 变换，与旧 PD 的裸速率差不是同一个量（这正是合并的收益）；
3. 偏航饱和参与保护缩放——差动推力的偏航权限很紧，旧路径下分配器的 clamp
   会静默削掉指令而保护层毫无感知。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


from _airframe_fixture import AIRFRAME_FIXTURE_C, AIRFRAME_SOURCE, PROP_MAP_SOURCE


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


HARNESS = AIRFRAME_FIXTURE_C + r"""
#include "drv_coax_ctrl.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { \
    if (!(condition)) { \
        fprintf(stderr, "check %d failed at line %d\n", (code), __LINE__); \
        return (code); \
    } \
} while (0)

static int nearly_equal(float left, float right, float tolerance)
{
    return fabsf(left - right) <= tolerance;
}

static void reset_case(DRV_COAX_CTRL_AttitudeInput *attitude,
                       DRV_COAX_CTRL_Reference *reference)
{
    DRV_COAX_CTRL_ResetParams();
    DRV_COAX_CTRL_ResetState();
    memset(attitude, 0, sizeof(*attitude));
    memset(reference, 0, sizeof(*reference));
    reference->dt_sec = 0.02f;
    reference->horizontal_velocity_valid = 1U;
    reference->navigation_position_valid = 1U;
    reference->navigation_velocity_valid = 1U;
    attitude->acceleration_valid = 1U;
}

/* Pre-migration yaw PD, transcribed verbatim from 85a5cacb. */
static float legacy_yaw_torque(const DRV_COAX_CTRL_Params *params,
                               float yaw_ref_rad,
                               float yaw_meas_rad,
                               float yaw_rate_ref_rad_s,
                               float gyro_z_rad_s,
                               float yaw_accel_ref_rad_s2)
{
    float yaw_err = yaw_ref_rad - yaw_meas_rad;
    float p_term;
    float d_term;

    while (yaw_err > 3.141592654f) { yaw_err -= 2.0f * 3.141592654f; }
    while (yaw_err < -3.141592654f) { yaw_err += 2.0f * 3.141592654f; }

    p_term = params->rate.kp[2] * params->attitude.att_kp[2] * yaw_err;
    d_term = params->rate.kp[2] * (yaw_rate_ref_rad_s - gyro_z_rad_s);
    return (params->yaw_inertia * yaw_accel_ref_rad_s2) + p_term + d_term;
}

int main(void)
{
    airframe_load_reference();
    DRV_COAX_CTRL_AttitudeInput attitude;
    DRV_COAX_CTRL_Reference reference;
    DRV_COAX_CTRL_Output output;
    DRV_COAX_CTRL_Debug debug;
    DRV_COAX_CTRL_Params params;
    float legacy;
    float naive_rate_error;

    /* ---- 1. Hover, small angles: new and old torque must match value for value ---- */
    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    attitude.yaw_rad = 0.03f;
    reference.yaw_rad = 0.08f;
    attitude.gyro_z_rad_s = 0.02f;
    reference.yaw_rate_rad_s = 0.05f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    legacy = legacy_yaw_torque(&params,
                               reference.yaw_rad,
                               attitude.yaw_rad,
                               reference.yaw_rate_rad_s,
                               attitude.gyro_z_rad_s,
                               reference.yaw_accel_rad_s2);
    /*
     * e_R uses sin(psi - psi_d) where the PD used the wrapped linear error, so the
     * two agree to O(err^3/6) rather than bit-exactly.  At 0.05 rad that is 3.8e-4
     * relative, i.e. hover-equivalent; the gap only opens at large yaw error.
     */
    CHECK(fabsf(debug.moment_cmd_n_m[2] - legacy) <= 1.0e-3f * fabsf(legacy), 10);
    CHECK(debug.moment_cmd_n_m[2] > 0.0f, 11);

    /* Shrink the error tenfold and the residual must fall by ~100x (cubic term). */
    reset_case(&attitude, &reference);
    attitude.yaw_rad = 0.03f;
    reference.yaw_rad = 0.035f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    legacy = legacy_yaw_torque(&params,
                               reference.yaw_rad,
                               attitude.yaw_rad,
                               reference.yaw_rate_rad_s,
                               attitude.gyro_z_rad_s,
                               reference.yaw_accel_rad_s2);
    CHECK(fabsf(debug.moment_cmd_n_m[2] - legacy) <= 1.0e-5f * fabsf(legacy), 14);

    /* Telemetry P/D terms stay in acceleration units; unchanged meaning. */
    CHECK(nearly_equal(debug.yaw_angle_p_rad_s,
                       params.attitude.att_kp[2] *
                           (reference.yaw_rad - attitude.yaw_rad),
                       1.0e-6f), 12);
    CHECK(nearly_equal(debug.yaw_rate_d_rad_s,
                       debug.rate_error_rad_s[2],
                       1.0e-6f), 13);

    /* ---- 2. Sign: reference left of measured must split thrust as the old path did ---- */
    reset_case(&attitude, &reference);
    attitude.yaw_rad = 0.0f;
    reference.yaw_rad = 0.2f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(debug.moment_cmd_n_m[2] > 0.0f, 20);
    CHECK(output.thrust_lower_n > output.thrust_upper_n, 21);

    reset_case(&attitude, &reference);
    attitude.yaw_rad = 0.2f;
    reference.yaw_rad = 0.0f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(debug.moment_cmd_n_m[2] < 0.0f, 22);
    CHECK(output.thrust_upper_n > output.thrust_lower_n, 23);

    /* ---- 3. Tilted: e_w carries the frame transform, unlike the raw rate difference ---- */
    reset_case(&attitude, &reference);
    attitude.roll_rad = 0.4f;
    attitude.pitch_rad = 0.3f;
    attitude.gyro_z_rad_s = 0.1f;
    reference.yaw_rate_rad_s = 0.6f;
    reference.direct_attitude_target_valid = 1U;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    naive_rate_error = reference.yaw_rate_rad_s - attitude.gyro_z_rad_s;
    /* e_w = w - R^T Rd w_d; at nonzero tilt it must separate from the raw difference. */
    CHECK(!nearly_equal(debug.rate_error_rad_s[2], naive_rate_error, 1.0e-3f), 30);
    /* The transform couples the reference rate into all three axes, not just z. */
    CHECK(fabsf(debug.rate_error_rad_s[0]) > 1.0e-3f, 31);

    /* ---- 4. Zeroed gains leave only the w x Jw feedforward (identically 0 when Ixx==Iyy) ---- */
    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.attitude.att_kp[2] = 0.0f;
    params.rate.kp[2] = 0.0f;
    DRV_COAX_CTRL_SetParams(&params);
    attitude.yaw_rad = 0.5f;
    attitude.gyro_x_rad_s = 0.7f;
    attitude.gyro_y_rad_s = -0.4f;
    attitude.gyro_z_rad_s = 0.3f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(
        debug.moment_cmd_n_m[2],
        (DRV_Airframe_Get()->iyy_kgm2 - DRV_Airframe_Get()->ixx_kgm2) *
            attitude.gyro_x_rad_s * attitude.gyro_y_rad_s,
        1.0e-9f), 40);

    /* ---- 5. Yaw saturation must feed the protection scale ---- */
    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.attitude.att_kp[2] = 40.0f;
    /* deliberately exhaust differential authority; the value tracks the yaw
     * reaction-torque coefficient k, which grew 50x on 2026-09-07 */
    params.rate.kp[2] = 0.5f;
    DRV_COAX_CTRL_SetParams(&params);
    attitude.yaw_rad = 0.0f;
    reference.yaw_rad = 1.2f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(debug.moment_utilization > 1.0f, 50);
    CHECK((debug.protection_flags & DRV_COAX_CTRL_PROTECT_MOMENT) != 0U, 51);
    CHECK(debug.horizontal_command_scale < 1.0f, 52);
    /* When the allocator clips, realised torque must fall below command; both observable. */
    CHECK(fabsf(debug.yaw_torque_cmd) < fabsf(debug.moment_cmd_n_m[2]), 53);

    /* Same yaw error at default gains must not trip the moment protection. */
    reset_case(&attitude, &reference);
    attitude.yaw_rad = 0.0f;
    reference.yaw_rad = 1.2f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK((debug.protection_flags & DRV_COAX_CTRL_PROTECT_MOMENT) == 0U, 60);
    CHECK(debug.horizontal_command_scale > 0.99f, 61);

    return 0;
}
"""


def test_yaw_is_produced_by_the_so3_law_not_a_separate_pd() -> None:
    wrapper = read("Driver/Src/drv_coax_ctrl.c")

    # 旧 PD 及其反向符号约定必须整个消失。
    assert "coax_ctrl_compute_yaw_torque_cmd" not in wrapper
    assert "coax_ctrl_wrap_pi" not in wrapper
    assert "reference->yaw_rad - attitude->yaw_rad" not in wrapper

    # 三轴同出姿态 P -> 角速度 PID，分配器才分叉。
    assert "DRV_AttitudeControl_Step" in wrapper
    assert "DRV_RateControl_Step" in wrapper
    assert "coax_ctrl_state.rate_output.moment_unsat" in wrapper
    # 2026-09-07：分配器改用**钳过**的偏航力矩。`moment_cmd_n_m[2]` 仍是未钳的
    # 原始需求（遥测据此还能看出"要了多少 vs 给了多少"），但它不再直接进分配器
    # —— 那条路径会让分配器去撞电机上下限，从而悄悄牺牲总推力。
    assert "yaw_torque_cmd = solution.yaw_moment_applied_n_m;" in wrapper
    assert (
        "solution->yaw_moment_applied_n_m =\n"
        "        coax_ctrl_state.rate_output.moment_cmd[2];"
    ) in wrapper

    # 旧参数名只作为显式换算 alias，不再冒充真实物理参数。
    assert 'strcmp(name, "coax.yaw_angle_kp")' not in wrapper
    assert 'strcmp(name, "coax.yaw_rate_kd")' not in wrapper

    # 偏航权限进入保护缩放。
    assert "coax_ctrl_yaw_limit_moment" in wrapper
    assert "yaw_utilization" in wrapper


def test_yaw_so3_runtime_matches_legacy_pd_in_hover(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")

    stub_dir = tmp_path / "stub"
    stub_dir.mkdir()
    (stub_dir / "bsp_pwm.h").write_text(
        "#ifndef BSP_PWM_H\n"
        "#define BSP_PWM_H\n"
        "#define BSP_PWM_ESC_MIN_US 1000U\n"
        "#define BSP_PWM_ESC_MAX_US 2000U\n"
        "#endif\n",
        encoding="ascii",
    )
    harness_path = tmp_path / "yaw_so3_harness.c"
    harness_path.write_text(HARNESS, encoding="ascii")
    executable = tmp_path / "yaw_so3_harness.exe"

    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{stub_dir}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(AIRFRAME_SOURCE),
            str(PROP_MAP_SOURCE),
            str(ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"),
            str(ROOT / "Driver" / "Src" / "drv_position_control.c"),
            str(ROOT / "Driver" / "Src" / "drv_attitude_control.c"),
            str(ROOT / "Driver" / "Src" / "drv_rate_control.c"),
            str(harness_path),
            "-lm",
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    result = subprocess.run([str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
