from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


HARNESS = r"""
#include "drv_coax_ctrl.h"
#include "drv_airframe_model.h"

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

static void rpy_matrix(const float rpy[3], float rotation[3][3])
{
    const float cp = cosf(rpy[1]);
    const float sp = sinf(rpy[1]);
    const float cr = cosf(rpy[0]);
    const float sr = sinf(rpy[0]);
    const float cy = cosf(rpy[2]);
    const float sy = sinf(rpy[2]);

    rotation[0][0] = cp * cy;
    rotation[0][1] = sr * sp * cy - cr * sy;
    rotation[0][2] = cr * sp * cy + sr * sy;
    rotation[1][0] = cp * sy;
    rotation[1][1] = sr * sp * sy + cr * cy;
    rotation[1][2] = cr * sp * sy - sr * cy;
    rotation[2][0] = -sp;
    rotation[2][1] = sr * cp;
    rotation[2][2] = cr * cp;
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
}

int main(void)
{
    DRV_COAX_CTRL_AttitudeInput attitude;
    DRV_COAX_CTRL_Reference reference;
    DRV_COAX_CTRL_Output output;
    DRV_COAX_CTRL_Debug debug;
    DRV_COAX_CTRL_Params params;
    float desired_body_r[3][3];
    float integral_before;

    DRV_COAX_CTRL_Init();

    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(output.alpha_rad, 0.0f, 1.0e-5f), 1);
    CHECK(nearly_equal(output.beta_rad, 0.0f, 1.0e-5f), 2);
    CHECK(nearly_equal(debug.total_force_n,
                       DRV_AIRFRAME_MASS_KG * DRV_AIRFRAME_GRAVITY_M_S2,
                       1.0e-3f), 3);
    CHECK(nearly_equal(output.thrust_upper_n, debug.total_force_n * 0.5f, 1.0e-3f), 4);
    CHECK(debug.protection_flags == 0U, 5);
    CHECK(nearly_equal(debug.horizontal_command_scale, 1.0f, 1.0e-6f), 6);

    reset_case(&attitude, &reference);
    reference.vx_m_s = 0.8f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(output.alpha_rad < 0.0f, 10);
    CHECK(fabsf(output.beta_rad) < 1.0e-4f, 11);
    CHECK(debug.desired_attitude_rpy_rad[1] > 0.0f, 12);
    CHECK(debug.moment_cmd_n_m[1] > 0.0f, 13);
    CHECK(output.servo_beta_us > DRV_COAX_CTRL_SERVO_BETA_CENTER_US, 14);
    CHECK(nearly_equal(
        debug.moment_cmd_n_m[1],
        -0.569f * DRV_AIRFRAME_PITCH_THRUST_LEVER_ARM_M *
            debug.total_force_n * sinf(output.alpha_rad) * cosf(output.beta_rad),
        1.0e-5f), 15);
    {
        const float force_norm = sqrtf(
            debug.accel_out_m_s2[0] * debug.accel_out_m_s2[0] +
            DRV_AIRFRAME_GRAVITY_M_S2 * DRV_AIRFRAME_GRAVITY_M_S2);
        const float target_pitch = atan2f(debug.accel_out_m_s2[0],
                                          DRV_AIRFRAME_GRAVITY_M_S2);
        CHECK(nearly_equal(debug.target_attitude_rp_rad[1],
                           target_pitch,
                           2.0e-4f), 16);
        CHECK(nearly_equal(debug.desired_attitude_rpy_rad[1],
                           target_pitch,
                           2.0e-4f), 17);
        rpy_matrix(debug.desired_attitude_rpy_rad, desired_body_r);
        CHECK(nearly_equal(desired_body_r[0][2],
                           debug.accel_out_m_s2[0] / force_norm,
                           5.0e-4f), 18);
        CHECK(nearly_equal(desired_body_r[2][2],
                           DRV_AIRFRAME_GRAVITY_M_S2 / force_norm,
                           2.0e-4f), 19);
    }

    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.pos_z_kp = 0.0f;
    params.vel_z_kd = 0.0f;
    params.pos_z_ki = 0.50f;
    DRV_COAX_CTRL_SetParams(&params);
    attitude.z_m = -0.20f;
    reference.z_m = -0.40f;
    for (int step = 0; step < 50; ++step) {
        DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    }
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(debug.pos_z_i_m_s2 < -0.09f, 102);
    CHECK(debug.accel_out_m_s2[2] < -0.09f, 103);
    CHECK(debug.total_force_n >
          DRV_AIRFRAME_MASS_KG * DRV_AIRFRAME_GRAVITY_M_S2,
          104);

    reset_case(&attitude, &reference);
    reference.vy_m_s = 0.8f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(output.beta_rad < 0.0f, 20);
    CHECK(fabsf(output.alpha_rad) < 1.0e-4f, 21);
    CHECK(debug.desired_attitude_rpy_rad[0] < 0.0f, 22);
    CHECK(debug.moment_cmd_n_m[0] > 0.0f, 23);
    CHECK(output.servo_alpha_us > DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US, 24);
    CHECK(nearly_equal(
        debug.moment_cmd_n_m[0],
        -0.581f * DRV_AIRFRAME_ROLL_THRUST_LEVER_ARM_M *
            debug.total_force_n * sinf(output.beta_rad),
        1.0e-5f), 25);
    {
        const float target_roll = -atan2f(debug.accel_out_m_s2[1],
                                          DRV_AIRFRAME_GRAVITY_M_S2);
        CHECK(nearly_equal(debug.target_attitude_rp_rad[0],
                           target_roll,
                           2.0e-4f), 26);
        CHECK(nearly_equal(debug.desired_attitude_rpy_rad[0],
                           target_roll,
                           2.0e-4f), 27);
    }

    reset_case(&attitude, &reference);
    reference.x_m = 0.20f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(debug.pos_p_m_s2[0] > 0.05f, 28);
    CHECK(debug.vel_d_m_s2[0] == 0.0f, 29);
    CHECK(output.alpha_rad < 0.0f, 30);

    reset_case(&attitude, &reference);
    reference.x_m = 0.20f;
    reference.vx_m_s = 0.8f;
    reference.direct_attitude_target_valid = 1U;
    reference.target_pitch_rad = 0.10f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(debug.pos_p_m_s2[0], 0.0f, 1.0e-6f), 70);
    CHECK(nearly_equal(debug.vel_d_m_s2[0], 0.0f, 1.0e-6f), 71);
    CHECK(nearly_equal(debug.accel_out_m_s2[0], 0.0f, 1.0e-6f), 72);
    CHECK(nearly_equal(debug.target_attitude_rp_rad[1], 0.10f, 1.0e-5f), 73);
    CHECK(output.alpha_rad < 0.0f, 74);

    reset_case(&attitude, &reference);
    attitude.x_m = -1.0f;
    attitude.y_m = 2.0f;
    attitude.z_m = -0.3f;
    attitude.vx_m_s = -1.5f;
    attitude.vy_m_s = 1.2f;
    attitude.vz_m_s = -0.4f;
    reference.x_m = 1.0f;
    reference.y_m = -2.0f;
    reference.z_m = 0.3f;
    reference.vx_m_s = 1.5f;
    reference.vy_m_s = -1.2f;
    reference.vz_m_s = 0.4f;
    reference.direct_attitude_target_valid = 1U;
    reference.manual_total_force_valid = 1U;
    reference.manual_total_force_n =
        DRV_COAX_CTRL_MotorPulseToTotalThrust(1604U);
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    for (int axis = 0; axis < 3; ++axis) {
        CHECK(nearly_equal(debug.pos_p_m_s2[axis], 0.0f, 1.0e-6f), 90 + axis);
        CHECK(nearly_equal(debug.vel_d_m_s2[axis], 0.0f, 1.0e-6f), 93 + axis);
        CHECK(nearly_equal(debug.accel_out_m_s2[axis], 0.0f, 1.0e-6f), 96 + axis);
    }
    CHECK(nearly_equal(debug.pos_z_i_m_s2, 0.0f, 1.0e-6f), 105);
    CHECK(nearly_equal(debug.total_force_n,
                       reference.manual_total_force_n,
                       1.0e-4f), 99);
    CHECK(output.motor_upper_us == 1604U, 100);
    CHECK(output.motor_lower_us == 1604U, 101);

    reset_case(&attitude, &reference);
    reference.direct_attitude_target_valid = 1U;
    reference.target_roll_rad = 0.10f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(debug.target_attitude_rp_rad[0], 0.10f, 1.0e-5f), 75);
    CHECK(nearly_equal(debug.desired_attitude_rpy_rad[0], 0.10f, 1.0e-5f), 76);
    CHECK(output.beta_rad > 0.0f, 77);
    CHECK(output.servo_alpha_us < DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US, 78);

    reset_case(&attitude, &reference);
    attitude.pitch_rad = 0.10f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    CHECK(output.alpha_rad > 0.0f, 31);
    CHECK(output.servo_beta_us < DRV_COAX_CTRL_SERVO_BETA_CENTER_US, 32);

    reset_case(&attitude, &reference);
    attitude.roll_rad = 0.10f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    CHECK(output.beta_rad < 0.0f, 33);

    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.roll_angle_kp = 0.0f;
    params.pitch_angle_kp = 0.0f;
    params.pitch_rate_kd = 0.0f;
    DRV_COAX_CTRL_SetParams(&params);
    /* APP attitude convention: increasing roll has gyro_x < 0. */
    attitude.gyro_x_rad_s = -0.70f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(debug.rate_error_rad_s[0] < 0.0f, 34);
    CHECK(debug.moment_cmd_n_m[0] > 0.0f, 35);
    CHECK(output.beta_rad < 0.0f, 36);

    reset_case(&attitude, &reference);
    reference.vx_m_s = 0.8f;
    reference.horizontal_velocity_valid = 0U;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(output.alpha_rad, 0.0f, 1.0e-5f), 40);
    CHECK((debug.protection_flags & DRV_COAX_CTRL_PROTECT_VELOCITY_INVALID) != 0U, 41);
    CHECK(nearly_equal(debug.horizontal_command_scale, 0.0f, 1.0e-6f), 42);

    reset_case(&attitude, &reference);
    reference.vx_m_s = 0.8f;
    for (int step = 0; step < 10; ++step) {
        DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    }
    DRV_COAX_CTRL_GetLastDebug(&debug);
    integral_before = debug.velocity_integral_m[0];
    CHECK(nearly_equal(integral_before, 0.0f, 1.0e-7f), 50);
    CHECK(debug.vel_d_m_s2[0] > 0.5f, 51);
    reference.horizontal_velocity_valid = 0U;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(debug.velocity_integral_m[0], 0.0f, 1.0e-7f), 52);

    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.pos_x_kp = 0.0f;
    params.pos_y_kp = 0.0f;
    params.vel_x_kd = 2.0f;
    params.vel_y_kd = 2.0f;
    DRV_COAX_CTRL_SetParams(&params);
    attitude.vx_m_s = -2.0f;
    attitude.vy_m_s = 0.0f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(debug.vel_d_m_s2[0], 3.70f, 1.0e-6f), 53);
    CHECK(nearly_equal(debug.vel_d_m_s2[1], 0.0f, 1.0e-6f), 54);
    CHECK(nearly_equal(debug.accel_out_m_s2[0], 3.70f, 1.0e-6f), 55);
    CHECK(nearly_equal(debug.accel_out_m_s2[1], 0.0f, 1.0e-6f), 56);
    CHECK(fabsf(debug.target_attitude_rp_rad[0]) < 0.01f, 57);
    CHECK((debug.target_attitude_rp_rad[1] > 0.35f) &&
          (debug.target_attitude_rp_rad[1] < 0.37f), 58);

    reset_case(&attitude, &reference);
    attitude.pitch_rad = 1.0f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(fabsf(output.alpha_rad) <= 0.4886922f + 1.0e-6f, 60);
    CHECK((debug.protection_flags & DRV_COAX_CTRL_PROTECT_ATTITUDE) != 0U, 61);

    reset_case(&attitude, &reference);
    attitude.pitch_rad = 2.967060f;
    reference.vx_m_s = 0.8f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK((debug.protection_flags & DRV_COAX_CTRL_PROTECT_ATTITUDE) != 0U, 62);
    CHECK(nearly_equal(debug.horizontal_command_scale, 0.0f, 1.0e-6f), 63);

    reset_case(&attitude, &reference);
    attitude.yaw_rad = 1.0f;
    reference.yaw_rad = 0.0f;
    reference.vx_m_s = 0.2f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK((debug.protection_flags & DRV_COAX_CTRL_PROTECT_ATTITUDE) == 0U, 64);
    CHECK(debug.horizontal_command_scale > 0.99f, 65);
    CHECK(fabsf(debug.vel_d_m_s2[0]) > 0.05f, 66);
    CHECK(fabsf(output.alpha_rad) > 1.0e-4f, 67);

    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.vel_loop_enable = 0.0f;
    DRV_COAX_CTRL_SetParams(&params);
    reference.ax_m_s2 = 1.0f;
    attitude.pitch_rad = atan2f(reference.ax_m_s2, DRV_AIRFRAME_GRAVITY_M_S2);
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(output.alpha_rad, 0.0f, 2.0e-4f), 70);
    CHECK(nearly_equal(output.beta_rad, 0.0f, 2.0e-4f), 71);
    CHECK(nearly_equal(debug.desired_attitude_rpy_rad[1],
                       attitude.pitch_rad,
                       2.0e-4f), 72);

    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.roll_angle_kp = 0.0f;
    params.pitch_angle_kp = 0.0f;
    params.roll_rate_kd = 0.0f;
    params.pitch_rate_kd = 0.0f;
    DRV_COAX_CTRL_SetParams(&params);
    attitude.gyro_x_rad_s = 0.7f;
    attitude.gyro_y_rad_s = -0.4f;
    attitude.gyro_z_rad_s = 0.3f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(
        debug.moment_cmd_n_m[0],
        (DRV_AIRFRAME_IZZ_KGM2 - DRV_AIRFRAME_IYY_KGM2) *
            attitude.gyro_y_rad_s * attitude.gyro_z_rad_s,
        1.0e-6f), 80);
    CHECK(nearly_equal(
        debug.moment_cmd_n_m[1],
        (DRV_AIRFRAME_IXX_KGM2 - DRV_AIRFRAME_IZZ_KGM2) *
            attitude.gyro_z_rad_s *
            attitude.gyro_x_rad_s,
        1.0e-6f), 81);

    return 0;
}
"""


def test_real_controller_runtime_math(tmp_path: Path) -> None:
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
    harness_path = tmp_path / "balance_harness.c"
    harness_path.write_text(HARNESS, encoding="ascii")
    executable = tmp_path / "balance_harness.exe"

    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{stub_dir}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"),
            str(harness_path),
            "-lm",
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)
