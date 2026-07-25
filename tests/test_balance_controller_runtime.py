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

static void gimbal_matrix(float alpha, float beta, float rotation[3][3])
{
    const float ca = cosf(alpha);
    const float sa = sinf(alpha);
    const float cb = cosf(beta);
    const float sb = sinf(beta);

    rotation[0][0] = ca;
    rotation[0][1] = sa * sb;
    rotation[0][2] = sa * cb;
    rotation[1][0] = 0.0f;
    rotation[1][1] = cb;
    rotation[1][2] = -sb;
    rotation[2][0] = -sa;
    rotation[2][1] = ca * sb;
    rotation[2][2] = ca * cb;
}

static void matrix_multiply(const float left[3][3],
                            const float right[3][3],
                            float output[3][3])
{
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            output[row][col] = 0.0f;
            for (int index = 0; index < 3; ++index) {
                output[row][col] += left[row][index] * right[index][col];
            }
        }
    }
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
    float gimbal_r[3][3];
    float thrust_r[3][3];
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
    CHECK(output.alpha_rad > 0.0f, 10);
    CHECK(fabsf(output.beta_rad) < 1.0e-4f, 11);
    CHECK(debug.desired_attitude_rpy_rad[1] > 0.0f, 12);
    CHECK(debug.moment_cmd_n_m[1] > 0.0f, 13);
    CHECK(output.servo_beta_us < DRV_COAX_CTRL_SERVO_BETA_CENTER_US, 14);
    CHECK(nearly_equal(
        debug.moment_cmd_n_m[1],
        0.569f * DRV_AIRFRAME_PITCH_THRUST_LEVER_ARM_M *
            debug.total_force_n * sinf(output.alpha_rad) * cosf(output.beta_rad),
        1.0e-5f), 15);
    rpy_matrix(debug.desired_attitude_rpy_rad, desired_body_r);
    gimbal_matrix(output.alpha_rad, output.beta_rad, gimbal_r);
    matrix_multiply(desired_body_r, gimbal_r, thrust_r);
    {
        const float force_norm = sqrtf(
            debug.accel_out_m_s2[0] * debug.accel_out_m_s2[0] +
            DRV_AIRFRAME_GRAVITY_M_S2 * DRV_AIRFRAME_GRAVITY_M_S2);
        CHECK(nearly_equal(thrust_r[0][2],
                           debug.accel_out_m_s2[0] / force_norm,
                           2.0e-4f), 16);
        CHECK(nearly_equal(thrust_r[2][2],
                           DRV_AIRFRAME_GRAVITY_M_S2 / force_norm,
                           2.0e-4f), 17);
    }

    reset_case(&attitude, &reference);
    reference.vy_m_s = 0.8f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(output.beta_rad < 0.0f, 20);
    CHECK(fabsf(output.alpha_rad) < 1.0e-4f, 21);
    CHECK(debug.desired_attitude_rpy_rad[0] < 0.0f, 22);
    CHECK(debug.moment_cmd_n_m[0] < 0.0f, 23);
    CHECK(output.servo_alpha_us > DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US, 24);
    CHECK(nearly_equal(
        debug.moment_cmd_n_m[0],
        0.581f * DRV_AIRFRAME_ROLL_THRUST_LEVER_ARM_M *
            debug.total_force_n * sinf(output.beta_rad),
        1.0e-5f), 25);

    reset_case(&attitude, &reference);
    attitude.pitch_rad = 0.10f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    CHECK(output.alpha_rad < 0.0f, 30);
    CHECK(output.servo_beta_us > DRV_COAX_CTRL_SERVO_BETA_CENTER_US, 31);

    reset_case(&attitude, &reference);
    attitude.roll_rad = 0.10f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    CHECK(output.beta_rad > 0.0f, 32);

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
    CHECK(debug.rate_error_rad_s[0] < 0.0f, 33);
    CHECK(debug.moment_cmd_n_m[0] > 0.0f, 34);
    CHECK(output.beta_rad > 0.0f, 35);

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
    CHECK(integral_before < 0.0f, 50);
    reference.horizontal_velocity_valid = 0U;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(debug.velocity_integral_m[0], integral_before, 1.0e-7f), 51);

    reset_case(&attitude, &reference);
    attitude.pitch_rad = 1.0f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(fabsf(output.alpha_rad) <= 0.314159f + 1.0e-6f, 60);
    CHECK((debug.protection_flags & DRV_COAX_CTRL_PROTECT_ATTITUDE) != 0U, 61);

    reset_case(&attitude, &reference);
    attitude.pitch_rad = 2.967060f;
    reference.vx_m_s = 0.8f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK((debug.protection_flags & DRV_COAX_CTRL_PROTECT_ATTITUDE) != 0U, 62);
    CHECK(nearly_equal(debug.horizontal_command_scale, 0.0f, 1.0e-6f), 63);

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
