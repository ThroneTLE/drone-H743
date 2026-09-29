from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


from _airframe_fixture import AIRFRAME_FIXTURE_C, AIRFRAME_SOURCE, PROP_MAP_SOURCE


ROOT = Path(__file__).resolve().parents[1]


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
    reference->navigation_position_valid = 1U;
    reference->navigation_velocity_valid = 1U;
    attitude->acceleration_valid = 1U;
}

int main(void)
{
    DRV_COAX_CTRL_AttitudeInput attitude;
    DRV_COAX_CTRL_Reference reference;
    DRV_COAX_CTRL_Output output;
    DRV_COAX_CTRL_Debug debug;
    DRV_COAX_CTRL_Params params;
    DRV_COAX_CTRL_ServoCalibration servo_cal;
    uint16_t servo_alpha_us;
    uint16_t servo_beta_us;
    float desired_body_r[3][3];

    DRV_COAX_CTRL_Init();
    airframe_load_reference();


    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(output.alpha_rad, 0.0f, 1.0e-5f), 1);
    CHECK(nearly_equal(output.beta_rad, 0.0f, 1.0e-5f), 2);
    CHECK(nearly_equal(debug.total_force_n,
                       DRV_Airframe_Get()->mass_kg * DRV_Airframe_Get()->gravity_m_s2,
                       1.0e-3f), 3);
    CHECK(nearly_equal(output.thrust_upper_n, debug.total_force_n * 0.5f, 1.0e-3f), 4);
    CHECK(debug.protection_flags == 0U, 5);
    CHECK(nearly_equal(debug.horizontal_command_scale, 1.0f, 1.0e-6f), 6);

    reset_case(&attitude, &reference);
    reference.vx_m_s = 0.8f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    /*
     * Tilt-to-moment polarity is now derived from the measured geometry
     * (2026-09-27: signed lever cg_z - servo-axis z): the tilt axis is below
     * the CG, so a positive tilt makes a positive FLU moment.  Flying forward needs
     * nose-down (+pitch), so the rotor axis must tilt REARWARD -- alpha > 0.
     * That is counter-intuitive and it is exactly what the old -1 model got
     * backwards; checks 12/13 are about the attitude target and are unchanged.
     */
    CHECK(output.alpha_rad > 0.0f, 10);
    CHECK(fabsf(output.beta_rad) < 1.0e-4f, 11);
    CHECK(debug.desired_attitude_rpy_rad[1] > 0.0f, 12);
    CHECK(debug.moment_cmd_n_m[1] > 0.0f, 13);
    CHECK(output.servo_beta_us < DRV_COAX_CTRL_SERVO_BETA_CENTER_US, 14);
    /*
     * 2026-09-27: the forward model is now the signed geometric lever
     * cg_z - servo2_axis_z (no EFFECTIVENESS factor).  The fixture's axis
     * height is chosen so this equals the retired 0.569 * 0.145, so the
     * expected number and its 1e-5 tolerance are unchanged.
     */
    CHECK(nearly_equal(
        debug.moment_achieved_n_m[1],
        (DRV_Airframe_Get()->cg_z_m - DRV_Airframe_Get()->servo2_axis_z_m) *
            debug.total_force_n * sinf(output.alpha_rad) * cosf(output.beta_rad),
        1.0e-5f), 15);
    {
        const float force_norm = sqrtf(
            debug.accel_out_m_s2[0] * debug.accel_out_m_s2[0] +
            DRV_Airframe_Get()->gravity_m_s2 * DRV_Airframe_Get()->gravity_m_s2);
        const float target_pitch = atan2f(debug.accel_out_m_s2[0],
                                          DRV_Airframe_Get()->gravity_m_s2);
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
                           DRV_Airframe_Get()->gravity_m_s2 / force_norm,
                           2.0e-4f), 19);
    }

    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.position.pos_kp[2] = 1.0f;
    params.position.vel_kp[2] = 0.0f;
    params.position.vel_ki[2] = 0.50f;
    DRV_COAX_CTRL_SetParams(&params);
    /* R-F6-2: +Z is up.  Same physical scenario as before the migration
     * (measured 0.20m up, target 0.40m up -- needs to climb), values negated. */
    attitude.z_m = 0.20f;
    reference.z_m = 0.40f;
    for (int step = 0; step < 50; ++step) {
        DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    }
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(debug.pos_z_i_m_s2 > 0.09f, 102);
    CHECK(debug.accel_out_m_s2[2] > 0.09f, 103);
    CHECK(debug.total_force_n >
          DRV_Airframe_Get()->mass_kg * DRV_Airframe_Get()->gravity_m_s2,
          104);

    /*
     * R-F6-2: removing FORCE_FRAME_ROLL_SIGN changes the servo-facing roll
     * output for this scenario (not a relabeling -- the attitude error is
     * genuinely different once desired_body_r is built from the unsigned
     * target_roll_rad).  target_attitude_rp_rad/desired_attitude_rpy_rad[0]
     * were already reported unsigned before this migration (the old code
     * multiplied the recovered angle back by FORCE_FRAME_ROLL_SIGN just for
     * debug/reporting), so checks 22/26/27 are unaffected; beta_rad,
     * moment_cmd_n_m[0] and servo_alpha_us flip, confirmed against the real
     * compiled controller (2026-09-06).
     *
     * 2026-09-06: the tilt-to-moment polarity is now derived from measured
     * geometry rather than a -1 macro, so beta_rad and servo_alpha_us flip once
     * more.  The attitude-side checks 22/23/26/27 are again unaffected -- they
     * describe what the aircraft should do, not how the actuator gets there.
     */
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
        debug.moment_achieved_n_m[0],
        (DRV_Airframe_Get()->cg_z_m - DRV_Airframe_Get()->servo1_axis_z_m) *
            debug.total_force_n * sinf(output.beta_rad),
        1.0e-5f), 25);
    {
        const float target_roll = -atan2f(debug.accel_out_m_s2[1],
                                          DRV_Airframe_Get()->gravity_m_s2);
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
    CHECK(debug.velocity_p_m_s2[0] > 0.05f, 29);
    CHECK(output.alpha_rad >= 0.0f, 30);

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
    CHECK(output.alpha_rad > 0.0f, 74);

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

    /* R-F6-2: same reasoning as checks 20-24 above. */
    reset_case(&attitude, &reference);
    reference.direct_attitude_target_valid = 1U;
    reference.target_roll_rad = 0.10f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(debug.target_attitude_rp_rad[0], 0.10f, 1.0e-5f), 75);
    CHECK(nearly_equal(debug.desired_attitude_rpy_rad[0], 0.10f, 1.0e-5f), 76);
    CHECK(output.beta_rad > 0.0f, 77);
    CHECK(output.servo_alpha_us < DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US, 78);

    /* Disturbance, not command: nose-down measured must be pushed back up, so
     * the tilt is the mirror of check 74's commanded nose-down. */
    reset_case(&attitude, &reference);
    attitude.pitch_rad = 0.10f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    CHECK(output.alpha_rad < 0.0f, 31);
    CHECK(output.servo_beta_us > DRV_COAX_CTRL_SERVO_BETA_CENTER_US, 32);

    /* Likewise the mirror of check 77's commanded right-wing-down. */
    reset_case(&attitude, &reference);
    attitude.roll_rad = 0.10f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    CHECK(output.beta_rad < 0.0f, 33);

    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.attitude.att_kp[0] = 0.0f;
    params.attitude.att_kp[1] = 0.0f;
    params.rate.kp[1] = 0.0f;
    DRV_COAX_CTRL_SetParams(&params);
    /* Canonical FLU: gyro_x > 0 IS the +roll direction (drv_frame_contract.h).
     * A negative roll rate must therefore be damped with a positive moment. */
    attitude.gyro_x_rad_s = -0.70f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(debug.rate_error_rad_s[0] > 0.0f, 34);
    CHECK(debug.moment_cmd_n_m[0] > 0.0f, 35);
    CHECK(output.beta_rad > 0.0f, 36);

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
    CHECK(debug.velocity_p_m_s2[0] > 0.3f, 51);
    reference.horizontal_velocity_valid = 0U;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(debug.horizontal_command_scale, 0.0f, 1.0e-6f), 52);

    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.position.pos_kp[0] = 0.0f;
    params.position.pos_kp[1] = 0.0f;
    params.position.vel_kp[0] = 2.0f;
    params.position.vel_kp[1] = 2.0f;
    DRV_COAX_CTRL_SetParams(&params);
    attitude.vx_m_s = -2.0f;
    attitude.vy_m_s = 0.0f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(debug.velocity_p_m_s2[0], 4.0f, 1.0e-6f), 53);
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
    CHECK(fabsf(debug.moment_cmd_n_m[1]) > 1.0e-4f, 67);

    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.vel_loop_enable = 0.0f;
    DRV_COAX_CTRL_SetParams(&params);
    reference.ax_m_s2 = 1.0f;
    attitude.pitch_rad = atan2f(reference.ax_m_s2, DRV_Airframe_Get()->gravity_m_s2);
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(output.alpha_rad, 0.0f, 2.0e-4f), 70);
    CHECK(nearly_equal(output.beta_rad, 0.0f, 2.0e-4f), 71);
    CHECK(nearly_equal(debug.desired_attitude_rpy_rad[1],
                       attitude.pitch_rad,
                       2.0e-4f), 72);

    reset_case(&attitude, &reference);
    DRV_COAX_CTRL_GetParams(&params);
    params.attitude.att_kp[0] = 0.0f;
    params.attitude.att_kp[1] = 0.0f;
    params.rate.kp[0] = 0.0f;
    params.rate.kp[1] = 0.0f;
    params.rate.ki[0] = 0.0f;   /* default pitch ki is non-zero since 2026-09-28: isolate the gyroscopic term */
    params.rate.ki[1] = 0.0f;
    DRV_COAX_CTRL_SetParams(&params);
    attitude.gyro_x_rad_s = 0.7f;
    attitude.gyro_y_rad_s = -0.4f;
    attitude.gyro_z_rad_s = 0.3f;
    DRV_COAX_CTRL_Run(&attitude, &reference, &output);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    CHECK(nearly_equal(
        debug.moment_cmd_n_m[0],
        (DRV_Airframe_Get()->izz_kgm2 - DRV_Airframe_Get()->iyy_kgm2) *
            attitude.gyro_y_rad_s * attitude.gyro_z_rad_s,
        1.0e-6f), 80);
    CHECK(nearly_equal(
        debug.moment_cmd_n_m[1],
        (DRV_Airframe_Get()->ixx_kgm2 - DRV_Airframe_Get()->izz_kgm2) *
            attitude.gyro_z_rad_s *
            attitude.gyro_x_rad_s,
        1.0e-6f), 81);

    DRV_COAX_CTRL_GetDefaultServoCalibration(&servo_cal);
    servo_cal.center_us[0] = 1475U;
    servo_cal.min_us[0] = 1200U;
    servo_cal.max_us[0] = 1800U;
    servo_cal.pulse_sign[0] = -1;
    servo_cal.center_us[1] = 1525U;
    servo_cal.min_us[1] = 1250U;
    servo_cal.max_us[1] = 1850U;
    servo_cal.pulse_sign[1] = 1;
    CHECK(DRV_COAX_CTRL_SetServoCalibration(&servo_cal) == 1U, 90);
    DRV_COAX_CTRL_BodyTiltRadToServoPulses(0.0f, 0.0f,
                                           &servo_alpha_us, &servo_beta_us);
    CHECK(servo_alpha_us == 1475U, 91);
    CHECK(servo_beta_us == 1525U, 92);
    DRV_COAX_CTRL_BodyTiltRadToServoPulses(0.0f, -0.1f,
                                           &servo_alpha_us, &servo_beta_us);
    CHECK(servo_alpha_us < 1475U, 93);
    servo_cal.pulse_sign[0] = 0;
    CHECK(DRV_COAX_CTRL_SetServoCalibration(&servo_cal) == 0U, 94);

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
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)
