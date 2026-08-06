from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_servo_output_compensates_90_degree_ccw_mounting() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")
    header = read("Driver/Inc/drv_coax_ctrl.h")

    assert "DRV_COAX_CTRL_SERVO_ALPHA_SIGN    (1.0f)" in source
    assert "DRV_COAX_CTRL_SERVO_BETA_SIGN     (1.0f)" in source
    assert "void DRV_COAX_CTRL_BodyTiltRadToServoPulses(float body_x_tilt_rad," in header
    assert "static void coax_ctrl_body_tilt_to_servo_tilts(float body_x_tilt_rad," in source
    assert "*servo_alpha_tilt_rad = -body_y_tilt_rad;" in source
    assert "*servo_beta_tilt_rad = -body_x_tilt_rad;" in source
    assert "DRV_COAX_CTRL_BodyTiltRadToServoPulses(output->alpha_rad," in source
    assert "output->beta_rad," in source
    assert "servo_alpha_tilt_rad * DRV_COAX_CTRL_SERVO_ALPHA_SIGN" in source
    assert "servo_beta_tilt_rad * DRV_COAX_CTRL_SERVO_BETA_SIGN" in source
    assert "output->alpha_rad * DRV_COAX_CTRL_SERVO_ALPHA_SIGN" not in source
    assert "output->beta_rad * DRV_COAX_CTRL_SERVO_BETA_SIGN" not in source


def test_tilt_limit_is_twenty_eight_degrees_in_driver_controller() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")
    app_control = read("App/Src/app_control.c")

    assert "DRV_COAX_CTRL_TILT_LIMIT_RAD 0.4886922f" in source
    assert "value <= DRV_COAX_CTRL_TILT_LIMIT_RAD" in source
    assert "#define APP_CONTROL_TILT_LIMIT_MAX_DEG 28.0f" in app_control
    assert "#define APP_CONTROL_TILT_LIMIT_DEFAULT_RAD 0.4886922f" in app_control
    assert "#define APP_CONTROL_TILT_LIMIT_LEGACY_18_RAD 0.31415927f" in app_control
    assert "#define APP_CONTROL_TILT_LIMIT_LEGACY_25_RAD 0.43633231f" in app_control
    assert "params.tilt_limit_rad = APP_CONTROL_TILT_LIMIT_DEFAULT_RAD;" in app_control
    assert "APP_CONTROL_SERVO_ANGLE_MAX_DEG" not in app_control


def test_generated_controller_is_not_built_or_called() -> None:
    cmake = read("CMakeLists.txt")
    source = read("Driver/Src/drv_coax_ctrl.c")

    assert "Driver/Generated/coax_ctrl" not in cmake
    assert "coax_tiltrotor_controller_codegen" not in cmake
    assert "coax_tiltrotor_controller_codegen(" not in source


def test_bus_servos_are_180_degree_centered_and_limited_to_90_degrees() -> None:
    header = read("Driver/Inc/drv_coax_ctrl.h")
    source = read("Driver/Src/drv_coax_ctrl.c")
    freertos = read("Core/Src/freertos.c")

    assert "#define DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US 1500U" in header
    assert "#define DRV_COAX_CTRL_SERVO_BETA_CENTER_US  1500U" in header
    assert "#define DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US  500U" in header
    assert "#define DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US 2500U" in header
    assert "#define DRV_COAX_CTRL_SERVO_CENTER_MIN_US(center_us)" in header
    assert "#define DRV_COAX_CTRL_SERVO_CENTER_MAX_US(center_us)" in header
    assert "#define DRV_COAX_CTRL_SERVO_ALPHA_MIN_US" in header
    assert "#define DRV_COAX_CTRL_SERVO_ALPHA_MAX_US" in header
    assert "#define DRV_COAX_CTRL_SERVO_BETA_MIN_US" in header
    assert "#define DRV_COAX_CTRL_SERVO_BETA_MAX_US" in header
    assert "DRV_COAX_CTRL_SERVO_MIN_US" not in header
    assert "DRV_COAX_CTRL_SERVO_MAX_US" not in header
    assert "#define DRV_COAX_CTRL_SERVO_LIMIT_DELTA_US  1000U" in header
    assert "#define DRV_COAX_CTRL_SERVO_TRAVEL_DEG       180.0f" in header
    assert "#define DRV_COAX_CTRL_SERVO_LIMIT_DEG         90.0f" in header
    assert "DRV_COAX_CTRL_AlphaTiltRadToServoPulse" in header
    assert "DRV_COAX_CTRL_BetaTiltRadToServoPulse" in header
    assert "DRV_COAX_CTRL_BodyTiltRadToServoPulses" in header
    assert "coax_ctrl_tilt_rad_to_servo_pulse(float tilt_rad," in source
    assert "DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US" in source
    assert "DRV_COAX_CTRL_SERVO_LIMIT_RAD" in source
    assert "DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US" in source
    assert "DRV_COAX_CTRL_SERVO_BETA_CENTER_US" in source
    assert "DRV_COAX_CTRL_SERVO_ALPHA_MIN_US" in source
    assert "DRV_COAX_CTRL_SERVO_ALPHA_MAX_US" in source
    assert "DRV_COAX_CTRL_SERVO_BETA_MIN_US" in source
    assert "DRV_COAX_CTRL_SERVO_BETA_MAX_US" in source
    assert "DRV_COAX_CTRL_BodyTiltRadToServoPulses(body_x_tilt_rad," in freertos
    assert "body_y_tilt_rad," in freertos
    assert "moves[0].pulse_us = DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US;" in freertos
    assert "moves[1].pulse_us = DRV_COAX_CTRL_SERVO_BETA_CENTER_US;" in freertos


def test_manual_and_ident_servo_limits_follow_each_calibrated_center() -> None:
    control = read("App/Src/app_control.c")
    ident = read("App/Src/app_ident.c")

    assert "app_control_servo_clamp_pulse(uint32_t index, uint16_t pulse_us)" in control
    assert "DRV_COAX_CTRL_SERVO_ALPHA_MIN_US" in control
    assert "DRV_COAX_CTRL_SERVO_ALPHA_MAX_US" in control
    assert "DRV_COAX_CTRL_SERVO_BETA_MIN_US" in control
    assert "DRV_COAX_CTRL_SERVO_BETA_MAX_US" in control
    assert "app_control_servo_clamp_pulse(index," in control
    assert "DRV_COAX_CTRL_SERVO_ALPHA_MIN_US" in ident
    assert "DRV_COAX_CTRL_SERVO_ALPHA_MAX_US" in ident
    assert "DRV_COAX_CTRL_SERVO_BETA_MIN_US" in ident
    assert "DRV_COAX_CTRL_SERVO_BETA_MAX_US" in ident
    assert "DRV_COAX_CTRL_SERVO_MIN_US" not in ident
    assert "DRV_COAX_CTRL_SERVO_MAX_US" not in ident


def test_mbd_controller_gains_are_runtime_coax_params() -> None:
    header = read("Driver/Inc/drv_coax_ctrl.h")
    wrapper = read("Driver/Src/drv_coax_ctrl.c")
    app_control = read("App/Src/app_control.c")

    assert "DRV_COAX_CTRL_Params" in header
    assert "DRV_COAX_CTRL_SetParam" in header
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(roll_angle_kp)' in wrapper
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(pitch_angle_kp)' in wrapper
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(roll_rate_kd)' in wrapper
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(pos_z_ki)' in wrapper
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_x_kp)' in wrapper
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(mass_kg)' not in wrapper
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(gravity_m_s2)' not in wrapper
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(tilt_lever_arm_m)' not in wrapper
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(yaw_inertia)' not in wrapper
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(motor_single_max_thrust_n)' not in wrapper
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(yaw_torque_upper_m_per_n)' not in wrapper
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(yaw_torque_lower_m_per_n)' not in wrapper
    assert "params->vel_loop_enable = 1.0f;" in wrapper
    assert "params->pos_x_kp = 0.30f;" in wrapper
    assert "params->pos_y_kp = 0.30f;" in wrapper
    assert "params->pos_z_ki = 0.25f;" in wrapper
    assert "params->vel_x_kd = 0.80f;" in wrapper
    assert "params->vel_y_kd = 0.80f;" in wrapper
    assert "params->vel_loop_x_kp = 0.0f;" in wrapper
    assert "params->vel_loop_x_ki = 0.0f;" in wrapper
    assert "params->vel_loop_y_kp = 0.0f;" in wrapper
    assert "params->vel_loop_y_ki = 0.0f;" in wrapper
    assert '"coax." #field' in wrapper
    assert "coax_tiltrotor_controller_codegen(" not in wrapper
    assert "DRV_COAX_CTRL_SetParam(name, value)" in app_control
    assert '{ "pos_z_kp",       "coax.pos_z_kp"       }' in app_control
    assert '{ "pos_z_ki",       "coax.pos_z_ki"       }' in app_control
    assert "PARAM name=%s value=%s" in app_control


def test_controller_limit_params_are_removed_except_tilt_angle() -> None:
    header = read("Driver/Inc/drv_coax_ctrl.h")
    wrapper = read("Driver/Src/drv_coax_ctrl.c")
    freertos = read("Core/Src/freertos.c")
    app_control = read("App/Src/app_control.c")
    capture = read("tools/vofa_serial_capture.py")
    flight_log = read("tools/flight_log_receive.py")

    removed = [
        "accel_xy_limit_m_s2",
        "accel_z_limit_m_s2",
        "vel_loop_output_limit_m_s2",
        "vel_loop_i_limit_m_s2",
        "yaw_rate_limit_rad_s",
        "min_total_force_n",
        "max_total_force_n",
    ]
    combined = "\n".join([header, wrapper, freertos, capture, flight_log])
    for name in removed:
        assert name not in combined
    for name in removed[:5]:
        assert name not in app_control
    assert "tilt_limit_rad" in header
    assert "DRV_COAX_CTRL_PARAM_ENTRY(tilt_limit_rad)" in wrapper
    assert "coax_ctrl_params.tilt_limit_rad" in wrapper


def test_controller_wrapper_exposes_velocity_first_vector_control_inputs() -> None:
    header = read("Driver/Inc/drv_coax_ctrl.h")
    wrapper = read("Driver/Src/drv_coax_ctrl.c")
    freertos = read("Core/Src/freertos.c")

    assert "float vx_m_s;" in header
    assert "float ax_m_s2;" in header
    assert "float dt_sec;" in header
    assert "uint8_t horizontal_velocity_valid;" in header
    assert "range height above ground is exposed to the controller as z = -height" in header
    assert "IMU axes are already rotated to body FRD before this layer" in header
    assert "float yaw_rate_rad_s;" in header
    assert "float yaw_accel_rad_s2;" in header
    assert "Paper psi_d, psi_d_dot and psi_d_ddot references" in header
    assert "reference->ax_m_s2" in wrapper
    assert "reference->vx_m_s" in wrapper
    assert "static void coax_ctrl_local_down_to_body" in wrapper
    assert "Controller inputs use the existing local frame: X forward, Y right, Z down" in wrapper
    assert "coax_ctrl_local_down_to_body(attitude," in wrapper
    assert "solution->desired_force_local_n," in wrapper
    assert "coax_ctrl_params.gravity_m_s2 - debug->accel_out_m_s2[2]" in wrapper
    assert "debug->pos_z_i_m_s2" in wrapper
    assert "DRV_COAX_CTRL_POS_Z_I_ACCEL_LIMIT_M_S2" in wrapper
    assert "reference->yaw_rate_rad_s - attitude->gyro_z_rad_s" in wrapper
    assert "reference->yaw_accel_rad_s2 +" in wrapper
    assert "coax_ctrl_wrap_pi(reference->yaw_rad - attitude->yaw_rad)" in wrapper
    assert "debug->force_cmd_n[0]" in wrapper
    assert "debug->force_cmd_n[1]" in wrapper
    assert "STABILIZER_XY_VEL_REF_MAX_M_S" in freertos
    assert (
        "reference.vx_m_s =\n"
        "              stabilizer_rc_normalized(ch[STABILIZER_RC_CH_PITCH])"
        in freertos
    )
    assert "static float stabilizer_rc_throttle_height_rate_m_s(uint16_t ch_us)" in freertos
    assert "return stabilizer_rc_normalized(ch_us) * STABILIZER_Z_REF_RATE_MAX_M_S;" in freertos
    assert "stabilizer_rc_throttle_thrust_bias_m_s2" not in freertos
    assert "STABILIZER_Z_THRUST_BIAS_MAX_M_S2" not in freertos
    assert "reference.ax_m_s2 =" in freertos
    assert "reference.az_m_s2 = 0.0f;" in freertos
    assert "height_ref_m +=\n                stabilizer_rc_throttle_height_rate_m_s(" in freertos
    assert "StabilizerVelocityPidState" not in freertos
    assert "stabilizer_velocity_pid_step" not in freertos
    assert "reference.dt_sec = ctrl_dt_sec;" in freertos
    assert "velocity_loop_enabled = (vel_loop_enable >= 0.5f) ? 1U : 0U;" in freertos
    assert "reference.horizontal_velocity_valid = velocity_loop_enabled;" in freertos
    assert "position_state_x_m +\n                                     velocity_control_x_m_s * ctrl_dt_sec" in freertos
    assert "position_ref_x_m += reference.vx_m_s * ctrl_dt_sec;" in freertos
    assert "reference.x_m = position_ref_x_m;" in freertos
    assert "reference.y_m = position_ref_y_m;" in freertos
    assert "#define STABILIZER_VELOCITY_MEAS_Y_SIGN (1.0f)" in freertos
    assert "stabilizer_velocity_estimator_control_ok(&vel_estimator, now)" not in freertos
    assert "attitude.vx_m_s = velocity_control_x_m_s;" in freertos
    assert "attitude.vy_m_s = velocity_control_y_m_s;" in freertos
    assert "relative_height_m = range_height_m - height_origin_m;" in freertos
    assert "attitude.z_m = -relative_height_m;" in freertos
    assert "attitude.vz_m_s = -range_velocity_m_s;" in freertos
    assert "stabilizer_clamp_f32(position_ref_z_m," in freertos
    assert "STABILIZER_Z_POS_ERR_MAX_M" in freertos
    assert "reference.vz_m_s = 0.0f;" in freertos
    assert "reference.yaw_rate_rad_s = yaw_rate_ref_rad_s;" in freertos
    assert "reference.yaw_accel_rad_s2 = 0.0f;" in freertos
    assert "velocity_state_x_m_s : 0.0f;" not in freertos
    assert "velocity_ref_x_m_s" not in freertos
    assert "memset(&reference, 0, sizeof(reference));" in freertos


def test_velocity_damping_has_independent_acceleration_limit() -> None:
    wrapper = read("Driver/Src/drv_coax_ctrl.c")
    accel_cmd = wrapper.split("static void coax_ctrl_compute_accel_cmd", 1)[1]
    accel_cmd = accel_cmd.split("static void coax_ctrl_compute_balance_solution", 1)[0]

    assert "#define DRV_COAX_CTRL_HORIZONTAL_ACCEL_LIMIT_M_S2 3.70f" in wrapper
    assert "#define DRV_COAX_CTRL_VEL_D_ACCEL_LIMIT_M_S2 3.70f" in wrapper
    assert "coax_ctrl_params.vel_x_kd * (reference->vx_m_s - attitude->vx_m_s)" in accel_cmd
    assert "coax_ctrl_params.vel_y_kd * (reference->vy_m_s - attitude->vy_m_s)" in accel_cmd
    assert (
        "coax_ctrl_clamp_f32(debug->vel_d_m_s2[0],\n"
        "                            -DRV_COAX_CTRL_VEL_D_ACCEL_LIMIT_M_S2,"
        in accel_cmd
    )
    assert (
        "coax_ctrl_clamp_f32(debug->vel_d_m_s2[1],\n"
        "                            -DRV_COAX_CTRL_VEL_D_ACCEL_LIMIT_M_S2,"
        in accel_cmd
    )
    assert "DRV_COAX_CTRL_HORIZONTAL_ACCEL_LIMIT_M_S2 / horizontal_norm" in accel_cmd


def test_manual_total_force_mode_bypasses_all_position_and_velocity_terms() -> None:
    header = read("Driver/Inc/drv_coax_ctrl.h")
    source = read("Driver/Src/drv_coax_ctrl.c")

    assert "uint8_t manual_total_force_valid;" in header
    assert "float manual_total_force_n;" in header
    assert "float DRV_COAX_CTRL_MotorPulseToTotalThrust(uint16_t pulse_us);" in header
    assert "DRV_COAX_CTRL_MotorPulseToTotalThrust(uint16_t pulse_us)" in source
    assert "if (reference->manual_total_force_valid != 0U)" in source
    assert "debug->pos_p_m_s2[2] = 0.0f;" in source
    assert "debug->vel_d_m_s2[2] = 0.0f;" in source
    assert "solution->desired_force_local_n[2] =\n            coax_ctrl_clamp_f32(reference->manual_total_force_n," in source


def test_roll_pitch_physical_moment_gains_are_runtime_params() -> None:
    wrapper = read("Driver/Src/drv_coax_ctrl.c")

    assert "float roll_angle_kp;" in read("Driver/Inc/drv_coax_ctrl.h")
    assert "float pitch_angle_kp;" in read("Driver/Inc/drv_coax_ctrl.h")
    # Gains are now magnitudes only; negative feedback comes from the control
    # law structure. See tests/test_coax_sign_convention.py for why: a stored
    # negative gain used to absorb polarity errors, so the aircraft self-levelled
    # while responding backwards to the stick.
    assert "params->roll_angle_kp = 0.0671f;" in wrapper
    assert "params->pitch_angle_kp = 0.0660f;" in wrapper
    assert "params->roll_rate_kd = 0.1104f;" in wrapper
    assert "params->pitch_rate_kd = 0.1138f;" in wrapper
    assert "const float kr_roll = fabsf(coax_ctrl_params.roll_angle_kp);" in wrapper
    assert "const float kr_pitch = fabsf(coax_ctrl_params.pitch_angle_kp);" in wrapper
    assert "const float kd_roll = fabsf(coax_ctrl_params.roll_rate_kd);" in wrapper
    assert "const float kd_pitch = fabsf(coax_ctrl_params.pitch_rate_kd);" in wrapper
    assert "params->yaw_angle_kp = 1.0f;" in wrapper
    assert "params->yaw_rate_kd = 0.15f;" in wrapper
    assert "#define DRV_COAX_CTRL_PROP9047_YAW_M_PER_N 0.0001f" in wrapper
    assert "params->vel_x_kd = 0.80f;" in wrapper
    assert "params->vel_y_kd = 0.80f;" in wrapper
    assert "params->vel_z_kd = 0.0f;" in wrapper
    assert "params->pos_z_ki = 0.25f;" in wrapper
    assert "return (value >= 0.0f) ? 1U : 0U;" in wrapper
    assert "entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_enable)" in wrapper


def test_nonlinear_balance_controller_uses_so3_error_and_realtime_moment_arm_inverse() -> None:
    wrapper = read("Driver/Src/drv_coax_ctrl.c")

    rpy_helper = wrapper.split("static void coax_ctrl_rpy_matrix", 1)[1]
    rpy_helper = rpy_helper.split("static void coax_ctrl_attitude_matrix", 1)[0]
    attitude_helper = wrapper.split("static void coax_ctrl_attitude_matrix", 1)[1]
    attitude_helper = attitude_helper.split("static void coax_ctrl_attitude_error", 1)[0]
    solve = wrapper.split("static void coax_ctrl_compute_balance_solution", 1)[1]
    solve = solve.split("static float coax_ctrl_balance_protection_scale", 1)[0]

    assert "#define DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN  (-1.0f)" in wrapper
    assert "#define DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN (1.0f)" in wrapper
    assert "#define DRV_COAX_CTRL_RATE_FRAME_ROLL_SIGN   (1.0f)" in wrapper
    assert "#define DRV_COAX_CTRL_RATE_FRAME_PITCH_SIGN  (1.0f)" in wrapper
    assert (
        "DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN * attitude->roll_rad"
        in attitude_helper
    )
    assert (
        "DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN * attitude->pitch_rad"
        in attitude_helper
    )
    assert "coax_ctrl_rpy_matrix(roll_rad, pitch_rad, attitude->yaw_rad, rotation);" in attitude_helper
    assert "rotation[0][2] = (cr * sp * cy) + (sr * sy);" in rpy_helper
    assert "rotation[1][2] = (cr * sp * sy) - (sr * cy);" in rpy_helper
    assert "rotation[2][2] = cr * cp;" in rpy_helper
    assert "DRV_COAX_CTRL_RATE_FRAME_ROLL_SIGN * attitude->gyro_x_rad_s" in solve
    assert "DRV_COAX_CTRL_RATE_FRAME_PITCH_SIGN * attitude->gyro_y_rad_s" in solve
    assert "coax_ctrl_gimbal_matrix" not in wrapper
    assert "coax_ctrl_build_thrust_frame" not in wrapper
    assert "DRV_COAX_CTRL_BALANCE_ITERATIONS" not in wrapper
    assert "thrust_frame_r" not in wrapper
    assert "Horizontal outer loop owns acceleration only" in solve
    assert "atan2f(solution->desired_force_local_n[0]," in solve
    assert "-atan2f(solution->desired_force_local_n[1] * cosf(target_pitch_rad)," in solve
    assert (
        "target_roll_force_rad =\n"
        "        DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN * target_roll_rad;"
        in solve
    )
    assert (
        "target_pitch_force_rad =\n"
        "        DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN * target_pitch_rad;"
        in solve
    )
    assert "coax_ctrl_rpy_matrix(target_roll_force_rad," in solve
    assert "target_pitch_force_rad," in solve
    assert "reference->direct_attitude_target_valid != 0U" in solve
    assert "coax_ctrl_clamp_f32(reference->target_roll_rad," in solve
    assert "coax_ctrl_clamp_f32(reference->target_pitch_rad," in solve
    assert "reference->yaw_rad," in solve
    assert "solution->desired_body_r" in solve
    assert "coax_ctrl_attitude_error(solution->desired_body_r," in solve
    assert "(-kr_roll * solution->attitude_error[0])" in solve
    assert "(kd_roll * solution->rate_error_rad_s[0])" in solve
    assert "DRV_AIRFRAME_IZZ_KGM2 - DRV_AIRFRAME_IYY_KGM2" in solve
    assert "DRV_AIRFRAME_IXX_KGM2 - DRV_AIRFRAME_IZZ_KGM2" in solve
    assert "DRV_AIRFRAME_IYY_KGM2 - DRV_AIRFRAME_IXX_KGM2" in solve
    assert "coax_ctrl_roll_moment_from_tilt" in wrapper
    assert "coax_ctrl_pitch_moment_from_tilt" in wrapper
    assert "sinf(beta_rad)" in wrapper
    assert "sinf(alpha_rad)" in wrapper
    assert "cosf(beta_rad)" in wrapper
    assert "coax_ctrl_solve_roll_tilt_from_moment" in solve
    assert "coax_ctrl_solve_pitch_tilt_from_moment" in solve
    assert "coax_ctrl_apply_attitude_force_feedback" not in wrapper
    assert "debug->force_cmd_n[0] +=" not in wrapper
    assert "debug->target_attitude_rp_rad[0] = target_roll_rad;" in solve
    assert "debug->target_attitude_rp_rad[1] = target_pitch_rad;" in solve
    assert (
        "debug->desired_attitude_rpy_rad[0] *= DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN;"
        in solve
    )
    assert "output->alpha_rad = solution.alpha_rad;" in wrapper
    assert "output->beta_rad = solution.beta_rad;" in wrapper
    assert "DRV_COAX_CTRL_BodyTiltRadToServoPulses(output->alpha_rad," in wrapper
    assert "output->motor_upper_us = DRV_COAX_CTRL_ThrustToMotorPulse" in wrapper


def test_balance_controller_has_no_horizontal_velocity_integral_but_keeps_z_integral() -> None:
    header = read("Driver/Inc/drv_coax_ctrl.h")
    wrapper = read("Driver/Src/drv_coax_ctrl.c")

    assert "float pos_z_i_m_s2;" in header
    assert "float pos_z_ki;" in header
    assert "DRV_COAX_CTRL_PROTECT_VELOCITY_INVALID" in header
    assert "DRV_COAX_CTRL_PROTECT_ATTITUDE" in header
    assert "DRV_COAX_CTRL_PROTECT_MOMENT" in header
    assert "DRV_COAX_CTRL_PROTECT_THRUST" in header
    assert "coax_ctrl_balance_protection_scale" in wrapper
    assert "DRV_COAX_CTRL_ATTITUDE_PROTECT_START_RAD" in wrapper
    assert "DRV_COAX_CTRL_MOMENT_PROTECT_START" in wrapper
    assert "DRV_COAX_CTRL_THRUST_PROTECT_START" in wrapper
    assert "float attitude_tilt_error_rad;" in wrapper
    assert "(desired[0][2] * actual[0][2])" in wrapper
    protection = wrapper.split("static float coax_ctrl_balance_protection_scale", 1)[1]
    protection = protection.split("static void coax_ctrl_compute_balance_command", 1)[0]
    assert "solution->attitude_tilt_error_rad" in protection
    assert "solution->attitude_error_angle_rad" not in protection
    assert "candidate_integral_m" not in wrapper
    assert "(attitude->vx_m_s - reference->vx_m_s) * dt_sec" not in wrapper
    assert "coax_ctrl_update_z_integral" in wrapper
    assert "coax_ctrl_state.pos_z_i_m_s2" in wrapper
    protected = wrapper.split("if (horizontal_scale >= 0.999f)", 1)[1]
    protected = protected.split("debug->horizontal_command_scale", 1)[0]
    assert "coax_ctrl_state.velocity_integral_m[0] = 0.0f;" in protected
    assert "coax_ctrl_state.velocity_integral_m," not in protected
    assert "horizontal_scale" in protected
    assert "DRV_COAX_CTRL_ResetState();" in read("Core/Src/freertos.c")


def test_vofa_exports_compact_slider_parameter_feedback() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define VOFA_DATA_SIZE 28U" in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.roll_rate_kd", &vofa_data[7]);' in freertos
    assert "vofa_data[7] = -vofa_data[7];" not in freertos
    assert "vofa_data[8] = -vofa_data[8];" not in freertos
    assert "vofa_data[9] = -vofa_data[9];" in freertos
    assert "vofa_data[10] = -vofa_data[10];" in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pos_x_kp", &vofa_data[11]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pos_y_kp", &vofa_data[12]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_x_kd", &vofa_data[13]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_y_kd", &vofa_data[14]);' in freertos
    assert "vofa_data[15] = vofa_debug.pos_est_m[0];" in freertos
    assert "vofa_data[16] = vofa_debug.pos_est_m[1];" in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_loop_enable", &vofa_data[17]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.roll_angle_kp", &vofa_data[18]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pitch_angle_kp", &vofa_data[19]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pos_z_kp", &vofa_data[20]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pos_z_ki", &vofa_data[21]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_z_kd", &vofa_data[22]);' in freertos
    assert "vofa_data[18] = -vofa_data[18];" in freertos
    assert "vofa_data[19] = -vofa_data[19];" in freertos
    assert '"coax.motor_single_max_thrust_n"' not in freertos
    assert '"coax.yaw_torque_upper_m_per_n"' not in freertos
    assert '"coax.yaw_torque_lower_m_per_n"' not in freertos
    assert "coax.mass_kg" not in freertos


def test_control_protocol_accepts_colon_param_updates_and_reports_back() -> None:
    app_control = read("App/Src/app_control.c")

    assert "app_control_handle_param_value_line(line)" in app_control
    assert "app_control_after_param_separator" in app_control
    assert "0xEFU" in app_control
    assert "0xBCU" in app_control
    assert "0x9AU" in app_control
    assert "app_control_report_coax_param_by_name(name);" in app_control
    assert "app_control_report_pid_legacy();" in app_control
    # The UI no longer flips any parameter's sign. It used to invert some gains
    # but not others, giving the UI and the control layer two different sign
    # conventions -- a display of -0.600 for an internally positive gain. Gains
    # are positive everywhere now, so this must stay a pass-through.
    assert "app_control_ui_sign_for_param" in app_control
    ui_sign = app_control.split("static float app_control_ui_sign_for_param", 1)[1]
    ui_sign = ui_sign.split("static float app_control_param_to_ui_value", 1)[0]
    assert "-1.0f" not in ui_sign
    for gain in ("coax.roll_rate_kd", "coax.pitch_rate_kd",
                 "coax.roll_angle_kp", "coax.pitch_angle_kp",
                 "coax.yaw_angle_kp", "coax.yaw_rate_kd"):
        assert f'strcmp(name, "{gain}") == 0' not in ui_sign
    assert "app_control_param_from_ui_value(name, value)" in app_control
    assert "app_control_param_to_ui_value(name, value)" in app_control
    assert "app_control_report_coax_param_by_name(map[map_index].param_name);" in app_control
    assert '"roll_angle_kp",  "coax.roll_angle_kp"' in app_control
    assert '"pitch_angle_kp", "coax.pitch_angle_kp"' in app_control
    assert '"roll_rate_kd",   "coax.roll_rate_kd"' in app_control
    assert '"yaw_angle_kp",   "coax.yaw_angle_kp"' in app_control
    assert '"Pitch_kp"' not in app_control
    assert '"Roll_kp"' not in app_control
    assert '"pos_x_kp",       "coax.pos_x_kp"' in app_control
    assert '"pos_y_kp",       "coax.pos_y_kp"' in app_control
    assert '"vel_x_kd",       "coax.vel_x_kd"' in app_control
    assert '"vel_y_kd",       "coax.vel_y_kd"' in app_control
    assert '"vel_loop_x_kp",  "coax.vel_loop_x_kp"' not in app_control


def test_synex_channels_exclude_executor_model_params() -> None:
    builder = read("tools/synex_config_builder.py")
    capture = read("tools/vofa_serial_capture.py")

    assert '"Pitch_kp"' not in builder
    assert '"Roll_kp"' not in builder
    assert '"Pos_X_KP"' in builder
    assert '"Pos_Y_KP"' in builder
    assert '"Vel_X_KD"' in builder
    assert '"Vel_Y_KD"' in builder
    assert '"Pos_X_m"' in builder
    assert '"Pos_Y_m"' in builder
    assert '"Vel_X_KI"' not in builder
    assert '"Vel_Y_KI"' not in builder
    assert '"Roll_Angle_KP"' in builder
    assert '"Pitch_Angle_KP"' in builder
    assert '"Pos_Z_KP"' in builder
    assert '"Pos_Z_KI"' in builder
    assert '"Vel_Z_KD"' in builder
    assert '"coax_pos_x_kp",' in capture
    assert '"coax_pos_y_kp",' in capture
    assert '"coax_vel_x_kd",' in capture
    assert '"coax_vel_y_kd",' in capture
    assert '"pos_x_m",' in capture
    assert '"pos_y_m",' in capture
    assert '"coax_roll_angle_kp",' in capture
    assert '"coax_pitch_angle_kp",' in capture
    assert '"coax_pos_z_kp",' in capture
    assert '"coax_pos_z_ki",' in capture
    assert '"coax_vel_z_kd",' in capture
    assert '"Single_Max_Thrust_N"' not in builder
    assert '"Yaw_Torque_Upper_MPN"' not in builder
    assert '"Yaw_Torque_Lower_MPN"' not in builder
    assert '"coax_motor_single_max_thrust_n",' not in capture
    assert '"coax_yaw_torque_upper_m_per_n",' not in capture
    assert '"coax_yaw_torque_lower_m_per_n",' not in capture
    assert '"coax_accel_xy_limit_m_s2"' not in capture
    assert '"vel_loop_output_limit_m_s2"' not in capture
