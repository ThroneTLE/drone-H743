from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_servo_output_compensates_90_degree_ccw_mounting() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")
    header = read("Driver/Inc/drv_coax_ctrl.h")

    assert "DRV_COAX_CTRL_ServoCalibration" in header
    assert "pulse_sign[DRV_COAX_CTRL_SERVO_COUNT]" in header
    assert "void DRV_COAX_CTRL_BodyTiltRadToServoPulses(float body_x_tilt_rad," in header
    assert "static void coax_ctrl_body_tilt_to_servo_tilts(float body_x_tilt_rad," in source
    assert "*servo_alpha_tilt_rad = -body_y_tilt_rad;" in source
    assert "*servo_beta_tilt_rad = -body_x_tilt_rad;" in source
    assert "DRV_COAX_CTRL_BodyTiltRadToServoPulses(output->alpha_rad," in source
    assert "output->beta_rad," in source
    assert "coax_ctrl_servo_calibration.pulse_sign[" in source
    assert "DRV_COAX_CTRL_SERVO_ALPHA_SIGN" not in source
    assert "DRV_COAX_CTRL_SERVO_BETA_SIGN" not in source


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
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

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
    # BodyTiltRadToServoPulses was only called from dead debug functions.
    assert "DRV_COAX_CTRL_BodyTiltRadToServoPulses(body_x_tilt_rad," not in freertos
    assert "DRV_COAX_CTRL_GetServoCalibration(&servo_calibration);" in freertos
    assert "servo_calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]" in freertos
    assert "servo_calibration.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX]" in freertos


def test_manual_and_ident_servo_limits_follow_each_calibrated_center() -> None:
    control = read("App/Src/app_control.c")
    ident = read("App/Src/app_ident.c")

    assert "app_control_servo_clamp_pulse(uint32_t index, uint16_t pulse_us)" in control
    assert "DRV_COAX_CTRL_GetServoCalibration(&calibration);" in control
    assert "calibration.min_us[index]" in control
    assert "calibration.max_us[index]" in control
    assert "app_control_servo_clamp_pulse(index," in control
    assert "DRV_COAX_CTRL_GetServoCalibration(&calibration);" in ident
    assert "calibration.min_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]" in ident
    assert "calibration.max_us[DRV_COAX_CTRL_SERVO_BETA_INDEX]" in ident
    assert "calibration.pulse_sign[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]" in ident
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
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")
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
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert "float vx_m_s;" in header
    assert "float ax_m_s2;" in header
    assert "float dt_sec;" in header
    assert "uint8_t horizontal_velocity_valid;" in header
    assert "range height above ground is exposed to the controller as z = -height" in header
    # R-F3 replaced the stale "already rotated to body FRD" claim with the frame
    # map the runtime actually delivers; see tests/test_flu_seam3_controller_frame.py.
    assert "canonical FLU body frame" in header
    assert "still the legacy (forward, right, down) local-level frame" in header
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
        "frame->reference.vx_m_s =\n"
        "        frame->rc.norm[APP_RC_FUNC_PITCH] *"
        in freertos
    )
    assert "static float stabilizer_rc_throttle_height_rate_m_s(float throttle_norm)" in freertos
    assert "return throttle_norm * STABILIZER_Z_REF_RATE_MAX_M_S;" in freertos
    assert "stabilizer_rc_throttle_thrust_bias_m_s2" not in freertos
    assert "STABILIZER_Z_THRUST_BIAS_MAX_M_S2" not in freertos
    assert "frame->reference.ax_m_s2 =" in freertos
    assert "frame->reference.az_m_s2 = 0.0f;" in freertos
    assert "ctx->height_ref_m +=\n          stabilizer_rc_throttle_height_rate_m_s(" in freertos
    assert "StabilizerVelocityPidState" not in freertos
    assert "stabilizer_velocity_pid_step" not in freertos
    assert "frame->reference.dt_sec = frame->ctrl_dt_sec;" in freertos
    assert "velocity_loop_enabled = (vel_loop_enable >= 0.5f) ? 1U : 0U;" in freertos
    assert "frame->reference.horizontal_velocity_valid = velocity_loop_enabled;" in freertos
    # R-M5-5：位置积分搬进 svc_flow_nav.c（按传感器时间轴变步长），稳定环只取成品。
    # 保留同一条约束的等价形式：位置状态确实进了控制器的 attitude 输入。
    assert "SVC_FlowNav_GetPosition(&position_state_x_m, &position_state_y_m);" in freertos
    assert "frame->attitude.x_m = position_state_x_m;" in freertos
    assert "frame->attitude.y_m = position_state_y_m;" in freertos
    assert "ctx->position_ref_x_m += frame->reference.vx_m_s * frame->ctrl_dt_sec;" in freertos
    assert "frame->reference.x_m = ctx->position_ref_x_m;" in freertos
    assert "frame->reference.y_m = ctx->position_ref_y_m;" in freertos
    assert "#define STABILIZER_VELOCITY_MEAS_Y_SIGN (1.0f)" in freertos
    assert "stabilizer_velocity_estimator_control_ok(&vel_estimator, now)" not in freertos
    assert "frame->attitude.vx_m_s = velocity_control_x_m_s;" in freertos
    assert "frame->attitude.vy_m_s = velocity_control_y_m_s;" in freertos
    assert "frame->relative_height_m = frame->range_height_m - ctx->height_origin_m;" in freertos
    assert "frame->attitude.z_m = -frame->relative_height_m;" in freertos
    assert "frame->attitude.vz_m_s = -frame->range_velocity_m_s;" in freertos
    assert "stabilizer_clamp_f32(ctx->position_ref_z_m," in freertos
    assert "STABILIZER_Z_POS_ERR_MAX_M" in freertos
    assert "frame->reference.vz_m_s = 0.0f;" in freertos
    assert "frame->reference.yaw_rate_rad_s = yaw_rate_ref_rad_s;" in freertos
    assert "frame->reference.yaw_accel_rad_s2 = 0.0f;" in freertos
    assert "velocity_state_x_m_s : 0.0f;" not in freertos
    assert "velocity_ref_x_m_s" not in freertos
    assert "memset(&frame->reference, 0, sizeof(frame->reference));" in freertos


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
    assert "DRV_COAX_CTRL_ResetState();" in read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")


def test_vofa_exports_compact_slider_parameter_feedback() -> None:
    # R-T1-1：通道装配从 Core/Src/freertos.c 搬到 App/Src/app_telem_port.c，
    # 参数回显保持原口径；坐标通道由 2026-09-03 作者授权统一为 FLU。
    freertos = read("App/Src/app_telem_port.c")

    # 帧长仍由通道表推出来：采样函数拒绝任何与表长不符的 count。
    assert "(values == NULL) || (count != (uint32_t)APP_TELEM_CH_COUNT)" in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.roll_rate_kd", &vofa_data[APP_TELEM_CH_ROLL_RATE_KD]);' in freertos
    assert "vofa_data[APP_TELEM_CH_ROLL_RATE_KD] = -vofa_data[APP_TELEM_CH_ROLL_RATE_KD];" not in freertos
    assert "vofa_data[APP_TELEM_CH_PITCH_RATE_KD] = -vofa_data[APP_TELEM_CH_PITCH_RATE_KD];" not in freertos
    # 审核实机复核（2026-09-03）：app_control_ui_sign_for_param 对所有参数返回 +1，
    # `PARAM?` 报正值而遥测取反后报负值，两条路径口径不一致，滑块钳死在 0。
    # 遥测回显不许再做任何符号处理。
    assert "vofa_data[APP_TELEM_CH_YAW_ANGLE_KP] = -vofa_data[APP_TELEM_CH_YAW_ANGLE_KP];" not in freertos
    assert "vofa_data[APP_TELEM_CH_YAW_RATE_KD] = -vofa_data[APP_TELEM_CH_YAW_RATE_KD];" not in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pos_x_kp", &vofa_data[APP_TELEM_CH_POS_X_KP]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pos_y_kp", &vofa_data[APP_TELEM_CH_POS_Y_KP]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_x_kd", &vofa_data[APP_TELEM_CH_VEL_X_KD]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_y_kd", &vofa_data[APP_TELEM_CH_VEL_Y_KD]);' in freertos
    assert "vofa_data[APP_TELEM_CH_POS_EST_X] = position_flu.x;" in freertos
    assert "vofa_data[APP_TELEM_CH_POS_EST_Y] = position_flu.y;" in freertos
    assert "APP_TELEM_CH_POS_EST_Y] = vofa_debug.pos_est_m[1]" not in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_loop_enable", &vofa_data[APP_TELEM_CH_VEL_LOOP_ENABLE]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.roll_angle_kp", &vofa_data[APP_TELEM_CH_ROLL_ANGLE_KP]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pitch_angle_kp", &vofa_data[APP_TELEM_CH_PITCH_ANGLE_KP]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pos_z_kp", &vofa_data[APP_TELEM_CH_POS_Z_KP]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pos_z_ki", &vofa_data[APP_TELEM_CH_POS_Z_KI]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_z_kd", &vofa_data[APP_TELEM_CH_VEL_Z_KD]);' in freertos
    assert "vofa_data[APP_TELEM_CH_ROLL_ANGLE_KP] = -vofa_data[APP_TELEM_CH_ROLL_ANGLE_KP];" not in freertos
    assert "vofa_data[APP_TELEM_CH_PITCH_ANGLE_KP] = -vofa_data[APP_TELEM_CH_PITCH_ANGLE_KP];" not in freertos
    assert "= -vofa_data[" not in freertos
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


def _function_body(source: str, name: str) -> str:
    start = source.index(name)
    open_brace = source.index("{", start)
    depth = 0
    for position in range(open_brace, len(source)):
        if source[position] == "{":
            depth += 1
        elif source[position] == "}":
            depth -= 1
            if depth == 0:
                return source[open_brace : position + 1]
    raise AssertionError(f"unbalanced braces after {name}")


def test_servo_calibration_criteria_stay_in_lockstep_across_layers() -> None:
    """app_flight_calibration.c 因 host 单测无法链接整个驱动而保留判据副本；
    本用例强制副本与 Driver 真源逐条一致，单方面改动即失败。"""
    driver = read("Driver/Src/drv_coax_ctrl.c")
    app = read("App/Src/app_flight_calibration.c")

    driver_valid = _function_body(driver, "uint8_t DRV_COAX_CTRL_ValidateServoCalibration(")
    app_valid = _function_body(app, "static uint8_t app_flight_cal_servo_valid(")
    normalize = lambda text: "".join(text.split())
    loop_marker = "for(index=0U;index<DRV_COAX_CTRL_SERVO_COUNT;++index){"
    driver_loop = normalize(driver_valid)
    app_loop = normalize(app_valid)
    assert loop_marker in driver_loop and loop_marker in app_loop
    assert driver_loop[driver_loop.index(loop_marker):] == app_loop[app_loop.index(loop_marker):], (
        "servo calibration validity criteria diverged between drv_coax_ctrl.c "
        "and app_flight_calibration.c; update both sides together"
    )

    driver_defaults = _function_body(driver, "void DRV_COAX_CTRL_GetDefaultServoCalibration(")
    app_defaults = _function_body(app, "static void app_flight_cal_servo_defaults(")
    for macro in (
        "DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US",
        "DRV_COAX_CTRL_SERVO_BETA_CENTER_US",
        "DRV_COAX_CTRL_SERVO_ALPHA_MIN_US",
        "DRV_COAX_CTRL_SERVO_BETA_MIN_US",
        "DRV_COAX_CTRL_SERVO_ALPHA_MAX_US",
        "DRV_COAX_CTRL_SERVO_BETA_MAX_US",
    ):
        assert macro in driver_defaults and macro in app_defaults
    assert normalize(driver_defaults).count("pulse_sign[") == 2
    assert normalize(app_defaults).count("pulse_sign[") == 2
