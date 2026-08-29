from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_freertos_documents_fixed_elrs_channel_map() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert "ELRS / CRSF 遥控器通道约定" in freertos
    assert "CH1 → 左右 / roll stick" in freertos
    assert "CH2 → 前后 / pitch stick" in freertos
    assert "CH3 → 左摇杆上下 / throttle" in freertos
    assert "稳定段设定激光定高目标，50% 保持当前高度，高于 50% 提高目标高度" in freertos
    assert "CH4 → 偏航 / yaw stick" in freertos
    assert "中位保持，高于中位累加 yaw_ref，低于中位减少 yaw_ref" in freertos
    assert "CH5 → 二值开关 / arm switch" in freertos
    assert "+100=开锁，-100=关锁" in freertos
    assert "CH6 → 姿态调试模式开关" in freertos
    assert "手动总推力 + 目标姿态" in freertos


def test_controller_uses_named_rc_channels_for_references() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    # 通道号和端点已经搬进可标定的 APP_RcConfig；出厂默认仍是 CH1..CH6 / 1000-1500-2000，
    # 由 tests/test_rc_mapping.py 逐条钉住。这里只钉"控制环按功能名取值"这件事。
    assert "STABILIZER_RC_CH_" not in freertos
    assert "frame->rc.norm[APP_RC_FUNC_PITCH]" in freertos
    assert "frame->rc.norm[APP_RC_FUNC_ROLL]" in freertos
    assert "frame->rc.norm[APP_RC_FUNC_YAW]" in freertos
    assert "frame->rc.norm[APP_RC_FUNC_THROTTLE]" in freertos
    # 阈值由绝对 us 换成各通道自身行程的百分比，语义等价：
    # 1500us@1000..2000 = 50%，1100us = 10%，1700us = 70%。
    assert "#define STABILIZER_RC_SWITCH_HIGH_PERCENT    50U" in freertos
    assert "#define STABILIZER_RC_THROTTLE_ARM_LOW_PERCENT 10U" in freertos
    assert "#define STABILIZER_RC_LOSS_TIMEOUT_MS  500U" in freertos
    assert "#define STABILIZER_XY_VEL_REF_MAX_M_S  0.40f" in freertos
    assert "#define STABILIZER_Z_REF_RATE_MAX_M_S  0.30f" in freertos
    assert "#define STABILIZER_Z_REF_MAX_M         0.40f" in freertos
    assert "#define STABILIZER_XY_POS_ERR_MAX_M    0.50f" in freertos
    assert "#define STABILIZER_Z_POS_ERR_MAX_M     0.35f" in freertos
    assert "STABILIZER_Z_THRUST_BIAS_MAX_M_S2" not in freertos
    assert "#define STABILIZER_YAW_RATE_REF_MAX_RAD_S 1.04719758f" in freertos
    assert (
        "frame->reference.vx_m_s =\n"
        "        frame->rc.norm[APP_RC_FUNC_PITCH] *"
        in freertos
    )
    assert (
        "frame->reference.vy_m_s =\n"
        "        frame->rc.norm[APP_RC_FUNC_ROLL] *"
        in freertos
    )
    assert "ctx->position_ref_x_m += frame->reference.vx_m_s * frame->ctrl_dt_sec;" in freertos
    assert "ctx->position_ref_y_m += frame->reference.vy_m_s * frame->ctrl_dt_sec;" in freertos
    assert "frame->reference.x_m = ctx->position_ref_x_m;" in freertos
    assert "frame->reference.y_m = ctx->position_ref_y_m;" in freertos
    assert "frame->reference.dt_sec = frame->ctrl_dt_sec;" in freertos
    assert "frame->reference.horizontal_velocity_valid = velocity_loop_enabled;" in freertos
    assert "stabilizer_clamp_f32(ctx->position_ref_z_m," in freertos
    assert "frame->attitude.z_m - STABILIZER_Z_POS_ERR_MAX_M" in freertos
    assert "frame->attitude.z_m + STABILIZER_Z_POS_ERR_MAX_M" in freertos
    assert "float height_origin_m;" in freertos
    assert "uint8_t height_origin_ready;" in freertos
    assert "frame->relative_height_m = frame->range_height_m - ctx->height_origin_m;" in freertos
    assert "frame->attitude.z_m = -frame->relative_height_m;" in freertos
    assert "ctx->height_ref_m = frame->relative_height_m;" in freertos
    assert "ctx->height_ref_m +=\n          stabilizer_rc_throttle_height_rate_m_s(" in freertos
    assert "ctx->position_ref_z_m = -ctx->height_ref_m;" in freertos
    assert "frame->reference.az_m_s2 = 0.0f;" in freertos
    assert "frame->reference.yaw_rad = ctx->yaw_ref_rad;" in freertos
    assert "frame->reference.yaw_rate_rad_s = yaw_rate_ref_rad_s;" in freertos
    assert "frame->reference.yaw_accel_rad_s2 = 0.0f;" in freertos
    assert "STABILIZER_YAW_REF_LIMIT_RAD" not in freertos


def test_ch6_selects_true_attitude_debug_mode_with_twenty_degree_limit() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert "#define STABILIZER_USE_RC_DIRECT_TILT_SERVO 0U" in freertos
    # RC_DIRECT_TILT_LIMIT_RAD removed — only used by deleted dead code.
    assert "STABILIZER_USE_RC_ATTITUDE_TARGET_TEST" not in freertos
    assert "#define STABILIZER_RC_ATTITUDE_TARGET_LIMIT_RAD 0.349065850f" in freertos
    assert "frame->rc_attitude_debug_mode =" in freertos
    assert "(frame->rc_use_stabilized_motor_mix != 0U) &&" in freertos
    assert "frame->rc.us[APP_RC_FUNC_MODE],\n" \
           "        STABILIZER_RC_SWITCH_HIGH_PERCENT" in freertos
    assert "if (frame->rc_attitude_debug_mode != 0U)" in freertos
    assert "frame->reference.direct_attitude_target_valid = 1U;" in freertos
    assert "frame->reference.manual_total_force_valid = 1U;" in freertos
    assert "frame->reference.manual_total_force_n =" in freertos
    assert "DRV_COAX_CTRL_MotorPulseToTotalThrust(frame->rc_throttle_motor_us);" in freertos
    assert "frame->reference.target_pitch_rad =" in freertos
    assert "frame->reference.target_roll_rad =" in freertos
    assert "frame->reference.horizontal_velocity_valid = 0U;" in freertos
    assert "frame->reference.z_m = frame->attitude.z_m;" in freertos
    assert "frame->reference.vz_m_s = frame->attitude.vz_m_s;" in freertos
    assert "ctx->position_ref_z_ready = 0U;" in freertos
    assert "frame->rc_control_motor_mix_allowed =" in freertos
    assert "(frame->rc_use_stabilized_motor_mix != 0U) ? 1U : 0U;" in freertos
    assert "rc_attitude_debug_mode != 0U)) ? 1U : 0U;" not in freertos
    assert "} else if (frame->rc_control_motor_mix_allowed == 0U) {" in freertos
    # RC-direct-tilt debug function removed (permanently dead code).
    # The coax controller path is the only compiled servo path.
    assert "static void stabilizer_map_rc_direct_to_servo" not in freertos
    assert "stabilizer_map_rc_direct_to_servo(ch, moves);" not in freertos
    assert "#if (STABILIZER_USE_RC_DIRECT_TILT_SERVO" not in freertos
    # DIRECT_BODY_* removed — only used by deleted dead debug functions.
    assert "DRV_COAX_CTRL_BodyTiltRadToServoPulses(body_x_tilt_rad," not in freertos


def test_arm_switch_gates_motor_output_but_not_controller_reference() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")
    header = read("Driver/Inc/drv_coax_ctrl.h")
    source = read("Driver/Src/drv_coax_ctrl.c")

    assert "static uint8_t stabilizer_rc_arm_latched = 0U;" in freertos
    assert "static uint8_t stabilizer_rc_switch_seen_low = 0U;" in freertos
    assert "static uint8_t stabilizer_rc_switch_prev_high = 0U;" in freertos
    assert "static uint8_t stabilizer_rc_update_armed(uint8_t switch_high," in freertos
    assert "frame->rc_link_ok = APP_ELRS_IsRcFresh(frame->now_ms, STABILIZER_RC_LOSS_TIMEOUT_MS);" in freertos
    assert "frame->rc_link_seen = (APP_ELRS_GetLastRcMs() != 0U) ? 1U : 0U;" in freertos
    assert "frame->rc_armed = stabilizer_rc_update_armed(frame->rc_arm_switch_high," in freertos
    assert "frame->rc_throttle_motor_us =" in freertos
    assert "stabilizer_rc_throttle_to_motor_pulse(frame->rc.throttle_01);" in freertos
    assert "frame->rc_use_stabilized_motor_mix =" in freertos
    assert "stabilizer_rc_use_stabilized_motor_mix(frame->rc.throttle_01);" in freertos
    assert "(frame->rc.throttle_01 <=\n" \
           "     ((float)STABILIZER_RC_THROTTLE_ARM_LOW_PERCENT / 100.0f)) ? 1U : 0U;" in freertos
    assert "stabilizer_rc_switch_prev_high == 0U" in freertos
    assert "} else if ((frame->rc_link_ok != 0U) && (frame->rc_armed != 0U)) {" in freertos
    assert "if ((frame->rc_control_motor_mix_allowed != 0U) &&" in freertos
    assert "(frame->imu_control_valid != 0U))" in freertos
    assert "BSP_PWM_SetEscPulse(1, frame->ctrl_out.motor_upper_us);" in freertos
    assert "BSP_PWM_SetEscPulse(2, frame->ctrl_out.motor_lower_us);" in freertos
    assert "APP_FLIGHT_LOG_MOTOR_REASON_ATTITUDE_DEBUG" in freertos
    assert "uint16_t motor_upper_us;" in header
    assert "uint16_t motor_lower_us;" in header
    assert "uint16_t DRV_COAX_CTRL_ThrustToMotorPulse(float thrust_n);" in header
    assert "BSP_PWM_SetEscPulse(1, frame->rc_throttle_motor_us);" in freertos
    assert "BSP_PWM_SetEscPulse(2, frame->rc_throttle_motor_us);" in freertos
    assert "BSP_PWM_SetEscPulse(1, BSP_PWM_ESC_MIN_US);" in freertos
    assert "BSP_PWM_SetEscPulse(2, BSP_PWM_ESC_MIN_US);" in freertos
    assert "BSP_PWM_DisableEsc(1);" in freertos
    assert "BSP_PWM_DisableEsc(2);" in freertos
    assert "coax_ctrl_tilt_rad_to_servo_pulse" in source
    assert "coax_tiltrotor_controller_codegen(" not in source


def test_disarmed_pwm_disables_output_without_changing_throttle_limits() -> None:
    header = read("BSP/Inc/bsp_pwm.h")
    source = read("BSP/Src/bsp_pwm.c")
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")
    coax = read("Driver/Src/drv_coax_ctrl.c")
    motor = read("Driver/Src/drv_motor.c")
    control = read("App/Src/app_control.c")

    assert "#define BSP_PWM_ESC_MIN_US      1100U" in header
    assert "#define BSP_PWM_ESC_MAX_US      1940U" in header
    assert "#define BSP_PWM_ESC_NEUTRAL_US  1100U" in header
    assert "#define BSP_PWM_ESC_CHANNEL_COUNT 2U" in header
    assert "#define BSP_PWM_SERVO_CHANNEL_COUNT 2U" in header
    assert "BSP_PWM_Status BSP_PWM_DisableEsc(uint32_t channel);" in header
    assert "pulse_us < BSP_PWM_ESC_MIN_US" in source
    assert "BSP_PWM_Status BSP_PWM_DisableEsc(uint32_t channel)" in source
    assert "__HAL_TIM_SET_COMPARE(&htim2, tim_channel, 0U);" in source
    assert "if (percent == 0U) {\n        return BSP_PWM_ESC_STOP_US;\n    }" not in source
    assert "BSP_PWM_DisableEsc(1);" in freertos
    assert "BSP_PWM_DisableEsc(2);" in freertos
    assert "DRV_COAX_CTRL_ThrustToMotorPulse(float thrust_n)" in coax
    assert "coax_ctrl_dual_thrust_g" in coax
    assert "pwm_status = BSP_PWM_DisableEsc(pwm_channel);" in motor
    assert "status = (percent == 0U) ? DRV_Motor_Stop(motor) : DRV_Motor_SetPercent(motor, percent);" in control
    assert "BSP_PWM_DisableEsc(channel) :" in control
    assert "uint16_t pulse = BSP_PWM_GetEscPulse(channel);" in control


def test_ch3_is_rc_intent_with_direct_throttle_below_20_percent() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert "#define STABILIZER_RC_STABILIZE_MIN_PERCENT 70U" in freertos
    # 油门归一化搬进 APP_RcConfig_Throttle01（按标定端点），控制环只消费结果。
    assert "float APP_RcConfig_Throttle01(" in read("App/Src/app_rc_config.c")
    assert "static float stabilizer_rc_throttle_height_rate_m_s(float throttle_norm)" in freertos
    assert "stabilizer_rc_throttle_thrust_bias_m_s2" not in freertos
    rc_config = read("App/Src/app_rc_config.c")
    assert "span = (int32_t)map->max_us - (int32_t)map->min_us;" in rc_config
    assert "value = (int32_t)channel_us - (int32_t)map->min_us;" in rc_config
    assert "stabilizer_clamp_f32(ctx->position_ref_z_m," in freertos
    assert "stabilizer_rc_throttle_height_rate_m_s(" in freertos
    assert "STABILIZER_Z_REF_RATE_MAX_M_S" in freertos
    assert "STABILIZER_Z_REF_STICK_SPAN_M" not in freertos
    assert "STABILIZER_Z_THRUST_BIAS_MAX_M_S2" not in freertos
    assert "static uint16_t stabilizer_rc_throttle_to_motor_pulse(float throttle_01)" in freertos
    assert "static uint8_t stabilizer_rc_use_stabilized_motor_mix(float throttle_01)" in freertos
    assert "return (throttle_01 >=\n" \
           "            ((float)STABILIZER_RC_STABILIZE_MIN_PERCENT / 100.0f)) ? 1U : 0U;" in freertos
    assert "} else if (frame->rc_control_motor_mix_allowed == 0U) {" in freertos
    assert "stabilizer_mix_rc_base_with_ctrl" not in freertos
    assert "ctrl_us - (int32_t)ctrl_avg_us" not in freertos


def test_yaw_stick_integrates_reference_and_wraps_at_pi_boundary() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")
    wrapper = read("Driver/Src/drv_coax_ctrl.c")

    assert "static float stabilizer_wrap_pi(float angle_rad)" in freertos
    assert "while (angle_rad > STABILIZER_PI)" in freertos
    assert "while (angle_rad < -STABILIZER_PI)" in freertos
    assert "static float stabilizer_rc_yaw_rate_rad_s(float yaw_norm)" in freertos
    assert "return yaw_norm * STABILIZER_YAW_RATE_REF_MAX_RAD_S;" in freertos
    assert "float yaw_ref_rad;" in freertos
    assert "uint8_t yaw_ref_ready;" in freertos
    assert freertos.count("yaw_ref_ready = 0U;") >= 2
    assert "ctx->yaw_ref_rad = frame->attitude.yaw_rad;" in freertos
    assert "ctx->yaw_ref_rad =\n        stabilizer_wrap_pi(ctx->yaw_ref_rad +" in freertos
    assert "yaw_rate_ref_rad_s * frame->ctrl_dt_sec" in freertos
    assert "frame->reference.yaw_rad = ctx->yaw_ref_rad;" in freertos
    assert "frame->reference.yaw_rate_rad_s = yaw_rate_ref_rad_s;" in freertos
    assert "yaw_ref_ready = 0U;" in freertos
    assert "const float yaw_err = coax_ctrl_wrap_pi(reference->yaw_rad - attitude->yaw_rad);" in wrapper


def test_elrs_link_freshness_uses_valid_rc_frames_only() -> None:
    app_source = read("App/Src/app_elrs.c")
    app_header = read("App/Inc/app_elrs.h")
    drv_source = read("Driver/Src/drv_elrs.c")
    drv_header = read("Driver/Inc/drv_elrs.h")

    assert "DRV_ELRS_MarkRcFrameTime(HAL_GetTick());" in app_source
    assert "uint8_t APP_ELRS_IsRcFresh(uint32_t now_ms, uint32_t timeout_ms);" in app_header
    assert "void     DRV_ELRS_MarkRcFrameTime(uint32_t now_ms);" in drv_header
    assert "uint8_t  DRV_ELRS_IsRcFresh(uint32_t now_ms, uint32_t timeout_ms);" in drv_header
    assert "static uint8_t Crsf_HandleRcChannels(const uint8_t *payload, uint8_t payload_len)" in drv_source
    assert "static uint8_t Crsf_HandleFrame(const uint8_t *frame, uint8_t total_len)" in drv_source
    assert "return Crsf_HandleRcChannels(payload, payload_len);" in drv_source
    assert "g_last_rc_tick = now_ms;" in drv_source
    assert "return ((now_ms - g_last_rc_tick) <= timeout_ms) ? 1U : 0U;" in drv_source


def test_raw_motor_commands_are_disabled_for_prop_safety() -> None:
    source = read("App/Src/app_control.c")

    assert "#define APP_CONTROL_ALLOW_RAW_PWM_COMMANDS 0U" in source
    assert "#define APP_CONTROL_ALLOW_RAW_MOTOR_COMMANDS 0U" in source
    assert "#define APP_CONTROL_ALLOW_IDENT_MOTOR_TEST 0U" in source
    assert "ERR raw pwm disabled; use ARM switch" in source
    assert "ERR raw motor disabled; use ARM switch" in source
    assert "ERR ident disabled for prop safety" in source
