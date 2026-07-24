from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_freertos_documents_fixed_elrs_channel_map() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "ELRS / CRSF 遥控器通道约定" in freertos
    assert "CH1 → 左右 / roll stick" in freertos
    assert "CH2 → 前后 / pitch stick" in freertos
    assert "CH3 → 左摇杆上下 / throttle" in freertos
    assert "稳定段设定激光定高目标，50% 保持当前高度，高于 50% 提高目标高度" in freertos
    assert "CH4 → 偏航 / yaw stick" in freertos
    assert "中位保持，高于中位累加 yaw_ref，低于中位减少 yaw_ref" in freertos
    assert "CH5 → 二值开关 / arm switch" in freertos
    assert "+100=开锁，-100=关锁" in freertos


def test_controller_uses_named_rc_channels_for_references() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define STABILIZER_RC_CH_ROLL          0U" in freertos
    assert "#define STABILIZER_RC_CH_PITCH         1U" in freertos
    assert "#define STABILIZER_RC_CH_THROTTLE_Z    2U" in freertos
    assert "#define STABILIZER_RC_CH_YAW           3U" in freertos
    assert "#define STABILIZER_RC_CH_ARM           4U" in freertos
    assert "#define STABILIZER_RC_ARM_THRESHOLD_US 1500U" in freertos
    assert "#define STABILIZER_RC_THROTTLE_INPUT_LOW_US  1000U" in freertos
    assert "#define STABILIZER_RC_THROTTLE_INPUT_HIGH_US 2000U" in freertos
    assert "#define STABILIZER_RC_THROTTLE_ARM_LOW_US    1100U" in freertos
    assert "#define STABILIZER_RC_LOSS_TIMEOUT_MS  500U" in freertos
    assert "#define STABILIZER_XY_VEL_REF_MAX_M_S  0.80f" in freertos
    assert "#define STABILIZER_Z_REF_RATE_MAX_M_S  0.30f" in freertos
    assert "#define STABILIZER_Z_REF_MAX_M         0.30f" in freertos
    assert "#define STABILIZER_Z_POS_ERR_MAX_M     0.35f" in freertos
    assert "STABILIZER_Z_THRUST_BIAS_MAX_M_S2" not in freertos
    assert "#define STABILIZER_YAW_RATE_REF_MAX_RAD_S 1.04719758f" in freertos
    assert "reference.vx_m_s = stabilizer_rc_normalized(ch[STABILIZER_RC_CH_PITCH])" in freertos
    assert "reference.vy_m_s = stabilizer_rc_normalized(ch[STABILIZER_RC_CH_ROLL])" in freertos
    assert "reference.x_m = attitude.x_m;" in freertos
    assert "reference.y_m = attitude.y_m;" in freertos
    assert "reference.dt_sec = ctrl_dt_sec;" in freertos
    assert "reference.horizontal_velocity_valid = velocity_control_ok;" in freertos
    assert "stabilizer_clamp_f32(position_ref_z_m," in freertos
    assert "attitude.z_m - STABILIZER_Z_POS_ERR_MAX_M" in freertos
    assert "attitude.z_m + STABILIZER_Z_POS_ERR_MAX_M" in freertos
    assert "float height_origin_m = 0.0f;" in freertos
    assert "uint8_t height_origin_ready = 0U;" in freertos
    assert "relative_height_m = range_height_m - height_origin_m;" in freertos
    assert "attitude.z_m = -relative_height_m;" in freertos
    assert "height_ref_m = relative_height_m;" in freertos
    assert "height_ref_m +=\n                stabilizer_rc_throttle_height_rate_m_s(" in freertos
    assert "position_ref_z_m = -height_ref_m;" in freertos
    assert "reference.az_m_s2 = 0.0f;" in freertos
    assert "reference.yaw_rad = yaw_ref_rad;" in freertos
    assert "reference.yaw_rate_rad_s = yaw_rate_ref_rad_s;" in freertos
    assert "reference.yaw_accel_rad_s2 = 0.0f;" in freertos
    assert "STABILIZER_YAW_REF_LIMIT_RAD" not in freertos


def test_controller_mode_keeps_rc_direct_tilt_as_explicit_debug_switch() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define STABILIZER_USE_RC_DIRECT_TILT_SERVO 0U" in freertos
    assert "#define STABILIZER_RC_DIRECT_TILT_LIMIT_RAD 0.314159265f" in freertos
    assert "static void stabilizer_map_rc_direct_to_servo" in freertos
    assert "#if (STABILIZER_USE_RC_DIRECT_TILT_SERVO != 0U)\nstatic void stabilizer_map_rc_direct_to_servo" in freertos
    assert "#define STABILIZER_DIRECT_BODY_X_SIGN   (1.0f)" in freertos
    assert "#define STABILIZER_DIRECT_BODY_Y_SIGN   (1.0f)" in freertos
    assert "stabilizer_rc_normalized(ch[STABILIZER_RC_CH_PITCH]) *" in freertos
    assert "stabilizer_rc_normalized(ch[STABILIZER_RC_CH_ROLL]) *" in freertos
    assert "DRV_COAX_CTRL_BodyTiltRadToServoPulses(body_x_tilt_rad," in freertos
    assert "body_y_tilt_rad," in freertos
    assert "&moves[0].pulse_us," in freertos
    assert "&moves[1].pulse_us);" in freertos
    assert "stabilizer_map_rc_direct_to_servo(ch, moves);" in freertos
    assert "未接位置/速度估计" in freertos


def test_arm_switch_gates_motor_output_but_not_controller_reference() -> None:
    freertos = read("Core/Src/freertos.c")
    header = read("Driver/Inc/drv_coax_ctrl.h")
    source = read("Driver/Src/drv_coax_ctrl.c")

    assert "static uint8_t stabilizer_rc_arm_latched = 0U;" in freertos
    assert "static uint8_t stabilizer_rc_switch_seen_low = 0U;" in freertos
    assert "static uint8_t stabilizer_rc_switch_prev_high = 0U;" in freertos
    assert "static uint8_t stabilizer_rc_update_armed(const uint16_t ch[CRSF_CHANNEL_COUNT]," in freertos
    assert "rc_link_ok = APP_ELRS_IsRcFresh(now, STABILIZER_RC_LOSS_TIMEOUT_MS);" in freertos
    assert "rc_link_seen = (APP_ELRS_GetLastRcMs() != 0U) ? 1U : 0U;" in freertos
    assert "rc_armed = stabilizer_rc_update_armed(ch, rc_link_ok);" in freertos
    assert "rc_throttle_motor_us =" in freertos
    assert "stabilizer_rc_throttle_to_motor_pulse(ch[STABILIZER_RC_CH_THROTTLE_Z]);" in freertos
    assert "rc_use_stabilized_motor_mix =" in freertos
    assert "stabilizer_rc_use_stabilized_motor_mix(ch[STABILIZER_RC_CH_THROTTLE_Z]);" in freertos
    assert "ch[STABILIZER_RC_CH_THROTTLE_Z] <= STABILIZER_RC_THROTTLE_ARM_LOW_US" in freertos
    assert "stabilizer_rc_switch_prev_high == 0U" in freertos
    assert "if ((rc_link_ok != 0U) && (rc_armed != 0U))" in freertos
    assert "if ((rc_use_stabilized_motor_mix != 0U) &&" in freertos
    assert "(imu_control_valid != 0U))" in freertos
    assert "BSP_PWM_SetEscPulse(1, ctrl_out.motor_upper_us);" in freertos
    assert "BSP_PWM_SetEscPulse(2, ctrl_out.motor_lower_us);" in freertos
    assert "uint16_t motor_upper_us;" in header
    assert "uint16_t motor_lower_us;" in header
    assert "uint16_t DRV_COAX_CTRL_ThrustToMotorPulse(float thrust_n);" in header
    assert "BSP_PWM_SetEscPulse(1, rc_throttle_motor_us);" in freertos
    assert "BSP_PWM_SetEscPulse(2, rc_throttle_motor_us);" in freertos
    assert "BSP_PWM_SetEscPulse(1, BSP_PWM_ESC_MIN_US);" in freertos
    assert "BSP_PWM_SetEscPulse(2, BSP_PWM_ESC_MIN_US);" in freertos
    assert "BSP_PWM_DisableEsc(1);" in freertos
    assert "BSP_PWM_DisableEsc(2);" in freertos
    assert "coax_ctrl_tilt_rad_to_servo_pulse" in source
    assert "coax_tiltrotor_controller_codegen(" not in source


def test_disarmed_pwm_disables_output_without_changing_throttle_limits() -> None:
    header = read("BSP/Inc/bsp_pwm.h")
    source = read("BSP/Src/bsp_pwm.c")
    freertos = read("Core/Src/freertos.c")
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
    freertos = read("Core/Src/freertos.c")

    assert "#define STABILIZER_RC_STABILIZE_MIN_PERCENT 20U" in freertos
    assert "#define STABILIZER_RC_STABILIZE_MIN_US \\" in freertos
    assert "static float stabilizer_rc_throttle_01(uint16_t ch_us)" in freertos
    assert "static float stabilizer_rc_throttle_height_rate_m_s(uint16_t ch_us)" in freertos
    assert "stabilizer_rc_throttle_thrust_bias_m_s2" not in freertos
    assert "(int32_t)STABILIZER_RC_THROTTLE_INPUT_HIGH_US -" in freertos
    assert "(int32_t)STABILIZER_RC_THROTTLE_INPUT_LOW_US" in freertos
    assert "int32_t value = (int32_t)ch_us - (int32_t)STABILIZER_RC_THROTTLE_INPUT_LOW_US;" in freertos
    assert "return (float)value / (float)span;" in freertos
    assert "stabilizer_clamp_f32(position_ref_z_m," in freertos
    assert "stabilizer_rc_throttle_height_rate_m_s(" in freertos
    assert "STABILIZER_Z_REF_RATE_MAX_M_S" in freertos
    assert "STABILIZER_Z_REF_STICK_SPAN_M" not in freertos
    assert "STABILIZER_Z_THRUST_BIAS_MAX_M_S2" not in freertos
    assert "static uint16_t stabilizer_rc_throttle_to_motor_pulse(uint16_t ch_us)" in freertos
    assert "static uint8_t stabilizer_rc_use_stabilized_motor_mix(uint16_t ch_us)" in freertos
    assert "return (ch_us >= STABILIZER_RC_STABILIZE_MIN_US) ? 1U : 0U;" in freertos
    assert "stabilizer_mix_rc_base_with_ctrl" not in freertos
    assert "ctrl_us - (int32_t)ctrl_avg_us" not in freertos


def test_yaw_stick_integrates_reference_and_wraps_at_pi_boundary() -> None:
    freertos = read("Core/Src/freertos.c")
    wrapper = read("Driver/Src/drv_coax_ctrl.c")

    assert "static float stabilizer_wrap_pi(float angle_rad)" in freertos
    assert "while (angle_rad > STABILIZER_PI)" in freertos
    assert "while (angle_rad < -STABILIZER_PI)" in freertos
    assert "static float stabilizer_rc_yaw_rate_rad_s(uint16_t ch_us)" in freertos
    assert "return stabilizer_rc_normalized(ch_us) * STABILIZER_YAW_RATE_REF_MAX_RAD_S;" in freertos
    assert "float yaw_ref_rad = 0.0f;" in freertos
    assert "uint8_t yaw_ref_ready = 0U;" in freertos
    assert freertos.count("yaw_ref_ready = 0U;") >= 3
    assert "yaw_ref_rad = attitude.yaw_rad;" in freertos
    assert "yaw_ref_rad =\n              stabilizer_wrap_pi(yaw_ref_rad +" in freertos
    assert "yaw_rate_ref_rad_s * ctrl_dt_sec" in freertos
    assert "reference.yaw_rad = yaw_ref_rad;" in freertos
    assert "reference.yaw_rate_rad_s = yaw_rate_ref_rad_s;" in freertos
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
