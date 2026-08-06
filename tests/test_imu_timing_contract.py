from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_stabilizer_uses_drdy_timed_fusion_ahrs() -> None:
    header = read("App/Inc/app_sensor.h")
    source = read("App/Src/app_sensor.c")
    freertos = read("Core/Src/freertos.c")

    assert "APP_IMU_ReadDataReadyTimestamp" in header
    assert "app_imu_drdy_timestamp.sequence" in source
    assert "SVC_Timestamp_Us();" in source
    assert "osKernelGetTickCount()" not in source
    assert "#define STABILIZER_USE_FIXED_IMU_DT    0U" in freertos
    assert "float dt_sec = SENSOR_IMU_DEFAULT_DT_SEC;" in freertos
    assert "#if (STABILIZER_USE_FIXED_IMU_DT == 0U)" in freertos
    assert "msg.base.timestamp_us - last_imu_timestamp_us" in freertos
    assert "DRV_AttitudeFusion_Update(&fusion_input, &attitude_fusion)" in freertos
    assert "APP_IMU_UpdateAttitude(&msg.imu" not in freertos
    assert "fusion_input.gyroscope_dps[0] = -msg.imu.gyro_x_dps;" in freertos
    assert "fusion_input.gyroscope_dps[1] =  msg.imu.gyro_y_dps;" in freertos
    assert "fusion_input.gyroscope_dps[2] =  msg.imu.gyro_z_dps;" in freertos
    assert "fusion_input.accelerometer_g[0] = -msg.imu.accel_x_g;" in freertos
    assert "fusion_input.accelerometer_g[1] =  msg.imu.accel_y_g;" in freertos
    assert "fusion_input.accelerometer_g[2] = -msg.imu.accel_z_g;" in freertos
    assert "atan2f(-msg.imu.accel_y_g, msg.imu.accel_z_g)" in freertos
    assert "atan2f(-msg.imu.accel_x_g," in freertos
    assert "fusion_input.dt_s = dt_sec;" in freertos
    assert "APP_IMU_AttitudeDebug" in header
    assert "float accel_norm_g;" in header
    assert "float accel_trust;" in header
    assert "float accel_residual_deg;" in header
    assert "msg.attitude_debug.accel_trust = 0.0f;" in freertos
    assert "msg.attitude_debug.alpha = 1.0f;" in freertos


def test_sensor_task_uses_irq_edge_timestamp_and_poll_fallback() -> None:
    sensor = read("App/Src/app_sensor.c")
    timestamp = read("Services/Src/svc_timestamp.c")
    freertos = read("Core/Src/freertos.c")

    callback = sensor[sensor.index("void HAL_GPIO_EXTI_Callback") :]
    assert callback.index("SVC_Timestamp_Us();") < callback.index("osThreadFlagsSet")
    assert "TIM17->SR & TIM_SR_UIF" in timestamp
    assert "if (update_pending != 0U)" in timestamp
    assert "uint64_t imu_sample_timestamp_us = APP_SENSOR_TIMESTAMP_INVALID;" in freertos
    assert "APP_IMU_ReadDataReadyTimestamp(&imu_sample_timestamp_us)" in freertos
    assert "imu_sample_timestamp_us = SVC_Timestamp_Us();" in freertos
    assert "msg.base.timestamp_us    = imu_sample_timestamp_us;" in freertos
    assert "msg.base.timestamp_us    = SVC_Timestamp_Us();" not in freertos
    assert "APP_IMU_ACCEL_CORRECTION_TAU_SEC" not in sensor


def test_slow_mag_step_is_not_run_for_every_imu_sample() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "SENSOR_MAG_PERIOD_US" in freertos
    assert "last_mag_step_us" in freertos
    assert "((now_us - last_mag_step_us) >= SENSOR_MAG_PERIOD_US)" in freertos


def test_vofa_stream_reports_imu_rate_and_interrupt_vs_poll_counts() -> None:
    messages = read("App/Inc/app_messages.h")
    freertos = read("Core/Src/freertos.c")

    assert "uint32_t imu_poll_ready_count;" in messages
    assert "float imu_irq_sample_rate_hz;" in messages
    assert "float imu_poll_sample_rate_hz;" in messages
    assert "float imu_age_ms;" in messages
    assert "APP_IMU_AttitudeDebug attitude_debug;" in messages
    assert "uint32_t imu_irq_ready_count = 0U;" in freertos
    assert "uint32_t imu_poll_ready_count = 0U;" in freertos
    assert "APP_Sensor_RateMeter imu_irq_rate_meter = {0};" in freertos
    assert "APP_Sensor_RateMeter imu_poll_rate_meter = {0};" in freertos
    assert "imu_irq_ready_count++;" in freertos
    assert "msg.imu_data_ready_count = imu_irq_ready_count;" in freertos
    assert "msg.imu_poll_ready_count = imu_poll_ready_count;" in freertos
    assert "msg.imu_irq_sample_rate_hz  = APP_SensorRateMeter_Update(&imu_irq_rate_meter," in freertos
    assert "msg.imu_poll_sample_rate_hz = APP_SensorRateMeter_Update(&imu_poll_rate_meter," in freertos
    assert "#define VOFA_SEND_PERIOD_MS            25U" in freertos
    assert "#define VOFA_DATA_SIZE 28U" in freertos
    assert "vofa_data[4] = (float)(SVC_Timestamp_Us() / 1000ULL) * 0.001f;" in freertos
    assert "vofa_data[23] = msg.fusion_acceleration_error_deg;" in freertos
    assert "vofa_data[27] = (float)msg.fusion_accel_norm_rejected;" in freertos


def test_message_task_does_not_consume_sensor_sample_queue() -> None:
    source = read("App/Src/app_message.c")

    assert "osMessageQueueGet(SensorSampleQueueHandle" not in source
    assert "APP_MESSAGE_IMU_STREAM_ENABLED" not in source


def test_sensor_task_uses_interrupt_with_bounded_ready_fallback() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "SENSOR_IMU_POLL_TIMEOUT_MS" not in freertos
    assert "osThreadFlagsWait(SENSOR_IMU_DATA_READY_FLAG, osFlagsWaitAny,\n                        SENSOR_IMU_DRDY_TIMEOUT_MS)" in freertos
    assert "BSP_IMU_IsDataReady(&imu_ready)" in freertos
    assert "imu_poll_ready_count++;" in freertos
    assert "SENSOR_IMU_DRDY_MISS_FAULT_LIMIT" in freertos
    assert "stabilizer_latch_imu_fault(STABILIZER_IMU_FAULT_DRDY_TIMEOUT);" in freertos


def test_imu_runtime_fault_does_not_auto_reinit_and_can_recover_on_good_sample() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define SENSOR_IMU_DRDY_TIMEOUT_MS" in freertos
    assert "#define SENSOR_IMU_DRDY_MISS_FAULT_LIMIT" in freertos
    assert "#define SENSOR_IMU_READ_FAIL_LIMIT" in freertos
    assert "stabilizer_latch_imu_fault(STABILIZER_IMU_FAULT_DRDY_TIMEOUT);" in freertos
    assert "stabilizer_latch_imu_fault(STABILIZER_IMU_FAULT_READ_FAIL);" in freertos
    assert "BSP_IMU_Invalidate();" in freertos
    assert freertos.count("BSP_IMU_Init()") == 1
    assert freertos.count("BSP_IMU_Invalidate();") == 1
    assert "stabilizer_clear_imu_fault();" in freertos


def test_stabilizer_holds_last_servo_target_on_imu_dropout_after_first_sample() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define STABILIZER_IMU_STALE_MS" in freertos
    assert "#define STABILIZER_IMU_FAILSAFE_MS" not in freertos
    assert "static volatile uint8_t stabilizer_imu_fault_latched = 0U;" in freertos
    assert "uint8_t imu_control_valid = 0U;" in freertos
    assert "((now - stabilizer_imu_last_sample_ms) <= STABILIZER_IMU_STALE_MS)" in freertos
    assert "if (imu_control_valid != 0U)" in freertos
    assert "moves[0].pulse_us = stabilizer_latest_servo_target_us[0];" in freertos
    assert "moves[1].pulse_us = stabilizer_latest_servo_target_us[1];" in freertos
    assert "else if (has_imu_sample == 0U)" in freertos
    assert "运行中 IMU 异常保持上一目标" in freertos
    assert "moves[0].pulse_us = DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US;" in freertos
    assert "moves[1].pulse_us = DRV_COAX_CTRL_SERVO_BETA_CENTER_US;" in freertos


def test_stabilizer_uses_boot_attitude_average_as_zero_point() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define STABILIZER_ATTITUDE_ZERO_MS 1500U" in freertos
    assert "float  roll_zero = 0.0f;" in freertos
    assert "float  pitch_zero = 0.0f;" in freertos
    assert "float  yaw_zero = 0.0f;" in freertos
    assert "roll_zero_sum += roll;" in freertos
    assert "pitch_zero_sum += pitch;" in freertos
    assert "yaw_zero_sum += yaw;" in freertos
    assert "roll_zero = roll_zero_sum / (float)attitude_zero_count;" in freertos
    assert "pitch_zero = pitch_zero_sum / (float)attitude_zero_count;" in freertos
    assert "yaw_zero = yaw_zero_sum / (float)attitude_zero_count;" in freertos
    assert "roll_control = roll - roll_zero;" in freertos
    assert "pitch_control = pitch - pitch_zero;" in freertos
    assert "yaw_control = yaw - yaw_zero;" in freertos
    assert "msg.roll_deg  = roll_control;" in freertos
    assert "msg.pitch_deg = pitch_control;" in freertos
    assert "msg.yaw_deg   = yaw_control;" in freertos
    assert "(attitude_zero_ready != 0U)" in freertos
    assert "attitude.roll_rad = roll_control * STABILIZER_DEG_TO_RAD;" in freertos
    assert "attitude.pitch_rad = pitch_control * STABILIZER_DEG_TO_RAD;" in freertos
    assert "attitude.yaw_rad = yaw_control * STABILIZER_DEG_TO_RAD;" in freertos


def test_gyro_bias_calibration_restarts_when_boot_motion_is_detected() -> None:
    header = read("App/Inc/app_sensor.h")
    source = read("App/Src/app_sensor.c")

    assert "#define APP_SENSOR_GYRO_BIAS_MAX_STATIC_DPS 5.0f" in header
    assert "fabsf(gx) > APP_SENSOR_GYRO_BIAS_MAX_STATIC_DPS" in source
    assert "fabsf(gy) > APP_SENSOR_GYRO_BIAS_MAX_STATIC_DPS" in source
    assert "fabsf(gz) > APP_SENSOR_GYRO_BIAS_MAX_STATIC_DPS" in source
    assert "cal->sum[0] = 0.0f;" in source
    assert "cal->sum[1] = 0.0f;" in source
    assert "cal->sum[2] = 0.0f;" in source
    assert "cal->count = 0U;" in source


def test_imu_cold_boot_soft_resets_and_waits_for_sensor_startup() -> None:
    driver = read("Driver/Src/drv_imu.c")
    bsp = read("BSP/Src/bsp_imu.c")

    assert "#define ICM42688_SENSOR_STARTUP_MS       500U" in driver
    assert "config.soft_reset_on_init = true;" in bsp
    assert "icm42688_delay_ms(dev, ICM42688_SENSOR_STARTUP_MS);" in driver


def test_sensor_lpf_first_sample_initializes_to_input() -> None:
    """The filter must start from the first real sample, not from zero.

    Starting at zero makes the output ramp up to gravity over the filter's
    settling time, which the attitude solver would read as a large false tilt.
    The filter is now a 2nd-order biquad, so all four history terms are seeded.
    """
    header = read("App/Inc/app_sensor.h")
    source = read("App/Src/app_sensor.c")

    assert "uint8_t initialized;" in header
    assert "if (lpf->initialized == 0U)" in source
    assert "lpf->initialized = 1U;" in source
    for term in ("lpf->x1 = input;", "lpf->x2 = input;",
                 "lpf->y1 = input;", "lpf->y2 = input;"):
        assert term in source


def test_attitude_zero_requires_completed_fusion_startup_and_static_window() -> None:
    messages = read("App/Inc/app_messages.h")
    freertos = read("Core/Src/freertos.c")

    assert "uint8_t gyro_bias_ready;" in messages
    assert "msg.gyro_bias_ready = gyro_bias.ready;" in freertos
    zero_block = freertos[
        freertos.index("if ((attitude_zero_ready == 0U) &&"):
        freertos.index("if (attitude_zero_ready != 0U)")
    ]
    assert "(msg.gyro_bias_ready != 0U)" in zero_block
    assert "(attitude_fusion.initialized != 0U)" in zero_block
    assert "(attitude_fusion.startup == 0U)" in zero_block
    assert "(attitude_fusion.accelerometer_ignored == 0U)" in zero_block
    assert "(attitude_fusion.accel_norm_rejected == 0U)" in zero_block
    assert "STABILIZER_ATTITUDE_ZERO_ERROR_MAX_DEG" in zero_block
    assert "STABILIZER_ATTITUDE_ZERO_ACCEL_MIN_G" in zero_block
    assert "STABILIZER_ATTITUDE_ZERO_ACCEL_MAX_G" in zero_block
    assert zero_block.count("STABILIZER_ATTITUDE_ZERO_GYRO_MAX_DPS") == 3
    assert "attitude_zero_start_ms = HAL_GetTick();" in zero_block
    assert "roll_zero_sum += roll;" in zero_block
    assert "pitch_zero_sum += pitch;" in zero_block
    assert "attitude_zero_start_ms = 0U;" in zero_block
    assert "attitude_zero_count = 0U;" in zero_block
    assert "roll_zero_sum = 0.0f;" in zero_block
    assert "pitch_zero_sum = 0.0f;" in zero_block


def test_nav_gravity_compensation_uses_absolute_attitude_not_boot_zero() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "nav_input.roll_rad = roll * STABILIZER_DEG_TO_RAD;" in freertos
    assert "nav_input.pitch_rad = pitch * STABILIZER_DEG_TO_RAD;" in freertos
    assert "nav_input.yaw_rad = yaw_control * STABILIZER_DEG_TO_RAD;" in freertos
    assert "nav_input.roll_rad = roll_control * STABILIZER_DEG_TO_RAD;" not in freertos
    assert "nav_input.pitch_rad = pitch_control * STABILIZER_DEG_TO_RAD;" not in freertos


def test_imu_spi_timeout_is_short_but_not_overly_aggressive() -> None:
    board = read("BSP/Src/bsp_board.c")

    assert "#define BSP_IMU_SPI_TIMEOUT_MS 5U" in board
    assert "imu_bus.timeout_ms = BSP_IMU_SPI_TIMEOUT_MS;" in board
    assert "imu_bus.timeout_ms = 100U;" not in board
