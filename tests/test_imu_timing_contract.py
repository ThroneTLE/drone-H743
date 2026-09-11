from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_stabilizer_uses_drdy_timed_fusion_ahrs() -> None:
    header = read("App/Inc/app_sensor.h")
    source = read("App/Src/app_sensor.c")
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert "APP_IMU_ReadDataReadyTimestamp" in header
    assert "app_imu_drdy_timestamp.sequence" in source
    assert "SVC_Timestamp_Us();" in source
    assert "osKernelGetTickCount()" not in source
    assert "#define STABILIZER_USE_FIXED_IMU_DT    0U" in freertos
    assert "float dt_sec = SENSOR_IMU_DEFAULT_DT_SEC;" in freertos
    # Always-true guard removed; timestamp dt is the only compiled path.
    assert "#if (STABILIZER_USE_FIXED_IMU_DT" not in freertos
    assert "msg->base.timestamp_us - ctx->last_imu_timestamp_us" in freertos
    assert "DRV_AttitudeFusion_Update(&fusion_input, &ctx->attitude_fusion)" in freertos
    assert "APP_IMU_UpdateAttitude(&msg.imu" not in freertos
    assert "fusion_input.gyroscope_dps[0] = -msg->imu.gyro_x_dps;" in freertos
    assert "fusion_input.gyroscope_dps[1] =  msg->imu.gyro_y_dps;" in freertos
    assert "fusion_input.gyroscope_dps[2] =  msg->imu.gyro_z_dps;" in freertos
    assert "fusion_input.accelerometer_g[0] = -msg->imu.accel_x_g;" in freertos
    assert "fusion_input.accelerometer_g[1] =  msg->imu.accel_y_g;" in freertos
    assert "fusion_input.accelerometer_g[2] = -msg->imu.accel_z_g;" in freertos
    assert "atan2f(-msg->imu.accel_y_g, msg->imu.accel_z_g)" in freertos
    assert "atan2f(-msg->imu.accel_x_g," in freertos
    assert "fusion_input.dt_s = dt_sec;" in freertos
    assert "APP_IMU_AttitudeDebug" in header
    assert "float accel_norm_g;" in header
    assert "float accel_trust;" in header
    assert "float accel_residual_deg;" in header
    assert "msg->attitude_debug.accel_trust = 0.0f;" in freertos
    assert "msg->attitude_debug.alpha = 1.0f;" in freertos


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


def test_every_exti_pin_is_one_the_drdy_callback_actually_consumes() -> None:
    """配成外部中断的引脚，必须是 DRDY 回调真的会处理的那些。

    中断进来又被掩码挡掉 = 纯浪费，而且浪费在优先级最高的中断路径上。
    MicoAir743v2 上 BMI088 是加计 + 陀螺两颗独立芯片，各有一路 DRDY 且**都实打实
    接到了 MCU**（hwdef: PC15 DRDY1_BMI088_G / PC14 DRDY2_BMI088_A）。但节拍只能由
    陀螺定（角速率是最内环），加计 ODR 1600 Hz 要是也开中断，每秒白进约 1600 次。
    所以 PC14 必须是普通输入而不是 EXTI。

    这条断言的是"EXTI 引脚集合 == 路由表覆盖的引脚集合"这个不变量，不是某个引脚名。

    2026-09-11 更新：回调不再用一个静态掩码同时接受两颗 IMU 的 DRDY，改为只认
    `BSP_IMU_GetDrdyPin()` 给出的那一个。原来的写法假设"另一颗没初始化就不会产生
    边沿"——冷启动成立，**软复位不成立**：`BOOT DFU CONFIRM`、看门狗复位都不给传感器
    掉电，上一轮配置过的那颗照旧发边沿。实测节拍因此从 1000 Hz 虚高到 1760 Hz，
    三成迭代拿到重复的陀螺样本。所以这里改成核对 BSP 的路由表。
    """
    import re

    ioc = read("drone-H743.ioc")
    sensor = read("App/Src/app_sensor.c")

    # .ioc 里配成外部中断的引脚号（GPXTI<n> / GPIO_EXTI<n>）。
    exti_pins = set()
    for match in re.finditer(r"^(P[A-K])(\d+).*\.Signal=GPXTI(\d+)$", ioc, re.MULTILINE):
        assert match.group(2) == match.group(3), "EXTI 线号必须等于引脚号"
        exti_pins.add(int(match.group(3)))
    # 带 OSC32 后缀的脚名格式不同，单独扫一遍。
    for match in re.finditer(r"^P[A-K](\d+)[^=]*\.Signal=GPXTI(\d+)$", ioc, re.MULTILINE):
        exti_pins.add(int(match.group(2)))

    assert exti_pins, ".ioc 里一个外部中断都没有，DRDY 会退到轮询兜底"

    # 回调只认选中那颗的引脚，具体是哪个由 BSP 的路由表决定。
    assert "BSP_IMU_GetDrdyPin()" in sensor, (
        "DRDY 回调必须按选中的芯片取引脚，不能再用静态掩码同时接受两颗"
    )
    assert "APP_IMU_DRDY_PIN_MASK" not in sensor, (
        "静态掩码已被 BSP_IMU_GetDrdyPin() 取代，别让两套并存"
    )

    bsp = read("BSP/Src/bsp_imu.c")
    routed = set()
    for macro in re.findall(r"return \(uint16_t\)(\w+_DRDY_Pin);", bsp):
        pin = re.search(r"#define\s+" + macro + r"\s+GPIO_PIN_(\d+)",
                        read("Core/Inc/main.h"))
        assert pin is not None, f"main.h 里找不到 {macro}"
        routed.add(int(pin.group(1)))

    assert exti_pins == routed, (
        f".ioc 的 EXTI 引脚 {sorted(exti_pins)} 与 BSP 路由表里的 {sorted(routed)} 对不上："
        f"多出来的会白进中断，少掉的会让控制环退到 20 ms 轮询兜底"
    )

    # 未选中那颗的 EXTI 必须在源头被屏蔽，而不是每次都靠回调比对挡掉。
    assert "EXTI_D1->IMR1" in bsp, (
        "选型之后要清掉未选中那颗的 EXTI 屏蔽位；"
        "一个永远会被忽略的中断不该处于使能状态"
    )


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
    # R-T1-1：通道装配从 freertos.c 搬到 App/Src/app_telem_port.c，速率默认值搬到
    # app_telem_stream.c。断言跟着搬，语义不变——这几条通道必须还在遥测里。
    port = read("App/Src/app_telem_port.c")
    stream = read("App/Src/app_telem_stream.c")
    assert "app_telem_stream.rate_hz           = APP_TELEM_RATE_HZ;" in stream
    assert "vofa_data[APP_TELEM_CH_TIME] = (float)(SVC_Timestamp_Us() / 1000ULL) * 0.001f;" in port
    assert "vofa_data[APP_TELEM_CH_FUSION_ACC_ERR] = msg.fusion_acceleration_error_deg;" in port
    assert "vofa_data[APP_TELEM_CH_FUSION_ACC_NORM_REJECTED] = (float)msg.fusion_accel_norm_rejected;" in port


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
    assert "APP_Stabilizer_LatchImuFault(STABILIZER_IMU_FAULT_DRDY_TIMEOUT);" in freertos


def test_imu_runtime_fault_does_not_auto_reinit_and_can_recover_on_good_sample() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define SENSOR_IMU_DRDY_TIMEOUT_MS" in freertos
    assert "#define SENSOR_IMU_DRDY_MISS_FAULT_LIMIT" in freertos
    assert "#define SENSOR_IMU_READ_FAIL_LIMIT" in freertos
    assert "APP_Stabilizer_LatchImuFault(STABILIZER_IMU_FAULT_DRDY_TIMEOUT);" in freertos
    assert "APP_Stabilizer_LatchImuFault(STABILIZER_IMU_FAULT_READ_FAIL);" in freertos
    assert "BSP_IMU_Invalidate();" in freertos
    assert freertos.count("BSP_IMU_Init()") == 1
    assert freertos.count("BSP_IMU_Invalidate();") == 1
    assert "APP_Stabilizer_ClearImuFault();" in freertos


def test_stabilizer_holds_last_servo_target_on_imu_dropout_after_first_sample() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert "#define STABILIZER_IMU_STALE_MS" in freertos
    assert "#define STABILIZER_IMU_FAILSAFE_MS" not in freertos
    assert "static volatile uint8_t stabilizer_imu_fault_latched = 0U;" in freertos
    assert "uint8_t imu_control_valid;" in freertos
    assert "((frame->now_ms - stabilizer_imu_last_sample_ms) <= STABILIZER_IMU_STALE_MS)" in freertos
    assert "} else if (frame->imu_control_valid != 0U) {" in freertos
    assert "frame->moves[0].pulse_us = stabilizer_latest_servo_target_us[0];" in freertos
    assert "frame->moves[1].pulse_us = stabilizer_latest_servo_target_us[1];" in freertos
    assert "} else if (ctx->has_imu_sample == 0U) {" in freertos
    assert "运行中 IMU 异常保持上一目标" in freertos
    assert "DRV_COAX_CTRL_GetServoCalibration(&servo_calibration);" in freertos
    assert "servo_calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]" in freertos
    assert "servo_calibration.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX]" in freertos


def test_stabilizer_uses_boot_attitude_average_as_zero_point() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert "#define STABILIZER_ATTITUDE_ZERO_MS 1500U" in freertos
    assert "float roll_zero;" in freertos
    assert "float pitch_zero;" in freertos
    assert "float yaw_zero;" in freertos
    assert "ctx->roll_zero_sum += ctx->roll;" in freertos
    assert "ctx->pitch_zero_sum += ctx->pitch;" in freertos
    assert "ctx->yaw_zero_sum += ctx->yaw;" in freertos
    assert "ctx->roll_zero = ctx->roll_zero_sum / (float)ctx->attitude_zero_count;" in freertos
    assert "ctx->pitch_zero = ctx->pitch_zero_sum / (float)ctx->attitude_zero_count;" in freertos
    assert "ctx->yaw_zero = ctx->yaw_zero_sum / (float)ctx->attitude_zero_count;" in freertos
    assert "ctx->roll_control = ctx->roll - ctx->roll_zero;" in freertos
    assert "ctx->pitch_control = ctx->pitch - ctx->pitch_zero;" in freertos
    assert "ctx->yaw_control = ctx->yaw - ctx->yaw_zero;" in freertos
    assert "msg->roll_deg  = ctx->roll_control;" in freertos
    assert "msg->pitch_deg = ctx->pitch_control;" in freertos
    assert "msg->yaw_deg   = ctx->yaw_control;" in freertos
    assert "(ctx->attitude_zero_ready != 0U)" in freertos
    assert "frame->attitude.roll_rad = ctx->roll_control * STABILIZER_DEG_TO_RAD;" in freertos
    assert "frame->attitude.pitch_rad = ctx->pitch_control * STABILIZER_DEG_TO_RAD;" in freertos
    assert "frame->attitude.yaw_rad = ctx->yaw_control * STABILIZER_DEG_TO_RAD;" in freertos


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
    # 配置构造搬进了 BSP_IMU_BuildConfig()（多颗候选芯片共用一份配置），
    # 所以现在是指针写法；"冷启动必须软复位"这条不变。
    assert "config->soft_reset_on_init = true;" in bsp
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
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert "uint8_t gyro_bias_ready;" in messages
    assert "msg.gyro_bias_ready = gyro_bias.ready;" in freertos
    zero_block = freertos[
        freertos.index("if ((ctx->attitude_zero_ready == 0U) &&"):
        freertos.index("if (ctx->attitude_zero_ready != 0U)")
    ]
    assert "(msg->gyro_bias_ready != 0U)" in zero_block
    assert "(ctx->attitude_fusion.initialized != 0U)" in zero_block
    assert "(ctx->attitude_fusion.startup == 0U)" in zero_block
    assert "(ctx->attitude_fusion.accelerometer_ignored == 0U)" in zero_block
    assert "(ctx->attitude_fusion.accel_norm_rejected == 0U)" in zero_block
    assert "STABILIZER_ATTITUDE_ZERO_ERROR_MAX_DEG" in zero_block
    assert "STABILIZER_ATTITUDE_ZERO_ACCEL_MIN_G" in zero_block
    assert "STABILIZER_ATTITUDE_ZERO_ACCEL_MAX_G" in zero_block
    assert zero_block.count("STABILIZER_ATTITUDE_ZERO_GYRO_MAX_DPS") == 3
    assert "ctx->attitude_zero_start_ms = HAL_GetTick();" in zero_block
    assert "ctx->roll_zero_sum += ctx->roll;" in zero_block
    assert "ctx->pitch_zero_sum += ctx->pitch;" in zero_block
    assert "ctx->attitude_zero_start_ms = 0U;" in zero_block
    assert "ctx->attitude_zero_count = 0U;" in zero_block
    assert "ctx->roll_zero_sum = 0.0f;" in zero_block
    assert "ctx->pitch_zero_sum = 0.0f;" in zero_block


def test_nav_gravity_compensation_uses_absolute_attitude_not_boot_zero() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    # 纯 IMU 积分通路（drv_imu_nav）已删除：其速度输出是不可靠的实验数据。
    # 水平加速度只剩喂 EKF 的这一路，
    # 它必须继续用零点补偿后的姿态，而不是原始角。
    assert "DRV_IMU_NAV_" not in freertos
    assert "nav_input" not in freertos
    assert "ctx->roll_control * STABILIZER_DEG_TO_RAD," in freertos
    assert "ctx->pitch_control * STABILIZER_DEG_TO_RAD," in freertos
    # yaw 不再传进这个适配器：加速度只转平到机头对齐的本地水平系，好和同一拍
    # 的光流速度同系。见 tests/test_flu_nav_frame_alignment.py。
    assert "stabilizer_compensated_imu_accel_level_xy(" in freertos


def test_imu_spi_timeout_is_short_but_not_overly_aggressive() -> None:
    board = read("BSP/Src/bsp_board.c")

    assert "#define BSP_IMU_SPI_TIMEOUT_MS 5U" in board
    assert "imu_bus.timeout_ms = BSP_IMU_SPI_TIMEOUT_MS;" in board
    assert "imu_bus.timeout_ms = 100U;" not in board
