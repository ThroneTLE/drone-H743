from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MICOLINK_HEAD = 0xEF
MICOLINK_DEVICE_ID = 0x0F
MICOLINK_SYSTEM_ID = 0x00
MICOLINK_MSG_ID = 0x51
MICOLINK_PAYLOAD_LEN = 20
MICOLINK_MAX_PAYLOAD_LEN = 64


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def le_u32(value: int) -> list[int]:
    value &= 0xFFFFFFFF
    return [
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    ]


def le_u16(value: int) -> list[int]:
    value &= 0xFFFF
    return [value & 0xFF, (value >> 8) & 0xFF]


def le_i16(value: int) -> list[int]:
    return list(int(value).to_bytes(2, "little", signed=True))


def checksum8(data: list[int]) -> int:
    return sum(data) & 0xFF


def make_micolink_frame(
    payload: list[int],
    *,
    msg_id: int = MICOLINK_MSG_ID,
    sequence: int = 0x42,
    device_id: int = MICOLINK_DEVICE_ID,
    system_id: int = MICOLINK_SYSTEM_ID,
) -> list[int]:
    header = [
        MICOLINK_HEAD,
        device_id & 0xFF,
        system_id & 0xFF,
        msg_id & 0xFF,
        sequence & 0xFF,
        len(payload) & 0xFF,
    ]
    return [*header, *payload, checksum8(header + payload)]


def make_micolink_range_flow_frame(
    *,
    time_ms: int = 123456,
    distance_mm: int = 2396,
    strength: int = 255,
    precision: int = 0,
    tof_status: int = 1,
    flow_vel_x: int = -1,
    flow_vel_y: int = -4,
    flow_quality: int = 82,
    flow_status: int = 1,
    msg_id: int = MICOLINK_MSG_ID,
    sequence: int = 0x42,
) -> list[int]:
    payload = [
        *le_u32(time_ms),
        *le_u32(distance_mm),
        strength & 0xFF,
        precision & 0xFF,
        tof_status & 0xFF,
        0xFF,
        *le_i16(flow_vel_x),
        *le_i16(flow_vel_y),
        flow_quality & 0xFF,
        flow_status & 0xFF,
        0xFF,
        0xFF,
    ]
    return make_micolink_frame(payload, msg_id=msg_id, sequence=sequence)


def consume_micolink_python(data: list[int]):
    buf = [0] * (MICOLINK_MAX_PAYLOAD_LEN + 7)
    offset = 0
    expected_len = 0
    frames = 0
    checksum_errors = 0
    frame_errors = 0
    ignored_messages = 0
    short_payload_errors = 0
    latest = None

    for byte in data:
        if offset == 0:
            if byte != MICOLINK_HEAD:
                continue
            buf[offset] = byte
            offset += 1
            continue

        if offset >= len(buf):
            frame_errors += 1
            offset = 0
            expected_len = 0
            if byte == MICOLINK_HEAD:
                buf[offset] = byte
                offset += 1
            continue

        buf[offset] = byte
        offset += 1

        if offset == 6:
            payload_len = buf[5]
            if payload_len > MICOLINK_MAX_PAYLOAD_LEN:
                frame_errors += 1
                offset = 0
                expected_len = 0
                continue
            expected_len = 6 + payload_len + 1

        if not expected_len or offset < expected_len:
            continue

        frame = buf[:expected_len]
        offset = 0
        expected_len = 0

        if checksum8(frame[:-1]) != frame[-1]:
            checksum_errors += 1
            if byte == MICOLINK_HEAD:
                buf[offset] = byte
                offset += 1
            continue

        device_id = frame[1]
        system_id = frame[2]
        msg_id = frame[3]
        sequence = frame[4]
        payload_len = frame[5]
        payload = frame[6:-1]
        if device_id != MICOLINK_DEVICE_ID or msg_id != MICOLINK_MSG_ID:
            ignored_messages += 1
            continue
        if payload_len < MICOLINK_PAYLOAD_LEN:
            short_payload_errors += 1
            continue

        frames += 1
        distance_mm = int.from_bytes(bytes(payload[4:8]), "little")
        flow_quality = payload[16]
        latest = {
            "device_id": device_id,
            "system_id": system_id,
            "msg_id": msg_id,
            "sequence": sequence,
            "time_ms": int.from_bytes(bytes(payload[0:4]), "little"),
            "distance_mm": distance_mm,
            "distance_valid": 1 if payload[10] == 1 and distance_mm >= 2 else 0,
            "strength": payload[8],
            "precision": payload[9],
            "tof_status": payload[10],
            "flow_vel_x": int.from_bytes(bytes(payload[12:14]), "little", signed=True),
            "flow_vel_y": int.from_bytes(bytes(payload[14:16]), "little", signed=True),
            "flow_quality": flow_quality,
            "flow_status": payload[17],
            "flow_valid": 1 if payload[17] == 1 and flow_quality else 0,
        }
        latest["valid"] = (
            1 if latest["distance_valid"] and latest["flow_valid"] else 0
        )

    return {
        "frames": frames,
        "checksum_errors": checksum_errors,
        "frame_errors": frame_errors,
        "ignored_messages": ignored_messages,
        "short_payload_errors": short_payload_errors,
        "latest": latest,
    }


def test_micolink_parser_accepts_live_range_and_flow_payload_shape() -> None:
    result = consume_micolink_python(make_micolink_range_flow_frame())

    assert result["frames"] == 1
    assert result["checksum_errors"] == 0
    assert result["frame_errors"] == 0
    assert result["latest"] == {
        "device_id": MICOLINK_DEVICE_ID,
        "system_id": MICOLINK_SYSTEM_ID,
        "msg_id": MICOLINK_MSG_ID,
        "sequence": 0x42,
        "time_ms": 123456,
        "distance_mm": 2396,
        "distance_valid": 1,
        "strength": 255,
        "precision": 0,
        "tof_status": 1,
        "flow_vel_x": -1,
        "flow_vel_y": -4,
        "flow_quality": 82,
        "flow_status": 1,
        "flow_valid": 1,
        "valid": 1,
    }


def test_micolink_parser_rejects_bad_checksum_and_oversize_payload() -> None:
    bad_checksum = make_micolink_range_flow_frame()
    bad_checksum[-1] ^= 0x01
    oversize = [MICOLINK_HEAD, MICOLINK_DEVICE_ID, 0, MICOLINK_MSG_ID, 0, 65]

    result = consume_micolink_python(bad_checksum + oversize)

    assert result["frames"] == 0
    assert result["checksum_errors"] == 1
    assert result["frame_errors"] == 1
    assert result["latest"] is None


def test_micolink_parser_resynchronizes_after_noise_and_ignores_other_messages() -> None:
    other = make_micolink_range_flow_frame(msg_id=0x52)
    flow_frame = make_micolink_range_flow_frame(
        distance_mm=2401,
        flow_vel_x=-10,
        flow_vel_y=20,
        sequence=0x43,
    )

    result = consume_micolink_python([0x00, 0x12] + other + flow_frame)

    assert result["frames"] == 1
    assert result["checksum_errors"] == 0
    assert result["ignored_messages"] == 1
    assert result["latest"]["distance_mm"] == 2401
    assert result["latest"]["flow_vel_x"] == -10
    assert result["latest"]["flow_vel_y"] == 20


def test_optical_flow_sources_are_wired_into_firmware_and_cubemx() -> None:
    cmake = read("CMakeLists.txt")
    uart = read("App/Src/app_uart.c")
    freertos = read("Core/Src/freertos.c")
    usart = read("Core/Src/usart.c")
    ioc = read("drone-H743.ioc")
    control = read("App/Src/app_control.c")
    flow_cmd = read("App/Src/app_cmd_flow.c")
    driver_header = read("Driver/Inc/drv_optical_flow.h")

    assert "Driver/Src/drv_optical_flow.c" in cmake
    assert "BSP/Src/bsp_optical_flow.c" in cmake
    assert "App/Src/app_optical_flow.c" in cmake
    assert "BSP_OPTICAL_FLOW_OnUartRxCplt(huart);" in uart
    assert "BSP_OPTICAL_FLOW_OnUartError(huart);" in uart
    assert "BSP_GPS_OnUartRxCplt" not in uart
    assert "BSP_GPS_OnUartError" not in uart
    assert "huart2.Init.BaudRate = 115200;" in usart
    assert "USART2.BaudRate=115200" in ioc
    assert "#define DRV_OPTICAL_FLOW_BAUD_RATE 115200U" in driver_header
    assert "FLOW?" in control
    assert "APP_OpticalFlow_Report();" in flow_cmd
    assert "FLOW PINGAB" not in control + flow_cmd
    assert "FLOW XCV rx_len hex..." in flow_cmd
    assert "BSP_OPTICAL_FLOW_TransceiveRaw" in flow_cmd
    assert "BSP_OPTICAL_FLOW_TransmitRaw" in flow_cmd
    assert "APP_Task_OpticalFlow_Init();" in freertos
    assert "APP_Task_OpticalFlow_Step();" in freertos


def test_velocity_source_uses_flow_dominant_ekf_without_imu_velocity_fallback() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")
    app_flow = read("App/Src/app_optical_flow.c")
    app_flow_header = read("App/Inc/app_optical_flow.h")
    service_c = read("Services/Src/svc_flow_nav.c")

    # 纯 IMU 积分的速度通路（drv_imu_nav）已删除：那是实验性质的加速度积分，
    # 结果完全不可靠，且从不进控制律；留着只会让人误以为光流丢失时有备份可切。
    # 水平速度只有 EKF 一个来源。
    assert "DRV_IMU_NAV_" not in freertos
    assert "ctx->nav_state" not in freertos
    assert "stabilizer_nav_flu_to_legacy_fwd_right_down" not in freertos
    assert "velocity_imu_x_m_s" not in freertos
    assert "APP_OpticalFlow_GetVelocitySample(&flow_vx_m_s," in freertos
    # R-M5-5：融合本体搬到 svc_flow_nav.c，稳定环只喂输入取输出。下面几条与搬家前
    # 的断言一一对应，只是换了归属文件。
    assert "flow_accepted = SVC_FlowNav_Fuse(&fuse_input);" in freertos
    assert "SVC_FlowNav_GetVelocity(&nav_vx_m_s, &nav_vy_m_s);" in freertos
    assert "DRV_NAV_EKF_Predict(&flow_nav_ctx.ekf," in service_c
    assert "DRV_NAV_EKF_FuseFlow(&flow_nav_ctx.ekf," in service_c
    assert "DRV_NAV_EKF_GetDiagnostics(&flow_nav_ctx.ekf, &flow_nav_ctx.diagnostics);" in service_c
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_IMU);" not in freertos
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_NONE);" in freertos
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_FLOW);" in freertos
    assert "APP_OPTICAL_FLOW_VEL_SOURCE_NONE = 0" in app_flow_header
    assert "APP_OpticalFlow_GetVelocitySample" in app_flow_header
    assert "uint32_t *sample_ms" in app_flow_header
    assert "return APP_OpticalFlow_GetVelocitySample(vx_m_s, vy_m_s, &sample_ms);" in app_flow
    assert "*sample_ms = flow_nav_ctx.velocity_sample_ms;" in service_c
    assert "flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_FLOW;" in app_flow
    assert "flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_NONE;" in app_flow
    assert 'return "none";' in app_flow
    # 时效门与“必须先有有效高度”这两条判决跟着搬进 Service，语义不变。
    assert "sample->frame_valid == 0U" in service_c
    assert "(now_ms - sample->distance_received_ms) > SVC_FLOW_NAV_TIMEOUT_MS" in service_c
    assert "(now_ms - sample->flow_received_ms) > SVC_FLOW_NAV_TIMEOUT_MS" in service_c
    assert "flow_nav_ctx.height_valid == 0U" in service_c
    assert "frame->valid == DRV_OPTICAL_FLOW_VALID" in app_flow


def test_optical_flow_fault_recovery_runs_outside_sensor_step() -> None:
    freertos = read("Core/Src/freertos.c")
    app_uart = read("App/Src/app_uart.c")
    app_flow = read("App/Src/app_optical_flow.c")
    app_flow_header = read("App/Inc/app_optical_flow.h")

    assert "APP_OPTICAL_FLOW_HEALTH_RETRYING" in app_flow_header
    assert "APP_OPTICAL_FLOW_HEALTH_FAILED" in app_flow_header
    assert "void APP_OpticalFlow_ServiceRecovery(void);" in app_flow_header
    assert "#define APP_FLOW_FAST_RETRY_LIMIT" in app_flow
    assert "#define APP_FLOW_FAILED_RETRY_MS" in app_flow
    assert "static void app_optical_flow_try_init" in app_flow
    assert "APP_OpticalFlow_ServiceRecovery();" in app_uart
    assert "APP_OpticalFlow_ServiceRecovery();" not in freertos
    assert "status->health = flow_ctx.health;" in app_flow
    assert "status->init_attempts = flow_ctx.init_attempts;" in app_flow
    assert "status->recovery_count = flow_ctx.recovery_count;" in app_flow


def test_micolink_height_and_unrotated_velocity_are_applied_in_the_flow_nav_service() -> None:
    """R-M5-5：高度 LPF 与未旋转速度换算从 App 搬到 Service，公式逐条不变。"""
    app_flow = read("App/Src/app_optical_flow.c")
    header = read("App/Inc/app_optical_flow.h")
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")
    service_h = read("Services/Inc/svc_flow_nav.h")
    service_c = read("Services/Src/svc_flow_nav.c")

    # 安装角 / 旋转补偿既不属于取数层，也不属于 Service。
    for owner in (app_flow, service_c):
        assert "APP_FLOW_MOUNT_COS_45" not in owner
        assert "APP_FLOW_MOUNT_SIN_45" not in owner
        assert "body_vx_m_s" not in owner
        assert "gyro_x_rad_s" not in owner
    assert "stabilizer_compensate_flow_rotation(" in freertos

    assert "#define SVC_FLOW_NAV_MEDIAN_WINDOW            5U" in service_h
    assert "#define SVC_FLOW_NAV_MEDIAN_MIN_SAMPLES       3U" in service_h
    assert "flow_nav_update_median_filter(sample);" in service_c
    assert "flow_nav_ctx.flow_filter_ready == 0U" in service_c
    assert "sensor_vx_m_s = (float)flow_nav_ctx.filtered_flow_vel_x * 0.01f *" in service_c
    assert "sensor_vy_m_s = (float)flow_nav_ctx.filtered_flow_vel_y * 0.01f *" in service_c
    assert "flow_nav_velocity_plausible(sensor_vx_m_s, sensor_vy_m_s)" in service_c
    assert "flow_nav_ctx.vx_m_s = sensor_vx_m_s;" in service_c
    assert "flow_nav_ctx.vy_m_s = sensor_vy_m_s;" in service_c
    assert "SVC_FLOW_NAV_HEIGHT_LPF_ALPHA" in service_c
    assert "#define SVC_FLOW_NAV_MAX_HEIGHT_STEP_M        0.18f" in service_h
    assert "raw_height_m = (float)sample->distance_mm * 0.001f;" in service_c
    assert "fabsf(raw_height_m - flow_nav_ctx.height_m)" in service_c
    assert "SVC_FLOW_NAV_MAX_HEIGHT_STEP_M" in service_c
    assert "flow_nav_ctx.previous_height_sample_ms" in service_c
    assert "sample->flow_received_ms == flow_nav_ctx.processed_flow_ms" in service_c
    assert "flow_nav_ctx.height_valid = 1U;" in service_c

    # App 层只剩透传出口，报告字段一个不少。
    assert "APP_OpticalFlow_GetHeightSample" in header
    assert "SVC_FlowNav_GetHeight(height_m, vertical_velocity_m_s, sample_ms," in app_flow
    assert "APP_OpticalFlow_GetHeightSample(&frame->range_height_m," in freertos
    assert "APP_Rangefinder_" not in freertos
    assert "APP_OpticalFlow_UpdateHeightFromRange" not in app_flow
    assert "APP_OpticalFlow_UpdateHeightFromPressure" not in app_flow
    assert "powf(" not in app_flow
    assert "height_raw_m" in header
    assert "vertical_velocity_m_s" in header
    assert "height_raw_mm" in app_flow


def test_optical_flow_requires_high_quality_before_control_use() -> None:
    app_flow = read("App/Src/app_optical_flow.c")
    header = read("App/Inc/app_optical_flow.h")
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")
    service_h = read("Services/Inc/svc_flow_nav.h")
    service_c = read("Services/Src/svc_flow_nav.c")

    # 硬拒门与噪声曲线锚点是两个宏，不许再被同一个值兼任。
    # 45 的依据见 PIPELINE：遮挡镜头 <5、暗处最低 50、一般 100~140、最好 190。
    assert "#define SVC_FLOW_NAV_MIN_QUALITY              45U" in service_h
    assert "#define SVC_FLOW_NAV_QUALITY_NOISE_LOW        80U" in service_h
    assert "APP_OPTICAL_FLOW_MIN_QUALITY" not in header
    assert "#define SVC_FLOW_NAV_FILTER_RESET_MS          250U" in service_h
    assert "sample->flow_quality < SVC_FLOW_NAV_MIN_QUALITY" in service_c
    assert "flow_nav_ctx.processed_flow_ms = sample->flow_received_ms;" in service_c
    assert "static void flow_nav_mark_velocity_invalid(void)" in service_c
    assert "flow_nav_reject_velocity_sample();" in service_c
    assert "flow_nav_mark_velocity_invalid();" in service_c
    assert "flow_nav_ctx.velocity_sample_ms = 0U;" in service_c
    assert "SVC_FLOW_NAV_FILTER_RESET_MS)" in service_c
    assert "quality=%u min_q=%u flow_st=%u" in app_flow
    assert "flow_status.flow_quality >= SVC_FLOW_NAV_MIN_QUALITY" in freertos
    assert "flow_status.flow_quality != 0U" not in freertos
    assert "flow_vel_x_filtered" in header
    assert "flow_filter_ready" in header
    assert "filt_vx=%d filt_vy=%d filt_ready=%u" in app_flow


def test_optical_flow_rejects_implausible_velocity_before_ekf() -> None:
    app_flow = read("App/Src/app_optical_flow.c")
    header = read("App/Inc/app_optical_flow.h")
    service_h = read("Services/Inc/svc_flow_nav.h")
    service_c = read("Services/Src/svc_flow_nav.c")

    assert "#define SVC_FLOW_NAV_MAX_SPEED_M_S            2.50f" in service_h
    assert "#define SVC_FLOW_NAV_MAX_SPEED_STEP_M_S       1.20f" in service_h
    assert "uint32_t velocity_reject_count;" in header
    assert "flow_nav_ctx.velocity_reject_count++;" in service_c
    assert "vel_rej=%lu" in app_flow


def test_flow_report_includes_recent_micolink_frame_statistics() -> None:
    driver_header = read("Driver/Inc/drv_optical_flow.h")
    driver = read("Driver/Src/drv_optical_flow.c")
    bsp_header = read("BSP/Inc/bsp_optical_flow.h")
    bsp = read("BSP/Src/bsp_optical_flow.c")
    app_header = read("App/Inc/app_optical_flow.h")
    app = read("App/Src/app_optical_flow.c")

    assert "#define DRV_OPTICAL_FLOW_RAW_WINDOW 32U" in driver_header
    assert "DRV_OPTICAL_FLOW_RawStats" in driver_header
    assert "raw_window[DRV_OPTICAL_FLOW_RAW_WINDOW]" in driver_header
    assert "flow_update_raw_stats(dev, &dev->latest);" in driver
    assert "flow_vel_x_peak_to_peak" in driver
    assert "sample_interval_peak_to_peak_us" in driver
    assert "distance_peak_to_peak_mm" in driver
    assert "typedef DRV_OPTICAL_FLOW_RawStats BSP_OPTICAL_FLOW_RawStats;" in bsp_header
    assert "status->raw_stats = flow_dev.raw_stats;" in bsp
    assert "raw_count" in app_header
    assert "flow_vel_x_mean" in app_header
    assert "distance_peak_to_peak_mm" in app_header
    assert "FLOW mico dev=0x%02X sys=0x%02X msg=0x%02X seq=%u" in app
    assert "dist_age=%lu" in app
    assert "flow_age=%lu" in app
    assert "FLOW raw n=%u vx_avg=%d vy_avg=%d dt_avg=%u dist_avg=%lu" in app


def test_micolink_initialization_has_no_lc307_configuration_phase() -> None:
    driver_header = read("Driver/Inc/drv_optical_flow.h")
    driver = read("Driver/Src/drv_optical_flow.c")
    app = read("App/Src/app_optical_flow.c")
    source = app + driver + driver_header

    assert "Config_Init_Uart" not in source
    assert "Sensor_cfg" not in source
    assert "LC307_" not in source
    assert "lc307_config_table" not in source
    assert "DRV_OPTICAL_FLOW_CONFIG_" not in source
    assert "MSP2" not in source
    assert "DRV_OPTICAL_FLOW_MSP" not in source
    assert "cfg_missing" not in app
    assert "FLOW cfg=" not in app
    assert "#define DRV_OPTICAL_FLOW_MICOLINK_HEAD 0xEFU" in driver_header
    assert "#define DRV_OPTICAL_FLOW_MICOLINK_DEVICE_ID 0x0FU" in driver_header
    assert "#define DRV_OPTICAL_FLOW_MICOLINK_MSG_ID 0x51U" in driver_header
    assert "DRV_OPTICAL_FLOW_RANGE_PAYLOAD_LEN" in driver
    assert "flow_checksum_ok" in driver
    assert "checksum = (uint8_t)(checksum + frame[i]);" in driver
    assert "payload_len = dev->frame[5];" in driver
    assert "parsed.time_ms = flow_get_u32_le(&payload[0]);" in driver
    assert "parsed.distance_mm = flow_get_u32_le(&payload[4]);" in driver
    assert "parsed.flow_vel_x = flow_get_i16_le(&payload[12]);" in driver
    assert "parsed.flow_vel_y = flow_get_i16_le(&payload[14]);" in driver
    assert "return (dev->rx_active != 0U) ? DRV_OPTICAL_FLOW_OK : DRV_OPTICAL_FLOW_ERROR;" in driver


def test_optical_flow_bus_stays_on_existing_usart2_dma_binding() -> None:
    header = read("Driver/Inc/drv_optical_flow.h")
    board = read("BSP/Src/bsp_board.c")

    assert "uint32_t timeout_ms;" in header
    assert "void (*delay_ms)(uint32_t ms);" in header
    assert "optical_flow_bus.huart = &huart2;" in board
    assert "optical_flow_bus.timeout_ms = 100U;" in board
    assert "optical_flow_bus.delay_ms = BSP_DelayMs;" in board
    assert "optical_flow_bus.baud_rate = DRV_OPTICAL_FLOW_BAUD_RATE;" in board
