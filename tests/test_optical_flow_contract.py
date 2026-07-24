from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MSP2_HEAD = [0x24, 0x58, 0x3C]
MSP2_RANGE_MSG_ID = 0x1F01
MSP2_FLOW_MSG_ID = 0x1F02
MSP2_RANGE_PAYLOAD_LEN = 5
MSP2_FLOW_PAYLOAD_LEN = 9
MSP2_MAX_PAYLOAD_LEN = 64


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


def s32_from_bytes(data: list[int]) -> int:
    value = int.from_bytes(bytes(data), "little", signed=False)
    return value - 0x100000000 if value & 0x80000000 else value


def crc8_dvb_s2(data: list[int]) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def make_msp2_frame(cmd: int, payload: list[int], flags: int = 0) -> list[int]:
    header = [flags & 0xFF, *le_u16(cmd), *le_u16(len(payload))]
    return [*MSP2_HEAD, *header, *payload, crc8_dvb_s2(header + payload)]


def make_msp2_range_frame(
    *,
    distance_mm: int = 2396,
    quality: int = 8,
    msg_id: int = MSP2_RANGE_MSG_ID,
) -> list[int]:
    payload = [quality & 0xFF, *le_u32(distance_mm)]
    return make_msp2_frame(msg_id, payload)


def make_msp2_flow_frame(
    *,
    flow_vel_x: int = -1,
    flow_vel_y: int = -4,
    quality: int = 82,
    msg_id: int = MSP2_FLOW_MSG_ID,
) -> list[int]:
    payload = [quality & 0xFF, *le_u32(flow_vel_x), *le_u32(flow_vel_y)]
    return make_msp2_frame(msg_id, payload)


def consume_msp2_python(data: list[int]):
    buf = [0] * (MSP2_MAX_PAYLOAD_LEN + 9)
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
            if byte != MSP2_HEAD[0]:
                continue
            buf[offset] = byte
            offset += 1
            continue

        if (offset == 1 and byte != MSP2_HEAD[1]) or (
            offset == 2 and byte != MSP2_HEAD[2]
        ):
            frame_errors += 1
            offset = 0
            expected_len = 0
            if byte == MSP2_HEAD[0]:
                buf[offset] = byte
                offset += 1
            continue

        if offset >= len(buf):
            frame_errors += 1
            offset = 0
            expected_len = 0
            if byte == MSP2_HEAD[0]:
                buf[offset] = byte
                offset += 1
            continue

        buf[offset] = byte
        offset += 1

        if offset == 8:
            payload_len = buf[6] | (buf[7] << 8)
            if payload_len > MSP2_MAX_PAYLOAD_LEN:
                frame_errors += 1
                offset = 0
                expected_len = 0
                continue
            expected_len = 8 + payload_len + 1

        if not expected_len or offset < expected_len:
            continue

        frame = buf[:expected_len]
        offset = 0
        expected_len = 0

        if crc8_dvb_s2(frame[3:-1]) != frame[-1]:
            checksum_errors += 1
            if byte == MSP2_HEAD[0]:
                buf[offset] = byte
                offset += 1
            continue

        cmd = frame[4] | (frame[5] << 8)
        payload_len = frame[6] | (frame[7] << 8)
        payload = frame[8:-1]
        if cmd not in (MSP2_RANGE_MSG_ID, MSP2_FLOW_MSG_ID):
            ignored_messages += 1
            continue
        if (cmd == MSP2_RANGE_MSG_ID and payload_len < MSP2_RANGE_PAYLOAD_LEN) or (
            cmd == MSP2_FLOW_MSG_ID and payload_len < MSP2_FLOW_PAYLOAD_LEN
        ):
            short_payload_errors += 1
            continue

        frames += 1
        if latest is None:
            latest = {
                "distance_mm": 0,
                "distance_valid": 0,
                "strength": 0,
                "flow_vel_x": 0,
                "flow_vel_y": 0,
                "flow_quality": 0,
                "flow_valid": 0,
            }
        latest["msp_cmd"] = cmd
        latest["msp_flags"] = frame[3]
        if cmd == MSP2_RANGE_MSG_ID:
            distance_mm = int.from_bytes(bytes(payload[1:5]), "little")
            latest["distance_mm"] = distance_mm
            latest["strength"] = payload[0]
            latest["distance_valid"] = 1 if payload[0] and distance_mm >= 10 else 0
        else:
            latest["flow_vel_x"] = s32_from_bytes(payload[1:5])
            latest["flow_vel_y"] = s32_from_bytes(payload[5:9])
            latest["flow_quality"] = payload[0]
            latest["flow_valid"] = 1 if payload[0] else 0
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


def test_msp2_parser_accepts_live_range_and_flow_payload_shapes() -> None:
    result = consume_msp2_python(make_msp2_range_frame() + make_msp2_flow_frame())

    assert result["frames"] == 2
    assert result["checksum_errors"] == 0
    assert result["frame_errors"] == 0
    assert result["latest"] == {
        "msp_cmd": MSP2_FLOW_MSG_ID,
        "msp_flags": 0,
        "distance_mm": 2396,
        "distance_valid": 1,
        "strength": 8,
        "flow_vel_x": -1,
        "flow_vel_y": -4,
        "flow_quality": 82,
        "flow_valid": 1,
        "valid": 1,
    }


def test_msp2_parser_rejects_bad_crc_and_oversize_payload() -> None:
    bad_checksum = make_msp2_range_frame()
    bad_checksum[-1] ^= 0x01
    oversize = [*MSP2_HEAD, 0, *le_u16(MSP2_RANGE_MSG_ID), 65, 0]

    result = consume_msp2_python(bad_checksum + oversize)

    assert result["frames"] == 0
    assert result["checksum_errors"] == 1
    assert result["frame_errors"] == 1
    assert result["latest"] is None


def test_msp2_parser_resynchronizes_after_noise_and_ignores_other_messages() -> None:
    other = make_msp2_frame(0x1F03, [1, 2, 3, 4])
    range_frame = make_msp2_range_frame(distance_mm=2401, quality=7)
    flow_frame = make_msp2_flow_frame(flow_vel_x=-10, flow_vel_y=20)

    result = consume_msp2_python([0x00, MSP2_HEAD[0], 0x12] + other + range_frame + flow_frame)

    assert result["frames"] == 2
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
    assert "APP_OpticalFlow_Report();" in control
    assert "FLOW PINGAB" not in control
    assert "FLOW XCV rx_len hex..." in control
    assert "BSP_OPTICAL_FLOW_TransceiveRaw" in control
    assert "BSP_OPTICAL_FLOW_TransmitRaw" in control
    assert "APP_Task_OpticalFlow_Init();" in freertos
    assert "APP_Task_OpticalFlow_Step();" in freertos


def test_velocity_source_prefers_flow_and_falls_back_to_imu() -> None:
    freertos = read("Core/Src/freertos.c")
    app_flow = read("App/Src/app_optical_flow.c")
    app_flow_header = read("App/Inc/app_optical_flow.h")

    assert "velocity_imu_x_m_s = nav_state.vel_m_s[0];" in freertos
    assert "APP_OpticalFlow_GetVelocitySample(&flow_vx_m_s," in freertos
    assert "stabilizer_velocity_estimator_step(&vel_estimator," in freertos
    assert "DRV_NAV_EKF_Predict(&state->ekf, acc_x_m_s2, acc_y_m_s2, dt_sec);" in freertos
    assert "DRV_NAV_EKF_FuseFlow(&state->ekf," in freertos
    assert "DRV_NAV_EKF_GetDiagnostics(&state->ekf, &state->diagnostics);" in freertos
    assert "flow_accepted = stabilizer_velocity_estimator_step(&vel_estimator," in freertos
    assert "velocity_state_x_m_s = vel_estimator.vel_m_s[0];" in freertos
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_IMU);" in freertos
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_FLOW);" in freertos
    assert "APP_OpticalFlow_GetVelocitySample" in app_flow_header
    assert "uint32_t *sample_ms" in app_flow_header
    assert "return APP_OpticalFlow_GetVelocitySample(vx_m_s, vy_m_s, &sample_ms);" in app_flow
    assert "*sample_ms = flow_ctx.velocity_sample_ms;" in app_flow
    assert "flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_FLOW;" in app_flow
    assert "flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_IMU;" in app_flow
    assert "frame->valid != DRV_OPTICAL_FLOW_VALID" in app_flow
    assert "(now_ms - frame->distance_received_ms) > APP_FLOW_TIMEOUT_MS" in app_flow
    assert "(now_ms - frame->flow_received_ms) > APP_FLOW_TIMEOUT_MS" in app_flow
    assert "flow_ctx.height_valid == 0U" in app_flow


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


def test_msp2_height_and_unrotated_velocity_are_applied_in_app_layer() -> None:
    app_flow = read("App/Src/app_optical_flow.c")
    header = read("App/Inc/app_optical_flow.h")
    freertos = read("Core/Src/freertos.c")

    assert "APP_FLOW_MOUNT_COS_45" not in app_flow
    assert "APP_FLOW_MOUNT_SIN_45" not in app_flow
    assert "body_vx_m_s" not in app_flow
    assert "body_vy_m_s" not in app_flow
    assert "sensor_vx_m_s = (float)frame->flow_vel_x * 0.01f * flow_ctx.height_m;" in app_flow
    assert "sensor_vy_m_s = (float)frame->flow_vel_y * 0.01f * flow_ctx.height_m;" in app_flow
    assert "app_flow_velocity_plausible(sensor_vx_m_s, sensor_vy_m_s)" in app_flow
    assert "flow_ctx.vx_m_s = sensor_vx_m_s;" in app_flow
    assert "flow_ctx.vy_m_s = sensor_vy_m_s;" in app_flow
    assert "APP_FLOW_HEIGHT_LPF_ALPHA" in app_flow
    assert "raw_height_m = (float)frame->distance_mm * 0.001f;" in app_flow
    assert "frame->distance_received_ms == flow_ctx.previous_height_sample_ms" in app_flow
    assert "frame->flow_received_ms != flow_ctx.processed_flow_ms" in app_flow
    assert "flow_ctx.height_valid = 1U;" in app_flow
    assert "APP_OpticalFlow_GetHeightSample" in header
    assert "APP_OpticalFlow_GetHeightSample(&range_height_m," in freertos
    assert "APP_Rangefinder_GetHeightSample" not in freertos
    assert "APP_OpticalFlow_UpdateHeightFromRange" not in app_flow
    assert "APP_OpticalFlow_UpdateHeightFromPressure" not in app_flow
    assert "powf(" not in app_flow
    assert "height_raw_m" in header
    assert "vertical_velocity_m_s" in header
    assert "height_raw_mm" in app_flow


def test_optical_flow_rejects_implausible_velocity_before_ekf() -> None:
    app_flow = read("App/Src/app_optical_flow.c")
    header = read("App/Inc/app_optical_flow.h")

    assert "#define APP_FLOW_MAX_SPEED_M_S       2.50f" in app_flow
    assert "#define APP_FLOW_MAX_SPEED_STEP_M_S  1.20f" in app_flow
    assert "uint32_t velocity_reject_count;" in header
    assert "flow_ctx.velocity_reject_count++;" in app_flow
    assert "vel_rej=%lu" in app_flow


def test_flow_report_includes_recent_msp2_frame_statistics() -> None:
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
    assert "FLOW msp cmd=0x%04X flags=0x%02X" in app
    assert "dist_age=%lu" in app
    assert "flow_age=%lu" in app
    assert "FLOW raw n=%u vx_avg=%d vy_avg=%d dt_avg=%u dist_avg=%lu" in app


def test_msp2_initialization_has_no_lc307_configuration_phase() -> None:
    driver = read("Driver/Src/drv_optical_flow.c")
    app = read("App/Src/app_optical_flow.c")
    source = app + driver

    assert "Config_Init_Uart" not in source
    assert "Sensor_cfg" not in source
    assert "LC307_" not in source
    assert "lc307_config_table" not in source
    assert "DRV_OPTICAL_FLOW_CONFIG_" not in source
    assert "MICOLINK" not in source
    assert "cfg_missing" not in app
    assert "FLOW cfg=" not in app
    assert "DRV_OPTICAL_FLOW_MSP_HEAD_0" in driver
    assert "DRV_OPTICAL_FLOW_RANGE_MSG_ID" in driver
    assert "DRV_OPTICAL_FLOW_FLOW_MSG_ID" in driver
    assert "DRV_OPTICAL_FLOW_RANGE_PAYLOAD_LEN" in driver
    assert "DRV_OPTICAL_FLOW_FLOW_PAYLOAD_LEN" in driver
    assert "flow_checksum_ok" in driver
    assert "flow_crc8_dvb_s2" in driver
    assert "parsed.distance_mm = flow_get_u32_le(&payload[1]);" in driver
    assert "parsed.flow_vel_x = flow_i32_to_i16_sat(flow_get_i32_le(&payload[1]));" in driver
    assert "parsed.flow_vel_y = flow_i32_to_i16_sat(flow_get_i32_le(&payload[5]));" in driver
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
