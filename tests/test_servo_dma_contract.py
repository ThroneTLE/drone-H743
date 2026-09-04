from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_stabilizer_uses_nonblocking_servo_dma_path() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert "BSP_BusServo_MoveManyAsync(frame->moves, 2U," in freertos
    assert "BSP_BusServo_MoveMany(moves, 2U" not in freertos
    assert "DRV_SERVO_MoveCmd moves[2]" in freertos
    assert "stabilizer_servo_should_send" in freertos
    assert "stabilizer_servo_command_slot_due" in freertos
    assert "stabilizer_servo_record_target(frame->moves);" in freertos
    assert "STABILIZER_SERVO_BUS_FRAME_MS" in freertos
    assert "STABILIZER_SERVO_REFRESH_MS" in freertos
    assert "STABILIZER_SERVO_DELTA_US" in freertos
    assert "#define STABILIZER_CONTROL_PERIOD_MS   2U" in freertos
    assert "#define STABILIZER_SERVO_BUS_FRAME_MS 10U" in freertos
    assert "#define STABILIZER_SERVO_MOVE_TIME_MS 0U" in freertos
    assert "#define STABILIZER_SERVO_REFRESH_MS    500U" in freertos
    # 遥测周期在 R-T1-1 之后由 app_telem_stream.c 按 TELEM RATE 算，不再是 freertos.c 的常量。
    assert "telem_stream_period_ms" in read("App/Src/app_telem_stream.c")
    assert "stabilizer_servo_commit_sent(frame->moves, frame->now_ms);" in freertos
    assert "stabilizer_servo_bus_diag.move_attempt_count++;" in freertos
    assert "stabilizer_servo_bus_diag.move_busy_count++;" in freertos
    assert "== DRV_SERVO_OK" in freertos


def test_stabilizer_selects_pwm_or_bus_without_running_bus_only_services() -> None:
    source = read("App/Src/app_stabilizer.c")
    start = source.index("static void stabilizer_control_commit(StabilizerContext *ctx,")
    end = source.index("\n}\n", start) + 2
    commit = source[start:end]

    assert '#include "app_servo_type.h"' in source
    assert "const APP_ServoType servo_type = APP_ServoType_GetActive();" in commit
    assert "if (servo_type == APP_SERVO_TYPE_BUS)" in commit
    assert "BSP_BusServo_Service(frame->now_ms);" in commit
    assert "if (servo_type == APP_SERVO_TYPE_PWM)" in commit
    assert "BSP_PWM_Status pwm_alpha_status;" in commit
    assert "BSP_PWM_Status pwm_beta_status;" in commit
    assert "BSP_PWM_SetServoPulse(1U, frame->moves[0].pulse_us);" in commit
    assert "BSP_PWM_SetServoPulse(2U, frame->moves[1].pulse_us);" in commit

    pwm = commit[commit.index("if (servo_type == APP_SERVO_TYPE_PWM)"):]
    pwm = pwm[:pwm.index("    } else {")]
    assert "stabilizer_servo_command_slot_due" not in pwm
    assert "stabilizer_servo_should_send" not in pwm
    assert "APP_ServoFeedback_Service" not in pwm
    assert "if ((pwm_alpha_status == BSP_PWM_OK) &&" in pwm
    assert "(pwm_beta_status == BSP_PWM_OK))" in pwm
    assert "stabilizer_servo_commit_sent(frame->moves, frame->now_ms);" in pwm

    bus = commit[commit.index("    } else {", commit.index("if (servo_type == APP_SERVO_TYPE_PWM)")):]
    assert "stabilizer_servo_command_slot_due" in bus
    assert "stabilizer_servo_should_send" in bus
    assert "BSP_BusServo_MoveManyAsync(frame->moves, 2U," in bus
    assert "APP_ServoFeedback_Service(" in bus


def test_servo_arbitration_precedes_both_hardware_outputs() -> None:
    source = read("App/Src/app_stabilizer.c")
    start = source.index("static void stabilizer_control_commit(StabilizerContext *ctx,")
    end = source.index("\n}\n", start) + 2
    commit = source[start:end]

    record = commit.index("stabilizer_servo_record_target(frame->moves);")
    pwm = commit.index("BSP_PWM_SetServoPulse(1U, frame->moves[0].pulse_us);")
    bus = commit.index("BSP_BusServo_MoveManyAsync(frame->moves, 2U,")
    assert record < pwm
    assert record < bus


def test_stabilizer_keeps_direct_servo_debug_switch_with_controller_path() -> None:
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    # Direct-angle-servo debug switch permanently disabled; dead code removed.
    assert "#define STABILIZER_USE_DIRECT_ANGLE_SERVO 0U" in freertos
    assert "static void stabilizer_map_angle_direct_to_servo" not in freertos
    assert "DRV_COAX_CTRL_Run(&frame->attitude, &frame->reference, &frame->ctrl_out);" in freertos
    assert "frame->moves[0].pulse_us = frame->ctrl_out.servo_alpha_us;" in freertos
    assert "frame->moves[1].pulse_us = frame->ctrl_out.servo_beta_us;" in freertos


def test_servo_async_path_uses_uart_dma_and_cache_clean() -> None:
    driver_header = read("Driver/Inc/drv_servo.h")
    bsp_header = read("BSP/Inc/bsp_bus_servo.h")
    bsp_source = read("BSP/Src/bsp_bus_servo.c")
    driver_source = read("Driver/Src/drv_servo.c")

    assert "DRV_SERVO_BUSY" in driver_header
    assert "DRV_SERVO_Diag" in driver_header
    assert "DRV_SERVO_MoveManyAsync" in driver_header
    assert "DRV_SERVO_GetDiag" in driver_header
    assert "DRV_SERVO_OnUartTxComplete" in driver_header
    assert "DRV_SERVO_OnUartError" in driver_header
    assert "BSP_BusServo_MoveManyAsync" in bsp_header
    assert "BSP_BusServo_GetDiag" in bsp_header
    assert "DRV_SERVO_MoveManyAsync(&servo_dev, moves, count, time_ms)" in bsp_source
    assert '__attribute__((section(".dma_buffer"), aligned(32)))' in driver_source
    assert "BSP_Cache_CleanDCache(servo_dma_tx_buffer, length);" in driver_source
    assert "command[used++] = 'G';" in driver_source
    assert "HAL_UART_Transmit_DMA(dev->bus.huart, servo_dma_tx_buffer, length)" in driver_source
    assert "dev->bus.huart->hdmatx == NULL" in driver_source
    assert "return DRV_SERVO_MoveMany(dev, moves, count, time_ms);" not in driver_source
    assert "servo_async_state != SERVO_ASYNC_IDLE" in driver_source
    assert "dev->bus.huart->gState != HAL_UART_STATE_READY" in driver_source
    assert "dev->bus.huart->RxState != HAL_UART_STATE_READY" in driver_source
    assert "return DRV_SERVO_BUSY;" in driver_source
    assert "servo_try_recover_stuck_dma(dev->bus.huart);" in driver_source
    assert "HAL_UART_AbortTransmit(huart)" in driver_source
    assert "servo_estimate_dma_timeout_ms(dev->bus.huart, length)" in driver_source
    assert "DRV_SERVO_DMA_MIN_TIMEOUT_MS" in driver_source
    assert "servo_diag.tx_busy_count++;" in driver_source
    assert "servo_diag.tx_complete_count++;" in driver_source


def test_uart_callbacks_route_uart7_to_servo_dma_diagnostics() -> None:
    app_uart = read("App/Src/app_uart.c")

    assert '#include "drv_servo.h"' in app_uart
    assert "if (huart->Instance == UART7)" in app_uart
    assert "DRV_SERVO_OnUartTxComplete(huart);" in app_uart
    assert "DRV_SERVO_OnUartError(huart);" in app_uart


def test_vofa_stream_sends_compact_dashboard_channels() -> None:
    # R-T1-1：通道装配搬到 App/Src/app_telem_port.c；坐标出口现统一为 FLU。
    freertos = read("App/Src/app_telem_port.c") + read("App/Src/app_stabilizer.c")

    assert "(values == NULL) || (count != (uint32_t)APP_TELEM_CH_COUNT)" in freertos
    assert "DRV_SERVO_Diag servo_diag;" not in freertos
    assert "BSP_BusServo_GetDiag(&servo_diag);" not in freertos
    assert "vofa_data[APP_TELEM_CH_TIME] = (float)(SVC_Timestamp_Us() / 1000ULL) * 0.001f;" in freertos
    assert "vofa_data[APP_TELEM_CH_VEL_EST_X] = velocity_flu.x;" in freertos
    assert "vofa_data[APP_TELEM_CH_VEL_EST_Y] = velocity_flu.y;" in freertos
    assert "APP_TELEM_CH_VEL_EST_Y] = vofa_debug.vel_est_m_s[1]" not in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.roll_rate_kd", &vofa_data[APP_TELEM_CH_ROLL_RATE_KD]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_loop_enable", &vofa_data[APP_TELEM_CH_VEL_LOOP_ENABLE]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.roll_angle_kp", &vofa_data[APP_TELEM_CH_ROLL_ANGLE_KP]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pitch_angle_kp", &vofa_data[APP_TELEM_CH_PITCH_ANGLE_KP]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pos_z_kp", &vofa_data[APP_TELEM_CH_POS_Z_KP]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pos_z_ki", &vofa_data[APP_TELEM_CH_POS_Z_KI]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_z_kd", &vofa_data[APP_TELEM_CH_VEL_Z_KD]);' in freertos
    assert "osDelay(ms);" in freertos


def test_vofa_runtime_frames_drop_instead_of_queueing_stale_samples() -> None:
    source = read("App/Src/app_vofa.c")

    assert "osMessageQueueGetCount(uartTxQueueHandle) != 0U" in source
    # 队列里还压着别的东西就整帧丢弃：实时曲线要新样本，不要排在旧文本后面的历史。
    assert "return 0U;" in source


def test_vofa_capture_parser_uses_compact_runtime_count() -> None:
    source = read("tools/vofa_serial_capture.py")

    assert "VOFA_FLOAT_COUNT = 28" in source
    assert 'struct.unpack(f"<{VOFA_FLOAT_COUNT}f", payload)' in source
    assert 'struct.unpack("<63f", payload)' not in source


def test_uart7_generated_dma_and_interrupts_are_present() -> None:
    usart = read("Core/Src/usart.c")
    dma = read("Core/Src/dma.c")
    interrupts = read("Core/Src/stm32h7xx_it.c")
    freertos_config = read("Core/Inc/FreeRTOSConfig.h")
    linker = read("STM32H743XX_FLASH.ld")

    assert "hdma_uart7_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;" in usart
    assert "__HAL_LINKDMA(uartHandle,hdmatx,hdma_uart7_tx);" in usart
    assert "hdma_uart7_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;" in usart
    assert "__HAL_LINKDMA(uartHandle,hdmarx,hdma_uart7_rx);" in usart
    assert "HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);" in usart
    assert "HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);" in dma
    assert "HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);" in dma
    assert "HAL_DMA_IRQHandler(&hdma_uart7_tx);" in interrupts
    assert "HAL_DMA_IRQHandler(&hdma_uart7_rx);" in interrupts
    assert "HAL_UART_IRQHandler(&huart7);" in interrupts
    assert "#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5" in freertos_config
    assert ".dma_buffer (NOLOAD)" in linker
    assert "} >RAM_D2" in linker
