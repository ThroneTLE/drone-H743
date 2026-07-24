from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_stabilizer_uses_nonblocking_servo_dma_path() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "BSP_BusServo_MoveManyAsync(moves, 2U,\n                                         STABILIZER_SERVO_MOVE_TIME_MS)" in freertos
    assert "BSP_BusServo_MoveMany(moves, 2U" not in freertos
    assert "DRV_SERVO_MoveCmd moves[2]" in freertos
    assert "BSP_PWM_SetServoPulse" not in freertos
    assert "stabilizer_servo_should_send" in freertos
    assert "stabilizer_servo_record_target(moves);" in freertos
    assert "STABILIZER_SERVO_REFRESH_MS" in freertos
    assert "STABILIZER_SERVO_DELTA_US" in freertos
    assert "#define STABILIZER_CONTROL_PERIOD_MS   2U" in freertos
    assert "#define STABILIZER_SERVO_MOVE_TIME_MS 0U" in freertos
    assert "#define STABILIZER_SERVO_REFRESH_MS    500U" in freertos
    assert "#define VOFA_SEND_PERIOD_MS            25U" in freertos
    assert "stabilizer_servo_commit_sent(moves, now);" in freertos
    assert "== DRV_SERVO_OK" in freertos


def test_stabilizer_keeps_direct_servo_debug_switch_with_controller_path() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define STABILIZER_USE_DIRECT_ANGLE_SERVO 0U" in freertos
    assert "static void stabilizer_map_angle_direct_to_servo" in freertos
    assert "float body_x_tilt_rad = pitch_deg * STABILIZER_DEG_TO_RAD *" in freertos
    assert "float body_y_tilt_rad = roll_deg * STABILIZER_DEG_TO_RAD *" in freertos
    assert "DRV_COAX_CTRL_BodyTiltRadToServoPulses(body_x_tilt_rad," in freertos
    assert "body_y_tilt_rad," in freertos
    assert "DRV_COAX_CTRL_Run(&attitude, &reference, &ctrl_out);" in freertos
    assert "moves[0].pulse_us = ctrl_out.servo_alpha_us;" in freertos
    assert "moves[1].pulse_us = ctrl_out.servo_beta_us;" in freertos


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
    assert "if (dev->bus.huart->gState != HAL_UART_STATE_READY)" in driver_source
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
    freertos = read("Core/Src/freertos.c")

    assert "#define VOFA_DATA_SIZE 22U" in freertos
    assert "DRV_SERVO_Diag servo_diag;" not in freertos
    assert "BSP_BusServo_GetDiag(&servo_diag);" not in freertos
    assert "vofa_data[4] = (float)(SVC_Timestamp_Us() / 1000ULL) * 0.001f;" in freertos
    assert "vofa_data[5] = vofa_debug.vel_est_m_s[0];" in freertos
    assert "vofa_data[6] = vofa_debug.vel_est_m_s[1];" in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.roll_rate_kd", &vofa_data[7]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_loop_enable", &vofa_data[17]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.roll_angle_kp", &vofa_data[18]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pitch_angle_kp", &vofa_data[19]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pos_z_kp", &vofa_data[20]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_z_kd", &vofa_data[21]);' in freertos
    assert "osDelay(VOFA_SEND_PERIOD_MS);" in freertos


def test_vofa_runtime_frames_drop_instead_of_queueing_stale_samples() -> None:
    source = read("App/Src/app_vofa.c")

    assert "osMessageQueueGetCount(uartTxQueueHandle) != 0U" in source
    assert "return;" in source


def test_vofa_capture_parser_uses_compact_runtime_count() -> None:
    source = read("tools/vofa_serial_capture.py")

    assert "VOFA_FLOAT_COUNT = 22" in source
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
