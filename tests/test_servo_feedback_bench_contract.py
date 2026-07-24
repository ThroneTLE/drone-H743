from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_driver_uses_manual_prad_request_and_async_dma_receive() -> None:
    source = read("Driver/Src/drv_servo.c")

    assert '"#%03uPRAD!"' in source
    assert "HAL_UARTEx_ReceiveToIdle_DMA" in source
    assert "DRV_SERVO_POSITION_RESPONSE_LEN 10U" in source
    assert "servo_parse_position_response" in source
    assert "BSP_Cache_InvalidateDCache" in source


def test_uart7_rx_event_is_routed_to_servo_driver() -> None:
    source = read("App/Src/app_uart.c")

    assert "if (huart->Instance == UART7)" in source
    assert "DRV_SERVO_OnUartRxEvent(huart, Size);" in source


def test_feedback_bench_is_opt_in_and_does_not_generate_motion_targets() -> None:
    source = read("App/Src/app_servo_feedback_bench.c")

    assert "APP_SERVO_FB_MODE_IDLE" in source
    assert "APP_ServoFeedbackBench_Init" in source
    assert "BSP_BusServo_RequestPositionAsync(moves[slot].id" in source
    assert "moves[slot].pulse_us" in source
    assert "sin(" not in source
    assert "cos(" not in source


def test_feedback_bench_commands_and_realistic_move_load_are_wired() -> None:
    control = read("App/Src/app_control.c")
    freertos = read("Core/Src/freertos.c")
    cmake = read("CMakeLists.txt")

    assert 'strcmp(tokens[1], "FB") == 0' in control
    assert 'strcmp(tokens[2], "START") == 0' in control
    assert 'strcmp(tokens[2], "SWEEP") == 0' in control
    assert 'strcmp(tokens[2], "STATUS") == 0' in control
    assert 'strcmp(tokens[2], "STOP") == 0' in control
    assert "APP_ServoFeedbackBench_MoveRefreshDue" in freertos
    assert "APP_ServoFeedbackBench_Step(now, moves);" in freertos
    assert "BSP_BusServo_Service(now);" in freertos
    assert "if (APP_ServoFeedbackBench_IsActive() != 0U)" in freertos
    assert "rc_armed = 0U;" in freertos
    assert "App/Src/app_servo_feedback_bench.c" in cmake


def test_sweep_covers_requested_feedback_rates() -> None:
    source = read("App/Src/app_servo_feedback_bench.c")

    assert "10U, 25U, 50U, 75U, 100U, 125U, 150U, 175U, 200U" in source
    assert "APP_SERVO_FB_MOVE_PERIOD_MS      10U" in source
    assert "reply_permille" in source
    assert "achieved_hz_x100" in source
    assert "target_changes" in source
