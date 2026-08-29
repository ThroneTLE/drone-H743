from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_led_status_reports_arm_block_reasons_and_flow_health() -> None:
    header = read("App/Inc/app_led.h")
    source = read("App/Src/app_led.c")
    uart = read("App/Src/app_uart.c")
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert "APP_LED_ARM_BLOCK_NO_RC" in header
    assert "APP_LED_ARM_BLOCK_RC_LOSS" in header
    assert "APP_LED_ARM_BLOCK_ARM_SWITCH" in header
    assert "APP_LED_ARM_BLOCK_THROTTLE_HIGH" in header
    assert "APP_LED_ARM_BLOCK_IMU" in header
    assert "void APP_LED_SetArmStatus" in header
    assert "APP_OPTICAL_FLOW_HEALTH_OK" in source
    assert "APP_OPTICAL_FLOW_HEALTH_RETRYING" in source
    assert "APP_OPTICAL_FLOW_HEALTH_FAILED" in source
    assert "app_led_group_blink(now_ms, (uint8_t)reason)" in source
    assert "APP_LED_Task_Step();" in uart
    assert "APP_Task_LED_Init();" in freertos
    assert "APP_LED_SetArmStatus(frame->rc_armed, frame->led_arm_block_reason);" in freertos
