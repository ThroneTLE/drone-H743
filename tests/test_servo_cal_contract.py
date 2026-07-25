from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_servo_cal_uses_release_startup_save_and_restore_without_center_save() -> None:
    source = read("App/Src/app_servo_cal.c")
    cmake = read("CMakeLists.txt")

    assert "App/Src/app_servo_cal.c" in cmake
    assert "BSP_BusServo_ReleaseTorque(1U)" in source
    assert "BSP_BusServo_ReleaseTorque(2U)" in source
    assert "BSP_BusServo_SetStartupPosition(1U)" in source
    assert "BSP_BusServo_SetStartupPosition(2U)" in source
    assert "BSP_BusServo_RestoreTorque(1U)" in source
    assert "BSP_BusServo_RestoreTorque(2U)" in source
    assert "BSP_BusServo_SaveCenter" not in source
    assert 'APP_Control_QueueText("OK servo_cal released\\r\\n")' in source
    assert 'APP_Control_QueueText("OK servo_cal startup_saved\\r\\n")' in source


def test_servo_cal_requires_disarmed_low_throttle_rc_gate_and_corner_hold() -> None:
    source = read("App/Src/app_servo_cal.c")

    assert "#define APP_SERVO_CAL_HOLD_MS        800U" in source
    assert "rc_link_ok == 0U" in source
    assert "rc_arm_switch_high != 0U" in source
    assert "servo_cal_low(ch[APP_SERVO_CAL_CH_THROTTLE])" in source
    assert "servo_cal_low(ch[APP_SERVO_CAL_CH_YAW])" in source
    assert "servo_cal_high(ch[APP_SERVO_CAL_CH_YAW])" in source
    assert "servo_cal_high(ch[APP_SERVO_CAL_CH_ROLL])" in source
    assert "servo_cal_low(ch[APP_SERVO_CAL_CH_ROLL])" in source


def test_stabilizer_freezes_motors_and_skips_normal_servo_send_during_cal() -> None:
    freertos = read("Core/Src/freertos.c")

    assert '#include "app_servo_cal.h"' in freertos
    assert "APP_ServoCal_Init();" in freertos
    assert "APP_ServoCal_Step(ch, rc_link_ok, rc_arm_switch_high, now)" in freertos
    assert "servo_cal_active = APP_ServoCal_IsActive();" in freertos
    normal_servo_block = freertos[
        freertos.index("if (servo_cal_active == 0U) {"):
        freertos.index("#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)",
                       freertos.index("if (servo_cal_active == 0U) {"))
    ]
    assert "APP_ServoFeedbackBench_ApplyTargets(now, moves);" in normal_servo_block
    assert "stabilizer_servo_record_target(moves);" in normal_servo_block
    assert "BSP_BusServo_MoveManyAsync" in normal_servo_block
    assert "if (servo_cal_active != 0U) {\n          BSP_PWM_SetEscPulse(1, BSP_PWM_ESC_MIN_US);" in freertos
    assert "BSP_PWM_SetEscPulse(2, BSP_PWM_ESC_MIN_US);" in freertos


def test_led_servo_cal_mode_overrides_normal_status() -> None:
    header = read("App/Inc/app_led.h")
    source = read("App/Src/app_led.c")

    assert "APP_LED_SERVO_CAL_RELEASED" in header
    assert "APP_LED_SetServoCalMode(APP_LED_ServoCalMode mode)" in header
    assert "if (servo_cal_mode == APP_LED_SERVO_CAL_RELEASED)" in source
    assert "if (servo_cal_mode == APP_LED_SERVO_CAL_SAVE_ACK)" in source
    assert "if (servo_cal_mode == APP_LED_SERVO_CAL_ERROR)" in source
    assert source.index("if (servo_cal_mode == APP_LED_SERVO_CAL_RELEASED)") < source.index("heartbeat_period_ms =")
