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
    # 事件文本经通告缓冲由通信任务补发，不允许在 500Hz 状态机里直接阻塞发送
    # （见 tests/test_control_loop_blocking_contract.py）。
    assert 'servo_cal_post_notice("OK servo_cal released\\r\\n")' in source
    assert 'servo_cal_post_notice("OK servo_cal startup_saved\\r\\n")' in source


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
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert '#include "app_servo_cal.h"' in freertos
    assert "APP_ServoCal_Init();" in freertos
    assert "APP_ServoCal_Step(frame->ch, frame->rc_link_ok, frame->rc_arm_switch_high, frame->now_ms)" in freertos
    assert "servo_cal_active = APP_ServoCal_IsActive();" in freertos
    normal_servo_block = freertos[
        freertos.index("if (frame->servo_cal_active == 0U) {"):
        freertos.index("if (frame->servo_cal_active != 0U) {",
                       freertos.index("if (frame->servo_cal_active == 0U) {"))
    ]
    assert "APP_ServoFeedbackBench_ApplyTargets(frame->now_ms, frame->moves);" in normal_servo_block
    assert "stabilizer_servo_record_target(frame->moves);" in normal_servo_block
    assert "BSP_BusServo_MoveManyAsync" in normal_servo_block
    assert "if (frame->servo_cal_active != 0U) {\n    BSP_PWM_SetEscPulse(1, BSP_PWM_ESC_MIN_US);" in freertos
    assert "BSP_PWM_SetEscPulse(2, BSP_PWM_ESC_MIN_US);" in freertos


def test_led_servo_cal_mode_overrides_normal_status() -> None:
    """标定期间灯必须跟着标定流程走，不能被常态状态盖掉。

    原来这条钉的是 `app_led.c` 里两条语句的**先后顺序**——标定的早返回要排在
    心跳赋值之前。RGB 重构之后压制关系不再靠语句顺序表达，而是由
    `SVC_LedSource` 的枚举顺序决定：仲裁取第一个有话说的源。所以断言改钉那个
    顺序本身，它对**每一个**源都成立，比钉一个函数里的两行强。
    """
    header = read("App/Inc/app_led.h")
    source = read("App/Src/app_led.c")
    service = read("Services/Inc/svc_led.h")

    assert "APP_LED_SERVO_CAL_RELEASED" in header
    assert "APP_LED_SetServoCalMode(APP_LED_ServoCalMode mode)" in header
    for mode in ("APP_LED_SERVO_CAL_RELEASED", "APP_LED_SERVO_CAL_SAVE_ACK",
                 "APP_LED_SERVO_CAL_ERROR"):
        assert f"case {mode}:" in source, mode
    # 标定发布在 CALIBRATION 源上，而该源排在所有常态源之前。
    assert "SVC_Led_Publish(SVC_LED_SOURCE_CALIBRATION, &pattern)" in source
    calibration_at = service.index("SVC_LED_SOURCE_CALIBRATION")
    for lower in ("SVC_LED_SOURCE_BLOCKED", "SVC_LED_SOURCE_WARNING",
                  "SVC_LED_SOURCE_STATUS", "SVC_LED_SOURCE_HEARTBEAT"):
        assert calibration_at < service.index(lower), (
            f"{lower} 排到了 CALIBRATION 前面，标定期间的灯会被它盖掉"
        )
    # 唯一该压过标定的是人工点名——人正盯着灯找是哪块板。
    assert service.index("SVC_LED_SOURCE_IDENTIFY") < calibration_at
