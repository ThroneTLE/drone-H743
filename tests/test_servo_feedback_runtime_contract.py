from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_runtime_feedback_uses_deterministic_50_hz_per_servo_slots() -> None:
    source = read("App/Src/app_servo_feedback.c")
    freertos = read("Core/Src/freertos.c")
    cmake = read("CMakeLists.txt")

    assert "APP_SERVO_FEEDBACK_FRAME_PERIOD_MS 10U" in source
    assert "APP_SERVO_FEEDBACK_QUERY_PHASE_MS   4U" in source
    assert "APP_SERVO_FEEDBACK_TIMEOUT_MS       5U" in source
    assert "APP_SERVO_FEEDBACK_STALE_MS         250U" in source
    assert "frame_start_ms = now_ms -" in source
    assert "servo_feedback_ctx.next_slot ^= 1U;" in source
    assert "BSP_BusServo_IsIdle() == 0U" in source
    assert "BSP_BusServo_RequestPositionAsync" in source
    assert "App/Src/app_servo_feedback.c" in cmake
    assert freertos.index("BSP_BusServo_MoveManyAsync") < freertos.index(
        "APP_ServoFeedback_Service("
    )
    assert "APP_ServoFeedbackBench_IsActive() == 0U" in freertos


def test_runtime_feedback_snapshot_carries_quality_metadata() -> None:
    header = read("App/Inc/app_servo_feedback.h")
    flight_header = read("App/Inc/app_flight_log.h")
    freertos = read("Core/Src/freertos.c")

    assert "uint16_t position_us[APP_SERVO_FEEDBACK_SLOT_COUNT];" in header
    assert "uint16_t age_ms[APP_SERVO_FEEDBACK_SLOT_COUNT];" in header
    assert "uint16_t sample_sequence[APP_SERVO_FEEDBACK_SLOT_COUNT];" in header
    assert "uint8_t valid_mask;" in header
    assert "uint32_t request_count;" in header
    assert "uint32_t response_count;" in header
    assert "uint32_t busy_count;" in header
    assert "servo_alpha_feedback_us" in flight_header
    assert "servo_beta_feedback_sequence" in flight_header
    assert "APP_ServoFeedback_GetLogSample(now, &servo_feedback_sample);" in freertos
