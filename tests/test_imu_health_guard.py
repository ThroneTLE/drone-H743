"""IMU 采样链静默降级的检测与拦截。

背景（2026-08-28 实测）：Sensor_Task 在 DRDY 中断没来时会退到 20ms 轮询兜底。
兜底成功会把 imu_drdy_miss_count 清零，于是"中断永久失效"被当成"中断正常"，
整条链静默降到 50Hz 而没有任何报警——实际发生过一次，150 个 50Hz 样本原样
存进了标定目录，看起来跟正常数据毫无区别。

50Hz 不是性能打折：低通系数按 dt=1ms 烤死，等效截止频率从 80Hz 掉到 4Hz、
群延迟 2.8ms 涨到约 56ms；ICM42688 抗混叠配在 213Hz，而 50Hz 采样的 Nyquist
只有 25Hz，桨叶 150~300Hz 振动会整个折叠进姿态带。因此判据是"拒绝起飞"，
不是"提示性能偏低"。
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from tools import drone_tcp_panel as panel
from tools import v1_metrology_session as v1


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    end = source.find("\n    def ", start + len(signature))
    return source[start:] if end < 0 else source[start:end]


# --------------------------------------------------------------------------
# A. 中断健康与数据可得性必须分开统计
# --------------------------------------------------------------------------

def test_poll_fallback_no_longer_clears_the_drdy_miss_counter() -> None:
    """这是根因：轮询兜底成功不等于中断健康。"""
    freertos = read("Core/Src/freertos.c")
    start = freertos.index("if ((BSP_IMU_IsDataReady(&imu_ready) == DRV_IMU_OK) && imu_ready)")
    end = freertos.index("} else {", start)
    poll_branch = freertos[start:end]

    assert "imu_drdy_miss_count = 0U;" not in poll_branch
    assert "imu_drdy_miss_count++;" in poll_branch
    assert "APP_ImuHealth_NoteSample(0U);" in poll_branch
    # 兜底持续足够久必须真的锁存故障。
    assert "SENSOR_IMU_DRDY_MISS_FAULT_LIMIT" in poll_branch


def test_irq_branch_still_clears_the_counter_and_marks_the_sample() -> None:
    freertos = read("Core/Src/freertos.c")
    start = freertos.index("if ((imu_ready_flags & SENSOR_IMU_DATA_READY_FLAG) != 0U)")
    end = freertos.index("} else {", start)
    irq_branch = freertos[start:end]

    assert "imu_drdy_miss_count = 0U;" in irq_branch
    assert "APP_ImuHealth_NoteSample(1U);" in irq_branch


def test_good_read_only_clears_the_fault_when_the_irq_chain_is_healthy() -> None:
    """以前无条件清除，导致刚锁存的 DRDY_TIMEOUT 下一帧就被抹掉。"""
    freertos = read("Core/Src/freertos.c")
    # 锚在读取成功之后，避免匹配到函数开头的变量声明。
    read_ok = freertos.index("BSP_IMU_ReadRaw(&raw) != DRV_IMU_OK")
    start = freertos.index("imu_read_fail_count = 0U;", read_ok)
    window = freertos[start:start + 600]

    assert "if (imu_drdy_miss_count == 0U) {" in window
    guard = window.index("if (imu_drdy_miss_count == 0U) {")
    clear = window.index("APP_Stabilizer_ClearImuFault();")
    assert guard < clear


def test_health_module_is_built_and_initialised() -> None:
    assert "App/Src/app_imu_health.c" in read("CMakeLists.txt")
    freertos = read("Core/Src/freertos.c")
    assert '#include "app_imu_health.h"' in freertos
    assert "APP_ImuHealth_Init();" in freertos
    assert "APP_ImuHealth_Update(HAL_GetTick());" in freertos


def test_health_thresholds_match_the_documented_policy() -> None:
    header = read("App/Inc/app_imu_health.h")
    assert "#define APP_IMU_HEALTH_RATE_NORMAL_HZ   900U" in header
    assert "#define APP_IMU_HEALTH_RATE_FAIL_HZ     500U" in header
    # 连续 2 个 100ms 窗口 = 200ms 才定性，避免单次调度抖动误判。
    assert "#define APP_IMU_HEALTH_WINDOW_MS        100U" in header
    assert "#define APP_IMU_HEALTH_FAIL_WINDOWS     2U" in header


# --------------------------------------------------------------------------
# B/C. 失效必须阻止解锁，且原因不能被别的分支盖掉
# --------------------------------------------------------------------------

def test_arm_is_blocked_while_the_sampling_chain_is_failed() -> None:
    stabilizer = read("App/Src/app_stabilizer.c")
    arm = function_body(stabilizer, "  static uint8_t stabilizer_rc_update_armed(")

    assert "APP_ImuHealth_IsArmBlocked() != 0U" in arm
    assert "stabilizer_rc_arm_latched = 0U;" in arm


def test_arm_block_reason_reports_imu_before_the_arm_switch_branch() -> None:
    """失效会把 rc_armed 清零；若顺序错了会误报成"拨杆没打"，把真因藏起来。"""
    stabilizer = read("App/Src/app_stabilizer.c")
    start = stabilizer.index("if (frame->rc_link_seen == 0U) {")
    chain = stabilizer[start:start + 1400]

    health = chain.index("APP_ImuHealth_IsArmBlocked() != 0U")
    arm_switch = chain.index("frame->rc_armed == 0U")
    assert health < arm_switch


def test_health_reaches_the_host_through_the_imu_snapshot() -> None:
    assert "imu_health_level" in read("App/Inc/app_stabilizer.h")
    assert "imu_sample_rate_hz" in read("App/Inc/app_stabilizer.h")
    control = read("App/Src/app_control.c")
    assert "IMU health level=%u rate_hz=%u fault=%u fault_ever=%u" in control


# --------------------------------------------------------------------------
# D. 采集侧护栏
# --------------------------------------------------------------------------

def rows_at_rate(rate_hz: float, count: int = 200) -> list[dict[str, str]]:
    step_us = 1_000_000.0 / rate_hz
    return [{"timestamp_us": str(int(i * step_us))} for i in range(count)]


def test_measured_rate_matches_a_synthetic_capture() -> None:
    rate, gaps = v1.measure_capture_rate(rows_at_rate(1000.0))
    assert rate == pytest.approx(1000.0, rel=1e-3)
    assert gaps == 0.0


def test_fifty_hertz_capture_is_rejected() -> None:
    """正是 2026-08-28 那次真实事故的形态。"""
    with pytest.raises(ValueError) as excinfo:
        v1.validate_capture_sample_rate(rows_at_rate(50.0))

    message = str(excinfo.value)
    assert "50 Hz" in message
    assert "DRDY" in message


def test_nominal_one_khz_capture_is_accepted() -> None:
    assert v1.validate_capture_sample_rate(rows_at_rate(992.0)) == pytest.approx(992.0, rel=1e-2)


def test_capture_with_dropouts_is_rejected() -> None:
    rows = rows_at_rate(1000.0, count=200)
    stamps = [int(r["timestamp_us"]) for r in rows]
    # 10% 的间隔拉长到中位值的 10 倍。
    for index in range(20, 200, 10):
        for tail in range(index, 200):
            stamps[tail] += 9000
    rows = [{"timestamp_us": str(value)} for value in stamps]

    with pytest.raises(ValueError) as excinfo:
        v1.validate_capture_sample_rate(rows)
    assert "丢帧" in str(excinfo.value)


def test_capture_without_timestamps_is_rejected() -> None:
    with pytest.raises(ValueError):
        v1.validate_capture_sample_rate([{"timestamp_us": ""} for _ in range(50)])


def test_rate_guard_sits_on_the_single_evidence_funnel() -> None:
    source = read("tools/v1_metrology_session.py")
    body = source[source.index("def rows_to_metrology_samples("):]
    body = body[: body.index("\ndef ", 1)]
    assert "validate_capture_sample_rate(rows)" in body
    # 出处校验和速率校验都必须在构造样本之前完成。
    assert body.index("validate_capture_sample_rate(rows)") < body.index("MetrologySample(")


# --------------------------------------------------------------------------
# 上位机就绪条件
# --------------------------------------------------------------------------

def health_subject(**health: str) -> SimpleNamespace:
    import time as _time

    subject = SimpleNamespace(
        validation_imu_health=dict(health),
        validation_imu_health_time=_time.monotonic() if health else 0.0,
        IMU_HEALTH_LABELS=panel.DronePanel.IMU_HEALTH_LABELS,
    )
    return subject


def health_state(**health: str) -> tuple[str, str, bool]:
    return panel.DronePanel._validation_health_state(health_subject(**health))


def test_normal_health_allows_collection() -> None:
    state, text, ok = health_state(level="0", rate_hz="992", fault="0", fault_ever="0")
    assert (state, ok) == ("ok", True)
    assert "992 Hz" in text


def test_failed_health_blocks_collection() -> None:
    state, text, ok = health_state(level="2", rate_hz="50", fault="1", fault_ever="1")
    assert (state, ok) == ("bad", False)
    assert "禁止解锁" in text


def test_degraded_health_blocks_collection_too() -> None:
    """降级下的数据同样不可信，不能只在完全失效时才拦。"""
    _state, _text, ok = health_state(level="1", rate_hz="700", fault="0", fault_ever="0")
    assert ok is False


def test_recovered_chain_warns_that_earlier_evidence_is_suspect() -> None:
    state, text, ok = health_state(level="0", rate_hz="992", fault="0", fault_ever="1")
    assert (state, ok) == ("wait", True)
    assert "曾降级" in text


def test_old_firmware_without_health_does_not_block() -> None:
    """旧固件不带这行，不能因此把用户卡死。"""
    state, _text, ok = health_state()
    assert (state, ok) == ("wait", True)


def test_readiness_gate_consumes_the_health_state() -> None:
    source = read("tools/panel_lib/pages/validation_v0.py")
    body = function_body(source, "    def _validation_refresh_readiness(")
    assert "_validation_health_state()" in body
    assert '_validation_set_gate("health"' in body
    assert "and health_ok" in body
