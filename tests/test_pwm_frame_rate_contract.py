"""执行器出口帧率契约（2026-09-07）。

改动的动机是延迟，不是功能：四路 PWM 原来共用 TIM2 的 50 Hz 帧，500 Hz 的角速率
环算 10 次只送得出去 1 次，零阶保持平均延迟 10 ms。这段死区是整条控制回路里最慢
的一环，也是任何非零 rate.kd 都发散的直接原因。

这里锁住三件事，因为它们分散在 CubeMX 生成代码、BSP 和引脚复用里，任何一处单独
回退都会静默地把延迟还回去：
  1. PA0~PA3 的**物理引脚不变**（硬件约束，外部接线不可改）；
  2. ESC 由 TIM5 驱动（AF2）、舵机由 TIM2 驱动（AF1）；
  3. 帧率归 BSP 所有，且一帧装得下最长脉冲。
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")


def test_actuator_pins_are_unchanged() -> None:
    """引脚是硬件约束：只允许换片内复用，不允许换脚。"""
    ioc = read("drone-H743.ioc")
    assert "PA0.Signal=S_TIM5_CH1" in ioc
    assert "PA1.Signal=S_TIM5_CH2" in ioc
    assert "PA2.Signal=S_TIM2_CH3" in ioc
    assert "PA3.Signal=S_TIM2_CH4" in ioc

    tim = read("Core/Src/tim.c")
    # 舵机：PA2/PA3 仍是 TIM2 的 AF1。
    assert "PA2     ------> TIM2_CH3" in tim
    assert "PA3     ------> TIM2_CH4" in tim
    assert "GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;" in tim
    # ESC：PA0/PA1 换成 TIM5 的 AF2，脚号不变。
    assert "PA0     ------> TIM5_CH1" in tim
    assert "PA1     ------> TIM5_CH2" in tim
    assert "GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;" in tim
    assert "GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;" in tim
    assert "GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;" in tim


def test_esc_and_servo_are_on_separate_timers() -> None:
    """两者分开才可能各自定帧率——这是整个改动的支点。"""
    bsp = read("BSP/Src/bsp_pwm.c")
    assert "HAL_TIM_PWM_Start(&htim5, pwm_esc_tim_channel(channel))" in bsp
    assert "HAL_TIM_PWM_Start(&htim2, pwm_servo_tim_channel(channel))" in bsp
    assert "__HAL_TIM_SET_COMPARE(&htim5, tim_channel, pulse_us);" in bsp
    assert "__HAL_TIM_SET_COMPARE(&htim5, tim_channel, 0U);" in bsp
    assert "__HAL_TIM_SET_COMPARE(&htim2, tim_channel, pulse_us);" in bsp
    # ESC 路径上不允许再出现 htim2。
    esc_setter = bsp.split("BSP_PWM_Status BSP_PWM_SetEscPulse")[1].split("\n}")[0]
    assert "htim2" not in esc_setter

    tim = read("Core/Src/tim.c")
    assert "void MX_TIM5_Init(void)" in tim
    assert "TIM_HandleTypeDef htim5;" in tim
    assert "__HAL_RCC_TIM5_CLK_ENABLE();" in tim
    assert "MX_TIM5_Init();" in read("Core/Src/main.c")
    assert "void MX_TIM5_Init(void);" in read("Core/Inc/tim.h")
    # TIM2 不再配置 CH1/CH2，否则 PA0/PA1 会被两个定时器同时驱动。
    tim2_init = tim.split("void MX_TIM2_Init(void)")[1].split("\n}")[0]
    assert "TIM_CHANNEL_1" not in tim2_init
    assert "TIM_CHANNEL_2" not in tim2_init
    assert "TIM_CHANNEL_3" in tim2_init
    assert "TIM_CHANNEL_4" in tim2_init


def test_frame_rates_are_owned_by_bsp_and_fit_the_pulses() -> None:
    """CubeMX 的 Period 只是初值；帧率必须由 BSP 在 Init 里重设，否则一次重新
    生成就能把 400 Hz 悄悄改回 50 Hz。"""
    header = read("BSP/Inc/bsp_pwm.h")
    bsp = read("BSP/Src/bsp_pwm.c")

    def macro(name: str) -> int:
        m = re.search(rf"#define\s+{name}\s+(\d+)U", header)
        assert m is not None, name
        return int(m.group(1))

    tick_hz = macro("BSP_PWM_TIMER_TICK_HZ")
    esc_hz = macro("BSP_PWM_ESC_FRAME_HZ")
    servo_hz = macro("BSP_PWM_SERVO_FRAME_HZ")
    esc_max_us = macro("BSP_PWM_ESC_MAX_US")
    servo_max_us = macro("BSP_PWM_SERVO_MAX_US")

    assert tick_hz == 1_000_000, "脉宽按 us 直接写 CCR，时基必须是 1 MHz"
    assert esc_hz == 400
    assert servo_hz == 50

    esc_frame_us = tick_hz // esc_hz
    servo_frame_us = tick_hz // servo_hz
    assert esc_frame_us > esc_max_us, "一帧装不下最长脉冲会退化成常高电平"
    assert servo_frame_us > servo_max_us

    assert "__HAL_TIM_SET_AUTORELOAD(&htim5, BSP_PWM_ESC_FRAME_US - 1U);" in bsp
    assert "__HAL_TIM_SET_AUTORELOAD(&htim2, BSP_PWM_SERVO_FRAME_US - 1U);" in bsp
    # 必须先设 ARR 再启动，否则计数器可能已越过新 ARR。
    assert bsp.index("__HAL_TIM_SET_AUTORELOAD(&htim5") < bsp.index(
        "HAL_TIM_PWM_Start(&htim5"
    )


def test_esc_frame_delay_is_an_order_of_magnitude_better() -> None:
    """量化收益：ESC 的零阶保持平均延迟从 10 ms 降到 1.25 ms。"""
    header = read("BSP/Inc/bsp_pwm.h")
    esc_hz = int(re.search(r"#define\s+BSP_PWM_ESC_FRAME_HZ\s+(\d+)U", header).group(1))

    old_zoh_ms = 1000.0 / 50 / 2.0
    new_zoh_ms = 1000.0 / esc_hz / 2.0
    assert old_zoh_ms == 10.0
    assert new_zoh_ms == 1.25
    assert old_zoh_ms / new_zoh_ms == 8.0
