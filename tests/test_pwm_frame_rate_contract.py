"""执行器出口帧率契约（2026-09-07 立，2026-09-10 随 MicoAir743v2 移植更新）。

原始动机是延迟，不是功能：四路 PWM 曾经共用 TIM2 的 50 Hz 帧，500 Hz 的角速率
环算 10 次只送得出去 1 次，零阶保持平均延迟 10 ms。这段死区是整条控制回路里最慢
的一环，也是任何非零 rate.kd 都发散的直接原因。

**这里锁的是"ESC 与舵机各用一个定时器、各自定帧率"这个性质，不是某两个定时器的
名字。** 移植到 MicoAir743v2 时 PA0~PA3 全被 UART4/USART2 占了，四路 PWM 只能搬到
电机焊盘（ESC→TIM1、舵机→TIM4）。当初写死 htim5/htim2 的断言在那次搬迁里除了逼人
改测试之外什么也没守住，所以现在改成从源码里推导出两个句柄再断言它们不同。

CubeMX 生成代码（Core/Src/tim.c）与 .ioc 是否同步，由
tests/test_micoair743v2_generated_code_sync.py 统一负责，不在本文件重复。
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")


def esc_and_servo_handles() -> tuple[str, str]:
    """从 bsp_pwm.c 里推出 ESC 与舵机各用哪个定时器句柄。"""
    bsp = read("BSP/Src/bsp_pwm.c")

    esc = re.search(
        r"HAL_TIM_PWM_Start\(&(htim\d+), pwm_esc_tim_channel\(channel\)\)", bsp
    )
    servo = re.search(
        r"HAL_TIM_PWM_Start\(&(htim\d+), pwm_servo_tim_channel\(channel\)\)", bsp
    )
    assert esc is not None, "找不到 ESC 的 PWM 启动调用"
    assert servo is not None, "找不到舵机的 PWM 启动调用"
    return esc.group(1), servo.group(1)


def test_actuator_pins_match_the_board() -> None:
    """引脚是硬件约束，必须与板级定义一致。

    MicoAir743v2 上 PA0/PA1 是 UART4、PA2/PA3 是 USART2，所以四路 PWM 落在电机焊盘：
        ESC   TIM1_CH1/CH2 → PE9 / PE11   (MOTOR4 / MOTOR3)
        舵机  TIM4_CH1/CH2 → PD12 / PD13  (MOTOR7 / MOTOR8)
    依据见 doc/micoair743v2/vendor/ardupilot-hwdef.dat。
    """
    ioc = read("drone-H743.ioc")

    assert "PE9.Signal=S_TIM1_CH1" in ioc
    assert "PE11.Signal=S_TIM1_CH2" in ioc
    assert "PD12.Signal=S_TIM4_CH1" in ioc
    assert "PD13.Signal=S_TIM4_CH2" in ioc

    # PA2/PA3 归光流串口，绝不能再被定时器占回去。
    for pin, owner in (("PA2", "USART2_TX"), ("PA3", "USART2_RX")):
        assert f"{pin}.Signal={owner}" in ioc

    # PA0/PA1 是板子的 UART4 焊盘（hwdef 有，PX4 当 TEL2 用）。ELRS 2026-09-10 搬到
    # 板载 RC 口 USART6 之后本工程不再启用 UART4，这两个脚在 .ioc 里没有分配——
    # 但它们仍然是串口焊盘，不许让定时器把它们抢去当 PWM。
    for pin in ("PA0", "PA1", "PA2", "PA3"):
        assert f"{pin}.Signal=S_TIM" not in ioc


def test_esc_and_servo_are_on_separate_timers() -> None:
    """两者分开才可能各自定帧率——这是整个改动的支点。"""
    bsp = read("BSP/Src/bsp_pwm.c")
    esc_handle, servo_handle = esc_and_servo_handles()

    assert esc_handle != servo_handle, (
        "ESC 与舵机共用一个定时器就只能共用一个 ARR，"
        "400 Hz 会被舵机的 50 Hz 拖回去"
    )

    assert f"__HAL_TIM_SET_COMPARE(&{esc_handle}, tim_channel, pulse_us);" in bsp
    assert f"__HAL_TIM_SET_COMPARE(&{esc_handle}, tim_channel, 0U);" in bsp
    assert f"__HAL_TIM_SET_COMPARE(&{servo_handle}, tim_channel, pulse_us);" in bsp

    # ESC 路径上不允许出现舵机的句柄，反之亦然。
    esc_setter = bsp.split("BSP_PWM_Status BSP_PWM_SetEscPulse")[1].split("\n}")[0]
    assert servo_handle not in esc_setter
    servo_setter = bsp.split("BSP_PWM_Status BSP_PWM_SetServoPulse")[1].split("\n}")[0]
    assert esc_handle not in servo_setter


def test_frame_rates_are_owned_by_bsp_and_fit_the_pulses() -> None:
    """CubeMX 的 Period 只是初值；帧率必须由 BSP 在 Init 里重设，否则一次重新
    生成就能把 400 Hz 悄悄改回 50 Hz。"""
    header = read("BSP/Inc/bsp_pwm.h")
    bsp = read("BSP/Src/bsp_pwm.c")
    esc_handle, servo_handle = esc_and_servo_handles()

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

    assert (
        f"__HAL_TIM_SET_AUTORELOAD(&{esc_handle}, BSP_PWM_ESC_FRAME_US - 1U);" in bsp
    )
    assert (
        f"__HAL_TIM_SET_AUTORELOAD(&{servo_handle}, BSP_PWM_SERVO_FRAME_US - 1U);"
        in bsp
    )
    # 必须先设 ARR 再启动，否则计数器可能已越过新 ARR。
    assert bsp.index(f"__HAL_TIM_SET_AUTORELOAD(&{esc_handle}") < bsp.index(
        f"HAL_TIM_PWM_Start(&{esc_handle}"
    )


def test_timer_prescaler_in_ioc_yields_the_bsp_tick() -> None:
    """时基归 CubeMX（Prescaler），帧长归 BSP（ARR）。两边对不上脉宽单位就不是 us。

    BSP 直接把微秒数写进 CCR，前提是定时器计数频率正好 1 MHz。定时器时钟 120 MHz，
    所以分频必须是 120-1。这一项写错不会报错，只会让所有脉宽整体缩放。
    """
    ioc = read("drone-H743.ioc")

    for timer in ("TIM1", "TIM4"):
        assert f"{timer}.Prescaler=120-1" in ioc, f"{timer} 分频必须给出 1 MHz 时基"

    assert "RCC.Tim1OutputFreq_Value=120000000" in ioc
    assert "RCC.Tim2OutputFreq_Value=120000000" in ioc


def test_esc_frame_delay_is_an_order_of_magnitude_better() -> None:
    """量化收益：ESC 的零阶保持平均延迟从 10 ms 降到 1.25 ms。"""
    header = read("BSP/Inc/bsp_pwm.h")
    esc_hz = int(re.search(r"#define\s+BSP_PWM_ESC_FRAME_HZ\s+(\d+)U", header).group(1))

    old_zoh_ms = 1000.0 / 50 / 2.0
    new_zoh_ms = 1000.0 / esc_hz / 2.0
    assert old_zoh_ms == 10.0
    assert new_zoh_ms == 1.25
    assert old_zoh_ms / new_zoh_ms == 8.0
