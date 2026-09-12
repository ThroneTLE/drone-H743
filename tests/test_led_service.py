"""RGB 状态灯的三层：效果算法（Driver）、仲裁（Service）、板级绑定（BSP）。

这一份和 `test_led_status_contract.py` 分工不同：那一份钉的是"解锁被拒的原因链
不能漏分支"，本文件钉的是"灯本身算得对不对、该听谁的听谁的、接线极性有没有写反"。

**极性为什么值得单独一条机检。** 2026-09-12 实机发现本板是共阳接法（写 0 点亮），
而固件按"写 1 点亮"驱动了整个移植期。极性错了不会让任何测试变红、也不会让灯不亮，
只会让每一个图案都反过来读——被拒"闪 1 下"在眼睛里变成"亮着偶尔黑一下"。
所以它必须是绑定里的一个显式常量，并且有一条断言盯着它。

harness 用宿主 gcc **真编译固件那两份 .c**，不在 Python 里重写一遍模型。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
DRIVER_INC = ROOT / "Driver" / "Inc"
DRIVER_SRC = ROOT / "Driver" / "Src"
SERVICES_INC = ROOT / "Services" / "Inc"
SERVICES_SRC = ROOT / "Services" / "Src"


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


HARNESS = r"""
#include "drv_rgb_led.h"
#include "svc_led.h"

#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { \
    if (!(condition)) { \
        fprintf(stderr, "check %d failed at line %d\n", (code), __LINE__); \
        return (code); \
    } \
} while (0)

static uint8_t sink_bits;
static uint32_t sink_calls;

static void test_sink(uint8_t bits)
{
    sink_bits = bits;
    ++sink_calls;
}

static DRV_RgbPattern solid(uint8_t r, uint8_t g, uint8_t b)
{
    DRV_RgbPattern p;
    memset(&p, 0, sizeof(p));
    p.color.r = r; p.color.g = g; p.color.b = b;
    p.effect = DRV_RGB_EFFECT_SOLID;
    return p;
}

/* ------------------------------------------------------ 效果算法 */

static int test_off_and_solid(void)
{
    DRV_RgbPattern p = solid(10U, 20U, 30U);
    DRV_RgbColor c;

    c = DRV_RgbLed_Sample(&p, 0U);
    CHECK(c.r == 10U && c.g == 20U && c.b == 30U, 10);
    c = DRV_RgbLed_Sample(&p, 123456U);
    CHECK(c.r == 10U && c.g == 20U && c.b == 30U, 11);

    p.effect = DRV_RGB_EFFECT_OFF;
    c = DRV_RgbLed_Sample(&p, 5U);
    CHECK(c.r == 0U && c.g == 0U && c.b == 0U, 12);

    /* 空指针不该把整机带走：状态灯是最不该因为自己出事的东西。 */
    c = DRV_RgbLed_Sample(NULL, 0U);
    CHECK(c.r == 0U && c.g == 0U && c.b == 0U, 13);
    return 0;
}

static int test_blink_duty_and_period(void)
{
    DRV_RgbPattern p = solid(0U, 0U, 255U);
    uint32_t lit = 0U;
    uint32_t t;

    p.effect = DRV_RGB_EFFECT_BLINK;
    p.on_ms = 120U;
    p.off_ms = 380U;

    for (t = 0U; t < 500U; ++t) {
        if (DRV_RgbLed_Sample(&p, t).b != 0U) { ++lit; }
    }
    CHECK(lit == 120U, 20);
    /* 下一个周期必须重复，相位只跟 now_ms 有关。 */
    CHECK(DRV_RgbLed_Sample(&p, 500U).b == DRV_RgbLed_Sample(&p, 0U).b, 21);
    CHECK(DRV_RgbLed_Sample(&p, 619U).b != 0U, 22);
    CHECK(DRV_RgbLed_Sample(&p, 620U).b == 0U, 23);
    return 0;
}

static int test_pulse_count_is_countable(void)
{
    /*
     * 数灯是现场唯一的读法，所以"闪 N 下"必须真的是 N 个上升沿，而且组间那段
     * 黑要足够长——否则 3 下和 4 下在眼睛里分不开。
     */
    uint8_t count;

    for (count = 1U; count <= 7U; ++count) {
        DRV_RgbPattern p = solid(255U, 110U, 0U);
        uint32_t cycle;
        uint32_t t;
        uint32_t edges = 0U;
        uint8_t previous = 0U;
        uint32_t longest_gap = 0U;
        uint32_t gap = 0U;

        p.effect = DRV_RGB_EFFECT_PULSES;
        p.on_ms = 160U; p.off_ms = 160U; p.gap_ms = 760U; p.count = count;
        cycle = ((uint32_t)count * 320U) + 760U;

        for (t = 0U; t < cycle; ++t) {
            uint8_t on = (DRV_RgbLed_Sample(&p, t).r != 0U) ? 1U : 0U;
            if ((on != 0U) && (previous == 0U)) { ++edges; }
            if (on == 0U) { ++gap; if (gap > longest_gap) { longest_gap = gap; } }
            else { gap = 0U; }
            previous = on;
        }
        CHECK(edges == count, 30);
        /* 组间停顿必须比组内的灭长，不然断不了句。 */
        CHECK(longest_gap >= 760U, 31);
    }
    return 0;
}

static int test_pulses_with_zero_count_stay_dark(void)
{
    DRV_RgbPattern p = solid(255U, 0U, 0U);

    p.effect = DRV_RGB_EFFECT_PULSES;
    p.on_ms = 160U; p.off_ms = 160U; p.gap_ms = 760U; p.count = 0U;
    CHECK(DRV_RgbLed_Sample(&p, 0U).r == 0U, 40);
    CHECK(DRV_RgbLed_Sample(&p, 77U).r == 0U, 41);
    return 0;
}

static int test_breathe_reaches_both_ends_and_is_symmetric(void)
{
    DRV_RgbPattern p = solid(0U, 255U, 0U);
    uint32_t t;
    uint8_t peak = 0U;
    uint8_t trough = 255U;

    p.effect = DRV_RGB_EFFECT_BREATHE;
    p.period_ms = 2000U;
    p.dim = 0U;

    for (t = 0U; t < 2000U; ++t) {
        uint8_t v = DRV_RgbLed_Sample(&p, t).g;
        if (v > peak) { peak = v; }
        if (v < trough) { trough = v; }
    }
    CHECK(peak == 255U, 50);
    CHECK(trough == 0U, 51);
    /* 上升沿与下降沿对称：同一条曲线走两遍，顶点和谷底都不许跳。 */
    CHECK(DRV_RgbLed_Sample(&p, 500U).g == DRV_RgbLed_Sample(&p, 1500U).g, 52);
    CHECK(DRV_RgbLed_Sample(&p, 1000U).g == 255U, 53);

    /* dim 是"最暗处还留多少"，用来避免呼吸到全黑时像灯坏了。 */
    p.dim = 40U;
    trough = 255U;
    for (t = 0U; t < 2000U; ++t) {
        uint8_t v = DRV_RgbLed_Sample(&p, t).g;
        if (v < trough) { trough = v; }
    }
    CHECK(trough == 40U, 54);
    return 0;
}

static int test_breathe_spends_real_time_dim(void)
{
    /*
     * 线性扫的呼吸看起来是"一直很亮、突然暗一下"，因为眼睛对亮度接近平方响应。
     * 驱动里做了 gamma，所以低段必须占到足够多的时间——这条就是钉那个 gamma 在。
     */
    DRV_RgbPattern p = solid(0U, 255U, 0U);
    uint32_t t;
    uint32_t below_half = 0U;

    p.effect = DRV_RGB_EFFECT_BREATHE;
    p.period_ms = 2000U;
    for (t = 0U; t < 2000U; ++t) {
        if (DRV_RgbLed_Sample(&p, t).g < 128U) { ++below_half; }
    }
    CHECK(below_half > 1300U, 60);   /* 线性曲线这里只会是 1000 */
    return 0;
}

/* ------------------------------------------------------ Σ-Δ 调制 */

static int test_modulator_hits_the_rails_exactly(void)
{
    DRV_RgbDither state;
    DRV_RgbColor full = {255U, 255U, 255U};
    DRV_RgbColor dark = {0U, 0U, 0U};
    uint32_t t;

    DRV_RgbLed_DitherReset(&state);
    for (t = 0U; t < 1000U; ++t) {
        CHECK(DRV_RgbLed_Modulate(&state, full) == 0x07U, 70);
    }
    DRV_RgbLed_DitherReset(&state);
    for (t = 0U; t < 1000U; ++t) {
        CHECK(DRV_RgbLed_Modulate(&state, dark) == 0x00U, 71);
    }
    return 0;
}

static int test_modulator_average_tracks_the_level(void)
{
    static const uint8_t levels[5] = {1U, 32U, 128U, 200U, 254U};
    uint8_t index;

    for (index = 0U; index < 5U; ++index) {
        DRV_RgbDither state;
        DRV_RgbColor color = {0U, 0U, 0U};
        uint32_t lit = 0U;
        uint32_t t;
        uint32_t expected;
        uint32_t error;

        color.r = levels[index];
        DRV_RgbLed_DitherReset(&state);
        for (t = 0U; t < 2550U; ++t) {
            if ((DRV_RgbLed_Modulate(&state, color) & DRV_RGB_BIT_R) != 0U) { ++lit; }
        }
        expected = ((uint32_t)levels[index] * 2550U) / 255U;
        error = (lit > expected) ? (lit - expected) : (expected - lit);
        CHECK(error <= 2U, 80);
    }
    return 0;
}

static int test_half_brightness_alternates_every_tick(void)
{
    /*
     * 这是 Σ-Δ 相对普通 PWM 的全部意义：128/255 不是"亮 128 拍再灭 127 拍"
     * （那在 1 kHz 下就是 3.9 Hz 的肉眼闪烁），而是一拍一换，等效 500 Hz。
     */
    DRV_RgbDither state;
    DRV_RgbColor color = {128U, 0U, 0U};
    uint32_t t;
    uint32_t runs = 0U;
    uint8_t previous = 0xFFU;

    DRV_RgbLed_DitherReset(&state);
    for (t = 0U; t < 100U; ++t) {
        uint8_t on = ((DRV_RgbLed_Modulate(&state, color) & DRV_RGB_BIT_R) != 0U) ? 1U : 0U;
        if (on != previous) { ++runs; }
        previous = on;
    }
    CHECK(runs >= 95U, 90);
    return 0;
}

static int test_going_dark_clears_the_residue(void)
{
    /*
     * 亮度归零时若不清累加器，余量会在下次点亮的第一拍提前触发，表现为一次
     * 多余的闪——在"闪 N 下"的图案里那就是数错。
     */
    DRV_RgbDither state;
    DRV_RgbColor mid = {200U, 0U, 0U};
    DRV_RgbColor dark = {0U, 0U, 0U};
    DRV_RgbColor faint = {1U, 0U, 0U};

    DRV_RgbLed_DitherReset(&state);
    (void)DRV_RgbLed_Modulate(&state, mid);
    (void)DRV_RgbLed_Modulate(&state, dark);
    CHECK((DRV_RgbLed_Modulate(&state, faint) & DRV_RGB_BIT_R) == 0U, 100);
    return 0;
}

/* ------------------------------------------------------ 仲裁 */

static int test_lower_enum_wins(void)
{
    DRV_RgbPattern red = solid(255U, 0U, 0U);
    DRV_RgbPattern green = solid(0U, 255U, 0U);

    SVC_Led_Init(test_sink);
    CHECK(SVC_Led_ActiveSource() == SVC_LED_SOURCE_COUNT, 110);

    SVC_Led_Publish(SVC_LED_SOURCE_STATUS, &green);
    SVC_Led_Tick(0U);
    CHECK(SVC_Led_ActiveSource() == SVC_LED_SOURCE_STATUS, 111);
    CHECK(SVC_Led_ActiveColor().g == 255U, 112);

    SVC_Led_Publish(SVC_LED_SOURCE_ARMED, &red);
    SVC_Led_Tick(1U);
    CHECK(SVC_Led_ActiveSource() == SVC_LED_SOURCE_ARMED, 113);
    CHECK(SVC_Led_ActiveColor().r == 255U, 114);

    /* 撤销高优先级的，低优先级的必须自己重新露出来，不需要它再发布一次。 */
    SVC_Led_Publish(SVC_LED_SOURCE_ARMED, NULL);
    SVC_Led_Tick(2U);
    CHECK(SVC_Led_ActiveSource() == SVC_LED_SOURCE_STATUS, 115);
    return 0;
}

static int test_identify_beats_everything(void)
{
    DRV_RgbPattern white = solid(255U, 255U, 255U);
    DRV_RgbPattern red = solid(255U, 0U, 0U);
    uint8_t source;

    SVC_Led_Init(test_sink);
    for (source = 1U; source < (uint8_t)SVC_LED_SOURCE_COUNT; ++source) {
        SVC_Led_Publish((SVC_LedSource)source, &red);
    }
    SVC_Led_Publish(SVC_LED_SOURCE_IDENTIFY, &white);
    SVC_Led_Tick(0U);
    CHECK(SVC_Led_ActiveSource() == SVC_LED_SOURCE_IDENTIFY, 120);
    return 0;
}

static int test_timed_publish_expires_on_its_own(void)
{
    DRV_RgbPattern white = solid(255U, 255U, 255U);
    DRV_RgbPattern green = solid(0U, 255U, 0U);

    SVC_Led_Init(test_sink);
    SVC_Led_Publish(SVC_LED_SOURCE_STATUS, &green);
    SVC_Led_PublishFor(SVC_LED_SOURCE_IDENTIFY, &white, 1000U, 10000U);

    SVC_Led_Tick(10999U);
    CHECK(SVC_Led_ActiveSource() == SVC_LED_SOURCE_IDENTIFY, 130);
    SVC_Led_Tick(11000U);
    CHECK(SVC_Led_ActiveSource() == SVC_LED_SOURCE_STATUS, 131);
    return 0;
}

static int test_timed_publish_survives_the_millisecond_wrap(void)
{
    /*
     * 直接写 now >= expires 的话，在毫秒计数回绕那一刻发布的点名会卡住约 49 天。
     * 跑不到这条的人不会知道，因为板子很少连开这么久——但连开这么久的那次
     * 正好是最不想让灯停在测试色上的那次。
     */
    DRV_RgbPattern white = solid(255U, 255U, 255U);
    DRV_RgbPattern green = solid(0U, 255U, 0U);
    uint32_t near_wrap = 0xFFFFFF00U;

    SVC_Led_Init(test_sink);
    SVC_Led_Publish(SVC_LED_SOURCE_STATUS, &green);
    SVC_Led_PublishFor(SVC_LED_SOURCE_IDENTIFY, &white, 512U, near_wrap);

    SVC_Led_Tick(near_wrap + 511U);          /* 已经绕过 0 了 */
    CHECK(SVC_Led_ActiveSource() == SVC_LED_SOURCE_IDENTIFY, 140);
    SVC_Led_Tick(near_wrap + 512U);
    CHECK(SVC_Led_ActiveSource() == SVC_LED_SOURCE_STATUS, 141);
    return 0;
}

static int test_tick_always_drives_the_sink(void)
{
    SVC_Led_Init(test_sink);
    sink_calls = 0U;
    SVC_Led_Tick(0U);
    CHECK(sink_calls == 1U, 150);
    /* 没人说话时也要写一次——把灯写灭，而不是让它停在上一次的电平上。 */
    CHECK(sink_bits == 0x00U, 151);
    return 0;
}

static int test_a_service_without_a_sink_does_not_crash(void)
{
    DRV_RgbPattern red = solid(255U, 0U, 0U);

    SVC_Led_Init(NULL);
    SVC_Led_Publish(SVC_LED_SOURCE_ARMED, &red);
    SVC_Led_Tick(0U);
    CHECK(SVC_Led_ActiveSource() == SVC_LED_SOURCE_ARMED, 160);
    return 0;
}

static int test_out_of_range_source_is_ignored(void)
{
    DRV_RgbPattern red = solid(255U, 0U, 0U);

    SVC_Led_Init(test_sink);
    SVC_Led_Publish(SVC_LED_SOURCE_COUNT, &red);
    SVC_Led_Tick(0U);
    CHECK(SVC_Led_ActiveSource() == SVC_LED_SOURCE_COUNT, 170);
    return 0;
}

int main(void)
{
    int rc;

    rc = test_off_and_solid(); if (rc) { return rc; }
    rc = test_blink_duty_and_period(); if (rc) { return rc; }
    rc = test_pulse_count_is_countable(); if (rc) { return rc; }
    rc = test_pulses_with_zero_count_stay_dark(); if (rc) { return rc; }
    rc = test_breathe_reaches_both_ends_and_is_symmetric(); if (rc) { return rc; }
    rc = test_breathe_spends_real_time_dim(); if (rc) { return rc; }
    rc = test_modulator_hits_the_rails_exactly(); if (rc) { return rc; }
    rc = test_modulator_average_tracks_the_level(); if (rc) { return rc; }
    rc = test_half_brightness_alternates_every_tick(); if (rc) { return rc; }
    rc = test_going_dark_clears_the_residue(); if (rc) { return rc; }
    rc = test_lower_enum_wins(); if (rc) { return rc; }
    rc = test_identify_beats_everything(); if (rc) { return rc; }
    rc = test_timed_publish_expires_on_its_own(); if (rc) { return rc; }
    rc = test_timed_publish_survives_the_millisecond_wrap(); if (rc) { return rc; }
    rc = test_tick_always_drives_the_sink(); if (rc) { return rc; }
    rc = test_a_service_without_a_sink_does_not_crash(); if (rc) { return rc; }
    rc = test_out_of_range_source_is_ignored(); if (rc) { return rc; }

    printf("led harness ok\n");
    return 0;
}
"""


def test_rgb_led_driver_and_service_on_host_gcc(tmp_path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "harness.c"
    harness.write_text(HARNESS, encoding="utf-8")
    executable = tmp_path / "harness.exe"
    subprocess.run(
        [
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            f"-I{DRIVER_INC}", f"-I{SERVICES_INC}",
            str(DRIVER_SRC / "drv_rgb_led.c"),
            str(SERVICES_SRC / "svc_led.c"),
            str(harness), "-o", str(executable),
        ],
        check=True, capture_output=True, text=True,
    )
    result = subprocess.run([str(executable)], check=True, capture_output=True)
    assert "led harness ok" in result.stdout.decode("ascii", "replace")


def test_the_driver_and_service_stay_free_of_hardware_and_rtos() -> None:
    """它们一旦 include 了 HAL 或 RTOS，上面那个 harness 就编不过了。

    这正是把效果算法和仲裁从 `app_led.c` 拆出来的目的：颜色、节奏、优先级是会被
    反复调的，而调它不该需要烧一次板子才知道对不对。
    """
    for path in ("Driver/Src/drv_rgb_led.c", "Driver/Inc/drv_rgb_led.h",
                 "Services/Src/svc_led.c", "Services/Inc/svc_led.h"):
        text = read(path)
        for banned in ("stm32h7xx_hal", "main.h", "cmsis_os", "HAL_", "__disable_irq"):
            assert banned not in text, f"{path} 引入了 {banned}"


def test_the_board_polarity_is_an_explicit_binding() -> None:
    """MicoAir743v2 是共阳：写 0 点亮。

    这条实测出来的事实必须以常量形式留在绑定里。极性写反不会让任何测试变红、
    也不会让灯不亮，只会让每一个图案都反过来读——"闪 1 下"在眼睛里变成
    "亮着偶尔黑一下"，而那看起来完全像是一个正常的指示。
    """
    source = read("BSP/Src/bsp_rgb_led.c")

    assert "#define BSP_RGB_LED_ACTIVE_LOW 1U" in source
    assert "BSP_GPIO_PE3" in source and "BSP_GPIO_PE2" in source and "BSP_GPIO_PE4" in source
    # 极性只准在这一个文件里出现一次，别的层不许自己再取反一遍。
    for path in ("App/Src/app_led.c", "Services/Src/svc_led.c",
                 "Driver/Src/drv_rgb_led.c"):
        assert "ACTIVE_LOW" not in read(path), path


def test_the_led_task_is_not_scheduled_below_the_other_periodic_tasks() -> None:
    """灯的节拍任务不能排在会长期 Ready 的任务下面。

    2026-09-12 实测：按 `osPriorityLow` 建的 LED 任务 `ticks=0`——任务建起来了、
    `RTOS?` 报 state=1(Ready)，**一整拍都没跑过**。Low 低于 messageTask /
    backgroundTask / TELEM（都是 BelowNormal），而 backgroundTask 长期 Ready。

    这和 2026-09-11 遥测任务是同一个坑（`freertos.c` 里 VOFA_Task 的注释写着
    "stream=1 却一个字节都不来"）。灯出这个问题比遥测更难发现：遥测不来一眼看得出，
    灯停了只是僵在某个电平上，看起来完全像一盏正常亮着的灯。

    所以这条钉的是"不许是 Low"，不是"必须等于某个值"——以后要提到 Normal 也行。
    """
    tasks = read("App/Src/app_tasks.c")
    start = tasks.index("LEDTask_attributes")
    block = tasks[start:tasks.index("};", start)]

    assert "osPriorityLow" not in block, (
        "LED 任务不能用 osPriorityLow：backgroundTask 在 BelowNormal 上长期 Ready，"
        "Low 永远轮不到，实测 ticks 一直是 0"
    )
    assert "osPriorityBelowNormal" in block or "osPriorityNormal" in block

    # 栈余量也栽过一次：128 字时 RTOS? 报 free_stack_words=18。
    assert ".stack_size = 256 * 4" in block

    # 节拍用 osDelayUntil 而不是 osDelay：调光靠节拍密度，被挤掉多久误差就累多久。
    assert "osDelayUntil" in tasks


def test_the_led_tick_counter_is_reported() -> None:
    """`LED?` 必须报节拍计数。

    没有它，"灯没在动"有两种完全不同的原因分不开：**没人有话说**（src=idle，
    正常）和**任务根本没跑**（ticks 不涨）。上面那个 ticks=0 的缺陷就是靠它一眼
    定位的——在此之前只能看到一个说得通的 `src=idle`。
    """
    assert "ticks=%lu" in read("App/Src/app_cmd_led.c")
    assert "debug->ticks = app_led_ticks;" in read("App/Src/app_led.c")
    # 任务也要出现在 RTOS? 的任务表里，否则看不到它是 Ready 还是 Blocked。
    assert 'app_control_report_task_stack("LED", LEDTaskHandle);' in read(
        "App/Src/app_cmd_system.c")


def test_the_legacy_binary_led_api_is_gone() -> None:
    """旧的 `bsp_led.c` 已退役——它的引脚表还占着 PE3/PE2/PE4。

    两个模块同时认领同一组引脚，平时看不出来（旧 API 的调用点全被
    `*_TX_LED_ENABLED 0U` 编译掉了），出事时表现为"灯不听话"，而两边代码单看都对。
    """
    assert not (ROOT / "BSP" / "Src" / "bsp_led.c").exists()
    assert not (ROOT / "BSP" / "Inc" / "bsp_led.h").exists()
    for path in ("App/Src/app_uart.c", "BSP/Src/bsp_uart.c", "BSP/Src/bsp.c"):
        assert "bsp_led.h" not in read(path), path
        assert "BSP_LED_" not in read(path), path
