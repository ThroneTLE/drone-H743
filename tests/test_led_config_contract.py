"""状态灯颜色绑定表（`App/*/app_led_config.*`）的契约。

这份配置是 **Flash ABI**：它整体嵌在 CFG 记录里，存在用户板子上。所以本文件钉的
不是"代码长什么样"，而是三件出了错不会报警的事：

1. **默认值必须逐位等于改造前的硬编码常量。** 否则升一次固件，现场那套
   "红=解锁 / 绿呼吸=就绪 / 琥珀数闪=被拒"的语言悄悄变了，而文档还是旧的。
2. **闪烁次数不许进配置。** 它就是解锁被拒的原因码。一旦可配，现场数到 3 下、
   而 `ARM?` 报 block=5，两边单看都自洽，排查的人会照着假原因一路查下去。
3. **校验要拦住那些"驱动会替你改掉"的值。** `drv_rgb_led.c` 把 period_ms=0 换成
   2000、on/off_ms=0 换成 1 ms。存进去一个 0、界面显示 0、灯却按 2000 走——
   三者互相矛盾，而没有任何一处报错。

harness 用宿主 gcc **真编译固件那份 .c**，不在 Python 里重写一遍模型。
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
APP_INC = ROOT / "App" / "Inc"
APP_SRC = ROOT / "App" / "Src"
DRIVER_INC = ROOT / "Driver" / "Inc"
DRIVER_SRC = ROOT / "Driver" / "Src"


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def strip_c_comments(text: str) -> str:
    """去掉 C 注释。

    断言"源码里**没有**某个标识符"时，注释是假阳性的主要来源：解释"为什么没有它"
    的那段话必然会提到它，于是断言永远为真。
    """
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


HARNESS = r"""
#include "app_led_config.h"

#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { \
    if (!(condition)) { \
        fprintf(stderr, "check %d failed at line %d\n", (code), __LINE__); \
        return (code); \
    } \
} while (0)

/* ---------------------------------------------------- 布局是 Flash ABI */

static int test_layout_is_frozen(void)
{
    CHECK(sizeof(APP_LedBinding) == 14U, 10);
    CHECK(sizeof(APP_LedConfig) == 252U, 11);
    CHECK((uint8_t)APP_LED_BIND_COUNT == 16U, 12);
    /* 解锁被拒那 7 条必须连续且排在最后，索引反推原因码靠的就是这个。 */
    CHECK((uint8_t)APP_LED_BIND_BLOCK_BASE == 9U, 13);
    CHECK((uint8_t)APP_LED_BIND_BLOCK_BASE + APP_LED_BIND_BLOCK_COUNT ==
          (uint8_t)APP_LED_BIND_COUNT, 14);
    return 0;
}

static int test_block_reason_maps_one_to_one(void)
{
    uint8_t reason;

    /* 0 = 没有被拒，不该有绑定；越界同理。两者都返回 COUNT。 */
    CHECK(APP_LedConfig_BindingForBlockReason(0U) == (uint8_t)APP_LED_BIND_COUNT, 20);
    CHECK(APP_LedConfig_BindingForBlockReason(8U) == (uint8_t)APP_LED_BIND_COUNT, 21);
    CHECK(APP_LedConfig_BindingForBlockReason(255U) == (uint8_t)APP_LED_BIND_COUNT, 22);

    for (reason = 1U; reason <= APP_LED_BIND_BLOCK_COUNT; reason++) {
        uint8_t id = APP_LedConfig_BindingForBlockReason(reason);
        CHECK(id < (uint8_t)APP_LED_BIND_COUNT, 23);
        CHECK(id == (uint8_t)APP_LED_BIND_BLOCK_BASE + (reason - 1U), 24);
    }
    return 0;
}

/* -------------------------------------------- 次数永远不来自配置 */

static int test_get_pattern_never_writes_count(void)
{
    /*
     * 这是本文件最重要的一条。GetPattern 必须把 count 留成 0，让调用方注入
     * 原因码。哪怕将来有人给 APP_LedBinding 加了 count 字段，只要 GetPattern
     * 还是不写它，"数到 N 下 == block=N" 就仍然成立。
     */
    uint8_t id;

    for (id = 0U; id < (uint8_t)APP_LED_BIND_COUNT; id++) {
        DRV_RgbPattern pattern;

        memset(&pattern, 0xAA, sizeof(pattern));   /* 先填脏，确认它真被清了 */
        APP_LedConfig_GetPattern(id, &pattern);
        CHECK(pattern.count == 0U, 30);
    }
    return 0;
}

static int test_out_of_range_binding_is_dark_not_garbage(void)
{
    DRV_RgbPattern pattern;

    memset(&pattern, 0xAA, sizeof(pattern));
    APP_LedConfig_GetPattern((uint8_t)APP_LED_BIND_COUNT, &pattern);
    CHECK(pattern.effect == DRV_RGB_EFFECT_OFF, 40);
    CHECK(pattern.color.r == 0U && pattern.color.g == 0U && pattern.color.b == 0U, 41);
    return 0;
}

/* ------------------------------------------------------ 默认表 */

static int test_defaults_are_valid_and_self_describing(void)
{
    APP_LedConfig config;

    APP_LedConfig_Defaults(&config);
    CHECK(config.magic == APP_LED_CFG_MAGIC, 50);
    CHECK(config.schema == APP_LED_CFG_SCHEMA, 51);
    CHECK(config.size == sizeof(APP_LedConfig), 52);
    CHECK(config.customized == 0U, 53);
    CHECK(APP_LedConfig_Validate(&config) == 1U, 54);
    return 0;
}

static int test_defaults_match_the_shipped_colour_language(void)
{
    /*
     * 逐位对 2026-09-12 那版 app_led.c 的常量。数字写死在这里是刻意的：
     * 从固件源码里再算一遍就等于让被测对象自己出题。
     */
    APP_LedConfig c;

    APP_LedConfig_Defaults(&c);

    CHECK(c.binding[APP_LED_BIND_ARMED].r == 255U, 60);
    CHECK(c.binding[APP_LED_BIND_ARMED].g == 0U, 61);
    CHECK(c.binding[APP_LED_BIND_ARMED].b == 0U, 62);
    CHECK(c.binding[APP_LED_BIND_ARMED].effect == DRV_RGB_EFFECT_SOLID, 63);

    CHECK(c.binding[APP_LED_BIND_READY].g == 255U, 64);
    CHECK(c.binding[APP_LED_BIND_READY].effect == DRV_RGB_EFFECT_BREATHE, 65);
    CHECK(c.binding[APP_LED_BIND_READY].period_ms == 2600U, 66);
    CHECK(c.binding[APP_LED_BIND_READY].dim == 10U, 67);

    CHECK(c.binding[APP_LED_BIND_HEARTBEAT].b == 255U, 68);
    CHECK(c.binding[APP_LED_BIND_HEARTBEAT].effect == DRV_RGB_EFFECT_BLINK, 69);
    CHECK(c.binding[APP_LED_BIND_HEARTBEAT].on_ms == 120U, 70);
    CHECK(c.binding[APP_LED_BIND_HEARTBEAT].off_ms == 380U, 71);

    CHECK(c.binding[APP_LED_BIND_CAL_RELEASED].effect == DRV_RGB_EFFECT_BLINK, 72);
    CHECK(c.binding[APP_LED_BIND_CAL_RELEASED].on_ms == 120U, 73);
    CHECK(c.binding[APP_LED_BIND_CAL_SAVE_ACK].on_ms == 320U, 74);
    CHECK(c.binding[APP_LED_BIND_CAL_ERROR].on_ms == 80U, 75);
    CHECK(c.binding[APP_LED_BIND_FLOW_STARTING].on_ms == 500U, 76);
    CHECK(c.binding[APP_LED_BIND_FLOW_RETRYING].on_ms == 160U, 77);
    CHECK(c.binding[APP_LED_BIND_FLOW_FAILED].effect == DRV_RGB_EFFECT_PULSES, 78);
    CHECK(c.binding[APP_LED_BIND_FLOW_FAILED].gap_ms == 760U, 79);

    /* 青 = (0, 200, 255)，琥珀 = (255, 110, 0)。 */
    CHECK(c.binding[APP_LED_BIND_FLOW_FAILED].g == 200U, 80);
    CHECK(c.binding[APP_LED_BIND_FLOW_FAILED].b == 255U, 81);
    return 0;
}

static int test_all_seven_block_reasons_default_to_amber_pulses(void)
{
    /* 默认行为必须和改造前一模一样：七个原因共用琥珀，靠闪几下区分。
     * 能分别配色是新增能力，不是新的默认值。 */
    APP_LedConfig c;
    uint8_t reason;

    APP_LedConfig_Defaults(&c);
    for (reason = 1U; reason <= APP_LED_BIND_BLOCK_COUNT; reason++) {
        const APP_LedBinding *b =
            &c.binding[APP_LedConfig_BindingForBlockReason(reason)];

        CHECK(b->r == 255U && b->g == 110U && b->b == 0U, 90);
        CHECK(b->effect == DRV_RGB_EFFECT_PULSES, 91);
        CHECK(b->on_ms == 160U && b->off_ms == 160U && b->gap_ms == 760U, 92);
    }
    return 0;
}

/* ------------------------------------------------------ 校验 */

static int test_validate_rejects_values_the_driver_would_silently_replace(void)
{
    APP_LedConfig c;

    /* BLINK 的 on/off 为 0：驱动会换成 1 ms，在 1 kHz 上看起来是半亮常亮。 */
    APP_LedConfig_Defaults(&c);
    c.binding[APP_LED_BIND_HEARTBEAT].on_ms = 0U;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 100);

    APP_LedConfig_Defaults(&c);
    c.binding[APP_LED_BIND_HEARTBEAT].off_ms = 0U;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 101);

    /* BREATHE 的 period 为 0：驱动会换成 2000。 */
    APP_LedConfig_Defaults(&c);
    c.binding[APP_LED_BIND_READY].period_ms = 0U;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 102);

    /* 周期过短也拒：低于 100 ms 的"呼吸"是抖动，不是呼吸。 */
    APP_LedConfig_Defaults(&c);
    c.binding[APP_LED_BIND_READY].period_ms = 50U;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 103);
    return 0;
}

static int test_validate_keeps_pulse_groups_countable(void)
{
    APP_LedConfig c;
    uint8_t id = APP_LedConfig_BindingForBlockReason(7U);

    /* 组间停顿太短，7 下和 6 下肉眼分不开，而固件照样"正常工作"。 */
    APP_LedConfig_Defaults(&c);
    c.binding[id].gap_ms = 100U;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 110);

    /* 停顿必须长过组内的灭，否则一组和两组连成一片。 */
    APP_LedConfig_Defaults(&c);
    c.binding[id].off_ms = 400U;
    c.binding[id].gap_ms = 500U;          /* < 2*400 */
    CHECK(APP_LedConfig_Validate(&c) == 0U, 111);

    APP_LedConfig_Defaults(&c);
    c.binding[id].off_ms = 200U;
    c.binding[id].gap_ms = 500U;          /* >= 2*200 且 >= 400 */
    CHECK(APP_LedConfig_Validate(&c) == 1U, 112);
    return 0;
}

static int test_validate_rejects_bad_effect_and_header(void)
{
    APP_LedConfig c;

    APP_LedConfig_Defaults(&c);
    c.binding[APP_LED_BIND_READY].effect = 99U;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 120);

    APP_LedConfig_Defaults(&c);
    c.magic = 0xDEADBEEFUL;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 121);

    APP_LedConfig_Defaults(&c);
    c.size = 4U;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 122);

    APP_LedConfig_Defaults(&c);
    c.schema = 99U;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 123);

    CHECK(APP_LedConfig_Validate(NULL) == 0U, 124);
    return 0;
}

static int test_validate_rejects_looking_like_armed(void)
{
    /*
     * 作者选定的唯一一条策略性硬拦：把"就绪"配成和"已解锁"一个样，会让人以为
     * 没解锁而去装桨。其余撞色不拦。
     */
    APP_LedConfig c;

    APP_LedConfig_Defaults(&c);
    c.binding[APP_LED_BIND_READY].r = c.binding[APP_LED_BIND_ARMED].r;
    c.binding[APP_LED_BIND_READY].g = c.binding[APP_LED_BIND_ARMED].g;
    c.binding[APP_LED_BIND_READY].b = c.binding[APP_LED_BIND_ARMED].b;
    c.binding[APP_LED_BIND_READY].effect = DRV_RGB_EFFECT_SOLID;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 130);

    /* 差几个灰阶仍然算撞：量化到 32 一档，眼睛分不出来的就是同一个。 */
    APP_LedConfig_Defaults(&c);
    c.binding[APP_LED_BIND_READY].r = 250U;
    c.binding[APP_LED_BIND_READY].g = 3U;
    c.binding[APP_LED_BIND_READY].b = 2U;
    c.binding[APP_LED_BIND_READY].effect = DRV_RGB_EFFECT_SOLID;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 131);

    /* 同色但不同效果不算撞——红常亮 vs 红呼吸，眼睛分得出来。 */
    APP_LedConfig_Defaults(&c);
    c.binding[APP_LED_BIND_READY].r = 255U;
    c.binding[APP_LED_BIND_READY].g = 0U;
    c.binding[APP_LED_BIND_READY].b = 0U;
    CHECK(c.binding[APP_LED_BIND_READY].effect == DRV_RGB_EFFECT_BREATHE, 132);
    CHECK(APP_LedConfig_Validate(&c) == 1U, 133);
    return 0;
}

static int test_seven_block_reasons_may_share_a_colour(void)
{
    /*
     * 默认就是七条同色同节奏。撞色检测只针对 ARMED，所以这必须通过——
     * 否则出厂默认值自己就过不了自己的校验。
     */
    APP_LedConfig c;

    APP_LedConfig_Defaults(&c);
    CHECK(APP_LedConfig_Validate(&c) == 1U, 140);
    return 0;
}

static int test_each_block_reason_can_get_its_own_colour(void)
{
    /* 作者要的能力：每种报错单独配色。 */
    APP_LedConfig c;
    uint8_t reason;
    DRV_RgbPattern pattern;

    APP_LedConfig_Defaults(&c);
    for (reason = 1U; reason <= APP_LED_BIND_BLOCK_COUNT; reason++) {
        uint8_t id = APP_LedConfig_BindingForBlockReason(reason);

        c.binding[id].r = (uint8_t)(reason * 30U);
        c.binding[id].g = (uint8_t)(255U - (reason * 30U));
        c.binding[id].b = (uint8_t)(reason * 10U);
    }
    CHECK(APP_LedConfig_Validate(&c) == 1U, 150);
    CHECK(APP_LedConfig_PublishActive(&c) == 1U, 151);

    for (reason = 1U; reason <= APP_LED_BIND_BLOCK_COUNT; reason++) {
        uint8_t id = APP_LedConfig_BindingForBlockReason(reason);

        APP_LedConfig_GetPattern(id, &pattern);
        CHECK(pattern.color.r == (uint8_t)(reason * 30U), 152);
        /* 颜色变了，次数仍然不来自配置。 */
        CHECK(pattern.count == 0U, 153);
    }
    return 0;
}

/* -------------------------------------- 闪几下：运行期常量，不来自配置 */

static int test_every_pulsing_binding_can_actually_be_counted(void)
{
    /*
     * 本文件第二重要的一条。`drv_rgb_led.c` 的 PULSES 遇到 count==0 **直接返回黑**。
     * 所以"谁该闪几下"这张表漏一条，症状不是闪错次数，而是那一档**完全不亮**，
     * 且因为它占着一个优先级槽，还会盖住下面的心跳——和"固件死了"一模一样。
     *
     * 2026-09-12 就漏过一次：注入次数的算式写成 `id >= BLOCK_BASE`，而
     * flow_failed=8 落在外面，于是光流失败从"青色 3 连闪"变成了"全黑"。
     */
    APP_LedConfig c;
    uint8_t id;

    CHECK(APP_LedConfig_PulseCount((uint8_t)APP_LED_BIND_FLOW_FAILED) == 3U, 200);
    CHECK(APP_LedConfig_PulseCount((uint8_t)APP_LED_BIND_COUNT) == 0U, 201);
    for (id = (uint8_t)APP_LED_BIND_BLOCK_BASE; id < (uint8_t)APP_LED_BIND_COUNT; id++) {
        CHECK(APP_LedConfig_PulseCount(id) ==
              (uint8_t)(id - (uint8_t)APP_LED_BIND_BLOCK_BASE + 1U), 202);
    }

    /* 出厂表里每一条 PULSES 都必须数得出次数。这一条是真正的闸门：
     * 以后谁把某条默认值改成 PULSES 而忘了给它次数，这里当场变红。 */
    APP_LedConfig_Defaults(&c);
    for (id = 0U; id < (uint8_t)APP_LED_BIND_COUNT; id++) {
        if (c.binding[id].effect == DRV_RGB_EFFECT_PULSES) {
            CHECK(APP_LedConfig_PulseCount(id) != 0U, 203);
        }
    }
    return 0;
}

static int test_a_pulses_binding_nobody_counts_is_refused(void)
{
    /* 能存进去就等于允许用户配出一条永久黑灯，而灯、LEDMAP? 回包、上位机波形
     * 三处各自自洽地说它在闪。宁可当场拒。 */
    APP_LedConfig c;

    APP_LedConfig_Defaults(&c);
    CHECK(APP_LedConfig_PulseCount(APP_LED_BIND_READY) == 0U, 210);
    c.binding[APP_LED_BIND_READY].effect = DRV_RGB_EFFECT_PULSES;
    c.binding[APP_LED_BIND_READY].on_ms = 160U;
    c.binding[APP_LED_BIND_READY].off_ms = 160U;
    c.binding[APP_LED_BIND_READY].gap_ms = 760U;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 211);
    return 0;
}

static int test_looking_like_armed_cannot_be_smuggled_past_by_effect(void)
{
    /*
     * 撞色检测只比 effect 的话，两条都能大摇大摆绕过去——而光比枚举更早到眼睛。
     * 这道拦的存在理由是"别让人以为没解锁而去装桨"，绕过它就等于没有它。
     */
    APP_LedConfig c;

    /* 红色在 78%..100% 之间走 60 秒：肉眼就是红常亮。 */
    APP_LedConfig_Defaults(&c);
    c.binding[APP_LED_BIND_READY].r = 255U;
    c.binding[APP_LED_BIND_READY].g = 0U;
    c.binding[APP_LED_BIND_READY].b = 0U;
    c.binding[APP_LED_BIND_READY].effect = DRV_RGB_EFFECT_BREATHE;
    c.binding[APP_LED_BIND_READY].period_ms = 60000U;
    c.binding[APP_LED_BIND_READY].dim = 200U;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 220);

    /* 红亮 60 秒、灭 50 ms：同样是红常亮。 */
    APP_LedConfig_Defaults(&c);
    c.binding[APP_LED_BIND_READY].r = 255U;
    c.binding[APP_LED_BIND_READY].g = 0U;
    c.binding[APP_LED_BIND_READY].b = 0U;
    c.binding[APP_LED_BIND_READY].effect = DRV_RGB_EFFECT_BLINK;
    c.binding[APP_LED_BIND_READY].on_ms = 60000U;
    c.binding[APP_LED_BIND_READY].off_ms = 50U;
    CHECK(APP_LedConfig_Validate(&c) == 0U, 221);

    /*
     * 反方向同样要钉住：看得出来的快闪不许误伤。出厂的"标定出错"就是红色
     * 80/80——灭占了半个周期，谁都分得出它不是常亮。把阈值调过头的症状是
     * 出厂默认值自己过不了自己的校验，灯会整个退回默认。
     */
    APP_LedConfig_Defaults(&c);
    CHECK(c.binding[APP_LED_BIND_CAL_ERROR].r == 255U, 222);
    CHECK(c.binding[APP_LED_BIND_CAL_ERROR].on_ms == 80U, 223);
    CHECK(c.binding[APP_LED_BIND_CAL_ERROR].off_ms == 80U, 224);
    CHECK(APP_LedConfig_Validate(&c) == 1U, 225);
    return 0;
}

/* ------------------------------------------------------ 名字表 */

static int test_names_round_trip(void)
{
    uint8_t id;
    uint8_t effect;

    for (id = 0U; id < (uint8_t)APP_LED_BIND_COUNT; id++) {
        const char *name = APP_LedConfig_BindName(id);

        CHECK(name != NULL && name[0] != '\0' && name[0] != '-', 160);
        CHECK(APP_LedConfig_BindFromName(name) == id, 161);
    }
    CHECK(APP_LedConfig_BindName((uint8_t)APP_LED_BIND_COUNT)[0] == '-', 162);
    CHECK(APP_LedConfig_BindFromName("nope") == (uint8_t)APP_LED_BIND_COUNT, 163);
    CHECK(APP_LedConfig_BindFromName(NULL) == (uint8_t)APP_LED_BIND_COUNT, 164);

    for (effect = 0U; effect <= (uint8_t)DRV_RGB_EFFECT_BREATHE; effect++) {
        const char *name = APP_LedConfig_EffectName(effect);

        CHECK(name != NULL && name[0] != '-', 165);
        CHECK(APP_LedConfig_EffectFromName(name) == effect, 166);
    }
    /* 认不出的效果名返回 0xFF，不是 0——0 是合法的 OFF。 */
    CHECK(APP_LedConfig_EffectFromName("rainbow") == 0xFFU, 167);
    CHECK(APP_LedConfig_EffectFromName(NULL) == 0xFFU, 168);
    return 0;
}

/* ------------------------------------------------- 运行期快照 */

static int test_publish_read_and_generation(void)
{
    APP_LedConfig written;
    APP_LedConfig readback;
    uint32_t before;

    APP_LedConfig_ResetActive();
    CHECK(APP_LedConfig_GetActiveGeneration() == 0U, 170);

    APP_LedConfig_Defaults(&written);
    written.binding[APP_LED_BIND_READY].r = 7U;
    before = APP_LedConfig_GetActiveGeneration();
    CHECK(APP_LedConfig_PublishActive(&written) == 1U, 171);
    CHECK(APP_LedConfig_GetActiveGeneration() == before + 1U, 172);

    CHECK(APP_LedConfig_ReadActive(&readback) == 1U, 173);
    CHECK(readback.binding[APP_LED_BIND_READY].r == 7U, 174);
    CHECK(readback.generation == APP_LedConfig_GetActiveGeneration(), 175);
    return 0;
}

static int test_publishing_an_invalid_config_is_refused(void)
{
    /* 拒收之后 active 必须还是上一份好的，不能变成半新半旧。 */
    APP_LedConfig good;
    APP_LedConfig bad;
    APP_LedConfig readback;

    APP_LedConfig_Defaults(&good);
    good.binding[APP_LED_BIND_READY].r = 21U;
    CHECK(APP_LedConfig_PublishActive(&good) == 1U, 180);

    APP_LedConfig_Defaults(&bad);
    bad.binding[APP_LED_BIND_READY].effect = 42U;
    CHECK(APP_LedConfig_PublishActive(&bad) == 0U, 181);

    CHECK(APP_LedConfig_ReadActive(&readback) == 1U, 182);
    CHECK(readback.binding[APP_LED_BIND_READY].r == 21U, 183);
    return 0;
}

static int test_reading_before_any_publish_gives_defaults_not_black(void)
{
    /*
     * 静态区初值全零：全零配置每条都是 effect=OFF、颜色全黑，灯一直不亮，
     * 看起来和固件死了一模一样。宁可交出厂默认表。
     */
    DRV_RgbPattern pattern;

    /* ResetActive 之后 magic 是有效的，所以这里直接验 GetPattern 的兜底路径：
     * 它在 active 无效时读默认表。 */
    APP_LedConfig_ResetActive();
    APP_LedConfig_GetPattern(APP_LED_BIND_ARMED, &pattern);
    CHECK(pattern.color.r == 255U, 190);
    CHECK(pattern.effect == DRV_RGB_EFFECT_SOLID, 191);
    return 0;
}

int main(void)
{
    int rc;

    rc = test_layout_is_frozen(); if (rc) { return rc; }
    rc = test_block_reason_maps_one_to_one(); if (rc) { return rc; }
    rc = test_get_pattern_never_writes_count(); if (rc) { return rc; }
    rc = test_out_of_range_binding_is_dark_not_garbage(); if (rc) { return rc; }
    rc = test_defaults_are_valid_and_self_describing(); if (rc) { return rc; }
    rc = test_defaults_match_the_shipped_colour_language(); if (rc) { return rc; }
    rc = test_all_seven_block_reasons_default_to_amber_pulses(); if (rc) { return rc; }
    rc = test_validate_rejects_values_the_driver_would_silently_replace(); if (rc) { return rc; }
    rc = test_validate_keeps_pulse_groups_countable(); if (rc) { return rc; }
    rc = test_validate_rejects_bad_effect_and_header(); if (rc) { return rc; }
    rc = test_validate_rejects_looking_like_armed(); if (rc) { return rc; }
    rc = test_seven_block_reasons_may_share_a_colour(); if (rc) { return rc; }
    rc = test_each_block_reason_can_get_its_own_colour(); if (rc) { return rc; }
    rc = test_every_pulsing_binding_can_actually_be_counted(); if (rc) { return rc; }
    rc = test_a_pulses_binding_nobody_counts_is_refused(); if (rc) { return rc; }
    rc = test_looking_like_armed_cannot_be_smuggled_past_by_effect(); if (rc) { return rc; }
    rc = test_names_round_trip(); if (rc) { return rc; }
    rc = test_publish_read_and_generation(); if (rc) { return rc; }
    rc = test_publishing_an_invalid_config_is_refused(); if (rc) { return rc; }
    rc = test_reading_before_any_publish_gives_defaults_not_black(); if (rc) { return rc; }

    printf("led config harness ok\n");
    return 0;
}
"""


def test_led_config_on_host_gcc(tmp_path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "harness.c"
    harness.write_text(HARNESS, encoding="utf-8")
    executable = tmp_path / "harness.exe"
    subprocess.run(
        [
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            f"-I{APP_INC}", f"-I{DRIVER_INC}",
            str(APP_SRC / "app_led_config.c"),
            str(DRIVER_SRC / "drv_rgb_led.c"),
            str(harness), "-o", str(executable),
        ],
        check=True, capture_output=True, text=True,
    )
    result = subprocess.run([str(executable)], check=True, capture_output=True)
    assert "led config harness ok" in result.stdout.decode("ascii", "replace")


def test_the_count_field_is_absent_from_the_schema() -> None:
    """配置结构体里不许出现 count。

    上面那条 harness 钉的是"GetPattern 不写 count"。这一条钉的是更前面一步：
    **字段压根不存在**。两条都要，因为加字段的人多半会顺手让 GetPattern 也写上，
    那样两条断言会一起变红——而如果只有运行期那一条，加字段本身是悄无声息的。
    """
    # 先去注释。解释"为什么没有 count"的那段注释里必然出现 count 这个词——
    # 不去注释的话这条断言永远为真，而它恰恰是本文件最重要的一条。
    # （同一个坑 2026-09-12 在 test_imu_capture_contract.py 里刚踩过：
    #   一条断言被解释性注释喂饱，绿了半个月。）
    header = strip_c_comments(read("App/Inc/app_led_config.h"))

    end = header.index("} APP_LedBinding;")
    binding = header[header.rindex("typedef struct {", 0, end):end]
    assert "count" not in binding, f"count 是解锁被拒的原因码，不许进配置：{binding}"
    assert "reserved0" in binding, "留出来的那个字节要明确标成 reserved"


def test_the_config_layer_stays_free_of_hardware() -> None:
    """它在 App 层，禁止碰 HAL——否则上面那个 harness 直接编不过。"""
    for path in ("App/Inc/app_led_config.h", "App/Src/app_led_config.c"):
        text = read(path)
        for banned in ("stm32h7xx_hal", "main.h", "cmsis_os", "HAL_"):
            assert banned not in text, f"{path} 引入了 {banned}"


def test_app_led_injects_the_count_and_no_longer_hardcodes_colours() -> None:
    """`app_led.c` 只剩"哪种状态成立"，颜色全部来自配置。

    钉两头：颜色常量必须已经搬走；闪烁次数必须仍然在这里按绑定 ID 反推。
    只钉前者的话，把 count 也挪进配置会静默通过。
    """
    source = read("App/Src/app_led.c")

    for gone in ("app_led_amber", "app_led_cyan", "app_led_green",
                 "DRV_RGB_EFFECT_BREATHE, 2600", "app_led_pulses("):
        assert gone not in source, f"{gone} 还留在 app_led.c 里"

    assert "APP_LedConfig_GetPattern" in source
    assert "pattern->count =" in source, "闪烁次数必须仍由 app_led.c 注入"

    # 次数只准从那**一张表**取，而且只准赋值这一次。自己再写一遍
    # `id - BLOCK_BASE + 1` 的偏移算式，就会像 2026-09-12 那次一样漏掉
    # flow_failed（它不在 BLOCK 段里），而漏掉的那一条在板子上是**全黑**。
    #
    # 钉"赋值了什么"而不是"提到了哪个宏"：BLOCK_BASE 在这个文件里还有一处
    # 正当用途（认不出的原因码落回第一条），拿符号在不在当判据会误伤它。
    for path, expression in (("App/Src/app_led.c",
                              "APP_LedConfig_PulseCount(binding_id)"),
                             ("App/Src/app_cmd_ledmap.c",
                              "APP_LedConfig_PulseCount(index)")):
        assigned = re.findall(r"pattern(?:\.|->)count\s*=\s*([^;]+);",
                              strip_c_comments(read(path)))
        assert assigned == [expression], f"{path}: {assigned}"


def test_the_flash_record_counts_the_led_block() -> None:
    """CFG 记录的校验跨度必须把 LED 块算进去。

    v20 加机体模型块时就漏在这里：Save 写的 size 含新块、读回来的校验式不含，
    于是每一条自己写的记录都过不了自己的检查，而且**没有任何报错**——
    上位机看到 SAVE OK，下次上电配置回默认。
    """
    store = read("App/Src/app_control_config_store.c")

    size_macro = store[store.index("#define APP_CONTROL_CFG_CURRENT_SIZE"):]
    size_macro = size_macro[:size_macro.index("_Static_assert")]
    assert "->led)" in size_macro, "CURRENT_SIZE 漏算了 LED 块"

    # 旧版本记录里没有 LED 块，读取器必须显式把它落回默认。
    assert "app_cmd_ledmap_apply_config(NULL)" in store
    # 当前版本的读取器要应用它。
    assert "app_cmd_ledmap_apply_config(&record.led)" in store
    # 保存时要填。
    assert "record.led = *(const APP_LedConfig *)app_cmd_ledmap_config()" in store


def test_the_config_version_moved_and_kept_a_reader_for_the_old_one() -> None:
    header = read("App/Inc/app_control_config_store.h")
    store = read("App/Src/app_control_config_store.c")

    # 2026-09-20（R-MAG-1）：v23 在记录尾部追加磁力计校准块，当前版本号随之
    # 推进到 23；v20 的读取器与本条断言的关系不受影响。
    # 2026-09-28：v24 追加指令整形/出口陷波块；同日晚 v25 在该块尾部追加第二级出口陷波。
    # 2026-09-29（R-FLOWMOUNT-1）：v26 在机体块尾部追加光流安装两项，v25 由 config_read_v25 读取、两项落回 0/0。
    # 2026-09-30（R-ALTID-1）：v28 在 XY 块后追加竖直通道块，v27 由 config_read_v27 读取。
    assert "#define APP_CONTROL_CFG_VERSION     29U" in header
    assert "#define APP_CONTROL_CFG_VERSION_V27 27U" in header
    assert "#define APP_CONTROL_CFG_VERSION_V26 26U" in header
    assert "#define APP_CONTROL_CFG_VERSION_V20 20U" in header
    assert "APP_ControlFlashRecordV20" in store
    assert "config_read_v20" in store
    assert "case APP_CONTROL_CFG_VERSION_V20:" in store


def test_the_v22_reader_keeps_led_colours_and_defaults_the_new_mag_block() -> None:
    """v22 → v23 迁移覆盖：LED 颜色表要保留，新的磁力计块要落回未校准。

    2026-09-20（R-MAG-1）：v23 在记录尾部追加磁力计校准块（当前版本号见上一条
    测试）。config_read_v22 读一条 v22 记录时，LED 颜色表必须按记录里的真实
    字段应用（`&record.led`），不能因为加了新块就退化成像它自己没有这一块的
    版本（v20 及更早）那样落回默认色；磁力计块在 v22 记录里不存在，必须显式
    落回未校准（NULL）。
    """
    store = read("App/Src/app_control_config_store.c")
    reader = store[store.index("APP_CONTROL_DEFINE_LEGACY_READER(config_read_v22"):]
    reader = reader[:reader.index("APP_CONTROL_DEFINE_LEGACY_READER(config_read_v21")]

    assert "app_cmd_ledmap_apply_config(&record.led);" in reader
    assert "app_cmd_magcal_apply_config(NULL);" in reader


def test_every_block_reason_has_exactly_one_binding() -> None:
    """解锁原因码与绑定必须一一对应，不能一多一少。

    少一条：某个原因永远显示成别的颜色。多一条：枚举错位，**每一条都错**。
    两边的名单都从源码里抓，不手抄。
    """
    reasons = set(re.findall(r"APP_LED_ARM_BLOCK_(\w+)\s*=\s*[1-9]",
                             read("App/Inc/app_led.h")))
    bindings = set(re.findall(r"APP_LED_BIND_BLOCK_(\w+)\s*=\s*\d+",
                              read("App/Inc/app_led_config.h")))
    bindings.discard("BASE")
    bindings.discard("COUNT")

    # R-BATT-1 keeps the existing 252-byte Flash color table intact. Its new
    # battery alarm has a dedicated eight-pulse path, tested with actual C in
    # test_battery_warning.py; it must return before the legacy binding lookup.
    assert "BATTERY" in reasons
    led = read("App/Src/app_led.c")
    alarm = led.index("if (reason==APP_LED_ARM_BLOCK_BATTERY)")
    lookup = led.index("APP_LedConfig_BindingForBlockReason", alarm)
    assert "APP_Battery_PublishLedWarning();return;" in led[alarm:lookup]
    reasons.remove("BATTERY")

    # 名字不完全一样（THROTTLE_HIGH vs THROTTLE），所以只比个数与顺序位置。
    assert len(reasons) == 7, sorted(reasons)
    assert len(bindings) == 7, sorted(bindings)
