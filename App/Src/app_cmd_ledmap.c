/*
 * LEDMAP 命令族：上位机读写状态灯的颜色绑定表。
 *
 * 状态机与 `app_cmd_rcmap.c` 同构，因为两者解决的是同一类问题——一份存 Flash 的
 * 配置，要能在线改、立刻看到效果、确认满意了再落盘：
 *
 *   SET / RHYTHM / RESET   只改 RAM 工作副本并立刻 PublishActive（所见即所得）
 *   COMMIT                 才真正写 Flash
 *
 * 为什么不是每改一次就存：落盘是**同步**擦一个 128 KB 片内扇区再写 ~700 B，
 * 跑在 UARTTask 上下文里忙等。取色盘拖动时每次都存，就是几十次 128 KB 擦除
 * 加几十次秒级卡顿，而 UARTTask 优先级高于遥测任务，波形会被按秒掐断。
 *
 * SET（颜色）和 RHYTHM（节奏）刻意拆成两条子命令，不合成一条：合起来正好 11 个
 * token，而 `APP_Control_ProcessLine` 只给 tokens[10]，`app_control_tokenize`
 * 填满就**静默截断**——现象是"我明明写了 dim，它却回 usage"。
 */

#include "app_control.h"
#include "app_control_internal.h"

#include "app_led.h"
#include "app_led_config.h"
#include "app_flash_service.h"
#include "app_stabilizer.h"
#include "svc_led.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static APP_LedConfig ledmap_config;
static uint8_t ledmap_loaded;
static uint8_t ledmap_dirty;

static void ledmap_ensure_loaded(void)
{
    if (ledmap_loaded == 0U) {
        APP_LedConfig_Defaults(&ledmap_config);
        ledmap_loaded = 1U;
        (void)APP_LedConfig_PublishActive(&ledmap_config);
    }
}

/* ─────────────────────────────────────────────────────────── 回包 */

static void ledmap_report_line(uint8_t index)
{
    const APP_LedBinding *binding = &ledmap_config.binding[index];

    /*
     * `lock=` 告诉上位机哪些字段它不该让用户编辑，**事实源唯一留在固件里**。
     * 上位机再抄一份规则表就会分叉：界面允许改、固件拒收，用户只能靠猜。
     *
     * 目前只有一种锁：闪几下是运行期常量（解锁被拒 = 原因码，光流失败 = 3），
     * 它压根不在配置里。判据用 PulseCount 而不是"索引在不在 BLOCK 段"——
     * 后者会漏掉 flow_failed，于是上位机以为那一条的次数可改。
     */
    const char *lock = (APP_LedConfig_PulseCount(index) != 0U) ? "count" : "-";

    APP_Control_QueueText(
        "LEDMAP bind=%s r=%u g=%u b=%u effect=%s on=%u off=%u gap=%u "
        "period=%u dim=%u lock=%s\r\n",
        APP_LedConfig_BindName(index),
        (unsigned int)binding->r,
        (unsigned int)binding->g,
        (unsigned int)binding->b,
        APP_LedConfig_EffectName(binding->effect),
        (unsigned int)binding->on_ms,
        (unsigned int)binding->off_ms,
        (unsigned int)binding->gap_ms,
        (unsigned int)binding->period_ms,
        (unsigned int)binding->dim,
        lock);
}

static void ledmap_report(const char *state, const char *bind, const char *reason)
{
    APP_Control_QueueText(
        "LEDMAP state=%s bind=%s n=%u gen=%lu dirty=%u customized=%u reason=%s\r\n",
        state,
        (bind != NULL) ? bind : "-",
        (unsigned int)APP_LED_BIND_COUNT,
        (unsigned long)APP_LedConfig_GetActiveGeneration(),
        (unsigned int)ledmap_dirty,
        (unsigned int)ledmap_config.customized,
        (reason != NULL) ? reason : "-");
}

static void ledmap_report_all(void)
{
    uint8_t index;

    ledmap_report("status", NULL, NULL);
    for (index = 0U; index < (uint8_t)APP_LED_BIND_COUNT; index++) {
        ledmap_report_line(index);
    }
}

/* ───────────────────────────────────────────────────── 参数解析 */

static uint8_t ledmap_parse_u16(const char *text, uint16_t *out)
{
    uint32_t value;

    if (app_control_parse_u32(text, &value) == 0U) {
        return 0U;
    }
    if (value > 65535U) {
        return 0U;
    }
    *out = (uint16_t)value;
    return 1U;
}

static uint8_t ledmap_parse_u8(const char *text, uint8_t *out)
{
    uint32_t value;

    if ((app_control_parse_u32(text, &value) == 0U) || (value > 255U)) {
        return 0U;
    }
    *out = (uint8_t)value;
    return 1U;
}

/*
 * 改灯色等于改"红是什么意思"。解锁状态下改一次，就可能让"红=已解锁"这条约定
 * 在桨正在转的时候失效——和 RCMAP / IMUFRAME 的安全门同源。
 */
static uint8_t ledmap_write_allowed(void)
{
    return (APP_Stabilizer_IsArmed() == 0U) ? 1U : 0U;
}

/*
 * 候选配置的落地：校验 → 存工作副本 → 立刻发布 → 置 dirty。
 * Validate 失败时把**具体**原因报出来，不是一律 rejected——否则用户只能靠猜
 * 是哪个字段不合法。
 */
static void ledmap_apply(const APP_LedConfig *candidate, const char *bind)
{
    if (APP_LedConfig_Validate(candidate) == 0U) {
        ledmap_report("rejected", bind, "invalid");
        return;
    }
    ledmap_config = *candidate;
    ledmap_config.customized = 1U;
    ledmap_dirty = 1U;
    (void)APP_LedConfig_PublishActive(&ledmap_config);
    ledmap_report("applied_ram", bind, NULL);
}

/* 在 Validate 之前先把能具名的原因挑出来，让回包说人话。 */
static const char *ledmap_reject_reason(const APP_LedConfig *candidate, uint8_t index)
{
    const APP_LedBinding *binding = &candidate->binding[index];

    if (binding->effect > (uint8_t)DRV_RGB_EFFECT_BREATHE) {
        return "bad_effect";
    }
    if (((binding->effect == (uint8_t)DRV_RGB_EFFECT_BLINK) ||
         (binding->effect == (uint8_t)DRV_RGB_EFFECT_PULSES)) &&
        ((binding->on_ms == 0U) || (binding->off_ms == 0U))) {
        return "zero_timing";
    }
    if (binding->effect == (uint8_t)DRV_RGB_EFFECT_PULSES) {
        /* 数不出次数的绑定配成数闪 = 永久黑灯。具名拒绝，别让用户猜。 */
        if (APP_LedConfig_PulseCount(index) == 0U) {
            return "pulses_not_countable";
        }
        if ((binding->gap_ms < APP_LED_CFG_MIN_GAP_MS) ||
            (binding->gap_ms < (uint16_t)(2U * binding->off_ms))) {
            return "gap_too_short";
        }
    }
    if ((binding->effect == (uint8_t)DRV_RGB_EFFECT_BREATHE) &&
        ((binding->period_ms < APP_LED_CFG_MIN_PERIOD_MS) ||
         (binding->period_ms > APP_LED_CFG_MAX_TIME_MS))) {
        return "bad_period";
    }
    if (binding->dim > APP_LED_CFG_MAX_DIM) {
        return "dim_too_high";
    }
    if ((index != (uint8_t)APP_LED_BIND_ARMED) &&
        (APP_LedConfig_SameSignature(binding,
                                     &candidate->binding[APP_LED_BIND_ARMED]) != 0U)) {
        return "conflicts_with_armed";
    }
    return NULL;
}

static void ledmap_apply_named(const APP_LedConfig *candidate, uint8_t index)
{
    const char *bind = APP_LedConfig_BindName(index);
    const char *reason = ledmap_reject_reason(candidate, index);

    if (reason != NULL) {
        ledmap_report("rejected", bind, reason);
        return;
    }
    ledmap_apply(candidate, bind);
}

/* ─────────────────────────────────────────────────── 子命令 */

static void ledmap_handle_set(char **tokens, uint32_t count)
{
    APP_LedConfig candidate;
    uint8_t index;

    if (count != 6U) {
        ledmap_report("invalid_usage", NULL, "need_bind_r_g_b");
        return;
    }
    index = APP_LedConfig_BindFromName(tokens[2]);
    if (index >= (uint8_t)APP_LED_BIND_COUNT) {
        ledmap_report("rejected", tokens[2], "bad_bind");
        return;
    }
    candidate = ledmap_config;
    if ((ledmap_parse_u8(tokens[3], &candidate.binding[index].r) == 0U) ||
        (ledmap_parse_u8(tokens[4], &candidate.binding[index].g) == 0U) ||
        (ledmap_parse_u8(tokens[5], &candidate.binding[index].b) == 0U)) {
        ledmap_report("rejected", tokens[2], "bad_channel");
        return;
    }
    ledmap_apply_named(&candidate, index);
}

static void ledmap_handle_rhythm(char **tokens, uint32_t count)
{
    APP_LedConfig candidate;
    uint8_t index;
    uint8_t effect;

    if (count != 9U) {
        ledmap_report("invalid_usage", NULL, "need_bind_effect_on_off_gap_period_dim");
        return;
    }
    index = APP_LedConfig_BindFromName(tokens[2]);
    if (index >= (uint8_t)APP_LED_BIND_COUNT) {
        ledmap_report("rejected", tokens[2], "bad_bind");
        return;
    }
    effect = APP_LedConfig_EffectFromName(tokens[3]);
    if (effect > (uint8_t)DRV_RGB_EFFECT_BREATHE) {
        ledmap_report("rejected", tokens[2], "bad_effect");
        return;
    }
    candidate = ledmap_config;
    candidate.binding[index].effect = effect;
    if ((ledmap_parse_u16(tokens[4], &candidate.binding[index].on_ms) == 0U) ||
        (ledmap_parse_u16(tokens[5], &candidate.binding[index].off_ms) == 0U) ||
        (ledmap_parse_u16(tokens[6], &candidate.binding[index].gap_ms) == 0U) ||
        (ledmap_parse_u16(tokens[7], &candidate.binding[index].period_ms) == 0U) ||
        (ledmap_parse_u8(tokens[8], &candidate.binding[index].dim) == 0U)) {
        ledmap_report("rejected", tokens[2], "bad_number");
        return;
    }
    ledmap_apply_named(&candidate, index);
}

static void ledmap_handle_reset(char **tokens, uint32_t count)
{
    APP_LedConfig candidate;
    uint8_t index;

    APP_LedConfig_Defaults(&candidate);
    if (count >= 3U) {
        index = APP_LedConfig_BindFromName(tokens[2]);
        if (index >= (uint8_t)APP_LED_BIND_COUNT) {
            ledmap_report("rejected", tokens[2], "bad_bind");
            return;
        }
        /* 只恢复这一条，其余保留用户值。 */
        {
            APP_LedBinding restored = candidate.binding[index];
            candidate = ledmap_config;
            candidate.binding[index] = restored;
        }
        if (APP_LedConfig_Validate(&candidate) == 0U) {
            ledmap_report("rejected", tokens[2], "invalid");
            return;
        }
        ledmap_config = candidate;
    } else {
        ledmap_config = candidate;
    }
    ledmap_dirty = 1U;
    (void)APP_LedConfig_PublishActive(&ledmap_config);
    ledmap_report("reset_ram", (count >= 3U) ? tokens[2] : "all", NULL);
}

/*
 * PREVIEW：把某条绑定**解析后**的图案放到点名源上，到点自动交还。
 *
 * 比 `LED RGB/BLINK/BREATHE` 强在能预览"数闪几下"——而那恰恰是最需要人眼确认
 * 的一种：7 下和 6 下在屏幕上一目了然，在板子上未必。
 */
static void ledmap_handle_preview(char **tokens, uint32_t count)
{
    DRV_RgbPattern pattern;
    uint8_t index;
    uint32_t hold_ms = ledmap_config.identify_hold_ms;

    if (count < 3U) {
        ledmap_report("invalid_usage", NULL, "need_bind");
        return;
    }
    index = APP_LedConfig_BindFromName(tokens[2]);
    if (index >= (uint8_t)APP_LED_BIND_COUNT) {
        ledmap_report("rejected", tokens[2], "bad_bind");
        return;
    }
    if (count >= 4U) {
        uint32_t requested;

        if ((app_control_parse_u32(tokens[3], &requested) == 0U) ||
            (requested > APP_LED_CFG_MAX_TIME_MS)) {
            ledmap_report("rejected", tokens[2], "bad_number");
            return;
        }
        hold_ms = requested;
    }

    APP_LedConfig_GetPattern(index, &pattern);
    /* 次数和运行期走同一张表，不在这里重算——两处各写一遍就会分叉，
     * 而分叉的症状是"预览闪 3 下、实际不亮"。 */
    pattern.count = APP_LedConfig_PulseCount(index);
    APP_LED_IdentifyPattern(&pattern, hold_ms);
    APP_Control_QueueText("LEDMAP state=preview bind=%s hold_ms=%lu count=%u\r\n",
                          APP_LedConfig_BindName(index),
                          (unsigned long)hold_ms,
                          (unsigned int)pattern.count);
}

static void ledmap_handle_commit(void)
{
    APP_FlashService_Status save_status;

    if (APP_LedConfig_Validate(&ledmap_config) == 0U) {
        ledmap_report("rejected", NULL, "invalid");
        return;
    }
    save_status = app_control_internal_commit_config_persist();
    if (save_status != APP_FLASH_SERVICE_OK) {
        APP_Control_QueueText("LEDMAP state=commit_failed st=%d\r\n", (int)save_status);
        return;
    }
    ledmap_dirty = 0U;
    ledmap_report("committed", NULL, NULL);
}

/* ─────────────────────────────────────────────────── 入口 */

uint8_t app_control_handle_ledmap(char **tokens, uint32_t count)
{
    if ((tokens == NULL) || (count == 0U)) {
        return 0U;
    }
    if ((strcmp(tokens[0], "LEDMAP?") != 0) && (strcmp(tokens[0], "LEDMAP") != 0)) {
        return 0U;
    }
    ledmap_ensure_loaded();

    if ((strcmp(tokens[0], "LEDMAP?") == 0) || (count == 1U)) {
        ledmap_report_all();
        return 1U;
    }

    if (strcmp(tokens[1], "PREVIEW") == 0) {
        /* 只读的预览不需要 disarmed：它不改配置，而且到点自动交还。 */
        ledmap_handle_preview(tokens, count);
        return 1U;
    }

    if (ledmap_write_allowed() == 0U) {
        ledmap_report("armed_blocked", NULL, NULL);
        return 1U;
    }

    if (strcmp(tokens[1], "SET") == 0) {
        ledmap_handle_set(tokens, count);
    } else if (strcmp(tokens[1], "RHYTHM") == 0) {
        ledmap_handle_rhythm(tokens, count);
    } else if (strcmp(tokens[1], "RESET") == 0) {
        ledmap_handle_reset(tokens, count);
    } else if (strcmp(tokens[1], "COMMIT") == 0) {
        ledmap_handle_commit();
    } else {
        ledmap_report("invalid_usage", NULL, "SET|RHYTHM|RESET|PREVIEW|COMMIT");
    }
    return 1U;
}

/* ───────────────────────── 与配置记录（CFG）的接口 */

void app_cmd_ledmap_apply_config(const void *config)
{
    const APP_LedConfig *loaded = (const APP_LedConfig *)config;

    if ((loaded == NULL) || (APP_LedConfig_Validate(loaded) == 0U)) {
        /*
         * 旧版本记录里没有 LED 块，或者存的那份过不了校验。
         * **必须显式落回默认**，不能什么都不做：什么都不做会让 RAM 里留着上一次
         * 的颜色，于是 LEDMAP? 报的和 Flash 里存的不是一回事，而用户据此认为
         * "存进去了"。airframe 当年漏的就是这半步。
         */
        APP_LedConfig_Defaults(&ledmap_config);
    } else {
        ledmap_config = *loaded;
    }
    ledmap_loaded = 1U;
    ledmap_dirty = 0U;
    (void)APP_LedConfig_PublishActive(&ledmap_config);
}

const void *app_cmd_ledmap_config(void)
{
    ledmap_ensure_loaded();
    return &ledmap_config;
}
