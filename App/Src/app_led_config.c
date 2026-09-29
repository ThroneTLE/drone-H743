#include "app_led_config.h"

#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(APP_LedBinding) == 14U,
               "binding layout is part of the flash ABI");
_Static_assert(sizeof(APP_LedConfig) == 252U,
               "record layout is part of the flash ABI");
_Static_assert((uint8_t)APP_LED_BIND_BLOCK_BASE + APP_LED_BIND_BLOCK_COUNT ==
                   (uint8_t)APP_LED_BIND_COUNT,
               "block reason bindings must be the last contiguous run");

/* ─────────────────────────────────────────────────────────── 默认表 */

/*
 * 默认值必须**逐位**等于 2026-09-12 那版 app_led.c 的硬编码常量。
 *
 * 不是洁癖：现场已经在用"红=解锁 / 绿呼吸=就绪 / 琥珀数闪=被拒"这套语言，
 * doc/micoair743v2/README.md 也按这张表写着。升一次固件颜色悄悄变了，
 * 而文档还是旧的——那是最难查的一类不一致，因为没有任何东西报错。
 * tests/test_led_config_contract.py 把两边对起来。
 */
#define LED_RED    255U,   0U,   0U
#define LED_GREEN    0U, 255U,   0U
#define LED_BLUE     0U,   0U, 255U
#define LED_AMBER  255U, 110U,   0U
#define LED_CYAN     0U, 200U, 255U

#define LED_PULSE_ON  160U
#define LED_PULSE_OFF 160U
#define LED_PULSE_GAP 760U

static const APP_LedBinding led_default_binding[APP_LED_BIND_COUNT] = {
    /*                        r,g,b      effect                   on  off  gap  period dim rsv */
    [APP_LED_BIND_ARMED]         = {LED_RED,   DRV_RGB_EFFECT_SOLID,   0,   0,   0,      0,  0, 0},
    [APP_LED_BIND_READY]         = {LED_GREEN, DRV_RGB_EFFECT_BREATHE, 0,   0,   0,   2600, 10, 0},
    [APP_LED_BIND_HEARTBEAT]     = {LED_BLUE,  DRV_RGB_EFFECT_BLINK, 120, 380,   0,      0,  0, 0},
    [APP_LED_BIND_CAL_RELEASED]  = {LED_CYAN,  DRV_RGB_EFFECT_BLINK, 120, 120,   0,      0,  0, 0},
    [APP_LED_BIND_CAL_SAVE_ACK]  = {LED_GREEN, DRV_RGB_EFFECT_BLINK, 320, 320,   0,      0,  0, 0},
    [APP_LED_BIND_CAL_ERROR]     = {LED_RED,   DRV_RGB_EFFECT_BLINK,  80,  80,   0,      0,  0, 0},
    [APP_LED_BIND_FLOW_STARTING] = {LED_CYAN,  DRV_RGB_EFFECT_BLINK, 500, 500,   0,      0,  0, 0},
    [APP_LED_BIND_FLOW_RETRYING] = {LED_CYAN,  DRV_RGB_EFFECT_BLINK, 160, 160,   0,      0,  0, 0},
    [APP_LED_BIND_FLOW_FAILED]   = {LED_CYAN,  DRV_RGB_EFFECT_PULSES,
                                    LED_PULSE_ON, LED_PULSE_OFF, LED_PULSE_GAP, 0, 0, 0},
    /*
     * 七个解锁被拒原因默认全是琥珀数闪——和改造前一模一样，闪几下区分。
     * 用户可以把它们改成各自的颜色；闪几下不归这里管，运行期按原因码注入。
     */
    [APP_LED_BIND_BLOCK_NO_RC]      = {LED_AMBER, DRV_RGB_EFFECT_PULSES,
                                       LED_PULSE_ON, LED_PULSE_OFF, LED_PULSE_GAP, 0, 0, 0},
    [APP_LED_BIND_BLOCK_RC_LOSS]    = {LED_AMBER, DRV_RGB_EFFECT_PULSES,
                                       LED_PULSE_ON, LED_PULSE_OFF, LED_PULSE_GAP, 0, 0, 0},
    [APP_LED_BIND_BLOCK_ARM_SWITCH] = {LED_AMBER, DRV_RGB_EFFECT_PULSES,
                                       LED_PULSE_ON, LED_PULSE_OFF, LED_PULSE_GAP, 0, 0, 0},
    [APP_LED_BIND_BLOCK_THROTTLE]   = {LED_AMBER, DRV_RGB_EFFECT_PULSES,
                                       LED_PULSE_ON, LED_PULSE_OFF, LED_PULSE_GAP, 0, 0, 0},
    [APP_LED_BIND_BLOCK_IMU]        = {LED_AMBER, DRV_RGB_EFFECT_PULSES,
                                       LED_PULSE_ON, LED_PULSE_OFF, LED_PULSE_GAP, 0, 0, 0},
    [APP_LED_BIND_BLOCK_FRAME]      = {LED_AMBER, DRV_RGB_EFFECT_PULSES,
                                       LED_PULSE_ON, LED_PULSE_OFF, LED_PULSE_GAP, 0, 0, 0},
    [APP_LED_BIND_BLOCK_AIRFRAME]   = {LED_AMBER, DRV_RGB_EFFECT_PULSES,
                                       LED_PULSE_ON, LED_PULSE_OFF, LED_PULSE_GAP, 0, 0, 0},
};

/* ────────────────────────────────────────────────────────── 名字表 */

static const char *const led_bind_name[APP_LED_BIND_COUNT] = {
    [APP_LED_BIND_ARMED]            = "armed",
    [APP_LED_BIND_READY]            = "ready",
    [APP_LED_BIND_HEARTBEAT]        = "heartbeat",
    [APP_LED_BIND_CAL_RELEASED]     = "cal_released",
    [APP_LED_BIND_CAL_SAVE_ACK]     = "cal_save_ack",
    [APP_LED_BIND_CAL_ERROR]        = "cal_error",
    [APP_LED_BIND_FLOW_STARTING]    = "flow_starting",
    [APP_LED_BIND_FLOW_RETRYING]    = "flow_retrying",
    [APP_LED_BIND_FLOW_FAILED]      = "flow_failed",
    [APP_LED_BIND_BLOCK_NO_RC]      = "block_no_rc",
    [APP_LED_BIND_BLOCK_RC_LOSS]    = "block_rc_loss",
    [APP_LED_BIND_BLOCK_ARM_SWITCH] = "block_arm_switch",
    [APP_LED_BIND_BLOCK_THROTTLE]   = "block_throttle_high",
    [APP_LED_BIND_BLOCK_IMU]        = "block_imu",
    [APP_LED_BIND_BLOCK_FRAME]      = "block_frame",
    [APP_LED_BIND_BLOCK_AIRFRAME]   = "block_airframe",
};

#define LED_EFFECT_NAME_COUNT 5U

static const char *const led_effect_name[LED_EFFECT_NAME_COUNT] = {
    [DRV_RGB_EFFECT_OFF]     = "off",
    [DRV_RGB_EFFECT_SOLID]   = "solid",
    [DRV_RGB_EFFECT_BLINK]   = "blink",
    [DRV_RGB_EFFECT_PULSES]  = "pulses",
    [DRV_RGB_EFFECT_BREATHE] = "breathe",
};

const char *APP_LedConfig_BindName(uint8_t binding_id)
{
    if (binding_id >= (uint8_t)APP_LED_BIND_COUNT) {
        return "-";
    }
    return led_bind_name[binding_id];
}

uint8_t APP_LedConfig_BindFromName(const char *name)
{
    uint8_t index;

    if (name == NULL) {
        return (uint8_t)APP_LED_BIND_COUNT;
    }
    for (index = 0U; index < (uint8_t)APP_LED_BIND_COUNT; index++) {
        if (strcmp(name, led_bind_name[index]) == 0) {
            return index;
        }
    }
    return (uint8_t)APP_LED_BIND_COUNT;
}

const char *APP_LedConfig_EffectName(uint8_t effect)
{
    if (effect >= LED_EFFECT_NAME_COUNT) {
        return "-";
    }
    return led_effect_name[effect];
}

uint8_t APP_LedConfig_EffectFromName(const char *name)
{
    uint8_t index;

    if (name == NULL) {
        return 0xFFU;
    }
    for (index = 0U; index < LED_EFFECT_NAME_COUNT; index++) {
        if (strcmp(name, led_effect_name[index]) == 0) {
            return index;
        }
    }
    return 0xFFU;      /* 名字而不是数字：写错时当场拒，不会被当成合法枚举 */
}

uint8_t APP_LedConfig_BindingForBlockReason(uint8_t reason)
{
    if ((reason == 0U) || (reason > APP_LED_BIND_BLOCK_COUNT)) {
        return (uint8_t)APP_LED_BIND_COUNT;
    }
    return (uint8_t)((uint8_t)APP_LED_BIND_BLOCK_BASE + (reason - 1U));
}

/* 光流失败改造前就是"青色 3 连闪"，这个 3 必须留在代码里而不是配置里。 */
#define LED_FLOW_FAILED_PULSES 3U

uint8_t APP_LedConfig_PulseCount(uint8_t binding_id)
{
    if (binding_id >= (uint8_t)APP_LED_BIND_COUNT) {
        return 0U;
    }
    if (binding_id >= (uint8_t)APP_LED_BIND_BLOCK_BASE) {
        /* 闪几下 == 解锁被拒的原因码。这条等式是整套诊断的地基。 */
        return (uint8_t)(binding_id - (uint8_t)APP_LED_BIND_BLOCK_BASE + 1U);
    }
    if (binding_id == (uint8_t)APP_LED_BIND_FLOW_FAILED) {
        return LED_FLOW_FAILED_PULSES;
    }
    return 0U;
}

/* ───────────────────────────────────────────────────── 默认与校验 */

void APP_LedConfig_Defaults(APP_LedConfig *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->magic = APP_LED_CFG_MAGIC;
    config->schema = (uint16_t)APP_LED_CFG_SCHEMA;
    config->size = (uint16_t)sizeof(*config);
    config->identify_hold_ms = 5000U;
    config->customized = 0U;
    memcpy(config->binding, led_default_binding, sizeof(config->binding));
}

/*
 * 视觉签名：把一条绑定压成"眼睛能分辨的粒度"。
 *
 * 颜色按 32 量化（8 档/通道）、时间按 50 ms 量化。差 3 个灰阶的两种红在这颗
 * 灯珠上分不出来，而逐字节比较会说它们不同——那样冲突检测等于没有。
 */
#define LED_COLOR_STEP 32U   /* 每通道 8 档 */
#define LED_TIME_STEP  50U

/*
 * "看起来就是一直亮着"。
 *
 * 光比 effect 枚举更早到达眼睛。`BREATHE` 的谷底够亮、或 `BLINK` 的灭只占周期
 * 的一小截时，人看到的就是常亮——而只比 effect 的话，
 * `RHYTHM ready breathe 0 0 0 60000 200`（红色在 78%..100% 之间走 60 秒）
 * 和 `RHYTHM ready blink 60000 50 0 0 0`（红亮 60 秒、灭 50 ms）都能大摇大摆
 * 绕过"不许和已解锁撞色"这唯一一道硬拦，而那道拦的存在理由是**别让人以为
 * 没解锁而去装桨**。
 */
#define LED_STEADY_DIM       160U   /* 谷底 ≥ 63%：起伏看不出来了 */
#define LED_STEADY_OFF_RATIO 4U     /* 灭 < 周期的 1/5 */

static uint8_t led_reads_as_steady(const APP_LedBinding *binding)
{
    switch (binding->effect) {
    case DRV_RGB_EFFECT_SOLID:
        return 1U;
    case DRV_RGB_EFFECT_BREATHE:
        return (binding->dim >= LED_STEADY_DIM) ? 1U : 0U;
    case DRV_RGB_EFFECT_BLINK:
        /* 注意别把"快闪"也算进来：出厂的标定出错是红色 80/80，
         * 灭占了半个周期，眼睛看得清清楚楚，它不该和红常亮撞上。 */
        return (((uint32_t)binding->off_ms * LED_STEADY_OFF_RATIO) <
                (uint32_t)binding->on_ms) ? 1U : 0U;
    case DRV_RGB_EFFECT_OFF:
    case DRV_RGB_EFFECT_PULSES:
    default:
        return 0U;
    }
}

uint8_t APP_LedConfig_SameSignature(const APP_LedBinding *a, const APP_LedBinding *b)
{
    if ((a == NULL) || (b == NULL)) {
        return 0U;
    }
    /*
     * 逐字段比量化值，不折成一个哈希。哈希会撞——而这里撞一次的后果是**拒掉
     * 一份本来合法的配置**，用户看到的是"我明明没和已解锁撞色，它却不让我存"。
     */
    if ((a->r / LED_COLOR_STEP) != (b->r / LED_COLOR_STEP)) { return 0U; }
    if ((a->g / LED_COLOR_STEP) != (b->g / LED_COLOR_STEP)) { return 0U; }
    if ((a->b / LED_COLOR_STEP) != (b->b / LED_COLOR_STEP)) { return 0U; }
    /* 两边都"看起来常亮"就算撞，哪怕 effect 枚举不一样。 */
    if ((led_reads_as_steady(a) != 0U) && (led_reads_as_steady(b) != 0U)) {
        return 1U;
    }
    if (a->effect != b->effect) { return 0U; }

    switch (a->effect) {
    case DRV_RGB_EFFECT_BLINK:
    case DRV_RGB_EFFECT_PULSES:
        /* gap 不参与：数灯区分靠的是次数，而次数不在配置里。两条 PULSES
         * 只要颜色和明暗节奏一样，眼睛就分不出来，gap 差一点也一样。 */
        if ((a->on_ms / LED_TIME_STEP) != (b->on_ms / LED_TIME_STEP)) { return 0U; }
        if ((a->off_ms / LED_TIME_STEP) != (b->off_ms / LED_TIME_STEP)) { return 0U; }
        break;
    case DRV_RGB_EFFECT_BREATHE:
        if ((a->period_ms / LED_TIME_STEP) != (b->period_ms / LED_TIME_STEP)) { return 0U; }
        break;
    case DRV_RGB_EFFECT_OFF:
    case DRV_RGB_EFFECT_SOLID:
    default:
        break;
    }
    return 1U;
}

/*
 * 单条绑定的合法性。
 *
 * 这里拦的**不是**审美，是两类会骗人的东西：
 *
 * 1. 驱动会替你改掉的值。`drv_rgb_led.c` 把 period_ms=0 换成 2000、
 *    on/off_ms=0 换成 1 ms（1 kHz Σ-Δ 上看起来是半亮常亮）、count=0 直接全黑。
 *    存进去一个 0，上位机显示 0，灯却按 2000 在走——三者互相矛盾。
 * 2. 数灯断不了句。PULSES 的组间停顿太短，6 下和 7 下肉眼分不开，
 *    而固件照样"正常工作"。
 */
static uint8_t led_binding_valid(const APP_LedBinding *binding, uint8_t index)
{
    if (binding->effect > (uint8_t)DRV_RGB_EFFECT_BREATHE) {
        return 0U;
    }
    if ((binding->on_ms > APP_LED_CFG_MAX_TIME_MS) ||
        (binding->off_ms > APP_LED_CFG_MAX_TIME_MS) ||
        (binding->gap_ms > APP_LED_CFG_MAX_TIME_MS) ||
        (binding->period_ms > APP_LED_CFG_MAX_TIME_MS)) {
        return 0U;
    }
    if (binding->dim > APP_LED_CFG_MAX_DIM) {
        return 0U;
    }

    switch (binding->effect) {
    case DRV_RGB_EFFECT_BLINK:
        if ((binding->on_ms == 0U) || (binding->off_ms == 0U)) {
            return 0U;
        }
        break;
    case DRV_RGB_EFFECT_PULSES:
        /*
         * 数不出次数的绑定不许配成数闪。次数是运行期注入的，没人给它注入就是
         * count=0，而驱动遇到 count=0 直接返回黑：灯灭、`LEDMAP?` 照回
         * effect=pulses、上位机照画波形——三处显示各自自洽地说灯在闪。
         */
        if (APP_LedConfig_PulseCount(index) == 0U) {
            return 0U;
        }
        if ((binding->on_ms == 0U) || (binding->off_ms == 0U)) {
            return 0U;
        }
        if (binding->gap_ms < APP_LED_CFG_MIN_GAP_MS) {
            return 0U;
        }
        /* 组间停顿还要长过组内的灭，否则"一组"和"两组"连成一片。 */
        if (binding->gap_ms < (uint16_t)(2U * binding->off_ms)) {
            return 0U;
        }
        break;
    case DRV_RGB_EFFECT_BREATHE:
        if ((binding->period_ms < APP_LED_CFG_MIN_PERIOD_MS) ||
            (binding->period_ms > APP_LED_CFG_MAX_TIME_MS)) {
            return 0U;
        }
        break;
    case DRV_RGB_EFFECT_OFF:
    case DRV_RGB_EFFECT_SOLID:
    default:
        break;
    }
    return 1U;
}

uint8_t APP_LedConfig_Validate(const APP_LedConfig *config)
{
    uint8_t index;

    if (config == NULL) {
        return 0U;
    }
    if ((config->magic != APP_LED_CFG_MAGIC) ||
        (config->schema != (uint16_t)APP_LED_CFG_SCHEMA) ||
        (config->size != (uint16_t)sizeof(*config))) {
        return 0U;
    }
    if ((config->identify_hold_ms == 0U) ||
        (config->identify_hold_ms > APP_LED_CFG_MAX_TIME_MS)) {
        return 0U;
    }
    for (index = 0U; index < (uint8_t)APP_LED_BIND_COUNT; index++) {
        if (led_binding_valid(&config->binding[index], index) == 0U) {
            return 0U;
        }
    }
    /*
     * 唯一的策略性硬拦（作者裁决）：任何一条都不许和"已解锁"长一个样。
     * 把"就绪"配成和"已解锁"同样的红常亮，会让人以为没解锁而去装桨。
     * 其余撞色不拦——上位机把波形并排画出来，由人自己判断。
     */
    for (index = 1U; index < (uint8_t)APP_LED_BIND_COUNT; index++) {
        if (APP_LedConfig_SameSignature(&config->binding[index],
                                        &config->binding[APP_LED_BIND_ARMED]) != 0U) {
            return 0U;
        }
    }
    return 1U;
}

/* ─────────────────────────────────────────────── 运行期快照 */

static APP_LedConfig led_active;
static volatile uint32_t led_active_sequence;
static volatile uint32_t led_active_generation;

void APP_LedConfig_ResetActive(void)
{
    led_active_sequence++;
    APP_LedConfig_Defaults(&led_active);
    led_active_generation = 0U;
    led_active.generation = 0U;
    led_active_sequence++;
}

uint8_t APP_LedConfig_PublishActive(const APP_LedConfig *config)
{
    if (APP_LedConfig_Validate(config) == 0U) {
        return 0U;
    }
    led_active_sequence++;
    led_active = *config;
    led_active_generation++;
    led_active.generation = led_active_generation;
    led_active_sequence++;
    return 1U;
}

uint8_t APP_LedConfig_ReadActive(APP_LedConfig *config)
{
    uint32_t before;
    uint32_t after;
    uint8_t attempt;

    if (config == NULL) {
        return 0U;
    }
    for (attempt = 0U; attempt < 4U; attempt++) {
        before = led_active_sequence;
        if ((before & 1U) != 0U) {
            continue;           /* 写者正在中间，重读 */
        }
        *config = led_active;
        after = led_active_sequence;
        if (before == after) {
            if (config->magic != APP_LED_CFG_MAGIC) {
                /*
                 * 首次 Publish 之前静态区还是全零。全零配置的每条绑定都是
                 * effect=OFF、颜色全黑——灯一直不亮，看起来和固件死了一样。
                 * 宁可交出厂默认表。
                 */
                APP_LedConfig_Defaults(config);
            }
            return 1U;
        }
    }
    APP_LedConfig_Defaults(config);
    return 0U;
}

uint32_t APP_LedConfig_GetActiveGeneration(void)
{
    return led_active_generation;
}

void APP_LedConfig_GetPattern(uint8_t binding_id, DRV_RgbPattern *out)
{
    const APP_LedBinding *binding;

    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (binding_id >= (uint8_t)APP_LED_BIND_COUNT) {
        return;             /* effect = OFF，灯灭；调用方不该走到这里 */
    }
    /*
     * 直接读 active 而不先拷一份整表：LED 任务栈只有 1 KB。
     * 撕裂的代价这里是可接受的——最坏是某一拍用了半新半旧的图案，
     * 下一拍（1 ms 后）就对了。为此不值得在 1 kHz 的路径上拷 252 字节。
     */
    if (led_active.magic != APP_LED_CFG_MAGIC) {
        binding = &led_default_binding[binding_id];
    } else {
        binding = &led_active.binding[binding_id];
    }
    out->color.r = binding->r;
    out->color.g = binding->g;
    out->color.b = binding->b;
    out->effect = (DRV_RgbEffect)binding->effect;
    out->on_ms = binding->on_ms;
    out->off_ms = binding->off_ms;
    out->gap_ms = binding->gap_ms;
    out->period_ms = binding->period_ms;
    out->dim = binding->dim;
    /* out->count 不写：由调用方按解锁被拒原因码注入。 */
}
