#include "app_rc_config.h"

#include <string.h>

/*
 * 默认映射刻意等于改造前 app_stabilizer.c 里写死的那一套（CH1..CH6，1000/1500/2000），
 * 这样没有标定记录的飞控行为与旧固件逐位一致，升级不会改变飞行手感。
 */
static const uint8_t app_rc_default_channel[APP_RC_FUNC_COUNT] = {
    0U, /* ROLL     -> CH1 */
    1U, /* PITCH    -> CH2 */
    2U, /* THROTTLE -> CH3 */
    3U, /* YAW      -> CH4 */
    4U, /* ARM      -> CH5 */
    5U, /* MODE     -> CH6 */
};

static const char *const app_rc_function_name[APP_RC_FUNC_COUNT] = {
    "roll", "pitch", "throttle", "yaw", "arm", "mode",
};

static APP_RcConfig app_rc_active;
static volatile uint32_t app_rc_active_sequence;
static volatile uint32_t app_rc_active_generation;

static float app_rc_clamp_f32(float value, float lo, float hi)
{
    if (value < lo) { return lo; }
    if (value > hi) { return hi; }
    return value;
}

void APP_RcConfig_Defaults(APP_RcConfig *config)
{
    uint8_t index;

    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->magic = APP_RC_CONFIG_MAGIC;
    config->schema = APP_RC_CONFIG_SCHEMA;
    config->size = (uint16_t)sizeof(*config);
    for (index = 0U; index < APP_RC_FUNC_COUNT; ++index) {
        config->function[index].channel = app_rc_default_channel[index];
        config->function[index].reversed = 0U;
        config->function[index].min_us = APP_RC_DEFAULT_MIN_US;
        config->function[index].mid_us = APP_RC_DEFAULT_MID_US;
        config->function[index].max_us = APP_RC_DEFAULT_MAX_US;
    }
    config->deadband_us = APP_RC_DEFAULT_DEADBAND_US;
    config->calibrated = 0U;
    config->generation = 0U;
}

uint8_t APP_RcConfig_Validate(const APP_RcConfig *config)
{
    uint32_t used_channels = 0U;
    uint8_t index;

    if (config == NULL) {
        return 0U;
    }
    if ((config->magic != APP_RC_CONFIG_MAGIC) ||
        (config->schema != APP_RC_CONFIG_SCHEMA) ||
        (config->size != (uint16_t)sizeof(*config))) {
        return 0U;
    }
    if (config->deadband_us > APP_RC_MAX_DEADBAND_US) {
        return 0U;
    }

    for (index = 0U; index < APP_RC_FUNC_COUNT; ++index) {
        const APP_RcFunctionMap *map = &config->function[index];

        if (map->channel == APP_RC_CHANNEL_UNBOUND) {
            continue;
        }
        if (map->channel >= CRSF_CHANNEL_COUNT) {
            return 0U;
        }
        /*
         * 一个通道绑两个功能一定是误操作：比如把 ARM 和 THROTTLE 绑到同一路，
         * 推油门就等于解锁。这里直接拒绝，不留给上层去发现。
         */
        if ((used_channels & (1UL << map->channel)) != 0U) {
            return 0U;
        }
        used_channels |= (1UL << map->channel);

        if ((map->min_us < APP_RC_US_MIN) || (map->max_us > APP_RC_US_MAX)) {
            return 0U;
        }
        if (map->min_us >= map->max_us) {
            return 0U;
        }
        if ((uint16_t)(map->max_us - map->min_us) < APP_RC_MIN_SPAN_US) {
            return 0U;
        }
        if ((map->mid_us <= map->min_us) || (map->mid_us >= map->max_us)) {
            return 0U;
        }
        if (map->reversed > 1U) {
            return 0U;
        }
    }
    return 1U;
}

const char *APP_RcConfig_FunctionName(uint8_t function)
{
    if (function >= APP_RC_FUNC_COUNT) {
        return "unknown";
    }
    return app_rc_function_name[function];
}

uint8_t APP_RcConfig_FunctionFromName(const char *name)
{
    uint8_t index;

    if (name == NULL) {
        return APP_RC_FUNC_COUNT;
    }
    for (index = 0U; index < APP_RC_FUNC_COUNT; ++index) {
        if (strcmp(name, app_rc_function_name[index]) == 0) {
            return index;
        }
    }
    return APP_RC_FUNC_COUNT;
}

float APP_RcConfig_Normalize(const APP_RcConfig *config,
                             uint8_t function,
                             uint16_t channel_us)
{
    const APP_RcFunctionMap *map;
    int32_t centered;
    int32_t span;
    float result;

    if ((config == NULL) || (function >= APP_RC_FUNC_COUNT)) {
        return 0.0f;
    }
    map = &config->function[function];
    if (map->channel == APP_RC_CHANNEL_UNBOUND) {
        return 0.0f;
    }

    centered = (int32_t)channel_us - (int32_t)map->mid_us;
    if ((centered > -(int32_t)config->deadband_us) &&
        (centered < (int32_t)config->deadband_us)) {
        return 0.0f;
    }

    /*
     * 上下两半分别用各自的跨度归一化。摇杆中位很少正好落在行程中点，
     * 若统一除以一个跨度，回中附近两侧增益就会不同，打杆手感会偏。
     */
    if (centered > 0) {
        span = (int32_t)map->max_us - (int32_t)map->mid_us;
        if (centered > span) { centered = span; }
    } else {
        span = (int32_t)map->mid_us - (int32_t)map->min_us;
        if (centered < -span) { centered = -span; }
    }
    if (span <= 0) {
        return 0.0f;
    }

    result = (float)centered / (float)span;
    if (map->reversed != 0U) {
        result = -result;
    }
    return app_rc_clamp_f32(result, -1.0f, 1.0f);
}

float APP_RcConfig_Throttle01(const APP_RcConfig *config, uint16_t channel_us)
{
    const APP_RcFunctionMap *map;
    int32_t span;
    int32_t value;
    float result;

    if (config == NULL) {
        return 0.0f;
    }
    map = &config->function[APP_RC_FUNC_THROTTLE];
    if (map->channel == APP_RC_CHANNEL_UNBOUND) {
        return 0.0f;
    }

    span = (int32_t)map->max_us - (int32_t)map->min_us;
    if (span <= 0) {
        return 0.0f;
    }
    value = (int32_t)channel_us - (int32_t)map->min_us;
    if (value < 0) { value = 0; }
    if (value > span) { value = span; }

    result = (float)value / (float)span;
    if (map->reversed != 0U) {
        result = 1.0f - result;
    }
    return app_rc_clamp_f32(result, 0.0f, 1.0f);
}

void APP_RcConfig_Resolve(const APP_RcConfig *config,
                          const uint16_t channels_us[CRSF_CHANNEL_COUNT],
                          APP_RcInputs *inputs)
{
    APP_RcConfig fallback;
    uint8_t index;

    if (inputs == NULL) {
        return;
    }
    memset(inputs, 0, sizeof(*inputs));
    if (channels_us == NULL) {
        return;
    }
    if ((config == NULL) || (APP_RcConfig_Validate(config) == 0U)) {
        /*
         * 配置不可用时退回默认映射而不是输出全 0：全 0 会把油门当成"最低"、
         * 把 ARM 当成"低电平"，看起来安全，但真实后果是遥控器整个失效且无提示。
         * 退回默认至少行为等同于旧固件，异常本身由上层的校验状态上报。
         */
        APP_RcConfig_Defaults(&fallback);
        config = &fallback;
    }

    for (index = 0U; index < APP_RC_FUNC_COUNT; ++index) {
        uint8_t channel = config->function[index].channel;

        if (channel >= CRSF_CHANNEL_COUNT) {
            inputs->us[index] = config->function[index].mid_us;
            inputs->norm[index] = 0.0f;
            continue;
        }
        inputs->bound_mask |= (uint8_t)(1U << index);
        inputs->us[index] = channels_us[channel];
        inputs->norm[index] =
            APP_RcConfig_Normalize(config, index, channels_us[channel]);
    }
    inputs->throttle_01 =
        APP_RcConfig_Throttle01(config, inputs->us[APP_RC_FUNC_THROTTLE]);
}

void APP_RcConfig_ResetActive(void)
{
    app_rc_active_sequence++;
    __asm volatile ("dmb 0xF" ::: "memory");
    APP_RcConfig_Defaults(&app_rc_active);
    app_rc_active_generation = 0U;
    __asm volatile ("dmb 0xF" ::: "memory");
    app_rc_active_sequence++;
}

uint8_t APP_RcConfig_PublishActive(const APP_RcConfig *config)
{
    if ((config == NULL) || (APP_RcConfig_Validate(config) == 0U)) {
        return 0U;
    }

    app_rc_active_sequence++;
    __asm volatile ("dmb 0xF" ::: "memory");
    app_rc_active = *config;
    app_rc_active_generation++;
    app_rc_active.generation = app_rc_active_generation;
    __asm volatile ("dmb 0xF" ::: "memory");
    app_rc_active_sequence++;
    return 1U;
}

uint8_t APP_RcConfig_ReadActive(APP_RcConfig *config)
{
    uint32_t before;
    uint32_t after;
    uint8_t attempt;

    if (config == NULL) {
        return 0U;
    }
    for (attempt = 0U; attempt < 4U; ++attempt) {
        before = app_rc_active_sequence;
        if ((before & 1U) != 0U) {
            continue;
        }
        __asm volatile ("dmb 0xF" ::: "memory");
        *config = app_rc_active;
        __asm volatile ("dmb 0xF" ::: "memory");
        after = app_rc_active_sequence;
        if (before == after) {
            if (config->generation == 0U) {
                /*
                 * 首次 PublishActive 之前静态区还是全零：全零配置会让
                 * 百分比判定 span=0 恒回 0（方向安全但不可用）。统一回退
                 * 出厂默认，读方拿到的永远是一份可解析的映射。
                 */
                APP_RcConfig_Defaults(config);
            }
            return 1U;
        }
    }
    return 0U;
}

uint32_t APP_RcConfig_GetActiveGeneration(void)
{
    return app_rc_active_generation;
}
