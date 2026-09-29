#include "app_led.h"
#include "app_battery.h"

#include "app_led_config.h"
#include "app_optical_flow.h"
#include "bsp_rgb_led.h"
#include "svc_led.h"
#include "svc_timestamp.h"

#include <string.h>

/*
 * 状态灯**策略**：此刻哪一种状态成立，该往哪个源发布哪条绑定。
 *
 * 这一层不知道灯接在哪根脚上（那在 bsp_rgb_led.c），不自己算闪烁相位
 * （那在 drv_rgb_led.c），不排"谁压谁"的 if-else 链（那在 svc_led.c），
 * 从 2026-09-12 起也**不再决定颜色**——颜色与节奏都从 `APP_LedConfig` 读，
 * 用户可以在上位机改并存进 Flash（`app_cmd_ledmap.c`）。
 *
 * 本文件唯一还硬编码的东西是**解锁被拒的闪烁次数**：它就是原因码本身，
 * 在这里注入，永远不经过配置。配置里连这个字段都没有——这样"现场数到 N 下"
 * 和 `ARM? block=N` 结构上不可能对不上，而不是"我们记得别改它"。
 *
 * 颜色语言（默认值）见 App/Src/app_led_config.c 的 led_default_binding[]，
 * 以及 doc/micoair743v2/README.md 的「状态灯」一节。
 */

#define APP_LED_POLICY_PERIOD_MS 20U

static volatile uint8_t app_led_armed;
static volatile uint8_t app_led_arm_published;
static volatile APP_LED_ArmBlockReason app_led_arm_block_reason =
    APP_LED_ARM_BLOCK_NO_RC;
static volatile APP_LED_ServoCalMode app_led_servo_cal_mode =
    APP_LED_SERVO_CAL_NONE;
static uint32_t app_led_policy_ms;
static volatile uint32_t app_led_ticks;

/* 取一条绑定的图案，并注入"闪几下"——那是运行期常量，配置里没有这个字段。 */
static void app_led_pattern(uint8_t binding_id, DRV_RgbPattern *pattern)
{
    APP_LedConfig_GetPattern(binding_id, pattern);
    /*
     * 次数从 APP_LedConfig_PulseCount 取，不在这里重写一遍偏移算式。
     * 写第二遍的代价刚刚付过：算式只覆盖了 BLOCK 段，`flow_failed` 落在外面
     * 拿到 count=0，于是光流失败时灯全黑（PULSES 遇 count=0 直接返回黑），
     * 而且因为 WARNING 压着 STATUS/HEARTBEAT，连心跳都被盖住。
     */
    pattern->count = APP_LedConfig_PulseCount(binding_id);
}

static void app_led_publish(SVC_LedSource source, uint8_t binding_id)
{
    DRV_RgbPattern pattern;

    app_led_pattern(binding_id, &pattern);
    SVC_Led_Publish(source, &pattern);
}

static void app_led_publish_servo_cal(APP_LED_ServoCalMode mode)
{
    uint8_t binding;

    switch (mode) {
    /* 扭矩已释放，可以用手掰舵机——这一档要提示"现在别通电测试"。 */
    case APP_LED_SERVO_CAL_RELEASED: binding = APP_LED_BIND_CAL_RELEASED; break;
    case APP_LED_SERVO_CAL_SAVE_ACK: binding = APP_LED_BIND_CAL_SAVE_ACK; break;
    case APP_LED_SERVO_CAL_ERROR:    binding = APP_LED_BIND_CAL_ERROR;    break;
    case APP_LED_SERVO_CAL_NONE:
    default:
        SVC_Led_Publish(SVC_LED_SOURCE_CALIBRATION, NULL);
        return;
    }
    app_led_publish(SVC_LED_SOURCE_CALIBRATION, binding);
}

static void app_led_publish_arm(void)
{
    APP_LED_ArmBlockReason reason = app_led_arm_block_reason;
    uint8_t binding;

    if (app_led_armed != 0U) {
        /*
         * 已解锁时这盏灯只说一件事：电机带电。任何拒绝原因（含低压）都不准
         * 盖掉它——3S 带载掉到 10.5 V 以下是常态，低压告警是个大半时间熄灭的
         * 8 闪图案，拿它换掉常亮红灯，等于在桨还在转的时候告诉旁边的人"已上锁"。
         * 飞行中的低压提示走上位机解锁横幅、电池页和 ELRS 回传——那才是飞手
         * 在看的地方；灯只在**未解锁**时用来解释"为什么解不了锁"。
         */
        app_led_publish(SVC_LED_SOURCE_ARMED, APP_LED_BIND_ARMED);
        SVC_Led_Publish(SVC_LED_SOURCE_BLOCKED, NULL);
        SVC_Led_Publish(SVC_LED_SOURCE_STATUS, NULL);
        /*
         * 上位机点名（IDENTIFY）在 svc_led 的枚举里排在 ARMED 之前，所以
         * `LED RGB r g b 0` 这种不带到期的点名会一直盖住解锁红灯——和低压告警
         * 曾经犯的是同一个错。点名是地面上"哪块板是哪块"的调试用途，解锁之后
         * 它没有任何理由比"桨随时会转"更该被看见。
         *
         * 放在每拍策略里撤销而不是只在命令入口拦：不管是谁、什么时候发布的
         * 点名，解锁后最多一拍（APP_LED_POLICY_PERIOD_MS）就会被收回。
         */
        SVC_Led_Publish(SVC_LED_SOURCE_IDENTIFY, NULL);
        return;
    }
    SVC_Led_Publish(SVC_LED_SOURCE_ARMED, NULL);

    if (app_led_arm_published == 0U) {
        /* 控制环还没跑过一圈：这时候既不能说"就绪"也不能说原因，
         * 两种都是编出来的。让位给心跳。 */
        SVC_Led_Publish(SVC_LED_SOURCE_BLOCKED, NULL);
        SVC_Led_Publish(SVC_LED_SOURCE_STATUS, NULL);
        return;
    }

    if (reason==APP_LED_ARM_BLOCK_BATTERY) {
        APP_Battery_PublishLedWarning();return;
    }

    if (reason == APP_LED_ARM_BLOCK_NONE) {
        app_led_publish(SVC_LED_SOURCE_STATUS, APP_LED_BIND_READY);
        SVC_Led_Publish(SVC_LED_SOURCE_BLOCKED, NULL);
        return;
    }

    /*
     * 原因码 → 绑定 ID 是一一对应的，闪烁次数由 app_led_pattern() 按 ID 反推。
     * 认不出的原因码（将来追加了枚举却忘了加绑定）落回第一条而不是不显示：
     * 不显示 = 用户看不到任何被拒提示，比显示错次数更糟。
     */
    binding = APP_LedConfig_BindingForBlockReason((uint8_t)reason);
    if (binding >= (uint8_t)APP_LED_BIND_COUNT) {
        binding = (uint8_t)APP_LED_BIND_BLOCK_BASE;
    }
    app_led_publish(SVC_LED_SOURCE_BLOCKED, binding);
    SVC_Led_Publish(SVC_LED_SOURCE_STATUS, NULL);
}

static void app_led_publish_warning(void)
{
    APP_OPTICAL_FLOW_Status flow_status;
    uint8_t binding;

    APP_OpticalFlow_GetStatus(&flow_status);
    switch (flow_status.health) {
    case APP_OPTICAL_FLOW_HEALTH_STARTING: binding = APP_LED_BIND_FLOW_STARTING; break;
    case APP_OPTICAL_FLOW_HEALTH_RETRYING: binding = APP_LED_BIND_FLOW_RETRYING; break;
    case APP_OPTICAL_FLOW_HEALTH_FAILED:   binding = APP_LED_BIND_FLOW_FAILED;   break;
    case APP_OPTICAL_FLOW_HEALTH_OK:
    default:
        SVC_Led_Publish(SVC_LED_SOURCE_WARNING, NULL);
        return;
    }
    app_led_publish(SVC_LED_SOURCE_WARNING, binding);
}

static void app_led_publish_state(void)
{
    app_led_publish(SVC_LED_SOURCE_HEARTBEAT, APP_LED_BIND_HEARTBEAT);
    app_led_publish_servo_cal(app_led_servo_cal_mode);
    app_led_publish_arm();
    app_led_publish_warning();
}

void APP_LED_Task_Init(void)
{
    BSP_RgbLed_Init();
    SVC_Led_Init(BSP_RgbLed_WriteBits);
    /*
     * 心跳在每拍策略里重发，不是这里发一次就完事。
     *
     * svc_led 存的是图案的**副本**（按值），所以改了配置之后不重发就不会生效。
     * 心跳恰恰是最不容易被发现没生效的一条——它平时被别的源盖着，等到真需要
     * 它出场（控制环还没发话）时，用户看到的是上一份颜色，而他早就改过了。
     */
    app_led_policy_ms = 0U;
}

void APP_LED_Task_Step(void)
{
    uint32_t now_ms = SVC_Timestamp_Ms();

    ++app_led_ticks;
    /*
     * 调制要按 1 kHz 走（Σ-Δ 就是靠节拍密度表达亮度），但"现在该亮什么"没必要
     * 每毫秒重算一遍——状态最快也就几十毫秒变一次，而重算要问光流健康。
     */
    if ((uint32_t)(now_ms - app_led_policy_ms) >= APP_LED_POLICY_PERIOD_MS) {
        app_led_policy_ms = now_ms;
        app_led_publish_state();
    }
    SVC_Led_Tick(now_ms);
}

void APP_LED_SetArmStatus(uint8_t armed, APP_LED_ArmBlockReason reason)
{
    app_led_armed = (armed != 0U) ? 1U : 0U;
    app_led_arm_block_reason = reason;
    app_led_arm_published = 1U;
}

void APP_LED_SetServoCalMode(APP_LED_ServoCalMode mode)
{
    app_led_servo_cal_mode = mode;
}

void APP_LED_Identify(const DRV_RgbColor *color, uint32_t hold_ms)
{
    DRV_RgbPattern pattern;

    if (color == NULL) {
        SVC_Led_Publish(SVC_LED_SOURCE_IDENTIFY, NULL);
        return;
    }
    memset(&pattern, 0, sizeof(pattern));
    pattern.color = *color;
    pattern.effect = DRV_RGB_EFFECT_SOLID;
    SVC_Led_PublishFor(SVC_LED_SOURCE_IDENTIFY, &pattern, hold_ms, SVC_Timestamp_Ms());
}

void APP_LED_IdentifyPattern(const DRV_RgbPattern *pattern, uint32_t hold_ms)
{
    if (pattern == NULL) {
        SVC_Led_Publish(SVC_LED_SOURCE_IDENTIFY, NULL);
        return;
    }
    SVC_Led_PublishFor(SVC_LED_SOURCE_IDENTIFY, pattern, hold_ms, SVC_Timestamp_Ms());
}

void APP_LED_GetDebug(APP_LED_Debug *debug)
{
    if (debug == NULL) {
        return;
    }
    debug->source = (uint8_t)SVC_Led_ActiveSource();
    debug->color = SVC_Led_ActiveColor();
    debug->armed = app_led_armed;
    debug->arm_published = app_led_arm_published;
    debug->block_reason = (uint8_t)app_led_arm_block_reason;
    debug->servo_cal = (uint8_t)app_led_servo_cal_mode;
    debug->active_low = BSP_RgbLed_IsActiveLow();
    debug->ticks = app_led_ticks;
}
