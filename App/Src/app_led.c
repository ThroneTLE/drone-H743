#include "app_led.h"
#include "app_battery.h"

#include "app_optical_flow.h"
#include "bsp_rgb_led.h"
#include "svc_led.h"
#include "svc_timestamp.h"

/*
 * 状态灯**策略**：什么状态该亮成什么样。
 *
 * 这一层不知道灯接在哪根脚上（那在 bsp_rgb_led.c），也不自己算闪烁相位
 * （那在 drv_rgb_led.c），更不排"谁压谁"的 if-else 链（那在 svc_led.c）。
 * 它只做一件事：把飞控状态翻译成"往哪个源发布哪个图案"。
 *
 * ---- 颜色语言 ----
 *
 *   红 常亮      已解锁。桨随时可能转——这是整块板上最该一眼看见的事实，
 *                所以它压过除"人工点名"之外的一切。
 *   绿 呼吸      就绪，可以解锁。**这个状态原来在本板上完全没有指示**：
 *                老代码里它是"蓝灯灭 + LED_4 亮"，而 LED_4 映射到 PD10——
 *                MicoAir743v2 上那是根空闲脚，没有灯。结果条件一旦全部满足，
 *                灯就直接黑掉，和"固件死了""板子没电"长得一模一样。
 *   琥珀 数闪 N  解锁被拒，N = APP_LED_ArmBlockReason 的数值（1..7）。
 *                数值与闪烁次数的对应关系照搬原设计，现场记忆不作废。
 *   青 闪        非阻塞告警（目前只有光流），节奏区分 starting/retrying/failed。
 *   蓝 快闪      控制环还没发布过解锁状态，也就是刚上电那几百毫秒。
 *   任意        上位机点名（`LED RGB/BLINK/BREATHE`），颜色由命令给，压过一切。
 *
 * 琥珀而不是黄：满绿加满红在这颗灯珠上偏绿，容易和"就绪"的绿混。
 */

#define APP_LED_POLICY_PERIOD_MS 20U

/* 数闪的节奏。沿用原实现的 160/160 + 760，现场数灯的手感不变。 */
#define APP_LED_PULSE_ON_MS  160U
#define APP_LED_PULSE_OFF_MS 160U
#define APP_LED_PULSE_GAP_MS 760U

static const DRV_RgbColor app_led_red    = {255U,   0U,   0U};
static const DRV_RgbColor app_led_green  = {  0U, 255U,   0U};
static const DRV_RgbColor app_led_blue   = {  0U,   0U, 255U};
static const DRV_RgbColor app_led_amber  = {255U, 110U,   0U};
static const DRV_RgbColor app_led_cyan   = {  0U, 200U, 255U};

static volatile uint8_t app_led_armed;
static volatile uint8_t app_led_arm_published;
static volatile APP_LED_ArmBlockReason app_led_arm_block_reason =
    APP_LED_ARM_BLOCK_NO_RC;
static volatile APP_LED_ServoCalMode app_led_servo_cal_mode =
    APP_LED_SERVO_CAL_NONE;
static uint32_t app_led_policy_ms;
static volatile uint32_t app_led_ticks;

static DRV_RgbPattern app_led_solid(DRV_RgbColor color)
{
    DRV_RgbPattern pattern = {0};

    pattern.color = color;
    pattern.effect = DRV_RGB_EFFECT_SOLID;
    return pattern;
}

static DRV_RgbPattern app_led_blink(DRV_RgbColor color, uint16_t on_ms, uint16_t off_ms)
{
    DRV_RgbPattern pattern = {0};

    pattern.color = color;
    pattern.effect = DRV_RGB_EFFECT_BLINK;
    pattern.on_ms = on_ms;
    pattern.off_ms = off_ms;
    return pattern;
}

static DRV_RgbPattern app_led_pulses(DRV_RgbColor color, uint8_t count)
{
    DRV_RgbPattern pattern = {0};

    pattern.color = color;
    pattern.effect = DRV_RGB_EFFECT_PULSES;
    pattern.on_ms = APP_LED_PULSE_ON_MS;
    pattern.off_ms = APP_LED_PULSE_OFF_MS;
    pattern.gap_ms = APP_LED_PULSE_GAP_MS;
    pattern.count = count;
    return pattern;
}

static DRV_RgbPattern app_led_breathe(DRV_RgbColor color, uint16_t period_ms, uint8_t dim)
{
    DRV_RgbPattern pattern = {0};

    pattern.color = color;
    pattern.effect = DRV_RGB_EFFECT_BREATHE;
    pattern.period_ms = period_ms;
    pattern.dim = dim;
    return pattern;
}

static void app_led_publish_servo_cal(APP_LED_ServoCalMode mode)
{
    DRV_RgbPattern pattern;

    switch (mode) {
    case APP_LED_SERVO_CAL_RELEASED:
        /* 扭矩已释放，可以用手掰舵机——急闪提示"现在别通电测试"。 */
        pattern = app_led_blink(app_led_cyan, 120U, 120U);
        break;
    case APP_LED_SERVO_CAL_SAVE_ACK:
        pattern = app_led_blink(app_led_green, 320U, 320U);
        break;
    case APP_LED_SERVO_CAL_ERROR:
        pattern = app_led_blink(app_led_red, 80U, 80U);
        break;
    case APP_LED_SERVO_CAL_NONE:
    default:
        SVC_Led_Publish(SVC_LED_SOURCE_CALIBRATION, NULL);
        return;
    }
    SVC_Led_Publish(SVC_LED_SOURCE_CALIBRATION, &pattern);
}

static void app_led_publish_arm(void)
{
    APP_LED_ArmBlockReason reason = app_led_arm_block_reason;
    if (reason==APP_LED_ARM_BLOCK_BATTERY) {
        APP_Battery_PublishLedWarning();return;
    }
    DRV_RgbPattern pattern;

    if ((app_led_armed != 0U) && (reason != APP_LED_ARM_BLOCK_BATTERY)) {
        pattern = app_led_solid(app_led_red);
        SVC_Led_Publish(SVC_LED_SOURCE_ARMED, &pattern);
        SVC_Led_Publish(SVC_LED_SOURCE_BLOCKED, NULL);
        SVC_Led_Publish(SVC_LED_SOURCE_STATUS, NULL);
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

    if (reason == APP_LED_ARM_BLOCK_NONE) {
        pattern = app_led_breathe(app_led_green, 2600U, 10U);
        SVC_Led_Publish(SVC_LED_SOURCE_BLOCKED, NULL);
        SVC_Led_Publish(SVC_LED_SOURCE_STATUS, &pattern);
        return;
    }

    pattern = app_led_pulses(app_led_amber, (uint8_t)reason);
    SVC_Led_Publish(SVC_LED_SOURCE_BLOCKED, &pattern);
    SVC_Led_Publish(SVC_LED_SOURCE_STATUS, NULL);
}

static void app_led_publish_warning(void)
{
    APP_OPTICAL_FLOW_Status flow_status;
    DRV_RgbPattern pattern;

    APP_OpticalFlow_GetStatus(&flow_status);
    switch (flow_status.health) {
    case APP_OPTICAL_FLOW_HEALTH_STARTING:
        pattern = app_led_blink(app_led_cyan, 500U, 500U);
        break;
    case APP_OPTICAL_FLOW_HEALTH_RETRYING:
        pattern = app_led_blink(app_led_cyan, 160U, 160U);
        break;
    case APP_OPTICAL_FLOW_HEALTH_FAILED:
        pattern = app_led_pulses(app_led_cyan, 3U);
        break;
    case APP_OPTICAL_FLOW_HEALTH_OK:
    default:
        SVC_Led_Publish(SVC_LED_SOURCE_WARNING, NULL);
        return;
    }
    SVC_Led_Publish(SVC_LED_SOURCE_WARNING, &pattern);
}

static void app_led_publish_state(void)
{
    app_led_publish_servo_cal(app_led_servo_cal_mode);
    app_led_publish_arm();
    app_led_publish_warning();
}

void APP_LED_Task_Init(void)
{
    DRV_RgbPattern heartbeat;

    BSP_RgbLed_Init();
    SVC_Led_Init(BSP_RgbLed_WriteBits);
    /* 心跳常驻最低优先级：只要没人有更要紧的话说，它就证明固件还在跑。 */
    heartbeat = app_led_blink(app_led_blue, 120U, 380U);
    SVC_Led_Publish(SVC_LED_SOURCE_HEARTBEAT, &heartbeat);
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
    pattern = app_led_solid(*color);
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
