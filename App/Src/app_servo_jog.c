#include "app_servo_jog.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_control.h"

/*
 * 地面机械校准点动（SERVO JOG，保持型）。
 *
 * 背景：稳定环 commit 在非手势标定状态下持续流式下发舵机目标（3µs 死区 +
 * 500ms 强制刷新 + 10ms 总线槽）。一次性 BSP_BusServo_Move 慢移必然在下一次
 * 强制刷新时被稳定环目标覆盖——机械校准页表现为"走到一半被拉回中点附近"。
 *
 * 因此点动不再直接碰总线，而是在稳定环 commit 的输出仲裁点改写目标脉宽
 * （见 app_stabilizer.c）：优先级 手势标定 > 验收覆盖 > 反馈台架 > 解锁 > 点动，
 * 任一更高优先级出现，点动立即让位并通告。接管期间从稳定环当前目标起以
 * APP_SERVO_JOG_SLEW_US_PER_S 匀速斜坡走到点动目标并保持；距最后一次请求
 * 超过 APP_SERVO_JOG_HOLD_TIMEOUT_MS 自动交还，防止遗忘的点动长期霸占输出。
 *
 * 斜坡速率随请求走，不是全局参数（R-S7-7）：
 * - APP_ServoJog_Request（`SERVO JOG ch us`）= 500µs/s 慢速，mechanical.py
 *   拆桨标定专用，慢是为了机构受力可控，语义冻结；
 * - APP_ServoJog_RequestImmediate（`SERVO JOG ch us NOW`）= 一拍到位，
 *   servo_debug.py 调试页专用，响应对齐总线 SERVO MOVE 小 time 值。
 * 两者只共用"接管/保持/让位/超时"这套仲裁，不共用速率参数。
 *
 * 上下文契约（同 app_servo_cal.c）：
 * - HandleCommand 运行在通信任务上下文，允许直接 APP_Control_QueueText；
 * - Apply / ForceRelease 运行在 500Hz 控制环，绝不能同步排队 USB 文本，
 *   事件写入单条通告缓冲，由 app_control_tick_common 取走后再排队；
 * - 请求侧只写 requested/target/request_ms（M7 对齐标量写为原子），
 *   斜坡状态（engaged/current_x256）仅控制环访问。
 */

typedef struct {
    volatile uint8_t requested;   /* 请求侧写入：期望点动接管该通道 */
    volatile uint16_t target_us;
    volatile uint16_t slew_us_per_s; /* 本次请求的斜坡速率；0 = 一拍到位 */
    volatile uint32_t request_ms;
    uint8_t engaged;              /* 控制环视角：斜坡已从流式目标接管 */
    int32_t current_x256;         /* 当前斜坡位置 [µs<<8]，仅控制环访问 */
} ServoJogChannel;

static ServoJogChannel servo_jog_channel[APP_SERVO_JOG_CHANNEL_COUNT];
static uint32_t servo_jog_last_apply_ms;

static char servo_jog_notice_text[64];
static volatile uint8_t servo_jog_notice_pending;

static void servo_jog_post_notice(const char *format, ...)
{
    va_list args;
    int written;

    va_start(args, format);
    written = vsnprintf(servo_jog_notice_text, sizeof(servo_jog_notice_text),
                        format, args);
    va_end(args);

    if (written > 0) {
        servo_jog_notice_pending = 1U;
    }
}

void APP_ServoJog_Init(void)
{
    memset(servo_jog_channel, 0, sizeof(servo_jog_channel));
    servo_jog_last_apply_ms = 0U;
    servo_jog_notice_pending = 0U;
    servo_jog_notice_text[0] = '\0';
}

uint8_t APP_ServoJog_IsActive(void)
{
    return ((servo_jog_channel[0].requested != 0U) ||
            (servo_jog_channel[1].requested != 0U)) ? 1U : 0U;
}

uint16_t APP_ServoJog_TakeNotice(char *out, uint16_t capacity)
{
    size_t length;

    if ((out == NULL) || (capacity == 0U) || (servo_jog_notice_pending == 0U)) {
        return 0U;
    }
    /* 先清标志再拷贝：拷贝期间控制环若写入新事件会重新置位，下个 Tick 取走。 */
    servo_jog_notice_pending = 0U;
    length = strlen(servo_jog_notice_text);
    if (length >= capacity) {
        length = (size_t)capacity - 1U;
    }
    memcpy(out, servo_jog_notice_text, length);
    out[length] = '\0';
    return (uint16_t)length;
}

static uint8_t servo_jog_request_slew(uint32_t channel, uint32_t target_us,
                                      uint32_t now_ms, uint16_t slew_us_per_s)
{
    if ((channel >= APP_SERVO_JOG_CHANNEL_COUNT) ||
        (target_us < APP_SERVO_JOG_MIN_US) ||
        (target_us > APP_SERVO_JOG_MAX_US)) {
        return 0U;
    }
    /* requested 最后写：斜坡速率必须先于接管标志对控制环可见。 */
    servo_jog_channel[channel].target_us = (uint16_t)target_us;
    servo_jog_channel[channel].slew_us_per_s = slew_us_per_s;
    servo_jog_channel[channel].request_ms = now_ms;
    servo_jog_channel[channel].requested = 1U;
    return 1U;
}

uint8_t APP_ServoJog_Request(uint32_t channel, uint32_t target_us, uint32_t now_ms)
{
    return servo_jog_request_slew(channel, target_us, now_ms,
                                  (uint16_t)APP_SERVO_JOG_SLEW_US_PER_S);
}

uint8_t APP_ServoJog_RequestImmediate(uint32_t channel, uint32_t target_us,
                                      uint32_t now_ms)
{
    return servo_jog_request_slew(channel, target_us, now_ms,
                                  (uint16_t)APP_SERVO_JOG_SLEW_IMMEDIATE);
}

void APP_ServoJog_ReleaseAll(void)
{
    servo_jog_channel[0].requested = 0U;
    servo_jog_channel[1].requested = 0U;
}

void APP_ServoJog_ForceRelease(const char *reason)
{
    if (APP_ServoJog_IsActive() == 0U) {
        return;
    }
    APP_ServoJog_ReleaseAll();
    servo_jog_post_notice("ERR servo_jog released (%s)\r\n",
                          (reason != NULL) ? reason : "yield");
}

static void servo_jog_channel_apply(ServoJogChannel *channel,
                                    uint32_t channel_index,
                                    uint32_t now_ms,
                                    uint32_t dt_ms,
                                    uint16_t *pulse_us)
{
    int32_t target_x256;
    int32_t step_x256;
    uint16_t slew_us_per_s;

    if (channel->requested == 0U) {
        channel->engaged = 0U;
        return;
    }

    if ((uint32_t)(now_ms - channel->request_ms) >= APP_SERVO_JOG_HOLD_TIMEOUT_MS) {
        channel->requested = 0U;
        channel->engaged = 0U;
        servo_jog_post_notice("OK servo_jog released timeout ch=%lu\r\n",
                              (unsigned long)channel_index);
        return;
    }

    if (channel->engaged == 0U) {
        /* 从稳定环当前目标接管，保证无跳变。 */
        channel->current_x256 = (int32_t)((uint32_t)*pulse_us << 8);
        channel->engaged = 1U;
    }

    target_x256 = (int32_t)((uint32_t)channel->target_us << 8);
    slew_us_per_s = channel->slew_us_per_s;

    if (slew_us_per_s == 0U) {
        /* 调试页即时通路：本拍直接到位，随后与慢速点动同样保持并计超时。 */
        channel->current_x256 = target_x256;
        *pulse_us = channel->target_us;
        return;
    }

    step_x256 = (int32_t)(((uint32_t)slew_us_per_s * 256UL * dt_ms) / 1000UL);
    if (step_x256 < 1) {
        step_x256 = 1;
    }

    if (channel->current_x256 < target_x256) {
        channel->current_x256 += step_x256;
        if (channel->current_x256 > target_x256) {
            channel->current_x256 = target_x256;
        }
    } else if (channel->current_x256 > target_x256) {
        channel->current_x256 -= step_x256;
        if (channel->current_x256 < target_x256) {
            channel->current_x256 = target_x256;
        }
    }

    *pulse_us = (uint16_t)((uint32_t)(channel->current_x256 + 128) >> 8);
}

void APP_ServoJog_Apply(uint32_t now_ms,
                        const char *yield_reason,
                        uint16_t *alpha_pulse_us,
                        uint16_t *beta_pulse_us)
{
    uint32_t dt_ms;

    if (yield_reason != NULL) {
        APP_ServoJog_ForceRelease(yield_reason);
        servo_jog_last_apply_ms = now_ms;
        return;
    }

    dt_ms = now_ms - servo_jog_last_apply_ms;
    servo_jog_last_apply_ms = now_ms;
    if (dt_ms < 1U) {
        dt_ms = 1U;
    }
    if (dt_ms > 20U) {
        dt_ms = 20U;
    }

    if (alpha_pulse_us != NULL) {
        servo_jog_channel_apply(&servo_jog_channel[0], 0U, now_ms, dt_ms,
                                alpha_pulse_us);
    }
    if (beta_pulse_us != NULL) {
        servo_jog_channel_apply(&servo_jog_channel[1], 1U, now_ms, dt_ms,
                                beta_pulse_us);
    }
}

void APP_ServoJog_HandleCommand(char *tokens[], uint32_t count, uint32_t now_ms)
{
    if ((count >= 3U) && (strcmp(tokens[2], "STOP") == 0)) {
        APP_ServoJog_ReleaseAll();
        APP_Control_QueueText("OK servo_jog released manual\r\n");
        return;
    }

    /* 4 词 = 标定页慢速点动（语义冻结）；第 5 词只认 NOW = 调试页即时通路。 */
    if ((count == 4U) || ((count == 5U) && (strcmp(tokens[4], "NOW") == 0))) {
        char *end = NULL;
        unsigned long channel = strtoul(tokens[2], &end, 10);

        if ((end != NULL) && (*end == '\0')) {
            unsigned long target_us = strtoul(tokens[3], &end, 10);

            if ((end != NULL) && (*end == '\0')) {
                if (count == 5U) {
                    if (APP_ServoJog_RequestImmediate((uint32_t)channel,
                                                      (uint32_t)target_us,
                                                      now_ms) != 0U) {
                        APP_Control_QueueText(
                            "OK servo_jog ch=%lu target=%lu slew=immediate hold_s=%lu\r\n",
                            channel, target_us,
                            (unsigned long)(APP_SERVO_JOG_HOLD_TIMEOUT_MS / 1000UL));
                        return;
                    }
                } else if (APP_ServoJog_Request((uint32_t)channel,
                                                (uint32_t)target_us,
                                                now_ms) != 0U) {
                    APP_Control_QueueText(
                        "OK servo_jog ch=%lu target=%lu slew=%u hold_s=%lu\r\n",
                        channel, target_us,
                        (unsigned int)APP_SERVO_JOG_SLEW_US_PER_S,
                        (unsigned long)(APP_SERVO_JOG_HOLD_TIMEOUT_MS / 1000UL));
                    return;
                }
            }
        }
    }

    APP_Control_QueueText(
        "ERR usage SERVO JOG ch(0..1) pulse(500..2500) [NOW] | SERVO JOG STOP\r\n");
}
