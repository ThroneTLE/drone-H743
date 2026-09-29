#include "app_prop_spin.h"

#include <string.h>

/*
 * 状态放在文件级静态量里而不是调用方的 context：这个窗口全机只有一个，
 * 而且收命令的文本任务和执行的控制环必须看到**同一个**它。两份状态意味着
 * "上位机以为关了、控制环还在转"——这正是本模块存在的理由所要排除的情形。
 *
 * 几个跨任务读写的量标 volatile：控制环在 500 Hz 上读 active/percent，
 * 文本任务在命令到达时写它们。
 */
static volatile uint8_t  spin_active;
static volatile uint8_t  spin_channel;
static volatile uint8_t  spin_percent[APP_PROP_SPIN_CHANNEL_COUNT];
static volatile uint8_t  spin_max_percent = APP_PROP_SPIN_DEFAULT_MAX_PERCENT;
static volatile uint32_t spin_last_heartbeat_ms;
static volatile uint8_t  spin_last_stop_reason;

static void spin_clear_outputs(void)
{
    uint32_t i;

    for (i = 0U; i < APP_PROP_SPIN_CHANNEL_COUNT; i++) {
        spin_percent[i] = 0U;
    }
    spin_channel = 0U;
}

void APP_PropSpin_Reset(void)
{
    spin_active = 0U;
    spin_clear_outputs();
    spin_max_percent = APP_PROP_SPIN_DEFAULT_MAX_PERCENT;
    spin_last_heartbeat_ms = 0U;
    spin_last_stop_reason = (uint8_t)APP_PROP_SPIN_STOP_NONE;
}

uint8_t APP_PropSpin_Open(uint32_t now_ms, uint8_t max_percent)
{
    if ((max_percent == 0U) ||
        (max_percent > (uint8_t)APP_PROP_SPIN_HARD_MAX_PERCENT)) {
        /*
         * 非法上限不开窗，**也不碰已经开着的窗口**。后半句是故意的：一条参数写
         * 错的 ARM 不该顺手把正在转的电机停掉，那会让操作者在一次手滑之后失去
         * 对当前这一路的判断依据；它该做的只是"这条命令不算数"。
         */
        return 0U;
    }

    if (spin_active != 0U) {
        /*
         * 已经开着就当成一次心跳，但**不保留上一次的油门**——重连之后的第一拍
         * 应该是停着的，让操作者重新决定给多少，而不是接着上次的转速往下跑。
         *
         * 上限只降不升，理由见头文件。同值重发因此仍然是幂等的。
         */
        if (max_percent < spin_max_percent) {
            spin_max_percent = max_percent;
        }
        spin_clear_outputs();
        spin_last_heartbeat_ms = now_ms;
        return 1U;
    }

    spin_clear_outputs();
    spin_max_percent = max_percent;
    spin_last_heartbeat_ms = now_ms;
    spin_last_stop_reason = (uint8_t)APP_PROP_SPIN_STOP_NONE;
    /* active 最后置：在它置起来之前，控制环读到的输出已经是干净的 0，
     * 而上限也已经就位——不存在"窗口开了但还按上一次的上限判"的缝。 */
    spin_active = 1U;
    return 1U;
}

uint8_t APP_PropSpin_Command(uint32_t now_ms, uint8_t channel, uint8_t percent)
{
    uint32_t i;

    if (spin_active == 0U) {
        return 0U;
    }
    if ((channel == 0U) || (channel > (uint8_t)APP_PROP_SPIN_CHANNEL_COUNT) ||
        (percent > spin_max_percent)) {
        /*
         * 不是"忽略这一条"，是关窗。忽略的话，一个把百分比算错成 80 的上位机
         * 会一直被当作活着的心跳，电机保持上一次的转速，而界面上的数字在变——
         * 两边各自自洽，谁也不会发现。宁可当场停。
         */
        APP_PropSpin_Close(now_ms, (uint8_t)APP_PROP_SPIN_STOP_REJECTED);
        return 0U;
    }

    for (i = 0U; i < APP_PROP_SPIN_CHANNEL_COUNT; i++) {
        spin_percent[i] = ((i + 1U) == (uint32_t)channel) ? percent : 0U;
    }
    spin_channel = channel;
    spin_last_heartbeat_ms = now_ms;
    return 1U;
}

void APP_PropSpin_Close(uint32_t now_ms, uint8_t reason)
{
    (void)now_ms;

    /* 先停输出再落 active：两者之间被抢走 CPU 时，控制环看到的是
     * "还开着但油门是 0"，而不是"已经关了但油门还留着上一次的值"。 */
    spin_clear_outputs();
    spin_active = 0U;
    spin_last_stop_reason = reason;
}

void APP_PropSpin_Step(uint32_t now_ms, uint8_t inhibit)
{
    if (spin_active == 0U) {
        return;
    }
    if (inhibit != 0U) {
        APP_PropSpin_Close(now_ms, (uint8_t)APP_PROP_SPIN_STOP_INHIBIT);
        return;
    }
    /* 无符号差值：now_ms 回绕（约 49 天）时仍然得到真实的经过时间。 */
    if ((uint32_t)(now_ms - spin_last_heartbeat_ms) >= APP_PROP_SPIN_TIMEOUT_MS) {
        APP_PropSpin_Close(now_ms, (uint8_t)APP_PROP_SPIN_STOP_HEARTBEAT);
    }
}

uint8_t APP_PropSpin_IsActive(void)
{
    return spin_active;
}

void APP_PropSpin_GetOutput(APP_PropSpinOutput *out)
{
    uint32_t i;

    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (spin_active == 0U) {
        /* 关着的时候报默认上限而不是上一次用过的值：界面的滑条跟着这个数走，
         * 停下来之后它应该缩回 20%，而不是把 100% 留在那里等人误拖。 */
        out->max_percent = (uint8_t)APP_PROP_SPIN_DEFAULT_MAX_PERCENT;
        return;             /* channel=0、percent 全 0：调用方照发就是停机 */
    }
    out->active = 1U;
    out->channel = spin_channel;
    out->max_percent = spin_max_percent;
    for (i = 0U; i < APP_PROP_SPIN_CHANNEL_COUNT; i++) {
        out->percent[i] = spin_percent[i];
    }
}

uint8_t APP_PropSpin_LastStopReason(void)
{
    return spin_last_stop_reason;
}

uint32_t APP_PropSpin_HeartbeatAgeMs(uint32_t now_ms)
{
    if (spin_active == 0U) {
        return 0U;
    }
    return (uint32_t)(now_ms - spin_last_heartbeat_ms);
}

const char *APP_PropSpin_StopReasonName(uint8_t reason)
{
    switch ((APP_PropSpinStopReason)reason) {
    case APP_PROP_SPIN_STOP_NONE:      return "none";
    case APP_PROP_SPIN_STOP_REQUEST:   return "request";
    case APP_PROP_SPIN_STOP_HEARTBEAT: return "heartbeat_lost";
    case APP_PROP_SPIN_STOP_INHIBIT:   return "inhibited";
    case APP_PROP_SPIN_STOP_REJECTED:  return "rejected";
    default:                           return "unknown";
    }
}
