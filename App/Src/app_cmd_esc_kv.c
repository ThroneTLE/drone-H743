/*
 * ESC KV：经 DShot 写 AM32 电机 KV。
 *
 * `ESC KV <kv> CONFIRM | ESC KV ?` —— 把两路 AM32 电调的“电机 KV”设置写成 <kv>。
 *
 * 作者 2026-09-23 授权：推力台实测推力在 50% 油门后不再上升，根因是 AM32 默认
 * KV=2220 触发低转速功率限制（见 app_esc_command.h）。作者没有 AM32 配置器，
 * 所以由飞控经同一根 DShot 信号线写入。
 *
 * 帧序列是编译期常量（app_esc_command.c），命令行只提供 KV 值，且只接受
 * 300..1900 中 AM32 能精确表示的值；末尾必须带 CONFIRM，防止敲错一行就写进电调。
 * DShot 单向，电调不回 ACK：回包只能证明“15 帧已连续发出”。是否生效看电调重新
 * 上电后的最高转速（最大推力测试），不能把 sent=15 说成“已改好”。
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "app_esc_command.h"
#include "app_stabilizer.h"
#include "bsp_esc_protocol.h"

#include <stddef.h>
#include <string.h>

static uint16_t esc_kv_requested;

static void esc_kv_report(const char *event)
{
    uint8_t kv_byte = 0U;

    (void)APP_EscCommand_MotorKvByte(esc_kv_requested, &kv_byte);
    APP_Control_QueueText(
        "ESC KV event=%s state=%s kv=%u byte=%u sent=%lu frames=%u reason=%s proto=%u\r\n",
        event,
        APP_EscCommand_StateName(APP_EscCommand_GetState()),
        (unsigned int)esc_kv_requested,
        (unsigned int)kv_byte,
        (unsigned long)APP_EscCommand_SentFrames(),
        (unsigned int)APP_EscCommand_PlannedFrames(),
        APP_EscCommand_AbortReasonName(APP_EscCommand_LastAbortReason()),
        (unsigned int)BSP_ESC_PROTOCOL);
}

uint8_t app_control_handle_esc_kv(char **tokens, uint32_t count)
{
    uint32_t kv = 0U;
    uint8_t kv_byte = 0U;

    if ((tokens == NULL) || (count < 3U) || (strcmp(tokens[0], "ESC") != 0) ||
        (strcmp(tokens[1], "KV") != 0)) {
        return 0U;
    }
    if ((count == 3U) && (strcmp(tokens[2], "?") == 0)) {
        esc_kv_report("status");
        return 1U;
    }
    if ((count != 4U) || (strcmp(tokens[3], "CONFIRM") != 0) ||
        !app_control_parse_u32(tokens[2], &kv)) {
        APP_Control_QueueText(
            "ESC KV event=rejected reason=usage usage=ESC KV <300..1900> CONFIRM\r\n");
        return 1U;
    }
    if ((kv > 0xFFFFU) || (APP_EscCommand_MotorKvByte((uint16_t)kv, &kv_byte) == 0U)) {
        APP_Control_QueueText(
            "ESC KV event=rejected reason=kv_not_representable kv=%lu rule=20+40n,300..1900\r\n",
            (unsigned long)kv);
        return 1U;
    }

#if !BSP_ESC_PROTOCOL_IS_DSHOT
    APP_Control_QueueText(
        "ESC KV event=rejected reason=not_dshot proto=%u\r\n",
        (unsigned int)BSP_ESC_PROTOCOL);
    return 1U;
#else
    /* 与 ESC EDT 同两道门：这里给可读理由，500 Hz 侧的 Step() 负责真正拦住。 */
    if (APP_Stabilizer_IsArmed() != 0U) {
        APP_Control_QueueText("ESC KV event=rejected reason=armed\r\n");
        return 1U;
    }
    if (APP_EscCommand_RequestMotorKv((uint16_t)kv) == 0U) {
        APP_Control_QueueText("ESC KV event=rejected reason=busy\r\n");
        return 1U;
    }
    esc_kv_requested = (uint16_t)kv;
    esc_kv_report("queued");
    return 1U;
#endif
}
