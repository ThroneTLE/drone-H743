/*
 * `ESC EDT ON|OFF|?` —— 打开/关闭电调的扩展遥测（EDT）。
 *
 * ──────────────── 为什么需要这条命令 ────────────────
 *
 * 双向 DShot 默认只回传电周期（转速）。**逐路电流属于 EDT**，而 EDT 不是电调配置
 * 器里的一个开关，是由飞控发一条 DShot 特殊命令（13 = enable / 14 = disable）打开的。
 * 在此之前本仓库根本不发特殊命令——`drv_dshot.h` 开头就写着"拒绝特殊命令 1..47"。
 *
 * ──────────────── 为什么命令号不从命令行取 ────────────────
 *
 * 1..47 里躺着 7/8/20/21（改电机转向）和 12（写电调 Flash）。做成 `ESC CMD <n>`
 * 意味着敲错一个数字就把飞机的转向改了、还存进了电调——那不是重启能恢复的。
 * 所以这里只认 `ON`/`OFF` 两个词，命令号是编译期常量。
 *
 * ──────────────── 为什么要回报"发完了"而不是"生效了" ────────────────
 *
 * DShot 是单向发送，电调不对特殊命令回 ACK。本命令能证明的只有"10 帧连续的
 * 命令帧已经提交出去"。**EDT 是否真的打开，唯一的判据是之后有没有收到 EDT 帧**
 * ——看 `PROPCAL escdiag` 的 fr 计数和电流字段。所以回包里写的是 sent=N，
 * 不是"已启用"。把"发出去了"说成"打开了"，会让人在下一步去查接收侧。
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "app_esc_command.h"
#include "app_stabilizer.h"
#include "bsp_esc_protocol.h"

#include <stddef.h>
#include <string.h>

static void esc_edt_report(const char *event)
{
    const APP_EscCommandState state = APP_EscCommand_GetState();

    APP_Control_QueueText(
        "ESC EDT event=%s state=%s cmd=%u sent=%lu repeats=%u reason=%s proto=%u\r\n",
        event,
        APP_EscCommand_StateName(state),
        (unsigned int)APP_EscCommand_LastCommand(),
        (unsigned long)APP_EscCommand_SentFrames(),
        (unsigned int)APP_ESC_COMMAND_REPEATS,
        APP_EscCommand_AbortReasonName(APP_EscCommand_LastAbortReason()),
        (unsigned int)BSP_ESC_PROTOCOL);
}

uint8_t app_control_handle_esc_edt(char **tokens, uint32_t count)
{
    if ((tokens == NULL) || (count != 2U) || (strcmp(tokens[0], "ESC") != 0)) {
        return 0U;
    }
    if (strcmp(tokens[1], "?") == 0) {
        esc_edt_report("status");
        return 1U;
    }
    /* 不是 EDT 就交还给链上后面的人——`ESC` 开头的命令字不止这一条。 */
    if (strcmp(tokens[1], "EDT") != 0) {
        return 0U;
    }
    esc_edt_report("usage");
    return 1U;
}

uint8_t app_control_handle_esc_edt_set(char **tokens, uint32_t count)
{
    uint16_t command;

    if ((tokens == NULL) || (count != 3U) || (strcmp(tokens[0], "ESC") != 0) ||
        (strcmp(tokens[1], "EDT") != 0)) {
        return 0U;
    }

    if (strcmp(tokens[2], "ON") == 0) {
        command = (uint16_t)APP_ESC_COMMAND_EDT_ENABLE;
    } else if (strcmp(tokens[2], "OFF") == 0) {
        command = (uint16_t)APP_ESC_COMMAND_EDT_DISABLE;
    } else {
        APP_Control_QueueText(
            "ESC EDT event=rejected reason=usage usage=ESC EDT ON|OFF\r\n");
        return 1U;
    }

#if !BSP_ESC_PROTOCOL_IS_DSHOT
    /*
     * PWM 档没有 DShot 这回事。在这里当场拒绝而不是让它走下去失败：走下去的话
     * 回包里是 aborted/output_failed，读起来像是"发了但没发成功"，而真相是
     * 这一档根本没有这条通道。
     */
    (void)command;
    APP_Control_QueueText(
        "ESC EDT event=rejected reason=not_dshot proto=%u\r\n",
        (unsigned int)BSP_ESC_PROTOCOL);
    return 1U;
#else
    /*
     * 解锁状态下拒绝。命令帧会顶掉那一拍的油门，而解锁飞行时每一拍油门都算数。
     * 500 Hz 那一侧的 APP_EscCommand_Step() 也会判同一件事——两道门是有意重复的：
     * 这一道给出可读的拒绝理由，那一道保证即使从别处请求也停得下来。
     */
    if (APP_Stabilizer_IsArmed() != 0U) {
        APP_Control_QueueText("ESC EDT event=rejected reason=armed\r\n");
        return 1U;
    }
    if (APP_EscCommand_Request(command) == 0U) {
        APP_Control_QueueText("ESC EDT event=rejected reason=busy\r\n");
        return 1U;
    }
    esc_edt_report("queued");
    return 1U;
#endif
}
