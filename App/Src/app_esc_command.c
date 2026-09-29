#include "app_esc_command.h"

#include <stddef.h>

/*
 * 和 app_prop_spin.c 同样的理由用文件级静态量：这个窗口全机只有一个，而请求它的
 * 文本任务和执行它的 500 Hz 控制环必须看到同一个它。两份状态意味着"上位机以为发完了、
 * 控制环还在占着输出"。跨任务读写的量标 volatile。
 */
static volatile uint8_t  command_state = (uint8_t)APP_ESC_COMMAND_IDLE;
static volatile uint16_t command_code;
static volatile uint16_t command_frames[APP_ESC_COMMAND_MAX_FRAMES];
static volatile uint8_t  command_planned;
static volatile uint8_t  command_remaining;
static volatile uint8_t  command_abort_reason;
static volatile uint32_t command_sent_frames;

void APP_EscCommand_Reset(void)
{
    command_state = (uint8_t)APP_ESC_COMMAND_IDLE;
    command_code = 0U;
    command_planned = 0U;
    command_remaining = 0U;
    command_abort_reason = (uint8_t)APP_ESC_COMMAND_ABORT_NONE;
    command_sent_frames = 0U;
}

uint8_t APP_EscCommand_Request(uint16_t command)
{
    if ((command != (uint16_t)APP_ESC_COMMAND_EDT_ENABLE) &&
        (command != (uint16_t)APP_ESC_COMMAND_EDT_DISABLE)) {
        return 0U;
    }
    if (command_state == (uint8_t)APP_ESC_COMMAND_SENDING) {
        /*
         * 不打断、也不排队。一条发到一半被顶掉，电调那边两条都没收满连续帧，
         * 结果是两条都不生效而界面显示"已发送"——比直接拒绝难查得多。
         */
        return 0U;
    }
    command_code = command;
    for (uint8_t i = 0U; i < (uint8_t)APP_ESC_COMMAND_REPEATS; ++i) {
        command_frames[i] = command;
    }
    command_planned = (uint8_t)APP_ESC_COMMAND_REPEATS;
    command_sent_frames = 0U;
    command_abort_reason = (uint8_t)APP_ESC_COMMAND_ABORT_NONE;
    command_remaining = (uint8_t)APP_ESC_COMMAND_REPEATS;
    /* state 最后置：在它置起来之前，控制环读到的 remaining 已经就位。 */
    command_state = (uint8_t)APP_ESC_COMMAND_SENDING;
    return 1U;
}

uint8_t APP_EscCommand_MotorKvByte(uint16_t kv, uint8_t *out_byte)
{
    if ((out_byte == NULL) || (kv < (uint16_t)APP_ESC_KV_MIN) ||
        (kv > (uint16_t)APP_ESC_KV_MAX) || (((kv - 20U) % 40U) != 0U)) {
        return 0U;
    }
    *out_byte = (uint8_t)((kv - 20U) / 40U);
    return 1U;
}

uint8_t APP_EscCommand_RequestMotorKv(uint16_t kv)
{
    uint8_t kv_byte = 0U;
    uint8_t n = 0U;

    if ((APP_EscCommand_MotorKvByte(kv, &kv_byte) == 0U) ||
        (command_state == (uint8_t)APP_ESC_COMMAND_SENDING)) {
        return 0U;
    }
    for (uint8_t i = 0U; i < (uint8_t)APP_ESC_PROGRAM_ENTER_REPEATS; ++i) {
        command_frames[n++] = (uint16_t)APP_ESC_PROGRAM_ENTER;
    }
    command_frames[n++] = (uint16_t)APP_ESC_EEPROM_MOTOR_KV;
    command_frames[n++] = (uint16_t)kv_byte;
    command_frames[n++] = (uint16_t)APP_ESC_PROGRAM_COMMIT;
    for (uint8_t i = 0U; i < (uint8_t)APP_ESC_SAVE_REPEATS; ++i) {
        command_frames[n++] = (uint16_t)APP_ESC_SAVE_SETTINGS;
    }
    command_code = (uint16_t)APP_ESC_PROGRAM_ENTER;
    command_planned = n;
    command_sent_frames = 0U;
    command_abort_reason = (uint8_t)APP_ESC_COMMAND_ABORT_NONE;
    command_remaining = n;
    /* state 最后置：同 Request()。 */
    command_state = (uint8_t)APP_ESC_COMMAND_SENDING;
    return 1U;
}

uint8_t APP_EscCommand_PlannedFrames(void)
{
    return command_planned;
}

void APP_EscCommand_Step(uint8_t inhibit)
{
    if (command_state != (uint8_t)APP_ESC_COMMAND_SENDING) {
        return;
    }
    if (inhibit != 0U) {
        command_remaining = 0U;
        command_state = (uint8_t)APP_ESC_COMMAND_ABORTED;
        command_abort_reason = (uint8_t)APP_ESC_COMMAND_ABORT_INHIBIT;
    }
}

uint8_t APP_EscCommand_Pending(uint16_t *out_command)
{
    if (out_command == NULL) {
        return 0U;
    }
    if ((command_state != (uint8_t)APP_ESC_COMMAND_SENDING) ||
        (command_remaining == 0U)) {
        return 0U;
    }
    *out_command = command_frames[command_planned - command_remaining];
    return 1U;
}

void APP_EscCommand_Consume(void)
{
    if (command_state != (uint8_t)APP_ESC_COMMAND_SENDING) {
        return;
    }
    command_sent_frames++;
    if (command_remaining > 0U) {
        command_remaining--;
    }
    if (command_remaining == 0U) {
        command_state = (uint8_t)APP_ESC_COMMAND_DONE;
    }
}

void APP_EscCommand_Fail(void)
{
    if (command_state != (uint8_t)APP_ESC_COMMAND_SENDING) {
        return;
    }
    /*
     * 整条作废而不是补发。电调认的是**连续**若干帧；中间漏一帧，后面补上去的
     * 那几帧对它来说是一条新的、长度不够的序列。假装补上只会让上层以为发成功了。
     */
    command_remaining = 0U;
    command_state = (uint8_t)APP_ESC_COMMAND_ABORTED;
    command_abort_reason = (uint8_t)APP_ESC_COMMAND_ABORT_OUTPUT;
}

uint8_t APP_EscCommand_IsActive(void)
{
    return (command_state == (uint8_t)APP_ESC_COMMAND_SENDING) ? 1U : 0U;
}

APP_EscCommandState APP_EscCommand_GetState(void)
{
    return (APP_EscCommandState)command_state;
}

uint8_t APP_EscCommand_LastAbortReason(void)
{
    return command_abort_reason;
}

uint16_t APP_EscCommand_LastCommand(void)
{
    return command_code;
}

uint32_t APP_EscCommand_SentFrames(void)
{
    return command_sent_frames;
}

const char *APP_EscCommand_StateName(APP_EscCommandState state)
{
    switch (state) {
    case APP_ESC_COMMAND_IDLE:    return "idle";
    case APP_ESC_COMMAND_SENDING: return "sending";
    case APP_ESC_COMMAND_DONE:    return "done";
    case APP_ESC_COMMAND_ABORTED: return "aborted";
    default:                      return "unknown";
    }
}

const char *APP_EscCommand_AbortReasonName(uint8_t reason)
{
    switch ((APP_EscCommandAbortReason)reason) {
    case APP_ESC_COMMAND_ABORT_NONE:    return "none";
    case APP_ESC_COMMAND_ABORT_INHIBIT: return "inhibited";
    case APP_ESC_COMMAND_ABORT_OUTPUT:  return "output_failed";
    default:                            return "unknown";
    }
}
