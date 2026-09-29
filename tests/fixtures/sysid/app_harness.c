/*
 * 宿主装置：把 App/Src/app_sysid.c 的平台边界填上，让辨识运行层能在 gcc 上
 * 真跑起来。
 *
 * 只替换平台边界——文本回复、二进制发送、老 ident 的忙标志、电调回传转速快照、
 * 电调命令窗口与地面点动的占用标志——**辨识本身的状态机、安全门、反解链、
 * 环形缓冲、桨位标定一行都不替**。用 Python 重写一份模型再去"验证"它，
 * 验的是模型不是固件；这套判据要能在真代码上失败才有意义。
 */

#include "app_sysid.h"
#include "app_esc_command.h"
#include "app_proto.h"
#include "app_servo_jog.h"
#include "bsp_dshot_rx.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HARNESS_TEXT_LINES 64
#define HARNESS_TEXT_BYTES 256
#define HARNESS_FRAMES     128
#define HARNESS_FRAME_BYTES 512

static char harness_text[HARNESS_TEXT_LINES][HARNESS_TEXT_BYTES];
static uint32_t harness_text_count;

static uint8_t harness_frames[HARNESS_FRAMES][HARNESS_FRAME_BYTES];
static uint16_t harness_frame_len[HARNESS_FRAMES];
static uint16_t harness_frame_fn[HARNESS_FRAMES];
static uint32_t harness_frame_count;

static uint8_t harness_ident_busy;
static uint8_t harness_send_failure;

void APP_Control_QueueText(const char *format, ...)
{
    va_list args;

    if (harness_text_count >= HARNESS_TEXT_LINES) {
        return;
    }
    va_start(args, format);
    (void)vsnprintf(harness_text[harness_text_count], HARNESS_TEXT_BYTES,
                    format, args);
    va_end(args);
    harness_text_count++;
}

/*
 * 目标板上这条路的真实上限，照抄过来一并模拟：
 *   - `APP_Proto_BuildFrame` 拒收 payload > `APP_PROTO_MAX_PAYLOAD`；
 *   - `APP_Diag_SendBinary` 把**成帧后**的字节写进
 *     `APP_UART_TxMessage.text[APP_UART_TX_TEXT_SIZE]`，$X 外层还占 9 字节。
 *
 * 装置不模拟这两道的话，把一批攒到发不出去的大小在宿主上是全绿的，
 * 到实机才表现成"stream 开着、一个字节都不来"。
 */
#define HARNESS_FRAMED_OVERHEAD 9U
#define HARNESS_TX_TEXT_SIZE    256U   /* APP_UART_TX_TEXT_SIZE */

uint8_t APP_Diag_SendBinary(uint16_t function, const uint8_t *payload,
                            uint16_t length)
{
    if (harness_send_failure) { return 0U; }
    if (length > APP_PROTO_MAX_PAYLOAD) {
        return 0U;   /* BuildFrame 会拒 */
    }
    if ((uint32_t)length + HARNESS_FRAMED_OVERHEAD > HARNESS_TX_TEXT_SIZE) {
        return 0U;   /* 成帧之后放不进发送缓冲 */
    }
    if ((harness_frame_count >= HARNESS_FRAMES) ||
        (length > HARNESS_FRAME_BYTES)) {
        return 0U;
    }
    harness_frame_fn[harness_frame_count] = function;
    harness_frame_len[harness_frame_count] = length;
    if ((payload != NULL) && (length > 0U)) {
        memcpy(harness_frames[harness_frame_count], payload, length);
    }
    harness_frame_count++;
    return 1U;
}

uint8_t APP_Ident_IsRunning(void) { return harness_ident_busy; }

/*
 * 电调回传快照：目标板上由 BSP（bsp_dshot_rx.c / bsp_dshot_bitbang.c）按 ESC 通道
 * 下标（0 = 通道 1）填。这里整份由测试给，默认 available=0（非双向档）。
 */
static BSP_DShotRxSnapshot harness_rx;
static uint8_t harness_esc_command_active;
static uint8_t harness_servo_jog_active;

void BSP_DShotRx_GetSnapshot(BSP_DShotRxSnapshot *out)
{
    if (out != NULL) {
        *out = harness_rx;
    }
}
uint8_t APP_EscCommand_IsActive(void) { return harness_esc_command_active; }
uint8_t APP_ServoJog_IsActive(void) { return harness_servo_jog_active; }

void harness_set_rx_available(uint8_t available) { harness_rx.available = available; }
void harness_set_rotor(uint32_t index, uint32_t erpm, uint32_t sample_ms,
                       uint8_t valid, uint8_t not_spinning)
{
    if (index < 2U) {
        harness_rx.erpm[index] = erpm;
        harness_rx.sample_ms[index] = sample_ms;
        harness_rx.valid[index] = valid;
        harness_rx.not_spinning[index] = not_spinning;
    }
}
void harness_set_esc_command_active(uint8_t active) { harness_esc_command_active = active; }
void harness_set_servo_jog_active(uint8_t active) { harness_servo_jog_active = active; }
/*
 * 抢占模拟：第 N 次进临界区之前，先以命令任务的身份执行一次 SYSID STOP。
 * 实机上 UARTTask 与稳定环同优先级、时间片轮转，STOP 可以插进一次 Update 的中间；
 * 宿主上单线程跑，只能在 Update 内部唯一会调到的桩上把它"插"进去。
 */
static uint32_t harness_stop_countdown;
static uint8_t harness_in_stop;
uint32_t BSP_Critical_Enter(void)
{
    if ((harness_stop_countdown != 0U) && (harness_in_stop == 0U)) {
        harness_stop_countdown--;
        if (harness_stop_countdown == 0U) {
            harness_in_stop = 1U;
            APP_SysId_Stop("command");
            harness_in_stop = 0U;
        }
    }
    return 0U;
}
void harness_set_stop_on_critical(uint32_t nth) { harness_stop_countdown = nth; }
/*
 * 同上，但插在第 N 次**出**临界区之后：实机上临界区里挂起的 SysTick 正是在出口处
 * 被响应、切到命令任务的。"末条入环之后、收尾之前"这段窗口只有从这里够得着。
 */
static uint32_t harness_stop_exit_countdown;
void BSP_Critical_Exit(uint32_t state)
{
    (void)state;
    if ((harness_stop_exit_countdown != 0U) && (harness_in_stop == 0U)) {
        harness_stop_exit_countdown--;
        if (harness_stop_exit_countdown == 0U) {
            harness_in_stop = 1U;
            APP_SysId_Stop("command");
            harness_in_stop = 0U;
        }
    }
}
void harness_set_stop_after_critical(uint32_t nth) { harness_stop_exit_countdown = nth; }
void BSP_Critical_MemoryBarrier(void) {}
uint8_t APP_SysId_PortBegin(uint32_t hz) { (void)hz; return 1U; }
uint8_t APP_SysId_PortSend(const uint8_t *payload, uint16_t length)
{ return APP_Diag_SendBinary(APP_PROTO_MSG_SYSID_BATCH, payload, length); }


/* ------------------------------------------------------------------ 探针 */

void harness_reset(void)
{
    harness_stop_countdown = 0U;
    harness_stop_exit_countdown = 0U;
    harness_send_failure = 0U;
    harness_text_count = 0U;
    harness_frame_count = 0U;
    harness_ident_busy = 0U;
    harness_esc_command_active = 0U;
    harness_servo_jog_active = 0U;
    memset(&harness_rx, 0, sizeof(harness_rx));
}

void harness_set_ident_busy(uint8_t busy) { harness_ident_busy = busy; }
void harness_set_send_failure(uint8_t failed) { harness_send_failure = failed; }

uint32_t harness_text_lines(void) { return harness_text_count; }

const char *harness_text_line(uint32_t index)
{
    return (index < harness_text_count) ? harness_text[index] : "";
}

uint32_t harness_frame_total(void) { return harness_frame_count; }
uint16_t harness_frame_function(uint32_t i) { return (i < harness_frame_count) ? harness_frame_fn[i] : 0U; }
uint16_t harness_frame_length(uint32_t i) { return (i < harness_frame_count) ? harness_frame_len[i] : 0U; }
const uint8_t *harness_frame_data(uint32_t i)
{
    return (i < harness_frame_count) ? harness_frames[i] : NULL;
}
