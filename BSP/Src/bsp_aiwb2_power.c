/*
 * Ai-WB2 WiFi 模块的电源与按键控制。
 *
 * ==================== MicoAir743v2 上这个模块并不存在 ====================
 * 老板子外挂了一颗 Ai-WB2：PC6 是它的使能脚、PB5 是随它一起的按键。
 * MicoAir743v2 上板子没有这个模块，而且那两个脚已经另有其主——
 * **PC6 是 USART6_TX，也就是 ELRS 接收机那条线**（PC7 是 RX）。
 *
 * 所以这里不能再碰 GPIO：
 *   - 写 PC6 在 AF 模式下会被引脚忽略，看着"没事"，其实是在对 RC 串口的发送脚
 *     做无意义的操作，哪天引脚模式变了就会真的打断遥控链路；
 *   - 更糟的是**读** PC6 —— 以前 BSP_AiWB2_IsEnabled() 直接读该脚电平当作使能状态，
 *     现在读到的是 UART 正在发送的数据位，随机高低。app_aiwb2.c 里到处是
 *     `if (IsEnabled() == 0) SetEnabled(1)`，拿随机值去喂它会让它反复抖动。
 *   - PB5 在本板上是 UART5_RX，没有按键，浮空读出来也是随机值，
 *     而 BSP_AiWB2_UpdateButton() 是在控制环里逐周期调用的（app_stabilizer.c）。
 *
 * 因此改成**纯状态记账**：接口原样保留（app_control.c 等调用点一个字不改），
 * 记住上层写了什么就回报什么，不去碰任何引脚。诊断页读到的是"固件认为的状态"，
 * 而不是一个编出来的电平——板上没有这颗模块，这就是能给出的最诚实的答案。
 */
#include "bsp_aiwb2_power.h"

static uint8_t aiwb2_last_written_state = 1U;
static uint32_t aiwb2_write_count;

void BSP_AiWB2_PowerInit(void)
{
    BSP_AiWB2_SetEnabled(1U);
}

void BSP_AiWB2_SetEnabled(uint8_t enabled)
{
    aiwb2_last_written_state = (enabled != 0U) ? 1U : 0U;
    ++aiwb2_write_count;
}

void BSP_AiWB2_UpdateButton(void)
{
    /* 本板没有这颗模块的按键（PB5 已归 UART5），无事可做。 */
}

uint8_t BSP_AiWB2_IsEnabled(void)
{
    return aiwb2_last_written_state;
}

uint8_t BSP_AiWB2_GetLastWrittenState(void)
{
    return aiwb2_last_written_state;
}

uint32_t BSP_AiWB2_GetWriteCount(void)
{
    return aiwb2_write_count;
}
