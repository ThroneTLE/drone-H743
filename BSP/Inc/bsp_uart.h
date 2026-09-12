#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"

#include "bsp_uart_tx.h"

#include <stdint.h>

#define BSP_UART_USART1_OUTPUT_ENABLED 1U

void BSP_UART_Release_USART1_ForExternalDebug(void);

/* ============================================================ 维护 / 调参链路 */

/*
 * "维护口"是一个**角色**，不是一个实例号。
 *
 * 本板（MicoAir743V2）上这个角色由板载蓝牙模块承担，接在 UART8（PE1/PE0，
 * 115200）。换板子时它可能变成 USART2、或者一个 USB 转串口——那时只需要改
 * 本文件顶部的绑定，App 层的 app_maint_uart.c 一行都不用动。
 * 这次移植最费时间的改动，几乎全都是"实例名被写进了上层逻辑"造成的。
 *
 * 上电顺序：MX_*_Init 建好句柄之后、APP_Init 之前调用 BSP_UART_MaintInit。
 */
void BSP_UART_MaintInit(void);

/* 维护口的 HAL 句柄。只给 BSP 内部用（bsp_uart_link.c 按角色取句柄）。 */
UART_HandleTypeDef *BSP_UART_MaintHandle(void);

/* 该链路的人类可读名字，用于诊断回包（例如 "uart8"）。 */
const char *BSP_UART_MaintName(void);

/* 这个 HAL 句柄是不是维护口。给 HAL 回调分发用。 */
uint8_t BSP_UART_IsMaint(const UART_HandleTypeDef *huart);

/* 收一个字节（中断方式）。返回 0 = 启动失败。 */
uint8_t BSP_UART_MaintRxStart(uint8_t *byte);

/* 清错误标志、中止当前接收并重新开收。溢出/噪声之后用。 */
void BSP_UART_MaintRxRecover(uint8_t *byte);

/*
 * 排队发送。非阻塞、全有或全无，走 DMA；细节见 bsp_uart_tx.h。
 * 返回 0 = 队列放不下，整包已丢弃并计入统计。
 */
uint8_t BSP_UART_MaintWrite(const uint8_t *data, uint16_t length);

/* 当前排队字节数。上层据此决定要不要丢掉一帧新的遥测。 */
uint32_t BSP_UART_MaintTxPending(void);

void BSP_UART_MaintTxGetStats(BSP_UartTxStats *stats);
void BSP_UART_MaintTxResetStats(void);

/* HAL 回调转接：app_uart.c 的回调枢纽把 UART 事件送进来。 */
void BSP_UART_MaintOnTxComplete(UART_HandleTypeDef *huart);
void BSP_UART_MaintOnTxError(UART_HandleTypeDef *huart);

HAL_StatusTypeDef BSP_UART_Transmit_USART1(const uint8_t *data,
                                           uint16_t length,
                                           uint32_t timeout_ms);
uint32_t BSP_UART_GetUSART1TxCount(void);

/* ---------------- 诊断：任意 UART 事务 ---------------- */

/*
 * 让上位机直接往某个串口发几个字节、再把回来的收上来，不必为每次试验重新编译。
 *
 * 总线号用板上的实例编号（1/2/3/6/7/8），与丝印和 hwdef 一致。
 * **UART7 是半双工总线舵机口**——往上面发字节会让舵机动，所以是否允许调用
 * 由 App 层按解锁状态把关，这里只负责老实收发。
 */

#define BSP_UART_XFER_MAX 64U

/* 总线号 → HAL 句柄。未知或未启用的总线返回 NULL。 */
UART_HandleTypeDef *BSP_UART_GetHandle(uint8_t bus_index);

/* 该总线是否是半双工（发完要切回接收），供诊断如实回报。 */
uint8_t BSP_UART_IsHalfDuplex(uint8_t bus_index);

/*
 * 先发 tx_len 字节，再在 timeout_ms 内尽量收满 rx_len 字节。
 * *rx_got 回报实际收到多少（收不满不算错——对面可能本来就没那么多话说）。
 */
HAL_StatusTypeDef BSP_UART_DebugXfer(uint8_t bus_index,
                                     const uint8_t *tx, uint16_t tx_len,
                                     uint8_t *rx, uint16_t rx_len,
                                     uint16_t *rx_got,
                                     uint32_t timeout_ms);

#endif
