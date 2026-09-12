#include "bsp_uart.h"

#include "bsp_led.h"
#include "usart.h"

#include <stddef.h>

#define BSP_UART_TX_LED_ENABLED 0U

/* ---------------------------------------------------------- 维护口的板级绑定 */

/*
 * 换板子时**只改这三行**：句柄、名字、发送队列容量。
 * 其余所有代码（BSP 的发送引擎、App 的维护口逻辑）都不认识 UART8 这个名字。
 */
#define BSP_UART_MAINT_HANDLE     (&huart8)
#define BSP_UART_MAINT_NAME       "uart8"
#define BSP_UART_MAINT_TX_SIZE    2048U

/*
 * 发送队列放 `.dma_buffer`（RAM_D2 @0x30000000，32 字节对齐）。
 * 不能放默认的 .bss —— 那是 DTCM（0x20000000），DMA1/DMA2 根本够不到，
 * 放那儿的症状是"HAL 返回 OK 但一个字节都没出去"。
 *
 * 2048 字节 ≈ 115200 下 178 ms 的缓冲。够吸收命令回包的突发（单条最长 256 B），
 * 又不至于大到让遥测帧在队列里排出几百毫秒的陈旧延迟——后者由上层的 backlog
 * 判据挡住，见 app_maint_uart.c。
 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t bsp_uart_maint_tx_buffer[BSP_UART_MAINT_TX_SIZE];

static BSP_UartTx bsp_uart_maint_tx;

static volatile uint32_t bsp_uart_usart1_tx_count;

void BSP_UART_Release_USART1_ForExternalDebug(void)
{
#if (BSP_UART_USART1_OUTPUT_ENABLED == 0U)
    (void)HAL_UART_DeInit(&huart1);
#endif
}

HAL_StatusTypeDef BSP_UART_Transmit_USART1(const uint8_t *data,
                                           uint16_t length,
                                           uint32_t timeout_ms)
{
#if (BSP_UART_USART1_OUTPUT_ENABLED == 0U)
    (void)data;
    (void)length;
    (void)timeout_ms;
    return HAL_OK;
#else
    if ((data == 0) || (length == 0U)) {
        return HAL_OK;
    }

    ++bsp_uart_usart1_tx_count;
#if (BSP_UART_TX_LED_ENABLED != 0U)
    BSP_LED_On(LED_1);
#endif
    return HAL_UART_Transmit(&huart1, (uint8_t *)data, length, timeout_ms);
#endif
}

uint32_t BSP_UART_GetUSART1TxCount(void)
{
    return bsp_uart_usart1_tx_count;
}

/* ============================================================ 维护 / 调参链路 */

void BSP_UART_MaintInit(void)
{
    BSP_UartTx_Attach(&bsp_uart_maint_tx, BSP_UART_MAINT_HANDLE,
                      bsp_uart_maint_tx_buffer, BSP_UART_MAINT_TX_SIZE);
}

UART_HandleTypeDef *BSP_UART_MaintHandle(void)
{
    return BSP_UART_MAINT_HANDLE;
}

const char *BSP_UART_MaintName(void)
{
    return BSP_UART_MAINT_NAME;
}

uint8_t BSP_UART_IsMaint(const UART_HandleTypeDef *huart)
{
    return ((huart != NULL) &&
            (huart->Instance == BSP_UART_MAINT_HANDLE->Instance)) ? 1U : 0U;
}

uint8_t BSP_UART_MaintRxStart(uint8_t *byte)
{
    if (byte == NULL) {
        return 0U;
    }

    return (HAL_UART_Receive_IT(BSP_UART_MAINT_HANDLE, byte, 1U) == HAL_OK)
               ? 1U : 0U;
}

void BSP_UART_MaintRxRecover(uint8_t *byte)
{
    UART_HandleTypeDef *huart = BSP_UART_MAINT_HANDLE;

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                 UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)HAL_UART_AbortReceive(huart);
    (void)BSP_UART_MaintRxStart(byte);
}

uint8_t BSP_UART_MaintWrite(const uint8_t *data, uint16_t length)
{
    return BSP_UartTx_Write(&bsp_uart_maint_tx, data, length);
}

uint32_t BSP_UART_MaintTxPending(void)
{
    return BSP_UartTx_Pending(&bsp_uart_maint_tx);
}

void BSP_UART_MaintTxGetStats(BSP_UartTxStats *stats)
{
    BSP_UartTx_GetStats(&bsp_uart_maint_tx, stats);
}

void BSP_UART_MaintTxResetStats(void)
{
    BSP_UartTx_ResetStats(&bsp_uart_maint_tx);
}

void BSP_UART_MaintOnTxComplete(UART_HandleTypeDef *huart)
{
    BSP_UartTx_OnComplete(&bsp_uart_maint_tx, huart);
}

void BSP_UART_MaintOnTxError(UART_HandleTypeDef *huart)
{
    BSP_UartTx_OnError(&bsp_uart_maint_tx, huart);
}

/* ============================================================ 诊断：任意 UART 事务 */

UART_HandleTypeDef *BSP_UART_GetHandle(uint8_t bus_index)
{
    switch (bus_index) {
    case 1U: return &huart1;
    case 2U: return &huart2;
    case 3U: return &huart3;
    case 6U: return &huart6;
    case 7U: return &huart7;
    case 8U: return &huart8;
    default: return NULL;
    }
}

uint8_t BSP_UART_IsHalfDuplex(uint8_t bus_index)
{
    /* UART7 走总线舵机的单线协议，收发共用一根线。其余都是全双工。 */
    return (bus_index == 7U) ? 1U : 0U;
}

/*
 * 阻塞式收发。接收用逐字节 HAL_UART_Receive 而不是一次收满，是为了让
 * "对面只回了半句"也能把已经收到的部分交出来——诊断里这比"要么收满要么报超时"
 * 有用得多：帧长不对、波特率不对，往往正是从残缺的回包上看出来的。
 */
HAL_StatusTypeDef BSP_UART_DebugXfer(uint8_t bus_index,
                                     const uint8_t *tx, uint16_t tx_len,
                                     uint8_t *rx, uint16_t rx_len,
                                     uint16_t *rx_got,
                                     uint32_t timeout_ms)
{
    UART_HandleTypeDef *huart = BSP_UART_GetHandle(bus_index);
    HAL_StatusTypeDef hal = HAL_OK;
    uint16_t got = 0U;

    if (rx_got != NULL) { *rx_got = 0U; }

    if ((huart == NULL) ||
        (tx_len > (uint16_t)BSP_UART_XFER_MAX) ||
        (rx_len > (uint16_t)BSP_UART_XFER_MAX) ||
        ((tx_len != 0U) && (tx == NULL)) ||
        ((rx_len != 0U) && (rx == NULL))) {
        return HAL_ERROR;
    }

    if (tx_len != 0U) {
        if (BSP_UART_IsHalfDuplex(bus_index) != 0U) {
            (void)HAL_HalfDuplex_EnableTransmitter(huart);
        }
        hal = HAL_UART_Transmit(huart, (uint8_t *)(uintptr_t)tx, tx_len,
                                timeout_ms);
        if (BSP_UART_IsHalfDuplex(bus_index) != 0U) {
            (void)HAL_HalfDuplex_EnableReceiver(huart);
        }
        if (hal != HAL_OK) { return hal; }
    }

    while (got < rx_len) {
        if (HAL_UART_Receive(huart, &rx[got], 1U, timeout_ms) != HAL_OK) {
            break;   /* 收不满不算错，把已收到的交出去 */
        }
        got++;
    }

    if (rx_got != NULL) { *rx_got = got; }
    return HAL_OK;
}
