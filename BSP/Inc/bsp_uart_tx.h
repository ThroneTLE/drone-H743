#ifndef BSP_UART_TX_H
#define BSP_UART_TX_H

#include "main.h"

#include "drv_tx_ring.h"

#include <stdint.h>

/*
 * 任意 UART 的 DMA 发送引擎：排队 + 零拷贝取段 + D-cache 维护 + 完成中断续发。
 *
 * === 为什么是"任意 UART"而不是 UART8 专用 ===
 *
 * 这个模块是为了让板载蓝牙（本板在 UART8）别再用阻塞发送而写的，但按板子写死
 * 正是这次移植踩过的坑：换一块板子，蓝牙可能挂在 USART2，PWM 可能从 TIM5 搬到
 * TIM1 —— 凡是把实例名焊进逻辑里的代码，换板时都得重写一遍。
 * 所以这里只认调用方给进来的 `UART_HandleTypeDef *` 与一块缓冲区，
 * "哪个 UART 是维护口"这件事由 bsp_uart.c 一处决定。
 *
 * === 契约 ===
 *
 * - `buffer` 必须落在**该 UART 的 DMA 主设备够得到**的 RAM 里。
 *   本工程用 linker 段 `.dma_buffer`（RAM_D2 @0x30000000，32 字节对齐）。
 *   DTCM（0x20000000）对 DMA1/DMA2 不可达，放那儿会静默发不出去。
 * - `buffer` 是 cacheable 的，发送前本模块自己 clean，调用方不必管。
 * - `Write` 非阻塞、全有或全无：排不下就整包丢弃并计数，绝不写半包。
 * - `Write` 可在任务或中断上下文调用；内部用 PRIMASK 临界区保护索引。
 * - 该 UART 的 `HAL_UART_TxCpltCallback` / `HAL_UART_ErrorCallback` 必须转接到
 *   `BSP_UartTx_OnComplete` / `BSP_UartTx_OnError`，否则发完第一段就停住了。
 * - 没有 hdmatx 时自动退回阻塞发送，并在 `fallback` 里如实计数——
 *   CubeMX 重新生成时掉了 DMA 请求，要能一眼看出来，而不是"怎么变慢了"。
 */

typedef struct {
    uint32_t bytes;          /* 成功排进队列的字节数 */
    uint32_t writes;         /* 成功的写次数 */
    uint32_t drops;          /* 队列满、整包丢弃的次数 */
    uint32_t dropped_bytes;
    uint32_t starts;         /* 启动过多少段 DMA 传输 */
    uint32_t errors;         /* 启动失败 / UART 报错 */
    uint32_t fallback;       /* 退回阻塞发送的次数 */
    uint32_t peak_used;      /* 队列水位峰值，用来判断容量够不够 */
    uint32_t pending;        /* 当前排队字节数 */
    uint32_t size;           /* 队列容量 */
    uint8_t  busy;           /* 是否有一段 DMA 正在传 */
    uint8_t  dma;            /* 1 = 走 DMA，0 = 退化成阻塞发送 */
} BSP_UartTxStats;

typedef struct {
    UART_HandleTypeDef *huart;
    DRV_TxRing          ring;
    volatile uint8_t    busy;
    uint32_t            inflight;
    /* 统计量，与 BSP_UartTxStats 同义，单独存是为了不在热路径上组装结构体。 */
    uint32_t bytes;
    uint32_t writes;
    uint32_t drops;
    uint32_t dropped_bytes;
    uint32_t starts;
    uint32_t errors;
    uint32_t fallback;
    uint32_t peak_used;
} BSP_UartTx;

/* 绑定 UART 与缓冲区。重复调用会丢弃队列里尚未发出的内容。 */
void BSP_UartTx_Attach(BSP_UartTx *tx, UART_HandleTypeDef *huart,
                       uint8_t *buffer, uint32_t size);

/* 排队一段字节。返回 0 = 队列放不下，整包已丢弃并计数。 */
uint8_t BSP_UartTx_Write(BSP_UartTx *tx, const uint8_t *data, uint16_t length);

/* 当前排队（含正在传的那一段）字节数。App 层据此决定要不要丢新帧。 */
uint32_t BSP_UartTx_Pending(const BSP_UartTx *tx);

/* 队列已空且没有在途传输。 */
uint8_t BSP_UartTx_IsIdle(const BSP_UartTx *tx);

/* HAL 回调转接。huart 不是本实例绑定的那个时直接返回，可以无脑串接。 */
void BSP_UartTx_OnComplete(BSP_UartTx *tx, UART_HandleTypeDef *huart);
void BSP_UartTx_OnError(BSP_UartTx *tx, UART_HandleTypeDef *huart);

void BSP_UartTx_GetStats(const BSP_UartTx *tx, BSP_UartTxStats *stats);
void BSP_UartTx_ResetStats(BSP_UartTx *tx);

#endif /* BSP_UART_TX_H */
