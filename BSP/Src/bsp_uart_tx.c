#include "bsp_uart_tx.h"

#include "bsp_cache.h"
#include "bsp_critical.h"

#include <stddef.h>
#include <string.h>

/* 退回阻塞发送时的单次超时。只有 hdmatx 缺失或 DMA 启动失败才会走到。 */
#define BSP_UART_TX_FALLBACK_TIMEOUT_MS 100U

/*
 * 临界区用关中断而不是 RTOS 的锁：
 * 本模块的 Write 允许在中断里调用（发送完成中断里续发就是），RTOS 互斥量在
 * 中断上下文不可用；而且 BSP 层不该知道有没有 RTOS。
 * 保护的只有几条索引赋值，关中断时间在 100 ns 量级，1 kHz 控制环看不见。
 */
#define uart_tx_enter() BSP_Critical_Enter()
#define uart_tx_exit(state) BSP_Critical_Exit(state)

static void uart_tx_kick(BSP_UartTx *tx)
{
    const uint8_t *chunk = NULL;
    uint32_t       length;
    uint32_t       primask;

    primask = uart_tx_enter();
    if ((tx->huart == NULL) || (tx->busy != 0U)) {
        uart_tx_exit(primask);
        return;
    }
    length = DRV_TxRing_PeekContiguous(&tx->ring, &chunk);
    if (length == 0U) {
        uart_tx_exit(primask);
        return;
    }
    if (length > 0xFFFFU) {
        length = 0xFFFFU;
    }
    tx->busy     = 1U;
    tx->inflight = length;
    uart_tx_exit(primask);

    /*
     * 发前 clean：缓冲区是 cacheable 的，刚 memcpy 进去的字节可能还只在
     * D-cache 里，DMA 读的是 RAM。漏掉这一步的症状是"偶尔发出一段旧内容"，
     * 且只在 cache 行被换出的时机才出现——最难查的那一类。
     *
     * clean 按 32 字节行对齐向外取整，会连带写回相邻字节；这是安全的：
     * 写回的是 CPU 自己此前写进去的同一份值（invalidate 才有覆盖风险）。
     */
    BSP_Cache_CleanDCache(chunk, length);

    if (tx->huart->hdmatx != NULL) {
        if (HAL_UART_Transmit_DMA(tx->huart, (const uint8_t *)chunk,
                                  (uint16_t)length) == HAL_OK) {
            tx->starts++;
            return;
        }
        tx->errors++;
    }

    /*
     * 没有 DMA 或启动失败：这一段用阻塞发送送出去，然后照常推进队列。
     * 慢，但绝不静默丢内容——命令回包丢了比慢危险得多。
     */
    tx->fallback++;
    (void)HAL_UART_Transmit(tx->huart, (const uint8_t *)chunk, (uint16_t)length,
                            BSP_UART_TX_FALLBACK_TIMEOUT_MS);

    primask = uart_tx_enter();
    DRV_TxRing_Release(&tx->ring, tx->inflight);
    tx->inflight = 0U;
    tx->busy     = 0U;
    uart_tx_exit(primask);
}

void BSP_UartTx_Attach(BSP_UartTx *tx, UART_HandleTypeDef *huart,
                       uint8_t *buffer, uint32_t size)
{
    if (tx == NULL) {
        return;
    }

    memset(tx, 0, sizeof(*tx));
    tx->huart = huart;
    DRV_TxRing_Init(&tx->ring, buffer, size);
}

uint8_t BSP_UartTx_Write(BSP_UartTx *tx, const uint8_t *data, uint16_t length)
{
    uint32_t primask;
    uint32_t used;
    uint8_t  pushed;

    if ((tx == NULL) || (tx->huart == NULL) || (data == NULL)) {
        return 0U;
    }
    if (length == 0U) {
        return 1U;
    }

    primask = uart_tx_enter();
    pushed = DRV_TxRing_Push(&tx->ring, data, (uint32_t)length);
    used   = DRV_TxRing_Used(&tx->ring);
    if (pushed != 0U) {
        tx->bytes += (uint32_t)length;
        tx->writes++;
        if (used > tx->peak_used) {
            tx->peak_used = used;
        }
    } else {
        tx->drops++;
        tx->dropped_bytes += (uint32_t)length;
    }
    uart_tx_exit(primask);

    if (pushed == 0U) {
        return 0U;
    }

    uart_tx_kick(tx);
    return 1U;
}

uint32_t BSP_UartTx_Pending(const BSP_UartTx *tx)
{
    if (tx == NULL) {
        return 0U;
    }

    return DRV_TxRing_Used(&tx->ring);
}

uint8_t BSP_UartTx_IsIdle(const BSP_UartTx *tx)
{
    if (tx == NULL) {
        return 1U;
    }

    return ((tx->busy == 0U) && (DRV_TxRing_Used(&tx->ring) == 0U)) ? 1U : 0U;
}

void BSP_UartTx_OnComplete(BSP_UartTx *tx, UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if ((tx == NULL) || (tx->huart == NULL) || (huart != tx->huart)) {
        return;
    }

    primask = uart_tx_enter();
    DRV_TxRing_Release(&tx->ring, tx->inflight);
    tx->inflight = 0U;
    tx->busy     = 0U;
    uart_tx_exit(primask);

    /*
     * 在完成中断里立刻续发下一段。等回任务里再发的话，队列里攒着的字节要多等
     * 一个调度周期才出门，蓝牙那条链路上这就是几毫秒的白等。
     * HAL 在调用本回调之前已经把 gState 置回 READY，所以这里重新启动是安全的。
     */
    uart_tx_kick(tx);
}

void BSP_UartTx_OnError(BSP_UartTx *tx, UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if ((tx == NULL) || (tx->huart == NULL) || (huart != tx->huart)) {
        return;
    }

    primask = uart_tx_enter();
    if (tx->busy != 0U) {
        /*
         * 出错的那一段就当发过了：留在队列里等重发，只会让后面所有内容排在
         * 一段永远发不出去的字节后面，整条出口从此静默——这正是要避免的失败形态。
         */
        DRV_TxRing_Release(&tx->ring, tx->inflight);
        tx->inflight = 0U;
        tx->busy     = 0U;
        tx->errors++;
    }
    uart_tx_exit(primask);

    uart_tx_kick(tx);
}

void BSP_UartTx_GetStats(const BSP_UartTx *tx, BSP_UartTxStats *stats)
{
    if (stats == NULL) {
        return;
    }

    memset(stats, 0, sizeof(*stats));
    if (tx == NULL) {
        return;
    }

    stats->bytes         = tx->bytes;
    stats->writes        = tx->writes;
    stats->drops         = tx->drops;
    stats->dropped_bytes = tx->dropped_bytes;
    stats->starts        = tx->starts;
    stats->errors        = tx->errors;
    stats->fallback      = tx->fallback;
    stats->peak_used     = tx->peak_used;
    stats->pending       = DRV_TxRing_Used(&tx->ring);
    stats->size          = tx->ring.size;
    stats->busy          = tx->busy;
    stats->dma = ((tx->huart != NULL) && (tx->huart->hdmatx != NULL)) ? 1U : 0U;
}

void BSP_UartTx_ResetStats(BSP_UartTx *tx)
{
    uint32_t primask;

    if (tx == NULL) {
        return;
    }

    primask = uart_tx_enter();
    tx->bytes         = 0U;
    tx->writes        = 0U;
    tx->drops         = 0U;
    tx->dropped_bytes = 0U;
    tx->starts        = 0U;
    tx->errors        = 0U;
    tx->fallback      = 0U;
    tx->peak_used     = DRV_TxRing_Used(&tx->ring);
    uart_tx_exit(primask);
}
