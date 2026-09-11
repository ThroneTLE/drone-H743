#ifndef DRV_TX_RING_H
#define DRV_TX_RING_H

#include <stdint.h>

/*
 * 发送用字节环形队列。纯逻辑：不含 HAL、不含 RTOS、不做 I/O，
 * 存储由调用方给进来，因此可以在 PC 上直接单测（见 tests/test_tx_ring.py）。
 *
 * 为什么单独成一个 Driver 模块，而不是写进那个 UART 模块里：
 * 环形索引的回绕、满/空判定、"整包要么全进要么不进"这三件事是这次改造里
 * 唯一会算错的部分，而它们恰好一点硬件都不需要。留在 BSP 里就只能靠烧录验证。
 *
 * === 契约 ===
 *
 * **不是线程安全的**。索引由调用方在临界区里更新——本仓库里的调用方是
 * BSP/Src/bsp_uart8_tx.c，它用 PRIMASK 把 Push 与 Release 各自围起来。
 * 把锁放进这里会强迫这个模块知道 RTOS，那它就不再能在 PC 上单测了。
 *
 * **Push 是全有或全无的**。塞不下就一个字节都不写、返回 0。
 * 这条不是风格问题：出口上跑的是 $X 帧，写进去半帧会让上位机把后面的字节
 * 当成帧体解析，一路错到下一个偶然出现的 0x24 0x58——比丢掉整帧糟得多。
 */

typedef struct {
    uint8_t *buffer;
    uint32_t size;
    uint32_t head;   /* 下一个写入位置 */
    uint32_t tail;   /* 下一个待发位置 */
    uint32_t used;   /* 已占用字节数。head==tail 时靠它区分空与满 */
} DRV_TxRing;

/* buffer 为 NULL 或 size 为 0 时队列置空，之后所有 Push 都返回 0。 */
void DRV_TxRing_Init(DRV_TxRing *ring, uint8_t *buffer, uint32_t size);

uint32_t DRV_TxRing_Used(const DRV_TxRing *ring);
uint32_t DRV_TxRing_Free(const DRV_TxRing *ring);

/* 全有或全无。成功返回 1。 */
uint8_t DRV_TxRing_Push(DRV_TxRing *ring, const uint8_t *data, uint32_t length);

/*
 * 从 tail 起**不跨越缓冲末尾**的一段连续字节，供 DMA 直接取用（零拷贝）。
 * 返回该段长度，0 表示队列为空。*chunk 只在返回值非 0 时有意义。
 *
 * 一次只给连续的一段，是因为 DMA 一次只能搬一段连续内存：跨过末尾的部分
 * 留到下一次取，正好由发送完成中断接着发。
 */
uint32_t DRV_TxRing_PeekContiguous(const DRV_TxRing *ring, const uint8_t **chunk);

/* 确认前 length 字节已发走。length 超过当前占用量时按占用量截断。 */
void DRV_TxRing_Release(DRV_TxRing *ring, uint32_t length);

#endif /* DRV_TX_RING_H */
