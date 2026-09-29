#ifndef BSP_DSHOT_RX_H
#define BSP_DSHOT_RX_H

#include <stdint.h>

#include "bsp_esc_protocol.h"
#include "drv_dshot_telemetry.h"

/*
 * 双向 DShot 的**接收相**：发完一帧后把 TIM1 的通道从输出比较翻成输入捕获，
 * 收电调回传的 21 bit GCR 帧。
 *
 * ──────────────── 为什么不需要新的 DMA 流 ────────────────
 *
 * 全部 16 条 DMA 流（DMA1/DMA2 各 8 条）在本工程里**已经分配光了**，没有富余给
 * 输入捕获。但发送用的 DMA1_Stream2 在两帧之间是空闲的：500 Hz 提交，一帧发完
 * 只占几十微秒，剩下将近 2 ms 什么都不干。
 *
 * 所以这里复用同一条流，靠改 DMAMUX 的请求号在两相之间切换：
 *   发送相 → DMA_REQUEST_TIM1_UP(15)
 *   接收相 → DMA_REQUEST_TIM1_CH1(11) 或 TIM1_CH2(12)
 * 两相在时间上严格不重叠，所以不存在争用。代价是**一次只能收一路**，
 * 因此两个电调轮流采——每路的转速更新率是提交率的一半（500 Hz 提交 → 250 Hz）。
 *
 * ──────────────── 为什么不用中断收尾 ────────────────
 *
 * 回传帧在发完之后约 30 us 开始、约 56 us 结束，而下一次提交在 2 ms 之后。
 * 也就是说到下一拍时，捕获要么早就完成、要么根本没来。于是收尾不需要任何中断：
 * 下一次 `BSP_DShot_Submit` 进临界区时顺手把 DMA 的剩余计数读出来就行。
 * 这给 500 Hz 控制路径增加的中断数是**零**。
 */

typedef struct {
    uint32_t erpm[2];          /* 最近一次成功解码的电转速 */
    uint16_t period_us[2];     /* 对应的电周期，us */
    uint32_t frames[2];        /* 成功解码累计 */
    uint32_t crc_errors[2];    /* GCR/校验失败累计 */
    uint32_t timeouts[2];      /* 到下一拍仍未收到足够边沿 */
    uint32_t age_ticks[2];     /* 距上次成功已过多少个发送帧；0 = 本拍刚更新 */
    uint32_t sample_ms[2];     /* 最近一次 eRPM/未旋转回包的真实 HAL ms */
    uint32_t current_sample_ms[2]; /* 最近一次 EDT current 回包的真实 HAL ms */
    uint16_t current_a[2];     /* EDT 0x06，1 A/LSB；valid 见 current_valid */
    /*
     * 开机自检宽限期的剩余帧数。非 0 表示**本模块此刻根本没在听**——线正被驱动在
     * 空闲高，好让电调自检出双向模式（见 bsp_dshot_rx.c 文件头）。它必须可见：
     * 一个看不见的等待期会让人把"还没开始收"误判成"收不到"，转去查完全无关的地方。
     */
    uint16_t detect_grace;
    uint8_t  not_spinning[2];  /* 电调明确回报未旋转（有效回包） */
    uint8_t  valid[2];         /* 该路是否拿到过至少一帧 */
    uint8_t  current_valid[2]; /* 每路独立；不代表硬件一定支持 */
    uint8_t  available;        /* 0 = 本次构建不是双向档，其余字段无意义 */
} BSP_DShotRxSnapshot;

#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR

/* 复位接收状态与计数。由 BSP_DShot_Init 调用。 */
void BSP_DShotRx_Init(uint32_t timer_clock_hz);

/*
 * 启动接收相。**由发送完成的 DMA 回调调用**（ISR 上下文，只写寄存器）。
 * 调用前定时器必须已停、UDE 已关——发送侧的完成回调本来就做了这两件事。
 */
void BSP_DShotRx_Start(void);

/*
 * 收尾并解码。由下一次提交（或禁用）在临界区里调用。
 * 无论有没有收到东西，返回时定时器与 DMA 都已恢复成可发送状态。
 */
void BSP_DShotRx_Harvest(void);

/* 立即取消接收相并把通道恢复成输出。禁用路径用，可在关中断时调用。 */
void BSP_DShotRx_Cancel(void);

void BSP_DShotRx_GetSnapshot(BSP_DShotRxSnapshot *out);

#else  /* 非双向档：给出同名空实现，调用点不必到处写条件编译 */

static inline void BSP_DShotRx_Init(uint32_t timer_clock_hz) { (void)timer_clock_hz; }
static inline void BSP_DShotRx_Start(void) { }
static inline void BSP_DShotRx_Harvest(void) { }
static inline void BSP_DShotRx_Cancel(void) { }
void BSP_DShotRx_GetSnapshot(BSP_DShotRxSnapshot *out);

#endif

#endif /* BSP_DSHOT_RX_H */
