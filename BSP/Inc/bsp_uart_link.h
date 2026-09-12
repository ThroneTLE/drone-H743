#ifndef BSP_UART_LINK_H
#define BSP_UART_LINK_H

#include "bsp_uart_events.h"

#include <stdint.h>

/*
 * 一条 UART 链路的收发机制，**按角色**取用（角色定义见 bsp_uart_events.h）。
 *
 * === 为什么不是"把 HAL 调用换个名字" ===
 *
 * 这层的接口刻意按**能力**描述，不按 HAL 的函数名描述：
 *   "开始收，收到线路空闲为止"   而不是 HAL_UARTEx_ReceiveToIdle_DMA
 *   "缓冲里已经有多少字节"       而不是 __HAL_DMA_GET_COUNTER
 *   "这条链路出错了吗"           而不是 huart->Instance->ISR 的位
 * 换一族 MCU（甚至换成 Linux 上的 tty）时，这些问题依然成立，只是答案的实现变了；
 * 而上层的行帧组装、停摆重启策略、CRSF 解析一行都不用改。
 *
 * 之前这些调用直接散在 `app_uart.c` 与 `app_elrs.c` 里，于是 App 层同时绑上了
 * ST 的 HAL、DMA 流的内部结构、和这块板上的实例号。
 *
 * === 契约 ===
 *
 * - 所有函数对**未绑定的角色**安全返回（0 / 无操作），不会解空指针。
 * - 接收缓冲由调用方提供，且必须落在该 UART 的 DMA 够得到的 RAM
 *   （本工程用 `.dma_buffer` 段；DTCM 对 DMA1/2 不可达）。cache 维护仍归调用方，
 *   因为只有它知道什么时候读了这段缓冲。
 * - 本模块只做机制，不做策略：什么时候重启、停摆算多久，都由上层决定。
 */

/* 接收是否仍在跑。0 = 停了，上层该重启。 */
uint8_t BSP_UartLink_RxIsRunning(BSP_UartRole role);

/* 该链路有没有配 DMA（收 / 发各问一次）。 */
uint8_t BSP_UartLink_HasRxDma(BSP_UartRole role);
uint8_t BSP_UartLink_HasTxDma(BSP_UartRole role);

/*
 * 开始接收，收到线路空闲（IDLE）或缓冲写满为止，事件经
 * BSP_UartEvents 的 rx_event 回调上报。返回 0 = 启动失败。
 *
 * 半满中断一并关掉：上层要的是"一帧收完了"，半满事件只会让它多醒一次。
 */
uint8_t BSP_UartLink_StartRxToIdle(BSP_UartRole role, uint8_t *buffer,
                                   uint16_t size);

/* 同上，但不用 DMA（纯中断）。给没有 DMA 预算的板子留的退路。 */
uint8_t BSP_UartLink_StartRxToIdleIt(BSP_UartRole role, uint8_t *buffer,
                                     uint16_t size);

/* 缓冲里已经收到多少字节（0..size）。没在收或没有 DMA 时返回 0。 */
uint16_t BSP_UartLink_RxFilled(BSP_UartRole role, uint16_t size);

void BSP_UartLink_AbortRx(BSP_UartRole role);

/*
 * 链路错误的**可移植**分类位。
 *
 * 刻意不透传 HAL 的错误码：上层要分开统计"溢出"和"帧错"是因为这两者指向完全
 * 不同的原因（前者是自己读得不够快，后者是线上电平不对），而这个区分在任何
 * MCU 上都成立；`HAL_UART_ERROR_ORE` 这个具体数值则不成立。
 */
#define BSP_UART_LINK_ERR_OVERRUN 0x01U  /* 收得比取得快，字节被覆盖 */
#define BSP_UART_LINK_ERR_FRAMING 0x02U  /* 停止位不对：波特率或电平问题 */
#define BSP_UART_LINK_ERR_NOISE   0x04U  /* 采样点之间电平抖动 */
#define BSP_UART_LINK_ERR_PARITY  0x08U
#define BSP_UART_LINK_ERR_TIMEOUT 0x10U  /* 接收超时（RTOF） */

/*
 * 取走并清掉该链路的错误位，返回上面那组标志的按位或（0 = 没有错误）。
 *
 * 取和清必须是同一次调用：分成两步的话，两步之间新来的错误会被无声清掉，
 * 而计数器上不会留下任何痕迹。
 */
uint32_t BSP_UartLink_TakeErrors(BSP_UartRole role);

/*
 * 丢弃接收 FIFO 里的残字节，并把链路的错误状态复位。
 *
 * 出错后必须做：不丢 FIFO 的话，重启接收会把出错那一刻的半个字节当成新帧的
 * 第一个字节，错位会一直传染下去。ELRS 那条链路实测过——把这一步去掉，
 * 真帧率从 919/s 掉到 280/s，坏帧从 8/s 涨到 700+/s。
 */
void BSP_UartLink_FlushRx(BSP_UartRole role);

/* 发送是否空闲（可以发下一段）。 */
uint8_t BSP_UartLink_TxIsIdle(BSP_UartRole role);

/*
 * 硬件上这一帧是不是真发完了——**不看驱动的记账，只看线上的状态**。
 *
 * 存在的理由：发送完成中断有可能被错过（实测见过 HAL 已经回到 READY、而上层的
 * busy 标志还挂着）。只等回调的话，发送路径会就此死锁，表现为"数传突然不说话了"。
 * 这个函数问的是"最后一个字节的停止位走完了吗、还有没有待搬的 DMA 请求"，
 * 在任何 UART 上都答得出来，所以上层可以拿它做兜底。
 */
uint8_t BSP_UartLink_TxIsComplete(BSP_UartRole role);

/* DMA 发送一段。返回 0 = 没发出去。 */
uint8_t BSP_UartLink_TransmitDma(BSP_UartRole role, const uint8_t *data,
                                 uint16_t length);

/*
 * 关掉该链路的接收错误中断。
 *
 * 给单线/半双工或线上本来就会出现空闲毛刺的链路用（ELRS 接收机未上电时
 * RX 脚悬空，噪声会把 ORE/FE 刷满）。丢的只是"报错"，数据照收；
 * 上层仍能通过 BSP_UartLink_GetError 主动查。
 */
void BSP_UartLink_DisableRxErrorInterrupts(BSP_UartRole role);

/*
 * 给接收脚加偏置。接收机没插或没上电时，悬空的 RX 会让 UART 疯狂报帧错。
 * 具体是上拉还是下拉取决于总线的空闲电平，所以由 BSP 按角色决定，
 * 上层只说"请把这条链路的接收脚偏置好"。
 */
void BSP_UartLink_ApplyRxBias(BSP_UartRole role);

/* 释放该链路（反初始化），把引脚交还给外部工具。 */
void BSP_UartLink_Release(BSP_UartRole role);

/*
 * 把发送 DMA 强制设回 NORMAL 模式。
 *
 * CubeMX 把某条流生成成 CIRCULAR 时，第一帧发完 DMA 不会停，后面每帧都跟着
 * 重发前一帧的尾巴。这是"配置漂移"的补救，不是常规路径。
 */
void BSP_UartLink_ForceTxDmaNormalMode(BSP_UartRole role);

#endif /* BSP_UART_LINK_H */
