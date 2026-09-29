#ifndef DRV_DSHOT_BITBANG_H
#define DRV_DSHOT_BITBANG_H

#include <stddef.h>
#include <stdint.h>

#include "drv_dshot.h"
#include "drv_dshot_telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 双向 DShot300 的 bitbang 收发层：纯数据变换，不含寄存器、DMA、RTOS，
 * 也没有全局可变状态。
 *
 * ──────────────── 为什么不用 DMA burst ────────────────
 *
 * 本板双向档最初用的是"DMA burst 写 TIM1->DMAR"加"发完把通道翻成输入捕获"，
 * 实机验证：帧值手算核对过、线上极性量过（引脚 98~100% idle 高，正确）、
 * 协议档位确认过，但电调完全不响应——不转、也不回传。参考实现明确记载
 * burst DMA 与双向 DShot 不兼容，所以改走 bitbang：一条 DMA 流按固定节拍
 * 写 GPIO 的 BSRR（发送）或读 IDR（接收），两路电调信号脚同在一个 GPIO 口
 * （本板 PE9/PE11 都在 GPIOE），天然可以用同一条 DMA 流同时驱动/采样两路。
 *
 * 引脚号、口线分配是板级事实，归 BSP；本模块只认调用方给的 BSRR 掩码和
 * IDR 采样，不认识具体是哪个引脚。
 *
 * ──────────────── 发送：子槽近似占空比 ────────────────
 *
 * bitbang 没有定时器的占空比寄存器可用，只能在固定节拍上逐点写 BSRR 来
 * 逼近 DShot 的 3/8、6/8 占空比。一个数据位切成 DRV_DSHOT_BB_SLOTS_PER_BIT
 * （8）个等长子槽——8 是刻意选的：DShot 规范的高电平占比是 37.5%（bit=0）
 * 与 75%（bit=1），正好是 3/8 与 6/8，8 子槽能精确命中这两个比例，不是
 * "四舍五入凑个近似值"。
 *
 * ──────────────── 接收：过采样代替边沿捕获 ────────────────
 *
 * DMA burst 方案里回传靠定时器输入捕获直接给出边沿时刻（见
 * drv_dshot_telemetry.h 的 BitsFromEdges）。bitbang 没有输入捕获通道，
 * 只能按固定采样率轮询整个 GPIO 口的 IDR，所以本模块的解码入口是
 * "过采样序列"而不是"边沿时刻表"，边沿要自己从采样序列里找——
 * 这是与 BitsFromEdges 唯一的本质差别，找不到边沿必须明确报错，
 * 不能把"没找到"悄悄当成"信号一直是某个电平"。
 */

/* 一个数据位切成 8 个等长子槽。DShot 规范的 T_H 是 37.5% 与 75%，
 * 正好是 3/8 和 6/8 —— 8 子槽精确命中，不是近似。 */
#define DRV_DSHOT_BB_SLOTS_PER_BIT 8U
#define DRV_DSHOT_BB_FRAME_BITS    16U
/* 帧后再留若干子槽保持空闲电平，保证最后一位真的走完。 */
#define DRV_DSHOT_BB_TAIL_SLOTS    8U
#define DRV_DSHOT_BB_FRAME_WORDS \
    ((DRV_DSHOT_BB_FRAME_BITS * DRV_DSHOT_BB_SLOTS_PER_BIT) + DRV_DSHOT_BB_TAIL_SLOTS)

/* 一路信号脚在 GPIO BSRR 里的两个掩码。BSRR 低 16 位置位（拉高）、
 * 高 16 位复位（拉低），所以两者不是互为补码，必须分别给。
 * 引脚号是板级事实，归 BSP；本模块只做数据变换，不认识引脚。 */
typedef struct {
    uint32_t set_mask;    /* 把该通道拉高 */
    uint32_t reset_mask;  /* 把该通道拉低 */
} DRV_DShotBitbangPins;

/*
 * 两路已编码好的 16 bit 数字帧 -> 每子槽一个 BSRR 值。
 *
 * `inverted` 非零 = 双向档线电平取反：空闲**高**、数据位拉**低**，
 * 于是 bit=0 低 3/8、bit=1 低 6/8。为零 = 单向档：空闲低、数据位拉高。
 *
 * 两路在同一个 BSRR 字里合并，所以它们天然同步，不存在通道间偏移。
 * 只写 DRV_DSHOT_BB_FRAME_WORDS 个字；capacity 不足返回 INVALID，不截断。
 * packet/pins/out 任一为 NULL、或 capacity_words 不够，都不写任何字。
 */
DRV_DShotStatus DRV_DShotBitbang_BuildFrame(const uint16_t packet[2],
                                            const DRV_DShotBitbangPins pins[2],
                                            uint8_t inverted,
                                            uint32_t *out, size_t capacity_words);

/*
 * 过采样的 IDR 序列 -> 21 bit 原始码流。
 *
 * `samples` 是逐次读到的整个端口 IDR；`channel_mask` 取出本路那一位。
 * `samples_per_bit_q8` 是"回传的一个位占多少个采样"的 Q8 定点值，由 BSP 按
 * 真实采样率算出来传进来（回传波特率 = 发送波特率 × 5/4），不在这里写死。
 *
 * 回传是**低有效**：空闲高，起始位是第一个下降沿。序列里找不到下降沿要明确
 * 报 BAD_EDGES，不能把"一片高电平"解成一串 0 —— 那会把"电调没回话"变成
 * 一个看起来合法的读数。同理，起始下降沿之后如果再没有任何翻转，说明剩下
 * 20 bit 没有任何证据支持，同样报 BAD_EDGES，不能悄悄补成"一直保持起始位
 * 那个电平"。
 *
 * 解出 raw21 之后复用 drv_dshot_telemetry.h 里的 DRV_DShotTelem_DecodeRaw()
 * 与 DRV_DShotTelem_ValueFromPayload()，本函数不做 GCR/校验，只负责把
 * 采样流变回比特流。
 *
 * 与 DRV_DShotTelem_BitsFromEdges 的差别：那边的 edge_ticks 由 BSP 用定时器
 * 输入捕获直接给出边沿时刻，第一个元素就是起始下降沿；这里的 samples 是
 * 整段过采样原始数据，起始下降沿要本函数自己从里面找。另外 samples 是
 * 数组下标而不是硬件计数器读数，没有 16 位回绕问题，直接做无符号减法。
 */
DRV_DShotTelemStatus DRV_DShotBitbang_BitsFromSamples(const uint32_t *samples,
                                                      size_t count,
                                                      uint32_t channel_mask,
                                                      uint32_t samples_per_bit_q8,
                                                      uint32_t *out_raw21);

#ifdef __cplusplus
}
#endif
#endif
