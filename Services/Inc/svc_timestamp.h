#ifndef SVC_TIMESTAMP_H
#define SVC_TIMESTAMP_H

#include <stdint.h>

/*
 * Timestamp Service
 *
 * Provides a 64-bit microsecond counter using TIM17.
 *
 * CubeMX must set TIM17 prescaler = 119 (120 MHz → 1 MHz tick)
 * and period = 65535 (16-bit max, overflows every 65,536 µs).
 *
 * The overflow ISR (HAL_TIM_PeriodElapsedCallback → SVC_Timestamp_Tick)
 * increments a 32-bit high word to extend the 16-bit hardware counter
 * to a full 64-bit microsecond timestamp.
 *
 * Usage:
 *   uint64_t t = SVC_Timestamp_Us();
 *   // for profiling: uint32_t dt = (uint32_t)(SVC_Timestamp_Us() - t0);
 */

void SVC_Timestamp_Init(void);   /* Start TIM17 with update interrupt */
void SVC_Timestamp_Tick(void);   /* Call from HAL_TIM_PeriodElapsedCallback */
uint64_t SVC_Timestamp_Us(void); /* 64-bit microsecond since init */

/*
 * 毫秒版，给"超时 / 节流 / 多久没动静"这类粗粒度判断用。
 *
 * 存在的理由是 D1-2：App 与 Services 不该为了拿个时间就去 #include HAL 头。
 * 换芯片时只有本服务的实现要改，上层所有超时判断一行不动。
 *
 * **它返回的就是 HAL 时基那个计数器，不是 TIM17 的 Us()/1000。**
 * 这一条是硬要求，不是实现细节：`Driver/` 里的 `drv_gps.c`、
 * `drv_optical_flow.c`、`drv_servo.c` 都用 HAL 时基给数据打时间戳
 * （`last_rx_ms` / `received_ms`），这些值经 BSP 一路交到 App 手里被相减。
 * 要是这里换成另一个计数器，上下两半就在拿**两个不同的钟**比大小——
 * 偏差恒定、方向固定，症状是"某个超时永远不触发"或"一上来就超时"，
 * 而两边代码单看都对。想把整条链路改到 TIM17 上去，得连 Driver 一起改，
 * 那时也只改本文件这一处实现。
 *
 * 32 位毫秒约 49.7 天回绕。判超时请一律写成 `(now - then) >= limit` 的
 * 无符号差值形式，回绕时才仍然成立。
 */
uint32_t SVC_Timestamp_Ms(void);

/*
 * 忙等若干毫秒。**不让出 CPU**，只在调度器还没起来时用（上电阶段）。
 *
 * 调度器跑起来之后一律用 osDelay：在这儿忙等会把同优先级的任务一起堵住。
 * 提供它是为了让 App 层的"上电阶段等一下"不必去调 HAL_Delay()。
 */
void SVC_Timestamp_BusyWaitMs(uint32_t ms);

#endif /* SVC_TIMESTAMP_H */
