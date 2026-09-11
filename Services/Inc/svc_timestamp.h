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
 * 毫秒版，给"超时/节流/多久没动静"这类粗粒度判断用。
 *
 * 加这个是为了让 App 层不必为了拿个时间去 #include HAL 头调 HAL_GetTick()
 * （D1-2：App/Services 不直接依赖 HAL）。换芯片时时基实现换掉，
 * 上层这些超时判断一行不用改。
 *
 * 32 位毫秒约 49.7 天回绕。判超时请一律写成 `(now - then) >= limit` 的
 * 无符号差值形式，回绕时才仍然成立。
 */
uint32_t SVC_Timestamp_Ms(void);

#endif /* SVC_TIMESTAMP_H */
