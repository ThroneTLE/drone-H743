#include "svc_timestamp.h"

#include "tim.h"   /* htim17 */

/*
 * Lock-free 64-bit timestamp using a 16-bit hardware counter + 32-bit overflow.
 *
 * TIM17 runs at 1 MHz (120 MHz / 120), so CNT increments every 1 µs.
 * On overflow (every 65536 µs), the ISR increments high_count.
 *
 * SVC_Timestamp_Us uses a do-while retry loop: if the ISR fires between
 * reading high_count and CNT, the retry picks up the consistent pair.
 * This avoids disabling interrupts and has no race window.
 */

static volatile uint32_t svc_timestamp_high;

/* ------------------------------------------------------------------ */
/*  Public API                                                        */
/* ------------------------------------------------------------------ */

void SVC_Timestamp_Init(void)
{
    svc_timestamp_high = 0U;
    HAL_TIM_Base_Start_IT(&htim17);
}

void SVC_Timestamp_Tick(void)
{
    ++svc_timestamp_high;
}

uint64_t SVC_Timestamp_Us(void)
{
    uint32_t high;
    uint32_t low;

    do {
        high = svc_timestamp_high;
        low  = TIM17->CNT;
    } while (high != svc_timestamp_high);

    return (((uint64_t)high) << 16) | (uint64_t)low;
}
