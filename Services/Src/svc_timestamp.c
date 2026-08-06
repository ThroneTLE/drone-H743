#include "svc_timestamp.h"

#include "tim.h"   /* htim17 */

/*
 * Lock-free 64-bit timestamp using a 16-bit hardware counter + 32-bit overflow.
 *
 * TIM17 runs at 1 MHz (120 MHz / 120), so CNT increments every 1 µs.
 * On overflow (every 65536 µs), the ISR increments high_count.
 *
 * SVC_Timestamp_Us retries if the overflow ISR updates the high word while it
 * reads. It also accounts for a pending UIF when called from an equal-priority
 * ISR, where the TIM17 handler cannot preempt the caller.
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
    uint32_t high_before;
    uint32_t high_after;
    uint32_t low;
    uint32_t update_pending;

    do {
        high_before = svc_timestamp_high;
        low  = TIM17->CNT;
        update_pending = TIM17->SR & TIM_SR_UIF;
        if (update_pending != 0U) {
            low = TIM17->CNT;
        }
        high_after = svc_timestamp_high;
    } while (high_before != high_after);

    if (update_pending != 0U) {
        ++high_after;
    }
    return (((uint64_t)high_after) << 16) | (uint64_t)low;
}
