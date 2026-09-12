/* STM32H743 binding for the PX4-derived drv_dshot encoder.
 * PX4 v1.16.0 dshot.c: interleaved CCR burst, DMA UP -> DMAR, cache clean.
 * Local HAL binding is not the PX4/NuttX DMA allocator. See fixture provenance.
 * No cyclic DMA: a stalled control producer cannot replay a nonzero frame.
 */
#include "bsp_dshot.h"
#include "bsp_cache.h"
#include "bsp_critical.h"
#include "drv_dshot.h"
#include "tim.h"

#include <stddef.h>
#include <string.h>

/* 40 words make the whole object cache-line-sized. NOLOAD requires explicit init. */
static uint32_t dma_words[40] __attribute__((section(".dma_buffer"), aligned(32)));
static DRV_DShotTiming timing;
static BSP_DShotSnapshot state;
static uint32_t generation;
static uint8_t initialized;

static DMA_HandleTypeDef *dma_handle(void) { return htim1.hdma[TIM_DMA_ID_UPDATE]; }

static void channel_mode(uint8_t channel, uint8_t enabled)
{
    const uint32_t mode = enabled ? TIM_OCMODE_PWM1 : TIM_OCMODE_FORCED_INACTIVE;
    if (channel == 0U) {
        MODIFY_REG(htim1.Instance->CCMR1, TIM_CCMR1_OC1M, mode);
    } else {
        MODIFY_REG(htim1.Instance->CCMR1, TIM_CCMR1_OC2M, mode << 8U);
    }
}

static void hold_low(uint8_t mask)
{
    if (htim1.Instance != TIM1) { return; }
    if ((mask & 1U) != 0U) { channel_mode(0U, 0U); }
    if ((mask & 2U) != 0U) { channel_mode(1U, 0U); }
    if ((mask & 3U) == 3U) {
        /* OSSI + reset idle levels make MOE=0 a driven-low idle. */
        htim1.Instance->BDTR &= ~TIM_BDTR_MOE;
    }
}

/* Bounded register-only abort. HAL_DMA_Abort uses HAL_GetTick and is forbidden
 * here: ROM DFU calls Disable with interrupts masked. No RTOS/HAL tick wait.
 * Caller holds the short critical section; requests are disabled first.
 */
static uint8_t abort_dma(void)
{
    DMA_HandleTypeDef *dma = dma_handle();
    htim1.Instance->DIER &= ~TIM_DIER_UDE;
    htim1.Instance->CR1 &= ~TIM_CR1_CEN;
    if ((dma == NULL) || (dma->Instance != DMA1_Stream2)) { return 0U; }
    DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)dma->Instance;
    stream->CR &= ~(DMA_SxCR_EN | DMA_SxCR_TCIE | DMA_SxCR_HTIE |
                    DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    for (uint32_t tries = 0U; tries < 256U; ++tries) {
        if ((stream->CR & DMA_SxCR_EN) == 0U) {
            __HAL_DMA_CLEAR_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma) |
                __HAL_DMA_GET_HT_FLAG_INDEX(dma) | __HAL_DMA_GET_TE_FLAG_INDEX(dma) |
                __HAL_DMA_GET_DME_FLAG_INDEX(dma) | __HAL_DMA_GET_FE_FLAG_INDEX(dma));
            HAL_NVIC_ClearPendingIRQ(DMA1_Stream2_IRQn);
            dma->State = HAL_DMA_STATE_READY;
            dma->Lock = HAL_UNLOCKED;
            return 1U;
        }
    }
    return 0U;
}

static void latch_fault(void)
{
    hold_low(3U);
    (void)abort_dma();
    state.enabled_mask = 0U;
    state.busy = 0U;
    state.fault = 1U;
    state.errors++;
    generation++;
}

static void dma_complete(DMA_HandleTypeDef *dma)
{
    uint32_t lock = BSP_Critical_Enter();
    if ((dma == dma_handle()) && (state.busy == 2U) && !state.fault) {
        /* HAL records TE/DME/FE before invoking TC, but invokes its error
         * callback afterwards. A combined IRQ must never become a success. */
        if (dma->ErrorCode != HAL_DMA_ERROR_NONE) {
            latch_fault();
            BSP_Critical_Exit(lock);
            return;
        }
        /* Two zero tail slots ensure the final data bit has finished even with
         * CCR preload. At TC the first zero is active, second zero is preloaded. */
        htim1.Instance->DIER &= ~TIM_DIER_UDE;
        htim1.Instance->CR1 &= ~TIM_CR1_CEN;
        state.busy = 0U;
        state.completed++;
    }
    BSP_Critical_Exit(lock);
}

static void dma_error(DMA_HandleTypeDef *dma)
{
    uint32_t lock = BSP_Critical_Enter();
    if ((dma == dma_handle()) && (state.busy == 2U)) { latch_fault(); }
    BSP_Critical_Exit(lock);
}

static uint32_t timer_clock(void)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    const uint32_t pclk = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_TIMPRE) == 0U) {
        return clocks.APB2CLKDivider == RCC_APB2_DIV1 ? pclk : 2U * pclk;
    }
    return (clocks.APB2CLKDivider == RCC_APB2_DIV1 ||
            clocks.APB2CLKDivider == RCC_APB2_DIV2 ||
            clocks.APB2CLKDivider == RCC_APB2_DIV4) ? HAL_RCC_GetHCLKFreq() : 4U * pclk;
}

BSP_DShotStatus BSP_DShot_Init(void)
{
    DMA_HandleTypeDef *dma = dma_handle();
    if (htim1.Instance != TIM1) { return BSP_DSHOT_ERROR; }
    uint32_t lock = BSP_Critical_Enter();
    if (state.busy != 0U) { BSP_Critical_Exit(lock); return BSP_DSHOT_BUSY; }
    hold_low(3U);
    initialized = 0U;
    memset(&state, 0, sizeof(state));
    generation++;
    if ((dma == NULL) || (dma->Instance != DMA1_Stream2) ||
        (dma->Init.Request != DMA_REQUEST_TIM1_UP) ||
        (dma->Init.Direction != DMA_MEMORY_TO_PERIPH) ||
        (dma->Init.Mode != DMA_NORMAL) ||
        (dma->Init.MemDataAlignment != DMA_MDATAALIGN_WORD) ||
        (dma->Init.PeriphDataAlignment != DMA_PDATAALIGN_WORD) ||
        (dma->Init.MemInc != DMA_MINC_ENABLE) ||
        (dma->Init.PeriphInc != DMA_PINC_DISABLE) || !abort_dma()) {
        state.fault = 1U;
        BSP_Critical_Exit(lock);
        return BSP_DSHOT_ERROR;
    }
    BSP_Critical_Exit(lock);
    if (DRV_DShot_MakeTiming(timer_clock(), &timing) != DRV_DSHOT_OK) {
        state.fault = 1U;
        return BSP_DSHOT_ERROR;
    }
    memset(dma_words, 0, sizeof(dma_words));
    htim1.Instance->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;
    htim1.Instance->PSC = timing.prescaler;
    htim1.Instance->ARR = timing.auto_reload;
    htim1.Instance->RCR = 0U;
    htim1.Instance->CCR1 = 0U;
    htim1.Instance->CCR2 = 0U;
    htim1.Instance->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    htim1.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    htim1.Instance->CR2 &= ~(TIM_CR2_OIS1 | TIM_CR2_OIS2);
    htim1.Instance->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR;
    htim1.Instance->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_2TRANSFERS;
    htim1.Instance->EGR = TIM_EGR_UG;
    htim1.Instance->SR = 0U;
    dma->XferCpltCallback = dma_complete;
    dma->XferErrorCallback = dma_error;
    dma->XferHalfCpltCallback = NULL;
    dma->XferAbortCallback = NULL;
    state.timer_clock_hz = timing.timer_clock_hz;
    initialized = 1U;
    return BSP_DSHOT_OK;
}

BSP_DShotStatus BSP_DShot_Submit(const uint16_t code[2], uint8_t enabled_mask)
{
    uint16_t frame;
    if ((code == NULL) || (enabled_mask > 3U) ||
        DRV_DShot_Encode(code[0], &frame) != DRV_DSHOT_OK ||
        DRV_DShot_Encode(code[1], &frame) != DRV_DSHOT_OK) { return BSP_DSHOT_INVALID; }
    if (enabled_mask == 0U) { return BSP_DShot_Disable(3U); }
    uint32_t lock = BSP_Critical_Enter();
    if (!initialized || state.fault) { BSP_Critical_Exit(lock); return BSP_DSHOT_ERROR; }
    if (state.busy) {
        state.busy_rejected++;
        BSP_Critical_Exit(lock);
        return BSP_DSHOT_BUSY;
    }
    state.busy = 1U; /* reserve buffer while preparing outside critical section */
    const uint32_t ticket = generation;
    BSP_Critical_Exit(lock);
    (void)DRV_DShot_BuildBurst(code, &timing, dma_words, DRV_DSHOT_BURST_WORDS);
    BSP_Cache_CleanDCache(dma_words, sizeof(dma_words));
    lock = BSP_Critical_Enter();
    if (ticket != generation || state.fault) {
        state.busy = 0U;
        BSP_Critical_Exit(lock);
        return state.fault ? BSP_DSHOT_ERROR : BSP_DSHOT_BUSY;
    }
    hold_low(3U);
    if (!abort_dma()) {
        latch_fault(); BSP_Critical_Exit(lock); return BSP_DSHOT_ERROR;
    }
    htim1.Instance->CCR1 = 0U;
    htim1.Instance->CCR2 = 0U;
    htim1.Instance->CNT = 0U;
    htim1.Instance->EGR = TIM_EGR_UG;
    htim1.Instance->SR = 0U;
    channel_mode(0U, enabled_mask & 1U);
    channel_mode(1U, enabled_mask & 2U);
    if (HAL_DMA_Start_IT(dma_handle(), (uint32_t)(uintptr_t)dma_words,
                         (uint32_t)(uintptr_t)&htim1.Instance->DMAR,
                         DRV_DSHOT_BURST_WORDS) != HAL_OK) {
        latch_fault(); BSP_Critical_Exit(lock); return BSP_DSHOT_ERROR;
    }
    state.enabled_mask = enabled_mask;
    state.code[0] = code[0]; state.code[1] = code[1];
    state.busy = 2U;
    state.submitted++;
    htim1.Instance->DIER |= TIM_DIER_UDE;
    htim1.Instance->BDTR |= TIM_BDTR_MOE;
    htim1.Instance->CR1 |= TIM_CR1_CEN;
    BSP_Critical_Exit(lock);
    return BSP_DSHOT_OK;
}

BSP_DShotStatus BSP_DShot_Disable(uint8_t channel_mask)
{
    if (channel_mask == 0U || channel_mask > 3U) { return BSP_DSHOT_INVALID; }
    uint32_t lock = BSP_Critical_Enter();
    hold_low(channel_mask);
    state.enabled_mask &= (uint8_t)~channel_mask;
    generation++; /* cancel a concurrently prepared (not yet started) frame */
    if (state.busy == 1U) {
        /* Buffer remains reserved until its producer observes the generation. */
        state.cancelled++;
    } else if (state.enabled_mask == 0U && htim1.Instance == TIM1) {
        if (state.busy) { state.cancelled++; }
        if (!abort_dma()) {
            state.fault = 1U; state.errors++; state.busy = 0U;
            BSP_Critical_Exit(lock); return BSP_DSHOT_ERROR;
        }
        state.busy = 0U;
    }
    BSP_Critical_Exit(lock);
    return BSP_DSHOT_OK;
}

void BSP_DShot_GetSnapshot(BSP_DShotSnapshot *out)
{
    if (out == NULL) { return; }
    uint32_t lock = BSP_Critical_Enter();
    *out = state;
    out->busy = state.busy != 0U;
    BSP_Critical_Exit(lock);
}
