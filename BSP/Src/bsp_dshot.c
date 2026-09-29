/* STM32H743 binding for the PX4-derived drv_dshot encoder.
 * PX4 v1.16.0 dshot.c: interleaved CCR burst, DMA UP -> DMAR, cache clean.
 * Local HAL binding is not the PX4/NuttX DMA allocator. See fixture provenance.
 * No cyclic DMA: a stalled control producer cannot replay a nonzero frame.
 */
#include "bsp_dshot.h"
#include "bsp_cache.h"
#include "bsp_critical.h"
#include "bsp_esc_protocol.h"
#include "bsp_dshot_rx.h"
#include "drv_dshot.h"
#include "drv_dshot_telemetry.h"
#include "tim.h"

#include <stddef.h>
#include <string.h>

/*
 * 本文件是双向档的 **TIMER 后端**（DMA burst + 输入捕获），以及单向档/PWM 档
 * 共用的发送路径。双向档选 BITBANG 后端时整份实现不参与编译——两个后端提供
 * 同一组 BSP_DShot_* 符号，同时编进去会重复定义。见 bsp_esc_protocol.h。
 */
#if !BSP_ESC_PROTOCOL_IS_BITBANG

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

/*
 * 把通道停到"无命令"电平。
 *
 * 单向 DShot 的空闲是**低**，双向 DShot 的空闲是**高**（线电平整体取反）。
 * 两种情况下本函数都走 FORCED_INACTIVE，因为双向档把 CCxP 设成了低有效，
 * OCxREF=0 经过反相之后正好是高——所以这一段代码不必分叉，但名字只对单向档
 * 字面成立。两种空闲电平对电调都是"收不到帧"，超时后停转，失效方向一致。
 */
static void hold_idle(uint8_t mask)
{
    if (htim1.Instance != TIM1) { return; }
    if ((mask & 1U) != 0U) { channel_mode(0U, 0U); }
    if ((mask & 2U) != 0U) { channel_mode(1U, 0U); }
    if ((mask & 3U) == 3U) {
        /* OSSI + idle levels make MOE=0 a driven idle (low uni / high bidir). */
        htim1.Instance->BDTR &= ~TIM_BDTR_MOE;
    }
}

/*
 * 两档 DShot 的唯一数字差别是**校验取反**，线上交错完全一样，所以交错那段
 * PX4 来源的代码只保留一份（DRV_DShot_BuildBurstFromPackets）。
 */
static uint8_t encode_packets(const uint16_t code[2], uint16_t packet[2])
{
    for (uint8_t i = 0U; i < 2U; ++i) {
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR
        if (DRV_DShotTelem_EncodeRequest(code[i], 0U, &packet[i]) !=
            DRV_DSHOT_TELEM_OK) { return 0U; }
#else
        if (DRV_DShot_Encode(code[i], &packet[i]) != DRV_DSHOT_OK) { return 0U; }
#endif
    }
    return 1U;
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
    hold_idle(3U);
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
        /*
         * 发送刚停、下一拍还有约 2 ms，正是电调回话的窗口。接收相只写寄存器，
         * 不解码、不等待——解码留到下一拍的 Harvest，所以这里仍然是短 ISR。
         * 非双向档里这是个空函数，连分支都不会生成。
         */
        BSP_DShotRx_Start();
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
    hold_idle(3U);
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
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR
    /*
     * 双向 DShot 线电平取反：空闲高、数据位拉低。两路设成低有效（CCxP=1），
     * 于是 OCxREF=0 对应引脚为高。MOE=0 时的空闲电平由 OISx 决定，同样必须是高——
     * 否则关输出的一瞬间会给电调一段长低电平，那在双向协议里是一串假起始位。
     */
    htim1.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P |
                           TIM_CCER_CC2E | TIM_CCER_CC2P;
    htim1.Instance->CR2 |= TIM_CR2_OIS1 | TIM_CR2_OIS2;
#else
    htim1.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    htim1.Instance->CR2 &= ~(TIM_CR2_OIS1 | TIM_CR2_OIS2);
#endif
    htim1.Instance->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR;
    htim1.Instance->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_2TRANSFERS;
    htim1.Instance->EGR = TIM_EGR_UG;
    htim1.Instance->SR = 0U;
    dma->XferCpltCallback = dma_complete;
    dma->XferErrorCallback = dma_error;
    dma->XferHalfCpltCallback = NULL;
    dma->XferAbortCallback = NULL;
    state.timer_clock_hz = timing.timer_clock_hz;
    BSP_DShotRx_Init(timing.timer_clock_hz);
    initialized = 1U;
    return BSP_DSHOT_OK;
}

/*
 * 特殊命令帧（1..47）。载荷与油门帧同构，差别只在 telemetry 位和取值范围，
 * 所以这里同样只做"选哪个编码器"，校验属于 Driver。
 *
 * 为什么不把它并进 encode_packets()：油门路径必须**永远**编不出特殊命令来。
 * 两者共用一个入口，就等于把"数值算错时会发生什么"从"电调收到一个离谱油门"
 * 变成"电调可能收到一条改转向或写 Flash 的命令"。分开是判据，不是洁癖。
 */
static uint8_t encode_command_packets(uint16_t command, uint16_t packet[2])
{
    for (uint8_t i = 0U; i < 2U; ++i) {
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR
        if (DRV_DShotTelem_EncodeCommand(command, &packet[i]) !=
            DRV_DSHOT_TELEM_OK) { return 0U; }
#else
        if (DRV_DShot_EncodeCommand(command, &packet[i]) != DRV_DSHOT_OK) {
            return 0U;
        }
#endif
    }
    return 1U;
}

/*
 * 已经编好的两帧 -> 线上。`Submit` 与 `SubmitCommand` 唯一的差别是 packet 从哪来；
 * 之后的临界区、Harvest 收尾、abort、DMA 启动顺序**必须是同一段代码**。复制一份
 * 出来意味着以后改 DMA 时序要改两处，而漏改的那处只在一种用法下暴露。
 *
 * `code[]` 只进快照供诊断读，不参与线上编码——命令帧填的是命令号。
 */
static BSP_DShotStatus submit_packets(const uint16_t packet[2],
                                      const uint16_t code[2],
                                      uint8_t enabled_mask)
{
    uint32_t lock = BSP_Critical_Enter();
    if (!initialized || state.fault) { BSP_Critical_Exit(lock); return BSP_DSHOT_ERROR; }
    /*
     * 上一拍的回传就在这时候收尾：此刻发送必定已经结束（busy 在完成回调里清零），
     * 定时器和 DMA 都归本函数支配。收完把两者恢复成可发送状态，下面照常走。
     */
    BSP_DShotRx_Harvest();
    if (state.busy) {
        state.busy_rejected++;
        BSP_Critical_Exit(lock);
        return BSP_DSHOT_BUSY;
    }
    state.busy = 1U; /* reserve buffer while preparing outside critical section */
    const uint32_t ticket = generation;
    BSP_Critical_Exit(lock);
    (void)DRV_DShot_BuildBurstFromPackets(packet, &timing, dma_words,
                                          DRV_DSHOT_BURST_WORDS);
    BSP_Cache_CleanDCache(dma_words, sizeof(dma_words));
    lock = BSP_Critical_Enter();
    if (ticket != generation || state.fault) {
        state.busy = 0U;
        BSP_Critical_Exit(lock);
        return state.fault ? BSP_DSHOT_ERROR : BSP_DSHOT_BUSY;
    }
    hold_idle(3U);
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

BSP_DShotStatus BSP_DShot_Submit(const uint16_t code[2], uint8_t enabled_mask)
{
    uint16_t packet[2];
    if ((code == NULL) || (enabled_mask > 3U) ||
        (encode_packets(code, packet) == 0U)) { return BSP_DSHOT_INVALID; }
    if (enabled_mask == 0U) { return BSP_DShot_Disable(3U); }
    return submit_packets(packet, code, enabled_mask);
}

BSP_DShotStatus BSP_DShot_SubmitCommand(uint16_t command, uint8_t enabled_mask)
{
    uint16_t packet[2];
    uint16_t code[2];

    /*
     * mask=0 在这里是**非法**，不像 Submit 那样当成"全部禁用"。给一条命令配一个
     * 空目标是调用方写错了，而把它翻译成"顺便把两路输出关掉"是一个没人要求过的
     * 副作用——诊断命令不该有副作用。
     */
    if ((enabled_mask == 0U) || (enabled_mask > 3U)) { return BSP_DSHOT_INVALID; }
    if (encode_command_packets(command, packet) == 0U) { return BSP_DSHOT_INVALID; }
    code[0] = command; code[1] = command;
    return submit_packets(packet, code, enabled_mask);
}

BSP_DShotStatus BSP_DShot_Disable(uint8_t channel_mask)
{
    if (channel_mask == 0U || channel_mask > 3U) { return BSP_DSHOT_INVALID; }
    uint32_t lock = BSP_Critical_Enter();
    /* 禁用要立即生效：接收相还占着定时器通道时，先把它撤了再停输出。 */
    BSP_DShotRx_Cancel();
    hold_idle(channel_mask);
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

#endif /* !BSP_ESC_PROTOCOL_IS_BITBANG */

/*
 * PE9 = TIM1_CH1 = 电调 1，PE11 = TIM1_CH2 = 电调 2（Core/Src/tim.c 的 MspInit）。
 * 直接读 IDR：引脚配成定时器复用输出时 IDR 仍然反映真实电平，所以这是量出来的，
 * 不是把刚写进去的寄存器再念一遍。
 */
uint8_t BSP_DShot_ReadEscPinLevels(void)
{
    const uint32_t idr = GPIOE->IDR;
    uint8_t levels = 0U;

    if ((idr & (1UL << 9U)) != 0U) { levels |= 1U; }
    if ((idr & (1UL << 11U)) != 0U) { levels |= 2U; }
    return levels;
}
