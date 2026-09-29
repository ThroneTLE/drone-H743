/*
 * 双向 DShot 接收相的 STM32H743 绑定。
 *
 * 只做三件事：把 TIM1 通道在"输出比较"和"输入捕获"之间翻面、把 DMA1_Stream2 的
 * DMAMUX 请求号在 TIM1_UP 和 TIM1_CHx 之间切换、把捕获到的时刻交给纯解码层。
 * 协议本身（GCR 表、校验、指数尾数）一行都不在这里——那些在
 * `Driver/Src/drv_dshot_telemetry.c`，由 `tests/test_dshot_telemetry.py` 判对错。
 */
#include "bsp_dshot_rx.h"

/*
 * 只有双向档的 **TIMER 后端**用这份输入捕获接收相。BITBANG 后端自己按端口
 * 过采样，并提供同一个 BSP_DShotRx_GetSnapshot，两边同时编进去会重复定义。
 */
#if (BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR) && !BSP_ESC_PROTOCOL_IS_BITBANG

#include "bsp_cache.h"
#include "bsp_critical.h"
#include "drv_dshot.h"
#include "tim.h"

#include <string.h>

/* 捕获缓冲与发送缓冲一样住在 .dma_buffer（RAM_D2，DMA 可达），32 字节对齐。 */
static uint32_t rx_words[DRV_DSHOT_TELEM_MAX_EDGES]
    __attribute__((section(".dma_buffer"), aligned(32)));

static BSP_DShotRxSnapshot rx_state;
static uint32_t rx_ticks_per_bit_q8;
static uint8_t rx_channel;      /* 本次捕获的通道下标，0/1 轮流 */
static uint8_t rx_active;
static uint8_t rx_ready;

/*
 * ──────────── 开机后先给电调一段干净的高电平（2026-09-21） ────────────
 *
 * AM32 不是靠配置开关进双向模式的，是**自检线上电平**（`Src/dshot.c`
 * `computeDshotDMA()`）：
 *
 *     if (!armed) { if (dshot_telemetry == 0) {
 *         if (getInputPinState()) { high_pin_count++;
 *             if (high_pin_count > 100) { dshot_telemetry = 1; } } } }
 *
 * 也就是**未解锁时每收一帧采样一次引脚，读到高才计数，累计过 100 才切双向**。
 *
 * 而本模块的接收相会把当轮通道翻成输入捕获，一直到下一拍提交（约 2 ms 后）的
 * Harvest 才翻回输出——两路轮流，于是每个电调只有**一半的帧**能看到被驱动的高
 * 电平，另一半采样落在高阻窗口里，读到什么取决于电调那侧有没有上拉。计数虽然
 * 是累计的不会倒退，但"能不能涨"本身就被这个窗口打了对折，而且高阻线在电机
 * PWM 环境里读低是更可能的一侧。
 *
 * 参考实现没有这个问题不是因为做了额外处理，是因为**帧率不同**：Betaflight 在
 * 4~8 kHz 上跑 DShot，一帧 53 us、周期 125 us，高阻窗口只有几十微秒；我们在
 * 500 Hz 上跑，同一段窗口被拉长到近 2 ms。
 *
 * 真正的修法是给接收相一个定时终止点，但 `TIM1_UP_IRQHandler` 在本工程的
 * `Core/Src/stm32h7xx_it.c` 里**不存在**（向量指向 Default_Handler），开中断要
 * 连 CubeMX 生成文件一起改。在花那个代价之前，先用一条零成本的办法把自检这段
 * 让出来：**开机后的头 RX_DETECT_GRACE_FRAMES 帧完全不开接收相**，线 100% 驻在
 * 空闲高（除了帧本身），让 AM32 以满速率累计到阈值。
 *
 * 500 Hz 下 500 帧 = 1 秒，而阈值只要 100 次，留五倍裕量。`dshot_telemetry` 一旦
 * 置位就不再清零（AM32 那段代码里没有任何复位路径），所以之后恢复轮流捕获不会
 * 把它退回去。
 *
 * 这段宽限期是**可观测的**：剩余帧数进快照，`PROPCAL escdiag` 里能看到它归零。
 * 一个看不见的等待期会让人把"还没开始收"误判成"收不到"。
 *
 * ──────────── 实测结论：它**没有**解决电调不回话的问题 ────────────
 *
 * 2026-09-21 实机：宽限期跑满（grace=0，整整 1 秒、500 次采样，而阈值只要 100）
 * 之后仍然 `fr=0 crc=0`，只有超时在涨。随后用 `BSP_DShot_ReadEscPinLevels()` 在一个
 * 完整帧周期上统计，两路引脚 **98~100% 都是高**（低的那 1.7% 正是帧数据本身）——
 * 也就是说**我们的空闲电平本来就是对的**，"高阻窗口打了对折"这个假设不成立。
 *
 * 所以本段保留的理由不再是"它是修复"，而是：它让自检窗口从 50% 占空变成 100%，
 * 方向正确、代价只有开机 1 秒，且已被 `bsp_harness.c` 的 `grace_period()` 钉住。
 * **别把它当成电调不回话的解释**——那个问题在电调侧，见 doc/esc-output.md。
 */
#define BSP_DSHOT_RX_DETECT_GRACE_FRAMES 500U
static uint16_t rx_detect_grace;

/*
 * DMA1_Stream0..7 对应 DMAMUX1_Channel0..7。切请求号必须在 EN=0 时做，
 * 否则 DMAMUX 会把旧请求的残留同步过来。
 */
static void rx_set_request(uint32_t request_id)
{
    MODIFY_REG(DMAMUX1_Channel2->CCR, DMAMUX_CxCR_DMAREQ_ID, request_id);
}

static void rx_stop_stream(void)
{
    DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)DMA1_Stream2;

    stream->CR &= ~(DMA_SxCR_EN | DMA_SxCR_TCIE | DMA_SxCR_HTIE |
                    DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    /* 有界等待：DMA 停流最多几个总线周期，这里不允许出现无界死等。 */
    for (uint32_t tries = 0U; tries < 256U; ++tries) {
        if ((stream->CR & DMA_SxCR_EN) == 0U) {
            break;
        }
    }
    DMA1->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |
                  DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;
}

/* 通道翻成输入捕获，双边沿。 */
static void rx_channel_to_input(uint8_t channel)
{
    if (channel == 0U) {
        MODIFY_REG(htim1.Instance->CCMR1,
                   TIM_CCMR1_CC1S | TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC,
                   TIM_CCMR1_CC1S_0 | (0x3UL << TIM_CCMR1_IC1F_Pos));
        htim1.Instance->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;
    } else {
        MODIFY_REG(htim1.Instance->CCMR1,
                   TIM_CCMR1_CC2S | TIM_CCMR1_IC2F | TIM_CCMR1_IC2PSC,
                   TIM_CCMR1_CC2S_0 | (0x3UL << TIM_CCMR1_IC2F_Pos));
        htim1.Instance->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP;
    }
}

/*
 * 通道翻回输出比较。极性保持**低有效**：双向 DShot 空闲为高，
 * 所以 FORCED_INACTIVE 得到的是高电平，正好是空闲态。
 */
static void rx_channel_to_output(uint8_t channel)
{
    if (channel == 0U) {
        htim1.Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NP);
        MODIFY_REG(htim1.Instance->CCMR1,
                   TIM_CCMR1_CC1S | TIM_CCMR1_OC1M | TIM_CCMR1_IC1F,
                   TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE);
        htim1.Instance->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P;
    } else {
        htim1.Instance->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NP);
        MODIFY_REG(htim1.Instance->CCMR1,
                   TIM_CCMR1_CC2S | TIM_CCMR1_OC2M | TIM_CCMR1_IC2F,
                   ((uint32_t)TIM_CCMR1_OC2M_2) | TIM_CCMR1_OC2PE);
        htim1.Instance->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2P;
    }
}

void BSP_DShotRx_Init(uint32_t timer_clock_hz)
{
    memset(&rx_state, 0, sizeof(rx_state));
    memset(rx_words, 0, sizeof(rx_words));
    rx_state.available = 1U;
    rx_active = 0U;
    rx_ready = 0U;
    rx_channel = 0U;
    rx_detect_grace = (uint16_t)BSP_DSHOT_RX_DETECT_GRACE_FRAMES;
    if (DRV_DShotTelem_TicksPerBitQ8(timer_clock_hz, DRV_DSHOT_BIT_RATE,
                                     &rx_ticks_per_bit_q8) != DRV_DSHOT_TELEM_OK) {
        rx_ticks_per_bit_q8 = 0U;
        return;
    }
    rx_ready = 1U;
}

void BSP_DShotRx_Start(void)
{
    DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)DMA1_Stream2;

    if ((rx_ready == 0U) || (rx_active != 0U) || (htim1.Instance != TIM1)) {
        return;
    }
    if (rx_detect_grace != 0U) {
        /*
         * 自检宽限期内不开接收相：这一拍不把通道翻成输入，线继续被驱动在空闲高，
         * 好让电调那段 `high_pin_count` 以满速率累计。见文件头的说明。
         * 这里只递减不记 timeouts——本拍我们根本没在听，记成超时是在编故障。
         */
        rx_detect_grace--;
        return;
    }

    /* 两路都先停在空闲高电平，再把要收的那一路翻成输入。 */
    htim1.Instance->DIER &= ~(TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC2DE);
    htim1.Instance->CR1 &= ~TIM_CR1_CEN;
    rx_stop_stream();

    rx_channel_to_input(rx_channel);
    rx_set_request((rx_channel == 0U) ? DMA_REQUEST_TIM1_CH1
                                      : DMA_REQUEST_TIM1_CH2);

    /* 捕获相按最高分辨率自由计数：PSC=0、ARR 满量程。 */
    htim1.Instance->PSC = 0U;
    htim1.Instance->ARR = 0xFFFFU;
    htim1.Instance->CNT = 0U;
    htim1.Instance->EGR = TIM_EGR_UG;
    htim1.Instance->SR = 0U;

    stream->PAR = (rx_channel == 0U)
        ? (uint32_t)(uintptr_t)&htim1.Instance->CCR1
        : (uint32_t)(uintptr_t)&htim1.Instance->CCR2;
    stream->M0AR = (uint32_t)(uintptr_t)rx_words;
    stream->NDTR = DRV_DSHOT_TELEM_MAX_EDGES;
    /* 外设->内存、字宽、内存自增、高优先级、不开任何中断（收尾靠下一拍轮询）。 */
    stream->CR = DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
                 DMA_SxCR_MINC;
    stream->FCR &= ~DMA_SxFCR_DMDIS;
    stream->CR |= DMA_SxCR_EN;

    htim1.Instance->DIER |= (rx_channel == 0U) ? TIM_DIER_CC1DE : TIM_DIER_CC2DE;
    htim1.Instance->CR1 |= TIM_CR1_CEN;
    rx_active = 1U;
}

static void rx_restore_output_phase(void)
{
    htim1.Instance->DIER &= ~(TIM_DIER_CC1DE | TIM_DIER_CC2DE);
    htim1.Instance->CR1 &= ~TIM_CR1_CEN;
    rx_stop_stream();
    rx_channel_to_output(0U);
    rx_channel_to_output(1U);
    rx_set_request(DMA_REQUEST_TIM1_UP);
    /* 发送相的时基由 BSP_DShot_Submit 在启动前重写，这里不抢它的活。 */
    rx_active = 0U;
}

void BSP_DShotRx_Harvest(void)
{
    uint16_t edges[DRV_DSHOT_TELEM_MAX_EDGES];
    DRV_DShotTelemValue value;
    uint32_t captured;
    uint8_t channel = rx_channel;

    if (rx_active == 0U) {
        return;
    }

    captured = DRV_DSHOT_TELEM_MAX_EDGES -
               (uint32_t)(((DMA_Stream_TypeDef *)DMA1_Stream2)->NDTR);
    rx_restore_output_phase();

    /* 两路的"年龄"都要加一拍；本拍解出来的那路随后清零。 */
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (rx_state.age_ticks[i] < 0xFFFFFFFFU) {
            rx_state.age_ticks[i]++;
        }
    }
    /* 下一拍换另一路——一条 DMA 流只能同时收一个通道。 */
    rx_channel = (uint8_t)(channel ^ 1U);

    if (captured < 2U) {
        rx_state.timeouts[channel]++;
        return;
    }
    if (captured > DRV_DSHOT_TELEM_MAX_EDGES) {
        captured = DRV_DSHOT_TELEM_MAX_EDGES;
    }

    BSP_Cache_InvalidateDCache(rx_words, sizeof(rx_words));
    for (uint32_t i = 0U; i < captured; ++i) {
        edges[i] = (uint16_t)rx_words[i];
    }

    if (DRV_DShotTelem_Decode(edges, (size_t)captured, rx_ticks_per_bit_q8,
                              &value) != DRV_DSHOT_TELEM_OK) {
        rx_state.crc_errors[channel]++;
        return;
    }

    if (value.kind == (uint8_t)DRV_DSHOT_TELEM_VALUE_EDT) {
        if (value.edt_type == DRV_DSHOT_EDT_CURRENT) {
            rx_state.current_a[channel] = value.edt_value;
            rx_state.current_sample_ms[channel] = HAL_GetTick();
            rx_state.current_valid[channel] = 1U;
        }
        /* EDT is a valid decoded frame, but it says nothing about current RPM. */
        rx_state.frames[channel]++;
        return;
    }

    rx_state.erpm[channel] = value.erpm;
    rx_state.period_us[channel] = value.period_us;
    rx_state.not_spinning[channel] = value.not_spinning;
    rx_state.valid[channel] = 1U;
    rx_state.sample_ms[channel] = HAL_GetTick();
    rx_state.age_ticks[channel] = 0U;
    rx_state.frames[channel]++;
}

void BSP_DShotRx_Cancel(void)
{
    if (rx_active == 0U) {
        return;
    }
    rx_restore_output_phase();
}

void BSP_DShotRx_GetSnapshot(BSP_DShotRxSnapshot *out)
{
    if (out == NULL) {
        return;
    }
    uint32_t lock = BSP_Critical_Enter();
    *out = rx_state;
    /* 宽限期计数不住在 rx_state 里（它只在 Start 里被 500 Hz 侧改），临界区内
     * 一并取，保证与其余字段是同一拍的视图。 */
    out->detect_grace = rx_detect_grace;
    BSP_Critical_Exit(lock);
}

#elif !BSP_ESC_PROTOCOL_IS_BITBANG  /* 非双向档：只留一个空快照 */

#include <string.h>

void BSP_DShotRx_GetSnapshot(BSP_DShotRxSnapshot *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->available = 0U;
}

#endif
