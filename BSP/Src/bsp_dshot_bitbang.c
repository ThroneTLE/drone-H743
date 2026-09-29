/*
 * 双向 DShot300 的 **bitbang** 后端：STM32H743 绑定。
 *
 * 只做四件事：按固定节拍把预先算好的 BSRR 字 DMA 到 GPIOE、把两个信号脚在
 * 输出与输入之间翻面、把整个端口的 IDR 过采样 DMA 回内存、把采样交给纯解码层。
 * 协议本身（子槽占空、GCR、校验）一行都不在这里——那些在
 * `Driver/Src/drv_dshot_bitbang.c` 与 `drv_dshot_telemetry.c`，由宿主测试判对错。
 *
 * ──────────────── 为什么换掉 DMA burst ────────────────
 *
 * 原来的 TIMER 后端用 DMA burst 写 `TIM1->DMAR`，发完把通道翻成输入捕获。实机
 * 逐项查过：帧值手算核对、引脚电平量测（98~100% idle 高，正确）、协议档位确认
 * （`proto=2 avail=1`），而电调**完全不响应**——不转、也不回传一帧。参考实现
 * 明确记载 burst DMA 与双向 DShot 不兼容。作者据此决定改用参考实现普遍采用的
 * bitbang，理由不是"burst 一定是元凶"，而是**停止在自研路子上试错**。
 *
 * ──────────────── 为什么一条 DMA 流够用 ────────────────
 *
 * 本板两路电调信号脚 PE9/PE11 **同在 GPIOE**。bitbang 按**端口**操作：一个 BSRR
 * 字同时决定两路的电平，一次 IDR 读同时拿到两路的采样。于是发送与接收都只需要
 * 一条流，而且两路天生同步、回传**同时**收——TIMER 后端是两路轮流，每路只有
 * 提交率的一半。
 *
 * 发送相与接收相在时间上严格不重叠，所以复用同一条 DMA1_Stream2；两相的请求源
 * 都是 TIM1_UP，连 DMAMUX 请求号都不用切（TIMER 后端需要切）。
 *
 * ──────────────── 定时器退化成纯时钟 ────────────────
 *
 * TIM1 不再用任何输出比较通道，只靠 update 事件按 `BB_SAMPLE_HZ` 产生 DMA 请求。
 * 引脚因此由 TIM1 的复用功能改成**普通 GPIO**，由本模块在运行期配置——这是 BSP
 * 对自己独占的引脚做的绑定，与 `.ioc` 里单向档的 AF 配置不冲突（那一档不编译本文件）。
 *
 * ──────────────── 接收窗口为什么带上拉 ────────────────
 *
 * 接收相引脚是输入，线由电调驱动。加内部上拉不是为了"读得更稳"，是为了让**没有
 * 任何一方驱动**的那几十微秒里线仍然停在空闲高——双向 DShot 的空闲电平是高，而
 * AM32 正是靠"未解锁时读到线上是高"来自检双向模式的。浮空线在电机 PWM 环境里
 * 读低是更可能的一侧。
 */

#include "bsp_esc_protocol.h"

#if BSP_ESC_PROTOCOL_IS_BITBANG

#include "bsp_dshot.h"
#include "bsp_dshot_rx.h"
#include "bsp_cache.h"
#include "bsp_critical.h"
#include "drv_dshot.h"
#include "drv_dshot_bitbang.h"
#include "drv_dshot_telemetry.h"
#include "tim.h"

#include <stddef.h>
#include <string.h>

/* PE9 = 电调 1、PE11 = 电调 2（Core/Src/tim.c 的 MspInit 注释即板级事实）。 */
#define BB_PIN1 GPIO_PIN_9
#define BB_PIN2 GPIO_PIN_11
#define BB_PINS (BB_PIN1 | BB_PIN2)

/*
 * 子槽速率。DShot300 的位速率 300 kHz × 8 子槽 = 2.4 MHz。
 * TIM1 时钟 120 MHz ÷ 50 = 2.4 MHz 整除，没有累积相位误差——这正是 8 子槽
 * （而不是参考实现常用的 3 子槽近似）在本板上额外划算的地方。
 */
#define BB_SAMPLE_HZ 2400000UL

/*
 * 接收窗口的采样数。电调在帧尾之后约 30 us 开始回话，21 bit @ 375 kbit/s
 * 约 56 us，合计约 90 us；按 2.4 MHz 采样是 216 点，取 256 留余量并凑整。
 * 采多了只是多占内存：没有回传时后面全是空闲高，解码层会明确报 BAD_EDGES。
 */
#define BB_RX_SAMPLES 256U

/*
 * 回传一个位占多少个采样，Q8 定点。回传波特率 = 发送波特率 × 5/4 = 375 kbit/s，
 * 采样 2.4 MHz，故 6.4 采样/位 -> 6.4 × 256 = 1638。
 * 这里写成表达式而不是常数，改采样率时不会忘了同步改它。
 */
#define BB_SAMPLES_PER_BIT_Q8 \
    ((BB_SAMPLE_HZ * 256UL * DRV_DSHOT_TELEM_RATE_DEN) / \
     (DRV_DSHOT_BIT_RATE * DRV_DSHOT_TELEM_RATE_NUM))

/* 发送与接收缓冲都住在 .dma_buffer（RAM_D2，DMA 可达），32 字节对齐。 */
static uint32_t bb_tx_words[DRV_DSHOT_BB_FRAME_WORDS]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint32_t bb_rx_words[BB_RX_SAMPLES]
    __attribute__((section(".dma_buffer"), aligned(32)));

static BSP_DShotSnapshot state;
static BSP_DShotRxSnapshot rx_state;
static uint32_t generation;
static uint8_t initialized;
static uint8_t rx_active;

static const DRV_DShotBitbangPins bb_pins[2] = {
    { BB_PIN1, (uint32_t)BB_PIN1 << 16U },
    { BB_PIN2, (uint32_t)BB_PIN2 << 16U },
};

static DMA_HandleTypeDef *dma_handle(void) { return htim1.hdma[TIM_DMA_ID_UPDATE]; }

/* 与 TIMER 后端逐字相同的推导（APB2 分频 + TIMPRE）。两处各留一份是因为两个
 * 后端只会编译其中一个，抽公共文件反而要为一份三行的算式新开一个模块。 */
static uint32_t bb_timer_clock_hz(void)
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

/* 引脚翻面。输出用推挽高速；输入带上拉，理由见文件头。 */
static void bb_pins_output(void)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = BB_PINS;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOE, &init);
}

static void bb_pins_input(void)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = BB_PINS;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_PULLUP;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOE, &init);
}

/*
 * 把两路停在空闲电平（双向档 = 高）。引脚是普通输出，直接写 BSRR 即可——
 * 不经过定时器，所以无论 DMA 处在什么状态这一步都成立。
 */
static void bb_hold_idle(void)
{
    bb_pins_output();
    GPIOE->BSRR = BB_PINS;   /* 低 16 位 = 置位 = 拉高 */
}

static void bb_latch_fault(void)
{
    state.enabled_mask = 0U;
    state.busy = 0U;
    state.fault = 1U;
    state.errors++;
    generation++;
    bb_hold_idle();
}

/* 有界的纯寄存器 abort，照搬 TIMER 后端的约束：不调 HAL_DMA_Abort（它用
 * HAL_GetTick），不等中断，禁用请求在先、关流在后。 */
static uint8_t bb_abort_dma(void)
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

/* 定时器只当时钟：不碰任何 CCMR/CCER，只设分频与周期。 */
static void bb_timer_clock(uint32_t rate_hz)
{
    const uint32_t arr = (state.timer_clock_hz / rate_hz);

    htim1.Instance->CR1 &= ~TIM_CR1_CEN;
    htim1.Instance->PSC = 0U;
    htim1.Instance->ARR = (arr > 0U) ? (arr - 1U) : 0U;
    htim1.Instance->CNT = 0U;
    htim1.Instance->EGR = TIM_EGR_UG;
    htim1.Instance->SR = 0U;
}

/*
 * 接收相：引脚翻成输入，DMA 改成"外设->内存"从 IDR 取样。
 * 由发送完成的 DMA 回调调用（ISR 上下文，只写寄存器）。
 */
static void bb_rx_start(void)
{
    DMA_HandleTypeDef *dma = dma_handle();
    DMA_Stream_TypeDef *stream;

    if ((dma == NULL) || (dma->Instance != DMA1_Stream2)) { return; }
    stream = (DMA_Stream_TypeDef *)dma->Instance;

    bb_pins_input();
    stream->NDTR = BB_RX_SAMPLES;
    stream->PAR = (uint32_t)(uintptr_t)&GPIOE->IDR;
    stream->M0AR = (uint32_t)(uintptr_t)bb_rx_words;
    /* 外设->内存、32 位、内存自增、高优先级、不开任何中断：收尾靠下一拍轮询，
     * 给 500 Hz 控制路径增加的中断数是零。 */
    stream->CR = DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
                 DMA_SxCR_MINC;
    stream->CR |= DMA_SxCR_EN;
    htim1.Instance->DIER |= TIM_DIER_UDE;
    htim1.Instance->CR1 |= TIM_CR1_CEN;
    rx_active = 1U;
}

/* 发送完成。停发、立刻进接收相。 */
static void bb_dma_complete(DMA_HandleTypeDef *dma)
{
    uint32_t lock = BSP_Critical_Enter();
    if ((dma == dma_handle()) && (state.busy == 2U) && !state.fault) {
        if (dma->ErrorCode != HAL_DMA_ERROR_NONE) {
            bb_latch_fault();
            BSP_Critical_Exit(lock);
            return;
        }
        htim1.Instance->DIER &= ~TIM_DIER_UDE;
        htim1.Instance->CR1 &= ~TIM_CR1_CEN;
        state.busy = 0U;
        state.completed++;
        bb_rx_start();
    }
    BSP_Critical_Exit(lock);
}

static void bb_dma_error(DMA_HandleTypeDef *dma)
{
    uint32_t lock = BSP_Critical_Enter();
    if (dma == dma_handle()) { bb_latch_fault(); }
    BSP_Critical_Exit(lock);
}

/*
 * 收尾并解码。由下一次提交在临界区外调用：解码是几百个周期的纯整数运算，
 * 放在临界区里会把 500 Hz 路径的关中断时间拉长，没有必要。
 */
static void bb_rx_harvest(void)
{
    uint32_t captured;
    DMA_HandleTypeDef *dma = dma_handle();

    if ((rx_active == 0U) || (dma == NULL)) { return; }
    rx_active = 0U;

    captured = BB_RX_SAMPLES - ((DMA_Stream_TypeDef *)dma->Instance)->NDTR;
    (void)bb_abort_dma();
    BSP_Cache_InvalidateDCache(bb_rx_words, sizeof(bb_rx_words));

    for (uint32_t ch = 0U; ch < 2U; ++ch) {
        const uint32_t mask = (ch == 0U) ? (uint32_t)BB_PIN1 : (uint32_t)BB_PIN2;
        uint32_t raw21 = 0U;
        uint16_t payload = 0U;
        DRV_DShotTelemValue value;

        rx_state.age_ticks[ch]++;
        if (captured < 2U) { rx_state.timeouts[ch]++; continue; }

        if (DRV_DShotBitbang_BitsFromSamples(bb_rx_words, captured, mask,
                                             (uint32_t)BB_SAMPLES_PER_BIT_Q8,
                                             &raw21) != DRV_DSHOT_TELEM_OK) {
            rx_state.timeouts[ch]++;
            continue;
        }
        if (DRV_DShotTelem_DecodeRaw(raw21, &payload) != DRV_DSHOT_TELEM_OK) {
            rx_state.crc_errors[ch]++;
            continue;
        }
        if (DRV_DShotTelem_ValueFromPayload(payload, &value) != DRV_DSHOT_TELEM_OK) {
            rx_state.crc_errors[ch]++;
            continue;
        }

        rx_state.frames[ch]++;
        if (value.kind == (uint8_t)DRV_DSHOT_TELEM_VALUE_EDT) {
            if (value.edt_type == DRV_DSHOT_EDT_CURRENT) {
                rx_state.current_a[ch] = value.edt_value;
                rx_state.current_valid[ch] = 1U;
                rx_state.current_sample_ms[ch] = HAL_GetTick();
            }
            /* EDT 是有效回包，但它不含转速——不能顺手刷新 erpm。 */
            continue;
        }
        /* Only eRPM (including the explicit stopped report) refreshes speed. */
        rx_state.age_ticks[ch] = 0U;
        rx_state.valid[ch] = 1U;
        rx_state.sample_ms[ch] = HAL_GetTick();
        rx_state.erpm[ch] = value.erpm;
        rx_state.period_us[ch] = value.period_us;
        rx_state.not_spinning[ch] = value.not_spinning;
    }
}

/* ──────────────────────────────── 公共接口 */

BSP_DShotStatus BSP_DShot_Init(void)
{
    DMA_HandleTypeDef *dma = dma_handle();

    if ((htim1.Instance != TIM1) || (dma == NULL) ||
        (dma->Instance != DMA1_Stream2)) {
        return BSP_DSHOT_ERROR;
    }
    if (state.busy != 0U) { return BSP_DSHOT_BUSY; }

    memset(&state, 0, sizeof(state));
    memset(&rx_state, 0, sizeof(rx_state));
    memset(bb_tx_words, 0, sizeof(bb_tx_words));
    memset(bb_rx_words, 0, sizeof(bb_rx_words));
    rx_active = 0U;

    /*
     * 时钟来源与 TIMER 后端同一套推导（APB2 分频 + TIMPRE），不写死：写死会在
     * 时钟树改动时静默错位——子槽速率错了表现为"电调不认帧"，和现在这个症状
     * 一模一样，从外面分不出来。
     */
    state.timer_clock_hz = bb_timer_clock_hz();
    if ((state.timer_clock_hz == 0U) ||
        ((state.timer_clock_hz % BB_SAMPLE_HZ) != 0U)) {
        /*
         * 不整除就拒绝初始化。子槽速率是靠 ARR 整数分频得到的，除不尽意味着
         * 每个子槽都带固定相位误差，累计 16 bit 之后足以让电调判错位宽。
         * 与其发一串电调不认的帧，不如明确起不来。
         */
        return BSP_DSHOT_ERROR;
    }

    (void)bb_abort_dma();
    bb_hold_idle();
    bb_timer_clock(BB_SAMPLE_HZ);

    dma->XferCpltCallback = bb_dma_complete;
    dma->XferErrorCallback = bb_dma_error;
    dma->XferHalfCpltCallback = NULL;
    dma->XferAbortCallback = NULL;

    rx_state.available = 1U;
    initialized = 1U;
    return BSP_DSHOT_OK;
}

static BSP_DShotStatus bb_submit_packets(const uint16_t packet[2],
                                         const uint16_t code[2],
                                         uint8_t enabled_mask)
{
    uint32_t lock;
    uint32_t ticket;
    DMA_HandleTypeDef *dma = dma_handle();
    DMA_Stream_TypeDef *stream;

    if (DRV_DShotBitbang_BuildFrame(packet, bb_pins, 1U, bb_tx_words,
                                    DRV_DSHOT_BB_FRAME_WORDS) != DRV_DSHOT_OK) {
        return BSP_DSHOT_INVALID;
    }

    lock = BSP_Critical_Enter();
    if (!initialized || state.fault) { BSP_Critical_Exit(lock); return BSP_DSHOT_ERROR; }
    if (state.busy) {
        state.busy_rejected++;
        BSP_Critical_Exit(lock);
        return BSP_DSHOT_BUSY;
    }
    state.busy = 1U;
    ticket = generation;
    BSP_Critical_Exit(lock);

    /* 上一拍的回传在这里收尾：此刻发送必定已经结束，定时器与 DMA 归本函数支配。 */
    bb_rx_harvest();
    BSP_Cache_CleanDCache(bb_tx_words, sizeof(bb_tx_words));

    lock = BSP_Critical_Enter();
    if ((ticket != generation) || state.fault) {
        state.busy = 0U;
        BSP_Critical_Exit(lock);
        return state.fault ? BSP_DSHOT_ERROR : BSP_DSHOT_BUSY;
    }
    if (!bb_abort_dma()) {
        bb_latch_fault(); BSP_Critical_Exit(lock); return BSP_DSHOT_ERROR;
    }
    /*
     * 被禁用的那一路：BuildFrame 已经按 packet 算出了两路的波形，但禁用的通道
     * 不该出帧。最省事也最可靠的做法是把它留在输出模式、整帧维持空闲高——
     * 所以这里不改波形，改的是引脚：禁用位对应的脚先拉高，随后 DMA 写 BSRR 时
     * 那一路的位仍会被改写。**因此禁用必须靠 enabled_mask 在调用方就把 code 置零**，
     * 与 TIMER 后端的语义一致：Disable 走 BSP_DShot_Disable，不走这里。
     */
    bb_pins_output();
    GPIOE->BSRR = BB_PINS;
    bb_timer_clock(BB_SAMPLE_HZ);

    stream = (DMA_Stream_TypeDef *)dma->Instance;
    stream->NDTR = DRV_DSHOT_BB_FRAME_WORDS;
    stream->PAR = (uint32_t)(uintptr_t)&GPIOE->BSRR;
    stream->M0AR = (uint32_t)(uintptr_t)bb_tx_words;
    stream->CR = DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
                 DMA_SxCR_MINC | DMA_SxCR_DIR_0 |
                 DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    stream->CR |= DMA_SxCR_EN;

    state.enabled_mask = enabled_mask;
    state.code[0] = code[0]; state.code[1] = code[1];
    state.busy = 2U;
    state.submitted++;
    dma->State = HAL_DMA_STATE_BUSY;
    htim1.Instance->DIER |= TIM_DIER_UDE;
    htim1.Instance->CR1 |= TIM_CR1_CEN;
    BSP_Critical_Exit(lock);
    return BSP_DSHOT_OK;
}

BSP_DShotStatus BSP_DShot_Submit(const uint16_t code[2], uint8_t enabled_mask)
{
    uint16_t packet[2];

    if ((code == NULL) || (enabled_mask > 3U)) { return BSP_DSHOT_INVALID; }
    if (enabled_mask == 0U) { return BSP_DShot_Disable(3U); }
    for (uint8_t i = 0U; i < 2U; ++i) {
        const uint16_t throttle =
            ((enabled_mask & (uint8_t)(1U << i)) != 0U) ? code[i] : DRV_DSHOT_STOP;
        if (DRV_DShotTelem_EncodeRequest(throttle, 0U, &packet[i]) !=
            DRV_DSHOT_TELEM_OK) {
            return BSP_DSHOT_INVALID;
        }
    }
    return bb_submit_packets(packet, code, enabled_mask);
}

BSP_DShotStatus BSP_DShot_SubmitCommand(uint16_t command, uint8_t enabled_mask)
{
    uint16_t packet[2];
    uint16_t code[2];

    if ((enabled_mask == 0U) || (enabled_mask > 3U)) { return BSP_DSHOT_INVALID; }
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (DRV_DShotTelem_EncodeCommand(command, &packet[i]) !=
            DRV_DSHOT_TELEM_OK) {
            return BSP_DSHOT_INVALID;
        }
    }
    code[0] = command; code[1] = command;
    return bb_submit_packets(packet, code, enabled_mask);
}

BSP_DShotStatus BSP_DShot_Disable(uint8_t channel_mask)
{
    uint32_t lock;

    if ((channel_mask == 0U) || (channel_mask > 3U)) { return BSP_DSHOT_INVALID; }
    lock = BSP_Critical_Enter();
    rx_active = 0U;
    (void)bb_abort_dma();
    bb_hold_idle();
    state.enabled_mask &= (uint8_t)~channel_mask;
    state.busy = 0U;
    state.cancelled++;
    generation++;
    BSP_Critical_Exit(lock);
    return BSP_DSHOT_OK;
}

void BSP_DShot_GetSnapshot(BSP_DShotSnapshot *out)
{
    if (out == NULL) { return; }
    uint32_t lock = BSP_Critical_Enter();
    *out = state;
    BSP_Critical_Exit(lock);
}

void BSP_DShotRx_GetSnapshot(BSP_DShotRxSnapshot *out)
{
    if (out == NULL) { return; }
    uint32_t lock = BSP_Critical_Enter();
    *out = rx_state;
    /*
     * bitbang 两路**同时**采样，不存在 TIMER 后端那种轮流带来的自检窗口问题，
     * 所以没有宽限期。报 0 而不是省略字段：上位机按字段在不在来区分"旧固件"
     * 与"本档没有"，省略会让它以为对面是旧固件。
     */
    out->detect_grace = 0U;
    BSP_Critical_Exit(lock);
}

#endif /* BSP_ESC_PROTOCOL_IS_BITBANG */
