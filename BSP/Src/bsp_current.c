#include "bsp_current.h"
#include "adc.h"
#include <stddef.h>

static uint8_t current_initialized;
/* Distinguishes "never initialized" from "latched off after a fault": the second
 * one never recovers on its own, and it silently blocks arming forever. Reporting
 * both as NOT_READY would tell the operator to wait for something that never comes. */
static uint8_t current_latched_fault;
/* 上一对没走完整个序列。rank 顺序是"哪个数是电流、哪个是电压"的唯一依据，
 * 恢复到已知位置之前不能再取数，否则两路可能整体错位一格。 */
static uint8_t sequencer_unproven;
/* PC1 正处在数字输入自检模式。见 bsp_current.h 里 ProbeMode 那段。 */
static uint8_t probe_active;
static BSP_VoltageSample voltage_sample={.status=BSP_CURRENT_NOT_READY};

/*
 * 字段校验 + 内部校准。这里同时是**错误路径的复位手段**：
 * HAL_ADCEx_Calibration_Start() 的第一步就是 ADC_Disable()，走一遍等于把 ADC
 * 关掉再打开，状态机整体复位，之后必然从 rank1 开始。
 *
 * ──────────── 2026-09-21：试过改普通扫描，无效，已回退 ────────────
 *
 * 起因：出厂 PX4 固件在同一引脚读到 ch11=1226 LSB，本模块只读到约 6 LSB，于是
 * 怀疑是采样方式。试过把 DiscontinuousConvMode 关掉、改成一次 Start 转完整个
 * 序列（.ioc 与 Core/Src/adc.c 同步改）：**ch11 读数一点没变**，还引入了约 5%
 * 的采样失败，已整体回退。PCSEL / SQR1 / SMPR / DIFSEL / OFR / CCR 也逐位核过，
 * 两个通道完全对称。结论：问题不在采样方式，别再往这个方向改。
 *
 * 更要紧的是那个「差 26 倍」的结论**是拿错尺子量出来的**：作者随后用万用表直接
 * 量电调 Curr 焊盘，同工况只有 4~8 mV(=79~159 LSB)；1226 LSB 折合 61.7 mV，表上
 * 不可能看不见，所以 PX4 那个基准本身不可信。AM32 55A 标称 12.75 mV/A，0.2 A 只
 * 对应 2.5 mV = 50 LSB —— 这条链路在小电流下本来就贴着噪声底，别把它当故障查。
 * 证据见 data/calibration/2026-09-21/px4-baseline-adc-current.md。
 *
 * 间断模式本身的理由仍然成立：一对采样中途失败时 sequencer 停在 rank2，而
 * "Stop 会不会把子组指针倒回 rank1"本仓库拿不出依据，所以走 adc_prepare()
 * 复位而不是赌。不用 DMA/中断的理由也没变（50 Hz 不值得引入 H7 cache 问题）。
 */
static BSP_CurrentStatus adc_prepare(void)
{
    if (hadc1.Instance != ADC1 || hadc1.Init.Resolution != ADC_RESOLUTION_16B ||
        hadc1.Init.NbrOfConversion != 2U || hadc1.Init.ContinuousConvMode != DISABLE ||
        hadc1.Init.ScanConvMode != ADC_SCAN_ENABLE || hadc1.Init.DiscontinuousConvMode != ENABLE ||
        hadc1.Init.NbrOfDiscConversion != 1U || hadc1.Init.EOCSelection != ADC_EOC_SINGLE_CONV ||
        hadc1.Init.ConversionDataManagement != ADC_CONVERSIONDATA_DR ||
        hadc1.Init.LeftBitShift != ADC_LEFTBITSHIFT_NONE || hadc1.Init.OversamplingMode != DISABLE) {
        return BSP_CURRENT_ERROR;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
                                   ADC_SINGLE_ENDED) != HAL_OK) {
        return BSP_CURRENT_ERROR;
    }
    return BSP_CURRENT_OK;
}

BSP_CurrentStatus BSP_Current_Init(void)
{
    current_initialized = 0U;
    current_latched_fault = 0U;
    sequencer_unproven = 0U;
    probe_active = 0U;
    voltage_sample = (BSP_VoltageSample){.status=BSP_CURRENT_ERROR};
    if (adc_prepare() != BSP_CURRENT_OK) {
        return BSP_CURRENT_ERROR;
    }
    current_initialized = 1U;
    voltage_sample.status = BSP_CURRENT_NOT_READY;
    return BSP_CURRENT_OK;
}

/* 等一次转换并取值；调用方负责已经 Start。 */
static BSP_CurrentStatus poll_one(uint32_t *value)
{
    HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&hadc1, 1U);
    uint32_t error = HAL_ADC_GetError(&hadc1);
    if (error != HAL_ADC_ERROR_NONE) { return BSP_CURRENT_ERROR; }
    if (status == HAL_TIMEOUT) { return BSP_CURRENT_TIMEOUT; }
    if (status != HAL_OK) { return BSP_CURRENT_ERROR; }
    *value = HAL_ADC_GetValue(&hadc1);
    return *value <= 65535U ? BSP_CURRENT_OK : BSP_CURRENT_ERROR;
}

/* 间断模式下每次触发只转一个 rank，所以每个 rank 各 Start 一次。 */
static BSP_CurrentStatus read_rank(uint32_t *value)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK) { return BSP_CURRENT_ERROR; }
    return poll_one(value);
}

BSP_CurrentStatus BSP_Current_SeqBurst(uint32_t *out, uint32_t count,
                                       uint32_t *out_done)
{
    uint32_t index;

    if ((out == NULL) || (out_done == NULL) || (count == 0U) ||
        (count > BSP_CURRENT_SEQ_BURST_MAX)) {
        return BSP_CURRENT_ERROR;
    }
    *out_done = 0U;
    if (probe_active) { return BSP_CURRENT_NOT_READY; }
    if (!current_initialized) {
        return current_latched_fault ? BSP_CURRENT_ERROR : BSP_CURRENT_NOT_READY;
    }

    /*
     * 走和正常采样**完全相同**的起点：如果 sequencer 位置不明就先 adc_prepare()
     * 复位。诊断如果用一条与生产路径不同的起点，测出来的东西就不是生产路径的行为。
     */
    if (sequencer_unproven) {
        if (adc_prepare() != BSP_CURRENT_OK) {
            current_initialized = 0U; current_latched_fault = 1U;
            return BSP_CURRENT_ERROR;
        }
        sequencer_unproven = 0U;
    }

    for (index = 0U; index < count; ++index) {
        uint32_t value = 0U;
        BSP_CurrentStatus status = read_rank(&value);
        if (status != BSP_CURRENT_OK) {
            sequencer_unproven = 1U;
            (void)HAL_ADC_Stop(&hadc1);
            return status;
        }
        out[index] = value;
        *out_done = index + 1U;
    }

    /*
     * 读了 count 次而不是整数对，序列多半停在半路，所以明确标记位置不明，
     * 让下一次正常采样先复位。诊断命令不能给生产路径留下一个错位的 ADC。
     */
    sequencer_unproven = 1U;
    (void)HAL_ADC_Stop(&hadc1);
    return BSP_CURRENT_OK;
}

BSP_CurrentStatus BSP_Current_SetPinProbeMode(BSP_CurrentPinProbeMode mode)
{
    GPIO_InitTypeDef init = {0};

    /* 只动 PC1。PC0（电压）正在正常工作，没有理由跟着一起被重配。 */
    init.Pin   = GPIO_PIN_1;
    init.Speed = GPIO_SPEED_FREQ_LOW;

    switch (mode) {
    case BSP_CURRENT_PIN_PROBE_PULLUP:
        init.Mode = GPIO_MODE_INPUT;
        init.Pull = GPIO_PULLUP;
        break;
    case BSP_CURRENT_PIN_PROBE_PULLDOWN:
        init.Mode = GPIO_MODE_INPUT;
        init.Pull = GPIO_PULLDOWN;
        break;
    case BSP_CURRENT_PIN_PROBE_RESTORE:
        init.Mode = GPIO_MODE_ANALOG;
        init.Pull = GPIO_NOPULL;
        break;
    default:
        return BSP_CURRENT_ERROR;
    }

    HAL_GPIO_Init(GPIOC, &init);

    if (mode == BSP_CURRENT_PIN_PROBE_RESTORE) {
        probe_active = 0U;
        /*
         * 走完整的 Init 而不是只改回引脚模式：自检期间 messageTask 仍在按 20 ms
         * 敲 BSP_Current_Read()，那些调用虽然被 probe_active 挡在门外，但 ADC 自己
         * 的状态机停在哪儿本仓库拿不出依据。用已有且已测的复位路径回到确定状态，
         * 不赌。
         */
        return BSP_Current_Init();
    }

    probe_active = 1U;
    return BSP_CURRENT_OK;
}

uint8_t BSP_Current_ReadPinLevel(void)
{
    if (!probe_active) { return 0U; }
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET) ? 1U : 0U;
}

BSP_CurrentStatus BSP_Current_Read(uint32_t *raw)
{
    if (raw == NULL) { return BSP_CURRENT_ERROR; }
    /*
     * 自检期间 PC1 不是模拟脚。继续转换会产出一个没有物理意义的数，而它会被
     * 上层当成"电流读数"存进快照、喂进遥测——那是在编数据，比缺一拍危险。
     */
    if (probe_active) { return BSP_CURRENT_NOT_READY; }
    if (!current_initialized) {
        return current_latched_fault ? BSP_CURRENT_ERROR : BSP_CURRENT_NOT_READY;
    }
    if (sequencer_unproven) {
        /* 先恢复到已知位置再取数；恢复失败按停止失败同样处理：锁存不可用。 */
        if (adc_prepare()!=BSP_CURRENT_OK) {
            current_initialized=0U;current_latched_fault=1U;
            voltage_sample.sequence++;voltage_sample.status=BSP_CURRENT_ERROR;
            return BSP_CURRENT_ERROR;
        }
        sequencer_unproven=0U;
    }
    uint32_t current_raw=0U, voltage_raw=0U;
    /*
     * 普通扫描：**一次 Start 转完整个序列**，rank1(ch11 电流) 与 rank2(ch10 电压)
     * 在同一次转换序列里按顺序出来，中间不触发、不禁用 ADC。
     *
     * 这样防错位比原来的间断模式更强：序列一旦开始就跑完两个 rank，根本不存在
     * "停在半路"的状态，所以"Stop 会不会把子组指针倒回 rank1"这个赌局从前提上
     * 消失了。中途失败仍然标记位置不明，让下一拍先 adc_prepare() 复位——那条
     * 保险留着不花钱。
     */
    BSP_CurrentStatus status=read_rank(&current_raw);
    if (status==BSP_CURRENT_OK) { status=read_rank(&voltage_raw); }
    if (status!=BSP_CURRENT_OK) { sequencer_unproven=1U; }
    /*
     * 序列已经自然结束（NbrOfConversion=2，两次 EOC 之后 EOS 置位），这里 Stop
     * 只是把 ADSTART 清干净、保持与原实现相同的收尾语义。失败仍按锁存处理。
     */
    if (HAL_ADC_Stop(&hadc1)!=HAL_OK) {
        current_initialized=0U;current_latched_fault=1U;status=BSP_CURRENT_ERROR;
    }
    voltage_sample.sequence++;
    voltage_sample.status=status;
    if (status!=BSP_CURRENT_OK) { return status; }
    voltage_sample.raw=voltage_raw;
    *raw=current_raw;
    return BSP_CURRENT_OK;
}

void BSP_Current_GetVoltageSample(BSP_VoltageSample *out)
{
    if (out!=NULL) { *out=voltage_sample; }
}
