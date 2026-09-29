/*
 * 双向 DShot300 bitbang 收发层的纯数据变换实现。
 *
 * 为什么存在这个文件：BSP 侧的 DMA burst + 输入捕获方案在本板双向档上
 * 实机不响应（帧值、极性、协议档位都核对过，电调仍然既不转也不回传），
 * 参考实现明确记载 burst DMA 与双向 DShot 不兼容。两路电调信号脚同在
 * GPIOE（PE9/PE11），改成一条 DMA 流定节拍写 BSRR / 读 IDR 天然能同时
 * 驱动两路——这就是本文件要做的事，具体引脚号仍然是 BSP 的事，这里
 * 只认调用方传入的掩码。
 */
#include "drv_dshot_bitbang.h"

/*
 * 一个数据位在线上的占空比固定是 3/8（bit=0）或 6/8（bit=1）——与
 * drv_dshot.h 里 DRV_DShotTiming.zero_ticks/one_ticks 说的是同一件事，
 * 只是定时器 PWM 模式靠占空比寄存器连续逼近，bitbang 只能靠固定数量的
 * 子槽离散逼近，8 恰好让 3/8、6/8 都是整数子槽数，不用四舍五入。
 */
static uint32_t dshot_bb_duty_slots(uint32_t bit_value)
{
    return (bit_value != 0U) ? 6U : 3U;
}

DRV_DShotStatus DRV_DShotBitbang_BuildFrame(const uint16_t packet[2],
                                            const DRV_DShotBitbangPins pins[2],
                                            uint8_t inverted,
                                            uint32_t *out, size_t capacity_words)
{
    uint32_t pulse_mask[2];
    uint32_t idle_mask[2];
    uint32_t idle_word;
    size_t word_index = 0U;

    if ((packet == NULL) || (pins == NULL) || (out == NULL) ||
        (capacity_words < DRV_DSHOT_BB_FRAME_WORDS)) {
        return DRV_DSHOT_INVALID;
    }

    /*
     * inverted!=0（双向档）：空闲高、数据位拉低，所以"数据脉冲"那部分用
     * reset_mask、"空闲"那部分用 set_mask；inverted==0（单向档）相反。
     * 两路各自独立取自己的两个掩码，互不影响——这也是两路能在同一个
     * BSRR 字里合并却互不干扰的原因：每路每个子槽只贡献自己那一个掩码。
     */
    for (uint32_t ch = 0U; ch < 2U; ++ch) {
        if (inverted != 0U) {
            pulse_mask[ch] = pins[ch].reset_mask;
            idle_mask[ch] = pins[ch].set_mask;
        } else {
            pulse_mask[ch] = pins[ch].set_mask;
            idle_mask[ch] = pins[ch].reset_mask;
        }
    }

    /* 所有校验在这之前已经做完，下面开始才碰调用方缓冲。 */
    for (uint32_t bit = 0U; bit < DRV_DSHOT_BB_FRAME_BITS; ++bit) {
        uint32_t duty[2];

        for (uint32_t ch = 0U; ch < 2U; ++ch) {
            /* MSB 先发：bit=0 对应 packet 的第 15 位。 */
            const uint32_t bit_value =
                ((uint32_t)packet[ch] >> (15U - bit)) & 1U;
            duty[ch] = dshot_bb_duty_slots(bit_value);
        }
        for (uint32_t slot = 0U; slot < DRV_DSHOT_BB_SLOTS_PER_BIT; ++slot) {
            uint32_t word = 0U;

            for (uint32_t ch = 0U; ch < 2U; ++ch) {
                word |= (slot < duty[ch]) ? pulse_mask[ch] : idle_mask[ch];
            }
            out[word_index] = word;
            ++word_index;
        }
    }

    /* 帧尾保持空闲电平，让最后一位真正走完，也给下一帧起始留出干净的
     * 空闲基线。两路的空闲掩码合并写进同一批字，和数据段用的是同一条
     * "各路各贡献自己掩码"的规则。 */
    idle_word = idle_mask[0] | idle_mask[1];
    for (uint32_t slot = 0U; slot < DRV_DSHOT_BB_TAIL_SLOTS; ++slot) {
        out[word_index] = idle_word;
        ++word_index;
    }

    return DRV_DSHOT_OK;
}

DRV_DShotTelemStatus DRV_DShotBitbang_BitsFromSamples(const uint32_t *samples,
                                                      size_t count,
                                                      uint32_t channel_mask,
                                                      uint32_t samples_per_bit_q8,
                                                      uint32_t *out_raw21)
{
    size_t start;
    size_t last_edge;
    uint32_t level;
    uint32_t value = 0U;
    uint32_t filled = 0U;
    uint32_t edges_found = 1U; /* 起始下降沿本身算第 1 个 */

    if ((samples == NULL) || (out_raw21 == NULL) ||
        (channel_mask == 0U) || (samples_per_bit_q8 == 0U) || (count < 2U)) {
        return DRV_DSHOT_TELEM_INVALID;
    }

    /*
     * 找第一个下降沿：空闲高 -> 起始位低。整段没有高->低的翻转，可能是
     * 全程高电平（电调没回话）、全程低电平（没等到真正的空闲期就开始采样）
     * 或者只有一次低->高的翻转——三种都不能猜，必须报 BAD_EDGES。
     */
    start = count; /* 哨兵：未找到 */
    {
        uint32_t prev = (uint32_t)((samples[0] & channel_mask) != 0U);

        for (size_t i = 1U; i < count; ++i) {
            const uint32_t cur = (uint32_t)((samples[i] & channel_mask) != 0U);

            if ((prev == 1U) && (cur == 0U)) {
                start = i;
                break;
            }
            prev = cur;
        }
    }
    if (start == count) {
        return DRV_DSHOT_TELEM_BAD_EDGES;
    }

    /*
     * 从起始下降沿开始逐样本找电平翻转，把"翻转间隔了多少个采样"换算成
     * "占多少个位宽"——与 DRV_DShotTelem_BitsFromEdges 同一套 delta/span
     * 逻辑，只是把定时器 tick 差值换成采样点下标差值。采样下标是数组位置，
     * 不是硬件计数器读数，没有回绕问题，直接做无符号减法即可。
     */
    level = 0U; /* 下降沿之后线路读到低，即起始位 */
    last_edge = start;
    for (size_t i = start + 1U;
         (i < count) && (filled < DRV_DSHOT_TELEM_FRAME_BITS); ++i) {
        const uint32_t cur = (uint32_t)((samples[i] & channel_mask) != 0U);
        uint32_t delta;
        uint32_t span;

        if (cur == level) {
            continue; /* 还在同一个电平，等下一次翻转再结算这一段游程 */
        }

        delta = (uint32_t)(i - last_edge);
        span = (uint32_t)((((uint64_t)delta << 8U) + (samples_per_bit_q8 / 2U)) /
                          samples_per_bit_q8);
        if (span == 0U) {
            /* 比一个位还短的翻转只可能是毛刺，整帧作废而不是猜。 */
            return DRV_DSHOT_TELEM_BAD_EDGES;
        }
        if (span > (DRV_DSHOT_TELEM_FRAME_BITS - filled)) {
            span = DRV_DSHOT_TELEM_FRAME_BITS - filled;
        }
        for (uint32_t bit = 0U; bit < span; ++bit) {
            value = (value << 1U) | level;
        }
        filled += span;
        last_edge = i;
        level = cur;
        ++edges_found;
    }

    if (edges_found < 2U) {
        /* 起始下降沿之后再没有任何翻转：样本没给出关于剩余 20 bit 的任何
         * 证据，绝不能把"没翻转"悄悄补成全部保持起始位那个电平。 */
        return DRV_DSHOT_TELEM_BAD_EDGES;
    }

    /* 最后一次翻转之后样本保持到缓冲区末尾，按该电平补齐——前提是调用方
     * 按惯例留够了尾部余量（同 DRV_DSHOT_TELEM_MAX_EDGES 的"留一倍余量给
     * 毛刺"惯例），不是本函数在猜。 */
    while (filled < DRV_DSHOT_TELEM_FRAME_BITS) {
        value = (value << 1U) | level;
        ++filled;
    }

    *out_raw21 = value & 0x1FFFFFU;
    return DRV_DSHOT_TELEM_OK;
}
