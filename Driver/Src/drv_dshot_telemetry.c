/*
 * 双向 DShot300 回传帧的纯解码实现。
 *
 * GCR 五位组表与"先异或右移一位再查表"的顺序来自 DShot 双向遥测的公开定义
 * （Betaflight/AM32 同一套）。这里刻意把它写成**查表 + 显式失败码**而不是
 * 算术推导：五位组只有 16 个合法值，剩下 16 个必须当错误报出来。
 * 把非法五位组默默映射成 0，会让一个坏电平变成一个看着正常的转速。
 */
#include "drv_dshot_telemetry.h"

/* nibble -> 五位组。索引即 4 bit 数值。 */
static const uint8_t dshot_telem_gcr_encode[16] = {
    0x19U, 0x1BU, 0x12U, 0x13U, 0x1DU, 0x15U, 0x16U, 0x17U,
    0x1AU, 0x09U, 0x0AU, 0x0BU, 0x1EU, 0x0DU, 0x0EU, 0x0FU
};

/* 五位组 -> nibble；0xFF = 非法。由上表在首次使用时反推，避免两份表写不一致。 */
static uint8_t dshot_telem_gcr_decode[32];
static uint8_t dshot_telem_tables_ready;

static void dshot_telem_build_tables(void)
{
    if (dshot_telem_tables_ready != 0U) {
        return;
    }
    for (uint32_t i = 0U; i < 32U; ++i) {
        dshot_telem_gcr_decode[i] = 0xFFU;
    }
    for (uint32_t nibble = 0U; nibble < 16U; ++nibble) {
        dshot_telem_gcr_decode[dshot_telem_gcr_encode[nibble]] = (uint8_t)nibble;
    }
    dshot_telem_tables_ready = 1U;
}

/*
 * 双向 DShot 的 4 bit 校验：单向公式（三个 nibble 异或折叠）取反。
 * `value12` 是 value<<1 | telemetry，请求帧与命令帧共用同一条公式，只是
 * value 字段的含义不同；提取成一份实现，避免本文件里出现第二份手抄。
 */
static uint16_t dshot_telem_crc4(uint16_t value12)
{
    /* 双向 DShot 的判别特征：校验取反。单向是同一个式子不取反。 */
    return (uint16_t)((~(value12 ^ (value12 >> 4U) ^ (value12 >> 8U))) & 0x0FU);
}

DRV_DShotTelemStatus DRV_DShotTelem_EncodeRequest(uint16_t throttle,
                                                  uint8_t telemetry_request,
                                                  uint16_t *out)
{
    uint16_t value12;

    if ((out == NULL) || (throttle > 2047U) ||
        ((throttle != 0U) && (throttle < 48U)) ||
        (telemetry_request > 1U)) {
        return DRV_DSHOT_TELEM_INVALID;
    }
    value12 = (uint16_t)((throttle << 1U) | (uint16_t)telemetry_request);
    *out = (uint16_t)((value12 << 4U) | dshot_telem_crc4(value12));
    return DRV_DSHOT_TELEM_OK;
}

DRV_DShotTelemStatus DRV_DShotTelem_EncodeCommand(uint16_t command, uint16_t *out)
{
    uint16_t value12;

    /* 边界与 drv_dshot.h 的 DRV_DSHOT_CMD_MIN/MAX 一致，但两个模块不互相
     * 包含头文件（见 test_module_is_hardware_independent 的 include 白名单），
     * 只能各自用字面量重写一遍；改范围要同时改两处。 */
    if ((out == NULL) || (command < 1U) || (command > 47U)) {
        return DRV_DSHOT_TELEM_INVALID;
    }
    /* 与单向命令帧同一件事：telemetry 位固定为 1，电调据此把该帧当命令
     * 而不是极低油门来解释；这里只是校验取反，载荷公式不变。 */
    value12 = (uint16_t)((command << 1U) | 1U);
    *out = (uint16_t)((value12 << 4U) | dshot_telem_crc4(value12));
    return DRV_DSHOT_TELEM_OK;
}

DRV_DShotTelemStatus DRV_DShotTelem_TicksPerBitQ8(uint32_t timer_clock_hz,
                                                  uint32_t bit_rate_hz,
                                                  uint32_t *out_q8)
{
    uint64_t telem_rate;
    uint64_t q8;

    if ((out_q8 == NULL) || (timer_clock_hz == 0U) || (bit_rate_hz == 0U)) {
        return DRV_DSHOT_TELEM_INVALID;
    }
    telem_rate = ((uint64_t)bit_rate_hz * DRV_DSHOT_TELEM_RATE_NUM) /
                 DRV_DSHOT_TELEM_RATE_DEN;
    if (telem_rate == 0U) {
        return DRV_DSHOT_TELEM_INVALID;
    }
    q8 = (((uint64_t)timer_clock_hz << 8U) + (telem_rate / 2U)) / telem_rate;
    if ((q8 == 0U) || (q8 > 0xFFFFFFFFU)) {
        return DRV_DSHOT_TELEM_INVALID;
    }
    *out_q8 = (uint32_t)q8;
    return DRV_DSHOT_TELEM_OK;
}

DRV_DShotTelemStatus DRV_DShotTelem_BitsFromEdges(const uint16_t *edge_ticks,
                                                  size_t edge_count,
                                                  uint32_t ticks_per_bit_q8,
                                                  uint32_t *out_raw21)
{
    uint32_t value = 0U;
    uint32_t filled = 0U;
    uint32_t level = 0U; /* 第一个边沿是下降沿，之后线上是低电平 */

    if ((edge_ticks == NULL) || (out_raw21 == NULL) || (edge_count < 2U) ||
        (edge_count > DRV_DSHOT_TELEM_MAX_EDGES) || (ticks_per_bit_q8 == 0U)) {
        return DRV_DSHOT_TELEM_INVALID;
    }

    for (size_t i = 1U; (i < edge_count) &&
                        (filled < DRV_DSHOT_TELEM_FRAME_BITS); ++i) {
        /* 16 位捕获值允许回绕，无符号差值天然处理。 */
        const uint32_t delta =
            (uint32_t)((uint16_t)(edge_ticks[i] - edge_ticks[i - 1U]));
        uint32_t span =
            (((delta << 8U) + (ticks_per_bit_q8 / 2U)) / ticks_per_bit_q8);

        if (span == 0U) {
            /* 比一个位还短的翻转只可能是毛刺，整帧作废而不是猜。 */
            return DRV_DSHOT_TELEM_BAD_EDGES;
        }
        /*
         * 越过帧尾的部分截断而不是判错：帧最后一位若是低电平，线路必然还有一次
         * 回空闲高电平的上升沿，那是**合法**的，按错处理会把好帧全丢掉。
         * 真正的乱码由后面的 GCR 反查表和 4 bit 校验挡，不靠这里的位数猜。
         */
        if (span > (DRV_DSHOT_TELEM_FRAME_BITS - filled)) {
            span = DRV_DSHOT_TELEM_FRAME_BITS - filled;
        }
        for (uint32_t bit = 0U; bit < span; ++bit) {
            value = (value << 1U) | level;
        }
        filled += span;
        level ^= 1U;
    }

    /* 最后一次翻转之后线路保持到帧尾（回空闲高电平），按该电平补齐。 */
    while (filled < DRV_DSHOT_TELEM_FRAME_BITS) {
        value = (value << 1U) | level;
        ++filled;
    }

    *out_raw21 = value & 0x1FFFFFU;
    return DRV_DSHOT_TELEM_OK;
}

DRV_DShotTelemStatus DRV_DShotTelem_DecodeRaw(uint32_t raw21, uint16_t *out_payload12)
{
    uint32_t gcr;
    uint32_t decoded = 0U;
    uint32_t csum;

    if (out_payload12 == NULL) {
        return DRV_DSHOT_TELEM_INVALID;
    }
    dshot_telem_build_tables();

    /* 线上是"跳变即 1"的编码，异或右移一位还原成 20 bit GCR 载荷。 */
    gcr = (raw21 ^ (raw21 >> 1U)) & 0x0FFFFFU;

    for (uint32_t quintet = 0U; quintet < 4U; ++quintet) {
        const uint8_t nibble =
            dshot_telem_gcr_decode[(gcr >> (5U * quintet)) & 0x1FU];
        if (nibble == 0xFFU) {
            return DRV_DSHOT_TELEM_BAD_GCR;
        }
        decoded |= (uint32_t)nibble << (4U * quintet);
    }

    /* 低 4 bit 是校验：全 16 bit 折叠异或必须得到 0xF。 */
    csum = decoded;
    csum ^= (csum >> 8U);
    csum ^= (csum >> 4U);
    if ((csum & 0x0FU) != 0x0FU) {
        return DRV_DSHOT_TELEM_BAD_CRC;
    }

    *out_payload12 = (uint16_t)((decoded >> 4U) & 0x0FFFU);
    return DRV_DSHOT_TELEM_OK;
}

DRV_DShotTelemStatus DRV_DShotTelem_ValueFromPayload(uint16_t payload12,
                                                     DRV_DShotTelemValue *out)
{
    uint32_t exponent;
    uint32_t mantissa;
    uint32_t period_us;

    if ((out == NULL) || (payload12 > 0x0FFFU)) {
        return DRV_DSHOT_TELEM_INVALID;
    }
    out->period_us = 0U;
    out->erpm = 0U;
    out->not_spinning = 0U;
    out->kind = (uint8_t)DRV_DSHOT_TELEM_VALUE_ERPM;
    out->edt_type = 0U;
    out->edt_value = 0U;

    if (payload12 == DRV_DSHOT_TELEM_NOT_SPINNING) {
        out->not_spinning = 1U;
        return DRV_DSHOT_TELEM_OK;
    }

    /*
     * EDT reuses the otherwise forbidden `ppp0 mmmmmmmm` period patterns.
     * Type zero remains eRPM; nonzero even nibbles are typed EDT.  Classification
     * happens before period arithmetic so current=0 A can never masquerade as
     * the special zero-period/not-spinning case.
     *
     * Source: Betaflight DShot API / extended-dshot-telemetry specification.
     * This decoder is passive: it does not send command 13 or persist ESC state.
     */
    if ((((uint32_t)payload12 & 0x0100U) == 0U) &&
        (((uint32_t)payload12 & 0x0E00U) != 0U)) {
        out->kind = (uint8_t)DRV_DSHOT_TELEM_VALUE_EDT;
        out->edt_type = (uint8_t)(((uint32_t)payload12 >> 8U) & 0x0FU);
        out->edt_value = (uint8_t)((uint32_t)payload12 & 0xFFU);
        return DRV_DSHOT_TELEM_OK;
    }

    exponent = ((uint32_t)payload12 >> 9U) & 0x07U;
    mantissa = (uint32_t)payload12 & 0x1FFU;
    period_us = mantissa << exponent;
    if (period_us == 0U) {
        /* 周期为零没有物理含义，按未旋转处理而不是除零。 */
        out->not_spinning = 1U;
        return DRV_DSHOT_TELEM_OK;
    }
    if (period_us > 0xFFFFU) {
        period_us = 0xFFFFU;
    }
    out->period_us = (uint16_t)period_us;
    out->erpm = (uint32_t)(60000000U / period_us);
    return DRV_DSHOT_TELEM_OK;
}

DRV_DShotTelemStatus DRV_DShotTelem_Decode(const uint16_t *edge_ticks,
                                           size_t edge_count,
                                           uint32_t ticks_per_bit_q8,
                                           DRV_DShotTelemValue *out)
{
    uint32_t raw21 = 0U;
    uint16_t payload12 = 0U;
    DRV_DShotTelemStatus status;

    if (out == NULL) {
        return DRV_DSHOT_TELEM_INVALID;
    }
    status = DRV_DShotTelem_BitsFromEdges(edge_ticks, edge_count,
                                          ticks_per_bit_q8, &raw21);
    if (status != DRV_DSHOT_TELEM_OK) {
        return status;
    }
    status = DRV_DShotTelem_DecodeRaw(raw21, &payload12);
    if (status != DRV_DSHOT_TELEM_OK) {
        return status;
    }
    return DRV_DShotTelem_ValueFromPayload(payload12, out);
}

uint32_t DRV_DShotTelem_MechanicalRpm(uint32_t erpm, uint8_t pole_pairs)
{
    if (pole_pairs == 0U) {
        return 0U;
    }
    return erpm / (uint32_t)pole_pairs;
}
