/*
 * Copyright (C) 2024 PX4 Development Team. All rights reserved.
 * Author: Igor Misic <igy1000mb@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Adapted from PX4 v1.16.0, commit 6ea3539157ca358c70a515878b77077af7d4611d,
 * platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c:
 * dshot_motor_data_set() packet, XOR checksum and MSB-first interleaving.
 * Local changes: pure caller-owned API, two channels, reject commands/telemetry,
 * exact 3/8 and 6/8 timer duty, two explicit low tail slots, PWM command mapping.
 * Hardware burst startup and DMA remain the responsibility of the BSP.
 * Frozen source and provenance: tests/fixtures/dshot_px4/README.md.
 */
#include "drv_dshot.h"

#include <string.h>

DRV_DShotStatus DRV_DShot_MakeTiming(uint32_t timer_clock_hz,
                                    DRV_DShotTiming *out)
{
    DRV_DShotTiming timing;
    uint32_t ticks;

    if ((out == NULL) || (timer_clock_hz == 0U) ||
        ((timer_clock_hz % DRV_DSHOT_BIT_RATE) != 0U)) {
        return DRV_DSHOT_INVALID;
    }
    ticks = timer_clock_hz / DRV_DSHOT_BIT_RATE;
    if ((ticks < 8U) || (ticks > 65536U) || ((ticks % 8U) != 0U)) {
        return DRV_DSHOT_INVALID;
    }
    timing.timer_clock_hz = timer_clock_hz;
    timing.prescaler = 0U;
    timing.auto_reload = ticks - 1U;
    timing.zero_ticks = (ticks / 8U) * 3U;
    timing.one_ticks = (ticks / 8U) * 6U;
    *out = timing;
    return DRV_DSHOT_OK;
}

DRV_DShotStatus DRV_DShot_FromPulseUs(uint16_t pulse_us, uint16_t *out)
{
    if ((out == NULL) || (pulse_us < DRV_DSHOT_EQUIV_MIN_US) ||
        (pulse_us > DRV_DSHOT_EQUIV_MAX_US)) {
        return DRV_DSHOT_INVALID;
    }
    if (pulse_us == DRV_DSHOT_EQUIV_MIN_US) {
        *out = DRV_DSHOT_STOP;
    } else {
        const uint32_t offset = pulse_us - DRV_DSHOT_EQUIV_MIN_US - 1U;
        const uint32_t span = DRV_DSHOT_EQUIV_MAX_US - DRV_DSHOT_EQUIV_MIN_US - 1U;
        *out = (uint16_t)(DRV_DSHOT_THROTTLE_MIN +
                         offset * (DRV_DSHOT_THROTTLE_MAX - DRV_DSHOT_THROTTLE_MIN) / span);
    }
    return DRV_DSHOT_OK;
}

/*
 * 单向 DShot 的 4 bit 校验：对 packet 高 12 bit（value<<1 | telemetry）
 * 三个 nibble 做异或折叠。throttle 帧与命令帧共用同一条公式，只是 value 字段
 * 的含义不同；提取成一份实现，避免同一算法在本文件里出现第二份手抄。
 * `packet` 传入时校验 nibble 必须是 0（调用方先移位拼好高 12 bit 再调用）。
 */
static uint16_t dshot_crc4(uint16_t packet)
{
    const uint16_t x = (uint16_t)(packet >> 4U);

    return (uint16_t)((x ^ (x >> 4U) ^ (x >> 8U)) & 0x0FU);
}

DRV_DShotStatus DRV_DShot_Encode(uint16_t throttle, uint16_t *out)
{
    uint16_t packet;

    if ((out == NULL) || (throttle > DRV_DSHOT_THROTTLE_MAX) ||
        ((throttle != DRV_DSHOT_STOP) && (throttle < DRV_DSHOT_THROTTLE_MIN))) {
        return DRV_DSHOT_INVALID;
    }
    /* PX4 dshot_motor_data_set(): telemetry bit stays zero for this port. */
    packet = (uint16_t)(throttle << 5U);
    packet |= dshot_crc4(packet);
    *out = packet;
    return DRV_DSHOT_OK;
}

DRV_DShotStatus DRV_DShot_EncodeCommand(uint16_t command, uint16_t *out)
{
    uint16_t value12;
    uint16_t packet;

    if ((out == NULL) || (command < DRV_DSHOT_CMD_MIN) || (command > DRV_DSHOT_CMD_MAX)) {
        return DRV_DSHOT_INVALID;
    }
    /* Betaflight dshotCommandWrite() 对命令帧固定置 requestTelemetry=1；
     * 这一位写 0，电调会把命令值当成一个极低的油门而不是命令来执行。 */
    value12 = (uint16_t)((command << 1U) | 1U);
    packet = (uint16_t)(value12 << 4U);
    packet |= dshot_crc4(packet);
    *out = packet;
    return DRV_DSHOT_OK;
}

static DRV_DShotStatus dshot_check_timing(const DRV_DShotTiming *timing,
                                          const uint32_t *out,
                                          size_t capacity_words)
{
    DRV_DShotTiming expected;

    if ((timing == NULL) || (out == NULL) ||
        (capacity_words < DRV_DSHOT_BURST_WORDS) ||
        (DRV_DShot_MakeTiming(timing->timer_clock_hz, &expected) != DRV_DSHOT_OK)) {
        return DRV_DSHOT_INVALID;
    }
    if ((timing->prescaler != expected.prescaler) ||
        (timing->auto_reload != expected.auto_reload) ||
        (timing->zero_ticks != expected.zero_ticks) ||
        (timing->one_ticks != expected.one_ticks)) {
        return DRV_DSHOT_INVALID;
    }
    return DRV_DSHOT_OK;
}

DRV_DShotStatus DRV_DShot_BuildBurstFromPackets(
    const uint16_t packet_in[DRV_DSHOT_CHANNELS],
    const DRV_DShotTiming *timing,
    uint32_t *out, size_t capacity_words)
{
    uint16_t packet[DRV_DSHOT_CHANNELS];

    if (packet_in == NULL) {
        return DRV_DSHOT_INVALID;
    }
    if (dshot_check_timing(timing, out, capacity_words) != DRV_DSHOT_OK) {
        return DRV_DSHOT_INVALID;
    }
    for (size_t channel = 0U; channel < DRV_DSHOT_CHANNELS; ++channel) {
        packet[channel] = packet_in[channel];
    }
    /* All validation is complete before modifying the caller's buffer. */
    for (size_t bit = 0U; bit < DRV_DSHOT_BITS; ++bit) {
        for (size_t channel = 0U; channel < DRV_DSHOT_CHANNELS; ++channel) {
            out[bit * DRV_DSHOT_CHANNELS + channel] =
                ((packet[channel] & 0x8000U) != 0U) ? timing->one_ticks : timing->zero_ticks;
            packet[channel] = (uint16_t)(packet[channel] << 1U);
        }
    }
    memset(&out[DRV_DSHOT_BITS * DRV_DSHOT_CHANNELS], 0,
           DRV_DSHOT_TAIL_SLOTS * DRV_DSHOT_CHANNELS * sizeof(*out));
    return DRV_DSHOT_OK;
}

DRV_DShotStatus DRV_DShot_BuildBurst(const uint16_t throttle[DRV_DSHOT_CHANNELS],
                                    const DRV_DShotTiming *timing,
                                    uint32_t *out, size_t capacity_words)
{
    uint16_t packet[DRV_DSHOT_CHANNELS];

    if (throttle == NULL) {
        return DRV_DSHOT_INVALID;
    }
    if (dshot_check_timing(timing, out, capacity_words) != DRV_DSHOT_OK) {
        return DRV_DSHOT_INVALID;
    }
    for (size_t channel = 0U; channel < DRV_DSHOT_CHANNELS; ++channel) {
        if (DRV_DShot_Encode(throttle[channel], &packet[channel]) != DRV_DSHOT_OK) {
            return DRV_DSHOT_INVALID;
        }
    }
    return DRV_DShot_BuildBurstFromPackets(packet, timing, out, capacity_words);
}
