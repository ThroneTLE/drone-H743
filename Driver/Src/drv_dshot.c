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

DRV_DShotStatus DRV_DShot_Encode(uint16_t throttle, uint16_t *out)
{
    uint16_t checksum = 0U;
    uint16_t packet;
    uint16_t csum_data;

    if ((out == NULL) || (throttle > DRV_DSHOT_THROTTLE_MAX) ||
        ((throttle != DRV_DSHOT_STOP) && (throttle < DRV_DSHOT_THROTTLE_MIN))) {
        return DRV_DSHOT_INVALID;
    }
    /* PX4 dshot_motor_data_set(): telemetry bit stays zero for this port. */
    packet = (uint16_t)(throttle << 5U);
    csum_data = (uint16_t)(packet >> 4U);
    for (uint8_t i = 0U; i < 3U; ++i) {
        checksum ^= (uint16_t)(csum_data & 0x0FU);
        csum_data >>= 4U;
    }
    packet |= (uint16_t)(checksum & 0x0FU);
    *out = packet;
    return DRV_DSHOT_OK;
}

DRV_DShotStatus DRV_DShot_BuildBurst(const uint16_t throttle[DRV_DSHOT_CHANNELS],
                                    const DRV_DShotTiming *timing,
                                    uint32_t *out, size_t capacity_words)
{
    DRV_DShotTiming expected;
    uint16_t packet[DRV_DSHOT_CHANNELS];

    if ((throttle == NULL) || (timing == NULL) || (out == NULL) ||
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
    for (size_t channel = 0U; channel < DRV_DSHOT_CHANNELS; ++channel) {
        if (DRV_DShot_Encode(throttle[channel], &packet[channel]) != DRV_DSHOT_OK) {
            return DRV_DSHOT_INVALID;
        }
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
