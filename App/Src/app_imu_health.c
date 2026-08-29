#include "app_imu_health.h"

#include <string.h>

/*
 * 全部整数运算：本模块在 1kHz 采样热路径上被调用，不引入浮点。
 */

static volatile APP_ImuHealthLevel imu_health_level;
static volatile uint8_t  imu_health_fault_active;
static volatile uint8_t  imu_health_fault_ever;
static volatile uint16_t imu_health_sample_rate_hz;
static volatile uint16_t imu_health_irq_rate_hz;
static volatile uint32_t imu_health_irq_samples;
static volatile uint32_t imu_health_poll_samples;
static volatile uint32_t imu_health_degraded_windows;
static volatile uint32_t imu_health_failed_windows;

/* 当前窗口内的计数 */
static volatile uint32_t imu_health_window_samples;
static volatile uint32_t imu_health_window_irq;
static uint32_t imu_health_window_start_ms;
static uint8_t  imu_health_window_started;
static uint32_t imu_health_consecutive_fail;
static uint32_t imu_health_consecutive_normal;

void APP_ImuHealth_Init(void)
{
    imu_health_level = APP_IMU_HEALTH_NORMAL;
    imu_health_fault_active = 0U;
    imu_health_fault_ever = 0U;
    imu_health_sample_rate_hz = 0U;
    imu_health_irq_rate_hz = 0U;
    imu_health_irq_samples = 0U;
    imu_health_poll_samples = 0U;
    imu_health_degraded_windows = 0U;
    imu_health_failed_windows = 0U;
    imu_health_window_samples = 0U;
    imu_health_window_irq = 0U;
    imu_health_window_start_ms = 0U;
    imu_health_window_started = 0U;
    imu_health_consecutive_fail = 0U;
    imu_health_consecutive_normal = 0U;
}

void APP_ImuHealth_NoteSample(uint8_t from_irq)
{
    imu_health_window_samples++;
    if (from_irq != 0U) {
        imu_health_window_irq++;
        imu_health_irq_samples++;
    } else {
        imu_health_poll_samples++;
    }
}

void APP_ImuHealth_Update(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    uint32_t samples;
    uint32_t irq;
    uint32_t rate_hz;
    uint32_t irq_rate_hz;
    uint32_t poll_pct;
    APP_ImuHealthLevel level;

    if (imu_health_window_started == 0U) {
        imu_health_window_start_ms = now_ms;
        imu_health_window_started = 1U;
        return;
    }

    elapsed_ms = now_ms - imu_health_window_start_ms;
    if (elapsed_ms < APP_IMU_HEALTH_WINDOW_MS) {
        return;
    }

    samples = imu_health_window_samples;
    irq = imu_health_window_irq;
    imu_health_window_samples = 0U;
    imu_health_window_irq = 0U;
    imu_health_window_start_ms = now_ms;

    rate_hz = (samples * 1000U) / elapsed_ms;
    irq_rate_hz = (irq * 1000U) / elapsed_ms;
    imu_health_sample_rate_hz =
        (rate_hz > 65535U) ? 65535U : (uint16_t)rate_hz;
    imu_health_irq_rate_hz =
        (irq_rate_hz > 65535U) ? 65535U : (uint16_t)irq_rate_hz;

    /*
     * 轮询占比：正常情况下应当为 0。这里刻意用"轮询帧数"而不是"是否拿到数据"
     * 作为分子——中断漏了就是漏了，哪怕兜底成功也不算健康。
     */
    poll_pct = (samples == 0U) ? 100U : (((samples - irq) * 100U) / samples);

    if (rate_hz < APP_IMU_HEALTH_RATE_FAIL_HZ) {
        level = APP_IMU_HEALTH_FAILED;
    } else if ((rate_hz < APP_IMU_HEALTH_RATE_NORMAL_HZ) ||
               (poll_pct > APP_IMU_HEALTH_POLL_WARN_PCT)) {
        level = APP_IMU_HEALTH_DEGRADED;
    } else {
        level = APP_IMU_HEALTH_NORMAL;
    }
    imu_health_level = level;

    if (level == APP_IMU_HEALTH_FAILED) {
        imu_health_failed_windows++;
        imu_health_consecutive_normal = 0U;
        imu_health_consecutive_fail++;
        if (imu_health_consecutive_fail >= APP_IMU_HEALTH_FAIL_WINDOWS) {
            imu_health_fault_active = 1U;
            imu_health_fault_ever = 1U;
        }
        return;
    }

    imu_health_consecutive_fail = 0U;
    if (level == APP_IMU_HEALTH_DEGRADED) {
        imu_health_degraded_windows++;
        imu_health_consecutive_normal = 0U;
        return;
    }

    /*
     * 只有连续 2s 完全正常才解除锁存。fault_ever 不清除：那是本次上电确实
     * 发生过降级的证据，采集和报告需要据此作废可疑数据。
     */
    imu_health_consecutive_normal++;
    if (imu_health_consecutive_normal >= APP_IMU_HEALTH_RECOVER_WINDOWS) {
        imu_health_fault_active = 0U;
    }
}

APP_ImuHealthLevel APP_ImuHealth_GetLevel(void)
{
    return imu_health_level;
}

uint8_t APP_ImuHealth_IsArmBlocked(void)
{
    return (imu_health_fault_active != 0U) ? 1U : 0U;
}

void APP_ImuHealth_GetStatus(APP_ImuHealthStatus *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->level = imu_health_level;
    out->fault_active = imu_health_fault_active;
    out->fault_ever = imu_health_fault_ever;
    out->sample_rate_hz = imu_health_sample_rate_hz;
    out->irq_rate_hz = imu_health_irq_rate_hz;
    out->irq_samples = imu_health_irq_samples;
    out->poll_samples = imu_health_poll_samples;
    out->degraded_windows = imu_health_degraded_windows;
    out->failed_windows = imu_health_failed_windows;
}
