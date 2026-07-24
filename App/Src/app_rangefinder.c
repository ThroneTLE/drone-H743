#include "app_rangefinder.h"

#include "app_control.h"
#include "bsp_rangefinder.h"

#include "main.h"

#include <string.h>

#define APP_RANGEFINDER_TIMEOUT_MS       100U
#define APP_RANGEFINDER_MIN_DISTANCE_M   0.02f
#define APP_RANGEFINDER_MAX_DISTANCE_M  12.0f
#define APP_RANGEFINDER_MIN_STRENGTH     80U
#define APP_RANGEFINDER_HEIGHT_ALPHA     0.35f
#define APP_RANGEFINDER_VELOCITY_ALPHA   0.20f
#define APP_RANGEFINDER_MAX_VELOCITY_M_S 5.0f

typedef struct {
    APP_RangefinderStatus status;
    uint32_t processed_frames;
    float previous_height_m;
    uint32_t previous_sample_ms;
} APP_RangefinderContext;

static APP_RangefinderContext range_ctx;

static float rangefinder_clamp(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

void APP_Rangefinder_Init(void)
{
    memset(&range_ctx, 0, sizeof(range_ctx));
    range_ctx.status.initialized =
        (BSP_Rangefinder_Init() == DRV_TFMINI_OK) ? 1U : 0U;
}

void APP_Rangefinder_Step(void)
{
    BSP_RangefinderStatus bsp_status;
    uint32_t now = HAL_GetTick();

    BSP_Rangefinder_Service();
    BSP_Rangefinder_GetStatus(&bsp_status);
    range_ctx.status.initialized = bsp_status.initialized;
    range_ctx.status.bytes = bsp_status.bytes;
    range_ctx.status.frames = bsp_status.frames;
    range_ctx.status.checksum_errors = bsp_status.checksum_errors;
    range_ctx.status.frame_errors = bsp_status.frame_errors;
    range_ctx.status.rx_restarts = bsp_status.rx_restarts;
    range_ctx.status.dma_events = bsp_status.dma_events;
    range_ctx.status.uart_errors = bsp_status.uart_errors;
    range_ctx.status.last_uart_error = bsp_status.last_uart_error;

    if (bsp_status.frames != range_ctx.processed_frames) {
        float raw_distance_m = (float)bsp_status.latest.distance_cm * 0.01f;
        uint8_t distance_valid =
            ((raw_distance_m >= APP_RANGEFINDER_MIN_DISTANCE_M) &&
             (raw_distance_m <= APP_RANGEFINDER_MAX_DISTANCE_M)) ? 1U : 0U;
        uint8_t strength_valid =
            ((bsp_status.latest.strength >= APP_RANGEFINDER_MIN_STRENGTH) &&
             (bsp_status.latest.strength != 0xFFFFU)) ? 1U : 0U;
        uint8_t sample_valid =
            ((distance_valid != 0U) &&
             (strength_valid != 0U)) ? 1U : 0U;

        range_ctx.processed_frames = bsp_status.frames;
        range_ctx.status.distance_cm = bsp_status.latest.distance_cm;
        range_ctx.status.strength = bsp_status.latest.strength;
        range_ctx.status.raw_distance_m = raw_distance_m;
        range_ctx.status.temperature_c = bsp_status.latest.temperature_c;
        range_ctx.status.sample_ms = bsp_status.latest.received_ms;

        if (sample_valid != 0U) {
            if (range_ctx.previous_sample_ms == 0U) {
                range_ctx.status.height_m = raw_distance_m;
                range_ctx.status.vertical_velocity_m_s = 0.0f;
            } else {
                uint32_t dt_ms = bsp_status.latest.received_ms -
                                 range_ctx.previous_sample_ms;

                range_ctx.status.height_m += APP_RANGEFINDER_HEIGHT_ALPHA *
                    (raw_distance_m - range_ctx.status.height_m);
                if ((dt_ms >= 2U) && (dt_ms <= APP_RANGEFINDER_TIMEOUT_MS)) {
                    float velocity =
                        (range_ctx.status.height_m - range_ctx.previous_height_m) /
                        ((float)dt_ms * 0.001f);
                    velocity = rangefinder_clamp(velocity,
                                                  -APP_RANGEFINDER_MAX_VELOCITY_M_S,
                                                   APP_RANGEFINDER_MAX_VELOCITY_M_S);
                    range_ctx.status.vertical_velocity_m_s +=
                        APP_RANGEFINDER_VELOCITY_ALPHA *
                        (velocity - range_ctx.status.vertical_velocity_m_s);
                }
            }
            range_ctx.previous_height_m = range_ctx.status.height_m;
            range_ctx.previous_sample_ms = bsp_status.latest.received_ms;
            range_ctx.status.valid = 1U;
        } else {
            if ((range_ctx.previous_sample_ms == 0U) ||
                ((now - range_ctx.previous_sample_ms) > APP_RANGEFINDER_TIMEOUT_MS)) {
                range_ctx.status.valid = 0U;
            }
            range_ctx.status.rejected_samples++;
            if (strength_valid == 0U) {
                range_ctx.status.strength_rejects++;
            }
        }
    }

    range_ctx.status.age_ms = (range_ctx.status.sample_ms != 0U) ?
                           (now - range_ctx.status.sample_ms) : 0xFFFFFFFFUL;
    if ((range_ctx.previous_sample_ms == 0U) ||
        ((now - range_ctx.previous_sample_ms) > APP_RANGEFINDER_TIMEOUT_MS)) {
        range_ctx.status.valid = 0U;
        range_ctx.status.vertical_velocity_m_s = 0.0f;
    }

}

uint8_t APP_Rangefinder_GetHeightSample(float *height_m,
                                        float *vertical_velocity_m_s,
                                        uint32_t *sample_ms)
{
    if (range_ctx.status.valid == 0U) {
        return 0U;
    }
    if (height_m != NULL) {
        *height_m = range_ctx.status.height_m;
    }
    if (vertical_velocity_m_s != NULL) {
        *vertical_velocity_m_s = range_ctx.status.vertical_velocity_m_s;
    }
    if (sample_ms != NULL) {
        *sample_ms = range_ctx.status.sample_ms;
    }
    return 1U;
}

void APP_Rangefinder_GetStatus(APP_RangefinderStatus *status)
{
    if (status != NULL) {
        *status = range_ctx.status;
    }
}

void APP_Rangefinder_Report(void)
{
    APP_RangefinderStatus status;

    APP_Rangefinder_GetStatus(&status);
    APP_Control_QueueText("RANGE ok=%u init=%u age_ms=%lu dist_cm=%u raw_mm=%ld height_mm=%ld vz_mm_s=%ld strength=%u min_strength=%u rej=%lu strength_rej=%lu step_rej=%lu temp_cdeg=%ld frames=%lu bytes=%lu cksum=%lu frame_err=%lu rst=%lu dma_evt=%lu uerr=%lu last_err=0x%lX\r\n",
                          (unsigned int)status.valid,
                          (unsigned int)status.initialized,
                          (unsigned long)status.age_ms,
                          (unsigned int)status.distance_cm,
                          (long)(status.raw_distance_m * 1000.0f),
                          (long)(status.height_m * 1000.0f),
                          (long)(status.vertical_velocity_m_s * 1000.0f),
                          (unsigned int)status.strength,
                          (unsigned int)APP_RANGEFINDER_MIN_STRENGTH,
                          (unsigned long)status.rejected_samples,
                          (unsigned long)status.strength_rejects,
                          (unsigned long)status.step_rejects,
                          (long)(status.temperature_c * 100.0f),
                          (unsigned long)status.frames,
                          (unsigned long)status.bytes,
                          (unsigned long)status.checksum_errors,
                          (unsigned long)status.frame_errors,
                          (unsigned long)status.rx_restarts,
                          (unsigned long)status.dma_events,
                          (unsigned long)status.uart_errors,
                          (unsigned long)status.last_uart_error);
}
