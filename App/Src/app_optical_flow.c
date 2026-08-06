#include "app_optical_flow.h"

#include "app_control.h"
#include "bsp_optical_flow.h"
#include "drv_optical_flow.h"

#include <math.h>
#include <string.h>

#define APP_FLOW_TIMEOUT_MS          100U
#define APP_FLOW_STARTUP_GRACE_MS    1500U
#define APP_FLOW_RETRY_INTERVAL_MS   1000U
#define APP_FLOW_FAILED_RETRY_MS     5000U
#define APP_FLOW_FAST_RETRY_LIMIT    5U
#define APP_FLOW_HEIGHT_LPF_ALPHA    0.35f
#define APP_FLOW_VELOCITY_LPF_ALPHA  0.20f
#define APP_FLOW_MIN_QUALITY         APP_OPTICAL_FLOW_MIN_QUALITY
#define APP_FLOW_MAX_HEIGHT_M        12.0f
#define APP_FLOW_MAX_HEIGHT_STEP_M   0.18f
#define APP_FLOW_MAX_VERTICAL_VELOCITY_M_S 5.0f
#define APP_FLOW_MAX_SPEED_M_S       2.50f
#define APP_FLOW_MAX_SPEED_STEP_M_S  1.20f
#define APP_FLOW_MEDIAN_WINDOW       5U
#define APP_FLOW_MEDIAN_MIN_SAMPLES  3U
#define APP_FLOW_FILTER_RESET_MS     250U

typedef struct {
    uint8_t initialized;
    int32_t init_status;
    APP_OPTICAL_FLOW_VelSource velocity_source;
    float height_m;
    float height_raw_m;
    uint8_t height_valid;
    float vertical_velocity_m_s;
    uint32_t height_sample_ms;
    float previous_height_m;
    uint32_t previous_height_sample_ms;
    float vx_m_s;
    float vy_m_s;
    uint8_t velocity_valid;
    uint32_t velocity_sample_ms;
    APP_OPTICAL_FLOW_Health health;
    uint32_t init_attempts;
    uint32_t recovery_count;
    uint32_t velocity_reject_count;
    uint32_t processed_frames;
    uint32_t processed_flow_ms;
    int16_t flow_vel_x_window[APP_FLOW_MEDIAN_WINDOW];
    int16_t flow_vel_y_window[APP_FLOW_MEDIAN_WINDOW];
    uint8_t flow_median_count;
    uint8_t flow_median_pos;
    int16_t filtered_flow_vel_x;
    int16_t filtered_flow_vel_y;
    uint8_t flow_filter_ready;
    uint32_t last_init_attempt_ms;
    uint32_t last_good_ms;
    float last_accept_vx_m_s;
    float last_accept_vy_m_s;
    BSP_OPTICAL_FLOW_Status bsp_status;
} APP_OpticalFlow_Context;

static APP_OpticalFlow_Context flow_ctx;

static float app_flow_square(float value)
{
    return value * value;
}

static float app_flow_clamp(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static void app_flow_reset_median_filter(void)
{
    memset(flow_ctx.flow_vel_x_window, 0, sizeof(flow_ctx.flow_vel_x_window));
    memset(flow_ctx.flow_vel_y_window, 0, sizeof(flow_ctx.flow_vel_y_window));
    flow_ctx.flow_median_count = 0U;
    flow_ctx.flow_median_pos = 0U;
    flow_ctx.filtered_flow_vel_x = 0;
    flow_ctx.filtered_flow_vel_y = 0;
    flow_ctx.flow_filter_ready = 0U;
}

static void app_flow_clear_velocity_sample(void)
{
    flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_NONE;
    flow_ctx.vx_m_s = 0.0f;
    flow_ctx.vy_m_s = 0.0f;
    flow_ctx.velocity_valid = 0U;
    flow_ctx.velocity_sample_ms = 0U;
    flow_ctx.last_good_ms = 0U;
    flow_ctx.last_accept_vx_m_s = 0.0f;
    flow_ctx.last_accept_vy_m_s = 0.0f;
}

static void app_flow_mark_velocity_invalid(void)
{
    flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_NONE;
    flow_ctx.velocity_valid = 0U;
    flow_ctx.velocity_sample_ms = 0U;
}

static void app_flow_reject_velocity_sample(void)
{
    flow_ctx.velocity_reject_count++;
    app_flow_mark_velocity_invalid();
}

static void app_flow_reset_samples(void)
{
    flow_ctx.height_m = 0.0f;
    flow_ctx.height_raw_m = 0.0f;
    flow_ctx.height_valid = 0U;
    flow_ctx.vertical_velocity_m_s = 0.0f;
    flow_ctx.height_sample_ms = 0U;
    flow_ctx.previous_height_m = 0.0f;
    flow_ctx.previous_height_sample_ms = 0U;
    app_flow_reset_median_filter();
    app_flow_clear_velocity_sample();
    flow_ctx.processed_frames = 0U;
    flow_ctx.processed_flow_ms = 0U;
}

static int16_t app_flow_median_i16(const int16_t *values, uint8_t count)
{
    int16_t sorted[APP_FLOW_MEDIAN_WINDOW];

    if ((values == NULL) || (count == 0U)) {
        return 0;
    }
    if (count > APP_FLOW_MEDIAN_WINDOW) {
        count = APP_FLOW_MEDIAN_WINDOW;
    }

    for (uint8_t i = 0U; i < count; ++i) {
        sorted[i] = values[i];
    }
    for (uint8_t i = 1U; i < count; ++i) {
        int16_t key = sorted[i];
        uint8_t j = i;
        while ((j > 0U) && (sorted[j - 1U] > key)) {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = key;
    }

    return sorted[count / 2U];
}

static void app_flow_update_median_filter(const BSP_OPTICAL_FLOW_Frame *frame)
{
    if (frame == NULL) {
        app_flow_reset_median_filter();
        return;
    }

    flow_ctx.flow_vel_x_window[flow_ctx.flow_median_pos] = frame->flow_vel_x;
    flow_ctx.flow_vel_y_window[flow_ctx.flow_median_pos] = frame->flow_vel_y;
    flow_ctx.flow_median_pos++;
    if (flow_ctx.flow_median_pos >= APP_FLOW_MEDIAN_WINDOW) {
        flow_ctx.flow_median_pos = 0U;
    }
    if (flow_ctx.flow_median_count < APP_FLOW_MEDIAN_WINDOW) {
        flow_ctx.flow_median_count++;
    }

    flow_ctx.filtered_flow_vel_x =
        app_flow_median_i16(flow_ctx.flow_vel_x_window,
                            flow_ctx.flow_median_count);
    flow_ctx.filtered_flow_vel_y =
        app_flow_median_i16(flow_ctx.flow_vel_y_window,
                            flow_ctx.flow_median_count);
    flow_ctx.flow_filter_ready =
        (flow_ctx.flow_median_count >= APP_FLOW_MEDIAN_MIN_SAMPLES) ? 1U : 0U;
}

static void app_optical_flow_try_init(uint32_t now_ms)
{
    BSP_OPTICAL_FLOW_StatusCode status;
    BSP_OPTICAL_FLOW_Status bsp_status;

    flow_ctx.last_init_attempt_ms = now_ms;
    flow_ctx.init_attempts++;
    status = BSP_OPTICAL_FLOW_Init();
    BSP_OPTICAL_FLOW_GetStatus(&bsp_status);
    flow_ctx.init_status = (int32_t)status;
    flow_ctx.bsp_status = bsp_status;
    flow_ctx.initialized = bsp_status.initialized;
    app_flow_reset_samples();
    if (status == DRV_OPTICAL_FLOW_OK) {
        flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_STARTING;
    } else {
        flow_ctx.health = (flow_ctx.initialized != 0U) ?
                          APP_OPTICAL_FLOW_HEALTH_STARTING :
                          APP_OPTICAL_FLOW_HEALTH_RETRYING;
    }
}

static uint8_t app_flow_velocity_plausible(float vx_m_s, float vy_m_s)
{
    float speed_sq = app_flow_square(vx_m_s) + app_flow_square(vy_m_s);
    float dvx;
    float dvy;
    float step_sq;

    if (speed_sq > app_flow_square(APP_FLOW_MAX_SPEED_M_S)) {
        return 0U;
    }

    if (flow_ctx.last_good_ms == 0U) {
        return 1U;
    }

    dvx = vx_m_s - flow_ctx.last_accept_vx_m_s;
    dvy = vy_m_s - flow_ctx.last_accept_vy_m_s;
    step_sq = app_flow_square(dvx) + app_flow_square(dvy);
    return (step_sq <= app_flow_square(APP_FLOW_MAX_SPEED_STEP_M_S)) ? 1U : 0U;
}

static void app_flow_update_height(const BSP_OPTICAL_FLOW_Frame *frame)
{
    float raw_height_m;
    float filtered_height_m;

    if ((frame == NULL) || (frame->distance_valid == 0U) ||
        (frame->distance_mm == 0UL) || (frame->distance_received_ms == 0UL) ||
        (frame->distance_received_ms == flow_ctx.previous_height_sample_ms)) {
        return;
    }

    raw_height_m = (float)frame->distance_mm * 0.001f;
    if ((raw_height_m <= 0.0f) || (raw_height_m > APP_FLOW_MAX_HEIGHT_M)) {
        return;
    }
    if ((flow_ctx.height_valid != 0U) &&
        (fabsf(raw_height_m - flow_ctx.height_m) >
         APP_FLOW_MAX_HEIGHT_STEP_M)) {
        return;
    }

    if (flow_ctx.previous_height_sample_ms == 0U) {
        filtered_height_m = raw_height_m;
        flow_ctx.vertical_velocity_m_s = 0.0f;
    } else {
        uint32_t dt_ms =
            frame->distance_received_ms - flow_ctx.previous_height_sample_ms;
        filtered_height_m = flow_ctx.height_m +
            APP_FLOW_HEIGHT_LPF_ALPHA * (raw_height_m - flow_ctx.height_m);
        if ((dt_ms >= 2U) && (dt_ms <= APP_FLOW_TIMEOUT_MS)) {
            float vz_m_s = (filtered_height_m - flow_ctx.previous_height_m) /
                           ((float)dt_ms * 0.001f);
            vz_m_s = app_flow_clamp(vz_m_s,
                                    -APP_FLOW_MAX_VERTICAL_VELOCITY_M_S,
                                     APP_FLOW_MAX_VERTICAL_VELOCITY_M_S);
            flow_ctx.vertical_velocity_m_s += APP_FLOW_VELOCITY_LPF_ALPHA *
                (vz_m_s - flow_ctx.vertical_velocity_m_s);
        }
    }

    flow_ctx.height_raw_m = raw_height_m;
    flow_ctx.height_m = filtered_height_m;
    flow_ctx.height_valid = 1U;
    flow_ctx.height_sample_ms = frame->distance_received_ms;
    flow_ctx.previous_height_m = flow_ctx.height_m;
    flow_ctx.previous_height_sample_ms = frame->distance_received_ms;
}

static uint8_t app_flow_frame_usable(const BSP_OPTICAL_FLOW_Frame *frame,
                                     uint32_t now_ms)
{
    if ((frame == NULL) ||
        (frame->valid != DRV_OPTICAL_FLOW_VALID) ||
        (frame->distance_valid == 0U) ||
        (frame->flow_valid == 0U) ||
        (frame->distance_received_ms == 0UL) ||
        (frame->flow_received_ms == 0UL) ||
        ((now_ms - frame->distance_received_ms) > APP_FLOW_TIMEOUT_MS) ||
        ((now_ms - frame->flow_received_ms) > APP_FLOW_TIMEOUT_MS) ||
        (frame->flow_quality < APP_FLOW_MIN_QUALITY) ||
        (flow_ctx.height_valid == 0U)) {
        return 0U;
    }

    return 1U;
}

void APP_OpticalFlow_Init(void)
{
    uint32_t now = HAL_GetTick();

    memset(&flow_ctx, 0, sizeof(flow_ctx));
    flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_STARTING;
    app_optical_flow_try_init(now);
}

void APP_OpticalFlow_Step(void)
{
    uint32_t now = HAL_GetTick();
    const BSP_OPTICAL_FLOW_Frame *frame;
    uint8_t new_flow = 0U;

    if (flow_ctx.initialized == 0U) {
        flow_ctx.health =
            (flow_ctx.init_attempts > APP_FLOW_FAST_RETRY_LIMIT) ?
            APP_OPTICAL_FLOW_HEALTH_FAILED :
            APP_OPTICAL_FLOW_HEALTH_RETRYING;
        return;
    }

    BSP_OPTICAL_FLOW_Service();
    BSP_OPTICAL_FLOW_GetStatus(&flow_ctx.bsp_status);
    frame = &flow_ctx.bsp_status.latest;

    if (flow_ctx.height_sample_ms != 0U) {
        if ((now - flow_ctx.height_sample_ms) > APP_FLOW_TIMEOUT_MS) {
            flow_ctx.height_valid = 0U;
            flow_ctx.vertical_velocity_m_s = 0.0f;
            app_flow_mark_velocity_invalid();
            if ((flow_ctx.last_good_ms == 0U) ||
                ((now - flow_ctx.last_good_ms) > APP_FLOW_FILTER_RESET_MS)) {
                app_flow_reset_median_filter();
            }
        }
    }

    if (flow_ctx.bsp_status.frames != flow_ctx.processed_frames) {
        flow_ctx.processed_frames = flow_ctx.bsp_status.frames;
        app_flow_update_height(frame);
        if ((frame->flow_received_ms != 0UL) &&
            (frame->flow_received_ms != flow_ctx.processed_flow_ms)) {
            new_flow = 1U;
        }
    }

    if (new_flow != 0U) {
        float sensor_vx_m_s;
        float sensor_vy_m_s;

        flow_ctx.processed_flow_ms = frame->flow_received_ms;
        if (app_flow_frame_usable(frame, now) == 0U) {
            app_flow_reject_velocity_sample();
            if ((flow_ctx.last_good_ms == 0U) ||
                ((now - flow_ctx.last_good_ms) > APP_FLOW_FILTER_RESET_MS)) {
                app_flow_reset_median_filter();
            }
            flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_STARTING;
            return;
        }

        app_flow_update_median_filter(frame);
        if (flow_ctx.flow_filter_ready == 0U) {
            app_flow_mark_velocity_invalid();
            flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_STARTING;
            return;
        }

        sensor_vx_m_s =
            (float)flow_ctx.filtered_flow_vel_x * 0.01f * flow_ctx.height_m;
        sensor_vy_m_s =
            (float)flow_ctx.filtered_flow_vel_y * 0.01f * flow_ctx.height_m;

        if (app_flow_velocity_plausible(sensor_vx_m_s, sensor_vy_m_s) == 0U) {
            app_flow_reject_velocity_sample();
            if ((flow_ctx.last_good_ms == 0U) ||
                ((now - flow_ctx.last_good_ms) > APP_FLOW_FILTER_RESET_MS)) {
                app_flow_reset_median_filter();
            }
            flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_STARTING;
            return;
        }

        flow_ctx.vx_m_s = sensor_vx_m_s;
        flow_ctx.vy_m_s = sensor_vy_m_s;
        flow_ctx.velocity_valid = 1U;
        flow_ctx.velocity_sample_ms = frame->flow_received_ms;
        flow_ctx.last_good_ms = frame->flow_received_ms;
        flow_ctx.last_accept_vx_m_s = flow_ctx.vx_m_s;
        flow_ctx.last_accept_vy_m_s = flow_ctx.vy_m_s;
        flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_OK;
        return;
    }

    if ((flow_ctx.last_good_ms != 0U) &&
        ((now - flow_ctx.last_good_ms) <= APP_FLOW_TIMEOUT_MS)) {
        flow_ctx.velocity_valid = 1U;
        flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_OK;
        return;
    }

    flow_ctx.velocity_valid = 0U;
    {
        uint32_t good_age_ms = (flow_ctx.last_good_ms != 0U) ?
                               (now - flow_ctx.last_good_ms) :
                               (now - flow_ctx.last_init_attempt_ms);
        if (good_age_ms > APP_FLOW_STARTUP_GRACE_MS) {
            flow_ctx.health =
                (flow_ctx.init_attempts > APP_FLOW_FAST_RETRY_LIMIT) ?
                APP_OPTICAL_FLOW_HEALTH_FAILED :
                APP_OPTICAL_FLOW_HEALTH_RETRYING;
        } else {
            flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_STARTING;
        }
    }
}

void APP_OpticalFlow_ServiceRecovery(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t interval_ms;
    uint8_t needs_recovery;

    needs_recovery =
        ((flow_ctx.health == APP_OPTICAL_FLOW_HEALTH_RETRYING) ||
         (flow_ctx.health == APP_OPTICAL_FLOW_HEALTH_FAILED)) ? 1U : 0U;
    if (needs_recovery == 0U) {
        return;
    }

    interval_ms = (flow_ctx.health == APP_OPTICAL_FLOW_HEALTH_FAILED) ?
                  APP_FLOW_FAILED_RETRY_MS :
                  APP_FLOW_RETRY_INTERVAL_MS;
    if ((now - flow_ctx.last_init_attempt_ms) < interval_ms) {
        return;
    }

    flow_ctx.recovery_count++;
    app_optical_flow_try_init(now);
}

uint8_t APP_OpticalFlow_GetVelocity(float *vx_m_s, float *vy_m_s)
{
    uint32_t sample_ms;

    return APP_OpticalFlow_GetVelocitySample(vx_m_s, vy_m_s, &sample_ms);
}

uint8_t APP_OpticalFlow_GetVelocitySample(float *vx_m_s,
                                          float *vy_m_s,
                                          uint32_t *sample_ms)
{
    uint32_t now = HAL_GetTick();

    if ((flow_ctx.velocity_valid == 0U) ||
        (flow_ctx.velocity_sample_ms == 0U) ||
        ((now - flow_ctx.velocity_sample_ms) > APP_FLOW_TIMEOUT_MS)) {
        flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_NONE;
        return 0U;
    }

    if (vx_m_s != NULL) {
        *vx_m_s = flow_ctx.vx_m_s;
    }
    if (vy_m_s != NULL) {
        *vy_m_s = flow_ctx.vy_m_s;
    }
    if (sample_ms != NULL) {
        *sample_ms = flow_ctx.velocity_sample_ms;
    }
    flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_FLOW;
    return 1U;
}

uint8_t APP_OpticalFlow_GetHeightSample(float *height_m,
                                        float *vertical_velocity_m_s,
                                        uint32_t *sample_ms)
{
    uint32_t now = HAL_GetTick();

    if ((flow_ctx.height_valid == 0U) ||
        (flow_ctx.height_sample_ms == 0U) ||
        ((now - flow_ctx.height_sample_ms) > APP_FLOW_TIMEOUT_MS)) {
        return 0U;
    }

    if (height_m != NULL) {
        *height_m = flow_ctx.height_m;
    }
    if (vertical_velocity_m_s != NULL) {
        *vertical_velocity_m_s = flow_ctx.vertical_velocity_m_s;
    }
    if (sample_ms != NULL) {
        *sample_ms = flow_ctx.height_sample_ms;
    }
    return 1U;
}

void APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VelSource source)
{
    flow_ctx.velocity_source = source;
}

const char *APP_OpticalFlow_VelSourceName(APP_OPTICAL_FLOW_VelSource source)
{
    if (source == APP_OPTICAL_FLOW_VEL_SOURCE_FLOW) {
        return "flow";
    }
    if (source == APP_OPTICAL_FLOW_VEL_SOURCE_IMU) {
        return "imu";
    }
    return "none";
}

void APP_OpticalFlow_GetStatus(APP_OPTICAL_FLOW_Status *status)
{
    uint32_t now = HAL_GetTick();
    const BSP_OPTICAL_FLOW_Frame *frame;

    if (status == NULL) {
        return;
    }

    BSP_OPTICAL_FLOW_GetStatus(&flow_ctx.bsp_status);
    frame = &flow_ctx.bsp_status.latest;

    memset(status, 0, sizeof(*status));
    status->initialized = flow_ctx.initialized;
    status->init_status = flow_ctx.init_status;
    status->health = flow_ctx.health;
    status->init_attempts = flow_ctx.init_attempts;
    status->recovery_count = flow_ctx.recovery_count;
    status->velocity_reject_count = flow_ctx.velocity_reject_count;
    status->valid = frame->valid;
    status->velocity_valid = flow_ctx.velocity_valid;
    status->velocity_source = flow_ctx.velocity_source;
    status->bytes = flow_ctx.bsp_status.bytes;
    status->frames = flow_ctx.bsp_status.frames;
    status->checksum_errors = flow_ctx.bsp_status.checksum_errors;
    status->frame_errors = flow_ctx.bsp_status.frame_errors;
    status->ignored_messages = flow_ctx.bsp_status.ignored_messages;
    status->short_payload_errors = flow_ctx.bsp_status.short_payload_errors;
    status->rx_restarts = flow_ctx.bsp_status.rx_restarts;
    status->dma_events = flow_ctx.bsp_status.dma_events;
    status->dma_last_size = flow_ctx.bsp_status.dma_last_size;
    status->uart_errors = flow_ctx.bsp_status.uart_errors;
    status->last_uart_error = flow_ctx.bsp_status.last_uart_error;
    status->last_rx_ms = flow_ctx.bsp_status.last_rx_ms;
    status->age_ms = (flow_ctx.bsp_status.last_rx_ms != 0U) ?
                     (now - flow_ctx.bsp_status.last_rx_ms) :
                     0xFFFFFFFFUL;
    status->baud_rate = flow_ctx.bsp_status.baud_rate;
    status->device_id = frame->device_id;
    status->system_id = frame->system_id;
    status->msg_id = frame->msg_id;
    status->sequence = frame->sequence;
    status->sensor_time_ms = frame->time_ms;
    status->distance_mm = frame->distance_mm;
    status->distance_age_ms = (frame->distance_received_ms != 0UL) ?
                              (now - frame->distance_received_ms) :
                              0xFFFFFFFFUL;
    status->flow_age_ms = (frame->flow_received_ms != 0UL) ?
                          (now - frame->flow_received_ms) :
                          0xFFFFFFFFUL;
    status->distance_valid = frame->distance_valid;
    status->strength = frame->strength;
    status->precision = frame->precision;
    status->tof_status = frame->tof_status;
    status->flow_vel_x = frame->flow_vel_x;
    status->flow_vel_y = frame->flow_vel_y;
    status->flow_vel_x_filtered = flow_ctx.filtered_flow_vel_x;
    status->flow_vel_y_filtered = flow_ctx.filtered_flow_vel_y;
    status->flow_quality = frame->flow_quality;
    status->flow_status = frame->flow_status;
    status->flow_filter_ready = flow_ctx.flow_filter_ready;
    status->sample_interval_us = frame->sample_interval_us;
    status->raw_count = flow_ctx.bsp_status.raw_stats.count;
    status->flow_vel_x_mean = flow_ctx.bsp_status.raw_stats.flow_vel_x_mean;
    status->flow_vel_y_mean = flow_ctx.bsp_status.raw_stats.flow_vel_y_mean;
    status->sample_interval_mean_us =
        flow_ctx.bsp_status.raw_stats.sample_interval_mean_us;
    status->distance_mean_mm = flow_ctx.bsp_status.raw_stats.distance_mean_mm;
    status->strength_mean = flow_ctx.bsp_status.raw_stats.strength_mean;
    status->flow_quality_mean =
        flow_ctx.bsp_status.raw_stats.flow_quality_mean;
    status->flow_vel_x_peak_to_peak =
        flow_ctx.bsp_status.raw_stats.flow_vel_x_peak_to_peak;
    status->flow_vel_y_peak_to_peak =
        flow_ctx.bsp_status.raw_stats.flow_vel_y_peak_to_peak;
    status->sample_interval_peak_to_peak_us =
        flow_ctx.bsp_status.raw_stats.sample_interval_peak_to_peak_us;
    status->distance_peak_to_peak_mm =
        flow_ctx.bsp_status.raw_stats.distance_peak_to_peak_mm;
    status->height_valid = flow_ctx.height_valid;
    status->height_m = flow_ctx.height_m;
    status->height_raw_m = flow_ctx.height_raw_m;
    status->vertical_velocity_m_s = flow_ctx.vertical_velocity_m_s;
    status->height_filter_alpha = APP_FLOW_HEIGHT_LPF_ALPHA;
    status->vx_m_s = flow_ctx.vx_m_s;
    status->vy_m_s = flow_ctx.vy_m_s;
}

void APP_OpticalFlow_Report(void)
{
    APP_OPTICAL_FLOW_Status status;
    int32_t height_mm;
    int32_t height_raw_mm;
    int32_t vz_mm_s;
    int32_t vx_mm_s;
    int32_t vy_mm_s;

    APP_OpticalFlow_GetStatus(&status);
    height_mm = (int32_t)(status.height_m * 1000.0f);
    height_raw_mm = (int32_t)(status.height_raw_m * 1000.0f);
    vz_mm_s = (int32_t)(status.vertical_velocity_m_s * 1000.0f);
    vx_mm_s = (int32_t)(status.vx_m_s * 1000.0f);
    vy_mm_s = (int32_t)(status.vy_m_s * 1000.0f);
    APP_Control_QueueText("FLOW ok=%u init=%ld health=%u attempts=%lu recover=%lu vel_rej=%lu baud=%lu bytes=%lu frames=%lu valid=%u age_ms=%lu source=%s vel_valid=%u height_valid=%u\r\n",
                          (unsigned int)status.initialized,
                          (long)status.init_status,
                          (unsigned int)status.health,
                          (unsigned long)status.init_attempts,
                          (unsigned long)status.recovery_count,
                          (unsigned long)status.velocity_reject_count,
                          (unsigned long)status.baud_rate,
                          (unsigned long)status.bytes,
                          (unsigned long)status.frames,
                          (unsigned int)status.valid,
                          (unsigned long)status.age_ms,
                          APP_OpticalFlow_VelSourceName(status.velocity_source),
                          (unsigned int)status.velocity_valid,
                          (unsigned int)status.height_valid);
    APP_Control_QueueText("FLOW mico dev=0x%02X sys=0x%02X msg=0x%02X seq=%u t_ms=%lu dist_mm=%lu dist_valid=%u range_q=%u dist_age=%lu flow_vx=%d flow_vy=%d filt_vx=%d filt_vy=%d filt_ready=%u quality=%u min_q=%u flow_st=%u flow_age=%lu sample_us=%u\r\n",
                          (unsigned int)status.device_id,
                          (unsigned int)status.system_id,
                          (unsigned int)status.msg_id,
                          (unsigned int)status.sequence,
                          (unsigned long)status.sensor_time_ms,
                          (unsigned long)status.distance_mm,
                          (unsigned int)status.distance_valid,
                          (unsigned int)status.strength,
                          (unsigned long)status.distance_age_ms,
                          (int)status.flow_vel_x,
                          (int)status.flow_vel_y,
                          (int)status.flow_vel_x_filtered,
                          (int)status.flow_vel_y_filtered,
                          (unsigned int)status.flow_filter_ready,
                          (unsigned int)status.flow_quality,
                          (unsigned int)APP_FLOW_MIN_QUALITY,
                          (unsigned int)status.flow_status,
                          (unsigned long)status.flow_age_ms,
                          (unsigned int)status.sample_interval_us);
    APP_Control_QueueText("FLOW data height_raw_mm=%ld height_mm=%ld vz_mm_s=%ld vx_mm_s=%ld vy_mm_s=%ld cksum=%lu frame_err=%lu ignored=%lu short=%lu rst=%lu dma_evt=%lu dma_size=%lu uerr=%lu last_err=0x%lX\r\n",
                          (long)height_raw_mm,
                          (long)height_mm,
                          (long)vz_mm_s,
                          (long)vx_mm_s,
                          (long)vy_mm_s,
                          (unsigned long)status.checksum_errors,
                          (unsigned long)status.frame_errors,
                          (unsigned long)status.ignored_messages,
                          (unsigned long)status.short_payload_errors,
                          (unsigned long)status.rx_restarts,
                          (unsigned long)status.dma_events,
                          (unsigned long)status.dma_last_size,
                          (unsigned long)status.uart_errors,
                          (unsigned long)status.last_uart_error);
    APP_Control_QueueText("FLOW raw n=%u vx_avg=%d vy_avg=%d dt_avg=%u dist_avg=%lu strength_avg=%u q_avg=%u vx_pp=%d vy_pp=%d dt_pp=%u dist_pp=%lu\r\n",
                          (unsigned int)status.raw_count,
                          (int)status.flow_vel_x_mean,
                          (int)status.flow_vel_y_mean,
                          (unsigned int)status.sample_interval_mean_us,
                          (unsigned long)status.distance_mean_mm,
                          (unsigned int)status.strength_mean,
                          (unsigned int)status.flow_quality_mean,
                          (int)status.flow_vel_x_peak_to_peak,
                          (int)status.flow_vel_y_peak_to_peak,
                          (unsigned int)status.sample_interval_peak_to_peak_us,
                          (unsigned long)status.distance_peak_to_peak_mm);
}
