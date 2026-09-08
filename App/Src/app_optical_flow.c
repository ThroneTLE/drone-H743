#include "app_optical_flow.h"

#include "app_control.h"
#include "bsp_optical_flow.h"
#include "drv_optical_flow.h"
#include "svc_flow_nav.h"

#include <string.h>

/*
 * R-M5-5：本文件只剩设备生命周期（初始化 / 健康 / 恢复）与报告组装。
 * 高度 LPF、光流中值与速度换算、质量门控、速度合理性门全部搬到
 * Services/Src/svc_flow_nav.c，这里不再保留任何一份副本，只做透传。
 */
#define APP_FLOW_STARTUP_GRACE_MS    1500U
#define APP_FLOW_RETRY_INTERVAL_MS   1000U
#define APP_FLOW_FAILED_RETRY_MS     5000U
#define APP_FLOW_FAST_RETRY_LIMIT    5U

typedef struct {
    uint8_t initialized;
    int32_t init_status;
    APP_OPTICAL_FLOW_VelSource velocity_source;
    APP_OPTICAL_FLOW_Health health;
    uint32_t init_attempts;
    uint32_t recovery_count;
    uint32_t last_init_attempt_ms;
    BSP_OPTICAL_FLOW_Status bsp_status;
} APP_OpticalFlow_Context;

static APP_OpticalFlow_Context flow_ctx;

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
    flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_NONE;
    SVC_FlowNav_Reset();
    if (status == DRV_OPTICAL_FLOW_OK) {
        flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_STARTING;
    } else {
        flow_ctx.health = (flow_ctx.initialized != 0U) ?
                          APP_OPTICAL_FLOW_HEALTH_STARTING :
                          APP_OPTICAL_FLOW_HEALTH_RETRYING;
    }
}

/*
 * ★ 光流安装方言的唯一边界 ★
 *
 * PMW3901 类传感器按 X前/Y右(FRD) 报数，规范机体系是 FLU（X前/Y左），故 Y 取负。
 * 这是"芯片说了什么"变成"载具测到了什么"的那一刻，方言只应该死在这里。
 *
 * 为什么不放在下游：2026-09-07 之前它在 app_stabilizer.c 的
 * stabilizer_compensate_flow_rotation() **末尾**做，而同一函数里的旋转补偿项是用
 * 已经转成 FLU 的陀螺算的（APP_Sensor_ApplyFrameCorrection 早已把 accel/gyro 一起
 * 转过），于是 FLU 的补偿向量被加到了尚未转换的 FRD 速度上：
 *     X 前向两系同号 → 补偿正确，看不出任何问题；
 *     Y 在 FLU 朝左、FRD 朝右 → 补偿反号，净效果是减了两倍。
 * 手飞画圆时靠滚转产生向心加速度，φ ≈ -a_y/g，故 ω_x = dφ/dt 与 v_y **同相**，
 * 错号的 Y 补偿正比于 ω_x，正好反相抵消真实横向速度：误差/信号 = 2(h+r_z)Ω²/g，
 * h≈1 m 时 Ω≈2.2 rad/s（约 2.9 s 一圈）即完全抵消。实测表现就是传感器页原始光流
 * 两轴都是正弦、状态页只剩 X，而且飞得慢或飞得低时会自己"变轻"，极易误判成已修好。
 *
 * 所以边界必须在混入任何机体量之前。从这里往下游——Service、EKF、控制器、遥测、
 * 上位机——全部是规范 FLU，**任何一处都不许再出现 FrdToFlu/FluToFrd**；
 * tests/test_flow_rotation_comp_frame.py 会挡住这种回潮。
 *
 * 例外：flow_vel_x/y 的**原始**计数照旧出现在 APP_OPTICAL_FLOW_Status 里（传感器
 * 页显示用），那是"芯片原话"，不带坐标系语义，不要顺手把它也转了。
 */
static void app_flow_fill_sample(const BSP_OPTICAL_FLOW_Frame *frame,
                                 SVC_FLOW_NAV_Sample *sample)
{
    memset(sample, 0, sizeof(*sample));
    if (frame == NULL) {
        return;
    }

    sample->frame_valid =
        (frame->valid == DRV_OPTICAL_FLOW_VALID) ? 1U : 0U;
    sample->distance_valid = frame->distance_valid;
    sample->flow_valid = frame->flow_valid;
    sample->distance_mm = frame->distance_mm;
    sample->distance_received_ms = frame->distance_received_ms;
    sample->flow_received_ms = frame->flow_received_ms;
    sample->sensor_time_ms = frame->time_ms;
    sample->sample_interval_us = frame->sample_interval_us;
    /* FRD → FLU：X 同向，Y 取负。INT16_MIN 取负会溢出，钳到 INT16_MAX。 */
    sample->flow_vel_x = frame->flow_vel_x;
    sample->flow_vel_y = (frame->flow_vel_y == INT16_MIN) ?
                         INT16_MAX : (int16_t)(-frame->flow_vel_y);
    sample->flow_quality = frame->flow_quality;
}

void APP_OpticalFlow_Init(void)
{
    uint32_t now = HAL_GetTick();

    memset(&flow_ctx, 0, sizeof(flow_ctx));
    flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_STARTING;
    SVC_FlowNav_Init();
    app_optical_flow_try_init(now);
}

void APP_OpticalFlow_Step(void)
{
    uint32_t now = HAL_GetTick();
    SVC_FLOW_NAV_Sample sample;
    SVC_FLOW_NAV_SampleResult result;
    uint32_t last_good_ms;

    if (flow_ctx.initialized == 0U) {
        flow_ctx.health =
            (flow_ctx.init_attempts > APP_FLOW_FAST_RETRY_LIMIT) ?
            APP_OPTICAL_FLOW_HEALTH_FAILED :
            APP_OPTICAL_FLOW_HEALTH_RETRYING;
        return;
    }

    BSP_OPTICAL_FLOW_Service();
    BSP_OPTICAL_FLOW_GetStatus(&flow_ctx.bsp_status);
    app_flow_fill_sample(&flow_ctx.bsp_status.latest, &sample);

    SVC_FlowNav_Age(now);
    result = SVC_FlowNav_PushSample(&sample, now);

    switch (result) {
    case SVC_FLOW_NAV_SAMPLE_ACCEPTED:
    case SVC_FLOW_NAV_SAMPLE_HOLD:
        flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_OK;
        return;
    case SVC_FLOW_NAV_SAMPLE_REJECTED:
    case SVC_FLOW_NAV_SAMPLE_WARMUP:
        flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_NONE;
        flow_ctx.health = APP_OPTICAL_FLOW_HEALTH_STARTING;
        return;
    case SVC_FLOW_NAV_SAMPLE_STALE:
    default:
        break;
    }

    flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_NONE;
    last_good_ms = SVC_FlowNav_GetLastGoodMs();
    {
        uint32_t good_age_ms = (last_good_ms != 0U) ?
                               (now - last_good_ms) :
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
    if (SVC_FlowNav_GetSensorVelocity(vx_m_s, vy_m_s, sample_ms,
                                      HAL_GetTick()) == 0U) {
        flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_NONE;
        return 0U;
    }

    flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_FLOW;
    return 1U;
}

uint8_t APP_OpticalFlow_GetHeightSample(float *height_m,
                                        float *vertical_velocity_m_s,
                                        uint32_t *sample_ms)
{
    return SVC_FlowNav_GetHeight(height_m, vertical_velocity_m_s, sample_ms,
                                 HAL_GetTick());
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
    SVC_FLOW_NAV_State nav;

    if (status == NULL) {
        return;
    }

    BSP_OPTICAL_FLOW_GetStatus(&flow_ctx.bsp_status);
    frame = &flow_ctx.bsp_status.latest;
    SVC_FlowNav_GetState(&nav);

    memset(status, 0, sizeof(*status));
    status->initialized = flow_ctx.initialized;
    status->init_status = flow_ctx.init_status;
    status->health = flow_ctx.health;
    status->init_attempts = flow_ctx.init_attempts;
    status->recovery_count = flow_ctx.recovery_count;
    status->velocity_reject_count = nav.velocity_reject_count;
    status->valid = frame->valid;
    status->velocity_valid = nav.velocity_valid;
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
    status->flow_vel_x_filtered = nav.flow_vel_x_filtered;
    status->flow_vel_y_filtered = nav.flow_vel_y_filtered;
    status->flow_quality = frame->flow_quality;
    status->flow_status = frame->flow_status;
    status->flow_filter_ready = nav.flow_filter_ready;
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
    status->height_valid = nav.height_valid;
    status->height_m = nav.height_m;
    status->height_raw_m = nav.height_raw_m;
    status->vertical_velocity_m_s = nav.vertical_velocity_m_s;
    status->height_filter_alpha = nav.height_filter_alpha;
    status->vx_m_s = nav.vx_m_s;
    status->vy_m_s = nav.vy_m_s;
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
                          (unsigned int)SVC_FLOW_NAV_MIN_QUALITY,
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
    /*
     * R-M5-5 新增行，纯追加：既有 FLOW ok/mico/data/raw 四行一个字段没动，
     * 键名也与上位机校准页 / 监控页在用的键全部错开，不影响 R-S1-2。
     */
    {
        float pos_x_m = 0.0f;
        float pos_y_m = 0.0f;
        float disp_x_m = 0.0f;
        float disp_y_m = 0.0f;
        float vel_x_m_s = 0.0f;
        float vel_y_m_s = 0.0f;

        SVC_FlowNav_GetPosition(&pos_x_m, &pos_y_m);
        SVC_FlowNav_GetDisplacement(&disp_x_m, &disp_y_m);
        SVC_FlowNav_GetVelocity(&vel_x_m_s, &vel_y_m_s);
        APP_Control_QueueText("FLOW nav pos_x_mm=%ld pos_y_mm=%ld disp_x_mm=%ld disp_y_mm=%ld vel_x_mm_s=%ld vel_y_mm_s=%ld steps=%lu dt_us=%lu\r\n",
                              (long)(pos_x_m * 1000.0f),
                              (long)(pos_y_m * 1000.0f),
                              (long)(disp_x_m * 1000.0f),
                              (long)(disp_y_m * 1000.0f),
                              (long)(vel_x_m_s * 1000.0f),
                              (long)(vel_y_m_s * 1000.0f),
                              (unsigned long)SVC_FlowNav_GetIntegratedStepCount(),
                              (unsigned long)SVC_FlowNav_GetLastIntegrationDtUs());
    }
}
