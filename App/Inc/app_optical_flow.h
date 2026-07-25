#ifndef APP_OPTICAL_FLOW_H
#define APP_OPTICAL_FLOW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_OPTICAL_FLOW_VEL_SOURCE_IMU = 0,
    APP_OPTICAL_FLOW_VEL_SOURCE_FLOW = 1
} APP_OPTICAL_FLOW_VelSource;

typedef enum {
    APP_OPTICAL_FLOW_HEALTH_STARTING = 0,
    APP_OPTICAL_FLOW_HEALTH_OK = 1,
    APP_OPTICAL_FLOW_HEALTH_RETRYING = 2,
    APP_OPTICAL_FLOW_HEALTH_FAILED = 3
} APP_OPTICAL_FLOW_Health;

typedef struct {
    uint8_t initialized;
    int32_t init_status;
    APP_OPTICAL_FLOW_Health health;
    uint32_t init_attempts;
    uint32_t recovery_count;
    uint32_t velocity_reject_count;
    uint8_t valid;
    uint8_t velocity_valid;
    APP_OPTICAL_FLOW_VelSource velocity_source;
    uint32_t bytes;
    uint32_t frames;
    uint32_t checksum_errors;
    uint32_t frame_errors;
    uint32_t ignored_messages;
    uint32_t short_payload_errors;
    uint32_t rx_restarts;
    uint32_t dma_events;
    uint32_t dma_last_size;
    uint32_t uart_errors;
    uint32_t last_uart_error;
    uint32_t last_rx_ms;
    uint32_t age_ms;
    uint32_t baud_rate;
    uint8_t device_id;
    uint8_t system_id;
    uint8_t msg_id;
    uint8_t sequence;
    uint32_t sensor_time_ms;
    uint32_t distance_mm;
    uint32_t distance_age_ms;
    uint32_t flow_age_ms;
    uint8_t distance_valid;
    uint8_t strength;
    uint8_t precision;
    uint8_t tof_status;
    int16_t flow_vel_x;
    int16_t flow_vel_y;
    uint8_t flow_quality;
    uint8_t flow_status;
    uint16_t sample_interval_us;
    uint16_t raw_count;
    int16_t flow_vel_x_mean;
    int16_t flow_vel_y_mean;
    uint16_t sample_interval_mean_us;
    uint32_t distance_mean_mm;
    uint8_t strength_mean;
    uint8_t flow_quality_mean;
    int16_t flow_vel_x_peak_to_peak;
    int16_t flow_vel_y_peak_to_peak;
    uint16_t sample_interval_peak_to_peak_us;
    uint32_t distance_peak_to_peak_mm;
    uint8_t height_valid;
    float height_m;
    float height_raw_m;
    float vertical_velocity_m_s;
    float height_filter_alpha;
    float vx_m_s;
    float vy_m_s;
} APP_OPTICAL_FLOW_Status;

void APP_OpticalFlow_Init(void);
void APP_OpticalFlow_Step(void);
void APP_OpticalFlow_ServiceRecovery(void);
uint8_t APP_OpticalFlow_GetVelocity(float *vx_m_s, float *vy_m_s);
uint8_t APP_OpticalFlow_GetVelocitySample(float *vx_m_s,
                                          float *vy_m_s,
                                          uint32_t *sample_ms);
uint8_t APP_OpticalFlow_GetHeightSample(float *height_m,
                                        float *vertical_velocity_m_s,
                                        uint32_t *sample_ms);
void APP_OpticalFlow_GetStatus(APP_OPTICAL_FLOW_Status *status);
void APP_OpticalFlow_Report(void);
const char *APP_OpticalFlow_VelSourceName(APP_OPTICAL_FLOW_VelSource source);
void APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VelSource source);

#ifdef __cplusplus
}
#endif

#endif
