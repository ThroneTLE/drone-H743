#ifndef DRV_NAV_EKF_H
#define DRV_NAV_EKF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * R-F6-1（seam 2 导航口径，只钉现状不改数值）：
 *
 * 本驱动是一个纯 2D 线性卡尔曼滤波器（水平速度 + 加速度计零偏），不知道、
 * 也不定义任何坐标系——`vel_m_s[2]`/`accel_bias_m_s2[2]` 的轴 0/轴 1 语义完全
 * 由调用方（Services/Src/svc_flow_nav.c）决定：predict 吃 X/Y 比力，
 * update 吃同一轴序的光流速度，本驱动只做数值滤波，不做任何旋转或重映射。
 * 不要把这个头文件当作导航坐标系的权威定义；权威定义见
 * Services/Inc/svc_flow_nav.h 与 Driver/Inc/drv_frame_contract.h。
 */
typedef struct {
    float process_accel_noise_m_s2;
    float bias_random_walk_m_s3;
    float flow_noise_m_s;
    float flow_gate_nis;
    float initial_velocity_variance;
    float initial_bias_variance;
    float max_dt_sec;
    float max_velocity_m_s;
    float predict_leak_hz;
} DRV_NAV_EKF_Config;

typedef struct {
    float vel_m_s[2];
    float accel_bias_m_s2[2];
    float covariance[4][4];
    DRV_NAV_EKF_Config config;
    uint8_t initialized;
    uint32_t predict_count;
    uint32_t flow_update_count;
    uint32_t flow_reject_count;
    uint32_t flow_skip_count;
    uint32_t last_flow_sample_ms;
    uint32_t last_flow_update_ms;
    float last_dt_sec;
    float last_flow_noise_m_s;
    float last_innovation_m_s[2];
    float last_nis;
    float last_gate_nis;
} DRV_NAV_EKF_State;

typedef struct {
    float vel_m_s[2];
    float accel_bias_m_s2[2];
    float covariance_diag[4];
    uint32_t predict_count;
    uint32_t flow_update_count;
    uint32_t flow_reject_count;
    uint32_t flow_skip_count;
    uint32_t last_flow_update_ms;
    float last_dt_sec;
    float last_flow_noise_m_s;
    float last_innovation_m_s[2];
    float last_nis;
    float last_gate_nis;
    uint8_t initialized;
} DRV_NAV_EKF_Diagnostics;

void DRV_NAV_EKF_DefaultConfig(DRV_NAV_EKF_Config *config);
void DRV_NAV_EKF_Reset(DRV_NAV_EKF_State *state,
                       const DRV_NAV_EKF_Config *config);
void DRV_NAV_EKF_Predict(DRV_NAV_EKF_State *state,
                         float acc_x_m_s2,
                         float acc_y_m_s2,
                         float dt_sec);
uint8_t DRV_NAV_EKF_FuseFlow(DRV_NAV_EKF_State *state,
                             float flow_vx_m_s,
                             float flow_vy_m_s,
                             uint8_t flow_valid,
                             uint32_t flow_sample_ms,
                             float flow_noise_m_s);
void DRV_NAV_EKF_GetVelocity(const DRV_NAV_EKF_State *state,
                             float *vx_m_s,
                             float *vy_m_s);
void DRV_NAV_EKF_GetDiagnostics(const DRV_NAV_EKF_State *state,
                                DRV_NAV_EKF_Diagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif
