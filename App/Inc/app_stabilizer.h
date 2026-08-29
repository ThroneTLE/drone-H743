/*
 * ============================================================================
 * app_stabilizer.h —— 姿态稳定器公共接口
 * ============================================================================
 * 从 Core/Src/freertos.c 的 StabilizerTask 拆分出的独立模块对外暴露的最小面：
 *   - StabilizerImuFaultReason / StabilizerVofaDebug 跨任务公共类型
 *   - Sensor_Task / VOFA_task 需要的 4 个 API
 *   - APP_Stabilizer_Run() 主循环入口（任务句柄由参数注入）
 */

#ifndef APP_STABILIZER_H
#define APP_STABILIZER_H

#include <stdint.h>
#include "cmsis_os2.h"

  typedef enum
  {
    STABILIZER_IMU_FAULT_NONE = 0U,
    STABILIZER_IMU_FAULT_DRDY_TIMEOUT = 1U,
    STABILIZER_IMU_FAULT_READ_FAIL = 2U,
  } StabilizerImuFaultReason;

  typedef struct {
    float acc_nav_m_s2[3];
    float vel_est_m_s[3];
    float pos_est_m[2];
    float vel_ref_m_s[2];
    float vel_err_m_s[2];
    float vel_pid_out_m_s2[2];
    float vel_pid_p_m_s2[2];
    float vel_pid_i_m_s2[2];
    float vel_pid_d_m_s2[2];
    float servo_alpha_us;
    float servo_beta_us;
    float motor_upper_us;
    float motor_lower_us;
    float nav_accel_lpf_alpha;
    float nav_velocity_leak_hz;
    float vel_loop_active;
    float range_vertical_velocity_m_s;
    float altitude_ref_m;
    float altitude_correction_us;
  } StabilizerVofaDebug;

  /*
   * Read-only validation view after the active sensor-frame correction and
   * Fusion adapter.  `imu_frame_orientation_code` identifies whether vectors
   * are still legacy (0xFF) or use a RAM/Flash FLU candidate (0..23); callers
   * must preserve that provenance instead of guessing from field names.
   */
  typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence;
    uint32_t sample_count; /* Alias of the per-type message sequence. */
    float accel_g[3];
    float gyro_dps[3];
    float temperature_c;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float fusion_acceleration_error_deg;
    float fusion_acceleration_recovery_trigger;
    uint32_t fusion_accel_correction_count;
    uint32_t calibration_generation;
    uint16_t esc_pulse_us[2];
    uint8_t gyro_bias_ready;
    uint8_t fusion_accelerometer_ignored;
    uint8_t fusion_acceleration_recovery;
    uint8_t fusion_angular_rate_recovery;
    uint8_t fusion_accel_norm_rejected;
    uint8_t imu_frame_orientation_code;
    uint8_t calibration_valid_mask;
    uint8_t armed;
    /* 采样链健康：见 app_imu_health.h。上位机据此拒收可疑标定证据。 */
    uint16_t imu_sample_rate_hz;
    uint8_t imu_health_level;
    uint8_t imu_health_fault_active;
    uint8_t imu_health_fault_ever;
  } StabilizerValidationImuSnapshot;

void APP_Stabilizer_LatchImuFault(StabilizerImuFaultReason reason);
void APP_Stabilizer_ClearImuFault(void);
void APP_Stabilizer_MarkImuSample(uint32_t now_ms);
void APP_Stabilizer_ReadVofaDebug(StabilizerVofaDebug *out);
/* Returns 1 only after a real sensor message has traversed the pipeline. */
uint8_t APP_Stabilizer_ReadValidationImuSnapshot(
  StabilizerValidationImuSnapshot *out);
/* Nonzero while a partial FLU migration must remain physically disarmed. */
uint8_t APP_Stabilizer_IsImuFrameArmLocked(void);
/* Independent hard lock while a V1 RAM candidate exists or is committing. */
void APP_Stabilizer_SetImuCalibrationCandidateArmLock(uint8_t locked);
uint8_t APP_Stabilizer_IsImuCalibrationCandidateArmLocked(void);
void APP_Stabilizer_Run(osSemaphoreId_t imu_ready_sem,
                        osMessageQueueId_t sensor_sample_q,
                        osMessageQueueId_t vofa_log_q);

#endif /* APP_STABILIZER_H */
