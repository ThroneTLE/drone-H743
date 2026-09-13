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
    float acc_nav_m_s2[2];
    float vel_est_m_s[2];
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

  /*
   * One flow sample after the runtime rotation/lever-arm compensation seam.
   * Export vectors are converted from the current controller Y-right adapter
   * to canonical FLU Y-left and carry orientation provenance explicitly.
   */
  typedef struct {
    uint32_t sample_ms;
    uint16_t frame_contract;
    uint8_t orientation_code;
    uint8_t valid;
    float sensor_velocity_flu_m_s[2];
    float optical_rot_comp_flu_m_s[2];
    float offset_rot_comp_flu_m_s[2];
    float corrected_velocity_flu_m_s[2];
  } StabilizerFlowCompensationSnapshot;

void APP_Stabilizer_LatchImuFault(StabilizerImuFaultReason reason);
void APP_Stabilizer_ClearImuFault(void);
void APP_Stabilizer_MarkImuSample(uint32_t now_ms);
void APP_Stabilizer_ReadVofaDebug(StabilizerVofaDebug *out);
/* Returns 1 only after a real sensor message has traversed the pipeline. */
uint8_t APP_Stabilizer_ReadValidationImuSnapshot(
  StabilizerValidationImuSnapshot *out);
uint8_t APP_Stabilizer_ReadFlowCompensationSnapshot(
  StabilizerFlowCompensationSnapshot *out);
/* Nonzero while a partial FLU migration must remain physically disarmed. */
uint8_t APP_Stabilizer_IsImuFrameArmLocked(void);
/* Latest armed state as seen by the control loop; used to gate config writes. */
uint8_t APP_Stabilizer_IsArmed(void);

/*
 * 解锁状态快照，供上位机在主页面显眼处显示"能不能解锁、为什么不能"。
 *
 * 为什么要把每一个条件都单独带出来，而不是只报一个原因码：原因链是**有序**的
 * （见 stabilizer_control_prepare 末尾），它只能报出第一条不满足的。现场常见
 * 的是同时缺两样——比如既没插遥控器又没写机体模型——只看第一条会让人修完
 * 一个还是解不了锁，以为没修对。把条件全带出来，上位机就能一次性列清单。
 *
 * 所有字段都是"1 = 这一项满足/通过"，block_reason 是 APP_LED_ArmBlockReason。
 */
typedef struct {
  uint8_t  armed;
  uint8_t  block_reason;
  uint8_t  rc_link_seen;
  uint8_t  rc_link_ok;
  uint8_t  arm_switch_high;
  uint8_t  throttle_low;
  uint8_t  imu_control_valid;
  uint8_t  imu_health_ok;
  uint8_t  frame_migration_ok;
  uint8_t  airframe_valid;
  uint8_t  servo_cal_idle;
  uint8_t  acceptance_idle;
  uint8_t  published;          /* 0 = 控制环还没跑过一圈，下面全是占位零值 */
  uint32_t now_ms;
  uint8_t battery_ok; /* pre-arm only; low voltage never clears an armed latch */
} APP_Stabilizer_ArmStatus;

void APP_Stabilizer_GetArmStatus(APP_Stabilizer_ArmStatus *out);
/* Independent hard lock while a V1 RAM candidate exists or is committing. */
void APP_Stabilizer_SetImuCalibrationCandidateArmLock(uint8_t locked);
uint8_t APP_Stabilizer_IsImuCalibrationCandidateArmLocked(void);
/* Independent hard lock while servo-mechanical FCAL preview is active. */
void APP_Stabilizer_SetServoCalibrationCandidateArmLock(uint8_t locked);
uint8_t APP_Stabilizer_IsServoCalibrationCandidateArmLocked(void);
void APP_Stabilizer_Run(osSemaphoreId_t imu_ready_sem,
                        osMessageQueueId_t sensor_sample_q,
                        osMessageQueueId_t vofa_log_q);

#endif /* APP_STABILIZER_H */
