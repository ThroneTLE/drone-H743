/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
 * ============================================================================
 * 模块依赖说明
 * ============================================================================
 * App 层（应用逻辑）：
 *   app_diag.h       — 诊断记录（栈溢出 / malloc 失败时记录到 FLASH）
 *   app_sensor.h     — 传感器数据处理（零偏校准、坐标系对齐、低通滤波）
 *   app_messages.h   — 消息协议编解码（JSON / 二进制帧封装）
 *   app_tasks.h      — 各任务 Init/Step 函数声明
 *   app_vofa.h       — VOFA 上位机通信（浮点数组帧发送）
 *   app_elrs.h       — ELRS 遥控器链路（通道值读取）
 *
 * BSP 层（板级支持包——硬件抽象）：
 *   bsp_baro.h       — 气压计 SPL06-007 驱动
 *   bsp_imu.h        — IMU (ICM-42688-P) 驱动
 *   bsp_pwm.h        — TIM2 PWM 输出（CH1/2 电调，CH3/4 普通舵机）
 *   bsp_aiwb2_power.h — Ai-WB2 WiFi 模块电源控制
 *
 * Driver 层（外设驱动——算法 / 协议）：
 *   drv_coax_ctrl.h  — 同轴倾转旋翼控制器（论文控制分配）
 */
#include "app_diag.h"
#include "app_flight_log.h"
#include "app_led.h"
#include "app_nav_estimator.h"
#include "app_sensor.h"
#include "app_messages.h"
#include "app_tasks.h"
#include "app_ident.h"
#include "app_servo_cal.h"
#include <math.h>
#include <string.h>

#include "app_vofa.h"
#include "app_elrs.h"
#include "app_optical_flow.h"
#include "bsp_baro.h"
#include "bsp_imu.h"
#include "bsp_bus_servo.h"
#include "bsp_pwm.h"
#include "bsp_aiwb2_power.h"
#include "drv_airframe_model.h"
#include "drv_coax_ctrl.h"
#include "drv_imu_nav.h"
#include "drv_nav_ekf.h"

#define STABILIZER_SERVO_MOVE_TIME_MS 0U
#define STABILIZER_NAV_ACCEL_LPF_ALPHA 0.94f
#define STABILIZER_NAV_VEL_LEAK_HZ 0.25f
#define STABILIZER_NAV_EKF_FLOW_NOISE_M_S 0.25f
#define STABILIZER_NAV_EKF_IMU_BRIDGE_TIMEOUT_MS 80U
#define STABILIZER_NAV_EKF_FLOW_LOST_DECAY_HZ 8.0f
#define STABILIZER_NAV_EKF_CONTROL_TIMEOUT_MS 150U
#define STABILIZER_NAV_EKF_CONTROL_MAX_SPEED_M_S 1.50f

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * ============================================================================
 * 传感器相关常量
 * ============================================================================
 * IMU 数据就绪优先由 PC0 外部中断触发，中断中设置 Thread Flag。
 * 若短时间未等到中断，只读取一次 IMU ready 状态作为兜底；坏帧/短暂丢帧会被跳过。
 */
#define SENSOR_IMU_DATA_READY_FLAG     0x0001U  /* 位 0：IMU 数据就绪事件标志          */
#define SENSOR_IMU_DEFAULT_DT_SEC      0.001f   /* 默认 IMU 采样间隔 1ms → dt = 0.001s */
#define SENSOR_IMU_DRDY_TIMEOUT_MS     20U      /* 单次等待 DRDY 的超时时间            */
#define SENSOR_IMU_DRDY_MISS_FAULT_LIMIT 50U    /* 连续 1s 无 DRDY/ready 才锁存故障    */
#define SENSOR_IMU_READ_FAIL_LIMIT     25U      /* 连续读失败次数，超过后锁存故障      */
#define SENSOR_MAG_PERIOD_US           50000ULL /* 磁力计步进间隔 50ms = 20Hz          */
#define STABILIZER_USE_FIXED_IMU_DT    0U       /* 1=固定 1ms dt, 0=时间戳 dt          */

/*
 * ============================================================================
 * 姿态稳定器常量
 * ============================================================================
 * 控制周期 20ms（50Hz）—— 舵机是慢速设备，不需要和 IMU 1kHz 同步。
 *
 * 舵机模式选择：
 *   STABILIZER_USE_DIRECT_ANGLE_SERVO = 1 → 角度直驱（舵机调试）
 *         姿态角直接映射为舵机脉宽，不经过同轴控制器。
 *         先得到机体系 X前/Y右 倾角，再由驱动层补偿舵机座逆时针 90° 安装。
 *   STABILIZER_USE_DIRECT_ANGLE_SERVO = 0 → 同轴控制器（论文控制分配）
 *         经过完整的同轴倾转旋翼控制律，RC 遥控器参与参考输入。
 */
#define STABILIZER_CONTROL_PERIOD_MS   2U       /* 控制输出周期 2ms = 500Hz            */
#define STABILIZER_IMU_STALE_MS        50U      /* 超过此年龄后不再用旧姿态算新舵机   */
#define STABILIZER_PI                  3.141592654f
#define STABILIZER_DEG_TO_RAD          0.0174532925f /* 度 → 弧度  (π/180)            */
#define STABILIZER_USE_DIRECT_ANGLE_SERVO 0U     /* 1=角度直驱舵机, 0=同轴控制器       */
#define STABILIZER_DIRECT_BODY_X_SIGN   (1.0f)   /* 前后/X 轴直驱方向符号              */
#define STABILIZER_DIRECT_BODY_Y_SIGN   (1.0f)   /* 左右/Y 轴直驱方向符号              */
#define STABILIZER_YAW_RATE_REF_MAX_RAD_S 1.04719758f /* CH4 偏航参考累加最大速率 [rad/s] */
#define STABILIZER_XY_VEL_REF_MAX_M_S  0.80f     /* CH1/CH2 水平速度目标最大值 [m/s]    */
#define STABILIZER_XY_POS_ERR_MAX_M    0.35f     /* 速度意图积分后的虚拟位置误差限幅 [m] */
#define STABILIZER_Z_REF_STICK_SPAN_M  0.30f     /* CH3 相对 50% 油门的高度目标跨度 [m]  */
#define STABILIZER_Z_REF_MAX_M         0.30f     /* 上电光流测高基准以上高度上限 [m]     */
#define STABILIZER_Z_POS_ERR_MAX_M     0.35f     /* Z 位置 PID 单次位置误差限幅 [m]      */
#define STABILIZER_Z_THRUST_BIAS_MAX_M_S2 8.63f  /* CH3 100% 额外向上推力偏置 [m/s^2]   */
#define STABILIZER_RC_DEADBAND_US      20        /* RC 摇杆死区 [μs]，中位 1500±20      */

/*
 * ============================================================================
 * ELRS / CRSF 遥控器通道约定
 * ============================================================================
 * 这个表就是本机遥控器映射的唯一维护入口，后续不要再靠口头记忆：
 *
 *   CH1 → 左右 / roll stick      → 自稳定速度意图 / 控制器 y_ref，右为正，回中 0
 *   CH2 → 前后 / pitch stick     → 自稳定速度意图 / 控制器 x_ref，前为正，回中 0
 *   CH3 → 左摇杆上下 / throttle  → 低段直通油门；稳定段设定激光定高目标，50% 保持当前高度，高于 50% 提高目标高度
 *   CH4 → 偏航 / yaw stick       → 中位保持，高于中位累加 yaw_ref，低于中位减少 yaw_ref
 *   CH5 → 二值开关 / arm switch  → +100=开锁，-100=关锁
 *
 * CRSF 驱动输出的是 16 路 us 值，数组下标从 0 开始，所以 CH1 对应 ch[0]。
 * CH5 用阈值判断：>1500us 视为开锁，<=1500us 视为上锁。
 * 解锁还必须满足 CH3 低油门：CH3 <=1100us。防止开关误触后电机带油门启动。
 */
#define STABILIZER_RC_CH_ROLL          0U
#define STABILIZER_RC_CH_PITCH         1U
#define STABILIZER_RC_CH_THROTTLE_Z    2U
#define STABILIZER_RC_CH_YAW           3U
#define STABILIZER_RC_CH_ARM           4U
#define STABILIZER_RC_ARM_THRESHOLD_US 1500U
#define STABILIZER_RC_THROTTLE_INPUT_LOW_US  1000U
#define STABILIZER_RC_THROTTLE_INPUT_HIGH_US 2000U
#define STABILIZER_RC_THROTTLE_ARM_LOW_US    1100U
#define STABILIZER_RC_STABILIZE_MIN_PERCENT 20U
#define STABILIZER_RC_STABILIZE_MIN_US \
  (STABILIZER_RC_THROTTLE_INPUT_LOW_US + \
   (((STABILIZER_RC_THROTTLE_INPUT_HIGH_US - STABILIZER_RC_THROTTLE_INPUT_LOW_US) * \
     STABILIZER_RC_STABILIZE_MIN_PERCENT) / 100U))
#define STABILIZER_RC_LOSS_TIMEOUT_MS  500U
#define STABILIZER_FLIGHT_LOG_TAIL_RECORDS 125U /* 250 Hz log tail, about 500 ms */
#define STABILIZER_USE_RC_DIRECT_TILT_SERVO 0U   /* 0=自稳定控制器, 1=CH1/CH2 直控舵机调试 */
#define STABILIZER_RC_DIRECT_TILT_LIMIT_RAD 0.314159265f /* 遥控直控调试最大 ±18° */

/*
 * ============================================================================
 * 舵机通信常量
 * ============================================================================
 * 舵机通过 UART7 半双工总线控制，每次移动命令指定目标脉宽和到位时间。
 * 为避免总线拥塞，只有脉宽变化超过阈值或超过强制刷新间隔才发送。
 */
#define STABILIZER_SERVO_REFRESH_MS    500U      /* 强制刷新间隔 [ms]（即使脉宽未变）    */
#define STABILIZER_SERVO_DELTA_US      3U        /* 脉宽变化死区 [μs]（小于此值不发送）  */
#define STABILIZER_ATTITUDE_ZERO_MS 1500U        /* 上电后姿态零偏采集时长 [ms]          */
#define VOFA_SEND_PERIOD_MS            25U       /* 57600 数传下 22-float VOFA 约 40Hz */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/*
 * ============================================================================
 * 全局同步对象
 * ============================================================================
 * imuDataReadySemaphore — IMU 数据就绪信号量
 *   生产者：Sensor_Task（每次 IMU 采样完成后 release）
 *   消费者：StabilizerTask（acquire 后开始消费 SensorSampleQueue）
 *   初始值 0，最大值 1（二进制信号量，只做事件通知不做计数）
 *
 * vofaStreamActive — VOFA 数据流开关
 *   由上位机通过消息系统远程控制，置 0 时暂停 VOFA 发送
 */
osSemaphoreId_t imuDataReadySemaphore;
volatile uint8_t vofaStreamActive = 0U;  /* 默认关闭，发 Sensor_Data:1 开启 */

typedef enum
{
  STABILIZER_IMU_FAULT_NONE = 0U,
  STABILIZER_IMU_FAULT_DRDY_TIMEOUT = 1U,
  STABILIZER_IMU_FAULT_READ_FAIL = 2U,
} StabilizerImuFaultReason;

static volatile uint8_t stabilizer_imu_fault_latched = 0U;
static volatile StabilizerImuFaultReason stabilizer_imu_fault_reason =
  STABILIZER_IMU_FAULT_NONE;
static volatile uint32_t stabilizer_imu_fault_count = 0U;
static volatile uint32_t stabilizer_imu_last_sample_ms = 0U;
static uint8_t stabilizer_rc_arm_latched = 0U;
static uint8_t stabilizer_rc_switch_seen_low = 0U;
static uint8_t stabilizer_rc_switch_prev_high = 0U;

static void stabilizer_latch_imu_fault(StabilizerImuFaultReason reason)
{
  if (stabilizer_imu_fault_latched == 0U) {
    stabilizer_imu_fault_reason = reason;
  }
  stabilizer_imu_fault_latched = 1U;
  stabilizer_imu_fault_count++;
}

static void stabilizer_clear_imu_fault(void)
{
  stabilizer_imu_fault_latched = 0U;
  stabilizer_imu_fault_reason = STABILIZER_IMU_FAULT_NONE;
}

/*
 * ============================================================================
 * 稳定器辅助函数
 * ============================================================================
 * 以下 3 个 static 函数仅供 StabilizerTask 内部使用，不暴露到头文件。
 */

/*
 * stabilizer_rc_normalized() — RC 通道值归一化到 [-1, +1]
 *   输入：RC 脉宽 [μs]，典型范围 1000~2000，中位 1500
 *   处理：减去中位 1500 → 死区过滤 → 限幅 ±500 → 除以 500
 *   仅在 USE_DIRECT_ANGLE_SERVO=0（同轴控制器模式）时编译
 */
#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
static float stabilizer_rc_normalized(uint16_t ch_us)
{
  int32_t centered = (int32_t)ch_us - 1500;

  if ((centered > -STABILIZER_RC_DEADBAND_US) &&
      (centered < STABILIZER_RC_DEADBAND_US)) {
    return 0.0f;
  }
  if (centered > 500) { centered = 500; }
  if (centered < -500) { centered = -500; }

  return (float)centered / 500.0f;
}

static float stabilizer_rc_throttle_01(uint16_t ch_us)
{
  const int32_t span =
    (int32_t)STABILIZER_RC_THROTTLE_INPUT_HIGH_US -
    (int32_t)STABILIZER_RC_THROTTLE_INPUT_LOW_US;
  int32_t value = (int32_t)ch_us - (int32_t)STABILIZER_RC_THROTTLE_INPUT_LOW_US;

  if (value < 0) { value = 0; }
  if (value > span) { value = span; }

  return (float)value / (float)span;
}

static float stabilizer_rc_throttle_height_offset_m(uint16_t ch_us)
{
  return stabilizer_rc_normalized(ch_us) * STABILIZER_Z_REF_STICK_SPAN_M;
}

static float stabilizer_rc_throttle_thrust_bias_m_s2(uint16_t ch_us)
{
  return stabilizer_rc_normalized(ch_us) * STABILIZER_Z_THRUST_BIAS_MAX_M_S2;
}

static float stabilizer_wrap_pi(float angle_rad)
{
  while (angle_rad > STABILIZER_PI) {
    angle_rad -= 2.0f * STABILIZER_PI;
  }
  while (angle_rad < -STABILIZER_PI) {
    angle_rad += 2.0f * STABILIZER_PI;
  }
  return angle_rad;
}

static float stabilizer_rc_yaw_rate_rad_s(uint16_t ch_us)
{
  return stabilizer_rc_normalized(ch_us) * STABILIZER_YAW_RATE_REF_MAX_RAD_S;
}

static uint16_t stabilizer_motor_pulse_clamp(int32_t pulse_us)
{
  if (pulse_us < (int32_t)BSP_PWM_ESC_MIN_US) {
    return BSP_PWM_ESC_MIN_US;
  }
  if (pulse_us > (int32_t)BSP_PWM_ESC_MAX_US) {
    return BSP_PWM_ESC_MAX_US;
  }
  return (uint16_t)pulse_us;
}

static uint16_t stabilizer_rc_throttle_to_motor_pulse(uint16_t ch_us)
{
  float throttle_01 = stabilizer_rc_throttle_01(ch_us);
  float pulse_f = (float)BSP_PWM_ESC_MIN_US +
                  throttle_01 * (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);
  int32_t pulse_i = (int32_t)(pulse_f + 0.5f);

  return stabilizer_motor_pulse_clamp(pulse_i);
}

static uint8_t stabilizer_rc_use_stabilized_motor_mix(uint16_t ch_us)
{
  return (ch_us >= STABILIZER_RC_STABILIZE_MIN_US) ? 1U : 0U;
}

static float stabilizer_clamp_f32(float value, float lo, float hi)
{
  if (value < lo) { return lo; }
  if (value > hi) { return hi; }
  return value;
}

static uint8_t stabilizer_rc_update_armed(const uint16_t ch[CRSF_CHANNEL_COUNT],
                                          uint8_t rc_link_ok)
{
  uint8_t switch_high =
    (ch[STABILIZER_RC_CH_ARM] > STABILIZER_RC_ARM_THRESHOLD_US) ? 1U : 0U;
  uint8_t throttle_low =
    (ch[STABILIZER_RC_CH_THROTTLE_Z] <= STABILIZER_RC_THROTTLE_ARM_LOW_US) ? 1U : 0U;

  if (rc_link_ok == 0U) {
    stabilizer_rc_arm_latched = 0U;
    stabilizer_rc_switch_seen_low = 0U;
    stabilizer_rc_switch_prev_high = 0U;
    return 0U;
  }

  if (switch_high == 0U) {
    stabilizer_rc_arm_latched = 0U;
    stabilizer_rc_switch_seen_low = 1U;
    stabilizer_rc_switch_prev_high = 0U;
    return 0U;
  }

  /* Arm only on a low-to-high switch edge while throttle is low. */
  if ((stabilizer_rc_arm_latched == 0U) &&
      (stabilizer_rc_switch_seen_low != 0U) &&
      (stabilizer_rc_switch_prev_high == 0U) &&
      (throttle_low != 0U)) {
    stabilizer_rc_arm_latched = 1U;
  }

  stabilizer_rc_switch_prev_high = 1U;
  return stabilizer_rc_arm_latched;
}

#if (STABILIZER_USE_RC_DIRECT_TILT_SERVO != 0U)
static void stabilizer_map_rc_direct_to_servo(const uint16_t ch[CRSF_CHANNEL_COUNT],
                                              DRV_SERVO_MoveCmd moves[2])
{
  float body_x_tilt_rad =
    stabilizer_rc_normalized(ch[STABILIZER_RC_CH_PITCH]) *
    STABILIZER_RC_DIRECT_TILT_LIMIT_RAD *
    STABILIZER_DIRECT_BODY_X_SIGN;
  float body_y_tilt_rad =
    stabilizer_rc_normalized(ch[STABILIZER_RC_CH_ROLL]) *
    STABILIZER_RC_DIRECT_TILT_LIMIT_RAD *
    STABILIZER_DIRECT_BODY_Y_SIGN;

  DRV_COAX_CTRL_BodyTiltRadToServoPulses(body_x_tilt_rad,
                                         body_y_tilt_rad,
                                         &moves[0].pulse_us,
                                         &moves[1].pulse_us);
}
#endif
#endif

/*
 * stabilizer_map_angle_direct_to_servo() — 角度直驱模式：姿态角 → 舵机脉宽
 *   先得到机体系 X前/Y右 倾角，再由 DRV_COAX_CTRL_BodyTiltRadToServoPulses()
 *   补偿当前舵机座逆时针 90° 安装：1号舵机=左右，2号舵机=前后。
 */
#if (STABILIZER_USE_DIRECT_ANGLE_SERVO != 0U)
static void stabilizer_map_angle_direct_to_servo(float roll_deg,
                                                 float pitch_deg,
                                                 DRV_SERVO_MoveCmd moves[2])
{
  float body_x_tilt_rad = pitch_deg * STABILIZER_DEG_TO_RAD *
                          STABILIZER_DIRECT_BODY_X_SIGN;
  float body_y_tilt_rad = roll_deg * STABILIZER_DEG_TO_RAD *
                          STABILIZER_DIRECT_BODY_Y_SIGN;

  DRV_COAX_CTRL_BodyTiltRadToServoPulses(body_x_tilt_rad,
                                         body_y_tilt_rad,
                                         &moves[0].pulse_us,
                                         &moves[1].pulse_us);
}
#endif

/*
 * stabilizer_servo_should_send() — 判断是否需要向舵机发送新指令
 *   返回非零的条件（满足任一即发送）：
 *     1. 任一舵机目标脉宽与上次发送值相差 ≥ SERVO_DELTA_US (3μs)
 *     2. 距离上次发送超过 SERVO_REFRESH_MS (500ms) 强制刷新
 *   目的：避免在脉宽几乎不变时反复占用舵机总线
 */
static volatile uint16_t stabilizer_latest_servo_target_us[2] = {
  DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US,
  DRV_COAX_CTRL_SERVO_BETA_CENTER_US,
};
static volatile uint16_t stabilizer_last_sent_servo_pulse_us[2] = {
  DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US,
  DRV_COAX_CTRL_SERVO_BETA_CENTER_US,
};
static uint32_t stabilizer_last_servo_send_ms;

typedef struct {
  float acc_nav_m_s2[3];
  float vel_est_m_s[3];
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
  float vel_loop_x_kp;
  float vel_loop_x_ki;
  float vel_loop_x_kd;
  float vel_loop_y_kp;
  float vel_loop_y_ki;
  float vel_loop_y_kd;
  float nav_accel_lpf_alpha;
  float nav_velocity_leak_hz;
  float vel_loop_active;
  float range_vertical_velocity_m_s;
  float altitude_ref_m;
  float altitude_correction_us;
} StabilizerVofaDebug;

typedef struct {
  float integrator_m_s2;
  float prev_meas_m_s;
  uint8_t prev_valid;
} StabilizerVelocityPidState;

typedef struct {
  float vel_m_s[2];
  DRV_NAV_EKF_State ekf;
  DRV_NAV_EKF_Diagnostics diagnostics;
} StabilizerVelocityEstimatorState;

static StabilizerVofaDebug stabilizer_vofa_debug;

static void stabilizer_velocity_estimator_reset(StabilizerVelocityEstimatorState *state)
{
  DRV_NAV_EKF_Config config;

  if (state == NULL) {
    return;
  }

  DRV_NAV_EKF_DefaultConfig(&config);
  config.flow_noise_m_s = STABILIZER_NAV_EKF_FLOW_NOISE_M_S;
  DRV_NAV_EKF_Reset(&state->ekf, &config);
  DRV_NAV_EKF_GetDiagnostics(&state->ekf, &state->diagnostics);
  APP_NavEstimator_PublishVelocityEKF(&state->diagnostics);
  state->vel_m_s[0] = state->diagnostics.vel_m_s[0];
  state->vel_m_s[1] = state->diagnostics.vel_m_s[1];
}

static uint8_t stabilizer_velocity_estimator_step(StabilizerVelocityEstimatorState *state,
                                                  float acc_x_m_s2,
                                                  float acc_y_m_s2,
                                                  float imu_vx_m_s,
                                                  float imu_vy_m_s,
                                                  float flow_vx_m_s,
                                                  float flow_vy_m_s,
                                                  uint8_t flow_valid,
                                                  uint32_t flow_sample_ms,
                                                  float dt_sec)
{
  uint8_t flow_accepted;
  uint8_t imu_bridge_ok = 0U;
  uint32_t now_ms = HAL_GetTick();

  if (state == NULL) {
    return 0U;
  }

  if (dt_sec <= 0.0f) {
    dt_sec = SENSOR_IMU_DEFAULT_DT_SEC;
  }

  (void)imu_vx_m_s;
  (void)imu_vy_m_s;
  if ((state->diagnostics.flow_update_count != 0U) &&
      (state->diagnostics.last_flow_update_ms != 0U) &&
      ((now_ms - state->diagnostics.last_flow_update_ms) <=
       STABILIZER_NAV_EKF_IMU_BRIDGE_TIMEOUT_MS)) {
    imu_bridge_ok = 1U;
  }

  if (imu_bridge_ok != 0U) {
    DRV_NAV_EKF_Predict(&state->ekf, acc_x_m_s2, acc_y_m_s2, dt_sec);
  } else {
    float decay = 1.0f - (STABILIZER_NAV_EKF_FLOW_LOST_DECAY_HZ * dt_sec);
    decay = stabilizer_clamp_f32(decay, 0.0f, 1.0f);
    state->ekf.vel_m_s[0] *= decay;
    state->ekf.vel_m_s[1] *= decay;
    state->ekf.accel_bias_m_s2[0] = 0.0f;
    state->ekf.accel_bias_m_s2[1] = 0.0f;
  }
  flow_accepted = DRV_NAV_EKF_FuseFlow(&state->ekf,
                                       flow_vx_m_s,
                                       flow_vy_m_s,
                                       flow_valid,
                                       flow_sample_ms,
                                       STABILIZER_NAV_EKF_FLOW_NOISE_M_S);
  DRV_NAV_EKF_GetDiagnostics(&state->ekf, &state->diagnostics);
  APP_NavEstimator_PublishVelocityEKF(&state->diagnostics);
  state->vel_m_s[0] = state->diagnostics.vel_m_s[0];
  state->vel_m_s[1] = state->diagnostics.vel_m_s[1];
  return flow_accepted;
}

static uint8_t stabilizer_velocity_estimator_control_ok(
  const StabilizerVelocityEstimatorState *state,
  uint32_t now_ms)
{
  float vx;
  float vy;

  if ((state == NULL) ||
      (state->diagnostics.initialized == 0U) ||
      (state->diagnostics.flow_update_count == 0U) ||
      (state->diagnostics.last_flow_update_ms == 0U) ||
      ((now_ms - state->diagnostics.last_flow_update_ms) >
       STABILIZER_NAV_EKF_CONTROL_TIMEOUT_MS)) {
    return 0U;
  }

  vx = state->diagnostics.vel_m_s[0];
  vy = state->diagnostics.vel_m_s[1];
  if ((fabsf(vx) > STABILIZER_NAV_EKF_CONTROL_MAX_SPEED_M_S) ||
      (fabsf(vy) > STABILIZER_NAV_EKF_CONTROL_MAX_SPEED_M_S)) {
    return 0U;
  }

  return 1U;
}

static void stabilizer_velocity_pid_reset(StabilizerVelocityPidState *state)
{
  if (state == NULL) {
    return;
  }

  state->integrator_m_s2 = 0.0f;
  state->prev_meas_m_s = 0.0f;
  state->prev_valid = 0U;
}

static float stabilizer_velocity_pid_step(StabilizerVelocityPidState *state,
                                          float err_m_s,
                                          float meas_m_s,
                                          float kp,
                                          float ki,
                                          float kd,
                                          float dt_sec,
                                          float *p_term,
                                          float *i_term,
                                          float *d_term)
{
  float p = kp * err_m_s;
  float d = 0.0f;
  float out;

  if (state == NULL) {
    return 0.0f;
  }

  if (dt_sec <= 0.0f) {
    dt_sec = (float)STABILIZER_CONTROL_PERIOD_MS * 0.001f;
  }

  if (state->prev_valid != 0U) {
    d = -kd * (meas_m_s - state->prev_meas_m_s) / dt_sec;
  }

  state->integrator_m_s2 += ki * err_m_s * dt_sec;
  state->prev_meas_m_s = meas_m_s;
  state->prev_valid = 1U;

  out = p + state->integrator_m_s2 + d;

  if (p_term != NULL) { *p_term = p; }
  if (i_term != NULL) { *i_term = state->integrator_m_s2; }
  if (d_term != NULL) { *d_term = d; }

  return out;
}

static void stabilizer_vofa_debug_publish(const StabilizerVofaDebug *debug)
{
  if (debug == NULL) {
    return;
  }

  stabilizer_vofa_debug = *debug;
}

static void stabilizer_vofa_debug_read(StabilizerVofaDebug *debug)
{
  if (debug == NULL) {
    return;
  }

  *debug = stabilizer_vofa_debug;
}

static uint8_t stabilizer_servo_should_send(const DRV_SERVO_MoveCmd moves[2],
                                            uint32_t now_ms)
{
  uint8_t should_send = 0U;

  for (uint32_t i = 0U; i < 2U; ++i) {
    uint16_t previous = stabilizer_last_sent_servo_pulse_us[i];
    uint16_t current = moves[i].pulse_us;
    uint16_t delta = (current >= previous) ? (uint16_t)(current - previous)
                                           : (uint16_t)(previous - current);

    if (delta >= STABILIZER_SERVO_DELTA_US) {
      should_send = 1U;
    }
  }

  if ((now_ms - stabilizer_last_servo_send_ms) >= STABILIZER_SERVO_REFRESH_MS) {
    should_send = 1U;
  }

  return should_send;
}

static void stabilizer_servo_commit_sent(const DRV_SERVO_MoveCmd moves[2],
                                         uint32_t now_ms)
{
  stabilizer_last_sent_servo_pulse_us[0] = moves[0].pulse_us;
  stabilizer_last_sent_servo_pulse_us[1] = moves[1].pulse_us;
  stabilizer_last_servo_send_ms = now_ms;
}

static void stabilizer_servo_record_target(const DRV_SERVO_MoveCmd moves[2])
{
  stabilizer_latest_servo_target_us[0] = moves[0].pulse_us;
  stabilizer_latest_servo_target_us[1] = moves[1].pulse_us;
}

/* USER CODE END Variables */
/* Definitions for Stabilizer */
osThreadId_t StabilizerHandle;
const osThreadAttr_t Stabilizer_attributes = {
  .name = "Stabilizer",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for messageTask */
osThreadId_t messageTaskHandle;
const osThreadAttr_t messageTask_attributes = {
  .name = "messageTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for backgroundTask */
osThreadId_t backgroundTaskHandle;
const osThreadAttr_t backgroundTask_attributes = {
  .name = "backgroundTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for VOFA_Task */
osThreadId_t VOFA_TaskHandle;
const osThreadAttr_t VOFA_Task_attributes = {
  .name = "VOFA_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartTxQueue */
osMessageQueueId_t uartTxQueueHandle;
const osMessageQueueAttr_t uartTxQueue_attributes = {
  .name = "uartTxQueue"
};
/* Definitions for SensorSampleQueue */
osMessageQueueId_t SensorSampleQueueHandle;
const osMessageQueueAttr_t SensorSampleQueue_attributes = {
  .name = "SensorSampleQueue"
};
/* Definitions for backgroundReqQueue */
osMessageQueueId_t backgroundReqQueueHandle;
const osMessageQueueAttr_t backgroundReqQueue_attributes = {
  .name = "backgroundReqQueue"
};
/* Definitions for backgroundRespQueue */
osMessageQueueId_t backgroundRespQueueHandle;
const osMessageQueueAttr_t backgroundRespQueue_attributes = {
  .name = "backgroundRespQueue"
};
/* Definitions for vofaLogQueue */
osMessageQueueId_t vofaLogQueueHandle;
const osMessageQueueAttr_t vofaLogQueue_attributes = {
  .name = "vofaLogQueue"
};
/* Definitions for flashBusMutex */
osMutexId_t flashBusMutexHandle;
const osMutexAttr_t flashBusMutex_attributes = {
  .name = "flashBusMutex",
  .attr_bits = osMutexRecursive,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StabilizerTask(void *argument);
void Sensor_Task(void *argument);
void message_push(void *argument);
void UART_fun(void *argument);
void BackgroundTask(void *argument);
void VOFA_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
/*
 * vApplicationStackOverflowHook — 任务栈溢出钩子
 *
 * 触发条件：configCHECK_FOR_STACK_OVERFLOW 设置为 1 或 2 时，
 * FreeRTOS 在每个任务切换时检查栈指针是否越界。
 *
 * 处理策略：
 *   1. 记录栈溢出事件到诊断系统（APP_Diag_RecordStackOverflow）
 *   2. 关全局中断（taskDISABLE_INTERRUPTS）
 *   3. 死循环——栈溢出是不可恢复的错误，继续运行会导致内存损坏
 *
 * 排查方法：如果某个任务反复触发此钩子，增大该任务的 .stack_size，
 * 或检查函数内的局部变量是否过大（大数组应改为 static）。
 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   (void)xTask;
   APP_Diag_RecordStackOverflow(pcTaskName);
   taskDISABLE_INTERRUPTS();
   for(;;)
   {
   }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
/*
 * vApplicationMallocFailedHook — 动态内存分配失败钩子
 *
 * 触发条件：configUSE_MALLOC_FAILED_HOOK = 1 时，
 * pvPortMalloc() 返回 NULL（堆内存耗尽）时调用。
 *
 * FreeRTOS 内部在创建任务/队列/信号量/定时器时都会调用 pvPortMalloc()。
 * 堆大小由 FreeRTOSConfig.h 中的 configTOTAL_HEAP_SIZE 定义。
 *
 * 处理策略：与栈溢出相同——记录诊断信息后死循环。
 *
 * 排查方法：检查 configTOTAL_HEAP_SIZE 是否足够；
 * 检查是否有内存泄漏（创建对象后未删除）；
 * 使用 xPortGetFreeHeapSize() 监控剩余堆空间。
 */
void vApplicationMallocFailedHook(void)
{
   APP_Diag_RecordMallocFailed();
   taskDISABLE_INTERRUPTS();
   for(;;)
   {
   }
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Create the recursive mutex(es) */
  /* creation of flashBusMutex */
  flashBusMutexHandle = osMutexNew(&flashBusMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 创建 IMU 数据就绪信号量：初始 0，最大 1，二进制事件通知 */
  imuDataReadySemaphore = osSemaphoreNew(1U, 0U, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uartTxQueue */
  uartTxQueueHandle = osMessageQueueNew (32, sizeof(APP_UART_TxMessage), &uartTxQueue_attributes);

  /* creation of SensorSampleQueue */
  SensorSampleQueueHandle = osMessageQueueNew (8, sizeof(APP_Sensor_SampleMessage), &SensorSampleQueue_attributes);

  /* creation of backgroundReqQueue */
  backgroundReqQueueHandle = osMessageQueueNew (8, sizeof(APP_BackgroundRequest), &backgroundReqQueue_attributes);

  /* creation of backgroundRespQueue */
  backgroundRespQueueHandle = osMessageQueueNew (8, sizeof(APP_BackgroundResponse), &backgroundRespQueue_attributes);

  /* creation of vofaLogQueue */
  vofaLogQueueHandle = osMessageQueueNew (1, sizeof(APP_Sensor_SampleMessage), &vofaLogQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  APP_Task_LED_Init();
  APP_ServoCal_Init();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Stabilizer */
  StabilizerHandle = osThreadNew(StabilizerTask, NULL, &Stabilizer_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(Sensor_Task, NULL, &SensorTask_attributes);

  /* creation of messageTask */
  messageTaskHandle = osThreadNew(message_push, NULL, &messageTask_attributes);

  /* creation of UARTTask */
  UARTTaskHandle = osThreadNew(UART_fun, NULL, &UARTTask_attributes);

  /* creation of backgroundTask */
  backgroundTaskHandle = osThreadNew(BackgroundTask, NULL, &backgroundTask_attributes);

  /* creation of VOFA_Task */
  VOFA_TaskHandle = osThreadNew(VOFA_task, NULL, &VOFA_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StabilizerTask */
/**
  * @brief  StabilizerTask —— 姿态融合 + 控制输出（核心控制线程）
  * @param  argument: Not used
  * @retval None
  *
  * 这是整个飞控的核心任务，负责：
  *   1. 等待 IMU 数据就绪（通过 imuDataReadySemaphore 信号量）
  *   2. 从 SensorSampleQueue 消费传感器数据
  *   3. 互补滤波姿态解算 → roll / pitch / yaw（调用 APP_IMU_UpdateAttitude）
  *   4. 将融合后的姿态写入 vofaLogQueue（VOFA_task 在上位机显示）
  *   5. 按 25Hz 控制周期输出舵机指令
  *
  * 数据流：
  *   Sensor_Task → SensorSampleQueue → 这里
  *     ① 互补滤波算出 roll/pitch/yaw
  *     ② 写入 vofaLogQueue（VOFA_task 50Hz 发送到上位机）
  *     ③ 舵机控制输出（25Hz）
  *
  * 舵机控制有两种模式（编译期切换）：
  *   模式 A (USE_DIRECT_ANGLE_SERVO=1)：姿态角直接映射为舵机脉宽
  *   模式 B (USE_DIRECT_ANGLE_SERVO=0)：经过 Simulink 生成的同轴控制器
  *
  * 注意：未满足安全条件时断开电调 PWM 输出（CCR=0）。
  */
/* USER CODE END Header_StabilizerTask */
void StabilizerTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StabilizerTask */

  APP_Sensor_SampleMessage msg;        /* 传感器消息（从队列中取出）            */
  float  roll  = 0.0f;                /* 当前横滚角 [度]                       */
  float  pitch = 0.0f;                /* 当前俯仰角 [度]                       */
  float  yaw   = 0.0f;                /* 当前偏航角 [度]                       */
  float  roll_zero = 0.0f;
  float  pitch_zero = 0.0f;
  float  yaw_zero = 0.0f;
  float  roll_control = 0.0f;
  float  pitch_control = 0.0f;
  float  yaw_control = 0.0f;
  float  roll_zero_sum = 0.0f;
  float  pitch_zero_sum = 0.0f;
  float  yaw_zero_sum = 0.0f;
  uint32_t attitude_zero_count = 0U;
  uint32_t attitude_zero_start_ms = 0U;
  uint8_t attitude_zero_ready = 0U;
#if (STABILIZER_USE_FIXED_IMU_DT == 0U)
  uint64_t last_imu_timestamp_us = 0ULL; /* 上一帧 IMU 时间戳（用于计算 dt）   */
#endif
  uint32_t stabilizer_seq = 0U;       /* 姿态解算帧序号                         */
  uint32_t last_out_ms = 0U;          /* 上次控制输出时刻 [ms]                  */
  uint8_t has_imu_sample = 0U;        /* 是否已收到至少一帧 IMU 数据            */
#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
  float velocity_state_x_m_s = 0.0f;
  float velocity_state_y_m_s = 0.0f;
  float velocity_imu_x_m_s = 0.0f;
  float velocity_imu_y_m_s = 0.0f;
  float position_ref_x_m = 0.0f;
  float position_ref_y_m = 0.0f;
  float position_ref_z_m = 0.0f;
  float height_ref_base_m = 0.0f;
  float height_origin_m = 0.0f;
  uint8_t height_origin_ready = 0U;
  uint8_t position_ref_z_ready = 0U;
  float yaw_ref_rad = 0.0f;
  uint8_t yaw_ref_ready = 0U;
  DRV_IMU_NAV_State nav_state;
  StabilizerVelocityPidState vel_pid_x;
  StabilizerVelocityPidState vel_pid_y;
  StabilizerVelocityEstimatorState vel_estimator;
  StabilizerVofaDebug vofa_debug = {0};
  uint32_t last_ctrl_model_ms = 0U;
  uint8_t flight_log_divider = 0U;
  uint16_t flight_log_tail_records = 0U;
#endif

#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
  DRV_IMU_NAV_Reset(&nav_state);
  stabilizer_velocity_pid_reset(&vel_pid_x);
  stabilizer_velocity_pid_reset(&vel_pid_y);
  stabilizer_velocity_estimator_reset(&vel_estimator);
#endif

  for(;;)
  {
    /*
     * 步骤 0：阻塞等待 IMU 数据就绪信号量
     * Sensor_Task 在每次 IMU 采样完成后 release 此信号量。
     * 超时 1ms——正常情况下信号量在中断后立即到来。
     */
    (void)osSemaphoreAcquire(imuDataReadySemaphore, 1U);

    /* 步骤 0.5：ELRS 遥控器链路状态机步进（非阻塞，每次 IMU 采样后执行） */
    APP_ELRS_Step();

    /*
     * 步骤 1：消费 SensorSampleQueue 中的所有待处理数据
     * 使用 while 循环 + 零超时（0U）一次性清空队列，只保留最新一帧。
     * 如果消费者慢于生产者（1kHz IMU），队列深度 8 提供缓冲。
     */
    while (osMessageQueueGet(SensorSampleQueueHandle, &msg, 0U, 0U) == osOK) {
      float dt_sec = SENSOR_IMU_DEFAULT_DT_SEC;

      stabilizer_seq++;
#if (STABILIZER_USE_FIXED_IMU_DT == 0U)
      if (last_imu_timestamp_us != 0ULL) {
        uint64_t dt_us = msg.base.timestamp_us - last_imu_timestamp_us;
        dt_sec = (float)dt_us * 0.000001f;   /* 微秒 → 秒                  */
      }
      last_imu_timestamp_us = msg.base.timestamp_us;
#endif

      /*
       * ① 互补滤波姿态解算
       * 调用 APP_IMU_UpdateAttitude()，融合加速度计（低频）和陀螺仪（高频）数据。
       * 输入：msg.imu（加速度 + 陀螺仪原始值）、dt_sec、帧序号
       * 输出：roll / pitch / yaw（全局变量，被舵机输出阶段使用）
       */
      APP_IMU_UpdateAttitude(&msg.imu, &roll, &pitch, &yaw,
                             dt_sec, stabilizer_seq);
      APP_IMU_GetAttitudeDebug(&msg.attitude_debug);
      has_imu_sample = 1U;

      if ((attitude_zero_ready == 0U) &&
          (msg.gyro_bias_ready != 0U)) {
        if (attitude_zero_start_ms == 0U) {
          attitude_zero_start_ms = HAL_GetTick();
        }
        roll_zero_sum += roll;
        pitch_zero_sum += pitch;
        yaw_zero_sum += yaw;
        ++attitude_zero_count;

        if (((HAL_GetTick() - attitude_zero_start_ms) >= STABILIZER_ATTITUDE_ZERO_MS) &&
            (attitude_zero_count > 0U)) {
          roll_zero = roll_zero_sum / (float)attitude_zero_count;
          pitch_zero = pitch_zero_sum / (float)attitude_zero_count;
          yaw_zero = yaw_zero_sum / (float)attitude_zero_count;
          attitude_zero_ready = 1U;
        }
      }

      if (attitude_zero_ready != 0U) {
        roll_control = roll - roll_zero;
        pitch_control = pitch - pitch_zero;
        yaw_control = yaw - yaw_zero;
        if (yaw_control > 180.0f) {
          yaw_control -= 360.0f;
        } else if (yaw_control < -180.0f) {
          yaw_control += 360.0f;
        }
      } else {
        roll_control = 0.0f;
        pitch_control = 0.0f;
        yaw_control = 0.0f;
      }

#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
      {
        DRV_IMU_NAV_Input nav_input;

        nav_input.accel_x_g = msg.imu.accel_x_g;
        nav_input.accel_y_g = msg.imu.accel_y_g;
        nav_input.accel_z_g = msg.imu.accel_z_g;
        nav_input.roll_rad = roll * STABILIZER_DEG_TO_RAD;
        nav_input.pitch_rad = pitch * STABILIZER_DEG_TO_RAD;
        nav_input.yaw_rad = yaw_control * STABILIZER_DEG_TO_RAD;
        nav_input.dt_sec = dt_sec;
        nav_input.gravity_m_s2 = DRV_AIRFRAME_GRAVITY_M_S2;
        nav_input.accel_lpf_alpha = STABILIZER_NAV_ACCEL_LPF_ALPHA;
        nav_input.velocity_leak_rate_hz = STABILIZER_NAV_VEL_LEAK_HZ;

        if ((attitude_zero_ready != 0U) && (nav_state.bias_ready == 0U)) {
          DRV_IMU_NAV_CaptureBias(&nav_state, &nav_input);
        }
        if (attitude_zero_ready != 0U) {
          DRV_IMU_NAV_Update(&nav_state, &nav_input);
        }

        velocity_imu_x_m_s = nav_state.vel_m_s[0];
        velocity_imu_y_m_s = nav_state.vel_m_s[1];
        {
          float flow_vx_m_s = 0.0f;
          float flow_vy_m_s = 0.0f;
          uint32_t flow_sample_ms = 0U;
          uint8_t flow_valid =
            APP_OpticalFlow_GetVelocitySample(&flow_vx_m_s,
                                              &flow_vy_m_s,
                                              &flow_sample_ms);
          uint8_t flow_accepted = 0U;

          flow_accepted = stabilizer_velocity_estimator_step(&vel_estimator,
                                                             nav_state.acc_nav_m_s2[0],
                                                             nav_state.acc_nav_m_s2[1],
                                                             velocity_imu_x_m_s,
                                                             velocity_imu_y_m_s,
                                                             flow_vx_m_s,
                                                             flow_vy_m_s,
                                                             flow_valid,
                                                             flow_sample_ms,
                                                             dt_sec);
          velocity_state_x_m_s = vel_estimator.vel_m_s[0];
          velocity_state_y_m_s = vel_estimator.vel_m_s[1];
          if (flow_accepted != 0U) {
            APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_FLOW);
          } else {
            APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_IMU);
          }
        }
        vofa_debug.acc_nav_m_s2[0] = nav_state.acc_nav_m_s2[0];
        vofa_debug.acc_nav_m_s2[1] = nav_state.acc_nav_m_s2[1];
        vofa_debug.acc_nav_m_s2[2] = nav_state.acc_nav_m_s2[2];
        vofa_debug.vel_est_m_s[0] = velocity_state_x_m_s;
        vofa_debug.vel_est_m_s[1] = velocity_state_y_m_s;
        vofa_debug.vel_est_m_s[2] = nav_state.vel_m_s[2];
        vofa_debug.nav_accel_lpf_alpha = nav_state.accel_lpf_alpha;
        vofa_debug.nav_velocity_leak_hz = nav_state.velocity_leak_rate_hz;
        stabilizer_vofa_debug_publish(&vofa_debug);
      }
#endif

      /*
       * ② 写入 VOFA 日志队列（覆盖模式）
       * vofaLogQueue 深度为 1，如果队列满则丢弃旧数据再写入新数据。
       * VOFA_task 以 50Hz 频率从此队列读取并发送到上位机。
       */
      msg.roll_deg  = roll_control;
      msg.pitch_deg = pitch_control;
      msg.yaw_deg   = yaw_control;

      if (osMessageQueuePut(vofaLogQueueHandle, &msg, 0U, 0U) != osOK) {
        APP_Sensor_SampleMessage drop;
        (void)osMessageQueueGet(vofaLogQueueHandle, &drop, 0U, 0U);
        (void)osMessageQueuePut(vofaLogQueueHandle, &msg, 0U, 0U);
      }
    }

    /*
     * 步骤 2：按固定周期（50Hz）输出舵机控制
     * 控制输出与姿态解算解耦——不管 IMU 来多快，舵机始终 50Hz 更新。
     */
    {
      uint32_t now = HAL_GetTick();

      if ((now - last_out_ms) >= STABILIZER_CONTROL_PERIOD_MS) {
#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
        uint16_t ch[16];
#if (STABILIZER_USE_RC_DIRECT_TILT_SERVO == 0U)
        DRV_COAX_CTRL_AttitudeInput attitude;
        DRV_COAX_CTRL_Reference reference;
#endif
        DRV_COAX_CTRL_Output ctrl_out;
        uint8_t rc_armed = 0U;
        uint8_t rc_link_ok = 0U;
        uint8_t rc_link_seen = 0U;
        uint16_t rc_throttle_motor_us = BSP_PWM_ESC_MIN_US;
        uint8_t rc_use_stabilized_motor_mix = 0U;
        uint8_t rc_arm_switch_high = 0U;
        uint8_t rc_arm_throttle_low = 0U;
        uint8_t range_height_valid = 0U;
        float range_height_m = 0.0f;
        float range_velocity_m_s = 0.0f;
        float relative_height_m = 0.0f;
        uint32_t range_sample_ms = 0U;
        APP_LED_ArmBlockReason led_arm_block_reason =
          APP_LED_ARM_BLOCK_NO_RC;
        APP_FlightLogMotorOutputReason motor_output_reason =
          APP_FLIGHT_LOG_MOTOR_REASON_UNKNOWN;
        float ctrl_dt_sec = (float)STABILIZER_CONTROL_PERIOD_MS * 0.001f;
#endif
        DRV_SERVO_MoveCmd moves[2];     /* [0]=servo 1 alpha/left-right, [1]=servo 2 beta/front-back */
        uint8_t imu_control_valid = 0U;
        uint8_t ident_running = 0U;
        uint8_t servo_cal_active = 0U;

        last_out_ms = now;

#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
#if (STABILIZER_USE_RC_DIRECT_TILT_SERVO == 0U)
        memset(&attitude, 0, sizeof(attitude));
        memset(&reference, 0, sizeof(reference));
#endif
        if (last_ctrl_model_ms != 0U) {
          uint32_t elapsed_ms = now - last_ctrl_model_ms;
          if ((elapsed_ms > 0U) && (elapsed_ms <= STABILIZER_IMU_STALE_MS)) {
            ctrl_dt_sec = (float)elapsed_ms * 0.001f;
          }
        }
        last_ctrl_model_ms = now;

        APP_ELRS_GetChannels(ch);       /* 读取 ELRS 遥控器 16 通道             */
        rc_link_ok = APP_ELRS_IsRcFresh(now, STABILIZER_RC_LOSS_TIMEOUT_MS);
        rc_link_seen = (APP_ELRS_GetLastRcMs() != 0U) ? 1U : 0U;
        rc_arm_switch_high =
          (ch[STABILIZER_RC_CH_ARM] > STABILIZER_RC_ARM_THRESHOLD_US) ? 1U : 0U;
        rc_arm_throttle_low =
          (ch[STABILIZER_RC_CH_THROTTLE_Z] <= STABILIZER_RC_THROTTLE_ARM_LOW_US) ? 1U : 0U;
        rc_armed = stabilizer_rc_update_armed(ch, rc_link_ok);
        rc_throttle_motor_us =
          stabilizer_rc_throttle_to_motor_pulse(ch[STABILIZER_RC_CH_THROTTLE_Z]);
        rc_use_stabilized_motor_mix =
          stabilizer_rc_use_stabilized_motor_mix(ch[STABILIZER_RC_CH_THROTTLE_Z]);
        (void)APP_ServoCal_Step(ch, rc_link_ok, rc_arm_switch_high, now);
        servo_cal_active = APP_ServoCal_IsActive();
        if (servo_cal_active != 0U) {
          rc_armed = 0U;
        }
#endif
        BSP_AiWB2_UpdateButton();       /* 更新 WiFi 模块按键状态                */

        /* 默认保持上一条有效舵机目标；短暂 IMU 异常不能直接回中。 */
        moves[0].id = 1U;
        moves[1].id = 2U;
        moves[0].pulse_us = stabilizer_latest_servo_target_us[0];
        moves[1].pulse_us = stabilizer_latest_servo_target_us[1];

        if ((has_imu_sample != 0U) &&
            (attitude_zero_ready != 0U) &&
            ((now - stabilizer_imu_last_sample_ms) <= STABILIZER_IMU_STALE_MS)) {
          imu_control_valid = 1U;
        }

        APP_Ident_Update(now);
        ident_running = APP_Ident_IsRunning();

#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
        if (rc_link_seen == 0U) {
          led_arm_block_reason = APP_LED_ARM_BLOCK_NO_RC;
        } else if (rc_link_ok == 0U) {
          led_arm_block_reason = APP_LED_ARM_BLOCK_RC_LOSS;
        } else if (rc_armed == 0U) {
          if ((rc_arm_switch_high != 0U) && (rc_arm_throttle_low == 0U)) {
            led_arm_block_reason = APP_LED_ARM_BLOCK_THROTTLE_HIGH;
          } else {
            led_arm_block_reason = APP_LED_ARM_BLOCK_ARM_SWITCH;
          }
        } else if ((rc_use_stabilized_motor_mix != 0U) &&
                   (imu_control_valid == 0U) &&
                   (ident_running == 0U)) {
          led_arm_block_reason = APP_LED_ARM_BLOCK_IMU;
        } else {
          led_arm_block_reason = APP_LED_ARM_BLOCK_NONE;
        }
        APP_LED_SetArmStatus(rc_armed, led_arm_block_reason);
#endif

        if (ident_running != 0U) {
          uint16_t ident_alpha_us;
          uint16_t ident_beta_us;

          APP_Ident_GetServoTargets(&ident_alpha_us, &ident_beta_us);
          moves[0].pulse_us = ident_alpha_us;
          moves[1].pulse_us = ident_beta_us;
#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
        } else if (rc_use_stabilized_motor_mix == 0U) {
          /*
           * Throttle below the 20% stabilization threshold is the low-power
           * direct-throttle stage. Keep tilt servos centered there so stick
           * motion cannot move the airframe before the test is intentionally
           * brought into the active range.
           */
          velocity_state_x_m_s = 0.0f;
          velocity_state_y_m_s = 0.0f;
          position_ref_x_m = 0.0f;
          position_ref_y_m = 0.0f;
          height_ref_base_m = 0.0f;
          /* 上电高度原点只在首次有效测高时锁存，低油门直通不重新归零。 */
          position_ref_z_ready = 0U;
          yaw_ref_ready = 0U;
          DRV_IMU_NAV_Reset(&nav_state);
          stabilizer_velocity_estimator_reset(&vel_estimator);
          stabilizer_velocity_pid_reset(&vel_pid_x);
          stabilizer_velocity_pid_reset(&vel_pid_y);
          position_ref_z_ready = 0U;
          yaw_ref_ready = 0U;
          vofa_debug.vel_loop_active = 0.0f;
          stabilizer_vofa_debug_publish(&vofa_debug);
          moves[0].pulse_us = DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US;
          moves[1].pulse_us = DRV_COAX_CTRL_SERVO_BETA_CENTER_US;
#endif
        } else if (imu_control_valid != 0U) {
#if (STABILIZER_USE_DIRECT_ANGLE_SERVO != 0U)
          /*
           * 模式 A：角度直驱
           *   pitch/roll → 机体系 X/Y 倾角 → 驱动层 90° 安装补偿 → 舵机1/2
           */
          stabilizer_map_angle_direct_to_servo(roll_control, pitch_control, moves);
#else
          /*
           * 模式 B：同轴控制器（论文控制分配）
           *   输入：姿态角 + 角速度（全部转为弧度）+ RC 参考（归一化）
           *   输出：两个舵机脉宽（servo_alpha_us / servo_beta_us）
           */
#if (STABILIZER_USE_RC_DIRECT_TILT_SERVO != 0U)
          /*
           * 当前台架遥控调试模式：
           *   CH2 前后、CH1 左右先形成机体系 X/Y 倾角。
           *   当前机械安装补偿后：1号舵机左右，2号舵机前后。
           *   这样先验证遥控通道和机械方向，不受未接位置/速度估计的控制器影响。
           */
          stabilizer_map_rc_direct_to_servo(ch, moves);
          ctrl_out.thrust_upper_n = 0.0f;
          ctrl_out.thrust_lower_n = 0.0f;
          ctrl_out.motor_upper_us = BSP_PWM_ESC_MIN_US;
          ctrl_out.motor_lower_us = BSP_PWM_ESC_MIN_US;
#else
          range_height_valid =
            APP_OpticalFlow_GetHeightSample(&range_height_m,
                                            &range_velocity_m_s,
                                            &range_sample_ms);
          (void)range_sample_ms;
          if (range_height_valid != 0U) {
            if (height_origin_ready == 0U) {
              height_origin_m = range_height_m;
              height_origin_ready = 1U;
            }
            relative_height_m = range_height_m - height_origin_m;
            if (relative_height_m < 0.0f) {
              relative_height_m = 0.0f;
            }
          }
          {
            uint8_t velocity_control_ok =
              stabilizer_velocity_estimator_control_ok(&vel_estimator, now);

          attitude.roll_rad = roll_control * STABILIZER_DEG_TO_RAD;
          attitude.pitch_rad = pitch_control * STABILIZER_DEG_TO_RAD;
          attitude.yaw_rad = yaw_control * STABILIZER_DEG_TO_RAD;
          attitude.x_m = 0.0f;
          attitude.y_m = 0.0f;
          attitude.z_m = -relative_height_m;
          attitude.vx_m_s = (velocity_control_ok != 0U) ?
                            velocity_state_x_m_s : 0.0f;
          attitude.vy_m_s = (velocity_control_ok != 0U) ?
                            velocity_state_y_m_s : 0.0f;
          attitude.vz_m_s = -range_velocity_m_s;
          if (range_height_valid == 0U) {
            attitude.z_m = 0.0f;
            attitude.vz_m_s = 0.0f;
          }
          attitude.gyro_x_rad_s = msg.imu.gyro_x_dps * STABILIZER_DEG_TO_RAD;
          attitude.gyro_y_rad_s = msg.imu.gyro_y_dps * STABILIZER_DEG_TO_RAD;
          attitude.gyro_z_rad_s = msg.imu.gyro_z_dps * STABILIZER_DEG_TO_RAD;

          reference.vx_m_s = stabilizer_rc_normalized(ch[STABILIZER_RC_CH_PITCH]) *
                             STABILIZER_XY_VEL_REF_MAX_M_S;
          reference.vy_m_s = stabilizer_rc_normalized(ch[STABILIZER_RC_CH_ROLL]) *
                             STABILIZER_XY_VEL_REF_MAX_M_S;
          reference.vz_m_s = 0.0f;
          {
            float vel_ref_x_m_s = reference.vx_m_s;
            float vel_ref_y_m_s = reference.vy_m_s;
            float vel_loop_enable = 0.0f;
            float vel_loop_x_kp = 0.0f;
            float vel_loop_x_ki = 0.0f;
            float vel_loop_x_kd = 0.0f;
            float vel_loop_y_kp = 0.0f;
            float vel_loop_y_ki = 0.0f;
            float vel_loop_y_kd = 0.0f;
            float vel_err_x_m_s;
            float vel_err_y_m_s;

            (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_enable", &vel_loop_enable);
            (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_x_kp", &vel_loop_x_kp);
            (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_x_ki", &vel_loop_x_ki);
            (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_x_kd", &vel_loop_x_kd);
            (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_y_kp", &vel_loop_y_kp);
            (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_y_ki", &vel_loop_y_ki);
            (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_y_kd", &vel_loop_y_kd);

            vel_err_x_m_s = vel_ref_x_m_s - velocity_state_x_m_s;
            vel_err_y_m_s = vel_ref_y_m_s - velocity_state_y_m_s;
            vofa_debug.vel_ref_m_s[0] = vel_ref_x_m_s;
            vofa_debug.vel_ref_m_s[1] = vel_ref_y_m_s;
            vofa_debug.vel_err_m_s[0] = vel_err_x_m_s;
            vofa_debug.vel_err_m_s[1] = vel_err_y_m_s;
            vofa_debug.vel_loop_x_kp = vel_loop_x_kp;
            vofa_debug.vel_loop_x_ki = vel_loop_x_ki;
            vofa_debug.vel_loop_x_kd = vel_loop_x_kd;
            vofa_debug.vel_loop_y_kp = vel_loop_y_kp;
            vofa_debug.vel_loop_y_ki = vel_loop_y_ki;
            vofa_debug.vel_loop_y_kd = vel_loop_y_kd;

            if ((vel_loop_enable >= 0.5f) &&
                (velocity_control_ok != 0U)) {
              position_ref_x_m = 0.0f;
              position_ref_y_m = 0.0f;
              reference.ax_m_s2 =
                stabilizer_velocity_pid_step(&vel_pid_x,
                                             vel_err_x_m_s,
                                             velocity_state_x_m_s,
                                             vel_loop_x_kp,
                                             vel_loop_x_ki,
                                             vel_loop_x_kd,
                                             ctrl_dt_sec,
                                             &vofa_debug.vel_pid_p_m_s2[0],
                                             &vofa_debug.vel_pid_i_m_s2[0],
                                             &vofa_debug.vel_pid_d_m_s2[0]);
              reference.ay_m_s2 =
                stabilizer_velocity_pid_step(&vel_pid_y,
                                             -vel_err_y_m_s,
                                             velocity_state_y_m_s,
                                             vel_loop_y_kp,
                                             vel_loop_y_ki,
                                             vel_loop_y_kd,
                                             ctrl_dt_sec,
                                             &vofa_debug.vel_pid_p_m_s2[1],
                                             &vofa_debug.vel_pid_i_m_s2[1],
                                             &vofa_debug.vel_pid_d_m_s2[1]);
              reference.vx_m_s = attitude.vx_m_s;
              reference.vy_m_s = attitude.vy_m_s;
              vofa_debug.vel_loop_active = 1.0f;
            } else {
              stabilizer_velocity_pid_reset(&vel_pid_x);
              stabilizer_velocity_pid_reset(&vel_pid_y);
              reference.ax_m_s2 = 0.0f;
              reference.ay_m_s2 = 0.0f;
              vofa_debug.vel_pid_p_m_s2[0] = 0.0f;
              vofa_debug.vel_pid_i_m_s2[0] = 0.0f;
              vofa_debug.vel_pid_d_m_s2[0] = 0.0f;
              vofa_debug.vel_pid_p_m_s2[1] = 0.0f;
              vofa_debug.vel_pid_i_m_s2[1] = 0.0f;
              vofa_debug.vel_pid_d_m_s2[1] = 0.0f;
              vofa_debug.vel_loop_active = 0.0f;
            }

            vofa_debug.vel_pid_out_m_s2[0] = reference.ax_m_s2;
            vofa_debug.vel_pid_out_m_s2[1] = reference.ay_m_s2;
          }
          reference.az_m_s2 =
            -stabilizer_rc_throttle_thrust_bias_m_s2(
              ch[STABILIZER_RC_CH_THROTTLE_Z]);
          if ((range_height_valid != 0U) &&
              (height_origin_ready != 0U) &&
              (relative_height_m >= STABILIZER_Z_REF_MAX_M) &&
              (reference.az_m_s2 < 0.0f)) {
            reference.az_m_s2 = 0.0f;
          }
          if (vofa_debug.vel_loop_active >= 0.5f) {
            reference.x_m = attitude.x_m;
            reference.y_m = attitude.y_m;
          } else {
            position_ref_x_m =
              stabilizer_clamp_f32(position_ref_x_m +
                                   reference.vx_m_s * ctrl_dt_sec,
                                   -STABILIZER_XY_POS_ERR_MAX_M,
                                    STABILIZER_XY_POS_ERR_MAX_M);
            position_ref_y_m =
              stabilizer_clamp_f32(position_ref_y_m +
                                   reference.vy_m_s * ctrl_dt_sec,
                                   -STABILIZER_XY_POS_ERR_MAX_M,
                                    STABILIZER_XY_POS_ERR_MAX_M);

            reference.x_m = position_ref_x_m;
            reference.y_m = position_ref_y_m;
          }
          {
            if ((range_height_valid == 0U) ||
                (height_origin_ready == 0U)) {
              height_ref_base_m = 0.0f;
              position_ref_z_ready = 0U;
              position_ref_z_m = attitude.z_m;
            } else if (rc_use_stabilized_motor_mix == 0U) {
              height_ref_base_m = relative_height_m;
              height_ref_base_m =
                stabilizer_clamp_f32(height_ref_base_m,
                                     0.0f,
                                     STABILIZER_Z_REF_MAX_M);
              position_ref_z_m = -height_ref_base_m;
              position_ref_z_ready = 1U;
            } else {
              float height_ref_m;

              if (position_ref_z_ready == 0U) {
                height_ref_base_m = relative_height_m;
                height_ref_base_m =
                  stabilizer_clamp_f32(height_ref_base_m,
                                       0.0f,
                                       STABILIZER_Z_REF_MAX_M);
                position_ref_z_ready = 1U;
              }
              height_ref_m = height_ref_base_m +
                             stabilizer_rc_throttle_height_offset_m(
                               ch[STABILIZER_RC_CH_THROTTLE_Z]);
              height_ref_m =
                stabilizer_clamp_f32(height_ref_m,
                                     0.0f,
                                     STABILIZER_Z_REF_MAX_M);
              position_ref_z_m = -height_ref_m;
            }
          }
          if ((range_height_valid == 0U) ||
              (height_origin_ready == 0U)) {
            reference.z_m = attitude.z_m;
          } else {
            reference.z_m =
              stabilizer_clamp_f32(position_ref_z_m,
                                   attitude.z_m - STABILIZER_Z_POS_ERR_MAX_M,
                                   attitude.z_m + STABILIZER_Z_POS_ERR_MAX_M);
          }
          {
            float yaw_rate_ref_rad_s =
              stabilizer_rc_yaw_rate_rad_s(ch[STABILIZER_RC_CH_YAW]);

            if (yaw_ref_ready == 0U) {
              yaw_ref_rad = attitude.yaw_rad;
              yaw_ref_ready = 1U;
            }
            yaw_ref_rad =
              stabilizer_wrap_pi(yaw_ref_rad +
                                 yaw_rate_ref_rad_s * ctrl_dt_sec);
            reference.yaw_rad = yaw_ref_rad;
            reference.yaw_rate_rad_s = yaw_rate_ref_rad_s;
            reference.yaw_accel_rad_s2 = 0.0f;
          }

          DRV_COAX_CTRL_Run(&attitude, &reference, &ctrl_out);

          vofa_debug.range_vertical_velocity_m_s = range_velocity_m_s;
          vofa_debug.altitude_ref_m = -reference.z_m;
          vofa_debug.altitude_correction_us = 0.0f;
          }

          moves[0].pulse_us = ctrl_out.servo_alpha_us;
          moves[1].pulse_us = ctrl_out.servo_beta_us;
          vofa_debug.servo_alpha_us = (float)moves[0].pulse_us;
          vofa_debug.servo_beta_us = (float)moves[1].pulse_us;
          vofa_debug.motor_upper_us = (float)ctrl_out.motor_upper_us;
          vofa_debug.motor_lower_us = (float)ctrl_out.motor_lower_us;
          stabilizer_vofa_debug_publish(&vofa_debug);
#endif
#endif
        } else if (has_imu_sample == 0U) {
#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
          DRV_IMU_NAV_Reset(&nav_state);
          stabilizer_velocity_estimator_reset(&vel_estimator);
          stabilizer_velocity_pid_reset(&vel_pid_x);
          stabilizer_velocity_pid_reset(&vel_pid_y);
          vofa_debug.vel_loop_active = 0.0f;
          stabilizer_vofa_debug_publish(&vofa_debug);
#endif
          /* 上电尚无有效姿态时才使用中位；运行中 IMU 异常保持上一目标。 */
          moves[0].pulse_us = DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US;
          moves[1].pulse_us = DRV_COAX_CTRL_SERVO_BETA_CENTER_US;
        }

        if (ident_running != 0U) {
          APP_IdentObserve ident_obs = {
            .now_ms = now,
            .roll_deg = roll_control,
            .pitch_deg = pitch_control,
            .gyro_x_dps = msg.imu.gyro_x_dps,
            .gyro_y_dps = msg.imu.gyro_y_dps,
            .rc_link_ok = rc_link_ok,
            .rc_armed = rc_armed,
            .imu_valid = imu_control_valid,
            .throttle_us = rc_throttle_motor_us,
          };
          APP_Ident_Observe(&ident_obs);
        }

        if (servo_cal_active == 0U) {
          stabilizer_servo_record_target(moves);

          /* Send only on change, with a 500 ms forced refresh, to avoid bus flooding. */
          if (stabilizer_servo_should_send(moves, now) != 0U) {
            if (BSP_BusServo_MoveManyAsync(moves, 2U,
                                         STABILIZER_SERVO_MOVE_TIME_MS) == DRV_SERVO_OK) {
              stabilizer_servo_commit_sent(moves, now);
            }
          }
        }

#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
        if (servo_cal_active != 0U) {
          BSP_PWM_SetEscPulse(1, BSP_PWM_ESC_MIN_US);
          BSP_PWM_SetEscPulse(2, BSP_PWM_ESC_MIN_US);
          motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_DISARMED_MIN;
        } else if ((rc_link_ok != 0U) && (rc_armed != 0U)) {
          if ((rc_use_stabilized_motor_mix != 0U) &&
              (ident_running == 0U) &&
              (imu_control_valid != 0U)) {
            BSP_PWM_SetEscPulse(1, ctrl_out.motor_upper_us);
            BSP_PWM_SetEscPulse(2, ctrl_out.motor_lower_us);
            motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_STABILIZED_MIX;
          } else {
            BSP_PWM_SetEscPulse(1, rc_throttle_motor_us);
            BSP_PWM_SetEscPulse(2, rc_throttle_motor_us);
            if (ident_running != 0U) {
              motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_IDENT_DIRECT;
            } else if ((rc_use_stabilized_motor_mix != 0U) &&
                       (imu_control_valid == 0U)) {
              motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_IMU_INVALID_DIRECT;
            } else {
              motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_DIRECT_THROTTLE;
            }
          }
        } else if ((rc_link_ok != 0U) || (rc_link_seen == 0U)) {
          BSP_PWM_SetEscPulse(1, BSP_PWM_ESC_MIN_US);
          BSP_PWM_SetEscPulse(2, BSP_PWM_ESC_MIN_US);
          motor_output_reason = (rc_link_seen == 0U) ?
            APP_FLIGHT_LOG_MOTOR_REASON_NO_RC_SEEN_MIN :
            APP_FLIGHT_LOG_MOTOR_REASON_DISARMED_MIN;
        } else
#endif
        {
          BSP_PWM_DisableEsc(1);
          BSP_PWM_DisableEsc(2);
#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
          motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_RC_LOSS_DISABLE;
#endif
        }
#if (STABILIZER_USE_DIRECT_ANGLE_SERVO == 0U)
        if (flight_log_divider == 0U) {
          APP_FlightLogSnapshot flog_snapshot;
          uint8_t flight_log_active =
            ((rc_link_ok != 0U) &&
             (rc_armed != 0U) &&
             (rc_use_stabilized_motor_mix != 0U)) ? 1U : 0U;
          uint8_t flight_log_should_record = flight_log_active;

          memset(&flog_snapshot, 0, sizeof(flog_snapshot));
          flog_snapshot.timestamp_us = msg.base.timestamp_us;
          flog_snapshot.tick_ms = now;
          flog_snapshot.imu_sequence = msg.base.sequence;
          flog_snapshot.imu_raw = msg.raw_imu;
          flog_snapshot.imu = msg.imu;
          flog_snapshot.roll_deg = roll_control;
          flog_snapshot.pitch_deg = pitch_control;
          flog_snapshot.yaw_deg = yaw_control;
          memcpy(flog_snapshot.rc_channels,
                 ch,
                 sizeof(flog_snapshot.rc_channels));
          flog_snapshot.throttle_us = rc_throttle_motor_us;
          flog_snapshot.servo_alpha_us = moves[0].pulse_us;
          flog_snapshot.servo_beta_us = moves[1].pulse_us;
          flog_snapshot.motor_upper_us = BSP_PWM_GetEscPulse(1);
          flog_snapshot.motor_lower_us = BSP_PWM_GetEscPulse(2);
          flog_snapshot.rc_armed = rc_armed;
          flog_snapshot.rc_link_ok = rc_link_ok;
          flog_snapshot.throttle_over_20 = rc_use_stabilized_motor_mix;
          flog_snapshot.imu_valid = imu_control_valid;
          flog_snapshot.motor_output_reason = (uint8_t)motor_output_reason;
          flog_snapshot.rc_link_seen = rc_link_seen;
          flog_snapshot.arm_switch_high = rc_arm_switch_high;
          flog_snapshot.arm_throttle_low = rc_arm_throttle_low;
          flog_snapshot.arm_switch_seen_low = stabilizer_rc_switch_seen_low;
          flog_snapshot.arm_switch_prev_high = stabilizer_rc_switch_prev_high;
          flog_snapshot.imu_fault_latched = stabilizer_imu_fault_latched;
          flog_snapshot.imu_fault_reason = (uint8_t)stabilizer_imu_fault_reason;
          memcpy(flog_snapshot.acc_nav_m_s2,
                 vofa_debug.acc_nav_m_s2,
                 sizeof(flog_snapshot.acc_nav_m_s2));
          memcpy(flog_snapshot.vel_est_m_s,
                 vofa_debug.vel_est_m_s,
                 sizeof(flog_snapshot.vel_est_m_s));
          memcpy(flog_snapshot.vel_ref_m_s,
                 vofa_debug.vel_ref_m_s,
                 sizeof(flog_snapshot.vel_ref_m_s));
          memcpy(flog_snapshot.vel_err_m_s,
                 vofa_debug.vel_err_m_s,
                 sizeof(flog_snapshot.vel_err_m_s));
          memcpy(flog_snapshot.vel_pid_out_m_s2,
                 vofa_debug.vel_pid_out_m_s2,
                 sizeof(flog_snapshot.vel_pid_out_m_s2));
          memcpy(flog_snapshot.vel_pid_p_m_s2,
                 vofa_debug.vel_pid_p_m_s2,
                 sizeof(flog_snapshot.vel_pid_p_m_s2));
          memcpy(flog_snapshot.vel_pid_i_m_s2,
                 vofa_debug.vel_pid_i_m_s2,
                 sizeof(flog_snapshot.vel_pid_i_m_s2));
          memcpy(flog_snapshot.vel_pid_d_m_s2,
                 vofa_debug.vel_pid_d_m_s2,
                 sizeof(flog_snapshot.vel_pid_d_m_s2));
          flog_snapshot.vel_loop_active = vofa_debug.vel_loop_active;
          DRV_COAX_CTRL_GetLastDebug(&flog_snapshot.ctrl_debug);
          flog_snapshot.z_ref_m = vofa_debug.altitude_ref_m;

          if (flight_log_active != 0U) {
            flight_log_tail_records = STABILIZER_FLIGHT_LOG_TAIL_RECORDS;
          } else if (flight_log_tail_records > 0U) {
            flight_log_tail_records--;
            flight_log_should_record = 1U;
          }

          APP_FlightLog_Observe(flight_log_should_record ? &flog_snapshot : NULL,
                                flight_log_should_record);
        }
        flight_log_divider ^= 1U;
#endif
      }
    }
  }
  /* USER CODE END StabilizerTask */
}

/* USER CODE BEGIN Header_Sensor_Task */
/**
  * @brief  Sensor_Task —— IMU 传感器采集与数据预处理（1kHz，中断驱动）
  * @param  argument: Not used
  * @retval None
  *
  * 这是整个飞控的数据源头，负责：
  *   1. 初始化 IMU（ICM-42688-P）、气压计（SPL06-007）、磁力计（QMC5883L）
  *   2. 等待 PC0 外部中断通知（IMU 数据就绪），超时后轮询兜底
  *   3. 读取 IMU 原始寄存器值 → 转换为物理单位（dps / g）
  *   4. 陀螺仪零偏校准（前 1000 个样本自动均值，要求静止）
  *   5. 坐标系对齐（ICM 芯片坐标系 → 机体 NED 坐标系）
  *   6. 低通滤波（陀螺 80Hz、加速度 30Hz，IIR 一阶）
  *   7. 气压计降采样读取（1kHz 中每 32 次读一次 ≈ 32Hz）
  *   8. 组装 SensorSampleMessage → 推入 SensorSampleQueue
  *   9. Release imuDataReadySemaphore 通知 StabilizerTask 消费
  *
  * 运行时序：
  *   硬件 IRQ ──→ Thread Flag ──→ 本任务唤醒 ──→ SPI 读取 ──→ 处理流水线
  *   ──→ 推送队列 ──→ Release 信号量 ──→ 再次阻塞等待 Thread Flag
  *
  * 涉及的硬件接口：
  *   SPI1 → ICM-42688-P (IMU, 加速度 + 陀螺仪)
  *   I2C1 → SPL06-007   (气压计)
  *   I2C1 → QMC5883L    (磁力计)
  *   PC0  → EXTI 中断   (IMU INT1 数据就绪引脚)
  */
/* USER CODE END Header_Sensor_Task */
void Sensor_Task(void *argument)
{
  /* USER CODE BEGIN Sensor_Task */
  BSP_IMU_Invalidate();                  /* 复位 IMU 驱动内部状态              */

  /* APP_Task_GPS_Init();  暂时停止 GPS */
  APP_Task_OpticalFlow_Init();
  APP_Task_MAG_Init();                   /* 初始化磁力计 QMC5883L              */

  /* ---- 初始化 IMU（重试直到成功） ---- */
  while (BSP_IMU_Init() != DRV_IMU_OK) {
    osDelay(50);
  }
  BSP_BARO_Init();   /* 气压计初始化，失败时在读数据时重试 */

  DRV_IMU_RawData    raw;               /* IMU 原始 ADC 值（寄存器原始读数）    */
  DRV_IMU_ScaledData scaled;            /* IMU 物理单位值（dps / g）            */
  uint32_t sample_count = 0U;           /* 总采样帧计数（溢出回绕是安全的）     */
  uint32_t baro_cnt     = 0U;           /* 气压计降采样计数器（32 分频）        */
  uint64_t last_mag_step_us = 0ULL;     /* 磁力计上次步进时间 [μs]              */
  float    baro_pa      = 0.0f;         /* 当前气压值 [Pa]，保持旧值直到更新    */
  float    baro_temp    = 0.0f;         /* 当前气压计温度 [°C]                  */

  /* 陀螺仪零偏校准状态 */
  APP_Sensor_GyroBias gyro_bias = {0};
  APP_Sensor_RateMeter imu_rate_meter = {0}; /* IMU 实际采样率统计              */
  APP_Sensor_RateMeter imu_irq_rate_meter = {0};
  APP_Sensor_RateMeter imu_poll_rate_meter = {0};

  /*
   * 低通滤波器：陀螺 80Hz，加速度 30Hz
   * dt = 0.001s（@ 1kHz 采样率）
   * 陀螺 80Hz 截止频率的选择依据：共轴飞行器机械振动主要在 50~100Hz，
   * 需要在保留有效角速度信号的同时衰减高频振动噪声。
   * 加速度 30Hz 更加激进——加速度计噪声大且姿态解算只关心重力方向。
   */
  APP_Sensor_Lpf gyro_lpf[3], acc_lpf[3];
  uint32_t imu_irq_ready_count = 0U;
  uint32_t imu_poll_ready_count = 0U;
  uint32_t imu_read_fail_count = 0U;
  uint32_t imu_drdy_miss_count = 0U;
  for (uint32_t i = 0U; i < 3U; i++) {
    APP_Sensor_LpfInit(&gyro_lpf[i], 80.0f, 0.001f);
    APP_Sensor_LpfInit(&acc_lpf[i], 30.0f, 0.001f);
  }

  osDelay(10);

  for(;;)
  {
    /*
     * 步骤 1：等待 IMU 数据就绪
     * PC0 EXTI → HAL_GPIO_EXTI_Callback → osThreadFlagsSet → 本任务被唤醒。
     * 未等到中断时只读一次 ready 状态；若仍未 ready，则跳过本轮继续等下一帧。
     */
   if ((osThreadFlagsWait(SENSOR_IMU_DATA_READY_FLAG, osFlagsWaitAny,
                           SENSOR_IMU_DRDY_TIMEOUT_MS) &
         SENSOR_IMU_DATA_READY_FLAG) != 0U) {
      imu_irq_ready_count++;
      imu_drdy_miss_count = 0U;
    } else {
      bool imu_ready = false;

      if ((BSP_IMU_IsDataReady(&imu_ready) == DRV_IMU_OK) && imu_ready) {
        imu_poll_ready_count++;
        imu_drdy_miss_count = 0U;
      } else {
        imu_drdy_miss_count++;
        if (imu_drdy_miss_count >= SENSOR_IMU_DRDY_MISS_FAULT_LIMIT) {
          stabilizer_latch_imu_fault(STABILIZER_IMU_FAULT_DRDY_TIMEOUT);
          (void)osSemaphoreRelease(imuDataReadySemaphore);
        }
        continue;
      }
    }

    /* ---- 步骤 2：SPI 读取 IMU 原始值，转换为物理单位 ---- */
    if (BSP_IMU_ReadRaw(&raw) != DRV_IMU_OK) {
      imu_read_fail_count++;
      if (imu_read_fail_count >= SENSOR_IMU_READ_FAIL_LIMIT) {
        stabilizer_latch_imu_fault(STABILIZER_IMU_FAULT_READ_FAIL);
        (void)osSemaphoreRelease(imuDataReadySemaphore);
      }
      continue;
    }
    imu_read_fail_count = 0U;
    stabilizer_clear_imu_fault();
    stabilizer_imu_last_sample_ms = HAL_GetTick();

    sample_count++;
    APP_IMU_RawToScaled(&raw, &scaled);  /* ADC → 物理单位（dps / g）         */

    /*
     * 步骤 3：陀螺仪零偏校准
     * APP_Sensor_CalibrateGyroBias() 内部累积前 1000 个样本求均值。
     * 校准完成后（gyro_bias.ready == true），后续所有采样都减去零偏。
     * 重要：校准时飞行器必须完全静止，否则零偏不准确。
     */
    {
      float g[3] = {scaled.gyro_x_dps, scaled.gyro_y_dps, scaled.gyro_z_dps};
      if (APP_Sensor_CalibrateGyroBias(g[0], g[1], g[2], &gyro_bias)) {
        /* 校准刚完成，第一次减零偏自然生效 */
      }
      if (gyro_bias.ready) {
        g[0] -= gyro_bias.bias[0];
        g[1] -= gyro_bias.bias[1];
        g[2] -= gyro_bias.bias[2];
      }
      scaled.gyro_x_dps = g[0];
      scaled.gyro_y_dps = g[1];
      scaled.gyro_z_dps = g[2];
    }

    /*
     * 步骤 4：坐标系对齐 + 低通滤波
     * ICM-42688-P 芯片坐标系与机体坐标系不一定一致（取决于 PCB 焊接方向）。
     * APP_Sensor_AlignToAirframe() 通过轴重排和取反将传感器轴映射到机体轴。
     * 机体坐标系定义（NED 约定）：
     *   X → 机头前方,  Y → 机身右侧,  Z → 机身下方
     * 对齐后分别对陀螺和加速度施加一阶 IIR 低通滤波。
     */
    {
      float a[3] = {scaled.accel_x_g, scaled.accel_y_g, scaled.accel_z_g};
      float g[3] = {scaled.gyro_x_dps, scaled.gyro_y_dps, scaled.gyro_z_dps};
      float a_align[3], g_align[3];

      APP_Sensor_AlignToAirframe(a, a_align);
      APP_Sensor_AlignToAirframe(g, g_align);
      APP_Sensor_LpfApply3f(gyro_lpf, g_align, g_align); /* IIR 低通：陀螺    */
      APP_Sensor_LpfApply3f(acc_lpf,  a_align, a_align); /* IIR 低通：加速度  */

      scaled.accel_x_g  = a_align[0];  scaled.accel_y_g = a_align[1];  scaled.accel_z_g = a_align[2];
      scaled.gyro_x_dps = g_align[0];  scaled.gyro_y_dps = g_align[1]; scaled.gyro_z_dps = g_align[2];
    }

    /*
     * 步骤 5：气压计降采样读取（≈32Hz）
     * 气压计不需要 1kHz 更新——大气压力变化缓慢（< 10Hz）。
     * 每 32 次 IMU 循环（= 32ms @ 1kHz）读取一次，实际频率 ≈ 31.25Hz。
     * SPI06-007 偶有上电复位后无响应的问题——读取失败时重新初始化。
     */
    uint8_t baro_fresh = 0U;

    if (++baro_cnt >= 32U) {
      baro_cnt = 0U;
      uint8_t buf[6];                    /* 3 字节压力原始值 + 3 字节温度原始值 */

      if (BSP_BARO_ReadRawRegisters(0x00U, buf, 6U) == DRV_BARO_OK) {
        /* SPL06-007：24 位补码，MSB 在前，左移后算术右移完成符号扩展 */
        int32_t prs_raw = ((int32_t)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8))) >> 8;
        int32_t tmp_raw = ((int32_t)(((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 8))) >> 8;

        APP_IMU_ConvertBaro(prs_raw, tmp_raw, &baro_pa, &baro_temp);
        baro_fresh = 1U;
      } else {
        /* 读取失败：复位驱动状态 + 重新初始化传感器 */
        BSP_BARO_Invalidate();
        BSP_BARO_Init();
      }
    }

    /*
     * 步骤 6：组装传感器消息 → 推送队列 → 通知 StabilizerTask
     * SensorSampleMessage 包含：
     *   - 时间戳（SVC_Timestamp_Us：32 位微秒硬件计数器）
     *   - IMU 数据（加速度 + 陀螺仪，已滤波 + 坐标对齐 + 零偏校正）
     *   - 气压计数据（压力 + 温度，baro_updated 标志是否本帧新鲜）
     *   - IMU 采样率统计
     *
     * 队列满处理：丢弃最旧的一帧再写入（覆盖模式），保证 Stabilizer 拿到最新数据。
     *
     * 顺序关键：必须先 Push 队列，再 Release 信号量！
     * 否则 StabilizerTask 被唤醒后发现队列空，白跑一趟。
     */
    APP_Sensor_SampleMessage msg;

    msg.base.timestamp_us    = SVC_Timestamp_Us();
    msg.base.type            = APP_SENSOR_TYPE_IMU;
    msg.base.sequence        = sample_count;
    msg.raw_imu              = raw;
    msg.imu                  = scaled;
    msg.roll_deg             = 0.0f;   /* Stabilizer 填入 */
    msg.pitch_deg            = 0.0f;
    msg.yaw_deg              = 0.0f;
    msg.baro_updated         = baro_fresh;
    msg.baro_pressure_pa     = baro_pa;
    msg.baro_temperature_c   = baro_temp;
    msg.imu_sample_rate_hz   = APP_SensorRateMeter_Update(&imu_rate_meter,
                                                          msg.base.timestamp_us,
                                                          sample_count);
    msg.imu_irq_sample_rate_hz  = APP_SensorRateMeter_Update(&imu_irq_rate_meter,
                                                              msg.base.timestamp_us,
                                                              imu_irq_ready_count);
    msg.imu_poll_sample_rate_hz = APP_SensorRateMeter_Update(&imu_poll_rate_meter,
                                                              msg.base.timestamp_us,
                                                              imu_poll_ready_count);
    msg.imu_age_ms = 0.0f;
    memset(&msg.attitude_debug, 0, sizeof(msg.attitude_debug));
    msg.gyro_bias_ready = gyro_bias.ready;
    msg.imu_data_ready_count = imu_irq_ready_count;
    msg.imu_poll_ready_count = imu_poll_ready_count;

    if (osMessageQueuePut(SensorSampleQueueHandle, &msg, 0U, 0U) != osOK) {
      APP_Sensor_SampleMessage drop;
      (void)osMessageQueueGet(SensorSampleQueueHandle, &drop, 0U, 0U);
      (void)osMessageQueuePut(SensorSampleQueueHandle, &msg, 0U, 0U);
    }

    /* 步骤 7：Release 信号量 → 唤醒 StabilizerTask */
    (void)osSemaphoreRelease(imuDataReadySemaphore);

    /*
     * 步骤 8：磁力计步进（非阻塞，20Hz 独立节奏）
     * 磁力计挂在 I2C 总线上，读取速度慢（~100Hz max），且数据不需要 1kHz 更新。
     * 使用独立的时间间隔 SENSOR_MAG_PERIOD_US (50ms = 20Hz) 来控制步进。
     */
                             /* APP_Task_GPS_Step();  暂时停止 GPS */
    APP_Task_OpticalFlow_Step();
    {
      uint64_t now_us = msg.base.timestamp_us;

      if ((last_mag_step_us == 0ULL) ||
          ((now_us - last_mag_step_us) >= SENSOR_MAG_PERIOD_US)) {
        last_mag_step_us = now_us;
        APP_Task_MAG_Step();
      }
    }
  }
  /* USER CODE END Sensor_Task */
}

/* USER CODE BEGIN Header_message_push */
/**
  * @brief  messageTask —— 消息协议处理（JSON 解析 / 二进制帧编解码）
  * @param  argument: Not used
  * @retval None
  *
  * 功能：
  *   处理来自上位机 / 遥控器的消息协议，包括：
  *   - JSON 消息解析与应答
  *   - 二进制帧封包 / 解包
  *   - 参数读写请求的协议层处理
  *   - WiFi 模块（Ai-WB2）的命令交互
  *
  * 优先级 BelowNormal：消息处理不影响飞控实时性。
  * 栈 1024×4：JSON 处理和字符串操作需要较大栈空间。
  */
/* USER CODE END Header_message_push */
void message_push(void *argument)
{
  /* USER CODE BEGIN message_push */
  APP_Task_Message_Init();
  /* Infinite loop */
  for(;;)
  {
    APP_Task_Message_Step();
  }
  /* USER CODE END message_push */
}

/* USER CODE BEGIN Header_UART_fun */
/**
  * @brief  UARTTask —— UART 收发调度
  * @param  argument: Not used
  * @retval None
  *
  * 功能：
  *   统一管理所有 UART 外设的数据收发：
  *   - UART4 (PD.1) → 舵机总线（半双工串行舵机协议）
  *   - UART7 → ELRS 遥控器接收机（Crossfire 协议）
  *   - UART8 → Ai-WB2 WiFi 模块（AT 命令 + 数据透传）
  *
  * 从 uartTxQueue 中取出待发送数据，分别投递到对应的 UART 外设。
  * 各 UART 的接收在中断（IDLE / RXNE）中完成，此任务做分发调度。
  *
  * 优先级 Normal：UART 发送不及时会导致舵机丢帧或 WiFi 阻塞。
  * 栈 1024×4：三个 UART 通道的缓冲区和协议栈需要较大空间。
  */
/* USER CODE END Header_UART_fun */
void UART_fun(void *argument)
{
  /* USER CODE BEGIN UART_fun */
  APP_Task_UART_Init();
  /* Infinite loop */
  for(;;)
  {
    APP_Task_UART_Step();
  }
  /* USER CODE END UART_fun */
}

/* USER CODE BEGIN Header_BackgroundTask */
/**
  * @brief  BackgroundTask —— 后台低优先级任务（FLASH 参数管理）
  * @param  argument: Not used
  * @retval None
  *
  * 功能：
  *   执行不应该阻塞控制环（SensorTask / StabilizerTask）的慢操作：
  *   - 参数从 FLASH 自动加载（上电后首次运行）
  *   - 参数异步保存（修改参数后延迟写入 FLASH，减少擦写次数）
  *   - 维护模式 FLASH 块读写（坏块管理、磨损均衡等）
  *
  * 交互方式：请求-响应模式
  *   - 请求者（任意任务）→ backgroundReqQueue → 本任务处理
  *   - 本任务 → backgroundRespQueue → 请求者取走结果
  *
  * 所有 FLASH 操作前必须获取 flashBusMutex（递归互斥锁）。
  *
  * 优先级 BelowNormal：只在所有实时任务空闲时执行。
  * 栈 1024×4：FLASH 页缓冲需要较大栈空间（一页 = 128 字节，需双缓冲）。
  */
/* USER CODE END Header_BackgroundTask */
void BackgroundTask(void *argument)
{
  /* USER CODE BEGIN BackgroundTask */
  APP_Task_Background_Init();
  /* Infinite loop */
  for(;;)
  {
    APP_Task_Background_Step();
  }
  /* USER CODE END BackgroundTask */
}

/* USER CODE BEGIN Header_VOFA_task */
/**
  * @brief  VOFA_Task —— VOFA 上位机数据发送（50Hz 定时）
  * @param  argument: Not used
  * @retval None
  *
  * 功能：
  *   从 vofaLogQueue 取出姿态/传感器数据，按固定格式打包后
  *   通过 WiFi 透传发送到 VOFA 上位机进行实时可视化。
  *
  * 发送数据帧（22 个 float，88 字节 + 4 字节帧尾）：
  *   [0]  roll      横滚角 [°]
  *   [1]  pitch     俯仰角 [°]
  *   [2]  yaw       偏航角 [°]
  *   [3]  flow_height 二合一光流测距高度 [m]，测距无效时为 0
  *   [4]  time       时间戳 [s]
  *   [5]  vel_est_x  融合后的 X 速度估计 [m/s]
  *   [6]  vel_est_y  融合后的 Y 速度估计 [m/s]
  *   [7..10]  姿态控制参数滑块反馈
  *   [11..17] 速度环参数滑块反馈
  *   [18..19] 姿态角 P 参数滑块反馈
  *   [20..21] Z 高度位置/速度参数滑块反馈
  *
  * vofaStreamActive 标志可由上位机远程控制，方便暂停 / 恢复数据流。
  *
  * 优先级 Low：可视化数据允许延迟或丢帧，不影响飞行安全。
  * 周期 40Hz（osDelay(25)）：92 字节帧约占 3680 B/s，保留命令响应余量。
  */
/* USER CODE END Header_VOFA_task */
void VOFA_task(void *argument)
{
  /* USER CODE BEGIN VOFA_task */

  APP_Sensor_SampleMessage msg;
  #define VOFA_DATA_SIZE 22U
  float vofa_data[VOFA_DATA_SIZE];
  StabilizerVofaDebug vofa_debug;
  APP_OPTICAL_FLOW_Status flow_status;

  for(;;)
  {
    /*
     * VOFA 诊断降频发送，避免 WiFi/USART1 调试流量影响实时任务调度。
     */
    osDelay(VOFA_SEND_PERIOD_MS);

    if (!vofaStreamActive) {
      continue;
    }
    if (APP_FlightLog_IsExportActive() != 0U) {
      continue;
    }

    if (osMessageQueueGet(vofaLogQueueHandle, &msg, 0U, 0U) == osOK) {
      /* ---- 组装 VOFA 数据帧 ---- */

      APP_OpticalFlow_GetStatus(&flow_status);

      stabilizer_vofa_debug_read(&vofa_debug);

      vofa_data[0] = msg.roll_deg;
      vofa_data[1] = msg.pitch_deg;
      vofa_data[2] = msg.yaw_deg;

      /* 二合一光流测距高度 [m]；失效或超时立即输出 0，避免保留陈旧值。 */
      vofa_data[3] = (flow_status.height_valid != 0U) ?
                     flow_status.height_m : 0.0f;

      vofa_data[4] = (float)(SVC_Timestamp_Us() / 1000ULL) * 0.001f;
      vofa_data[5] = vofa_debug.vel_est_m_s[0];
      vofa_data[6] = vofa_debug.vel_est_m_s[1];
      (void)DRV_COAX_CTRL_GetParam("coax.roll_rate_kd", &vofa_data[7]);
      (void)DRV_COAX_CTRL_GetParam("coax.pitch_rate_kd", &vofa_data[8]);
      (void)DRV_COAX_CTRL_GetParam("coax.yaw_angle_kp", &vofa_data[9]);
      (void)DRV_COAX_CTRL_GetParam("coax.yaw_rate_kd", &vofa_data[10]);
      (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_x_kp", &vofa_data[11]);
      (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_y_kp", &vofa_data[12]);
      (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_x_ki", &vofa_data[13]);
      (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_y_ki", &vofa_data[14]);
      (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_x_kd", &vofa_data[15]);
      (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_y_kd", &vofa_data[16]);
      (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_enable", &vofa_data[17]);
      (void)DRV_COAX_CTRL_GetParam("coax.roll_angle_kp", &vofa_data[18]);
      (void)DRV_COAX_CTRL_GetParam("coax.pitch_angle_kp", &vofa_data[19]);
      (void)DRV_COAX_CTRL_GetParam("coax.pos_z_kp", &vofa_data[20]);
      (void)DRV_COAX_CTRL_GetParam("coax.vel_z_kd", &vofa_data[21]);
      /* Present operator-facing gains as positive values; controller internals keep the tested signs. */
      vofa_data[7] = -vofa_data[7];
      vofa_data[8] = -vofa_data[8];
      vofa_data[9] = -vofa_data[9];
      vofa_data[10] = -vofa_data[10];
      vofa_data[18] = -vofa_data[18];
      vofa_data[19] = -vofa_data[19];
      /* 22 floats + VOFA tail = 92 bytes. */
      APP_VOFA_SendFloats(vofa_data, VOFA_DATA_SIZE);
    }
  }
  /* USER CODE END VOFA_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

