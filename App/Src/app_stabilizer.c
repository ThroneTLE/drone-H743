/*
 * ============================================================================
 * app_stabilizer.c —— 姿态稳定器（姿态融合 + 同轴倾转旋翼控制）
 * ============================================================================
 * 从 Core/Src/freertos.c 的 StabilizerTask 中拆分而来，对齐 Crazyflie
 * stabilizer 任务的分层结构：init / imu_step（感知）/ control_step（控制）。
 *
 * 数据流（与拆分前完全一致，纯机械移动）：
 *   APP_Stabilizer_Run  —— 主循环：信号量 → ELRS 步进 → 队列抽干(imu_step) → control_step
 *   stabilizer_init     —— 上下文清零 + Fusion / NAV / 速度估计 / 同轴控制器复位
 *   stabilizer_imu_step —— 单帧 IMU 处理：dt / 陀螺角加速度 / Fusion /
 *                          零偏校准 / NAV / 速度估计 / VOFA 调试发布
 *   stabilizer_control_step —— 500Hz 周期控制：prepare / compute / commit
 *     prepare —— RC 读取、开锁判定、定高基准、ident、LED
 *     compute —— 模式分支（ident / 直通 / 同轴控制器 / 无 IMU 保持）
 *     commit  —— 舵机总线发送、ESC 输出、飞行日志
 *
 * 跨任务公共 API（APP_Stabilizer_*）供 freertos.c 的 Sensor_Task /
 * VOFA_task 调用；任务句柄由 APP_Stabilizer_Run 参数注入。
 *
 * 常量、辅助函数、静态量均原样从 freertos.c 搬入（仅加 s_ 前缀句柄）。
 */

#include "app_stabilizer.h"
#include "app_battery.h"
#include <math.h>
#include <string.h>

#include "app_elrs.h"
#include "app_flight_calibration.h"
#include "app_flight_log.h"
#include "app_flight_log_nav.h"
#include "app_ident.h"
#include "app_sysid.h"
#include "app_sysid_alt.h"
#include "app_thrust_lut.h"
#include "app_imu_capture.h"
#include "app_imu_health.h"
#include "app_led.h"
#include "app_mag.h"
#include "app_magxy.h"
#include "app_nav_estimator.h"
#include "app_optical_flow.h"
#include "app_esc_command.h"
#include "app_prop_spin.h"
#include "app_rpm_notch.h"
#include "app_servo_backlash.h"
#include "app_thrust_bench.h"
#include "app_hover_adapt.h"
#include "app_rc_aux.h"
#include "app_rc_config.h"
#include "app_rc_intent.h"
#include "app_sensor.h"
#include "app_messages.h"
#include "app_servo_cal.h"
#include "app_servo_jog.h"
#include "app_servo_type.h"
#include "app_servo_feedback.h"
#include "app_servo_feedback_bench.h"
#include "app_acceptance.h"
#include "app_control_scheduler.h"
#include "bsp_critical.h"
#include "bsp_aiwb2_power.h"
#include "bsp_bus_servo.h"
#include "bsp_pwm.h"
#include "drv_airframe_params.h"
#include "drv_attitude_fusion.h"
#include "drv_coax_ctrl.h"
#include "drv_flight_throttle.h"
#include "drv_frame_contract.h"
#include "drv_imu_calibration.h"
#include "drv_prop_map.h"
#include "drv_z_estimator.h"
#include "app_hover_thrust.h"
#include "svc_flow_nav.h"
#include "svc_mag_heading.h"
#include "svc_timestamp.h"
#include "drv_servo.h"

/* ============================================================================
 * 常量（从 freertos.c 搬入）
 * ========================================================================== */
#define STABILIZER_SERVO_MOVE_TIME_MS 0U
/* 速度估计的全部整定量已归 Services/Inc/svc_flow_nav.h，此处不留副本。 */
#define STABILIZER_FLOW_ROT_COMP_ENABLE 1U
#define STABILIZER_FLOW_ROT_COMP_GAIN 1.0f
/*
 * 光流模块相对重心的安装位置，规范 FLU（X 前、Y 左、Z 上）。作者实测：相对飞控板
 * (0, −16, −16) cm（Y 负半轴下方）；重心在板下 2.2 cm，故 Z = −0.16 + 0.022。
 */
#define STABILIZER_FLOW_SENSOR_OFFSET_X_M 0.0f
#define STABILIZER_FLOW_SENSOR_OFFSET_Y_M -0.16f
#define STABILIZER_FLOW_SENSOR_OFFSET_Z_M -0.138f
#define STABILIZER_IMU_LEVER_ARM_X_M 0.0f
#define STABILIZER_IMU_LEVER_ARM_Y_M 0.0f
#define STABILIZER_IMU_LEVER_ARM_Z_M (-0.10f)
#define STABILIZER_IMU_RATE_WEIGHT_SOFT_RAD_S 2.0f
#define STABILIZER_IMU_ALPHA_WEIGHT_SOFT_RAD_S2 20.0f

#define SENSOR_IMU_DEFAULT_DT_SEC      0.001f   /* 默认 IMU 采样间隔 1ms → dt = 0.001s */
#define SENSOR_IMU_MIN_DT_US           100ULL   /* Fusion 有效采样周期下限 0.1ms        */
#define SENSOR_IMU_MAX_DT_US           30000ULL /* Fusion 有效采样周期上限 30ms         */
#define SENSOR_IMU_DRDY_TIMEOUT_MS     20U      /* 单次等待 DRDY 的超时时间            */
#define SENSOR_IMU_DRDY_MISS_FAULT_LIMIT 50U    /* 连续 1s 无 DRDY/ready 才锁存故障    */
#define SENSOR_IMU_READ_FAIL_LIMIT     25U      /* 连续读失败次数，超过后锁存故障      */
#define SENSOR_MAG_PERIOD_US           50000ULL /* 磁力计步进间隔 50ms = 20Hz          */
#define STABILIZER_USE_FIXED_IMU_DT    0U       /* 1=固定 1ms dt, 0=时间戳 dt          */
#define STABILIZER_VALIDATION_SNAPSHOT_READ_RETRIES 4U

/*
 * ============================================================================
 * 姿态稳定器常量
 * ============================================================================
 * 控制器每 2 ms 更新一次目标，舵机半双工总线每 10 ms 最多发送一次移动命令。
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
#define STABILIZER_USE_DIRECT_ANGLE_SERVO 0U     /* 1=角度直驱舵机, 0=同轴控制器(永久) */
#define STABILIZER_YAW_RATE_REF_MAX_RAD_S 1.04719758f /* CH4 偏航参考累加最大速率 [rad/s] */
#define STABILIZER_XY_VEL_REF_MAX_M_S  0.40f     /* CH1/CH2 满杆速度的兜底值 [m/s]（正常取 coax.pos_xy_vel_max_m_s） */
#define STABILIZER_XY_VEL_STICK_MIN_M_S 0.10f    /* 参数可接受范围：低于/高于则退回兜底值 */
#define STABILIZER_XY_VEL_STICK_MAX_M_S 5.00f
#define STABILIZER_XY_POS_ERR_MAX_M    0.50f     /* 水平位置外环单次误差限幅 [m]        */
/* 定点档松杆"先刹停再锁位"（同 PX4 手动定点）：速度降到此值以下才锁当前位置；刹车最长这么久后强制锁。 */
#define STABILIZER_XY_LOCK_SPEED_M_S   0.15f
#define STABILIZER_XY_BRAKE_MAX_S      2.0f
#define STABILIZER_Z_REF_MAX_M         0.40f     /* 上电光流测高基准以上高度上限 [m]     */
#define STABILIZER_Z_REF_RATE_MAX_M_S  0.30f     /* CH3 满杆高度目标积分速度 [m/s]       */
#define STABILIZER_Z_POS_ERR_MAX_M     0.35f     /* Z 位置 PID 单次位置误差限幅 [m]      */
/* 摇杆死区已移入 APP_RcConfig.deadband_us（可标定），此处不再定义。 */

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
 *   CH6 → 姿态调试模式开关       → 高位启用手动总推力 + 目标姿态
 *
 * CRSF 驱动输出的是 16 路 us 值，数组下标从 0 开始，所以 CH1 对应 ch[0]。
 * CH5 用阈值判断：高于中位视为开锁，否则上锁。
 * 解锁还必须满足低油门：油门 <=10% 行程。防止开关误触后电机带油门启动。
 *
 * 上面这张表现在只是"出厂默认值"（见 app_rc_config.c 的 app_rc_default_channel），
 * 实际通道号、正反向和端点由 APP_RcConfig 提供，可在上位机标定后写入 Flash。
 * 本文件的 RC 判读不再出现通道下标字面量——要改映射请走 RCMAP 协议，不要改代码。
 * 已知例外：第 1740 行附近仍把原始 ch[] 直传给 app_servo_cal.c 的摇杆手势状态机，
 * 该模块内部还有写死的 CH1..CH4 下标，属 RC 映射迁移未完成部分（见 PIPELINE 副线）。
 */
/* 开关判高低、油门判低位，都按各自标定端点的百分比算，不再用绝对 us 阈值。 */
#define STABILIZER_RC_SWITCH_HIGH_PERCENT    50U
#define STABILIZER_RC_THROTTLE_ARM_LOW_PERCENT 10U
#define STABILIZER_RC_STABILIZE_MIN_PERCENT 70U
#define STABILIZER_RC_LOSS_TIMEOUT_MS  500U
#define STABILIZER_FLIGHT_LOG_TAIL_RECORDS 125U /* 以 125 Hz 计的 1 s 尾巴条数；实际按子分频折算，见 flight_log_tail_for_subdiv */
#define STABILIZER_USE_RC_DIRECT_TILT_SERVO 0U   /* 0=自稳定控制器(永久), 1=CH1/CH2直控舵机调试 */
#define STABILIZER_RC_ATTITUDE_TARGET_LIMIT_RAD 0.349065850f /* CH6 姿态调试最大 ±20° */
/*
 * 摇杆 → 机体意图的极性不在本文件。它由 app_rc_intent 这一个 Adapter 决定，
 * 每条都是「标定向导的物理提示 + drv_frame_contract.h」推出来的，本文件只按
 * 名字调用，禁止在这里补符号。
 */

/*
 * ============================================================================
 * 舵机通信常量
 * ============================================================================
 * 舵机通过 UART7 半双工总线控制，每次移动命令指定目标脉宽和到位时间。
 * 为避免总线拥塞，只有脉宽变化超过阈值或超过强制刷新间隔才发送。
 */
#define STABILIZER_SERVO_REFRESH_MS    500U      /* 强制刷新间隔 [ms]（即使脉宽未变）    */
#define STABILIZER_SERVO_DELTA_US      3U        /* 脉宽变化死区 [μs]（小于此值不发送）  */
#define STABILIZER_SERVO_BUS_FRAME_MS 10U        /* 总线移动命令上限 100 Hz              */
#define STABILIZER_ATTITUDE_ZERO_MS 1500U        /* 上电后姿态零偏采集时长 [ms]          */
#define STABILIZER_ATTITUDE_ZERO_GYRO_MAX_DPS 2.0f
#define STABILIZER_ATTITUDE_ZERO_ACCEL_MIN_G 0.95f
#define STABILIZER_ATTITUDE_ZERO_ACCEL_MAX_G 1.05f
#define STABILIZER_ATTITUDE_ZERO_ERROR_MAX_DEG 3.0f

/* ============================================================================
 * 辅助函数 / 静态量（原 freertos.c "稳定器辅助函数"段，逐字节搬入）
 * ========================================================================== */
  static volatile uint8_t stabilizer_imu_fault_latched = 0U;
  static volatile StabilizerImuFaultReason stabilizer_imu_fault_reason =
    STABILIZER_IMU_FAULT_NONE;
  static volatile uint32_t stabilizer_imu_fault_count = 0U;
  static volatile uint32_t stabilizer_imu_last_sample_ms = 0U;
  static uint8_t stabilizer_rc_arm_latched = 0U;
  static uint8_t stabilizer_rc_switch_seen_low = 0U;
  static uint8_t stabilizer_rc_switch_prev_high = 0U;
  /* 飞行油门模式：上一控制拍的相对高度/垂直速度，供本拍状态机做落地判定（约 16 字节）。 */
  static uint8_t stabilizer_ft_fb_valid = 0U;
  static uint32_t stabilizer_ft_last_ms = 0U;
  static float stabilizer_ft_fb_height_m = 0.0f;
  static float stabilizer_ft_fb_vz_m_s = 0.0f;

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
   * 摇杆量已经由 APP_RcConfig_Resolve() 按标定端点归一化到 [-1,+1] / [0,1]，
   * 这里的函数只负责把它换算成物理量，不再自己解释脉宽。
   */
  static float stabilizer_rc_throttle_height_rate_m_s(float throttle_norm)
  {
    return throttle_norm * STABILIZER_Z_REF_RATE_MAX_M_S;
  }

  /*
   * 开关高低不能直接比绝对 us：两段开关的低位常在 1000 附近、高位在 2000 附近，
   * 但三段开关或做过端点标定的通道中位并不是 1500。统一按该通道自身行程的百分比判。
   */
  static uint8_t stabilizer_rc_channel_above_percent(
    const APP_RcConfig *config, uint8_t function, uint16_t ch_us, uint8_t percent)
  {
    const APP_RcFunctionMap *map;
    int32_t span;
    int32_t threshold;

    if ((config == NULL) || (function >= APP_RC_FUNC_COUNT)) {
      return 0U;
    }
    map = &config->function[function];
    if (map->channel == APP_RC_CHANNEL_UNBOUND) {
      return 0U;
    }
    span = (int32_t)map->max_us - (int32_t)map->min_us;
    if (span <= 0) {
      return 0U;
    }
    threshold = (int32_t)map->min_us + ((span * (int32_t)percent) / 100);
    if (map->reversed != 0U) {
      /* 与正向分支严格镜像：value > threshold 经 value ↦ min+max−value 映射后
       * 等价于 value < min+max−threshold（同为严格不等号，阈值点两侧语义一致）。 */
      return ((int32_t)ch_us < ((int32_t)map->min_us + (int32_t)map->max_us -
                                threshold)) ? 1U : 0U;
    }
    return ((int32_t)ch_us > threshold) ? 1U : 0U;
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

  /*
   * 飞行限幅取参数（作者 2026-10-01："飞机限速/加速度等参数让我可以在上位机可以随意配置"）：
   * 读 coax.* 参数，缺失或超出 [lo, hi] 时退回原写死常量 fallback（照 coax.pos_xy_vel_max_m_s 先例）。
   */
  static float stabilizer_flight_limit(const char *name, float fallback, float lo, float hi)
  {
    float value = 0.0f;

    if ((DRV_COAX_CTRL_GetParam(name, &value) != 0U) && (value >= lo) && (value <= hi)) {
      return value;
    }
    return fallback;
  }

  static float stabilizer_alt_max_m(void)
  {
    return stabilizer_flight_limit("coax.alt_max_m", STABILIZER_Z_REF_MAX_M, 0.1f, 20.0f);
  }

  /* 角度档满杆倾角：≤ 参数 coax.tilt_limit_rad（控制器总倾角限幅），不会比控制器肯给的更大。 */
  static float stabilizer_manual_tilt_max_rad(void)
  {
    float tilt = stabilizer_flight_limit("coax.manual_tilt_max_rad",
                                         STABILIZER_RC_ATTITUDE_TARGET_LIMIT_RAD, 0.05f, 0.785f);
    float limit = 0.0f;

    if ((DRV_COAX_CTRL_GetParam("coax.tilt_limit_rad", &limit) != 0U) && (limit > 0.0f) &&
        (tilt > limit)) {
      tilt = limit;
    }
    return tilt;
  }

  static float stabilizer_rc_yaw_rate_rad_s(float yaw_norm)
  {
    return APP_RcIntent_YawRateLeft(yaw_norm,
        stabilizer_flight_limit("coax.yaw_stick_rate_rad_s",
                                STABILIZER_YAW_RATE_REF_MAX_RAD_S, 0.1f, 6.0f));
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

  static uint16_t stabilizer_rc_throttle_to_motor_pulse(float throttle_01)
  {
    float pulse_f = (float)BSP_PWM_ESC_MIN_US +
                    throttle_01 * (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);
    int32_t pulse_i = (int32_t)(pulse_f + 0.5f);

    return stabilizer_motor_pulse_clamp(pulse_i);
  }

  static uint8_t stabilizer_rc_use_stabilized_motor_mix(float throttle_01)
  {
    return (throttle_01 >=
            ((float)STABILIZER_RC_STABILIZE_MIN_PERCENT / 100.0f)) ? 1U : 0U;
  }

  static float stabilizer_clamp_f32(float value, float lo, float hi)
  {
    if (value < lo) { return lo; }
    if (value > hi) { return hi; }
    return value;
  }

  static float stabilizer_square_f32(float value)
  {
    return value * value;
  }

  static void stabilizer_cross3(const float a[3],
                                const float b[3],
                                float out[3])
  {
    out[0] = (a[1] * b[2]) - (a[2] * b[1]);
    out[1] = (a[2] * b[0]) - (a[0] * b[2]);
    out[2] = (a[0] * b[1]) - (a[1] * b[0]);
  }

  /*
   * IMU 比力 → **机头对齐的本地水平系**水平加速度。
   *
   * 只用 roll/pitch 把比力转平，**故意不乘 yaw**。这不是省事，是让它和同一
   * 拍喂进 EKF 的光流速度处在同一个坐标系里：光流给的是机体水平地速，本函数
   * 若再乘一次 yaw，就会把加速度转进"上电时刻朝向"的固定世界系，两者夹角等于
   * 累计偏航角。2026-09-02 实录里偏航跑了 224°、七成样本超 ±10°，那样融合出来
   * 的横向速度是错的（预测与观测互相对抗），而且偏航为 0 时完全看不出来。
   * 下游（EKF、控制器目标姿态、遥测 frame=body_flu）本来就全是机头相对口径，
   * 所以对齐到机头、而不是把其余部分改成固定世界系。
   *
   * 二阶残差已补：机头对齐系随偏航转动，速度分量的导数满足 dv/dt|分量 = a - ω×v。
   * 该项在 svc_flow_nav.c 的 Fuse 里加（它把两轴耦合起来，属坐标系语义，不归
   * drv_nav_ekf——后者被定义为 X/Y 互不串扰的纯数值 2D KF）。本函数只管转平，
   * 不要在这里补 ω×v。
   */
  static void stabilizer_compensated_imu_accel_level_xy(float accel_x_g,
                                                       float accel_y_g,
                                                       float accel_z_g,
                                                       float roll_rad,
                                                       float pitch_rad,
                                                       const float gyro_rad_s[3],
                                                       const float alpha_rad_s2[3],
                                                       uint8_t alpha_valid,
                                                       float *acc_x_m_s2,
                                                       float *acc_y_m_s2,
                                                       float *weight_out)
  {
    const float gravity = DRV_Airframe_Get()->gravity_m_s2;
    const float r_imu_m[3] = {
      STABILIZER_IMU_LEVER_ARM_X_M,
      STABILIZER_IMU_LEVER_ARM_Y_M,
      STABILIZER_IMU_LEVER_ARM_Z_M,
    };
    const float gyro_zero[3] = {0.0f, 0.0f, 0.0f};
    const float alpha_zero[3] = {0.0f, 0.0f, 0.0f};
    const float *omega = (gyro_rad_s != NULL) ? gyro_rad_s : gyro_zero;
    const float *alpha = ((alpha_valid != 0U) && (alpha_rad_s2 != NULL)) ?
                         alpha_rad_s2 : alpha_zero;
    float alpha_cross_r[3];
    float omega_cross_r[3];
    float omega_cross_omega_cross_r[3];
    float f_body_m_s2[3];
    float f_cg_body_m_s2[3];
    float cr;
    float sr;
    float cp;
    float sp;
    float rate_norm;
    float alpha_norm;
    float weight;

    if ((acc_x_m_s2 == NULL) || (acc_y_m_s2 == NULL)) {
      return;
    }

    f_body_m_s2[0] = accel_x_g * gravity;
    f_body_m_s2[1] = accel_y_g * gravity;
    f_body_m_s2[2] = accel_z_g * gravity;

    stabilizer_cross3(alpha, r_imu_m, alpha_cross_r);
    stabilizer_cross3(omega, r_imu_m, omega_cross_r);
    stabilizer_cross3(omega, omega_cross_r, omega_cross_omega_cross_r);

    for (uint32_t axis = 0U; axis < 3U; ++axis) {
      f_cg_body_m_s2[axis] = f_body_m_s2[axis] -
                             alpha_cross_r[axis] -
                             omega_cross_omega_cross_r[axis];
    }

    cr = cosf(roll_rad);
    sr = sinf(roll_rad);
    cp = cosf(pitch_rad);
    sp = sinf(pitch_rad);

    /* Ry(pitch)*Rx(roll) only -- the heading-aligned level frame.  Adding a
     * Rz(yaw) factor here is what used to put this vector in a different frame
     * from the optical-flow velocity it is fused with. */
    *acc_x_m_s2 =
      (cp * f_cg_body_m_s2[0]) +
      ((sp * sr) * f_cg_body_m_s2[1]) +
      ((sp * cr) * f_cg_body_m_s2[2]);
    *acc_y_m_s2 =
      (cr * f_cg_body_m_s2[1]) -
      (sr * f_cg_body_m_s2[2]);

    rate_norm = sqrtf(stabilizer_square_f32(omega[0]) +
                      stabilizer_square_f32(omega[1]) +
                      stabilizer_square_f32(omega[2]));
    alpha_norm = sqrtf(stabilizer_square_f32(alpha[0]) +
                       stabilizer_square_f32(alpha[1]) +
                       stabilizer_square_f32(alpha[2]));
    weight = 1.0f /
      (1.0f +
       stabilizer_square_f32(rate_norm / STABILIZER_IMU_RATE_WEIGHT_SOFT_RAD_S) +
       stabilizer_square_f32(alpha_norm / STABILIZER_IMU_ALPHA_WEIGHT_SOFT_RAD_S2));

    *acc_x_m_s2 *= weight;
    *acc_y_m_s2 *= weight;
    if (weight_out != NULL) {
      *weight_out = weight;
    }
  }

  static uint8_t stabilizer_rc_update_armed(uint8_t switch_high,
                                            uint8_t throttle_low,
                                            uint8_t rc_link_ok)
  {

    /* V0 may preview/persist only the sensor+Fusion seam.  Keep the aircraft
     * physically disarmed until every downstream FLU migration bit is proven. */
    if (APP_Stabilizer_IsImuFrameArmLocked() != 0U) {
      stabilizer_rc_arm_latched = 0U;
      stabilizer_rc_switch_seen_low = 0U;
      stabilizer_rc_switch_prev_high = switch_high;
      return 0U;
    }

    /*
     * 采样链降到 DRDY 失效档时禁止解锁。这不是"性能偏低"：低通系数按 1kHz
     * 烤死，50Hz 下等效截止频率掉到 4Hz、群延迟涨到约 56ms，且 213Hz 抗混叠
     * 滤波器远高于 25Hz 的 Nyquist，桨叶振动会整个折叠进姿态带。此状态下
     * 控制律的前提已经不成立，只能拒绝起飞。
     */
    if (APP_ImuHealth_IsArmBlocked() != 0U) {
      stabilizer_rc_arm_latched = 0U;
      stabilizer_rc_switch_seen_low = 0U;
      stabilizer_rc_switch_prev_high = switch_high;
      return 0U;
    }

    /*
     * 没有有效机体模型就禁止解锁。机体数据的唯一来源是上位机写进 Flash 的那份
     * （drv_airframe_params.h），代码里不再保留任何默认值——因为一旦有默认值，
     * 一块从没量过的飞机就会"看起来能飞"，而它用的是别人的质量和惯量。
     *
     * 这里不是保守，是这几个数没了控制律就没有物理含义：质量在每条力/加速度
     * 换算里，惯量是角加速度的分母，r_z 的正负直接决定倾转力矩的方向——
     * 漏填会让姿态环从负反馈变成正反馈，起飞即翻。失效必须朝安全方向倒。
     */
    if (DRV_Airframe_IsValid() == 0U) {
      stabilizer_rc_arm_latched = 0U;
      stabilizer_rc_switch_seen_low = 0U;
      stabilizer_rc_switch_prev_high = switch_high;
      return 0U;
    }

    /*
     * 桨叶与电机接线标定（drv_prop_map）**不在这里设门**：作者 2026-09-13 明确
     * 要求由他自己决定什么时候需要它，不由固件替他拦。
     *
     * 因此未标定时的行为是：`coax_ctrl_yaw_torque_polarity()` 返回 0，偏航通道
     * 没有权限（分配式只按总推力对半分），而不是朝一个猜出来的方向使劲；
     * 执行器按角色查通道，查不到就物理禁用。缺标定是"没有偏航"，不是"偏航可能反"。
     */

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
        (throttle_low != 0U) &&
        (APP_Battery_CanArm() != 0U)) {
      stabilizer_rc_arm_latched = 1U;
    }

    stabilizer_rc_switch_prev_high = 1U;
    return stabilizer_rc_arm_latched;
  }


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
  static volatile uint16_t stabilizer_last_successful_servo_pulse_us[2];
  /*
   * 解锁状态的文件级镜像。rc_armed 定义在控制分支内部的作用域里，而原始 IMU
   * 采集需要在融合之后就标注解锁状态（此时该分支尚未执行）。
   */
  static volatile uint8_t stabilizer_capture_armed;
  /*
   * 磁力计融合诊断的文件级镜像，供 APP_Stabilizer_GetMagFusionStatus() 读取。
   * 和 stabilizer_capture_armed 同一个理由：跨任务只读快照，全是标量，不加锁。
   */
  static volatile APP_Stabilizer_MagFusionStatus stabilizer_mag_fusion_mirror;
  static volatile APP_Stabilizer_MagXYStatus stabilizer_magxy_mirror;
  static uint32_t stabilizer_last_servo_send_ms;
  static uint32_t stabilizer_last_servo_command_frame_ms = 0xFFFFFFFFUL;

  typedef struct {
    uint32_t move_attempt_count;
    uint32_t move_sent_count;
    uint32_t move_busy_count;
    uint32_t move_error_count;
  } StabilizerServoBusDiag;

  typedef struct {
    float sensor_velocity_m_s[2];
    float optical_rot_comp_m_s[2];
    float offset_rot_comp_m_s[2];
    float corrected_velocity_m_s[2];
  } StabilizerFlowDebug;

  static StabilizerServoBusDiag stabilizer_servo_bus_diag;
  static StabilizerFlowDebug stabilizer_flow_debug;
  static volatile uint32_t stabilizer_flow_comp_seqlock;
  static volatile uint8_t stabilizer_flow_comp_valid;
  static volatile StabilizerFlowCompensationSnapshot
    stabilizer_flow_comp_snapshot;

  static void stabilizer_flow_compensation_publish(
    const StabilizerFlowDebug *debug,
    uint32_t sample_ms,
    uint8_t orientation_code,
    uint8_t valid)
  {
    StabilizerFlowCompensationSnapshot next;

    memset(&next, 0, sizeof(next));
    next.sample_ms = sample_ms;
    next.frame_contract = DRV_FRAME_CONTRACT_VERSION;
    next.orientation_code = orientation_code;
    next.valid = (valid != 0U) ? 1U : 0U;
    if ((debug != NULL) && (valid != 0U)) {
      next.sensor_velocity_flu_m_s[0] = debug->sensor_velocity_m_s[0];
      next.sensor_velocity_flu_m_s[1] = debug->sensor_velocity_m_s[1];
      next.optical_rot_comp_flu_m_s[0] = debug->optical_rot_comp_m_s[0];
      next.optical_rot_comp_flu_m_s[1] = debug->optical_rot_comp_m_s[1];
      next.offset_rot_comp_flu_m_s[0] = debug->offset_rot_comp_m_s[0];
      next.offset_rot_comp_flu_m_s[1] = debug->offset_rot_comp_m_s[1];
      next.corrected_velocity_flu_m_s[0] = debug->corrected_velocity_m_s[0];
      next.corrected_velocity_flu_m_s[1] = debug->corrected_velocity_m_s[1];
    }

    stabilizer_flow_comp_seqlock++;
    BSP_Critical_MemoryBarrier();
    stabilizer_flow_comp_snapshot = next;
    stabilizer_flow_comp_valid = next.valid;
    BSP_Critical_MemoryBarrier();
    stabilizer_flow_comp_seqlock++;
  }

  static StabilizerVofaDebug stabilizer_vofa_debug;

  static void stabilizer_compensate_flow_rotation(float *flow_vx_m_s,
                                                   float *flow_vy_m_s,
                                                   float height_m,
                                                   float gyro_x_rad_s,
                                                   float gyro_y_rad_s,
                                                   float gyro_z_rad_s,
                                                   StabilizerFlowDebug *debug)
  {
    float body_vx_m_s;
    float body_vy_m_s;

    if ((flow_vx_m_s == NULL) || (flow_vy_m_s == NULL) || (debug == NULL)) {
      return;
    }

    memset(debug, 0, sizeof(*debug));

    /*
     * 入参已经是规范 FLU（X前/Y左）。方言转换归 svc_flow_nav.c 的采集边界所有，
     * 就在整数计数变成 m/s 的那一行——**这里不要再转一次**。
     *
     * 2026-09-07 之前它在本函数末尾做，于是用 FLU 陀螺算出的 FLU 补偿向量被加到
     * 了尚未转换的 FRD 速度上：X 前向两系同号看不出问题，Y 反号相当于减两倍。
     * 手飞画圆时 ω_x 与 v_y 同相（φ ≈ -a_y/g），横向速度被整条抵消——传感器页
     * 两轴都是正弦，状态页只剩 X。在这里补一次方言转换只会把同样的错挪个位置再犯
     * 一遍，所以转换被移到了任何机体量混入之前的采集边界。本文件里**不允许出现任何
     * 方言转换调用**，tests/test_flow_rotation_comp_frame.py 会挡住回潮。
     */
    body_vx_m_s = *flow_vx_m_s;
    body_vy_m_s = *flow_vy_m_s;
    debug->sensor_velocity_m_s[0] = body_vx_m_s;
    debug->sensor_velocity_m_s[1] = body_vy_m_s;

    if (height_m > 0.0f) {
      /*
       * 光流把"地面点在像里的运动"报成载具速度：读数 = v_传感器 + ω × r_地面，
       * FLU 下 r_地面 = (0, 0, -h)。转动伪像 ω × r_地面 = (-hω_y, +hω_x)，补偿要减掉它。
       *
       * 2026-09-30 水平槽台架手摆实测：纯绕杆摆动时原始读数与旧补偿项 12/12 同号、
       * 幅值比约 1.18（=(h+杆下 8 cm)/h），即旧写法把伪像加了第二遍，每弧度漏进约 1 m
       * 假位移（data/analysis/sysid-rig-params/2026-09-30/xy_rock_check_*.txt）。
       */
      debug->optical_rot_comp_m_s[0] =
         STABILIZER_FLOW_ROT_COMP_GAIN * height_m * gyro_y_rad_s;
      debug->optical_rot_comp_m_s[1] =
        -STABILIZER_FLOW_ROT_COMP_GAIN * height_m * gyro_x_rad_s;

      /* 传感器相对 CG 的安装偏置：-(ω × r_安装)，偏置常量同样按 FLU 定义（Z 向下为负）。 */
      debug->offset_rot_comp_m_s[0] =
        -((gyro_y_rad_s * STABILIZER_FLOW_SENSOR_OFFSET_Z_M) -
          (gyro_z_rad_s * STABILIZER_FLOW_SENSOR_OFFSET_Y_M));
      debug->offset_rot_comp_m_s[1] =
        -((gyro_z_rad_s * STABILIZER_FLOW_SENSOR_OFFSET_X_M) -
          (gyro_x_rad_s * STABILIZER_FLOW_SENSOR_OFFSET_Z_M));
    }

    body_vx_m_s += debug->optical_rot_comp_m_s[0] +
                   debug->offset_rot_comp_m_s[0];
    body_vy_m_s += debug->optical_rot_comp_m_s[1] +
                   debug->offset_rot_comp_m_s[1];
    debug->corrected_velocity_m_s[0] = body_vx_m_s;
    debug->corrected_velocity_m_s[1] = body_vy_m_s;

    /*
     * 出口无需转换：进出都是 FLU。顺带修好的还有遥测——这三个 debug 分量原先在
     * 末尾被整体翻了一次 Y，地面站看到的补偿量一直是反的。
     */
    *flow_vx_m_s = body_vx_m_s;
    *flow_vy_m_s = body_vy_m_s;
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

  static uint8_t stabilizer_servo_command_slot_due(uint32_t now_ms)
  {
    uint32_t frame_start_ms = now_ms -
      (now_ms % STABILIZER_SERVO_BUS_FRAME_MS);

    if (stabilizer_last_servo_command_frame_ms == frame_start_ms) {
      return 0U;
    }
    stabilizer_last_servo_command_frame_ms = frame_start_ms;
    return 1U;
  }

  static void stabilizer_servo_commit_sent(const DRV_SERVO_MoveCmd moves[2],
                                           uint32_t now_ms)
  {
    stabilizer_last_sent_servo_pulse_us[0] = moves[0].pulse_us;
    stabilizer_last_sent_servo_pulse_us[1] = moves[1].pulse_us;
    stabilizer_last_successful_servo_pulse_us[0] = moves[0].pulse_us;
    stabilizer_last_successful_servo_pulse_us[1] = moves[1].pulse_us;
    stabilizer_last_servo_send_ms = now_ms;
  }

  static void stabilizer_servo_record_target(const DRV_SERVO_MoveCmd moves[2])
  {
    stabilizer_latest_servo_target_us[0] = moves[0].pulse_us;
    stabilizer_latest_servo_target_us[1] = moves[1].pulse_us;
  }

typedef struct
{
  APP_Sensor_SampleMessage last_msg;    /* 最后一次消费的传感器帧（含后处理字段） */
  float roll;                           /* 当前横滚角 [度] */
  float pitch;                          /* 当前俯仰角 [度] */
  float yaw;                            /* 当前偏航角 [度] */
  float roll_zero;
  float pitch_zero;
  float yaw_zero;
  float roll_control;
  float pitch_control;
  float yaw_control;
  float roll_zero_sum;
  float pitch_zero_sum;
  float yaw_zero_sum;
  uint32_t attitude_zero_count;
  uint32_t attitude_zero_start_ms;
  uint8_t attitude_zero_ready;
  uint64_t last_imu_timestamp_us;
  uint32_t last_out_ms;
  uint8_t has_imu_sample;
  DRV_AttitudeFusionOutput attitude_fusion;
  SVC_MAG_HeadingState magxy_state;
  float position_ref_x_m;
  float position_ref_y_m;
  uint8_t position_ref_xy_ready;
  uint8_t xy_brake_pending;        /* 定点档松杆后刹车中，尚未锁位 */
  float xy_brake_elapsed_s;
  float position_ref_z_m;
  uint8_t position_ref_z_ready;
  float height_ref_m;
  float height_origin_m;
  uint8_t height_origin_ready;
  float last_vertical_velocity_m_s;
  float vertical_accel_m_s2;
  uint32_t last_vertical_velocity_sample_ms;
  uint8_t vertical_accel_ready;
  float yaw_ref_rad;
  uint8_t yaw_ref_ready;
  StabilizerVofaDebug vofa_debug;
  APP_ControlSchedulerState control_scheduler;
  uint8_t flight_log_divider;
  uint8_t flight_log_subdiv_count; /* 四分之一相位上的子分频计数，见 stabilizer_flight_log_subdiv_due */
  uint16_t flight_log_tail_records;
  float last_gyro_rad_s[3];
  float gyro_ctrl_rad_s[3]; /* notched copy for control loops only; bitwise == gyro_rad_s when bypassed */
  uint8_t last_gyro_ready;
  uint8_t imu_frame_orientation_code;
  DRV_IMU_Calibration imu_calibration;
  uint32_t imu_calibration_generation;
  uint8_t imu_calibration_valid_mask;
  uint8_t imu_calibration_snapshot_ready;
} StabilizerContext;

typedef struct
{
  uint32_t now_ms;
  uint64_t now_us;
  uint16_t ch[16];
  APP_RcConfig rc_config;
  APP_RcInputs rc;
  DRV_COAX_CTRL_AttitudeInput attitude;
  DRV_COAX_CTRL_Reference reference;
  DRV_COAX_CTRL_Output ctrl_out;
  uint8_t rc_armed;
  uint8_t rc_link_ok;
  uint8_t rc_link_seen;
  uint16_t rc_throttle_motor_us;
  uint8_t rc_use_stabilized_motor_mix;
  uint8_t rc_attitude_debug_mode;
  uint8_t rc_control_motor_mix_allowed;
  /* 飞行油门模式（FLIGHT 且未旁路）才为 1；0 = 旧语义，下面几项全部不用。 */
  uint8_t ft_active_mode;
  uint8_t ft_spoolup;
  uint8_t ft_landed_idle;
  float ft_climb_rate_m_s;
  float ft_manual_force_n;   /* 状态机给的手动总推力：SPOOLUP 斜坡，或角度档 FLYING 的手控油门 */
  uint8_t ft_manual_integrate; /* 角度档推力够大（≥0.8·hover）：速率环积分照常工作 */
  float ft_hover_n;
  float ft_ground_spin_01;
  uint8_t ft_xy_velocity_hold; /* CH6 中位：水平只做速度闭环（杆给速度，回中=0），不跑位置 P */
  uint8_t rc_arm_switch_high;
  uint8_t rc_arm_throttle_low;
  uint8_t range_height_valid;
  float range_height_m;
  float range_velocity_m_s;
  float relative_height_m;
  uint32_t range_sample_ms;
  APP_LED_ArmBlockReason led_arm_block_reason;
  APP_FlightLogMotorOutputReason motor_output_reason;
  float ctrl_dt_sec;
  APP_ControlSchedule cascade_schedule;
  DRV_SERVO_MoveCmd moves[2];
  uint8_t imu_control_valid;
  /*
   * "有辨识在占用执行器"。两套辨识（老的 app_ident 与光杆台架的 app_sysid）对
   * 执行器链路的要求完全一样——接管舵机、电机走遥控器直通油门——所以下游全部
   * 联锁（LED 拦解锁原因、点桨窗口 inhibit、ESC 直通、飞行日志 motor_reason）
   * 共用这一个标志，新模块一条都不用重新接，也就不会漏接。
   */
  uint8_t ident_running;
  /* 具体是哪一套在跑，只用来决定舵机目标从谁那儿取。 */
  uint8_t sysid_running;
  APP_IdentAttLog ident_att_log;
  uint8_t servo_cal_active;
  /* 这一拍舵机脉宽是谁算的（APP_ServoBacklashSource），在算出脉宽的那一支里填；0 = 不补回差。 */
  uint8_t servo_source;
  /* 竖直通道估计器本拍输出；z_fusion = coax.z_vel_fusion ≥ 0.5 且估计有效，才替换测距量。 */
  DRV_ZEstOutput z_est;
  uint8_t z_fusion;
  /* 悬停推力在线学习（app_hover_thrust.h）的输入：准备阶段记下竖直加速度与原始测距。 */
  float az_up_m_s2;
  uint8_t range_raw_valid;
  float range_raw_m;
} StabilizerControlFrame;

/* 任务句柄（由 APP_Stabilizer_Run 参数注入） */
static osSemaphoreId_t s_imu_ready_sem;
static osMessageQueueId_t s_sensor_q;
static osMessageQueueId_t s_vofa_q;

/*
 * Single-writer/multi-reader validation snapshot.  The stabilizer task writes
 * the payload while the text-control task reads it.  The odd/even seqlock plus
 * data-memory barriers prevents a reader from accepting a torn 64-bit
 * timestamp or a mixture of two 1 kHz IMU samples on Cortex-M7.
 */
static volatile uint32_t stabilizer_validation_imu_seqlock;
static volatile uint8_t stabilizer_validation_imu_valid;
static volatile StabilizerValidationImuSnapshot
  stabilizer_validation_imu_snapshot;
/* 解锁状态快照（published=0 表示控制环还没跑过一圈）。写者只有控制环。 */
static APP_Stabilizer_ArmStatus stabilizer_arm_status;
/* 竖直通道估计器（IMU 竖直加速度 + 原始测距，drv_z_estimator.h）。写者只有控制环。 */
static DRV_ZEst stabilizer_z_est;

static volatile uint8_t stabilizer_imu_calibration_candidate_arm_lock;
static volatile uint8_t stabilizer_servo_calibration_candidate_arm_lock;

static void stabilizer_validation_imu_reset(void)
{
  stabilizer_validation_imu_seqlock++;
  BSP_Critical_MemoryBarrier();
  stabilizer_validation_imu_valid = 0U;
  BSP_Critical_MemoryBarrier();
  stabilizer_validation_imu_seqlock++;
}

static void stabilizer_validation_imu_publish(
  const APP_Sensor_SampleMessage *msg,
  uint32_t calibration_generation,
  uint8_t calibration_valid_mask)
{
  StabilizerValidationImuSnapshot next;

  if ((msg == NULL) || (msg->base.type != APP_SENSOR_TYPE_IMU)) {
    return;
  }

  memset(&next, 0, sizeof(next));
  next.timestamp_us = msg->base.timestamp_us;
  next.sequence = msg->base.sequence;
  next.sample_count = msg->base.sequence;
  next.accel_g[0] = msg->imu.accel_x_g;
  next.accel_g[1] = msg->imu.accel_y_g;
  next.accel_g[2] = msg->imu.accel_z_g;
  next.gyro_dps[0] = msg->imu.gyro_x_dps;
  next.gyro_dps[1] = msg->imu.gyro_y_dps;
  next.gyro_dps[2] = msg->imu.gyro_z_dps;
  next.temperature_c = msg->imu.temperature_c;
  next.roll_deg = msg->roll_deg;
  next.pitch_deg = msg->pitch_deg;
  next.yaw_deg = msg->yaw_deg;
  next.gyro_bias_ready = msg->gyro_bias_ready;
  next.fusion_acceleration_error_deg =
    msg->fusion_acceleration_error_deg;
  next.fusion_acceleration_recovery_trigger =
    msg->fusion_acceleration_recovery_trigger;
  next.fusion_accel_correction_count =
    msg->fusion_accel_correction_count;
  next.calibration_generation = calibration_generation;
  next.fusion_accelerometer_ignored =
    msg->fusion_accelerometer_ignored;
  next.fusion_acceleration_recovery =
    msg->fusion_acceleration_recovery;
  next.fusion_angular_rate_recovery =
    msg->fusion_angular_rate_recovery;
  next.fusion_accel_norm_rejected =
    msg->fusion_accel_norm_rejected;
  next.imu_frame_orientation_code = msg->imu_frame_orientation_code;
  next.calibration_valid_mask = calibration_valid_mask;
  next.armed = stabilizer_capture_armed;
  {
    APP_ImuHealthStatus health;

    APP_ImuHealth_GetStatus(&health);
    next.imu_sample_rate_hz = health.sample_rate_hz;
    next.imu_health_level = (uint8_t)health.level;
    next.imu_health_fault_active = health.fault_active;
    next.imu_health_fault_ever = health.fault_ever;
  }
  next.esc_pulse_us[0] = BSP_PWM_GetEscPulse(1U);
  next.esc_pulse_us[1] = BSP_PWM_GetEscPulse(2U);

  stabilizer_validation_imu_seqlock++;
  BSP_Critical_MemoryBarrier();
  stabilizer_validation_imu_snapshot = next;
  stabilizer_validation_imu_valid = 1U;
  BSP_Critical_MemoryBarrier();
  stabilizer_validation_imu_seqlock++;
}

/* ============================================================================
 * 跨任务公共 API（Sensor_Task / VOFA_task 调用）
 * ========================================================================== */
void APP_Stabilizer_LatchImuFault(StabilizerImuFaultReason reason)
{
  stabilizer_latch_imu_fault(reason);
}

void APP_Stabilizer_ClearImuFault(void)
{
  stabilizer_clear_imu_fault();
}

void APP_Stabilizer_MarkImuSample(uint32_t now_ms)
{
  stabilizer_imu_last_sample_ms = now_ms;
}

void APP_Stabilizer_ReadVofaDebug(StabilizerVofaDebug *out)
{
  stabilizer_vofa_debug_read(out);
}

uint8_t APP_Stabilizer_ReadValidationImuSnapshot(
  StabilizerValidationImuSnapshot *out)
{
  StabilizerValidationImuSnapshot current;
  uint32_t before;
  uint32_t after;
  uint32_t attempt;
  uint8_t valid;

  if (out == NULL) {
    return 0U;
  }

  for (attempt = 0U;
       attempt < STABILIZER_VALIDATION_SNAPSHOT_READ_RETRIES;
       ++attempt) {
    before = stabilizer_validation_imu_seqlock;
    if ((before & 1U) != 0U) {
      continue;
    }

    BSP_Critical_MemoryBarrier();
    current = stabilizer_validation_imu_snapshot;
    valid = stabilizer_validation_imu_valid;
    BSP_Critical_MemoryBarrier();
    after = stabilizer_validation_imu_seqlock;

    if ((before == after) && ((after & 1U) == 0U)) {
      if (valid == 0U) {
        return 0U;
      }
      *out = current;
      return 1U;
    }
  }

  return 0U;
}

uint8_t APP_Stabilizer_ReadFlowCompensationSnapshot(
  StabilizerFlowCompensationSnapshot *out)
{
  StabilizerFlowCompensationSnapshot current;
  uint32_t before;
  uint32_t after;
  uint32_t attempt;
  uint8_t valid;

  if (out == NULL) {
    return 0U;
  }
  for (attempt = 0U;
       attempt < STABILIZER_VALIDATION_SNAPSHOT_READ_RETRIES;
       ++attempt) {
    before = stabilizer_flow_comp_seqlock;
    if ((before & 1U) != 0U) {
      continue;
    }
    BSP_Critical_MemoryBarrier();
    current = stabilizer_flow_comp_snapshot;
    valid = stabilizer_flow_comp_valid;
    BSP_Critical_MemoryBarrier();
    after = stabilizer_flow_comp_seqlock;
    if ((before == after) && ((after & 1U) == 0U)) {
      if (valid == 0U) {
        return 0U;
      }
      *out = current;
      return 1U;
    }
  }
  return 0U;
}

uint8_t APP_Stabilizer_IsArmed(void)
{
  return stabilizer_capture_armed;
}

/* IMUZERO：命令任务置请求，稳定任务在姿态零点逻辑里取走、清零重新采样。 */
static volatile uint8_t stabilizer_rezero_request;
static volatile uint8_t stabilizer_attitude_zero_ready_mirror;

void APP_Stabilizer_RequestAttitudeRezero(void)
{
  stabilizer_rezero_request = 1U;
}

uint8_t APP_Stabilizer_IsAttitudeZeroReady(void)
{
  return stabilizer_attitude_zero_ready_mirror;
}

void APP_Stabilizer_GetMagFusionStatus(APP_Stabilizer_MagFusionStatus *out)
{
  if (out == NULL) {
    return;
  }
  out->subsystem_enabled = stabilizer_mag_fusion_mirror.subsystem_enabled;
  out->field_rejected = stabilizer_mag_fusion_mirror.field_rejected;
  out->used = stabilizer_mag_fusion_mirror.used;
  out->ignored = stabilizer_mag_fusion_mirror.ignored;
  out->recovery = stabilizer_mag_fusion_mirror.recovery;
  out->error_deg = stabilizer_mag_fusion_mirror.error_deg;
}

void APP_Stabilizer_GetMagXYStatus(APP_Stabilizer_MagXYStatus *out)
{
  if (out == NULL) {
    return;
  }
  out->ready_count = stabilizer_magxy_mirror.ready_count;
  out->tilt_count = stabilizer_magxy_mirror.tilt_count;
  out->z_mean_mgauss = stabilizer_magxy_mirror.z_mean_mgauss;
  out->enabled = stabilizer_magxy_mirror.enabled;
  out->last_result = stabilizer_magxy_mirror.last_result;
}

/*
 * 结构体整体读写，没有 seqlock：全部是 uint8/uint32 标量，撕裂最坏是两个相邻
 * 周期的字段混在一条报文里——而这些条件本来就在各自变化，混一拍不会得出
 * 一个"不存在的状态"。为一条 2 Hz 的显示报文加锁反而会把 1 kHz 控制环拖进来。
 */
void APP_Stabilizer_GetArmStatus(APP_Stabilizer_ArmStatus *out)
{
  if (out != NULL) {
    *out = stabilizer_arm_status;
  }
}

uint8_t APP_Stabilizer_IsImuFrameArmLocked(void)
{
  return ((stabilizer_imu_calibration_candidate_arm_lock != 0U) ||
          (stabilizer_servo_calibration_candidate_arm_lock != 0U) ||
          ((APP_Sensor_IsFluOrientationActive() != 0U) &&
           (DRV_FRAME_RUNTIME_MIGRATION_COMPLETE == 0U))) ? 1U : 0U;
}

void APP_Stabilizer_SetImuCalibrationCandidateArmLock(uint8_t locked)
{
  stabilizer_imu_calibration_candidate_arm_lock = (locked != 0U) ? 1U : 0U;
  BSP_Critical_MemoryBarrier();
}

uint8_t APP_Stabilizer_IsImuCalibrationCandidateArmLocked(void)
{
  return stabilizer_imu_calibration_candidate_arm_lock;
}

void APP_Stabilizer_SetServoCalibrationCandidateArmLock(uint8_t locked)
{
  stabilizer_servo_calibration_candidate_arm_lock =
    (locked != 0U) ? 1U : 0U;
  BSP_Critical_MemoryBarrier();
}

uint8_t APP_Stabilizer_IsServoCalibrationCandidateArmLocked(void)
{
  return stabilizer_servo_calibration_candidate_arm_lock;
}

static void stabilizer_init(StabilizerContext *ctx)
{
  memset(ctx, 0, sizeof(*ctx));
  APP_RpmNotch_Init();
  APP_ServoBacklash_Init();
  stabilizer_imu_calibration_candidate_arm_lock = 0U;
  stabilizer_servo_calibration_candidate_arm_lock = 0U;
  stabilizer_flow_comp_seqlock = 0U;
  stabilizer_flow_comp_valid = 0U;
  memset((void *)&stabilizer_flow_comp_snapshot, 0,
         sizeof(stabilizer_flow_comp_snapshot));
  ctx->imu_frame_orientation_code = APP_SENSOR_FLU_ORIENTATION_LEGACY;
  stabilizer_validation_imu_reset();
  DRV_AttitudeFusion_Init();
  SVC_FlowNav_ResetEstimator();
  DRV_COAX_CTRL_ResetState();
  APP_ControlScheduler_Reset(&ctx->control_scheduler);
  (void)DRV_ZEst_Init(&stabilizer_z_est, NULL);
  APP_HoverThrust_Init();
}

static void stabilizer_reset_for_imu_frame(
  StabilizerContext *ctx,
  uint8_t orientation_code,
  uint32_t calibration_generation)
{
  const uint8_t flu_active =
    (orientation_code != APP_SENSOR_FLU_ORIENTATION_LEGACY) ? 1U : 0U;

  ctx->roll = 0.0f;
  ctx->pitch = 0.0f;
  ctx->yaw = 0.0f;
  ctx->roll_zero = 0.0f;
  ctx->pitch_zero = 0.0f;
  ctx->yaw_zero = 0.0f;
  ctx->roll_control = 0.0f;
  ctx->pitch_control = 0.0f;
  ctx->yaw_control = 0.0f;
  ctx->roll_zero_sum = 0.0f;
  ctx->pitch_zero_sum = 0.0f;
  ctx->yaw_zero_sum = 0.0f;
  ctx->attitude_zero_count = 0U;
  ctx->attitude_zero_start_ms = 0U;
  ctx->attitude_zero_ready = 0U;
  ctx->last_imu_timestamp_us = 0ULL;
  ctx->has_imu_sample = 0U;
  memset(&ctx->attitude_fusion, 0, sizeof(ctx->attitude_fusion));
  SVC_MAG_HeadingReset(&ctx->magxy_state);
  SVC_FlowNav_ResetEstimator();
  ctx->position_ref_x_m = 0.0f;
  ctx->position_ref_y_m = 0.0f;
  ctx->position_ref_xy_ready = 0U;
  ctx->xy_brake_pending = 0U;
  ctx->xy_brake_elapsed_s = 0.0f;
  ctx->position_ref_z_m = 0.0f;
  ctx->position_ref_z_ready = 0U;
  ctx->height_ref_m = 0.0f;
  ctx->height_origin_m = 0.0f;
  ctx->height_origin_ready = 0U;
  ctx->yaw_ref_rad = 0.0f;
  ctx->yaw_ref_ready = 0U;
  memset(ctx->last_gyro_rad_s, 0, sizeof(ctx->last_gyro_rad_s));
  ctx->last_gyro_ready = 0U;
  APP_RpmNotch_ResetState();
  ctx->last_vertical_velocity_sample_ms = 0U;
  ctx->vertical_accel_ready = 0U;
  ctx->imu_frame_orientation_code = orientation_code;
  ctx->imu_calibration_generation = calibration_generation;

  DRV_AttitudeFusion_InitForConvention(
    (flu_active != 0U) ? DRV_ATTITUDE_FUSION_CONVENTION_NWU :
                         DRV_ATTITUDE_FUSION_CONVENTION_NED);
  SVC_FlowNav_ResetEstimator();
  DRV_COAX_CTRL_ResetState();
  APP_ControlScheduler_Reset(&ctx->control_scheduler);
  DRV_ZEst_Reset(&stabilizer_z_est);   /* 加速度轴向换了，学到的零偏不再算数 */
  APP_HoverThrust_RequestReset();      /* 同理：按旧轴向学的悬停推力作废 */
  stabilizer_rc_arm_latched = 0U;
  stabilizer_validation_imu_reset();
}

static void stabilizer_imu_step(StabilizerContext *ctx,
                                 APP_Sensor_SampleMessage *msg)
{
  float dt_sec = SENSOR_IMU_DEFAULT_DT_SEC;
  float gyro_rad_s[3];
  float alpha_rad_s2[3] = {0.0f, 0.0f, 0.0f};
  uint8_t alpha_valid = 0U;
  const uint8_t orientation_code =
    APP_Sensor_ApplyFrameCorrection(&msg->imu);
  const uint8_t flu_active =
    (orientation_code != APP_SENSOR_FLU_ORIENTATION_LEGACY) ? 1U : 0U;
  APP_FlightCalibrationSnapshot calibration_snapshot;
  DRV_COAX_CTRL_ServoCalibration servo_calibration;
  uint8_t calibration_changed = 0U;

  msg->imu_frame_orientation_code = orientation_code;
  if (APP_FlightCalibration_ReadActive(&calibration_snapshot) != 0U) {
    if ((ctx->imu_calibration_snapshot_ready == 0U) ||
        (orientation_code != ctx->imu_frame_orientation_code) ||
        (calibration_snapshot.generation !=
         ctx->imu_calibration_generation)) {
      ctx->imu_calibration_valid_mask =
        APP_FlightCalibration_BuildImuCalibration(
          &calibration_snapshot.calibration,
          orientation_code,
          &ctx->imu_calibration);
      (void)APP_FlightCalibration_BuildServoMechanical(
        &calibration_snapshot.calibration,
        &servo_calibration);
      (void)DRV_COAX_CTRL_SetServoCalibration(&servo_calibration);
      stabilizer_latest_servo_target_us[0] =
        servo_calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX];
      stabilizer_latest_servo_target_us[1] =
        servo_calibration.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX];
      ctx->imu_calibration_snapshot_ready = 1U;
      calibration_changed = 1U;
    }
  } else if (orientation_code != ctx->imu_frame_orientation_code) {
    /* Never carry coefficients across a frame change without provenance. */
    memset(&ctx->imu_calibration, 0, sizeof(ctx->imu_calibration));
    ctx->imu_calibration_valid_mask = 0U;
    ctx->imu_calibration_snapshot_ready = 0U;
    DRV_COAX_CTRL_ResetServoCalibration();
    DRV_COAX_CTRL_GetServoCalibration(&servo_calibration);
    stabilizer_latest_servo_target_us[0] =
      servo_calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX];
    stabilizer_latest_servo_target_us[1] =
      servo_calibration.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX];
    calibration_snapshot.generation = ctx->imu_calibration_generation;
    calibration_changed = 1U;
  }
  if (calibration_changed != 0U) {
    stabilizer_reset_for_imu_frame(ctx,
                                   orientation_code,
                                   calibration_snapshot.generation);
  }

  /*
   * V1 is applied exactly once to the complete frame: after V0 has rotated
   * both vectors together, and before Fusion/navigation/controller reads any
   * component. Sensor_Task already removed its per-boot stationary gyro trim,
   * therefore gyro_bias_ref is the residual after that trim.
   */
  {
    float accel_g[3] = {
      msg->imu.accel_x_g, msg->imu.accel_y_g, msg->imu.accel_z_g
    };
    float gyro_dps[3] = {
      msg->imu.gyro_x_dps, msg->imu.gyro_y_dps, msg->imu.gyro_z_dps
    };
    const uint8_t applied = DRV_IMU_Calibration_Apply(
      &ctx->imu_calibration, msg->imu.temperature_c, accel_g, gyro_dps);

    if ((applied & DRV_IMU_CAL_VALID_ACCEL) != 0U) {
      msg->imu.accel_x_g = accel_g[0];
      msg->imu.accel_y_g = accel_g[1];
      msg->imu.accel_z_g = accel_g[2];
    }
    if ((applied & DRV_IMU_CAL_VALID_GYRO) != 0U) {
      msg->imu.gyro_x_dps = gyro_dps[0];
      msg->imu.gyro_y_dps = gyro_dps[1];
      msg->imu.gyro_z_dps = gyro_dps[2];
    }
  }

  if ((ctx->last_imu_timestamp_us != 0ULL) &&
      (msg->base.timestamp_us > ctx->last_imu_timestamp_us)) {
    uint64_t dt_us = msg->base.timestamp_us - ctx->last_imu_timestamp_us;
    if ((dt_us >= SENSOR_IMU_MIN_DT_US) &&
        (dt_us <= SENSOR_IMU_MAX_DT_US)) {
      dt_sec = (float)dt_us * 0.000001f; /* 微秒 → 秒                  */
    }
  }
  ctx->last_imu_timestamp_us = msg->base.timestamp_us;
  gyro_rad_s[0] = msg->imu.gyro_x_dps * STABILIZER_DEG_TO_RAD;
  gyro_rad_s[1] = msg->imu.gyro_y_dps * STABILIZER_DEG_TO_RAD;
  gyro_rad_s[2] = msg->imu.gyro_z_dps * STABILIZER_DEG_TO_RAD;
  /* 转速陷波只给控制环（app_rpm_notch.h）；下面的 alpha/Fusion 仍读原始陀螺。 */
  APP_RpmNotch_ApplySample(gyro_rad_s, ctx->gyro_ctrl_rad_s, msg->base.timestamp_us);
  if ((ctx->last_gyro_ready != 0U) &&
      (dt_sec > 0.0f) &&
      (dt_sec <= 0.02f)) {
    alpha_rad_s2[0] = (gyro_rad_s[0] - ctx->last_gyro_rad_s[0]) / dt_sec;
    alpha_rad_s2[1] = (gyro_rad_s[1] - ctx->last_gyro_rad_s[1]) / dt_sec;
    alpha_rad_s2[2] = (gyro_rad_s[2] - ctx->last_gyro_rad_s[2]) / dt_sec;
    alpha_valid = 1U;
  }
  ctx->last_gyro_rad_s[0] = gyro_rad_s[0];
  ctx->last_gyro_rad_s[1] = gyro_rad_s[1];
  ctx->last_gyro_rad_s[2] = gyro_rad_s[2];
  ctx->last_gyro_ready = 1U;

  /*
   * ① x-io Fusion AHRS。
   *
   * legacy 模式保留原 NED 过渡适配，以便新固件在尚未应用候选时行为不变。
   * 一旦 V0 候选生效，msg->imu 已由同一个 proper rotation 同时变为 FLU；
   * Fusion 随 frame generation 重置为 NWU，并直接消费 FLU accel/gyro，顶层
   * 不再靠分散负号补偿安装方向。
   */
  if (msg->gyro_bias_ready != 0U) {
    DRV_AttitudeFusionInput fusion_input = {0};
    fusion_input.time_us = msg->base.timestamp_us;
    fusion_input.dt_s = dt_sec;
    if (flu_active != 0U) {
      fusion_input.gyroscope_dps[0] = msg->imu.gyro_x_dps;
      fusion_input.gyroscope_dps[1] = msg->imu.gyro_y_dps;
      fusion_input.gyroscope_dps[2] = msg->imu.gyro_z_dps;
      fusion_input.accelerometer_g[0] = msg->imu.accel_x_g;
      fusion_input.accelerometer_g[1] = msg->imu.accel_y_g;
      fusion_input.accelerometer_g[2] = msg->imu.accel_z_g;
    } else {
      fusion_input.gyroscope_dps[0] = -msg->imu.gyro_x_dps;
      fusion_input.gyroscope_dps[1] =  msg->imu.gyro_y_dps;
      fusion_input.gyroscope_dps[2] =  msg->imu.gyro_z_dps;
      fusion_input.accelerometer_g[0] = -msg->imu.accel_x_g;
      fusion_input.accelerometer_g[1] =  msg->imu.accel_y_g;
      fusion_input.accelerometer_g[2] = -msg->imu.accel_z_g;
    }

    /* The default 3-D MAGCAL branch is unchanged. The explicitly enabled
     * XY mode uses a separate physical axis proof, the pre-MAGCAL
     * FLU vector and only one Fusion update per fresh 20-Hz magnetic sample.
     * Snapshot/config reads are bounded critical-section copies, no I/O.
     * The fourth (field-magnitude) gate remains in drv_attitude_fusion.
     */
    {
      APP_MAG_Snapshot mag_snapshot;
      SVC_MAG_HeadingConfig magxy_config;
      uint64_t mag_now_us;
      uint64_t mag_age_us;

      APP_MAG_GetSnapshot(&mag_snapshot);
      APP_MagXY_GetConfig(&magxy_config);
      mag_now_us = SVC_Timestamp_Us();
      mag_age_us = (mag_now_us >= mag_snapshot.timestamp_us) ?
        (mag_now_us - mag_snapshot.timestamp_us) : 0ULL;

      stabilizer_magxy_mirror.enabled = magxy_config.enabled;
      if (magxy_config.enabled != 0U) {
        if ((magxy_config.axis_verified != 0U) &&
            (magxy_config.frame_contract_version ==
             (uint32_t)DRV_FRAME_CONTRACT_VERSION) &&
            (mag_snapshot.sensor_healthy != 0U) &&
            (mag_snapshot.timestamp_us != 0ULL) &&
            (mag_age_us <= APP_MAG_SNAPSHOT_MAX_AGE_US)) {
          SVC_MAG_HeadingInput magxy_input;
          SVC_MAG_HeadingResult result;
          float corrected_flu_mgauss[3];

          magxy_input.timestamp_us = mag_snapshot.timestamp_us;
          magxy_input.raw_flu_mgauss[0] =
            mag_snapshot.raw_field_flu_mgauss[0];
          magxy_input.raw_flu_mgauss[1] =
            mag_snapshot.raw_field_flu_mgauss[1];
          magxy_input.raw_flu_mgauss[2] =
            mag_snapshot.raw_field_flu_mgauss[2];
          magxy_input.roll_deg = ctx->roll;
          magxy_input.pitch_deg = ctx->pitch;
          magxy_input.yaw_deg = ctx->yaw;
          result = SVC_MAG_HeadingUpdate(&magxy_config, &ctx->magxy_state,
                                         &magxy_input,
                                         corrected_flu_mgauss);
          if (result != SVC_MAG_HEADING_REPEATED) {
            stabilizer_magxy_mirror.last_result = (uint8_t)result;
          }
          stabilizer_magxy_mirror.ready_count =
            ctx->magxy_state.ready_count;
          stabilizer_magxy_mirror.tilt_count =
            ctx->magxy_state.tilt_count;
          stabilizer_magxy_mirror.z_mean_mgauss =
            ctx->magxy_state.last_z_mean_mgauss;
          if (result == SVC_MAG_HEADING_READY) {
            fusion_input.magnetometer_mgauss[0] = corrected_flu_mgauss[0];
            fusion_input.magnetometer_mgauss[1] = corrected_flu_mgauss[1];
            fusion_input.magnetometer_mgauss[2] = corrected_flu_mgauss[2];
            fusion_input.magnetometer_valid = 1U;
          }
        } else {
          stabilizer_magxy_mirror.last_result =
            (uint8_t)SVC_MAG_HEADING_INVALID;
        }
      } else if ((mag_snapshot.calibrated != 0U) &&
           (mag_snapshot.axis_verified != 0U) &&
           (mag_snapshot.timestamp_us != 0ULL) &&
           (mag_age_us <= APP_MAG_SNAPSHOT_MAX_AGE_US)) {
        fusion_input.magnetometer_mgauss[0] = mag_snapshot.field_flu_mgauss[0];
        fusion_input.magnetometer_mgauss[1] = mag_snapshot.field_flu_mgauss[1];
        fusion_input.magnetometer_mgauss[2] = mag_snapshot.field_flu_mgauss[2];
        fusion_input.magnetometer_valid = 1U;
      }
    }

    if (DRV_AttitudeFusion_Update(&fusion_input, &ctx->attitude_fusion) != 0U) {
      ctx->roll = ctx->attitude_fusion.roll_deg;
      ctx->pitch = ctx->attitude_fusion.pitch_deg;
      ctx->yaw = ctx->attitude_fusion.yaw_deg;
      ctx->has_imu_sample = 1U;
    }
    stabilizer_mag_fusion_mirror.subsystem_enabled =
      ctx->attitude_fusion.magnetometer_subsystem_enabled;
    stabilizer_mag_fusion_mirror.field_rejected =
      ctx->attitude_fusion.magnetometer_field_rejected;
    stabilizer_mag_fusion_mirror.used = ctx->attitude_fusion.magnetometer_used;
    stabilizer_mag_fusion_mirror.ignored =
      ctx->attitude_fusion.magnetometer_ignored;
    stabilizer_mag_fusion_mirror.recovery =
      ctx->attitude_fusion.magnetic_recovery;
    stabilizer_mag_fusion_mirror.error_deg =
      ctx->attitude_fusion.magnetic_error_deg;
  }

  msg->fusion_acceleration_error_deg =
    ctx->attitude_fusion.acceleration_error_deg;
  msg->fusion_acceleration_recovery_trigger =
    ctx->attitude_fusion.acceleration_recovery_trigger;
  msg->fusion_accel_correction_count =
    ctx->attitude_fusion.accel_correction_count;
  msg->fusion_accelerometer_ignored =
    ctx->attitude_fusion.accelerometer_ignored;
  msg->fusion_acceleration_recovery =
    ctx->attitude_fusion.acceleration_recovery;
  msg->fusion_angular_rate_recovery =
    ctx->attitude_fusion.angular_rate_recovery;
  msg->fusion_accel_norm_rejected =
    ctx->attitude_fusion.accel_norm_rejected;

  /*
   * 把姿态估计和融合健康状态标注到刚采集的原始帧上。放在这里而不是控制
   * 输出之后，是因为这些量此刻最新，而且不依赖解锁状态——地面定速测振时
   * 同样需要它们。舵机指令在本帧尚未算出，改由下一帧带上。
   */
  {
    uint8_t capture_fusion_flags = 0U;
    if (ctx->attitude_fusion.accelerometer_ignored != 0U) {
      capture_fusion_flags |= APP_IMU_CAPTURE_FUSION_ACCEL_IGNORED;
    }
    if (ctx->attitude_fusion.accel_norm_rejected != 0U) {
      capture_fusion_flags |= APP_IMU_CAPTURE_FUSION_NORM_REJECTED;
    }
    if (ctx->attitude_fusion.acceleration_recovery != 0U) {
      capture_fusion_flags |= APP_IMU_CAPTURE_FUSION_ACCEL_RECOVERY;
    }
    if (ctx->attitude_fusion.angular_rate_recovery != 0U) {
      capture_fusion_flags |= APP_IMU_CAPTURE_FUSION_RATE_RECOVERY;
    }
    if (ctx->attitude_fusion.startup != 0U) {
      capture_fusion_flags |= APP_IMU_CAPTURE_FUSION_STARTUP;
    }
    APP_IMU_Capture_AnnotateControl(
      (uint32_t)msg->base.timestamp_us,
      ctx->roll, ctx->pitch, ctx->yaw,
      stabilizer_last_successful_servo_pulse_us[0],
      stabilizer_last_successful_servo_pulse_us[1],
      ctx->attitude_fusion.acceleration_error_deg,
      capture_fusion_flags,
      stabilizer_capture_armed);
  }

  /* Apparent acceleration tilt is retained for diagnosis only. */
  if (flu_active != 0U) {
    msg->attitude_debug.roll_acc_deg =
      atan2f(msg->imu.accel_y_g, msg->imu.accel_z_g) /
      STABILIZER_DEG_TO_RAD;
    msg->attitude_debug.pitch_acc_deg =
      atan2f(-msg->imu.accel_x_g,
             sqrtf(msg->imu.accel_y_g * msg->imu.accel_y_g +
                   msg->imu.accel_z_g * msg->imu.accel_z_g)) /
      STABILIZER_DEG_TO_RAD;
  } else {
    msg->attitude_debug.roll_acc_deg =
      atan2f(-msg->imu.accel_y_g, msg->imu.accel_z_g) /
      STABILIZER_DEG_TO_RAD;
    msg->attitude_debug.pitch_acc_deg =
      atan2f(-msg->imu.accel_x_g,
             sqrtf(msg->imu.accel_y_g * msg->imu.accel_y_g +
                   msg->imu.accel_z_g * msg->imu.accel_z_g)) /
      STABILIZER_DEG_TO_RAD;
  }
  msg->attitude_debug.roll_gyro_deg = ctx->roll;
  msg->attitude_debug.pitch_gyro_deg = ctx->pitch;
  msg->attitude_debug.roll_residual_deg =
    msg->attitude_debug.roll_acc_deg - ctx->roll;
  msg->attitude_debug.pitch_residual_deg =
    msg->attitude_debug.pitch_acc_deg - ctx->pitch;
  msg->attitude_debug.accel_norm_g =
    sqrtf(msg->imu.accel_x_g * msg->imu.accel_x_g +
          msg->imu.accel_y_g * msg->imu.accel_y_g +
          msg->imu.accel_z_g * msg->imu.accel_z_g);
  msg->attitude_debug.accel_trust = 0.0f;
  msg->attitude_debug.accel_residual_deg =
    sqrtf(msg->attitude_debug.roll_residual_deg *
          msg->attitude_debug.roll_residual_deg +
          msg->attitude_debug.pitch_residual_deg *
          msg->attitude_debug.pitch_residual_deg);
  msg->attitude_debug.alpha = 1.0f;
  msg->attitude_debug.dt_ms = dt_sec * 1000.0f;

  if (stabilizer_rezero_request != 0U) {
    stabilizer_rezero_request = 0U;
    ctx->attitude_zero_ready = 0U;
    ctx->attitude_zero_start_ms = 0U;
    ctx->attitude_zero_count = 0U;
    ctx->roll_zero_sum = 0.0f;
    ctx->pitch_zero_sum = 0.0f;
    ctx->yaw_zero_sum = 0.0f;
  }
  if ((ctx->attitude_zero_ready == 0U) &&
      (msg->gyro_bias_ready != 0U) &&
      (ctx->attitude_fusion.initialized != 0U) &&
      (ctx->attitude_fusion.startup == 0U) &&
      (ctx->attitude_fusion.accelerometer_ignored == 0U) &&
      (ctx->attitude_fusion.accel_norm_rejected == 0U) &&
      (ctx->attitude_fusion.acceleration_error_deg <=
       STABILIZER_ATTITUDE_ZERO_ERROR_MAX_DEG) &&
      (msg->attitude_debug.accel_norm_g >=
       STABILIZER_ATTITUDE_ZERO_ACCEL_MIN_G) &&
      (msg->attitude_debug.accel_norm_g <=
       STABILIZER_ATTITUDE_ZERO_ACCEL_MAX_G) &&
      (fabsf(msg->imu.gyro_x_dps) <=
       STABILIZER_ATTITUDE_ZERO_GYRO_MAX_DPS) &&
      (fabsf(msg->imu.gyro_y_dps) <=
       STABILIZER_ATTITUDE_ZERO_GYRO_MAX_DPS) &&
      (fabsf(msg->imu.gyro_z_dps) <=
       STABILIZER_ATTITUDE_ZERO_GYRO_MAX_DPS)) {
    if (ctx->attitude_zero_start_ms == 0U) {
      ctx->attitude_zero_start_ms = SVC_Timestamp_Ms();
    }
    ctx->roll_zero_sum += ctx->roll;
    ctx->pitch_zero_sum += ctx->pitch;
    ctx->yaw_zero_sum += ctx->yaw;
    ++ctx->attitude_zero_count;

    if (((SVC_Timestamp_Ms() - ctx->attitude_zero_start_ms) >= STABILIZER_ATTITUDE_ZERO_MS) &&
        (ctx->attitude_zero_count > 0U)) {
      ctx->roll_zero = ctx->roll_zero_sum / (float)ctx->attitude_zero_count;
      ctx->pitch_zero = ctx->pitch_zero_sum / (float)ctx->attitude_zero_count;
      ctx->yaw_zero = ctx->yaw_zero_sum / (float)ctx->attitude_zero_count;
      ctx->attitude_zero_ready = 1U;
    }
  } else if (ctx->attitude_zero_ready == 0U) {
    /* Require one uninterrupted static window; motion restarts calibration. */
    ctx->attitude_zero_start_ms = 0U;
    ctx->attitude_zero_count = 0U;
    ctx->roll_zero_sum = 0.0f;
    ctx->pitch_zero_sum = 0.0f;
    ctx->yaw_zero_sum = 0.0f;
  }
  stabilizer_attitude_zero_ready_mirror = ctx->attitude_zero_ready;

  if (ctx->attitude_zero_ready != 0U) {
    ctx->roll_control = ctx->roll - ctx->roll_zero;
    ctx->pitch_control = ctx->pitch - ctx->pitch_zero;
    ctx->yaw_control = ctx->yaw - ctx->yaw_zero;
    if (ctx->yaw_control > 180.0f) {
      ctx->yaw_control -= 360.0f;
    } else if (ctx->yaw_control < -180.0f) {
      ctx->yaw_control += 360.0f;
    }
  } else {
    ctx->roll_control = 0.0f;
    ctx->pitch_control = 0.0f;
    ctx->yaw_control = 0.0f;
  }

  {
    {
      float imu_accel_x_m_s2 = 0.0f;
      float imu_accel_y_m_s2 = 0.0f;
      float imu_accel_weight = 0.0f;
      float flow_vx_m_s = 0.0f;
      float flow_vy_m_s = 0.0f;
      float flow_height_m = 0.0f;
      uint32_t flow_sample_ms = 0U;
      APP_OPTICAL_FLOW_Status flow_status;
      uint32_t flow_height_sample_ms = 0U;
      uint8_t flow_valid =
        APP_OpticalFlow_GetVelocitySample(&flow_vx_m_s,
                                          &flow_vy_m_s,
                                          &flow_sample_ms);
      uint8_t flow_accepted = 0U;

      memset(&flow_status, 0, sizeof(flow_status));
      APP_OpticalFlow_GetStatus(&flow_status);
      if (ctx->attitude_zero_ready != 0U) {
        /* 转平用 Fusion 绝对姿态，不用减过开机零位的 *_control：比力和 Fusion 姿态出自同一只
         * 加速度计，只有两者配对才能把重力扣干净。用零位姿态会把开机倾角（地面不平、IMU 装歪、
         * 加速度计零偏）留成 g·sin(零位) 的假水平加速度——2026-10-01 台架实测零位约 1.9°/1.3°，
         * 合 0.2~0.4 m/s²，正卡在零速判定门限附近。 */
        stabilizer_compensated_imu_accel_level_xy(
          msg->imu.accel_x_g,
          msg->imu.accel_y_g,
          msg->imu.accel_z_g,
          ctx->roll * STABILIZER_DEG_TO_RAD,
          ctx->pitch * STABILIZER_DEG_TO_RAD,
          gyro_rad_s,
          alpha_rad_s2,
          alpha_valid,
          &imu_accel_x_m_s2,
          &imu_accel_y_m_s2,
          &imu_accel_weight);
      }

      if (flow_valid != 0U) {
        if ((APP_OpticalFlow_GetHeightSample(&flow_height_m,
                                             NULL,
                                             &flow_height_sample_ms) == 0U) ||
            (flow_height_sample_ms == 0U)) {
          flow_height_m = 0.0f;
        }
        stabilizer_compensate_flow_rotation(
          &flow_vx_m_s,
          &flow_vy_m_s,
          flow_height_m,
          gyro_rad_s[0],
          gyro_rad_s[1],
          gyro_rad_s[2],
          &stabilizer_flow_debug);
      } else {
        memset(&stabilizer_flow_debug, 0, sizeof(stabilizer_flow_debug));
      }
      stabilizer_flow_compensation_publish(
        &stabilizer_flow_debug,
        flow_sample_ms,
        msg->imu_frame_orientation_code,
        flow_valid);

      {
        SVC_FLOW_NAV_FuseInput fuse_input;

        memset(&fuse_input, 0, sizeof(fuse_input));
        fuse_input.accel_x_m_s2 = imu_accel_x_m_s2;
        fuse_input.accel_y_m_s2 = imu_accel_y_m_s2;
        /* 机头对齐系随偏航转动，Service 用它补 -ω_z × v 运动学项。 */
        fuse_input.yaw_rate_rad_s = gyro_rad_s[2];
        fuse_input.flow_vx_m_s = flow_vx_m_s;
        fuse_input.flow_vy_m_s = flow_vy_m_s;
        fuse_input.flow_valid = flow_valid;
        fuse_input.flow_quality = flow_status.flow_quality;
        fuse_input.flow_sample_ms = flow_sample_ms;
        fuse_input.dt_sec = dt_sec;
        fuse_input.now_ms = SVC_Timestamp_Ms();
        flow_accepted = SVC_FlowNav_Fuse(&fuse_input);
      }
      APP_NavEstimator_PublishVelocityEKF();
      if (flow_accepted != 0U) {
        APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_FLOW);
      } else {
        APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_NONE);
      }
      ctx->vofa_debug.acc_nav_m_s2[0] = imu_accel_x_m_s2;
      ctx->vofa_debug.acc_nav_m_s2[1] = imu_accel_y_m_s2;
      (void)imu_accel_weight;
    }
    {
      float nav_vx_m_s = 0.0f;
      float nav_vy_m_s = 0.0f;

      SVC_FlowNav_GetVelocity(&nav_vx_m_s, &nav_vy_m_s);
      ctx->vofa_debug.vel_est_m_s[0] = nav_vx_m_s;
      ctx->vofa_debug.vel_est_m_s[1] = nav_vy_m_s;
    }
    stabilizer_vofa_debug_publish(&ctx->vofa_debug);
  }

  /*
   * ② 写入 VOFA 日志队列（覆盖模式）
   * vofaLogQueue 深度为 1，如果队列满则丢弃旧数据再写入新数据。
   * VOFA_task 以 50Hz 频率从此队列读取并发送到上位机。
   */
  msg->roll_deg  = ctx->roll_control;
  msg->pitch_deg = ctx->pitch_control;
  msg->yaw_deg   = ctx->yaw_control;
  stabilizer_validation_imu_publish(msg,
                                    ctx->imu_calibration_generation,
                                    ctx->imu_calibration_valid_mask);

  if (osMessageQueuePut(s_vofa_q, msg, 0U, 0U) != osOK) {
    APP_Sensor_SampleMessage drop;
    (void)osMessageQueueGet(s_vofa_q, &drop, 0U, 0U);
    (void)osMessageQueuePut(s_vofa_q, msg, 0U, 0U);
  }
}

/*
 * 竖直通道估计器的一拍（drv_z_estimator.h）。每个控制拍都跑，coax.z_vel_fusion 只决定下游
 * 用不用——切换时估计早已收敛，下游不跳；开关为 0 时下游拿到的仍是原来的测距量。
 * 加速度由调用处给（与 ALT 观测同一个值）；测距用服务里未平滑的原始值及其到达时刻，
 * 与 frame->now_ms 同一个 ms 钟。
 */
static void stabilizer_z_estimator_step(StabilizerControlFrame *frame,
                                        const SVC_FLOW_NAV_State *nav,
                                        float a_up_m_s2)
{
  DRV_ZEstTickInput in;
  float z_vel_fusion = 0.0f;

  in.now_us = frame->now_us;
  in.now_ms = frame->now_ms;
  in.a_up_m_s2 = a_up_m_s2;
  in.accel_valid = frame->imu_control_valid;
  in.range_valid = nav->height_valid;
  in.range_m = nav->height_raw_m;
  in.range_sample_ms = nav->height_sample_ms;
  (void)DRV_ZEst_Tick(&stabilizer_z_est, &in);
  DRV_ZEst_GetOutput(&stabilizer_z_est, &frame->z_est);
  (void)DRV_COAX_CTRL_GetParam("coax.z_vel_fusion", &z_vel_fusion);
  frame->z_fusion = ((z_vel_fusion >= 0.5f) && (frame->z_est.valid != 0U)) ? 1U : 0U;
}

static void stabilizer_control_prepare(StabilizerContext *ctx,
                                       StabilizerControlFrame *frame)
{
  memset(&frame->attitude, 0, sizeof(frame->attitude));
  memset(&frame->reference, 0, sizeof(frame->reference));
  APP_ELRS_GetChannels(frame->ch);       /* 读取 ELRS 遥控器 16 通道             */
  /*
   * 每周期重读一次映射快照：上位机改绑定/标定后必须立刻生效，否则用户得重启飞控
   * 才能看到效果。seqlock 读是几十个周期的拷贝，1kHz 下可以忽略。
   */
  if (APP_RcConfig_ReadActive(&frame->rc_config) == 0U) {
    APP_RcConfig_Defaults(&frame->rc_config);
  }
  APP_RcConfig_Resolve(&frame->rc_config, frame->ch, &frame->rc);
  frame->rc_link_ok = APP_ELRS_IsRcFresh(frame->now_ms, STABILIZER_RC_LOSS_TIMEOUT_MS);
  frame->rc_link_seen = (APP_ELRS_GetLastRcMs() != 0U) ? 1U : 0U;
  frame->rc_arm_switch_high =
    stabilizer_rc_channel_above_percent(&frame->rc_config, APP_RC_FUNC_ARM,
                                        frame->rc.us[APP_RC_FUNC_ARM],
                                        STABILIZER_RC_SWITCH_HIGH_PERCENT);
  frame->rc_arm_throttle_low =
    (frame->rc.throttle_01 <=
     ((float)STABILIZER_RC_THROTTLE_ARM_LOW_PERCENT / 100.0f)) ? 1U : 0U;
  frame->rc_armed = stabilizer_rc_update_armed(frame->rc_arm_switch_high,
                                               frame->rc_arm_throttle_low,
                                               frame->rc_link_ok);
  stabilizer_capture_armed = frame->rc_armed;
  APP_RcAux_Update(frame->now_ms, frame->ch, frame->rc_link_ok, frame->rc_armed); /* CH9 手动 IMU 归零 */
  frame->rc_throttle_motor_us =
    stabilizer_rc_throttle_to_motor_pulse(frame->rc.throttle_01);
  frame->rc_use_stabilized_motor_mix =
    stabilizer_rc_use_stabilized_motor_mix(frame->rc.throttle_01);
  frame->rc_attitude_debug_mode =
    ((frame->rc_link_ok != 0U) &&
     (frame->rc_use_stabilized_motor_mix != 0U) &&
     (stabilizer_rc_channel_above_percent(
        &frame->rc_config, APP_RC_FUNC_MODE,
        frame->rc.us[APP_RC_FUNC_MODE],
        STABILIZER_RC_SWITCH_HIGH_PERCENT) != 0U)) ? 1U : 0U;
  frame->rc_control_motor_mix_allowed =
    (frame->rc_use_stabilized_motor_mix != 0U) ? 1U : 0U;
  (void)APP_ServoCal_Step(frame->ch, frame->rc_link_ok, frame->rc_arm_switch_high, frame->now_ms);
  frame->servo_cal_active = APP_ServoCal_IsActive();
  if (frame->servo_cal_active != 0U) {
    frame->rc_armed = 0U;
    if (APP_ServoFeedbackBench_IsActive() != 0U) {
      APP_ServoFeedbackBench_Stop("servo_cal", frame->now_ms);
    }
  }
  if (APP_ServoFeedbackBench_IsActive() != 0U) {
    frame->rc_armed = 0U;
  }
  APP_Acceptance_Service(frame->now_ms);
  if (APP_Acceptance_IsActive() != 0U) {
    frame->rc_armed = 0U;
    stabilizer_rc_arm_latched = 0U;
    stabilizer_capture_armed = 0U;
  }
  BSP_AiWB2_UpdateButton();       /* 更新 WiFi 模块按键状态                */

  /* 默认保持上一条有效舵机目标；短暂 IMU 异常不能直接回中。 */
  frame->moves[0].id = 1U;
  frame->moves[1].id = 2U;
  frame->moves[0].pulse_us = stabilizer_latest_servo_target_us[0];
  frame->moves[1].pulse_us = stabilizer_latest_servo_target_us[1];

  if ((ctx->has_imu_sample != 0U) &&
      (ctx->attitude_zero_ready != 0U) &&
      ((frame->now_ms - stabilizer_imu_last_sample_ms) <= STABILIZER_IMU_STALE_MS)) {
    frame->imu_control_valid = 1U;
  }

  APP_Ident_Update(frame->now_ms);
  APP_IdentAtt_Update(frame->now_ms);
  {
    /*
     * 光杆台架辨识的一拍。`APP_SysId_Update` 里就把激励、力矩反解、舵机脉宽和
     * 采样全做完了——观测与执行放在同一拍同一处，才谈得上"发出去的"和"量回来
     * 的"时间戳一致，而这套辨识测的正是两者之间的时移。
     *
     * 标志在 Update **之后**才取：本拍触发 abort 时状态已经落到 aborted，
     * 不会留下"标志说在跑、模块已经停了"的那一拍。
     *
     * 高度辨识（ALT）的观测只读：测高/原始测距/电池快照，不动生产路径的任何状态。
     */
    SVC_FLOW_NAV_State sysid_nav;
    APP_BatterySnapshot sysid_battery;
    float sysid_height_m = 0.0f, sysid_vz_m_s = 0.0f;
    uint32_t sysid_height_ms = 0U;
    const uint8_t sysid_height_ok =
      APP_OpticalFlow_GetHeightSample(&sysid_height_m, &sysid_vz_m_s, &sysid_height_ms);
    float sysid_flow_vel_m_s[2] = { 0.0f, 0.0f };
    float sysid_flow_pos_m[2] = { 0.0f, 0.0f };

    SVC_FlowNav_GetState(&sysid_nav);
    /* 水平槽 XY 辨识的光流观测（只读，与生产位置环同一份 Service 成品估计，规范 FLU）。 */
    SVC_FlowNav_GetVelocity(&sysid_flow_vel_m_s[0], &sysid_flow_vel_m_s[1]);
    SVC_FlowNav_GetPosition(&sysid_flow_pos_m[0], &sysid_flow_pos_m[1]);
    APP_Battery_GetSnapshot(&sysid_battery);
    APP_SysIdObserve sysid_obs = {
      .now_ms = frame->now_ms,
      .now_us = (uint32_t)(frame->now_us & 0xFFFFFFFFULL),
      .gyro_rad_s = {
        ctx->last_msg.imu.gyro_x_dps * STABILIZER_DEG_TO_RAD,
        ctx->last_msg.imu.gyro_y_dps * STABILIZER_DEG_TO_RAD,
        ctx->last_msg.imu.gyro_z_dps * STABILIZER_DEG_TO_RAD,
      },
      .roll_rad = ctx->roll_control * STABILIZER_DEG_TO_RAD,
      .pitch_rad = ctx->pitch_control * STABILIZER_DEG_TO_RAD,
      .throttle_us = frame->rc_throttle_motor_us,
      .rc_link_ok = frame->rc_link_ok,
      .rc_armed = frame->rc_armed,
      .imu_valid = frame->imu_control_valid,
      .actuator_inhibit = (APP_Acceptance_IsActive() || frame->servo_cal_active ||
                          APP_ServoFeedbackBench_IsActive() || APP_ThrustBench_IsActive() ||
                          APP_PropSpin_IsActive() || APP_Ident_IsRunning()),
      .thrust_valid = APP_ThrustLut_IsFresh(),
      .rc_throttle_low = frame->rc_arm_throttle_low,
      .gyro_ctrl_rad_s = {
        ctx->gyro_ctrl_rad_s[0], ctx->gyro_ctrl_rad_s[1], ctx->gyro_ctrl_rad_s[2],
      },
      .height_valid = sysid_height_ok,
      .height_sample_ms = sysid_height_ms,
      .height_m = sysid_height_m,
      .height_raw_m = sysid_nav.height_raw_m,
      .vz_m_s = sysid_vz_m_s,
      .az_m_s2 = APP_SysIdAlt_VerticalAccel(
        ctx->last_msg.imu.accel_x_g, ctx->last_msg.imu.accel_y_g, ctx->last_msg.imu.accel_z_g,
        ctx->roll_control * STABILIZER_DEG_TO_RAD, ctx->pitch_control * STABILIZER_DEG_TO_RAD,
        DRV_Airframe_Get()->gravity_m_s2),
      .vbat_v = (sysid_battery.state.valid != 0U) ?
                ((float)sysid_battery.state.voltage_mv * 0.001f) : 0.0f,
      /* 生产导航位置有效的同一判据：光流速度有效且测距有效；样本时刻取光流速度样本的 ms token。 */
      .flow_valid = ((sysid_nav.velocity_valid != 0U) && (sysid_height_ok != 0U)) ? 1U : 0U,
      .flow_sample_ms = sysid_nav.velocity_sample_ms,
      /* 控制角 = 融合角 − 开机零点，所以重力水平在控制角里是 −零点。 */
      .level_valid = ctx->attitude_zero_ready,
      .level_roll_rad = -ctx->roll_zero * STABILIZER_DEG_TO_RAD,
      .level_pitch_rad = -ctx->pitch_zero * STABILIZER_DEG_TO_RAD,
      .flow_vel_m_s = { sysid_flow_vel_m_s[0], sysid_flow_vel_m_s[1] },
      .flow_pos_m = { sysid_flow_pos_m[0], sysid_flow_pos_m[1] },
    };
    /* 竖直通道估计器：加速度就用这份观测里的 az，测距用同一份服务快照。 */
    stabilizer_z_estimator_step(frame, &sysid_nav, sysid_obs.az_m_s2);
    frame->az_up_m_s2 = sysid_obs.az_m_s2;             /* 悬停推力学习用同一份加速度与原始测距 */
    frame->range_raw_valid = sysid_nav.height_valid;
    frame->range_raw_m = sysid_nav.height_raw_m;
    if (frame->z_fusion != 0U) {
      sysid_obs.height_m = frame->z_est.z_m;   /* 原始测距与样本时刻不动 */
      sysid_obs.vz_m_s = frame->z_est.vz_m_s;
    }
    APP_SysId_Update(&sysid_obs);
  }
  frame->sysid_running = APP_SysId_IsEngaged();
  frame->ident_running =
    ((APP_Ident_IsRunning() != 0U) || (frame->sysid_running != 0U)) ? 1U : 0U;
  {
    /*
     * 飞行油门模式（doc/flight-throttle-contract.md）。FLIGHT 且未被旁路时用状态机结果
     * 覆盖上面按 70% 门限算出的三个标志；LEGACY 或旁路（辨识/推力台/旋向/舵机标定）
     * 时一个字都不动，保持旧行为。
     */
    DRV_FlightThrottleInput ft_in;
    DRV_FlightThrottleOutput ft_out;
    const DRV_Airframe_Params *ft_airframe = DRV_Airframe_Get();

    memset(&ft_in, 0, sizeof(ft_in));
    ft_in.dt_s = (stabilizer_ft_last_ms == 0U) ? 0.0f :
      (float)(frame->now_ms - stabilizer_ft_last_ms) * 0.001f;
    stabilizer_ft_last_ms = frame->now_ms;
    ft_in.armed = frame->rc_armed;
    ft_in.link_ok = frame->rc_link_ok;
    ft_in.throttle_01 = frame->rc.throttle_01;
    /* CH6 三段拨码的开关量；未绑定时给 0（定点），与旧"未绑定=低位"一致。 */
    ft_in.mode_switch_01 =
      ((frame->rc.bound_mask & (uint8_t)(1U << APP_RC_FUNC_MODE)) != 0U) ?
        0.5f * (frame->rc.norm[APP_RC_FUNC_MODE] + 1.0f) : 0.0f;
    ft_in.height_valid = stabilizer_ft_fb_valid;
    ft_in.height_m = stabilizer_ft_fb_height_m;
    ft_in.vz_m_s = stabilizer_ft_fb_vz_m_s;
    ft_in.imu_valid = frame->imu_control_valid;
    ft_in.bypass =
      ((frame->ident_running != 0U) || (frame->servo_cal_active != 0U) ||
       (APP_ThrustBench_IsActive() != 0U) || (APP_PropSpin_IsActive() != 0U)) ? 1U : 0U;
    frame->ft_hover_n = DRV_COAX_CTRL_EffectiveMassKg(ft_airframe->mass_kg) *
                        ft_airframe->gravity_m_s2;
    ft_in.hover_thrust_n = frame->ft_hover_n;
    /* 定高档满杆升/降速度 = 位置环竖直限速参数（读不到则保持 0 → 驱动退回 0.30/0.20 常量）。 */
    (void)DRV_COAX_CTRL_GetParam("coax.pos_z_vel_up_max_m_s", &ft_in.v_up_m_s);
    (void)DRV_COAX_CTRL_GetParam("coax.pos_z_vel_down_max_m_s", &ft_in.v_dn_m_s);
    ft_in.spool_start_force_n = DRV_COAX_CTRL_MotorPulseToTotalThrust(
      stabilizer_rc_throttle_to_motor_pulse(DRV_FLIGHT_THROTTLE_GROUND_MAX_01));
    ft_in.max_thrust_n = DRV_COAX_CTRL_MotorPulseToTotalThrust(BSP_PWM_ESC_MAX_US);
    /* 反馈量只用一拍：本拍的控制计算会重新写入，没跑到就当无效。 */
    stabilizer_ft_fb_valid = 0U;
    DRV_FlightThrottle_Update(&ft_in, &ft_out);
    if (ft_out.legacy_semantics == 0U) {
      frame->ft_active_mode = 1U;
      frame->rc_use_stabilized_motor_mix = ft_out.active;
      frame->rc_control_motor_mix_allowed = ft_out.active;
      frame->ft_spoolup =
        (ft_out.state == DRV_FLIGHT_THROTTLE_STATE_SPOOLUP) ? 1U : 0U;
      frame->rc_attitude_debug_mode =
        ((frame->rc_link_ok != 0U) &&
         (ft_out.state == DRV_FLIGHT_THROTTLE_STATE_FLYING) &&
         (ft_out.xy_mode == DRV_FLIGHT_THROTTLE_XY_ANGLE)) ? 1U : 0U;
      frame->ft_xy_velocity_hold =
        (ft_out.xy_mode == DRV_FLIGHT_THROTTLE_XY_VELOCITY) ? 1U : 0U;
      /* LANDED：直通分支里电机必须是 ESC 最小，绝不能把杆位当油门。 */
      frame->ft_landed_idle =
        (ft_out.state == DRV_FLIGHT_THROTTLE_STATE_LANDED) ? 1U : 0U;
      frame->ft_climb_rate_m_s = ft_out.climb_rate_m_s;
      frame->ft_manual_force_n = ft_out.manual_force_n;
      frame->ft_manual_integrate = ft_out.manual_integrate;
      frame->ft_ground_spin_01 = ft_out.ground_spin_01;
    }
  }
  {
    APP_IdentAttObserve ident_att_obs = {0};

    ident_att_obs.now_ms = frame->now_ms;
    ident_att_obs.roll_deg = ctx->roll_control;
    ident_att_obs.pitch_deg = ctx->pitch_control;
    ident_att_obs.gyro_x_dps = ctx->last_msg.imu.gyro_x_dps;
    ident_att_obs.gyro_y_dps = ctx->last_msg.imu.gyro_y_dps;
    ident_att_obs.rc_link_ok = frame->rc_link_ok;
    ident_att_obs.rc_armed = frame->rc_armed;
    ident_att_obs.imu_valid = frame->imu_control_valid;
    ident_att_obs.throttle_over_20 = frame->rc_use_stabilized_motor_mix;
    ident_att_obs.control_valid =
      ((frame->rc_use_stabilized_motor_mix != 0U) &&
       (frame->imu_control_valid != 0U) &&
       (frame->ident_running == 0U)) ? 1U : 0U;
    APP_IdentAtt_Observe(&ident_att_obs);
  }

  if (frame->rc_link_seen == 0U) {
    frame->led_arm_block_reason = APP_LED_ARM_BLOCK_NO_RC;
  } else if (frame->rc_link_ok == 0U) {
    frame->led_arm_block_reason = APP_LED_ARM_BLOCK_RC_LOSS;
  } else if (APP_Stabilizer_IsImuFrameArmLocked() != 0U) {
    /*
     * 与下面 IMU 健康那一档同源、同理由：坐标迁移未完成的硬锁在
     * stabilizer_rc_update_armed() 最开头就把 rc_armed 清零并返回，所以必须
     * 排在 rc_armed 判定之前。缺了这一档时，拨杆已打、油门已收的正常状态会
     * 一路掉进最后的 else，把"坐标迁移未完成"误报成"拨杆没打"——实机上
     * 2026-09-05 抓到的正是这个：IMUFRAME arm_lock=1、RC norm arm=+1000、
     * thr01=0，灯却闪 3 下说拨杆没打。
     */
    frame->led_arm_block_reason = APP_LED_ARM_BLOCK_FRAME;
  } else if (APP_ImuHealth_IsArmBlocked() != 0U) {
    /*
     * 必须排在 rc_armed 判定之前：采样链失效会把 rc_armed 强制清零，
     * 否则这里会误报成"拨杆没打"，把真正原因藏起来。
     */
    frame->led_arm_block_reason = APP_LED_ARM_BLOCK_IMU;
  } else if (DRV_Airframe_IsValid() == 0U) {
    /*
     * 与上面两档同源、同理由：机体模型无效时 stabilizer_rc_update_armed() 会
     * 直接把 rc_armed 清零并返回，所以必须排在 rc_armed 判定之前。否则拨杆已打、
     * 油门已收的正常状态会一路掉进最后的 else，把"没写机体模型"误报成"拨杆没打"。
     */
    frame->led_arm_block_reason = APP_LED_ARM_BLOCK_AIRFRAME;
  } else if (APP_Battery_CanArm() == 0U) {
    frame->led_arm_block_reason = APP_LED_ARM_BLOCK_BATTERY;
  } else if (frame->rc_armed == 0U) {
    if ((frame->rc_arm_switch_high != 0U) && (frame->rc_arm_throttle_low == 0U)) {
      frame->led_arm_block_reason = APP_LED_ARM_BLOCK_THROTTLE_HIGH;
    } else {
      frame->led_arm_block_reason = APP_LED_ARM_BLOCK_ARM_SWITCH;
    }
  } else if ((frame->rc_control_motor_mix_allowed != 0U) &&
             (frame->imu_control_valid == 0U) &&
             (frame->ident_running == 0U)) {
    frame->led_arm_block_reason = APP_LED_ARM_BLOCK_IMU;
  } else {
    frame->led_arm_block_reason = APP_LED_ARM_BLOCK_NONE;
  }
  APP_LED_SetArmStatus(frame->rc_armed, frame->led_arm_block_reason);

  /*
   * 同一处、同一批数据既点灯也发给上位机。分成两处算的话，灯和屏迟早各说各的，
   * 而现场判断"为什么解不了锁"恰恰是拿这两个互相印证的。
   */
  {
    APP_Stabilizer_ArmStatus status;

    status.armed = frame->rc_armed;
    status.block_reason = (uint8_t)frame->led_arm_block_reason;
    status.rc_link_seen = frame->rc_link_seen;
    status.rc_link_ok = frame->rc_link_ok;
    status.arm_switch_high = frame->rc_arm_switch_high;
    status.throttle_low = frame->rc_arm_throttle_low;
    status.imu_control_valid = frame->imu_control_valid;
    status.imu_health_ok = (APP_ImuHealth_IsArmBlocked() == 0U) ? 1U : 0U;
    status.frame_migration_ok =
      (APP_Stabilizer_IsImuFrameArmLocked() == 0U) ? 1U : 0U;
    status.airframe_valid = DRV_Airframe_IsValid();
    status.servo_cal_idle = (frame->servo_cal_active == 0U) ? 1U : 0U;
    status.acceptance_idle = (APP_Acceptance_IsActive() == 0U) ? 1U : 0U;
    status.published = 1U;
    status.now_ms = frame->now_ms;
    status.battery_ok = APP_Battery_CanArm();
    stabilizer_arm_status = status;
  }
}

static void stabilizer_control_compute(StabilizerContext *ctx,
                                       StabilizerControlFrame *frame)
{
  if (frame->ident_running != 0U) {
    uint16_t ident_alpha_us;
    uint16_t ident_beta_us;

    DRV_COAX_CTRL_ResetState();
    /*
     * 光杆辨识占用时不清调度器：清零后 last_rate_us=0，下一次循环 rate_due 恒真，
     * 控制拍就从 500 Hz 变成"每次被 IMU 信号量唤醒都跑"（~1 kHz 且抖动），两次
     * 唤醒挨得近时间隔 <0.5 ms，辨识的 control_dt 门就中止了这一轮——2026-09-26
     * 杆上三轮 1/102/4 条样本即中止都是这个。控制器状态照清，节拍保持。
     */
    if (frame->sysid_running == 0U) {
      APP_ControlScheduler_Reset(&ctx->control_scheduler);
    }
    if (frame->sysid_running != 0U) {
      APP_SysId_GetServoTargets(&ident_alpha_us, &ident_beta_us);
    } else {
      APP_Ident_GetServoTargets(&ident_alpha_us, &ident_beta_us);
    }
    frame->moves[0].pulse_us = ident_alpha_us;
    frame->moves[1].pulse_us = ident_beta_us;
    /* 取完目标再问还在不在跑：STOP 插在两者之间时，这一拍按"不补"处理。 */
    if ((frame->sysid_running != 0U) && (APP_SysId_IsRunning() != 0U)) {
      frame->servo_source = (uint8_t)APP_SERVO_BACKLASH_SRC_SYSID;
    }
  } else if (frame->rc_control_motor_mix_allowed == 0U) {
    DRV_COAX_CTRL_ServoCalibration servo_calibration;
    /*
     * Throttle below the stabilization threshold is the low-power
     * direct-throttle stage. Keep tilt servos centered there so stick
     * motion cannot move the airframe before the test is intentionally
     * brought into the active range.
     */
    ctx->position_ref_x_m = 0.0f;
    ctx->position_ref_y_m = 0.0f;
    ctx->position_ref_xy_ready = 0U;
    ctx->height_ref_m = 0.0f;
    /* 上电高度原点只在首次有效测高时锁存，低油门直通不重新归零。 */
    ctx->position_ref_z_ready = 0U;
    ctx->yaw_ref_ready = 0U;
    SVC_FlowNav_ResetEstimator();
    DRV_COAX_CTRL_ResetState();
    APP_ControlScheduler_Reset(&ctx->control_scheduler);
    ctx->last_gyro_ready = 0U;
    ctx->last_vertical_velocity_sample_ms = 0U;
    ctx->vertical_accel_ready = 0U;
    ctx->position_ref_z_ready = 0U;
    ctx->yaw_ref_ready = 0U;
    ctx->vofa_debug.vel_loop_active = 0.0f;
    stabilizer_vofa_debug_publish(&ctx->vofa_debug);
    DRV_COAX_CTRL_GetServoCalibration(&servo_calibration);
    frame->moves[0].pulse_us =
      servo_calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX];
    frame->moves[1].pulse_us =
      servo_calibration.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX];
  } else if (frame->imu_control_valid != 0U) {

    /*
     * 模式 B：同轴控制器（论文控制分配）
     *   输入：姿态角 + 角速度（全部转为弧度）+ RC 参考（归一化）
     *   输出：两个舵机脉宽（servo_alpha_us / servo_beta_us）
     */
    frame->range_height_valid =
      APP_OpticalFlow_GetHeightSample(&frame->range_height_m,
                                      &frame->range_velocity_m_s,
                                      &frame->range_sample_ms);
    if ((frame->range_height_valid != 0U) &&
        (frame->range_sample_ms != ctx->last_vertical_velocity_sample_ms)) {
      const float velocity_z_m_s = frame->range_velocity_m_s;
      const uint32_t elapsed_ms = frame->range_sample_ms -
                                  ctx->last_vertical_velocity_sample_ms;
      if ((ctx->last_vertical_velocity_sample_ms != 0U) &&
          (elapsed_ms > 0U) && (elapsed_ms <= SVC_FLOW_NAV_TIMEOUT_MS)) {
        ctx->vertical_accel_m_s2 =
          (velocity_z_m_s - ctx->last_vertical_velocity_m_s) /
          ((float)elapsed_ms * 0.001f);
        ctx->vertical_accel_ready = 1U;
      } else {
        ctx->vertical_accel_m_s2 = 0.0f;
        ctx->vertical_accel_ready = 0U;
      }
      ctx->last_vertical_velocity_m_s = velocity_z_m_s;
      ctx->last_vertical_velocity_sample_ms = frame->range_sample_ms;
    } else if (frame->range_height_valid == 0U) {
      ctx->vertical_accel_m_s2 = 0.0f;
      ctx->vertical_accel_ready = 0U;
      ctx->last_vertical_velocity_sample_ms = 0U;
    }
    if (frame->range_height_valid != 0U) {
      if (ctx->height_origin_ready == 0U) {
        ctx->height_origin_m = frame->range_height_m;
        ctx->height_origin_ready = 1U;
      }
      frame->relative_height_m = frame->range_height_m - ctx->height_origin_m;
      if (frame->z_fusion != 0U) {
        /* 融合开：换成估计器高度，原点照旧（首次有效测距锁存的那个）。 */
        frame->relative_height_m = frame->z_est.z_m - ctx->height_origin_m;
      }
      if (frame->relative_height_m < 0.0f) {
        frame->relative_height_m = 0.0f;
      }
    }
    {
      uint8_t velocity_loop_enabled = 0U;
      float nav_vx_m_s = 0.0f;
      float nav_vy_m_s = 0.0f;
      float position_state_x_m = 0.0f;
      float position_state_y_m = 0.0f;
      float velocity_control_x_m_s;
      float velocity_control_y_m_s;
      float vel_loop_enable = 0.0f;
      SVC_FLOW_NAV_State nav_state;

      /*
       * 速度与位置都直接取 Service 的成品估计：位置在 Service 里已经按传感器
       * 自己的时间轴积分好了，这里不再自己积一遍，只做机体系符号映射——沿用
       * 原来"位置由带符号速度积出"的关系，SIGN 提到积分外面等价。
       */
      SVC_FlowNav_GetVelocity(&nav_vx_m_s, &nav_vy_m_s);
      SVC_FlowNav_GetPosition(&position_state_x_m, &position_state_y_m);
      SVC_FlowNav_GetState(&nav_state);
      velocity_control_x_m_s = nav_vx_m_s;
      velocity_control_y_m_s = nav_vy_m_s;

      frame->attitude.roll_rad = ctx->roll_control * STABILIZER_DEG_TO_RAD;
      frame->attitude.pitch_rad = ctx->pitch_control * STABILIZER_DEG_TO_RAD;
      frame->attitude.yaw_rad = ctx->yaw_control * STABILIZER_DEG_TO_RAD;
      frame->attitude.z_m = frame->relative_height_m;
      frame->attitude.vx_m_s = velocity_control_x_m_s;
      frame->attitude.vy_m_s = velocity_control_y_m_s;
      frame->attitude.vz_m_s = frame->range_velocity_m_s;
      if (frame->range_height_valid == 0U) {
        frame->attitude.z_m = 0.0f;
        frame->attitude.vz_m_s = 0.0f;
      }
      frame->attitude.gyro_x_rad_s = ctx->gyro_ctrl_rad_s[0];
      frame->attitude.gyro_y_rad_s = ctx->gyro_ctrl_rad_s[1];
      frame->attitude.gyro_z_rad_s = ctx->gyro_ctrl_rad_s[2];
      frame->attitude.accel_m_s2[0] = ctx->vofa_debug.acc_nav_m_s2[0];
      frame->attitude.accel_m_s2[1] = ctx->vofa_debug.acc_nav_m_s2[1];
      frame->attitude.accel_m_s2[2] = ctx->vertical_accel_m_s2;
      frame->attitude.acceleration_valid =
        ((nav_state.velocity_valid != 0U) &&
         (ctx->vertical_accel_ready != 0U)) ? 1U : 0U;
      if (frame->z_fusion != 0U) {
        /*
         * 融合开：垂直速度换成估计器输出（测距无效时仍按上面清零）；速度环 D 项的测量
         * 加速度改用 a_up − 零偏（IMU），不再对测距速度差分。
         */
        if (frame->range_height_valid != 0U) {
          frame->attitude.vz_m_s = frame->z_est.vz_m_s;
        }
        frame->attitude.accel_m_s2[2] = frame->z_est.accel_m_s2;
        frame->attitude.acceleration_valid =
          ((nav_state.velocity_valid != 0U) && (frame->z_est.accel_valid != 0U)) ? 1U : 0U;
      }

      stabilizer_ft_fb_valid = frame->range_height_valid;
      stabilizer_ft_fb_height_m = frame->relative_height_m;
      stabilizer_ft_fb_vz_m_s = frame->attitude.vz_m_s;

      (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_enable", &vel_loop_enable);
      velocity_loop_enabled = (vel_loop_enable >= 0.5f) ? 1U : 0U;
      if ((frame->rc_attitude_debug_mode != 0U) || (frame->ft_spoolup != 0U)) {
        frame->reference.vx_m_s = 0.0f;
        frame->reference.vy_m_s = 0.0f;
        frame->reference.direct_attitude_target_valid = 1U;
        frame->reference.manual_total_force_valid = 1U;
        if (frame->ft_spoolup != 0U) {
          /* SPOOLUP：水平姿态目标 0/0，总推力走斜坡，速度环不跑。 */
          frame->reference.manual_total_force_n = frame->ft_manual_force_n;
          frame->reference.target_pitch_rad = 0.0f;
          frame->reference.target_roll_rad = 0.0f;
        } else {
          frame->reference.manual_total_force_n =
            (frame->ft_active_mode != 0U) ?
              frame->ft_manual_force_n :    /* 角度档手控油门：状态机按杆算好（空中保底不停桨） */
              DRV_COAX_CTRL_MotorPulseToTotalThrust(frame->rc_throttle_motor_us);
          frame->reference.target_pitch_rad =
            APP_RcIntent_TargetPitch(
              frame->rc.norm[APP_RC_FUNC_PITCH],
              stabilizer_manual_tilt_max_rad());
          frame->reference.target_roll_rad =
            APP_RcIntent_TargetRoll(
              frame->rc.norm[APP_RC_FUNC_ROLL],
              stabilizer_manual_tilt_max_rad());
        }
        frame->reference.horizontal_velocity_valid = 0U;
        velocity_loop_enabled = 0U;
      } else {
        /*
         * 满杆速度 = 位置/速度环的水平限速 coax.pos_xy_vel_max_m_s：一个参数同时管"杆推到底多快"和
         * "控制器最多给多快"，地面站参数页就能改，不用刷固件。作者 2026-10-01 自由飞后："速度保持那个
         * 杆子推到底跑的还是很慢……目前可以飞了可以给速度提高到正常水平"（原为写死 0.40 m/s）。
         */
        float xy_stick_max_m_s = STABILIZER_XY_VEL_REF_MAX_M_S;
        float xy_speed_param = 0.0f;
        if ((DRV_COAX_CTRL_GetParam("coax.pos_xy_vel_max_m_s", &xy_speed_param) != 0U) &&
            (xy_speed_param >= STABILIZER_XY_VEL_STICK_MIN_M_S) &&
            (xy_speed_param <= STABILIZER_XY_VEL_STICK_MAX_M_S)) {
          xy_stick_max_m_s = xy_speed_param;
        }
        frame->reference.vx_m_s = APP_RcIntent_ForwardVelocity(
          frame->rc.norm[APP_RC_FUNC_PITCH], xy_stick_max_m_s);
        frame->reference.vy_m_s = APP_RcIntent_LeftVelocity(
          frame->rc.norm[APP_RC_FUNC_ROLL], xy_stick_max_m_s);
        frame->reference.horizontal_velocity_valid =
          ((velocity_loop_enabled != 0U) &&
           (nav_state.velocity_valid != 0U)) ? 1U : 0U;
      }
      frame->reference.navigation_velocity_valid = nav_state.velocity_valid;
      frame->reference.navigation_position_valid =
        ((nav_state.velocity_valid != 0U) &&
         (frame->range_height_valid != 0U)) ? 1U : 0U;
      /* 分轴：水平看光流（上两项），竖直只看测距——光流失效时高度照常闭环。 */
      frame->reference.split_axis_validity = 1U;
      frame->reference.vertical_measurement_valid = frame->range_height_valid;
      frame->reference.position_control_bypass = 0U;
      frame->reference.vz_m_s = 0.0f;
      frame->reference.ax_m_s2 = 0.0f;
      frame->reference.ay_m_s2 = 0.0f;
      frame->reference.dt_sec = frame->ctrl_dt_sec;

      if ((velocity_loop_enabled != 0U) && (frame->ft_xy_velocity_hold != 0U)) {
        /*
         * 速度保持（CH6 中位）：位置参考每拍跟到当前估计，位置误差恒为 0，位置环只剩杆给的
         * 速度前馈——即"速度为 0 闭环、没有位置 P"。ready 清零，切回定点时从切换那一刻的位置起锁。
         */
        ctx->position_ref_x_m = position_state_x_m;
        ctx->position_ref_y_m = position_state_y_m;
        ctx->position_ref_xy_ready = 0U;
        ctx->xy_brake_pending = 1U;          /* 切到定点档时先刹停再锁位，不在运动中锁住再被拉回 */
        ctx->xy_brake_elapsed_s = 0.0f;
      } else if ((velocity_loop_enabled != 0U) &&
                 ((frame->reference.vx_m_s != 0.0f) || (frame->reference.vy_m_s != 0.0f) ||
                  (ctx->xy_brake_pending != 0U))) {
        /*
         * 定点档（CH6 低）打杆与松杆刹车：打杆时同速度保持（位置参考跟到估计，杆直接给速度）；
         * 松杆后速度目标为 0 刹车，速度降到 LOCK_SPEED 以下（或刹满 BRAKE_MAX）才在当时位置锁位。
         * 旧做法让位置参考按杆速一直往前积分，松杆时参考已跑到前面、机体带着约 0.25 s 滞后冲过去
         * 再被拉回（2026-10-01 自由飞数据仿真松杆过冲约 0.35 m）；作者"要定点就能定点，要移动就最快响应"。
         */
        if ((frame->reference.vx_m_s != 0.0f) || (frame->reference.vy_m_s != 0.0f)) {
          ctx->xy_brake_pending = 1U;
          ctx->xy_brake_elapsed_s = 0.0f;
        } else {
          ctx->xy_brake_elapsed_s += frame->ctrl_dt_sec;
          if (((frame->attitude.vx_m_s * frame->attitude.vx_m_s) +
               (frame->attitude.vy_m_s * frame->attitude.vy_m_s) <
               (STABILIZER_XY_LOCK_SPEED_M_S * STABILIZER_XY_LOCK_SPEED_M_S)) ||
              (ctx->xy_brake_elapsed_s >= STABILIZER_XY_BRAKE_MAX_S)) {
            ctx->xy_brake_pending = 0U;      /* 下一拍走锁位分支，从那时的位置起锁 */
          }
        }
        ctx->position_ref_x_m = position_state_x_m;
        ctx->position_ref_y_m = position_state_y_m;
        ctx->position_ref_xy_ready = 0U;
      } else if (velocity_loop_enabled != 0U) {
        if (ctx->position_ref_xy_ready == 0U) {
          ctx->position_ref_x_m = position_state_x_m;
          ctx->position_ref_y_m = position_state_y_m;
          ctx->position_ref_xy_ready = 1U;
        }
        ctx->position_ref_x_m += frame->reference.vx_m_s * frame->ctrl_dt_sec;
        ctx->position_ref_y_m += frame->reference.vy_m_s * frame->ctrl_dt_sec;
        ctx->position_ref_x_m =
          stabilizer_clamp_f32(ctx->position_ref_x_m,
                               position_state_x_m - STABILIZER_XY_POS_ERR_MAX_M,
                               position_state_x_m + STABILIZER_XY_POS_ERR_MAX_M);
        ctx->position_ref_y_m =
          stabilizer_clamp_f32(ctx->position_ref_y_m,
                               position_state_y_m - STABILIZER_XY_POS_ERR_MAX_M,
                               position_state_y_m + STABILIZER_XY_POS_ERR_MAX_M);
        ctx->position_ref_x_m =
          stabilizer_clamp_f32(ctx->position_ref_x_m,
                               -SVC_FLOW_NAV_POSITION_LIMIT_M,
                                SVC_FLOW_NAV_POSITION_LIMIT_M);
        ctx->position_ref_y_m =
          stabilizer_clamp_f32(ctx->position_ref_y_m,
                               -SVC_FLOW_NAV_POSITION_LIMIT_M,
                                SVC_FLOW_NAV_POSITION_LIMIT_M);
      } else {
        ctx->position_ref_x_m = position_state_x_m;
        ctx->position_ref_y_m = position_state_y_m;
        ctx->position_ref_xy_ready = 0U;
        ctx->xy_brake_pending = 0U;
      }

      frame->attitude.x_m = position_state_x_m;
      frame->attitude.y_m = position_state_y_m;
      frame->reference.x_m = ctx->position_ref_x_m;
      frame->reference.y_m = ctx->position_ref_y_m;
      ctx->vofa_debug.pos_est_m[0] = position_state_x_m;
      ctx->vofa_debug.pos_est_m[1] = position_state_y_m;
      ctx->vofa_debug.vel_ref_m_s[0] = frame->reference.vx_m_s;
      ctx->vofa_debug.vel_ref_m_s[1] = frame->reference.vy_m_s;
      ctx->vofa_debug.vel_err_m_s[0] = frame->reference.vx_m_s - frame->attitude.vx_m_s;
      ctx->vofa_debug.vel_err_m_s[1] = frame->reference.vy_m_s - frame->attitude.vy_m_s;
      ctx->vofa_debug.vel_loop_active =
        (velocity_loop_enabled != 0U) ? 1.0f : 0.0f;
    }
    frame->reference.az_m_s2 = 0.0f;
    if ((frame->rc_attitude_debug_mode != 0U) || (frame->ft_spoolup != 0U)) {
      frame->reference.z_m = frame->attitude.z_m;
      frame->reference.vz_m_s = frame->attitude.vz_m_s;
      ctx->height_ref_m = frame->relative_height_m;
      ctx->position_ref_z_m = frame->attitude.z_m;
      ctx->position_ref_z_ready = 0U;
    } else {
      if ((frame->range_height_valid == 0U) ||
          (ctx->height_origin_ready == 0U)) {
        ctx->height_ref_m = 0.0f;
        ctx->position_ref_z_ready = 0U;
        ctx->position_ref_z_m = frame->attitude.z_m;
      } else if (frame->rc_use_stabilized_motor_mix == 0U) {
        ctx->height_ref_m = frame->relative_height_m;
        ctx->height_ref_m =
          stabilizer_clamp_f32(ctx->height_ref_m,
                               0.0f,
                               stabilizer_alt_max_m());
        ctx->position_ref_z_m = ctx->height_ref_m;
        ctx->position_ref_z_ready = 1U;
      } else {
        if (ctx->position_ref_z_ready == 0U) {
          ctx->height_ref_m = frame->relative_height_m;
          ctx->height_ref_m =
            stabilizer_clamp_f32(ctx->height_ref_m,
                                 0.0f,
                                 stabilizer_alt_max_m());
          ctx->position_ref_z_ready = 1U;
        }
        ctx->height_ref_m +=
          ((frame->ft_active_mode != 0U) ?
             frame->ft_climb_rate_m_s :
             stabilizer_rc_throttle_height_rate_m_s(
               frame->rc.norm[APP_RC_FUNC_THROTTLE])) * frame->ctrl_dt_sec;
        ctx->height_ref_m =
          stabilizer_clamp_f32(ctx->height_ref_m,
                               0.0f,
                               stabilizer_alt_max_m());
        ctx->position_ref_z_m = ctx->height_ref_m;
      }
      if ((frame->range_height_valid == 0U) ||
          (ctx->height_origin_ready == 0U)) {
        frame->reference.z_m = frame->attitude.z_m;
      } else {
        frame->reference.z_m =
          stabilizer_clamp_f32(ctx->position_ref_z_m,
                               frame->attitude.z_m - STABILIZER_Z_POS_ERR_MAX_M,
                               frame->attitude.z_m + STABILIZER_Z_POS_ERR_MAX_M);
      }
    }
    {
      float yaw_rate_ref_rad_s =
        stabilizer_rc_yaw_rate_rad_s(frame->rc.norm[APP_RC_FUNC_YAW]);

      if (ctx->yaw_ref_ready == 0U) {
        ctx->yaw_ref_rad = frame->attitude.yaw_rad;
        ctx->yaw_ref_ready = 1U;
      }
      ctx->yaw_ref_rad =
        stabilizer_wrap_pi(ctx->yaw_ref_rad +
                           yaw_rate_ref_rad_s * frame->ctrl_dt_sec);
      frame->reference.yaw_rad = ctx->yaw_ref_rad;
      frame->reference.yaw_rate_rad_s = yaw_rate_ref_rad_s;
      frame->reference.yaw_accel_rad_s2 = 0.0f;
    }

    APP_IdentAtt_Apply(&frame->reference.ax_m_s2,
                       &frame->reference.ay_m_s2,
                       &frame->ident_att_log);

    {
      DRV_COAX_CTRL_Schedule schedule = {0};
      schedule.position_update = frame->cascade_schedule.position_due;
      schedule.velocity_update = frame->cascade_schedule.velocity_due;
      schedule.attitude_update = frame->cascade_schedule.attitude_due;
      schedule.rate_update = frame->cascade_schedule.rate_due;
      schedule.position_dt_s = frame->cascade_schedule.position_dt_s;
      schedule.velocity_dt_s = frame->cascade_schedule.velocity_dt_s;
      schedule.attitude_dt_s = frame->cascade_schedule.attitude_dt_s;
      schedule.rate_dt_s = frame->cascade_schedule.rate_dt_s;
      /* 角度档手控油门（PX4 自稳式）在推力够大时也积分；SPOOLUP 与旧调试档仍每拍清。 */
      schedule.integrator_enable =
        ((frame->rc_armed != 0U) && (frame->rc_link_ok != 0U) &&
         (frame->imu_control_valid != 0U) &&
         (((frame->reference.direct_attitude_target_valid == 0U) &&
           (frame->reference.manual_total_force_valid == 0U)) ||
          ((frame->rc_attitude_debug_mode != 0U) &&
           (frame->ft_manual_integrate != 0U)))) ? 1U : 0U;
      schedule.integrator_freeze =
        (schedule.integrator_enable == 0U) ? 1U : 0U;
      schedule.integrator_reset =
        ((schedule.integrator_enable == 0U) ||
         (frame->cascade_schedule.timestamp_fault != 0U)) ? 1U : 0U;
      DRV_COAX_CTRL_RunScheduled(&frame->attitude, &frame->reference,
                                 &schedule, &frame->ctrl_out);
    }
    {
      DRV_COAX_CTRL_Debug balance_debug;
      APP_IdentAttObserve ident_att_obs = {0};

      DRV_COAX_CTRL_GetLastDebug(&balance_debug);
      ctx->vofa_debug.vel_pid_out_m_s2[0] = balance_debug.accel_out_m_s2[0];
      ctx->vofa_debug.vel_pid_out_m_s2[1] = balance_debug.accel_out_m_s2[1];
      ctx->vofa_debug.vel_pid_p_m_s2[0] = balance_debug.velocity_p_m_s2[0];
      ctx->vofa_debug.vel_pid_p_m_s2[1] = balance_debug.velocity_p_m_s2[1];
      ctx->vofa_debug.vel_pid_i_m_s2[0] = balance_debug.velocity_i_m_s2[0];
      ctx->vofa_debug.vel_pid_i_m_s2[1] = balance_debug.velocity_i_m_s2[1];
      ctx->vofa_debug.vel_pid_d_m_s2[0] = balance_debug.velocity_d_m_s2[0];
      ctx->vofa_debug.vel_pid_d_m_s2[1] = balance_debug.velocity_d_m_s2[1];
      ident_att_obs.now_ms = frame->now_ms;
      ident_att_obs.roll_deg = ctx->roll_control;
      ident_att_obs.pitch_deg = ctx->pitch_control;
      ident_att_obs.gyro_x_dps = ctx->last_msg.imu.gyro_x_dps;
      ident_att_obs.gyro_y_dps = ctx->last_msg.imu.gyro_y_dps;
      ident_att_obs.tilt_out_rad[0] = balance_debug.tilt_out_rad[0];
      ident_att_obs.tilt_out_rad[1] = balance_debug.tilt_out_rad[1];
      ident_att_obs.protection_flags = balance_debug.protection_flags;
      ident_att_obs.rc_link_ok = frame->rc_link_ok;
      ident_att_obs.rc_armed = frame->rc_armed;
      ident_att_obs.imu_valid = frame->imu_control_valid;
      ident_att_obs.throttle_over_20 = frame->rc_use_stabilized_motor_mix;
      ident_att_obs.control_valid = 1U;
      APP_IdentAtt_Observe(&ident_att_obs);
    }

    ctx->vofa_debug.range_vertical_velocity_m_s = frame->range_velocity_m_s;
    ctx->vofa_debug.altitude_ref_m = frame->reference.z_m;
    ctx->vofa_debug.altitude_correction_us = 0.0f;

    frame->moves[0].pulse_us = frame->ctrl_out.servo_alpha_us;
    frame->moves[1].pulse_us = frame->ctrl_out.servo_beta_us;
    frame->servo_source = (uint8_t)APP_SERVO_BACKLASH_SRC_CONTROLLER;
    ctx->vofa_debug.servo_alpha_us = (float)frame->moves[0].pulse_us;
    ctx->vofa_debug.servo_beta_us = (float)frame->moves[1].pulse_us;
    ctx->vofa_debug.motor_upper_us = (float)frame->ctrl_out.motor_upper_us;
    ctx->vofa_debug.motor_lower_us = (float)frame->ctrl_out.motor_lower_us;
    stabilizer_vofa_debug_publish(&ctx->vofa_debug);
  } else if (ctx->has_imu_sample == 0U) {
    DRV_COAX_CTRL_ServoCalibration servo_calibration;
    SVC_FlowNav_ResetEstimator();
    DRV_COAX_CTRL_ResetState();
    APP_ControlScheduler_Reset(&ctx->control_scheduler);
    ctx->position_ref_x_m = 0.0f;
    ctx->position_ref_y_m = 0.0f;
    ctx->position_ref_xy_ready = 0U;
    ctx->vofa_debug.vel_loop_active = 0.0f;
    stabilizer_vofa_debug_publish(&ctx->vofa_debug);
    /* 上电尚无有效姿态时才使用中位；运行中 IMU 异常保持上一目标。 */
    DRV_COAX_CTRL_GetServoCalibration(&servo_calibration);
    frame->moves[0].pulse_us =
      servo_calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX];
    frame->moves[1].pulse_us =
      servo_calibration.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX];
  } else {
    /* Runtime IMU stale/fault: hold the last actuator target but discard all
     * controller derivative/integrator history before a future recovery. */
    DRV_COAX_CTRL_ResetState();
    APP_ControlScheduler_Reset(&ctx->control_scheduler);
    /* 保持的是控制器上一拍（补偿前）的目标：回差补偿照旧，方向不变，发出去的脉宽也就不变。 */
    frame->servo_source = (uint8_t)APP_SERVO_BACKLASH_SRC_CONTROLLER;
  }

  /*
   * 只喂老的 app_ident：`ident_running` 现在是"两套辨识之一在占用执行器"，
   * 光杆台架那套的观测已经在 APP_SysId_Update 里做过了，再喂一次会让 app_ident
   * 以为自己在跑。
   */
  if ((frame->ident_running != 0U) && (frame->sysid_running == 0U)) {
    APP_IdentObserve ident_obs = {
      .now_ms = frame->now_ms,
      .roll_deg = ctx->roll_control,
      .pitch_deg = ctx->pitch_control,
      .gyro_x_dps = ctx->last_msg.imu.gyro_x_dps,
      .gyro_y_dps = ctx->last_msg.imu.gyro_y_dps,
      .rc_link_ok = frame->rc_link_ok,
      .rc_armed = frame->rc_armed,
      .imu_valid = frame->imu_control_valid,
      .throttle_us = frame->rc_throttle_motor_us,
    };
    APP_Ident_Observe(&ident_obs);
  }
}

/*
 * 把"上桨/下桨推力"发到**它们实际焊在哪个 ESC 通道上**。
 *
 * 以前这里是 `SetEscPulse(1, upper); SetEscPulse(2, lower);`——按数组下标当接线。
 * 接反了之后不是"偏航反了"这么简单：上下桨的偏航力臂 ku/kl 不同，推力分配整体
 * 错位，倾转力矩也跟着偏，而飞机看起来还能自稳。归属现在由上位机标定
 * （`drv_prop_map.h`）。
 *
 * 查不到通道（未标定）时**物理禁用**，不退回下标顺序：退回去正好是接反时最
 * 危险的那种行为。正常情况下走不到这里——没有标定就不许解锁——但这段代码是
 * 执行器的最后一道关，它不该依赖"上游一定拦住了"这个假设。
 */
static void stabilizer_commit_rotor_pulses(uint16_t upper_us, uint16_t lower_us)
{
  const uint8_t upper_channel =
    DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_UPPER);
  const uint8_t lower_channel =
    DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_LOWER);

  if ((upper_channel == 0U) || (lower_channel == 0U)) {
    BSP_PWM_DisableEsc(1U);
    BSP_PWM_DisableEsc(2U);
    return;
  }
  BSP_PWM_SetEscPulse(upper_channel, upper_us);
  BSP_PWM_SetEscPulse(lower_channel, lower_us);
}

/* 按角色回读刚才发出去的等效微秒命令；未标定时回落到给定通道号。 */
static uint16_t stabilizer_rotor_pulse_readback(uint8_t role,
                                                uint8_t fallback_channel)
{
  const uint8_t channel = DRV_PropMap_EscChannelForRole(role);

  return BSP_PWM_GetEscPulse((channel != 0U) ? channel : fallback_channel);
}

/*
 * 悬停推力在线学习的一拍（app_hover_thrust.h）。不回写 coax.hover_thrust_n；学到的值由紧随其后的 APP_HoverAdapt_Update 接进控制器。
 *
 * 放在电机仲裁之后、从 BSP 回读"这一拍实际发出去的脉宽"：生产的稳定混控、辨识自动油门
 * （ALT/SYSID）和遥控直通油门三条路径提交后都落在同一处，取的是同一个口径，不必在仲裁
 * 分支里各埋一份。推力 = 上下桨脉宽各按"同脉宽合推力"换算的平均（共轴互扰不计，差速幅度小）。
 * 只有 motor_output_reason 属于"已解锁 + 链路正常"那一支时才算有下发推力。
 * 封顶：生产路径看 ctrl_out.thrust_saturated；自动油门路径 app_sysid 的封顶标志不对外，
 * 按它同一个公式（脉宽 ≥ 最高油门 % 对应的封顶脉宽）判。
 */
/*
 * 飞行日志子分频：500 Hz 环每 4 拍取一次四分之一相位（125 Hz），再每 N 次记 1 次（125/N Hz）。
 * 返回 0 的拍整个日志块都不执行，也不调用 APP_FlightLog_Observe——
 * Observe(NULL) 会把 recording 清零并请求 flush，跳拍调用会反复触发 flush。
 */
static uint8_t stabilizer_flight_log_subdiv_due(StabilizerContext *ctx)
{
  uint8_t subdiv = APP_FlightLog_GetSubdiv();

  ctx->flight_log_subdiv_count++;
  if (ctx->flight_log_subdiv_count >= subdiv) {
    ctx->flight_log_subdiv_count = 0U;
    return 1U;
  }
  return 0U;
}

/* 上锁后尾巴按实际记录频率保持约 1 s（125/N 条）。 */
static uint16_t stabilizer_flight_log_tail_records(void)
{
  return (uint16_t)(STABILIZER_FLIGHT_LOG_TAIL_RECORDS / APP_FlightLog_GetSubdiv());
}

static void stabilizer_hover_thrust_step(const StabilizerContext *ctx,
                                         const StabilizerControlFrame *frame)
{
  APP_HoverThrustInput in;
  const DRV_Airframe_Params *airframe = DRV_Airframe_Get();
  const APP_FlightLogMotorOutputReason reason = frame->motor_output_reason;
  const uint8_t production_mix =
    ((reason == APP_FLIGHT_LOG_MOTOR_REASON_STABILIZED_MIX) ||
     (reason == APP_FLIGHT_LOG_MOTOR_REASON_ATTITUDE_DEBUG)) ? 1U : 0U;
  const uint8_t flight_branch =
    ((production_mix != 0U) ||
     (reason == APP_FLIGHT_LOG_MOTOR_REASON_IDENT_DIRECT) ||
     (reason == APP_FLIGHT_LOG_MOTOR_REASON_IMU_INVALID_DIRECT) ||
     (reason == APP_FLIGHT_LOG_MOTOR_REASON_DIRECT_THROTTLE)) ? 1U : 0U;
  float hover_cfg_n = 0.0f;
  uint8_t saturated = 0U;

  in.thrust_n = 0.0f;
  if (flight_branch != 0U) {
    const uint16_t upper_us =
      stabilizer_rotor_pulse_readback((uint8_t)DRV_PROP_ROLE_UPPER, 1U);
    const uint16_t lower_us =
      stabilizer_rotor_pulse_readback((uint8_t)DRV_PROP_ROLE_LOWER, 2U);

    in.thrust_n = 0.5f * (DRV_COAX_CTRL_MotorPulseToTotalThrust(upper_us) +
                          DRV_COAX_CTRL_MotorPulseToTotalThrust(lower_us));
    if (production_mix != 0U) {
      saturated = frame->ctrl_out.thrust_saturated;
    } else if (frame->sysid_running != 0U) {
      float sysid_target_n = 0.0f;
      float sysid_max_pct = 100.0f;
      const float span = (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);

      APP_SysId_GetThrottle(&sysid_target_n, &sysid_max_pct);
      saturated = (upper_us >= (uint16_t)((float)BSP_PWM_ESC_MIN_US +
                                          (sysid_max_pct * span / 100.0f) + 0.5f)) ? 1U : 0U;
    }
  }
  (void)DRV_COAX_CTRL_GetParam("coax.hover_thrust_n", &hover_cfg_n);
  in.now_ms = frame->now_ms;
  in.dt_s = frame->ctrl_dt_sec;
  /* 上锁时电机输出也走这几支（怠速脉宽），只看 flight_branch 会恒判"已解锁"——2026-09-30 首次上板
   * 上锁状态下 HOVER? gate 的解锁位是 1，支撑面高度因此不会在上锁时重锁存。必须叠加真实解锁。 */
  in.armed = ((frame->rc_armed != 0U) && (flight_branch != 0U)) ? 1U : 0U;
  in.thrust_valid = flight_branch;
  in.saturated = saturated;
  in.roll_rad = ctx->roll_control * STABILIZER_DEG_TO_RAD;
  in.pitch_rad = ctx->pitch_control * STABILIZER_DEG_TO_RAD;
  in.a_up_m_s2 = frame->az_up_m_s2;
  in.accel_valid = frame->imu_control_valid;
  in.range_valid = frame->range_raw_valid;
  in.range_m = frame->range_raw_m;
  in.gravity_m_s2 = airframe->gravity_m_s2;
  in.hover_cfg_n = hover_cfg_n;
  in.mass_kg = (DRV_Airframe_IsValid() != 0U) ? airframe->mass_kg : 0.0f;
  APP_HoverThrust_Step(&in);
  /* 学到的悬停推力接进控制器（收敛才用、±15%、慢过渡、每次解锁重学；app_hover_adapt.h），下一拍生效。 */
  APP_HoverAdapt_Update(frame->ctrl_dt_sec, frame->rc_armed,
                        ((frame->ident_running != 0U) || (frame->servo_cal_active != 0U) ||
                         (APP_ThrustBench_IsActive() != 0U) || (APP_PropSpin_IsActive() != 0U)) ? 1U : 0U);
}

static void stabilizer_control_commit(StabilizerContext *ctx,
                                      StabilizerControlFrame *frame)
{
  const APP_ServoType servo_type = APP_ServoType_GetActive();

  if (servo_type == APP_SERVO_TYPE_BUS) {
    BSP_BusServo_Service(frame->now_ms);
  }
  if (frame->servo_cal_active != 0U) {
    /* 手势标定已释放扭矩，地面点动必须立即让位。 */
    APP_ServoJog_ForceRelease("servo_cal");
  }
  if (frame->servo_cal_active == 0U) {
    uint16_t acceptance_alpha_us;
    uint16_t acceptance_beta_us;
    uint8_t acceptance_override;
    const char *jog_yield_reason = NULL;

    acceptance_override = APP_Acceptance_GetServoOverride(&acceptance_alpha_us,
                                                          &acceptance_beta_us);
    if (acceptance_override != 0U) {
      frame->moves[0].pulse_us = acceptance_alpha_us;
      frame->moves[1].pulse_us = acceptance_beta_us;
    }

    APP_ServoFeedbackBench_ApplyTargets(frame->now_ms, frame->moves);

    /* 地面点动仲裁：验收覆盖/反馈台架/解锁任一存在则点动立即让位。 */
    if (acceptance_override != 0U) {
      jog_yield_reason = "acceptance";
    } else if (APP_ServoFeedbackBench_IsActive() != 0U) {
      jog_yield_reason = "fb_bench";
    } else if (frame->rc_armed != 0U) {
      jog_yield_reason = "armed";
    }
    APP_ServoJog_Apply(frame->now_ms, jog_yield_reason,
                       &frame->moves[0].pulse_us,
                       &frame->moves[1].pulse_us);
    stabilizer_servo_record_target(frame->moves);
    /*
     * 舵机回差补偿（app_servo_backlash.h）：只补在飞控制器（已解锁）与光杆辨识的指令，
     * 验收/反馈台架/点动接管的这一拍原样。原地改成发往硬件的脉宽；上面记下的保持目标、
     * 飞行日志 servo_*_us 与辨识记录都是补偿前的，servo_*_sent_us 是补偿后实际发出的。
     */
    APP_ServoBacklash_Apply(frame->now_ms, (APP_ServoBacklashSource)frame->servo_source,
                            frame->rc_armed,
                            ((acceptance_override != 0U) ||
                             (APP_ServoFeedbackBench_IsActive() != 0U) ||
                             (APP_ServoJog_IsActive() != 0U)) ? 1U : 0U,
                            &frame->moves[0].pulse_us, &frame->moves[1].pulse_us);

    /* 仲裁后的最终目标按类型落到硬件；PWM 不进入总线时隙/死区/反馈路径。 */
    if (servo_type == APP_SERVO_TYPE_PWM) {
      BSP_PWM_Status pwm_alpha_status;
      BSP_PWM_Status pwm_beta_status;

      pwm_alpha_status = BSP_PWM_SetServoPulse(1U, frame->moves[0].pulse_us);
      pwm_beta_status = BSP_PWM_SetServoPulse(2U, frame->moves[1].pulse_us);
      if ((pwm_alpha_status == BSP_PWM_OK) &&
          (pwm_beta_status == BSP_PWM_OK)) {
        stabilizer_servo_commit_sent(frame->moves, frame->now_ms);
      }
    } else {
      uint8_t servo_command_slot_due;

      servo_command_slot_due = stabilizer_servo_command_slot_due(frame->now_ms);
      if ((servo_command_slot_due != 0U) &&
          ((stabilizer_servo_should_send(frame->moves, frame->now_ms) != 0U) ||
           (APP_ServoFeedbackBench_MoveRefreshDue(
              frame->now_ms, stabilizer_last_servo_send_ms) != 0U))) {
        DRV_SERVO_Status servo_move_status =
          BSP_BusServo_MoveManyAsync(frame->moves, 2U,
                                     STABILIZER_SERVO_MOVE_TIME_MS);

        stabilizer_servo_bus_diag.move_attempt_count++;
        APP_ServoFeedbackBench_RecordMoveResult(servo_move_status);
        if (servo_move_status == DRV_SERVO_OK) {
          stabilizer_servo_bus_diag.move_sent_count++;
          stabilizer_servo_commit_sent(frame->moves, frame->now_ms);
        } else if (servo_move_status == DRV_SERVO_BUSY) {
          stabilizer_servo_bus_diag.move_busy_count++;
        } else {
          stabilizer_servo_bus_diag.move_error_count++;
        }
      }

      APP_ServoFeedbackBench_Step(frame->now_ms, frame->moves);
      APP_ServoFeedback_Service(
        frame->now_ms, frame->moves,
        (APP_ServoFeedbackBench_IsActive() == 0U) ? 1U : 0U);
    }
  }

  /*
   * 点电机窗口的超时判定必须跑在**这里**，不在收命令的文本任务里。文本任务会
   * 因为 Flash 擦写、大段打印阻塞几十毫秒甚至更久，而"该停了"的判断如果和收
   * 命令的代码在同一个任务里，它们会一起卡住——电机就一直转着。放在 500 Hz 的
   * 提交点上，唯一能让它停不下来的情形是控制环本身死了，而那时看门狗会复位。
   *
   * inhibit：只要有更高优先级的东西在用执行器，窗口立刻关。它不是"让位"，
   * 是关窗——重新开窗需要人再确认一次。
   */
  APP_PropSpin_Step(frame->now_ms,
                    ((frame->rc_armed != 0U) ||
                     (APP_Acceptance_IsActive() != 0U) ||
                     (frame->servo_cal_active != 0U) ||
                     (frame->ident_running != 0U) ||
                     (APP_ThrustBench_IsActive() != 0U)) ? 1U : 0U);
  APP_ThrustBench_Step(frame->now_ms,
                      ((frame->rc_armed != 0U) ||
                       (APP_Acceptance_IsActive() != 0U) ||
                       (frame->servo_cal_active != 0U) ||
                       (frame->ident_running != 0U) ||
                       (APP_PropSpin_IsActive() != 0U)) ? 1U : 0U);
  /*
   * 电调特殊命令窗口的抢占判定同样归这里。它比上面两个更严：**点电机和台架
   * 也算抢占**。理由是命令帧会顶掉这一拍的油门，而那两者正靠连续的油门帧维持
   * 电调解锁——中间插几帧命令，电机会掉出解锁状态，表现为"转着转着停了"。
   */
  APP_EscCommand_Step(((frame->rc_armed != 0U) ||
                       (APP_Acceptance_IsActive() != 0U) ||
                       (frame->servo_cal_active != 0U) ||
                       (frame->ident_running != 0U) ||
                       (APP_PropSpin_IsActive() != 0U) ||
                       (APP_ThrustBench_IsActive() != 0U)) ? 1U : 0U);

  uint8_t thrust_bench_commit_guard = 0U;

  if (APP_Acceptance_IsActive() != 0U) {
    BSP_PWM_DisableEsc(1U);
    BSP_PWM_DisableEsc(2U);
    frame->motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_RC_LOSS_DISABLE;
  } else if (APP_ThrustBench_IsActive() != 0U) {
    APP_ThrustBenchOutput bench;
    BSP_PWM_Status upper_status;
    BSP_PWM_Status lower_status;

    APP_ThrustBench_GetOutput(&bench);
    thrust_bench_commit_guard = 1U;
    upper_status = BSP_PWM_SetEscPulse(1U, bench.pulse_us[0]);
    lower_status = BSP_PWM_SetEscPulse(2U, bench.pulse_us[1]);
    if ((upper_status != BSP_PWM_OK) || (lower_status != BSP_PWM_OK)) {
      APP_ThrustBench_Close(frame->now_ms,
                            APP_THRUST_BENCH_STOP_OUTPUT_ERROR);
      (void)BSP_PWM_DisableEsc(1U);
      (void)BSP_PWM_DisableEsc(2U);
    }
    frame->motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_PROP_SPIN_TEST;
  } else if (APP_PropSpin_IsActive() != 0U) {
    /*
     * 桨叶旋向标定：一次只给一路油门，另一路发零油门码（不是 Disable）——
     * 让电调保持"已上电待命"，否则每换一路都要重新等它上电自检，而中途那几秒
     * 输出是未定义的。窗口关着时 GetOutput 给的就是两路全 0，照发即停。
     */
    APP_PropSpinOutput spin;

    APP_PropSpin_GetOutput(&spin);
    (void)BSP_PWM_SetEscPercent(1U, (uint32_t)spin.percent[0]);
    (void)BSP_PWM_SetEscPercent(2U, (uint32_t)spin.percent[1]);
    frame->motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_PROP_SPIN_TEST;
  } else if (frame->servo_cal_active != 0U) {
    BSP_PWM_SetEscPulse(1, BSP_PWM_ESC_MIN_US);
    BSP_PWM_SetEscPulse(2, BSP_PWM_ESC_MIN_US);
    frame->motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_DISARMED_MIN;
  } else if ((frame->rc_link_ok != 0U) && (frame->rc_armed != 0U)) {
    if ((frame->rc_control_motor_mix_allowed != 0U) &&
        (frame->ident_running == 0U) &&
        (frame->imu_control_valid != 0U)) {
      stabilizer_commit_rotor_pulses(frame->ctrl_out.motor_upper_us,
                                     frame->ctrl_out.motor_lower_us);
      frame->motor_output_reason = (frame->ft_spoolup != 0U) ?
        APP_FLIGHT_LOG_MOTOR_REASON_SPOOLUP :
        ((frame->rc_attitude_debug_mode != 0U) ?
          APP_FLIGHT_LOG_MOTOR_REASON_ATTITUDE_DEBUG :
          APP_FLIGHT_LOG_MOTOR_REASON_STABILIZED_MIX);
    } else {
      uint16_t direct_us = frame->rc_throttle_motor_us;
      uint16_t direct_lower_us;
      uint16_t sysid_upper_us;
      uint16_t sysid_lower_us;

      direct_lower_us = direct_us;
      /* 光杆辨识的自动油门只在这一支（已解锁 + 链路正常）生效；上锁/失联走下面的分支。
       * 吊绳偏航辨识（YAW）上下桨脉宽不同（差速出偏航力矩），其余模式两路相同。 */
      if ((frame->sysid_running != 0U) &&
          (APP_SysId_GetMotorPulsePair(&sysid_upper_us, &sysid_lower_us) != 0U)) {
        direct_us = sysid_upper_us;
        direct_lower_us = sysid_lower_us;
      }
      if (frame->ft_landed_idle != 0U) {
        /* 飞行油门模式 LANDED：地面跟杆慢转（上限 GROUND_MAX，飞不起来），杆位不直接当油门 */
        direct_us = stabilizer_rc_throttle_to_motor_pulse(frame->ft_ground_spin_01);
        direct_lower_us = direct_us;
      }
      if (direct_us == direct_lower_us) {
        BSP_PWM_SetEscPulse(1, direct_us);
        BSP_PWM_SetEscPulse(2, direct_us);
      } else {
        /* 差速只发生在辨识 YAW：按桨位标定把上/下桨脉宽送到各自的 ESC 通道。 */
        stabilizer_commit_rotor_pulses(direct_us, direct_lower_us);
      }
      if (frame->ft_landed_idle != 0U) {
        frame->motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_LANDED_IDLE;
      } else if (frame->ident_running != 0U) {
        frame->motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_IDENT_DIRECT;
      } else if ((frame->rc_control_motor_mix_allowed != 0U) &&
                 (frame->imu_control_valid == 0U)) {
        frame->motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_IMU_INVALID_DIRECT;
      } else {
        frame->motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_DIRECT_THROTTLE;
      }
    }
  } else if ((frame->rc_link_ok != 0U) || (frame->rc_link_seen == 0U)) {
    BSP_PWM_SetEscPulse(1, BSP_PWM_ESC_MIN_US);
    BSP_PWM_SetEscPulse(2, BSP_PWM_ESC_MIN_US);
    frame->motor_output_reason = (frame->rc_link_seen == 0U) ?
      APP_FLIGHT_LOG_MOTOR_REASON_NO_RC_SEEN_MIN :
      APP_FLIGHT_LOG_MOTOR_REASON_DISARMED_MIN;
  } else
  {
    BSP_PWM_DisableEsc(1);
    BSP_PWM_DisableEsc(2);
    frame->motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_RC_LOSS_DISABLE;
  }
  {
    /*
     * 命令帧顶掉这一拍的油门帧。**只在这一层做选择，不动上面整条仲裁**：
     * 上面算出来的油门照常写进 BSP 暂存，只是这一拍不提交它——命令发完之后
     * 下一拍照常提交，电调看到的是"命令 ×N，然后油门回来"，而不是"油门被清零过"。
     * 清零会让电调掉出解锁状态。
     *
     * 抢占已经在 APP_EscCommand_Step() 里判过了：解锁/验收/标定/辨识/点电机/台架
     * 任何一个在跑，窗口此刻已经是 aborted，Pending 返回 0，这里根本走不到。
     */
    uint16_t esc_command = 0U;
    BSP_PWM_Status commit_status;
    if (APP_EscCommand_Pending(&esc_command) != 0U) {
      commit_status = BSP_PWM_CommitEscCommand(esc_command);
      if (commit_status == BSP_PWM_OK) {
        APP_EscCommand_Consume();
      } else if (commit_status != BSP_PWM_BUSY) {
        /* BUSY 是上一帧还没发完，下一拍重来，序列没断；其余错误就是断了。 */
        APP_EscCommand_Fail();
      }
    } else {
      commit_status = BSP_PWM_CommitEsc();
    }
    if ((thrust_bench_commit_guard != 0U) &&
        (commit_status != BSP_PWM_OK)) {
      APP_ThrustBench_Close(frame->now_ms,
                            APP_THRUST_BENCH_STOP_OUTPUT_ERROR);
      (void)BSP_PWM_DisableEsc(1U);
      (void)BSP_PWM_DisableEsc(2U);
    }
  } /* One two-channel frame after the existing arbitration. */
  if (APP_Acceptance_IsActive() != 0U) {
    APP_AcceptanceObservation observation;
    APP_ServoFeedbackLogSample feedback;
    DRV_COAX_CTRL_Debug debug;
    DRV_COAX_CTRL_Params params;
    memset(&observation, 0, sizeof(observation));
    APP_ServoFeedback_GetLogSample(frame->now_ms, &feedback);
    DRV_COAX_CTRL_GetLastDebug(&debug);
    DRV_COAX_CTRL_GetParams(&params);
    observation.timestamp_us = ctx->last_msg.base.timestamp_us;
    observation.sequence = ctx->last_msg.base.sequence;
    observation.calibration_generation = ctx->imu_calibration_generation;
    SVC_FlowNav_GetVelocity(&observation.nav_velocity_m_s[0],
                            &observation.nav_velocity_m_s[1]);
    observation.angle_deg[0] = ctx->roll_control;
    observation.angle_deg[1] = ctx->pitch_control;
    observation.rate_dps[0] = ctx->last_msg.imu.gyro_x_dps;
    observation.rate_dps[1] = ctx->last_msg.imu.gyro_y_dps;
    observation.moment_n_m[0] = debug.moment_cmd_n_m[0];
    observation.moment_n_m[1] = debug.moment_cmd_n_m[1];
    observation.restoring_moment_n_m[0] =
      -params.rate.kp[0] * params.attitude.att_kp[0] *
       debug.attitude_error[0];
    observation.restoring_moment_n_m[1] =
      -params.rate.kp[1] * params.attitude.att_kp[1] *
       debug.attitude_error[1];
    observation.damping_moment_n_m[0] =
      debug.rate_p_n_m[0] - observation.restoring_moment_n_m[0];
    observation.damping_moment_n_m[1] =
      debug.rate_p_n_m[1] - observation.restoring_moment_n_m[1];
    observation.rc_us[0] = frame->rc.us[APP_RC_FUNC_ROLL];
    observation.rc_us[1] = frame->rc.us[APP_RC_FUNC_PITCH];
    observation.rc_us[2] = frame->rc.us[APP_RC_FUNC_YAW];
    observation.servo_command_us[0] = frame->moves[0].pulse_us;
    observation.servo_command_us[1] = frame->moves[1].pulse_us;
    observation.servo_sent_us[0] = stabilizer_last_successful_servo_pulse_us[0];
    observation.servo_sent_us[1] = stabilizer_last_successful_servo_pulse_us[1];
    observation.servo_feedback_us[0] = feedback.position_us[0];
    observation.servo_feedback_us[1] = feedback.position_us[1];
    observation.servo_feedback_age_ms[0] = feedback.age_ms[0];
    observation.servo_feedback_age_ms[1] = feedback.age_ms[1];
    observation.orientation_code = ctx->imu_frame_orientation_code;
    observation.calibration_valid_mask = ctx->imu_calibration_valid_mask;
    observation.servo_feedback_valid_mask = feedback.valid_mask;
    observation.link_present = frame->rc_link_ok;
    APP_Acceptance_PublishObservation(&observation);
  }

  stabilizer_hover_thrust_step(ctx, frame);

  if (((ctx->flight_log_divider & 0x03U) == 0U) && (stabilizer_flight_log_subdiv_due(ctx) != 0U)) {
    APP_FlightLogSnapshot flog_snapshot;
    APP_ServoFeedbackLogSample servo_feedback_sample;
    APP_OPTICAL_FLOW_Status flow_status;
    uint8_t flight_log_active =
      ((frame->rc_link_ok != 0U) &&
       (frame->rc_armed != 0U) &&
       (frame->rc_control_motor_mix_allowed != 0U)) ? 1U : 0U;
    uint8_t flight_log_should_record = flight_log_active;

    memset(&flog_snapshot, 0, sizeof(flog_snapshot));
    APP_OpticalFlow_GetStatus(&flow_status);
    /*
     * 坐标溯源（R-F5b）：全部取自控制环里本就现成的量，无额外开销。
     * 读方据 frame_orientation_code 判断这批记录的姿态是规范 FLU（0..23）
     * 还是 legacy 中间轴（255），不必再靠录制日期猜。
     */
    flog_snapshot.frame_orientation_code = ctx->imu_frame_orientation_code;
    flog_snapshot.frame_contract_version = DRV_FRAME_CONTRACT_VERSION;
    flog_snapshot.calibration_generation = ctx->imu_calibration_generation;
    flog_snapshot.timestamp_us = ctx->last_msg.base.timestamp_us;
    flog_snapshot.tick_ms = frame->now_ms;
    flog_snapshot.imu_sequence = ctx->last_msg.base.sequence;
    flog_snapshot.imu_raw = ctx->last_msg.raw_imu;
    flog_snapshot.imu = ctx->last_msg.imu;
    flog_snapshot.roll_deg = ctx->roll_control;
    flog_snapshot.pitch_deg = ctx->pitch_control;
    flog_snapshot.yaw_deg = ctx->yaw_control;
    memcpy(flog_snapshot.rc_channels,
           frame->ch,
           sizeof(flog_snapshot.rc_channels));
    flog_snapshot.throttle_us = frame->rc_throttle_motor_us;
    /* 补偿前的目标（与辨识同一口径）；补偿后实际发出的是下面的 *_sent_us。 */
    flog_snapshot.servo_alpha_us = stabilizer_latest_servo_target_us[0];
    flog_snapshot.servo_beta_us = stabilizer_latest_servo_target_us[1];
    flog_snapshot.servo_alpha_sent_us =
      stabilizer_last_successful_servo_pulse_us[0];
    flog_snapshot.servo_beta_sent_us =
      stabilizer_last_successful_servo_pulse_us[1];
    APP_ServoFeedback_GetLogSample(frame->now_ms, &servo_feedback_sample);
    flog_snapshot.servo_alpha_feedback_us =
      servo_feedback_sample.position_us[0];
    flog_snapshot.servo_beta_feedback_us =
      servo_feedback_sample.position_us[1];
    flog_snapshot.servo_alpha_feedback_age_ms =
      servo_feedback_sample.age_ms[0];
    flog_snapshot.servo_beta_feedback_age_ms =
      servo_feedback_sample.age_ms[1];
    flog_snapshot.servo_alpha_feedback_sequence =
      servo_feedback_sample.sample_sequence[0];
    flog_snapshot.servo_beta_feedback_sequence =
      servo_feedback_sample.sample_sequence[1];
    flog_snapshot.servo_feedback_valid_mask =
      servo_feedback_sample.valid_mask;
    flog_snapshot.servo_move_attempt_count =
      stabilizer_servo_bus_diag.move_attempt_count;
    flog_snapshot.servo_move_sent_count =
      stabilizer_servo_bus_diag.move_sent_count;
    flog_snapshot.servo_move_busy_count =
      stabilizer_servo_bus_diag.move_busy_count;
    flog_snapshot.servo_move_error_count =
      stabilizer_servo_bus_diag.move_error_count;
    flog_snapshot.servo_feedback_request_count =
      servo_feedback_sample.request_count;
    flog_snapshot.servo_feedback_response_count =
      servo_feedback_sample.response_count;
    flog_snapshot.servo_feedback_timeout_count =
      servo_feedback_sample.timeout_count;
    flog_snapshot.servo_feedback_parse_error_count =
      servo_feedback_sample.parse_error_count;
    flog_snapshot.servo_feedback_uart_error_count =
      servo_feedback_sample.uart_error_count;
    flog_snapshot.servo_feedback_busy_count =
      servo_feedback_sample.busy_count;
    flog_snapshot.flow_raw_x = flow_status.flow_vel_x;
    flog_snapshot.flow_raw_y = flow_status.flow_vel_y;
    flog_snapshot.flow_sample_age_ms =
      (uint16_t)((flow_status.flow_age_ms > 65535U) ?
                 65535U : flow_status.flow_age_ms);
    flog_snapshot.flow_height_age_ms =
      (uint16_t)((flow_status.distance_age_ms > 65535U) ?
                 65535U : flow_status.distance_age_ms);
    flog_snapshot.flow_quality = flow_status.flow_quality;
    flog_snapshot.flow_valid =
      ((flow_status.valid != 0U) &&
       (flow_status.flow_status == 1U) &&
       (flow_status.flow_quality >= SVC_FLOW_NAV_MIN_QUALITY)) ?
      1U : 0U;
    flog_snapshot.flow_velocity_valid = flow_status.velocity_valid;
    flog_snapshot.flow_height_valid = flow_status.height_valid;
    flog_snapshot.flow_height_raw_m = flow_status.height_raw_m;
    flog_snapshot.flow_height_m = flow_status.height_m;
    memcpy(flog_snapshot.flow_sensor_velocity_m_s,
           stabilizer_flow_debug.sensor_velocity_m_s,
           sizeof(flog_snapshot.flow_sensor_velocity_m_s));
    memcpy(flog_snapshot.flow_optical_rot_comp_m_s,
           stabilizer_flow_debug.optical_rot_comp_m_s,
           sizeof(flog_snapshot.flow_optical_rot_comp_m_s));
    memcpy(flog_snapshot.flow_offset_rot_comp_m_s,
           stabilizer_flow_debug.offset_rot_comp_m_s,
           sizeof(flog_snapshot.flow_offset_rot_comp_m_s));
    memcpy(flog_snapshot.flow_corrected_velocity_m_s,
           stabilizer_flow_debug.corrected_velocity_m_s,
           sizeof(flog_snapshot.flow_corrected_velocity_m_s));
    /*
     * 按角色回读，不按通道号。这两列会进系统辨识：上下桨的偏航力臂不同，
     * 标反了之后辨识出来的是另一架飞机的参数，而数据本身看起来毫无异常。
     * 未标定时回落到通道顺序——那种状态下解锁被挡着，两路发的是同一个
     * 停机值，回落不可能把有意义的东西贴错标签。
     */
    flog_snapshot.motor_upper_us = stabilizer_rotor_pulse_readback(
      (uint8_t)DRV_PROP_ROLE_UPPER, 1U);
    flog_snapshot.motor_lower_us = stabilizer_rotor_pulse_readback(
      (uint8_t)DRV_PROP_ROLE_LOWER, 2U);
    flog_snapshot.rc_armed = frame->rc_armed;
    flog_snapshot.rc_link_ok = frame->rc_link_ok;
    flog_snapshot.throttle_over_20 = frame->rc_use_stabilized_motor_mix;
    flog_snapshot.imu_valid = frame->imu_control_valid;
    flog_snapshot.motor_output_reason = (uint8_t)frame->motor_output_reason;
    flog_snapshot.rc_link_seen = frame->rc_link_seen;
    flog_snapshot.arm_switch_high = frame->rc_arm_switch_high;
    flog_snapshot.arm_throttle_low = frame->rc_arm_throttle_low;
    flog_snapshot.arm_switch_seen_low = stabilizer_rc_switch_seen_low;
    flog_snapshot.arm_switch_prev_high = stabilizer_rc_switch_prev_high;
    flog_snapshot.imu_fault_latched = stabilizer_imu_fault_latched;
    flog_snapshot.imu_fault_reason = (uint8_t)stabilizer_imu_fault_reason;
    memcpy(flog_snapshot.acc_nav_m_s2,
           ctx->vofa_debug.acc_nav_m_s2,
           sizeof(flog_snapshot.acc_nav_m_s2));
    memcpy(flog_snapshot.vel_est_m_s,
           ctx->vofa_debug.vel_est_m_s,
           sizeof(flog_snapshot.vel_est_m_s));
    memcpy(flog_snapshot.vel_ref_m_s,
           ctx->vofa_debug.vel_ref_m_s,
           sizeof(flog_snapshot.vel_ref_m_s));
    memcpy(flog_snapshot.vel_err_m_s,
           ctx->vofa_debug.vel_err_m_s,
           sizeof(flog_snapshot.vel_err_m_s));
    memcpy(flog_snapshot.vel_pid_out_m_s2,
           ctx->vofa_debug.vel_pid_out_m_s2,
           sizeof(flog_snapshot.vel_pid_out_m_s2));
    memcpy(flog_snapshot.vel_pid_p_m_s2,
           ctx->vofa_debug.vel_pid_p_m_s2,
           sizeof(flog_snapshot.vel_pid_p_m_s2));
    memcpy(flog_snapshot.vel_pid_i_m_s2,
           ctx->vofa_debug.vel_pid_i_m_s2,
           sizeof(flog_snapshot.vel_pid_i_m_s2));
    memcpy(flog_snapshot.vel_pid_d_m_s2,
           ctx->vofa_debug.vel_pid_d_m_s2,
           sizeof(flog_snapshot.vel_pid_d_m_s2));
    flog_snapshot.vel_loop_active = ctx->vofa_debug.vel_loop_active;
    DRV_COAX_CTRL_GetLastDebug(&flog_snapshot.ctrl_debug);
    flog_snapshot.z_ref_m = ctx->vofa_debug.altitude_ref_m;
    flog_snapshot.ident_att = frame->ident_att_log;
    /* v12 尾块：低字节标志由模块取，高字节是本拍稳定器自己的有效位。 */
    APP_FlightLogNav_Capture(
      &flog_snapshot.nav,
      (uint16_t)(
        ((frame->range_height_valid != 0U) ?
           APP_FLIGHT_LOG_NAV_FLAG_RANGE_HEIGHT_VALID : 0U) |
        ((frame->reference.navigation_position_valid != 0U) ?
           APP_FLIGHT_LOG_NAV_FLAG_POSITION_VALID : 0U) |
        ((frame->reference.horizontal_velocity_valid != 0U) ?
           APP_FLIGHT_LOG_NAV_FLAG_HORIZ_VEL_VALID : 0U) |
        ((frame->attitude.acceleration_valid != 0U) ?
           APP_FLIGHT_LOG_NAV_FLAG_ACCEL_VALID : 0U) |
        ((frame->rc_attitude_debug_mode != 0U) ?
           APP_FLIGHT_LOG_NAV_FLAG_ATTITUDE_DEBUG : 0U)));

    if (flight_log_active != 0U) {
      ctx->flight_log_tail_records = stabilizer_flight_log_tail_records();
    } else if (ctx->flight_log_tail_records > 0U) {
      ctx->flight_log_tail_records--;
      flight_log_should_record = 1U;
    }

    APP_FlightLog_Observe(flight_log_should_record ? &flog_snapshot : NULL,
                          flight_log_should_record);
  }
  ctx->flight_log_divider = (uint8_t)((uint32_t)(ctx->flight_log_divider + 1U) & 0x03U);
}

static void stabilizer_control_step(StabilizerContext *ctx)
{
  StabilizerControlFrame frame;
  SVC_FLOW_NAV_State navigation_state;

  memset(&frame, 0, sizeof(frame));
  memset(&navigation_state, 0, sizeof(navigation_state));
  frame.now_us = SVC_Timestamp_Us();
  frame.now_ms = SVC_Timestamp_Ms();
  SVC_FlowNav_GetState(&navigation_state);
  /*
   * 位置/速度环（含高度）在光流速度或测距任一有新样本时推进：令牌把两个样本时刻拼在一起，任一变化即"新"。
   * 2026-10-01 自由飞：机身快速自旋时光流速度判无效 1.7 s，旧令牌只认光流，高度环随之冻结在最大推力上
   * 冲到 1.7 m；分轴有效性见 DRV_COAX_CTRL_Reference.split_axis_validity。
   */
  APP_ControlScheduler_Step(&ctx->control_scheduler,
                            frame.now_us,
                            (((uint64_t)navigation_state.height_sample_ms) << 32) |
                              (uint64_t)navigation_state.velocity_sample_ms,
                            ((navigation_state.velocity_valid != 0U) ||
                             (navigation_state.height_valid != 0U)) ? 1U : 0U,
                            &frame.cascade_schedule);

  if (frame.cascade_schedule.rate_due != 0U) {
    APP_ControlScheduler_Commit(&ctx->control_scheduler,
                                frame.now_us,
                                (uint64_t)navigation_state.velocity_sample_ms,
                                &frame.cascade_schedule);
    ctx->last_out_ms = frame.now_ms;

    /* 声明里的非零初值，memset 后需显式恢复 */
    frame.rc_throttle_motor_us = BSP_PWM_ESC_MIN_US;
    frame.led_arm_block_reason = APP_LED_ARM_BLOCK_NO_RC;
    frame.motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_UNKNOWN;
    frame.ctrl_dt_sec = frame.cascade_schedule.rate_dt_s;

    stabilizer_control_prepare(ctx, &frame);
    stabilizer_control_compute(ctx, &frame);
    stabilizer_control_commit(ctx, &frame);
    APP_RpmNotch_Tick();
  }
}

void APP_Stabilizer_Run(osSemaphoreId_t imu_ready_sem,
                        osMessageQueueId_t sensor_sample_q,
                        osMessageQueueId_t vofa_log_q)
{
  StabilizerContext ctx;
  APP_Sensor_SampleMessage msg;

  s_imu_ready_sem = imu_ready_sem;
  s_sensor_q = sensor_sample_q;
  s_vofa_q = vofa_log_q;

  stabilizer_init(&ctx);

  for (;;)
  {
  /*
   * 步骤 0：阻塞等待 IMU 数据就绪信号量
   * Sensor_Task 在每次 IMU 采样完成后 release 此信号量。
   * 超时 1ms——正常情况下信号量在中断后立即到来。
   */
    (void)osSemaphoreAcquire(s_imu_ready_sem, 1U);

  /* 步骤 0.5：ELRS 遥控器链路状态机步进（非阻塞，每次 IMU 采样后执行） */
    APP_ELRS_Step();

  /*
   * 步骤 1：消费 SensorSampleQueue 中的所有待处理数据
   * 使用 while 循环 + 零超时（0U）一次性清空队列，只保留最新一帧。
   * 如果消费者慢于生产者（1kHz IMU），队列深度 8 提供缓冲。
   */
    while (osMessageQueueGet(s_sensor_q, &msg, 0U, 0U) == osOK) {
      stabilizer_imu_step(&ctx, &msg);
    }
    ctx.last_msg = msg;

  /*
   * 步骤 2：按固定周期（50Hz）输出舵机控制
   * 控制输出与姿态解算解耦——不管 IMU 来多快，舵机始终 50Hz 更新。
   */
    stabilizer_control_step(&ctx);
  }
}
