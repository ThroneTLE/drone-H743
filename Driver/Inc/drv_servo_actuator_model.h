#ifndef DRV_SERVO_ACTUATOR_MODEL_H
#define DRV_SERVO_ACTUATOR_MODEL_H

/*
 * 总线舵机的执行器辨识结果（FOPDT 拟合）。
 *
 * 这个文件是 drv_airframe_model.h 的残留。那个文件原来还装着整机质量、重心、
 * 惯量、力臂、旋向等等——2026-09-11 全部搬去了运行时参数
 * （Driver/Inc/drv_airframe_params.h），唯一来源改成上位机写进 Flash 的那一份，
 * 代码里不再保留副本。留下的只有下面这组**辨识产物**。
 *
 * 为什么这组还留在代码里：
 *   1. 它不是量出来的尺寸，是一次台架辨识的拟合系数，连同 R²、RMSE、样本数
 *      一起才有意义——这是一条**证据记录**，不是一个可以随手改的参数。
 *   2. 飞控固件目前**一处都不用它**，只有 tools/sim_xz 的仿真被控对象在用。
 *      把它做成可在线改写的参数，等于给一份历史实验结果开一个写入口。
 *
 * 待办（机体模型改造的第二阶段）：若将来要做舵机前馈补偿，这组系数应该和
 * IMU/舵机机械标定一样，走"标定记录 + 版本 + 时间戳"的持久化路径，而不是
 * 变成 airframe.* 里的几个裸浮点数。
 *
 * 采集条件：满电、电机停转、100 Hz PRAD。
 * Alpha 是软件舵机索引 0 / ID1，驱动云台 beta（横滚力矩）。
 * Beta  是软件舵机索引 1 / ID2，驱动云台 alpha（俯仰力矩）。
 * 这两组 ±200 us 阶跃的 FOPDT 拟合是大信号工程模型；中立位反馈是编码器读数，
 * 不是机械零位修正量。
 */

#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_IDENT_STEP_US      200.0f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_GAIN                 0.781794f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_DELAY_S              0.016231f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_TAU_INCREASE_S       0.071236f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_TAU_DECREASE_S       0.306416f
#define DRV_AIRFRAME_SERVO_ALPHA_NEUTRAL_FEEDBACK_US        1457.177648f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_FIT_R2               0.963346f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_FIT_RMSE_US          17.696884f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_FIT_SAMPLE_COUNT    835U

#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_IDENT_STEP_US       200.0f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_GAIN                  0.769332f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_DELAY_S               0.041320f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_TAU_INCREASE_S        0.076881f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_TAU_DECREASE_S        0.057051f
#define DRV_AIRFRAME_SERVO_BETA_NEUTRAL_FEEDBACK_US         1509.527201f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_FIT_R2                0.994138f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_FIT_RMSE_US            8.459764f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_FIT_SAMPLE_COUNT    1253U

#endif /* DRV_SERVO_ACTUATOR_MODEL_H */
