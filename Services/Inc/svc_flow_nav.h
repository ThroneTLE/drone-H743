#ifndef SVC_FLOW_NAV_H
#define SVC_FLOW_NAV_H

#include <stdint.h>

#include "drv_nav_ekf.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 功能：光流 / 组合测距导航服务层。
 *
 * 这个模块没有自己的任务，不属于 App 执行体。它是整条光流链路上唯一一处做数学
 * 运算的地方，是"给算法层用的光流数据"的唯一出口：
 *
 *   Driver（drv_optical_flow.c）  只做 MSP/MicoLink 取帧，不含任何滤波与判决
 *        ↓ 原始帧
 *   Service（本模块）             高度 LPF、光流中值 + 速度换算、质量/时效/合理性
 *                                门控、水平速度估计（EKF）、位置与累计位移积分
 *        ↓ 成品导航量
 *   App（app_optical_flow.c / app_stabilizer.c）
 *                                设备生命周期（初始化/健康/恢复）、坐标系换算、
 *                                旋转补偿、控制律；不再自己算滤波与估计
 *
 * 两个调用上下文：
 *   - 传感器任务：SVC_FlowNav_PushSample()，每拿到一帧新数据调一次；
 *   - 控制任务：  SVC_FlowNav_Fuse()，每个控制节拍调一次。
 * 二者共享的都是 32 位对齐标量，与本仓库既有的 flow/stabilizer 跨任务读法一致，
 * 不额外加锁。
 *
 * 时间基准（重要，两者不是一回事，不能混用）：
 *   - EKF predict 用控制环 dt：那是 IMU 加速度的传播步长，本来就该按 IMU 节拍走；
 *   - 位置 / 累计位移积分用传感器自己的 time_ms 差分（退化时用 sample_interval_us）：
 *     光流速度是传感器在自己时间轴上测出来的量，拿控制环节拍或上位机轮询周期去积
 *     分它，丢帧时会凭空多算或少算路程。
 */

/*
 * 硬拒门：低于此值整帧丢弃。MicoLink flow_quality 是 uint8（0~255）。
 *
 * 取 45 的依据（作者实机目视观察，2026-09-03）：
 *   - 手掌遮住镜头 → quality < 5，这才是"传感器什么也看不见"的地板；
 *   - 一般光照 100~140，条件最好可达 190；
 *   - 从亮处移到暗处，自动曝光重新收敛约需 3s，但暗处最低也有 50。
 * 45 落在地板（<5）与暗处最低值（50）之间，既能挡住真正瞎掉的帧，又不会在
 * 每次光照变化时把整段曝光收敛期判死。
 *
 * 历史值 80 无任何出处，且高于暗处最低值 50：光照一变就连续拒帧，超过
 * SVC_FLOW_NAV_EKF_FLOW_STALE_RESET_MS 后估计器会把水平速度归零——把一个
 * 可自愈的瞬态放大成了估计器复位。
 */
#define SVC_FLOW_NAV_MIN_QUALITY              45U

/*
 * 质量自适应量测噪声曲线的低锚点，与上面的硬拒门**分开**。
 *
 * 这两件事本来被同一个宏兼任：降拒门会连带把曲线重锚，等于顺手提高了对刚好
 * 过门数据的信任度。拆开之后：
 *   q < 45          → 整帧丢弃（真瞎了）
 *   45 <= q < 80    → 收下，但 R 取最大值 0.45 m/s，EKF 几乎不信它；
 *                     好处是不丢量测、不触发 250ms 归零、中值窗保持热态
 *   80 <= q <= 190  → 曲线原样，与重构前逐字节一致
 * 这个锚点该不该动是独立的调参问题，需要各自的证据。
 */
#define SVC_FLOW_NAV_QUALITY_NOISE_LOW        80U

/* 高度与垂直速度 */
#define SVC_FLOW_NAV_TIMEOUT_MS               100U
#define SVC_FLOW_NAV_HEIGHT_LPF_ALPHA         0.35f
#define SVC_FLOW_NAV_VELOCITY_LPF_ALPHA       0.20f
#define SVC_FLOW_NAV_MAX_HEIGHT_M             12.0f
#define SVC_FLOW_NAV_MAX_HEIGHT_STEP_M        0.18f
#define SVC_FLOW_NAV_MAX_VERTICAL_VEL_M_S     5.0f

/* 光流中值窗与传感器系速度合理性 */
#define SVC_FLOW_NAV_MEDIAN_WINDOW            5U
#define SVC_FLOW_NAV_MEDIAN_MIN_SAMPLES       3U
#define SVC_FLOW_NAV_FILTER_RESET_MS          250U
#define SVC_FLOW_NAV_MAX_SPEED_M_S            2.50f
#define SVC_FLOW_NAV_MAX_SPEED_STEP_M_S       1.20f

/* 水平速度估计（EKF 段，数值与重构前逐一对应） */
#define SVC_FLOW_NAV_EKF_FLOW_NOISE_M_S       0.25f
#define SVC_FLOW_NAV_EKF_FLOW_NOISE_MIN_M_S   0.04f
#define SVC_FLOW_NAV_EKF_FLOW_NOISE_MAX_M_S   0.45f
#define SVC_FLOW_NAV_EKF_FLOW_QUALITY_HIGH    180U
#define SVC_FLOW_NAV_EKF_IMU_BRIDGE_TIMEOUT_MS 80U
#define SVC_FLOW_NAV_EKF_FLOW_SOFT_HOLD_MS    150U
#define SVC_FLOW_NAV_EKF_FLOW_STALE_RESET_MS  250U
#define SVC_FLOW_NAV_EKF_FLOW_LOST_DECAY_HZ   1.0f
#define SVC_FLOW_NAV_EKF_FLOW_STALE_DECAY_HZ  12.0f
#define SVC_FLOW_NAV_EKF_CONTROL_MAX_SPEED_M_S 1.50f
#define SVC_FLOW_NAV_EKF_ZERO_FLOW_SPEED_M_S  0.035f
#define SVC_FLOW_NAV_EKF_ZERO_ACCEL_M_S2      0.30f
#define SVC_FLOW_NAV_EKF_ZERO_FLOW_COUNT      8U
#define SVC_FLOW_NAV_FLOW_ONLY_MAX_ACCEL_M_S2 30.0f
#define SVC_FLOW_NAV_FLOW_ONLY_MIN_STEP_M_S   0.30f
#define SVC_FLOW_NAV_DEFAULT_DT_SEC           0.001f

/* 位置积分 */
#define SVC_FLOW_NAV_POSITION_LIMIT_M         2.00f
/*
 * 单步积分允许的传感器时间跨度 [µs]。100Hz 正常是 10000µs；丢帧时按实际跨度积分，
 * 但超过 60ms 的空档只重新落锚不外推——那已经超过 SVC_FLOW_NAV_TIMEOUT_MS，
 * 速度早就作废了，硬积进去只会造出不存在的路程。
 */
#define SVC_FLOW_NAV_MIN_INTEGRATION_DT_US    1000UL
#define SVC_FLOW_NAV_MAX_INTEGRATION_DT_US    60000UL

typedef enum {
    SVC_FLOW_NAV_SAMPLE_HOLD = 0,     /* 无新帧，上一帧仍在有效期内 */
    SVC_FLOW_NAV_SAMPLE_STALE,        /* 无新帧且上一帧已过期 */
    SVC_FLOW_NAV_SAMPLE_REJECTED,     /* 质量 / 时效 / 合理性门拒绝 */
    SVC_FLOW_NAV_SAMPLE_WARMUP,       /* 中值窗未满，还不能出速度 */
    SVC_FLOW_NAV_SAMPLE_ACCEPTED
} SVC_FLOW_NAV_SampleResult;

/*
 * Driver 解析出来的一帧原始数据，字段与 DRV_OPTICAL_FLOW_Frame 一一对应。
 *
 * R-F6-1（seam 2 导航口径，只钉现状不改数值）：
 *   - flow_vel_x / flow_vel_y：光流定点角速率（int16，单位 0.01 rad/s），
 *     **已是规范 FLU（X 前 / Y 左）**。传感器自身按 X前/Y右(FRD) 报数，方言适配
 *     在 app_optical_flow.c 的 app_flow_fill_sample() 里一次性完成——那是"芯片
 *     原话"变成"载具量测"的唯一边界。SVC_FlowNav 原样透传，自己不重映射轴，
 *     下游也不许再转（见 tests/test_flow_rotation_comp_frame.py）。
 *   - distance_mm：测距传感器读数，标量，不含机体轴信息。
 *   - distance_received_ms / flow_received_ms：本板 HAL_GetTick() 时基（ms），
 *     只用于新鲜度/超时判定，不用于积分步长。
 *   - sensor_time_ms：传感器自带时钟（ms），与 distance_received_ms 不是同一条
 *     时间轴；仅用于位移积分步长（见 SVC_FlowNav_GetLastIntegrationDtUs）。
 *   - sample_interval_us：传感器上报的采样间隔，sensor_time_ms 差分退化时的
 *     后备步长来源。
 */
typedef struct {
    uint8_t  frame_valid;
    uint8_t  distance_valid;
    uint8_t  flow_valid;
    uint32_t distance_mm;
    uint32_t distance_received_ms;
    uint32_t flow_received_ms;
    uint32_t sensor_time_ms;
    uint16_t sample_interval_us;
    int16_t  flow_vel_x;
    int16_t  flow_vel_y;
    uint8_t  flow_quality;
} SVC_FLOW_NAV_Sample;

/*
 * 控制环每拍喂进来的融合输入。
 *
 * R-F6-1（只钉现状）：
 *   - accel_x_m_s2 / accel_y_m_s2：水平比力换算出的加速度，由调用方
 *     （app_stabilizer.c 的 stabilizer_compensated_imu_accel_level_xy）**只用
 *     roll/pitch** 转平，得到**机头对齐的本地水平系**：X 前 / Y 左，随机体
 *     偏航一起转。它不是固定世界系，更不是 NED/ENU——偏航是上电后陀螺积分的
 *     相对值、会漂，不对应真北。
 *     该函数刻意不乘 yaw：多乘一次就会把加速度送进"上电朝向"的固定世界系，
 *     与下面机体系的光流速度差出一个累计偏航角，融合即失效（偏航为 0 时完全
 *     看不出来）。两个输入必须同系是本 Service 的前提。
 *   - flow_vx_m_s / flow_vy_m_s：调用方已在传感器出口完成安装标定、旋转与
 *     偏置补偿；进入本 Service 时必须是规范机体 FLU（X 前/Y 左）。本 Service
 *     与 EKF 不允许再知道传感器原始轴或安装符号。
 *
 *   - yaw_rate_rad_s：规范 FLU 机体偏航角速率 ω_z（正 = 机头左转），取自陀螺
 *     Z 轴。上面那个"机头对齐系"是**随偏航转动的系**，所以速度分量的导数不等
 *     于加速度：
 *         dv/dt|分量 = a - ω × v,  ω = (0, 0, ω_z)
 *         ω × v = (-ω_z*v_y, +ω_z*v_x, 0)
 *     本 Service 据此在调用 EKF predict 前把加速度换成
 *         a_eff = (a_x + ω_z*v_y,  a_y - ω_z*v_x)
 *     用的是**当前**状态估计。它不是坐标变换、更不是符号补偿，而是旋转系里
 *     少不掉的运动学项：不补的话，定速直飞中原地转向会被 EKF 读成"速度不变"，
 *     而机头相对的速度分量其实正在互换。
 *     留 0 等于退回不补偿的旧行为（memset 的调用方因此行为不变）。
 *   - dt_sec：控制环节拍（秒），只喂给 EKF predict，不用于位移积分。
 *   - now_ms：调用方的 HAL_GetTick()，与 SVC_FLOW_NAV_Sample 的
 *     distance_received_ms/flow_received_ms 同一条时间轴。
 */
typedef struct {
    float    accel_x_m_s2;
    float    accel_y_m_s2;
    float    yaw_rate_rad_s;
    float    flow_vx_m_s;
    float    flow_vy_m_s;
    uint8_t  flow_valid;
    uint8_t  flow_quality;
    uint32_t flow_sample_ms;
    float    dt_sec;
    uint32_t now_ms;
} SVC_FLOW_NAV_FuseInput;

/*
 * 只读快照，供 App 组装 FLOW? / STATUS 报告。
 *
 * R-F6-1（只钉现状）：
 *   - height_m / height_raw_m：测距高度，标量，正值 = 离地高度增加（向上），
 *     不是带符号的导航系 Z 坐标；R-F6-2 起调用方（app_stabilizer.c）直接
 *     采用该正值作为自己内部 Z-up 表述，不再取负号。
 *   - vertical_velocity_m_s：height_m 的时间导数，符号跟随 height_m——正值
 *     表示高度增加（上升）。
 *   - vx_m_s / vy_m_s：EKF 融合后的水平速度，与 SVC_FLOW_NAV_FuseInput 的
 *     flow_vx_m_s/flow_vy_m_s 同一规范 FLU 约定（X 前 / Y 左），
 *     EKF 本身不做任何额外旋转。
 *   - height_sample_ms / velocity_sample_ms：HAL_GetTick() 时基。
 *   - flow_vel_x_filtered / flow_vel_y_filtered：中值滤波后的定点计数，与
 *     SVC_FLOW_NAV_Sample.flow_vel_x/y 同口径，同样已是规范 FLU。
 *
 * 光流方言在 app_optical_flow.c 的采集边界就转完了，本 Service 不旋转、不换轴。
 * 对外给出的每一个速度向量——GetSensorVelocity、GetState、GetVelocity——都已经是
 * 规范 FLU。**下游（App、控制器、遥测）不得再做任何 FrdToFlu/FluToFrd。**
 */
typedef struct {
    uint8_t  height_valid;
    float    height_m;
    float    height_raw_m;
    float    vertical_velocity_m_s;
    float    height_filter_alpha;
    uint32_t height_sample_ms;
    uint8_t  velocity_valid;
    float    vx_m_s;
    float    vy_m_s;
    uint32_t velocity_sample_ms;
    uint32_t velocity_reject_count;
    int16_t  flow_vel_x_filtered;
    int16_t  flow_vel_y_filtered;
    uint8_t  flow_filter_ready;
} SVC_FLOW_NAV_State;

void SVC_FlowNav_Init(void);
/* 传感器重新初始化 / 掉线时清掉全部滤波与估计状态。 */
void SVC_FlowNav_Reset(void);

/* --- 传感器侧（传感器任务上下文） --- */
SVC_FLOW_NAV_SampleResult SVC_FlowNav_PushSample(
    const SVC_FLOW_NAV_Sample *sample, uint32_t now_ms);
/* 没有新帧时推进老化：高度/速度超时作废、中值窗按需复位。 */
void SVC_FlowNav_Age(uint32_t now_ms);
uint8_t SVC_FlowNav_GetHeight(float *height_m,
                              float *vertical_velocity_m_s,
                              uint32_t *sample_ms,
                              uint32_t now_ms);
uint8_t SVC_FlowNav_GetSensorVelocity(float *vx_m_s,
                                      float *vy_m_s,
                                      uint32_t *sample_ms,
                                      uint32_t now_ms);
void SVC_FlowNav_GetState(SVC_FLOW_NAV_State *state);
/* 最近一次被接受的光流样本时刻；0 表示上电以来还没有过。 */
uint32_t SVC_FlowNav_GetLastGoodMs(void);

/* --- 估计侧（控制任务上下文） --- */
/* 返回本拍光流量测是否被 EKF 接受。 */
uint8_t SVC_FlowNav_Fuse(const SVC_FLOW_NAV_FuseInput *input);
/*
 * 机体系水平速度，规范 FLU：X 前 / Y 左，与
 * SVC_FLOW_NAV_FuseInput.flow_vx_m_s/flow_vy_m_s 同一约定），未旋转到导航系。
 */
void SVC_FlowNav_GetVelocity(float *vx_m_s, float *vy_m_s);
/*
 * 对 SVC_FlowNav_GetVelocity() 输出按传感器时基做限幅累计
 * 积分，轴约定与速度一致（X 前 / Y 左），不是带符号的导航系坐标。
 */
void SVC_FlowNav_GetPosition(float *x_m, float *y_m);
/*
 * 未限幅的累计位移（里程计），与位置同源同步长，只是不做安全限幅，
 * 也不随控制模式切换归零。轴约定同 SVC_FlowNav_GetPosition()。
 */
void SVC_FlowNav_GetDisplacement(float *dx_m, float *dy_m);
uint32_t SVC_FlowNav_GetIntegratedStepCount(void);
uint32_t SVC_FlowNav_GetLastIntegrationDtUs(void);
/*
 * 低油门直通 / 尚无 IMU 姿态时把估计器与控制位置归零。稳定环每个控制周期都会
 * 调，所以这条路径**不碰**累计位移。
 */
void SVC_FlowNav_ResetEstimator(void);
void SVC_FlowNav_ResetPosition(void);
/* 里程计归零，只在传感器重初始化或显式请求时调。 */
void SVC_FlowNav_ResetDisplacement(void);
void SVC_FlowNav_GetEkfDiagnostics(DRV_NAV_EKF_Diagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif
