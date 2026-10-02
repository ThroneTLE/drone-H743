/*
 * 飞行油门模式（方案 B）纯逻辑：分区、起飞/落地状态机、爬升率、SPOOLUP 斜坡、角度模式分段推力。
 * 契约见 doc/flight-throttle-contract.md。无 HAL、无 RTOS，可宿主编译；
 * app_stabilizer.c 只做胶水。
 */
#ifndef DRV_FLIGHT_THROTTLE_H
#define DRV_FLIGHT_THROTTLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 契约常量 ---- */
#define DRV_FLIGHT_THROTTLE_LOW_MAX        0.10f  /* t <  此值 = LOW                         */
#define DRV_FLIGHT_THROTTLE_DESCEND_MAX    0.40f  /* t <  此值 = DESCEND                     */
#define DRV_FLIGHT_THROTTLE_HOLD_MAX       0.60f  /* t <= 此值 = HOLD，其上 = CLIMB          */
#define DRV_FLIGHT_THROTTLE_V_UP_M_S       0.30f
#define DRV_FLIGHT_THROTTLE_V_DN_M_S       0.20f
#define DRV_FLIGHT_THROTTLE_TAKEOFF_HOLD_S 0.30f  /* CLIMB 区连续时间 -> SPOOLUP             */
#define DRV_FLIGHT_THROTTLE_SPOOLUP_S      1.00f
#define DRV_FLIGHT_THROTTLE_SPOOLUP_FRAC   0.85f  /* 斜坡终点 = 此值 * hover 推力            */
#define DRV_FLIGHT_THROTTLE_LAND1_H_M      0.06f
#define DRV_FLIGHT_THROTTLE_LAND1_VZ_M_S   0.10f
#define DRV_FLIGHT_THROTTLE_LAND1_S        1.00f
#define DRV_FLIGHT_THROTTLE_LAND2_H_M      0.15f
#define DRV_FLIGHT_THROTTLE_LAND2_S        0.50f
#define DRV_FLIGHT_THROTTLE_ANGLE_MID      0.50f  /* 角度模式：杆位 0.5 = 悬停推力           */
/* LANDED 地面跟杆慢转（ESC 行程比例）：作者 2026-10-01"油门在底部基本不转，这样我没办法判定桨叶
 * 有没有异常，低速转也作为判断依据"，并选"只跟杆慢转"（解锁不转、推杆转速慢慢上来）。
 * t<0.10 不转；0.10..0.60 线性 MIN..MAX；>0.60 停在 MAX，SPOOLUP 从 MAX 推力起步。
 * MAX 30% ≈ 总推力 2.8 N（12 V 推力表），不到机重三分之一，飞不起来。 */
#define DRV_FLIGHT_THROTTLE_GROUND_MIN_01  0.05f
#define DRV_FLIGHT_THROTTLE_GROUND_MAX_01  0.30f
/* CH6 三段拨码（作者 2026-10-01："CH6中改为不是定点模式，改成速度为0闭环，也就是没有位置P"，
 * 并确认"CH6是三段拨码"）：开关量 s∈[0,1]（按标定 min..max，中位约 0.5）。
 * s < POSITION_MAX 定点；POSITION_MAX..ANGLE_MIN 速度保持（无位置 P）；> ANGLE_MIN 角度（飞手接管）。 */
#define DRV_FLIGHT_THROTTLE_SW_POSITION_MAX 0.3333f
#define DRV_FLIGHT_THROTTLE_SW_ANGLE_MIN    0.6667f
/* 角度档手控油门（作者 2026-10-01："这个姿态模式貌似也套用了位置模式的油门定高，我想让它更像
 * px4那个一样手控油门"）：杆过一次底后，t ≥ LOW_MAX 即直接按杆出推力（无 60%/0.3 s 起飞门槛、无 1 s 斜坡）；
 * 空中杆到底不停桨，推力保底为 t = LOW_MAX 那一档；推力 ≥ 此比例·hover 才让速率环积分工作（地面不卷积分）。 */
#define DRV_FLIGHT_THROTTLE_MANUAL_I_MIN_FRAC 0.80f

typedef enum {
    DRV_FLIGHT_THROTTLE_MODE_FLIGHT = 0,
    DRV_FLIGHT_THROTTLE_MODE_LEGACY = 1,
} DRV_FlightThrottleMode;

typedef enum {
    DRV_FLIGHT_THROTTLE_STATE_DISARMED = 0,
    DRV_FLIGHT_THROTTLE_STATE_LANDED = 1,
    DRV_FLIGHT_THROTTLE_STATE_SPOOLUP = 2,
    DRV_FLIGHT_THROTTLE_STATE_FLYING = 3,
} DRV_FlightThrottleState;

typedef enum {
    DRV_FLIGHT_THROTTLE_ZONE_LOW = 0,
    DRV_FLIGHT_THROTTLE_ZONE_DESCEND = 1,
    DRV_FLIGHT_THROTTLE_ZONE_HOLD = 2,
    DRV_FLIGHT_THROTTLE_ZONE_CLIMB = 3,
} DRV_FlightThrottleZone;

typedef enum {
    DRV_FLIGHT_THROTTLE_XY_POSITION = 0,  /* 定点：位置 P + 速度环                     */
    DRV_FLIGHT_THROTTLE_XY_VELOCITY = 1,  /* 速度保持：杆给速度，回中=速度 0，无位置 P  */
    DRV_FLIGHT_THROTTLE_XY_ANGLE = 2,     /* 角度：摇杆直接给姿态，油门直接给推力       */
} DRV_FlightThrottleXyMode;

typedef struct {
    float dt_s;
    uint8_t armed;            /* 已解锁                                                   */
    uint8_t link_ok;          /* 遥控链路正常                                             */
    float throttle_01;        /* rc.throttle_01（按标定 min/max）                         */
    float mode_switch_01;     /* CH6 开关量 [0,1]（未绑定时调用方给 0 = 定点）             */
    uint8_t height_valid;     /* 相对高度有效（上一控制拍的结果）                         */
    float height_m;
    float vz_m_s;
    uint8_t imu_valid;
    uint8_t bypass;           /* sysid/ident/推力台/旋向/舵机标定 任一在跑                */
    float hover_thrust_n;     /* 悬停推力（调用方按 coax.hover_thrust_n / 机重选好）       */
    float spool_start_force_n; /* GROUND_MAX 慢转对应的总推力（SPOOLUP 斜坡起点；0 = 从 0 起） */
    float max_thrust_n;       /* 角度档手控油门满杆总推力（ESC 满行程）；≤hover 时按 hover   */
    /* 定高档满杆升/降速度 [m/s]（作者 2026-10-01 要求上位机可配；调用方取 coax.pos_z_vel_up/down_max_m_s）。
     * ≤0 或非有限时退回 DRV_FLIGHT_THROTTLE_V_UP_M_S / V_DN_M_S，memset 0 的旧调用方行为不变。 */
    float v_up_m_s;
    float v_dn_m_s;
} DRV_FlightThrottleInput;

typedef struct {
    DRV_FlightThrottleState state;
    DRV_FlightThrottleZone zone;
    uint8_t legacy_semantics; /* 1 = LEGACY 模式或被旁路：调用方走旧 70% 逻辑，其余字段不用 */
    uint8_t active;           /* state 属于 {SPOOLUP, FLYING} 且非旧语义 -> 稳定混控      */
    float climb_rate_m_s;     /* 定点模式爬升率                                           */
    uint8_t manual_force_valid; /* SPOOLUP 或角度档 FLYING：总推力走 manual_total_force    */
    float manual_force_n;
    float ground_spin_01;     /* LANDED：地面跟杆慢转油门（ESC 行程比例），其余状态 0       */
    DRV_FlightThrottleXyMode xy_mode; /* CH6 三档解码结果（任何状态都给出）             */
    uint8_t manual_integrate; /* 角度档 FLYING 且推力 ≥ MANUAL_I_MIN_FRAC·hover：速率环积分开 */
} DRV_FlightThrottleOutput;

void DRV_FlightThrottle_Init(void);
void DRV_FlightThrottle_Update(const DRV_FlightThrottleInput *in,
                               DRV_FlightThrottleOutput *out);

DRV_FlightThrottleZone DRV_FlightThrottle_ZoneOf(float throttle_01);
float DRV_FlightThrottle_ClimbRate(float throttle_01);
/* 同上，满杆升/降速度由调用方给（≤0 或非有限退回默认常量）；分区边界与 DESCEND 线性形状不变。 */
float DRV_FlightThrottle_ClimbRateWith(float throttle_01, float v_up_m_s, float v_dn_m_s);
/* LANDED 地面跟杆慢转：t<0.10 -> 0；0.10..0.60 -> GROUND_MIN..GROUND_MAX；其上 GROUND_MAX。 */
float DRV_FlightThrottle_GroundSpin01(float throttle_01);
/* CH6 三档：s<1/3 定点、1/3..2/3 速度保持、>2/3 角度；NaN 按定点。 */
DRV_FlightThrottleXyMode DRV_FlightThrottle_XyModeOf(float switch_01);
/* 角度模式总推力：t<=0.5 -> hover*t/0.5；其上线性到 max。 */
float DRV_FlightThrottle_AngleModeForce(float throttle_01, float hover_n, float max_n);
/* 角度档飞行中实际用的推力：杆位先抬到不低于 LOW_MAX（空中杆到底不停桨），再按上式。 */
float DRV_FlightThrottle_ManualForce(float throttle_01, float hover_n, float max_n);

/* 模式开关（RAM）。切换是否允许由调用方（命令）按"未解锁"把关。 */
void DRV_FlightThrottle_SetMode(DRV_FlightThrottleMode mode);
DRV_FlightThrottleMode DRV_FlightThrottle_GetMode(void);

/* 观测（THRMODE?）。 */
DRV_FlightThrottleState DRV_FlightThrottle_GetState(void);
DRV_FlightThrottleZone DRV_FlightThrottle_GetZone(void);
DRV_FlightThrottleXyMode DRV_FlightThrottle_GetXyMode(void);
float DRV_FlightThrottle_GetThrottle01(void);
uint8_t DRV_FlightThrottle_GetHeight(float *height_m); /* 返回高度是否有效 */

#ifdef __cplusplus
}
#endif

#endif
