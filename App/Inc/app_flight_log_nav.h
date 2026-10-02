#ifndef APP_FLIGHT_LOG_NAV_H
#define APP_FLIGHT_LOG_NAV_H

/*
 * 飞行日志 v12 的「导航/电池」尾块：为离线分析光流定点悬停补的那几个量。
 * 位置/速度/PID 各项/保护缩放本来就在 DRV_COAX_CTRL_Debug 里（ctrl_*），
 * 这里只放它们之外、事后复盘还缺的东西：
 *   - EKF 加速度零偏与新息：零偏漂移、光流量测被拒是定点飘走的两大根因，
 *     没有它们只能看到"速度估计不对"而分不清是传感器还是估计器；
 *   - 窗口去毛刺后的光流计数（已是规范 FLU）：和 flow_raw_x/y（传感器原始帧，
 *     FRD、未经安装方向）并排才能判断是滤波丢了尺度还是传感器本身；
 *   - 电池电压：推力随电压掉，定点后半程变松多半是它；
 *   - 状态位：各路"有效"标志，解释为什么某一段位置环被旁路。
 *
 * 本头文件只依赖 <stdint.h>，宿主侧测试可单独编译。
 * 布局冻结：以后加字段升日志版本、另起新尾块，不改这里的顺序。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* nav_flags：低 8 位由本模块从服务层/电池取，高位由调用方（稳定器本拍的 frame）填。 */
#define APP_FLIGHT_LOG_NAV_FLAG_VELOCITY_VALID     (1U << 0) /* 光流导航速度有效 */
#define APP_FLIGHT_LOG_NAV_FLAG_FLOW_FILTER_READY  (1U << 1) /* 光流去毛刺窗已填满 */
#define APP_FLIGHT_LOG_NAV_FLAG_EKF_INITIALIZED    (1U << 2)
#define APP_FLIGHT_LOG_NAV_FLAG_BATTERY_VALID      (1U << 3)
#define APP_FLIGHT_LOG_NAV_FLAG_BATTERY_LOW        (1U << 4)
#define APP_FLIGHT_LOG_NAV_FLAG_BATTERY_CAN_ARM    (1U << 5)
#define APP_FLIGHT_LOG_NAV_FLAG_BATTERY_SATURATED  (1U << 6)
#define APP_FLIGHT_LOG_NAV_FLAG_RANGE_HEIGHT_VALID (1U << 8)  /* 测距高度有效 */
#define APP_FLIGHT_LOG_NAV_FLAG_POSITION_VALID     (1U << 9)  /* 位置环测量有效 */
#define APP_FLIGHT_LOG_NAV_FLAG_HORIZ_VEL_VALID    (1U << 10) /* 水平速度环在用光流速度 */
#define APP_FLIGHT_LOG_NAV_FLAG_ACCEL_VALID        (1U << 11) /* 速度环的加速度测量有效 */
#define APP_FLIGHT_LOG_NAV_FLAG_ATTITUDE_DEBUG     (1U << 12) /* 姿态调试模式（水平环旁路） */
#define APP_FLIGHT_LOG_NAV_CALLER_MASK             0xFF00U

typedef struct __attribute__((packed)) {
    float ekf_accel_bias_m_s2[2];   /* EKF 加速度零偏，机头对齐水平系（X 前 / Y 左） */
    float ekf_innovation_m_s[2];    /* 最近一次光流量测的新息 */
    float ekf_nis;                  /* 最近一次新息的归一化平方 */
    uint32_t ekf_flow_update_count; /* 累计被 EKF 接受的光流量测数 */
    uint32_t ekf_flow_reject_count; /* 累计被 NIS 门拒绝的光流量测数 */
    int16_t flow_filtered_x;        /* 去毛刺窗均值后的光流计数，规范 FLU（同 SVC_FLOW_NAV_Sample） */
    int16_t flow_filtered_y;
    uint16_t battery_mv;            /* 电池总压 [mV]，截断到 65535；无效时 0 */
    uint16_t nav_flags;             /* APP_FLIGHT_LOG_NAV_FLAG_* */
} APP_FlightLogNavTail;

_Static_assert(sizeof(APP_FlightLogNavTail) == 36U,
               "flight log nav tail must match tools/flight_log_receive.py");

/* caller_flags 只取高字节（APP_FLIGHT_LOG_NAV_CALLER_MASK），低字节由本函数填。 */
void APP_FlightLogNav_Capture(APP_FlightLogNavTail *tail, uint16_t caller_flags);

#ifdef __cplusplus
}
#endif

#endif
