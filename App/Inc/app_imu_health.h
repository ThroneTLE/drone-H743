#ifndef APP_IMU_HEALTH_H
#define APP_IMU_HEALTH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * IMU 采样链健康监测。
 *
 * 为什么需要它：Sensor_Task 在 DRDY 中断没来时会退到 20ms 轮询兜底。兜底本身
 * 是对的（能扛偶发漏中断），但一旦 DRDY 永久失效，轮询会"成功"拿到数据，于是
 * 整条链静默降到 50Hz 而没有任何报警。
 *
 * 50Hz 不是"性能打折"，是控制律前提失效：
 *   - 低通系数在 APP_Sensor_LpfInit(fc, dt) 里按 dt=1ms 烤死，喂 50Hz 样本时
 *     等效截止频率变成 fc*(50/1000)，陀螺 80Hz -> 4Hz，群延迟 ~2.8ms -> ~56ms；
 *   - ICM42688 的抗混叠滤波器配在 213Hz（匹配 1kHz 采样），50Hz 下 Nyquist 只有
 *     25Hz，桨叶 150~300Hz 振动会整个折叠进姿态信号带且无法分辨。
 *
 * 因此这里的判据不是"提示性能偏低"，而是"拒绝起飞"。
 */

typedef enum {
    APP_IMU_HEALTH_NORMAL   = 0,  /* 采样率达标且基本由中断驱动 */
    APP_IMU_HEALTH_DEGRADED = 1,  /* 有轮询兜底或速率偏低，但仍可用 */
    APP_IMU_HEALTH_FAILED   = 2   /* 速率已跌破控制律前提 */
} APP_ImuHealthLevel;

typedef struct {
    APP_ImuHealthLevel level;
    uint8_t  fault_active;        /* 当前是否处于失效锁存（阻止解锁） */
    uint8_t  fault_ever;          /* 本次上电是否曾经失效（证据，不自动清除） */
    uint16_t sample_rate_hz;      /* 最近一个评估窗口的实际采样率 */
    uint16_t irq_rate_hz;         /* 最近一个评估窗口的 DRDY 中断速率 */
    uint32_t irq_samples;         /* 累计：由中断唤醒取到的帧 */
    uint32_t poll_samples;        /* 累计：由轮询兜底取到的帧 */
    uint32_t degraded_windows;    /* 累计：判为降级的窗口数 */
    uint32_t failed_windows;      /* 累计：判为失效的窗口数 */
} APP_ImuHealthStatus;

/* 评估窗口与判据。窗口取 100ms：足够统计 1kHz 的速率，又能在 200ms 内定性。 */
#define APP_IMU_HEALTH_WINDOW_MS        100U
#define APP_IMU_HEALTH_RATE_NORMAL_HZ   900U
#define APP_IMU_HEALTH_RATE_FAIL_HZ     500U
/* 连续 2 个窗口（200ms）低于 FAIL 速率才锁存，避免单次调度抖动误判。 */
#define APP_IMU_HEALTH_FAIL_WINDOWS     2U
/* 连续 20 个窗口（2s）完全正常才解除锁存。 */
#define APP_IMU_HEALTH_RECOVER_WINDOWS  20U
/* 轮询占比超过这个百分比即判降级（正常时应当为 0）。 */
#define APP_IMU_HEALTH_POLL_WARN_PCT    5U

void APP_ImuHealth_Init(void);

/*
 * 每取到一帧 IMU 数据调用一次。from_irq=1 表示这帧由 DRDY 中断唤醒，
 * 0 表示是 20ms 超时后轮询兜底拿到的。
 *
 * 关键：调用方绝不能把"轮询成功"当成"中断健康"——那正是这套监测要修的 bug。
 */
void APP_ImuHealth_NoteSample(uint8_t from_irq);

/* 每帧调用；窗口未满时立即返回，热路径开销可忽略。 */
void APP_ImuHealth_Update(uint32_t now_ms);

APP_ImuHealthLevel APP_ImuHealth_GetLevel(void);

/* 非零表示必须禁止解锁。 */
uint8_t APP_ImuHealth_IsArmBlocked(void);

void APP_ImuHealth_GetStatus(APP_ImuHealthStatus *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_HEALTH_H */
