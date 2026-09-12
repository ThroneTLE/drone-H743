#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_PWM_OK = 0,
    BSP_PWM_ERROR,
    BSP_PWM_INVALID_PARAM
} BSP_PWM_Status;

#define BSP_PWM_ESC_MIN_US      1100U
#define BSP_PWM_ESC_MAX_US      1940U
#define BSP_PWM_ESC_NEUTRAL_US  1100U
#define BSP_PWM_ESC_MAX_PERCENT 100U
#define BSP_PWM_TIM_CHANNEL_COUNT 4U
#define BSP_PWM_ESC_CHANNEL_COUNT 2U

#define BSP_PWM_SERVO_CHANNEL_COUNT 2U
#define BSP_PWM_SERVO_MIN_US       500U
#define BSP_PWM_SERVO_MAX_US      2500U
#define BSP_PWM_SERVO_CENTER_US   1500U

/*
 * 两路 ESC 与两路舵机的帧率。
 *
 * 2026-09-07 之前四路都挂在 TIM2 上，被舵机的 50 Hz 拖着一起走：角速率环 500 Hz
 * 算 10 次只送得出去 1 次，零阶保持平均延迟 10 ms。这段死区是当时整个控制回路
 * 的瓶颈——它比 IMU、比 CPU、比算法都慢一个量级，也是任何非零 rate.kd 都发散的
 * 直接原因（推导见 drv_coax_ctrl.c 中 alpha_lpf_cutoff_rad_s 默认值上方）。
 *
 * 现在 ESC 走 TIM5_CH1/CH2、舵机留在 TIM2_CH3/CH4。**引脚一根没动**：PA0~PA3
 * 都同时具备 TIM2(AF1) 与 TIM5(AF2)，换的只是片内复用，外部接线与设备不变。
 * 于是两者可以各自定帧率：ESC 400 Hz（ZOH 平均延迟 10 ms → 1.25 ms），舵机维持
 * 50 Hz —— 舵机自身机械带宽只有 5~10 Hz，提帧率收益有限，且模拟舵机不一定接受
 * 高帧率，所以先不动，等确认型号是数字舵机再考虑提到 250~333 Hz。
 *
 * 帧率由 BSP 拥有：BSP_PWM_Init() 会按这两个常数重设 ARR，CubeMX 里的 Period
 * 只是初值。这样一次 CubeMX 重新生成不会把帧率悄悄改回去。
 */
/*
 * 2026-09-10 移植到 MicoAir743v2：PA0~PA3 在这块板上是 UART4 与 USART2，
 * 四路 PWM 全部搬到电机焊盘。分两个定时器的理由完全不变（各自定帧率），
 * 变的只是挂哪个定时器和哪几个脚。
 */
#define BSP_PWM_TIMER_TICK_HZ  1000000U   /* Prescaler=120-1，脉宽直接按 us 写 CCR */
#define BSP_PWM_ESC_FRAME_HZ       400U   /* TIM1_CH1/CH2 -> PE9/PE11  (MOTOR4/3) */
#define BSP_PWM_SERVO_FRAME_HZ      50U   /* TIM4_CH1/CH2 -> PD12/PD13 (MOTOR7/8) */
#define BSP_PWM_ESC_FRAME_US   (BSP_PWM_TIMER_TICK_HZ / BSP_PWM_ESC_FRAME_HZ)
#define BSP_PWM_SERVO_FRAME_US (BSP_PWM_TIMER_TICK_HZ / BSP_PWM_SERVO_FRAME_HZ)

BSP_PWM_Status BSP_PWM_Init(void);
BSP_PWM_Status BSP_PWM_SetEscPulse(uint32_t channel, uint16_t pulse_us);
BSP_PWM_Status BSP_PWM_SetEscPercent(uint32_t channel, uint32_t percent);
BSP_PWM_Status BSP_PWM_DisableEsc(uint32_t channel);
BSP_PWM_Status BSP_PWM_SetServoPulse(uint32_t channel, uint16_t pulse_us);
uint16_t BSP_PWM_PercentToPulse(uint32_t percent);
uint16_t BSP_PWM_GetEscPulse(uint32_t channel);
uint16_t BSP_PWM_GetServoPulse(uint32_t channel);
uint8_t BSP_PWM_GetStartStatus(uint32_t channel);

/*
 * 一个定时器的寄存器快照，给 `PWM?` 诊断用。
 *
 * 由 BSP 提供而不是让 App 自己读寄存器，理由是这条诊断已经因此错过一次：
 * 移植前四路 PWM 在 TIM2，`app_control_report_pwm()` 就直接写了 `TIM2->CR1`；
 * 搬到 TIM1/TIM4 之后那段代码没人改，于是 `PWM?` 一直在报一颗**本板没有初始化**
 * 的定时器，实测输出 `cr1=0x00000000 psc=0 arr=0`——查"PWM 没输出"的人看到这行
 * 会认定定时器没配好，而真正的 TIM1/TIM4 其实好好的。诊断说谎比诊断缺失更糟。
 *
 * 现在定时器归属只有 bsp_pwm.c 一处知道，换板改那一处，`PWM?` 自动跟着对。
 */
typedef struct {
    const char *name;     /* "tim1" / "tim4"，随绑定走，不写死在上层 */
    uint32_t cr1;
    uint32_t ccer;
    uint32_t ccmr1;
    uint32_t ccmr2;
    uint32_t psc;
    uint32_t arr;
    uint32_t cnt;
    uint32_t ccr[BSP_PWM_TIM_CHANNEL_COUNT];
} BSP_PWM_TimerDebug;

/* ESC（高帧率）与舵机（低帧率）各自那颗定时器的快照。 */
void BSP_PWM_GetEscTimerDebug(BSP_PWM_TimerDebug *out);
void BSP_PWM_GetServoTimerDebug(BSP_PWM_TimerDebug *out);

#ifdef __cplusplus
}
#endif

#endif
