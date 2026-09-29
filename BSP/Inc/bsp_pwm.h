#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_PWM_OK = 0,
    BSP_PWM_ERROR,
    BSP_PWM_INVALID_PARAM,
    BSP_PWM_BUSY
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

/* MicoAir743v2：ESC=M4/M3，舵机=M7/M8。
 * ESC_PROTOCOL 在构建时选择；默认 DShot300，实际帧由 500 Hz 仲裁点提交。
 * 下列 ESC 帧率/tick 常量只描述普通 PWM 回退后端，不能用来计算 DShot 位时序。
 * SetEscPulse/GetEscPulse 在 DShot 模式下是等效微秒命令，不是引脚实际脉宽。
 * 0 表示物理禁用；1100 表示发送零油门码。两路 setter 暂存后统一 CommitEsc。
 * 舵机由独立 TIM4 输出，上电 333 Hz（YM-3245 数字舵机；2026-09-28 杆上 A/B：带载纯延迟 30→22 ms、
 * 闭环滞后 46→38 ms、无额外抖动）；上锁时可经 `SERVOHZ` 在 50–333 Hz 间切换（只在 RAM），
 * 换模拟舵机须改回 50。校准与极性沿用既有配置。
 */
#define BSP_PWM_TIMER_TICK_HZ  1000000U   /* Prescaler=120-1，脉宽直接按 us 写 CCR */
#define BSP_PWM_ESC_FRAME_HZ       400U   /* TIM1_CH1/CH2 -> PE9/PE11  (MOTOR4/3) */
#define BSP_PWM_SERVO_FRAME_HZ     333U   /* TIM4_CH1/CH2 -> PD12/PD13 (MOTOR7/8)；2026-09-28 起开机 333 Hz */
#define BSP_PWM_ESC_FRAME_US   (BSP_PWM_TIMER_TICK_HZ / BSP_PWM_ESC_FRAME_HZ)
#define BSP_PWM_SERVO_FRAME_US (BSP_PWM_TIMER_TICK_HZ / BSP_PWM_SERVO_FRAME_HZ)
#define BSP_PWM_SERVO_FRAME_HZ_MIN  50U   /* 模拟舵机的标准帧率 */
#define BSP_PWM_SERVO_FRAME_HZ_MAX 333U   /* 数字舵机常见上限；3003 us 帧仍装得下 2500 us 脉冲 */

BSP_PWM_Status BSP_PWM_Init(void);
uint8_t BSP_PWM_IsInitialized(void);
BSP_PWM_Status BSP_PWM_SetEscPulse(uint32_t channel, uint16_t pulse_us);
BSP_PWM_Status BSP_PWM_SetEscPercent(uint32_t channel, uint32_t percent);
BSP_PWM_Status BSP_PWM_DisableEsc(uint32_t channel);
/* DShot setters stage equivalent-us commands; one Commit after final arbitration
 * submits both channels. PWM Commit is a no-op. Disable is always immediate.
 */
BSP_PWM_Status BSP_PWM_CommitEsc(void);
/*
 * 这一拍改发 DShot 特殊命令（两路同发）。与 CommitEsc 互斥：发了命令就没发油门，
 * 暂存的油门不动，下一拍照常提交。PWM 档没有命令这回事，返回 ERROR 而不是默默成功
 * ——默默成功会让上位机显示"已发送"。连发几帧的策略归 App/app_esc_command.c。
 */
BSP_PWM_Status BSP_PWM_CommitEscCommand(uint16_t command);
const char *BSP_PWM_EscProtocol(void);
/*
 * 这一路 ESC 接在板上哪个焊盘。做成 BSP 查询而不是让上位机自己写死一张
 * "通道1=M4"的表：焊盘归属是板级事实，换板只改这里一处。桨叶旋向标定
 * （`PROPCAL`）要靠这行字告诉操作者该盯着哪个电机看，抄错一份表的后果是
 * 把标定结果整个填反，而两边各自都说得通。通道非法返回 "-"。
 */
const char *BSP_PWM_EscChannelLabel(uint32_t channel);
BSP_PWM_Status BSP_PWM_SetServoPulse(uint32_t channel, uint16_t pulse_us);
/*
 * 舵机 PWM 帧率 [Hz]，[MIN, MAX] 之外返回 INVALID_PARAM，未初始化返回 ERROR。
 * 新帧长经 ARR 预装载从下一个更新事件起生效：不会出现计数值已越过新 ARR、要绕满 16 位
 * （约 65 ms 无脉冲）才回卷的长帧。调用方负责只在舵机不承担飞行控制时切换（上锁）。
 */
BSP_PWM_Status BSP_PWM_SetServoFrameHz(uint32_t frame_hz);
uint32_t BSP_PWM_GetServoFrameHz(void);
uint16_t BSP_PWM_PercentToPulse(uint32_t percent);
uint16_t BSP_PWM_GetEscPulse(uint32_t channel);
/* Atomic pair snapshot of the final staged command. This is command readback,
 * not proof that the ESC or motor physically executed it. */
void BSP_PWM_GetEscPulses(uint16_t out[BSP_PWM_ESC_CHANNEL_COUNT]);
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
