/*
 * 悬停推力自适应的"何时用"策略层（算术在 Driver/Inc/drv_hover_adapt.h，学习器在 app_hover_thrust.c）。
 *
 * 作者 2026-10-01："让自适应学习出现效果，然后可以配合积分适当补偿"，并确认：
 * "收敛之后才用；限制在初值上下 15% 以内，慢慢过渡；每次解锁重新学"。
 *
 *   - 每次解锁（上升沿）：学习器重置、已用值回到配置值（地面上积分本来就清零，无冲击）。
 *     学习器真正重置之前（快照 samples ≠ 0）一律按配置值，不拿上一次的收敛结果。
 *   - 上锁：已用值 = 配置值（THRMODE? 的 hover_mn 显示配置值）。
 *   - 辨识/推力台/旋向/舵机标定在跑：保持当前已用值不动。
 *   - 其余（已解锁）：目标 = DRV_HoverAdapt_Target，已用值按 DRV_HoverAdapt_Slew 过渡，
 *     交给 DRV_COAX_CTRL_SetHoverThrustAdapt（它做竖直积分无扰补偿）。
 *   - `HOVER ADAPT OFF`：目标恒为配置值（同样慢慢过渡回去），供 A/B；上电默认 ON，只在 RAM。
 */
#ifndef APP_HOVER_ADAPT_H
#define APP_HOVER_ADAPT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    dt_s;
    uint8_t  armed;
    uint8_t  bypass;             /* 辨识/推力台/旋向/舵机标定 */
    uint8_t  enabled;
    uint8_t  converged;          /* 学习器已收敛 */
    uint32_t samples;            /* 学习器累计接收样本（0 = 刚重置） */
    float    est_n;
    float    cfg_n;              /* 配置值（coax.hover_thrust_n，未设则 m·g） */
} APP_HoverAdaptInput;

typedef struct {
    float    applied_n;          /* 当前交给控制器的悬停推力（0 = 未起步） */
    uint8_t  armed_prev;
    uint8_t  wait_reset;         /* 已请求学习器重置、还没看到 samples 归零 */
} APP_HoverAdaptState;

#define APP_HOVER_ADAPT_ACTION_NONE          0U
#define APP_HOVER_ADAPT_ACTION_RESET_LEARNER 1U

/* 纯判定（不调外部函数），宿主可测。返回 APP_HOVER_ADAPT_ACTION_*，已用值写回 state->applied_n。 */
uint8_t APP_HoverAdapt_Step(APP_HoverAdaptState *state, const APP_HoverAdaptInput *in);

/* 控制任务每拍调用（在学习器之后、下一拍控制之前）。 */
void APP_HoverAdapt_Update(float dt_s, uint8_t armed, uint8_t bypass);

void APP_HoverAdapt_SetEnabled(uint8_t enabled);
uint8_t APP_HoverAdapt_IsEnabled(void);
float APP_HoverAdapt_GetApplied(void);

#ifdef __cplusplus
}
#endif

#endif
