/*
 * `HOVER?` / `HOVER RESET` —— 悬停推力在线估计的读取与重置。
 *
 * 估计器不改 coax.hover_thrust_n；学到的值经 app_hover_adapt.c 接进控制器（`HOVER ADAPT ON|OFF`，见 app_hover_adapt.h）。
 *
 *   HOVER?      回一行，字段：
 *     est_n      当前估计的悬停推力 [N]（推力查补表口径，与 coax.hover_thrust_n 同口径）
 *     std_n      估计标准差 [N]（卡尔曼 P 的平方根；样本相关会让它偏乐观）
 *     converged  1 = 已接收样本 ≥ 500 且 std_n ≤ 0.3
 *     learning   1 = 最近一拍六个学习条件全满足
 *     samples    累计接收进滤波器的样本数；rejected 是被新息门限拒收的
 *     innov      最近一个被评估样本的新息 [m/s²]
 *     init_n     初值（起步或 RESET 时的 coax.hover_thrust_n，否则 m·g）
 *     meas_std   自适应测量噪声 σ [m/s²]
 *     gate       六个学习条件的位掩码（0x01 解锁 0x02 有推力 0x04 未封顶 0x08 倾角<20°
 *                0x10 加速度有效 0x20 已离开支撑面）；0x3f = 全满足
 *     airborne   1 = 已判为离开支撑面；above_m 是测距相对支撑面的高度 [m]
 *   HOVER RESET 把估计回到初值（下一拍生效，按那时的 coax.hover_thrust_n 重新取初值）。
 *               地面站 AI 接口只放行以 ? 结尾的查询，会拒绝它；作者在终端手发。
 *
 * 分发挂在 app_cmd_fallback.c 的兜底链上，不碰 app_control.c。
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "app_hover_adapt.h"
#include "app_hover_thrust.h"
#include "drv_coax_ctrl.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* newlib-nano 没有浮点 printf（%f 打出来是空的：2026-09-30 首次上板 HOVER? 各数字段全空），
 * 按千分之一取整拼十进制。 */
static void hover_fmt(char *buf, size_t size, float value)
{
    long milli;

    if (isfinite(value) == 0) {
        (void)snprintf(buf, size, "nan");
        return;
    }
    milli = lroundf(value * 1000.0f);
    (void)snprintf(buf, size, "%s%ld.%03ld", (milli < 0L) ? "-" : "",
                   labs(milli) / 1000L, labs(milli) % 1000L);
}

uint8_t app_control_handle_hover(char **tokens, uint32_t count)
{
    APP_HoverThrustSnapshot snap;
    char est[16], std[16], innov[16], init[16], meas[16], above[16], applied[16], cfg[16];

    if ((tokens == NULL) || (count == 0U)) {
        return 0U;
    }
    if ((count == 1U) && (strcmp(tokens[0], "HOVER?") == 0)) {
        if (APP_HoverThrust_GetSnapshot(&snap) == 0U) {
            APP_Control_QueueText("HOVER state=not_ready\r\n");
            return 1U;
        }
        hover_fmt(est, sizeof(est), snap.est.est_n);
        hover_fmt(std, sizeof(std), snap.est.std_n);
        hover_fmt(innov, sizeof(innov), snap.est.innov_m_s2);
        hover_fmt(init, sizeof(init), snap.est.init_n);
        hover_fmt(meas, sizeof(meas), snap.est.meas_std_m_s2);
        hover_fmt(above, sizeof(above), snap.above_ground_m);
        hover_fmt(applied, sizeof(applied), APP_HoverAdapt_GetApplied());
        hover_fmt(cfg, sizeof(cfg), DRV_COAX_CTRL_ConfiguredHoverThrustN());
        /* 末尾三项是自适应接入（app_hover_adapt.h）：applied_n 是此刻交给控制器的悬停推力。
         * 追加在同一行而不是另起一行——地面站 XY 页把收到的第一条 `HOVER ` 行当结果解析。 */
        APP_Control_QueueText(
            "HOVER est_n=%s std_n=%s converged=%u learning=%u samples=%lu rejected=%lu "
            "innov=%s init_n=%s meas_std=%s gate=0x%02x airborne=%u above_m=%s "
            "adapt=%s applied_n=%s cfg_n=%s\r\n",
            est, std, (unsigned int)snap.est.converged, (unsigned int)snap.est.learning,
            (unsigned long)snap.est.samples, (unsigned long)snap.est.rejected,
            innov, init, meas, (unsigned int)snap.gate_mask,
            (unsigned int)snap.airborne, above,
            (APP_HoverAdapt_IsEnabled() != 0U) ? "on" : "off", applied, cfg);
        return 1U;
    }
    if ((count == 2U) && (strcmp(tokens[0], "HOVER") == 0) &&
        (strcmp(tokens[1], "RESET") == 0)) {
        APP_HoverThrust_RequestReset();
        APP_Control_QueueText("HOVER reset=requested\r\n");
        return 1U;
    }
    if ((count == 3U) && (strcmp(tokens[0], "HOVER") == 0) &&
        (strcmp(tokens[1], "ADAPT") == 0)) {
        /* 学到的悬停推力是否接进控制器（A/B 用，只在 RAM，上电 ON）。 */
        if (strcmp(tokens[2], "ON") == 0) {
            APP_HoverAdapt_SetEnabled(1U);
        } else if (strcmp(tokens[2], "OFF") == 0) {
            APP_HoverAdapt_SetEnabled(0U);
        } else {
            APP_Control_QueueText("ERR usage HOVER ADAPT ON|OFF\r\n");
            return 1U;
        }
        APP_Control_QueueText("HOVER adapt=%s\r\n",
                              (APP_HoverAdapt_IsEnabled() != 0U) ? "on" : "off");
        return 1U;
    }
    return 0U;
}
