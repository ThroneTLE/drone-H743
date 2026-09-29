/*
 * app_cmd_backlash.c —— `BACKLASH` 命令族与辨识溯源行。策略与状态在 app_servo_backlash.c。
 *
 *   BACKLASH | BACKLASH ?                 2 行状态（cfg / count）
 *   BACKLASH ON | OFF                     A/B 开关
 *   BACKLASH SET ALPHA|BETA <0..87>       回差半宽 [mrad]（横滚 alpha = PWM ch1，俯仰 beta = PWM ch2）
 *   BACKLASH SET THR <1..50>              换向迟滞 [mrad]
 *
 * ON/OFF/SET 只在未解锁、且辨识没占着台架时收（照 RPMNOTCH 的先例）：飞行中切换等于
 * 在回路里换一段执行器特性；辨识一轮跑到一半换，那一轮的溯源（SYSID BACKLASH 行）就
 * 不再代表整轮。阶段一只在 RAM，不写 Flash。
 *
 * 大小写敏感，同 RPMNOTCH。printf 只用整数（newlib-nano 不带 %f）。
 * 挂在 app_cmd_fallback.c 的兜底链上，不碰只减不增的 app_control.c。
 */

#include "app_servo_backlash.h"

#include "app_control.h"
#include "app_control_internal.h"
#include "app_stabilizer.h"
#include "app_sysid.h"
#include "bsp_pwm.h"

#include <stddef.h>
#include <string.h>

#define BACKLASH_USAGE "BACKLASH ?|ON|OFF|SET ALPHA|BETA|THR <mrad>"

static void backlash_report_status(void)
{
    APP_ServoBacklashStatus s;

    APP_ServoBacklash_GetStatus(&s);
    APP_Control_QueueText(
        "BACKLASH cfg en=%u alpha_mrad=%u beta_mrad=%u thr_mrad=%u active=%u src=%s "
        "dir_alpha=%d dir_beta=%d off_alpha_us=%d off_beta_us=%d\r\n",
        (unsigned int)s.cfg.enable, (unsigned int)s.cfg.half_mrad[APP_SERVO_BACKLASH_ALPHA],
        (unsigned int)s.cfg.half_mrad[APP_SERVO_BACKLASH_BETA], (unsigned int)s.cfg.thr_mrad,
        (unsigned int)s.active, APP_ServoBacklash_SourceName(s.source),
        (int)s.direction[APP_SERVO_BACKLASH_ALPHA], (int)s.direction[APP_SERVO_BACKLASH_BETA],
        (int)s.offset_us[APP_SERVO_BACKLASH_ALPHA], (int)s.offset_us[APP_SERVO_BACKLASH_BETA]);
    APP_Control_QueueText(
        "BACKLASH count rev_alpha=%lu rev_beta=%lu reset=%lu nonfinite=%lu ticks=%lu clamped=%lu\r\n",
        (unsigned long)s.reversals[APP_SERVO_BACKLASH_ALPHA],
        (unsigned long)s.reversals[APP_SERVO_BACKLASH_BETA], (unsigned long)s.resets,
        (unsigned long)s.nonfinite, (unsigned long)s.ticks, (unsigned long)s.clamped);
}

static uint8_t backlash_reject(const char *reason)
{
    APP_Control_QueueText("BACKLASH event=rejected reason=%s\r\n", reason);
    return 1U;
}

static uint8_t backlash_usage(void)
{
    APP_Control_QueueText("BACKLASH event=rejected reason=usage usage=" BACKLASH_USAGE "\r\n");
    return 1U;
}

static uint8_t backlash_range(const char *key)
{
    APP_Control_QueueText("BACKLASH event=rejected reason=range key=%s\r\n", key);
    return 1U;
}

/* SET <key> <mrad>：解析失败回 usage，范围不对回 range。返回 1 = 已回复（拒绝），0 = cfg 已改好。 */
static uint8_t backlash_apply_set(const char *key, const char *text, APP_ServoBacklashConfig *cfg)
{
    uint32_t mrad = 0U;

    if ((strcmp(key, "ALPHA") != 0) && (strcmp(key, "BETA") != 0) && (strcmp(key, "THR") != 0)) {
        return backlash_usage();
    }
    if (app_control_parse_u32(text, &mrad) == 0U) {
        return backlash_usage();
    }
    if (strcmp(key, "THR") == 0) {
        if ((mrad < APP_SERVO_BACKLASH_THR_MRAD_MIN) || (mrad > APP_SERVO_BACKLASH_THR_MRAD_MAX)) {
            return backlash_range(key);
        }
        cfg->thr_mrad = (uint16_t)mrad;
        return 0U;
    }
    if (mrad > APP_SERVO_BACKLASH_HALF_MRAD_MAX) {
        return backlash_range(key);
    }
    cfg->half_mrad[(strcmp(key, "ALPHA") == 0) ? APP_SERVO_BACKLASH_ALPHA : APP_SERVO_BACKLASH_BETA] =
        (uint16_t)mrad;
    return 0U;
}

uint8_t APP_ServoBacklash_Command(char **tokens, uint32_t count)
{
    APP_ServoBacklashConfig cfg;
    const char *sub;
    uint8_t change;

    if ((tokens == NULL) || (count == 0U) || (strcmp(tokens[0], "BACKLASH") != 0)) {
        return 0U;
    }
    if ((count == 1U) || ((count == 2U) && (strcmp(tokens[1], "?") == 0))) {
        backlash_report_status();
        return 1U;
    }
    sub = tokens[1];
    change = (((count == 2U) && ((strcmp(sub, "ON") == 0) || (strcmp(sub, "OFF") == 0))) ||
              ((count == 4U) && (strcmp(sub, "SET") == 0))) ? 1U : 0U;
    if (change == 0U) {
        return backlash_usage();
    }
    if (APP_Stabilizer_IsArmed() != 0U) {
        return backlash_reject("armed");
    }
    if (APP_SysId_IsEngaged() != 0U) {
        return backlash_reject("sysid");
    }
    APP_ServoBacklash_GetConfig(&cfg);
    if (strcmp(sub, "ON") == 0) {
        cfg.enable = 1U;
    } else if (strcmp(sub, "OFF") == 0) {
        cfg.enable = 0U;
    } else if (backlash_apply_set(tokens[2], tokens[3], &cfg) != 0U) {
        return 1U;
    }
    if (APP_ServoBacklash_RequestConfig(&cfg) == 0U) {
        return backlash_range("cfg");   /* 单项都查过，走不到 */
    }
    backlash_report_status();
    return 1U;
}

void APP_ServoBacklash_ReportProvenance(uint16_t run_id)
{
    APP_ServoBacklashStatus s;

    APP_ServoBacklash_GetStatus(&s);
    APP_Control_QueueText(
        "SYSID BACKLASH run=%u en=%u alpha_mrad=%u beta_mrad=%u thr_mrad=%u servo_hz=%lu\r\n",
        (unsigned int)run_id, (unsigned int)s.cfg.enable,
        (unsigned int)s.cfg.half_mrad[APP_SERVO_BACKLASH_ALPHA],
        (unsigned int)s.cfg.half_mrad[APP_SERVO_BACKLASH_BETA], (unsigned int)s.cfg.thr_mrad,
        (unsigned long)BSP_PWM_GetServoFrameHz());   /* 舵机帧率决定等帧延迟（SERVOHZ） */
}
