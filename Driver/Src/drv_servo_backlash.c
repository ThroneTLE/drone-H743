/*
 * 舵机回差逆补偿。全部是纯算术；能在 PC 上判对错，因此不允许出现任何硬件依赖。
 * 设计理由（换向跳 2b、迟滞阈值）与信号契约见 Driver/Inc/drv_servo_backlash.h。
 */
#include "drv_servo_backlash.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

void DRV_ServoBacklash_DefaultConfig(DRV_ServoBacklashConfig *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->half_gap_rad = 0.0f;
    cfg->threshold_rad = DRV_SERVO_BACKLASH_THRESHOLD_DEFAULT_RAD;
    cfg->enable = 0U;
}

uint8_t DRV_ServoBacklash_ConfigValid(const DRV_ServoBacklashConfig *cfg)
{
    if (cfg == NULL) {
        return 0U;
    }
    /* 写成"在范围内"的正向判断：NaN 让每个比较都为假，自然落到不合法。 */
    if (!((cfg->half_gap_rad >= 0.0f) && (cfg->half_gap_rad <= DRV_SERVO_BACKLASH_HALF_GAP_MAX_RAD))) {
        return 0U;
    }
    if (!((cfg->threshold_rad > 0.0f) && (cfg->threshold_rad <= DRV_SERVO_BACKLASH_THRESHOLD_MAX_RAD))) {
        return 0U;
    }
    return (cfg->enable <= 1U) ? 1U : 0U;
}

uint8_t DRV_ServoBacklash_Init(DRV_ServoBacklash *bl, const DRV_ServoBacklashConfig *cfg)
{
    uint8_t valid;

    if (bl == NULL) {
        return 0U;
    }
    memset(bl, 0, sizeof(*bl));
    valid = DRV_ServoBacklash_ConfigValid(cfg);
    if (valid != 0U) {
        bl->cfg = *cfg;
    } else {
        DRV_ServoBacklash_DefaultConfig(&bl->cfg);
    }
    return valid;
}

void DRV_ServoBacklash_Reset(DRV_ServoBacklash *bl)
{
    if (bl == NULL) {
        return;
    }
    bl->direction = 0;
    bl->primed = 0U;
    bl->reference_rad = 0.0f;
}

float DRV_ServoBacklash_Step(DRV_ServoBacklash *bl, float cmd_rad)
{
    float thr;

    if (bl == NULL) {
        return cmd_rad;
    }
    thr = bl->cfg.threshold_rad;
    if (bl->cfg.enable == 0U) {
        DRV_ServoBacklash_Reset(bl);   /* 关着不留方向：再打开时从头定向 */
        return cmd_rad;
    }
    if (isfinite(cmd_rad) == 0) {
        bl->nonfinite_count++;
        DRV_ServoBacklash_Reset(bl);
        return cmd_rad;
    }
    if (bl->primed == 0U) {
        bl->primed = 1U;
        bl->reference_rad = cmd_rad;
        return cmd_rad;               /* 复位后第一个样本：不知道负载贴在哪一边，不加偏置 */
    }
    if (bl->direction == 0) {
        if (cmd_rad > (bl->reference_rad + thr)) {
            bl->direction = 1;
            bl->reference_rad = cmd_rad;
        } else if (cmd_rad < (bl->reference_rad - thr)) {
            bl->direction = -1;
            bl->reference_rad = cmd_rad;
        } else {
            return cmd_rad;           /* 还在起始迟滞带里：原样输出 */
        }
    } else if (bl->direction > 0) {
        if (cmd_rad > bl->reference_rad) {
            bl->reference_rad = cmd_rad;
        } else if (cmd_rad < (bl->reference_rad - thr)) {
            bl->direction = -1;
            bl->reference_rad = cmd_rad;
            bl->reversal_count++;
        }
    } else {
        if (cmd_rad < bl->reference_rad) {
            bl->reference_rad = cmd_rad;
        } else if (cmd_rad > (bl->reference_rad + thr)) {
            bl->direction = 1;
            bl->reference_rad = cmd_rad;
            bl->reversal_count++;
        }
    }
    return (bl->direction > 0) ? (cmd_rad + bl->cfg.half_gap_rad) : (cmd_rad - bl->cfg.half_gap_rad);
}
