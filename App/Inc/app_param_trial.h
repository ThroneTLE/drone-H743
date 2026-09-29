#ifndef APP_PARAM_TRIAL_H
#define APP_PARAM_TRIAL_H

/*
 * 控制增益"试用"记录：`SYSID PARAM` 写的值只活在 RAM 里，任何一次存 Flash 都换回试用前的值。
 *
 * 为什么要有它（2026-09-27 实机）：`SYSID PARAM` 本身不排自动保存，但 Flash 保存存的是
 * **整份**配置。杆上"临时应用 50%"之后发了一条普通 `PARAM SET` 改积分限幅，1.5 s 后的
 * 自动保存把 RAM 里的 50% 试用增益一起永久化了。只要试用值还在 RAM，任何保存路径
 * （自动保存、SAVE、RCMAP/LEDMAP/PROPCAL/MAGCAL 的 COMMIT）都会顺手把它带进去。
 *
 * 契约：
 *   1. 某名字第一次 `SYSID PARAM` 时记下它试用前的值（当时 RAM 里的值——没有试用时，
 *      保存存的就是它，所以它就是"正式配置"）。同名再试用不覆盖这份记录。
 *   2. 保存时（APP_ControlConfigStore_Save 从 RAM 捕获增益块与 v24 指令整形/出口陷波块之后）
 *      有记录的名字一律换成记录值；RAM 不动，控制器照旧用试用值。
 *   3. 同名普通 `PARAM SET`（显式持久写）= 定下来：清掉这条记录，之后按 RAM 值存。
 *   4. `SYSID PARAM` 写回的值等于记录值（容差 1e-6·max(1,|x|)，即绝对/相对 1e-6）时清掉记录
 *      ——上位机"恢复原参数"就是这样发的。
 *   5. 只在 RAM：重启自然清空；`LOAD`（RAM 整份换回 Flash）与 `DEFAULTS`（整表显式重写）
 *      清空全部。
 *   6. `SYSID PARAM` / `SYSID PARAM ?` 只读查询：
 *          SYSID TRIAL n=<count>
 *          SYSID TRIAL name=<n> ram=<v> saved=<v>      （每条一行，六位小数）
 *   7. 容量 = 可试用名单长度（驱动具名表里全部 coax.rate_* / coax.att_*，含 v24 的
 *      coax.rate_out_notch_* / coax.att_ref_* 与 v25 的 coax.rate_out_notch2_*；外加高度环
 *      z 通道 coax.pos_z_kp / coax.vel_z_kp / _ki / _kd / _i_limit_m_s2），每个都放得下；
 *      真满了就拒绝新的试用（ERR，不写 RAM），绝不静默丢记录。
 */

#include "app_control_config_compat.h"

#include <stdint.h>

typedef enum {
    APP_PARAM_TRIAL_OK = 0,
    APP_PARAM_TRIAL_NOT_TRIALABLE,   /* 不在可试用名单里：保存时认不出它，就还原不了 */
    APP_PARAM_TRIAL_FULL,            /* 记录表满：拒绝，RAM 未改 */
    APP_PARAM_TRIAL_REJECTED         /* 控制器拒收这个值（非有限/越界），RAM 未改 */
} APP_ParamTrialStatus;

/* 写 RAM 并维护记录（契约 1、4）。成功时 *applied 为控制器实际采用的值。 */
APP_ParamTrialStatus APP_ParamTrial_Apply(const char *name, float value, float *applied);
/* 显式持久写（PARAM SET 成功之后）调用：该名字的试用结束（契约 3）。不认识的名字什么都不做。 */
void APP_ParamTrial_Clear(const char *name);
/* LOAD / DEFAULTS：全部试用结束（契约 5）。 */
void APP_ParamTrial_ClearAll(void);
uint32_t APP_ParamTrial_Count(void);
/* 保存路径：把刚从 RAM 捕获的两块里、有记录的字段换回记录值（契约 2）。NULL 的块跳过。 */
void APP_ParamTrial_RestorePersistent(APP_ControlCoaxTunableParams *gains,
                                      APP_ControlCoaxShapingParams *shaping);
/* `SYSID PARAM ?` 的回报（契约 6）。 */
void APP_ParamTrial_Report(void);

#endif /* APP_PARAM_TRIAL_H */
