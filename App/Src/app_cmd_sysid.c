/*
 * app_cmd_sysid.c —— `SYSID` 命令族，外加从 `app_control.c` 搬出来的 `IDENT` 那一组。
 *
 * ──────────────── 为什么 IDENT 在这里 ────────────────
 *
 * `App/Src/app_control.c` 是**只减不增**的文件。`IDENT` 的姿态/舵机子命令
 * （?/ARM/DISARM/STOP/ATT/CENTER/APPLY/STEP/DOUBLET/PRBS）只调 `APP_Ident_*`
 * 的公开接口，和那个文件的静态状态没有任何牵连，所以整组搬过来，净减约 215 行。
 *
 * **`IDENT START` 没搬**：那条是电机阶梯测试，读写 app_control.c 的 `ident_*`
 * 静态量、并由它自己的服务拍推进（`app_control_ident_step`）。搬它要连那个服务
 * 拍一起搬，是另一件事，混进来只会让这次改动难以复核。
 *
 * ──────────────── SYSID 的命令面 ────────────────
 *
 *   SYSID              / SYSID ?        状态
 *   SYSID SCHEMA                        字段表（主机照它解码，不在主机侧写死布局）
 *   SYSID RIG [psi_deg= axis_off_m= imu_off_m=]
 *   SYSID EXC [profile= amp= dur_ms= hold_ms= repeat= ramp_ms= f0= f1= bit_ms= seed=
 *              servo_tilt_mrad=]          servo_tilt_mrad 只用于 SERVO 模式（10..262）
 *   SYSID MODE FF|RATE|ANGLE|SERVO|ALT|XY|YAW [deg]  也认 0..6；SERVO = 电机不转、只动舵机；
 *                                       ALT = 光杆台架高度辨识（app_sysid_alt.h）；
 *                                       XY = 水平槽速度/位置辨识（app_sysid_xy.h）；
 *                                       YAW = 吊绳偏航辨识（app_sysid_yaw.h）
 *   SYSID ALT [inject=break|vel|pos mass_g= win_mm= lift_mm= bottom_mm= top_mm=]
 *             高度辨识配置，只回一行
 *   SYSID XY [inject=tilt|vel|pos win_mm= mass_g=]
 *             水平槽 XY 辨识配置，只回一行
 *   SYSID YAW [inject=diff|rate thrust_mn=<mN> twist_deg=<90..1440>]
 *             吊绳偏航辨识配置，只回一行
 *   SYSID RATE <hz>                     线上采样率（控制拍恒为 500 Hz）
 *   SYSID INERTIA <kg*m^2>              前馈用的假定惯量；不给则取 airframe.ixx
 *   SYSID LIMIT [angle_deg= resid_dps=]
 *   SYSID THROTTLE [target_n= max_pct=]  自动油门；target_n=0 为遥控器手动给油门
 *   SYSID THR?                          只回一行 SYSID THR（轮询解锁/油门杆/阶段）
 *   SYSID PARAM coax.<rate_*|att_*|pos_z_kp|vel_z_*|pos_x_kp|vel_x_*> <v>
 *                                       只写 RAM 的增益试用（不排自动保存；别处触发的保存
 *                                       也存试用前的值，见 app_param_trial.h）
 *   SYSID PARAM | SYSID PARAM ?         列出试用中的名字（SYSID TRIAL 行）
 *   SYSID START | SYSID STOP            自动油门下 STOP 立即把油门交还遥控器
 *
 * 不带参数的子命令一律是**读回**而不是复位。台架程序里"想看一眼却把配置清了"
 * 是最容易让人白跑一趟的交互错误。
 */

#include "app_sysid.h"
#include "app_sysid_alt.h"
#include "app_sysid_xy.h"
#include "app_sysid_yaw.h"

#include "app_control.h"
#include "app_control_internal.h"
#include "app_ident.h"
#include "app_param_trial.h"
#include "app_rpm_notch.h"
#include "app_servo_backlash.h"

#include <math.h>
#include <string.h>

#define SYSID_DEG_TO_RAD 0.01745329251994329577f

/* ------------------------------------------------------------------ 小工具 */

static uint8_t sysid_kv_f32(char **tokens, uint32_t count,
                            const char *key, float *out, uint8_t *seen)
{
    const char *text = app_control_token_value(tokens, count, key);

    if (text == NULL) {
        return 1U;  /* 没给这个键：保持原值，不算错误 */
    }
    if (app_control_parse_f32(text, out) == 0U) {
        APP_Control_QueueText("ERR sysid %s\r\n", key);
        return 0U;
    }
    *seen = 1U;
    return 1U;
}

static uint8_t sysid_kv_u32(char **tokens, uint32_t count,
                            const char *key, uint32_t *out, uint8_t *seen)
{
    const char *text = app_control_token_value(tokens, count, key);

    if (text == NULL) {
        return 1U;
    }
    if (app_control_parse_u32(text, out) == 0U) {
        APP_Control_QueueText("ERR sysid %s\r\n", key);
        return 0U;
    }
    *seen = 1U;
    return 1U;
}

static uint8_t sysid_profile_from_name(const char *name, uint8_t *out)
{
    if (name == NULL) {
        return 0U;
    }
    if (strcmp(name, "step") == 0) {
        *out = (uint8_t)DRV_SYSID_PROFILE_STEP;
    } else if (strcmp(name, "doublet") == 0) {
        *out = (uint8_t)DRV_SYSID_PROFILE_DOUBLET;
    } else if (strcmp(name, "chirp") == 0) {
        *out = (uint8_t)DRV_SYSID_PROFILE_CHIRP;
    } else if (strcmp(name, "prbs") == 0) {
        *out = (uint8_t)DRV_SYSID_PROFILE_PRBS;
    } else {
        return 0U;
    }
    return 1U;
}

/* ------------------------------------------------------------------ 子命令 */

static void sysid_cmd_rig(char **tokens, uint32_t count)
{
    DRV_SysIdRig rig;
    float psi_deg;
    uint8_t seen = 0U;

    APP_SysId_GetRig(&rig);
    psi_deg = rig.azimuth_rad / SYSID_DEG_TO_RAD;

    if (sysid_kv_f32(tokens, count, "psi_deg", &psi_deg, &seen) == 0U) { return; }
    if (sysid_kv_f32(tokens, count, "axis_off_m",
                     &rig.axis_offset_above_cg_m, &seen) == 0U) { return; }
    if (sysid_kv_f32(tokens, count, "imu_off_m",
                     &rig.imu_above_cg_m, &seen) == 0U) { return; }

    if (seen != 0U) {
        rig.azimuth_rad = psi_deg * SYSID_DEG_TO_RAD;
        if (APP_SysId_SetRig(&rig) == 0U) {
            APP_Control_QueueText("ERR sysid rig range\r\n");
            return;
        }
    }
    APP_SysId_ReportStatus();
}

static void sysid_cmd_exc(char **tokens, uint32_t count)
{
    DRV_SysIdExcitation spec;
    const char *profile_text;
    float servo_tilt_mrad = 0.0f;
    uint8_t seen = 0U;
    uint8_t tilt_seen = 0U;

    APP_SysId_GetExcitation(&spec);

    profile_text = app_control_token_value(tokens, count, "profile");
    if (profile_text != NULL) {
        if (sysid_profile_from_name(profile_text, &spec.profile) == 0U) {
            APP_Control_QueueText("ERR sysid profile step|doublet|chirp|prbs\r\n");
            return;
        }
        seen = 1U;
    }

    if (sysid_kv_f32(tokens, count, "amp", &spec.amplitude_rad_s, &seen) == 0U) { return; }
    if (sysid_kv_u32(tokens, count, "dur_ms", &spec.duration_ms, &seen) == 0U) { return; }
    if (sysid_kv_u32(tokens, count, "hold_ms", &spec.hold_ms, &seen) == 0U) { return; }
    if (sysid_kv_u32(tokens, count, "repeat", &spec.repeat, &seen) == 0U) { return; }
    if (sysid_kv_u32(tokens, count, "ramp_ms", &spec.ramp_ms, &seen) == 0U) { return; }
    if (sysid_kv_f32(tokens, count, "f0", &spec.chirp_f0_hz, &seen) == 0U) { return; }
    if (sysid_kv_f32(tokens, count, "f1", &spec.chirp_f1_hz, &seen) == 0U) { return; }
    if (sysid_kv_u32(tokens, count, "bit_ms", &spec.prbs_bit_ms, &seen) == 0U) { return; }
    if (sysid_kv_u32(tokens, count, "seed", &spec.prbs_seed, &seen) == 0U) { return; }
    if (sysid_kv_f32(tokens, count, "servo_tilt_mrad", &servo_tilt_mrad, &tilt_seen) == 0U) { return; }

    if ((seen != 0U) || (tilt_seen != 0U)) {
        /*
         * 体检不通过就整条拒绝，不接受部分生效。半套参数跑出来的数据
         * 看起来完全正常，只是它不是你以为的那条激励。舵机倾转幅值先判范围，
         * 剖面过了体检才一起落下。没给幅值时沿用原值（不经 mrad 往返，免得端点
         * 值被舍入到范围外）；给了按除法换算，端点 262 与常量逐位相等。
         */
        const float servo_tilt_rad = (tilt_seen != 0U) ? (servo_tilt_mrad / 1000.0f)
                                                       : APP_SysId_GetServoTilt();

        if ((servo_tilt_rad < APP_SYSID_SERVO_TILT_MIN_RAD) ||
            (servo_tilt_rad > APP_SYSID_SERVO_TILT_MAX_RAD) ||
            (APP_SysId_SetExcitation(&spec) == 0U) ||
            (APP_SysId_SetServoTilt(servo_tilt_rad) == 0U)) {
            APP_Control_QueueText("ERR sysid excitation rejected\r\n");
            return;
        }
    }
    APP_SysId_ReportStatus();
}

static void sysid_cmd_limit(char **tokens, uint32_t count)
{
    float angle_deg = APP_SysId_GetAngleLimit() / SYSID_DEG_TO_RAD;
    float resid_dps = APP_SysId_GetResidualLimit() / SYSID_DEG_TO_RAD;
    uint8_t seen = 0U;

    if (sysid_kv_f32(tokens, count, "angle_deg", &angle_deg, &seen) == 0U) { return; }
    if (sysid_kv_f32(tokens, count, "resid_dps", &resid_dps, &seen) == 0U) { return; }

    if (seen != 0U) {
        if (APP_SysId_SetAngleLimit(angle_deg * SYSID_DEG_TO_RAD) == 0U) {
            APP_Control_QueueText("ERR sysid angle_deg range\r\n");
            return;
        }
        if (APP_SysId_SetResidualLimit(resid_dps * SYSID_DEG_TO_RAD) == 0U) {
            APP_Control_QueueText("ERR sysid resid_dps range\r\n");
            return;
        }
    }
    APP_SysId_ReportStatus();
}

static void sysid_cmd_throttle(char **tokens, uint32_t count)
{
    float target_n;
    float max_pct;
    uint8_t seen = 0U;

    APP_SysId_GetThrottle(&target_n, &max_pct);
    if (sysid_kv_f32(tokens, count, "target_n", &target_n, &seen) == 0U) { return; }
    if (sysid_kv_f32(tokens, count, "max_pct", &max_pct, &seen) == 0U) { return; }
    if ((seen != 0U) && (APP_SysId_SetThrottle(target_n, max_pct) == 0U)) {
        APP_Control_QueueText("ERR sysid throttle: target_n=0 (manual) or 2..max_total_force_n, "
                              "max_pct 10..95, idle only\r\n");
        return;
    }
    APP_SysId_ReportStatus();
}

/*
 * SYSID ALT [inject=break|vel|pos] [mass_g=<g>] [win_mm=<mm>] [lift_mm=<mm>]
 *           [bottom_mm=<TOF mm>] [top_mm=<TOF mm>]：高度辨识配置。
 * 带不带参数都只回一行 SYSID ALT（上位机逐项核对它），不刷整份状态报告；拒绝回
 * `SYSID ALT event=rejected reason=running|range|usage`，整条不生效。不带参数是读回，
 * 运行中也可以读。
 */
static void sysid_cmd_alt(char **tokens, uint32_t count)
{
    static const char *const keys[] = {
        "inject=", "mass_g=", "win_mm=", "lift_mm=", "bottom_mm=", "top_mm="
    };
    APP_SysIdAltConfig config;
    uint16_t *numbers[5];
    const char *reason = NULL;

    APP_SysIdAlt_GetConfig(&config);
    numbers[0] = &config.mass_g;
    numbers[1] = &config.win_mm;
    numbers[2] = &config.lift_mm;
    numbers[3] = &config.bottom_mm;
    numbers[4] = &config.top_mm;
    for (uint32_t i = 2U; (i < count) && (reason == NULL); ++i) {
        uint32_t key = 0U;
        uint32_t value = 0U;

        while ((key < 6U) && (strncmp(tokens[i], keys[key], strlen(keys[key])) != 0)) {
            key++;
        }
        if (key == 0U) {
            if (APP_SysIdAlt_InjectFromName(tokens[i] + strlen(keys[0]), &config.inject) == 0U) {
                reason = "usage";
            }
        } else if ((key >= 6U) || (app_control_parse_u32(tokens[i] + strlen(keys[key]), &value) == 0U)) {
            reason = "usage";
        } else if (value > ((key >= 4U) ? APP_SYSID_ALT_ENDPOINT_MAX_MM : 65535U)) {
            reason = "range";
        } else {
            *numbers[key - 1U] = (uint16_t)value;
        }
    }
    if ((reason == NULL) && (count > 2U)) {
        reason = (APP_SysId_IsRunning() != 0U) ? "running" :
                 (APP_SysIdAlt_SetConfig(&config) == 0U) ? "range" : NULL;
    }
    if (reason != NULL) {
        APP_Control_QueueText("SYSID ALT event=rejected reason=%s\r\n", reason);
        return;
    }
    APP_SysIdAlt_ReportConfig();
}

/*
 * SYSID XY [inject=tilt|vel|pos] [win_mm=<mm>] [mass_g=<g>]：水平槽 XY 辨识配置。
 * 带不带参数都只回一行 SYSID XY；拒绝回 `SYSID XY event=rejected reason=running|range|usage`，
 * 整条不生效。不带参数是读回，运行中也可以读。
 */
static void sysid_cmd_xy(char **tokens, uint32_t count)
{
    static const char *const keys[] = { "inject=", "win_mm=", "mass_g=" };
    APP_SysIdXyConfig config;
    uint16_t *numbers[2];
    const char *reason = NULL;

    APP_SysIdXy_GetConfig(&config);
    numbers[0] = &config.win_mm;
    numbers[1] = &config.mass_g;
    for (uint32_t i = 2U; (i < count) && (reason == NULL); ++i) {
        uint32_t key = 0U;
        uint32_t value = 0U;

        while ((key < 3U) && (strncmp(tokens[i], keys[key], strlen(keys[key])) != 0)) {
            key++;
        }
        if (key == 0U) {
            if (APP_SysIdXy_InjectFromName(tokens[i] + strlen(keys[0]), &config.inject) == 0U) {
                reason = "usage";
            }
        } else if ((key >= 3U) || (app_control_parse_u32(tokens[i] + strlen(keys[key]), &value) == 0U)) {
            reason = "usage";
        } else if (value > 65535U) {
            reason = "range";
        } else {
            *numbers[key - 1U] = (uint16_t)value;
        }
    }
    if ((reason == NULL) && (count > 2U)) {
        reason = (APP_SysId_IsRunning() != 0U) ? "running" :
                 (APP_SysIdXy_SetConfig(&config) == 0U) ? "range" : NULL;
    }
    if (reason != NULL) {
        APP_Control_QueueText("SYSID XY event=rejected reason=%s\r\n", reason);
        return;
    }
    APP_SysIdXy_ReportConfig();
}

/*
 * SYSID YAW [inject=diff|rate] [thrust_mn=<mN>] [twist_deg=<deg>]：吊绳偏航辨识配置。
 * 带不带参数都只回一行 SYSID YAW；拒绝回 `SYSID YAW event=rejected reason=running|range|lift|usage`，
 * 整条不生效。不带参数是读回，运行中也可以读。
 */
static void sysid_cmd_yaw(char **tokens, uint32_t count)
{
    static const char *const keys[] = { "inject=", "thrust_mn=", "twist_deg=" };
    APP_SysIdYawConfig config;
    uint16_t *numbers[2];
    const char *reason = NULL;

    APP_SysIdYaw_GetConfig(&config);
    numbers[0] = &config.thrust_mn;
    numbers[1] = &config.twist_deg;
    for (uint32_t i = 2U; (i < count) && (reason == NULL); ++i) {
        uint32_t key = 0U;
        uint32_t value = 0U;

        while ((key < 3U) && (strncmp(tokens[i], keys[key], strlen(keys[key])) != 0)) {
            key++;
        }
        if (key == 0U) {
            if (APP_SysIdYaw_InjectFromName(tokens[i] + strlen(keys[0]), &config.inject) == 0U) {
                reason = "usage";
            }
        } else if ((key >= 3U) || (app_control_parse_u32(tokens[i] + strlen(keys[key]), &value) == 0U)) {
            reason = "usage";
        } else if (value > 65535U) {
            reason = "range";
        } else {
            *numbers[key - 1U] = (uint16_t)value;
        }
    }
    if ((reason == NULL) && (count > 2U)) {
        reason = (APP_SysId_IsRunning() != 0U) ? "running" : APP_SysIdYaw_SetConfig(&config);
    }
    if (reason != NULL) {
        APP_Control_QueueText("SYSID YAW event=rejected reason=%s\r\n", reason);
        return;
    }
    APP_SysIdYaw_ReportConfig();
}

uint8_t app_control_handle_sysid(char **tokens, uint32_t count)
{
    const char *sub;

    if ((tokens == NULL) || (count == 0U)) {
        return 0U;
    }
    if ((strcmp(tokens[0], "SYSID") != 0) && (strcmp(tokens[0], "SYSID?") != 0)) {
        return 0U;
    }

    if ((count < 2U) || (strcmp(tokens[0], "SYSID?") == 0)) {
        APP_SysId_ReportStatus();
        return 1U;
    }

    sub = tokens[1];

    if (strcmp(sub, "HOLD") == 0) {
        APP_SysId_Hold();
        APP_SysId_ReportStatus();
        return 1U;
    }

    if (strcmp(sub, "DISCARD") == 0) {
        APP_Control_QueueText(APP_SysId_Discard() ? "OK sysid discarded stopped run\r\n" :
                                                     "ERR sysid still running\r\n");
        return 1U;
    }

    if (strcmp(sub, "MODE") == 0 && count >= 3U) {
        float angle = 3.0f;
        /* 名字与 READY 行里报的 mode= 数字两种写法都认。 */
        APP_SysIdMode mode =
            (strcmp(tokens[2], "FF") == 0 || strcmp(tokens[2], "0") == 0) ? APP_SYSID_FEEDFORWARD :
            (strcmp(tokens[2], "RATE") == 0 || strcmp(tokens[2], "1") == 0) ? APP_SYSID_RATE :
            (strcmp(tokens[2], "ANGLE") == 0 || strcmp(tokens[2], "2") == 0) ? APP_SYSID_ANGLE :
            (strcmp(tokens[2], "SERVO") == 0 || strcmp(tokens[2], "3") == 0) ? APP_SYSID_SERVO :
            (strcmp(tokens[2], "ALT") == 0 || strcmp(tokens[2], "4") == 0) ? APP_SYSID_ALT :
            (strcmp(tokens[2], "XY") == 0 || strcmp(tokens[2], "5") == 0) ? APP_SYSID_XY :
            (strcmp(tokens[2], "YAW") == 0 || strcmp(tokens[2], "6") == 0) ? APP_SYSID_YAW :
            (APP_SysIdMode)99;
        if ((count > 3U && !app_control_parse_f32(tokens[3], &angle)) ||
            !APP_SysId_SetMode(mode, angle * 0.01745329252f)) {
            APP_Control_QueueText("ERR sysid mode: MODE FF|RATE|ANGLE|SERVO|ALT|XY|YAW [0<deg<=15], idle only\r\n");
        } else { APP_SysId_ReportStatus(); }
        return 1U;
    }

    if ((strcmp(sub, "?") == 0) || (strcmp(sub, "STATUS") == 0)) {
        APP_SysId_ReportStatus();
        return 1U;
    }
    if (strcmp(sub, "SCHEMA") == 0) {
        APP_SysId_ReportSchema();
        return 1U;
    }
    if (strcmp(sub, "RIG") == 0) {
        sysid_cmd_rig(tokens, count);
        return 1U;
    }
    if (strcmp(sub, "EXC") == 0) {
        sysid_cmd_exc(tokens, count);
        return 1U;
    }
    if (strcmp(sub, "ALT") == 0) {
        sysid_cmd_alt(tokens, count);
        return 1U;
    }
    if (strcmp(sub, "XY") == 0) {
        sysid_cmd_xy(tokens, count);
        return 1U;
    }
    if (strcmp(sub, "YAW") == 0) {
        sysid_cmd_yaw(tokens, count);
        return 1U;
    }
    if (strcmp(sub, "LIMIT") == 0) {
        sysid_cmd_limit(tokens, count);
        return 1U;
    }
    if (strcmp(sub, "THROTTLE") == 0) {
        sysid_cmd_throttle(tokens, count);
        return 1U;
    }
    if (strcmp(sub, "PARAM") == 0) {
        /*
         * 只写 RAM 的增益试用。普通 PARAM SET 成功后 1.5 s 会自动存 Flash，把一组还没
         * 在杆上验证过的候选增益变成下次上电的默认值（2026-09-27 审查确认）。这里只认
         * 控制增益，不排自动保存；旧固件会回 ERR unknown，上位机据此拒绝应用。
         *
         * 光不排自动保存还不够：别的命令排的保存存的是整份配置，会把 RAM 里的试用值一起
         * 带进去（2026-09-27 实机：试用 50% 后改了一次积分限幅，1.5 s 后 50% 被永久化）。
         * 所以不走 PARAM SET 那条写入口（它等于显式持久写，会结束试用），而走试用记录：
         * 记下试用前的值，任何保存都换回它。
         *
         * 高度环 z 通道的 coax.pos_z_kp / coax.vel_z_*（R-ALTID-1）同样只写 RAM，供 ALT 验证；
         * 水平 x 通道的 coax.pos_x_kp / coax.vel_x_*（R-XYID-1）同理，供水平槽 XY 验证。
         */
        float value;
        const char *name = (count >= 4U) ? tokens[2] : NULL;

        if ((count == 2U) || ((count == 3U) && (strcmp(tokens[2], "?") == 0))) {
            APP_ParamTrial_Report();
            return 1U;
        }
        if ((name == NULL) || (app_control_parse_f32(tokens[3], &value) == 0U)) {
            APP_Control_QueueText("ERR usage SYSID PARAM coax.<rate_*|att_*|pos_z_kp|vel_z_*|pos_x_kp|vel_x_*> <value>\r\n");
            return 1U;
        }
        if ((strncmp(name, "coax.rate_", 10U) != 0) && (strncmp(name, "coax.att_", 9U) != 0) &&
            (strcmp(name, "coax.pos_z_kp") != 0) && (strncmp(name, "coax.vel_z_", 11U) != 0) &&
            (strcmp(name, "coax.pos_x_kp") != 0) && (strncmp(name, "coax.vel_x_", 11U) != 0)) {
            APP_Control_QueueText("ERR sysid param only coax.rate_* / coax.att_* / coax.pos_z_kp / coax.vel_z_* / coax.pos_x_kp / coax.vel_x_*\r\n");
            return 1U;
        }
        switch (APP_ParamTrial_Apply(name, value, &value)) {
        case APP_PARAM_TRIAL_OK:
            break;
        case APP_PARAM_TRIAL_FULL:
            APP_Control_QueueText("ERR sysid param trial full %s\r\n", name);
            return 1U;
        default:
            APP_Control_QueueText("ERR sysid param %s\r\n", name);
            return 1U;
        }
        {
            /* newlib-nano 没有浮点 printf：按百万分之一拼十进制。 */
            const uint8_t negative = (value < 0.0f) ? 1U : 0U;
            const double magnitude = negative ? -(double)value : (double)value;
            const unsigned long long micro = (unsigned long long)(magnitude * 1000000.0 + 0.5);
            APP_Control_QueueText("OK sysid param name=%s value=%s%lu.%06lu ram=1\r\n", name,
                                  negative ? "-" : "",
                                  (unsigned long)(micro / 1000000ULL),
                                  (unsigned long)(micro % 1000000ULL));
        }
        return 1U;
    }
    if (strcmp(sub, "THR?") == 0) {
        /* 上位机每秒轮询解锁/油门杆状态用：只回一行，不刷六行状态报告。 */
        APP_SysId_ReportThrottle();
        return 1U;
    }
    if (strcmp(sub, "RATE") == 0) {
        uint32_t hz;

        if ((count < 3U) || (app_control_parse_u32(tokens[2], &hz) == 0U) ||
            (APP_SysId_SetSampleRate(hz) == 0U)) {
            APP_Control_QueueText("ERR usage SYSID RATE %u..%u\r\n",
                                  (unsigned int)APP_SYSID_RATE_MIN_HZ,
                                  (unsigned int)APP_SYSID_RATE_MAX_HZ);
            return 1U;
        }
        APP_SysId_ReportStatus();
        return 1U;
    }
    if (strcmp(sub, "INERTIA") == 0) {
        float value;

        if ((count < 3U) || (app_control_parse_f32(tokens[2], &value) == 0U) ||
            (APP_SysId_SetInertia(value) == 0U)) {
            APP_Control_QueueText("ERR usage SYSID INERTIA <kg*m^2>\r\n");
            return 1U;
        }
        APP_SysId_ReportStatus();
        return 1U;
    }
    if (strcmp(sub, "START") == 0) {
        /*
         * 开跑成功后紧跟两行溯源：SYSID NOTCH（控制用陀螺的陷波配置）与 SYSID BACKLASH
         * （舵机回差补偿配置）。两者台架期间都改不了，所以开跑这一刻的配置代表整轮。
         */
        if (APP_SysId_Start() != 0U) {
            APP_RpmNotch_ReportProvenance(APP_SysId_GetRunId());
            APP_ServoBacklash_ReportProvenance(APP_SysId_GetRunId());
        }
        return 1U;
    }
    if (strcmp(sub, "STOP") == 0) {
        APP_SysId_Stop("command");
        APP_SysId_ReportStatus();
        return 1U;
    }

    APP_Control_QueueText("ERR unknown sysid subcmd %s\r\n", sub);
    return 1U;
}

/* ------------------------------------------------------------------ IDENT */

static uint8_t sysid_ident_att(char **tokens, uint32_t count)
{
    if (count < 3U) {
        APP_Control_QueueText("ERR usage IDENT ATT PRBS roll|pitch amp_mdeg=<v> bit_ms=<v> duration_ms=<v> [seed=<v>]\r\n");
        return 1U;
    }
    if ((strcmp(tokens[2], "?") == 0) || (strcmp(tokens[2], "STATUS") == 0)) {
        APP_Ident_ReportStatus();
        return 1U;
    }
    if (strcmp(tokens[2], "STOP") == 0) {
        APP_IdentAtt_Stop("command");
        return 1U;
    }
    if (strcmp(tokens[2], "PRBS") == 0) {
        uint32_t bit_ms;
        uint32_t duration_ms;
        uint32_t seed = 1U;
        int32_t amp_mdeg;
        const char *amp_text;
        const char *bit_text;
        const char *duration_text;
        const char *seed_text;

        if (count < 4U) {
            APP_Control_QueueText("ERR usage IDENT ATT PRBS roll|pitch amp_mdeg=<v> bit_ms=<v> duration_ms=<v> [seed=<v>]\r\n");
            return 1U;
        }
        amp_text = app_control_token_value(tokens, count, "amp_mdeg");
        bit_text = app_control_token_value(tokens, count, "bit_ms");
        duration_text = app_control_token_value(tokens, count, "duration_ms");
        seed_text = app_control_token_value(tokens, count, "seed");
        if ((seed_text != NULL) && (app_control_parse_u32(seed_text, &seed) == 0U)) {
            APP_Control_QueueText("ERR ident att seed\r\n");
            return 1U;
        }
        if ((amp_text == NULL) || (bit_text == NULL) || (duration_text == NULL) ||
            (app_control_parse_i32(amp_text, &amp_mdeg) == 0U) ||
            (app_control_parse_u32(bit_text, &bit_ms) == 0U) ||
            (app_control_parse_u32(duration_text, &duration_ms) == 0U)) {
            APP_Control_QueueText("ERR usage IDENT ATT PRBS roll|pitch amp_mdeg=<v> bit_ms=<v> duration_ms=<v> [seed=<v>]\r\n");
            return 1U;
        }
        (void)APP_IdentAtt_StartPrbs(tokens[3], amp_mdeg, bit_ms, duration_ms, seed);
        return 1U;
    }
    APP_Control_QueueText("ERR unknown ident att subcmd %s\r\n", tokens[2]);
    return 1U;
}

uint8_t app_cmd_sysid_handle_ident(char **tokens, uint32_t count)
{
    const char *sub;

    if ((tokens == NULL) || (count < 2U)) {
        return 0U;
    }
    sub = tokens[1];

    if ((strcmp(sub, "?") == 0) || (strcmp(sub, "STATUS") == 0)) {
        APP_Ident_ReportStatus();
        return 1U;
    }
    /*
     * 光杆台架那套在跑时不许启动老 ident：两者都接管舵机，同时跑只会有一个真正
     * 生效，另一个照样把"我发了这个"记成数据。读状态与 STOP 不受此限。
     */
    if ((APP_SysId_IsEngaged() != 0U) && (strcmp(sub, "STOP") != 0)) {
        APP_Control_QueueText("ERR ident sysid running\r\n");
        return 1U;
    }
    if (strcmp(sub, "ARM") == 0) {
        (void)APP_Ident_Arm();
        return 1U;
    }
    if (strcmp(sub, "DISARM") == 0) {
        APP_Ident_Disarm();
        return 1U;
    }
    if (strcmp(sub, "STOP") == 0) {
        APP_Ident_Stop("command");
        return 1U;
    }
    if (strcmp(sub, "ATT") == 0) {
        return sysid_ident_att(tokens, count);
    }
    if (strcmp(sub, "CENTER") == 0) {
        uint32_t alpha_us;
        uint32_t beta_us;
        const char *alpha_text = app_control_token_value(tokens, count, "alpha_us");
        const char *beta_text = app_control_token_value(tokens, count, "beta_us");

        if ((alpha_text == NULL) || (beta_text == NULL) ||
            (app_control_parse_u32(alpha_text, &alpha_us) == 0U) ||
            (app_control_parse_u32(beta_text, &beta_us) == 0U) ||
            (alpha_us > 65535U) || (beta_us > 65535U)) {
            APP_Control_QueueText("ERR usage IDENT CENTER alpha_us=<v> beta_us=<v>\r\n");
            return 1U;
        }
        (void)APP_Ident_SetCenter((uint16_t)alpha_us, (uint16_t)beta_us);
        return 1U;
    }
    if (strcmp(sub, "APPLY") == 0) {
        const char *kp_text;
        const char *kd_text;

        if (count < 3U) {
            APP_Control_QueueText("ERR usage IDENT APPLY roll|pitch rate_kp=<v>\r\n");
            return 1U;
        }
        kp_text = app_control_token_value(tokens, count, "rate_kp");
        kd_text = app_control_token_value(tokens, count, "kd");
        if ((count != 4U) || (kp_text == NULL) || (kd_text != NULL)) {
            APP_Control_QueueText("ERR usage IDENT APPLY roll|pitch rate_kp=<v>\r\n");
            return 1U;
        }
        (void)APP_Ident_ApplyRateKp(tokens[2], kp_text);
        return 1U;
    }
    if (strcmp(sub, "STEP") == 0) {
        uint32_t duration_ms;
        int32_t pulse_us;
        const char *pulse_text;
        const char *duration_text;

        if (count < 3U) {
            APP_Control_QueueText("ERR usage IDENT STEP roll|pitch pulse_us=<v> duration_ms=<v>\r\n");
            return 1U;
        }
        pulse_text = app_control_token_value(tokens, count, "pulse_us");
        duration_text = app_control_token_value(tokens, count, "duration_ms");
        if ((pulse_text == NULL) || (duration_text == NULL) ||
            (app_control_parse_i32(pulse_text, &pulse_us) == 0U) ||
            (app_control_parse_u32(duration_text, &duration_ms) == 0U)) {
            APP_Control_QueueText("ERR usage IDENT STEP roll|pitch pulse_us=<v> duration_ms=<v>\r\n");
            return 1U;
        }
        (void)APP_Ident_StartStep(tokens[2], pulse_us, duration_ms);
        return 1U;
    }
    if (strcmp(sub, "DOUBLET") == 0) {
        uint32_t hold_ms;
        uint32_t repeat;
        int32_t pulse_us;
        const char *pulse_text;
        const char *hold_text;
        const char *repeat_text;

        if (count < 3U) {
            APP_Control_QueueText("ERR usage IDENT DOUBLET roll|pitch pulse_us=<v> hold_ms=<v> repeat=<v>\r\n");
            return 1U;
        }
        pulse_text = app_control_token_value(tokens, count, "pulse_us");
        hold_text = app_control_token_value(tokens, count, "hold_ms");
        repeat_text = app_control_token_value(tokens, count, "repeat");
        if ((pulse_text == NULL) || (hold_text == NULL) || (repeat_text == NULL) ||
            (app_control_parse_i32(pulse_text, &pulse_us) == 0U) ||
            (app_control_parse_u32(hold_text, &hold_ms) == 0U) ||
            (app_control_parse_u32(repeat_text, &repeat) == 0U)) {
            APP_Control_QueueText("ERR usage IDENT DOUBLET roll|pitch pulse_us=<v> hold_ms=<v> repeat=<v>\r\n");
            return 1U;
        }
        (void)APP_Ident_StartDoublet(tokens[2], pulse_us, hold_ms, repeat);
        return 1U;
    }
    if (strcmp(sub, "PRBS") == 0) {
        uint32_t bit_ms;
        uint32_t duration_ms;
        uint32_t seed = 1U;
        int32_t pulse_us;
        const char *pulse_text;
        const char *bit_text;
        const char *duration_text;
        const char *seed_text;

        if (count < 3U) {
            APP_Control_QueueText("ERR usage IDENT PRBS roll|pitch pulse_us=<v> bit_ms=<v> duration_ms=<v> [seed=<v>]\r\n");
            return 1U;
        }
        pulse_text = app_control_token_value(tokens, count, "pulse_us");
        bit_text = app_control_token_value(tokens, count, "bit_ms");
        duration_text = app_control_token_value(tokens, count, "duration_ms");
        seed_text = app_control_token_value(tokens, count, "seed");
        if ((seed_text != NULL) && (app_control_parse_u32(seed_text, &seed) == 0U)) {
            APP_Control_QueueText("ERR ident seed\r\n");
            return 1U;
        }
        if ((pulse_text == NULL) || (bit_text == NULL) || (duration_text == NULL) ||
            (app_control_parse_i32(pulse_text, &pulse_us) == 0U) ||
            (app_control_parse_u32(bit_text, &bit_ms) == 0U) ||
            (app_control_parse_u32(duration_text, &duration_ms) == 0U)) {
            APP_Control_QueueText("ERR usage IDENT PRBS roll|pitch pulse_us=<v> bit_ms=<v> duration_ms=<v> [seed=<v>]\r\n");
            return 1U;
        }
        (void)APP_Ident_StartPrbs(tokens[2], pulse_us, bit_ms, duration_ms, seed);
        return 1U;
    }

    /* START 留在 app_control.c（电机阶梯，耦合那边的静态状态与服务拍）。 */
    return 0U;
}
