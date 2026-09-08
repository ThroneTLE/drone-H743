#include "app_telemetry.h"

#include "app_control.h"
#include "drv_frame_contract.h"

#include <stdio.h>
#include <stdlib.h>

#define APP_TELEM_BODY_FRAME_NAME "body_flu"
#define APP_TELEM_FRAME_CONTRACT_VERSION DRV_FRAME_CONTRACT_VERSION

/*
 * 通道表 —— 顺序必须与 app_telem_port.c 的填充顺序一致。
 * 填充代码用 APP_TELEM_CH_* 枚举名下标，因此"顺序一致"由枚举保证，
 * 本表只负责元数据。
 *
 * 旧增益通道名作为显式换算 alias 保留在线路上，避免重排历史编号；V19 的
 * 真实物理参数名由 DRV_COAX_CTRL_ParamCount/ParamName 报告。pos_z_ki 只保留
 * 只读零值占位，旧位置积分不会偷换成新速度积分。
 */
static const APP_TelemChannel app_telem_channels[APP_TELEM_CH_COUNT] = {
    [APP_TELEM_CH_ROLL]        = {"roll",        "deg",   "attitude", -180.0f, 180.0f, "-"},
    [APP_TELEM_CH_PITCH]       = {"pitch",       "deg",   "attitude",  -90.0f,  90.0f, "-"},
    [APP_TELEM_CH_YAW]         = {"yaw",         "deg",   "attitude", -180.0f, 180.0f, "-"},
    [APP_TELEM_CH_FLOW_HEIGHT] = {"flow_height", "m",     "nav",         0.0f,   5.0f, "-"},
    [APP_TELEM_CH_TIME]        = {"uptime",      "s",     "system",      0.0f, 3600.0f, "-"},
    [APP_TELEM_CH_VEL_EST_X]   = {"vel_est_x",   "m/s",   "nav",        -5.0f,   5.0f, "-"},
    [APP_TELEM_CH_VEL_EST_Y]   = {"vel_est_y",   "m/s",   "nav",        -5.0f,   5.0f, "-"},

    [APP_TELEM_CH_ROLL_RATE_KD]    = {"roll_rate_kd",    "-", "gain", 0.0f, 10.0f, "coax.roll_rate_kd"},
    [APP_TELEM_CH_PITCH_RATE_KD]   = {"pitch_rate_kd",   "-", "gain", 0.0f, 10.0f, "coax.pitch_rate_kd"},
    [APP_TELEM_CH_YAW_ANGLE_KP]    = {"yaw_angle_kp",    "-", "gain", 0.0f, 10.0f, "coax.yaw_angle_kp"},
    [APP_TELEM_CH_YAW_RATE_KD]     = {"yaw_rate_kd",     "-", "gain", 0.0f, 10.0f, "coax.yaw_rate_kd"},
    [APP_TELEM_CH_POS_X_KP]        = {"pos_x_kp",        "-", "gain", 0.0f, 10.0f, "coax.pos_x_kp"},
    [APP_TELEM_CH_POS_Y_KP]        = {"pos_y_kp",        "-", "gain", 0.0f, 10.0f, "coax.pos_y_kp"},
    [APP_TELEM_CH_VEL_X_KD]        = {"vel_x_kd",        "-", "gain", 0.0f, 10.0f, "coax.vel_x_kd"},
    [APP_TELEM_CH_VEL_Y_KD]        = {"vel_y_kd",        "-", "gain", 0.0f, 10.0f, "coax.vel_y_kd"},

    [APP_TELEM_CH_POS_EST_X] = {"pos_est_x", "m", "nav", -10.0f, 10.0f, "-"},
    [APP_TELEM_CH_POS_EST_Y] = {"pos_est_y", "m", "nav", -10.0f, 10.0f, "-"},

    [APP_TELEM_CH_VEL_LOOP_ENABLE] = {"vel_loop_enable", "-", "gain", 0.0f,  1.0f, "coax.vel_loop_enable"},
    [APP_TELEM_CH_ROLL_ANGLE_KP]   = {"roll_angle_kp",   "-", "gain", 0.0f, 10.0f, "coax.roll_angle_kp"},
    [APP_TELEM_CH_PITCH_ANGLE_KP]  = {"pitch_angle_kp",  "-", "gain", 0.0f, 10.0f, "coax.pitch_angle_kp"},
    [APP_TELEM_CH_POS_Z_KP]        = {"pos_z_kp",        "-", "gain", 0.0f, 10.0f, "coax.pos_z_kp"},
    [APP_TELEM_CH_POS_Z_KI]        = {"pos_z_ki",        "-", "legacy", 0.0f, 1.0f, "-"},
    [APP_TELEM_CH_VEL_Z_KD]        = {"vel_z_kd",        "-", "gain", 0.0f, 10.0f, "coax.vel_z_kd"},

    [APP_TELEM_CH_FUSION_ACC_ERR]           = {"fusion_acc_err",         "deg",   "fusion", 0.0f,  180.0f, "-"},
    [APP_TELEM_CH_FUSION_ACC_IGNORED]       = {"fusion_acc_ignored",     "-",     "fusion", 0.0f,    1.0f, "-"},
    [APP_TELEM_CH_FUSION_ACC_RECOVERY]      = {"fusion_acc_recovery",    "-",     "fusion", 0.0f,    1.0f, "-"},
    [APP_TELEM_CH_FUSION_ACC_CORRECTIONS]   = {"fusion_acc_corrections", "count", "fusion", 0.0f, 1000.0f, "-"},
    [APP_TELEM_CH_FUSION_ACC_NORM_REJECTED] = {"fusion_acc_norm_rej",    "count", "fusion", 0.0f, 1000.0f, "-"},
    [APP_TELEM_CH_CTRL_POS_SP_X] = {"ctrl_pos_sp_x", "m", "cascade", -5.0f, 5.0f, "-"},
    [APP_TELEM_CH_CTRL_POS_SP_Y] = {"ctrl_pos_sp_y", "m", "cascade", -5.0f, 5.0f, "-"},
    [APP_TELEM_CH_CTRL_POS_SP_Z] = {"ctrl_pos_sp_z", "m", "cascade", -5.0f, 5.0f, "-"},
    [APP_TELEM_CH_CTRL_POS_ERR_X] = {"ctrl_pos_err_x", "m", "cascade", -2.0f, 2.0f, "-"},
    [APP_TELEM_CH_CTRL_POS_ERR_Y] = {"ctrl_pos_err_y", "m", "cascade", -2.0f, 2.0f, "-"},
    [APP_TELEM_CH_CTRL_POS_ERR_Z] = {"ctrl_pos_err_z", "m", "cascade", -2.0f, 2.0f, "-"},
    [APP_TELEM_CH_CTRL_VEL_SP_X] = {"ctrl_vel_sp_x", "m/s", "cascade", -5.0f, 5.0f, "-"},
    [APP_TELEM_CH_CTRL_VEL_SP_Y] = {"ctrl_vel_sp_y", "m/s", "cascade", -5.0f, 5.0f, "-"},
    [APP_TELEM_CH_CTRL_VEL_SP_Z] = {"ctrl_vel_sp_z", "m/s", "cascade", -5.0f, 5.0f, "-"},
    [APP_TELEM_CH_CTRL_VEL_ERR_X] = {"ctrl_vel_err_x", "m/s", "cascade", -5.0f, 5.0f, "-"},
    [APP_TELEM_CH_CTRL_VEL_ERR_Y] = {"ctrl_vel_err_y", "m/s", "cascade", -5.0f, 5.0f, "-"},
    [APP_TELEM_CH_CTRL_VEL_ERR_Z] = {"ctrl_vel_err_z", "m/s", "cascade", -5.0f, 5.0f, "-"},
    [APP_TELEM_CH_CTRL_ACCEL_SP_X] = {"ctrl_accel_sp_x", "m/s2", "cascade", -10.0f, 10.0f, "-"},
    [APP_TELEM_CH_CTRL_ACCEL_SP_Y] = {"ctrl_accel_sp_y", "m/s2", "cascade", -10.0f, 10.0f, "-"},
    [APP_TELEM_CH_CTRL_ACCEL_SP_Z] = {"ctrl_accel_sp_z", "m/s2", "cascade", -10.0f, 10.0f, "-"},
    [APP_TELEM_CH_CTRL_ATT_ERR_X] = {"ctrl_att_err_x", "rad", "cascade", -3.2f, 3.2f, "-"},
    [APP_TELEM_CH_CTRL_ATT_ERR_Y] = {"ctrl_att_err_y", "rad", "cascade", -3.2f, 3.2f, "-"},
    [APP_TELEM_CH_CTRL_ATT_ERR_Z] = {"ctrl_att_err_z", "rad", "cascade", -3.2f, 3.2f, "-"},
    [APP_TELEM_CH_CTRL_RATE_SP_X] = {"ctrl_rate_sp_x", "rad/s", "cascade", -10.0f, 10.0f, "-"},
    [APP_TELEM_CH_CTRL_RATE_SP_Y] = {"ctrl_rate_sp_y", "rad/s", "cascade", -10.0f, 10.0f, "-"},
    [APP_TELEM_CH_CTRL_RATE_SP_Z] = {"ctrl_rate_sp_z", "rad/s", "cascade", -10.0f, 10.0f, "-"},
    [APP_TELEM_CH_CTRL_RATE_ERR_X] = {"ctrl_rate_err_x", "rad/s", "cascade", -10.0f, 10.0f, "-"},
    [APP_TELEM_CH_CTRL_RATE_ERR_Y] = {"ctrl_rate_err_y", "rad/s", "cascade", -10.0f, 10.0f, "-"},
    [APP_TELEM_CH_CTRL_RATE_ERR_Z] = {"ctrl_rate_err_z", "rad/s", "cascade", -10.0f, 10.0f, "-"},
    [APP_TELEM_CH_CTRL_MOMENT_CMD_X] = {"ctrl_moment_cmd_x", "N*m", "cascade", -0.2f, 0.2f, "-"},
    [APP_TELEM_CH_CTRL_MOMENT_CMD_Y] = {"ctrl_moment_cmd_y", "N*m", "cascade", -0.2f, 0.2f, "-"},
    [APP_TELEM_CH_CTRL_MOMENT_CMD_Z] = {"ctrl_moment_cmd_z", "N*m", "cascade", -0.01f, 0.01f, "-"},
    [APP_TELEM_CH_CTRL_MOMENT_ACH_X] = {"ctrl_moment_ach_x", "N*m", "cascade", -0.2f, 0.2f, "-"},
    [APP_TELEM_CH_CTRL_MOMENT_ACH_Y] = {"ctrl_moment_ach_y", "N*m", "cascade", -0.2f, 0.2f, "-"},
    [APP_TELEM_CH_CTRL_MOMENT_ACH_Z] = {"ctrl_moment_ach_z", "N*m", "cascade", -0.01f, 0.01f, "-"},
    [APP_TELEM_CH_CTRL_SAT_POS_X] = {"ctrl_sat_pos_x", "bool", "cascade", 0.0f, 1.0f, "-"},
    [APP_TELEM_CH_CTRL_SAT_POS_Y] = {"ctrl_sat_pos_y", "bool", "cascade", 0.0f, 1.0f, "-"},
    [APP_TELEM_CH_CTRL_SAT_POS_Z] = {"ctrl_sat_pos_z", "bool", "cascade", 0.0f, 1.0f, "-"},
    [APP_TELEM_CH_CTRL_SAT_NEG_X] = {"ctrl_sat_neg_x", "bool", "cascade", 0.0f, 1.0f, "-"},
    [APP_TELEM_CH_CTRL_SAT_NEG_Y] = {"ctrl_sat_neg_y", "bool", "cascade", 0.0f, 1.0f, "-"},
    [APP_TELEM_CH_CTRL_SAT_NEG_Z] = {"ctrl_sat_neg_z", "bool", "cascade", 0.0f, 1.0f, "-"},

    /*
     * 串级四环的真名增益（通道 64 起）。量程不是拍脑袋的"0..10"：滑块拖到头
     * 的那个值必须是这一项物理上还说得通的上界，否则拖到 30% 就已经炸机。
     * 每条的上界都锚在默认值或机体惯量上，注释里写清锚点，改机体时跟着改。
     *
     * 下界一律 0：DRV_COAX_CTRL_SetParam 直接拒绝负值，给负量程只会让滑块
     * 左半段全是静默失败。
     */
    /* 角速度环 P：N.m/(rad/s)。默认 roll/pitch 0.1104/0.1138，yaw = Izz*0.15。 */
    [APP_TELEM_CH_RATE_ROLL_KP]  = {"rate_roll_kp",  "N.m/(rad/s)", "gain", 0.0f, 1.0f,    "coax.rate_roll_kp"},
    [APP_TELEM_CH_RATE_PITCH_KP] = {"rate_pitch_kp", "N.m/(rad/s)", "gain", 0.0f, 1.0f,    "coax.rate_pitch_kp"},
    [APP_TELEM_CH_RATE_YAW_KP]   = {"rate_yaw_kp",   "N.m/(rad/s)", "gain", 0.0f, 0.002f,  "coax.rate_yaw_kp"},
    /* 角速度环 I：N.m/rad。上界锚在各轴积分限幅（0.010 / 0.010 / 0.0002 N.m）。 */
    [APP_TELEM_CH_RATE_ROLL_KI]  = {"rate_roll_ki",  "N.m/rad", "gain", 0.0f, 0.5f,   "coax.rate_roll_ki"},
    [APP_TELEM_CH_RATE_PITCH_KI] = {"rate_pitch_ki", "N.m/rad", "gain", 0.0f, 0.5f,   "coax.rate_pitch_ki"},
    [APP_TELEM_CH_RATE_YAW_KI]   = {"rate_yaw_ki",   "N.m/rad", "gain", 0.0f, 0.01f,  "coax.rate_yaw_ki"},
    /*
     * 角速度环 D：作用在**差分得到的角加速度**上，量纲就是转动惯量 kg.m^2，
     * 所以上界锚在机体惯量本身（Ixx=Iyy=0.051、Izz=0.00035）。Kd 取到 1×J
     * 相当于把一整个惯量的加速度反馈回去，再大就该怀疑是差分噪声在驱动力矩了。
     */
    [APP_TELEM_CH_RATE_ROLL_KD]  = {"rate_roll_kd",  "kg.m^2", "gain", 0.0f, 0.05f,   "coax.rate_roll_kd"},
    [APP_TELEM_CH_RATE_PITCH_KD] = {"rate_pitch_kd", "kg.m^2", "gain", 0.0f, 0.05f,   "coax.rate_pitch_kd"},
    [APP_TELEM_CH_RATE_YAW_KD]   = {"rate_yaw_kd",   "kg.m^2", "gain", 0.0f, 0.001f,  "coax.rate_yaw_kd"},
    /* 姿态环 P：1/s。默认 roll/pitch 约 0.61/0.58，yaw 约 6.67。 */
    [APP_TELEM_CH_ATT_ROLL_KP]  = {"att_roll_kp",  "1/s", "gain", 0.0f,  5.0f, "coax.att_roll_kp"},
    [APP_TELEM_CH_ATT_PITCH_KP] = {"att_pitch_kp", "1/s", "gain", 0.0f,  5.0f, "coax.att_pitch_kp"},
    [APP_TELEM_CH_ATT_YAW_KP]   = {"att_yaw_kp",   "1/s", "gain", 0.0f, 20.0f, "coax.att_yaw_kp"},
    /* 速度环 P / I。默认 kp 0.8/0.8/1.0，ki 全 0（积分限幅 1.5 m/s^2）。 */
    [APP_TELEM_CH_VEL_X_KP] = {"vel_x_kp", "1/s",   "gain", 0.0f, 5.0f, "coax.vel_x_kp"},
    [APP_TELEM_CH_VEL_Y_KP] = {"vel_y_kp", "1/s",   "gain", 0.0f, 5.0f, "coax.vel_y_kp"},
    [APP_TELEM_CH_VEL_Z_KP] = {"vel_z_kp", "1/s",   "gain", 0.0f, 5.0f, "coax.vel_z_kp"},
    [APP_TELEM_CH_VEL_X_KI] = {"vel_x_ki", "1/s^2", "gain", 0.0f, 5.0f, "coax.vel_x_ki"},
    [APP_TELEM_CH_VEL_Y_KI] = {"vel_y_ki", "1/s^2", "gain", 0.0f, 5.0f, "coax.vel_y_ki"},
    [APP_TELEM_CH_VEL_Z_KI] = {"vel_z_ki", "1/s^2", "gain", 0.0f, 5.0f, "coax.vel_z_ki"},
    /*
     * 两个微分低通截止。它们不是"手感"参数：D 项吃的是差分噪声还是真信号
     * 全看这两个值，调 Kd 之前先把它定下来，所以必须和 Kd 摆在同一屏。
     * 默认 18.8496 rad/s（3 Hz）与 20 Hz；0 表示不滤波（直接用原始差分）。
     * 角加速度那个 3 Hz 不是保守，是被 50 Hz 的执行器出口逼出来的——推导见
     * drv_coax_ctrl.c 里 alpha_lpf_cutoff_rad_s 默认值上方的注释。
     */
    [APP_TELEM_CH_ANGULAR_ACCEL_LPF] = {"angular_accel_lpf", "rad/s", "gain", 0.0f, 628.0f, "coax.angular_accel_lpf_cutoff_rad_s"},
    [APP_TELEM_CH_ACCEL_LPF]         = {"accel_lpf",         "Hz",    "gain", 0.0f, 100.0f, "coax.accel_lpf_cutoff_hz"},
};

_Static_assert((sizeof(app_telem_channels) / sizeof(app_telem_channels[0])) ==
                   (size_t)APP_TELEM_CH_COUNT,
               "telemetry channel table must cover every channel id");

/*
 * 量程文本格式化。
 *
 * 不复用 app_control.c 的 app_control_format_float：那个函数按 1e6 缩放到
 * int，value 超过约 2145 就会整型溢出，而本表存在 uptime(3600) 与
 * 计数类(1000) 通道。这里按 1e3 缩放，配合 ±1e6 钳位，int32 不会溢出。
 */
static void app_telem_format_float(float value, char *buffer, uint32_t size)
{
    int32_t scaled;
    int32_t integral;
    int32_t fraction;

    if ((buffer == NULL) || (size == 0U)) {
        return;
    }

    if (value > 1000000.0f) {
        value = 1000000.0f;
    } else if (value < -1000000.0f) {
        value = -1000000.0f;
    }

    scaled   = (int32_t)((value * 1000.0f) + ((value >= 0.0f) ? 0.5f : -0.5f));
    integral = scaled / 1000;
    fraction = scaled % 1000;

    if (integral < 0) {
        integral = -integral;
    }
    if (fraction < 0) {
        fraction = -fraction;
    }

    (void)snprintf(buffer,
                   size,
                   "%s%ld.%03lu",
                   (scaled < 0) ? "-" : "",
                   (long)integral,
                   (unsigned long)fraction);
}

/* FNV-1a 32 位。选它是因为实现短、无表、便于上位机与 pytest 逐字节复算。 */
static uint32_t app_telem_hash_bytes(uint32_t hash, const char *text)
{
    if (text == NULL) {
        return hash;
    }

    while (*text != '\0') {
        hash ^= (uint32_t)(uint8_t)*text;
        hash *= 0x01000193UL;
        text++;
    }

    return hash;
}

uint32_t APP_Telemetry_ChannelCount(void)
{
    return (uint32_t)APP_TELEM_CH_COUNT;
}

const APP_TelemChannel *APP_Telemetry_GetChannel(uint32_t index)
{
    if (index >= (uint32_t)APP_TELEM_CH_COUNT) {
        return NULL;
    }

    return &app_telem_channels[index];
}

uint8_t APP_Telemetry_ChannelHasParam(uint32_t index)
{
    const APP_TelemChannel *channel = APP_Telemetry_GetChannel(index);

    if ((channel == NULL) || (channel->param == NULL)) {
        return 0U;
    }

    return ((channel->param[0] != '\0') && (channel->param[0] != '-')) ? 1U : 0U;
}

/*
 * 指纹覆盖版本、通道数、速率、坐标来源与每条通道的全部元数据。
 * 规范化文本为 "v<ver>|<count>|<rate>|<frame>|<contract>\n" 后接每通道
 * "<idx>|<name>|<unit>|<min>|<max>|<group>|<param>\n"，与 tests 中的复算实现一致。
 * v2 起把 param 也纳进来：上位机的滑块是按 param 数据驱动生成的，param 变了
 * 而 hash 不变，滑块就会绑到错的参数上而且没有任何人报错。
 */
uint32_t APP_Telemetry_SchemaHash(void)
{
    static uint32_t cached_hash  = 0U;
    static uint8_t  hash_is_valid = 0U;

    uint32_t hash = 0x811C9DC5UL;
    char     scratch[64];
    uint32_t index;

    if (hash_is_valid != 0U) {
        return cached_hash;
    }

    (void)snprintf(scratch,
                   sizeof(scratch),
                   "v%u|%u|%u|%s|%u\n",
                   (unsigned int)APP_TELEM_SCHEMA_VERSION,
                   (unsigned int)APP_TELEM_CH_COUNT,
                   (unsigned int)APP_TELEM_RATE_HZ,
                   APP_TELEM_BODY_FRAME_NAME,
                   (unsigned int)APP_TELEM_FRAME_CONTRACT_VERSION);
    hash = app_telem_hash_bytes(hash, scratch);

    for (index = 0U; index < (uint32_t)APP_TELEM_CH_COUNT; ++index) {
        const APP_TelemChannel *channel = &app_telem_channels[index];

        (void)snprintf(scratch, sizeof(scratch), "%u|", (unsigned int)index);
        hash = app_telem_hash_bytes(hash, scratch);
        hash = app_telem_hash_bytes(hash, channel->name);
        hash = app_telem_hash_bytes(hash, "|");
        hash = app_telem_hash_bytes(hash, channel->unit);
        hash = app_telem_hash_bytes(hash, "|");

        app_telem_format_float(channel->min, scratch, (uint32_t)sizeof(scratch));
        hash = app_telem_hash_bytes(hash, scratch);
        hash = app_telem_hash_bytes(hash, "|");

        app_telem_format_float(channel->max, scratch, (uint32_t)sizeof(scratch));
        hash = app_telem_hash_bytes(hash, scratch);
        hash = app_telem_hash_bytes(hash, "|");

        hash = app_telem_hash_bytes(hash, channel->group);
        hash = app_telem_hash_bytes(hash, "|");
        hash = app_telem_hash_bytes(hash, channel->param);
        hash = app_telem_hash_bytes(hash, "\n");
    }

    cached_hash   = hash;
    hash_is_valid = 1U;
    return cached_hash;
}

void APP_Telemetry_ReportHeader(void)
{
    APP_Control_QueueText("TELEM ver=%u n=%u rate=%u page=%u hash=%08lX "
                          "frame=%s contract=%u\r\n",
                          (unsigned int)APP_TELEM_SCHEMA_VERSION,
                          (unsigned int)APP_TELEM_CH_COUNT,
                          (unsigned int)APP_TELEM_RATE_HZ,
                          (unsigned int)APP_TELEM_PAGE_SIZE,
                          (unsigned long)APP_Telemetry_SchemaHash(),
                          APP_TELEM_BODY_FRAME_NAME,
                          (unsigned int)APP_TELEM_FRAME_CONTRACT_VERSION);
}

void APP_Telemetry_ReportPage(uint32_t from)
{
    char     min_text[32];
    char     max_text[32];
    uint32_t index;
    uint32_t last;
    long     next;

    if (from >= (uint32_t)APP_TELEM_CH_COUNT) {
        APP_Control_QueueText("ERR telem range from=%lu n=%u\r\n",
                              (unsigned long)from,
                              (unsigned int)APP_TELEM_CH_COUNT);
        return;
    }

    last = from + APP_TELEM_PAGE_SIZE;
    if (last > (uint32_t)APP_TELEM_CH_COUNT) {
        last = (uint32_t)APP_TELEM_CH_COUNT;
    }

    for (index = from; index < last; ++index) {
        const APP_TelemChannel *channel = &app_telem_channels[index];

        app_telem_format_float(channel->min, min_text, (uint32_t)sizeof(min_text));
        app_telem_format_float(channel->max, max_text, (uint32_t)sizeof(max_text));

        APP_Control_QueueText(
            "TELEM CH idx=%lu name=%s unit=%s min=%s max=%s grp=%s param=%s\r\n",
            (unsigned long)index,
            channel->name,
            channel->unit,
            min_text,
            max_text,
            channel->group,
            channel->param);
    }

    /* next=-1 表示已到表尾，上位机据此结束翻页循环。 */
    next = (last < (uint32_t)APP_TELEM_CH_COUNT) ? (long)last : -1L;
    APP_Control_QueueText("TELEM PAGE from=%lu count=%lu next=%ld\r\n",
                          (unsigned long)from,
                          (unsigned long)(last - from),
                          next);
}
