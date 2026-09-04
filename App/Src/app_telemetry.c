#include "app_telemetry.h"

#include "app_control.h"
#include "drv_frame_contract.h"

#include <stdio.h>
#include <stdlib.h>

#define APP_TELEM_BODY_FRAME_NAME "body_flu"
#define APP_TELEM_FRAME_CONTRACT_VERSION DRV_FRAME_CONTRACT_VERSION

/*
 * 通道表 —— 顺序必须与 Core/Src/freertos.c 中 VOFA_task 的填充顺序一致。
 * 填充代码用 APP_TELEM_CH_* 枚举名下标，因此"顺序一致"由枚举保证，
 * 本表只负责元数据。
 *
 * 关于增益通道的符号：freertos.c 在发送前会对 yaw_angle_kp / yaw_rate_kd /
 * roll_angle_kp / pitch_angle_kp 取反，让操作者看到正值，控制器内部保留
 * 实测符号。此处登记的是"上位机看到的值"的量程，与该约定一致。
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
    [APP_TELEM_CH_POS_Z_KI]        = {"pos_z_ki",        "-", "gain", 0.0f, 10.0f, "coax.pos_z_ki"},
    [APP_TELEM_CH_VEL_Z_KD]        = {"vel_z_kd",        "-", "gain", 0.0f, 10.0f, "coax.vel_z_kd"},

    [APP_TELEM_CH_FUSION_ACC_ERR]           = {"fusion_acc_err",         "deg",   "fusion", 0.0f,  180.0f, "-"},
    [APP_TELEM_CH_FUSION_ACC_IGNORED]       = {"fusion_acc_ignored",     "-",     "fusion", 0.0f,    1.0f, "-"},
    [APP_TELEM_CH_FUSION_ACC_RECOVERY]      = {"fusion_acc_recovery",    "-",     "fusion", 0.0f,    1.0f, "-"},
    [APP_TELEM_CH_FUSION_ACC_CORRECTIONS]   = {"fusion_acc_corrections", "count", "fusion", 0.0f, 1000.0f, "-"},
    [APP_TELEM_CH_FUSION_ACC_NORM_REJECTED] = {"fusion_acc_norm_rej",    "count", "fusion", 0.0f, 1000.0f, "-"},
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
