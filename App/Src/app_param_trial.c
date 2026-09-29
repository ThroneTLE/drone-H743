/*
 * app_param_trial.c —— 控制增益试用记录（契约见 app_param_trial.h）。
 *
 * 记录的是"名字 → 试用前的值"，保存时按名字把 Flash 记录里对应字段换回去。名字到字段
 * 的对应由下面的名单给出：可试用的名字恰好都与 Flash 块里的字段同名（去掉 "coax." 前缀），
 * 所以两边用同一个记号生成，字段写错编译就过不去。字段分在两块里：增益块
 * （APP_ControlCoaxTunableParams）与 v24 的指令整形/出口陷波块（APP_ControlCoaxShapingParams）。
 * 名单与驱动具名表是否一致由 tests/test_param_trial.py 对照两份源码检查。
 *
 * 任务归属：记录只在命令任务里被改（SYSID PARAM / PARAM SET / LOAD / DEFAULTS 都是命令），
 * 保存（自动保存拍、SAVE、各 COMMIT）也在同一个命令任务里串行执行。增删与保存时的
 * 读取仍包在极短临界区里：将来保存挪进别的任务，也读不到半条记录。
 */

#include "app_param_trial.h"

#include "app_control.h"
#include "bsp_critical.h"
#include "drv_coax_ctrl.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    PARAM_TRIAL_BLOCK_GAINS = 0,   /* APP_ControlCoaxTunableParams */
    PARAM_TRIAL_BLOCK_SHAPING      /* APP_ControlCoaxShapingParams（CFG v24 起，v25 加第二级陷波） */
} ParamTrialBlock;

typedef struct {
    const char *name;
    uint8_t block;     /* ParamTrialBlock */
    uint16_t offset;   /* 在该块里的字节偏移 */
} ParamTrialField;

#define PARAM_TRIAL_GAIN(field) \
    { "coax." #field, (uint8_t)PARAM_TRIAL_BLOCK_GAINS, \
      (uint16_t)offsetof(APP_ControlCoaxTunableParams, field) }
#define PARAM_TRIAL_SHAPING(field) \
    { "coax." #field, (uint8_t)PARAM_TRIAL_BLOCK_SHAPING, \
      (uint16_t)offsetof(APP_ControlCoaxShapingParams, field) }

static const ParamTrialField param_trial_fields[] = {
    PARAM_TRIAL_GAIN(att_roll_kp),
    PARAM_TRIAL_GAIN(att_pitch_kp),
    PARAM_TRIAL_GAIN(att_yaw_kp),
    PARAM_TRIAL_GAIN(rate_roll_kp),
    PARAM_TRIAL_GAIN(rate_pitch_kp),
    PARAM_TRIAL_GAIN(rate_yaw_kp),
    PARAM_TRIAL_GAIN(rate_roll_ki),
    PARAM_TRIAL_GAIN(rate_pitch_ki),
    PARAM_TRIAL_GAIN(rate_yaw_ki),
    PARAM_TRIAL_GAIN(rate_roll_kd),
    PARAM_TRIAL_GAIN(rate_pitch_kd),
    PARAM_TRIAL_GAIN(rate_yaw_kd),
    PARAM_TRIAL_GAIN(rate_roll_i_limit_n_m),
    PARAM_TRIAL_GAIN(rate_pitch_i_limit_n_m),
    PARAM_TRIAL_GAIN(rate_yaw_i_limit_n_m),
    PARAM_TRIAL_GAIN(rate_roll_ff),
    PARAM_TRIAL_GAIN(rate_pitch_ff),
    PARAM_TRIAL_GAIN(rate_yaw_ff),
    /* 高度环 z 通道（R-ALTID-1，光杆台架高度辨识验证用），同在增益块里。 */
    PARAM_TRIAL_GAIN(pos_z_kp),
    PARAM_TRIAL_GAIN(vel_z_kp),
    PARAM_TRIAL_GAIN(vel_z_ki),
    PARAM_TRIAL_GAIN(vel_z_kd),
    PARAM_TRIAL_GAIN(vel_z_i_limit_m_s2),
    PARAM_TRIAL_SHAPING(rate_out_notch_hz),
    PARAM_TRIAL_SHAPING(rate_out_notch_q),
    PARAM_TRIAL_SHAPING(rate_out_notch2_hz),
    PARAM_TRIAL_SHAPING(rate_out_notch2_q),
    PARAM_TRIAL_SHAPING(att_ref_wr_rad_s),
    PARAM_TRIAL_SHAPING(att_ref_delay_ms),
};

#define PARAM_TRIAL_FIELD_COUNT \
    ((uint32_t)(sizeof(param_trial_fields) / sizeof(param_trial_fields[0])))

/* 容量 = 名单长度，每个可试用名字都放得下。测试装置用 -D 调小它，专门验"满了就拒绝"。 */
#ifndef APP_PARAM_TRIAL_CAPACITY
#define APP_PARAM_TRIAL_CAPACITY PARAM_TRIAL_FIELD_COUNT
#endif

_Static_assert(sizeof(param_trial_fields) / sizeof(param_trial_fields[0]) <= 255U,
               "record keeps the field index in a uint8_t");

typedef struct {
    uint8_t field;   /* param_trial_fields 下标 */
    float saved;     /* 试用前的值：保存时写进 Flash 的就是它 */
} ParamTrialRecord;

static ParamTrialRecord param_trial_records[APP_PARAM_TRIAL_CAPACITY];
static uint32_t param_trial_count;

static int32_t param_trial_field_index(const char *name)
{
    if (name == NULL) {
        return -1;
    }
    for (uint32_t i = 0U; i < PARAM_TRIAL_FIELD_COUNT; ++i) {
        if (strcmp(name, param_trial_fields[i].name) == 0) {
            return (int32_t)i;
        }
    }
    return -1;
}

static int32_t param_trial_find(int32_t field)
{
    for (uint32_t i = 0U; i < param_trial_count; ++i) {
        if ((int32_t)param_trial_records[i].field == field) {
            return (int32_t)i;
        }
    }
    return -1;
}

/* 按插入顺序前移补位，查询输出的先后因此就是试用的先后。 */
static void param_trial_remove(uint32_t slot)
{
    const uint32_t state = BSP_Critical_Enter();

    for (uint32_t i = slot + 1U; i < param_trial_count; ++i) {
        param_trial_records[i - 1U] = param_trial_records[i];
    }
    param_trial_count--;
    BSP_Critical_Exit(state);
}

static uint8_t param_trial_same(float a, float b)
{
    const float scale = fmaxf(1.0f, fmaxf(fabsf(a), fabsf(b)));

    return (fabsf(a - b) <= (1.0e-6f * scale)) ? 1U : 0U;
}

/* newlib-nano 没有浮点 printf：按百万分之一拼十进制，与 SYSID PARAM 回显同一写法。 */
static void param_trial_format(float value, char *out, size_t size)
{
    const uint8_t negative = (value < 0.0f) ? 1U : 0U;
    double magnitude = negative ? -(double)value : (double)value;
    unsigned long long micro;

    if (!(magnitude < 1.0e12)) {
        magnitude = 1.0e12;   /* 非有限或大得离谱：别让整数转换溢出 */
    }
    micro = (unsigned long long)(magnitude * 1000000.0 + 0.5);
    (void)snprintf(out, size, "%s%lu.%06lu", negative ? "-" : "",
                   (unsigned long)(micro / 1000000ULL),
                   (unsigned long)(micro % 1000000ULL));
}

APP_ParamTrialStatus APP_ParamTrial_Apply(const char *name, float value, float *applied)
{
    const int32_t field = param_trial_field_index(name);
    int32_t slot;
    float before;
    float now;

    if (field < 0) {
        return APP_PARAM_TRIAL_NOT_TRIALABLE;
    }
    slot = param_trial_find(field);
    if ((slot < 0) && (param_trial_count >= APP_PARAM_TRIAL_CAPACITY)) {
        /* 宁可拒绝这次试用，也不能少记一条：少记的那个试用值下次保存就进了 Flash。 */
        return APP_PARAM_TRIAL_FULL;
    }
    if ((isfinite(value) == 0) ||
        (DRV_COAX_CTRL_GetParam(name, &before) == 0U) ||
        (DRV_COAX_CTRL_SetParam(name, value) == 0U) ||
        (DRV_COAX_CTRL_GetParam(name, &now) == 0U)) {
        return APP_PARAM_TRIAL_REJECTED;
    }

    if (slot >= 0) {
        if (param_trial_same(now, param_trial_records[slot].saved) != 0U) {
            param_trial_remove((uint32_t)slot);   /* 写回了原值：试用结束 */
        }
    } else if (param_trial_same(now, before) == 0U) {
        const uint32_t state = BSP_Critical_Enter();

        param_trial_records[param_trial_count].field = (uint8_t)field;
        param_trial_records[param_trial_count].saved = before;
        param_trial_count++;
        BSP_Critical_Exit(state);
    }

    if (applied != NULL) {
        *applied = now;
    }
    return APP_PARAM_TRIAL_OK;
}

void APP_ParamTrial_Clear(const char *name)
{
    const int32_t field = param_trial_field_index(name);
    const int32_t slot = (field < 0) ? -1 : param_trial_find(field);

    if (slot >= 0) {
        param_trial_remove((uint32_t)slot);
    }
}

void APP_ParamTrial_ClearAll(void)
{
    const uint32_t state = BSP_Critical_Enter();

    param_trial_count = 0U;
    BSP_Critical_Exit(state);
}

uint32_t APP_ParamTrial_Count(void)
{
    return param_trial_count;
}

void APP_ParamTrial_RestorePersistent(APP_ControlCoaxTunableParams *gains,
                                      APP_ControlCoaxShapingParams *shaping)
{
    uint32_t state;

    /* 至多二十来次 4 字节拷贝；不在栈上另拷一份表——调用方 Save 的栈帧里已压着两条整记录。 */
    state = BSP_Critical_Enter();
    for (uint32_t i = 0U; i < param_trial_count; ++i) {
        const ParamTrialField *field = &param_trial_fields[param_trial_records[i].field];
        uint8_t *block = (field->block == (uint8_t)PARAM_TRIAL_BLOCK_SHAPING) ?
                         (uint8_t *)shaping : (uint8_t *)gains;

        if (block != NULL) {
            memcpy(block + field->offset, &param_trial_records[i].saved,
                   sizeof(param_trial_records[i].saved));
        }
    }
    BSP_Critical_Exit(state);
}

void APP_ParamTrial_Report(void)
{
    APP_Control_QueueText("SYSID TRIAL n=%lu\r\n", (unsigned long)param_trial_count);
    for (uint32_t i = 0U; i < param_trial_count; ++i) {
        const char *name = param_trial_fields[param_trial_records[i].field].name;
        char ram_text[24];
        char saved_text[24];
        float ram = 0.0f;

        (void)DRV_COAX_CTRL_GetParam(name, &ram);
        param_trial_format(ram, ram_text, sizeof(ram_text));
        param_trial_format(param_trial_records[i].saved, saved_text, sizeof(saved_text));
        APP_Control_QueueText("SYSID TRIAL name=%s ram=%s saved=%s\r\n",
                              name, ram_text, saved_text);
    }
}
