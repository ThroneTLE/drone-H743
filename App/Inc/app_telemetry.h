#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include <stdint.h>

/*
 * VOFA 遥测通道表 —— "第 N 个 float 是什么" 的单一事实源。
 *
 * 在此之前，通道映射同时存在于三处：Core/Src/freertos.c 的填充代码、
 * VOFA_task 头部的注释、以及上位机工程文件。三者没有任何机制保证一致，
 * 插入一个通道就会让上位机所有曲线静默错位（不报错，只是数值"有点怪"）。
 *
 * 现在：索引由下面的枚举给出，元数据由 app_telemetry.c 的表给出，
 * 上位机通过 TELEM? 查询自动建表。填充代码用枚举名下标，编译期断言
 * 锁死表长与 VOFA 帧长一致。
 */

/*
 * 通道索引。填充顺序即上位机看到的 float 顺序，不要重排 —— 插入新通道
 * 一律追加到 COUNT 之前，否则历史日志与新固件的通道号会错位。
 */
typedef enum {
    APP_TELEM_CH_ROLL = 0,          /* 横滚角 */
    APP_TELEM_CH_PITCH,             /* 俯仰角 */
    APP_TELEM_CH_YAW,               /* 偏航角 */
    APP_TELEM_CH_FLOW_HEIGHT,       /* 二合一光流测距高度，无效时为 0 */
    APP_TELEM_CH_TIME,              /* 飞控运行时间戳 */
    APP_TELEM_CH_VEL_EST_X,         /* 融合 X 速度估计 */
    APP_TELEM_CH_VEL_EST_Y,         /* 融合 Y 速度估计 */
    APP_TELEM_CH_ROLL_RATE_KD,      /* 以下为参数回显，供上位机滑块反馈 */
    APP_TELEM_CH_PITCH_RATE_KD,
    APP_TELEM_CH_YAW_ANGLE_KP,
    APP_TELEM_CH_YAW_RATE_KD,
    APP_TELEM_CH_POS_X_KP,
    APP_TELEM_CH_POS_Y_KP,
    APP_TELEM_CH_VEL_X_KD,
    APP_TELEM_CH_VEL_Y_KD,
    APP_TELEM_CH_POS_EST_X,         /* 融合 X 位置估计 */
    APP_TELEM_CH_POS_EST_Y,         /* 融合 Y 位置估计 */
    APP_TELEM_CH_VEL_LOOP_ENABLE,   /* 速度环使能标志 */
    APP_TELEM_CH_ROLL_ANGLE_KP,
    APP_TELEM_CH_PITCH_ANGLE_KP,
    APP_TELEM_CH_POS_Z_KP,
    APP_TELEM_CH_POS_Z_KI,
    APP_TELEM_CH_VEL_Z_KD,
    APP_TELEM_CH_FUSION_ACC_ERR,    /* 以下为 Fusion 加速度拒绝/恢复诊断 */
    APP_TELEM_CH_FUSION_ACC_IGNORED,
    APP_TELEM_CH_FUSION_ACC_RECOVERY,
    APP_TELEM_CH_FUSION_ACC_CORRECTIONS,
    APP_TELEM_CH_FUSION_ACC_NORM_REJECTED,
    APP_TELEM_CH_COUNT
} APP_TelemChannelId;

/*
 * 通道号是掩码帧里的位号（doc/telemetry-scope-plan.md §2.2 的 u64 mask），
 * 所以表长有硬上限。超过 64 路要先升帧格式版本，不是悄悄加一条。
 */
_Static_assert((int)APP_TELEM_CH_COUNT <= 64,
               "telemetry mask is u64: adding a 65th channel needs a frame version bump");

/* 遥测帧的标称周期与速率。app_telem_stream.c 用它做上电默认值。 */
#define APP_TELEM_PERIOD_MS 25U
#define APP_TELEM_RATE_HZ   (1000U / APP_TELEM_PERIOD_MS)

/*
 * 协议版本。改动 TELEM 回包格式（而非通道内容）时递增。
 * v2（R-T1-1）：通道行新增 `param=`，SchemaHash 覆盖该字段。
 * v3（坐标纠错）：表头新增 `frame=body_flu contract=<version>`，轴向通道
 * 统一为规范 FLU，并把坐标来源纳入 SchemaHash。
 */
#define APP_TELEM_SCHEMA_VERSION 3U

/* 单次 TELEM CH 请求最多回几条通道行。
 *
 * uartTxQueue 深度 32 且满时丢弃最旧的一条，一次性把 28 条通道推进队列会
 * 在链路慢时丢掉开头几条。分页把单次请求的回包压到 PAGE_SIZE+1 条。
 */
#define APP_TELEM_PAGE_SIZE 6U

typedef struct {
    const char *name;   /* 通道标识，上位机据此绑定，禁止含空格 */
    const char *unit;   /* 单位文本，无量纲用 "-" */
    const char *group;  /* 上位机分组标题 */
    float       min;    /* 标称量程下限，供仪表盘刻度使用 */
    float       max;    /* 标称量程上限 */
    /*
     * 该通道回显的是哪个参数（`DRV_COAX_CTRL_*Param` 的键），没有参数的通道写 "-"。
     * 两件事靠它：上位机据此**数据驱动**地生成滑块（不再在上位机里硬编一张增益表），
     * 遥测流据此判定"这条通道要按变化回显、平时不占带宽"。
     */
    const char *param;
} APP_TelemChannel;

/* 该通道是否为参数回显通道（param 非空且不是 "-"）。 */
uint8_t APP_Telemetry_ChannelHasParam(uint32_t index);

/* 通道数量（等于 APP_TELEM_CH_COUNT，供不便包含枚举的调用方使用）。 */
uint32_t APP_Telemetry_ChannelCount(void);

/* 取第 index 个通道的元数据；index 越界返回 NULL。 */
const APP_TelemChannel *APP_Telemetry_GetChannel(uint32_t index);

/*
 * 通道表指纹。上位机缓存 "hash -> 已建好的仪表盘"，重连时只比对 hash，
 * 未变化则完全跳过重建。表内容任何改动都会改变该值。
 */
uint32_t APP_Telemetry_SchemaHash(void);

/* 回复 TELEM? ：仅一行表头。 */
void APP_Telemetry_ReportHeader(void);

/* 回复 TELEM CH from=<n> ：至多 APP_TELEM_PAGE_SIZE 条通道行 + 一行页脚。 */
void APP_Telemetry_ReportPage(uint32_t from);

#endif /* APP_TELEMETRY_H */
