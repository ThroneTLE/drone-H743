#ifndef APP_LED_CONFIG_H
#define APP_LED_CONFIG_H

#include <stdint.h>

#include "drv_rgb_led.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 状态灯的颜色绑定表：每一种"飞控想说的话"配一个图案，用户可改，存 Flash。
 *
 * 它是 `App/Src/app_led.c` 里那堆硬编码常量的唯一替代品。灯珠接在哪根脚、
 * 什么极性仍然是板级事实（`BSP/Src/bsp_rgb_led.c`），不在这里，也不该可配——
 * 极性写反不会让任何测试变红，只会让每个图案反过来读。
 *
 * ── 本结构体是 Flash ABI ──
 * 它整体嵌在 CFG 配置记录（`app_control_config_store.c`）里，和增益、遥控映射、
 * 机体模型同住一条记录。所以：
 *   * 枚举值**只能追加，不能重排**。重排会让已经存在用户板子上的配置整体错位到
 *     别的状态上，而且不报错——灯照亮，只是每一条都在说错话。
 *   * 结构体字段只能往 `reserved` 里吃，`size` 跟着变。
 */

#define APP_LED_CFG_MAGIC   0x4344454CUL   /* "LEDC" 小端 */
#define APP_LED_CFG_SCHEMA  1U

/*
 * 绑定的键是**语义状态**，不是 `SVC_LedSource`。
 *
 * 按源建表会把 CALIBRATION 下的 3 个状态、WARNING 下的 3 个状态各自压成一种
 * 颜色，等于把现有的区分度直接删掉——而那 6 条恰恰是最需要分开看的。
 *
 * 解锁被拒的 7 个原因各占一条（作者 2026-09-12 裁决：每种报错要能单独配色）。
 * 它们必须**连续**排在 BLOCK_BASE 之后，索引 = BLOCK_BASE + (原因码 - 1)。
 */
typedef enum {
    APP_LED_BIND_ARMED         = 0,   /* 已解锁 */
    APP_LED_BIND_READY         = 1,   /* 就绪，可以解锁 */
    APP_LED_BIND_HEARTBEAT     = 2,   /* 兜底心跳：控制环还没发过话 */
    APP_LED_BIND_CAL_RELEASED  = 3,   /* 舵机标定 · 扭矩已释放 */
    APP_LED_BIND_CAL_SAVE_ACK  = 4,   /* 舵机标定 · 保存成功 */
    APP_LED_BIND_CAL_ERROR     = 5,   /* 舵机标定 · 出错 */
    APP_LED_BIND_FLOW_STARTING = 6,   /* 光流启动中 */
    APP_LED_BIND_FLOW_RETRYING = 7,   /* 光流重试中 */
    APP_LED_BIND_FLOW_FAILED   = 8,   /* 光流失败 */
    /* ↓ 解锁被拒，顺序必须与 APP_LED_ArmBlockReason 的 1..7 一一对应 ↓ */
    APP_LED_BIND_BLOCK_NO_RC        = 9,
    APP_LED_BIND_BLOCK_RC_LOSS      = 10,
    APP_LED_BIND_BLOCK_ARM_SWITCH   = 11,
    APP_LED_BIND_BLOCK_THROTTLE     = 12,
    APP_LED_BIND_BLOCK_IMU          = 13,
    APP_LED_BIND_BLOCK_FRAME        = 14,
    APP_LED_BIND_BLOCK_AIRFRAME     = 15,
    APP_LED_BIND_COUNT              = 16
} APP_LedBindingId;

#define APP_LED_BIND_BLOCK_BASE  APP_LED_BIND_BLOCK_NO_RC
#define APP_LED_BIND_BLOCK_COUNT 7U

/*
 * 一条绑定。**没有 count 字段，这是刻意的。**
 *
 * 闪几下等于解锁被拒的原因码（`App/Inc/app_led.h` 的 APP_LED_ArmBlockReason），
 * 由 `app_led.c` 在运行期注入。把它做成可配置的话，现场数到 3 下、而 `ARM?`
 * 报 block=5，两边单看都自洽，排查的人会照着假原因一路查下去。
 * 不入库 = 结构上不可能对不上，而不是"我们记得别改它"。
 */
typedef struct {
    uint8_t  r;            /* +0 */
    uint8_t  g;            /* +1 */
    uint8_t  b;            /* +2 */
    uint8_t  effect;       /* +3  DRV_RgbEffect */
    uint16_t on_ms;        /* +4  BLINK / PULSES */
    uint16_t off_ms;       /* +6  BLINK / PULSES */
    uint16_t gap_ms;       /* +8  PULSES 组间停顿 */
    uint16_t period_ms;    /* +10 BREATHE 一个来回 */
    uint8_t  dim;          /* +12 BREATHE 谷底亮度 */
    uint8_t  reserved0;    /* +13 */
} APP_LedBinding;

typedef struct {
    uint32_t       magic;                          /* +0   */
    uint16_t       schema;                         /* +4   */
    uint16_t       size;                           /* +6   = sizeof(APP_LedConfig) */
    uint16_t       identify_hold_ms;               /* +8   点名/预览默认占灯时长 */
    uint8_t        customized;                     /* +10  0 = 从没被写过 */
    uint8_t        reserved0;                      /* +11  */
    APP_LedBinding binding[APP_LED_BIND_COUNT];    /* +12 .. +235 */
    uint32_t       generation;                     /* +236 上位机据此判"是否已生效" */
    uint32_t       reserved1[3];                   /* +240 .. +251 */
} APP_LedConfig;

/* 校验字段的取值上限。与 `app_cmd_led.c` 的 LED BLINK/BREATHE 共用同一份，
 * 别两处各写一遍——分叉之后两条命令对同一个值一个收一个拒。 */
#define APP_LED_CFG_MAX_TIME_MS   60000U
#define APP_LED_CFG_MIN_PERIOD_MS 100U
#define APP_LED_CFG_MAX_DIM       200U
/*
 * PULSES 的组间停顿下限。数灯靠这段黑断句，太短的话 7 下和 6 下肉眼分不开，
 * 而固件照样"正常工作"——这正是一条会说谎的诊断。
 */
#define APP_LED_CFG_MIN_GAP_MS    400U

void APP_LedConfig_Defaults(APP_LedConfig *config);

/*
 * 校验的是"这份配置拿出去会不会骗人 / 会不会喂给驱动一个它要替你改掉的值"，
 * 不是"好不好看"。失败时调用方**整份**落回 Defaults，不做部分修复：
 * 半份用户值半份默认值，比两者都难排查。
 */
uint8_t APP_LedConfig_Validate(const APP_LedConfig *config);

void     APP_LedConfig_ResetActive(void);
uint8_t  APP_LedConfig_PublishActive(const APP_LedConfig *config);
uint8_t  APP_LedConfig_ReadActive(APP_LedConfig *config);
uint32_t APP_LedConfig_GetActiveGeneration(void);

/*
 * 取一条绑定解析好的图案。
 *
 * 只给"按 ID 取一条"，**不给"按值读整份"**：LED 任务栈 1 KB，而 APP_LedConfig
 * 有 252 字节。读整份要在栈上放一个副本，而那个任务已经因为按值取
 * APP_OPTICAL_FLOW_Status 而余量吃紧过一次（128 字时只剩 18 字）。
 *
 * 本函数**永远不写 `out->count`**——那是调用方按原因码注入的。
 */
void APP_LedConfig_GetPattern(uint8_t binding_id, DRV_RgbPattern *out);

/* 解锁被拒原因码（1..7）→ 绑定 ID。越界返回 APP_LED_BIND_COUNT。 */
uint8_t APP_LedConfig_BindingForBlockReason(uint8_t reason);

/*
 * 这条绑定用 PULSES 时该闪几下。**运行期常量，永远不来自配置。**
 *
 * 全仓库只此一份。以前这条规则是散在调用方的一句 `id >= BLOCK_BASE` 偏移算式，
 * 于是 `flow_failed`（PULSES，改造前固定 3 下）落在算式之外拿到 count=0，
 * 而 `drv_rgb_led.c` 的 PULSES 遇到 count=0 **直接返回黑**——光流失败时整盏灯
 * 不亮，且因为 WARNING 优先级高于 STATUS/HEARTBEAT，连心跳都被盖住。
 * 现象与"固件死了/板子没电"完全一样，而这正是状态灯要排除的那种假象。
 *
 * 返回 0 表示"这条数不出次数"，此时 Validate 拒绝把它配成 PULSES——
 * 否则用户能配出一条永久黑灯，而三处显示（灯、LEDMAP?、上位机波形）各说各话。
 */
uint8_t APP_LedConfig_PulseCount(uint8_t binding_id);

/* 名字 ↔ ID。**全仓库只此一份**：命令族、日志、上位机文档都引用它。
 * 分叉的症状是 `LEDMAP?` 报 bind=cal_save_ack、而 SET 同一个名字回 bad_bind。 */
const char *APP_LedConfig_BindName(uint8_t binding_id);
uint8_t     APP_LedConfig_BindFromName(const char *name);  /* 不认识返回 COUNT */
const char *APP_LedConfig_EffectName(uint8_t effect);
uint8_t     APP_LedConfig_EffectFromName(const char *name); /* 不认识返回 0xFF */

/*
 * 两条绑定在眼睛里是不是同一个东西。量化后比较，不是逐字节相等：
 * 差 3 个灰阶的两种红在板子上分不出来，而逐字节比较会说它们不同。
 */
uint8_t APP_LedConfig_SameSignature(const APP_LedBinding *a, const APP_LedBinding *b);

#ifdef __cplusplus
}
#endif

#endif /* APP_LED_CONFIG_H */
