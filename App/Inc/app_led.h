#ifndef APP_LED_H
#define APP_LED_H

#include <stddef.h>
#include <stdint.h>

#include "drv_rgb_led.h"

/*
 * 解锁被拒的原因。数值即琥珀色的闪烁次数，所以**只能追加，不能重排**——
 * 现场是靠数闪几下来判断的，改了编号等于把所有人的记忆作废。
 *
 * 每个能让 stabilizer_rc_update_armed() 拒绝解锁的条件，都必须在这里有一个
 * 对应值，并在 stabilizer_publish_led() 的原因链里有一条分支。少一条的后果
 * 不是"没有提示"，而是**掉进后面的分支报出一个假原因**——排查的人会照着
 * 假原因去查，越查越远。tests/test_led_status_contract.py 有机检把关。
 */
typedef enum {
    APP_LED_ARM_BLOCK_NONE = 0,
    APP_LED_ARM_BLOCK_NO_RC = 1,
    APP_LED_ARM_BLOCK_RC_LOSS = 2,
    APP_LED_ARM_BLOCK_ARM_SWITCH = 3,
    APP_LED_ARM_BLOCK_THROTTLE_HIGH = 4,
    APP_LED_ARM_BLOCK_IMU = 5,
    /* 坐标系运行时迁移未完成（DRV_FRAME_RUNTIME_MIGRATION_COMPLETE == 0）
     * 且已激活 FLU 朝向，或 IMU/舵机标定候选未提交。见 R-F6 工单。 */
    APP_LED_ARM_BLOCK_FRAME = 6,
    /*
     * Flash 里没有有效机体模型（DRV_Airframe_IsValid() == 0）。刚烧完固件、
     * 还没从上位机写过机体数据的板子就是这个状态：没有质量、没有惯量、没有
     * 力臂，控制律算出来的每个力矩都没有物理含义。缺哪一项由
     * DRV_Airframe_FirstInvalidName() 具名报出，不用数闪灯猜。
     */
    APP_LED_ARM_BLOCK_AIRFRAME = 7,
} APP_LED_ArmBlockReason;

typedef enum {
    APP_LED_SERVO_CAL_NONE = 0,
    APP_LED_SERVO_CAL_RELEASED = 1,
    APP_LED_SERVO_CAL_SAVE_ACK = 2,
    APP_LED_SERVO_CAL_ERROR = 3,
} APP_LED_ServoCalMode;

typedef struct {
    DRV_RgbColor color;         /* 此刻算出来的颜色（未经调制） */
    uint8_t      source;        /* SVC_LedSource；== SVC_LED_SOURCE_COUNT 表示没人说话 */
    uint8_t      armed;
    uint8_t      arm_published; /* 控制环是否已经发布过解锁状态 */
    uint8_t      block_reason;
    uint8_t      servo_cal;
    uint32_t     ticks;         /* 灯的节拍计数，用来区分"没话说"和"根本没在跑" */
    uint8_t      active_low;    /* 板级极性，报出来是为了让"看到的"和"写的"能对账 */
} APP_LED_Debug;

void APP_LED_Task_Init(void);
void APP_LED_Task_Step(void);
void APP_LED_SetArmStatus(uint8_t armed, APP_LED_ArmBlockReason reason);
void APP_LED_SetServoCalMode(APP_LED_ServoCalMode mode);

/* 上位机点名：hold_ms 之后自动交还，免得测试完忘记恢复把灯占死。 */
void APP_LED_Identify(const DRV_RgbColor *color, uint32_t hold_ms);
void APP_LED_IdentifyPattern(const DRV_RgbPattern *pattern, uint32_t hold_ms);
void APP_LED_GetDebug(APP_LED_Debug *debug);

#endif
