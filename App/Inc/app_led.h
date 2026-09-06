#ifndef APP_LED_H
#define APP_LED_H

#include <stdint.h>

/*
 * 解锁被拒的原因。数值即 LED_3 的闪烁次数，所以**只能追加，不能重排**——
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
} APP_LED_ArmBlockReason;

typedef enum {
    APP_LED_SERVO_CAL_NONE = 0,
    APP_LED_SERVO_CAL_RELEASED = 1,
    APP_LED_SERVO_CAL_SAVE_ACK = 2,
    APP_LED_SERVO_CAL_ERROR = 3,
} APP_LED_ServoCalMode;

void APP_LED_Task_Init(void);
void APP_LED_Task_Step(void);
void APP_LED_SetArmStatus(uint8_t armed, APP_LED_ArmBlockReason reason);
void APP_LED_SetServoCalMode(APP_LED_ServoCalMode mode);

#endif
