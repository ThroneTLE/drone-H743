/*
 * ARM 命令族：把"能不能解锁、为什么不能"如实报给上位机。
 *
 * 为什么值得单独一条报文：解锁被拒时，飞控只有一个 LED 闪灯次数可以表达原因，
 * 而闪灯要数、要查表、在室外还看不清。更麻烦的是原因链是**有序**的——它只报
 * 第一条不满足的条件，同时缺两样时修完一个仍然解不了锁，很容易让人以为没修对。
 *
 * 所以这条报文同时给出两样东西：
 *   1. block=<原因>   —— 与 LED 闪灯、与固件内部判定**同一个**值，不另算一遍；
 *   2. 每项条件的通过与否 —— 上位机据此一次列全清单，不用逐条试。
 *
 * D5-3：控制环还没跑过一圈时（published=0），这里报 block=unknown 而不是
 * block=none。"还不知道"和"没问题"是两回事，后者会让人以为已经可以解锁了。
 */

#include "app_control.h"
#include "app_control_internal.h"

#include "app_led.h"
#include "app_stabilizer.h"
#include "drv_airframe_params.h"

#include <stddef.h>
#include <string.h>

static const char *arm_block_name(uint8_t reason)
{
    switch ((APP_LED_ArmBlockReason)reason) {
    case APP_LED_ARM_BLOCK_NONE:          return "none";
    case APP_LED_ARM_BLOCK_NO_RC:         return "no_rc";
    case APP_LED_ARM_BLOCK_RC_LOSS:       return "rc_loss";
    case APP_LED_ARM_BLOCK_ARM_SWITCH:    return "arm_switch";
    case APP_LED_ARM_BLOCK_THROTTLE_HIGH: return "throttle_high";
    case APP_LED_ARM_BLOCK_IMU:           return "imu";
    case APP_LED_ARM_BLOCK_FRAME:         return "frame_migration";
    case APP_LED_ARM_BLOCK_AIRFRAME:      return "airframe";
    default:                              return "unknown";
    }
}

void app_control_report_arm(void)
{
    APP_Stabilizer_ArmStatus status;
    const char *missing;

    APP_Stabilizer_GetArmStatus(&status);

    /*
     * 机体模型缺哪一项，直接问机体模型自己，不在这里另抄一份判定规则。
     * blinks 就是 LED_3 的闪烁次数，报出来是为了让"数灯"和"看屏"能对上账。
     */
    missing = DRV_Airframe_FirstInvalidName();

    APP_Control_QueueText(
        "RSP id=0 mod=ARM op=STATUS armed=%u block=%s blinks=%u known=%u "
        "rc_seen=%u rc_ok=%u switch=%u throttle_low=%u imu=%u imu_health=%u "
        "frame=%u airframe=%u servo_cal_idle=%u accept_idle=%u "
        "airframe_missing=%s t_ms=%lu\r\n",
        (unsigned int)status.armed,
        (status.published != 0U) ? arm_block_name(status.block_reason) : "unknown",
        (unsigned int)status.block_reason,
        (unsigned int)status.published,
        (unsigned int)status.rc_link_seen,
        (unsigned int)status.rc_link_ok,
        (unsigned int)status.arm_switch_high,
        (unsigned int)status.throttle_low,
        (unsigned int)status.imu_control_valid,
        (unsigned int)status.imu_health_ok,
        (unsigned int)status.frame_migration_ok,
        (unsigned int)status.airframe_valid,
        (unsigned int)status.servo_cal_idle,
        (unsigned int)status.acceptance_idle,
        (missing != NULL) ? missing : "-",
        (unsigned long)status.now_ms);
}

uint8_t app_control_req_arm(uint32_t id, const char *mod, const char *op)
{
    if ((mod == NULL) || (strcmp(mod, "ARM") != 0)) {
        return 0U;
    }

    if ((op != NULL) && (strcmp(op, "STATUS") != 0)) {
        APP_Control_QueueText("ERR id=%lu mod=ARM op=%s code=BAD_OP\r\n",
                              (unsigned long)id, op);
        return 1U;
    }

    /*
     * 只读。这里**没有**也不会有 ARM/DISARM 操作：解锁必须是遥控器上一个真实的
     * 低到高拨杆动作，这样才保证下指令的人看得见飞机。一条能从上位机解锁的命令
     * 会绕过那个前提，而绕过之后没有任何办法补回来。
     */
    app_control_report_arm();
    return 1U;
}
