#include "app_servo_cal.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_led.h"
#include "app_servo_bus_guard.h"
#include "bsp_bus_servo.h"

#define APP_SERVO_CAL_CH_ROLL       0U
#define APP_SERVO_CAL_CH_PITCH      1U
#define APP_SERVO_CAL_CH_THROTTLE   2U
#define APP_SERVO_CAL_CH_YAW        3U

#define APP_SERVO_CAL_LOW_US        1150U
#define APP_SERVO_CAL_HIGH_US       1850U
#define APP_SERVO_CAL_HOLD_MS        800U
#define APP_SERVO_CAL_ACK_MS        1200U
#define APP_SERVO_CAL_ERROR_MS      1500U

static APP_ServoCalState servo_cal_state;
static uint32_t servo_cal_release_hold_start_ms;
static uint32_t servo_cal_save_hold_start_ms;
static uint32_t servo_cal_transient_start_ms;

/*
 * 本模块整条状态机跑在 500Hz 控制环（stabilizer_control_prepare）里，
 * 绝不能在这里同步调用 APP_Control_QueueText：那条路径会阻塞等 USB CDC
 * 发送（最坏 3x 超时）。事件文本先存进单条通告缓冲，由通信任务上下文的
 * APP_Control_Tick 通过 APP_ServoCal_TakeNotice() 取走后再排队发送。
 * 只保留最新一条：标定事件是人操作频率，覆盖旧通告可接受。
 */
static char servo_cal_notice_text[64];
static volatile uint8_t servo_cal_notice_pending;
static uint8_t servo_cal_pwm_notice_sent;

static void servo_cal_post_notice(const char *format, ...)
{
    va_list args;
    int written;

    va_start(args, format);
    written = vsnprintf(servo_cal_notice_text, sizeof(servo_cal_notice_text),
                        format, args);
    va_end(args);

    if (written > 0) {
        servo_cal_notice_pending = 1U;
    }
}

static uint8_t servo_cal_low(uint16_t value)
{
    return (value <= APP_SERVO_CAL_LOW_US) ? 1U : 0U;
}

static uint8_t servo_cal_high(uint16_t value)
{
    return (value >= APP_SERVO_CAL_HIGH_US) ? 1U : 0U;
}

static uint8_t servo_cal_safe_gate(const uint16_t ch[16],
                                   uint8_t rc_link_ok,
                                   uint8_t rc_arm_switch_high)
{
    if ((ch == NULL) || (rc_link_ok == 0U) || (rc_arm_switch_high != 0U)) {
        return 0U;
    }

    return servo_cal_low(ch[APP_SERVO_CAL_CH_THROTTLE]);
}

static uint8_t servo_cal_release_gesture(const uint16_t ch[16])
{
    return ((servo_cal_low(ch[APP_SERVO_CAL_CH_THROTTLE]) != 0U) &&
            (servo_cal_low(ch[APP_SERVO_CAL_CH_YAW]) != 0U) &&
            (servo_cal_low(ch[APP_SERVO_CAL_CH_PITCH]) != 0U) &&
            (servo_cal_high(ch[APP_SERVO_CAL_CH_ROLL]) != 0U)) ? 1U : 0U;
}

static uint8_t servo_cal_save_gesture(const uint16_t ch[16])
{
    return ((servo_cal_low(ch[APP_SERVO_CAL_CH_THROTTLE]) != 0U) &&
            (servo_cal_high(ch[APP_SERVO_CAL_CH_YAW]) != 0U) &&
            (servo_cal_low(ch[APP_SERVO_CAL_CH_PITCH]) != 0U) &&
            (servo_cal_low(ch[APP_SERVO_CAL_CH_ROLL]) != 0U)) ? 1U : 0U;
}

static void servo_cal_set_error(const char *op, uint8_t id, DRV_SERVO_Status status,
                                uint32_t now_ms)
{
    servo_cal_state = APP_SERVO_CAL_STATE_ERROR;
    servo_cal_transient_start_ms = now_ms;
    APP_LED_SetServoCalMode(APP_LED_SERVO_CAL_ERROR);
    servo_cal_post_notice("ERR servo_cal %s id=%u st=%u\r\n",
                          op,
                          (unsigned int)id,
                          (unsigned int)status);
}

static uint8_t servo_cal_release_all(uint32_t now_ms)
{
    DRV_SERVO_Status status;

    status = BSP_BusServo_ReleaseTorque(1U);
    if (status != DRV_SERVO_OK) {
        servo_cal_set_error("release", 1U, status, now_ms);
        return 0U;
    }

    status = BSP_BusServo_ReleaseTorque(2U);
    if (status != DRV_SERVO_OK) {
        servo_cal_set_error("release", 2U, status, now_ms);
        return 0U;
    }

    servo_cal_state = APP_SERVO_CAL_STATE_RELEASED;
    servo_cal_save_hold_start_ms = 0U;
    APP_LED_SetServoCalMode(APP_LED_SERVO_CAL_RELEASED);
    servo_cal_post_notice("OK servo_cal released\r\n");
    return 1U;
}

static uint8_t servo_cal_restore_all(uint32_t now_ms, uint8_t report_error)
{
    DRV_SERVO_Status status;

    status = BSP_BusServo_RestoreTorque(1U);
    if ((status != DRV_SERVO_OK) && (report_error != 0U)) {
        servo_cal_set_error("restore", 1U, status, now_ms);
        return 0U;
    }

    status = BSP_BusServo_RestoreTorque(2U);
    if ((status != DRV_SERVO_OK) && (report_error != 0U)) {
        servo_cal_set_error("restore", 2U, status, now_ms);
        return 0U;
    }

    return 1U;
}

static uint8_t servo_cal_save_startup_all(uint32_t now_ms)
{
    DRV_SERVO_Status status;

    status = BSP_BusServo_SetStartupPosition(1U);
    if (status != DRV_SERVO_OK) {
        servo_cal_set_error("save_startup", 1U, status, now_ms);
        return 0U;
    }

    status = BSP_BusServo_SetStartupPosition(2U);
    if (status != DRV_SERVO_OK) {
        servo_cal_set_error("save_startup", 2U, status, now_ms);
        return 0U;
    }

    if (servo_cal_restore_all(now_ms, 1U) == 0U) {
        return 0U;
    }

    servo_cal_state = APP_SERVO_CAL_STATE_SAVE_LOCK_ACK;
    servo_cal_transient_start_ms = now_ms;
    APP_LED_SetServoCalMode(APP_LED_SERVO_CAL_SAVE_ACK);
    servo_cal_post_notice("OK servo_cal startup_saved\r\n");
    return 1U;
}

void APP_ServoCal_Init(void)
{
    servo_cal_state = APP_SERVO_CAL_STATE_IDLE;
    servo_cal_release_hold_start_ms = 0U;
    servo_cal_save_hold_start_ms = 0U;
    servo_cal_transient_start_ms = 0U;
    servo_cal_notice_pending = 0U;
    servo_cal_pwm_notice_sent = 0U;
    servo_cal_notice_text[0] = '\0';
    APP_LED_SetServoCalMode(APP_LED_SERVO_CAL_NONE);
}

uint16_t APP_ServoCal_TakeNotice(char *out, uint16_t capacity)
{
    size_t length;

    if ((out == NULL) || (capacity == 0U) || (servo_cal_notice_pending == 0U)) {
        return 0U;
    }
    /* 先清标志再拷贝：拷贝期间控制环若写入新事件会重新置位，下个 Tick 取走。 */
    servo_cal_notice_pending = 0U;
    length = strlen(servo_cal_notice_text);
    if (length >= capacity) {
        length = (size_t)capacity - 1U;
    }
    memcpy(out, servo_cal_notice_text, length);
    out[length] = '\0';
    return (uint16_t)length;
}

APP_ServoCalResult APP_ServoCal_Step(const uint16_t ch[16],
                                     uint8_t rc_link_ok,
                                     uint8_t rc_arm_switch_high,
                                     uint32_t now_ms)
{
    if (APP_ServoBusGuard_IsPwmMode() != 0U) {
        uint8_t state_active =
            (servo_cal_state != APP_SERVO_CAL_STATE_IDLE) ? 1U : 0U;
        uint8_t gesture_active = 0U;

        servo_cal_release_hold_start_ms = 0U;
        servo_cal_save_hold_start_ms = 0U;
        if (servo_cal_state != APP_SERVO_CAL_STATE_IDLE) {
            servo_cal_state = APP_SERVO_CAL_STATE_IDLE;
            APP_LED_SetServoCalMode(APP_LED_SERVO_CAL_NONE);
        }
        if (servo_cal_safe_gate(ch, rc_link_ok, rc_arm_switch_high) != 0U) {
            gesture_active = (uint8_t)((servo_cal_release_gesture(ch) != 0U) ||
                                       (servo_cal_save_gesture(ch) != 0U));
        }
        if ((state_active == 0U) && (gesture_active == 0U)) {
            servo_cal_pwm_notice_sent = 0U;
        } else if (servo_cal_pwm_notice_sent == 0U) {
            servo_cal_post_notice("ERR servo_cal unsupported mode=pwm\r\n");
            servo_cal_pwm_notice_sent = 1U;
        }
        return APP_SERVO_CAL_RESULT_NONE;
    }
    servo_cal_pwm_notice_sent = 0U;

    if (servo_cal_state == APP_SERVO_CAL_STATE_SAVE_LOCK_ACK) {
        if ((now_ms - servo_cal_transient_start_ms) >= APP_SERVO_CAL_ACK_MS) {
            servo_cal_state = APP_SERVO_CAL_STATE_IDLE;
            APP_LED_SetServoCalMode(APP_LED_SERVO_CAL_NONE);
        }
        return APP_SERVO_CAL_RESULT_NONE;
    }

    if (servo_cal_state == APP_SERVO_CAL_STATE_ERROR) {
        if ((now_ms - servo_cal_transient_start_ms) >= APP_SERVO_CAL_ERROR_MS) {
            (void)servo_cal_restore_all(now_ms, 0U);
            servo_cal_state = APP_SERVO_CAL_STATE_IDLE;
            APP_LED_SetServoCalMode(APP_LED_SERVO_CAL_NONE);
        }
        return APP_SERVO_CAL_RESULT_NONE;
    }

    if (servo_cal_safe_gate(ch, rc_link_ok, rc_arm_switch_high) == 0U) {
        servo_cal_release_hold_start_ms = 0U;
        servo_cal_save_hold_start_ms = 0U;
        if (servo_cal_state == APP_SERVO_CAL_STATE_RELEASED) {
            (void)servo_cal_restore_all(now_ms, 0U);
            servo_cal_state = APP_SERVO_CAL_STATE_IDLE;
            APP_LED_SetServoCalMode(APP_LED_SERVO_CAL_NONE);
            servo_cal_post_notice("ERR servo_cal aborted\r\n");
            return APP_SERVO_CAL_RESULT_ERROR;
        }
        return APP_SERVO_CAL_RESULT_NONE;
    }

    if (servo_cal_state == APP_SERVO_CAL_STATE_IDLE) {
        if (servo_cal_release_gesture(ch) == 0U) {
            servo_cal_release_hold_start_ms = 0U;
            return APP_SERVO_CAL_RESULT_NONE;
        }

        if (servo_cal_release_hold_start_ms == 0U) {
            servo_cal_release_hold_start_ms = now_ms;
            return APP_SERVO_CAL_RESULT_NONE;
        }

        if ((now_ms - servo_cal_release_hold_start_ms) >= APP_SERVO_CAL_HOLD_MS) {
            servo_cal_release_hold_start_ms = 0U;
            return (servo_cal_release_all(now_ms) != 0U) ?
                   APP_SERVO_CAL_RESULT_RELEASED : APP_SERVO_CAL_RESULT_ERROR;
        }
        return APP_SERVO_CAL_RESULT_NONE;
    }

    if (servo_cal_state == APP_SERVO_CAL_STATE_RELEASED) {
        if (servo_cal_save_gesture(ch) == 0U) {
            servo_cal_save_hold_start_ms = 0U;
            return APP_SERVO_CAL_RESULT_NONE;
        }

        if (servo_cal_save_hold_start_ms == 0U) {
            servo_cal_save_hold_start_ms = now_ms;
            return APP_SERVO_CAL_RESULT_NONE;
        }

        if ((now_ms - servo_cal_save_hold_start_ms) >= APP_SERVO_CAL_HOLD_MS) {
            servo_cal_save_hold_start_ms = 0U;
            return (servo_cal_save_startup_all(now_ms) != 0U) ?
                   APP_SERVO_CAL_RESULT_SAVED : APP_SERVO_CAL_RESULT_ERROR;
        }
    }

    return APP_SERVO_CAL_RESULT_NONE;
}

uint8_t APP_ServoCal_IsActive(void)
{
    return (servo_cal_state != APP_SERVO_CAL_STATE_IDLE) ? 1U : 0U;
}

APP_ServoCalState APP_ServoCal_GetState(void)
{
    return servo_cal_state;
}
