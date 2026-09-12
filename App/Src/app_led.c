#include "app_led.h"

#include "app_aiwb2.h"
#include "app_optical_flow.h"
#include "bsp_led.h"

#include "svc_timestamp.h"
static volatile uint8_t app_led_armed;
static volatile APP_LED_ArmBlockReason app_led_arm_block_reason =
    APP_LED_ARM_BLOCK_NO_RC;
static volatile APP_LED_ServoCalMode app_led_servo_cal_mode =
    APP_LED_SERVO_CAL_NONE;

static uint8_t app_led_group_blink(uint32_t now_ms, uint8_t count)
{
    const uint32_t slot_ms = 160U;
    const uint32_t gap_ms = 760U;
    uint32_t cycle_ms;
    uint32_t phase_ms;
    uint32_t slot;

    if (count == 0U) {
        return 0U;
    }

    cycle_ms = ((uint32_t)count * 2U * slot_ms) + gap_ms;
    phase_ms = now_ms % cycle_ms;
    if (phase_ms >= ((uint32_t)count * 2U * slot_ms)) {
        return 0U;
    }

    slot = phase_ms / slot_ms;
    return ((slot & 1U) == 0U) ? 1U : 0U;
}

static void app_led_write(BSP_LED_ID id, uint8_t on)
{
    if (on != 0U) {
        BSP_LED_On(id);
    } else {
        BSP_LED_Off(id);
    }
}

void APP_LED_Task_Init(void)
{
    BSP_LED_Off(LED_RED);
    BSP_LED_Off(LED_2);
    BSP_LED_Off(LED_3);
    BSP_LED_Off(LED_4);
}

void APP_LED_Task_Step(void)
{
    APP_OPTICAL_FLOW_Status flow_status;
    uint32_t now_ms = SVC_Timestamp_Ms();
    uint32_t heartbeat_period_ms;
    uint8_t flow_led_on = 0U;
    APP_LED_ArmBlockReason reason = app_led_arm_block_reason;
    APP_LED_ServoCalMode servo_cal_mode = app_led_servo_cal_mode;

    APP_OpticalFlow_GetStatus(&flow_status);

    if (servo_cal_mode == APP_LED_SERVO_CAL_RELEASED) {
        uint8_t on = (((now_ms / 120U) & 1U) == 0U) ? 1U : 0U;
        app_led_write(LED_RED, on);
        app_led_write(LED_2, on);
        app_led_write(LED_3, on);
        app_led_write(LED_4, on);
        return;
    }

    if (servo_cal_mode == APP_LED_SERVO_CAL_SAVE_ACK) {
        uint8_t on = (((now_ms / 320U) & 1U) == 0U) ? 1U : 0U;
        app_led_write(LED_RED, on);
        app_led_write(LED_2, on);
        app_led_write(LED_3, on);
        app_led_write(LED_4, on);
        return;
    }

    if (servo_cal_mode == APP_LED_SERVO_CAL_ERROR) {
        app_led_write(LED_RED, (((now_ms / 80U) & 1U) == 0U) ? 1U : 0U);
        app_led_write(LED_2, 0U);
        app_led_write(LED_3, 0U);
        app_led_write(LED_4, 0U);
        return;
    }

    heartbeat_period_ms = (app_led_armed != 0U) ? 120U :
                          ((APP_AiWB2_IsTransparent() != 0U) ? 250U : 500U);
    app_led_write(LED_RED, (((now_ms / heartbeat_period_ms) & 1U) == 0U) ? 1U : 0U);

    switch (flow_status.health) {
    case APP_OPTICAL_FLOW_HEALTH_OK:
        flow_led_on = 1U;
        break;
    case APP_OPTICAL_FLOW_HEALTH_STARTING:
        flow_led_on = (((now_ms / 500U) & 1U) == 0U) ? 1U : 0U;
        break;
    case APP_OPTICAL_FLOW_HEALTH_RETRYING:
        flow_led_on = (((now_ms / 160U) & 1U) == 0U) ? 1U : 0U;
        break;
    case APP_OPTICAL_FLOW_HEALTH_FAILED:
    default:
        flow_led_on = app_led_group_blink(now_ms, 3U);
        break;
    }
    app_led_write(LED_2, flow_led_on);

    if (app_led_armed != 0U) {
        app_led_write(LED_3, 1U);
        app_led_write(LED_4, 0U);
    } else if (reason == APP_LED_ARM_BLOCK_NONE) {
        app_led_write(LED_3, 0U);
        app_led_write(LED_4, 1U);
    } else {
        app_led_write(LED_3, app_led_group_blink(now_ms, (uint8_t)reason));
        app_led_write(LED_4, 0U);
    }
}

void APP_LED_SetArmStatus(uint8_t armed, APP_LED_ArmBlockReason reason)
{
    app_led_armed = (armed != 0U) ? 1U : 0U;
    app_led_arm_block_reason = reason;
}

void APP_LED_SetServoCalMode(APP_LED_ServoCalMode mode)
{
    app_led_servo_cal_mode = mode;
}
