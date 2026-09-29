/*
 * SERVOHZ 命令族（协议见 App/Inc/app_servo_hz.h）。
 *
 * 为什么只在上锁时收：切帧率的那一帧舵机要么多等一会儿、要么提前收到脉冲，
 * 飞行中这就是一次执行器扰动；台架 A/B 本来就在上锁时切，没有理由放开。
 * 辨识在跑时同理（包括上锁跑的舵机单独模式），否则一轮数据里混着两种延迟。
 */
#include "app_servo_hz.h"

#include "app_control.h"
#include "app_control_internal.h"
#include "app_stabilizer.h"
#include "app_sysid.h"
#include "bsp_pwm.h"

#include <stdlib.h>
#include <string.h>

static void servohz_report(void)
{
    const uint32_t hz = BSP_PWM_GetServoFrameHz();

    APP_Control_QueueText("SERVOHZ hz=%lu frame_us=%lu\r\n", (unsigned long)hz,
                          (unsigned long)((hz > 0U) ? (1000000UL / hz) : 0UL));
}

static uint8_t servohz_reject(const char *reason)
{
    APP_Control_QueueText("SERVOHZ event=rejected reason=%s\r\n", reason);
    return 1U;
}

uint8_t APP_ServoHz_Command(char **tokens, uint32_t count)
{
    char *end = NULL;
    unsigned long hz;

    if ((tokens == NULL) || (count == 0U) || (tokens[0] == NULL) ||
        (strcmp(tokens[0], "SERVOHZ") != 0)) {
        return 0U;
    }
    if ((count == 1U) || ((count == 2U) && (strcmp(tokens[1], "?") == 0))) {
        servohz_report();
        return 1U;
    }
    if (count != 2U) {
        return servohz_reject("usage");
    }
    hz = strtoul(tokens[1], &end, 10);
    if ((end == tokens[1]) || (*end != '\0') || (hz < BSP_PWM_SERVO_FRAME_HZ_MIN) ||
        (hz > BSP_PWM_SERVO_FRAME_HZ_MAX)) {
        return servohz_reject("range");
    }
    if (APP_Stabilizer_IsArmed() != 0U) {
        return servohz_reject("armed");
    }
    if (APP_SysId_IsRunning() != 0U) {
        return servohz_reject("sysid");
    }
    if (BSP_PWM_SetServoFrameHz((uint32_t)hz) != BSP_PWM_OK) {
        return servohz_reject("pwm");
    }
    servohz_report();
    return 1U;
}
