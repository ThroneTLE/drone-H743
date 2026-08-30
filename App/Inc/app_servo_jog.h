#ifndef APP_SERVO_JOG_H
#define APP_SERVO_JOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_SERVO_JOG_CHANNEL_COUNT        2U
#define APP_SERVO_JOG_MIN_US             500U
#define APP_SERVO_JOG_MAX_US            2500U
#define APP_SERVO_JOG_SLEW_US_PER_S      500U
#define APP_SERVO_JOG_HOLD_TIMEOUT_MS 120000UL

void APP_ServoJog_Init(void);

/* 通信任务上下文：SERVO JOG 子命令入口（tokens[0]="SERVO" tokens[1]="JOG"）。 */
void APP_ServoJog_HandleCommand(char *tokens[], uint32_t count, uint32_t now_ms);

/* 任意上下文：请求/释放只写对齐标量，控制环在 Apply 中消费。 */
uint8_t APP_ServoJog_Request(uint32_t channel, uint32_t target_us, uint32_t now_ms);
void APP_ServoJog_ReleaseAll(void);
uint8_t APP_ServoJog_IsActive(void);

/* 控制环上下文：输出仲裁点调用；yield_reason 非空表示更高优先级占用输出。 */
void APP_ServoJog_Apply(uint32_t now_ms,
                        const char *yield_reason,
                        uint16_t *alpha_pulse_us,
                        uint16_t *beta_pulse_us);
void APP_ServoJog_ForceRelease(const char *reason);

/* 通信任务上下文：取走控制环缓存的事件通告（模式同 APP_ServoCal_TakeNotice）。 */
uint16_t APP_ServoJog_TakeNotice(char *out, uint16_t capacity);

#ifdef __cplusplus
}
#endif

#endif
