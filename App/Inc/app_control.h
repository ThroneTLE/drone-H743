#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>

#define APP_CONTROL_SERVO_COUNT 2U
#define APP_CONTROL_PID_AXIS_COUNT 3U

typedef struct {
    int16_t kp;
    int16_t ki;
    int16_t kd;
    int16_t integral_limit;
    int16_t output_limit;
} APP_ControlPidConfig;

typedef struct {
    uint8_t  id;
    uint16_t pulse_us;
    uint16_t time_ms;
    uint8_t  mode;
    uint8_t  enabled;
} APP_ControlServoConfig;

typedef struct {
    uint8_t loaded_from_flash;
    uint8_t flash_valid;
    uint8_t last_flash_status;
    APP_ControlServoConfig servo[APP_CONTROL_SERVO_COUNT];
    APP_ControlPidConfig rate_pid[APP_CONTROL_PID_AXIS_COUNT];
    APP_ControlPidConfig angle_pid[APP_CONTROL_PID_AXIS_COUNT];
    APP_ControlPidConfig altitude_pid;
} APP_ControlConfig;

void APP_Control_Init(void);
void APP_Control_Tick(void);
void APP_Control_ProcessLine(const char *line);
void APP_Control_MaintTick(void);
void APP_Control_ProcessMaintLine(const char *line);
void APP_Control_ProcessProtoRequest(uint16_t function,
                                     const uint8_t *payload,
                                     uint16_t payload_length);
void APP_Control_GetConfig(APP_ControlConfig *config);
void APP_Control_ReportUartStats(uint32_t rx_bytes,
                                  uint32_t rx_lines,
                                  uint32_t rx_overflows,
                                  uint32_t rx_errors);

#endif
