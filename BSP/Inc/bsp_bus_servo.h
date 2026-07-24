#ifndef BSP_BUS_SERVO_H
#define BSP_BUS_SERVO_H

#include "drv_servo.h"

#include <stdint.h>

typedef DRV_SERVO_Status BSP_BusServoStatus;
typedef DRV_SERVO_MoveCmd BSP_BusServoMove;

#define BSP_BUS_SERVO_OK            DRV_SERVO_OK
#define BSP_BUS_SERVO_ERROR         DRV_SERVO_ERROR
#define BSP_BUS_SERVO_INVALID_PARAM DRV_SERVO_INVALID_PARAM
#define BSP_BUS_SERVO_TIMEOUT       DRV_SERVO_TIMEOUT
#define BSP_BUS_SERVO_BUSY          DRV_SERVO_BUSY

DRV_SERVO_Status BSP_BusServo_SendRaw(const char *command);
uint16_t BSP_BusServo_ReadResponse(char *buf, uint16_t max_len);
uint32_t BSP_BusServo_GetBaudRate(void);
DRV_SERVO_Status BSP_BusServo_SetBaudRate(uint32_t baud_rate);
uint16_t BSP_BusServo_PositionToPulse(uint16_t position);
DRV_SERVO_Status BSP_BusServo_Move(uint8_t id, uint16_t pulse_us, uint16_t time_ms);
DRV_SERVO_Status BSP_BusServo_MovePosition(uint8_t id,
                                           uint16_t position,
                                           uint16_t time_ms);
DRV_SERVO_Status BSP_BusServo_MoveMany(const DRV_SERVO_MoveCmd *moves,
                                       uint8_t count, uint16_t time_ms);
DRV_SERVO_Status BSP_BusServo_MoveManyAsync(const DRV_SERVO_MoveCmd *moves,
                                             uint8_t count, uint16_t time_ms);
DRV_SERVO_Status BSP_BusServo_RequestPositionAsync(uint8_t id,
                                                   uint32_t timeout_ms);
void BSP_BusServo_Service(uint32_t now_ms);
uint8_t BSP_BusServo_IsIdle(void);
void BSP_BusServo_GetDiag(DRV_SERVO_Diag *diag);
void BSP_BusServo_GetFeedbackDiag(DRV_SERVO_FeedbackDiag *diag);
DRV_SERVO_Status BSP_BusServo_ReadVersion(uint8_t id);
DRV_SERVO_Status BSP_BusServo_ReadId(uint8_t id);
DRV_SERVO_Status BSP_BusServo_SetId(uint8_t old_id, uint8_t new_id);
DRV_SERVO_Status BSP_BusServo_ReadPosition(uint8_t id);
DRV_SERVO_Status BSP_BusServo_SetMode(uint8_t id, uint8_t mode);
DRV_SERVO_Status BSP_BusServo_ReadMode(uint8_t id);
DRV_SERVO_Status BSP_BusServo_ReleaseTorque(uint8_t id);
DRV_SERVO_Status BSP_BusServo_RestoreTorque(uint8_t id);
DRV_SERVO_Status BSP_BusServo_Pause(uint8_t id);
DRV_SERVO_Status BSP_BusServo_Continue(uint8_t id);
DRV_SERVO_Status BSP_BusServo_Stop(uint8_t id);
DRV_SERVO_Status BSP_BusServo_SetBaud(uint8_t id, uint8_t baud_code);
DRV_SERVO_Status BSP_BusServo_SaveCenter(uint8_t id);
DRV_SERVO_Status BSP_BusServo_SetStartupPosition(uint8_t id);
DRV_SERVO_Status BSP_BusServo_ClearStartupPosition(uint8_t id);
DRV_SERVO_Status BSP_BusServo_RestoreStartupPosition(uint8_t id);
DRV_SERVO_Status BSP_BusServo_SetMinPosition(uint8_t id);
DRV_SERVO_Status BSP_BusServo_SetMaxPosition(uint8_t id);
DRV_SERVO_Status BSP_BusServo_FactoryResetKeepId(uint8_t id);
DRV_SERVO_Status BSP_BusServo_FactoryResetFull(uint8_t id);
DRV_SERVO_Status BSP_BusServo_RunDemoStep(void);

#endif
