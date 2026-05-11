#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

typedef enum {
    DRV_SERVO_OK = 0,
    DRV_SERVO_ERROR,
    DRV_SERVO_INVALID_PARAM,
    DRV_SERVO_TIMEOUT
} DRV_SERVO_Status;

typedef struct {
    uint8_t  id;
    uint16_t pulse_us;
} DRV_SERVO_MoveCmd;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t            timeout_ms;
} DRV_SERVO_Bus;

typedef struct {
    DRV_SERVO_Bus bus;
} DRV_SERVO_Device;

DRV_SERVO_Status DRV_SERVO_SendRaw(DRV_SERVO_Device *dev, const char *command);
DRV_SERVO_Status DRV_SERVO_Move(DRV_SERVO_Device *dev, uint8_t id,
                                uint16_t pulse_us, uint16_t time_ms);
DRV_SERVO_Status DRV_SERVO_MoveMany(DRV_SERVO_Device *dev,
                                    const DRV_SERVO_MoveCmd *moves,
                                    uint8_t count, uint16_t time_ms);
DRV_SERVO_Status DRV_SERVO_ReadVersion(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_ReadId(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetId(DRV_SERVO_Device *dev, uint8_t old_id, uint8_t new_id);
DRV_SERVO_Status DRV_SERVO_ReadPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetMode(DRV_SERVO_Device *dev, uint8_t id, uint8_t mode);
DRV_SERVO_Status DRV_SERVO_ReadMode(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_ReleaseTorque(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_RestoreTorque(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_Pause(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_Continue(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_Stop(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetBaud(DRV_SERVO_Device *dev, uint8_t id, uint8_t baud_code);
DRV_SERVO_Status DRV_SERVO_SaveCenter(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetStartupPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_ClearStartupPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_RestoreStartupPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetMinPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_SetMaxPosition(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_FactoryResetKeepId(DRV_SERVO_Device *dev, uint8_t id);
DRV_SERVO_Status DRV_SERVO_FactoryResetFull(DRV_SERVO_Device *dev, uint8_t id);

#ifdef __cplusplus
}
#endif

#endif
