#include "bsp_bus_servo.h"
#include "bsp_board.h"

#include <string.h>

static DRV_SERVO_Device servo_dev;
static uint8_t          servo_bound;

static void servo_bind_bus(void)
{
    if (servo_bound != 0U) { return; }
    memset(&servo_dev, 0, sizeof(servo_dev));
    servo_dev.bus = *BSP_Board_GetServoBus();
    servo_bound = 1U;
}

DRV_SERVO_Status BSP_BusServo_SendRaw(const char *command)
{ servo_bind_bus(); return DRV_SERVO_SendRaw(&servo_dev, command); }

uint16_t BSP_BusServo_ReadResponse(char *buf, uint16_t max_len)
{ servo_bind_bus(); return DRV_SERVO_ReadResponse(&servo_dev, buf, max_len); }

uint32_t BSP_BusServo_GetBaudRate(void)
{ servo_bind_bus(); return DRV_SERVO_GetBaudRate(&servo_dev); }

DRV_SERVO_Status BSP_BusServo_SetBaudRate(uint32_t baud_rate)
{ servo_bind_bus(); return DRV_SERVO_SetBaudRate(&servo_dev, baud_rate); }

uint16_t BSP_BusServo_PositionToPulse(uint16_t position)
{ return DRV_SERVO_PositionToPulse(position); }

DRV_SERVO_Status BSP_BusServo_Move(uint8_t id, uint16_t pulse_us, uint16_t time_ms)
{ servo_bind_bus(); return DRV_SERVO_Move(&servo_dev, id, pulse_us, time_ms); }

DRV_SERVO_Status BSP_BusServo_MovePosition(uint8_t id,
                                           uint16_t position,
                                           uint16_t time_ms)
{ servo_bind_bus(); return DRV_SERVO_MovePosition(&servo_dev, id, position, time_ms); }

DRV_SERVO_Status BSP_BusServo_MoveMany(const DRV_SERVO_MoveCmd *moves,
                                       uint8_t count, uint16_t time_ms)
{ servo_bind_bus(); return DRV_SERVO_MoveMany(&servo_dev, moves, count, time_ms); }

DRV_SERVO_Status BSP_BusServo_ReadVersion(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_ReadVersion(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_ReadId(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_ReadId(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_SetId(uint8_t old_id, uint8_t new_id)
{ servo_bind_bus(); return DRV_SERVO_SetId(&servo_dev, old_id, new_id); }

DRV_SERVO_Status BSP_BusServo_ReadPosition(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_ReadPosition(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_SetMode(uint8_t id, uint8_t mode)
{ servo_bind_bus(); return DRV_SERVO_SetMode(&servo_dev, id, mode); }

DRV_SERVO_Status BSP_BusServo_ReadMode(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_ReadMode(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_ReleaseTorque(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_ReleaseTorque(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_RestoreTorque(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_RestoreTorque(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_Pause(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_Pause(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_Continue(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_Continue(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_Stop(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_Stop(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_SetBaud(uint8_t id, uint8_t baud_code)
{ servo_bind_bus(); return DRV_SERVO_SetBaud(&servo_dev, id, baud_code); }

DRV_SERVO_Status BSP_BusServo_SaveCenter(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_SaveCenter(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_SetStartupPosition(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_SetStartupPosition(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_ClearStartupPosition(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_ClearStartupPosition(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_RestoreStartupPosition(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_RestoreStartupPosition(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_SetMinPosition(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_SetMinPosition(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_SetMaxPosition(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_SetMaxPosition(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_FactoryResetKeepId(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_FactoryResetKeepId(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_FactoryResetFull(uint8_t id)
{ servo_bind_bus(); return DRV_SERVO_FactoryResetFull(&servo_dev, id); }

DRV_SERVO_Status BSP_BusServo_RunDemoStep(void)
{
    static uint8_t direction;
    DRV_SERVO_MoveCmd moves[2];

    if (direction == 0U) {
        moves[0].id = 1U; moves[0].pulse_us = 500U;
        moves[1].id = 2U; moves[1].pulse_us = 2500U;
        direction = 1U;
    } else {
        moves[0].id = 1U; moves[0].pulse_us = 2500U;
        moves[1].id = 2U; moves[1].pulse_us = 500U;
        direction = 0U;
    }

    return BSP_BusServo_MoveMany(moves, 2U, 1000U);
}
