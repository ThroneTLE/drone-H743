#include "drv_servo.h"

#include <stdio.h>
#include <string.h>

#define DRV_SERVO_MAX_ID          255U
#define DRV_SERVO_MIN_PULSE_US    500U
#define DRV_SERVO_MAX_PULSE_US    2500U
#define DRV_SERVO_MAX_TIME_MS     9999U
#define DRV_SERVO_MAX_ITEMS       8U
#define DRV_SERVO_TX_TIMEOUT_MS   100U
#define DRV_SERVO_MIN_MODE        1U
#define DRV_SERVO_MAX_MODE        8U
#define DRV_SERVO_MAX_BAUD_CODE   7U

static DRV_SERVO_Status servo_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:      return DRV_SERVO_OK;
    case HAL_TIMEOUT:  return DRV_SERVO_TIMEOUT;
    default:           return DRV_SERVO_ERROR;
    }
}

static uint8_t servo_is_valid_move(const DRV_SERVO_MoveCmd *move)
{
    if (move == NULL) { return 0U; }
    if ((move->pulse_us < DRV_SERVO_MIN_PULSE_US) ||
        (move->pulse_us > DRV_SERVO_MAX_PULSE_US)) { return 0U; }
    return 1U;
}

static DRV_SERVO_Status servo_send_id_command(DRV_SERVO_Device *dev,
                                              uint8_t id, const char *suffix)
{
    char command[24];
    int written;

    if (suffix == NULL) { return DRV_SERVO_INVALID_PARAM; }

    written = snprintf(command, sizeof(command), "#%03uP%s!", (unsigned int)id, suffix);
    if ((written < 0) || ((uint32_t)written >= sizeof(command))) {
        return DRV_SERVO_INVALID_PARAM;
    }

    return DRV_SERVO_SendRaw(dev, command);
}

DRV_SERVO_Status DRV_SERVO_SendRaw(DRV_SERVO_Device *dev, const char *command)
{
    size_t length;
    uint32_t timeout = (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms
                                                    : DRV_SERVO_TX_TIMEOUT_MS;

    if (command == NULL) { return DRV_SERVO_INVALID_PARAM; }

    length = strlen(command);
    if (length == 0U) { return DRV_SERVO_INVALID_PARAM; }

    return servo_from_hal(HAL_UART_Transmit(dev->bus.huart, (uint8_t *)command,
                                            (uint16_t)length, timeout));
}

DRV_SERVO_Status DRV_SERVO_Move(DRV_SERVO_Device *dev, uint8_t id,
                                uint16_t pulse_us, uint16_t time_ms)
{
    DRV_SERVO_MoveCmd move = { .id = id, .pulse_us = pulse_us };
    return DRV_SERVO_MoveMany(dev, &move, 1U, time_ms);
}

DRV_SERVO_Status DRV_SERVO_MoveMany(DRV_SERVO_Device *dev,
                                    const DRV_SERVO_MoveCmd *moves,
                                    uint8_t count, uint16_t time_ms)
{
    char command[128];
    int written;
    uint32_t used = 0U;

    if ((moves == NULL) || (count == 0U) || (count > DRV_SERVO_MAX_ITEMS) ||
        (time_ms > DRV_SERVO_MAX_TIME_MS)) {
        return DRV_SERVO_INVALID_PARAM;
    }

    command[used++] = '{';

    for (uint8_t index = 0U; index < count; ++index) {
        if (servo_is_valid_move(&moves[index]) == 0U) {
            return DRV_SERVO_INVALID_PARAM;
        }

        written = snprintf(&command[used], sizeof(command) - used,
                           "#%03uP%04uT%04u!",
                           (unsigned int)moves[index].id,
                           (unsigned int)moves[index].pulse_us,
                           (unsigned int)time_ms);
        if ((written < 0) || ((uint32_t)written >= (sizeof(command) - used))) {
            return DRV_SERVO_INVALID_PARAM;
        }
        used += (uint32_t)written;
    }

    if ((used + 2U) > sizeof(command)) { return DRV_SERVO_INVALID_PARAM; }

    command[used++] = '}';
    command[used] = '\0';

    return DRV_SERVO_SendRaw(dev, command);
}

DRV_SERVO_Status DRV_SERVO_ReadVersion(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "VER"); }

DRV_SERVO_Status DRV_SERVO_ReadId(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "ID"); }

DRV_SERVO_Status DRV_SERVO_SetId(DRV_SERVO_Device *dev, uint8_t old_id, uint8_t new_id)
{
    char suffix[8];
    int written;
    written = snprintf(suffix, sizeof(suffix), "ID%03u", (unsigned int)new_id);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return DRV_SERVO_INVALID_PARAM;
    }
    return servo_send_id_command(dev, old_id, suffix);
}

DRV_SERVO_Status DRV_SERVO_ReadPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "RAD"); }

DRV_SERVO_Status DRV_SERVO_SetMode(DRV_SERVO_Device *dev, uint8_t id, uint8_t mode)
{
    char suffix[6];
    int written;
    if ((mode < DRV_SERVO_MIN_MODE) || (mode > DRV_SERVO_MAX_MODE)) {
        return DRV_SERVO_INVALID_PARAM;
    }
    written = snprintf(suffix, sizeof(suffix), "MOD%u", (unsigned int)mode);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return DRV_SERVO_INVALID_PARAM;
    }
    return servo_send_id_command(dev, id, suffix);
}

DRV_SERVO_Status DRV_SERVO_ReadMode(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "MOD"); }

DRV_SERVO_Status DRV_SERVO_ReleaseTorque(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "ULK"); }

DRV_SERVO_Status DRV_SERVO_RestoreTorque(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "ULR"); }

DRV_SERVO_Status DRV_SERVO_Pause(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "DPT"); }

DRV_SERVO_Status DRV_SERVO_Continue(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "DCT"); }

DRV_SERVO_Status DRV_SERVO_Stop(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "DST"); }

DRV_SERVO_Status DRV_SERVO_SetBaud(DRV_SERVO_Device *dev, uint8_t id, uint8_t baud_code)
{
    char suffix[6];
    int written;
    if (baud_code > DRV_SERVO_MAX_BAUD_CODE) { return DRV_SERVO_INVALID_PARAM; }
    written = snprintf(suffix, sizeof(suffix), "BD%u", (unsigned int)baud_code);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return DRV_SERVO_INVALID_PARAM;
    }
    return servo_send_id_command(dev, id, suffix);
}

DRV_SERVO_Status DRV_SERVO_SaveCenter(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "SCK"); }

DRV_SERVO_Status DRV_SERVO_SetStartupPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CSD"); }

DRV_SERVO_Status DRV_SERVO_ClearStartupPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CSM"); }

DRV_SERVO_Status DRV_SERVO_RestoreStartupPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CSR"); }

DRV_SERVO_Status DRV_SERVO_SetMinPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "SMI"); }

DRV_SERVO_Status DRV_SERVO_SetMaxPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "SMX"); }

DRV_SERVO_Status DRV_SERVO_FactoryResetKeepId(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CLEO"); }

DRV_SERVO_Status DRV_SERVO_FactoryResetFull(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CLE"); }
