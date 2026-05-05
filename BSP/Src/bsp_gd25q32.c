#include "bsp_gd25q32.h"

#include <string.h>

#define GD25Q32_CMD_RELEASE_POWER_DOWN 0xABU
#define GD25Q32_CMD_READ_STATUS1       0x05U
#define GD25Q32_CMD_READ_DATA          0x03U
#define GD25Q32_CMD_READ_JEDEC_ID      0x9FU

#define GD25Q32_DEFAULT_TIMEOUT_MS     100U
#define GD25Q32_WAKE_DELAY_MS          1U

static uint32_t gd25q32_timeout_ms(const BSP_GD25Q32_Device *dev)
{
    return (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms : GD25Q32_DEFAULT_TIMEOUT_MS;
}

static void gd25q32_delay_ms(BSP_GD25Q32_Device *dev, uint32_t delay_ms)
{
    if (dev->bus.delay_ms != NULL) {
        dev->bus.delay_ms(delay_ms);
    } else {
        HAL_Delay(delay_ms);
    }
}

static void gd25q32_cs_low(BSP_GD25Q32_Device *dev)
{
    HAL_GPIO_WritePin(dev->bus.cs_port, dev->bus.cs_pin, GPIO_PIN_RESET);
}

static void gd25q32_cs_high(BSP_GD25Q32_Device *dev)
{
    HAL_GPIO_WritePin(dev->bus.cs_port, dev->bus.cs_pin, GPIO_PIN_SET);
}

static BSP_GD25Q32_Status gd25q32_from_hal_status(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return BSP_GD25Q32_OK;
    case HAL_TIMEOUT:
        return BSP_GD25Q32_TIMEOUT;
    case HAL_ERROR:
    case HAL_BUSY:
    default:
        return BSP_GD25Q32_ERROR;
    }
}

static BSP_GD25Q32_Status gd25q32_read_after_command(BSP_GD25Q32_Device *dev,
                                                     const uint8_t *command,
                                                     uint16_t command_length,
                                                     uint8_t *data,
                                                     uint16_t data_length)
{
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (command == NULL) || (command_length == 0U) ||
        (data == NULL) || (data_length == 0U)) {
        return BSP_GD25Q32_INVALID_ARG;
    }

    gd25q32_cs_low(dev);
    hal_status = HAL_SPI_Transmit(dev->bus.hspi,
                                  (uint8_t *)(uintptr_t)command,
                                  command_length,
                                  gd25q32_timeout_ms(dev));
    if (hal_status == HAL_OK) {
        hal_status = HAL_SPI_Receive(dev->bus.hspi,
                                     data,
                                     data_length,
                                     gd25q32_timeout_ms(dev));
    }
    gd25q32_cs_high(dev);

    return gd25q32_from_hal_status(hal_status);
}

BSP_GD25Q32_Status BSP_GD25Q32_Init(BSP_GD25Q32_Device *dev,
                                    const BSP_GD25Q32_Bus *bus)
{
    BSP_GD25Q32_Status status;
    BSP_GD25Q32_JedecId jedec_id;

    if ((dev == NULL) || (bus == NULL) || (bus->hspi == NULL) ||
        (bus->cs_port == NULL)) {
        return BSP_GD25Q32_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->bus = *bus;

    gd25q32_cs_high(dev);

    status = BSP_GD25Q32_ReleaseFromPowerDown(dev);
    if (status != BSP_GD25Q32_OK) {
        return status;
    }

    status = BSP_GD25Q32_ReadJedecId(dev, &jedec_id);
    if (status != BSP_GD25Q32_OK) {
        return status;
    }

    dev->jedec_id = jedec_id;
    if ((jedec_id.manufacturer_id != BSP_GD25Q32_JEDEC_MANUFACTURER_ID) ||
        (jedec_id.memory_type != BSP_GD25Q32_JEDEC_MEMORY_TYPE) ||
        (jedec_id.capacity_id != BSP_GD25Q32_JEDEC_CAPACITY_ID)) {
        return BSP_GD25Q32_BAD_ID;
    }

    return BSP_GD25Q32_OK;
}

BSP_GD25Q32_Status BSP_GD25Q32_ReleaseFromPowerDown(BSP_GD25Q32_Device *dev)
{
    HAL_StatusTypeDef hal_status;
    uint8_t command = GD25Q32_CMD_RELEASE_POWER_DOWN;

    if (dev == NULL) {
        return BSP_GD25Q32_INVALID_ARG;
    }

    gd25q32_cs_low(dev);
    hal_status = HAL_SPI_Transmit(dev->bus.hspi, &command, 1U, gd25q32_timeout_ms(dev));
    gd25q32_cs_high(dev);

    if (hal_status != HAL_OK) {
        return gd25q32_from_hal_status(hal_status);
    }

    gd25q32_delay_ms(dev, GD25Q32_WAKE_DELAY_MS);
    return BSP_GD25Q32_OK;
}

BSP_GD25Q32_Status BSP_GD25Q32_ReadJedecId(BSP_GD25Q32_Device *dev,
                                           BSP_GD25Q32_JedecId *jedec_id)
{
    uint8_t command = GD25Q32_CMD_READ_JEDEC_ID;
    uint8_t response[3];
    BSP_GD25Q32_Status status;

    if ((dev == NULL) || (jedec_id == NULL)) {
        return BSP_GD25Q32_INVALID_ARG;
    }

    status = gd25q32_read_after_command(dev, &command, 1U, response, sizeof(response));
    if (status != BSP_GD25Q32_OK) {
        return status;
    }

    jedec_id->manufacturer_id = response[0];
    jedec_id->memory_type = response[1];
    jedec_id->capacity_id = response[2];
    dev->jedec_id = *jedec_id;

    return BSP_GD25Q32_OK;
}

BSP_GD25Q32_Status BSP_GD25Q32_ReadStatus1(BSP_GD25Q32_Device *dev,
                                           uint8_t *status1)
{
    uint8_t command = GD25Q32_CMD_READ_STATUS1;

    return gd25q32_read_after_command(dev, &command, 1U, status1, 1U);
}

BSP_GD25Q32_Status BSP_GD25Q32_ReadData(BSP_GD25Q32_Device *dev,
                                        uint32_t address,
                                        uint8_t *data,
                                        uint32_t length)
{
    uint8_t command[4];
    uint32_t remaining;
    uint32_t offset = 0U;
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (data == NULL) || (length == 0U)) {
        return BSP_GD25Q32_INVALID_ARG;
    }

    if ((address >= BSP_GD25Q32_FLASH_SIZE_BYTES) ||
        (length > BSP_GD25Q32_FLASH_SIZE_BYTES) ||
        ((address + length) > BSP_GD25Q32_FLASH_SIZE_BYTES)) {
        return BSP_GD25Q32_INVALID_ARG;
    }

    command[0] = GD25Q32_CMD_READ_DATA;
    command[1] = (uint8_t)(address >> 16U);
    command[2] = (uint8_t)(address >> 8U);
    command[3] = (uint8_t)address;

    gd25q32_cs_low(dev);
    hal_status = HAL_SPI_Transmit(dev->bus.hspi,
                                  command,
                                  sizeof(command),
                                  gd25q32_timeout_ms(dev));
    remaining = length;
    while ((hal_status == HAL_OK) && (remaining > 0U)) {
        uint16_t chunk = (remaining > 0xFFFFU) ? 0xFFFFU : (uint16_t)remaining;

        hal_status = HAL_SPI_Receive(dev->bus.hspi,
                                     &data[offset],
                                     chunk,
                                     gd25q32_timeout_ms(dev));
        offset += chunk;
        remaining -= chunk;
    }
    gd25q32_cs_high(dev);

    return gd25q32_from_hal_status(hal_status);
}
