#include "bsp_flash.h"

#include "main.h"
#include "spi.h"

#include <string.h>

static BSP_GD25Q32_Device flash_dev;
static uint8_t flash_bound;

static void flash_bind_bus(void)
{
    if (flash_bound != 0U) {
        return;
    }

    memset(&flash_dev, 0, sizeof(flash_dev));
    flash_dev.bus.hspi = &hspi1;
    flash_dev.bus.cs_port = FLASH_CS_GPIO_Port;
    flash_dev.bus.cs_pin = FLASH_CS_Pin;
    flash_dev.bus.timeout_ms = 100U;
    flash_dev.bus.delay_ms = HAL_Delay;
    flash_bound = 1U;
}

BSP_GD25Q32_Status BSP_FLASH_Init(void)
{
    flash_bind_bus();
    return BSP_GD25Q32_Init(&flash_dev, &flash_dev.bus);
}

BSP_GD25Q32_Status BSP_FLASH_ProbeJedecId(BSP_GD25Q32_JedecId *jedec_id)
{
    BSP_GD25Q32_Status status;

    flash_bind_bus();

    status = BSP_GD25Q32_ReleaseFromPowerDown(&flash_dev);
    if (status != BSP_GD25Q32_OK) {
        return status;
    }

    status = BSP_GD25Q32_ReadJedecId(&flash_dev, jedec_id);
    if (status != BSP_GD25Q32_OK) {
        return status;
    }

    if ((jedec_id->manufacturer_id != BSP_GD25Q32_JEDEC_MANUFACTURER_ID) ||
        (jedec_id->memory_type != BSP_GD25Q32_JEDEC_MEMORY_TYPE) ||
        (jedec_id->capacity_id != BSP_GD25Q32_JEDEC_CAPACITY_ID)) {
        return BSP_GD25Q32_BAD_ID;
    }

    return BSP_GD25Q32_OK;
}

BSP_GD25Q32_Status BSP_FLASH_ReadStatus1(uint8_t *status1)
{
    flash_bind_bus();
    return BSP_GD25Q32_ReadStatus1(&flash_dev, status1);
}

BSP_GD25Q32_Status BSP_FLASH_ReadData(uint32_t address, uint8_t *data, uint32_t length)
{
    flash_bind_bus();
    return BSP_GD25Q32_ReadData(&flash_dev, address, data, length);
}

const BSP_GD25Q32_Device *BSP_FLASH_GetDevice(void)
{
    flash_bind_bus();
    return &flash_dev;
}

void BSP_FLASH_Invalidate(void)
{
    memset(&flash_dev, 0, sizeof(flash_dev));
    flash_bound = 0U;
}
