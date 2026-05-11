#include "app_flash_service.h"

#include "bsp_flash_bus.h"

#include <string.h>

static DRV_GD25Q32_Device flash_device;
static uint8_t flash_bound;

#define APP_FLASH_SERVICE_LOCK_TIMEOUT_MS 10000U

static void flash_service_bind(void)
{
    if (flash_bound != 0U) {
        return;
    }

    memset(&flash_device, 0, sizeof(flash_device));
    flash_device.bus = *BSP_FlashBus_GetBus();
    BSP_FlashBus_RegisterDmaDevice(&flash_device);
    flash_bound = 1U;
}

static APP_FlashService_Status flash_service_lock(void)
{
    return BSP_FlashBus_Acquire(APP_FLASH_SERVICE_LOCK_TIMEOUT_MS);
}

static void flash_service_unlock(void)
{
    BSP_FlashBus_Release();
}

APP_FlashService_Status APP_FlashService_Init(void)
{
    APP_FlashService_Status status;
    DRV_GD25Q32_Bus bus;

    flash_service_bind();
    bus = flash_device.bus;
    status = flash_service_lock();
    if (status != DRV_GD25Q32_OK) {
        return status;
    }

    status = DRV_GD25Q32_Init(&flash_device, &bus);
    BSP_FlashBus_RegisterDmaDevice(&flash_device);
    flash_service_unlock();
    return status;
}

APP_FlashService_Status APP_FlashService_ProbeJedecId(APP_FlashService_JedecId *jedec_id)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status != DRV_GD25Q32_OK) {
        return status;
    }

    status = DRV_GD25Q32_ReleaseFromPowerDown(&flash_device);
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_ReadJedecId(&flash_device, jedec_id);
    }
    if ((status == DRV_GD25Q32_OK) &&
        ((jedec_id->manufacturer_id != DRV_GD25Q32_JEDEC_MANUFACTURER_ID) ||
         (jedec_id->memory_type != DRV_GD25Q32_JEDEC_MEMORY_TYPE) ||
         (jedec_id->capacity_id != DRV_GD25Q32_JEDEC_CAPACITY_ID))) {
        status = DRV_GD25Q32_BAD_ID;
    }

    flash_service_unlock();
    return status;
}

APP_FlashService_Status APP_FlashService_ReadStatus1(uint8_t *status1)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_ReadStatus1(&flash_device, status1);
        flash_service_unlock();
    }
    return status;
}

APP_FlashService_Status APP_FlashService_ReadData(uint32_t address, uint8_t *data, uint32_t length)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_ReadData(&flash_device, address, data, length);
        flash_service_unlock();
    }
    return status;
}

APP_FlashService_Status APP_FlashService_ReadDataFast(uint32_t address, uint8_t *data, uint32_t length)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_ReadDataFast(&flash_device, address, data, length);
        flash_service_unlock();
    }
    return status;
}

APP_FlashService_Status APP_FlashService_EraseSector(uint32_t address)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_EraseSector(&flash_device, address);
        flash_service_unlock();
    }
    return status;
}

APP_FlashService_Status APP_FlashService_EraseBlock32K(uint32_t address)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_EraseBlock32K(&flash_device, address);
        flash_service_unlock();
    }
    return status;
}

APP_FlashService_Status APP_FlashService_EraseBlock64K(uint32_t address)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_EraseBlock64K(&flash_device, address);
        flash_service_unlock();
    }
    return status;
}

APP_FlashService_Status APP_FlashService_PageProgram(uint32_t address,
                                                     const uint8_t *data,
                                                     uint16_t length)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_PageProgram(&flash_device, address, data, length);
        flash_service_unlock();
    }
    return status;
}

APP_FlashService_Status APP_FlashService_WriteData(uint32_t address,
                                                   const uint8_t *data,
                                                   uint32_t length)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_WriteData(&flash_device, address, data, length);
        flash_service_unlock();
    }
    return status;
}

void APP_FlashService_Invalidate(void)
{
    memset(&flash_device, 0, sizeof(flash_device));
    BSP_FlashBus_InvalidateBinding();
    flash_bound = 0U;
}
