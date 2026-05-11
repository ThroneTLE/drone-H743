#include "bsp_flash_bus.h"

#include "bsp_board.h"
#include "bsp_spi.h"
#include "rtos_objects.h"

#include "cmsis_os2.h"

const DRV_GD25Q32_Bus *BSP_FlashBus_GetBus(void)
{
    return BSP_Board_GetFlashBus();
}

DRV_GD25Q32_Status BSP_FlashBus_Acquire(uint32_t timeout_ms)
{
    if (osKernelGetState() != osKernelRunning) {
        return DRV_GD25Q32_OK;
    }

    if (flashBusMutexHandle == NULL) {
        return DRV_GD25Q32_ERROR;
    }

    return (osMutexAcquire(flashBusMutexHandle, timeout_ms) == osOK) ?
           DRV_GD25Q32_OK : DRV_GD25Q32_TIMEOUT;
}

void BSP_FlashBus_Release(void)
{
    if ((osKernelGetState() == osKernelRunning) && (flashBusMutexHandle != NULL)) {
        (void)osMutexRelease(flashBusMutexHandle);
    }
}

void BSP_FlashBus_RegisterDmaDevice(DRV_GD25Q32_Device *dev)
{
    BSP_SPI_RegisterFlashDevice(dev);
}

void BSP_FlashBus_InvalidateBinding(void)
{
    BSP_SPI_RegisterFlashDevice(NULL);
}
