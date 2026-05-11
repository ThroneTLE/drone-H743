#ifndef DRONE_H743_FLASH_BUS_H
#define DRONE_H743_FLASH_BUS_H

#include "drv_gd25q32.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

const DRV_GD25Q32_Bus *BSP_FlashBus_GetBus(void);
DRV_GD25Q32_Status BSP_FlashBus_Acquire(uint32_t timeout_ms);
void BSP_FlashBus_Release(void);

void BSP_FlashBus_RegisterDmaDevice(DRV_GD25Q32_Device *dev);
void BSP_FlashBus_InvalidateBinding(void);

#ifdef __cplusplus
}
#endif

#endif
