#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "main.h"
#include "drv_gd25q32.h"

#ifdef __cplusplus
extern "C" {
#endif

void BSP_SPI_RegisterFlashDevice(DRV_GD25Q32_Device *dev);

#ifdef __cplusplus
}
#endif

#endif
