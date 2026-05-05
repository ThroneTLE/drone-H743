#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_gd25q32.h"

BSP_GD25Q32_Status BSP_FLASH_Init(void);
BSP_GD25Q32_Status BSP_FLASH_ProbeJedecId(BSP_GD25Q32_JedecId *jedec_id);
BSP_GD25Q32_Status BSP_FLASH_ReadStatus1(uint8_t *status1);
BSP_GD25Q32_Status BSP_FLASH_ReadData(uint32_t address, uint8_t *data, uint32_t length);
const BSP_GD25Q32_Device *BSP_FLASH_GetDevice(void);
void BSP_FLASH_Invalidate(void);

#ifdef __cplusplus
}
#endif

#endif
