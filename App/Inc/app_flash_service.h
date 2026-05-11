#ifndef APP_FLASH_SERVICE_H
#define APP_FLASH_SERVICE_H

#include "drv_gd25q32.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_FLASH_SERVICE_JEDEC_MANUFACTURER_ID DRV_GD25Q32_JEDEC_MANUFACTURER_ID
#define APP_FLASH_SERVICE_JEDEC_MEMORY_TYPE     DRV_GD25Q32_JEDEC_MEMORY_TYPE
#define APP_FLASH_SERVICE_JEDEC_CAPACITY_ID     DRV_GD25Q32_JEDEC_CAPACITY_ID
#define APP_FLASH_SERVICE_SIZE_BYTES            DRV_GD25Q32_SIZE_BYTES
#define APP_FLASH_SERVICE_SECTOR_SIZE           DRV_GD25Q32_SECTOR_SIZE
#define APP_FLASH_SERVICE_BLOCK32K_SIZE         DRV_GD25Q32_BLOCK32K_SIZE
#define APP_FLASH_SERVICE_BLOCK64K_SIZE         DRV_GD25Q32_BLOCK64K_SIZE
#define APP_FLASH_SERVICE_PAGE_SIZE             DRV_GD25Q32_PAGE_SIZE

typedef DRV_GD25Q32_Status  APP_FlashService_Status;
typedef DRV_GD25Q32_JedecId APP_FlashService_JedecId;

#define APP_FLASH_SERVICE_OK          DRV_GD25Q32_OK
#define APP_FLASH_SERVICE_ERROR       DRV_GD25Q32_ERROR
#define APP_FLASH_SERVICE_TIMEOUT     DRV_GD25Q32_TIMEOUT
#define APP_FLASH_SERVICE_BAD_ID      DRV_GD25Q32_BAD_ID
#define APP_FLASH_SERVICE_INVALID_ARG DRV_GD25Q32_INVALID_ARG
#define APP_FLASH_SERVICE_BUSY        DRV_GD25Q32_BUSY
#define APP_FLASH_SERVICE_DMA_ERROR   DRV_GD25Q32_DMA_ERROR

APP_FlashService_Status APP_FlashService_Init(void);
APP_FlashService_Status APP_FlashService_ProbeJedecId(APP_FlashService_JedecId *jedec_id);
APP_FlashService_Status APP_FlashService_ReadStatus1(uint8_t *status1);
APP_FlashService_Status APP_FlashService_ReadData(uint32_t address, uint8_t *data, uint32_t length);
APP_FlashService_Status APP_FlashService_ReadDataFast(uint32_t address, uint8_t *data, uint32_t length);
APP_FlashService_Status APP_FlashService_EraseSector(uint32_t address);
APP_FlashService_Status APP_FlashService_EraseBlock32K(uint32_t address);
APP_FlashService_Status APP_FlashService_EraseBlock64K(uint32_t address);
APP_FlashService_Status APP_FlashService_PageProgram(uint32_t address, const uint8_t *data, uint16_t length);
APP_FlashService_Status APP_FlashService_WriteData(uint32_t address, const uint8_t *data, uint32_t length);
void APP_FlashService_Invalidate(void);

#ifdef __cplusplus
}
#endif

#endif
