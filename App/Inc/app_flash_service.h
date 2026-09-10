#ifndef APP_FLASH_SERVICE_H
#define APP_FLASH_SERVICE_H

/*
 * 持久化门面。
 *
 * 历史上这一层就是外部 SPI NOR（GD25Q32）的薄封装。移植到 MicoAir743v2 之后
 * 板上**没有**外部 NOR，于是它变成一个**按地址路由的门面**，对上仍然保持
 * 完全相同的 4 MB 平坦地址空间与 NOR 几何（4 KB 扇区 / 32 KB 块 / 字节寻址），
 * 因此 svc_param 与 app_flight_log 的上层逻辑一行都不用改。
 *
 * 路由规则（见 app_flash_service.c 顶部的详细说明）：
 *   [PARAM_REGION_START, SIZE)  —— 参数双槽 → 片内 Flash（drv_intflash）
 *   其余地址                     —— 飞行日志等 → SD 裸块（drv_sdblock）
 *
 * 仍然保留的 NOR 专有接口（JEDEC 探测、状态寄存器、写保护）继续打给真正的
 * SPI NOR 驱动。MicoAir 板上没有这颗芯片，它们会如实返回失败——这是想要的行为：
 * 诊断页应该显示"没有外部 Flash"，而不是假装有一颗。
 */

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

/*
 * 参数区在逻辑地址空间的**顶部三个扇区**，与 svc_param.c 的 SLOT_A/SLOT_B 定义
 * 必须一致（svc_param.c 里有 _Static_assert 钉住这一点）。
 * 前两个扇区就是双槽 A / B，第三个（最顶上那个）没人用，留作边界缓冲。
 */
#define APP_FLASH_SERVICE_PARAM_REGION_START \
    (APP_FLASH_SERVICE_SIZE_BYTES - 3U * APP_FLASH_SERVICE_SECTOR_SIZE)
#define APP_FLASH_SERVICE_PARAM_SLOT_A_OFFSET APP_FLASH_SERVICE_PARAM_REGION_START
#define APP_FLASH_SERVICE_PARAM_SLOT_B_OFFSET \
    (APP_FLASH_SERVICE_SIZE_BYTES - 2U * APP_FLASH_SERVICE_SECTOR_SIZE)

/* 后端选择，供诊断命令如实回报某个地址落在哪块介质上。 */
typedef enum {
    APP_FLASH_BACKEND_NONE = 0,
    APP_FLASH_BACKEND_INTERNAL,
    APP_FLASH_BACKEND_SDBLOCK
} APP_FlashService_Backend;

APP_FlashService_Backend APP_FlashService_BackendFor(uint32_t address);
const char *APP_FlashService_BackendName(APP_FlashService_Backend backend);
uint8_t APP_FlashService_IsLogStorageReady(void);

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
APP_FlashService_Status APP_FlashService_ReadStatus2(uint8_t *status2);
APP_FlashService_Status APP_FlashService_ReadStatus3(uint8_t *status3);
APP_FlashService_Status APP_FlashService_WriteEnableProbe(uint8_t *status_before,
                                                          uint8_t *status_after);
APP_FlashService_Status APP_FlashService_ClearProtection(uint8_t *status1_before,
                                                         uint8_t *status2_before,
                                                         uint8_t *status1_after,
                                                         uint8_t *status2_after);
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
