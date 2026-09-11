#ifndef APP_CONTROL_CONFIG_STORE_H
#define APP_CONTROL_CONFIG_STORE_H

#include "app_control.h"
#include "app_control_config_compat.h"
#include "app_flash_service.h"

#define APP_CONTROL_CFG_MAGIC       0x44524346UL
/*
 * v20：记录里新增机体模型块（DRV_Airframe_Params）。
 * 机体数据以前是 drv_airframe_model.h 里的编译期常量，现在唯一来源是 Flash，
 * 由上位机写入——代码里不再保留副本，就不会出现两份数据打架。
 * 从 v19 迁移过来时记录里**没有**机体块，那就是真的没有：DRV_Airframe_Clear()
 * 之后 valid 为 0，解锁被挡住，直到有人把量出来的数填进去。
 */
#define APP_CONTROL_CFG_VERSION     20U
#define APP_CONTROL_CFG_VERSION_V19 19U
#define APP_CONTROL_CFG_VERSION_V18 18U
#define APP_CONTROL_CFG_VERSION_V17 17U
#define APP_CONTROL_CFG_VERSION_V16 16U
#define APP_CONTROL_CFG_VERSION_V15 15U
#define APP_CONTROL_CFG_ADDRESS     (APP_FLASH_SERVICE_SIZE_BYTES - 4096UL)

APP_FlashService_Status APP_ControlConfigStore_Load(APP_ControlConfig *config);
APP_FlashService_Status APP_ControlConfigStore_Save(const APP_ControlConfig *config);
void APP_ControlConfigStore_CaptureTunables(APP_ControlCoaxTunableParams *out);

#endif /* APP_CONTROL_CONFIG_STORE_H */
