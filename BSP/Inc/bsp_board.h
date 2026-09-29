#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include "drv_bmi088.h"
#include "drv_bmi270.h"
#include "drv_imu.h"
#include "drv_baro.h"
#include "drv_gd25q32.h"
#include "drv_intflash.h"
#include "drv_mag.h"
#include "drv_gps.h"
#include "drv_optical_flow.h"
#include "drv_sdblock.h"
#include "drv_servo.h"

#ifdef __cplusplus
extern "C" {
#endif

void BSP_Board_Init(void);

/*
 * 开机 SD 初始化窗口。CubeMX 生成的 MX_SDMMC1_SD_Init 在 USER CODE 里调 Begin/End；
 * 没插卡时 HAL_SD_Init 失败，生成代码会进 Error_Handler 把整机卡死在开机。
 * Error_Handler 的 USER CODE 先问 SdInitFailed()：正处在这个窗口里就记为无卡、返回 1，
 * 由调用者返回继续开机；窗口外返回 0，其他外设出错照旧停机。
 * BSP_Board_Init 在无卡时不把 SD 交给日志驱动，日志降级为不记录。
 */
void BSP_Board_SdInitBegin(void);
void BSP_Board_SdInitEnd(void);
uint8_t BSP_Board_SdInitFailed(void);
uint8_t BSP_Board_SdCardPresent(void);
/* 开机识别失败时 hsd1 的 HAL ErrorCode；识别成功为 0。 */
uint32_t BSP_Board_SdInitError(void);

void BSP_DelayMs(uint32_t ms);

const DRV_IMU_Bus   *BSP_Board_GetImuBus(void);
const DRV_BMI088_Bus *BSP_Board_GetBmi088Bus(void);
const DRV_BMI270_Bus *BSP_Board_GetBmi270Bus(void);
const DRV_BARO_Bus  *BSP_Board_GetBaroBus(void);
const DRV_GD25Q32_Bus *BSP_Board_GetFlashBus(void);
const DRV_INTFLASH_Bus *BSP_Board_GetIntFlashBus(void);
const DRV_SDBLOCK_Bus *BSP_Board_GetSdBlockBus(void);
const DRV_MAG_Bus   *BSP_Board_GetMagBus(void);
const DRV_GPS_Bus   *BSP_Board_GetGpsBus(void);
const DRV_OPTICAL_FLOW_Bus *BSP_Board_GetOpticalFlowBus(void);
const DRV_SERVO_Bus *BSP_Board_GetServoBus(void);

#ifdef __cplusplus
}
#endif

#endif
