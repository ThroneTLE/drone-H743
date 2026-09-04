#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include "drv_imu.h"
#include "drv_baro.h"
#include "drv_gd25q32.h"
#include "drv_mag.h"
#include "drv_gps.h"
#include "drv_optical_flow.h"
#include "drv_servo.h"

#ifdef __cplusplus
extern "C" {
#endif

void BSP_Board_Init(void);

void BSP_DelayMs(uint32_t ms);

const DRV_IMU_Bus   *BSP_Board_GetImuBus(void);
const DRV_BARO_Bus  *BSP_Board_GetBaroBus(void);
const DRV_GD25Q32_Bus *BSP_Board_GetFlashBus(void);
const DRV_MAG_Bus   *BSP_Board_GetMagBus(void);
const DRV_GPS_Bus   *BSP_Board_GetGpsBus(void);
const DRV_OPTICAL_FLOW_Bus *BSP_Board_GetOpticalFlowBus(void);
const DRV_SERVO_Bus *BSP_Board_GetServoBus(void);

#ifdef __cplusplus
}
#endif

#endif
