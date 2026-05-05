#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "bsp_icm42688.h"

#ifdef __cplusplus
extern "C" {
#endif

BSP_ICM42688_Status BSP_IMU_Init(void);
BSP_ICM42688_Status BSP_IMU_ReadRaw(BSP_ICM42688_RawData *raw);
BSP_ICM42688_Status BSP_IMU_ReadScaled(BSP_ICM42688_ScaledData *scaled);
BSP_ICM42688_Status BSP_IMU_IsDataReady(bool *ready);
uint8_t BSP_IMU_GetWhoAmI(void);
const BSP_ICM42688_Device *BSP_IMU_GetDevice(void);
void BSP_IMU_Invalidate(void);

#ifdef __cplusplus
}
#endif

#endif
