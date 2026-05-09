#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "bsp_icm42688.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t mode0_tokmas;
    uint8_t mode0_msb;
    uint8_t mode0_bit0;
    uint8_t mode3_tokmas;
    uint8_t mode3_msb;
    uint8_t mode3_bit0;
    uint8_t burst_m0_b0_1;
    uint8_t burst_m0_b0_2;
    uint8_t burst_m0_b0_3;
    uint8_t burst_m0_b0_4;
    uint8_t burst_m3_tok_1;
    uint8_t burst_m3_tok_2;
    uint8_t burst_m3_tok_3;
    uint8_t burst_m3_tok_4;
    uint8_t best_mode;
    uint8_t best_header;
    uint8_t valid;
} BSP_IMU_Diag;

BSP_ICM42688_Status BSP_IMU_Init(void);
BSP_ICM42688_Status BSP_IMU_ReadRaw(BSP_ICM42688_RawData *raw);
BSP_ICM42688_Status BSP_IMU_ReadScaled(BSP_ICM42688_ScaledData *scaled);
BSP_ICM42688_Status BSP_IMU_IsDataReady(bool *ready);
uint8_t BSP_IMU_GetWhoAmI(void);
void BSP_IMU_GetDiag(BSP_IMU_Diag *diag);
const BSP_ICM42688_Device *BSP_IMU_GetDevice(void);
void BSP_IMU_Invalidate(void);

#ifdef __cplusplus
}
#endif

#endif
