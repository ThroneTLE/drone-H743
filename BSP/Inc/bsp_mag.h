#ifndef BSP_MAG_H
#define BSP_MAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "drv_mag.h"

typedef DRV_MAG_Status     BSP_MAG_StatusCode;
typedef DRV_MAG_Type       BSP_MAG_Type;
typedef DRV_MAG_RawData    BSP_MAG_RawData;
typedef DRV_MAG_ScaledData BSP_MAG_ScaledData;

#define BSP_MAG_OK          DRV_MAG_OK
#define BSP_MAG_ERROR       DRV_MAG_ERROR
#define BSP_MAG_TIMEOUT     DRV_MAG_TIMEOUT
#define BSP_MAG_BAD_ID      DRV_MAG_BAD_ID
#define BSP_MAG_INVALID_ARG DRV_MAG_INVALID_ARG
#define BSP_MAG_NOT_READY   DRV_MAG_NOT_READY
#define BSP_MAG_TYPE_NONE   DRV_MAG_TYPE_NONE
#define BSP_MAG_TYPE_IST8310  DRV_MAG_TYPE_IST8310
#define BSP_MAG_TYPE_HMC5883  DRV_MAG_TYPE_HMC5883
#define BSP_MAG_TYPE_QMC5883L DRV_MAG_TYPE_QMC5883L

typedef struct {
    DRV_MAG_Type        type;
    uint8_t             address;
    uint8_t             who_am_i;
    uint8_t             initialized;
    DRV_MAG_Status      last_status;
    uint32_t            sample_count;
    DRV_MAG_RawData     raw;
    DRV_MAG_ScaledData  scaled;
    uint8_t             detected_ist8310;
    uint8_t             detected_hmc5883;
    uint8_t             detected_qmc5883;
    uint8_t             hmc_id[3];
} BSP_MAG_Status;

DRV_MAG_Status BSP_MAG_Init(void);
DRV_MAG_Status BSP_MAG_Read(DRV_MAG_RawData *raw, DRV_MAG_ScaledData *scaled);
DRV_MAG_Status BSP_MAG_Probe(BSP_MAG_Status *status);
void BSP_MAG_GetStatus(BSP_MAG_Status *status);
void BSP_MAG_Invalidate(void);
const char *BSP_MAG_TypeName(DRV_MAG_Type type);

#ifdef __cplusplus
}
#endif

#endif
