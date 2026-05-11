#ifndef DRV_MAG_H
#define DRV_MAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

typedef enum {
    DRV_MAG_OK = 0,
    DRV_MAG_ERROR,
    DRV_MAG_TIMEOUT,
    DRV_MAG_BAD_ID,
    DRV_MAG_INVALID_ARG,
    DRV_MAG_NOT_READY
} DRV_MAG_Status;

typedef enum {
    DRV_MAG_TYPE_NONE = 0,
    DRV_MAG_TYPE_IST8310,
    DRV_MAG_TYPE_HMC5883,
    DRV_MAG_TYPE_QMC5883L
} DRV_MAG_Type;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} DRV_MAG_RawData;

typedef struct {
    int32_t x_mgauss;
    int32_t y_mgauss;
    int32_t z_mgauss;
} DRV_MAG_ScaledData;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint32_t           timeout_ms;
} DRV_MAG_Bus;

typedef struct {
    DRV_MAG_Type     type;
    uint8_t          address;
    uint8_t          who_am_i;
    uint8_t          hmc_id[3];
    uint8_t          detected_ist8310;
    uint8_t          detected_hmc5883;
    uint8_t          detected_qmc5883;
} DRV_MAG_Info;

typedef struct {
    DRV_MAG_Bus      bus;
    DRV_MAG_Info     info;
    uint8_t          initialized;
    DRV_MAG_Status   last_status;
    uint32_t         sample_count;
    DRV_MAG_RawData  raw;
    DRV_MAG_ScaledData scaled;
} DRV_MAG_Device;

DRV_MAG_Status DRV_MAG_Init(DRV_MAG_Device *dev, const DRV_MAG_Bus *bus);
DRV_MAG_Status DRV_MAG_Read(DRV_MAG_Device *dev, DRV_MAG_RawData *raw,
                            DRV_MAG_ScaledData *scaled);
void DRV_MAG_Invalidate(DRV_MAG_Device *dev);
const char *DRV_MAG_TypeName(DRV_MAG_Type type);

#ifdef __cplusplus
}
#endif

#endif
