#ifndef DRV_IMU_H
#define DRV_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define DRV_IMU_CHIP_ID_VALUE    0x47U
#define DRV_IMU_WHO_AM_I_VALUE   DRV_IMU_CHIP_ID_VALUE

typedef enum {
    DRV_IMU_OK = 0,
    DRV_IMU_ERROR,
    DRV_IMU_TIMEOUT,
    DRV_IMU_BAD_ID,
    DRV_IMU_INVALID_ARG
} DRV_IMU_Status;

typedef enum {
    DRV_IMU_INIT_STAGE_NONE = 0,
    DRV_IMU_INIT_STAGE_BANK_SELECT,
    DRV_IMU_INIT_STAGE_RESET,
    DRV_IMU_INIT_STAGE_WHO_AM_I,
    DRV_IMU_INIT_STAGE_GYRO_CONFIG,
    DRV_IMU_INIT_STAGE_ACCEL_CONFIG,
    DRV_IMU_INIT_STAGE_FILTER_CONFIG,
    DRV_IMU_INIT_STAGE_PWR_MGMT,
    DRV_IMU_INIT_STAGE_SIGNAL_RESET,
    DRV_IMU_INIT_STAGE_READY
} DRV_IMU_InitStage;

typedef enum {
    DRV_IMU_ACCEL_RANGE_16G = 0U,
    DRV_IMU_ACCEL_RANGE_8G  = 1U,
    DRV_IMU_ACCEL_RANGE_4G  = 2U,
    DRV_IMU_ACCEL_RANGE_2G  = 3U
} DRV_IMU_AccelRange;

typedef enum {
    DRV_IMU_GYRO_RANGE_2000DPS  = 0U,
    DRV_IMU_GYRO_RANGE_1000DPS  = 1U,
    DRV_IMU_GYRO_RANGE_500DPS   = 2U,
    DRV_IMU_GYRO_RANGE_250DPS   = 3U,
    DRV_IMU_GYRO_RANGE_125DPS   = 4U,
    DRV_IMU_GYRO_RANGE_62D5DPS  = 5U,
    DRV_IMU_GYRO_RANGE_31D25DPS = 6U,
    DRV_IMU_GYRO_RANGE_15D625DPS = 7U
} DRV_IMU_GyroRange;

typedef enum {
    DRV_IMU_ODR_32KHZ   = 1U,
    DRV_IMU_ODR_16KHZ   = 2U,
    DRV_IMU_ODR_8KHZ    = 3U,
    DRV_IMU_ODR_4KHZ    = 4U,
    DRV_IMU_ODR_2KHZ    = 5U,
    DRV_IMU_ODR_1KHZ    = 6U,
    DRV_IMU_ODR_200HZ   = 7U,
    DRV_IMU_ODR_100HZ   = 8U,
    DRV_IMU_ODR_50HZ    = 9U,
    DRV_IMU_ODR_25HZ    = 10U,
    DRV_IMU_ODR_12D5HZ  = 11U,
    DRV_IMU_ODR_6D25HZ  = 12U,
    DRV_IMU_ODR_3D125HZ = 13U,
    DRV_IMU_ODR_1D5625HZ = 14U,
    DRV_IMU_ODR_500HZ   = 15U
} DRV_IMU_Odr;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           timeout_ms;
    void             (*delay_ms)(uint32_t ms);
} DRV_IMU_Bus;

typedef struct {
    DRV_IMU_AccelRange accel_range;
    DRV_IMU_GyroRange  gyro_range;
    DRV_IMU_Odr        accel_odr;
    DRV_IMU_Odr        gyro_odr;
    uint8_t            accel_filter_bw;
    uint8_t            gyro_filter_bw;
    bool               enable_temp;
    bool               soft_reset_on_init;
} DRV_IMU_Config;

typedef struct {
    int16_t temperature;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} DRV_IMU_RawData;

typedef struct {
    float temperature_c;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} DRV_IMU_ScaledData;

typedef struct {
    DRV_IMU_Bus       bus;
    DRV_IMU_Config    config;
    uint8_t           who_am_i;
    DRV_IMU_InitStage init_stage;
    DRV_IMU_Status    last_error;
} DRV_IMU_Device;

void DRV_IMU_DefaultConfig(DRV_IMU_Config *config);

DRV_IMU_Status DRV_IMU_Init(DRV_IMU_Device *dev,
                            const DRV_IMU_Bus *bus,
                            const DRV_IMU_Config *config);

DRV_IMU_Status DRV_IMU_Reset(DRV_IMU_Device *dev);
DRV_IMU_Status DRV_IMU_ReadWhoAmI(DRV_IMU_Device *dev, uint8_t *who_am_i);
DRV_IMU_Status DRV_IMU_ReadRegister(DRV_IMU_Device *dev, uint8_t reg, uint8_t *value);
DRV_IMU_Status DRV_IMU_WriteRegister(DRV_IMU_Device *dev, uint8_t reg, uint8_t value);
DRV_IMU_Status DRV_IMU_ReadRegisters(DRV_IMU_Device *dev, uint8_t reg,
                                     uint8_t *data, uint16_t len);
DRV_IMU_Status DRV_IMU_WriteRegisters(DRV_IMU_Device *dev, uint8_t reg,
                                      const uint8_t *data, uint16_t len);
DRV_IMU_Status DRV_IMU_ReadRaw(DRV_IMU_Device *dev, DRV_IMU_RawData *raw);
DRV_IMU_Status DRV_IMU_ReadScaled(DRV_IMU_Device *dev, DRV_IMU_ScaledData *scaled);
DRV_IMU_Status DRV_IMU_IsDataReady(DRV_IMU_Device *dev, bool *ready);

void DRV_IMU_ConvertRaw(const DRV_IMU_Device *dev,
                        const DRV_IMU_RawData *raw,
                        DRV_IMU_ScaledData *scaled);

float DRV_IMU_AccelLsbPerG(DRV_IMU_AccelRange range);
float DRV_IMU_GyroLsbPerDps(DRV_IMU_GyroRange range);

#ifdef __cplusplus
}
#endif

#endif
