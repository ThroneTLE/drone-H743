#ifndef BSP_ICM42688_H
#define BSP_ICM42688_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define BSP_ICM42688_WHO_AM_I_VALUE 0x47U

/*
 * CubeMX / board requirements for this driver:
 * - SPI must run in 8-bit data size.
 * - SPI must use manual chip select (software NSS).
 * - SPI mode 0 is the default choice for bring-up.
 */

typedef enum {
    BSP_ICM42688_OK = 0,
    BSP_ICM42688_ERROR,
    BSP_ICM42688_TIMEOUT,
    BSP_ICM42688_BAD_ID,
    BSP_ICM42688_INVALID_ARG
} BSP_ICM42688_Status;

typedef enum {
    BSP_ICM42688_INIT_STAGE_NONE = 0,
    BSP_ICM42688_INIT_STAGE_BANK_SELECT,
    BSP_ICM42688_INIT_STAGE_RESET,
    BSP_ICM42688_INIT_STAGE_WHO_AM_I,
    BSP_ICM42688_INIT_STAGE_GYRO_CONFIG,
    BSP_ICM42688_INIT_STAGE_ACCEL_CONFIG,
    BSP_ICM42688_INIT_STAGE_FILTER_CONFIG,
    BSP_ICM42688_INIT_STAGE_PWR_MGMT,
    BSP_ICM42688_INIT_STAGE_SIGNAL_RESET,
    BSP_ICM42688_INIT_STAGE_READY
} BSP_ICM42688_InitStage;

typedef enum {
    BSP_ICM42688_ACCEL_RANGE_16G = 0U,
    BSP_ICM42688_ACCEL_RANGE_8G  = 1U,
    BSP_ICM42688_ACCEL_RANGE_4G  = 2U,
    BSP_ICM42688_ACCEL_RANGE_2G  = 3U
} BSP_ICM42688_AccelRange;

typedef enum {
    BSP_ICM42688_GYRO_RANGE_2000DPS  = 0U,
    BSP_ICM42688_GYRO_RANGE_1000DPS  = 1U,
    BSP_ICM42688_GYRO_RANGE_500DPS   = 2U,
    BSP_ICM42688_GYRO_RANGE_250DPS   = 3U,
    BSP_ICM42688_GYRO_RANGE_125DPS   = 4U,
    BSP_ICM42688_GYRO_RANGE_62D5DPS  = 5U,
    BSP_ICM42688_GYRO_RANGE_31D25DPS = 6U,
    BSP_ICM42688_GYRO_RANGE_15D625DPS = 7U
} BSP_ICM42688_GyroRange;

typedef enum {
    BSP_ICM42688_ODR_32KHZ   = 1U,
    BSP_ICM42688_ODR_16KHZ   = 2U,
    BSP_ICM42688_ODR_8KHZ    = 3U,
    BSP_ICM42688_ODR_4KHZ    = 4U,
    BSP_ICM42688_ODR_2KHZ    = 5U,
    BSP_ICM42688_ODR_1KHZ    = 6U,
    BSP_ICM42688_ODR_200HZ   = 7U,
    BSP_ICM42688_ODR_100HZ   = 8U,
    BSP_ICM42688_ODR_50HZ    = 9U,
    BSP_ICM42688_ODR_25HZ    = 10U,
    BSP_ICM42688_ODR_12D5HZ  = 11U,
    BSP_ICM42688_ODR_6D25HZ  = 12U,
    BSP_ICM42688_ODR_3D125HZ = 13U,
    BSP_ICM42688_ODR_1D5625HZ = 14U,
    BSP_ICM42688_ODR_500HZ   = 15U
} BSP_ICM42688_Odr;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           timeout_ms;
    void             (*delay_ms)(uint32_t ms);
} BSP_ICM42688_Bus;

typedef struct {
    BSP_ICM42688_AccelRange accel_range;
    BSP_ICM42688_GyroRange  gyro_range;
    BSP_ICM42688_Odr        accel_odr;
    BSP_ICM42688_Odr        gyro_odr;
    uint8_t                 accel_filter_bw;
    uint8_t                 gyro_filter_bw;
    bool                    enable_temp;
    bool                    soft_reset_on_init;
} BSP_ICM42688_Config;

typedef struct {
    int16_t temperature;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} BSP_ICM42688_RawData;

typedef struct {
    float temperature_c;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} BSP_ICM42688_ScaledData;

typedef struct {
    BSP_ICM42688_Bus    bus;
    BSP_ICM42688_Config config;
    uint8_t             who_am_i;
    BSP_ICM42688_InitStage init_stage;
    BSP_ICM42688_Status last_error;
} BSP_ICM42688_Device;

void BSP_ICM42688_DefaultConfig(BSP_ICM42688_Config *config);

BSP_ICM42688_Status BSP_ICM42688_Init(BSP_ICM42688_Device *dev,
                                      const BSP_ICM42688_Bus *bus,
                                      const BSP_ICM42688_Config *config);

BSP_ICM42688_Status BSP_ICM42688_Reset(BSP_ICM42688_Device *dev);
BSP_ICM42688_Status BSP_ICM42688_ReadWhoAmI(BSP_ICM42688_Device *dev, uint8_t *who_am_i);
BSP_ICM42688_Status BSP_ICM42688_ReadRegister(BSP_ICM42688_Device *dev,
                                              uint8_t reg,
                                              uint8_t *value);
BSP_ICM42688_Status BSP_ICM42688_WriteRegister(BSP_ICM42688_Device *dev,
                                               uint8_t reg,
                                               uint8_t value);
BSP_ICM42688_Status BSP_ICM42688_ReadRegisters(BSP_ICM42688_Device *dev,
                                               uint8_t reg,
                                               uint8_t *data,
                                               uint16_t len);
BSP_ICM42688_Status BSP_ICM42688_WriteRegisters(BSP_ICM42688_Device *dev,
                                                uint8_t reg,
                                                const uint8_t *data,
                                                uint16_t len);
BSP_ICM42688_Status BSP_ICM42688_ReadRaw(BSP_ICM42688_Device *dev,
                                         BSP_ICM42688_RawData *raw);
BSP_ICM42688_Status BSP_ICM42688_ReadScaled(BSP_ICM42688_Device *dev,
                                            BSP_ICM42688_ScaledData *scaled);
BSP_ICM42688_Status BSP_ICM42688_IsDataReady(BSP_ICM42688_Device *dev, bool *ready);

void BSP_ICM42688_ConvertRaw(const BSP_ICM42688_Device *dev,
                             const BSP_ICM42688_RawData *raw,
                             BSP_ICM42688_ScaledData *scaled);

float BSP_ICM42688_AccelLsbPerG(BSP_ICM42688_AccelRange range);
float BSP_ICM42688_GyroLsbPerDps(BSP_ICM42688_GyroRange range);

#ifdef __cplusplus
}
#endif

#endif
