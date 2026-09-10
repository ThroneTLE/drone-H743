#ifndef DRV_IMU_H
#define DRV_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 值类型（Status / InitStage / Range / Odr / Config / RawData / ScaledData）
 * 住在无 HAL 的 drv_imu_types.h 里，供换算表与宿主侧单测复用。本头继续原样导出
 * 它们，所以既有的 #include "drv_imu.h" 一处都不用改。
 */
#include "drv_imu_types.h"

#include <stdbool.h>
#include <stdint.h>

#define DRV_IMU_CHIP_ID_VALUE    0x47U
#define DRV_IMU_WHO_AM_I_VALUE   DRV_IMU_CHIP_ID_VALUE

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           timeout_ms;
    void             (*delay_ms)(uint32_t ms);
} DRV_IMU_Bus;

typedef struct {
    DRV_IMU_Bus       bus;
    DRV_IMU_Config    config;
    uint8_t           who_am_i;
    DRV_IMU_InitStage init_stage;
    DRV_IMU_Status    last_error;
    /* AAF cutoffs actually programmed, after snapping to supported values. */
    uint16_t          accel_aaf_actual_hz;
    uint16_t          gyro_aaf_actual_hz;
} DRV_IMU_Device;

void DRV_IMU_DefaultConfig(DRV_IMU_Config *config);

/*
 * Resolve a requested AAF cutoff to the nearest supported hardware setting.
 * Returns an opaque handle to the driver's internal coefficient entry and, when
 * actual_hz is non-NULL, the cutoff that will really be programmed. Exposed so
 * tests can assert the snapping behaviour.
 */
const void *DRV_IMU_AafSettingForCutoff(uint16_t desired_hz,
                                        uint16_t *actual_hz);

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
