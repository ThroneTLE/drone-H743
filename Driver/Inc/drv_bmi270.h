#ifndef DRV_BMI270_H
#define DRV_BMI270_H

/*
 * Bosch BMI270 驱动（MicoAir743v2 板载第二颗 IMU，挂 SPI3，CS = PA15）。
 *
 * 与 BMI088 的三个关键差异：
 *   1. 单芯片六轴，一个片选就够；
 *   2. **上电必须灌入 328 字节初始化配置**（drv_bmi270_config.h），不灌的话
 *      寄存器读写全部"成功"但数据恒为 0，且 INTERNAL_STATUS 永远不等于 1；
 *   3. GYR_RANGE 的 ois_range 位有一个 datasheet 未记载的坑，见 drv_bmi270_tables.h。
 *
 * 和 BMI088 相同的一点：SPI 读在地址字节后有一个 dummy 字节要丢弃，
 * 且上电后需要先做一次丢弃结果的 CHIP_ID 读把接口锁进 SPI 模式。
 *
 * 输出契约：ReadScaled 给的是**芯片自身轴向**的 g 与 dps，机体变换归 svc_imu。
 */

#include "drv_bmi270_config.h"
#include "drv_bmi270_tables.h"
#include "drv_imu.h"
#include "drv_imu_iface.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_BMI270_CHIP_ID 0x24U

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           timeout_ms;
    void             (*delay_ms)(uint32_t ms);
} DRV_BMI270_Bus;

typedef struct {
    DRV_BMI270_Bus    bus;
    DRV_IMU_Config    config;
    uint8_t           chip_id;
    uint8_t           internal_status;   /* INTERNAL_STATUS(0x21) 的最后一次读数 */
    uint8_t           config_upload_tries;
    DRV_IMU_InitStage init_stage;
    DRV_IMU_Status    last_error;
    uint16_t          accel_bandwidth_actual_hz;
    uint16_t          gyro_bandwidth_actual_hz;
} DRV_BMI270_Device;

const DRV_IMU_Ops *DRV_BMI270_GetOps(void);

DRV_IMU_Status DRV_BMI270_Probe(DRV_BMI270_Device *dev, uint8_t *chip_id);
DRV_IMU_Status DRV_BMI270_Init(DRV_BMI270_Device *dev, const DRV_IMU_Config *config);
DRV_IMU_Status DRV_BMI270_ReadRaw(DRV_BMI270_Device *dev, DRV_IMU_RawData *raw);
DRV_IMU_Status DRV_BMI270_ReadScaled(DRV_BMI270_Device *dev, DRV_IMU_ScaledData *scaled);
DRV_IMU_Status DRV_BMI270_IsDataReady(DRV_BMI270_Device *dev, bool *ready);

#ifdef __cplusplus
}
#endif

#endif
