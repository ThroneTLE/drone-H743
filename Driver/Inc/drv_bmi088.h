#ifndef DRV_BMI088_H
#define DRV_BMI088_H

/*
 * Bosch BMI088 驱动（MicoAir743v2 板载主 IMU，挂 SPI2）。
 *
 * BMI088 在一个封装里是**两颗独立的芯片**：加速度计和陀螺仪各有自己的片选、
 * 自己的寄存器空间、自己的 CHIP_ID 和自己的 DRDY 引脚。所以这里的 Bus 有两个 CS，
 * 而不是复用 DRV_IMU_Bus 的单片选。
 *
 * 两个必须注意的芯片怪癖（写错了表现为"读出来全 0 或全 FF"）：
 *   1. 加计的 SPI 读在地址字节之后**多一个 dummy 字节**才是数据；陀螺没有。
 *   2. 加计上电后处于 suspend 且总线模式未定，必须先做一次**丢弃结果的 CHIP_ID 读**，
 *      用那次 CSB 上升沿把它锁进 SPI 模式，之后的读才有效。软复位后同样要再做一次。
 *
 * 输出契约：ReadScaled 给的是**芯片自身轴向**的 g 与 dps，不做机体变换。
 * 芯片轴 → 机体 FLU 的装配旋转归 Services/svc_imu（decoupling-spec D4-2）。
 */

#include "drv_bmi088_tables.h"
#include "drv_imu.h"
#include "drv_imu_iface.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_BMI088_ACC_CHIP_ID   0x1EU
#define DRV_BMI088_GYRO_CHIP_ID  0x0FU

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *acc_cs_port;
    uint16_t           acc_cs_pin;
    GPIO_TypeDef      *gyro_cs_port;
    uint16_t           gyro_cs_pin;
    uint32_t           timeout_ms;
    void             (*delay_ms)(uint32_t ms);
} DRV_BMI088_Bus;

typedef struct {
    DRV_BMI088_Bus    bus;
    DRV_IMU_Config    config;
    uint8_t           acc_chip_id;
    uint8_t           gyro_chip_id;
    DRV_IMU_InitStage init_stage;
    DRV_IMU_Status    last_error;
    /* 实际写进芯片的带宽，按支持值向下取整后回报，便于上位机核对。 */
    uint16_t          accel_bandwidth_actual_hz;
    uint16_t          gyro_bandwidth_actual_hz;
    float             accel_lsb_per_g;
    float             gyro_lsb_per_dps;
} DRV_BMI088_Device;

const DRV_IMU_Ops *DRV_BMI088_GetOps(void);

DRV_IMU_Status DRV_BMI088_Probe(DRV_BMI088_Device *dev, uint8_t *chip_id);
DRV_IMU_Status DRV_BMI088_Init(DRV_BMI088_Device *dev, const DRV_IMU_Config *config);
DRV_IMU_Status DRV_BMI088_ReadRaw(DRV_BMI088_Device *dev, DRV_IMU_RawData *raw);
DRV_IMU_Status DRV_BMI088_ReadScaled(DRV_BMI088_Device *dev, DRV_IMU_ScaledData *scaled);
DRV_IMU_Status DRV_BMI088_IsDataReady(DRV_BMI088_Device *dev, bool *ready);

/*
 * 量程 / ODR / 带宽换算表在 drv_bmi088_tables.h（纯函数、无 HAL、宿主可编译）。
 * 本头已经把它 include 进来，用的人不必再单独引。
 */

#ifdef __cplusplus
}
#endif

#endif
