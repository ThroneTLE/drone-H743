#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "drv_imu.h"
#include "svc_imu.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef DRV_IMU_Status      BSP_ICM42688_Status;
typedef DRV_IMU_InitStage   BSP_ICM42688_InitStage;
typedef DRV_IMU_AccelRange  BSP_ICM42688_AccelRange;
typedef DRV_IMU_GyroRange   BSP_ICM42688_GyroRange;
typedef DRV_IMU_Odr         BSP_ICM42688_Odr;
typedef DRV_IMU_Bus         BSP_ICM42688_Bus;
typedef DRV_IMU_Config      BSP_ICM42688_Config;
typedef DRV_IMU_RawData     BSP_ICM42688_RawData;
typedef DRV_IMU_ScaledData  BSP_ICM42688_ScaledData;
typedef DRV_IMU_Device      BSP_ICM42688_Device;

#define BSP_ICM42688_CHIP_ID_VALUE   DRV_IMU_CHIP_ID_VALUE
#define BSP_ICM42688_WHO_AM_I_VALUE  DRV_IMU_WHO_AM_I_VALUE
#define BSP_ICM42688_OK              DRV_IMU_OK
#define BSP_ICM42688_ERROR           DRV_IMU_ERROR
#define BSP_ICM42688_TIMEOUT         DRV_IMU_TIMEOUT
#define BSP_ICM42688_BAD_ID          DRV_IMU_BAD_ID
#define BSP_ICM42688_INVALID_ARG     DRV_IMU_INVALID_ARG

#define BSP_ICM42688_ACCEL_RANGE_16G  DRV_IMU_ACCEL_RANGE_16G
#define BSP_ICM42688_ACCEL_RANGE_8G   DRV_IMU_ACCEL_RANGE_8G
#define BSP_ICM42688_ACCEL_RANGE_4G   DRV_IMU_ACCEL_RANGE_4G
#define BSP_ICM42688_ACCEL_RANGE_2G   DRV_IMU_ACCEL_RANGE_2G
#define BSP_ICM42688_GYRO_RANGE_2000DPS   DRV_IMU_GYRO_RANGE_2000DPS
#define BSP_ICM42688_GYRO_RANGE_1000DPS   DRV_IMU_GYRO_RANGE_1000DPS
#define BSP_ICM42688_GYRO_RANGE_500DPS    DRV_IMU_GYRO_RANGE_500DPS
#define BSP_ICM42688_GYRO_RANGE_250DPS    DRV_IMU_GYRO_RANGE_250DPS
#define BSP_ICM42688_GYRO_RANGE_125DPS    DRV_IMU_GYRO_RANGE_125DPS
#define BSP_ICM42688_GYRO_RANGE_62D5DPS   DRV_IMU_GYRO_RANGE_62D5DPS
#define BSP_ICM42688_GYRO_RANGE_31D25DPS  DRV_IMU_GYRO_RANGE_31D25DPS
#define BSP_ICM42688_GYRO_RANGE_15D625DPS DRV_IMU_GYRO_RANGE_15D625DPS
#define BSP_ICM42688_ODR_100HZ            DRV_IMU_ODR_100HZ
#define BSP_ICM42688_ODR_32KHZ          DRV_IMU_ODR_32KHZ
#define BSP_ICM42688_ODR_16KHZ          DRV_IMU_ODR_16KHZ
#define BSP_ICM42688_ODR_8KHZ           DRV_IMU_ODR_8KHZ
#define BSP_ICM42688_ODR_4KHZ           DRV_IMU_ODR_4KHZ
#define BSP_ICM42688_ODR_2KHZ           DRV_IMU_ODR_2KHZ
#define BSP_ICM42688_ODR_1KHZ           DRV_IMU_ODR_1KHZ
#define BSP_ICM42688_ODR_200HZ          DRV_IMU_ODR_200HZ
#define BSP_ICM42688_ODR_50HZ           DRV_IMU_ODR_50HZ
#define BSP_ICM42688_ODR_25HZ           DRV_IMU_ODR_25HZ
#define BSP_ICM42688_ODR_12D5HZ         DRV_IMU_ODR_12D5HZ
#define BSP_ICM42688_ODR_6D25HZ         DRV_IMU_ODR_6D25HZ
#define BSP_ICM42688_ODR_3D125HZ        DRV_IMU_ODR_3D125HZ
#define BSP_ICM42688_ODR_1D5625HZ       DRV_IMU_ODR_1D5625HZ
#define BSP_ICM42688_ODR_500HZ          DRV_IMU_ODR_500HZ

#define BSP_ICM42688_INIT_STAGE_NONE          DRV_IMU_INIT_STAGE_NONE
#define BSP_ICM42688_INIT_STAGE_BANK_SELECT   DRV_IMU_INIT_STAGE_BANK_SELECT
#define BSP_ICM42688_INIT_STAGE_RESET         DRV_IMU_INIT_STAGE_RESET
#define BSP_ICM42688_INIT_STAGE_WHO_AM_I      DRV_IMU_INIT_STAGE_WHO_AM_I
#define BSP_ICM42688_INIT_STAGE_GYRO_CONFIG   DRV_IMU_INIT_STAGE_GYRO_CONFIG
#define BSP_ICM42688_INIT_STAGE_ACCEL_CONFIG  DRV_IMU_INIT_STAGE_ACCEL_CONFIG
#define BSP_ICM42688_INIT_STAGE_FILTER_CONFIG DRV_IMU_INIT_STAGE_FILTER_CONFIG
#define BSP_ICM42688_INIT_STAGE_PWR_MGMT      DRV_IMU_INIT_STAGE_PWR_MGMT
#define BSP_ICM42688_INIT_STAGE_SIGNAL_RESET  DRV_IMU_INIT_STAGE_SIGNAL_RESET
#define BSP_ICM42688_INIT_STAGE_READY         DRV_IMU_INIT_STAGE_READY

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

/*
 * 芯片无关的 IMU 概况。
 *
 * 板上可能是 BMI088 / BMI270 / ICM-42688 中的任意一颗，三者的设备结构体各不相同，
 * 所以上层不能再直接读 DRV_IMU_Device。需要"当前量程 / 带宽 / 初始化到哪一步"
 * 的地方一律走这个结构体。
 */
typedef struct {
    DRV_IMU_ChipKind   kind;
    uint8_t            chip_id;
    uint8_t            initialized;
    DRV_IMU_InitStage  init_stage;
    DRV_IMU_Status     last_error;
    DRV_IMU_AccelRange accel_range;
    DRV_IMU_GyroRange  gyro_range;
    /* 实际生效的抗混叠 / 数字带宽，各芯片按支持值向下取整后回报。 */
    uint16_t           accel_bandwidth_hz;
    uint16_t           gyro_bandwidth_hz;
} BSP_IMU_Info;

DRV_IMU_Status BSP_IMU_Init(void);
DRV_IMU_Status BSP_IMU_ReadRaw(DRV_IMU_RawData *raw);
DRV_IMU_Status BSP_IMU_ReadScaled(DRV_IMU_ScaledData *scaled);
DRV_IMU_Status BSP_IMU_IsDataReady(bool *ready);
uint8_t BSP_IMU_GetWhoAmI(void);
void BSP_IMU_GetDiag(BSP_IMU_Diag *diag);
void BSP_IMU_Invalidate(void);

void             BSP_IMU_GetInfo(BSP_IMU_Info *info);
DRV_IMU_ChipKind BSP_IMU_GetChipKind(void);

/* 探测记账：每颗候选芯片读到的 ID 与结果，供诊断命令原样回报。 */
void BSP_IMU_GetSelection(SVC_IMU_Selection *selection);

/*
 * 原始计数 → 物理量，按当前选中的芯片换算。
 * 各芯片的量程刻度不同（BMI088 是 ±3/6/12/24 g，ICM 是 ±2/4/8/16 g），
 * 所以不能在上层用一份固定的 LSB 表。
 */
void BSP_IMU_RawToScaled(const DRV_IMU_RawData *raw, DRV_IMU_ScaledData *scaled);

/*
 * 仅当选中的是 ICM-42688 时返回其设备结构体，否则返回 NULL。
 * 保留它只为兼容既有调用点；新代码请用 BSP_IMU_GetInfo()。
 */
const DRV_IMU_Device *BSP_IMU_GetDevice(void);

#ifdef __cplusplus
}
#endif

#endif
