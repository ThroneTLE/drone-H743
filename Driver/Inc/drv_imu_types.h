#ifndef DRV_IMU_TYPES_H
#define DRV_IMU_TYPES_H

/*
 * IMU 的公共值类型 —— **不含 HAL、不含 RTOS**。
 *
 * 为什么单独拆一个头：这些枚举和数据结构原本住在 `drv_imu.h` 里，而那个头必须
 * `#include "main.h"` 才能声明 SPI 句柄和片选端口。结果是任何想复用这些类型的
 * 纯逻辑代码（量程换算表、装配旋转、宿主侧单测）都被迫拖进整套 HAL。
 *
 * 拆开之后：
 *   drv_imu_types.h  —— 纯值类型，host gcc 可直接编译（decoupling-spec D5-1）
 *   drv_imu.h        —— 包含本文件，再加 SPI 总线句柄与 ICM-42688 的函数声明
 *
 * `drv_imu.h` 仍然导出这里的全部内容，所以既有的 `#include "drv_imu.h"` 一处都不用改。
 *
 * 单位与符号约定（跨模块契约，D4-3）：
 *   RawData    —— 芯片原始计数，芯片自身轴向，未去零偏。
 *   ScaledData —— 加速度 g、角速度 dps、温度 °C，**仍是芯片自身轴向**。
 *                 芯片轴 → 机体 FLU 的装配旋转由 Services/svc_imu 负责，驱动不做。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRV_IMU_OK = 0,
    DRV_IMU_ERROR,
    DRV_IMU_TIMEOUT,
    DRV_IMU_BAD_ID,
    DRV_IMU_INVALID_ARG
} DRV_IMU_Status;

/*
 * 板上可能装的 IMU 型号。放在值类型里（而不是 drv_imu_iface.h）是为了让
 * Services 层能在**不引入 HAL** 的前提下按型号选装配旋转 —— 函数表那一头
 * 带 SPI 句柄，Services 不许碰（decoupling-spec D1-2）。
 */
typedef enum {
    DRV_IMU_CHIP_NONE = 0,
    DRV_IMU_CHIP_ICM42688,
    DRV_IMU_CHIP_BMI088,
    DRV_IMU_CHIP_BMI270
} DRV_IMU_ChipKind;

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

/*
 * 量程枚举沿用 ICM-42688 的刻度命名。BMI088 的物理刻度是 ±3/6/12/24 g，
 * 与这里的 2/4/8/16 g 对不上，由 drv_bmi088_tables.c 做一次显式映射，
 * 映射规则写在那里，不在这里假装两者相同。
 */
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
    DRV_IMU_AccelRange accel_range;
    DRV_IMU_GyroRange  gyro_range;
    DRV_IMU_Odr        accel_odr;
    DRV_IMU_Odr        gyro_odr;
    uint8_t            accel_filter_bw;
    uint8_t            gyro_filter_bw;
    /*
     * Requested anti-alias filter cutoffs in Hz. The hardware only supports a
     * fixed set of cutoffs, so the driver selects the highest supported value
     * not exceeding the request and reports it back in the device struct.
     *
     * BMI088/BMI270 没有 ICM-42688 那种独立 AAF，这两个字段在它们的驱动里
     * 被解释为"期望的数字带宽"，同样按支持值向下取整。
     */
    uint16_t           accel_aaf_hz;
    uint16_t           gyro_aaf_hz;
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

#ifdef __cplusplus
}
#endif

#endif
