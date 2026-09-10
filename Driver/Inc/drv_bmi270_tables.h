#ifndef DRV_BMI270_TABLES_H
#define DRV_BMI270_TABLES_H

/*
 * BMI270 的量程 / ODR / 带宽换算表 —— **纯函数，不含 HAL**。
 * 拆分理由同 drv_bmi088_tables.h：让宿主 gcc 能直接编译并单测这些映射。
 */

#include "drv_imu_types.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ACC_RANGE(0x41)：0=±2g 1=±4g 2=±8g 3=±16g（注意与通用枚举的顺序相反） */
#define DRV_BMI270_ACC_RANGE_2G  0x00U
#define DRV_BMI270_ACC_RANGE_4G  0x01U
#define DRV_BMI270_ACC_RANGE_8G  0x02U
#define DRV_BMI270_ACC_RANGE_16G 0x03U

/* GYR_RANGE(0x43) bit[2:0]：0=±2000 1=±1000 2=±500 3=±250 4=±125 dps */
#define DRV_BMI270_GYR_RANGE_2000DPS 0x00U
#define DRV_BMI270_GYR_RANGE_1000DPS 0x01U
#define DRV_BMI270_GYR_RANGE_500DPS  0x02U
#define DRV_BMI270_GYR_RANGE_250DPS  0x03U
#define DRV_BMI270_GYR_RANGE_125DPS  0x04U

/*
 * GYR_RANGE bit3 = ois_range。
 *
 * 这一位**必须置 1**，否则在某些数据通路下主量程会悄悄退化成 ±250 dps ——
 * datasheet 没有记载，是 ArduPilot 在 AP_InertialSensor_BMI270.cpp 里踩出来并
 * 留下注释的（"or else the gyro scale will be 250dps ... not documented"）。
 * 这类错误不报错、不崩溃，只让角速度整体差 8 倍，是最难查的一类。
 */
#define DRV_BMI270_GYR_RANGE_OIS_BIT 0x08U

/* 过采样码：加计在 ACC_CONF bit[6:4]，陀螺在 GYR_CONF bit[5:4] */
#define DRV_BMI270_BWP_OSR4   0x00U
#define DRV_BMI270_BWP_OSR2   0x01U
#define DRV_BMI270_BWP_NORMAL 0x02U

uint8_t DRV_BMI270_AccelRangeCode(DRV_IMU_AccelRange range);
float   DRV_BMI270_AccelLsbPerG(DRV_IMU_AccelRange range);
uint8_t DRV_BMI270_GyroRangeCode(DRV_IMU_GyroRange range);
float   DRV_BMI270_GyroLsbPerDps(DRV_IMU_GyroRange range);

/*
 * ODR 码（ACC_CONF / GYR_CONF 的 bit[3:0]）。
 * BMI270 加计与陀螺共用同一套 ODR 编码：0x08=100Hz 起，每档翻倍到 0x0C=1600Hz，
 * 陀螺多一档 0x0D=3200Hz。**没有 1000 Hz 这一档**，请求 1 kHz 时落到 800 Hz。
 */
uint8_t DRV_BMI270_OdrCode(DRV_IMU_Odr odr, uint8_t is_gyro, uint16_t *actual_odr_hz);

/*
 * 3dB 截止频率按 ODR 的固定比例给出（datasheet 的 bwp 比例，两个锚点已被实测确认：
 * 加计 osr4 @1600Hz = 188Hz、陀螺 normal @3200Hz = 751Hz，均见 ArduPilot 注释）。
 * 选"不超过请求带宽的最高档"。
 */
uint8_t DRV_BMI270_BwpCode(uint16_t odr_hz, uint16_t desired_bw_hz,
                           uint8_t is_gyro, uint16_t *actual_bw_hz);

uint8_t DRV_BMI270_BuildAccConf(DRV_IMU_Odr odr, uint16_t desired_bw_hz,
                                uint16_t *actual_bw_hz);
uint8_t DRV_BMI270_BuildGyrConf(DRV_IMU_Odr odr, uint16_t desired_bw_hz,
                                uint16_t *actual_bw_hz);

void DRV_BMI270_ConvertRaw(DRV_IMU_AccelRange accel_range,
                           DRV_IMU_GyroRange gyro_range,
                           const DRV_IMU_RawData *raw,
                           DRV_IMU_ScaledData *scaled);

/* 16 位有符号、1/512 K per LSB、偏置 23 °C；0x8000 表示无效。 */
float DRV_BMI270_ConvertTemperature(uint8_t lsb, uint8_t msb, uint8_t *valid);

#ifdef __cplusplus
}
#endif

#endif
