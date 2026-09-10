#ifndef DRV_BMI088_TABLES_H
#define DRV_BMI088_TABLES_H

/*
 * BMI088 的量程 / ODR / 带宽换算表 —— **纯函数，不含 HAL**。
 *
 * 单独成文件的理由：这些映射是最容易写错、也最该被单测钉住的部分（量程码填错
 * 表现为"姿态角差一个固定倍数"，不会报错），而传输层要 HAL，宿主 gcc 编译不了。
 * 拆开后 tests/ 可以直接编译本文件断言每一条映射（decoupling-spec D5-1）。
 */

#include "drv_imu_types.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ACC_RANGE(0x41) 取值：0=±3g 1=±6g 2=±12g 3=±24g */
#define DRV_BMI088_ACC_RANGE_3G   0x00U
#define DRV_BMI088_ACC_RANGE_6G   0x01U
#define DRV_BMI088_ACC_RANGE_12G  0x02U
#define DRV_BMI088_ACC_RANGE_24G  0x03U

/* GYRO_RANGE(0x0F) 取值：0=±2000 1=±1000 2=±500 3=±250 4=±125 dps */
#define DRV_BMI088_GYRO_RANGE_2000DPS 0x00U
#define DRV_BMI088_GYRO_RANGE_1000DPS 0x01U
#define DRV_BMI088_GYRO_RANGE_500DPS  0x02U
#define DRV_BMI088_GYRO_RANGE_250DPS  0x03U
#define DRV_BMI088_GYRO_RANGE_125DPS  0x04U

/* ACC_CONF(0x40) 的 acc_bwp 字段（bit[7:4]） */
#define DRV_BMI088_ACC_BWP_OSR4   0x08U
#define DRV_BMI088_ACC_BWP_OSR2   0x09U
#define DRV_BMI088_ACC_BWP_NORMAL 0x0AU

/*
 * 通用量程枚举 → BMI088 寄存器码。
 *
 * 注意刻度对不上：通用枚举是 ICM-42688 的 ±2/4/8/16 g，BMI088 物理上是 ±3/6/12/24 g。
 * 映射规则是"取不小于请求量程的最小档"，即 2G→±3g、4G→±6g、8G→±12g、16G→±24g。
 * 这样换算出来的物理量永远不会削顶，代价是分辨率略有富余。
 */
uint8_t DRV_BMI088_AccelRangeCode(DRV_IMU_AccelRange range);
float   DRV_BMI088_AccelLsbPerG(DRV_IMU_AccelRange range);

/*
 * 陀螺量程。BMI088 最细只到 ±125 dps，比 ±125 更细的三档（62.5/31.25/15.625）
 * 一律夹到 ±125，不静默按错误刻度换算。
 */
uint8_t DRV_BMI088_GyroRangeCode(DRV_IMU_GyroRange range);
float   DRV_BMI088_GyroLsbPerDps(DRV_IMU_GyroRange range);

/*
 * 加计 ACC_CONF：ODR 码（bit[3:0]）与过采样码（bit[7:4]）。
 * BMI088 加计的 ODR 档位是 12.5/25/50/100/200/400/800/1600 Hz，没有 1 kHz 这一档，
 * 所以请求 1 kHz 时取 1600 Hz（过采样，DRDY 由陀螺提供，加计多采不影响节拍）。
 * actual_bw_hz 回报实际 3dB 带宽，便于上位机核对。
 */
uint8_t DRV_BMI088_AccelOdrCode(DRV_IMU_Odr odr, uint16_t *actual_odr_hz);
uint8_t DRV_BMI088_AccelBwpCode(uint16_t odr_hz, uint16_t desired_bw_hz,
                                uint16_t *actual_bw_hz);
uint8_t DRV_BMI088_BuildAccConf(DRV_IMU_Odr odr, uint16_t desired_bw_hz,
                                uint16_t *actual_bw_hz);

/*
 * 陀螺 GYRO_BANDWIDTH(0x10)：ODR 与带宽是**绑定的**，一个码同时定两者，
 * 不能像加计那样分开选。bit7 在该寄存器上恒读 1，写回时一并置上。
 */
uint8_t DRV_BMI088_GyroBandwidthCode(DRV_IMU_Odr odr, uint16_t desired_bw_hz,
                                     uint16_t *actual_odr_hz,
                                     uint16_t *actual_bw_hz);

/* 原始计数 → 物理量。芯片自身轴向，不做机体变换。 */
void DRV_BMI088_ConvertRaw(DRV_IMU_AccelRange accel_range,
                           DRV_IMU_GyroRange gyro_range,
                           const DRV_IMU_RawData *raw,
                           DRV_IMU_ScaledData *scaled);

/*
 * 温度寄存器是 11 位有符号、0.125 °C/LSB、偏置 23 °C，而且**不是**普通的
 * MSB/LSB 拼接：低 3 位无效，要先右移。写错了温补会整体偏，但读数看起来"像样"，
 * 所以这里单独暴露出来给单测钉住。
 */
float DRV_BMI088_ConvertTemperature(uint8_t msb, uint8_t lsb);

#ifdef __cplusplus
}
#endif

#endif
