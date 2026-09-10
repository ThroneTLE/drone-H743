/*
 * BMI270 换算表实现。纯函数，不含 HAL / RTOS / I/O。
 *
 * 数值来源：Bosch BMI270 datasheet 寄存器表，带宽比例的两个锚点用
 * ArduPilot `AP_InertialSensor_BMI270.cpp` 的实测注释校准过：
 *   加计 osr4 @ 1600 Hz ODR = 188 Hz  →  188/1600 = 0.1175
 *   陀螺 normal @ 3200 Hz ODR = 751 Hz →  751/3200 = 0.2347
 * 每提高一档过采样，截止频率减半，据此得到下面的比例表。
 */

#include "drv_bmi270_tables.h"

#include <stddef.h>

/* ---------------------------------------------------------------- 量程 */

uint8_t DRV_BMI270_AccelRangeCode(DRV_IMU_AccelRange range)
{
    /* 通用枚举是 16G=0 递减到 2G=3，BMI270 是 2g=0 递增到 16g=3，顺序相反。 */
    switch (range) {
    case DRV_IMU_ACCEL_RANGE_2G:  return DRV_BMI270_ACC_RANGE_2G;
    case DRV_IMU_ACCEL_RANGE_4G:  return DRV_BMI270_ACC_RANGE_4G;
    case DRV_IMU_ACCEL_RANGE_8G:  return DRV_BMI270_ACC_RANGE_8G;
    case DRV_IMU_ACCEL_RANGE_16G:
    default:                      return DRV_BMI270_ACC_RANGE_16G;
    }
}

float DRV_BMI270_AccelLsbPerG(DRV_IMU_AccelRange range)
{
    switch (range) {
    case DRV_IMU_ACCEL_RANGE_2G:  return 16384.0f;
    case DRV_IMU_ACCEL_RANGE_4G:  return 8192.0f;
    case DRV_IMU_ACCEL_RANGE_8G:  return 4096.0f;
    case DRV_IMU_ACCEL_RANGE_16G:
    default:                      return 2048.0f;
    }
}

uint8_t DRV_BMI270_GyroRangeCode(DRV_IMU_GyroRange range)
{
    switch (range) {
    case DRV_IMU_GYRO_RANGE_2000DPS: return DRV_BMI270_GYR_RANGE_2000DPS;
    case DRV_IMU_GYRO_RANGE_1000DPS: return DRV_BMI270_GYR_RANGE_1000DPS;
    case DRV_IMU_GYRO_RANGE_500DPS:  return DRV_BMI270_GYR_RANGE_500DPS;
    case DRV_IMU_GYRO_RANGE_250DPS:  return DRV_BMI270_GYR_RANGE_250DPS;
    case DRV_IMU_GYRO_RANGE_125DPS:
    case DRV_IMU_GYRO_RANGE_62D5DPS:
    case DRV_IMU_GYRO_RANGE_31D25DPS:
    case DRV_IMU_GYRO_RANGE_15D625DPS:
    default:
        /* BMI270 最细到 ±125 dps，更细的档一律夹住而不是按错误刻度换算。 */
        return DRV_BMI270_GYR_RANGE_125DPS;
    }
}

float DRV_BMI270_GyroLsbPerDps(DRV_IMU_GyroRange range)
{
    switch (DRV_BMI270_GyroRangeCode(range)) {
    case DRV_BMI270_GYR_RANGE_1000DPS: return 32768.0f / 1000.0f;
    case DRV_BMI270_GYR_RANGE_500DPS:  return 32768.0f / 500.0f;
    case DRV_BMI270_GYR_RANGE_250DPS:  return 32768.0f / 250.0f;
    case DRV_BMI270_GYR_RANGE_125DPS:  return 32768.0f / 125.0f;
    case DRV_BMI270_GYR_RANGE_2000DPS:
    default:                           return 32768.0f / 2000.0f;
    }
}

/* ------------------------------------------------------------------ ODR */

typedef struct {
    DRV_IMU_Odr odr;
    uint16_t    hz;
    uint8_t     code;
} BMI270_OdrEntry;

static const BMI270_OdrEntry odr_table[] = {
    { DRV_IMU_ODR_100HZ,   100U, 0x08U },
    { DRV_IMU_ODR_200HZ,   200U, 0x09U },
    { DRV_IMU_ODR_500HZ,   400U, 0x0AU },  /* 没有 500 Hz 档 */
    { DRV_IMU_ODR_1KHZ,    800U, 0x0BU },  /* 没有 1 kHz 档，取相邻的 800 Hz */
    { DRV_IMU_ODR_2KHZ,   1600U, 0x0CU },
    { DRV_IMU_ODR_4KHZ,   1600U, 0x0CU },
    { DRV_IMU_ODR_8KHZ,   1600U, 0x0CU },
    { DRV_IMU_ODR_16KHZ,  1600U, 0x0CU },
    { DRV_IMU_ODR_32KHZ,  1600U, 0x0CU },
};

#define ODR_TABLE_COUNT (sizeof(odr_table) / sizeof(odr_table[0]))

uint8_t DRV_BMI270_OdrCode(DRV_IMU_Odr odr, uint8_t is_gyro, uint16_t *actual_odr_hz)
{
    uint8_t code = 0x0BU;   /* 表外一律落到 800 Hz，不去赌最高档 */
    uint16_t hz = 800U;
    uint32_t i;

    for (i = 0U; i < ODR_TABLE_COUNT; i++) {
        if (odr_table[i].odr == odr) {
            code = odr_table[i].code;
            hz = odr_table[i].hz;
            break;
        }
    }

    /* 陀螺比加计多一档 3200 Hz，但本工程用不到，留着只是说明差异存在。 */
    (void)is_gyro;

    if (actual_odr_hz != NULL) { *actual_odr_hz = hz; }
    return code;
}

/* ----------------------------------------------------------------- 带宽 */

uint8_t DRV_BMI270_BwpCode(uint16_t odr_hz, uint16_t desired_bw_hz,
                           uint8_t is_gyro, uint16_t *actual_bw_hz)
{
    /*
     * 比例按每档过采样减半：
     *   加计 normal 0.469 / osr2 0.234 / osr4 0.117
     *   陀螺 normal 0.235 / osr2 0.117 / osr4 0.059
     * 用整数千分比运算，避免在换算表里引入浮点。
     */
    const uint16_t normal_permille = is_gyro ? 235U : 469U;
    const uint16_t osr2_permille   = is_gyro ? 117U : 234U;
    const uint16_t osr4_permille   = is_gyro ?  59U : 117U;

    uint16_t bw_normal = (uint16_t)(((uint32_t)odr_hz * normal_permille) / 1000U);
    uint16_t bw_osr2   = (uint16_t)(((uint32_t)odr_hz * osr2_permille) / 1000U);
    uint16_t bw_osr4   = (uint16_t)(((uint32_t)odr_hz * osr4_permille) / 1000U);

    uint8_t code;
    uint16_t bw;

    /* 取不超过请求的最高档；已经混叠进来的转子谐波后面滤不掉，宁可滤狠一点。 */
    if (desired_bw_hz >= bw_normal) {
        code = DRV_BMI270_BWP_NORMAL;
        bw = bw_normal;
    } else if (desired_bw_hz >= bw_osr2) {
        code = DRV_BMI270_BWP_OSR2;
        bw = bw_osr2;
    } else {
        code = DRV_BMI270_BWP_OSR4;
        bw = bw_osr4;
    }

    if (actual_bw_hz != NULL) { *actual_bw_hz = bw; }
    return code;
}

uint8_t DRV_BMI270_BuildAccConf(DRV_IMU_Odr odr, uint16_t desired_bw_hz,
                                uint16_t *actual_bw_hz)
{
    uint16_t odr_hz = 0U;
    uint8_t odr_code = DRV_BMI270_OdrCode(odr, 0U, &odr_hz);
    uint8_t bwp_code = DRV_BMI270_BwpCode(odr_hz, desired_bw_hz, 0U, actual_bw_hz);

    /* bit7 = acc_filter_perf，置 1 走高性能（低噪声）滤波路径。 */
    return (uint8_t)(0x80U | (uint8_t)(bwp_code << 4U) | (odr_code & 0x0FU));
}

uint8_t DRV_BMI270_BuildGyrConf(DRV_IMU_Odr odr, uint16_t desired_bw_hz,
                                uint16_t *actual_bw_hz)
{
    uint16_t odr_hz = 0U;
    uint8_t odr_code = DRV_BMI270_OdrCode(odr, 1U, &odr_hz);
    uint8_t bwp_code = DRV_BMI270_BwpCode(odr_hz, desired_bw_hz, 1U, actual_bw_hz);

    /* bit7 = gyr_filter_perf，bit6 = gyr_noise_perf，两者都要高性能。 */
    return (uint8_t)(0xC0U | (uint8_t)((bwp_code & 0x03U) << 4U) |
                     (odr_code & 0x0FU));
}

/* ----------------------------------------------------------------- 换算 */

void DRV_BMI270_ConvertRaw(DRV_IMU_AccelRange accel_range,
                           DRV_IMU_GyroRange gyro_range,
                           const DRV_IMU_RawData *raw,
                           DRV_IMU_ScaledData *scaled)
{
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;

    if ((raw == NULL) || (scaled == NULL)) { return; }

    accel_lsb_per_g = DRV_BMI270_AccelLsbPerG(accel_range);
    gyro_lsb_per_dps = DRV_BMI270_GyroLsbPerDps(gyro_range);

    scaled->accel_x_g = (float)raw->accel_x / accel_lsb_per_g;
    scaled->accel_y_g = (float)raw->accel_y / accel_lsb_per_g;
    scaled->accel_z_g = (float)raw->accel_z / accel_lsb_per_g;

    scaled->gyro_x_dps = (float)raw->gyro_x / gyro_lsb_per_dps;
    scaled->gyro_y_dps = (float)raw->gyro_y / gyro_lsb_per_dps;
    scaled->gyro_z_dps = (float)raw->gyro_z / gyro_lsb_per_dps;

    /* raw->temperature 是芯片原始 16 位计数：1/512 K per LSB，偏置 23 °C。 */
    scaled->temperature_c = ((float)raw->temperature / 512.0f) + 23.0f;
}

float DRV_BMI270_ConvertTemperature(uint8_t lsb, uint8_t msb, uint8_t *valid)
{
    uint16_t word = (uint16_t)(((uint16_t)msb << 8U) | (uint16_t)lsb);

    /* 0x8000 是"温度无效"的哨兵值，不是 -64 °C。按数值算会得到一个像样的假读数。 */
    if (word == 0x8000U) {
        if (valid != NULL) { *valid = 0U; }
        return 23.0f;
    }

    if (valid != NULL) { *valid = 1U; }
    return ((float)(int16_t)word / 512.0f) + 23.0f;
}
