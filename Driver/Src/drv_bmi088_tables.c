/*
 * BMI088 换算表实现。纯函数，不含 HAL / RTOS / I/O。
 *
 * 数值来源：Bosch BMI088 datasheet（BST-BMI088-DS001）第 5 章寄存器表，
 * 并与 ArduPilot `AP_InertialSensor_BMI088.cpp` 的实测配置交叉核对过
 * （该实现用 ACC_CONF=0x9C 并注明 "OSR2 gives 234Hz LPF @ 1.6kHz ODR"，
 *  与下表 1600 Hz 行的 osr2=234 Hz 一致）。
 */

#include "drv_bmi088_tables.h"

#include <stddef.h>

/* ---------------------------------------------------------------- 加计量程 */

uint8_t DRV_BMI088_AccelRangeCode(DRV_IMU_AccelRange range)
{
    switch (range) {
    case DRV_IMU_ACCEL_RANGE_2G:  return DRV_BMI088_ACC_RANGE_3G;
    case DRV_IMU_ACCEL_RANGE_4G:  return DRV_BMI088_ACC_RANGE_6G;
    case DRV_IMU_ACCEL_RANGE_8G:  return DRV_BMI088_ACC_RANGE_12G;
    case DRV_IMU_ACCEL_RANGE_16G:
    default:                      return DRV_BMI088_ACC_RANGE_24G;
    }
}

float DRV_BMI088_AccelLsbPerG(DRV_IMU_AccelRange range)
{
    /* 满量程 g 值直接由寄存器码决定：±3 / ±6 / ±12 / ±24。16 位有符号 → 32768。 */
    switch (range) {
    case DRV_IMU_ACCEL_RANGE_2G:  return 32768.0f / 3.0f;
    case DRV_IMU_ACCEL_RANGE_4G:  return 32768.0f / 6.0f;
    case DRV_IMU_ACCEL_RANGE_8G:  return 32768.0f / 12.0f;
    case DRV_IMU_ACCEL_RANGE_16G:
    default:                      return 32768.0f / 24.0f;
    }
}

/* ---------------------------------------------------------------- 陀螺量程 */

uint8_t DRV_BMI088_GyroRangeCode(DRV_IMU_GyroRange range)
{
    switch (range) {
    case DRV_IMU_GYRO_RANGE_2000DPS: return DRV_BMI088_GYRO_RANGE_2000DPS;
    case DRV_IMU_GYRO_RANGE_1000DPS: return DRV_BMI088_GYRO_RANGE_1000DPS;
    case DRV_IMU_GYRO_RANGE_500DPS:  return DRV_BMI088_GYRO_RANGE_500DPS;
    case DRV_IMU_GYRO_RANGE_250DPS:  return DRV_BMI088_GYRO_RANGE_250DPS;
    case DRV_IMU_GYRO_RANGE_125DPS:
    case DRV_IMU_GYRO_RANGE_62D5DPS:
    case DRV_IMU_GYRO_RANGE_31D25DPS:
    case DRV_IMU_GYRO_RANGE_15D625DPS:
    default:
        /* 比 ±125 dps 更细的档 BMI088 没有，夹到最细档而不是按错误刻度换算。 */
        return DRV_BMI088_GYRO_RANGE_125DPS;
    }
}

float DRV_BMI088_GyroLsbPerDps(DRV_IMU_GyroRange range)
{
    switch (DRV_BMI088_GyroRangeCode(range)) {
    case DRV_BMI088_GYRO_RANGE_1000DPS: return 32768.0f / 1000.0f;
    case DRV_BMI088_GYRO_RANGE_500DPS:  return 32768.0f / 500.0f;
    case DRV_BMI088_GYRO_RANGE_250DPS:  return 32768.0f / 250.0f;
    case DRV_BMI088_GYRO_RANGE_125DPS:  return 32768.0f / 125.0f;
    case DRV_BMI088_GYRO_RANGE_2000DPS:
    default:                            return 32768.0f / 2000.0f;
    }
}

/* ------------------------------------------------------------ 加计 ODR/带宽 */

typedef struct {
    DRV_IMU_Odr odr;
    uint16_t    hz;
    uint8_t     code;
} BMI088_AccelOdrEntry;

static const BMI088_AccelOdrEntry accel_odr_table[] = {
    { DRV_IMU_ODR_12D5HZ,   12U, 0x05U },
    { DRV_IMU_ODR_25HZ,     25U, 0x06U },
    { DRV_IMU_ODR_50HZ,     50U, 0x07U },
    { DRV_IMU_ODR_100HZ,   100U, 0x08U },
    { DRV_IMU_ODR_200HZ,   200U, 0x09U },
    { DRV_IMU_ODR_500HZ,   400U, 0x0AU },  /* 没有 500 Hz 档，落到 400 Hz */
    { DRV_IMU_ODR_1KHZ,   1600U, 0x0CU },  /* 没有 1 kHz 档，过采到 1600 Hz */
    { DRV_IMU_ODR_2KHZ,   1600U, 0x0CU },
    { DRV_IMU_ODR_4KHZ,   1600U, 0x0CU },
    { DRV_IMU_ODR_8KHZ,   1600U, 0x0CU },
    { DRV_IMU_ODR_16KHZ,  1600U, 0x0CU },
    { DRV_IMU_ODR_32KHZ,  1600U, 0x0CU },
};

#define ACCEL_ODR_TABLE_COUNT \
    (sizeof(accel_odr_table) / sizeof(accel_odr_table[0]))

uint8_t DRV_BMI088_AccelOdrCode(DRV_IMU_Odr odr, uint16_t *actual_odr_hz)
{
    /* 默认落到 800 Hz：请求落在表外时给一个安全的中间档，而不是最高档。 */
    uint8_t code = 0x0BU;
    uint16_t hz = 800U;
    uint32_t i;

    for (i = 0U; i < ACCEL_ODR_TABLE_COUNT; i++) {
        if (accel_odr_table[i].odr == odr) {
            code = accel_odr_table[i].code;
            hz = accel_odr_table[i].hz;
            break;
        }
    }

    if (actual_odr_hz != NULL) { *actual_odr_hz = hz; }
    return code;
}

typedef struct {
    uint16_t odr_hz;
    uint16_t bw_normal_hz;
    uint16_t bw_osr2_hz;
    uint16_t bw_osr4_hz;
} BMI088_AccelBwEntry;

/* datasheet 表 "acc_bwp vs. 3dB cut-off frequency"，只列本工程会用到的高 ODR 行。 */
static const BMI088_AccelBwEntry accel_bw_table[] = {
    {  400U, 145U,  80U,  40U },
    {  800U, 230U, 140U,  80U },
    { 1600U, 684U, 234U, 145U },
};

#define ACCEL_BW_TABLE_COUNT \
    (sizeof(accel_bw_table) / sizeof(accel_bw_table[0]))

uint8_t DRV_BMI088_AccelBwpCode(uint16_t odr_hz, uint16_t desired_bw_hz,
                                uint16_t *actual_bw_hz)
{
    const BMI088_AccelBwEntry *row = &accel_bw_table[ACCEL_BW_TABLE_COUNT - 1U];
    uint8_t code;
    uint16_t bw;
    uint32_t i;

    for (i = 0U; i < ACCEL_BW_TABLE_COUNT; i++) {
        if (accel_bw_table[i].odr_hz == odr_hz) {
            row = &accel_bw_table[i];
            break;
        }
    }

    /*
     * 取"不超过请求带宽的最高档"。宁可滤得更狠也不要放进更多带外能量——
     * 已经混叠进来的转子谐波，后面任何软件滤波都分不出来了。
     */
    if (desired_bw_hz >= row->bw_normal_hz) {
        code = DRV_BMI088_ACC_BWP_NORMAL;
        bw = row->bw_normal_hz;
    } else if (desired_bw_hz >= row->bw_osr2_hz) {
        code = DRV_BMI088_ACC_BWP_OSR2;
        bw = row->bw_osr2_hz;
    } else {
        code = DRV_BMI088_ACC_BWP_OSR4;
        bw = row->bw_osr4_hz;
    }

    if (actual_bw_hz != NULL) { *actual_bw_hz = bw; }
    return code;
}

uint8_t DRV_BMI088_BuildAccConf(DRV_IMU_Odr odr, uint16_t desired_bw_hz,
                                uint16_t *actual_bw_hz)
{
    uint16_t odr_hz = 0U;
    uint8_t odr_code = DRV_BMI088_AccelOdrCode(odr, &odr_hz);
    uint8_t bwp_code = DRV_BMI088_AccelBwpCode(odr_hz, desired_bw_hz, actual_bw_hz);

    return (uint8_t)((uint8_t)(bwp_code << 4U) | (odr_code & 0x0FU));
}

/* ------------------------------------------------------------ 陀螺 ODR/带宽 */

typedef struct {
    uint16_t odr_hz;
    uint16_t bw_hz;
    uint8_t  code;
} BMI088_GyroBwEntry;

/*
 * GYRO_BANDWIDTH(0x10) 一个码同时定 ODR 和带宽，两者不能分开选。
 * 表按 datasheet 顺序，同一 ODR 的多行按带宽从高到低排，方便"取不超过请求的最高档"。
 */
static const BMI088_GyroBwEntry gyro_bw_table[] = {
    { 2000U, 532U, 0x00U },
    { 2000U, 230U, 0x01U },
    { 1000U, 116U, 0x02U },
    {  400U,  47U, 0x03U },
    {  200U,  64U, 0x06U },
    {  200U,  23U, 0x04U },
    {  100U,  32U, 0x07U },
    {  100U,  12U, 0x05U },
};

#define GYRO_BW_TABLE_COUNT \
    (sizeof(gyro_bw_table) / sizeof(gyro_bw_table[0]))

static uint16_t gyro_target_odr_hz(DRV_IMU_Odr odr)
{
    switch (odr) {
    case DRV_IMU_ODR_100HZ:  return 100U;
    case DRV_IMU_ODR_200HZ:  return 200U;
    case DRV_IMU_ODR_500HZ:  return 400U;   /* 没有 500 Hz 档 */
    case DRV_IMU_ODR_1KHZ:   return 1000U;
    case DRV_IMU_ODR_2KHZ:
    case DRV_IMU_ODR_4KHZ:
    case DRV_IMU_ODR_8KHZ:
    case DRV_IMU_ODR_16KHZ:
    case DRV_IMU_ODR_32KHZ:  return 2000U;  /* 陀螺最高 2 kHz */
    default:                 return 1000U;
    }
}

uint8_t DRV_BMI088_GyroBandwidthCode(DRV_IMU_Odr odr, uint16_t desired_bw_hz,
                                     uint16_t *actual_odr_hz,
                                     uint16_t *actual_bw_hz)
{
    uint16_t target_odr = gyro_target_odr_hz(odr);
    const BMI088_GyroBwEntry *chosen = NULL;
    uint32_t i;

    /* 先锁 ODR（节拍不能让步），再在同 ODR 里挑不超过请求的最高带宽。 */
    for (i = 0U; i < GYRO_BW_TABLE_COUNT; i++) {
        if (gyro_bw_table[i].odr_hz != target_odr) { continue; }
        if (chosen == NULL) { chosen = &gyro_bw_table[i]; }
        if (gyro_bw_table[i].bw_hz <= desired_bw_hz) {
            chosen = &gyro_bw_table[i];
            break;
        }
    }

    if (chosen == NULL) { chosen = &gyro_bw_table[2]; }  /* 1000 Hz / 116 Hz */

    if (actual_odr_hz != NULL) { *actual_odr_hz = chosen->odr_hz; }
    if (actual_bw_hz != NULL)  { *actual_bw_hz = chosen->bw_hz; }
    return chosen->code;
}

/* -------------------------------------------------------------------- 换算 */

void DRV_BMI088_ConvertRaw(DRV_IMU_AccelRange accel_range,
                           DRV_IMU_GyroRange gyro_range,
                           const DRV_IMU_RawData *raw,
                           DRV_IMU_ScaledData *scaled)
{
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;

    if ((raw == NULL) || (scaled == NULL)) { return; }

    accel_lsb_per_g = DRV_BMI088_AccelLsbPerG(accel_range);
    gyro_lsb_per_dps = DRV_BMI088_GyroLsbPerDps(gyro_range);

    scaled->accel_x_g = (float)raw->accel_x / accel_lsb_per_g;
    scaled->accel_y_g = (float)raw->accel_y / accel_lsb_per_g;
    scaled->accel_z_g = (float)raw->accel_z / accel_lsb_per_g;

    scaled->gyro_x_dps = (float)raw->gyro_x / gyro_lsb_per_dps;
    scaled->gyro_y_dps = (float)raw->gyro_y / gyro_lsb_per_dps;
    scaled->gyro_z_dps = (float)raw->gyro_z / gyro_lsb_per_dps;

    /*
     * raw->temperature 存的是驱动已经折过符号的 11 位原始计数（不是 °C），
     * 换算与 DRV_BMI088_ConvertTemperature 的后半段一致：0.125 °C/LSB + 23 °C。
     */
    scaled->temperature_c = ((float)raw->temperature * 0.125f) + 23.0f;
}

float DRV_BMI088_ConvertTemperature(uint8_t msb, uint8_t lsb)
{
    /*
     * TEMP_MSB(0x22) 是高 8 位，TEMP_LSB(0x23) 的高 3 位是低 3 位，合起来 11 位有符号。
     * 先拼成无符号 11 位，再按 2 的补码折回负数，最后 0.125 °C/LSB + 23 °C 偏置。
     */
    int32_t value = ((int32_t)msb * 8) + ((int32_t)(lsb >> 5) & 0x07);

    if (value > 1023) { value -= 2048; }

    return ((float)value * 0.125f) + 23.0f;
}
