#include "app_sensor.h"

#include "app_messages.h"
#include "app_tasks.h"
#include "bsp_baro.h"
#include "bsp_imu.h"
#include "cmsis_os2.h"

#include <math.h>
#include <string.h>

/*
 * APP_IMU 模块
 *
 * 传感器度量转换 API（由 freertos.c 中的 Sensor_Task 调用）：
 *   APP_IMU_RawToScaled      — ICM-42688 原始读数 → 物理单位
 *   DRV_AttitudeFusion_Update — x-io Fusion AHRS（由 StabilizerTask 调用）
 *   APP_IMU_ConvertBaro      — SPL06-007 气压计 (TODO)
 *
 * 中断和诊断：
 *   HAL_GPIO_EXTI_Callback  — PC0 数据就绪 → 线程标志唤醒 Sensor_Task
 *   APP_IMU_GetStatus       — 寄存器级诊断快照
 *   APP_IMU_GetLastSample   — 向后兼容的样本读取接口
 */

#define APP_IMU_DATA_READY_FLAG 0x0001U

typedef struct {
    volatile uint32_t sequence;
    volatile uint32_t timestamp_low;
    volatile uint32_t timestamp_high;
} APP_IMU_DataReadyTimestampLatch;

static APP_IMU_DataReadyTimestampLatch app_imu_drdy_timestamp;

/* ════════════════════════════════════════════════════════════════════════ */
/*  IMU 原始读数 → 物理单位转换                                             */
/*                                                                        */
/*  ICM-42688 16 位有符号数 (±32768)，量程在 BSP 层配置。                   */
/*  刻度必须由实际配置的量程推导，不能写死：曾经这里硬编码 8192 LSB/g       */
/*  (±4G)，一旦 BSP 改量程，姿态就会整体错一个倍数且不报错。                */
/*  当前 BSP 配置为 ±16G / ±1000dps —— 见 BSP_IMU_Init() 中关于振动削顶     */
/*  的说明。                                                               */
/*    温度: 128 LSB/°C，偏移 +25°C                                         */
/* ════════════════════════════════════════════════════════════════════════ */

#define APP_IMU_TEMP_LSB_PER_C    128.0f
#define APP_IMU_TEMP_OFFSET_C     25.0f

void APP_IMU_RawToScaled(const DRV_IMU_RawData *raw,
                         DRV_IMU_ScaledData *scaled)
{
    const DRV_IMU_Device *dev;
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;

    if ((raw == NULL) || (scaled == NULL)) return;

    /* 从设备实际配置推导刻度，回退值与 BSP_IMU_Init() 的配置保持一致。 */
    dev = BSP_IMU_GetDevice();
    if (dev != NULL) {
        accel_lsb_per_g  = DRV_IMU_AccelLsbPerG(dev->config.accel_range);
        gyro_lsb_per_dps = DRV_IMU_GyroLsbPerDps(dev->config.gyro_range);
    } else {
        accel_lsb_per_g  = DRV_IMU_AccelLsbPerG(DRV_IMU_ACCEL_RANGE_16G);
        gyro_lsb_per_dps = DRV_IMU_GyroLsbPerDps(DRV_IMU_GYRO_RANGE_1000DPS);
    }

    scaled->temperature_c = (float)raw->temperature / APP_IMU_TEMP_LSB_PER_C
                          + APP_IMU_TEMP_OFFSET_C;
    scaled->accel_x_g    = (float)raw->accel_x / accel_lsb_per_g;
    scaled->accel_y_g    = (float)raw->accel_y / accel_lsb_per_g;
    scaled->accel_z_g    = (float)raw->accel_z / accel_lsb_per_g;
    scaled->gyro_x_dps   = (float)raw->gyro_x / gyro_lsb_per_dps;
    scaled->gyro_y_dps   = (float)raw->gyro_y / gyro_lsb_per_dps;
    scaled->gyro_z_dps   = (float)raw->gyro_z / gyro_lsb_per_dps;
}

/* ════════════════════════════════════════════════════════════════════════ */
/*  气压计转换                                                            */
/*                                                                        */
/*  SPL06-007 使用校准系数补偿公式算出真正的 Pa 和 °C:                      */
/*                                                                        */
/*  比例系数（由 PRS_CFG/TMP_CFG 决定）：                                   */
/*    PRS_CFG = 0x53 → PM_PRC = 8x → kP = 7864320                        */
/*    TMP_CFG = 0xB0 → TMP_PRC = 1x → kT = 524288                        */
/*                                                                        */
/*  公式：                                                                 */
/*    Praw_sc = pressure_raw / kP   （中间量）                              */
/*    Traw_sc = temperature_raw / kT                                       */
/*    Tcomp   = c0*0.5 + c1*Traw_sc                                       */
/*    Pcomp   = c00 + Praw_sc*(c10 + Praw_sc*(c20 + Praw_sc*c30))         */
/*             + Traw_sc*c01 + Traw_sc*Praw_sc*(c11 + Praw_sc*c21)        */
/*                                                                        */
/*  校准系数从芯片寄存器 0x10-0x21（18 字节）读取，首次调用时缓存。          */
/* ════════════════════════════════════════════════════════════════════════ */

#define SPL06_KP_LSCALE 7864320.0f   /* PRS_CFG = 0x53, 8x oversampling */
#define SPL06_KT_LSCALE 524288.0f    /* TMP_CFG = 0xB0, 1x oversampling */

/* 校准系数缓存 */
typedef struct {
    int16_t c0, c1;
    int32_t c00, c10;
    int16_t c01, c11, c20, c21, c30;
    uint8_t loaded;
} SPL06_Calib;

static SPL06_Calib spl06_calib;

static int32_t spl06_sign_extend(uint32_t value, uint8_t bits)
{
    uint32_t sign_bit = 1UL << (bits - 1U);
    uint32_t mask     = (1UL << bits) - 1UL;
    value &= mask;
    if ((value & sign_bit) != 0UL) {
        value |= ~mask;
    }
    return (int32_t)value;
}

static void spl06_load_calib(void)
{
    uint8_t c[18];

    if (BSP_BARO_ReadRawRegisters(0x10U, c, 18U) != DRV_BARO_OK) {
        return;
    }

    spl06_calib.c0  = (int16_t)spl06_sign_extend(((uint32_t)c[0] << 4) | ((uint32_t)c[1] >> 4), 12U);
    spl06_calib.c1  = (int16_t)spl06_sign_extend((((uint32_t)c[1] & 0x0FU) << 8) | (uint32_t)c[2], 12U);
    spl06_calib.c00 = spl06_sign_extend(((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4) | ((uint32_t)c[5] >> 4), 20U);
    spl06_calib.c10 = spl06_sign_extend((((uint32_t)c[5] & 0x0FU) << 16) | ((uint32_t)c[6] << 8) | (uint32_t)c[7], 20U);
    spl06_calib.c01 = (int16_t)(((uint16_t)c[8]  << 8) | (uint16_t)c[9]);
    spl06_calib.c11 = (int16_t)(((uint16_t)c[10] << 8) | (uint16_t)c[11]);
    spl06_calib.c20 = (int16_t)(((uint16_t)c[12] << 8) | (uint16_t)c[13]);
    spl06_calib.c21 = (int16_t)(((uint16_t)c[14] << 8) | (uint16_t)c[15]);
    spl06_calib.c30 = (int16_t)(((uint16_t)c[16] << 8) | (uint16_t)c[17]);
    spl06_calib.loaded = 1U;
}

void APP_IMU_ConvertBaro(const int32_t pressure_raw,
                         const int32_t temperature_raw,
                         float *pressure_pa,
                         float *temperature_c)
{
    if (!spl06_calib.loaded) {
        spl06_load_calib();
    }
    if (!spl06_calib.loaded) {
        if (pressure_pa   != NULL) { *pressure_pa   = 0.0f; }
        if (temperature_c != NULL) { *temperature_c = 0.0f; }
        return;
    }

    /* 中间量 */
    double p_raw_sc = (double)pressure_raw    / SPL06_KP_LSCALE;
    double t_raw_sc = (double)temperature_raw / SPL06_KT_LSCALE;

    /* 补偿公式（双精度保证精度） */
    double temp = (double)spl06_calib.c0 * 0.5 + (double)spl06_calib.c1 * t_raw_sc;
    double pres = (double)spl06_calib.c00
                + p_raw_sc * ((double)spl06_calib.c10
                             + p_raw_sc * ((double)spl06_calib.c20
                                          + p_raw_sc * (double)spl06_calib.c30))
                + t_raw_sc * (double)spl06_calib.c01
                + t_raw_sc * p_raw_sc * ((double)spl06_calib.c11
                                        + p_raw_sc * (double)spl06_calib.c21);

    if (temperature_c != NULL) {
        *temperature_c = (float)temp;
    }
    if (pressure_pa != NULL) {
        *pressure_pa = (float)pres;
    }
}

/* ════════════════════════════════════════════════════════════════════════ */
/*  低通滤波器 (二阶 Butterworth biquad)                                   */
/*                                                                        */
/*  设计依据见 app_sensor.h。RBJ cookbook 低通系数：                        */
/*    w0 = 2π*fc*dt,  alpha = sin(w0) / (2Q),  Q = 1/sqrt(2)              */
/*    b = [(1-cos)/2, 1-cos, (1-cos)/2],  a = [1+alpha, -2cos, 1-alpha]   */
/*  全部系数预先除以 a0，运行时只需 5 乘 4 加。                             */
/* ════════════════════════════════════════════════════════════════════════ */

void APP_Sensor_LpfInit(APP_Sensor_Lpf *lpf, float cutoff_hz, float dt_sec)
{
    float w0, cos_w0, sin_w0, alpha, a0_inv;

    if (lpf == NULL) return;

    memset(lpf, 0, sizeof(*lpf));

    /*
     * 截止频率必须低于 Nyquist，否则 cookbook 公式退化。夹在 0.45/dt 以内，
     * 留出余量；非法输入退化为直通，避免产生 NaN 污染姿态。
     */
    if ((dt_sec <= 0.0f) || (cutoff_hz <= 0.0f)) {
        lpf->b0 = 1.0f;
        return;
    }
    if (cutoff_hz > (0.45f / dt_sec)) {
        cutoff_hz = 0.45f / dt_sec;
    }

    w0 = 6.2831853f * cutoff_hz * dt_sec;
    cos_w0 = cosf(w0);
    sin_w0 = sinf(w0);
    alpha = sin_w0 * 0.70710678f;   /* sin(w0) / (2 * 1/sqrt(2)) */

    a0_inv = 1.0f / (1.0f + alpha);
    lpf->b0 = ((1.0f - cos_w0) * 0.5f) * a0_inv;
    lpf->b1 = (1.0f - cos_w0) * a0_inv;
    lpf->b2 = lpf->b0;
    lpf->a1 = (-2.0f * cos_w0) * a0_inv;
    lpf->a2 = (1.0f - alpha) * a0_inv;
}

float APP_Sensor_LpfApply(APP_Sensor_Lpf *lpf, float input)
{
    float output;

    if (lpf == NULL) return input;

    if (lpf->initialized == 0U) {
        /*
         * 用首个样本填充历史，使滤波器从稳态启动。否则零初始化会让输出从 0
         * 爬升到重力量级，姿态解算在启动瞬间会看到一个假的大倾角。
         */
        lpf->x1 = input;
        lpf->x2 = input;
        lpf->y1 = input;
        lpf->y2 = input;
        lpf->initialized = 1U;
        return input;
    }

    output = (lpf->b0 * input) + (lpf->b1 * lpf->x1) + (lpf->b2 * lpf->x2)
           - (lpf->a1 * lpf->y1) - (lpf->a2 * lpf->y2);

    lpf->x2 = lpf->x1;
    lpf->x1 = input;
    lpf->y2 = lpf->y1;
    lpf->y1 = output;

    return output;
}

void APP_Sensor_LpfApply3f(APP_Sensor_Lpf lpf[3],
                           const float in[3], float out[3])
{
    if ((lpf == NULL) || (in == NULL) || (out == NULL)) return;
    for (uint32_t i = 0U; i < 3U; i++) {
        out[i] = APP_Sensor_LpfApply(&lpf[i], in[i]);
    }
}

/* ════════════════════════════════════════════════════════════════════════ */
/*  陀螺仪零偏校准                                                        */
/*                                                                        */
/*  静止采集 APP_SENSOR_GYRO_BIAS_SAMPLES(1000) 个样本做均值 = 零偏        */
/* ════════════════════════════════════════════════════════════════════════ */

uint8_t APP_Sensor_CalibrateGyroBias(float gx, float gy, float gz,
                                     APP_Sensor_GyroBias *cal)
{
    if (cal == NULL) return 0U;

    if (cal->ready) return 0U;

    if ((fabsf(gx) > APP_SENSOR_GYRO_BIAS_MAX_STATIC_DPS) ||
        (fabsf(gy) > APP_SENSOR_GYRO_BIAS_MAX_STATIC_DPS) ||
        (fabsf(gz) > APP_SENSOR_GYRO_BIAS_MAX_STATIC_DPS)) {
        cal->sum[0] = 0.0f;
        cal->sum[1] = 0.0f;
        cal->sum[2] = 0.0f;
        cal->count = 0U;
        return 0U;
    }

    cal->sum[0] += gx;
    cal->sum[1] += gy;
    cal->sum[2] += gz;
    cal->count++;

    if (cal->count >= APP_SENSOR_GYRO_BIAS_SAMPLES) {
        float inv = 1.0f / (float)APP_SENSOR_GYRO_BIAS_SAMPLES;
        cal->bias[0] = cal->sum[0] * inv;
        cal->bias[1] = cal->sum[1] * inv;
        cal->bias[2] = cal->sum[2] * inv;
        cal->ready = 1U;
        return 1U;  /* 刚完成校准 */
    }
    return 0U;
}

void APP_SensorRateMeter_Reset(APP_Sensor_RateMeter *meter)
{
    if (meter == NULL) return;

    meter->window_start_us = 0ULL;
    meter->window_start_count = 0U;
    meter->hz = 0.0f;
}

float APP_SensorRateMeter_Update(APP_Sensor_RateMeter *meter,
                                  uint64_t timestamp_us,
                                  uint32_t sample_count)
{
    uint64_t elapsed_us;
    uint32_t elapsed_samples;

    if (meter == NULL) return 0.0f;

    if (meter->window_start_us == 0ULL) {
        meter->window_start_us = timestamp_us;
        meter->window_start_count = sample_count;
        return meter->hz;
    }

    elapsed_us = timestamp_us - meter->window_start_us;
    if (elapsed_us < 250000ULL) {
        return meter->hz;
    }

    elapsed_samples = sample_count - meter->window_start_count;
    meter->hz = ((float)elapsed_samples * 1000000.0f) / (float)elapsed_us;
    meter->window_start_us = timestamp_us;
    meter->window_start_count = sample_count;

    return meter->hz;
}

/* ════════════════════════════════════════════════════════════════════════ */
/*  采集轴对齐（IMU 芯片坐标系 → 本机标定中间轴）                           */
/*                                                                        */
/*  当前飞控板安装方向：                                                    */
/*    IMU +Y 朝飞机下方，IMU +Z 朝飞机后方，IMU +X 朝飞机左方。              */
/*                                                                        */
/*  下列映射仅定义采集后的中间轴。实机确认的最终姿态符号补偿为：               */
/*    roll rate = -gyro X, pitch rate = +gyro Y, yaw rate = +gyro Z           */
/*    specific force = [-accel X, +accel Y, -accel Z]                         */
/*  该补偿在 StabilizerTask 的 Fusion AHRS 输入边界执行，使动态角速度与       */
/*  静态重力得到的 roll/pitch 方向一致。                                      */
/*                                                                        */
/*  因此轴映射为：                                                           */
/*    body X = -imu Z                                                       */
/*    body Y = -imu X                                                       */
/*    body Z =  imu Y                                                       */
/*                                                                        */
/*  这里的采集轴变换必须同时用于加速度和陀螺仪；Fusion 边界再转换到上述       */
/*  已由实机确认的姿态/比力契约。                                             */
/* ════════════════════════════════════════════════════════════════════════ */

void APP_Sensor_AlignToAirframe(const float in[3], float out[3])
{
    if ((in == NULL) || (out == NULL)) return;
    out[0] = -in[2];
    out[1] = -in[0];
    out[2] =  in[1];
}

uint8_t APP_IMU_ReadDataReadyTimestamp(uint64_t *timestamp_us)
{
    uint32_t sequence_before;
    uint32_t sequence_after;
    uint32_t timestamp_low;
    uint32_t timestamp_high;

    if (timestamp_us == NULL) {
        return 0U;
    }
    for (;;) {
        sequence_before = app_imu_drdy_timestamp.sequence;
        if ((sequence_before & 1U) != 0U) {
            continue;
        }
        __DMB();
        timestamp_low = app_imu_drdy_timestamp.timestamp_low;
        timestamp_high = app_imu_drdy_timestamp.timestamp_high;
        __DMB();
        sequence_after = app_imu_drdy_timestamp.sequence;
        if ((sequence_before == sequence_after) &&
            ((sequence_after & 1U) == 0U)) {
            break;
        }
    }

    if (sequence_after == 0U) {
        return 0U;
    }
    *timestamp_us = ((uint64_t)timestamp_high << 32) |
                    (uint64_t)timestamp_low;
    return 1U;
}

/* ════════════════════════════════════════════════════════════════════════ */
/*  最后样本读取接口（向后兼容）                                              */
/* ════════════════════════════════════════════════════════════════════════ */

static APP_IMU_SampleMessage imu_last_sample;

const APP_IMU_SampleMessage *APP_IMU_GetLastSample(void)
{
    return &imu_last_sample;
}

/* ════════════════════════════════════════════════════════════════════════ */
/*  PC0 EXTI0 中断 — 数据就绪 → 唤醒 Sensor_Task                           */
/* ════════════════════════════════════════════════════════════════════════ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        const uint64_t timestamp_us = SVC_Timestamp_Us();
        const uint32_t write_sequence =
            app_imu_drdy_timestamp.sequence + 1U;
        app_imu_drdy_timestamp.sequence = write_sequence;
        __DMB();
        app_imu_drdy_timestamp.timestamp_low = (uint32_t)timestamp_us;
        app_imu_drdy_timestamp.timestamp_high =
            (uint32_t)(timestamp_us >> 32);
        __DMB();
        app_imu_drdy_timestamp.sequence = write_sequence + 1U;
        (void)osThreadFlagsSet(SensorTaskHandle, APP_IMU_DATA_READY_FLAG);
    }
}

/* ════════════════════════════════════════════════════════════════════════ */
/*  状态查询（由 app_control.c 诊断处理器调用）                               */
/* ════════════════════════════════════════════════════════════════════════ */

void APP_IMU_GetStatus(APP_IMU_Status *status)
{
    BSP_IMU_Diag diag;

    if (status == 0) { return; }

    BSP_IMU_GetDiag(&diag);

    const DRV_IMU_Device *dev = BSP_IMU_GetDevice();

    status->initialized   = (dev != 0) ? (uint8_t)(dev->init_stage >= BSP_ICM42688_INIT_STAGE_READY) : 0U;
    status->who_am_i      = (dev != 0) ? dev->who_am_i : 0U;
    status->init_stage    = (dev != 0) ? (uint8_t)dev->init_stage : 0U;
    status->last_status   = (dev != 0) ? (int32_t)dev->last_error : (int32_t)DRV_IMU_ERROR;
    status->last_error    = status->last_status;
    status->sample_count  = 0U;

    /* 缩放/姿态字段已过时；实时数据通过 SensorSampleQueue 传递 */
    status->temperature_cdeg = 0;
    status->accel_x_mg = 0;  status->accel_y_mg = 0;  status->accel_z_mg = 0;
    status->gyro_x_mdps = 0; status->gyro_y_mdps = 0; status->gyro_z_mdps = 0;
    status->roll_cdeg = 0;   status->pitch_cdeg = 0;  status->yaw_cdeg = 0;

    status->diag_mode0_tokmas   = diag.mode0_tokmas;
    status->diag_mode0_msb      = diag.mode0_msb;
    status->diag_mode0_bit0     = diag.mode0_bit0;
    status->diag_mode3_tokmas   = diag.mode3_tokmas;
    status->diag_mode3_msb      = diag.mode3_msb;
    status->diag_mode3_bit0     = diag.mode3_bit0;
    status->diag_burst_m0_b0_1  = diag.burst_m0_b0_1;
    status->diag_burst_m0_b0_2  = diag.burst_m0_b0_2;
    status->diag_burst_m0_b0_3  = diag.burst_m0_b0_3;
    status->diag_burst_m0_b0_4  = diag.burst_m0_b0_4;
    status->diag_burst_m3_tok_1 = diag.burst_m3_tok_1;
    status->diag_burst_m3_tok_2 = diag.burst_m3_tok_2;
    status->diag_burst_m3_tok_3 = diag.burst_m3_tok_3;
    status->diag_burst_m3_tok_4 = diag.burst_m3_tok_4;
    status->diag_best_mode      = diag.best_mode;
    status->diag_best_header    = diag.best_header;
    status->diag_valid          = diag.valid;
}
