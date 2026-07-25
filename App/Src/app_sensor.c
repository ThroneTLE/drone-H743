#include "app_sensor.h"

#include "app_messages.h"
#include "app_tasks.h"
#include "bsp_baro.h"
#include "bsp_imu.h"
#include "cmsis_os2.h"

#include <math.h>

/*
 * APP_IMU 模块
 *
 * 传感器度量转换 API（由 freertos.c 中的 Sensor_Task 调用）：
 *   APP_IMU_RawToScaled      — ICM-42688 原始读数 → 物理单位
 *   APP_IMU_UpdateAttitude   — 互补滤波姿态解算
 *   APP_IMU_ConvertBaro      — SPL06-007 气压计 (TODO)
 *
 * 中断和诊断：
 *   HAL_GPIO_EXTI_Callback  — PC0 数据就绪 → 线程标志唤醒 Sensor_Task
 *   APP_IMU_GetStatus       — 寄存器级诊断快照
 *   APP_IMU_GetLastSample   — 向后兼容的样本读取接口
 */

#define APP_IMU_DATA_READY_FLAG 0x0001U

/* ════════════════════════════════════════════════════════════════════════ */
/*  IMU 原始读数 → 物理单位转换                                             */
/*                                                                        */
/*  ICM-42688 16 位有符号数 (±32768)，量程在 BSP 层配置：                   */
/*    加速度 ±4G    → 8192 LSB/g   (= 32768 / 4)                         */
/*    陀螺仪 ±1000dps → 32.8 LSB/dps (数据手册 §14.2)                    */
/*    温度: 128 LSB/°C，偏移 +25°C                                         */
/* ════════════════════════════════════════════════════════════════════════ */

#define APP_IMU_ACCEL_LSB_PER_G   8192.0f
#define APP_IMU_GYRO_LSB_PER_DPS  32.8f
#define APP_IMU_TEMP_LSB_PER_C    128.0f
#define APP_IMU_TEMP_OFFSET_C     25.0f

void APP_IMU_RawToScaled(const DRV_IMU_RawData *raw,
                         DRV_IMU_ScaledData *scaled)
{
    if ((raw == NULL) || (scaled == NULL)) return;

    scaled->temperature_c = (float)raw->temperature / APP_IMU_TEMP_LSB_PER_C
                          + APP_IMU_TEMP_OFFSET_C;
    scaled->accel_x_g    = (float)raw->accel_x / APP_IMU_ACCEL_LSB_PER_G;
    scaled->accel_y_g    = (float)raw->accel_y / APP_IMU_ACCEL_LSB_PER_G;
    scaled->accel_z_g    = (float)raw->accel_z / APP_IMU_ACCEL_LSB_PER_G;
    scaled->gyro_x_dps   = (float)raw->gyro_x / APP_IMU_GYRO_LSB_PER_DPS;
    scaled->gyro_y_dps   = (float)raw->gyro_y / APP_IMU_GYRO_LSB_PER_DPS;
    scaled->gyro_z_dps   = (float)raw->gyro_z / APP_IMU_GYRO_LSB_PER_DPS;
}

/* ════════════════════════════════════════════════════════════════════════ */
/*  姿态估计 — 互补滤波                                                     */
/*                                                                        */
/*  alpha 接近 1.0 → 更信任陀螺仪积分；                                      */
/*  (1-alpha)      → 更信任加速度计重力向量。                                */
/* ════════════════════════════════════════════════════════════════════════ */

#define APP_IMU_ACCEL_CORRECTION_TAU_SEC  0.05f
#define APP_IMU_DEFAULT_DT_SEC            0.001f
#define APP_IMU_MAX_DT_SEC                0.05f
#define APP_IMU_RAD_TO_DEG        57.2957795f
#define APP_IMU_ROLL_SIGN         (1.0f)
#define APP_IMU_PITCH_SIGN        (-1.0f)
#define APP_IMU_GYRO_ROLL_SIGN    (-1.0f)
#define APP_IMU_GYRO_PITCH_SIGN   (1.0f)
#define APP_IMU_GYRO_YAW_SIGN     (1.0f)//-1

static APP_IMU_AttitudeDebug imu_attitude_debug;

void APP_IMU_UpdateAttitude(const DRV_IMU_ScaledData *imu,
                            float *roll_deg,
                            float *pitch_deg,
                            float *yaw_deg,
                            float dt_sec,
                            uint32_t sample_count)
{
    float alpha;

    if (dt_sec <= 0.0f) {
        dt_sec = APP_IMU_DEFAULT_DT_SEC;
    } else if (dt_sec > APP_IMU_MAX_DT_SEC) {
        dt_sec = APP_IMU_MAX_DT_SEC;
    }

    alpha = APP_IMU_ACCEL_CORRECTION_TAU_SEC /
            (APP_IMU_ACCEL_CORRECTION_TAU_SEC + dt_sec);

    float roll_acc  = APP_IMU_ROLL_SIGN *
                      atan2f(imu->accel_y_g, imu->accel_z_g) * APP_IMU_RAD_TO_DEG;
    float pitch_acc = APP_IMU_PITCH_SIGN *
                      atan2f(-imu->accel_x_g,
                             sqrtf(imu->accel_y_g * imu->accel_y_g +
                                   imu->accel_z_g * imu->accel_z_g)) * APP_IMU_RAD_TO_DEG;
    float roll_gyro = *roll_deg +
                      APP_IMU_GYRO_ROLL_SIGN * imu->gyro_x_dps * dt_sec;
    float pitch_gyro = *pitch_deg +
                       APP_IMU_GYRO_PITCH_SIGN * imu->gyro_y_dps * dt_sec;

    if (sample_count <= 1U) {
        *roll_deg  = roll_acc;
        *pitch_deg = pitch_acc;
        roll_gyro = roll_acc;
        pitch_gyro = pitch_acc;
    } else {
        *roll_deg  = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
        *pitch_deg = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
        *yaw_deg  += APP_IMU_GYRO_YAW_SIGN * imu->gyro_z_dps * dt_sec;
    }

    imu_attitude_debug.roll_acc_deg = roll_acc;
    imu_attitude_debug.pitch_acc_deg = pitch_acc;
    imu_attitude_debug.roll_gyro_deg = roll_gyro;
    imu_attitude_debug.pitch_gyro_deg = pitch_gyro;
    imu_attitude_debug.roll_residual_deg = roll_acc - roll_gyro;
    imu_attitude_debug.pitch_residual_deg = pitch_acc - pitch_gyro;
    imu_attitude_debug.alpha = alpha;
    imu_attitude_debug.dt_ms = dt_sec * 1000.0f;

    if (*yaw_deg > 180.0f)       *yaw_deg -= 360.0f;
    else if (*yaw_deg < -180.0f) *yaw_deg += 360.0f;
}

void APP_IMU_GetAttitudeDebug(APP_IMU_AttitudeDebug *debug)
{
    if (debug == NULL) return;
    *debug = imu_attitude_debug;
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
/*  低通滤波器 (一阶 IIR)                                                  */
/*                                                                        */
/*  公式：alpha = 1 - exp(-2π * fc * dt)                                  */
/*        output = state + alpha * (input - state)                        */
/* ════════════════════════════════════════════════════════════════════════ */

void APP_Sensor_LpfInit(APP_Sensor_Lpf *lpf, float cutoff_hz, float dt_sec)
{
    if (lpf == NULL) return;
    float rc = 1.0f / (6.2831853f * cutoff_hz);
    lpf->alpha = dt_sec / (rc + dt_sec);
    lpf->state = 0.0f;
    lpf->initialized = 0U;
}

float APP_Sensor_LpfApply(APP_Sensor_Lpf *lpf, float input)
{
    if (lpf == NULL) return input;
    if (lpf->initialized == 0U) {
        lpf->state = input;
        lpf->initialized = 1U;
        return lpf->state;
    }
    lpf->state += lpf->alpha * (input - lpf->state);
    return lpf->state;
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
/*  坐标系对齐（IMU 芯片坐标系 → 机体坐标系）                               */
/*                                                                        */
/*  当前飞控板安装方向：                                                    */
/*    IMU +Y 朝飞机下方，IMU +Z 朝飞机后方，IMU +X 朝飞机左方。              */
/*                                                                        */
/*  机体系采用前右下约定（与当前互补滤波公式一致）：                          */
/*    body X = 前，body Y = 右，body Z = 下。                                */
/*                                                                        */
/*  因此轴映射为：                                                           */
/*    body X = -imu Z                                                       */
/*    body Y = -imu X                                                       */
/*    body Z =  imu Y                                                       */
/*                                                                        */
/*  加速度和陀螺仪都必须使用同一套刚体轴变换。                               */
/* ════════════════════════════════════════════════════════════════════════ */

void APP_Sensor_AlignToAirframe(const float in[3], float out[3])
{
    if ((in == NULL) || (out == NULL)) return;
    out[0] = -in[2];
    out[1] = -in[0];
    out[2] =  in[1];
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
