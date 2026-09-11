#include "app_sensor.h"

#include "app_messages.h"
#include "app_stabilizer.h"
#include "app_tasks.h"
#include "bsp_baro.h"
#include "bsp_imu.h"
#include "cmsis_os2.h"
#include "svc_imu.h"

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
/*    温度: 132.48 LSB/°C，偏移 +25°C（ICM-42688 数据手册标度）             */
/* ════════════════════════════════════════════════════════════════════════ */

#define APP_IMU_TEMP_LSB_PER_C    132.48f
#define APP_IMU_TEMP_OFFSET_C     25.0f

void APP_IMU_RawToScaled(const DRV_IMU_RawData *raw,
                         DRV_IMU_ScaledData *scaled)
{
    if ((raw == NULL) || (scaled == NULL)) return;

    /*
     * 换算交给 BSP 按**当前选中的芯片**做。
     *
     * 这里原本写死了 ICM-42688 的 LSB 表。板上换成 BMI088 之后那张表是错的：
     * 同一个"±16 g"枚举，ICM 是 2048 LSB/g，BMI088 映射到 ±24 g 后是 1365 LSB/g，
     * 差 1.5 倍。这种错误不会报错，只会让姿态和高度整体缩放——必须按芯片分派。
     * 温度同理：各芯片的偏置与刻度都不一样，一并由驱动的换算函数负责。
     */
    BSP_IMU_RawToScaled(raw, scaled);
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
/*  当前飞控板安装方向（由 M2 实测并已写入 Flash 的 V0 结果反推，2026-08-30）：*/
/*    IMU +X 朝飞机左方，IMU +Y 朝飞机上方，IMU +Z 朝飞机前方。              */
/*                                                                        */
/*  下列映射仅定义采集后的中间轴，不是机体系：                                */
/*    legacy_intermediate_v1 X = -imu Z                                     */
/*    legacy_intermediate_v1 Y = -imu X                                     */
/*    legacy_intermediate_v1 Z =  imu Y                                     */
/*  按上面的安装方向，该中间轴实为「X 后 / Y 右 / Z 上」，即相对 FLU 绕 Z     */
/*  轴 180°。中间轴的物理含义由持久化的 V0 拟合定义，不由本注释定义。          */
/*                                                                        */
/*  叠加持久化 orientation code 3（"-x,-y,+z" = diag(-1,-1,+1)）后，本 seam  */
/*  发布的合成映射即规范 FLU：                                               */
/*    published X(前) = imu Z, published Y(左) = imu X, published Z(上) = imu Y */
/*  该合成为 proper rotation（det=+1），accel/gyro 取同一旋转；由             */
/*  tests/test_flu_seam0_sensor_frame.py 以 host 编译执行装置钉死。           */
/*                                                                        */
/*  仅当没有 V0 候选（legacy 哨兵）时，才在 StabilizerTask 的 Fusion 输入     */
/*  边界退回旧符号补偿：                                                     */
/*    roll rate = -gyro X, pitch rate = +gyro Y, yaw rate = +gyro Z           */
/*    specific force = [-accel X, +accel Y, -accel Z]                         */
/*  该补偿把中间轴送成 FRD 以配合 Fusion 的 NED 约定，属未迁移回退路径。       */
/*                                                                        */
/*  APP_Sensor_AlignToAirframe() 保持为本机标定中间轴固定映射。V0 候选由     */
/*  APP_Sensor_ApplyFrameCorrection() 在完整 IMU 样本上一次性应用，避免       */
/*  accel/gyro 两次调用之间切换方向码而造成同帧坐标不一致。                  */
/* ════════════════════════════════════════════════════════════════════════ */

typedef struct {
    const char *descriptor;
    int8_t output_axis[3];
} APP_Sensor_FluOrientation;

/*
 * output_axis encodes R_FLU<-legacy_intermediate_v1 as signed 1-based axes:
 * +/-1 = +/-legacy X, +/-2 = +/-legacy Y, +/-3 = +/-legacy Z.
 * The 0..23 table order is a persistent Flash ABI used by the IMUFRAME Param
 * blob.  Never reorder existing entries.  Descriptors are the human/protocol
 * representation of those stable codes.
 */
static const APP_Sensor_FluOrientation
app_sensor_flu_orientations[APP_SENSOR_FLU_ORIENTATION_COUNT] = {
    { "+x,+y,+z", { +1, +2, +3 } },
    { "+x,-y,-z", { +1, -2, -3 } },
    { "-x,+y,-z", { -1, +2, -3 } },
    { "-x,-y,+z", { -1, -2, +3 } },
    { "+x,+z,-y", { +1, +3, -2 } },
    { "+x,-z,+y", { +1, -3, +2 } },
    { "-x,+z,+y", { -1, +3, +2 } },
    { "-x,-z,-y", { -1, -3, -2 } },
    { "+y,+x,-z", { +2, +1, -3 } },
    { "+y,-x,+z", { +2, -1, +3 } },
    { "-y,+x,+z", { -2, +1, +3 } },
    { "-y,-x,-z", { -2, -1, -3 } },
    { "+y,+z,+x", { +2, +3, +1 } },
    { "+y,-z,-x", { +2, -3, -1 } },
    { "-y,+z,-x", { -2, +3, -1 } },
    { "-y,-z,+x", { -2, -3, +1 } },
    { "+z,+x,+y", { +3, +1, +2 } },
    { "+z,-x,-y", { +3, -1, -2 } },
    { "-z,+x,-y", { -3, +1, -2 } },
    { "-z,-x,+y", { -3, -1, +2 } },
    { "+z,+y,-x", { +3, +2, -1 } },
    { "+z,-y,+x", { +3, -2, +1 } },
    { "-z,+y,+x", { -3, +2, +1 } },
    { "-z,-y,-x", { -3, -2, -1 } },
};

/* Cortex-M7 byte loads/stores are atomic; publish only this one-byte state. */
static volatile uint8_t app_sensor_flu_orientation =
    APP_SENSOR_FLU_ORIENTATION_LEGACY;

static float APP_Sensor_SelectSignedAxis(const float vector[3],
                                         int8_t signed_axis)
{
    uint8_t axis = (uint8_t)((signed_axis < 0) ? -signed_axis : signed_axis);
    float value = vector[axis - 1U];
    return (signed_axis < 0) ? -value : value;
}

uint8_t APP_Sensor_SetFluOrientationCode(uint8_t code)
{
    if ((code >= APP_SENSOR_FLU_ORIENTATION_COUNT) &&
        (code != APP_SENSOR_FLU_ORIENTATION_LEGACY)) {
        return 0U;
    }
    app_sensor_flu_orientation = code;
    return 1U;
}

uint8_t APP_Sensor_SetFluOrientation(const char *descriptor)
{
    uint8_t orientation;

    if (descriptor == NULL) {
        return 0U;
    }
    if (strcmp(descriptor, "legacy") == 0) {
        return APP_Sensor_SetFluOrientationCode(
            APP_SENSOR_FLU_ORIENTATION_LEGACY);
    }
    for (orientation = 0U;
         orientation < APP_SENSOR_FLU_ORIENTATION_COUNT;
         ++orientation) {
        if (strcmp(descriptor,
                   app_sensor_flu_orientations[orientation].descriptor) == 0) {
            return APP_Sensor_SetFluOrientationCode(orientation);
        }
    }
    return 0U;
}

/*
 * 解析出真正生效的方向码。
 *
 * `app_sensor_flu_orientation` 平时来自 Flash 里的 IMUFRAME 参数。但**全新的板子
 * 参数区是空的**，它会停在 legacy 哨兵上，于是整条链退回老板子实测的符号补偿——
 * 那套补偿是给 ICM-42688 那个安装方向的，用在 MicoAir 的 BMI088 上横滚方向是反的。
 *
 * 所以哨兵状态下按**当前探测到的芯片**给默认值（推导见 svc_imu.h）。
 * ICM-42688 与"没探到芯片"仍返回哨兵，老板子的行为一个字节都不变。
 *
 * 这里只做解析、不改状态：数据通路与诊断读到的是同一个值，
 * 不会出现"诊断说 legacy、实际按 3 在算"的情况（decoupling-spec D5-3）。
 */
static uint8_t APP_Sensor_EffectiveOrientationCode(void)
{
    uint8_t code = app_sensor_flu_orientation;

    if (code == APP_SENSOR_FLU_ORIENTATION_LEGACY) {
        code = SVC_IMU_DefaultOrientationCode(BSP_IMU_GetChipKind());
    }

    return code;
}

uint8_t APP_Sensor_GetFluOrientation(void)
{
    return APP_Sensor_EffectiveOrientationCode();
}

uint8_t APP_Sensor_IsFluOrientationActive(void)
{
    return (APP_Sensor_EffectiveOrientationCode() <
            APP_SENSOR_FLU_ORIENTATION_COUNT)
               ? 1U
               : 0U;
}

const char *APP_Sensor_GetFluOrientationDescriptorForCode(uint8_t code)
{
    if (code < APP_SENSOR_FLU_ORIENTATION_COUNT) {
        return app_sensor_flu_orientations[code].descriptor;
    }
    if (code == APP_SENSOR_FLU_ORIENTATION_LEGACY) {
        return "legacy";
    }
    return "invalid";
}

const char *APP_Sensor_GetFluOrientationDescriptor(void)
{
    return APP_Sensor_GetFluOrientationDescriptorForCode(
        APP_Sensor_EffectiveOrientationCode());
}

void APP_Sensor_AlignToAirframe(const float in[3], float out[3])
{
    if ((in == NULL) || (out == NULL)) return;

    /*
     * 芯片轴 → legacy_intermediate_v1，映射随**板上实际是哪颗 IMU** 而变。
     *
     * 老板子只有 ICM-42688，这里原本是三行写死的映射；MicoAir743v2 上换成了
     * BMI088 / BMI270，两颗的贴装朝向还各不相同。推导放在 Services/svc_imu.c
     * （纯函数、可在 PC 上单测），这里只负责取当前选中的芯片。
     *
     * 注意**不要**在别处再叠一层旋转：第二段的方向码表是持久化 ABI，
     * 叠加会双重应用，表现为横滚符号反了但姿态看着"差不多对"。
     */
    SVC_IMU_ChipToIntermediate(BSP_IMU_GetChipKind(), in, out);
}

static void APP_Sensor_ApplyOrientation(const float in[3],
                                        float out[3],
                                        uint8_t orientation)
{
    const int8_t *axis =
        app_sensor_flu_orientations[orientation].output_axis;
    out[0] = APP_Sensor_SelectSignedAxis(in, axis[0]);
    out[1] = APP_Sensor_SelectSignedAxis(in, axis[1]);
    out[2] = APP_Sensor_SelectSignedAxis(in, axis[2]);
}

uint8_t APP_Sensor_ApplyFrameCorrection(DRV_IMU_ScaledData *imu)
{
    uint8_t orientation = APP_Sensor_EffectiveOrientationCode();
    float accel_in[3];
    float gyro_in[3];
    float accel_out[3];
    float gyro_out[3];

    if (imu == NULL) {
        return APP_SENSOR_FLU_ORIENTATION_LEGACY;
    }
    if (orientation >= APP_SENSOR_FLU_ORIENTATION_COUNT) {
        return orientation;
    }

    accel_in[0] = imu->accel_x_g;
    accel_in[1] = imu->accel_y_g;
    accel_in[2] = imu->accel_z_g;
    gyro_in[0] = imu->gyro_x_dps;
    gyro_in[1] = imu->gyro_y_dps;
    gyro_in[2] = imu->gyro_z_dps;
    APP_Sensor_ApplyOrientation(accel_in, accel_out, orientation);
    APP_Sensor_ApplyOrientation(gyro_in, gyro_out, orientation);
    imu->accel_x_g = accel_out[0];
    imu->accel_y_g = accel_out[1];
    imu->accel_z_g = accel_out[2];
    imu->gyro_x_dps = gyro_out[0];
    imu->gyro_y_dps = gyro_out[1];
    imu->gyro_z_dps = gyro_out[2];
    return orientation;
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
/*  IMU DRDY 外部中断 — 数据就绪 → 唤醒 Sensor_Task                        */
/*                                                                        */
/*  老板子是 PC0/EXTI0。MicoAir743v2 上 PC0 是电池电压采样，DRDY 改到：      */
/*    PC15 = BMI088 陀螺 DRDY（主 IMU，节拍由陀螺定，角速率是最内环）        */
/*    PB7  = BMI270 DRDY（备用 IMU）                                        */
/*  两个引脚都接受：探测到哪颗就由哪颗发中断，这里不需要知道选中的是谁，     */
/*  另一颗没初始化就不会产生边沿。误判的代价只是多一次空唤醒，              */
/*  而漏判会让整个控制环退到 20 ms 轮询兜底——宁可宽松。                     */
/* ════════════════════════════════════════════════════════════════════════ */

#define APP_IMU_DRDY_PIN_MASK (GPIO_PIN_15 | GPIO_PIN_7)

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin & APP_IMU_DRDY_PIN_MASK) != 0U) {
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

    /* 芯片无关：板上可能是 BMI088 / BMI270 / ICM-42688，不能直接读某一颗的结构体。 */
    BSP_IMU_Info imu_info;
    BSP_IMU_GetInfo(&imu_info);

    status->initialized   = (uint8_t)(imu_info.init_stage >= DRV_IMU_INIT_STAGE_READY);
    status->who_am_i      = imu_info.chip_id;
    status->init_stage    = (uint8_t)imu_info.init_stage;
    status->last_status   = (int32_t)imu_info.last_error;
    status->last_error    = status->last_status;
    /*
     * 这些字段以前被硬写成 0，理由是"实时数据改走 SensorSampleQueue，这里已过时"。
     * 但它们仍然照常打印出去，于是一颗 810 Hz 满血运转的 IMU 在诊断里长这样：
     *     n=0 ax=0 ay=0 az=0 gx=0 gy=0 gz=0
     * 和"IMU 彻底死了"一个字都不差。2026-09-10 首刷 MicoAir743v2 时就按这个假象
     * 去查 SPI 接线了，而真实故障根本不在那儿。诊断沉默可以，撒谎不行（D5-3）。
     *
     * 改为从稳定器的只读验证快照取数——那正是 `IMU?` 打印真实样本时用的同一份
     * 数据，两条命令从此不会再互相矛盾。取不到快照就维持 0，但 initialized 字段
     * 已经独立表达了"链路有没有起来"，不会再被误读成数据为零。
     */
    StabilizerValidationImuSnapshot snapshot;

    status->sample_count     = 0U;
    status->temperature_cdeg = 0;
    status->accel_x_mg = 0;  status->accel_y_mg = 0;  status->accel_z_mg = 0;
    status->gyro_x_mdps = 0; status->gyro_y_mdps = 0; status->gyro_z_mdps = 0;
    status->roll_cdeg = 0;   status->pitch_cdeg = 0;  status->yaw_cdeg = 0;

    if (APP_Stabilizer_ReadValidationImuSnapshot(&snapshot) != 0U) {
        status->sample_count     = snapshot.sample_count;
        status->temperature_cdeg = (int16_t)lrintf(snapshot.temperature_c * 100.0f);
        status->accel_x_mg       = (int16_t)lrintf(snapshot.accel_g[0] * 1000.0f);
        status->accel_y_mg       = (int16_t)lrintf(snapshot.accel_g[1] * 1000.0f);
        status->accel_z_mg       = (int16_t)lrintf(snapshot.accel_g[2] * 1000.0f);
        status->gyro_x_mdps      = (int32_t)lrintf(snapshot.gyro_dps[0] * 1000.0f);
        status->gyro_y_mdps      = (int32_t)lrintf(snapshot.gyro_dps[1] * 1000.0f);
        status->gyro_z_mdps      = (int32_t)lrintf(snapshot.gyro_dps[2] * 1000.0f);
        status->roll_cdeg        = (int16_t)lrintf(snapshot.roll_deg * 100.0f);
        status->pitch_cdeg       = (int16_t)lrintf(snapshot.pitch_deg * 100.0f);
        status->yaw_cdeg         = (int16_t)lrintf(snapshot.yaw_deg * 100.0f);
    }

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
