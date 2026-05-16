#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#include <stdint.h>

#include "drv_imu.h"
#include "svc_timestamp.h"   /* SVC_Timestamp_Us() */

/*
 * Sensor abstraction layer — C "inheritance" via embedded base struct.
 *
 * Every sensor reading starts with an APP_Sensor_Base header containing
 * a microsecond timestamp and type discriminator.  Because the base is
 * the first field, any derived sample can be safely cast to base:
 *
 *   APP_Sensor_IMU imu;
 *   APP_Sensor_Base *b = (APP_Sensor_Base *)&imu;
 *   // b->timestamp_us, b->type are valid
 */

#define APP_SENSOR_TIMESTAMP_INVALID 0ULL

/* ── sensor type discriminator ── */

typedef enum {
    APP_SENSOR_TYPE_NONE = 0,
    APP_SENSOR_TYPE_IMU  = 1,
    APP_SENSOR_TYPE_BARO = 2,
} APP_SensorType;

/* ── common base — first field in every sensor sample ── */

typedef struct {
    uint64_t       timestamp_us;   /* microsecond from SVC_Timestamp_Us() */
    APP_SensorType type;           /* discriminator for derived types */
    uint32_t       sequence;       /* per-type incrementing counter */
} APP_Sensor_Base;

/* ── IMU sample (base must stay first) ── */

typedef struct {
    APP_Sensor_Base base;

    /* physical units */
    float ax_g;          /* accel X    [g]   */
    float ay_g;          /* accel Y    [g]   */
    float az_g;          /* accel Z    [g]   */
    float gx_dps;        /* gyro X     [dps] */
    float gy_dps;        /* gyro Y     [dps] */
    float gz_dps;        /* gyro Z     [dps] */
    float temperature_c; /* die temp   [°C]  */

    /* attitude estimate from complementary filter */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} APP_Sensor_IMU;

/* ── Barometer sample (TODO: fill after SPL06-007 datasheet review) ── */

typedef struct {
    APP_Sensor_Base base;
    float           pressure_pa;    /* atmospheric pressure [Pa] */
    float           temperature_c;  /* temperature [°C]          */
} APP_Sensor_Baro;

/* ── legacy diagnostic struct ── */

typedef struct {
    uint8_t  initialized;
    uint8_t  who_am_i;
    uint8_t  init_stage;
    int32_t  last_status;
    int32_t  last_error;
    uint32_t sample_count;
    int16_t  temperature_cdeg;
    int16_t  accel_x_mg;
    int16_t  accel_y_mg;
    int16_t  accel_z_mg;
    int32_t  gyro_x_mdps;
    int32_t  gyro_y_mdps;
    int32_t  gyro_z_mdps;
    int16_t  roll_cdeg;
    int16_t  pitch_cdeg;
    int16_t  yaw_cdeg;
    uint8_t  diag_mode0_tokmas;
    uint8_t  diag_mode0_msb;
    uint8_t  diag_mode0_bit0;
    uint8_t  diag_mode3_tokmas;
    uint8_t  diag_mode3_msb;
    uint8_t  diag_mode3_bit0;
    uint8_t  diag_burst_m0_b0_1;
    uint8_t  diag_burst_m0_b0_2;
    uint8_t  diag_burst_m0_b0_3;
    uint8_t  diag_burst_m0_b0_4;
    uint8_t  diag_burst_m3_tok_1;
    uint8_t  diag_burst_m3_tok_2;
    uint8_t  diag_burst_m3_tok_3;
    uint8_t  diag_burst_m3_tok_4;
    uint8_t  diag_best_mode;
    uint8_t  diag_best_header;
    uint8_t  diag_valid;
} APP_IMU_Status;

/* ── sensor conversion API (called from Sensor_Task in freertos.c) ── */

/* ICM-42688 raw counts → physical units (g, dps, °C) */
void APP_IMU_RawToScaled(const DRV_IMU_RawData *raw,
                         DRV_IMU_ScaledData *scaled);

/* Complementary-filter attitude from accelerometer + gyroscope */
void APP_IMU_UpdateAttitude(const DRV_IMU_ScaledData *imu,
                            float *roll_deg,
                            float *pitch_deg,
                            float *yaw_deg,
                            uint32_t *last_tick_ms,
                            uint32_t sample_count);

/* TODO: barometer raw → pressure/temperature (SPL06-007, fill after datasheet review) */
void APP_IMU_ConvertBaro(const int32_t pressure_raw,
                         const int32_t temperature_raw,
                         float *pressure_pa,
                         float *temperature_c);

/* ════════════════════════════════════════════════════════════════════════ */
/*  低通滤波器 (一阶 IIR)                                                  */
/* ════════════════════════════════════════════════════════════════════════ */

typedef struct {
    float alpha;        /* 滤波系数，0~1，越小越平滑 */
    float state;        /* 上次输出 (用于连续滤波)     */
} APP_Sensor_Lpf;

/* 初始化滤波器：cutoff_hz = 截止频率, dt_sec = 采样周期 */
void APP_Sensor_LpfInit(APP_Sensor_Lpf *lpf, float cutoff_hz, float dt_sec);
/* 应用滤波：输入一个值，返回滤波后的值 */
float APP_Sensor_LpfApply(APP_Sensor_Lpf *lpf, float input);
/* 批量滤波三轴陀螺/加速度 */
void APP_Sensor_LpfApply3f(APP_Sensor_Lpf lpf[3],
                           const float in[3], float out[3]);

/* ════════════════════════════════════════════════════════════════════════ */
/*  陀螺仪零偏校准                                                        */
/*                                                                        */
/*  采集前 N 个样本的均值作为零偏偏移量                                    */
/* ════════════════════════════════════════════════════════════════════════ */

#define APP_SENSOR_GYRO_BIAS_SAMPLES 1000U

typedef struct {
    float   sum[3];     /* 累加和                  */
    uint32_t count;     /* 已采集样本数            */
    float   bias[3];    /* 计算出的零偏 (dps)       */
    uint8_t ready;      /* 1 = 校准完成             */
} APP_Sensor_GyroBias;

/* 每一帧调用，传入去零偏前的陀螺值 (dps)；
 * 内部累加，采集满后算出零偏。返回 1 表示刚完成校准。 */
uint8_t APP_Sensor_CalibrateGyroBias(float gx, float gy, float gz,
                                     APP_Sensor_GyroBias *cal);

/* ════════════════════════════════════════════════════════════════════════ */
/*  坐标系对齐（IMU 芯片坐标系 → 机体坐标系）                               */
/*                                                                        */
/*  默认直通（恒等变换）。如果 IMU 安装方向不同，修改此函数内的轴映射。       */
/* ════════════════════════════════════════════════════════════════════════ */

void APP_Sensor_AlignToAirframe(const float in[3], float out[3]);

/* ── diagnostics ── */

void APP_IMU_GetStatus(APP_IMU_Status *status);
struct APP_IMU_SampleMessage;
const struct APP_IMU_SampleMessage *APP_IMU_GetLastSample(void);

#endif /* APP_SENSOR_H */
