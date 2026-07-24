#ifndef APP_MESSAGES_H
#define APP_MESSAGES_H

#include <stdint.h>

#include "drv_imu.h"
#include "app_sensor.h"

#define APP_UART_TX_TEXT_SIZE 256U
#define APP_UART_TX_FUNCTION_TEXT 0U
#define APP_UART_TX_FUNCTION_VOFA_SOCKET 1U

typedef struct APP_IMU_SampleMessage {
    uint32_t sample_count;
    uint8_t  who_am_i;
    uint8_t  reserved;
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
    uint64_t timestamp_us;        /* from SVC_Timestamp_Us() */
} APP_IMU_SampleMessage;

/*
 * Unified sensor sample pushed to SensorSampleQueue.
 *
 * APP_Sensor_Base is the first field — every sample carries a
 * microsecond timestamp (SVC_Timestamp_Us) and type discriminator,
 * and can be safely cast to APP_Sensor_Base *.
 *
 * IMU fields are filled on each interrupt; baro fields are
 * filled when new barometer data is available (TODO).
 */
typedef struct {
    APP_Sensor_Base base;           /* timestamp + type + sequence */

    /* IMU — scaled to physical units */
    DRV_IMU_RawData raw_imu;
    DRV_IMU_ScaledData imu;

    /* Attitude estimate from complementary filter */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    /* Barometer (TODO: fill from SPL06-007) */
    float  baro_pressure_pa;        /* Pa     */
    float  baro_temperature_c;      /* deg C  */
    uint8_t baro_updated;           /* 1 when new baro data present */

    /* IMU sampling diagnostics */
    float imu_sample_rate_hz;
    float imu_irq_sample_rate_hz;
    float imu_poll_sample_rate_hz;
    float imu_age_ms;
    APP_IMU_AttitudeDebug attitude_debug;
    uint8_t gyro_bias_ready;

    /* IMU data-ready counter (debug) */
    uint32_t imu_data_ready_count;
    uint32_t imu_poll_ready_count;
} APP_Sensor_SampleMessage;

typedef struct {
    uint16_t function;
    uint16_t length;
    char     text[APP_UART_TX_TEXT_SIZE];
} APP_UART_TxMessage;

#endif
