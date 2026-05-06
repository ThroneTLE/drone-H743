#ifndef APP_MESSAGES_H
#define APP_MESSAGES_H

#include <stdint.h>

#define APP_UART_TX_TEXT_SIZE 256U

typedef struct {
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
} APP_IMU_SampleMessage;

typedef struct {
    uint16_t function;
    uint16_t length;
    char     text[APP_UART_TX_TEXT_SIZE];
} APP_UART_TxMessage;

#endif
