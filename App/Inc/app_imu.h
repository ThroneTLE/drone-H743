#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>

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

void APP_IMU_Task_Init(void);
void APP_IMU_Task_Step(void);
void APP_IMU_GetStatus(APP_IMU_Status *status);

#endif
