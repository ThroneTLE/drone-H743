#ifndef APP_BARO_H
#define APP_BARO_H

#include <stdint.h>

typedef struct {
    uint8_t report_done;
    int32_t init_status;
    int32_t split_status;
    int32_t txrx_status;
    uint8_t product_id;
    uint8_t split_id;
    uint8_t txrx_id;
    uint8_t bmp280_id;
    uint8_t cs_level;
    uint8_t miso_level;
} APP_Baro_Status;

typedef struct {
    APP_Baro_Status status;
    int32_t raw_status;
    int32_t coef_status;
    uint8_t raw_regs[14];
    uint8_t coef_regs[18];
    int32_t pressure_raw;
    int32_t temperature_raw;
    int32_t pressure_pa;
    int32_t temperature_cdeg;
    uint8_t scaled_valid;
    uint8_t prs_cfg;
    uint8_t tmp_cfg;
    uint8_t meas_cfg;
    uint8_t cfg_reg;
    uint8_t int_sts;
    uint8_t fifo_sts;
    uint8_t coef_srce;
    uint8_t id;
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} APP_Baro_Snapshot;

void APP_Baro_ReportStartup(void);
void APP_Baro_GetStatus(APP_Baro_Status *status);
void APP_Baro_ReadSnapshot(APP_Baro_Snapshot *snapshot);

#endif
